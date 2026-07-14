#include "engine/ecu_sched.h"
#include "engine/ecu_sched_internal.h"
#include "drv/ckp.h"
#include "engine/engine_config.h"
#include "engine/constants.h"
#include "engine/knock.h"
#include "engine/calibration.h"
#include "engine/quick_crank.h"
#include "hal/regs.h"
#include "hal/critical_section.h"

namespace si = ems::engine::sched_internal;

#include <stddef.h>
#include <stdint.h>

#if defined(EMS_HOST_TEST)
// Host stubs for the live path only: TIM5 dispatcher + GPIO BSRR/MODER init.
// TIM1/TIM3 OC register surface removed after BSRR migration (not written on device).
static uint32_t ems_test_rcc_ahb2enr1;
static uint32_t ems_test_gpio_moder;
// Legacy test API placeholders (set/get TIM1 CCR no longer drive outputs).
static uint32_t ems_test_tim1_ign_cnt;
static uint32_t ems_test_tim1_ign_ccr1;
static uint32_t ems_test_tim1_ign_ccr2;
static uint32_t ems_test_tim1_ign_ccr3;
static uint32_t ems_test_tim1_ign_ccr4;

#define RCC_AHB2ENR1 ems_test_rcc_ahb2enr1
#define GPIOC_MODER ems_test_gpio_moder
#define GPIOE_MODER ems_test_gpio_moder
#define RCC_AHB2ENR1_GPIOAEN 1U
#define RCC_AHB2ENR1_GPIOBEN 2U
#define RCC_AHB2ENR1_GPIOCEN 4U
#define RCC_AHB2ENR1_GPIOEEN 8U
#define TIM_SR_CC3IF 0x8U
#define TIM_DIER_CC3IE (1U << 3)

static uint32_t ems_test_tim5_ccr3 = 0u;
static uint32_t ems_test_tim5_sr   = 0u;
static uint32_t ems_test_tim5_dier = 0u;
static uint32_t ems_test_tim5_cnt  = 0u;
static uint32_t ems_test_gpioc_bsrr = 0u;
static uint32_t ems_test_gpioe_bsrr = 0u;
#define TIM5_CCR3   ems_test_tim5_ccr3
#define TIM5_SR     ems_test_tim5_sr
#define TIM5_DIER   ems_test_tim5_dier
#define TIM5_CNT    ems_test_tim5_cnt
// Host: both ports alias one cell (pre-existing); device uses real BSRR addresses.
#define STM32_REG32(addr) (*reinterpret_cast<volatile uint32_t*>( \
    ((addr) == 0x42021018UL) ? &ems_test_gpioe_bsrr : &ems_test_gpioc_bsrr))
#endif

#define ECU_CHANNELS      8U
#define ECU_IGN_CH_FIRST  4U
#define ECU_CYCLE_DEG     720U
#define STM32_MIN_COMPARE_LEAD_TICKS 125U  // 2 µs @ 62.5 MHz

// Verificações de consistência do clock em tempo de compilação.
// Scheduler usa TIM5_CNT (32-bit, 62.5 MHz, 16 ns/tick).
static_assert(ECU_SCHED_CLOCK_HZ == 62500000U,
    "ECU_SCHED_CLOCK_HZ deve ser 62 500 000 Hz (16 ns/tick)");
static_assert(ECU_SCHED_NS_PER_TICK == 16U,
    "ECU_SCHED_NS_PER_TICK deve ser 16 ns @ 62.5 MHz");
// TICKS_PER_US = 62.5 (não-inteiro) — usar macro ECU_SCHED_US_TO_TICKS() para
// conversão exacta de µs para ticks sem truncamento.
#define ECU_SCHED_US_TO_TICKS(us) ((us) * 125U / 2U)
#define TOOTH_NS_TO_SCHED(ns) ((uint32_t)((ns) / ECU_SCHED_NS_PER_TICK))

// Pin-metric index 0..3 INJ, 4..7 IGN — same mapping as old stm32_*_tim_ch()-1.
// Indexed by raw ECU_CH_* (0..7): INJ3,INJ4,INJ1,INJ2, IGN4,IGN3,IGN2,IGN1.
static constexpr uint8_t k_ch_to_pin_idx[8] = {2U, 3U, 0U, 1U, 7U, 6U, 5U, 4U};

// Inhibit mask bit for INJ/IGN channels (cyl 0..3). Indexed by ECU_CH_* for 0..3 / 4..7.
static constexpr uint8_t k_inj_ch_to_bit[8] = {
    (1U << 2), (1U << 3), (1U << 0), (1U << 1), 0U, 0U, 0U, 0U
};
static constexpr uint8_t k_ign_ch_to_bit[8] = {
    0U, 0U, 0U, 0U, (1U << 3), (1U << 2), (1U << 1), (1U << 0)
};

// Angle table lives in ecu_sched_angle.cpp (cold builders). Aliases for local use.
// Hot path reads si::g_angle_table* at tooth time only.

volatile uint32_t g_late_event_count = 0U;
volatile uint32_t g_calibration_clamp_count = 0U;
volatile uint32_t g_cycle_schedule_drop_count = 0U;

// ── Dwell watchdog (MS42 §2.2.2.1.3 — TD × 1.4) ──────────────────────────
// Escrito pela ISR (arm_channel), lido pelo main loop (ecu_sched_dwell_watchdog).
// volatile necessário: compilador não pode cachear em registo entre os dois contextos.
static volatile uint32_t g_dwell_arm_tick[4]  = {0U, 0U, 0U, 0U};  // TIM5_CNT no arm de DWELL_START; 0 = inactivo
static volatile uint32_t g_dwell_wdog_ticks[4] = {0U, 0U, 0U, 0U};  // 1.4 × dwell_ticks no momento do arm
static volatile uint32_t g_dwell_watchdog_count = 0U;

// ── Per-cylinder inhibit masks (MS42 §2.2.5) ──────────────────────────────
// Escrito pelo main loop, lido pela ISR (arm_channel). bit N = cilindro N.
static volatile uint8_t g_inj_inhibit_mask = 0U;
// Ignition inhibit: suprime ECU_ACT_DWELL_START → bobina não carrega → sem faísca.
// Usado pelo soft rev limiter por ignição (retardo + corte alternado de cilindros).
static volatile uint8_t g_ign_inhibit_mask = 0U;

// ── Multi-spark (MS42 §2.2.3) ──────────────────────────────────────────────
// count=0 desabilitado. Valores escritos por ecu_sched_set_mspark() (main loop),
// lidos por Calculate_Sequential_Cycle() (ISR context via tooth hook).

// ── Contadores de diagnóstico (cranking, mode switches) ─────────────────────
volatile uint32_t g_diag_presync_revs = 0U;    // revoluções em modo presync
volatile uint32_t g_diag_seq_revs = 0U;        // revoluções em modo sequencial
volatile uint32_t g_diag_prime_fired = 0U;     // prime pulse disparado
volatile uint32_t g_diag_clear_all_count = 0U; // clear_all_events calls (non-init)
volatile uint32_t g_diag_unsync_teeth_peak = 0U; // pico de dentes sem sync

// Shared with ecu_sched_angle.cpp (cold builders) — see ecu_sched_internal.h
namespace ems::engine::sched_internal {
volatile uint8_t  g_mspark_count            = 0U;
volatile uint32_t g_mspark_inter_dwell_ticks = 0U;
volatile uint32_t g_mspark_atdc_limit_deg    = 18U;
volatile uint32_t g_advance_deg = 10U;
volatile uint32_t g_dwell_ticks = 187500U;  // 3 ms @ 62.5 MHz
volatile uint32_t g_inj_pw_ticks = 187500U;  // 3 ms @ 62.5 MHz
volatile uint32_t g_eoi_lead_deg = 355U;
volatile uint8_t  g_presync_inj_mode = ECU_PRESYNC_INJ_SEMI_SEQUENTIAL;
volatile uint8_t  g_presync_bank_toggle = 0U;
volatile uint8_t  g_knock_sequential = 0U;
volatile uint32_t g_pw_duty_clamp_count = 0U;
}  // namespace ems::engine::sched_internal

// Local-only runtime state (hot façade; not needed by angle builders)
volatile uint8_t g_inj_pw_override = 0U;  // 1=lock si::g_inj_pw_ticks, ignore main loop writes
// EOI targeting notes: builders use si::g_eoi_lead_deg (shared).
static volatile uint8_t g_presync_enable = 1U;
static volatile uint8_t g_presync_inj_auto = 1U;
static volatile uint8_t g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
static volatile uint8_t g_hook_prev_valid = 0U;
static volatile uint16_t g_hook_prev_tooth = 0U;
static volatile uint8_t g_hook_schedule_this_gap = 1U;
static volatile uint8_t g_cmp_phase_seen = 0U;
volatile uint32_t g_dbg_inj_force_early = 0U;
volatile uint32_t g_dbg_ign_force_early = 0U;
volatile uint32_t g_dbg_clear_all_count = 0U;
volatile uint32_t g_dbg_presync_count = 0U;
volatile uint32_t g_dbg_phase_skip = 0U;
volatile uint32_t g_dbg_phase_fire = 0U;
volatile uint32_t g_dbg_seq_calls = 0U;
volatile uint32_t g_dbg_inj1_arm = 0U;  // arm_channel calls for INJ1
volatile uint32_t g_dbg_ign1_arm = 0U;  // arm_channel calls for IGN1

// ── Absolute-timestamp event scheduler (TIM5_CH3 dispatcher) ────────────
// Events are timestamped in the TIM5 32-bit domain (62.5 MHz = 16 ns/tick).
// The next event's timestamp is loaded into TIM5_CCR3. When CNT matches,
// TIM5 ISR fires, executes GPIO BSRR, and loads the next event.
// No OC mode — pure compare + software GPIO.

#define EVT_QUEUE_SIZE 32U

struct SchedEvent {
    uint32_t timestamp;   // TIM5 absolute tick
    uint8_t  channel;     // ECU_CH_INJ1..IGN4
    uint8_t  high;        // 1=ON/DWELL, 0=OFF/SPARK
    uint8_t  valid;
    uint8_t  _pad;
};

static SchedEvent g_evt_queue[EVT_QUEUE_SIZE];
static volatile uint8_t g_evt_count = 0U;
static volatile uint8_t g_evt_armed = 0U;  // 1 if CCR3 is loaded with next event
volatile uint32_t g_dbg_evt_dispatched = 0U;
volatile uint32_t g_dbg_evt_inserted = 0U;
volatile uint32_t g_dbg_evt_overflow = 0U;

// Timestamp capture ring for angle measurement (INJ1 ON/OFF)
#define TS_RING_SIZE 32U
struct TsEntry { uint32_t ts; uint8_t high; uint8_t channel; uint8_t _pad[2]; };
static_assert(sizeof(TsEntry) == 8U, "TsEntry wire layout is 8 bytes for protocol 'G'");
volatile TsEntry g_ts_ring[TS_RING_SIZE];
volatile uint8_t g_ts_ring_idx = 0U;
// Gap timestamp from CKP (written by tooth hook at rev boundary)
volatile uint32_t g_last_gap_ts = 0U;

// GPIO BSRR addresses for direct pin control (no OC mode needed)
// PC6=INJ1(TIM3_CH1), PC7=INJ2, PC8=INJ3, PC9=INJ4
// PE9=IGN1(TIM1_CH1), PE11=IGN2, PE13=IGN3, PE14=IGN4
#define GPIOC_BSRR STM32_REG32(0x42020818UL)
#define GPIOE_BSRR STM32_REG32(0x42021018UL)

// BSRR set/clr masks per ECU_CH_* (0..7). Port: 0=GPIOC (INJ), 1=GPIOE (IGN).
static constexpr uint32_t k_bsrr_set[8] = {
    (1U << 8), (1U << 9), (1U << 6), (1U << 7),   // INJ3 PC8, INJ4 PC9, INJ1 PC6, INJ2 PC7
    (1U << 14), (1U << 13), (1U << 11), (1U << 9) // IGN4 PE14, IGN3 PE13, IGN2 PE11, IGN1 PE9
};
static constexpr uint32_t k_bsrr_clr[8] = {
    (1U << 24), (1U << 25), (1U << 22), (1U << 23),
    (1U << 30), (1U << 29), (1U << 27), (1U << 25)
};
static constexpr uint8_t k_bsrr_port_e[8] = {0U, 0U, 0U, 0U, 1U, 1U, 1U, 1U};

static inline void gpio_set_pin(uint8_t channel, uint8_t high) {
    if (channel >= 8U) { return; }
    const uint32_t mask = high ? k_bsrr_set[channel] : k_bsrr_clr[channel];
    if (k_bsrr_port_e[channel] != 0U) {
        GPIOE_BSRR = mask;
    } else {
        GPIOC_BSRR = mask;
    }
}

static inline void pin_transition(uint8_t idx, uint8_t high, uint8_t is_safe_state = 0U);
static inline uint8_t channel_pin_idx(uint8_t ch) {
    return (ch < 8U) ? k_ch_to_pin_idx[ch] : 0xFFU;
}

// Insert event in sorted order (by timestamp). Called from tooth ISR.
static void evt_insert(uint32_t ts, uint8_t channel, uint8_t high) {
    if (g_evt_count >= EVT_QUEUE_SIZE) { ++g_dbg_evt_overflow; return; }
    ++g_dbg_evt_inserted;
    // Find insertion point (linear search, queue is small)
    uint8_t pos = g_evt_count;
    for (uint8_t i = 0; i < g_evt_count; ++i) {
        if ((int32_t)(ts - g_evt_queue[i].timestamp) < 0) {
            pos = i;
            break;
        }
    }
    // Shift right
    for (uint8_t i = g_evt_count; i > pos; --i) {
        g_evt_queue[i] = g_evt_queue[i - 1U];
    }
    g_evt_queue[pos].timestamp = ts;
    g_evt_queue[pos].channel = channel;
    g_evt_queue[pos].high = high;
    g_evt_queue[pos].valid = 1U;
    ++g_evt_count;

    // If this is the earliest event, arm CCR3
    if (pos == 0U) {
        TIM5_CCR3 = ts;
        TIM5_SR  = ~TIM_SR_CC3IF;  // rc_w0: só CC3IF é limpo
        TIM5_DIER |= TIM_DIER_CC3IE;
        g_evt_armed = 1U;
    }
}

// Fire queue head: BSRR + optional ts_ring + pin metrics + dequeue.
// capture_ts: path-1 (due) only — path-2 (already late) keeps prior work set (no ts_ring).
static inline void evt_execute_head(uint32_t now, uint8_t capture_ts) {
    const SchedEvent& e = g_evt_queue[0];
    gpio_set_pin(e.channel, e.high);
    if (capture_ts != 0U && (e.channel == ECU_CH_INJ1 || e.channel == ECU_CH_IGN1)) {
        const uint8_t ri = g_ts_ring_idx;
        g_ts_ring[ri].ts = now;
        g_ts_ring[ri].high = e.high;
        g_ts_ring[ri].channel = e.channel;
        g_ts_ring_idx = (ri + 1U) & (TS_RING_SIZE - 1U);
    }
    const uint8_t idx = channel_pin_idx(e.channel);
    if (idx != 0xFFU) {
        pin_transition(idx, e.high);
    }
    ++g_dbg_evt_dispatched;
    --g_evt_count;
    for (uint8_t i = 0; i < g_evt_count; ++i) {
        g_evt_queue[i] = g_evt_queue[i + 1U];
    }
}

// Called from TIM5 ISR when CC3IF fires
void ecu_sched_evt_dispatch(void) {
    const uint32_t now = TIM5_CNT;
    // Process all events that are due (handles simultaneous events)
    while (g_evt_count > 0U) {
        const SchedEvent& e = g_evt_queue[0];
        if ((int32_t)(e.timestamp - now) > 0) { break; }  // still in future
        evt_execute_head(now, 1U);
    }
    // Arm next event or disable interrupt
    // Loop: if the next event is already past, process it immediately
    while (g_evt_count > 0U) {
        const uint32_t next_ts = g_evt_queue[0].timestamp;
        if ((int32_t)(next_ts - TIM5_CNT) > 16) {  // >16 ticks (~0.25µs) in future
            TIM5_CCR3 = next_ts;
            TIM5_SR  = ~TIM_SR_CC3IF;  // rc_w0: só CC3IF é limpo
            g_evt_armed = 1U;
            return;
        }
        // Already past — process inline (no ts_ring; count as late for diag only)
        ++g_late_event_count;
        evt_execute_head(TIM5_CNT, 0U);
    }
    TIM5_DIER &= ~TIM_DIER_CC3IE;
    g_evt_armed = 0U;
}


// Pin transition verification: count every actual pin state change
volatile uint32_t g_pin_high_count[8];  // [0-3]=INJ CH1-4, [4-7]=IGN CH1-4
volatile uint32_t g_pin_low_count[8];
volatile uint32_t g_pin_seq_error[8];   // consecutive same-direction transitions
static uint8_t    g_pin_last_state[8];  // 0=LOW, 1=HIGH, 0xFF=unknown

static inline void pin_transition(uint8_t idx, uint8_t high, uint8_t is_safe_state) {
    if (g_pin_last_state[idx] == high && high != 0xFFU) {
        if (is_safe_state == 0U) { ++g_pin_seq_error[idx]; }
        return;  // redundant transition — don't double-count
    }
    if (high) {
        ++g_pin_high_count[idx];
    } else {
        ++g_pin_low_count[idx];
        // IGN pin LOW = spark/safe: release dwell watchdog for that coil.
        // Must not clear on SPARK *arm* — only when the pin actually drops,
        // otherwise a lost SPARK event can leave the driver HIGH forever.
        if (idx >= ECU_IGN_CH_FIRST && idx < (ECU_IGN_CH_FIRST + 4U)) {
            g_dwell_arm_tick[idx - ECU_IGN_CH_FIRST] = 0U;
        }
    }
    g_pin_last_state[idx] = high;
}

static inline uint32_t scheduler_counter(void)
{
    return TIM5_CNT;  // 32-bit, 62.5 MHz — sem wrap de 16-bit
}

static void sanitize_runtime_calibration(void)
{
    uint8_t clamped = 0U;
    if (si::g_advance_deg > 60U) { si::g_advance_deg = 60U; clamped = 1U; }
    // Clamps em ticks TIM5 (62.5 MHz): 100000 ticks ≈ 1.6ms dwell máx
    if (si::g_dwell_ticks > 625000U) { si::g_dwell_ticks = 625000U; clamped = 1U; }
    if (si::g_inj_pw_ticks > 1250000U) { si::g_inj_pw_ticks = 1250000U; clamped = 1U; }
    // EOI lead ∈ [0, 719]: 0–129 = fim na compressão (closed-valve, soak longo);
    // 130–359 = válvula aberta / admissão (estilo Speeduino, default 355);
    // 360–719 = fim no escape/pré-IVO (closed-valve OEM, soak curto na válvula
    // quente — candidato a eoi_idle). Presync mapeia via % 360.
    if (si::g_eoi_lead_deg >= ECU_CYCLE_DEG) { si::g_eoi_lead_deg = ECU_CYCLE_DEG - 1U; clamped = 1U; }
    if (si::g_presync_inj_mode > ECU_PRESYNC_INJ_SEMI_SEQUENTIAL) { si::g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS; clamped = 1U; }
    if (g_presync_ign_mode > ECU_PRESYNC_IGN_WASTED_SPARK) { g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK; clamped = 1U; }
    if (clamped != 0U) { ++g_calibration_clamp_count; }
}

static void force_output(uint8_t ch, uint8_t action, uint8_t is_safe_state = 0U)
{
    const uint8_t high = ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U;
    const uint8_t idx = channel_pin_idx(ch);
    if (idx != 0xFFU) {
        gpio_set_pin(ch, high);
        pin_transition(idx, high, is_safe_state);
    }
}

static void arm_channel(uint8_t ch, uint32_t target_cnv, uint8_t action)
{
    // Atomic: read TIM5_CNT + queue insert must not interleave with TIM5 dispatch ISR.
    ems::hal::CriticalSectionGuard guard;

    const uint8_t is_inj = (ch < ECU_IGN_CH_FIRST) ? 1U : 0U;
    const uint8_t pin_idx = channel_pin_idx(ch);
    const uint32_t now = scheduler_counter();  // TIM5_CNT, 32-bit

    if (pin_idx == 0xFFU) { ++g_cycle_schedule_drop_count; return; }

    // Inhibit masks: skip INJ_ON / DWELL_START for masked cylinders.
    if (is_inj != 0U && action == ECU_ACT_INJ_ON) {
        const uint8_t cyl_bit = (ch < 8U) ? k_inj_ch_to_bit[ch] : 0U;
        if (cyl_bit != 0U && (g_inj_inhibit_mask & cyl_bit) != 0U) { return; }
    }
    if (is_inj == 0U && action == ECU_ACT_DWELL_START) {
        const uint8_t cyl_bit = (ch < 8U) ? k_ign_ch_to_bit[ch] : 0U;
        if (cyl_bit != 0U && (g_ign_inhibit_mask & cyl_bit) != 0U) { return; }
    }

    const uint8_t high = ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U;
    if (ch == ECU_CH_INJ1) { ++g_dbg_inj1_arm; }
    if (ch == ECU_CH_IGN1) { ++g_dbg_ign1_arm; }

    // Dwell watchdog tracking (ignition channels only) — pin_idx 4..7 → ign 0..3.
    // Arm on DWELL_START; release only when the pin goes LOW (pin_transition) or
    // the watchdog itself trips. Do NOT clear on ECU_ACT_SPARK arm: both events
    // are typically queued before dwell starts, and clearing here left the coil
    // unprotected for the whole charge window (lost SPARK → stuck HIGH).
    if (is_inj == 0U && action == ECU_ACT_DWELL_START) {
        const uint8_t ign_idx = (uint8_t)(pin_idx - ECU_IGN_CH_FIRST);
        g_dwell_arm_tick[ign_idx] = now;
        g_dwell_wdog_ticks[ign_idx] = (si::g_dwell_ticks * 7U) / 5U;
    }

    // Absolute-timestamp insert (TIM5 domain). Re-read CNT for delta after inhibit work.
    // Min-lead: if target already past / too soon, schedule at tnow+MIN_LEAD
    // (timestamp policy unchanged). Do NOT count min-lead as STATUS_SCHED_LATE —
    // that flood made the late bit sticky under normal tooth-ISR latency. True
    // misses are counted only in dispatch path-2 (event already past at re-arm).
    {
        const uint32_t tnow = TIM5_CNT;
        const uint32_t delta = (target_cnv > tnow) ? (target_cnv - tnow) : 0U;
        if (delta < STM32_MIN_COMPARE_LEAD_TICKS) {
            evt_insert(tnow + STM32_MIN_COMPARE_LEAD_TICKS, ch, high);
        } else {
            evt_insert(tnow + delta, ch, high);
        }
        return;
    }
}

static void clear_all_events_and_drive_safe_outputs(void)
{
    si::clear_angle_table();
    // Clear TIM5 event queue
    g_evt_count = 0U;
    g_evt_armed = 0U;
    TIM5_DIER &= ~TIM_DIER_CC3IE;
    for (uint8_t i = 0U; i < ECU_CHANNELS; ++i) { force_output(i, (i < ECU_IGN_CH_FIRST) ? ECU_ACT_INJ_OFF : ECU_ACT_SPARK, 1U); }
    for (uint8_t i = 0U; i < 4U; ++i) { g_dwell_arm_tick[i] = 0U; }
    // Close any open knock window — sync lost means no valid combustion cylinder
    si::g_knock_sequential = 0U;
    ems::engine::knock_window_cycle_end();
}

void ECU_Hardware_Init(void)
{
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOBEN | RCC_AHB2ENR1_GPIOCEN | RCC_AHB2ENR1_GPIOEEN;

    // All INJ/IGN pins are GPIO outputs — driven by event scheduler via BSRR
    // INJ: PC6(INJ1), PC7(INJ2), PC8(INJ3), PC9(INJ4)
    for (uint8_t pin = 6U; pin <= 9U; ++pin) {
        GPIOC_MODER = (GPIOC_MODER & ~(3U << (pin * 2U))) | (1U << (pin * 2U));
    }
    GPIOC_BSRR = (1U<<22)|(1U<<23)|(1U<<24)|(1U<<25);  // PC6-9 LOW
    // IGN: PE9(IGN1), PE11(IGN2), PE13(IGN3), PE14(IGN4)
    {
        static const uint8_t ign_pins[] = {9U, 11U, 13U, 14U};
        for (uint8_t i = 0; i < 4; ++i) {
            const uint8_t pin = ign_pins[i];
            GPIOE_MODER = (GPIOE_MODER & ~(3U << (pin * 2U))) | (1U << (pin * 2U));
        }
    }
    GPIOE_BSRR = (1U<<25)|(1U<<27)|(1U<<29)|(1U<<30);  // PE9,11,13,14 LOW

    clear_all_events_and_drive_safe_outputs();
}

void ecu_sched_commit_calibration(uint32_t advance_deg, uint32_t dwell_ticks, uint32_t inj_pw_ticks, uint32_t eoi_lead_deg)
{
    ems::hal::CriticalSectionGuard guard;
    if (g_inj_pw_override == 0U) {
        si::g_advance_deg = advance_deg;
        si::g_dwell_ticks = dwell_ticks;
        si::g_inj_pw_ticks = inj_pw_ticks;
    } else if (g_inj_pw_override == 2U) {
        si::g_advance_deg = advance_deg;
        si::g_dwell_ticks = dwell_ticks;
        si::g_inj_pw_ticks = inj_pw_ticks;
        g_inj_pw_override = 1U;
    }
    si::g_eoi_lead_deg = eoi_lead_deg;
    sanitize_runtime_calibration();
}
void ecu_sched_set_advance_deg(uint32_t adv) { ems::hal::CriticalSectionGuard guard; si::g_advance_deg = adv; sanitize_runtime_calibration(); }
void ecu_sched_set_dwell_ticks(uint32_t dwell) { ems::hal::CriticalSectionGuard guard; si::g_dwell_ticks = dwell; sanitize_runtime_calibration(); }
void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks) { ems::hal::CriticalSectionGuard guard; if (g_inj_pw_override == 0U) { si::g_inj_pw_ticks = pw_ticks; } sanitize_runtime_calibration(); }
void ecu_sched_set_eoi_lead_deg(uint32_t eoi_lead_deg) { ems::hal::CriticalSectionGuard guard; si::g_eoi_lead_deg = eoi_lead_deg; sanitize_runtime_calibration(); }
void ecu_sched_set_presync_enable(uint8_t enable) { ems::hal::CriticalSectionGuard guard; g_presync_enable = (enable != 0U) ? 1U : 0U; }
void ecu_sched_set_presync_inj_auto(uint8_t on) { ems::hal::CriticalSectionGuard guard; g_presync_inj_auto = on ? 1U : 0U; }

void ecu_sched_set_presync_inj_mode(uint8_t mode) { ems::hal::CriticalSectionGuard guard; si::g_presync_inj_mode = mode; sanitize_runtime_calibration(); }
void ecu_sched_set_presync_ign_mode(uint8_t mode) { ems::hal::CriticalSectionGuard guard; g_presync_ign_mode = mode; sanitize_runtime_calibration(); }
uint32_t ecu_sched_pw_duty_clamp_count(void) { return si::g_pw_duty_clamp_count; }

void ecu_sched_dwell_watchdog(void)
{
    if (g_inj_pw_override != 0U) { return; }  // test mode — disable watchdog
    const uint32_t now = TIM5_CNT;
    for (uint8_t i = 0U; i < 4U; ++i) {
        ems::hal::CriticalSectionGuard guard;
        const uint32_t arm  = g_dwell_arm_tick[i];  // TIM5_CNT value
        const uint32_t tout = g_dwell_wdog_ticks[i];
        if (arm != 0U && (now - arm) >= tout) {  // 32-bit, sem mask de wrap
            // force_output → pin_transition(LOW) also clears g_dwell_arm_tick.
            force_output(si::kIgnCh[i], ECU_ACT_SPARK, 1U);
            g_dwell_arm_tick[i] = 0U;
            ++g_dwell_watchdog_count;
        }
    }
}

uint32_t ecu_sched_dwell_watchdog_count(void) { return g_dwell_watchdog_count; }

uint8_t ecu_sched_is_sequential(void) { return si::g_knock_sequential; }
uint8_t ecu_sched_presync_inj_mode(void) { return si::g_presync_inj_mode; }
uint8_t ecu_sched_presync_inj_auto(void) { return g_presync_inj_auto; }

void ecu_sched_reset_diagnostic_counters(void)
{
    ems::hal::CriticalSectionGuard guard;
    g_late_event_count = 0U;
    g_cycle_schedule_drop_count = 0U;
    g_calibration_clamp_count = 0U;
        si::g_pw_duty_clamp_count = 0U;
    g_dwell_watchdog_count = 0U;
}

void ecu_sched_fire_prime_pulse(uint32_t pw_us)
{
    if (pw_us == 0U) { return; }
    if (pw_us > 30000U) { pw_us = 30000U; }
    const uint32_t off_cnv = scheduler_counter() + ECU_SCHED_US_TO_TICKS(pw_us);
    for (uint8_t i = 0U; i < 4U; ++i) { force_output(si::kInjCh[i], ECU_ACT_INJ_ON); }
    for (uint8_t i = 0U; i < 4U; ++i) { arm_channel(si::kInjCh[i], off_cnv, ECU_ACT_INJ_OFF); }
    ++g_diag_prime_fired;
}

void ecu_sched_test_pulse_inj(uint8_t cyl, uint32_t pw_us)
{
    if (cyl > 3U || pw_us == 0U) { return; }
    if (pw_us > 30000U) { pw_us = 30000U; }
    const uint8_t ch = si::kInjCh[cyl];
    const uint32_t off_cnv = scheduler_counter() + ECU_SCHED_US_TO_TICKS(pw_us);
    force_output(ch, ECU_ACT_INJ_ON);
    arm_channel(ch, off_cnv, ECU_ACT_INJ_OFF);
}

void ecu_sched_test_pulse_ign(uint8_t cyl, uint32_t dwell_us)
{
    if (cyl > 3U) { return; }
    if (dwell_us == 0U) { dwell_us = 3000U; }
    if (dwell_us > 10000U) { dwell_us = 10000U; }
    const uint8_t ch = si::kIgnCh[cyl];
    const uint32_t spark_cnv = scheduler_counter() + ECU_SCHED_US_TO_TICKS(dwell_us);
    force_output(ch, ECU_ACT_DWELL_START);
    // Arm watchdog for this manual dwell (DWELL already forced HIGH; SPARK is
    // only queued). pin_transition(LOW) / watchdog release the arm tick.
    {
        ems::hal::CriticalSectionGuard guard;
        g_dwell_arm_tick[cyl]  = scheduler_counter();
        g_dwell_wdog_ticks[cyl] = (ECU_SCHED_US_TO_TICKS(dwell_us) * 7U) / 5U;
    }
    arm_channel(ch, spark_cnv, ECU_ACT_SPARK);
}

void ecu_sched_test_all_outputs_safe(void)
{
    ems::hal::CriticalSectionGuard guard;
    clear_all_events_and_drive_safe_outputs();
}

void ecu_sched_set_mspark(uint8_t count, uint32_t inter_dwell_ticks, uint32_t atdc_limit_deg)
{
    ems::hal::CriticalSectionGuard guard;
    si::g_mspark_count            = (count > 3U) ? 3U : count;
    si::g_mspark_inter_dwell_ticks = inter_dwell_ticks;
    si::g_mspark_atdc_limit_deg   = (atdc_limit_deg == 0U) ? 18U : atdc_limit_deg;
}

void ecu_sched_set_inj_inhibit_mask(uint8_t mask)
{
    ems::hal::CriticalSectionGuard guard;
    g_inj_inhibit_mask = mask & 0x0FU;
}
uint8_t ecu_sched_get_inj_inhibit_mask(void) { return g_inj_inhibit_mask; }

void ecu_sched_set_ign_inhibit_mask(uint8_t mask)
{
    ems::hal::CriticalSectionGuard guard;
    const uint8_t new_mask = mask & 0x0FU;
    g_ign_inhibit_mask = new_mask;
    // Spark-cut (limp rev_cut): drop any pending dwell/spark for inhibited
    // coils and force the pin LOW immediately so a mid-dwell cut cannot leave
    // the coil charged. Rev-limit production is fuel-only and leaves mask=0.
    if (new_mask != 0U) {
        uint8_t w = 0U;
        for (uint8_t r = 0U; r < g_evt_count; ++r) {
            const uint8_t ch = g_evt_queue[r].channel;
            const uint8_t bit = (ch < 8U) ? k_ign_ch_to_bit[ch] : 0U;
            if (bit != 0U && (new_mask & bit) != 0U) {
                continue;  // drop event for inhibited IGN channel
            }
            if (w != r) { g_evt_queue[w] = g_evt_queue[r]; }
            ++w;
        }
        g_evt_count = w;
        for (uint8_t cyl = 0U; cyl < 4U; ++cyl) {
            if ((new_mask & (1U << cyl)) != 0U) {
                force_output(si::kIgnCh[cyl], ECU_ACT_SPARK, 1U);
                g_dwell_arm_tick[cyl] = 0U;
            }
        }
        if (g_evt_count == 0U) {
            TIM5_DIER &= ~TIM_DIER_CC3IE;
            g_evt_armed = 0U;
        } else {
            TIM5_CCR3 = g_evt_queue[0].timestamp;
            TIM5_SR   = ~TIM_SR_CC3IF;
            TIM5_DIER |= TIM_DIER_CC3IE;
            g_evt_armed = 1U;
        }
    }
}
uint8_t ecu_sched_get_ign_inhibit_mask(void) { return g_ign_inhibit_mask; }

// Bench / protocol: lock PW for one commit then ignore main-loop PW writes (was raw g_inj_pw_override poke).
void ecu_sched_bench_pw_lock_next_commit(void)
{
    ems::hal::CriticalSectionGuard guard;
    g_inj_pw_override = 2U;  // write-once then lock
}

uint8_t ecu_sched_bench_pw_override_state(void)
{
    return g_inj_pw_override;
}

void ecu_sched_get_angle_trace(uint32_t *gap_ts,
                               uint8_t *ring_idx,
                               EcuSchedTsSample out_last8[8])
{
    // Snapshot without CS: single-reader protocol path; worst case is a torn
    // sample on concurrent ISR update — acceptable for angle diagnostics.
    if (gap_ts != nullptr) { *gap_ts = g_last_gap_ts; }
    const uint8_t ridx = g_ts_ring_idx;
    if (ring_idx != nullptr) { *ring_idx = ridx; }
    if (out_last8 == nullptr) { return; }
    for (uint8_t i = 0U; i < 8U; ++i) {
        const uint8_t ri = static_cast<uint8_t>((ridx + TS_RING_SIZE - 8U + i) & (TS_RING_SIZE - 1U));
        out_last8[i].ts = g_ts_ring[ri].ts;
        out_last8[i].high = g_ts_ring[ri].high;
        out_last8[i].channel = g_ts_ring[ri].channel;
    }
}

void ecu_sched_get_pin_counts_u32x24(uint32_t out[24])
{
    if (out == nullptr) { return; }
    for (uint8_t i = 0U; i < 8U; ++i) {
        out[i * 3U + 0U] = g_pin_high_count[i];
        out[i * 3U + 1U] = g_pin_low_count[i];
        out[i * 3U + 2U] = g_pin_seq_error[i];
    }
}

void ecu_sched_get_diag_snapshot(EcuSchedDiagSnapshot *out)
{
    if (out == nullptr) { return; }
    out->late_event_count = g_late_event_count;
    out->cycle_schedule_drop_count = g_cycle_schedule_drop_count;
    out->inj1_arm = g_dbg_inj1_arm;
    out->seq_calls = g_dbg_seq_calls;
    out->evt_overflow = g_dbg_evt_overflow;
    out->clear_all_count = g_dbg_clear_all_count;
    out->presync_count = g_dbg_presync_count;
    out->dwell_watchdog_count = g_dwell_watchdog_count;
    out->phase_skip = g_dbg_phase_skip;
    out->phase_fire = g_dbg_phase_fire;
    out->evt_inserted = g_dbg_evt_inserted;
    out->evt_dispatched = g_dbg_evt_dispatched;
    out->diag_presync_revs = g_diag_presync_revs;
    out->diag_seq_revs = g_diag_seq_revs;
    out->diag_clear_all_count = g_diag_clear_all_count;
}

namespace ems::engine {
extern bool g_prev_cranking;
void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept
{
    static uint8_t s_unsync_teeth = 0U;
    if ((snap.state != ems::drv::SyncState::FULL_SYNC) && (snap.state != ems::drv::SyncState::HALF_SYNC)) {
        ++s_unsync_teeth;
        if (s_unsync_teeth > g_diag_unsync_teeth_peak) { g_diag_unsync_teeth_peak = s_unsync_teeth; }
        if (s_unsync_teeth >= 60U && g_hook_prev_valid != 0U) {
            clear_all_events_and_drive_safe_outputs();
            g_hook_prev_valid = 0U; g_hook_prev_tooth = 0U; g_hook_schedule_this_gap = 1U; g_cmp_phase_seen = 0U;
            ++g_dbg_clear_all_count;
            ++g_diag_clear_all_count;
        }
        return;
    }
    s_unsync_teeth = 0U;

    // Cam-present gate: decoupled from phase_A (which now toggles at every gap).
    // Requires 2 validated CMP edges since last sync loss.
    g_cmp_phase_seen = (snap.cmp_confirms >= 2U) ? 1U : 0U;

    // Auto-select presync injection mode by cranking state
    // SIMULTANEOUS during crank (batch-fire), SEMI_SEQUENTIAL after
    if (g_presync_inj_auto) {
        si::g_presync_inj_mode = ems::engine::is_cranking()
            ? ECU_PRESYNC_INJ_SIMULTANEOUS
            : ECU_PRESYNC_INJ_SEMI_SEQUENTIAL;
    }

    const uint8_t rev_boundary = ((g_hook_prev_valid != 0U) && (snap.tooth_index == 0U) && (g_hook_prev_tooth != 0U)) ? 1U : 0U;
    if (rev_boundary != 0U) {
        g_last_gap_ts = snap.last_tim5_capture;  // TIM5 timestamp of gap (tooth 0)
        const bool use_presync = (snap.state == ems::drv::SyncState::HALF_SYNC && g_presync_enable != 0U && g_cmp_phase_seen == 0U)
                              || (snap.state == ems::drv::SyncState::FULL_SYNC && g_cmp_phase_seen == 0U);
        if (use_presync) {
            ++g_dbg_presync_count;
            ++g_diag_presync_revs;
            si::rebuild_presync_revolution(snap);
            // Rearma o toggle p/ que a PRIMEIRA fronteira sequencial após presync
            // compute sempre a tabela (Calculate_Sequential_Cycle). Sem isto, se um
            // ciclo sequencial anterior deixou o toggle em 0, a re-entrada saltava
            // uma volta — mantendo a tabela wasted (PHASE_ANY, meia-PW) por +360°.
            g_hook_schedule_this_gap = 1U;
        } else {
            if (g_hook_schedule_this_gap != 0U) {
                ++g_dbg_seq_calls;
                ++g_diag_seq_revs;
                si::rebuild_sequential_cycle(snap);
                g_hook_schedule_this_gap = 0U;
            } else {
                g_hook_schedule_this_gap = 1U;
            }
        }
    }

    const uint8_t tooth_index = (uint8_t)snap.tooth_index;
    const uint32_t tooth_mask = (tooth_index < 32U)
        ? (si::g_angle_tooth_mask_lo & (1UL << tooth_index))
        : (si::g_angle_tooth_mask_hi & (1UL << (tooth_index - 32U)));
    if (tooth_mask != 0U) {
        const uint32_t period_ns = (snap.predicted_tooth_period_ns != 0U)
            ? snap.predicted_tooth_period_ns
            : snap.tooth_period_ns;
        uint32_t tooth_ticks = TOOTH_NS_TO_SCHED(period_ns);
        const uint32_t now = scheduler_counter();
        const uint8_t current_phase = snap.phase_A ? ECU_PHASE_A : ECU_PHASE_B;
        for (uint8_t i = 0U; i < si::g_angle_table_count; ++i) {
            const AngleEvent_t *e = &si::g_angle_table[i];
            if (e->tooth_index != tooth_index) { continue; }
            if ((e->phase_A != ECU_PHASE_ANY) && (e->phase_A != current_phase)) { ++g_dbg_phase_skip; continue; }
            ++g_dbg_phase_fire;
            arm_channel(e->channel, now + ((e->sub_frac_x256 * tooth_ticks) >> 8U), e->action);
        }
    }

    g_hook_prev_valid = 1U; g_hook_prev_tooth = snap.tooth_index;
}
}

namespace ems::drv {
void schedule_on_tooth(const CkpSnapshot& snap) noexcept { ems::engine::ecu_sched_on_tooth_hook(snap); }
}

#if defined(EMS_HOST_TEST)
void ecu_sched_test_reset(void)
{
    g_late_event_count = 0U; g_cycle_schedule_drop_count = 0U; g_calibration_clamp_count = 0U;
    g_presync_enable = 1U; g_presync_inj_auto = 0U; si::g_presync_inj_mode = ECU_PRESYNC_INJ_SEMI_SEQUENTIAL; g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
    si::g_presync_bank_toggle = 0U; g_hook_prev_valid = 0U; g_hook_prev_tooth = 0U; g_hook_schedule_this_gap = 1U;
    si::g_advance_deg = 10U; si::g_dwell_ticks = 140625U; si::g_inj_pw_ticks = 140625U; si::g_eoi_lead_deg = 355U;
    si::g_angle_table_count = 0U; si::g_angle_tooth_mask_lo = 0U; si::g_angle_tooth_mask_hi = 0U;
    si::g_pw_duty_clamp_count = 0U;
    g_inj_inhibit_mask = 0U;
    g_ign_inhibit_mask = 0U;
    si::g_mspark_count = 0U; si::g_mspark_inter_dwell_ticks = 0U; si::g_mspark_atdc_limit_deg = 18U;
    // Reset dwell watchdog state
    for (uint8_t i = 0U; i < 4U; ++i) { g_dwell_arm_tick[i] = 0U; g_dwell_wdog_ticks[i] = 0U; }
    g_dwell_watchdog_count = 0U;
    g_inj_pw_override = 0U;
    // Reset TIM5 event queue
    g_evt_count = 0U; g_evt_armed = 0U;
    for (uint8_t i = 0U; i < EVT_QUEUE_SIZE; ++i) { g_evt_queue[i].valid = 0U; }
    ems_test_tim5_ccr3 = 0U; ems_test_tim5_sr = 0U; ems_test_tim5_dier = 0U; ems_test_tim5_cnt = 0U;
    // Reset mode/diag counters — testes de transição presync↔sequencial dependem
    // de arrancar em estado limpo (senão herdam contagem de testes anteriores).
    g_diag_presync_revs = 0U; g_diag_seq_revs = 0U;
    si::g_knock_sequential = 0U; g_cmp_phase_seen = 0U;
}
uint8_t ecu_sched_test_angle_table_size(void) { return si::g_angle_table_count; }
uint8_t ecu_sched_test_get_angle_event(uint8_t index, uint8_t *tooth, uint8_t *sub_frac, uint8_t *ch, uint8_t *action, uint8_t *phase)
{
    if ((index >= si::g_angle_table_count) || (si::g_angle_table[index].valid == 0U)) { return 0U; }
    *tooth = si::g_angle_table[index].tooth_index; *sub_frac = si::g_angle_table[index].sub_frac_x256; *ch = si::g_angle_table[index].channel; *action = si::g_angle_table[index].action; *phase = si::g_angle_table[index].phase_A; return 1U;
}
void ecu_sched_test_set_advance_deg(uint32_t adv) { ecu_sched_set_advance_deg(adv); }
void ecu_sched_test_set_dwell_ticks(uint32_t dwell) { ecu_sched_set_dwell_ticks(dwell); }
void ecu_sched_test_set_inj_pw_ticks(uint32_t pw_ticks) { ecu_sched_set_inj_pw_ticks(pw_ticks); }
void ecu_sched_test_set_eoi_lead_deg(uint32_t eoi_lead_deg) { ecu_sched_set_eoi_lead_deg(eoi_lead_deg); }
uint32_t ecu_sched_test_get_advance_deg(void) { return si::g_advance_deg; }
uint32_t ecu_sched_test_get_dwell_ticks(void) { return si::g_dwell_ticks; }
uint32_t ecu_sched_test_get_inj_pw_ticks(void) { return si::g_inj_pw_ticks; }
uint32_t ecu_sched_test_get_eoi_lead_deg(void) { return si::g_eoi_lead_deg; }
uint32_t ecu_sched_test_get_calibration_clamp_count(void) { return g_calibration_clamp_count; }
uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void) { return g_cycle_schedule_drop_count; }
uint32_t ecu_sched_test_get_late_event_count(void) { return g_late_event_count; }
uint32_t ecu_sched_test_get_pw_duty_clamp_count(void) { return si::g_pw_duty_clamp_count; }
void ecu_sched_test_set_mspark(uint8_t count, uint32_t inter_dwell_ticks, uint32_t atdc_limit_deg) {
    ecu_sched_set_mspark(count, inter_dwell_ticks, atdc_limit_deg);
}
uint8_t ecu_sched_test_get_mspark_count(void) { return si::g_mspark_count; }
void ecu_sched_test_set_tim1_cnt(uint32_t cnt) noexcept { ems_test_tim1_ign_cnt = cnt; }
void ecu_sched_test_set_tim2_cnt(uint32_t cnt) noexcept { ems_test_tim5_cnt = cnt; }
void ecu_sched_test_reset_ccr(void) noexcept {
    ems_test_tim1_ign_ccr1 = 0u; ems_test_tim1_ign_ccr2 = 0u;
    ems_test_tim1_ign_ccr3 = 0u; ems_test_tim1_ign_ccr4 = 0u;
    ems_test_tim5_ccr3 = 0u; g_evt_count = 0U; g_evt_armed = 0U;
}
uint32_t ecu_sched_test_get_tim1_ccr(uint8_t ch) noexcept {
    switch (ch) {
        case 1u: return ems_test_tim1_ign_ccr1;
        case 2u: return ems_test_tim1_ign_ccr2;
        case 3u: return ems_test_tim1_ign_ccr3;
        case 4u: return ems_test_tim1_ign_ccr4;
        default: return 0u;
    }
}
// TIM5 event-queue accessors for tests
uint8_t  ecu_sched_test_get_evt_count(void) noexcept { return g_evt_count; }
uint32_t ecu_sched_test_get_tim5_ccr3(void)  noexcept { return ems_test_tim5_ccr3; }
void     ecu_sched_test_set_tim5_cnt(uint32_t v) noexcept { ems_test_tim5_cnt = v; }
uint8_t  ecu_sched_test_get_evt(uint8_t index,
                                uint32_t *ts,
                                uint8_t *channel,
                                uint8_t *high) noexcept
{
    if (index >= g_evt_count) { return 0U; }
    if (ts != nullptr) { *ts = g_evt_queue[index].timestamp; }
    if (channel != nullptr) { *channel = g_evt_queue[index].channel; }
    if (high != nullptr) { *high = g_evt_queue[index].high; }
    return 1U;
}
uint32_t ecu_sched_test_get_presync_revs(void) { return g_diag_presync_revs; }
uint32_t ecu_sched_test_get_seq_revs(void) { return g_diag_seq_revs; }
#endif
