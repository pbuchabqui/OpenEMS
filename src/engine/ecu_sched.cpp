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
// Legacy test API placeholders (set/get TIM1 CCR no longer drive outputs).
static uint32_t ems_test_tim1_ign_cnt;
static uint32_t ems_test_tim1_ign_ccr1;
static uint32_t ems_test_tim1_ign_ccr2;
static uint32_t ems_test_tim1_ign_ccr3;
static uint32_t ems_test_tim1_ign_ccr4;

static uint32_t ems_test_gpioa_moder = 0u;
static uint32_t ems_test_gpiob_moder = 0u;
static uint32_t ems_test_gpioc_moder = 0u;
static uint32_t ems_test_gpioa_otyper = 0u;
static uint32_t ems_test_gpiob_otyper = 0u;
static uint32_t ems_test_gpioc_otyper = 0u;
static uint32_t ems_test_gpioa_pupdr = 0u;
static uint32_t ems_test_gpiob_pupdr = 0u;
static uint32_t ems_test_gpioc_pupdr = 0u;
static uint32_t ems_test_gpioa_afrh = 0u;
static uint32_t ems_test_gpioa_bsrr = 0u;
static uint32_t ems_test_gpiob_bsrr = 0u;
static uint32_t ems_test_gpioc_bsrr = 0u;

#define RCC_AHB2ENR1 ems_test_rcc_ahb2enr1
#define GPIOA_MODER ems_test_gpioa_moder
#define GPIOB_MODER ems_test_gpiob_moder
#define GPIOC_MODER ems_test_gpioc_moder
#define GPIOA_OTYPER ems_test_gpioa_otyper
#define GPIOB_OTYPER ems_test_gpiob_otyper
#define GPIOC_OTYPER ems_test_gpioc_otyper
#define GPIOA_PUPDR ems_test_gpioa_pupdr
#define GPIOB_PUPDR ems_test_gpiob_pupdr
#define GPIOC_PUPDR ems_test_gpioc_pupdr
#define GPIOA_AFRH  ems_test_gpioa_afrh
#define GPIOA_BSRR  ems_test_gpioa_bsrr
#define GPIOB_BSRR  ems_test_gpiob_bsrr
#define GPIOC_BSRR  ems_test_gpioc_bsrr
#define RCC_AHB2ENR1_GPIOAEN 1U
#define RCC_AHB2ENR1_GPIOBEN 2U
#define RCC_AHB2ENR1_GPIOCEN 4U
#define TIM_SR_CC3IF 0x8U
#define TIM_DIER_CC3IE (1U << 3)

static uint32_t ems_test_tim5_ccr3 = 0u;
static uint32_t ems_test_tim5_sr   = 0u;
static uint32_t ems_test_tim5_dier = 0u;
static uint32_t ems_test_tim5_cnt  = 0u;
#define TIM5_CCR3   ems_test_tim5_ccr3
#define TIM5_SR     ems_test_tim5_sr
#define TIM5_DIER   ems_test_tim5_dier
#define TIM5_CNT    ems_test_tim5_cnt
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

// ── Injector open watchdog (lost INJ_OFF / queue overflow backstop) ────────
// pin_idx 0..3 = INJ1..4. Arm on pin HIGH; release on pin LOW / trip.
// Timeout: 1.2 × current PW when armed via arm_channel; hard 36 ms floor for
// force_output/prime (prime clamps at 30 ms). Hard cap 36 ms always.
static volatile uint32_t g_inj_open_tick[4]   = {0U, 0U, 0U, 0U};
static volatile uint32_t g_inj_wdog_ticks[4]  = {0U, 0U, 0U, 0U};
static volatile uint32_t g_inj_watchdog_count = 0U;
static constexpr uint32_t kInjOpenWdogHardTicks = ECU_SCHED_US_TO_TICKS(36000U);

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
// Default 0 until first main commit — avoids angular fuel with default PW
// between first CKP edges and the first 2 ms policy tick (inhibit may still be 0).
volatile uint32_t g_inj_pw_ticks = 0U;
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

// Must hold a full tooth burst under multi-spark presync (wasted: 4 coils ×
// (1 primary + up to 3 extra) × 2 actions ≈ 32) plus concurrent inj edges.
// Match angle-table margin so arm_channel does not drop de-asserts under load.
#define EVT_QUEUE_SIZE 48U
static_assert(EVT_QUEUE_SIZE >= ECU_ANGLE_TABLE_SIZE,
              "event queue must cover a full angle-table tooth burst");

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

// GPIO BSRR — pin map via EMS_BOARD_* (board_pinout.h).
// Ordem ECU_CH_*: [INJ3, INJ4, INJ1, INJ2, IGN4, IGN3, IGN2, IGN1].
#include "hal/board_pinout.h"

enum : uint8_t { kPortA = 0U, kPortB = 1U, kPortC = 2U, kPortE = 3U };

#if EMS_BOARD_IS_VGT6
// VGT6 LQFP100 — GPIOE:
//   INJ1=PE0  INJ2=PE2  INJ3=PE4  INJ4=PE6
//   IGN1=PE9  IGN2=PE11 IGN3=PE13 IGN4=PE15
static constexpr uint8_t k_bsrr_port[8] = {
    kPortE, kPortE, kPortE, kPortE,
    kPortE, kPortE, kPortE, kPortE
};
static constexpr uint8_t k_bsrr_pin[8] = {
    4U, 6U, 0U, 2U,
    15U, 13U, 11U, 9U
};
#else
// RGT6 LQFP64 / WeAct:
//   INJ1=PA15 INJ2=PB3 INJ3=PC10 INJ4=PC11
//   IGN1=PC6  IGN2=PC7 IGN3=PC8  IGN4=PC9
static constexpr uint8_t k_bsrr_port[8] = {
    kPortC, kPortC, kPortA, kPortB,
    kPortC, kPortC, kPortC, kPortC
};
static constexpr uint8_t k_bsrr_pin[8] = {
    10U, 11U, 15U, 3U,
    9U, 8U, 7U, 6U
};
#endif

static inline void gpio_set_pin(uint8_t channel, uint8_t high) {
    if (channel >= 8U) { return; }
    const uint8_t pin = k_bsrr_pin[channel];
    const uint32_t mask = high ? (1U << pin) : (1U << (static_cast<uint32_t>(pin) + 16U));
    switch (k_bsrr_port[channel]) {
    case kPortA: GPIOA_BSRR = mask; break;
    case kPortB: GPIOB_BSRR = mask; break;
#if !defined(EMS_HOST_TEST)
    case kPortE: GPIOE_BSRR = mask; break;
#endif
    default:     GPIOC_BSRR = mask; break;
    }
}

static inline void pin_transition(uint8_t idx, uint8_t high, uint8_t is_safe_state = 0U);
static inline uint8_t channel_pin_idx(uint8_t ch) {
    return (ch < 8U) ? k_ch_to_pin_idx[ch] : 0xFFU;
}

// Drop one high=1 (ON/DWELL) event to make room for a de-assert (OFF/SPARK).
// Prefer same channel; else any assert. Returns 1 if a slot was freed.
static uint8_t evt_drop_one_assert(uint8_t prefer_channel)
{
    int8_t drop = -1;
    for (uint8_t i = 0U; i < g_evt_count; ++i) {
        if (g_evt_queue[i].high == 0U) { continue; }
        if (g_evt_queue[i].channel == prefer_channel) {
            drop = (int8_t)i;
            break;
        }
        if (drop < 0) { drop = (int8_t)i; }
    }
    if (drop < 0) { return 0U; }
    for (uint8_t i = (uint8_t)drop; i + 1U < g_evt_count; ++i) {
        g_evt_queue[i] = g_evt_queue[i + 1U];
    }
    --g_evt_count;
    return 1U;
}

// Insert event in sorted order (by timestamp). Called from tooth ISR.
// Overflow policy: never silently drop OFF/SPARK — drop an ON/DWELL first so
// an open injector/coil can still be closed. Drop new ON/DWELL if still full.
static void evt_insert(uint32_t ts, uint8_t channel, uint8_t high) {
    if (g_evt_count >= EVT_QUEUE_SIZE) {
        ++g_dbg_evt_overflow;
        if (high == 0U) {
            if (evt_drop_one_assert(channel) == 0U) { return; }
        } else {
            return;  // prefer keeping de-asserts already queued
        }
    }
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
    if (idx >= 8U) { return; }
    if (g_pin_last_state[idx] == high && high != 0xFFU) {
        if (is_safe_state == 0U) { ++g_pin_seq_error[idx]; }
        return;  // redundant transition — don't double-count
    }
    if (high) {
        ++g_pin_high_count[idx];
        if (idx < 4U) {
            // Injector open watchdog — pin HIGH arms the timer.
            g_inj_open_tick[idx] = TIM5_CNT | 1U;
            if (g_inj_wdog_ticks[idx] == 0U) {
                g_inj_wdog_ticks[idx] = kInjOpenWdogHardTicks;  // force/prime path
            }
        } else if (idx >= ECU_IGN_CH_FIRST && idx < (ECU_IGN_CH_FIRST + 4U)) {
            // Dwell watchdog starts when the coil pin actually goes HIGH — not when
            // DWELL is merely queued (sub-tooth lead can be several ms).
            const uint8_t ign_idx = (uint8_t)(idx - ECU_IGN_CH_FIRST);
            // OR 1: arm tick 0 is the inactive sentinel (TIM5_CNT can be 0).
            g_dwell_arm_tick[ign_idx] = TIM5_CNT | 1U;
            if (g_dwell_wdog_ticks[ign_idx] == 0U) {
                g_dwell_wdog_ticks[ign_idx] = (si::g_dwell_ticks * 7U) / 5U;
            }
        }
    } else {
        ++g_pin_low_count[idx];
        if (idx < 4U) {
            g_inj_open_tick[idx] = 0U;
            g_inj_wdog_ticks[idx] = 0U;
        } else if (idx >= ECU_IGN_CH_FIRST && idx < (ECU_IGN_CH_FIRST + 4U)) {
            // IGN pin LOW = spark/safe: release dwell watchdog for that coil.
            g_dwell_arm_tick[idx - ECU_IGN_CH_FIRST] = 0U;
        }
    }
    g_pin_last_state[idx] = high;
}

static void force_output(uint8_t ch, uint8_t action, uint8_t is_safe_state = 0U);

// Drop pending events for channels matching bit mask (inj or ign cylinder map).
// Also drive matching pins to safe (INJ_OFF / SPARK) and clear dwell arm.
static void purge_events_for_cyl_mask(uint8_t mask, uint8_t is_ign)
{
    if (mask == 0U) { return; }
    uint8_t w = 0U;
    for (uint8_t r = 0U; r < g_evt_count; ++r) {
        const uint8_t ch = g_evt_queue[r].channel;
        const uint8_t bit = (ch < 8U)
            ? (is_ign != 0U ? k_ign_ch_to_bit[ch] : k_inj_ch_to_bit[ch])
            : 0U;
        if (bit != 0U && (mask & bit) != 0U) {
            continue;  // drop
        }
        if (w != r) { g_evt_queue[w] = g_evt_queue[r]; }
        ++w;
    }
    g_evt_count = w;
    for (uint8_t cyl = 0U; cyl < 4U; ++cyl) {
        if ((mask & (1U << cyl)) == 0U) { continue; }
        if (is_ign != 0U) {
            force_output(si::kIgnCh[cyl], ECU_ACT_SPARK, 1U);
            g_dwell_arm_tick[cyl] = 0U;
        } else {
            force_output(si::kInjCh[cyl], ECU_ACT_INJ_OFF, 1U);
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

static void force_output(uint8_t ch, uint8_t action, uint8_t is_safe_state)
{
    // Safe-state transitions (INJ_OFF / SPARK) always allowed — never block a cut.
    // Non-safe ON paths honor inhibit masks so prime / test pulse cannot bypass
    // fuel-protect, half lockout, rev-limit, or flood-driven mask=0x0F.
    if (is_safe_state == 0U) {
        const uint8_t is_inj = (ch < ECU_IGN_CH_FIRST) ? 1U : 0U;
        if (is_inj != 0U && action == ECU_ACT_INJ_ON) {
            const uint8_t cyl_bit = (ch < 8U) ? k_inj_ch_to_bit[ch] : 0U;
            if (cyl_bit != 0U && (g_inj_inhibit_mask & cyl_bit) != 0U) { return; }
        }
        if (is_inj == 0U && action == ECU_ACT_DWELL_START) {
            const uint8_t cyl_bit = (ch < 8U) ? k_ign_ch_to_bit[ch] : 0U;
            if (cyl_bit != 0U && (g_ign_inhibit_mask & cyl_bit) != 0U) { return; }
        }
    }
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

    // Program watchdog timeouts at queue time; pin_transition starts the clock
    // only when the pin actually goes HIGH.
    if (is_inj != 0U && action == ECU_ACT_INJ_ON) {
        uint32_t t = (si::g_inj_pw_ticks * 6U) / 5U;  // 1.2 × PW
        if (t < ECU_SCHED_US_TO_TICKS(2000U)) { t = ECU_SCHED_US_TO_TICKS(2000U); }
        if (t > kInjOpenWdogHardTicks) { t = kInjOpenWdogHardTicks; }
        g_inj_wdog_ticks[pin_idx] = t;
    }
    if (is_inj == 0U && action == ECU_ACT_DWELL_START) {
        const uint8_t ign_idx = (uint8_t)(pin_idx - ECU_IGN_CH_FIRST);
        g_dwell_wdog_ticks[ign_idx] = (si::g_dwell_ticks * 7U) / 5U;
    }
    (void)now;

    // Absolute-timestamp insert (TIM5 domain). Signed lead is wrap-safe on
    // 32-bit TIM5; unsigned `target > tnow` misclassifies near-wrap futures.
    // Min-lead: if target already past / too soon, schedule at tnow+MIN_LEAD.
    // Do NOT count min-lead as STATUS_SCHED_LATE (tooth-ISR latency flood).
    {
        const uint32_t tnow = TIM5_CNT;
        const int32_t lead = (int32_t)(target_cnv - tnow);
        if (lead < (int32_t)STM32_MIN_COMPARE_LEAD_TICKS) {
            evt_insert(tnow + STM32_MIN_COMPARE_LEAD_TICKS, ch, high);
        } else {
            evt_insert(target_cnv, ch, high);
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
    for (uint8_t i = 0U; i < 4U; ++i) {
        g_dwell_arm_tick[i] = 0U;
        g_inj_open_tick[i] = 0U;
        g_inj_wdog_ticks[i] = 0U;
    }
    // Close any open knock window — sync lost means no valid combustion cylinder
    si::g_knock_sequential = 0U;
    ems::engine::knock_window_cycle_end();
}

void ecu_sched_outputs_safe_early(void)
{
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}

#if EMS_BOARD_IS_VGT6
    // VGT6: all INJ/IGN on GPIOE — push-pull LOW (active-high actuators).
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN;
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}
    static const uint8_t pe_pins[] = {0U, 2U, 4U, 6U, 9U, 11U, 13U, 15U};
    for (uint8_t i = 0U; i < 8U; ++i) {
        const uint8_t pin = pe_pins[i];
        GPIOE_OTYPER &= ~(1U << pin);
        GPIOE_PUPDR  = (GPIOE_PUPDR & ~(3U << (pin * 2U)));
        GPIOE_MODER  = (GPIOE_MODER & ~(3U << (pin * 2U))) | (1U << (pin * 2U));
    }
    GPIOE_BSRR = (1U << (0U + 16U)) | (1U << (2U + 16U)) | (1U << (4U + 16U))
               | (1U << (6U + 16U)) | (1U << (9U + 16U)) | (1U << (11U + 16U))
               | (1U << (13U + 16U)) | (1U << (15U + 16U));
#else
    // RGT6: INJ PA15/PB3/PC10/PC11 · IGN PC6–9
    // PA15 after reset is often JTDI with pull-up → HIGH until here.
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOBEN | RCC_AHB2ENR1_GPIOCEN;
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}

    GPIOA_AFRH = (GPIOA_AFRH & ~(0xFu << ((15U - 8U) * 4U)));
    GPIOA_OTYPER &= ~(1U << 15U);
    GPIOA_PUPDR  = (GPIOA_PUPDR & ~(3U << (15U * 2U)));
    GPIOB_OTYPER &= ~(1U << 3U);
    GPIOB_PUPDR  = (GPIOB_PUPDR & ~(3U << (3U * 2U)));
    for (uint8_t pin = 6U; pin <= 11U; ++pin) {
        GPIOC_OTYPER &= ~(1U << pin);
        GPIOC_PUPDR  = (GPIOC_PUPDR & ~(3U << (pin * 2U)));
    }

    GPIOA_MODER = (GPIOA_MODER & ~(3U << (15U * 2U))) | (1U << (15U * 2U));
    GPIOB_MODER = (GPIOB_MODER & ~(3U << (3U * 2U))) | (1U << (3U * 2U));
    for (uint8_t pin = 6U; pin <= 11U; ++pin) {
        GPIOC_MODER = (GPIOC_MODER & ~(3U << (pin * 2U))) | (1U << (pin * 2U));
    }

    GPIOA_BSRR = (1U << (15U + 16U));
    GPIOB_BSRR = (1U << (3U + 16U));
    GPIOC_BSRR = (1U << (6U + 16U)) | (1U << (7U + 16U))
               | (1U << (8U + 16U)) | (1U << (9U + 16U))
               | (1U << (10U + 16U)) | (1U << (11U + 16U));
#endif
}

void ECU_Hardware_Init(void)
{
    // Idempotent re-assert after late boot steps (USB delay, etc.).
    ecu_sched_outputs_safe_early();
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
        const uint32_t arm  = g_dwell_arm_tick[i];  // TIM5_CNT at pin HIGH
        const uint32_t tout = g_dwell_wdog_ticks[i];
        if (arm != 0U && tout != 0U && (now - arm) >= tout) {  // 32-bit wrap-safe
            // Force LOW *and* purge queued re-assert (DWELL still in queue after
            // a premature trip would re-charge the coil with no arm).
            purge_events_for_cyl_mask(static_cast<uint8_t>(1U << i), 1U);
            g_dwell_arm_tick[i] = 0U;
            g_dwell_wdog_ticks[i] = 0U;
            ++g_dwell_watchdog_count;
        }
    }
}

uint32_t ecu_sched_dwell_watchdog_count(void) { return g_dwell_watchdog_count; }

void ecu_sched_inj_watchdog(void)
{
    if (g_inj_pw_override != 0U) { return; }  // test/bench PW lock — disable
    const uint32_t now = TIM5_CNT;
    for (uint8_t i = 0U; i < 4U; ++i) {
        ems::hal::CriticalSectionGuard guard;
        const uint32_t open = g_inj_open_tick[i];
        const uint32_t tout = g_inj_wdog_ticks[i];
        if (open != 0U && tout != 0U && (now - open) >= tout) {
            // Force OFF + purge any pending re-assert for this cylinder.
            purge_events_for_cyl_mask(static_cast<uint8_t>(1U << i), 0U);
            g_inj_open_tick[i] = 0U;
            g_inj_wdog_ticks[i] = 0U;
            ++g_inj_watchdog_count;
        }
    }
}

uint32_t ecu_sched_inj_watchdog_count(void) { return g_inj_watchdog_count; }

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
    g_inj_watchdog_count = 0U;
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
        // pin_transition already armed on force HIGH; keep explicit ticks for tests.
        g_dwell_arm_tick[cyl]  = scheduler_counter() | 1U;
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
    const uint8_t new_mask = mask & 0x0FU;
    // Rising bits only: purge+force OFF for newly inhibited cylinders so a
    // mid-pulse fuel cut cannot leave an injector stuck open. Clearing the
    // mask (re-enable) only updates the mask; OFF/ON pairing resumes on next arm.
    const uint8_t newly = static_cast<uint8_t>(new_mask & ~g_inj_inhibit_mask);
    g_inj_inhibit_mask = new_mask;
    if (newly != 0U) {
        purge_events_for_cyl_mask(newly, 0U);
    }
}
uint8_t ecu_sched_get_inj_inhibit_mask(void) { return g_inj_inhibit_mask; }

void ecu_sched_set_ign_inhibit_mask(uint8_t mask)
{
    ems::hal::CriticalSectionGuard guard;
    const uint8_t new_mask = mask & 0x0FU;
    const uint8_t newly = static_cast<uint8_t>(new_mask & ~g_ign_inhibit_mask);
    g_ign_inhibit_mask = new_mask;
    // Spark-cut (limp rev_cut): drop any pending dwell/spark for inhibited
    // coils and force the pin LOW immediately so a mid-dwell cut cannot leave
    // the coil charged. Rev-limit production is fuel-only and leaves mask=0.
    if (newly != 0U) {
        purge_events_for_cyl_mask(newly, 1U);
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
        // Mode change presync↔sequential: drop pending events from the previous
        // table (wrong phase / bank / half-PW) before rebuilding.
        static uint8_t s_prev_sched_mode = 0xFFU;  // 0=presync, 1=seq, 0xFF=none
        const uint8_t mode = use_presync ? 0U : 1U;
        if (s_prev_sched_mode != 0xFFU && s_prev_sched_mode != mode) {
            g_evt_count = 0U;
            g_evt_armed = 0U;
            TIM5_DIER &= ~TIM_DIER_CC3IE;
            for (uint8_t i = 0U; i < ECU_CHANNELS; ++i) {
                force_output(i, (i < ECU_IGN_CH_FIRST) ? ECU_ACT_INJ_OFF : ECU_ACT_SPARK, 1U);
            }
            for (uint8_t i = 0U; i < 4U; ++i) {
                g_dwell_arm_tick[i] = 0U;
                g_inj_open_tick[i] = 0U;
                g_inj_wdog_ticks[i] = 0U;
            }
        }
        s_prev_sched_mode = mode;
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
            const uint32_t sub = (uint32_t)(((uint64_t)e->sub_frac_x256 * (uint64_t)tooth_ticks) >> 8U);
            arm_channel(e->channel, now + sub, e->action);
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
    // Reset dwell / inj open watchdog state
    for (uint8_t i = 0U; i < 4U; ++i) {
        g_dwell_arm_tick[i] = 0U; g_dwell_wdog_ticks[i] = 0U;
        g_inj_open_tick[i] = 0U; g_inj_wdog_ticks[i] = 0U;
    }
    g_dwell_watchdog_count = 0U;
    g_inj_watchdog_count = 0U;
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
