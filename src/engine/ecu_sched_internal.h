/**
 * @file ecu_sched_internal.h
 * @brief Shared state between ecu_sched.cpp (fila TIM5), ecu_sched_pins.cpp
 *        (GPIO + watchdogs de parede + prime) and the TIM2 encoder files
 *        (ecu_sched_encoder_*.cpp).
 *
 * GPIO write is header-inline via hal/out_pins.h (out_pin_write).
 * This header is private to the scheduler; public API remains ecu_sched.h.
 */
#pragma once

#include "engine/ecu_sched.h"
#include "engine/knock.h"
#include "hal/board_pinout.h"
#include "hal/out_pins.h"
#include "drv/ckp.h"

#include <stdint.h>

#if defined(EMS_HOST_TEST)
// Um único mock TIM5 partilhado por ecu_sched.cpp (fila) e
// ecu_sched_pins.cpp (relógios de pino). Definido em ecu_sched.cpp.
extern uint32_t ems_test_tim5_ccr3;
extern uint32_t ems_test_tim5_sr;
extern uint32_t ems_test_tim5_dier;
extern uint32_t ems_test_tim5_cnt;
#ifndef TIM_SR_CC3IF
#define TIM_SR_CC3IF 0x8U
#endif
#ifndef TIM_DIER_CC3IE
#define TIM_DIER_CC3IE (1U << 3)
#endif
#ifndef TIM5_CNT
#define TIM5_CCR3 ems_test_tim5_ccr3
#define TIM5_SR   ems_test_tim5_sr
#define TIM5_DIER ems_test_tim5_dier
#define TIM5_CNT  ems_test_tim5_cnt
#endif
#endif

inline constexpr uint8_t kEcuChannels = 8U;
inline constexpr uint8_t kIgnChFirst  = 4U;

inline uint8_t channel_pin_idx(uint8_t ch)
{
    return (ch < 8U) ? ems::hal::kOutChToPinIdx[ch] : 0xFFU;
}

namespace ems::engine::sched_internal {

// ── Clock helpers (same as ecu_sched.cpp) ───────────────────────────────────
inline constexpr uint32_t kCycleDeg = 720U;
inline constexpr uint32_t kMaxSeqInjPwDeg = 648U;      // 90% of 720°
inline constexpr uint32_t kMaxSeqInjPwCounts = (32768U * 9U) / 10U;  // 90% of 2 TIM2 revs
inline constexpr uint32_t kMaxPresyncInjPwDeg = 324U;  // 90% of 360°
inline constexpr uint32_t kMaxPresyncInjPwCounts = (16384U * 9U) / 10U;  // 90% of 1 TIM2 rev
// Max look-ahead to arm a sequential cylinder (dwell/inj_on): 60° of crank.
inline constexpr uint32_t kSeqArmWindowCounts = (16384U * 60U) / 360U;  // 2730
// Relative |Δω|/ω ×1000 to rewrite pending dwell/inj_on (2% default).
inline constexpr uint32_t kOmegaRefreshRelX1000 = 20U;

#define ECU_SCHED_US_TO_TICKS_INTERNAL(us) ((us) * 125U / 2U)
#define TOOTH_NS_TO_SCHED_INTERNAL(ns) \
    (static_cast<uint32_t>((ns) / ECU_SCHED_NS_PER_TICK))
inline constexpr uint32_t kInjOpenWdogHardTicks =
    ECU_SCHED_US_TO_TICKS_INTERNAL(36000U);

// Channel order cyl 0..3 — values match ECU_CH_* (legacy TIM map).
inline constexpr uint8_t kInjCh[4] = {
    ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
inline constexpr uint8_t kIgnCh[4] = {
    ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};

// Wasted-spark companions (engine_config.h): 0↔3 @ TDC 0°/360°, 2↔1 @ 180°/540°.
// Em presync (cego à fase) cada par dispara 1×/volta no ângulo do seu TDC
// colapsado — nunca as 4 bobinas no mesmo alvo.
inline constexpr uint8_t kWastedIgnPairA[2] = {ECU_CH_IGN1, ECU_CH_IGN4};  // cyl 0,3
inline constexpr uint8_t kWastedIgnPairB[2] = {ECU_CH_IGN3, ECU_CH_IGN2};  // cyl 2,1

// Inhibit mask bit for INJ/IGN channels (cyl 0..3), indexado por ECU_CH_*.
// Movido de ecu_sched.cpp (era static ali) — ecu_sched_encoder_queue.cpp
// precisa da mesma tabela para a varredura de purge da fila TIM2/CH3, e
// duplicar 8 bytes que têm de andar sempre em sincronia com ECU_CH_* é mais
// risco do que partilhar a única fonte.
inline constexpr uint8_t k_inj_ch_to_bit[8] = {
    (1U << 2), (1U << 3), (1U << 0), (1U << 1), 0U, 0U, 0U, 0U
};
inline constexpr uint8_t k_ign_ch_to_bit[8] = {
    0U, 0U, 0U, 0U, (1U << 3), (1U << 2), (1U << 1), (1U << 0)
};

// ── Calibration / mode read by builders (defined in ecu_sched.cpp) ──────────
extern volatile uint32_t g_advance_deg;
extern volatile uint32_t g_dwell_ticks;
extern volatile uint32_t g_inj_pw_ticks;
extern volatile uint32_t g_eoi_lead_deg;
extern volatile uint8_t  g_presync_inj_mode;
extern volatile uint8_t  g_presync_bank_toggle;
extern volatile uint8_t  g_knock_sequential;
extern volatile uint8_t  g_mspark_count;
extern volatile uint32_t g_mspark_inter_dwell_ticks;
extern volatile uint32_t g_mspark_atdc_limit_deg;
extern volatile uint32_t g_pw_duty_clamp_count;

// Alvos angulares (0..359, domínio 360°) do par wasted-spark A/B e do fim de
// injeção presync — geometria só (ecu_sched_encoder_builders.cpp despacha na
// fila TIM2/CH3). Par A @ TDC 0°, par B @ TDC 180° — 2 bobinas por evento.
struct PresyncWastedTargets {
    uint32_t spark_a;
    uint32_t spark_b;
    uint32_t eoi;
};

inline PresyncWastedTargets presync_wasted_targets(void) noexcept
{
    PresyncWastedTargets t;
    t.spark_a = (360U - (g_advance_deg % 360U)) % 360U;
    t.spark_b = (180U + 360U - (g_advance_deg % 360U)) % 360U;
    t.eoi     = (360U - (g_eoi_lead_deg % 360U)) % 360U;
    return t;
}

// Knock window on DWELL_START — single site for TIM5 + TIM2 arm paths.
// Gate: IGN channel, EMS_KNOCK_HW_PRESENT, sequential mode.
inline void maybe_knock_on_dwell_start(uint8_t ch) noexcept
{
    if (ch < ECU_CH_IGN4) { return; }
    if (!EMS_KNOCK_HW_PRESENT || g_knock_sequential == 0U) { return; }
    knock_window_cycle_end();
    knock_window_open(static_cast<uint8_t>(7U - ch));
}

// Multi-spark offset loop (deg domain). Call sites supply inter_deg
// (CKP: via tooth_period; encoder: via ω→counts→deg).
template <typename EmitFn>
inline void emit_multispark_deg(uint32_t spark_ang, uint32_t cycle_deg,
                                uint32_t inter_deg, EmitFn emit)
{
    const uint8_t ms_count = g_mspark_count;
    if (ms_count == 0U) { return; }
    const uint32_t step = inter_deg + 1U;
    const uint32_t window = g_advance_deg + g_mspark_atdc_limit_deg;
    for (uint8_t n = 1U; n <= ms_count; ++n) {
        const uint32_t add_spark_off = static_cast<uint32_t>(n) * step;
        if (add_spark_off >= window) { break; }
        const uint32_t add_dwell_off =
            static_cast<uint32_t>(n - 1U) * step + 1U;
        emit((spark_ang + add_dwell_off) % cycle_deg,
             (spark_ang + add_spark_off) % cycle_deg);
    }
}

// Hollow 60-2 angle table (builders are no-ops). Encoder uses TIM2 queue.
extern AngleEvent_t g_angle_table[ECU_ANGLE_TABLE_SIZE];
extern uint8_t g_angle_table_count;
extern uint32_t g_angle_tooth_mask_lo;
extern uint32_t g_angle_tooth_mask_hi;

void clear_angle_table(void);
void rebuild_sequential_cycle(const ems::drv::CkpSnapshot& snap);
void rebuild_presync_revolution(const ems::drv::CkpSnapshot& snap);

// ── MT6835/TIM2 encoder — fila TIM2/CH3 (ecu_sched_encoder_queue.cpp) ────
// Varredura adicional chamada por purge_events_for_cyl_mask()/
// clear_all_events_and_drive_safe_outputs() (ecu_sched.cpp) para também
// dropar eventos pendentes na fila em domínio de ângulo — um corte de
// cilindro/rev-limit tem de afetar as DUAS filas, não só a de tempo (ver
// docs/dev/mt6835_encoder_fork.md, secção 6). Sem efeito quando a fila TIM2
// está vazia (custo de chamar mesmo com EMS_MT6835_ENCODER=0: um early-return).
void encoder_purge_cyl_mask(uint8_t mask, uint8_t is_ign) noexcept;
void encoder_clear_all(void) noexcept;

}  // namespace ems::engine::sched_internal

// pin_transition: bookkeeping de transição de pino + arme dos watchdogs de
// dwell/injeção (definição em ecu_sched_pins.cpp, ligado sempre a TIM5/tempo).
// Exposto para a fila TIM2/CH3 poder disparar o mesmo mecanismo quando um
// evento em counts liga/desliga um pino.
void pin_transition(uint8_t idx, uint8_t high, uint8_t is_safe_state = 0U);

// force_close_cyl_mask: fecha pinos (SPARK/INJ_OFF) + limpa watchdog de dwell.
// Definição em ecu_sched_pins.cpp. Handoff encoder e purge TIM5 usam isto.
void force_close_cyl_mask(uint8_t mask, uint8_t is_ign);

// GPIO imediato + inhibit. Definição em ecu_sched_pins.cpp.
void force_output(uint8_t ch, uint8_t action, uint8_t is_safe_state = 0U,
                  uint8_t bypass_inhibit = 0U);

// Fila TIM5 — definição em ecu_sched.cpp. Prime/teste e watchdogs chamam.
void arm_channel(uint8_t ch, uint32_t target_cnv, uint8_t action);
void purge_events_for_cyl_mask(uint8_t mask, uint8_t is_ign);

// Relógios de pino (ecu_sched_pins.cpp). arm_channel programa o timeout;
// pin_transition arma o relógio no HIGH.
extern volatile uint32_t g_pin_high_count[8];
extern volatile uint32_t g_pin_low_count[8];
extern volatile uint32_t g_pin_seq_error[8];

extern volatile uint32_t g_dwell_arm_tick[4];
extern volatile uint32_t g_dwell_wdog_ticks[4];
extern volatile uint32_t g_dwell_watchdog_count;
extern volatile uint32_t g_inj_open_tick[4];
extern volatile uint32_t g_inj_wdog_ticks[4];
extern volatile uint32_t g_inj_watchdog_count;

// Inhibit + bench PW lock — escritos em ecu_sched.cpp, lidos por pins/watchdogs.
extern volatile uint8_t  g_inj_inhibit_mask;
extern volatile uint8_t  g_ign_inhibit_mask;
extern volatile uint8_t  g_inj_pw_override;
extern volatile uint32_t g_diag_prime_fired;
