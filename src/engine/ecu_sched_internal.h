/**
 * @file ecu_sched_internal.h
 * @brief Shared state between ecu_sched.cpp (hot path) and ecu_sched_angle.cpp
 *        (cold angle-table builders at rev boundary only).
 *
 * Hot-path rule (Tier 1.5): do NOT move evt_insert / arm_channel /
 * ecu_sched_evt_dispatch / tooth arm loop out of ecu_sched.cpp.
 * GPIO write is header-inline via hal/out_pins.h (out_pin_write) — not a
 * separate .cpp call — so it still inlines without LTO.
 *
 * This header is private to the scheduler; public API remains ecu_sched.h.
 */
#pragma once

#include "engine/ecu_sched.h"
#include "drv/ckp.h"

#include <stdint.h>

namespace ems::engine::sched_internal {

// ── Clock helpers (same as ecu_sched.cpp) ───────────────────────────────────
inline constexpr uint32_t kCycleDeg = 720U;
inline constexpr uint32_t kMaxSeqInjPwDeg = 648U;      // 90% of 720°
inline constexpr uint32_t kMaxPresyncInjPwDeg = 324U;  // 90% of 360°

#define ECU_SCHED_US_TO_TICKS_INTERNAL(us) ((us) * 125U / 2U)
#define TOOTH_NS_TO_SCHED_INTERNAL(ns) \
    (static_cast<uint32_t>((ns) / ECU_SCHED_NS_PER_TICK))

// Channel order cyl 0..3 — values match ECU_CH_* (legacy TIM map).
inline constexpr uint8_t kInjCh[4] = {
    ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
inline constexpr uint8_t kIgnCh[4] = {
    ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};

// Inhibit mask bit for INJ/IGN channels (cyl 0..3), indexado por ECU_CH_*.
// Movido de ecu_sched.cpp (era static ali) — ecu_sched_angle_encoder.cpp
// precisa da mesma tabela para a varredura de purge da fila TIM2/CH3, e
// duplicar 8 bytes que têm de andar sempre em sincronia com ECU_CH_* é mais
// risco do que partilhar a única fonte.
inline constexpr uint8_t k_inj_ch_to_bit[8] = {
    (1U << 2), (1U << 3), (1U << 0), (1U << 1), 0U, 0U, 0U, 0U
};
inline constexpr uint8_t k_ign_ch_to_bit[8] = {
    0U, 0U, 0U, 0U, (1U << 3), (1U << 2), (1U << 1), (1U << 0)
};

// ── Angle table (defined in ecu_sched_angle.cpp) ────────────────────────────
extern AngleEvent_t g_angle_table[ECU_ANGLE_TABLE_SIZE];
extern uint8_t g_angle_table_count;
extern uint32_t g_angle_tooth_mask_lo;
extern uint32_t g_angle_tooth_mask_hi;

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

// ── Cold builders (ecu_sched_angle.cpp) — called only at rev gap ─────────────
void clear_angle_table(void);
void rebuild_sequential_cycle(const ems::drv::CkpSnapshot& snap);
void rebuild_presync_revolution(const ems::drv::CkpSnapshot& snap);

// ── MT6835/TIM2 encoder — fila TIM2/CH3 (ecu_sched_angle_encoder.cpp) ───────
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
// dwell/injeção (definição em ecu_sched.cpp, ligado sempre a TIM5/tempo —
// os watchdogs não mudam de domínio, ver "Watchdogs — papel novo" no design
// doc). Exposto (era static) para a fila TIM2/CH3 poder disparar o mesmo
// mecanismo quando um evento em counts liga/desliga um pino — sem isto os
// watchdogs nunca armariam para eventos disparados pelo dispatcher em
// ângulo. Escopo global (não dentro de sched_internal) porque é onde a
// definição já vive em ecu_sched.cpp.
void pin_transition(uint8_t idx, uint8_t high, uint8_t is_safe_state = 0U);
