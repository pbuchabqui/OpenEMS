/**
 * @file ecu_sched_internal.h
 * @brief Shared state between ecu_sched.cpp (hot path) and ecu_sched_angle.cpp
 *        (cold angle-table builders at rev boundary only).
 *
 * Hot-path rule (Tier 1.5): do NOT move evt_insert / gpio_set_pin /
 * arm_channel / ecu_sched_evt_dispatch / tooth arm loop out of ecu_sched.cpp.
 * Those stay co-located so the compiler can inline without relying on LTO.
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

}  // namespace ems::engine::sched_internal
