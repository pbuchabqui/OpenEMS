/**
 * @file limp_gating.h
 * @brief Single writer of fuel/spark/ETB allow (FOME LimpManager, OpenEMS style).
 *
 * limp_gating_update() is the only path that computes allow flags, writes
 * g_fuel_cut_reasons / g_spark_cut_reasons, and commits inj/ign inhibit masks.
 */
#pragma once

#include <cstdint>

namespace ems::engine {

struct LimpGatingInputs {
    uint32_t rpm_x10;
    uint16_t map_bar_x100;
    int16_t  clt_degc_x10;
    uint16_t oil_press_bar_x1000;
    bool     oil_fault;
    bool     fuel_press_fault;
    uint16_t lambda_x1000;
    bool     lambda_valid;
    uint16_t lambda_target_x1000;
    uint16_t tps_pct_x10;
    bool     cranking;
    bool     full_sync;
    bool     half_sync;
    bool     phase_valid;
    bool     sequential;
    bool     etb_fault;
    uint8_t  inj_duty_pct;
    bool     inj_duty_cut;
    uint32_t now_ms;
    bool     map_fault;
    bool     overtemp_crit;
    bool     diag_critical;
    bool     flood_clear;
    bool     limp_rpm_cut;
};

struct LimpGatingResult {
    uint8_t  inj_inhibit_mask;
    uint8_t  ign_inhibit_mask;
    bool     etb_allow;
    bool     allow_injection;
    bool     allow_ignition;
    bool     rev_limit_active;
    bool     fuel_protect_cut;
    bool     half_fuel_lockout;
};

// Runtime knobs (0 = disabled so benches do not trip).
extern uint16_t boost_cut_map_bar_x100;
extern uint16_t oil_min_after_start_bar_x1000;
extern uint16_t lambda_protect_timeout_ms;
extern uint16_t lambda_protect_dev_x1000;
extern uint32_t lambda_protect_min_rpm_x10;
extern uint16_t lambda_protect_min_load_bar_x100;

LimpGatingResult limp_gating_update(const LimpGatingInputs& in) noexcept;

void limp_gating_fatal() noexcept;
void limp_gating_report_etb_problem() noexcept;
void limp_gating_reset() noexcept;

bool limp_gating_allow_etb() noexcept;
bool limp_gating_allow_injection() noexcept;
bool limp_gating_allow_ignition() noexcept;
bool limp_gating_fuel_protect() noexcept;
bool limp_gating_half_fuel_lockout() noexcept;

// DFCO is decided later in the fuel path; still a single writer of the bit.
void limp_gating_or_fuel_reason(uint16_t bit) noexcept;

}  // namespace ems::engine
