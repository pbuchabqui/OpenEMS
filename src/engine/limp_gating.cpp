#include "engine/limp_gating.h"

#include "engine/calibration.h"
#include "engine/constants.h"
#include "engine/cut_reason.h"
#include "engine/ecu_sched.h"
#include "engine/spark_skip.h"

#include <cstdint>

extern bool     g_rev_limit_active;
extern uint32_t g_dbg_rev_limit_trips;
extern uint32_t g_dbg_rev_limit_rpm_x10;

namespace ems::engine {

uint16_t boost_cut_map_bar_x100          = kBoostCutMapBarX100;
uint16_t oil_min_after_start_bar_x1000   = kOilMinAfterStartBarX1000;
uint16_t lambda_protect_timeout_ms       = kLambdaProtectTimeoutMs;
uint16_t lambda_protect_dev_x1000        = kLambdaProtectDevX1000;
uint32_t lambda_protect_min_rpm_x10      = kLambdaProtectMinRpmX10;
uint16_t lambda_protect_min_load_bar_x100 = kLambdaProtectMinLoadBarX100;

namespace {

struct Hysteresis {
    bool state = false;
    bool test(bool rising, bool falling) noexcept {
        if (rising) {
            state = true;
        } else if (falling) {
            state = false;
        }
        return state;
    }
};

Hysteresis g_rev_hyst;
Hysteresis g_boost_hyst;
Hysteresis g_inj_duty_hyst;

bool     g_fatal                 = false;
bool     g_etb_problem           = false;
uint32_t g_fault_rev_x10         = 0xFFFFFFFFu;
bool     g_had_oil_after_start   = false;
bool     g_seen_running          = false;
uint32_t g_run_start_ms          = 0u;
bool     g_lambda_cut            = false;
bool     g_lambda_seen           = false;
uint32_t g_lambda_good_ms        = 0u;
uint32_t g_oil_low_ms            = 0u;
bool     g_oil_low_seen          = false;

bool g_allow_etb      = true;
bool g_allow_inj      = true;
bool g_allow_ign      = true;
bool g_fuel_protect   = false;
bool g_half_lockout   = false;
bool g_rev_active     = false;
bool g_sensor_bypass  = false;

void set_fault_rev_limit_x10(uint32_t limit_x10) noexcept {
    if (limit_x10 < g_fault_rev_x10) {
        g_fault_rev_x10 = limit_x10;
    }
}

void commit_masks(uint8_t inj, uint8_t ign) noexcept {
    ::ecu_sched_set_inj_inhibit_mask(inj);
    ::ecu_sched_set_ign_inhibit_mask(ign);
}

}  // namespace

bool limp_gating_allow_etb() noexcept { return g_allow_etb; }
bool limp_gating_allow_injection() noexcept { return g_allow_inj; }
bool limp_gating_allow_ignition() noexcept { return g_allow_ign; }
bool limp_gating_fuel_protect() noexcept { return g_fuel_protect; }
bool limp_gating_half_fuel_lockout() noexcept { return g_half_lockout; }

void limp_gating_set_sensor_bypass(uint8_t on) noexcept {
    g_sensor_bypass = (on != 0);
    if (g_sensor_bypass) {
        g_lambda_cut = false;
        g_lambda_seen = false;
        g_oil_low_seen = false;
        g_had_oil_after_start = true;  // don't trip the 5 s window on release
    }
}

uint8_t limp_gating_sensor_bypass(void) noexcept {
    return g_sensor_bypass ? 1u : 0u;
}

void limp_gating_or_fuel_reason(uint16_t bit) noexcept {
    g_fuel_cut_reasons = static_cast<uint16_t>(g_fuel_cut_reasons | bit);
}

void limp_gating_reset() noexcept {
    g_rev_hyst.state = false;
    g_boost_hyst.state = false;
    g_inj_duty_hyst.state = false;
    g_fatal = false;
    g_etb_problem = false;
    g_fault_rev_x10 = 0xFFFFFFFFu;
    g_had_oil_after_start = false;
    g_seen_running = false;
    g_run_start_ms = 0u;
    g_lambda_cut = false;
    g_lambda_seen = false;
    g_lambda_good_ms = 0u;
    g_oil_low_ms = 0u;
    g_oil_low_seen = false;
    g_allow_etb = true;
    g_allow_inj = true;
    g_allow_ign = true;
    g_fuel_protect = false;
    g_half_lockout = false;
    g_rev_active = false;
    g_sensor_bypass = false;
    g_rev_limit_active = false;
    boost_cut_map_bar_x100 = kBoostCutMapBarX100;
    oil_min_after_start_bar_x1000 = kOilMinAfterStartBarX1000;
    lambda_protect_timeout_ms = kLambdaProtectTimeoutMs;
    lambda_protect_dev_x1000 = kLambdaProtectDevX1000;
    lambda_protect_min_rpm_x10 = kLambdaProtectMinRpmX10;
    lambda_protect_min_load_bar_x100 = kLambdaProtectMinLoadBarX100;
    g_fuel_cut_reasons = 0u;
    g_spark_cut_reasons = 0u;
    commit_masks(0u, 0u);
}

void limp_gating_fatal() noexcept {
    g_fatal = true;
    g_allow_etb = false;
    g_allow_inj = false;
    g_allow_ign = false;
    g_fuel_protect = true;
    set_fault_rev_limit_x10(0u);
    g_fuel_cut_reasons = kFuelCutFatal;
    g_spark_cut_reasons = kSparkCutFatal;
    commit_masks(0x0Fu, 0x0Fu);
}

void limp_gating_report_etb_problem() noexcept {
    g_etb_problem = true;
    g_allow_etb = false;
    set_fault_rev_limit_x10(kEtbFaultRevLimitRpmX10);
}

LimpGatingResult limp_gating_update(const LimpGatingInputs& in) noexcept {
    const bool bypass = g_sensor_bypass;
    if (in.etb_fault && !bypass) {
        limp_gating_report_etb_problem();
    }

    const uint32_t hard = rev_limit_rpm_x10;
    const uint32_t hyst = rev_limit_soft_window_x10;
    const uint32_t resume = (hard > hyst) ? hard - hyst : 0u;
    const bool rev_cut = g_rev_hyst.test(in.rpm_x10 >= hard, in.rpm_x10 <= resume);
    if (rev_cut && !g_rev_active) {
        ++g_dbg_rev_limit_trips;
        g_dbg_rev_limit_rpm_x10 = in.rpm_x10;
    }
    g_rev_active = rev_cut;
    g_rev_limit_active = rev_cut;

    // Oil range-fault is fuel-only (FOME). Spark stays so wasted-spark on a
    // bench without an oil sensor still pulses. Instant idle kill made the
    // encoder stim go silent (floating OIL ADC → fault → ign mask 0x0F).
    const bool fuel_rail_cut =
        in.fuel_press_fault && (in.rpm_x10 > kFuelRailProtectRpmX10);
    const bool overtemp_cut =
        in.overtemp_crit && (in.rpm_x10 > kOilProtectRpmX10);

    const bool running = !in.cranking && (in.rpm_x10 > 0u);
    bool oil_after_start_cut = false;
    if (!running) {
        g_had_oil_after_start = false;
        g_seen_running = false;
        g_oil_low_seen = false;
    } else {
        if (!g_seen_running) {
            g_run_start_ms = in.now_ms;
            g_seen_running = true;
        }
        const uint16_t min_oil = oil_min_after_start_bar_x1000;
        const uint32_t elapsed = in.now_ms - g_run_start_ms;
        if (bypass) {
            // UI sensor-bypass: do not accumulate oil-cut timers.
        } else if (in.oil_fault) {
            // Dead sensor: same 5 s window as "never saw pressure".
            if (elapsed > kOilAfterStartTimeoutMs) {
                oil_after_start_cut = true;
            }
        } else if (min_oil > 0u) {
            if (elapsed <= kOilAfterStartTimeoutMs) {
                if (in.oil_press_bar_x1000 >= min_oil) {
                    g_had_oil_after_start = true;
                }
            }
            if (elapsed > kOilAfterStartTimeoutMs && !g_had_oil_after_start) {
                oil_after_start_cut = true;
            }
        }
        // Running protect: live sensor below min for kOilRunningTimeoutMs.
        if (!bypass && min_oil > 0u && !in.oil_fault &&
            (in.rpm_x10 > kOilProtectRpmX10)) {
            if (in.oil_press_bar_x1000 >= min_oil) {
                g_oil_low_seen = false;
            } else {
                if (!g_oil_low_seen) {
                    g_oil_low_ms = in.now_ms;
                    g_oil_low_seen = true;
                }
                if ((in.now_ms - g_oil_low_ms) >= kOilRunningTimeoutMs) {
                    oil_after_start_cut = true;
                }
            }
        }
    }

    const uint16_t map_cut = boost_cut_map_bar_x100;
    bool boost_cut = false;
    if (map_cut != 0u) {
        const uint16_t fall = (map_cut > kBoostCutHystBarX100)
            ? static_cast<uint16_t>(map_cut - kBoostCutHystBarX100)
            : 0u;
        boost_cut = g_boost_hyst.test(in.map_bar_x100 > map_cut,
                                      in.map_bar_x100 < fall);
    } else {
        g_boost_hyst.state = false;
    }

    const bool inj_duty_hold = g_inj_duty_hyst.test(
        in.inj_duty_cut, in.inj_duty_pct < kInjDutyResumePct);

    const bool engine_phase_cut = in.sequential && !in.phase_valid;
    const bool no_sync_running =
        !in.full_sync && !in.half_sync && !in.cranking && (in.rpm_x10 > 0u);

    const uint16_t lam_timeout = lambda_protect_timeout_ms;
    if (bypass || lam_timeout == 0u) {
        g_lambda_cut = false;
        g_lambda_seen = false;
    } else {
        const bool in_band =
            (in.rpm_x10 >= lambda_protect_min_rpm_x10) &&
            (in.map_bar_x100 >= lambda_protect_min_load_bar_x100);
        const uint16_t max_lam = static_cast<uint16_t>(
            in.lambda_target_x1000 + lambda_protect_dev_x1000);
        const bool lean = in.lambda_valid && (in.lambda_x1000 > max_lam);
        const bool currently_good = !in_band || !lean;
        if (!g_lambda_seen || currently_good) {
            g_lambda_good_ms = in.now_ms;
            g_lambda_seen = true;
        }
        if (!g_lambda_cut &&
            (in.now_ms - g_lambda_good_ms) >= static_cast<uint32_t>(lam_timeout)) {
            g_lambda_cut = true;
        }
        // FOME restore: lean-out cut lifts only after leaving the load/rpm/TPS
        // band (driver lift), not the first good AFR sample at WOT.
        const bool restore_band =
            (in.rpm_x10 < lambda_protect_min_rpm_x10) ||
            (in.map_bar_x100 < lambda_protect_min_load_bar_x100) ||
            (in.tps_pct_x10 < 200u);
        if (g_lambda_cut && in.lambda_valid &&
            (in.lambda_x1000 <= max_lam) && restore_band) {
            g_lambda_cut = false;
            g_lambda_good_ms = in.now_ms;
        }
    }

    const bool etb_rev_cut =
        !bypass && g_etb_problem && (in.rpm_x10 > g_fault_rev_x10);
    const bool fatal_rev_cut = g_fatal && (in.rpm_x10 > g_fault_rev_x10);

    const bool fuel_protect =
        g_fatal || fatal_rev_cut || engine_phase_cut ||
        (!bypass && (in.limp_rpm_cut || in.map_fault || fuel_rail_cut ||
                     overtemp_cut || in.diag_critical || oil_after_start_cut));
    // HALF_SYNC is TIM2 360° absolute without CMP — wasted spark + semi-seq
    // fuel are the intended presync path, not "unknown crank". Lock fuel
    // only on flood/protect, or when there is no sync at all.
    const bool half_lockout =
        (in.half_sync && (in.flood_clear || fuel_protect)) ||
        no_sync_running;

    const bool inj_cut =
        fuel_protect || rev_cut || half_lockout || in.flood_clear ||
        etb_rev_cut ||
        (!bypass && (inj_duty_hold || boost_cut || g_lambda_cut));
    const bool ign_cut =
        g_fatal || engine_phase_cut ||
        (!bypass && (in.limp_rpm_cut || overtemp_cut || in.diag_critical));

    uint16_t fr = 0u;
    uint16_t sr = 0u;
    if (g_fatal)            { fr |= kFuelCutFatal;     sr |= kSparkCutFatal; }
    if (rev_cut)            { fr |= kFuelCutRevLimit; }
    if (!bypass && in.limp_rpm_cut)    { fr |= kFuelCutLimpRpm;   sr |= kSparkCutLimpRpm; }
    if (!bypass && in.map_fault)       { fr |= kFuelCutMapFault; }
    if (!bypass && oil_after_start_cut) {
        fr |= kFuelCutOilPress;
    }
    if (!bypass && fuel_rail_cut)      { fr |= kFuelCutFuelRail; }
    if (!bypass && overtemp_cut)       { fr |= kFuelCutOvertemp;  sr |= kSparkCutOvertemp; }
    if (!bypass && in.diag_critical)   { fr |= kFuelCutDiagCrit;  sr |= kSparkCutDiagCrit; }
    if (half_lockout || engine_phase_cut) {
        fr |= kFuelCutNoSync;
        if (engine_phase_cut) { sr |= kSparkCutNoSync; }
    }
    if (in.flood_clear)     { fr |= kFuelCutFlood; }
    if (!bypass && inj_duty_hold)      { fr |= kFuelCutInjDuty; }
    if (!bypass && boost_cut)          { fr |= kFuelCutBoost; }
    if (!bypass && g_lambda_cut)       { fr |= kFuelCutLambda; }
    if (etb_rev_cut)        { fr |= kFuelCutEtbFault; }

    const uint8_t skip = spark_skip_mask();
    if (skip != 0u) {
        sr |= kSparkSkipActive;
    }

    const uint8_t inj_mask = inj_cut ? 0x0Fu : 0u;
    const uint8_t ign_mask = static_cast<uint8_t>(
        (ign_cut ? 0x0Fu : 0u) | skip);

    g_allow_etb = !g_fatal && (bypass || !g_etb_problem);
    g_allow_inj = !inj_cut;
    g_allow_ign = !ign_cut;
    g_fuel_protect = fuel_protect;
    g_half_lockout = half_lockout;

    g_fuel_cut_reasons  = fr;
    g_spark_cut_reasons = sr;
    commit_masks(inj_mask, ign_mask);

    LimpGatingResult out{};
    out.inj_inhibit_mask  = inj_mask;
    out.ign_inhibit_mask  = ign_mask;
    out.etb_allow         = g_allow_etb;
    out.allow_injection   = g_allow_inj;
    out.allow_ignition    = g_allow_ign;
    out.rev_limit_active  = rev_cut;
    out.fuel_protect_cut  = fuel_protect;
    out.half_fuel_lockout = half_lockout;
    return out;
}

}  // namespace ems::engine
