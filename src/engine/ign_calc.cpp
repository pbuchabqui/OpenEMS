#include "engine/ign_calc.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"

#include <cstdint>

namespace {

static uint8_t g_antijerk_cycles_rem = 0u;
static int16_t g_antijerk_active_x10 = 0;

}  // namespace

namespace ems::engine {

int16_t get_advance_x10_prepared(const Table2dLookup& lookup) noexcept {
    return table3d_lookup_i8_x10_prepared(spark_table, lookup);
}

int16_t get_advance_x10(uint32_t rpm_x10, uint16_t load_bar_x100) noexcept {
    return get_advance_x10_prepared(
        table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100, rpm_x10, load_bar_x100));
}

int16_t clamp_advance_x10(int32_t advance_x10) noexcept {
    if (advance_x10 < kAdvanceMinX10) { return kAdvanceMinX10; }
    if (advance_x10 > kAdvanceMaxX10) { return kAdvanceMaxX10; }
    return static_cast<int16_t>(advance_x10);
}

int16_t calc_ign_iat_correction_x10(int16_t iat_x10) noexcept {
    return interp_i16_8pt(iat_spark_axis_x10, iat_spark_corr_deg,
                          kCorrectionTableSize, iat_x10, 10);
}

int16_t calc_ign_clt_correction_x10(int16_t clt_x10) noexcept {
    return interp_i16_8pt(clt_spark_axis_x10, clt_spark_corr_deg,
                          kCorrectionTableSize, clt_x10, 10);
}

int16_t calc_antijerk_retard_x10(int16_t tpsdot_x10) noexcept {
    const int16_t thr = static_cast<int16_t>(antijerk_tpsdot_threshold_x10);
    const int32_t max_ret_x10 = 10 * ((antijerk_retard_deg < 0) ? 0
        : (antijerk_retard_deg > 20 ? 20 : antijerk_retard_deg));

    // Re-arm only on rising tip-in above threshold (not tip-out).
    if (tpsdot_x10 > thr && g_antijerk_cycles_rem == 0u && max_ret_x10 > 0) {
        // Proportional: full retard at 100 %/s (tpsdot_x10=1000).
        int32_t ret = (static_cast<int32_t>(tpsdot_x10) * max_ret_x10 + 500) / 1000;
        if (ret < 1) { ret = 1; }
        if (ret > max_ret_x10) { ret = max_ret_x10; }
        g_antijerk_active_x10 = static_cast<int16_t>(ret);
        g_antijerk_cycles_rem = (antijerk_decay_cycles == 0u) ? 1u : antijerk_decay_cycles;
    }

    if (g_antijerk_cycles_rem > 0u) {
        --g_antijerk_cycles_rem;
        return g_antijerk_active_x10;
    }
    g_antijerk_active_x10 = 0;
    return 0;
}

void antijerk_reset() noexcept {
    g_antijerk_cycles_rem = 0u;
    g_antijerk_active_x10 = 0;
}

int16_t calc_total_advance_x10(int16_t base_x10, AdvanceCorrectionsX10 corr) noexcept {
    return clamp_advance_x10(static_cast<int32_t>(base_x10)
        + corr.iat + corr.clt + corr.idle
        - corr.antijerk_retard - corr.torque_retard);
}

int16_t calc_idle_spark_correction_x10(uint32_t rpm_x10,
                                       uint16_t idle_target_rpm_x10,
                                       uint16_t tps_pct_x10,
                                       uint16_t map_bar_x100) noexcept {
    if (idle_target_rpm_x10 == 0u ||
        tps_pct_x10 > idle_spark_tps_max_x10 ||
        map_bar_x100 > idle_spark_map_max_bar_x100 ||
        rpm_x10 < idle_spark_rpm_min_x10 ||
        rpm_x10 > static_cast<uint32_t>(idle_target_rpm_x10) + idle_spark_window_above_target_x10 ||
        idle_spark_rpm_per_deg_x10 == 0u) {
        return 0;
    }

    int32_t error_x10 = static_cast<int32_t>(idle_target_rpm_x10) -
                        static_cast<int32_t>(rpm_x10);
    const int32_t deadband = static_cast<int32_t>(idle_spark_deadband_rpm_x10);
    if (error_x10 > -deadband && error_x10 < deadband) {
        return 0;
    }

    if (error_x10 > 0) {
        error_x10 -= deadband;
    } else {
        error_x10 += deadband;
    }

    // Proportional gain in 0.1°: error (rpm×10) × 10 / (rpm×10 per degree).
    const int32_t corr_x10 = (error_x10 * 10) / static_cast<int32_t>(idle_spark_rpm_per_deg_x10);
    const int32_t lo = static_cast<int32_t>(idle_spark_retard_limit_deg) * 10;
    const int32_t hi = static_cast<int32_t>(idle_spark_advance_limit_deg) * 10;
    return static_cast<int16_t>(corr_x10 < lo ? lo : (corr_x10 > hi ? hi : corr_x10));
}

uint16_t dwell_ms_x10_from_vbatt(uint16_t vbatt_mv) noexcept {
    return interp_u16_8pt_u16x(dwell_vbatt_axis_mv, dwell_ms_x10_table,
                                kIgnitionDwellTableSize, vbatt_mv);
}

uint16_t dwell_ms_x10_from_vbatt_rpm(uint16_t vbatt_mv, uint32_t rpm_x10) noexcept {
    const uint16_t base = dwell_ms_x10_from_vbatt(vbatt_mv);

    // Eixo RPM armazenado em RPM (não ×10) — converte e clamp a uint16_t.
    const uint32_t rpm = rpm_x10 / 10u;
    const uint16_t rpm_u16 = static_cast<uint16_t>(rpm > 65535u ? 65535u : rpm);

    const uint16_t factor = interp_u16_8pt_u16x(dwell_rpm_axis_rpm, dwell_rpm_factor_q8,
                                                  kDwellRpmCorrSize, rpm_u16);

    // base × factor / 256, arredondado
    const uint32_t result = (static_cast<uint32_t>(base) * factor + 128u) / 256u;
    return static_cast<uint16_t>(result > 65535u ? 65535u : result);
}

uint32_t inj_pw_us_to_scheduler_ticks(uint32_t pw_us) noexcept {
    // Scheduler tick = 16 ns (62.5 MHz TIM5_CNT) — same on host and target.
    return pw_us * 125u / 2u;
}

}  // namespace ems::engine
