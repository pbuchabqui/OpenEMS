#include "engine/ign_calc.h"
#include "engine/calibration.h"
#include "engine/fuel_calc.h"
#include "engine/vehicle_inputs.h"
#include "hal/system.h"
#include "engine/math_utils.h"

#include <cstdint>

namespace {

// Anti-jerk: janela em tempo (ms), pico mantido e rampa linear até 0.
static bool     g_aj_active = false;
static bool     g_aj_lock = false;       // histerese: só rearma depois que tpsdot ≤ limiar
static int32_t  g_aj_peak_x10 = 0;
static uint32_t g_aj_elapsed_ms = 0u;
static uint32_t g_aj_last_ms = 0u;
static bool     g_aj_have_last = false;

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

int16_t calc_antijerk_retard_x10(int16_t tpsdot_x10, uint32_t now_ms, uint32_t rpm_x10) noexcept {
    const int16_t thr = static_cast<int16_t>(antijerk_tpsdot_threshold_x10);
    const int32_t max_ret_x10 = 10 * ((antijerk_retard_deg < 0) ? 0
        : (antijerk_retard_deg > 20 ? 20 : antijerk_retard_deg));

    uint32_t dt_ms = 0u;
    if (g_aj_have_last) {
        dt_ms = now_ms - g_aj_last_ms;
        if (dt_ms > 50u) { dt_ms = 50u; }
    }
    g_aj_have_last = true;
    g_aj_last_ms = now_ms;

    // Duração: calibração (unidades de 10 ms) com piso de 2 voltas — o avanço só chega ao
    // scheduler uma vez por volta, então uma janela mais curta pode nunca ser vista.
    uint32_t dur_ms = static_cast<uint32_t>(antijerk_decay_x10ms) * 10u;
    if (rpm_x10 != 0u) {
        const uint32_t two_revs_ms = 1200000u / rpm_x10;  // 2 × 60000 ms × 10 / rpm_x10
        if (dur_ms < two_revs_ms) { dur_ms = two_revs_ms; }
    }
    if (dur_ms > 500u) { dur_ms = 500u; }
    if (dur_ms == 0u) { dur_ms = 1u; }

    if (tpsdot_x10 <= thr) { g_aj_lock = false; }

    bool armed_now = false;
    if (tpsdot_x10 > thr && max_ret_x10 > 0 && !g_aj_lock) {
        // Proporcional: retardo cheio a 100 %/s (tpsdot_x10 = 1000).
        int32_t ret = (static_cast<int32_t>(tpsdot_x10) * max_ret_x10 + 500) / 1000;
        if (ret < 1) { ret = 1; }
        if (ret > max_ret_x10) { ret = max_ret_x10; }
        if (!g_aj_active) {
            g_aj_active = true;
            g_aj_elapsed_ms = 0u;
            g_aj_peak_x10 = ret;
            armed_now = true;      // 1º passo devolve o pico cheio; a rampa conta a partir daqui
        } else if (ret > g_aj_peak_x10) {
            g_aj_peak_x10 = ret;   // o pico do tip-in vale durante a janela
        }
    }

    if (!g_aj_active) { return 0; }

    if (!armed_now) { g_aj_elapsed_ms += dt_ms; }
    if (g_aj_elapsed_ms >= dur_ms) {
        g_aj_active = false;
        g_aj_peak_x10 = 0;
        g_aj_lock = (tpsdot_x10 > thr);   // pedal ainda subindo: espera cair abaixo do limiar
        return 0;
    }
    return static_cast<int16_t>((g_aj_peak_x10 * static_cast<int32_t>(dur_ms - g_aj_elapsed_ms)) /
                                static_cast<int32_t>(dur_ms));
}

void antijerk_reset() noexcept {
    g_aj_active = false;
    g_aj_lock = false;
    g_aj_peak_x10 = 0;
    g_aj_elapsed_ms = 0u;
    g_aj_have_last = false;
}

int16_t calc_total_advance_x10(int16_t base_x10, AdvanceCorrectionsX10 corr) noexcept {
    return clamp_advance_x10(static_cast<int32_t>(base_x10)
        + corr.iat + corr.clt + corr.idle
        - corr.antijerk_retard - corr.torque_retard - corr.overrun_retard);
}

int16_t ign_running_advance_x10(int16_t computed_x10) noexcept {
    return (timing_light_enable != 0u) ? clamp_advance_x10(timing_light_advance_x10)
                                       : computed_x10;
}

int16_t calc_overrun_spark_retard_x10(uint32_t rpm_x10,
                                      uint16_t tps_pct_x10,
                                      uint16_t map_bar_x100,
                                      uint16_t idle_target_rpm_x10) noexcept {
    uint8_t deg = overrun_spark_retard_deg;
    if (deg == 0u) {
        return 0;
    }
    if (deg > 30u) {
        deg = 30u;
    }
    const int16_t full = static_cast<int16_t>(static_cast<int16_t>(deg) * 10);
    const bool tps_closed = tps_pct_x10 <= decel_cut_tps_threshold_x10;
    const bool map_low = (decel_cut_map_max_bar_x100 == 0u) ||
                         (map_bar_x100 <= decel_cut_map_max_bar_x100);
    const uint32_t above_idle = static_cast<uint32_t>(idle_target_rpm_x10)
                              + idle_spark_window_above_target_x10;
    const bool trailing = tps_closed && map_low && (rpm_x10 > above_idle);
    if (fuel_decel_cut_active() || trailing) {
        return full;
    }
    const uint8_t frac = fuel_decel_cut_ramp_frac_q8();
    if (frac >= 255u) {
        return 0;
    }
    return static_cast<int16_t>(
        (static_cast<int32_t>(full) * static_cast<int32_t>(255u - frac)) / 255);
}

namespace {
uint32_t g_dl_rpm_prev = 0u;
uint32_t g_dl_rpm_prev2 = 0u;
uint8_t  g_dl_hist = 0u;
int16_t  g_dl_retard_x10 = 0;
}  // namespace

void driveline_reset() noexcept {
    g_dl_rpm_prev = 0u;
    g_dl_rpm_prev2 = 0u;
    g_dl_hist = 0u;
    g_dl_retard_x10 = 0;
}

int16_t calc_driveline_retard_x10(uint32_t rpm_x10,
                                  int16_t clt_x10,
                                  uint16_t map_bar_x100) noexcept {
    if (driveline_enable == 0u || driveline_max_retard_deg == 0u) {
        driveline_reset();
        return 0;
    }
    uint8_t deg = driveline_max_retard_deg;
    if (deg > 30u) {
        deg = 30u;
    }
    const int16_t full = static_cast<int16_t>(static_cast<int16_t>(deg) * 10);

    bool window = (clt_x10 >= driveline_clt_min_x10) &&
                  (rpm_x10 >= driveline_rpm_min_x10) &&
                  (rpm_x10 <= driveline_rpm_max_x10) &&
                  (map_bar_x100 >= driveline_map_min_bar_x100);
    if (driveline_speed_min_kmh != 0u || driveline_speed_max_kmh != 0u) {
        uint16_t kmh = 0u;
        if (vehicle_speed_kmh(kmh, ::millis())) {
            if (driveline_speed_min_kmh != 0u &&
                kmh < driveline_speed_min_kmh) {
                window = false;
            }
            if (driveline_speed_max_kmh != 0u &&
                kmh > driveline_speed_max_kmh) {
                window = false;
            }
        } else {
            // Velocidade inválida: com gate configurado, não retarda.
            window = false;
        }
    }

    bool have_dd = false;
    int32_t dd = 0;
    if (g_dl_hist >= 2u) {
        dd = static_cast<int32_t>(rpm_x10)
           - (static_cast<int32_t>(g_dl_rpm_prev) * 2)
           + static_cast<int32_t>(g_dl_rpm_prev2);
        have_dd = true;
    }
    g_dl_rpm_prev2 = g_dl_rpm_prev;
    g_dl_rpm_prev = rpm_x10;
    if (g_dl_hist < 2u) {
        ++g_dl_hist;
    }

    const int32_t thresh = (driveline_dd_thresh_rpm_x10 == 0u)
        ? 50
        : static_cast<int32_t>(driveline_dd_thresh_rpm_x10);
    int16_t target = 0;
    if (window && have_dd && (dd >= thresh || dd <= -thresh)) {
        target = full;
    }
    int16_t step = static_cast<int16_t>(full / 4);
    if (step < 1) {
        step = 1;
    }
    if (g_dl_retard_x10 < target) {
        const int32_t next = static_cast<int32_t>(g_dl_retard_x10) + step;
        g_dl_retard_x10 = static_cast<int16_t>(next > target ? target : next);
    } else if (g_dl_retard_x10 > target) {
        const int32_t next = static_cast<int32_t>(g_dl_retard_x10) - step;
        g_dl_retard_x10 = static_cast<int16_t>(next < target ? target : next);
    }
    if (g_dl_retard_x10 < 0) {
        g_dl_retard_x10 = 0;
    }
    return g_dl_retard_x10;
}

int16_t calc_idle_spark_correction_x10(uint32_t rpm_x10,
                                       uint16_t idle_target_rpm_x10,
                                       uint16_t tps_pct_x10,
                                       uint16_t map_bar_x100) noexcept {
    if (idle_target_rpm_x10 == 0u ||
        tps_pct_x10 > idle_spark_tps_max_x10 ||
        map_bar_x100 > idle_spark_map_max_bar_x100 ||
        rpm_x10 < idle_spark_rpm_min_x10 ||
        rpm_x10 > static_cast<uint32_t>(idle_target_rpm_x10) + idle_spark_window_above_target_x10) {
        return 0;
    }

    int32_t error_x10 = static_cast<int32_t>(idle_target_rpm_x10) -
                        static_cast<int32_t>(rpm_x10);
    const int32_t deadband = static_cast<int32_t>(idle_spark_deadband_rpm_x10);
    if (error_x10 > -deadband && error_x10 < deadband) {
        return 0;
    }

    const int32_t lo = static_cast<int32_t>(idle_spark_retard_limit_deg) * 10;
    const int32_t hi = static_cast<int32_t>(idle_spark_advance_limit_deg) * 10;

    bool curve = false;
    for (uint8_t i = 0u; i < kIdleSparkCurvePts; ++i) {
        if (idle_spark_err_corr_x10[i] != 0) {
            curve = true;
            break;
        }
    }
    if (!curve) {
        if (idle_spark_rpm_per_deg_x10 == 0u) {
            return 0;
        }
        if (error_x10 > 0) {
            error_x10 -= deadband;
        } else {
            error_x10 += deadband;
        }
        // Proportional gain in 0.1 deg: error (rpm x10) x 10 / (rpm x10 per degree).
        const int32_t corr_x10 =
            (error_x10 * 10) / static_cast<int32_t>(idle_spark_rpm_per_deg_x10);
        return static_cast<int16_t>(corr_x10 < lo ? lo : (corr_x10 > hi ? hi : corr_x10));
    }

    // The curve reads the raw error (target - rpm); outside the axis it holds the end value.
    constexpr int16_t kErrAxis[kIdleSparkCurvePts] = {-4000, -2000, -500, 500, 2000, 4000};
    int32_t raw = error_x10;
    if (raw < kErrAxis[0]) {
        raw = kErrAxis[0];
    } else if (raw > kErrAxis[kIdleSparkCurvePts - 1u]) {
        raw = kErrAxis[kIdleSparkCurvePts - 1u];
    }
    const int32_t corr = interp_i16_8pt(kErrAxis, idle_spark_err_corr_x10, kIdleSparkCurvePts,
                                        static_cast<int16_t>(raw));
    return static_cast<int16_t>(corr < lo ? lo : (corr > hi ? hi : corr));
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
