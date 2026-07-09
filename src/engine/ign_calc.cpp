#include "engine/ign_calc.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"

#include <cstdint>

namespace {

static uint8_t g_antijerk_cycles_rem = 0u;

uint16_t normalize_7200(int32_t deg_x10) noexcept {
    int32_t out = deg_x10 % 7200;
    if (out < 0) {
        out += 7200;
    }
    return static_cast<uint16_t>(out);
}

int16_t lerp_q8_i16(int16_t a, int16_t b, uint8_t frac_q8) noexcept {
    if (frac_q8 == 255u) { return b; }
    const int32_t delta = static_cast<int32_t>(b) - static_cast<int32_t>(a);
    return static_cast<int16_t>(a + ((delta * static_cast<int32_t>(frac_q8)) >> 8));
}

}  // namespace

namespace ems::engine {

// Trim de ignição vindo do controle coordenado ar+ignição da borboleta (ETB).
// O módulo etb_control fornece a implementação real quando linkado; aqui há um
// default fraco que retorna 0 (sem trim) para manter o caminho de ignição
// auto-contido e seguro quando o ETB não está presente no build.
// NOTA: a integração real precisa expor esta função no namespace ems::engine —
// etb_control.cpp hoje a declara em escopo global (extern "C"), o que não
// resolve esta chamada. Ver findings da revisão.
#if defined(__GNUC__)
__attribute__((weak))
#endif
int16_t etb_get_idle_spark_trim() noexcept { return 0; }



int16_t get_advance(uint32_t rpm_x10, uint16_t load_bar_x100) noexcept {
    const uint8_t xi = table_axis_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t yi = table_axis_index(kLoadAxisBarX100, kTableAxisSize, load_bar_x100);
    const uint8_t fx = table_axis_frac_q8(kRpmAxisX10, xi, rpm_x10);
    const uint8_t fy = table_axis_frac_q8(kLoadAxisBarX100, yi, load_bar_x100);

    const int16_t v00 = static_cast<int16_t>(spark_table[yi][xi]);
    const int16_t v10 = static_cast<int16_t>(spark_table[yi][xi + 1u]);
    const int16_t v01 = static_cast<int16_t>(spark_table[yi + 1u][xi]);
    const int16_t v11 = static_cast<int16_t>(spark_table[yi + 1u][xi + 1u]);

    const int16_t v0 = lerp_q8_i16(v00, v10, fx);
    const int16_t v1 = lerp_q8_i16(v01, v11, fx);
    return lerp_q8_i16(v0, v1, fy);
}

int16_t get_advance_prepared(const Table2dLookup& lookup) noexcept {
    return table3d_lookup_i8_prepared(spark_table, lookup);
}

int16_t clamp_advance_deg(int16_t advance_deg) noexcept {
    // 40° BTDC — margem de octanagem para E30/alta compressão.
    return clamp_i16(advance_deg, -10, 40);
}

int16_t calc_ign_iat_correction_deg(int16_t iat_x10) noexcept {
    return interp_i16_8pt(iat_spark_axis_x10, iat_spark_corr_deg,
                          kCorrectionTableSize, iat_x10);
}

int16_t calc_ign_clt_correction_deg(int16_t clt_x10) noexcept {
    return interp_i16_8pt(clt_spark_axis_x10, clt_spark_corr_deg,
                          kCorrectionTableSize, clt_x10);
}

int16_t calc_antijerk_retard_deg(bool ae_active) noexcept {
    if (ae_active && g_antijerk_cycles_rem == 0u) {
        g_antijerk_cycles_rem = antijerk_decay_cycles;
    }
    if (g_antijerk_cycles_rem > 0u) {
        --g_antijerk_cycles_rem;
        return antijerk_retard_deg;
    }
    return 0;
}

void antijerk_reset() noexcept {
    g_antijerk_cycles_rem = 0u;
}

int16_t calc_total_advance(int16_t base_advance_deg,
                           AdvanceCorrections corr) noexcept {
    const int32_t total = static_cast<int32_t>(base_advance_deg)
        + corr.iat_deg
        + corr.clt_deg
        + corr.idle_spark_deg
        - corr.knock_retard_deg
        - corr.antijerk_retard_deg;
    return clamp_advance_deg(static_cast<int16_t>(
        total < -32768 ? -32768 : total > 32767 ? 32767 : total));
}

int16_t calc_idle_spark_correction_deg(uint32_t rpm_x10,
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

    const int32_t corr = error_x10 / static_cast<int32_t>(idle_spark_rpm_per_deg_x10);
    return clamp_i16(static_cast<int16_t>(corr),
                     idle_spark_retard_limit_deg,
                     idle_spark_advance_limit_deg);
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

uint16_t calc_dwell_angle_x10(uint16_t dwell_ms_x10, uint16_t rpm) noexcept {
    // max intermediate: 420×8000×36 = 120M, fits uint32
    const uint32_t num = static_cast<uint32_t>(dwell_ms_x10) * rpm * 36u;
    const uint32_t raw = num / 6000u;
    return static_cast<uint16_t>(raw > 3599u ? 3599u : raw);
}

int32_t calc_dwell_start_deg_x10(int16_t spark_deg_x10,
                                 uint16_t dwell_ms_x10,
                                 uint16_t rpm) noexcept {
    const uint16_t dwell_angle_x10 = calc_dwell_angle_x10(dwell_ms_x10, rpm);
    return static_cast<int32_t>(spark_deg_x10) + dwell_angle_x10;
}

IgnScheduleParams build_ign_schedule(uint8_t cyl,
                                     int16_t spark_deg_x10,
                                     uint16_t dwell_ms_x10,
                                     uint16_t rpm) noexcept {
    const int32_t dwell_start = calc_dwell_start_deg_x10(spark_deg_x10, dwell_ms_x10, rpm);

    IgnScheduleParams out{};
    out.cyl = static_cast<uint8_t>(cyl & 0x3u);
    out.spark_x10 = normalize_7200(spark_deg_x10);
    out.dwell_start_x10 = normalize_7200(dwell_start);
    return out;
}

uint32_t inj_pw_us_to_scheduler_ticks(uint32_t pw_us) noexcept {
#if defined(TARGET_STM32H562)
    return pw_us * 125u / 2u;  // STM32 scheduler tick = 16 ns (62.5 MHz TIM5_CNT).
#else
    return pw_us * 60u;
#endif
}

}  // namespace ems::engine
