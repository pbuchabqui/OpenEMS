#include "engine/ign_calc.h"

#include <cstdint>

namespace {

constexpr uint8_t kDwellPoints = 8u;
constexpr uint16_t kDwellVbattAxisMv[kDwellPoints] = {9000u, 10000u, 11000u, 12000u, 13000u, 14000u, 15000u, 16000u};
constexpr uint16_t kDwellTableMsX10[kDwellPoints] = {42u, 38u, 35u, 30u, 28u, 25u, 23u, 22u};

int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

// Interpolação especializada para a tabela de dwell por tensão de bateria.
// Separada da interp_u16_8pt genérica de fuel_calc.cpp (assinatura diferente:
// eixo embutido vs eixo externo). Renomeada para evitar ambiguidade.
static uint16_t interp_dwell_by_vbatt(uint16_t x) noexcept {
    if (x <= kDwellVbattAxisMv[0]) {
        return kDwellTableMsX10[0];
    }
    if (x >= kDwellVbattAxisMv[kDwellPoints - 1u]) {
        return kDwellTableMsX10[kDwellPoints - 1u];
    }

    uint8_t idx = 0u;
    for (uint8_t i = 0u; i < (kDwellPoints - 1u); ++i) {
        if (x <= kDwellVbattAxisMv[i + 1u]) {
            idx = i;
            break;
        }
    }

    const uint16_t x0 = kDwellVbattAxisMv[idx];
    const uint16_t x1 = kDwellVbattAxisMv[idx + 1u];
    const uint16_t y0 = kDwellTableMsX10[idx];
    const uint16_t y1 = kDwellTableMsX10[idx + 1u];

    const uint32_t dx = static_cast<uint32_t>(x - x0);
    const uint32_t span = static_cast<uint32_t>(x1 - x0);
    if (span == 0u) {
        return y0;
    }

    const int32_t dy = static_cast<int32_t>(y1) - static_cast<int32_t>(y0);
    const int32_t y = static_cast<int32_t>(y0) + static_cast<int32_t>((dy * static_cast<int32_t>(dx)) / static_cast<int32_t>(span));
    if (y <= 0) {
        return 0u;
    }
    if (y >= 65535) {
        return 65535u;
    }
    return static_cast<uint16_t>(y);
}

uint16_t normalize_7200(int32_t deg_x10) noexcept {
    int32_t out = deg_x10 % 7200;
    if (out < 0) {
        out += 7200;
    }
    return static_cast<uint16_t>(out);
}

int16_t lerp_q8_i16(int16_t a, int16_t b, uint8_t frac_q8) noexcept {
    if (frac_q8 == 255u) { return b; }
    return static_cast<int16_t>(a + (((b - a) * static_cast<int16_t>(frac_q8)) >> 8u));
}

}  // namespace

namespace ems::engine {

int8_t spark_table[kTableAxisSize][kTableAxisSize] = {
    {40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40},
    {42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42},
    {44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44},
    {46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46},
    {48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48},
    {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
    {52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52},
    {54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54},
    {56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56},
    {58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58},
    {60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60},
    {62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62},
    {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
    {66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66},
    {68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68},
    {70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70},
};

// CYC-02: firing_order e cylinder_offset_deg são definidos como inline constexpr
// em ign_calc.h — não redefinir aqui para evitar ODR violation.

int16_t get_advance(uint16_t rpm_x10, uint16_t load_kpa) noexcept {
    const uint8_t xi = table_axis_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t yi = table_axis_index(kLoadAxisKpa, kTableAxisSize, load_kpa);
    const uint8_t fx = table_axis_frac_q8(kRpmAxisX10, xi, rpm_x10);
    const uint8_t fy = table_axis_frac_q8(kLoadAxisKpa, yi, load_kpa);

    const int16_t v00 = static_cast<int16_t>(spark_table[yi][xi]);
    const int16_t v10 = static_cast<int16_t>(spark_table[yi][xi + 1u]);
    const int16_t v01 = static_cast<int16_t>(spark_table[yi + 1u][xi]);
    const int16_t v11 = static_cast<int16_t>(spark_table[yi + 1u][xi + 1u]);

    const int16_t v0 = lerp_q8_i16(v00, v10, fx);
    const int16_t v1 = lerp_q8_i16(v01, v11, fx);
    const int16_t raw = lerp_q8_i16(v0, v1, fy);
    return static_cast<int16_t>(raw - 40);
}

int16_t clamp_advance_deg(int16_t advance_deg) noexcept {
    // Limite máximo: 40° BTDC — margem para motores de alta compressão e
    // combustíveis com maior octanagem (ex.: E30). Linha de alta carga da
    // tabela de desenvolvimento chega a 70°; o clamp protege contra valores
    // excessivos. Ajustar após calibração em bancada.
    return clamp_i16(advance_deg, -10, 40);
}

int16_t calc_total_advance(int16_t base_advance_deg,
                           int16_t corr_iat_deg,
                           int16_t corr_clt_deg,
                           int16_t knock_retard_deg) noexcept {
    const int16_t total = static_cast<int16_t>(base_advance_deg + corr_iat_deg + corr_clt_deg - knock_retard_deg);
    return clamp_advance_deg(total);
}

uint16_t dwell_ms_x10_from_vbatt(uint16_t vbatt_mv) noexcept {
    return interp_dwell_by_vbatt(vbatt_mv);
}

uint16_t calc_dwell_angle_x10(uint16_t dwell_ms_x10, uint16_t rpm) noexcept {
    // FIX-2: clamp antes do cast para uint16_t — sem clamp, RPM alto com dwell
    // longo overflow para valor mínimo e dispara ignição no ponto errado.
    // Limite de 3599 = 359,9° (mais de uma rotação inteira não tem sentido físico).
    //
    // Fórmula simplificada: dwell_ms_x10 × rpm × 360 × 10 / 600000
    //                     = dwell_ms_x10 × rpm × 6 / 1000
    // Overflow 32-bit: máx 80 × 12000 × 6 = 5.760.000 < 2^32 ✓ → usa UDIV nativo (1 ciclo)
    const uint32_t raw = (static_cast<uint32_t>(dwell_ms_x10) * rpm * 6u) / 1000u;
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

uint32_t inj_pw_us_to_ftm0_ticks(uint32_t pw_us) noexcept {
    return pw_us * 60u;
}

}  // namespace ems::engine
