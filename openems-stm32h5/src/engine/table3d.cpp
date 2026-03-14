#include "engine/table3d.h"

namespace ems::engine {

const uint16_t kRpmAxisX10[kTableAxisSize] = {
    500u,  750u,  1000u, 1250u,
    1500u, 2000u, 2500u, 3000u,
    4000u, 5000u, 6000u, 7000u,
    8000u, 9000u, 10000u, 12000u,
};

const uint16_t kLoadAxisKpa[kTableAxisSize] = {
    20u,  30u,  40u,  50u,
    60u,  70u,  80u,  90u,
    100u, 110u, 120u, 130u,
    140u, 150u, 175u, 200u,
};

uint8_t table_axis_index(const uint16_t* axis, uint8_t size, uint16_t value) noexcept {
    if (size < 2u) {
        return 0u;
    }
    if (value <= axis[0]) {
        return 0u;
    }
    const uint8_t last = static_cast<uint8_t>(size - 1u);
    if (value >= axis[last]) {
        return static_cast<uint8_t>(last - 1u);
    }

    // Busca binária O(log n): eixos são sempre ordenados crescentes.
    // Retorna i tal que axis[i] < value <= axis[i+1].
    uint8_t lo = 0u;
    uint8_t hi = static_cast<uint8_t>(size - 2u);
    while (lo < hi) {
        const uint8_t mid = static_cast<uint8_t>(lo + ((hi - lo) >> 1u));
        if (value > axis[mid + 1u]) {
            lo = static_cast<uint8_t>(mid + 1u);
        } else {
            hi = mid;
        }
    }
    return lo;
}

uint8_t table_axis_frac_q8(const uint16_t* axis, uint8_t idx, uint16_t value) noexcept {
    const uint16_t x0 = axis[idx];
    const uint16_t x1 = axis[idx + 1u];

    if (value <= x0) {
        return 0u;
    }
    if (value >= x1) {
        return 255u;
    }

    const uint16_t span = static_cast<uint16_t>(x1 - x0);
    if (span == 0u) {
        return 0u;
    }

    const uint32_t num = static_cast<uint32_t>(value - x0) << 8u;
    uint32_t frac = num / span;
    if (frac > 255u) {
        frac = 255u;
    }
    return static_cast<uint8_t>(frac);
}

static __attribute__((always_inline)) int32_t lerp_q8_s32(int32_t a, int32_t b, uint8_t frac_q8) noexcept {
    if (frac_q8 == 255u) { return b; }
    return a + (((b - a) * static_cast<int32_t>(frac_q8)) >> 8u);
}

uint8_t table3d_lookup_u8(const uint8_t table[kTableAxisSize][kTableAxisSize],
                          const uint16_t* x_axis,
                          const uint16_t* y_axis,
                          uint16_t x,
                          uint16_t y) noexcept {
    const uint8_t xi = table_axis_index(x_axis, kTableAxisSize, x);
    const uint8_t yi = table_axis_index(y_axis, kTableAxisSize, y);
    const uint8_t fx = table_axis_frac_q8(x_axis, xi, x);
    const uint8_t fy = table_axis_frac_q8(y_axis, yi, y);

    const int32_t v00 = table[yi][xi];
    const int32_t v10 = table[yi][xi + 1u];
    const int32_t v01 = table[yi + 1u][xi];
    const int32_t v11 = table[yi + 1u][xi + 1u];

    const int32_t v0 = lerp_q8_s32(v00, v10, fx);
    const int32_t v1 = lerp_q8_s32(v01, v11, fx);
    const int32_t v = lerp_q8_s32(v0, v1, fy);

    if (v <= 0) {
        return 0u;
    }
    if (v >= 255) {
        return 255u;
    }
    return static_cast<uint8_t>(v);
}

// Nova função otimizada para VE lookup com Q8 arithmetic
uint16_t table3d_lookup_ve_q8(const uint8_t ve_table[kTableAxisSize][kTableAxisSize],
                             const uint16_t* x_axis,
                             const uint16_t* y_axis,
                             uint16_t x,
                             uint16_t y) noexcept {
    const uint8_t xi = table_axis_index(x_axis, kTableAxisSize, x);
    const uint8_t yi = table_axis_index(y_axis, kTableAxisSize, y);
    const uint8_t fx = table_axis_frac_q8(x_axis, xi, x);
    const uint8_t fy = table_axis_frac_q8(y_axis, yi, y);

    // Carrega valores da tabela (já em Q8 scale)
    const uint16_t v00 = static_cast<uint16_t>(ve_table[yi][xi]) << 8;
    const uint16_t v10 = static_cast<uint16_t>(ve_table[yi][xi + 1]) << 8;
    const uint16_t v01 = static_cast<uint16_t>(ve_table[yi + 1][xi]) << 8;
    const uint16_t v11 = static_cast<uint16_t>(ve_table[yi + 1][xi + 1]) << 8;

    // Interpolação Q8 com aritmética signed explícita (FIX-1: evita right-shift
    // em valor negativo — implementation-defined em C++17; int32_t garante
    // comportamento correto em tabelas decrescentes).
    const int32_t d0  = (static_cast<int32_t>(v10) - static_cast<int32_t>(v00)) * static_cast<int32_t>(fx);
    const uint16_t v0 = static_cast<uint16_t>(static_cast<int32_t>(v00) + (d0 >> 8));
    const int32_t d1  = (static_cast<int32_t>(v11) - static_cast<int32_t>(v01)) * static_cast<int32_t>(fx);
    const uint16_t v1 = static_cast<uint16_t>(static_cast<int32_t>(v01) + (d1 >> 8));
    const int32_t dv  = (static_cast<int32_t>(v1) - static_cast<int32_t>(v0)) * static_cast<int32_t>(fy);
    const uint16_t v  = static_cast<uint16_t>(static_cast<int32_t>(v0) + (dv >> 8));

    return v;  // Resultado em Q8
}

// Função para advance lookup com Q10
int32_t table3d_lookup_advance_q10(const int16_t advance_table[kTableAxisSize][kTableAxisSize],
                                 const uint16_t* x_axis,
                                 const uint16_t* y_axis,
                                 uint16_t x,
                                 uint16_t y) noexcept {
    const uint8_t xi = table_axis_index(x_axis, kTableAxisSize, x);
    const uint8_t yi = table_axis_index(y_axis, kTableAxisSize, y);
    const uint8_t fx = table_axis_frac_q8(x_axis, xi, x);
    const uint8_t fy = table_axis_frac_q8(y_axis, yi, y);

    // Carrega valores da tabela (converte para Q10)
    const int32_t v00 = static_cast<int32_t>(advance_table[yi][xi]) << 10;
    const int32_t v10 = static_cast<int32_t>(advance_table[yi][xi + 1]) << 10;
    const int32_t v01 = static_cast<int32_t>(advance_table[yi + 1][xi]) << 10;
    const int32_t v11 = static_cast<int32_t>(advance_table[yi + 1][xi + 1]) << 10;

    // Interpolação Q10 otimizada
    const int32_t v0 = v00 + (((v10 - v00) * static_cast<int32_t>(fx)) >> 8);
    const int32_t v1 = v01 + (((v11 - v01) * static_cast<int32_t>(fx)) >> 8);
    const int32_t v = v0 + (((v1 - v0) * static_cast<int32_t>(fy)) >> 8);

    return v;  // Resultado em Q10 (150 = 15.0° BTDC)
}

int16_t table3d_lookup_s16(const int16_t table[kTableAxisSize][kTableAxisSize],
                           const uint16_t* x_axis,
                           const uint16_t* y_axis,
                           uint16_t x,
                           uint16_t y) noexcept {
    const uint8_t xi = table_axis_index(x_axis, kTableAxisSize, x);
    const uint8_t yi = table_axis_index(y_axis, kTableAxisSize, y);
    const uint8_t fx = table_axis_frac_q8(x_axis, xi, x);
    const uint8_t fy = table_axis_frac_q8(y_axis, yi, y);

    const int32_t v00 = table[yi][xi];
    const int32_t v10 = table[yi][xi + 1u];
    const int32_t v01 = table[yi + 1u][xi];
    const int32_t v11 = table[yi + 1u][xi + 1u];

    const int32_t v0 = lerp_q8_s32(v00, v10, fx);
    const int32_t v1 = lerp_q8_s32(v01, v11, fx);
    const int32_t v = lerp_q8_s32(v0, v1, fy);

    if (v <= -32768) {
        return -32768;
    }
    if (v >= 32767) {
        return 32767;
    }
    return static_cast<int16_t>(v);
}

}  // namespace ems::engine
