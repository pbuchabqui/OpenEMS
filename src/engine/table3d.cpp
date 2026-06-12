#include "engine/table3d.h"

namespace {

struct AxisLookupResult {
    uint8_t idx;
    uint8_t frac_q8;
};

AxisLookupResult axis_lookup(const uint32_t* axis, uint8_t size, uint32_t value) noexcept {
    AxisLookupResult out{};
    if (size < 2u) {
        return out;
    }

    if (value <= axis[0]) {
        return out;
    }

    const uint8_t last = static_cast<uint8_t>(size - 1u);
    if (value >= axis[last]) {
        out.idx = static_cast<uint8_t>(last - 1u);
        out.frac_q8 = 255u;
        return out;
    }

    uint8_t lo = 1u;
    uint8_t hi = last;
    while (lo < hi) {
        const uint8_t mid = static_cast<uint8_t>(lo + ((hi - lo) / 2u));
        if (value <= axis[mid]) {
            hi = mid;
        } else {
            lo = static_cast<uint8_t>(mid + 1u);
        }
    }

    out.idx = static_cast<uint8_t>(lo - 1u);
    const uint32_t x0 = axis[out.idx];
    const uint32_t x1 = axis[out.idx + 1u];
    if (value <= x0) {
        return out;
    }
    if (value >= x1) {
        out.frac_q8 = 255u;
        return out;
    }

    const uint32_t span = x1 - x0;
    if (span == 0u) {
        return out;
    }

    uint32_t frac = ((value - x0) << 8u) / span;
    if (frac > 255u) {
        frac = 255u;
    }
    out.frac_q8 = static_cast<uint8_t>(frac);
    return out;
}

}  // namespace

namespace ems::engine {

const uint32_t kRpmAxisX10[kTableAxisSize] = {
    5000u,   7500u,   10000u,  12500u,
    15000u,  20000u,  25000u,  30000u,
    35000u,  40000u,  45000u,  50000u,
    55000u,  60000u,  70000u,  80000u,
};

const uint32_t kLoadAxisBarX100[kTableAxisSize] = {
    20u,  30u,  40u,  50u,
    60u,  70u,  80u,  90u,
    100u, 115u, 130u, 145u,
    160u, 180u, 215u, 300u,
};

uint8_t table_axis_index(const uint32_t* axis, uint8_t size, uint32_t value) noexcept {
    return axis_lookup(axis, size, value).idx;
}

uint8_t table_axis_frac_q8(const uint32_t* axis, uint8_t idx, uint32_t value) noexcept {
    const uint32_t x0 = axis[idx];
    const uint32_t x1 = axis[idx + 1u];

    if (value <= x0) {
        return 0u;
    }
    if (value >= x1) {
        return 255u;
    }

    const uint32_t span = x1 - x0;
    if (span == 0u) {
        return 0u;
    }

    const uint32_t num = (value - x0) << 8u;
    uint32_t frac = num / span;
    if (frac > 255u) {
        frac = 255u;
    }
    return static_cast<uint8_t>(frac);
}

static int32_t lerp_q8_s32(int32_t a, int32_t b, uint8_t frac_q8) noexcept {
    if (frac_q8 == 255u) { return b; }
    return a + (((b - a) * static_cast<int32_t>(frac_q8)) >> 8u);
}

Table2dLookup table3d_prepare_lookup(const uint32_t* x_axis,
                                     const uint32_t* y_axis,
                                     uint32_t x,
                                     uint32_t y) noexcept {
    Table2dLookup lookup{};
    const AxisLookupResult lx = axis_lookup(x_axis, kTableAxisSize, x);
    const AxisLookupResult ly = axis_lookup(y_axis, kTableAxisSize, y);
    lookup.xi = lx.idx;
    lookup.yi = ly.idx;
    lookup.fx_q8 = lx.frac_q8;
    lookup.fy_q8 = ly.frac_q8;
    return lookup;
}

uint8_t table3d_lookup_u8_prepared(const uint8_t table[kTableAxisSize][kTableAxisSize],
                                   const Table2dLookup& lookup) noexcept {
    const int32_t v00 = table[lookup.yi][lookup.xi];
    const int32_t v10 = table[lookup.yi][lookup.xi + 1u];
    const int32_t v01 = table[lookup.yi + 1u][lookup.xi];
    const int32_t v11 = table[lookup.yi + 1u][lookup.xi + 1u];

    const int32_t v0 = lerp_q8_s32(v00, v10, lookup.fx_q8);
    const int32_t v1 = lerp_q8_s32(v01, v11, lookup.fx_q8);
    const int32_t v = lerp_q8_s32(v0, v1, lookup.fy_q8);

    if (v <= 0) {
        return 0u;
    }
    if (v >= 255) {
        return 255u;
    }
    return static_cast<uint8_t>(v);
}

int16_t table3d_lookup_i8_prepared(const int8_t table[kTableAxisSize][kTableAxisSize],
                                   const Table2dLookup& lookup) noexcept {
    const int32_t v00 = table[lookup.yi][lookup.xi];
    const int32_t v10 = table[lookup.yi][lookup.xi + 1u];
    const int32_t v01 = table[lookup.yi + 1u][lookup.xi];
    const int32_t v11 = table[lookup.yi + 1u][lookup.xi + 1u];

    const int32_t v0 = lerp_q8_s32(v00, v10, lookup.fx_q8);
    const int32_t v1 = lerp_q8_s32(v01, v11, lookup.fx_q8);
    return static_cast<int16_t>(lerp_q8_s32(v0, v1, lookup.fy_q8));
}

int16_t table3d_lookup_s16_prepared(const int16_t table[kTableAxisSize][kTableAxisSize],
                                    const Table2dLookup& lookup) noexcept {
    const int32_t v00 = table[lookup.yi][lookup.xi];
    const int32_t v10 = table[lookup.yi][lookup.xi + 1u];
    const int32_t v01 = table[lookup.yi + 1u][lookup.xi];
    const int32_t v11 = table[lookup.yi + 1u][lookup.xi + 1u];

    const int32_t v0 = lerp_q8_s32(v00, v10, lookup.fx_q8);
    const int32_t v1 = lerp_q8_s32(v01, v11, lookup.fx_q8);
    const int32_t v = lerp_q8_s32(v0, v1, lookup.fy_q8);

    if (v <= -32768) {
        return -32768;
    }
    if (v >= 32767) {
        return 32767;
    }
    return static_cast<int16_t>(v);
}

uint8_t table3d_lookup_u8(const uint8_t table[kTableAxisSize][kTableAxisSize],
                          const uint32_t* x_axis,
                          const uint32_t* y_axis,
                          uint32_t x,
                          uint32_t y) noexcept {
    const Table2dLookup lookup = table3d_prepare_lookup(x_axis, y_axis, x, y);
    return table3d_lookup_u8_prepared(table, lookup);
}

// Nova função otimizada para VE lookup com Q8 arithmetic
uint16_t table3d_lookup_ve_q8(const uint8_t ve_table[kTableAxisSize][kTableAxisSize],
                             const uint32_t* x_axis,
                             const uint32_t* y_axis,
                             uint32_t x,
                             uint32_t y) noexcept {
    const Table2dLookup lookup = table3d_prepare_lookup(x_axis, y_axis, x, y);

    // Carrega valores da tabela (já em Q8 scale)
    const uint16_t v00 = static_cast<uint16_t>(ve_table[lookup.yi][lookup.xi]) << 8;
    const uint16_t v10 = static_cast<uint16_t>(ve_table[lookup.yi][lookup.xi + 1u]) << 8;
    const uint16_t v01 = static_cast<uint16_t>(ve_table[lookup.yi + 1u][lookup.xi]) << 8;
    const uint16_t v11 = static_cast<uint16_t>(ve_table[lookup.yi + 1u][lookup.xi + 1u]) << 8;

    // Interpolação Q8 com aritmética signed explícita (FIX-1: evita right-shift
    // em valor negativo — implementation-defined em C++17; int32_t garante
    // comportamento correto em tabelas decrescentes).
    const int32_t d0  = (static_cast<int32_t>(v10) - static_cast<int32_t>(v00)) * static_cast<int32_t>(lookup.fx_q8);
    const uint16_t v0 = static_cast<uint16_t>(static_cast<int32_t>(v00) + (d0 >> 8));
    const int32_t d1  = (static_cast<int32_t>(v11) - static_cast<int32_t>(v01)) * static_cast<int32_t>(lookup.fx_q8);
    const uint16_t v1 = static_cast<uint16_t>(static_cast<int32_t>(v01) + (d1 >> 8));
    const int32_t dv  = (static_cast<int32_t>(v1) - static_cast<int32_t>(v0)) * static_cast<int32_t>(lookup.fy_q8);
    const uint16_t v  = static_cast<uint16_t>(static_cast<int32_t>(v0) + (dv >> 8));

    return v;  // Resultado em Q8
}

// Função para advance lookup com Q10
int32_t table3d_lookup_advance_q10(const int16_t advance_table[kTableAxisSize][kTableAxisSize],
                                 const uint32_t* x_axis,
                                 const uint32_t* y_axis,
                                 uint32_t x,
                                 uint32_t y) noexcept {
    const Table2dLookup lookup = table3d_prepare_lookup(x_axis, y_axis, x, y);

    // Carrega valores da tabela (converte para Q10)
    const int32_t v00 = static_cast<int32_t>(advance_table[lookup.yi][lookup.xi]) << 10;
    const int32_t v10 = static_cast<int32_t>(advance_table[lookup.yi][lookup.xi + 1u]) << 10;
    const int32_t v01 = static_cast<int32_t>(advance_table[lookup.yi + 1u][lookup.xi]) << 10;
    const int32_t v11 = static_cast<int32_t>(advance_table[lookup.yi + 1u][lookup.xi + 1u]) << 10;

    // Interpolação Q10 otimizada
    const int32_t v0 = v00 + (((v10 - v00) * static_cast<int32_t>(lookup.fx_q8)) >> 8);
    const int32_t v1 = v01 + (((v11 - v01) * static_cast<int32_t>(lookup.fx_q8)) >> 8);
    const int32_t v = v0 + (((v1 - v0) * static_cast<int32_t>(lookup.fy_q8)) >> 8);

    return v;  // Resultado em Q10 (150 = 15.0° BTDC)
}

int16_t table3d_lookup_s16(const int16_t table[kTableAxisSize][kTableAxisSize],
                           const uint32_t* x_axis,
                           const uint32_t* y_axis,
                           uint32_t x,
                           uint32_t y) noexcept {
    const Table2dLookup lookup = table3d_prepare_lookup(x_axis, y_axis, x, y);
    return table3d_lookup_s16_prepared(table, lookup);
}

}  // namespace ems::engine
