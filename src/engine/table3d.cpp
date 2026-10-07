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

    uint32_t frac = (((value - x0) << 8u) + (span / 2u)) / span;
    if (frac > 255u) {
        frac = 255u;
    }
    out.frac_q8 = static_cast<uint8_t>(frac);
    return out;
}

}  // namespace

namespace ems::engine {

uint32_t kRpmAxisX10[kTableAxisSize] = {
    5000u, 7500u, 10000u, 12500u, 15000u, 17500u, 20000u, 22500u, 25000u, 27500u,
    30000u, 35000u, 40000u, 45000u, 50000u, 55000u, 60000u, 65000u, 70000u, 80000u,
};

uint32_t kLoadAxisBarX100[kTableAxisSize] = {
    20u, 30u, 40u, 46u, 52u, 58u, 64u, 70u, 76u, 88u,
    94u, 100u, 110u, 130u, 160u, 190u, 220u, 250u, 273u, 300u,
};

bool table_axes_set(const uint16_t rpm[kTableAxisSize],
                    const uint16_t load_bar_x100[kTableAxisSize]) noexcept {
    if (rpm[0] == 0u || load_bar_x100[0] == 0u) {
        return false;
    }
    for (uint8_t i = 1u; i < kTableAxisSize; ++i) {
        if (rpm[i] <= rpm[i - 1u] || load_bar_x100[i] <= load_bar_x100[i - 1u]) {
            return false;
        }
    }
    for (uint8_t i = 0u; i < kTableAxisSize; ++i) {
        kRpmAxisX10[i] = static_cast<uint32_t>(rpm[i]) * 10u;
        kLoadAxisBarX100[i] = load_bar_x100[i];
    }
    return true;
}

void table_axes_get(uint16_t rpm[kTableAxisSize],
                    uint16_t load_bar_x100[kTableAxisSize]) noexcept {
    for (uint8_t i = 0u; i < kTableAxisSize; ++i) {
        const uint32_t r = kRpmAxisX10[i] / 10u;
        rpm[i] = static_cast<uint16_t>((r > 65535u) ? 65535u : r);
        const uint32_t l = kLoadAxisBarX100[i];
        load_bar_x100[i] = static_cast<uint16_t>((l > 65535u) ? 65535u : l);
    }
}

uint8_t table_axis_index(const uint32_t* axis, uint8_t size, uint32_t value) noexcept {
    return axis_lookup(axis, size, value).idx;
}

uint8_t table_axis_nearest_index(const uint32_t* axis, uint8_t size, uint32_t value) noexcept {
    const AxisLookupResult a = axis_lookup(axis, size, value);
    // frac ≥ 0.5 → sobe para o nó alto do segmento (inclui nó exacto: frac=255).
    if (a.frac_q8 >= 128u) {
        const uint8_t hi = static_cast<uint8_t>(a.idx + 1u);
        if (hi < size) {
            return hi;
        }
    }
    return a.idx;
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

    uint32_t frac = (((value - x0) << 8u) + (span / 2u)) / span;
    if (frac > 255u) {
        frac = 255u;
    }
    return static_cast<uint8_t>(frac);
}

// frac_q8 = 255 representa 1,0 (nó alto exacto). Arredonda ao mais próximo:
// o floor anterior dava viés de −0,5 LSB por eixo (≈ −1 LSB na bilinear).
int32_t lerp_q8_s32(int32_t a, int32_t b, uint8_t frac_q8) noexcept {
    if (frac_q8 == 255u) { return b; }
    return a + (((b - a) * static_cast<int32_t>(frac_q8) + 128) >> 8);
}

namespace {

// Bilinear num único passo com arredondamento ao mais próximo (um só
// arredondamento em vez de três floors encadeados).
int32_t bilerp(int32_t v00, int32_t v10, int32_t v01, int32_t v11,
               const Table2dLookup& lk) noexcept {
    const int32_t fx = (lk.fx_q8 == 255u) ? 256 : static_cast<int32_t>(lk.fx_q8);
    const int32_t fy = (lk.fy_q8 == 255u) ? 256 : static_cast<int32_t>(lk.fy_q8);
    const int64_t sum = static_cast<int64_t>(v00) * (256 - fx) * (256 - fy) +
                        static_cast<int64_t>(v10) * fx * (256 - fy) +
                        static_cast<int64_t>(v01) * (256 - fx) * fy +
                        static_cast<int64_t>(v11) * fx * fy;
    return static_cast<int32_t>((sum + 32768) >> 16);
}

template <typename T>
int32_t bilerp_table(const T table[kTableAxisSize][kTableAxisSize],
                     const Table2dLookup& lk) noexcept {
    return bilerp(table[lk.yi][lk.xi], table[lk.yi][lk.xi + 1u],
                  table[lk.yi + 1u][lk.xi], table[lk.yi + 1u][lk.xi + 1u], lk);
}

}  // namespace

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
    const int32_t v = bilerp_table(table, lookup);

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
    return static_cast<int16_t>(bilerp_table(table, lookup));
}

int16_t table3d_lookup_i8_x10_prepared(const int8_t table[kTableAxisSize][kTableAxisSize],
                                       const Table2dLookup& lk) noexcept {
    return static_cast<int16_t>(bilerp(10 * table[lk.yi][lk.xi], 10 * table[lk.yi][lk.xi + 1u],
                                       10 * table[lk.yi + 1u][lk.xi],
                                       10 * table[lk.yi + 1u][lk.xi + 1u], lk));
}

int16_t table3d_lookup_s16_prepared(const int16_t table[kTableAxisSize][kTableAxisSize],
                                    const Table2dLookup& lookup) noexcept {
    const int32_t v = bilerp_table(table, lookup);

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

int16_t table3d_lookup_s16(const int16_t table[kTableAxisSize][kTableAxisSize],
                           const uint32_t* x_axis,
                           const uint32_t* y_axis,
                           uint32_t x,
                           uint32_t y) noexcept {
    const Table2dLookup lookup = table3d_prepare_lookup(x_axis, y_axis, x, y);
    return table3d_lookup_s16_prepared(table, lookup);
}

}  // namespace ems::engine
