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

    for (uint8_t i = 0u; i < static_cast<uint8_t>(size - 1u); ++i) {
        if (value <= axis[i + 1u]) {
            return i;
        }
    }
    return static_cast<uint8_t>(size - 2u);
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

static int32_t lerp_q8_s32(int32_t a, int32_t b, uint8_t frac_q8) noexcept {
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
