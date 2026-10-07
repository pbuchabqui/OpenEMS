#pragma once

#include <cstdint>
#include <limits>

namespace ems::engine {

inline int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) noexcept {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) noexcept {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return static_cast<int16_t>(v);
}

inline uint8_t clamp_u8(uint32_t v) noexcept {
    return static_cast<uint8_t>(v > 255u ? 255u : v);
}

inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) noexcept {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) noexcept {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Kelvin×10 (°C×10 + 2730) for air-density terms (ρ ∝ 1/T). Shared by
// fuel_calc and map_estimator. Range [-73°C, 150°C] covers both callers;
// T_ref/T at 298.0 K maps to Q8 [180, 381].
inline int32_t clamp_iat_kelvin_x10(int16_t iat_x10) noexcept {
    int32_t iat_k_x10 = static_cast<int32_t>(iat_x10) + 2730;
    if (iat_k_x10 < 2000) { iat_k_x10 = 2000; }
    if (iat_k_x10 > 4230) { iat_k_x10 = 4230; }
    return iat_k_x10;
}

// Piecewise-linear lookup over n monotonic axis points; clamps to the end
// values and to Y's range. Non-monotonic segment (span<=0) returns y0.
template <typename X, typename Y>
inline Y interp_8pt(const X* axis, const Y* table, uint8_t n, X x,
                    int32_t scale = 1) noexcept {
    int32_t y;
    if (x <= axis[0]) {
        y = static_cast<int32_t>(table[0]) * scale;
    } else if (x >= axis[n - 1u]) {
        y = static_cast<int32_t>(table[n - 1u]) * scale;
    } else {
        uint8_t idx = 0u;
        while (idx < (n - 2u) && x > axis[idx + 1u]) { ++idx; }

        const int32_t x0 = axis[idx];
        const int32_t y0 = static_cast<int32_t>(table[idx]) * scale;
        const int32_t span = static_cast<int32_t>(axis[idx + 1u]) - x0;
        if (span <= 0) {
            y = y0;
        } else {
            // Round to nearest (truncation biased every correction by -0.5 LSB).
            const int32_t num = (static_cast<int32_t>(table[idx + 1u]) * scale - y0) *
                                (static_cast<int32_t>(x) - x0);
            y = y0 + (num + ((num >= 0) ? span / 2 : -span / 2)) / span;
        }
    }
    constexpr int32_t kMin = std::numeric_limits<Y>::min();
    constexpr int32_t kMax = std::numeric_limits<Y>::max();
    return static_cast<Y>(y < kMin ? kMin : (y > kMax ? kMax : y));
}

inline uint16_t interp_u16_8pt_u16x(const uint16_t* x_axis, const uint16_t* table,
                                     uint8_t n, uint16_t x) noexcept {
    return interp_8pt<uint16_t, uint16_t>(x_axis, table, n, x);
}

inline uint16_t interp_u16_8pt(const int16_t* axis, const uint16_t* table,
                               uint8_t n, int16_t x) noexcept {
    return interp_8pt<int16_t, uint16_t>(axis, table, n, x);
}

inline int16_t interp_i16_8pt(const int16_t* axis, const int16_t* table,
                              uint8_t n, int16_t x, int32_t scale = 1) noexcept {
    return interp_8pt<int16_t, int16_t>(axis, table, n, x, scale);
}

}  // namespace ems::engine
