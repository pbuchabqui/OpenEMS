#pragma once

#include <cstdint>

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

inline uint16_t interp_u16_8pt_u16x(const uint16_t* x_axis,
                                     const uint16_t* table,
                                     uint8_t n,
                                     uint16_t x) noexcept {
    if (x <= x_axis[0]) return table[0];
    if (x >= x_axis[n - 1u]) return table[n - 1u];

    uint8_t idx = 0u;
    while (idx < (n - 2u) && x > x_axis[idx + 1u]) { ++idx; }

    const uint16_t x0 = x_axis[idx];
    const uint16_t x1 = x_axis[idx + 1u];
    const uint16_t y0 = table[idx];
    const uint16_t y1 = table[idx + 1u];
    const uint32_t dx = static_cast<uint32_t>(x - x0);
    const uint32_t span = static_cast<uint32_t>(x1 - x0);
    if (span == 0u) return y0;

    const int32_t dy = static_cast<int32_t>(y1) - static_cast<int32_t>(y0);
    const int32_t y = static_cast<int32_t>(y0) +
        static_cast<int32_t>((dy * static_cast<int32_t>(dx)) / static_cast<int32_t>(span));
    if (y <= 0) return 0u;
    if (y >= 65535) return 65535u;
    return static_cast<uint16_t>(y);
}

inline uint16_t interp_u16_8pt(const int16_t* axis,
                               const uint16_t* table,
                               uint8_t n,
                               int16_t x) noexcept {
    if (x <= axis[0]) return table[0];
    if (x >= axis[n - 1u]) return table[n - 1u];

    uint8_t idx = 0u;
    while (idx < (n - 2u) && x > axis[idx + 1u]) { ++idx; }

    const int32_t x0 = axis[idx];
    const int32_t x1 = axis[idx + 1u];
    const int32_t y0 = table[idx];
    const int32_t y1 = table[idx + 1u];
    const int32_t span = x1 - x0;
    if (span <= 0) return static_cast<uint16_t>(y0);

    const int32_t y = y0 + ((y1 - y0) * (static_cast<int32_t>(x) - x0)) / span;
    if (y <= 0) return 0u;
    if (y >= 65535) return 65535u;
    return static_cast<uint16_t>(y);
}

inline int16_t interp_i16_8pt(const int16_t* axis,
                              const int16_t* table,
                              uint8_t n,
                              int16_t x) noexcept {
    if (x <= axis[0]) return table[0];
    if (x >= axis[n - 1u]) return table[n - 1u];

    uint8_t idx = 0u;
    while (idx < (n - 2u) && x > axis[idx + 1u]) { ++idx; }

    const int32_t x0 = axis[idx];
    const int32_t x1 = axis[idx + 1u];
    const int32_t y0 = table[idx];
    const int32_t y1 = table[idx + 1u];
    const int32_t span = x1 - x0;
    if (span <= 0) return static_cast<int16_t>(y0);

    const int32_t y = y0 + ((y1 - y0) * (static_cast<int32_t>(x) - x0)) / span;
    if (y < -32768) return -32768;
    if (y > 32767) return 32767;
    return static_cast<int16_t>(y);
}

}  // namespace ems::engine
