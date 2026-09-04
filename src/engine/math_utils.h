#pragma once

#include <cstdint>

namespace ems::engine {

// Extrapolação de tendência linear entre duas amostras consecutivas (a mais
// recente `a`, a anterior `b`), clampada a ±(clamp_base/clamp_den) para não
// deixar um outlier isolado disparar uma previsão descontrolada. `clamp_base`
// é explícito (não hardcoded a `a` nem a `b`) porque os dois usos existentes
// (drv/ckp.cpp::predict_next_period_ticks, engine/misfire_encoder.cpp::
// predict_current_delta_ticks) clampam em bases diferentes — a mais antiga
// (`b`) num caso, a mais recente (`a`) no outro — e colapsar isso
// silenciosamente mudaria o valor previsto de um dos dois. Guarda contra
// amostras patológicas (>INT32_MAX) antes de entrar em aritmética com sinal,
// devolvendo `a` sem previsão nesse caso.
inline uint32_t linear_trend_predict(uint32_t a, uint32_t b,
                                     uint32_t clamp_base,
                                     uint32_t clamp_den) noexcept {
    if (a > 0x7FFFFFFFu || b > 0x7FFFFFFFu) { return a; }
    int32_t trend = static_cast<int32_t>(a) - static_cast<int32_t>(b);
    const int32_t limit = static_cast<int32_t>(clamp_base / clamp_den);
    if (trend > limit) { trend = limit; }
    if (trend < -limit) { trend = -limit; }
    const int32_t predicted = static_cast<int32_t>(a) + trend;
    return (predicted > 0) ? static_cast<uint32_t>(predicted) : a;
}

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
