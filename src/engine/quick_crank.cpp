#include "engine/quick_crank.h"

#include <cstdint>

namespace {

struct P2 {
    int16_t x;
    uint16_t y;
};

constexpr uint32_t kCrankEnterRpmX10 = 4500u;
constexpr uint32_t kCrankExitRpmX10 = 7000u;
constexpr int16_t kCrankSparkDeg = 8;
constexpr uint32_t kCrankMinPwUs = 2500u;

constexpr P2 kCrankFuelMult[] = {
    {-400, 768},  // 3.00x
    {0, 614},     // 2.40x
    {200, 512},   // 2.00x
    {400, 435},   // 1.70x
    {700, 358},   // 1.40x
    {900, 320},   // 1.25x
    {1100, 294},  // 1.15x
};

constexpr P2 kAfterstartMultStart[] = {
    {-400, 346},  // 1.35x
    {0, 333},     // 1.30x
    {200, 320},   // 1.25x
    {400, 307},   // 1.20x
    {700, 294},   // 1.15x
    {900, 281},   // 1.10x
    {1100, 269},  // 1.05x
};

constexpr P2 kAfterstartDurationMs[] = {
    {-400, 2400},
    {0, 2000},
    {200, 1700},
    {400, 1400},
    {700, 1000},
    {900, 700},
    {1100, 500},
};

bool g_prev_cranking = false;
uint32_t g_afterstart_start_ms = 0u;
uint32_t g_afterstart_duration_ms = 0u;

uint16_t interp_u16(const P2* table, uint8_t n, int16_t x) noexcept {
    if (x <= table[0].x) {
        return table[0].y;
    }
    if (x >= table[n - 1u].x) {
        return table[n - 1u].y;
    }
    for (uint8_t i = 0u; i < (n - 1u); ++i) {
        if (x <= table[i + 1u].x) {
            const int32_t x0 = table[i].x;
            const int32_t x1 = table[i + 1u].x;
            const int32_t y0 = table[i].y;
            const int32_t y1 = table[i + 1u].y;
            const int32_t dx = static_cast<int32_t>(x) - x0;
            const int32_t span = x1 - x0;
            return static_cast<uint16_t>(y0 + ((y1 - y0) * dx) / span);
        }
    }
    return table[n - 1u].y;
}

uint16_t clamp_u16(uint32_t v, uint16_t lo, uint16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return static_cast<uint16_t>(v);
}

bool detect_cranking(uint32_t rpm_x10, bool full_sync) noexcept {
    if (!full_sync || rpm_x10 == 0u) {
        return false;
    }
    if (g_prev_cranking) {
        return rpm_x10 < kCrankExitRpmX10;
    }
    return rpm_x10 <= kCrankEnterRpmX10;
}

uint16_t afterstart_mult_x256(uint32_t now_ms, int16_t clt_x10) noexcept {
    if (g_afterstart_duration_ms == 0u) {
        return 256u;
    }
    const uint32_t elapsed = now_ms - g_afterstart_start_ms;
    if (elapsed >= g_afterstart_duration_ms) {
        return 256u;
    }
    const uint16_t start = interp_u16(
        kAfterstartMultStart,
        static_cast<uint8_t>(sizeof(kAfterstartMultStart) / sizeof(kAfterstartMultStart[0])),
        clt_x10);
    const uint32_t decay = static_cast<uint32_t>(start - 256u) * elapsed;
    const uint32_t mult = static_cast<uint32_t>(start) - (decay / g_afterstart_duration_ms);
    return clamp_u16(mult, 256u, 512u);
}

}  // namespace

namespace ems::engine {

void quick_crank_reset() noexcept {
    g_prev_cranking = false;
    g_afterstart_start_ms = 0u;
    g_afterstart_duration_ms = 0u;
}

QuickCrankOutput quick_crank_update(uint32_t now_ms,
                                    uint32_t rpm_x10,
                                    bool full_sync,
                                    int16_t clt_x10,
                                    int16_t base_spark_deg) noexcept {
    QuickCrankOutput out{};
    out.spark_deg = base_spark_deg;
    out.fuel_mult_x256 = 256u;
    out.min_pw_us = 0u;

    const bool cranking = detect_cranking(rpm_x10, full_sync);
    out.cranking = cranking;

    if (cranking) {
        out.spark_deg = kCrankSparkDeg;
        out.min_pw_us = kCrankMinPwUs;
        out.fuel_mult_x256 = interp_u16(
            kCrankFuelMult,
            static_cast<uint8_t>(sizeof(kCrankFuelMult) / sizeof(kCrankFuelMult[0])),
            clt_x10);
        g_afterstart_duration_ms = 0u;
    } else {
        if (g_prev_cranking && rpm_x10 >= kCrankExitRpmX10) {
            g_afterstart_start_ms = now_ms;
            g_afterstart_duration_ms = interp_u16(
                kAfterstartDurationMs,
                static_cast<uint8_t>(sizeof(kAfterstartDurationMs) / sizeof(kAfterstartDurationMs[0])),
                clt_x10);
        }
        const uint16_t as_mult = afterstart_mult_x256(now_ms, clt_x10);
        out.afterstart_active = (as_mult > 256u);
        out.fuel_mult_x256 = as_mult;
    }

    g_prev_cranking = cranking;
    return out;
}

uint32_t quick_crank_apply_pw_us(uint32_t base_pw_us,
                                 uint16_t fuel_mult_x256,
                                 uint32_t min_pw_us) noexcept {
    uint32_t out = static_cast<uint32_t>(
        (static_cast<uint64_t>(base_pw_us) * fuel_mult_x256) / 256u);
    if (out < min_pw_us) {
        out = min_pw_us;
    }
    if (out > 100000u) {
        out = 100000u;
    }
    return out;
}

}  // namespace ems::engine
