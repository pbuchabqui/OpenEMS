#pragma once

/**
 * @file crank_angle.h
 * @brief TIM2 counts → crank / cycle degrees. Single conversion site.
 *
 * TIM2 encoder: 16384 counts/rev (X4 of 4096 PPR). No 60-2 tooth index.
 */

#include <cstdint>

namespace ems::drv {

inline constexpr uint32_t kCountsPerRev = 16384u;
inline constexpr uint16_t kCrankDegPerRev = 360u;
inline constexpr uint16_t kCycleDeg = 720u;

inline uint16_t crank_deg(uint32_t tim2) noexcept
{
    return static_cast<uint16_t>(((tim2 % kCountsPerRev) * 360u) / kCountsPerRev);
}

inline uint16_t crank_deg_x10(uint32_t tim2) noexcept
{
    return static_cast<uint16_t>(((tim2 % kCountsPerRev) * 3600u) / kCountsPerRev);
}

inline uint16_t cycle_deg(uint32_t tim2, bool phase_A) noexcept
{
    const uint16_t d = crank_deg(tim2);
    return static_cast<uint16_t>(d + (phase_A ? 0u : 360u));
}

}  // namespace ems::drv
