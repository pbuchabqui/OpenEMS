#pragma once

#include <cstdint>

namespace ems::engine::cfg {

inline constexpr uint8_t kCylinderCount = 4u;
inline constexpr uint16_t kDisplacementCc = 2000u;
inline constexpr uint16_t kInjectorFlowCcMin = 450u;

// E30: lambda 1.00 equivale aproximadamente a AFR 13.0.
inline constexpr uint16_t kStoichAfrX100 = 1300u;
inline constexpr uint16_t kFuelDensityMgPerCc = 755u;
inline constexpr uint16_t kAirDensityMgPerCcX1000 = 1184u;

inline constexpr uint16_t kMapRefKpa = 100u;
inline constexpr uint32_t kDefaultSoiLeadDeg = 62u;
inline constexpr uint16_t kIvcAbdcDeg = 50u;

inline constexpr uint8_t kFiringOrder[kCylinderCount] = {0u, 2u, 3u, 1u};
inline constexpr uint16_t kCylinderTdcDeg[kCylinderCount] = {0u, 180u, 360u, 540u};

}  // namespace ems::engine::cfg
