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
inline constexpr uint16_t kDefaultSoiLeadDeg = 62u;
inline constexpr uint16_t kIvcAbdcDeg = 50u;

// Ângulo do motor quando o decodificador CKP está em tooth_index 0.
// 0 mantém o comportamento atual: tooth 0 coincide com TDC do cilindro 0.
// Ex.: se tooth 0 ocorre 84° antes do TDC, usar 636° (720 - 84).
inline constexpr uint16_t kTriggerTooth0EngineDeg = 0u;

inline constexpr uint8_t kFiringOrder[kCylinderCount] = {0u, 2u, 3u, 1u};

constexpr uint16_t cyl_tdc_deg(uint8_t cyl) noexcept {
    return static_cast<uint16_t>(cyl * (720u / kCylinderCount));
}

}  // namespace ems::engine::cfg
