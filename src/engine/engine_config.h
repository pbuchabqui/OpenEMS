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
// ASSUNÇÃO CRÍTICA: 0 mantém o comportamento atual: tooth 0 coincide com TDC do
// cilindro 0. Se o trigger wheel for montado com offset físico (ex: 84° antes
// do TDC), TODOS os ângulos de spark/injeção estarão errados.
// Ao calibrar para motor real:
//   - Medir o offset entre tooth 0 e TDC do cilindro 0
//   - Ajustar para: kTriggerTooth0EngineDeg = (720 - offset_deg) % 720
//   - Ex: offset 84° antes do TDC → kTriggerTooth0EngineDeg = 636
inline constexpr uint16_t kTriggerTooth0EngineDeg = 0u;
static_assert(kTriggerTooth0EngineDeg < 720u,
    "kTriggerTooth0EngineDeg deve estar em [0, 720) graus de virabrequim");

inline constexpr uint8_t kFiringOrder[kCylinderCount] = {0u, 2u, 3u, 1u};

constexpr uint16_t cyl_tdc_deg(uint8_t cyl) noexcept {
    return static_cast<uint16_t>(cyl * (720u / kCylinderCount));
}

// =============================================================================
// Runtime-configurable engine parameters (stored in Flash page 0)
// =============================================================================

struct EngineConfigRam {
    uint16_t displacement_cc;
    uint16_t injector_flow_cc_min;
    uint16_t stoich_afr_x100;
    uint16_t map_ref_kpa;
    uint16_t trigger_tooth0_engine_deg;
    uint16_t default_soi_lead_deg;
};

// Runtime config — initialized to compile-time defaults at startup.
// Overwritten by engine_config_load() if valid data found in Flash.
extern EngineConfigRam g_eng_cfg;

// Call at boot after nvm_load_calibration(0, page0_buf, 512).
// If page0 magic is valid, populates g_eng_cfg from page0_buf offsets 2-15.
// Otherwise keeps compile-time defaults.
void engine_config_load(const uint8_t* page0_buf, uint16_t len) noexcept;

// Call when UI writes page 0 and requests burn: updates g_eng_cfg from buf.
void engine_config_apply(const uint8_t* page0_buf, uint16_t len) noexcept;

// Validates runtime config: returns false if any value is out of safe range.
bool engine_config_valid(const EngineConfigRam& cfg) noexcept;

// Write current g_eng_cfg into page0_buf at the correct offsets.
// (for reading back via UI protocol)
void engine_config_serialize(uint8_t* page0_buf, uint16_t len) noexcept;

}  // namespace ems::engine::cfg
