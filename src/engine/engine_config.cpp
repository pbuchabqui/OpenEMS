#include "engine/engine_config.h"

#include <cstdint>
#include <cstring>

namespace ems::engine::cfg {

// Page 0 layout:
//  [0]    : reserved (was IVC ABDC; unused — EOI targeting)
//  [1]    : reserved/padding
//  [2-3]  : displacement_cc (uint16_t LE)
//  [4-5]  : injector_flow_cc_min (uint16_t LE)
//  [6-7]  : stoich_afr_x100 (uint16_t LE)
//  [8-9]  : map_ref_bar_x100 (uint16_t LE)
//  [10-11]: trigger_tooth0_engine_deg (uint16_t LE)
//  [12-13]: default_eoi_lead_deg (uint16_t LE)
//  [14-15]: magic 0x4544 (v2)
//
// Magic v1 (0x4543) → v2 (0x4544): o campo [12-13] mudou de semântica
// (SOI lead → EOI lead). Páginas gravadas com magic v1 são rejeitadas e a
// ECU volta aos defaults de compilação — reinterpretar um SOI antigo como
// EOI mudaria silenciosamente o timing de injecção de tunes existentes.

static constexpr uint16_t kMagicValue  = 0x4544u;  // v2 — semântica EOI
static constexpr uint16_t kMagicOffset = 14u;
static constexpr uint16_t kMinPageLen  = 16u;

static constexpr uint16_t kOffsetDisplacementCc         = 2u;
static constexpr uint16_t kOffsetInjectorFlowCcMin      = 4u;
static constexpr uint16_t kOffsetStoichAfrX100          = 6u;
static constexpr uint16_t kOffsetMapRefKpa              = 8u;
static constexpr uint16_t kOffsetTriggerTooth0EngineDeg = 10u;
static constexpr uint16_t kOffsetDefaultEoiLeadDeg      = 12u;

// Global runtime config, initialized to compile-time defaults.
EngineConfigRam g_eng_cfg = {
    kDisplacementCc,
    kInjectorFlowCcMin,
    kStoichAfrX100,
    kMapRefBarX100,
    kTriggerTooth0EngineDeg,
    kDefaultEoiLeadDeg,
};

static inline uint16_t read_u16_le(const uint8_t* buf, uint16_t offset) noexcept {
    uint16_t val = 0u;
    std::memcpy(&val, buf + offset, sizeof(val));
    return val;
}

static inline void write_u16_le(uint8_t* buf, uint16_t offset, uint16_t val) noexcept {
    std::memcpy(buf + offset, &val, sizeof(val));
}

namespace {

// Valid range per field. tools/ts/openems.ini uses exactly these limits, so
// TunerStudio cannot send a value the firmware would reject.
struct FieldSpec {
    uint16_t offset;
    uint16_t lo;
    uint16_t hi;
    uint16_t EngineConfigRam::*field;
};

constexpr FieldSpec kFields[] = {
    {kOffsetDisplacementCc,         200u, 10000u, &EngineConfigRam::displacement_cc},
    {kOffsetInjectorFlowCcMin,       50u,  3000u, &EngineConfigRam::injector_flow_cc_min},
    {kOffsetStoichAfrX100,          900u,  1800u, &EngineConfigRam::stoich_afr_x100},
    {kOffsetMapRefKpa,               50u,   250u, &EngineConfigRam::map_ref_bar_x100},
    {kOffsetTriggerTooth0EngineDeg,   0u,   719u, &EngineConfigRam::trigger_tooth0_engine_deg},
    {kOffsetDefaultEoiLeadDeg,        0u,   719u, &EngineConfigRam::default_eoi_lead_deg},
};
constexpr uint8_t kFieldCount = static_cast<uint8_t>(sizeof(kFields) / sizeof(kFields[0]));

uint8_t g_reject_mask = 0u;

}  // namespace

bool engine_config_valid(const EngineConfigRam& c) noexcept {
    for (const FieldSpec& f : kFields) {
        const uint16_t v = c.*(f.field);
        if (v < f.lo || v > f.hi) {
            return false;
        }
    }
    return true;
}

uint8_t engine_config_load(const uint8_t* page0_buf, uint16_t len) noexcept {
    if (page0_buf == nullptr || len < kMinPageLen) {
        return g_reject_mask;
    }
    if (read_u16_le(page0_buf, kMagicOffset) != kMagicValue) {
        g_reject_mask = kEngineConfigRejectMagic;  // keep current values
        return g_reject_mask;
    }
    // Per field: a value out of range is rejected alone (keeps its current
    // value) and flagged; the other fields still apply.
    uint8_t mask = 0u;
    for (uint8_t i = 0u; i < kFieldCount; ++i) {
        const FieldSpec& f = kFields[i];
        const uint16_t v = read_u16_le(page0_buf, f.offset);
        if (v < f.lo || v > f.hi) {
            mask = static_cast<uint8_t>(mask | (1u << i));
        } else {
            g_eng_cfg.*(f.field) = v;
        }
    }
    g_reject_mask = mask;
    return mask;
}

uint8_t engine_config_reject_mask() noexcept {
    return g_reject_mask;
}

void engine_config_serialize(uint8_t* page0_buf, uint16_t len) noexcept {
    if (page0_buf == nullptr || len < kMinPageLen) {
        return;
    }

    write_u16_le(page0_buf, kOffsetDisplacementCc,          g_eng_cfg.displacement_cc);
    write_u16_le(page0_buf, kOffsetInjectorFlowCcMin,       g_eng_cfg.injector_flow_cc_min);
    write_u16_le(page0_buf, kOffsetStoichAfrX100,           g_eng_cfg.stoich_afr_x100);
    write_u16_le(page0_buf, kOffsetMapRefKpa,               g_eng_cfg.map_ref_bar_x100);
    write_u16_le(page0_buf, kOffsetTriggerTooth0EngineDeg,  g_eng_cfg.trigger_tooth0_engine_deg);
    write_u16_le(page0_buf, kOffsetDefaultEoiLeadDeg,       g_eng_cfg.default_eoi_lead_deg);
    write_u16_le(page0_buf, kMagicOffset,                   kMagicValue);
}

}  // namespace ems::engine::cfg
