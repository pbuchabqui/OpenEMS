#include "engine/engine_config.h"

#include <cstdint>
#include <cstring>

namespace ems::engine::cfg {

// Page 0 layout:
//  [0-1]  : encoder_tdc1_origin_deg (uint16_t LE) — era reserved/padding (v2)
//  [2-3]  : displacement_cc (uint16_t LE)
//  [4-5]  : injector_flow_cc_min (uint16_t LE)
//  [6-7]  : stoich_afr_x100 (uint16_t LE)
//  [8-9]  : map_ref_bar_x100 (uint16_t LE)
//  [10-11]: trigger_tooth0_engine_deg (uint16_t LE)
//  [12-13]: default_eoi_lead_deg (uint16_t LE)
//  [14-15]: magic 0x4545 (v3)
//
// Magic v1 (0x4543) → v2 (0x4544): o campo [12-13] mudou de semântica
// (SOI lead → EOI lead). Páginas gravadas com magic v1 são rejeitadas e a
// ECU volta aos defaults de compilação — reinterpretar um SOI antigo como
// EOI mudaria silenciosamente o timing de injecção de tunes existentes.
// Magic v2 → v3: [0-1] passa de reserved/padding (bytes podiam conter lixo
// de firmwares antigos, nunca eram lidos) a encoder_tdc1_origin_deg —
// reinterpretar esse lixo como calibração TDC1 sem bump de magic dispararia
// sequencial sobre uma fase adivinhada. Mesma disciplina do bump v1→v2.

static constexpr uint16_t kMagicValue  = 0x4545u;  // v3 — soma encoder_tdc1_origin_deg
static constexpr uint16_t kMagicOffset = 14u;
static constexpr uint16_t kMinPageLen  = 16u;

static constexpr uint16_t kOffsetEncoderTdc1OriginDeg   = 0u;
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
    kEncoderTdc1OriginDeg,
};

static inline uint16_t read_u16_le(const uint8_t* buf, uint16_t offset) noexcept {
    uint16_t val = 0u;
    std::memcpy(&val, buf + offset, sizeof(val));
    return val;
}

static inline void write_u16_le(uint8_t* buf, uint16_t offset, uint16_t val) noexcept {
    std::memcpy(buf + offset, &val, sizeof(val));
}

bool engine_config_valid(const EngineConfigRam& c) noexcept {
    if (c.displacement_cc < 200u || c.displacement_cc > 10000u) {
        return false;
    }
    if (c.injector_flow_cc_min < 50u || c.injector_flow_cc_min > 3000u) {
        return false;
    }
    if (c.stoich_afr_x100 < 900u || c.stoich_afr_x100 > 1800u) {
        return false;
    }
    if (c.map_ref_bar_x100 < 50u || c.map_ref_bar_x100 > 250u) {
        return false;
    }
    if (c.trigger_tooth0_engine_deg > 719u) {
        return false;
    }
    if (c.default_eoi_lead_deg > 719u) {
        return false;
    }
    // Domínio de embrulho de 1 volta de TIM2 (360°), não 720° do ciclo do
    // motor — ver comentário junto a kEncoderTdc1OriginDeg.
    if (c.encoder_tdc1_origin_deg > 359u) {
        return false;
    }
    return true;
}

void engine_config_load(const uint8_t* page0_buf, uint16_t len) noexcept {
    if (page0_buf == nullptr || len < kMinPageLen) {
        return;
    }

    const uint16_t magic = read_u16_le(page0_buf, kMagicOffset);
    if (magic != kMagicValue) {
        return;  // Invalid magic — keep compile-time defaults
    }

    EngineConfigRam tmp = {};
    tmp.displacement_cc         = read_u16_le(page0_buf, kOffsetDisplacementCc);
    tmp.injector_flow_cc_min    = read_u16_le(page0_buf, kOffsetInjectorFlowCcMin);
    tmp.stoich_afr_x100         = read_u16_le(page0_buf, kOffsetStoichAfrX100);
    tmp.map_ref_bar_x100             = read_u16_le(page0_buf, kOffsetMapRefKpa);
    tmp.trigger_tooth0_engine_deg = read_u16_le(page0_buf, kOffsetTriggerTooth0EngineDeg);
    tmp.default_eoi_lead_deg    = read_u16_le(page0_buf, kOffsetDefaultEoiLeadDeg);
    tmp.encoder_tdc1_origin_deg  = read_u16_le(page0_buf, kOffsetEncoderTdc1OriginDeg);

    if (engine_config_valid(tmp)) {
        g_eng_cfg = tmp;
    }
}

void engine_config_apply(const uint8_t* page0_buf, uint16_t len) noexcept {
    engine_config_load(page0_buf, len);
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
    write_u16_le(page0_buf, kOffsetEncoderTdc1OriginDeg,    g_eng_cfg.encoder_tdc1_origin_deg);
    write_u16_le(page0_buf, kMagicOffset,                   kMagicValue);
}

}  // namespace ems::engine::cfg
