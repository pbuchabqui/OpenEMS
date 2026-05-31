#include "engine/misfire_detect.h"
#include "engine/engine_config.h"
#include "engine/constants.h"
#include "engine/ecu_sched.h"
#include "drv/ckp.h"

#include <cstdint>
#include <cstring>

namespace {

using ems::drv::SyncState;

// Posição TDC de cada cilindro na roda fônica 60-2.
// Calculado via kFiringOrder em misfire_init().
struct CylTdcPos {
    uint8_t tdc_tooth;  // tooth_index do TDC (0 ou 30 para motor 4-cil / 60-2)
    bool    phase_A;    // qual revolução do ciclo de 4 tempos
};

static CylTdcPos g_cyl_tdc[ems::engine::cfg::kCylinderCount];

// Tabela pré-computada: [phase_A(0/1)][tooth_index] → cilindro (-1 = fora de janela).
// Permite saída O(1) no ISR do CKP sem varrer todos os cilindros por dente.
// IMPORTANTE: preenchida em misfire_init(), que DEVE ser chamada antes de habilitar
// o ISR do CKP — BSS é zero, e 0 é um índice de cilindro válido.
static int8_t g_tooth_to_cyl[2][ems::engine::kRealTeeth60_2];

// Acumuladores por cilindro (escritos apenas no ISR do CKP → volatile).
static volatile uint32_t g_power_sum_ns[ems::engine::cfg::kCylinderCount];
static volatile uint32_t g_pred_sum_ns[ems::engine::cfg::kCylinderCount];
static volatile uint8_t  g_power_teeth[ems::engine::cfg::kCylinderCount];
static volatile uint8_t  g_debounce[ems::engine::cfg::kCylinderCount];

// Contadores de eventos lidos pelo main loop (uint8_t = atômico em ARM Cortex-M).
static volatile uint8_t  g_event_count[ems::engine::cfg::kCylinderCount];

// Inibe toda a detecção (ex.: durante decel cut — sem combustão é esperado).
// Escrito pelo main loop, lido na ISR do CKP (volatile bool).
static volatile bool g_all_inhibit = false;

constexpr uint8_t kN = ems::engine::cfg::kCylinderCount;

static void evaluate_window(uint8_t cyl) noexcept {
    // threshold = predicted_sum × kMisfireThresholdQ8 / 256
    const uint64_t thresh = (static_cast<uint64_t>(g_pred_sum_ns[cyl]) *
                             ems::engine::kMisfireThresholdQ8) >> 8u;
    const bool candidate = (g_power_sum_ns[cyl] > thresh);

    if (candidate) {
        if (g_debounce[cyl] < ems::engine::kMisfireDebounceCycles) {
            ++g_debounce[cyl];
        }
        if (g_debounce[cyl] >= ems::engine::kMisfireDebounceCycles) {
            if (g_event_count[cyl] < 255u) { ++g_event_count[cyl]; }
            g_debounce[cyl] = 0u;
        }
    } else {
        g_debounce[cyl] = 0u;
    }
}

}  // namespace

namespace ems::engine {

void misfire_init() noexcept {
    // Mapeia cada cilindro para sua posição TDC a partir de kFiringOrder.
    // O i-ésimo evento de ignição ocorre a i × 180° no ciclo de 720°.
    // Para roda 60-2: 180° = 30 dentes; os dois semiciclos distinguem-se por phase_A.
    //   i=0 → tooth  0, phA=true   (  0°)
    //   i=1 → tooth 30, phA=true   (180°)
    //   i=2 → tooth  0, phA=false  (360°)
    //   i=3 → tooth 30, phA=false  (540°)
    for (uint8_t i = 0u; i < kN; ++i) {
        const uint8_t cyl = cfg::kFiringOrder[i];
        g_cyl_tdc[cyl].tdc_tooth = static_cast<uint8_t>((i % 2u) * 30u);
        g_cyl_tdc[cyl].phase_A   = (i < 2u);
    }
    // Pré-computa mapa dente→cilindro para lookup O(1) no ISR.
    std::memset(g_tooth_to_cyl, 0xFF, sizeof(g_tooth_to_cyl));  // -1 em int8_t
    for (uint8_t c = 0u; c < kN; ++c) {
        const uint8_t phase = g_cyl_tdc[c].phase_A ? 0u : 1u;
        for (uint8_t t = 0u; t < ems::engine::kMisfireWindowTeeth; ++t) {
            const uint8_t tooth = g_cyl_tdc[c].tdc_tooth + t;
            if (tooth < ems::engine::kRealTeeth60_2) {
                g_tooth_to_cyl[phase][tooth] = static_cast<int8_t>(c);
            }
        }
    }
    misfire_reset();
}

void misfire_reset() noexcept {
    for (uint8_t c = 0u; c < kN; ++c) {
        g_power_sum_ns[c] = 0u;
        g_pred_sum_ns[c]  = 0u;
        g_power_teeth[c]  = 0u;
        g_debounce[c]     = 0u;
        g_event_count[c]  = 0u;
    }
}

uint8_t misfire_get_event_count(uint8_t cyl) noexcept {
    if (cyl >= kN) { return 0u; }
    return g_event_count[cyl];
}

void misfire_clear_events(uint8_t cyl) noexcept {
    if (cyl < kN) { g_event_count[cyl] = 0u; }
}

void misfire_set_all_inhibit(bool inhibit) noexcept {
    g_all_inhibit = inhibit;
}

}  // namespace ems::engine

// ── Hook ISR (ems::drv namespace — substitui símbolo fraco de ckp.cpp) ────────
namespace ems::drv {

void misfire_on_tooth(const CkpSnapshot& snap) noexcept {
    if (snap.state != SyncState::FULL_SYNC) { return; }
    if (snap.tooth_period_ns == 0u || snap.predicted_tooth_period_ns == 0u) { return; }
    // Cortes intencionais de combustão: não acumular → evitar DTCs falsos
    if (g_all_inhibit) { return; }

    const uint8_t ti    = static_cast<uint8_t>(snap.tooth_index & 0x3Fu);
    const uint8_t phase = snap.phase_A ? 0u : 1u;
    if (ti >= ems::engine::kRealTeeth60_2) { return; }
    const int8_t cyl_idx = g_tooth_to_cyl[phase][ti];
    if (cyl_idx < 0) { return; }
    const uint8_t c = static_cast<uint8_t>(cyl_idx);

    // Lê máscara de ignição: cilindros com DWELL inibido não têm combustão real
    const uint8_t ign_mask = static_cast<uint8_t>(::ecu_sched_get_ign_inhibit_mask());

    // Cilindro com faísca inibida pelo rev limiter: reseta janela sem avaliar
    if ((ign_mask >> c) & 1u) {
        g_power_sum_ns[c] = 0u;
        g_pred_sum_ns[c]  = 0u;
        g_power_teeth[c]  = 0u;
        g_debounce[c]     = 0u;
        return;
    }

    g_power_sum_ns[c] += snap.tooth_period_ns;
    g_pred_sum_ns[c]  += snap.predicted_tooth_period_ns;
    ++g_power_teeth[c];

    if (g_power_teeth[c] >= ems::engine::kMisfireWindowTeeth) {
        evaluate_window(c);
        g_power_sum_ns[c] = 0u;
        g_pred_sum_ns[c]  = 0u;
        g_power_teeth[c]  = 0u;
    }
}

}  // namespace ems::drv
