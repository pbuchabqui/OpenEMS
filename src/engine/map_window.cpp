#include "engine/map_window.h"
#include "engine/calibration.h"
#include "engine/engine_config.h"
#include "drv/crank_angle.h"

namespace ems::engine {

namespace {

constexpr uint8_t  kSlots        = 4u;
constexpr uint16_t kSlotSpanDeg  = 180u;
constexpr uint16_t kCycleDeg     = ems::drv::kCycleDeg;

// Acumulador da janela activa (uma de cada vez — as janelas não se sobrepõem).
uint32_t g_acc = 0u;
uint16_t g_cnt = 0u;
int8_t   g_active_slot = -1;  // -1 = nenhuma janela aberta

uint16_t g_slot_bar_x1000[kSlots] = {};
// EMA do desvio ×8 (guardado com 3 bits fraccionários p/ não morrer truncado).
int32_t  g_balance_x8[kSlots] = {};
uint8_t  g_fresh_mask = 0u;   // bit k = slot k fechado (ou saltado) neste ciclo
uint32_t g_cycles = 0u;

// Último slot cujo início de janela foi mesmo visitado (não só "fechado").
// Sentinela 0xFF = nenhum ainda (boot / reset / saída de sync). Persiste
// através dos intervalos "fora de janela" (ao contrário de g_active_slot,
// que volta a -1 nesses intervalos) — é o que permite detetar, no próximo
// slot visitado, se algum ficou pelo meio sem nenhuma amostra.
uint8_t  g_last_slot = 0xFFu;
uint32_t g_skip_count = 0u;  // diagnóstico: janelas inteiras saltadas

// Ciclo completo (as 4 janelas — reais ou saltadas — contabilizadas):
// média dos 4 e EMA do desvio por slot. Chamado tanto do fecho normal de
// janela como do handling de slots saltados — qualquer um pode ser o que
// completa a máscara.
void maybe_complete_cycle() noexcept {
    if (g_fresh_mask != 0x0Fu) {
        return;
    }
    g_fresh_mask = 0u;
    ++g_cycles;
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < kSlots; ++i) {
        sum += g_slot_bar_x1000[i];
    }
    const int32_t mean = static_cast<int32_t>(sum / kSlots);
    for (uint8_t i = 0u; i < kSlots; ++i) {
        const int32_t dev = static_cast<int32_t>(g_slot_bar_x1000[i]) - mean;
        // EMA α=1/8 sobre valor ×8: bal += (dev·8 − bal)/8
        g_balance_x8[i] += ((dev * 8) - g_balance_x8[i]) / 8;
    }
}

void close_active_window() noexcept {
    if (g_active_slot < 0) {
        return;
    }
    const uint8_t slot = static_cast<uint8_t>(g_active_slot);
    g_active_slot = -1;
    if (g_cnt == 0u) {
        return;
    }
    g_slot_bar_x1000[slot] = static_cast<uint16_t>(g_acc / g_cnt);
    g_fresh_mask = static_cast<uint8_t>(g_fresh_mask | (1u << slot));
    maybe_complete_cycle();
}

// Um salto de slot (rel avança ≥180° entre duas chamadas, ex.: 0 → 2 sem
// NENHUMA amostra cair no slot 1) não precisa de RPM absurdo — a cadência de
// 2ms (elapsed() em main_stm32.cpp) não é garantida, só é o caso normal:
// qualquer trabalho que atrase o loop principal (burn de calibração, secção
// crítica longa) estica o intervalo real entre polls. Mais direto ainda: um
// re-anchor de fase CMP (ecu_sched_encoder_phase_set_anchor()) pode fazer
// cycle_deg() saltar de forma descontínua de um poll para o outro, mesmo a
// RPM moderado — não é preciso RPM alto sustentado para isto acontecer.
// Sem isto, o bit do slot saltado nunca era posto em g_fresh_mask: o ciclo
// nunca completava (nem para os outros 3 slots, que continuavam vivos) e
// slot_bar_x1000(slot) ficava congelado no último valor válido para
// sempre, sem fault nenhum a avisar. Aqui marcamos os slots saltados como
// "fechados" (sem tocar no valor — fica o último conhecido, não lixo) para
// o ciclo continuar a progredir, e contamos quantos foram saltados
// (diagnóstico, map_window_skip_count() — ainda não ligado a nenhum
// consumidor nem ao protocolo 'D'; o valor stale continua a ser consumido
// silenciosamente por enc_cyl_setpoints.cpp, essa parte fica por wiring).
void mark_skipped_slots(uint8_t from_slot, uint8_t to_slot) noexcept {
    if (from_slot == 0xFFu) {
        return;  // nunca visitámos nenhum slot ainda — nada saltado
    }
    const uint8_t next_expected = static_cast<uint8_t>((from_slot + 1u) % kSlots);
    const uint8_t skipped = static_cast<uint8_t>(
        (static_cast<uint8_t>(to_slot + kSlots - next_expected)) % kSlots);
    for (uint8_t i = 0u; i < skipped; ++i) {
        const uint8_t slot = static_cast<uint8_t>((next_expected + i) % kSlots);
        g_fresh_mask = static_cast<uint8_t>(g_fresh_mask | (1u << slot));
    }
    g_skip_count += skipped;
    maybe_complete_cycle();
}

}  // namespace

void map_window_on_sample(uint16_t cycle_deg, uint16_t map_bar_x1000,
                          bool full_sync, bool cmp_ok) noexcept {
    if (map_window_enable == 0u) {
        return;
    }
    // Atribuição 720° exige sync pleno + fase de came confirmada.
    if (!full_sync || !cmp_ok) {
        g_active_slot = -1;  // aborta janela parcial (média não contaminada)
        g_fresh_mask  = 0u;
        // Saída de sync não é um "salto" de cadência de poll — reinicia a
        // detecção para não contar o hiato de sync como janela perdida.
        g_last_slot   = 0xFFu;
        return;
    }
    uint16_t deg = cycle_deg;
    if (deg >= kCycleDeg) {
        deg = static_cast<uint16_t>(deg % kCycleDeg);
    }
    uint16_t rel = static_cast<uint16_t>(deg + kCycleDeg - map_window_open_deg);
    if (rel >= kCycleDeg) {
        rel = static_cast<uint16_t>(rel - kCycleDeg);
    }
    const uint8_t  slot = static_cast<uint8_t>(rel / kSlotSpanDeg);
    const uint16_t off  = static_cast<uint16_t>(rel % kSlotSpanDeg);
    if (off < map_window_len_deg) {
        if (g_active_slot != static_cast<int8_t>(slot)) {
            // Fecha a anterior (janelas adjacentes com len=180) e abre esta.
            close_active_window();
            mark_skipped_slots(g_last_slot, slot);
            g_active_slot = static_cast<int8_t>(slot);
            g_last_slot   = slot;
            g_acc = 0u;
            g_cnt = 0u;
        }
        g_acc += map_bar_x1000;
        ++g_cnt;
    } else {
        close_active_window();
    }
}

uint16_t map_window_slot_bar_x1000(uint8_t slot) noexcept {
    return (slot < kSlots) ? g_slot_bar_x1000[slot] : 0u;
}

int16_t map_window_balance_x1000(uint8_t slot) noexcept {
    if (slot >= kSlots) {
        return 0;
    }
    const int32_t v = g_balance_x8[slot] / 8;
    return static_cast<int16_t>(v < -32768 ? -32768 : (v > 32767 ? 32767 : v));
}

uint32_t map_window_cycles() noexcept {
    return g_cycles;
}

uint8_t map_window_slot_for_cyl(uint8_t cyl) noexcept {
    if (cyl >= cfg::kCylinderCount) { return 0u; }
    return static_cast<uint8_t>((cfg::cyl_tdc_deg(cyl) / kSlotSpanDeg) % kSlots);
}

uint32_t map_window_skip_count() noexcept {
    return g_skip_count;
}

void map_window_reset() noexcept {
    g_acc = 0u;
    g_cnt = 0u;
    g_active_slot = -1;
    g_last_slot = 0xFFu;
    g_fresh_mask = 0u;
    g_cycles = 0u;
    g_skip_count = 0u;
    for (uint8_t i = 0u; i < kSlots; ++i) {
        g_slot_bar_x1000[i] = 0u;
        g_balance_x8[i] = 0;
    }
}

#if defined(EMS_HOST_TEST)
void map_window_test_set_slot_bar_x1000(uint8_t slot, uint16_t bar_x1000) noexcept {
    if (slot < kSlots) {
        g_slot_bar_x1000[slot] = bar_x1000;
    }
}
#endif

}  // namespace ems::engine
