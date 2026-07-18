#include "engine/map_window.h"
#include "engine/calibration.h"

namespace ems::engine {

namespace {

constexpr uint8_t  kSlots        = 4u;
constexpr uint16_t kSlotSpanDeg  = 180u;
constexpr uint16_t kCycleDeg     = 720u;
constexpr uint16_t kDegPerTooth  = 6u;

// Acumulador da janela activa (uma de cada vez — as janelas não se sobrepõem).
uint32_t g_acc = 0u;
uint16_t g_cnt = 0u;
int8_t   g_active_slot = -1;  // -1 = nenhuma janela aberta

uint16_t g_slot_bar_x1000[kSlots] = {};
// EMA do desvio ×8 (guardado com 3 bits fraccionários p/ não morrer truncado).
int32_t  g_balance_x8[kSlots] = {};
uint8_t  g_fresh_mask = 0u;   // bit k = slot k fechado neste ciclo
uint32_t g_cycles = 0u;

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
    if (g_fresh_mask != 0x0Fu) {
        return;
    }
    // Ciclo completo: média dos 4 e EMA do desvio por slot.
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

}  // namespace

void map_window_on_tooth(const ems::drv::CkpSnapshot& snap,
                         uint16_t map_bar_x1000) noexcept {
    if (map_window_enable == 0u) {
        return;
    }
    // Atribuição 720° exige sync pleno + fase de came confirmada.
    if (snap.state != ems::drv::SyncState::FULL_SYNC || snap.cmp_confirms < 2u) {
        g_active_slot = -1;  // aborta janela parcial (média não contaminada)
        g_fresh_mask  = 0u;
        return;
    }
    const uint16_t deg = static_cast<uint16_t>(
        snap.tooth_index * kDegPerTooth + (snap.phase_A ? 0u : 360u));
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
            g_active_slot = static_cast<int8_t>(slot);
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

void map_window_reset() noexcept {
    g_acc = 0u;
    g_cnt = 0u;
    g_active_slot = -1;
    g_fresh_mask = 0u;
    g_cycles = 0u;
    for (uint8_t i = 0u; i < kSlots; ++i) {
        g_slot_bar_x1000[i] = 0u;
        g_balance_x8[i] = 0;
    }
}

}  // namespace ems::engine
