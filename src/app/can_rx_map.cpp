#include "app/can_rx_map.h"

#include <cstdint>

namespace ems::app {

namespace {

struct SignalState {
    CanSignalDef def   = {};
    int32_t      value = 0;
    uint32_t     last_rx_ms = 0u;
    bool         seen  = false;
};

static SignalState g_signals[static_cast<uint8_t>(CanRxSignal::COUNT)] = {};

// Extrai valor bruto de um frame conforme a definição do sinal
static int32_t extract(const CanSignalDef& d, const uint8_t* data, uint8_t dlc) noexcept {
    if (d.byte_lo >= dlc) { return 0; }
    uint16_t raw = data[d.byte_lo];
    if (d.byte_hi != 0xFFu && d.byte_hi < dlc) {
        raw |= static_cast<uint16_t>(data[d.byte_hi]) << 8u;
    }
    raw >>= d.shift_right;
    raw  &= d.mask;
    return static_cast<int32_t>(raw) + static_cast<int32_t>(d.offset);
}

static inline bool timed_out(const SignalState& st, uint32_t now_ms) noexcept {
    if (!st.seen) { return true; }
    if (st.def.timeout_ms == 0u) { return false; }
    return static_cast<uint32_t>(now_ms - st.last_rx_ms) > st.def.timeout_ms;
}

} // namespace

void can_rx_map_set(CanRxSignal sig, const CanSignalDef& def) noexcept {
    const uint8_t i = static_cast<uint8_t>(sig);
    if (i >= static_cast<uint8_t>(CanRxSignal::COUNT)) { return; }
    g_signals[i].def    = def;
    g_signals[i].seen   = false;  // reset ao reconfigurar
    g_signals[i].value  = 0;
}

CanSignalDef can_rx_map_get(CanRxSignal sig) noexcept {
    const uint8_t i = static_cast<uint8_t>(sig);
    if (i >= static_cast<uint8_t>(CanRxSignal::COUNT)) { return {}; }
    return g_signals[i].def;
}

void can_rx_map_process(uint16_t id, const uint8_t* data, uint8_t dlc,
                        uint32_t now_ms) noexcept {
    for (uint8_t i = 0u; i < static_cast<uint8_t>(CanRxSignal::COUNT); ++i) {
        SignalState& st = g_signals[i];
        if (st.def.id == 0u || st.def.id != id) { continue; }
        st.value     = extract(st.def, data, dlc);
        st.last_rx_ms = now_ms;
        st.seen      = true;
    }
}

bool can_rx_gear(uint8_t& out_gear, uint32_t now_ms) noexcept {
    const SignalState& st = g_signals[static_cast<uint8_t>(CanRxSignal::GEAR)];
    if (st.def.id == 0u || timed_out(st, now_ms)) { return false; }
    const int32_t v = st.value;
    out_gear = (v < 0) ? 0u : (v > 6) ? 6u : static_cast<uint8_t>(v);
    return true;
}

bool can_rx_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept {
    const SignalState& st = g_signals[static_cast<uint8_t>(CanRxSignal::SPEED_KMH)];
    if (st.def.id == 0u || timed_out(st, now_ms)) { return false; }
    const int32_t v = st.value;
    out_kmh = (v < 0) ? 0u : (v > 65535) ? 65535u : static_cast<uint16_t>(v);
    return true;
}

} // namespace ems::app
