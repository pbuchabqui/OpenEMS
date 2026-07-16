#include "app/can_rx_map.h"

#include <cstdint>
#include <cstring>

namespace ems::app {

namespace {

struct SignalState {
    CanSignalDef def   = {};
    int32_t      value = 0;
    uint32_t     last_rx_ms = 0u;
    bool         seen  = false;
};

static SignalState g_signals[static_cast<uint8_t>(CanRxSignal::COUNT)] = {};

// Extrai valor bruto de um frame conforme a definição do sinal.
// 8-bit: byte_hi=0xFF. 16-bit LE: byte_lo=LSB, byte_hi=MSB, mask=0xFFFF.
static int32_t extract(const CanSignalDef& d, const uint8_t* data, uint8_t dlc) noexcept {
    if (d.byte_lo >= dlc) { return 0; }
    uint32_t raw = data[d.byte_lo];
    if (d.byte_hi != 0xFFu && d.byte_hi < dlc) {
        raw |= static_cast<uint32_t>(data[d.byte_hi]) << 8u;
    }
    raw >>= d.shift_right;
    raw  &= static_cast<uint32_t>(d.mask == 0u ? 0xFFFFu : d.mask);
    return static_cast<int32_t>(raw) + static_cast<int32_t>(d.offset);
}

static inline bool timed_out(const SignalState& st, uint32_t now_ms) noexcept {
    if (!st.seen) { return true; }
    if (st.def.timeout_ms == 0u) { return false; }
    return static_cast<uint32_t>(now_ms - st.last_rx_ms) > st.def.timeout_ms;
}

static bool read_u16_kmh(CanRxSignal sig, uint16_t& out_kmh, uint32_t now_ms) noexcept {
    const uint8_t i = static_cast<uint8_t>(sig);
    if (i >= static_cast<uint8_t>(CanRxSignal::COUNT)) { return false; }
    const SignalState& st = g_signals[i];
    if (st.def.id == 0u || timed_out(st, now_ms)) { return false; }
    const int32_t v = st.value;
    out_kmh = (v < 0) ? 0u : (v > 65535) ? 65535u : static_cast<uint16_t>(v);
    return true;
}

static void pack_def(uint8_t* dst, const CanSignalDef& d) noexcept {
    std::memcpy(dst + 0, &d.id, 2u);
    dst[2] = d.byte_lo;
    dst[3] = d.byte_hi;
    dst[4] = d.shift_right;
    dst[5] = d._pad;
    std::memcpy(dst + 6, &d.mask, 2u);
    std::memcpy(dst + 8, &d.offset, 2u);
    std::memcpy(dst + 10, &d.timeout_ms, 2u);
}

static CanSignalDef unpack_def(const uint8_t* src) noexcept {
    CanSignalDef d = {};
    std::memcpy(&d.id, src + 0, 2u);
    d.byte_lo = src[2];
    d.byte_hi = src[3];
    d.shift_right = src[4];
    d._pad = src[5];
    std::memcpy(&d.mask, src + 6, 2u);
    std::memcpy(&d.offset, src + 8, 2u);
    std::memcpy(&d.timeout_ms, src + 10, 2u);
    // Sanitize: 11-bit ID, byte indices
    d.id = static_cast<uint16_t>(d.id & 0x7FFu);
    if (d.byte_lo > 7u) { d.byte_lo = 7u; }
    if (d.byte_hi != 0xFFu && d.byte_hi > 7u) { d.byte_hi = 7u; }
    if (d.shift_right > 15u) { d.shift_right = 15u; }
    if (d.mask == 0u) { d.mask = 0xFFFFu; }
    return d;
}

} // namespace

void can_rx_map_set(CanRxSignal sig, const CanSignalDef& def) noexcept {
    const uint8_t i = static_cast<uint8_t>(sig);
    if (i >= static_cast<uint8_t>(CanRxSignal::COUNT)) { return; }
    g_signals[i].def    = def;
    g_signals[i].seen   = false;  // reset ao reconfigurar
    g_signals[i].value  = 0;
    g_signals[i].last_rx_ms = 0u;
}

CanSignalDef can_rx_map_get(CanRxSignal sig) noexcept {
    const uint8_t i = static_cast<uint8_t>(sig);
    if (i >= static_cast<uint8_t>(CanRxSignal::COUNT)) { return {}; }
    return g_signals[i].def;
}

void can_rx_map_process(uint16_t id, const uint8_t* data, uint8_t dlc,
                        uint32_t now_ms) noexcept {
    if (data == nullptr || dlc == 0u) { return; }
    for (uint8_t i = 0u; i < static_cast<uint8_t>(CanRxSignal::COUNT); ++i) {
        SignalState& st = g_signals[i];
        if (st.def.id == 0u || st.def.id != id) { continue; }
        st.value      = extract(st.def, data, dlc);
        st.last_rx_ms = now_ms;
        st.seen       = true;
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
    return read_u16_kmh(CanRxSignal::SPEED_KMH, out_kmh, now_ms);
}

bool can_rx_wheel_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept {
    return read_u16_kmh(CanRxSignal::WHEEL_SPEED_KMH, out_kmh, now_ms);
}

void can_rx_map_serialize_to_page0(uint8_t* page0, uint16_t len) noexcept {
    if (page0 == nullptr || len < (kCanRxMapPage0Off + kCanRxMapPage0Len)) {
        return;
    }
    for (uint8_t i = 0u; i < static_cast<uint8_t>(CanRxSignal::COUNT); ++i) {
        pack_def(page0 + kCanRxMapPage0Off + (i * kCanRxSignalWireLen),
                 g_signals[i].def);
    }
}

void can_rx_map_apply_from_page0(const uint8_t* page0, uint16_t len) noexcept {
    if (page0 == nullptr || len < (kCanRxMapPage0Off + kCanRxMapPage0Len)) {
        return;
    }
    for (uint8_t i = 0u; i < static_cast<uint8_t>(CanRxSignal::COUNT); ++i) {
        const CanSignalDef d = unpack_def(
            page0 + kCanRxMapPage0Off + (i * kCanRxSignalWireLen));
        can_rx_map_set(static_cast<CanRxSignal>(i), d);
    }
}

} // namespace ems::app
