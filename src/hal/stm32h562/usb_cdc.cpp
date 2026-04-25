#include "hal/stm32h562/usb_cdc.h"

#include <cstdint>

namespace {

constexpr uint16_t kRxSize = 256u;
constexpr uint16_t kTxSize = 256u;
constexpr uint16_t kRxMask = kRxSize - 1u;
constexpr uint16_t kTxMask = kTxSize - 1u;

static uint8_t g_rx[kRxSize] = {};
static uint8_t g_tx[kTxSize] = {};
static uint16_t g_rx_head = 0u;
static uint16_t g_rx_tail = 0u;
static uint16_t g_tx_head = 0u;
static uint16_t g_tx_tail = 0u;

// Phase-3 placeholder policy:
// - USB transport is now the only TunerStudio path.
// - Until low-level USB FS driver is finalized, DTR is treated as true
//   to keep protocol service path active.
static bool g_dtr = true;
static bool g_rts = true;

inline bool rx_pop(uint8_t& b) noexcept {
    if (g_rx_head == g_rx_tail) {
        return false;
    }
    b = g_rx[g_rx_tail];
    g_rx_tail = static_cast<uint16_t>((g_rx_tail + 1u) & kRxMask);
    return true;
}

inline bool tx_push(uint8_t b) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_tx_head + 1u) & kTxMask);
    if (next == g_tx_tail) {
        return false;
    }
    g_tx[g_tx_head] = b;
    g_tx_head = next;
    return true;
}

inline bool tx_pop(uint8_t& b) noexcept {
    if (g_tx_head == g_tx_tail) {
        return false;
    }
    b = g_tx[g_tx_tail];
    g_tx_tail = static_cast<uint16_t>((g_tx_tail + 1u) & kTxMask);
    return true;
}

}  // namespace

namespace ems::hal {

void usb_cdc_init() noexcept {
    g_rx_head = 0u;
    g_rx_tail = 0u;
    g_tx_head = 0u;
    g_tx_tail = 0u;
    g_dtr = true;
    g_rts = true;
}

void usb_cdc_poll() noexcept {
    // Stub transport: no hardware backend yet.
    // Keep queues consistent and drain TX queue as "sent".
    uint8_t b = 0u;
    while (tx_pop(b)) {
        (void)b;
    }
}

void usb_cdc_send_byte(uint8_t byte) noexcept {
    static_cast<void>(tx_push(byte));
}

void usb_cdc_send_bytes(const uint8_t* data, uint16_t len) noexcept {
    if (data == nullptr) {
        return;
    }
    for (uint16_t i = 0u; i < len; ++i) {
        if (!tx_push(data[i])) {
            return;
        }
    }
}

bool usb_cdc_available() noexcept {
    return g_rx_head != g_rx_tail;
}

uint8_t usb_cdc_read_byte() noexcept {
    uint8_t b = 0u;
    static_cast<void>(rx_pop(b));
    return b;
}

uint16_t usb_cdc_read_bytes(uint8_t* buffer, uint16_t max_len) noexcept {
    if (buffer == nullptr || max_len == 0u) {
        return 0u;
    }
    uint16_t n = 0u;
    while (n < max_len) {
        uint8_t b = 0u;
        if (!rx_pop(b)) {
            break;
        }
        buffer[n++] = b;
    }
    return n;
}

bool usb_cdc_dtr() noexcept {
    return g_dtr;
}

bool usb_cdc_rts() noexcept {
    return g_rts;
}

}  // namespace ems::hal
