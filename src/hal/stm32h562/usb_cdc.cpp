/**
 * @file hal/stm32h562/usb_cdc.cpp
 * @brief USB CDC-ACM Device Driver — STM32H562 USB DRD FS
 * VID 0x0483 / PID 0x5740 (ST VCP compatible)
 * EP0 ctrl 64B | EP1 IN bulk 64B | EP2 OUT bulk 64B | EP3 IN intr 8B
 *
 * Compiles under:
 *   EMS_HOST_TEST  — stub (no register access, queues only)
 *   TARGET_STM32H562 — full USB DRD FS driver, direct register access
 */

#include "hal/stm32h562/usb_cdc.h"

#include <cstdint>
#include <cstring>

// ── Circular FIFO helpers (shared between host-test and hardware paths) ────────
namespace {

constexpr uint16_t kRxSize = 256u;
constexpr uint16_t kTxSize = 256u;
constexpr uint16_t kRxMask = kRxSize - 1u;
constexpr uint16_t kTxMask = kTxSize - 1u;

static uint8_t  g_rx[kRxSize] = {};
static uint8_t  g_tx[kTxSize] = {};
static uint16_t g_rx_head = 0u;
static uint16_t g_rx_tail = 0u;
static uint16_t g_tx_head = 0u;
static uint16_t g_tx_tail = 0u;

static bool g_dtr = false;
static bool g_rts = false;

inline bool rx_push(uint8_t b) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_rx_head + 1u) & kRxMask);
    if (next == g_rx_tail) {
        return false;  // full
    }
    g_rx[g_rx_head] = b;
    g_rx_head = next;
    return true;
}

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

// ── Hardware driver (TARGET_STM32H562) ────────────────────────────────────────
#ifndef EMS_HOST_TEST

#include "hal/regs.h"  // STM32_REG32, nvic_enable_irq, nvic_set_priority, USB_*, GPIO_*, RCC_*

// ── PMA layout (byte offsets relative to USB_PBA_BASE) ───────────────────────
// BDTable: 0x000–0x01F  (4 eps × 8 bytes)
// EP0 TX:  0x040  (64 B)
// EP0 RX:  0x080  (64 B)
// EP1 TX:  0x0C0  (64 B)
// EP2 RX:  0x100  (64 B)
// EP3 TX:  0x140  (8 B)

constexpr uint32_t kPBA_BDTable  = 0x000u;
constexpr uint32_t kPBA_EP0TX    = 0x040u;
constexpr uint32_t kPBA_EP0RX    = 0x080u;
constexpr uint32_t kPBA_EP1TX    = 0x0C0u;
constexpr uint32_t kPBA_EP2RX    = 0x100u;
constexpr uint32_t kPBA_EP3TX    = 0x140u;

// COUNT_RX field for allocating 64-byte buffer:
//   BLSIZe=1 (bit15), NUM_BLOCK=1 (bits 14:10) → 2 × 32-byte blocks = 64 B
constexpr uint16_t kCOUNT_RX_64  = static_cast<uint16_t>((1u << 15) | (1u << 10));
// COUNT_RX for 8-byte buffer:
//   BLSIZe=0 (bit15), NUM_BLOCK=3 (bits 14:10) → 4 × 2-byte blocks = 8 B
// Actually: BLSIZe=0 → 2-byte blocks; NUM_BLOCK[4:0] → (NUM_BLOCK+1) blocks
// For 8 bytes: need 4 blocks → NUM_BLOCK=3 → (3u << 10)
// But EP3 is Interrupt IN only — no RX buffer needed, use 0 allocation.
// We still write a placeholder to keep BDTable aligned.
constexpr uint16_t kCOUNT_RX_8   = static_cast<uint16_t>((0u << 15) | (3u << 10));

// ── PMA half-word accessor ────────────────────────────────────────────────────
static inline volatile uint16_t& pba_hw(uint32_t byte_offset) noexcept {
    return *reinterpret_cast<volatile uint16_t*>(USB_PBA_BASE + byte_offset);
}

// BDTable entry accessors (each entry = 4 × uint16 = 8 bytes)
// Entry layout: [ADDR_TX][COUNT_TX][ADDR_RX][COUNT_RX]  (each 16-bit, 32-bit aligned slots)
// In STM32H5 PBA the 16-bit values live at actual byte addresses (no 32-bit stride).
static inline void bdtable_set(uint32_t ep, uint16_t addr_tx, uint16_t cnt_tx,
                                uint16_t addr_rx, uint16_t cnt_rx) noexcept {
    const uint32_t base = kPBA_BDTable + ep * 8u;
    pba_hw(base + 0u) = addr_tx;
    pba_hw(base + 2u) = cnt_tx;
    pba_hw(base + 4u) = addr_rx;
    pba_hw(base + 6u) = cnt_rx;
}

static inline uint16_t bdtable_count_tx(uint32_t ep) noexcept {
    return pba_hw(kPBA_BDTable + ep * 8u + 2u);
}

static inline void bdtable_set_count_tx(uint32_t ep, uint16_t cnt) noexcept {
    pba_hw(kPBA_BDTable + ep * 8u + 2u) = cnt;
}

static inline uint16_t bdtable_count_rx(uint32_t ep) noexcept {
    return pba_hw(kPBA_BDTable + ep * 8u + 6u) & 0x3FFu;  // lower 10 bits = received count
}

static inline void bdtable_set_count_rx_cfg(uint32_t ep, uint16_t cfg) noexcept {
    pba_hw(kPBA_BDTable + ep * 8u + 6u) = cfg;
}

// ── CHEPxR safe write helpers ─────────────────────────────────────────────────
// The STAT_TX and STAT_RX fields are toggle bits.
// CTR_TX and CTR_RX are RC_W0: writing 0 clears, writing 1 has no effect.
// DTOG_TX/RX are also toggle bits.
// Safe pattern: preserve CTR bits by keeping them at 1 (they were set by HW).
// To toggle STAT to a desired value, XOR current with desired to get toggle mask.

static inline volatile uint32_t& ep_reg(uint32_t ep) noexcept {
    return *reinterpret_cast<volatile uint32_t*>(USB_BASE + ep * 4u);
}

// Write EP register preserving CTR_RX/CTR_TX (keep=1) and not toggling DTOG.
// new_stat_tx and new_stat_rx are the desired 2-bit values (0–3).
static void ep_write(uint32_t ep, uint32_t type, uint32_t addr,
                     uint32_t new_stat_tx, uint32_t new_stat_rx) noexcept {
    volatile uint32_t& r = ep_reg(ep);
    const uint32_t cur = r;

    // Compute toggle bits needed to reach desired STAT values
    const uint32_t cur_stat_tx = (cur & USB_EP_STAT_TX_MASK) >> 4u;
    const uint32_t cur_stat_rx = (cur & USB_EP_STAT_RX_MASK) >> 12u;
    const uint32_t toggle_tx   = (cur_stat_tx ^ new_stat_tx) << 4u;
    const uint32_t toggle_rx   = (cur_stat_rx ^ new_stat_rx) << 12u;

    // Build new value:
    //   - EA = addr
    //   - EP_TYPE = type
    //   - STAT_TX toggle applied
    //   - STAT_RX toggle applied
    //   - CTR_TX = 1, CTR_RX = 1 (RC_W0: writing 1 = no change)
    //   - DTOG_TX = 0 (XOR 0 = no change since toggle)
    //   - DTOG_RX = 0 (XOR 0 = no change)
    //   - EP_KIND = 0
    //   - SETUP = read-only, don't write

    uint32_t v = addr & 0xFu;            // EA
    v |= type;                            // EP_TYPE
    v |= toggle_tx;                       // STAT_TX toggle
    v |= toggle_rx;                       // STAT_RX toggle
    v |= USB_EP_CTR_TX | USB_EP_CTR_RX;  // keep CTR bits (write 1 = no clear)
    // DTOG: write 0 to NOT toggle (toggle registers XOR with what you write)
    r = v;
}

// Change only STAT_TX, preserving everything else.
static void ep_set_stat_tx(uint32_t ep, uint32_t new_stat) noexcept {
    volatile uint32_t& r = ep_reg(ep);
    const uint32_t cur = r;
    const uint32_t cur_stat = (cur & USB_EP_STAT_TX_MASK) >> 4u;
    const uint32_t toggle   = (cur_stat ^ new_stat) << 4u;

    // Preserve EA, EP_TYPE, EP_KIND; keep CTR bits; don't toggle DTOG; apply STAT toggle
    uint32_t v = cur & (0xFu | USB_EP_TYPE_MASK | (1u << 8u));  // EA + TYPE + KIND
    v |= USB_EP_CTR_TX | USB_EP_CTR_RX;  // keep CTR bits
    v |= toggle;                           // STAT_TX toggle
    // DTOG_TX: write 0 (no toggle)
    r = v;
}

// Change only STAT_RX, preserving everything else.
static void ep_set_stat_rx(uint32_t ep, uint32_t new_stat) noexcept {
    volatile uint32_t& r = ep_reg(ep);
    const uint32_t cur = r;
    const uint32_t cur_stat = (cur & USB_EP_STAT_RX_MASK) >> 12u;
    const uint32_t toggle   = (cur_stat ^ new_stat) << 12u;

    uint32_t v = cur & (0xFu | USB_EP_TYPE_MASK | (1u << 8u));  // EA + TYPE + KIND
    v |= USB_EP_CTR_TX | USB_EP_CTR_RX;
    v |= toggle;
    r = v;
}

// Clear CTR_TX (write 0 to that bit, write 1 to CTR_RX to preserve it)
static void ep_clear_ctr_tx(uint32_t ep) noexcept {
    volatile uint32_t& r = ep_reg(ep);
    const uint32_t cur = r;
    // Preserve EA, TYPE, KIND, STAT_TX (write current = no toggle), STAT_RX (no toggle)
    // CTR_TX = 0 (clear), CTR_RX = 1 (keep)
    uint32_t v = cur & (0xFu | USB_EP_TYPE_MASK | (1u << 8u));
    v |= USB_EP_CTR_RX;  // keep CTR_RX, omit CTR_TX → clears it
    // STAT fields: write 0 in toggle positions → no change
    r = v;
}

// Clear CTR_RX
static void ep_clear_ctr_rx(uint32_t ep) noexcept {
    volatile uint32_t& r = ep_reg(ep);
    const uint32_t cur = r;
    uint32_t v = cur & (0xFu | USB_EP_TYPE_MASK | (1u << 8u));
    v |= USB_EP_CTR_TX;  // keep CTR_TX, omit CTR_RX → clears it
    r = v;
}

// ── USB state machine ─────────────────────────────────────────────────────────
static uint8_t  g_usb_addr_pending = 0u;   // address to apply after status stage
static bool     g_configured       = false;
static bool     g_ep1_tx_busy      = false; // true while PMA EP1 TX buffer is in flight

// CDC line coding (baud/format/parity/bits) — stored but not applied to HW
static uint8_t  g_line_coding[7] = {
    0x00u, 0xC2u, 0x01u, 0x00u,  // 115200 baud LE
    0x00u,                         // 1 stop bit
    0x00u,                         // no parity
    0x08u                          // 8 data bits
};

// ── USB Descriptors ───────────────────────────────────────────────────────────
static const uint8_t kDeviceDesc[18] = {
    18u,           // bLength
    0x01u,         // bDescriptorType = Device
    0x10u, 0x01u,  // bcdUSB = 1.10
    0x02u,         // bDeviceClass = CDC
    0x00u,         // bDeviceSubClass
    0x00u,         // bDeviceProtocol
    64u,           // bMaxPacketSize0 = 64
    0x83u, 0x04u,  // idVendor = 0x0483
    0x40u, 0x57u,  // idProduct = 0x5740
    0x00u, 0x02u,  // bcdDevice = 2.00
    0x01u,         // iManufacturer = 1
    0x02u,         // iProduct = 2
    0x00u,         // iSerialNumber = none
    0x01u          // bNumConfigurations = 1
};

// Total config descriptor length = 9+9+5+5+4+5+7+9+7+7 = 67
static const uint8_t kConfigDesc[67] = {
    // Configuration descriptor
    9u, 0x02u, 67u, 0x00u, 2u, 1u, 0u, 0x80u, 50u,

    // Interface 0: CDC Control
    9u, 0x04u, 0u, 0u, 1u, 0x02u, 0x02u, 0x01u, 0u,

    // CDC Header functional descriptor
    5u, 0x24u, 0x00u, 0x10u, 0x01u,

    // CDC Call Management functional descriptor
    5u, 0x24u, 0x01u, 0x00u, 0x01u,

    // CDC ACM functional descriptor
    4u, 0x24u, 0x02u, 0x02u,

    // CDC Union functional descriptor
    5u, 0x24u, 0x06u, 0x00u, 0x01u,

    // EP3 IN Interrupt (management notification)
    7u, 0x05u, 0x83u, 0x03u, 8u, 0x00u, 10u,

    // Interface 1: CDC Data
    9u, 0x04u, 1u, 0u, 2u, 0x0Au, 0x00u, 0x00u, 0u,

    // EP1 IN Bulk (TX: device→host)
    7u, 0x05u, 0x81u, 0x02u, 64u, 0x00u, 0u,

    // EP2 OUT Bulk (RX: host→device)
    7u, 0x05u, 0x02u, 0x02u, 64u, 0x00u, 0u
};

// String descriptor 0: language ID list (English US)
static const uint8_t kStrDesc0[4] = { 4u, 0x03u, 0x09u, 0x04u };

// Helper: build UTF-16LE string descriptor into a local buffer and send it.
// Returns total length written.
static uint16_t build_str_desc(const char* ascii, uint8_t* buf, uint16_t buflen) noexcept {
    const uint16_t slen = static_cast<uint16_t>(strlen(ascii));
    const uint16_t total = static_cast<uint16_t>(2u + slen * 2u);
    if (buflen < total) {
        return 0u;
    }
    buf[0] = static_cast<uint8_t>(total);
    buf[1] = 0x03u;  // bDescriptorType = String
    for (uint16_t i = 0u; i < slen; ++i) {
        buf[2u + i * 2u]     = static_cast<uint8_t>(ascii[i]);
        buf[2u + i * 2u + 1u] = 0x00u;
    }
    return total;
}

// ── PMA copy helpers ──────────────────────────────────────────────────────────

// Copy from RAM to PBA (for TX)
static void pba_write(uint32_t pba_offset, const uint8_t* src, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        pba_hw(pba_offset + i) = src[i];
    }
}

// Copy from PBA to RAM (for RX)
static void pba_read(uint32_t pba_offset, uint8_t* dst, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        dst[i] = static_cast<uint8_t>(pba_hw(pba_offset + i));
    }
}

// ── EP1 TX drain: load next chunk from g_tx FIFO into PMA and arm EP1 ─────────
static void ep1_tx_kick() noexcept {
    if (g_ep1_tx_busy) {
        return;
    }
    if (g_tx_head == g_tx_tail) {
        return;  // nothing to send
    }
    // Load up to 64 bytes into EP1 TX PMA buffer
    uint8_t  buf[64u];
    uint16_t n = 0u;
    uint8_t  b = 0u;
    while (n < 64u && tx_pop(b)) {
        buf[n++] = b;
    }
    pba_write(kPBA_EP1TX, buf, n);
    bdtable_set_count_tx(1u, n);
    g_ep1_tx_busy = true;
    ep_set_stat_tx(1u, 3u);  // VALID
}

// ── Setup packet handling ─────────────────────────────────────────────────────

// Send a ZLP IN (zero-length packet) on EP0 — status stage for OUT control transfers
static void ep0_send_zlp() noexcept {
    bdtable_set_count_tx(0u, 0u);
    ep_set_stat_tx(0u, 3u);  // VALID
}

// Queue data in EP0 TX buffer (up to 64 bytes at a time)
static void ep0_send(const uint8_t* data, uint16_t len) noexcept {
    const uint16_t chunk = (len > 64u) ? 64u : len;
    pba_write(kPBA_EP0TX, data, chunk);
    bdtable_set_count_tx(0u, chunk);
    ep_set_stat_tx(0u, 3u);  // VALID
}

// SETUP packet: 8 bytes
struct SetupPacket {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

static void handle_setup(const SetupPacket& s) noexcept {
    const uint8_t req_type_type = (s.bmRequestType >> 5u) & 0x03u;  // 0=Standard, 1=Class
    const uint8_t req_type_dir  = (s.bmRequestType >> 7u) & 0x01u;  // 0=Host→Dev, 1=Dev→Host
    (void)req_type_dir;

    if (req_type_type == 0u) {
        // Standard requests
        switch (s.bRequest) {
            case 0x05u: {
                // SET_ADDRESS
                g_usb_addr_pending = static_cast<uint8_t>(s.wValue & 0x7Fu);
                ep0_send_zlp();
                break;
            }
            case 0x06u: {
                // GET_DESCRIPTOR
                const uint8_t desc_type  = static_cast<uint8_t>(s.wValue >> 8u);
                const uint8_t desc_index = static_cast<uint8_t>(s.wValue & 0xFFu);
                switch (desc_type) {
                    case 0x01u: {
                        // Device descriptor
                        const uint16_t len = (s.wLength < sizeof(kDeviceDesc))
                                           ? s.wLength
                                           : static_cast<uint16_t>(sizeof(kDeviceDesc));
                        ep0_send(kDeviceDesc, len);
                        break;
                    }
                    case 0x02u: {
                        // Configuration descriptor
                        const uint16_t len = (s.wLength < sizeof(kConfigDesc))
                                           ? s.wLength
                                           : static_cast<uint16_t>(sizeof(kConfigDesc));
                        ep0_send(kConfigDesc, len);
                        break;
                    }
                    case 0x03u: {
                        // String descriptor
                        uint8_t  sbuf[64u];
                        uint16_t slen = 0u;
                        if (desc_index == 0u) {
                            slen = sizeof(kStrDesc0);
                            memcpy(sbuf, kStrDesc0, slen);
                        } else if (desc_index == 1u) {
                            slen = build_str_desc("STMicroelectronics", sbuf, sizeof(sbuf));
                        } else if (desc_index == 2u) {
                            slen = build_str_desc("OpenEMS VCP", sbuf, sizeof(sbuf));
                        }
                        if (slen > 0u) {
                            const uint16_t len = (s.wLength < slen) ? s.wLength : slen;
                            ep0_send(sbuf, len);
                        } else {
                            // Stall for unknown string
                            ep_set_stat_tx(0u, 1u);  // STALL
                        }
                        break;
                    }
                    default:
                        ep_set_stat_tx(0u, 1u);  // STALL
                        break;
                }
                break;
            }
            case 0x09u: {
                // SET_CONFIGURATION
                if ((s.wValue & 0xFFu) == 1u) {
                    g_configured = true;
                    // Arm EP1 IN (TX) with NAK until data is queued
                    ep_write(1u, USB_EP_TYPE_BULK, 1u, 2u, 0u);  // STAT_TX=NAK, STAT_RX=DISABLED
                    // Arm EP2 OUT (RX) VALID
                    ep_write(2u, USB_EP_TYPE_BULK, 2u, 0u, 3u);  // STAT_TX=DISABLED, STAT_RX=VALID
                    // Arm EP3 IN (interrupt) with NAK
                    ep_write(3u, USB_EP_TYPE_INT, 3u, 2u, 0u);   // STAT_TX=NAK, STAT_RX=DISABLED
                }
                ep0_send_zlp();
                break;
            }
            default:
                ep_set_stat_tx(0u, 1u);  // STALL unknown
                break;
        }
    } else if (req_type_type == 1u) {
        // CDC Class requests
        switch (s.bRequest) {
            case 0x20u: {
                // SET_LINE_CODING — host sends 7-byte line coding data
                // We accept but ignore. Arm EP0 RX to receive the data.
                ep_set_stat_rx(0u, 3u);  // VALID — accept the data stage
                break;
            }
            case 0x21u: {
                // GET_LINE_CODING — send current line coding
                const uint16_t len = (s.wLength < 7u) ? s.wLength : 7u;
                ep0_send(g_line_coding, len);
                break;
            }
            case 0x22u: {
                // SET_CONTROL_LINE_STATE
                g_dtr = ((s.wValue & 0x01u) != 0u);
                g_rts = ((s.wValue & 0x02u) != 0u);
                ep0_send_zlp();
                break;
            }
            default:
                ep_set_stat_tx(0u, 1u);  // STALL
                break;
        }
    } else {
        ep_set_stat_tx(0u, 1u);  // STALL unsupported request type
    }
}

// ── USB peripheral reset/init sequence ───────────────────────────────────────
static void usb_hw_reset() noexcept {
    g_usb_addr_pending = 0u;
    g_configured       = false;
    g_ep1_tx_busy      = false;

    // Set BTABLE to 0 (BDTable at start of PBA)
    USB_BTABLE = 0u;

    // Initialise BDTable entries
    bdtable_set(0u,
                static_cast<uint16_t>(kPBA_EP0TX),
                0u,
                static_cast<uint16_t>(kPBA_EP0RX),
                kCOUNT_RX_64);

    bdtable_set(1u,
                static_cast<uint16_t>(kPBA_EP1TX),
                0u,
                0u,    // no RX buffer for EP1 IN
                0u);

    bdtable_set(2u,
                0u,    // no TX buffer for EP2 OUT
                0u,
                static_cast<uint16_t>(kPBA_EP2RX),
                kCOUNT_RX_64);

    bdtable_set(3u,
                static_cast<uint16_t>(kPBA_EP3TX),
                0u,
                0u,    // no RX for EP3 IN
                kCOUNT_RX_8);

    // Configure EP0: Control type, address 0, STAT_TX=NAK, STAT_RX=VALID
    ep_write(0u, USB_EP_TYPE_CTRL, 0u, 2u, 3u);

    // Enable function at address 0
    USB_DADDR = USB_DADDR_EF | 0u;
}

// ── CTR (Correct Transfer) handler ───────────────────────────────────────────
static void handle_ctr() noexcept {
    const uint32_t istr = USB_ISTR;
    const uint32_t ep   = istr & USB_ISTR_EP_ID;
    const bool     dir  = (istr & USB_ISTR_DIR) != 0u;  // true = OUT/SETUP

    if (ep == 0u) {
        if (dir) {
            // EP0 OUT or SETUP
            const uint32_t ep0r = USB_CHEP0R;
            const bool is_setup = (ep0r & USB_EP_SETUP) != 0u;
            ep_clear_ctr_rx(0u);

            if (is_setup) {
                // Read 8-byte SETUP packet from EP0 RX PMA buffer
                uint8_t pkt[8u];
                pba_read(kPBA_EP0RX, pkt, 8u);
                SetupPacket s;
                s.bmRequestType = pkt[0];
                s.bRequest      = pkt[1];
                s.wValue        = static_cast<uint16_t>(pkt[2] | (static_cast<uint16_t>(pkt[3]) << 8u));
                s.wIndex        = static_cast<uint16_t>(pkt[4] | (static_cast<uint16_t>(pkt[5]) << 8u));
                s.wLength       = static_cast<uint16_t>(pkt[6] | (static_cast<uint16_t>(pkt[7]) << 8u));

                // Re-arm EP0 RX for next transfer
                ep_set_stat_rx(0u, 3u);

                handle_setup(s);
            } else {
                // EP0 OUT data (e.g. SET_LINE_CODING data stage)
                const uint16_t cnt = bdtable_count_rx(0u);
                if (cnt == 7u) {
                    // Received line coding — copy to g_line_coding
                    pba_read(kPBA_EP0RX, g_line_coding, 7u);
                }
                // Re-arm RX and send status ZLP
                ep_set_stat_rx(0u, 3u);
                ep0_send_zlp();
            }
        } else {
            // EP0 IN (TX completed)
            ep_clear_ctr_tx(0u);

            // Apply pending address after status stage ZLP is sent
            if (g_usb_addr_pending != 0u) {
                USB_DADDR = USB_DADDR_EF | g_usb_addr_pending;
                g_usb_addr_pending = 0u;
            }

            // Re-arm EP0 RX for next setup/out
            ep_set_stat_rx(0u, 3u);
        }
    } else if (ep == 1u) {
        // EP1 IN: TX complete
        ep_clear_ctr_tx(1u);
        g_ep1_tx_busy = false;
        // Drain more data if available
        ep1_tx_kick();
    } else if (ep == 2u) {
        // EP2 OUT: RX data received
        ep_clear_ctr_rx(2u);
        const uint16_t cnt = bdtable_count_rx(2u);
        if (cnt > 0u) {
            uint8_t buf[64u];
            pba_read(kPBA_EP2RX, buf, cnt);
            for (uint16_t i = 0u; i < cnt; ++i) {
                rx_push(buf[i]);
            }
        }
        // Re-arm EP2 RX buffer
        bdtable_set_count_rx_cfg(2u, kCOUNT_RX_64);
        ep_set_stat_rx(2u, 3u);  // VALID
    } else if (ep == 3u) {
        // EP3 IN: notification TX complete (not used for data)
        ep_clear_ctr_tx(3u);
    }
}

#endif  // !EMS_HOST_TEST

}  // anonymous namespace

// ── USB IRQ handler (hardware only) ──────────────────────────────────────────
#ifndef EMS_HOST_TEST
extern "C" void USB_IRQHandler() noexcept {
    uint32_t istr = USB_ISTR;

    if (istr & USB_ISTR_RESET) {
        USB_ISTR = ~USB_ISTR_RESET;  // clear reset flag
        usb_hw_reset();
    }

    if (istr & USB_ISTR_SUSP) {
        USB_ISTR = ~USB_ISTR_SUSP;
        USB_CNTR |= USB_CNTR_FSUSP;
    }

    if (istr & USB_ISTR_WKUP) {
        USB_ISTR = ~USB_ISTR_WKUP;
        USB_CNTR &= ~USB_CNTR_FSUSP;
    }

    // Handle all pending CTR events (may be multiple)
    while (USB_ISTR & USB_ISTR_CTR) {
        handle_ctr();
    }
}
#endif  // !EMS_HOST_TEST

// ── Public API ────────────────────────────────────────────────────────────────
namespace ems::hal {

void usb_cdc_init() noexcept {
    g_rx_head = 0u;
    g_rx_tail = 0u;
    g_tx_head = 0u;
    g_tx_tail = 0u;
    g_dtr     = false;
    g_rts     = false;

#ifndef EMS_HOST_TEST
    g_usb_addr_pending = 0u;
    g_configured       = false;
    g_ep1_tx_busy      = false;

    // 1. Enable GPIOA clock (for PA11/PA12)
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN;

    // 2. Configure PA11 (DM) and PA12 (DP) as AF10, very-high speed
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 11u, GPIO_AF10);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 12u, GPIO_AF10);

    // 3. Enable USB clock on APB2
    RCC_APB2ENR |= RCC_APB2ENR_USBEN;

    // 4. Power up USB transceiver: clear PDWN
    USB_CNTR = USB_CNTR_PDWN | USB_CNTR_USBRST;  // start with reset asserted
    // Small delay: ~1 µs startup time for analog block.
    // Without a timer here, execute a short spin (~1000 NOPs at 250 MHz >> 1 µs)
    for (volatile uint32_t i = 0u; i < 1000u; ++i) {
        __asm__ volatile ("nop");
    }
    USB_CNTR = USB_CNTR_USBRST;  // clear PDWN, keep USBRST

    // 5. Clear USB reset
    USB_CNTR = 0u;

    // 6. Clear any stale interrupt flags
    USB_ISTR = 0u;

    // 7. Set BTABLE = 0, configure initial endpoint and BDTable
    usb_hw_reset();

    // 8. Enable interrupts: Reset, CTR, Suspend, Wakeup
    USB_CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;

    // 9. Configure NVIC
    nvic_set_priority(IRQ_USB, 2u);
    nvic_enable_irq(IRQ_USB);
#else
    // Host test: treat DTR/RTS as always active for protocol tests
    g_dtr = true;
    g_rts = true;
#endif
}

void usb_cdc_poll() noexcept {
#ifndef EMS_HOST_TEST
    // Kick TX if we have data and EP1 is idle (belt-and-suspenders: IRQ handles it too)
    if (g_configured) {
        ep1_tx_kick();
    }
#else
    // Stub: drain TX queue as "sent"
    uint8_t b = 0u;
    while (tx_pop(b)) {
        (void)b;
    }
#endif
}

void usb_cdc_send_byte(uint8_t byte) noexcept {
    static_cast<void>(tx_push(byte));
#ifndef EMS_HOST_TEST
    if (g_configured) {
        ep1_tx_kick();
    }
#endif
}

void usb_cdc_send_bytes(const uint8_t* data, uint16_t len) noexcept {
    if (data == nullptr) {
        return;
    }
    for (uint16_t i = 0u; i < len; ++i) {
        if (!tx_push(data[i])) {
            break;
        }
    }
#ifndef EMS_HOST_TEST
    if (g_configured) {
        ep1_tx_kick();
    }
#endif
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
