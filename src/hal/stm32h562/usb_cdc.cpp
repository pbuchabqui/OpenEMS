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

// Diagnóstico de enumeração: maior estágio alcançado (ver usb_cdc.h). File-scope para
// ser visível tanto pela ISR (fora do namespace anônimo) quanto pelos handlers e accessor.
static volatile uint8_t g_usb_dbg_stage = 1u;
static inline void dbg_stage(uint8_t s) noexcept {
    if (g_usb_dbg_stage < s) { g_usb_dbg_stage = s; }
}

// ── Circular FIFO helpers (shared between host-test and hardware paths) ────────
namespace {

constexpr uint16_t kRxSize = 256u;
constexpr uint16_t kTxSize = 256u;
constexpr uint16_t kRxMask = kRxSize - 1u;
constexpr uint16_t kTxMask = kTxSize - 1u;

static uint8_t           g_rx[kRxSize] = {};
static uint8_t           g_tx[kTxSize] = {};
static volatile uint16_t g_rx_head = 0u;
static volatile uint16_t g_rx_tail = 0u;
static volatile uint16_t g_tx_head = 0u;
static volatile uint16_t g_tx_tail = 0u;

static volatile bool g_dtr = false;
static volatile bool g_rts = false;

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

// COUNT_RX configuration for the BDTable RXBD high half-word (BLSIZe + NUM_BLOCK).
// STM32H562 DRD FS: BLSIZe=1 (32-byte blocks); a 64-byte buffer needs NUM_BLOCK=2
// (2×32=64). NUM_BLOCK lives at bits[14:10], so the value is (1<<15)|(2<<10)=0x8800.
// (An earlier (1<<15)|(1<<10)=0x8400 encoded NUM_BLOCK=1 → only 32 B, too small for a
// 64-byte OUT packet. Matches the golden reference PMA_RX_64=0x8800.)
constexpr uint16_t kCOUNT_RX_64 = static_cast<uint16_t>((1u << 15) | (2u << 10));
// EP3 TX-only — no RX buffer allocated.
constexpr uint16_t kCOUNT_RX_8  = 0u;

// ── PMA 32-bit accessor ────────────────────────────────────────────────────────
// STM32H562 DRD FS PMA is 32-bit access only (LL comment: "PMA access 32bit only").
// All BDTable and data buffer operations use uint32_t reads/writes.
static inline volatile uint32_t& pba_w32(uint32_t byte_offset) noexcept {
    return *reinterpret_cast<volatile uint32_t*>(USB_PBA_BASE + byte_offset);
}

// ── BDTable entry accessors ────────────────────────────────────────────────────────
// STM32H5 USB DRD FS BDTable layout per endpoint (8 bytes = two 32-bit words):
//   TXBD (offset 0): bits[15:0]=ADDR_TX,  bits[25:16]=COUNT_TX
//   RXBD (offset 4): bits[15:0]=ADDR_RX,  bits[31:16]=COUNT_RX/BLSIZe/NUM_BLOCK
// Address is the LOW half-word, byte count is the HIGH half-word (RM0481 §52.7;
// matches ST HAL USB_DRD_SET_CHEP_TX_ADDRESS and the golden reference). The CMSIS
// USB_PMA_TXBD_ADDMSK=0xFFFF0000 mask is mislabelled — do NOT use it for packing.
static inline void bdtable_set(uint32_t ep, uint16_t addr_tx, uint16_t cnt_tx,
                                uint16_t addr_rx, uint16_t cnt_rx) noexcept {
    pba_w32(ep * 8u + 0u) = (static_cast<uint32_t>(cnt_tx) << 16) | addr_tx;  // TXBD
    pba_w32(ep * 8u + 4u) = (static_cast<uint32_t>(cnt_rx) << 16) | addr_rx;  // RXBD
}

static inline uint16_t bdtable_count_tx(uint32_t ep) noexcept {
    return static_cast<uint16_t>((pba_w32(ep * 8u) >> 16) & 0x3FFu);  // COUNT_TX bits[25:16]
}

static inline void bdtable_set_count_tx(uint32_t ep, uint16_t cnt) noexcept {
    volatile uint32_t& txbd = pba_w32(ep * 8u);
    txbd = (txbd & 0x0000FFFFu) | (static_cast<uint32_t>(cnt) << 16);  // keep ADDR, set COUNT
}

static inline uint16_t bdtable_count_rx(uint32_t ep) noexcept {
    return static_cast<uint16_t>((pba_w32(ep * 8u + 4u) >> 16) & 0x3FFu);  // COUNT_RX bits[25:16]
}

static inline void bdtable_set_count_rx_cfg(uint32_t ep, uint16_t cfg) noexcept {
    volatile uint32_t& rxbd = pba_w32(ep * 8u + 4u);
    rxbd = (rxbd & 0x0000FFFFu) | (static_cast<uint32_t>(cfg) << 16);  // keep ADDR, set cfg
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
static uint8_t           g_usb_addr_pending = 0u;
static volatile bool     g_configured       = false;
static volatile bool     g_ep1_tx_busy      = false;
// ZLP pendente: o último pacote IN foi full-size (64B). USB bulk exige um
// zero-length packet para terminar transferências múltiplas de wMaxPacketSize —
// sem ele o cdc_acm do host segura os dados no URB até chegar um short packet.
static volatile bool     g_ep1_zlp_pending  = false;
// EP2 (bulk OUT) back-pressure: when the RX FIFO can't hold another full 64-byte
// packet we leave the endpoint at NAK instead of re-arming RX VALID, so the host
// stalls instead of us silently dropping bytes. Re-armed once the consumer drains.
static volatile bool     g_rx_paused        = false;

// Free slots in the RX FIFO (one slot is always reserved to distinguish full/empty).
static inline uint16_t rx_free() noexcept {
    return static_cast<uint16_t>((g_rx_tail - g_rx_head - 1u) & kRxMask);
}

// Re-arm EP2 RX if it was paused for back-pressure and the FIFO now has room for a
// full max-size packet. Safe to call from the main loop and from read paths.
static void ep2_rx_resume() noexcept {
    __asm__ volatile("cpsid i" ::: "memory");
    const bool resume = g_rx_paused && (rx_free() >= 64u);
    if (resume) {
        g_rx_paused = false;
    }
    __asm__ volatile("cpsie i" ::: "memory");
    if (resume) {
        bdtable_set_count_rx_cfg(2u, kCOUNT_RX_64);
        ep_set_stat_rx(2u, 3u);  // VALID
    }
}

// EP0 multi-packet TX state
static const uint8_t* g_ep0_tx_ptr = nullptr;
static uint16_t       g_ep0_tx_rem = 0u;
static bool           g_ep0_tx_zlp = false;

// Stable buffer for EP0 string descriptor responses (not stack-local — ep0_tx_continue()
// may fire from ISR after handle_setup() has returned, so the pointer must stay valid).
static uint8_t g_ep0_str_buf[64u] = {};

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
    0x00u, 0x02u,  // bcdUSB = 2.00
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

// Copy from RAM to PBA (for TX) — 32-bit access only (H562 PMA requirement)
static void pba_write(uint32_t pba_offset, const uint8_t* src, uint16_t len) noexcept {
    const uint16_t nwords = static_cast<uint16_t>((len + 3u) / 4u);
    for (uint16_t i = 0u; i < nwords; ++i) {
        const uint16_t base = static_cast<uint16_t>(i * 4u);
        uint32_t val = 0u;
        for (uint8_t b = 0u; b < 4u; ++b) {
            if (static_cast<uint16_t>(base + b) < len) {
                val |= static_cast<uint32_t>(src[base + b]) << (8u * b);
            }
        }
        pba_w32(pba_offset + i * 4u) = val;
    }
}

// Copy from PBA to RAM (for RX) — 32-bit access only
static void pba_read(uint32_t pba_offset, uint8_t* dst, uint16_t len) noexcept {
    const uint16_t nwords = static_cast<uint16_t>((len + 3u) / 4u);
    for (uint16_t i = 0u; i < nwords; ++i) {
        const uint32_t val = pba_w32(pba_offset + i * 4u);
        const uint16_t base = static_cast<uint16_t>(i * 4u);
        for (uint8_t b = 0u; b < 4u; ++b) {
            if (static_cast<uint16_t>(base + b) < len) {
                dst[base + b] = static_cast<uint8_t>(val >> (8u * b));
            }
        }
    }
}

// ── EP1 TX drain: load next chunk from g_tx FIFO into PMA and arm EP1 ─────────
static void ep1_tx_kick() noexcept {
    // Atomically claim g_ep1_tx_busy before loading data.
    // CPSID/CPSIE prevent a race between main-loop callers and the USB ISR;
    // they are effectively no-ops when already executing inside the ISR.
    __asm__ volatile("cpsid i" ::: "memory");
    const bool busy_or_empty = g_ep1_tx_busy || (g_tx_head == g_tx_tail);
    if (!busy_or_empty) {
        g_ep1_tx_busy = true;
    }
    __asm__ volatile("cpsie i" ::: "memory");
    if (busy_or_empty) {
        return;
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
    g_ep1_zlp_pending = (n == 64u);  // full-size: terminar com ZLP se a fila esvaziar
    ep_set_stat_tx(1u, 3u);  // VALID
}

// ── Setup packet handling ─────────────────────────────────────────────────────

// Send a ZLP IN (zero-length packet) on EP0 — status stage for OUT control transfers
static void ep0_send_zlp() noexcept {
    g_ep0_tx_ptr = nullptr;
    g_ep0_tx_rem = 0u;
    g_ep0_tx_zlp = false;
    bdtable_set_count_tx(0u, 0u);
    ep_set_stat_tx(0u, 3u);  // VALID
}

// Send next chunk of a pending EP0 IN transfer (called from ep0_send and EP0 IN CTR).
static void ep0_tx_continue() noexcept {
    if (g_ep0_tx_rem == 0u) {
        if (g_ep0_tx_zlp) {
            // Transfer length was an exact multiple of 64 — send terminating ZLP
            g_ep0_tx_zlp = false;
            bdtable_set_count_tx(0u, 0u);
            ep_set_stat_tx(0u, 3u);
        }
        return;
    }
    const uint16_t chunk = (g_ep0_tx_rem > 64u) ? 64u : g_ep0_tx_rem;
    pba_write(kPBA_EP0TX, g_ep0_tx_ptr, chunk);
    bdtable_set_count_tx(0u, chunk);
    g_ep0_tx_ptr += chunk;
    g_ep0_tx_rem  = static_cast<uint16_t>(g_ep0_tx_rem - chunk);
    ep_set_stat_tx(0u, 3u);  // VALID
}

// Begin a multi-packet (or single-packet) EP0 IN data transfer.
static void ep0_send(const uint8_t* data, uint16_t len) noexcept {
    g_ep0_tx_ptr = data;
    g_ep0_tx_rem = len;
    // A ZLP is needed only when len is a non-zero exact multiple of max packet (64)
    // to signal end-of-transfer to the host.
    g_ep0_tx_zlp = (len > 0u && (len % 64u) == 0u);
    ep0_tx_continue();
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
    // Abort any in-progress multi-packet EP0 TX: a new SETUP supersedes whatever was in flight.
    g_ep0_tx_ptr = nullptr;
    g_ep0_tx_rem = 0u;
    g_ep0_tx_zlp = false;

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
                        dbg_stage(5u);  // host pediu GET_DESCRIPTOR(device)
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
                        // String descriptor — build into g_ep0_str_buf (static, not stack)
                        // so ep0_tx_continue() can safely dereference g_ep0_tx_ptr from ISR.
                        uint16_t slen = 0u;
                        if (desc_index == 0u) {
                            slen = sizeof(kStrDesc0);
                            memcpy(g_ep0_str_buf, kStrDesc0, slen);
                        } else if (desc_index == 1u) {
                            slen = build_str_desc("STMicroelectronics", g_ep0_str_buf,
                                                  sizeof(g_ep0_str_buf));
                        } else if (desc_index == 2u) {
                            slen = build_str_desc("OpenEMS VCP", g_ep0_str_buf,
                                                  sizeof(g_ep0_str_buf));
                        }
                        if (slen > 0u) {
                            const uint16_t len = (s.wLength < slen) ? s.wLength : slen;
                            ep0_send(g_ep0_str_buf, len);
                        } else {
                            ep_set_stat_tx(0u, 1u);  // STALL for unknown string index
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
                    dbg_stage(6u);  // SET_CONFIGURATION → enumerado
                    // USB spec: reset data toggles to DATA0 on all non-control endpoints.
                    // DTOG bits are toggle-on-write: write 1 to flip; write 0 to leave.
                    for (uint32_t ep = 1u; ep <= 3u; ++ep) {
                        volatile uint32_t& r = ep_reg(ep);
                        const uint32_t cur = r;
                        uint32_t v = cur & (0xFu | USB_EP_TYPE_MASK | (1u << 8u));
                        v |= USB_EP_CTR_TX | USB_EP_CTR_RX;
                        if (cur & USB_EP_DTOG_TX) { v |= USB_EP_DTOG_TX; }  // toggle → DATA0
                        if (cur & USB_EP_DTOG_RX) { v |= USB_EP_DTOG_RX; }  // toggle → DATA0
                        r = v;
                    }
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
                if (s.wLength != 7u) {
                    ep_set_stat_tx(0u, 1u);  // STALL: CDC spec requires exactly 7 bytes
                    break;
                }
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
    g_ep0_tx_ptr       = nullptr;
    g_ep0_tx_rem       = 0u;
    g_ep0_tx_zlp       = false;

    // BDTable is at PMA offset 0 (fixed on H562 DRD FS — BTABLE register is RESERVED).
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

            if (g_ep0_tx_rem > 0u || g_ep0_tx_zlp) {
                // More data to send — continue multi-packet transfer
                ep0_tx_continue();
            } else {
                // Transfer complete — re-arm EP0 RX for STATUS stage / next SETUP
                ep_set_stat_rx(0u, 3u);
            }
        }
    } else if (ep == 1u) {
        // EP1 IN: TX complete
        ep_clear_ctr_tx(1u);
        g_ep1_tx_busy = false;
        if (g_tx_head != g_tx_tail) {
            // Mais dados na fila — continua a transferência (ZLP só no fim)
            ep1_tx_kick();
        } else if (g_ep1_zlp_pending) {
            // Fila vazia após pacote full-size: ZLP termina a transferência no host
            g_ep1_zlp_pending = false;
            g_ep1_tx_busy = true;
            bdtable_set_count_tx(1u, 0u);
            ep_set_stat_tx(1u, 3u);  // VALID
        }
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
        // Back-pressure: only re-arm RX if the FIFO can absorb another full packet.
        // Otherwise leave the endpoint at NAK (set by HW after this transfer) so the
        // host retries instead of us dropping bytes; ep2_rx_resume() re-arms on drain.
        if (rx_free() >= 64u) {
            bdtable_set_count_rx_cfg(2u, kCOUNT_RX_64);
            ep_set_stat_rx(2u, 3u);  // VALID
        } else {
            g_rx_paused = true;
        }
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

    dbg_stage(2u);                                   // ISR USB disparou
    if (istr & USB_ISTR_RESET) { dbg_stage(3u); }    // RESET do host
    if (istr & USB_ISTR_CTR)   { dbg_stage(4u); }    // transferência (SETUP/CTR)

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
        // ep1_tx_kick removido: chamar EP1R do ISR antes do core resumir corrompe STAT_TX
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

    // 1. Enable VDDUSB 3.3V supply — required BEFORE USB peripheral init.
    // Without this the USB PHY has no power and enumeration never starts.
    // Confirmed from WeActStudio example: HAL_PWREx_EnableVddUSB() → SET_BIT(PWR->USBSCR, USB33SV)
    PWR_USBSCR |= PWR_USBSCR_USB33SV;
    // Wait for VDDUSB ready (PWR_VMSR.USB33RDY)
    for (volatile uint32_t i = 0u; (PWR_VMSR & PWR_VMSR_USB33RDY) == 0u && i < 100000u; ++i) {}

    // 2/3. NÃO configurar GPIO PA11/PA12.
    // O STM32H5 usa USB_DRD_FS com PHY full-speed EMBUTIDO: DP/DM são pinos DEDICADOS,
    // dirigidos diretamente pelo transceiver — NÃO se configura GPIO AF (isso é só para o
    // USB_OTG_FS de F4/F7/H7). Forçar PA11/PA12 como AF10 digital push-pull briga com o
    // PHY analógico e impede a sinalização → device anexa mas EP0 falha (descriptor -32).
    // O golden reference (USBCDC_H5, hardware-verificado) deixa esses pinos no reset.

    // 4. Select HSI48 as USB kernel clock (RCC_CCIPR4 USBSEL=11) and enable APB2 clock.
    // CRITICAL: USBSEL=00 is NOCLOCK on H5 — the USB peripheral would have no kernel
    // clock and never enumerate. HSI48 = 0b11 (matches the proven golden reference).
    RCC_CCIPR4 = (RCC_CCIPR4 & ~RCC_CCIPR4_USBSEL_MSK) | RCC_CCIPR4_USBSEL_HSI48;
    RCC_APB2ENR |= RCC_APB2ENR_USBEN;

    // 4a. CRS: trim do HSI48 contra o USB SOF. HSI48 não-trimado tem ±1-2% (fora da
    // tolerância USB FS de ±0.25%) → enumeração pode falhar (descriptor read -32). O golden
    // reference habilita CRS aqui; o usb_cdc_init() do OpenEMS dependia de system_stm32_init()
    // ter feito isso, mas o caminho minimal/isolado não chama. Reset defaults de CRS_CFGR já
    // têm RELOAD=47999/FELIM=34; só precisamos SYNCSRC=USB + AUTOTRIMEN + CEN.
    RCC_APB1LENR |= RCC_APB1LENR_CRSEN;
    CRS_CFGR = (CRS_CFGR & ~(3u << 28u)) | CRS_CFGR_SYNCSRC_USB;  // SYNCSRC=0b10 (USB SOF)
    CRS_CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    // 5. Power up USB transceiver: clear PDWN (PDWN=0) mantendo o core em reset (USBRST=1).
    // RM0481 §52.4.1: mínimo 1 µs de startup analógico DEPOIS de limpar PDWN — só então
    // libera o reset. (Antes o delay estava ANTES de limpar PDWN, dando ~0 de startup ao
    // transceiver, divergindo do reference que faz PDWN=0 → delay → reset=0.)
    USB_CNTR = USB_CNTR_USBRST;  // PDWN=0, USBRST=1
    __asm__ volatile("dsb" ::: "memory");
    for (volatile uint32_t i = 0u; i < 5000u; ++i) {  // t_STARTUP > 1 µs
        __asm__ volatile("nop");
    }

    // 6. Clear USB reset
    USB_CNTR = 0u;

    // 7. Clear any stale interrupt flags
    USB_ISTR = 0u;

    // 8. Configure BDTable (at fixed PMA offset 0) and initialise endpoints
    usb_hw_reset();

    // 9. Enable interrupts: Reset, CTR, Suspend, Wakeup
    USB_CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;

    // 10. Enable internal D+ pull-up BEFORE NVIC enable: DPPU asserts D+ to signal
    // USB FS presence to the host. Setting it before NVIC enable guarantees the
    // peripheral is fully initialised before the first RESET IRQ can fire.
    USB_BCDR |= (1u << 15u);

    // 11. Configure NVIC
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
    // Bench device: se autosuspend setou FSUSP, limpa imediatamente do thread context
    // Não fazer isso no WKUP ISR pois modifica EP1R antes do core USB estar pronto
    if (USB_CNTR & USB_CNTR_FSUSP) {
        USB_CNTR &= ~USB_CNTR_FSUSP;
    }
    if (g_configured) {
        ep1_tx_kick();
        ep2_rx_resume();

        // TX watchdog: se uma transação IN ficou órfã (e.g. suspend/resume corrompeu
        // STAT_TX — ver nota no WKUP ISR), g_ep1_tx_busy fica true para sempre e o
        // payload morre no PMA até o próximo OUT por acaso destravar. Após ~10 polls
        // consecutivos com busy, re-arma o MESMO payload (count já está no BDTable;
        // só STAT_TX=VALID de novo) — inócuo se a transação estiver realmente em curso.
        static uint16_t tx_stuck_polls = 0u;
        if (g_ep1_tx_busy) {
            if (++tx_stuck_polls >= 10u) {
                tx_stuck_polls = 0u;
                ep_set_stat_tx(1u, 3u);  // re-VALID
            }
        } else {
            tx_stuck_polls = 0u;
        }
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

uint8_t usb_cdc_dbg_stage() noexcept {
    return g_usb_dbg_stage;
}

}  // namespace ems::hal
