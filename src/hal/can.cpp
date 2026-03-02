#include "hal/can.h"

#include <cstdint>

namespace {

#if defined(EMS_HOST_TEST)
constexpr uint8_t kQueueSize = 16u;
static ems::hal::CanFrame g_rx_q[kQueueSize] = {};
static ems::hal::CanFrame g_tx_q[kQueueSize] = {};
static uint8_t g_rx_head = 0u;
static uint8_t g_rx_tail = 0u;
static uint8_t g_tx_head = 0u;
static uint8_t g_tx_tail = 0u;
static uint32_t g_ctrl1 = 0u;
#else
#define SIM_SCGC5 (*reinterpret_cast<volatile uint32_t*>(0x40048038u))
#define SIM_SCGC6 (*reinterpret_cast<volatile uint32_t*>(0x4004803Cu))
#define SIM_SCGC6_FLEXCAN0_MASK (1u << 4)
#define SIM_SCGC5_PORTA_MASK (1u << 9)

#define PORTA_PCR12 (*reinterpret_cast<volatile uint32_t*>(0x40049030u))
#define PORTA_PCR13 (*reinterpret_cast<volatile uint32_t*>(0x40049034u))
#define PCR_MUX_ALT2 (2u << 8)

#define CAN0_MCR (*reinterpret_cast<volatile uint32_t*>(0x40024000u))
#define CAN0_CTRL1 (*reinterpret_cast<volatile uint32_t*>(0x40024004u))
#define CAN0_TIMER (*reinterpret_cast<volatile uint32_t*>(0x40024008u))
#define CAN0_RXMGMASK (*reinterpret_cast<volatile uint32_t*>(0x40024010u))
#define CAN0_IFLAG1 (*reinterpret_cast<volatile uint32_t*>(0x40024030u))

#define CAN_MB_CS(base, n) (*reinterpret_cast<volatile uint32_t*>((base) + 0x80u + (n) * 0x10u + 0x0u))
#define CAN_MB_ID(base, n) (*reinterpret_cast<volatile uint32_t*>((base) + 0x80u + (n) * 0x10u + 0x4u))
#define CAN_MB_WORD0(base, n) (*reinterpret_cast<volatile uint32_t*>((base) + 0x80u + (n) * 0x10u + 0x8u))
#define CAN_MB_WORD1(base, n) (*reinterpret_cast<volatile uint32_t*>((base) + 0x80u + (n) * 0x10u + 0xCu))
#define CAN0_BASE 0x40024000u

#define CAN_MCR_MDIS (1u << 31)
#define CAN_MCR_FRZ (1u << 30)
#define CAN_MCR_HALT (1u << 28)
#define CAN_MCR_SOFTRST (1u << 25)
#define CAN_MCR_FRZACK (1u << 24)
#define CAN_MCR_SRXDIS (1u << 17)
#define CAN_MCR_MAXMB(x) ((x) & 0x7Fu)

#define CAN_CTRL1_CLKSRC (1u << 13)
#define CAN_CTRL1_PRESDIV(x) (((x) & 0xFFu) << 24)
#define CAN_CTRL1_PROPSEG(x) (((x) & 0x07u) << 0)
#define CAN_CTRL1_PSEG1(x) (((x) & 0x07u) << 19)
#define CAN_CTRL1_PSEG2(x) (((x) & 0x07u) << 16)
#define CAN_CTRL1_RJW(x) (((x) & 0x03u) << 22)

#define CAN_CS_CODE_TX_INACTIVE (0x8u << 24)
#define CAN_CS_CODE_TX_DATA (0xCu << 24)
#define CAN_CS_CODE_RX_EMPTY (0x4u << 24)
#define CAN_CS_CODE_RX_FULL (0x2u << 24)
#define CAN_CS_SRR (1u << 22)
#define CAN_CS_IDE (1u << 21)
#define CAN_CS_DLC(x) (((x) & 0xFu) << 16)

constexpr uint8_t kTxMb = 8u;
constexpr uint8_t kRxMb = 9u;

inline uint8_t clamp_dlc(uint8_t dlc) noexcept {
    return (dlc > 8u) ? 8u : dlc;
}
#endif

inline bool q_push(ems::hal::CanFrame* q, uint8_t& head, uint8_t tail, const ems::hal::CanFrame& frame) noexcept {
    const uint8_t next = static_cast<uint8_t>((head + 1u) & 0x0Fu);
    if (next == tail) {
        return false;
    }
    q[head] = frame;
    head = next;
    return true;
}

inline bool q_pop(const ems::hal::CanFrame* q, uint8_t& tail, uint8_t head, ems::hal::CanFrame& out) noexcept {
    if (tail == head) {
        return false;
    }
    out = q[tail];
    tail = static_cast<uint8_t>((tail + 1u) & 0x0Fu);
    return true;
}

}  // namespace

namespace ems::hal {

void can0_init() noexcept {
#if defined(EMS_HOST_TEST)
    can_test_reset();
    g_ctrl1 = 0u;
    g_ctrl1 |= (5u << 24);
    g_ctrl1 |= (5u << 0);
    g_ctrl1 |= (7u << 19);
    g_ctrl1 |= (4u << 16);
#else
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;

    PORTA_PCR12 = PCR_MUX_ALT2;
    PORTA_PCR13 = PCR_MUX_ALT2;

    CAN0_MCR |= CAN_MCR_MDIS;
    CAN0_MCR = 0u;
    CAN0_MCR = CAN_MCR_FRZ | CAN_MCR_HALT | CAN_MCR_SRXDIS | CAN_MCR_MAXMB(15u);
    while ((CAN0_MCR & CAN_MCR_FRZACK) == 0u) {
    }

    CAN0_MCR |= CAN_MCR_SOFTRST;
    while ((CAN0_MCR & CAN_MCR_SOFTRST) != 0u) {
    }
    while ((CAN0_MCR & CAN_MCR_FRZACK) == 0u) {
    }

    // 500 kbps: F_bus=60MHz, PRESDIV+1=6 → Tq=100ns, TotalTq=1+6+8+5=20 → 2µs/bit
    CAN0_CTRL1 = CAN_CTRL1_CLKSRC | CAN_CTRL1_PRESDIV(5u) | CAN_CTRL1_PROPSEG(5u) |
                 CAN_CTRL1_PSEG1(7u) | CAN_CTRL1_PSEG2(4u) | CAN_CTRL1_RJW(3u);

    CAN0_RXMGMASK = 0x1FFFFFFFu;

    for (uint8_t mb = 0u; mb < 16u; ++mb) {
        CAN_MB_CS(CAN0_BASE, mb) = 0u;
        CAN_MB_ID(CAN0_BASE, mb) = 0u;
        CAN_MB_WORD0(CAN0_BASE, mb) = 0u;
        CAN_MB_WORD1(CAN0_BASE, mb) = 0u;
    }

    CAN_MB_CS(CAN0_BASE, kTxMb) = CAN_CS_CODE_TX_INACTIVE;
    CAN_MB_CS(CAN0_BASE, kRxMb) = CAN_CS_CODE_RX_EMPTY;

    CAN0_IFLAG1 = 0xFFFFFFFFu;
    CAN0_MCR &= ~CAN_MCR_HALT;
    while ((CAN0_MCR & CAN_MCR_FRZACK) != 0u) {
    }
#endif
}

bool can0_tx(const CanFrame& frame) noexcept {
#if defined(EMS_HOST_TEST)
    return q_push(g_tx_q, g_tx_head, g_tx_tail, frame);
#else
    const uint8_t dlc = clamp_dlc(frame.dlc);
    if ((CAN0_IFLAG1 & (1u << kTxMb)) == 0u && (CAN_MB_CS(CAN0_BASE, kTxMb) & (0xFu << 24)) != CAN_CS_CODE_TX_INACTIVE) {
        return false;
    }

    CAN_MB_CS(CAN0_BASE, kTxMb) = CAN_CS_CODE_TX_INACTIVE;
    CAN_MB_ID(CAN0_BASE, kTxMb) = (frame.extended ? (frame.id & 0x1FFFFFFFu) : ((frame.id & 0x7FFu) << 18u));

    const uint32_t w0 = (static_cast<uint32_t>(frame.data[0]) << 24u) |
                        (static_cast<uint32_t>(frame.data[1]) << 16u) |
                        (static_cast<uint32_t>(frame.data[2]) << 8u) |
                        (static_cast<uint32_t>(frame.data[3]));
    const uint32_t w1 = (static_cast<uint32_t>(frame.data[4]) << 24u) |
                        (static_cast<uint32_t>(frame.data[5]) << 16u) |
                        (static_cast<uint32_t>(frame.data[6]) << 8u) |
                        (static_cast<uint32_t>(frame.data[7]));
    CAN_MB_WORD0(CAN0_BASE, kTxMb) = w0;
    CAN_MB_WORD1(CAN0_BASE, kTxMb) = w1;

    uint32_t cs = CAN_CS_CODE_TX_DATA | CAN_CS_DLC(dlc);
    if (frame.extended) {
        cs |= (CAN_CS_IDE | CAN_CS_SRR);
    }
    CAN_MB_CS(CAN0_BASE, kTxMb) = cs;
    return true;
#endif
}

bool can0_rx_pop(CanFrame& out) noexcept {
#if defined(EMS_HOST_TEST)
    return q_pop(g_rx_q, g_rx_tail, g_rx_head, out);
#else
    if ((CAN_MB_CS(CAN0_BASE, kRxMb) & (0xFu << 24)) != CAN_CS_CODE_RX_FULL) {
        return false;
    }

    const uint32_t cs = CAN_MB_CS(CAN0_BASE, kRxMb);
    const uint8_t dlc = static_cast<uint8_t>((cs >> 16u) & 0xFu);
    out.extended = ((cs & CAN_CS_IDE) != 0u);
    out.id = out.extended ? (CAN_MB_ID(CAN0_BASE, kRxMb) & 0x1FFFFFFFu)
                          : ((CAN_MB_ID(CAN0_BASE, kRxMb) >> 18u) & 0x7FFu);
    out.dlc = clamp_dlc(dlc);

    const uint32_t w0 = CAN_MB_WORD0(CAN0_BASE, kRxMb);
    const uint32_t w1 = CAN_MB_WORD1(CAN0_BASE, kRxMb);
    out.data[0] = static_cast<uint8_t>((w0 >> 24u) & 0xFFu);
    out.data[1] = static_cast<uint8_t>((w0 >> 16u) & 0xFFu);
    out.data[2] = static_cast<uint8_t>((w0 >> 8u) & 0xFFu);
    out.data[3] = static_cast<uint8_t>(w0 & 0xFFu);
    out.data[4] = static_cast<uint8_t>((w1 >> 24u) & 0xFFu);
    out.data[5] = static_cast<uint8_t>((w1 >> 16u) & 0xFFu);
    out.data[6] = static_cast<uint8_t>((w1 >> 8u) & 0xFFu);
    out.data[7] = static_cast<uint8_t>(w1 & 0xFFu);

    CAN0_IFLAG1 = (1u << kRxMb);
    static_cast<void>(CAN0_TIMER);
    CAN_MB_CS(CAN0_BASE, kRxMb) = CAN_CS_CODE_RX_EMPTY;
    return true;
#endif
}

#if defined(EMS_HOST_TEST)
void can_test_reset() noexcept {
    g_rx_head = 0u;
    g_rx_tail = 0u;
    g_tx_head = 0u;
    g_tx_tail = 0u;
    g_ctrl1 = 0u;
}

bool can_test_inject_rx(const CanFrame& frame) noexcept {
    return q_push(g_rx_q, g_rx_head, g_rx_tail, frame);
}

bool can_test_pop_tx(CanFrame& out) noexcept {
    return q_pop(g_tx_q, g_tx_tail, g_tx_head, out);
}

uint32_t can_test_ctrl1() noexcept {
    return g_ctrl1;
}
#endif

}  // namespace ems::hal

