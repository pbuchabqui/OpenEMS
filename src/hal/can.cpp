/**
 * @file hal/stm32h562/can.cpp
 * @brief FDCAN1 — CAN 2.0B Classic a 500 kbps para STM32H562RGT6
 *        Substitui hal/can.cpp da versão STM32.
 *
 * Configuração de bit timing para 500 kbps com clock FDCAN = 62.5 MHz:
 *   Prescaler (NBRP+1) = 5 → Tq clock = 62.5 MHz / 5 = 12.5 MHz
 *   Tq = 80 ns
 *   NTSEG1 = 10 → 11 Tq (propagation + phase seg 1)
 *   NTSEG2 = 3  → 4 Tq  (phase seg 2)
 *   NSJW   = 3  → 4 Tq  (resync jump width)
 *   Bit time = 1 + 11 + 4 = 16 Tq → 16 × 80 ns = 1280 ns → 781 kbps
 *
 *   Ajuste para 500 kbps exato:
 *   NBRP+1 = 8 → Tq = 62.5 MHz / 8 = 7.8125 MHz → 128 ns/Tq
 *   NTSEG1 = 12 (13 Tq) + NTSEG2 = 3 (4 Tq) + 1 = 18 Tq
 *   500 kbps: 1/(18 × 128 ns) = 434 kbps — próximo mas não exato
 *
 *   Opção final para 500 kbps exato com 62.5 MHz:
 *   NBRP+1 = 5, NTSEG1 = 14, NTSEG2 = 5, Total = 20 Tq
 *   Tq = 80 ns, bit time = 20 × 80 ns = 1600 ns → 625 kbps
 *
 *   Para 500 kbps exato: NBRP+1 = 5, total Tq = 25
 *   NTSEG1 = 18, NTSEG2 = 6: 25 × 80 ns = 2000 ns → 500 kbps ✓
 *
 * Pinos: PB8 (FDCAN1_RX), PB9 (FDCAN1_TX) — AF9
 * (PA11/PA12 reservados para USB CDC; ignição usa TIM8 em PC6-PC9)
 *
 * Message RAM layout (FDCAN_SRAM @ 0x4000AC00):
 *   Offset 0x000: Std ID filters  (2 × 4 bytes)
 *   Offset 0x008: RX FIFO0        (3 elementos × 16 bytes)
 *   Offset 0x038: TX Buffer       (2 elementos × 16 bytes)
 */

#ifndef EMS_HOST_TEST

#include "hal/can.h"
#include "hal/regs.h"

// ── Message RAM layout ────────────────────────────────────────────────────────
// Endereços offset dentro do FDCAN_SRAM (0x4000AC00)
static constexpr uint32_t kSramStdFilters = 0x000u;  // 2 filtros × 4 bytes
static constexpr uint32_t kSramRxFifo0    = 0x008u;  // 3 elem × 16 bytes
static constexpr uint32_t kSramTxBuf      = 0x038u;  // 2 elem × 16 bytes

// Para 8-byte CAN classic: 2 words header + 2 words data = 4 words = 16 bytes
static constexpr uint32_t kElemSizeWords = 4u;  // 4 × 32-bit words por elemento

// Inline: acesso a palavra na Message RAM
static inline volatile uint32_t& sram_word(uint32_t offset_bytes) noexcept {
    return *reinterpret_cast<volatile uint32_t*>(FDCAN_SRAM + offset_bytes);
}

namespace ems::hal {

// ── RX FIFO circular (software) ──────────────────────────────────────────────
// Poll-based: can0_rx_pop() é chamado do loop principal (não IRQ-driven)
static CanFrame g_rx_fifo[4] = {};
static volatile uint8_t g_rx_head = 0u;
static volatile uint8_t g_rx_tail = 0u;
static constexpr uint8_t kRxFifoSize = 4u;

static inline bool rx_fifo_push(const CanFrame& f) noexcept {
    const uint8_t next = static_cast<uint8_t>((g_rx_tail + 1u) % kRxFifoSize);
    if (next == g_rx_head) { return false; }  // full
    g_rx_fifo[g_rx_tail] = f;
    g_rx_tail = next;
    return true;
}

static inline bool rx_fifo_pop(CanFrame& out) noexcept {
    if (g_rx_head == g_rx_tail) { return false; }  // empty
    out = g_rx_fifo[g_rx_head];
    g_rx_head = static_cast<uint8_t>((g_rx_head + 1u) % kRxFifoSize);
    return true;
}

static uint32_t g_can_init_faults = 0u;

// ── Inicialização ─────────────────────────────────────────────────────────────

void can0_init() noexcept {
    // ── 1. Habilitar clock FDCAN1 ────────────────────────────────────────
    RCC_APB1LENR |= RCC_APB1LENR_FDCAN1EN;

    // ── 2. Configurar pinos PB8 (RX) e PB9 (TX) — AF9 ───────────────────
    // PA11/PA12 ficam reservados para USB CDC; FDCAN1 em PB8/PB9
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 8u, GPIO_AF9);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 9u, GPIO_AF9);

    // ── 3. Entrar em modo de inicialização ───────────────────────────────
    FDCAN1_CCCR |= FDCAN_CCCR_INIT;
    {
        constexpr uint32_t kTimeout = 30000u;
        for (uint32_t n = kTimeout; n > 0u; --n) {
            if (FDCAN1_CCCR & FDCAN_CCCR_INIT) { break; }
        }
        if ((FDCAN1_CCCR & FDCAN_CCCR_INIT) == 0u) { ++g_can_init_faults; }
    }
    FDCAN1_CCCR |= FDCAN_CCCR_CCE;   // habilita configuração

    // ── 4. Bit timing 500 kbps a 62.5 MHz ────────────────────────────────
    // NBRP+1=5, Tq=80ns, NTSEG1=17 (18 Tq), NTSEG2=5 (6 Tq) → 25 Tq → 500 kbps
    // Bit time = 1(sync) + 18(TSEG1) + 6(TSEG2) = 25 Tq × 80 ns = 2000 ns → 500 kbps ✓
    // Sample point = (1+18)/25 = 76% (dentro do range CAN spec 75–87.5%)
    // FDCAN_NBTP: NSJW[6:0] @ [31:25], NBRP[8:0] @ [24:16], NTSEG1[7:0] @ [15:8], NTSEG2[6:0] @ [6:0]
    FDCAN1_NBTP = ((4u  & 0x7Fu) << 25)   // NSJW = 4 (SJW = 4 Tq)
               | ((4u  & 0x1FFu) << 16)   // NBRP = 4 (prescaler = NBRP+1 = 5)
               | ((17u & 0xFFu)  << 8)    // NTSEG1 = 17 (TSEG1 = 18 Tq) → 500 kbps
               | ((5u  & 0x7Fu)  << 0);   // NTSEG2 = 5 (Phase_Seg2 = 6 Tq)

    // ── 5. Configurar Message RAM ─────────────────────────────────────────
    // Limpar área de Message RAM
    for (uint32_t i = 0u; i < 128u; ++i) {
        sram_word(i * 4u) = 0u;
    }

    // Std ID Filter: aceitar 0x180 (WBO2 RX) — passar para RX FIFO0
    // Elemento de filtro padrão: [31:30]=SFT, [29:27]=SFEC, [26:16]=SFID1, [10:0]=SFID2
    // Tipo 010 = Classic filter, SFID1=ID, SFID2=ID (exact match)
    sram_word(kSramStdFilters + 0u) =
        (0x2u << 30)       // SFT = 010 (classic filter)
      | (0x1u << 27)       // SFEC = 001 (store in RX FIFO0 on match)
      | (0x180u << 16)     // SFID1 = 0x180
      | (0x180u << 0);     // SFID2 = 0x180

    // Global filter: 2 filtros standard; frames não casados continuam aceitos em FIFO0.
    FDCAN1_RXGFC = (2u << 16);  // LSS = 2, LSE = 0, ANFS/ANFE = accept FIFO0

    // RX FIFO0: 3 elementos de 16 bytes cada
    FDCAN1_RXF0C = (kSramRxFifo0 / 4u)   // F0SA: start address em words
                 | (3u << 16);             // F0S: 3 elementos

    // TX Buffer: 2 elementos de 16 bytes cada
    FDCAN1_TXBC = (kSramTxBuf / 4u)      // TBSA: start address em words
               | (2u << 16);              // NDTB: 2 Dedicated TX Buffers

    // ── 6. Sair do modo de inicialização ─────────────────────────────────
    FDCAN1_CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1_CCCR &= ~FDCAN_CCCR_INIT;
    // Aguardar sincronização no barramento
    {
        constexpr uint32_t kTimeout = 30000u;
        for (uint32_t n = kTimeout; n > 0u; --n) {
            if ((FDCAN1_CCCR & FDCAN_CCCR_INIT) == 0u) { break; }
        }
        if (FDCAN1_CCCR & FDCAN_CCCR_INIT) { ++g_can_init_faults; }
    }
}

uint32_t can0_get_init_faults() noexcept {
    return g_can_init_faults;
}

// ── Transmissão ───────────────────────────────────────────────────────────────

bool can0_tx(const CanFrame& frame) noexcept {
    // Buffer dedicado 0: não sobrescrever enquanto a requisição estiver pendente.
    if (FDCAN1_TXBRP & (1u << 0)) {
        return false;
    }

    // Endereço do elemento TX Buffer 0 na Message RAM
    const uint32_t tx_addr = kSramTxBuf;

    // Word 0: ID + flags
    // Bit 30 = XTD (0=std 11-bit), Bit 29 = RTR, Bits [28:18] = ID[10:0]
    const uint32_t id_shifted = (frame.id & 0x7FFu) << 18;
    sram_word(tx_addr + 0u) = id_shifted;

    // Word 1: DLC + BRS/FDF bits (CAN classic: BRS=0, FDF=0)
    const uint8_t dlc = (frame.dlc > 8u) ? 8u : frame.dlc;
    sram_word(tx_addr + 4u) = (static_cast<uint32_t>(dlc) << 16);

    // Words 2-3: data (8 bytes = 2 words)
    uint32_t d0 = 0u, d1 = 0u;
    if (dlc >= 1u) { d0 |= static_cast<uint32_t>(frame.data[0]) << 0; }
    if (dlc >= 2u) { d0 |= static_cast<uint32_t>(frame.data[1]) << 8; }
    if (dlc >= 3u) { d0 |= static_cast<uint32_t>(frame.data[2]) << 16; }
    if (dlc >= 4u) { d0 |= static_cast<uint32_t>(frame.data[3]) << 24; }
    if (dlc >= 5u) { d1 |= static_cast<uint32_t>(frame.data[4]) << 0; }
    if (dlc >= 6u) { d1 |= static_cast<uint32_t>(frame.data[5]) << 8; }
    if (dlc >= 7u) { d1 |= static_cast<uint32_t>(frame.data[6]) << 16; }
    if (dlc >= 8u) { d1 |= static_cast<uint32_t>(frame.data[7]) << 24; }
    sram_word(tx_addr + 8u)  = d0;
    sram_word(tx_addr + 12u) = d1;

    // Solicitar transmissão do Buffer 0
    FDCAN1_TXBAR = (1u << 0);
    return true;
}

// ── Recepção ──────────────────────────────────────────────────────────────────

bool can0_rx_pop(CanFrame& out) noexcept {
    // 1. Verificar se há mensagem no RX FIFO0
    const uint32_t fqs = FDCAN1_RXF0S;
    if ((fqs & 0x7Fu) == 0u) {
        // Nenhum elemento disponível; drena o FIFO de software se houver
        return rx_fifo_pop(out);
    }

    // 2. Índice do próximo elemento a ler (hardware garante 0-2, mas valida defensivamente)
    const uint32_t get_idx_raw = (fqs >> 8) & 0x3Fu;
    const uint32_t get_idx = (get_idx_raw < 3u) ? get_idx_raw : 0u;
    const uint32_t elem_addr = kSramRxFifo0 + get_idx * (kElemSizeWords * 4u);

    // 3. Ler dados do elemento
    const uint32_t w0 = sram_word(elem_addr + 0u);
    const uint32_t w1 = sram_word(elem_addr + 4u);
    const uint32_t d0 = sram_word(elem_addr + 8u);
    const uint32_t d1 = sram_word(elem_addr + 12u);

    // 4. Decodificar
    out.id = (w0 >> 18) & 0x7FFu;  // ID 11-bit
    out.extended = false;
    out.dlc = static_cast<uint8_t>((w1 >> 16) & 0xFu);
    const uint8_t dlc = (out.dlc > 8u) ? 8u : out.dlc;

    out.data[0] = static_cast<uint8_t>(d0 >> 0);
    out.data[1] = static_cast<uint8_t>(d0 >> 8);
    out.data[2] = static_cast<uint8_t>(d0 >> 16);
    out.data[3] = static_cast<uint8_t>(d0 >> 24);
    out.data[4] = static_cast<uint8_t>(d1 >> 0);
    out.data[5] = static_cast<uint8_t>(d1 >> 8);
    out.data[6] = static_cast<uint8_t>(d1 >> 16);
    out.data[7] = static_cast<uint8_t>(d1 >> 24);
    (void)dlc;

    // 5. Liberar elemento no FIFO0 (Acknowledge)
    FDCAN1_RXF0A = get_idx;

    return true;
}

} // namespace ems::hal

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/can.h"
namespace ems::hal {
static CanFrame g_tx_buf[8];
static CanFrame g_rx_inject[8];
static uint8_t  g_tx_cnt = 0u, g_rx_cnt = 0u, g_rx_pop_idx = 0u;
static uint32_t g_test_ctrl1 = 0u;

void can0_init() noexcept {}
uint32_t can0_get_init_faults() noexcept { return 0u; }
bool can0_tx(const CanFrame& f) noexcept {
    if (g_tx_cnt < 8u) { g_tx_buf[g_tx_cnt++] = f; }
    return true;
}
bool can0_rx_pop(CanFrame& out) noexcept {
    if (g_rx_pop_idx >= g_rx_cnt) { return false; }
    out = g_rx_inject[g_rx_pop_idx++];
    return true;
}
void can_test_reset() noexcept {
    g_tx_cnt = g_rx_cnt = g_rx_pop_idx = 0u;
}
bool can_test_inject_rx(const CanFrame& f) noexcept {
    if (g_rx_cnt < 8u) { g_rx_inject[g_rx_cnt++] = f; return true; }
    return false;
}
bool can_test_pop_tx(CanFrame& out) noexcept {
    if (g_tx_cnt == 0u) { return false; }
    out = g_tx_buf[--g_tx_cnt]; return true;
}
uint32_t can_test_ctrl1() noexcept { return g_test_ctrl1; }
} // namespace ems::hal

#endif  // EMS_HOST_TEST
