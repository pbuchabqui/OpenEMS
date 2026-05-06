/**
 * @file hal/stm32h562/uart.cpp
 * @brief USART1 @ 115200 baud para STM32H562RGT6
 *        Substitui hal/uart.cpp da versão STM32 (Serial STM32 runtime).
 *
 * Pinos: PA9 (USART1_TX), PA10 (USART1_RX) — AF7
 * (PA2/PA3 reservados para ADC1_IN3/IN4 — sensores MAP e MAF)
 * (PB10/PB11 ficam livres para TIM2_CH3/CH4 — INJ3/INJ4)
 *
 * Clock USART1: APB2 = 125 MHz (HCLK/2)
 * BRR = APB2_CLK / baud = 125,000,000 / 115,200 = 1085
 *   Erro: 125,000,000 / 1085 = 115,207 bps → erro < 0.01%
 *
 * RX Buffer circular (software) para recepção sem bloqueio.
 * TX: polling com fila curta para manter a UI fora do caminho crítico.
 */

#ifndef EMS_HOST_TEST

#include "hal/uart.h"
#include "hal/regs.h"

// ── Clock USART para cálculo BRR ─────────────────────────────────────────────
static constexpr uint32_t kUartClockHz = 125000000u;  // APB2 = 125 MHz

// ── Buffer RX circular ───────────────────────────────────────────────────────
static constexpr uint8_t  kRxBufSize = 128u;
static uint8_t  g_rx_buf[kRxBufSize] = {};
static volatile uint8_t g_rx_head = 0u;  // read pointer (consumer)
static volatile uint8_t g_rx_tail = 0u;  // write pointer (producer)

// ── Buffer TX circular ───────────────────────────────────────────────────────
static constexpr uint16_t kTxBufSize = 256u;
static constexpr uint16_t kTxBufMask = kTxBufSize - 1u;
static uint8_t  g_tx_buf[kTxBufSize] = {};
static uint16_t g_tx_head = 0u;  // write pointer
static uint16_t g_tx_tail = 0u;  // read pointer

namespace ems::hal {

void uart0_init(uint32_t baud) noexcept {
    // ── 1. Habilitar clock USART1 (APB2) ────────────────────────────────
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;

    // ── 2. Configurar pinos PA9 (TX) e PA10 (RX) — AF7 ─────────────────
    // PB10/PB11 pertencem ao TIM2_CH3/CH4 no MVP de bancada.
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 9u, GPIO_AF7);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 10u, GPIO_AF7);

    // ── 3. Configurar USART1 ─────────────────────────────────────────────
    USART1_CR1 = 0u;   // desabilita durante config

    // BRR: divisão direta (OVER8=0 → 16× oversampling, padrão)
    USART1_BRR = kUartClockHz / baud;

    USART1_CR2 = 0u;   // 1 stop bit, sem LIN, sem clock síncrono
    USART1_CR3 = 0u;   // sem hardware flow control

    // Habilitar RX, TX e periférico
    USART1_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

void uart0_poll_rx(uint16_t max_bytes) noexcept {
    // Drena os bytes disponíveis no USART1 para o buffer circular
    for (uint16_t i = 0u; i < max_bytes; ++i) {
        if ((USART1_ISR & USART_ISR_RXNE) == 0u) { break; }
        const uint8_t byte = static_cast<uint8_t>(USART1_RDR & 0xFFu);
        const uint8_t next = static_cast<uint8_t>((g_rx_tail + 1u) % kRxBufSize);
        if (next != g_rx_head) {  // não sobrescreve se buffer cheio
            g_rx_buf[g_rx_tail] = byte;
            g_rx_tail = next;
        }
    }
}

bool uart0_tx_ready() noexcept {
    return (USART1_ISR & USART_ISR_TXE) != 0u;
}

bool uart0_tx_byte(uint8_t byte) noexcept {
    // Aguarda TXE (transmit register empty) — bloqueante com timeout
    constexpr uint32_t kTimeout = 100000u;
    for (uint32_t i = 0u; i < kTimeout; ++i) {
        if (USART1_ISR & USART_ISR_TXE) {
            USART1_TDR = static_cast<uint32_t>(byte);
            return true;
        }
    }
    return false;  // timeout
}

bool uart0_rx_available() noexcept {
    return g_rx_head != g_rx_tail;
}

bool uart0_rx_pop(uint8_t& byte) noexcept {
    if (g_rx_head == g_rx_tail) { return false; }
    byte = g_rx_buf[g_rx_head];
    g_rx_head = static_cast<uint8_t>((g_rx_head + 1u) % kRxBufSize);
    return true;
}

bool uart0_tx_push(uint8_t byte) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_tx_head + 1u) & kTxBufMask);
    if (next == g_tx_tail) { return false; }
    g_tx_buf[g_tx_head] = byte;
    g_tx_head = next;
    return true;
}

void uart0_tx_poll(uint16_t max_bytes) noexcept {
    for (uint16_t i = 0u; i < max_bytes && g_tx_head != g_tx_tail; ++i) {
        uint32_t timeout = 1000u;
        while (!(USART1_ISR & USART_ISR_TXE) && timeout-- != 0u) {}
        if (!(USART1_ISR & USART_ISR_TXE)) { break; }
        USART1_TDR = static_cast<uint32_t>(g_tx_buf[g_tx_tail]);
        g_tx_tail = static_cast<uint16_t>((g_tx_tail + 1u) & kTxBufMask);
    }
}

} // namespace ems::hal

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/uart.h"
#include <cstring>
namespace ems::hal {
static uint8_t g_tx_buf[256] = {};
static uint8_t g_rx_buf_test[256] = {};
static uint16_t g_tx_len = 0u, g_rx_len = 0u, g_rx_pos = 0u;
static uint32_t g_baud = 0u;

void uart0_init(uint32_t baud) noexcept { g_baud = baud; }
void uart0_poll_rx(uint16_t) noexcept { }
bool uart0_tx_ready() noexcept { return true; }
bool uart0_tx_byte(uint8_t b) noexcept {
    if (g_tx_len < 256u) { g_tx_buf[g_tx_len++] = b; }
    return true;
}
bool uart0_rx_available() noexcept { return g_rx_pos < g_rx_len; }
bool uart0_rx_pop(uint8_t& byte) noexcept {
    if (g_rx_pos >= g_rx_len) { return false; }
    byte = g_rx_buf_test[g_rx_pos++];
    return true;
}
bool uart0_tx_push(uint8_t b) noexcept {
    if (g_tx_len < 256u) { g_tx_buf[g_tx_len++] = b; return true; }
    return false;
}
void uart0_tx_poll(uint16_t) noexcept {}
// Test helpers (usados em test/hal/test_uart)
void uart_test_reset() noexcept { g_tx_len = g_rx_len = g_rx_pos = 0u; }
void uart_test_inject_rx(const uint8_t* data, uint16_t len) noexcept {
    uint16_t n = (len < 256u) ? len : 256u;
    std::memcpy(g_rx_buf_test, data, n);
    g_rx_len = n; g_rx_pos = 0u;
}
uint16_t uart_test_tx_len() noexcept { return g_tx_len; }
const uint8_t* uart_test_tx_buf() noexcept { return g_tx_buf; }
} // namespace ems::hal

#endif  // EMS_HOST_TEST
