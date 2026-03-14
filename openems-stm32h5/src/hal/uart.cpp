/**
 * @file hal/stm32h562/uart.cpp
 * @brief USART3 @ 115200 baud para STM32H562RGT6
 *        Substitui hal/uart.cpp da versão Kinetis (Serial Teensyduino).
 *
 * Pinos: PB10 (USART3_TX), PB11 (USART3_RX) — AF7
 * (PA2/PA3 reservados para ADC1_IN3/IN4 — sensores MAP e MAF)
 * (USART1/PA9/PA10 liberados para TIM1_CH2/CH3 — ignição IGN2/IGN3)
 *
 * Clock USART3: APB1 = 125 MHz (HCLK/2)
 * BRR = APB1_CLK / baud = 125,000,000 / 115,200 = 1085
 *   Erro: 125,000,000 / 1085 = 115,207 bps → erro < 0.01%
 *
 * RX Buffer circular (software) para recepção sem bloqueio.
 * TX: polling (uart0_tx_byte bloqueia até TXE).
 */

#ifndef EMS_HOST_TEST

#include "hal/uart.h"
#include "hal/regs.h"

// ── Clock APB1 para cálculo BRR ───────────────────────────────────────────────
static constexpr uint32_t kApb1ClockHz = 125000000u;  // 125 MHz

// ── Buffer RX circular ───────────────────────────────────────────────────────
static constexpr uint8_t kRxBufSize = 128u;
static uint8_t  g_rx_buf[kRxBufSize] = {};
static volatile uint8_t g_rx_head = 0u;
static volatile uint8_t g_rx_tail = 0u;

namespace ems::hal {

void uart0_init(uint32_t baud) noexcept {
    // ── 1. Habilitar clock USART3 (APB1) ────────────────────────────────
    RCC_APB1LENR |= RCC_APB1LENR_USART3EN;

    // ── 2. Configurar pinos PB10 (TX) e PB11 (RX) — AF7 ─────────────────
    // PA2/PA3 são ADC1_IN3/IN4 (MAP/MAF); PB10/PB11 sem conflito
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 10u, GPIO_AF7);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 11u, GPIO_AF7);

    // ── 3. Configurar USART3 ─────────────────────────────────────────────
    USART3_CR1 = 0u;   // desabilita durante config

    // BRR: divisão direta (OVER8=0 → 16× oversampling, padrão)
    USART3_BRR = kApb1ClockHz / baud;

    USART3_CR2 = 0u;   // 1 stop bit, sem LIN, sem clock síncrono
    USART3_CR3 = 0u;   // sem hardware flow control

    // Habilitar RX, TX e periférico
    USART3_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

void uart0_poll_rx(uint16_t max_bytes) noexcept {
    // Drena os bytes disponíveis no USART3 para o buffer circular
    for (uint16_t i = 0u; i < max_bytes; ++i) {
        if ((USART3_ISR & USART_ISR_RXNE) == 0u) { break; }
        const uint8_t byte = static_cast<uint8_t>(USART3_RDR & 0xFFu);
        const uint8_t next = static_cast<uint8_t>((g_rx_tail + 1u) % kRxBufSize);
        if (next != g_rx_head) {  // não sobrescreve se buffer cheio
            g_rx_buf[g_rx_tail] = byte;
            g_rx_tail = next;
        }
    }
}

bool uart0_tx_ready() noexcept {
    return (USART3_ISR & USART_ISR_TXE) != 0u;
}

bool uart0_tx_byte(uint8_t byte) noexcept {
    // Aguarda TXE (transmit register empty) — bloqueante com timeout
    constexpr uint32_t kTimeout = 100000u;
    for (uint32_t i = 0u; i < kTimeout; ++i) {
        if (USART3_ISR & USART_ISR_TXE) {
            USART3_TDR = static_cast<uint32_t>(byte);
            return true;
        }
    }
    return false;  // timeout
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
