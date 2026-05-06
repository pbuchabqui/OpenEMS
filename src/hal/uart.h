#pragma once
/**
 * @file hal/uart.h
 * @brief UART interface para STM32H562
 *
 * Exporta:
 *   uart0_init()       — inicializa UART a 115200 baud
 *   uart0_poll_rx()    — polifica RX (non-blocking)
 *   uart0_tx_ready()   — verifica se TX buffer está vazio
 *   uart0_tx_byte()    — transmite byte pela UART
 *
 * Uso: comunicacao da UI proprietaria
 */

#include <cstdint>

namespace ems::hal {

void uart0_init(uint32_t baud = 115200u) noexcept;
void uart0_poll_rx(uint16_t max_bytes = 64u) noexcept;
bool uart0_tx_ready() noexcept;
bool uart0_tx_byte(uint8_t byte) noexcept;

bool uart0_rx_available() noexcept;
bool uart0_rx_pop(uint8_t& byte) noexcept;
bool uart0_tx_push(uint8_t byte) noexcept;
void uart0_tx_poll(uint16_t max_bytes = 32u) noexcept;

}  // namespace ems::hal
