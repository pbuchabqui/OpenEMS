#pragma once

#include <cstdint>

namespace ems::hal {

void uart0_init(uint32_t baud = 115200u) noexcept;
void uart0_poll_rx(uint16_t max_bytes = 64u) noexcept;
bool uart0_tx_ready() noexcept;
bool uart0_tx_byte(uint8_t byte) noexcept;

}  // namespace ems::hal
