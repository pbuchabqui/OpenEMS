#pragma once
/**
 * @file hal/can.h
 * @brief CAN FD interface para STM32H562
 *
 * Exporta:
 *   can0_init()        — inicializa FDCAN1 em 500 kbps
 *   can0_tx()          — transmite frame CAN
 *   can0_rx_pop()      — recebe frame CAN (FIFO)
 *
 * Protocolos: WBO2 lambda RX (0x180), diagnósticos TX (0x400, 0x401)
 */

#include <cstdint>

namespace ems::hal {

struct CanFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool extended;
};

void can0_init() noexcept;
bool can0_tx(const CanFrame& frame) noexcept;
bool can0_rx_pop(CanFrame& out) noexcept;
uint32_t can0_get_init_faults() noexcept;

#if defined(EMS_HOST_TEST)
// Ganchos do stub de host (src/hal/can.cpp).
void can0_test_reset() noexcept;
void can0_test_model_single_tx_buffer(bool on) noexcept;  // recusa TX até can0_test_tx_complete()
void can0_test_tx_complete() noexcept;
uint16_t can0_test_tx_count() noexcept;
const CanFrame& can0_test_tx_at(uint16_t i) noexcept;
void can0_test_push_rx(const CanFrame& f) noexcept;
#endif

}  // namespace ems::hal

