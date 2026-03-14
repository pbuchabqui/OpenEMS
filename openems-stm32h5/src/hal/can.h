#pragma once

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

#if defined(EMS_HOST_TEST)
void can_test_reset() noexcept;
bool can_test_inject_rx(const CanFrame& frame) noexcept;
bool can_test_pop_tx(CanFrame& out) noexcept;
uint32_t can_test_ctrl1() noexcept;
#endif

}  // namespace ems::hal

