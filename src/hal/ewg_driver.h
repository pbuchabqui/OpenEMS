#pragma once
#include <cstdint>

namespace ems::hal {

bool ewg_driver_init() noexcept;
void ewg_driver_set_motor_pwm(int16_t pwm) noexcept;
uint16_t ewg_driver_read_position_raw() noexcept;
void ewg_driver_shutdown() noexcept;

#ifndef TARGET_STM32H562
// Ganchos do stub de host (src/hal/ewg_driver.cpp).
void ewg_driver_test_set_raw(uint16_t raw) noexcept;
int16_t ewg_driver_test_last_pwm() noexcept;
uint32_t ewg_driver_test_shutdown_count() noexcept;
#endif

}  // namespace ems::hal
