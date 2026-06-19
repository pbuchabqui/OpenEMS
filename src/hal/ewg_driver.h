#pragma once
#include <cstdint>

namespace ems::hal {

bool ewg_driver_init() noexcept;
void ewg_driver_set_motor_pwm(int16_t pwm) noexcept;
uint16_t ewg_driver_read_position_raw() noexcept;
void ewg_driver_shutdown() noexcept;

}  // namespace ems::hal
