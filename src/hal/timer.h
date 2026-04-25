#pragma once

#include <cstdint>

namespace ems::hal {

void tim5_ic_init(void);
uint16_t tim5_count() noexcept;

void tim3_pwm_init(uint32_t freq_hz);
void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

void tim4_pwm_init(uint32_t freq_hz);
void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

} // namespace ems::hal
