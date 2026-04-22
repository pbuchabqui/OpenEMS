#pragma once

#include <cstdint>

namespace ems::hal {

// Timer initialization for STM32H562
// TIM1  = Injection/Ignition events (output compare mode)
// TIM5  = CKP input capture (edge detection)
// TIM6  = Background loop tick (if needed)

void tim1_init_pwm(uint16_t prescaler, uint16_t period);
void tim1_set_oc_pulse(uint8_t channel, uint16_t pulse_ticks);

void tim5_init_ic();
void tim5_enable_cc_interrupt();

uint16_t tim_get_counter();
void tim_reset_counter();

} // namespace ems::hal
