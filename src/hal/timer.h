#pragma once

#include <cstdint>

namespace ems::hal {

void tim5_ic_init(void);
uint32_t tim5_count() noexcept;

// TIM3_CH1 PA6 AF2 — ETB motor PWM (RGT6). API legacy tim3_* ainda exposta.
// Injeção/ignição usam GPIO BSRR (não TIM3 OC).
void tim3_pwm_init(uint32_t freq_hz);
void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

// TIM2_CH3 (PB10) — PWM EWG; ⚠️ conflito com INJ3 no RGT6.
void tim2_pwm_init(uint32_t freq_hz);
void tim2_set_duty(uint16_t duty_pct_x10) noexcept;

void tim4_pwm_init(uint32_t freq_hz);
void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

// ETB PWM (nome legacy "tim15_etb"): no RGT6 = TIM3_CH1 @ PA6.
void tim15_etb_pwm_init(uint32_t freq_hz);
void tim15_etb_set_duty_x10(uint16_t duty_pct_x10) noexcept;

} // namespace ems::hal
