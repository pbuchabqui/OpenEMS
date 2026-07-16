#pragma once

#include <cstdint>

namespace ems::hal {

void tim5_ic_init(void);
uint32_t tim5_count() noexcept;

// TIM3_CH1 PA6 AF2 — general PWM (RGT6). Injeção/ignição usam GPIO BSRR.
void tim3_pwm_init(uint32_t freq_hz);
void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

// TIM2_CH3 (PB10) — PWM EWG; ⚠️ conflito com INJ3 no RGT6.
void tim2_pwm_init(uint32_t freq_hz);
void tim2_set_duty(uint16_t duty_pct_x10) noexcept;

void tim4_pwm_init(uint32_t freq_hz);
void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

// ETB motor PWM: VGT6=TIM15_CH1 PE5; RGT6=TIM3_CH1 PA6.
void etb_pwm_init(uint32_t freq_hz);
void etb_pwm_set_duty_x10(uint16_t duty_pct_x10) noexcept;

// Deprecated aliases (hygiene PR-13) — prefer etb_pwm_*.
inline void tim15_etb_pwm_init(uint32_t freq_hz) { etb_pwm_init(freq_hz); }
inline void tim15_etb_set_duty_x10(uint16_t duty_pct_x10) noexcept {
    etb_pwm_set_duty_x10(duty_pct_x10);
}

} // namespace ems::hal
