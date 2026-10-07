#pragma once

#include <cstdint>

namespace ems::hal {

void tim5_ic_init(void);
uint32_t tim5_count() noexcept;

// Polaridade de captura TIM5 + pull GPIOA (CKP=PA0 / CMP=PA1).
// falling=true → CC1P/CC2P e pull-up; false → subida e pull-down (default actual).
// Sequência: limpar CCxE → CCxP → repor CCxE (evitar captura espúria).
// Chamar após carregar page0 (tim5_ic_init corre antes da NVM).
void tim5_ic_set_capture_polarity(bool ckp_falling, bool cmp_falling) noexcept;

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

} // namespace ems::hal
