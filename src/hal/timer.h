#pragma once

#include <cstdint>

namespace ems::hal {

void tim5_ic_init(void);
uint32_t tim5_count() noexcept;

// ATENÇÃO: TIM3 é DEDICADO à injeção (OC, PC6-9, ARR=0xFFFF, ECU_Hardware_Init).
// NÃO reconfigurar o TIM3 como PWM — quebra o timing dos injetores. As funções
// tim3_pwm_init/tim3_set_duty abaixo ficaram sem uso após mover o EWG p/ TIM2.
void tim3_pwm_init(uint32_t freq_hz);
void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

// TIM2_CH3 (PB10) — PWM do motor EWG (movido do TIM3 p/ liberar a injeção).
void tim2_pwm_init(uint32_t freq_hz);
void tim2_set_duty(uint16_t duty_pct_x10) noexcept;

void tim4_pwm_init(uint32_t freq_hz);
void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

void tim15_etb_pwm_init(uint32_t freq_hz);
void tim15_etb_set_duty_x10(uint16_t duty_pct_x10) noexcept;

} // namespace ems::hal
