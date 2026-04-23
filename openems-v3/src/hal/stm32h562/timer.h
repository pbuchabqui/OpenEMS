#pragma once
/**
 * @file hal/stm32h562/timer.h
 * @brief Timer drivers para STM32H562
 *
 * Exporta:
 *   tim1_init_pwm()           — inicializa TIM1 para injeção/ignição (output compare)
 *   tim1_set_oc_pulse()       — programa pulse width no canal especificado
 *   tim5_init_ic()            — inicializa TIM5 para captura de arestas CKP
 *   tim5_enable_cc_interrupt() — ativa interrupção de captura
 *   tim_get_counter()         — retorna tick count do timer
 *   tim_reset_counter()       — reseta contador
 */

#include <cstdint>

namespace ems::hal {

void tim1_init_pwm(uint16_t prescaler, uint16_t period);
void tim1_set_oc_pulse(uint8_t channel, uint16_t pulse_ticks);

void tim5_init_ic();
void tim5_enable_cc_interrupt();

uint16_t tim_get_counter();
void tim_reset_counter();

} // namespace ems::hal
