#pragma once
/**
 * @file hal/stm32h562/system.h
 * @brief Clock, SysTick e Watchdog para STM32H562RGT6
 *
 * Exporta:
 *   system_stm32_init()  — configura PLL → 250 MHz, SysTick 1 ms, IWDG 100 ms
 *   millis()             — equivalente Teensyduino (contagem SysTick em ms)
 *   micros()             — contagem µs via SysTick
 *   iwdg_kick()          — equivalente ao pit1_kick() do Kinetis
 */

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/** Inicializa RCC (PLL 250 MHz), Flash latency, SysTick e IWDG. */
void system_stm32_init(void) noexcept;

/** Kick (reload) do IWDG — deve ser chamado a cada iteração do loop principal. */
void iwdg_kick(void) noexcept;

/** Milissegundos desde o boot (incrementado pelo SysTick_Handler). */
uint32_t millis(void) noexcept;

/** Microssegundos desde o boot (resolução 1 µs via SysTick reload). */
uint32_t micros(void) noexcept;

#ifdef __cplusplus
}
#endif
