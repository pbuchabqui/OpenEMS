#pragma once
/**
 * @file hal/stm32h562/system.h
 * @brief Clock, SysTick e Watchdog para STM32H562RGT6
 *
 * Exporta:
 *   system_stm32_init()  — configura PLL → 250 MHz, SysTick 1 ms, IWDG 100 ms
 *   millis()             — equivalente STM32 runtime (contagem SysTick em ms)
 *   micros()             — contagem µs via SysTick
 *   iwdg_kick()          — iwdg_kick() alimenta o watchdog IWDG
 */

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/** Inicializa RCC (PLL 250 MHz), Flash latency, SysTick e IWDG. */
void system_stm32_init(void) noexcept;

/**
 * true se o SYSCLK está no PLL1 (250 MHz). false = o HSE/PLL não travou e o core ficou no HSI:
 * TIM5 e SysTick contam ~7,8x mais devagar do que o firmware assume (dwell e PW 7,8x mais
 * longos). Com false, o boot não habilita a captura CKP/CMP e o teste de saídas é recusado.
 */
bool system_clock_ok(void) noexcept;

/**
 * DBGMCU_IDCODE (leitura): DEV_ID = bits [11:0] (0x484 = H562/563/573), REV_ID = bits [31:16]
 * (Rev A 0x1000, Z 0x1001, X 0x1007 — RM0481, DBGMCU_IDCODE). Somente leitura.
 */
uint32_t chip_idcode(void) noexcept;

/**
 * true se este boot veio de power-on/brown-out (RCC_RSR.BORRSTF). LÊ E LIMPA as flags de reset (RMVF):
 * um reset posterior (p.ex. NMI por ECC -> SYSRESETREQ) aparece como software e não repete a operação de boot.
 * Chamar uma vez, no início do boot.
 */
bool boot_reset_was_power_on(void) noexcept;

/** Kick (reload) do IWDG — deve ser chamado a cada iteração do loop principal. */
void iwdg_kick(void) noexcept;

/** Milissegundos desde o boot (incrementado pelo SysTick_Handler). */
uint32_t millis(void) noexcept;
#if defined(EMS_HOST_TEST)
void host_set_millis(uint32_t ms) noexcept;
void host_advance_millis(uint32_t dt) noexcept;
void host_set_clock_ok(bool ok) noexcept;
#endif

/** Microssegundos desde o boot (resolução 1 µs via SysTick reload). */
uint32_t micros(void) noexcept;

#ifdef __cplusplus
}
#endif
