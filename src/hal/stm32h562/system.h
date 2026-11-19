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
uint32_t host_icache_monitor_resets(void) noexcept;
#endif

/** Microssegundos desde o boot (resolução 1 µs via SysTick reload). */
uint32_t micros(void) noexcept;

/**
 * ICACHE (8 KB, RM0481 §8): o código roda da Flash a 5 wait states, então sem cache o tempo da ISR varia
 * com o estado do prefetch. Chamado uma vez em system_stm32_init(). Compilar com -DEMS_ICACHE=0 desliga
 * (comparação antes/depois na bancada). Habilita também os monitores de hit/miss.
 */
void icache_enable(void) noexcept;

/**
 * Com o ICACHE LIGADO, uma escrita cacheável na região é tratada como ERRO (RM0481 §8.4.5): erase/program
 * de Flash exigem o cache desligado. icache_suspend() desliga (o EN 1→0 invalida o cache inteiro, então
 * nada fica obsoleto) e devolve se estava ligado; icache_resume(was_enabled) religa só nesse caso, depois
 * de esperar o fim da invalidação. Usados por flash_unlock_bank2()/flash_lock_bank2().
 */
bool icache_suspend(void) noexcept;
void icache_resume(bool was_enabled) noexcept;

/** Contadores dos monitores (diagnóstico de bancada; zero com EMS_ICACHE=0). */
uint32_t icache_hits(void) noexcept;
uint32_t icache_misses(void) noexcept;
/** Zera os dois monitores (eles saturam, não dão a volta). Usado pelo comando serial 'l'. */
void icache_monitors_reset(void) noexcept;
/** true se o ICACHE está ligado agora (falso durante uma gravação de Flash ou com EMS_ICACHE=0). */
bool icache_is_enabled(void) noexcept;

#ifdef __cplusplus
}
#endif
