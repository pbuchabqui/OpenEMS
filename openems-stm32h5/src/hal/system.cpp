/**
 * @file hal/stm32h562/system.cpp
 * @brief Clock (PLL → 250 MHz), SysTick (1 ms), IWDG (100 ms)
 *        para STM32H562RGT6 — substitui runtime Teensyduino.
 *
 * Configuração de clock:
 *   HSE  =  8 MHz (cristal externo)
 *   PLL1: M=1, N=62, P=2  →  (8 / 1) × 62 / 2 = 248 MHz  ≈ 250 MHz *
 *         M=1, N=125, P=4 →  (8 / 1) × 125/ 4 = 250 MHz  (exato)
 *   SYSCLK = 250 MHz
 *   HCLK   = 250 MHz  (AHB prescaler = 1)
 *   APB1   = 125 MHz  (APB1 prescaler = 2)
 *   APB2   = 125 MHz  (APB2 prescaler = 2)
 *   Timers APB2 (TIM1): 250 MHz  (timer doubler ativo quando PPRE2 ≠ 1)
 *   Timers APB1 (TIM3/4/5/6): 250 MHz
 *
 * IWDG a 32 kHz LSI:
 *   Prescaler /32 → 1000 Hz; Reload = 99 → timeout ≈ 100 ms
 */

#ifndef EMS_HOST_TEST

#include "hal/system.h"
#include "hal/regs.h"

// ── Variável global de contagem SysTick ─────────────────────────────────────
static volatile uint32_t g_systick_ms = 0u;
// Reload value para 1 ms a 250 MHz (SysTick usa HCLK)
static constexpr uint32_t kSysTickReload = 250000u - 1u;  // 1 ms

// ── SysTick_Handler ──────────────────────────────────────────────────────────
// Chamado a cada 1 ms pelo SysTick timer (ARM core, prioridade configurável).
extern "C" void SysTick_Handler(void) noexcept {
    ++g_systick_ms;
}

// ── API pública ───────────────────────────────────────────────────────────────

uint32_t millis(void) noexcept {
    // Leitura atômica: uint32_t em ARM é leitura de uma instrução (LDR).
    return g_systick_ms;
}

uint32_t micros(void) noexcept {
    // Combina contagem de ms com o valor atual do SysTick (conta regressiva).
    // SysTick CVR: valor atual do contador (decresce de kSysTickReload a 0).
    // µs = ms × 1000 + (kSysTickReload - CVR) / 250
    //   onde 250 = kSysTickReload / 1000 = ciclos por µs @ 250 MHz.
    const uint32_t ms = g_systick_ms;
    // ARM SysTick CVR em 0xE000E018
    const uint32_t cvr = *reinterpret_cast<volatile uint32_t*>(0xE000E018u);
    const uint32_t elapsed_us = (kSysTickReload - cvr) / 250u;
    return ms * 1000u + elapsed_us;
}

void iwdg_kick(void) noexcept {
    IWDG_KR = IWDG_KR_REFRESH;
}

// ── Inicialização do sistema ──────────────────────────────────────────────────

void system_stm32_init(void) noexcept {
    // ── 1. Habilitar HSE e aguardar estabilização ─────────────────────────
    RCC_CR |= RCC_CR_HSEON;
    while ((RCC_CR & RCC_CR_HSERDY) == 0u) { /* aguarda */ }

    // ── 2. Flash latency + cache antes de aumentar clock ─────────────────
    // 5 WS exigidos para HCLK > 210 MHz @ VOS0 (RM0481 §9.3.3)
    FLASH_ACR = (FLASH_ACR & ~0xFu)
              | FLASH_ACR_LATENCY_5WS
              | FLASH_ACR_PRFTEN
              | FLASH_ACR_ICEN
              | FLASH_ACR_DCEN;
    // Aguarda que o novo valor seja aplicado
    while ((FLASH_ACR & 0xFu) != 5u) { /* aguarda */ }

    // ── 3. Configurar PLL1: HSE=8 MHz / M=1 × N=125 / P=4 = 250 MHz ────
    // PLL1CFGR: PLLSRC=HSE (01b), DIVM1=1 (M-1 = 0)
    RCC_PLL1CFGR = (1u << 0)   // PLLSRC = HSE (bits [1:0] = 01)
                 | (0u << 8);  // DIVM1 = 1 (valor M-1 = 0, bits [13:8])
    // PLL1DIVR: N=125 (DIVN1 = N-1 = 124), P=4 (DIVP1 = P/2-1 = 1)
    // Bits [8:0]  = DIVN1 (N-1) = 124 = 0x7C
    // Bits [14:9] = DIVP1 (P/2-1) = 1
    // Bits [22:16]= DIVQ1 — não usado, manter 0
    // Bits [30:24]= DIVR1 — não usado, manter 0
    RCC_PLL1DIVR = (124u << 0)   // DIVN1 = 124 → N = 125
                 | (1u << 9);    // DIVP1 = 1   → P = 4

    // ── 4. Ligar PLL1 e aguardar lock ────────────────────────────────────
    RCC_CR |= RCC_CR_PLL1ON;
    while ((RCC_CR & RCC_CR_PLL1RDY) == 0u) { /* aguarda lock */ }

    // ── 5. Configurar prescalers APB (manter AHB = SYSCLK) ───────────────
    // CFGR1: AHB prescaler = 1 (HPRE=0), APB1=/2 (PPRE1=100b), APB2=/2 (PPRE2=100b)
    // Para simplificar timers: usar APB1=APB2=HCLK/2 → timer clock = 2×APB = HCLK
    RCC_CFGR1 = (0u << 4)    // HPRE = 0  → HCLK = SYSCLK
              | (4u << 8)    // PPRE1 = 4 → APB1 = HCLK/2 = 125 MHz
              | (4u << 11);  // PPRE2 = 4 → APB2 = HCLK/2 = 125 MHz

    // ── 6. Selecionar PLL1 como SYSCLK ───────────────────────────────────
    RCC_CFGR1 = (RCC_CFGR1 & ~0x7u) | RCC_CFGR1_SW_PLL1;
    while ((RCC_CFGR1 & (7u << 3)) != RCC_CFGR1_SWS_PLL1) { /* aguarda */ }

    // ── 7. Habilitar clocks dos GPIOs ────────────────────────────────────
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN
                  | RCC_AHB2ENR1_GPIOBEN
                  | RCC_AHB2ENR1_GPIOCEN
                  | RCC_AHB2ENR1_GPIODEN
                  | RCC_AHB2ENR1_GPIOEEN;

    // ── 8. Configurar SysTick @ 1 ms ─────────────────────────────────────
    // ARM SysTick registers (CMSIS):
    //   0xE000E010 = STK_CTRL  (ENABLE | TICKINT | CLKSRC=processor)
    //   0xE000E014 = STK_LOAD
    //   0xE000E018 = STK_VAL
    volatile uint32_t* stk_load = reinterpret_cast<volatile uint32_t*>(0xE000E014u);
    volatile uint32_t* stk_val  = reinterpret_cast<volatile uint32_t*>(0xE000E018u);
    volatile uint32_t* stk_ctrl = reinterpret_cast<volatile uint32_t*>(0xE000E010u);

    *stk_load = kSysTickReload;
    *stk_val  = 0u;
    // CLKSRC=1 (processor clock), TICKINT=1 (interrupt enable), ENABLE=1
    *stk_ctrl = (1u << 2) | (1u << 1) | (1u << 0);

    // SysTick priority: equivalente ao PIT0 do Kinetis (prio 11)
    // ARM: SCB->SHP[11] = priority for SysTick (offset 0xE000ED23)
    *reinterpret_cast<volatile uint8_t*>(0xE000ED23u) = static_cast<uint8_t>(11u << 4u);

    // ── 9. Configurar IWDG ≈ 100 ms ─────────────────────────────────────
    IWDG_KR  = IWDG_KR_START;    // Inicia IWDG (habilita LSI automaticamente)
    IWDG_KR  = IWDG_KR_ACCESS;   // Desbloqueia PR e RLR
    IWDG_PR  = IWDG_PR_DIV32;    // Prescaler /32 → 1000 Hz
    IWDG_RLR = IWDG_RLR_100MS;   // Reload = 99 → 99/1000 ≈ 99 ms
    IWDG_KR  = IWDG_KR_REFRESH;  // Primeiro kick
}

#else  // EMS_HOST_TEST

#include "hal/system.h"
#include <cstdint>
static uint32_t g_mock_ms = 0u;
void system_stm32_init(void) noexcept { }
void iwdg_kick(void) noexcept { }
uint32_t millis(void) noexcept { return g_mock_ms; }
uint32_t micros(void) noexcept { return g_mock_ms * 1000u; }

#endif  // EMS_HOST_TEST
