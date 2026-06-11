/**
 * @file hal/stm32h562/system.cpp
 * @brief Clock (PLL → 250 MHz), SysTick (1 ms), IWDG (100 ms)
 *        para STM32H562RGT6 — substitui runtime STM32 runtime.
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
    // [DIAG] preserva o waypoint do boot anterior p/ relatório RSR/WP via CDC
    *reinterpret_cast<volatile uint32_t*>(0x20050008u) =
        *reinterpret_cast<volatile uint32_t*>(0x20050000u);

    // ── 0. VOS0: obrigatório ANTES de subir para 250 MHz ─────────────────
    // Reset default é VOS3 (máx ~100 MHz). VOS0 (11b) permite até 250 MHz.
    // Timeout não-fatal: VOSRDY pode demorar até ~1ms; continuar sem crash
    // caso o periférico PWR não confirme (e.g. primeiro boot após DFU).
    PWR_VOSCR = (PWR_VOSCR & ~PWR_VOSCR_VOS_MSK) | PWR_VOSCR_VOS0;
    for (uint32_t n = 200000u; (PWR_VOSSR & PWR_VOSSR_VOSRDY) == 0u; --n) {
        if (n == 0u) { break; }  // não-fatal: continua sem VOSRDY confirmado
    }

    // ── 1. Habilitar HSE e aguardar estabilização ─────────────────────────
    RCC_CR |= RCC_CR_HSEON;
    for (uint32_t n = 500000u; (RCC_CR & RCC_CR_HSERDY) == 0u; --n) {
        if (n == 0u) { break; }  // não-fatal: continua sem HSE
    }

    // ── 1a. Habilitar HSI48 para USB DRD FS (48 MHz) ─────────────────────
    RCC_CR |= RCC_CR_HSI48ON;
    for (uint32_t n = 100000u; (RCC_CR & RCC_CR_HSI48RDY) == 0u; --n) {
        if (n == 0u) { break; }  // não-fatal
    }

    // ── 2. Flash latency + WRHIGHFREQ antes de aumentar clock ────────────
    // 5 WS + WRHIGHFREQ=0b10 exigidos para 150<HCLK≤250 MHz @ VOS0 (RM0481 §7.3.3).
    // Sem WRHIGHFREQ a flash retorna lixo a 250MHz (crash após switch p/ PLL).
    FLASH_ACR = (FLASH_ACR & ~(0xFu | FLASH_ACR_WRHIGHFREQ_MSK))
              | FLASH_ACR_LATENCY_5WS
              | FLASH_ACR_WRHIGHFREQ_250
              | FLASH_ACR_PRFTEN;
    for (uint32_t n = 100000u; (FLASH_ACR & 0xFu) != 5u; --n) {
        if (n == 0u) { break; }  // não-fatal
    }

    // ── 3. Configurar PLL1: HSE=8 MHz / M=2 × N=125 / P=2 = 250 MHz ────
    // Configuração confirmada por WeAct SystemClock_Config (PLLM=2, PLLN=125, PLLP=2)
    //   VCO input  = 8 MHz / 2 = 4 MHz  (PLL1RGE=10b, range 4-8 MHz)
    //   VCO output = 4 MHz × 125 = 500 MHz (wide range, max 960 MHz ✓)
    //   SYSCLK     = 500 MHz / 2 = 250 MHz
    // PLL1CFGR:
    //   [1:0]  PLL1SRC=11b (HSE) — RM0481: 00=none, 01=HSI, 10=CSI, 11=HSE.
    //          BUG ANTERIOR: usava 01b achando que era HSE, mas 01b é HSI →
    //          VCO input fora do range → PLL1RDY nunca setava → SYSCLK ficava
    //          em HSI 32MHz (8x lento; boot de ~6s virava ~47s).
    //   [3:2]  PLL1RGE=10b (4-8 MHz VCI input range)
    //   [5]    PLL1VCOSEL=0 (wide VCO range 192-960 MHz)
    //   [13:8] PLL1M=2
    //   [16]   PLL1PEN=1 (enable P output for SYSCLK)
    RCC_PLL1CFGR = (3u << 0)    // PLL1SRC = 11b = HSE
                 | (2u << 2)    // PLL1RGE = 10b (4-8 MHz input)
                 | (0u << 5)    // PLL1VCOSEL = 0 (wide, 192-960 MHz)
                 | (2u << 8)    // PLL1M = 2
                 | (1u << 16);  // PLL1PEN = 1
    // PLL1DIVR (campos guardam divisor−1, cf. __HAL_RCC_PLL1_CONFIG):
    //   [8:0]  PLL1N=124 → ×125 (VCO = 500 MHz)
    //   [15:9] PLL1P=1   → ÷2   (SYSCLK = 250 MHz)
    // BUG ANTERIOR: P=0 → ÷1 → SYSCLK = 500 MHz (2× o máximo!) → lockup
    // instantâneo no switch; os guards passavam porque o PLL trava normalmente.
    RCC_PLL1DIVR = (124u << 0)   // N = 125
                 | (1u << 9);    // P = 2

    // ── 4. Ligar PLL1 e aguardar lock ────────────────────────────────────
    RCC_CR |= RCC_CR_PLL1ON;
    for (uint32_t n = 200000u; (RCC_CR & RCC_CR_PLL1RDY) == 0u; --n) {
        if (n == 0u) { break; }  // não-fatal: continua sem PLL (HSI16)
    }

    // ── 5. Configurar prescalers AHB/APB — ficam no CFGR2 no H5! ─────────
    // BUG ANTERIOR: escrevia no CFGR1 (que é SW/SWS/MCO) — os prescalers nunca
    // eram configurados (tudo /1) e bits espúrios entravam no CFGR1.
    // CFGR2: HPRE[3:0], PPRE1[6:4], PPRE2[10:8]. PPRE=100b → /2.
    // Timer clock = 2×APB = HCLK = 250 MHz (timer doubler activo quando PPRE≠1)
    RCC_CFGR2 = (0u << 0)    // HPRE = 0   → HCLK = SYSCLK = 250 MHz
              | (4u << 4)    // PPRE1 = 100b → APB1 = 125 MHz
              | (4u << 8);   // PPRE2 = 100b → APB2 = 125 MHz

    // ── 6. Selecionar PLL1 como SYSCLK — SÓ com pré-condições confirmadas ─
    // Guards: sem eles, trocar p/ 250MHz com VOS/flash/PLL errado mata o core
    // sem fault (lockup) e sem diagnóstico. Se algum guard falhar, fica em HSI
    // (vivo, USB ok) e o relatório de boot via CDC mostra qual registo falhou.
    {
        const bool pll_ok  = (RCC_CR & RCC_CR_PLL1RDY) != 0u;
        const bool acr_ok  = (FLASH_ACR & (0xFu | FLASH_ACR_WRHIGHFREQ_MSK))
                             == (FLASH_ACR_LATENCY_5WS | FLASH_ACR_WRHIGHFREQ_250);
        const bool vos_ok  = (PWR_VOSSR & PWR_VOSSR_VOSRDY) != 0u
                          && (PWR_VOSSR & PWR_VOSSR_ACTVOS_MSK) == PWR_VOSSR_ACTVOS_VOS0;
        if (pll_ok && acr_ok && vos_ok) {
            RCC_CFGR1 = (RCC_CFGR1 & ~0x7u) | RCC_CFGR1_SW_PLL1;
            for (uint32_t n = 100000u; (RCC_CFGR1 & (7u << 3)) != RCC_CFGR1_SWS_PLL1; --n) {
                if (n == 0u) { break; }  // não-fatal
            }
        }
    }

    // ── 7. Habilitar clocks dos GPIOs ────────────────────────────────────
    // STM32H562RGT6 (LQFP64): apenas GPIOA/B/C disponíveis no package
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN
                  | RCC_AHB2ENR1_GPIOBEN
                  | RCC_AHB2ENR1_GPIOCEN;

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

    // SysTick priority: prio 11
    // ARM: SCB->SHP[11] = priority for SysTick (offset 0xE000ED23)
    *reinterpret_cast<volatile uint8_t*>(0xE000ED23u) = static_cast<uint8_t>(11u << 4u);

    // ── 9. CRS — sincroniza HSI48 com USB SOF ────────────────────────────
    // HSI48 sem CRS tem ±1.5% de desvio — fora da tolerancia USB FS (±0.25%).
    // CRS trima HSI48 usando USB SOF (1 kHz) para < 0.1% de desvio.
    // Confirmado: WeAct SystemClock_Config activa CRS com SYNCSRC=USB.
    RCC_APB1LENR |= RCC_APB1LENR_CRSEN;
    // RELOAD = 48MHz/1kHz - 1 = 47999; FELIM = 34; SYNCSRC = USB SOF (10b)
    CRS_CFGR = (47999u)
             | (34u << 16)
             | CRS_CFGR_SYNCSRC_USB;
    CRS_CR |= CRS_CR_CEN | CRS_CR_AUTOTRIMEN;

    // ── 10. IWDG ≈ 10 s durante boot (estreitado p/ 100ms no main loop) ──
    IWDG_KR  = IWDG_KR_START;
    IWDG_KR  = IWDG_KR_ACCESS;
    IWDG_PR  = IWDG_PR_DIV256;
    IWDG_RLR = IWDG_RLR_10S;
    for (uint32_t n = 10000u; (IWDG_SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u; --n) {
        if (n == 0u) { break; }
    }
    IWDG_KR  = IWDG_KR_REFRESH;
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
