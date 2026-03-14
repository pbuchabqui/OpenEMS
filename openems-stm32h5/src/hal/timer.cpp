/**
 * @file hal/stm32h562/timer.cpp
 * @brief Implementação da HAL de timers para STM32H562RGT6
 *        — substitui hal/ftm.cpp na versão STM32.
 *
 * Mapeamento de periféricos (equivalência FTM → TIM):
 *
 *   FTM3 (IC CKP + CMP)   → TIM5 (32-bit, AF2)
 *     CH0 (PTD0 → CKP)    → TIM5_CH1 (PA0, AF2) rising edge capture
 *     CH1 (PTD1 → CMP)    → TIM5_CH2 (PA1, AF2) rising edge capture
 *     NVIC FTM3 prio 1    → NVIC TIM5 prio 1
 *
 *   FTM0 (OC ignição/injeção) → TIM1 (advanced, AF1)
 *     CH4-CH7 (IGN1-4)    → TIM1_CH1-CH4 (PA8,PA9,PA10,PA11, AF1)
 *     NVIC FTM0 prio 4    → NVIC TIM1_CC prio 4
 *     (PA9/PA10 liberados: USART movido USART1→USART2 em PA2/PA3)
 *     (PA11 liberado: FDCAN1 movido para PB8/PB9)
 *
 *   FTM1 (PWM IACV + WG)  → TIM3 (AF2)
 *     CH0 (PTA8 → IACV)   → TIM3_CH1 (PA6, AF2)
 *     CH1 (PTA9 → WG)     → TIM3_CH2 (PA7, AF2)
 *
 *   FTM2 (PWM VVT)        → TIM4 (AF2)
 *     CH0 (PTA10 → VVT E) → TIM4_CH1 (PB6, AF2)
 *     CH1 (PTA11 → VVT A) → TIM4_CH2 (PB7, AF2)
 *
 * Clock dos timers:
 *   TIM5, TIM3, TIM4 (APB1): timer clock = 250 MHz (timer doubler ativo)
 *   TIM1 (APB2):              timer clock = 250 MHz
 *   Prescaler TIM5/TIM1 = 3 → tick = 250 MHz / 4 = 62.5 MHz → 16 ns/tick
 *   (≈ FTM3/FTM0 @ 60 MHz → 16.667 ns/tick — menos de 4% de diferença)
 */

#ifndef EMS_HOST_TEST

#include "hal/ftm.h"
#include "hal/regs.h"
#include "drv/ckp.h"    // ckp_ftm3_ch0_isr / ckp_ftm3_ch1_isr

// ── Constantes de clock ───────────────────────────────────────────────────────
// Timer clock = 250 MHz, prescaler = 3 → 62.5 MHz → 16 ns/tick
static constexpr uint32_t kTimPrescaler = 3u;          // PSC = N-1 → /4
static constexpr uint32_t kTimClockHz   = 62500000u;   // 62.5 MHz

namespace ems::hal {

// ════════════════════════════════════════════════════════════════════════════
// TIM5 — Input Capture (CKP + CMP) — equivale ao FTM3
// ════════════════════════════════════════════════════════════════════════════

void ftm3_init(void) {
    // ── 1. Habilitar clock TIM5 ─────────────────────────────────────────
    RCC_APB1LENR |= RCC_APB1LENR_TIM5EN;

    // ── 2. Configurar pinos PA0 (TIM5_CH1/CKP) e PA1 (TIM5_CH2/CMP) ───
    // AF2 = TIM3/TIM4/TIM5 nos pinos PA0, PA1
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 0u, GPIO_AF2);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 1u, GPIO_AF2);

    // ── 3. Configurar TIM5 ───────────────────────────────────────────────
    // Desabilitar timer durante configuração
    TIM5_CR1 = 0u;
    TIM5_PSC = kTimPrescaler;              // /4 → 62.5 MHz
    TIM5_ARR = 0xFFFFFFFFu;               // TIM5 é 32-bit, livre
    TIM5_EGR = 1u;                         // UG: aplica prescaler imediatamente

    // CH1 = Input Capture, mapeado em TI1, rising edge, sem filtro
    TIM5_CCMR1 = TIM_CCMR1_CC1S_TI1      // CH1 → IC mode, source = TI1
               | TIM_CCMR1_IC1F_NONE      // sem filtro
               | TIM_CCMR1_CC2S_TI2       // CH2 → IC mode, source = TI2
               | TIM_CCMR1_IC2F_NONE;

    // Polaridade: rising edge para CH1 e CH2 (CC1P=0, CC2P=0)
    // CC1NP=0, CC2NP=0 → não inverte
    TIM5_CCER = TIM_CCER_CC1E             // habilita captura CH1
              | TIM_CCER_CC2E;            // habilita captura CH2

    // Habilitar interrupções CC1 (CKP) e CC2 (CMP)
    TIM5_DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE;

    // ── 4. NVIC TIM5 → prioridade 1 (máxima — equivalente ao FTM3 no Kinetis) ─
    nvic_set_priority(IRQ_TIM5, 1u);
    nvic_enable_irq(IRQ_TIM5);

    // ── 5. Iniciar contador ─────────────────────────────────────────────
    TIM5_CR1 = TIM_CR1_CEN;
}

uint16_t ftm3_count() noexcept {
    // TIM5 é 32-bit; retorna os 16 bits inferiores para compatibilidade
    // com a aritmética circular uint16_t do ckp.cpp
    return static_cast<uint16_t>(TIM5_CNT & 0xFFFFu);
}

// ════════════════════════════════════════════════════════════════════════════
// TIM1 — Output Compare (Ignição + Injeção) — equivale ao FTM0
// ════════════════════════════════════════════════════════════════════════════

void ftm0_init(void) {
    // ── 1. Habilitar clock TIM1 ─────────────────────────────────────────
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

    // ── 2. Configurar pinos TIM1_CH1-CH4 em PA8, PA9, PA10, PA11 ───────
    // AF1 = TIM1 em PA8 (CH1), PA9 (CH2), PA10 (CH3), PA11 (CH4)
    // NOTA: USART movido para USART2 (PA2/PA3); FDCAN1 movido para PB8/PB9
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR,  8u, GPIO_AF1);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR,  9u, GPIO_AF1);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 10u, GPIO_AF1);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 11u, GPIO_AF1);

    // ── 3. Configurar TIM1 ───────────────────────────────────────────────
    TIM1_CR1 = 0u;
    TIM1_PSC = kTimPrescaler;              // /4 → 62.5 MHz → 16 ns/tick
    TIM1_ARR = 0xFFFFu;                    // 16-bit free-running (compat. com FTM0)
    TIM1_EGR = 1u;                         // UG: aplica PSC

    // CH1-CH4: Output Compare, modo frozen até ftm0_arm_ignition() ser chamado
    TIM1_CCMR1 = TIM_CCMR1_OC1M_FROZEN | TIM_CCMR1_OC2M_FROZEN;
    TIM1_CCMR2 = 0u;   // OC3M=FROZEN, OC4M=FROZEN (0 = frozen em todos os campos)
    // Saídas habilitadas, polaridade ativa-alta
    TIM1_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    // MOE: main output enable (TIM1 advanced requer este bit)
    TIM1_BDTR = (1u << 15);   // MOE bit

    // Interrupções CC1-CC4 habilitadas
    TIM1_DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;

    // ── 4. NVIC TIM1_CC → prioridade 4 (equivalente ao FTM0 no Kinetis) ─
    nvic_set_priority(IRQ_TIM1_CC, 4u);
    nvic_enable_irq(IRQ_TIM1_CC);

    // ── 5. Iniciar contador ─────────────────────────────────────────────
    TIM1_CR1 = TIM_CR1_CEN;
}

void ftm0_set_compare(uint8_t ch, uint16_t ticks) noexcept {
    // Mapeia canal 0-7 do FTM0 → CH1-CH4 do TIM1
    // FTM0 CH4→IGN4, CH5→IGN3, CH6→IGN2, CH7→IGN1
    // TIM1 CH1→IGN4, CH2→IGN3, CH3→IGN2, CH4→IGN1
    const uint8_t tim_ch = (ch >= 4u) ? (ch - 3u) : 1u;
    switch (tim_ch) {
        case 1u: TIM1_CCR1 = ticks; break;
        case 2u: TIM1_CCR2 = ticks; break;
        case 3u: TIM1_CCR3 = ticks; break;
        case 4u: TIM1_CCR4 = ticks; break;
        default: break;
    }
}

void ftm0_clear_chf(uint8_t ch) noexcept {
    const uint8_t tim_ch = (ch >= 4u) ? (ch - 3u) : 1u;
    // Limpa o flag CC correspondente (W0C: escrever 0 limpa)
    TIM1_SR &= ~(1u << tim_ch);
}

uint16_t ftm0_count() noexcept {
    return static_cast<uint16_t>(TIM1_CNT & 0xFFFFu);
}

void ftm0_arm_ignition(uint8_t ch, uint16_t ticks) noexcept {
    // Equivalente ao FTM0_arm_ignition do Kinetis:
    // Output Compare "clear on match" (OC inactive on match = pino vai LOW)
    // No TIM1: OC mode = 010 (Force inactive) não existe nativamente como match;
    // usa-se OC mode "PWM2" com CCR = alvo, de forma que quando CNT >= CCR,
    // a saída vai LOW (PWM2 ativo abaixo do comparador).
    // Alternativa mais simples: modo OC "Active on match" depois "Inactive on match".
    //
    // Implementação: configura output compare "force inactive" ao atingir CCR.
    // OC mode = 100 (Force Inactive = LOW) ativado quando CHxIF flag set.
    // Para manter compatibilidade, usamos modo 010 (Inactive on match).
    const uint8_t tim_ch = (ch >= 4u) ? (ch - 3u) : 1u;

    // Configura output compare "Set inactive on match" (OC mode 010 = inactive)
    if (tim_ch == 1u || tim_ch == 2u) {
        uint32_t ccmr = TIM1_CCMR1;
        if (tim_ch == 1u) {
            ccmr &= ~0x70u;                    // limpa OC1M bits [6:4]
            ccmr |= TIM_CCMR1_OC1M_INACTIVE;  // OC1M = 010 (inactive on match)
            TIM1_CCMR1 = ccmr;
            TIM1_CCR1 = ticks;
        } else {
            ccmr &= ~0x7000u;                  // limpa OC2M bits [14:12]
            ccmr |= (2u << 12);                // OC2M = 010
            TIM1_CCMR1 = ccmr;
            TIM1_CCR2 = ticks;
        }
    } else {
        uint32_t ccmr = TIM1_CCMR2;
        if (tim_ch == 3u) {
            ccmr &= ~0x70u;                    // limpa OC3M bits [6:4]
            ccmr |= (2u << 4);                 // OC3M = 010 (inactive on match)
            TIM1_CCMR2 = ccmr;
            TIM1_CCR3 = ticks;
        } else {
            ccmr &= ~0x7000u;                  // limpa OC4M bits [14:12]
            ccmr |= (2u << 12);                // OC4M = 010 (inactive on match)
            TIM1_CCMR2 = ccmr;
            TIM1_CCR4 = ticks;
        }
    }
    // Habilitar interrupção do canal
    TIM1_DIER |= (1u << tim_ch);
}

// ════════════════════════════════════════════════════════════════════════════
// TIM3 — PWM (IACV CH1 + Wastegate CH2) — equivale ao FTM1
// ════════════════════════════════════════════════════════════════════════════

void ftm1_pwm_init(uint32_t freq_hz) {
    RCC_APB1LENR |= RCC_APB1LENR_TIM3EN;

    // PA6 = TIM3_CH1 (IACV), PA7 = TIM3_CH2 (Wastegate) — AF2
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 6u, GPIO_AF2);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 7u, GPIO_AF2);

    // Calcular ARR e PSC para freq_hz desejada
    // f_pwm = timer_clock / ((PSC+1) * (ARR+1))
    // Resolução máxima: PSC=0, ARR = timer_clock / freq_hz - 1
    uint32_t arr = kTimClockHz / freq_hz;
    if (arr > 0xFFFFu) { arr = 0xFFFFu; }
    if (arr > 0u) { arr -= 1u; }

    TIM3_CR1 = 0u;
    TIM3_PSC = 0u;
    TIM3_ARR = arr;
    TIM3_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE   // CH1: PWM1
               | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;  // CH2: PWM1
    TIM3_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;            // enable outputs
    TIM3_CCR1 = 0u;    // 0% duty inicial
    TIM3_CCR2 = 0u;
    TIM3_EGR  = 1u;    // UG: aplica valores
    TIM3_CR1  = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void ftm1_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    const uint32_t arr = TIM3_ARR;
    // CCR = (ARR+1) * duty_pct_x10 / 1000
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    if (ch == 0u) {
        TIM3_CCR1 = ccr;
    } else {
        TIM3_CCR2 = ccr;
    }
}

// ════════════════════════════════════════════════════════════════════════════
// TIM4 — PWM (VVT Exhaust CH1 + VVT Intake CH2) — equivale ao FTM2
// ════════════════════════════════════════════════════════════════════════════

void ftm2_pwm_init(uint32_t freq_hz) {
    RCC_APB1LENR |= RCC_APB1LENR_TIM4EN;

    // PB6 = TIM4_CH1 (VVT Exhaust), PB7 = TIM4_CH2 (VVT Intake) — AF2
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 6u, GPIO_AF2);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 7u, GPIO_AF2);

    uint32_t arr = kTimClockHz / freq_hz;
    if (arr > 0xFFFFu) { arr = 0xFFFFu; }
    if (arr > 0u) { arr -= 1u; }

    TIM4_CR1 = 0u;
    TIM4_PSC = 0u;
    TIM4_ARR = arr;
    TIM4_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE
               | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    TIM4_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM4_CCR1 = 0u;
    TIM4_CCR2 = 0u;
    TIM4_EGR  = 1u;
    TIM4_CR1  = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void ftm2_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    const uint32_t arr = TIM4_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    if (ch == 0u) {
        TIM4_CCR1 = ccr;
    } else {
        TIM4_CCR2 = ccr;
    }
}

// ════════════════════════════════════════════════════════════════════════════
// ISR Handlers
// ════════════════════════════════════════════════════════════════════════════

/**
 * @brief TIM5_IRQHandler — equivale ao FTM3_IRQHandler do Kinetis.
 *
 * TIM5 gera uma única IRQ para todos os canais. Verifica qual flag está
 * ativo (CC1IF = CKP, CC2IF = CMP) e despacha para os respectivos handlers.
 */
extern "C" void TIM5_IRQHandler(void) {
    const uint32_t sr = TIM5_SR;

    if (sr & TIM_SR_CC1IF) {
        TIM5_SR = ~TIM_SR_CC1IF;   // Clear CC1IF (W0C)
        ems::drv::ckp_ftm3_ch0_isr();
    }
    if (sr & TIM_SR_CC2IF) {
        TIM5_SR = ~TIM_SR_CC2IF;
        ems::drv::ckp_ftm3_ch1_isr();
    }
}

/**
 * @brief TIM1_CC_IRQHandler — equivale ao FTM0_IRQHandler do Kinetis.
 *
 * Chamado quando qualquer canal de output compare do TIM1 dispara.
 * O handler real é implementado em engine/ecu_sched.cpp (ECU_Scheduler).
 */
extern "C" void FTM0_IRQHandler(void);   // implementado em ecu_sched.cpp

extern "C" void TIM1_CC_IRQHandler(void) {
    const uint32_t sr = TIM1_SR;
    TIM1_SR = ~sr;       // limpa apenas os flags que dispararam esta IRQ (W0C)
    FTM0_IRQHandler();   // delega para o scheduler existente
}

// Stub para FTM3_IRQHandler — não usado no STM32 (substituído por TIM5_IRQHandler)
extern "C" void FTM3_IRQHandler(void) { }

} // namespace ems::hal

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/ftm.h"
namespace ems::hal {
static uint16_t g_mock_ftm0_cnt = 0u;
static uint16_t g_mock_ftm3_cnt = 0u;
void ftm0_init(void) {}
void ftm3_init(void) {}
void ftm1_pwm_init(uint32_t) {}
void ftm2_pwm_init(uint32_t) {}
void ftm0_set_compare(uint8_t, uint16_t) noexcept {}
void ftm0_clear_chf(uint8_t) noexcept {}
void ftm0_arm_ignition(uint8_t, uint16_t) noexcept {}
void ftm1_set_duty(uint8_t, uint16_t) noexcept {}
void ftm2_set_duty(uint8_t, uint16_t) noexcept {}
uint16_t ftm0_count() noexcept { return g_mock_ftm0_cnt; }
uint16_t ftm3_count() noexcept { return g_mock_ftm3_cnt; }
} // namespace ems::hal

#endif  // EMS_HOST_TEST
