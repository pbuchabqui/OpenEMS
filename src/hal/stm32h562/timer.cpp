/**
 * @file hal/stm32h562/timer.cpp
 * @brief Implementação da HAL de timers para STM32H562RGT6
 *        — backend STM32-only.
 *
 * Mapeamento de periféricos:
 *   TIM5_CH1 PA0: CKP input capture
 *   TIM5_CH2 PA1: CMP input capture
 *   TIM3_CH1 PA6: IACV PWM
 *   TIM3_CH2 PA7: Wastegate PWM
 *   TIM4_CH1 PB6: VVT escape PWM
 *   TIM4_CH2 PB7: VVT admissao PWM
 *
 * Clock dos timers:
 *   TIM5, TIM3, TIM4 (APB1): timer clock = 250 MHz (timer doubler ativo)
 *   TIM2/TIM8 scheduling: configurados em engine/ecu_sched.cpp a 10 MHz
 *   TIM5 CKP prescaler = 3 → tick = 250 MHz / 4 = 62.5 MHz → 16 ns/tick
 */

#ifndef EMS_HOST_TEST

#include "hal/timer.h"
#include "hal/regs.h"
#include "drv/ckp.h"    // ckp_tim5_ch1_isr / ckp_tim5_ch2_isr

// ── Constantes de clock ───────────────────────────────────────────────────────
// Timer clock = 250 MHz, prescaler = 3 → 62.5 MHz → 16 ns/tick
static constexpr uint32_t kTimPrescaler = 3u;          // PSC = N-1 → /4
static constexpr uint32_t kTimClockHz   = 62500000u;   // 62.5 MHz

namespace ems::hal {

// ════════════════════════════════════════════════════════════════════════════
// TIM5 — Input Capture (CKP + CMP) — TIM5
// ════════════════════════════════════════════════════════════════════════════

void tim5_ic_init(void) {
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

    // ── 4. NVIC TIM5 → prioridade 1 (máxima) ─
    nvic_set_priority(IRQ_TIM5, 1u);
    nvic_enable_irq(IRQ_TIM5);

    // ── 5. Iniciar contador ─────────────────────────────────────────────
    TIM5_CR1 = TIM_CR1_CEN;
}

uint32_t tim5_count() noexcept {
    return TIM5_CNT;
}

// ════════════════════════════════════════════════════════════════════════════
// TIM3 — PWM (IACV CH1 + Wastegate CH2)
// ════════════════════════════════════════════════════════════════════════════

void tim3_pwm_init(uint32_t freq_hz) {
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

void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
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
// TIM4 — PWM (VVT Exhaust CH1 + VVT Intake CH2)
// ════════════════════════════════════════════════════════════════════════════

void tim4_pwm_init(uint32_t freq_hz) {
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

void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
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
 * @brief TIM5_IRQHandler.
 *
 * TIM5 gera uma única IRQ para todos os canais. Verifica qual flag está
 * ativo (CC1IF = CKP, CC2IF = CMP) e despacha para os respectivos handlers.
 */
extern "C" void TIM5_IRQHandler(void) {
    const uint32_t sr = TIM5_SR;

    if (sr & TIM_SR_CC1IF) {
        TIM5_SR = ~TIM_SR_CC1IF;   // Clear CC1IF (W0C)
        ems::drv::ckp_tim5_ch1_isr();
    }
    if (sr & TIM_SR_CC2IF) {
        TIM5_SR = ~TIM_SR_CC2IF;
        ems::drv::ckp_tim5_ch2_isr();
    }
}

} // namespace ems::hal

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/timer.h"
namespace ems::hal {
static uint32_t g_mock_tim5_cnt = 0u;
void tim5_ic_init(void) {}
void tim3_pwm_init(uint32_t) {}
void tim4_pwm_init(uint32_t) {}
void tim3_set_duty(uint8_t, uint16_t) noexcept {}
void tim4_set_duty(uint8_t, uint16_t) noexcept {}
uint32_t tim5_count() noexcept { return g_mock_tim5_cnt; }
} // namespace ems::hal

#endif  // EMS_HOST_TEST
