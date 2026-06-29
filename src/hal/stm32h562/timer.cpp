/**
 * @file hal/stm32h562/timer.cpp
 * @brief Implementação da HAL de timers para STM32H562RGT6
 *        — backend STM32-only.
 *
 * Mapeamento de periféricos:
 *   TIM5_CH1 PA0: CKP input capture
 *   TIM5_CH2 PA1: CMP input capture
 *   TIM3_CH1-4 PC6-9: Injeção (OC, ARR=0xFFFF, ECU_Hardware_Init)
 *   TIM2_CH3 PB10: EWG PWM (motor wastegate)
 *   TIM4_CH1 PB6: VVT escape PWM
 *   TIM4_CH2 PB7: VVT admissao PWM
 *
 * Clock dos timers:
 *   TIM5, TIM3, TIM4, TIM2 (APB1): timer clock = 250 MHz (timer doubler ativo)
 *   TIM3/TIM1 scheduling: configurados em engine/ecu_sched.cpp a 10 MHz
 *   TIM5 CKP prescaler = 3 → tick = 250 MHz / 4 = 62.5 MHz → 16 ns/tick
 */

#ifndef EMS_HOST_TEST

#include "hal/timer.h"
#include "hal/regs.h"
#include "drv/ckp.h"    // ckp_tim5_ch1_isr / ckp_tim5_ch2_isr
#include "engine/ecu_sched.h"  // ecu_sched_evt_dispatch

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
              | TIM_CCER_CC2E             // habilita captura CH2
              | TIM_CCER_CC3E;            // habilita compare CH3 (event dispatcher)

    // CH3 = Output Compare, Frozen mode (event dispatcher — no pin output)
    // CCMR2.CC3S = 00 (output), OC3M = 000 (frozen)
    TIM5_CCMR2 = 0u;  // all zero = output compare frozen, no preload

    // Habilitar interrupções CC1 (CKP) e CC2 (CMP)
    // CC3IE enabled on-demand by evt_insert
    TIM5_DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE;

    // ── 4. NVIC TIM5 → prioridade 1 (mesma que CC ISRs — sem preemption)
    nvic_set_priority(IRQ_TIM5, 1u);
    nvic_enable_irq(IRQ_TIM5);

    // ── 5. Iniciar contador ─────────────────────────────────────────────
    TIM5_CR1 = TIM_CR1_CEN;
}

uint32_t tim5_count() noexcept {
    return TIM5_CNT;
}

// ════════════════════════════════════════════════════════════════════════════
// TIM3 — PWM legacy (CH1: EWG motor) — NÃO USAR
// ════════════════════════════════════════════════════════════════════════════
// ATENÇÃO: estas funções NÃO são mais chamadas. O EWG PWM migrou para
// TIM2_CH3/PB10 (tim2_pwm_init). TIM3 é dedicado à injeção (OC em PC6-9,
// ARR=0xFFFF em ECU_Hardware_Init). Não chame tim3_pwm_init — quebra o
// timing dos injetores.

void tim3_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }
    RCC_APB1LENR |= RCC_APB1LENR_TIM3EN;

    // PA6 = TIM3_CH1 (EWG PWM) — AF2
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 6u, GPIO_AF2);

    // For high frequencies (>1kHz), use prescaler to keep ARR in 16-bit range
    uint32_t psc = 0u;
    uint32_t arr = kTimClockHz / freq_hz;
    while (arr > 0xFFFFu) { ++psc; arr = kTimClockHz / (freq_hz * (psc + 1u)); }
    if (arr > 0u) { arr -= 1u; }

    TIM3_CR1 = 0u;
    TIM3_PSC = psc;
    TIM3_ARR = arr;
    TIM3_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;  // CH1: EWG
    TIM3_CCER = TIM_CCER_CC1E;
    TIM3_CCR1 = 0u;
    TIM3_EGR  = 1u;
    TIM3_CR1  = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    if (duty_pct_x10 > 1000u) { duty_pct_x10 = 1000u; }
    const uint32_t arr = TIM3_ARR;
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
    if (freq_hz == 0u) { return; }
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
    if (duty_pct_x10 > 1000u) { duty_pct_x10 = 1000u; }
    const uint32_t arr = TIM4_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    if (ch == 0u) {
        TIM4_CCR1 = ccr;
    } else {
        TIM4_CCR2 = ccr;
    }
}

// ════════════════════════════════════════════════════════════════════════════
// TIM2 — PWM do motor EWG (CH3 em PB10, AF1)
// ════════════════════════════════════════════════════════════════════════════
// Movido do TIM3 (que é dedicado à injeção: OC em PC6-9, ARR=0xFFFF). TIM3_CH1
// (PA6/EWG) colidia com INJ0 (PC6) — mesmo canal — e o tim3_pwm_init reescrevia
// o ARR do TIM3, quebrando o timing dos injetores.

void tim2_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;

    // PB10 = TIM2_CH3 (EWG PWM) — AF1
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 10u, GPIO_AF1);

    uint32_t psc = 0u;
    uint32_t arr = kTimClockHz / freq_hz;
    while (arr > 0xFFFFu) { ++psc; arr = kTimClockHz / (freq_hz * (psc + 1u)); }
    if (arr > 0u) { arr -= 1u; }

    TIM2_CR1   = 0u;
    TIM2_PSC   = psc;
    TIM2_ARR   = arr;
    TIM2_CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;  // CH3: EWG
    TIM2_CCER  = TIM_CCER_CC3E;
    TIM2_CCR3  = 0u;
    TIM2_EGR   = 1u;
    TIM2_CR1   = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void tim2_set_duty(uint16_t duty_pct_x10) noexcept {
    if (duty_pct_x10 > 1000u) { duty_pct_x10 = 1000u; }
    const uint32_t arr = TIM2_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    TIM2_CCR3 = ccr;
}

// ════════════════════════════════════════════════════════════════════════════
// TIM15 — ETB PWM (PE5 CH1, AF4)
// ════════════════════════════════════════════════════════════════════════════

void tim15_etb_pwm_init(uint32_t freq_hz) {
    RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN;

    gpio_set_af(&GPIOE_MODER, &GPIOE_AFRL, &GPIOE_AFRH, &GPIOE_OSPEEDR, 5u, GPIO_AF4);

    uint32_t arr = kTimClockHz / freq_hz;
    if (arr > 0xFFFFu) { arr = 0xFFFFu; }
    if (arr > 0u) { arr -= 1u; }

    TIM15_CR1 = 0u;
    TIM15_PSC = 0u;
    TIM15_ARR = arr;
    TIM15_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
    TIM15_CCER = TIM_CCER_CC1E;
    TIM15_CCR1 = 0u;
    TIM15_BDTR = (1u << 15);  // MOE
    TIM15_EGR = 1u;
    TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void tim15_etb_set_duty_x10(uint16_t duty_pct_x10) noexcept {
    const uint32_t arr = TIM15_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    TIM15_CCR1 = ccr;
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
    uint32_t sr = TIM5_SR;

    if (sr & TIM_SR_CC1IF) {
        TIM5_SR = ~TIM_SR_CC1IF;  // rc_w0: write-1-no-effect, só CC1IF é limpo
        ems::drv::ckp_tim5_ch1_isr();
    }
    // Re-read SR: CC2IF (CMP) may have arrived during CKP ISR
    // (a ISR CKP leva microsegundos; a borda CMP pode chegar nesse intervalo)
    sr = TIM5_SR;
    if (sr & TIM_SR_CC2IF) {
        TIM5_SR = ~TIM_SR_CC2IF;  // rc_w0: só CC2IF é limpo (não aniquila CC1IF nem CC3IF)
        ems::drv::ckp_tim5_ch2_isr();
    }
    // Re-read SR: CC3IF may have been set by evt_insert during tooth handler
    sr = TIM5_SR;
    if (sr & TIM_SR_CC3IF) {
        TIM5_SR = ~TIM_SR_CC3IF;  // rc_w0: só CC3IF é limpo
        ecu_sched_evt_dispatch();
    }
}

} // namespace ems::hal

// ════════════════════════════════════════════════════════════════════════════
// TIM15 — PWM para Borboleta Eletrônica (ETB) com dead-time (PE5 CH1, AF4)
// ════════════════════════════════════════════════════════════════════════════

void timer_etb_pwm_init(void) {
    RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN;

    gpio_set_af(&GPIOE_MODER, &GPIOE_AFRL, &GPIOE_AFRH, &GPIOE_OSPEEDR, 5u, GPIO_AF4);

    TIM15_CR1 = 0u;
    TIM15_PSC = 0u;
    TIM15_ARR = 12499u;                      // 20kHz @ 250MHz
    TIM15_EGR = TIM_EGR_UG;

    TIM15_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;

    TIM15_BDTR = TIM_BDTR_MOE
              | (50u << 0u);           // DTG = 50 → ~200ns

    TIM15_CCER = TIM_CCER_CC1E;

    TIM15_CCR1 = 0u;

    TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void timer_etb_set_duty(uint16_t duty) {
    if (duty > 1023u) { duty = 1023u; }
    uint32_t ccr1_val = ((uint32_t)duty * 12499u) / 1023u;
    TIM15_CCR1 = (uint16_t)ccr1_val;
}

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/timer.h"
namespace ems::hal {
static uint32_t g_mock_tim5_cnt = 0u;
void tim5_ic_init(void) {}
void tim3_pwm_init(uint32_t) {}
void tim4_pwm_init(uint32_t) {}
void tim2_pwm_init(uint32_t) {}
void tim3_set_duty(uint8_t, uint16_t) noexcept {}
void tim2_set_duty(uint16_t) noexcept {}
void tim4_set_duty(uint8_t, uint16_t) noexcept {}
void tim15_etb_pwm_init(uint32_t) {}
void tim15_etb_set_duty_x10(uint16_t) noexcept {}
uint32_t tim5_count() noexcept { return g_mock_tim5_cnt; }
} // namespace ems::hal

// ════════════════════════════════════════════════════════════════════════════
// TIM15 — PWM para Borboleta Eletrônica (ETB): stubs no host
// ════════════════════════════════════════════════════════════════════════════

void timer_etb_pwm_init(void) {}
void timer_etb_set_duty(uint16_t duty) { (void)duty; }

#endif  // EMS_HOST_TEST
