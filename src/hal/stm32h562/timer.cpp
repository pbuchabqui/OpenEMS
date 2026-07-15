/**
 * @file hal/stm32h562/timer.cpp
 * @brief Implementacao da HAL de timers para STM32H562RGT6
 *        - backend STM32-only.
 *
 * Mapeamento de perifericos:
 *   TIM5_CH1 PA0: CKP input capture (62.5 MHz, 16 ns/tick)
 *   TIM5_CH2 PA1: CMP input capture
 *   TIM5_CH3   --: event dispatcher (OC)
 *   TIM3_CH1-4 PC6-9: Injecao (OC, ARR=0xFFFF, ECU_Hardware_Init)
 *   TIM2_CH3 PB10: EWG PWM (motor wastegate)
 *   TIM4_CH1 PB6: VVT escape PWM
 *   TIM4_CH2 PB7: VVT admissao PWM
 *
 * Clock dos timers:
 *   TIM5, TIM3, TIM4, TIM2 (APB1): timer clock = 250 MHz (timer doubler ativo)
 *   TIM3/TIM1 scheduling: configurados em engine/ecu_sched.cpp a 10 MHz
 *   TIM5 prescaler = 3 -> tick = 250 MHz / 4 = 62.5 MHz -> 16 ns/tick
 */

#ifndef EMS_HOST_TEST

#include "hal/timer.h"
#include "hal/regs.h"
#include "drv/ckp.h"    // ckp_tim5_ch1_isr / ckp_tim5_ch2_isr
#include "engine/ecu_sched.h"  // ecu_sched_evt_dispatch

// -- Constantes de clock --------------------------------------------------------
static constexpr uint32_t kTimPrescaler = 3u;
static constexpr uint32_t kTimClockHz   = 62500000u;

namespace ems::hal {

// ------------------------------------------------------------------------------
// CKP via TIM5_CH1 (PA0/AF2) + CMP via TIM5_CH2 (PA1/AF2)
// Input capture hardware com 32-bit, 62.5 MHz, 16 ns/tick — zero jitter de ISR.
// ------------------------------------------------------------------------------

void tim5_ic_init(void) {
    // PA0 = TIM5_CH1/CKP (AF2), PA1 = TIM5_CH2/CMP (AF2)
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 0u, GPIO_AF2);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 1u, GPIO_AF2);
    // Pull-down em PA0 (CKP) e PA1 (CMP): sensores idle-LOW/pulsam-HIGH, captura por
    // borda de subida. Sem pull, um sensor desligado/fio partido deixa o pino a
    // flutuar → ruído gera bordas fantasma que fingem sync (FULL_SYNC falso → injeção
    // batch espúria nos 4 injetores). Pull-down força LOW estável quando desligado =
    // sem bordas de subida. Complementa o filtro IC (que corta glitch fino mas não
    // ruído de baixa frequência num pino aberto). (cf. uart.cpp:50, PA10.)
    GPIOA_PUPDR = (GPIOA_PUPDR & ~((0x3u << 0u) | (0x3u << 2u)))
                | (0x2u << 0u) | (0x2u << 2u);  // PA0(CKP)+PA1(CMP), 0b10 = pull-down

    // Ativar clocks
    RCC_APB1LENR |= RCC_APB1LENR_TIM5EN;
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;

    // TIM5 — 32-bit free-running @ 62.5 MHz, input capture CH1=CKP, CH2=CMP
    TIM5_CR1 = 0u; TIM5_PSC = 3u; TIM5_ARR = 0xFFFFFFFFu; TIM5_EGR = 1u;
    // CH1: CKP input capture (TI1, rising edge)
    // Filtro de entrada ~256 ns (IC1F/IC2F) rejeita glitches EMC no HW antes da
    // captura — CKP (PA0) e CMP (PA1) sem filtro deixavam ruído gerar bordas
    // fantasma que disparavam prime/falso-sync com o motor parado.
    TIM5_CCMR1 = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_IC1F_N8_DTS8
               | TIM_CCMR1_CC2S_TI2 | TIM_CCMR1_IC2F_N8_DTS8;
    TIM5_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM5_CCMR2 = 0u;
    TIM5_DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE;

    // TIM2_CH1 — já não usado para CKP (apenas PWM EWG no CH3)
    TIM2_CCER = 0u;
    TIM2_DIER = 0u;

    nvic_set_priority(IRQ_TIM5, 1u); nvic_enable_irq(IRQ_TIM5);
    TIM5_CR1 = TIM_CR1_CEN;
}

uint32_t tim5_count() noexcept {
    return TIM5_CNT;
}

// ----------------------------------------------------------------------------
// TIM3 - PWM legacy (CH1: EWG motor) - NAO USAR
// ----------------------------------------------------------------------------

void tim3_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }
    RCC_APB1LENR |= RCC_APB1LENR_TIM3EN;

    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 6u, GPIO_AF2);

    uint32_t psc = 0u;
    uint32_t arr = kTimClockHz / freq_hz;
    while (arr > 0xFFFFu) { ++psc; arr = kTimClockHz / (freq_hz * (psc + 1u)); }
    if (arr > 0u) { arr -= 1u; }

    TIM3_CR1 = 0u;
    TIM3_PSC = psc;
    TIM3_ARR = arr;
    TIM3_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
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

// ----------------------------------------------------------------------------
// TIM4 - PWM (VVT Exhaust CH1 + VVT Intake CH2)
// ----------------------------------------------------------------------------

void tim4_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }
    RCC_APB1LENR |= RCC_APB1LENR_TIM4EN;

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

// ----------------------------------------------------------------------------
// TIM2 - PWM do motor EWG (CH3 em PB10, AF1)
// ----------------------------------------------------------------------------

void tim2_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;

    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 10u, GPIO_AF1);

    uint32_t psc = 0u;
    uint32_t arr = kTimClockHz / freq_hz;
    while (arr > 0xFFFFu) { ++psc; arr = kTimClockHz / (freq_hz * (psc + 1u)); }
    if (arr > 0u) { arr -= 1u; }

    TIM2_CR1   = 0u;
    TIM2_PSC   = psc;
    TIM2_ARR   = arr;
    TIM2_CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
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

// ----------------------------------------------------------------------------
// TIM15 - ETB PWM (PE5 CH1, AF4)
// ----------------------------------------------------------------------------

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
    TIM15_BDTR = (1u << 15);
    TIM15_EGR = 1u;
    TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void tim15_etb_set_duty_x10(uint16_t duty_pct_x10) noexcept {
    const uint32_t arr = TIM15_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    TIM15_CCR1 = ccr;
}

// ----------------------------------------------------------------------------
// ISR Handlers
// ----------------------------------------------------------------------------

/**
 * @brief TIM5_IRQHandler — CKP (CH1) + CMP (CH2) + event dispatcher (CH3)
 */
extern "C" void TIM5_IRQHandler(void) {
    uint32_t sr = TIM5_SR;
    if (sr & TIM_SR_CC1IF) {
        TIM5_SR = ~TIM_SR_CC1IF;
        ems::drv::ckp_tim5_ch1_isr();
    }
    sr = TIM5_SR;
    if (sr & TIM_SR_CC2IF) {
        TIM5_SR = ~TIM_SR_CC2IF;
        ems::drv::ckp_tim5_ch2_isr();
    }
    sr = TIM5_SR;
    if (sr & TIM_SR_CC3IF) {
        TIM5_SR = ~TIM_SR_CC3IF;
        ecu_sched_evt_dispatch();
    }
}

} // namespace ems::hal

// ----------------------------------------------------------------------------
// TIM15 - PWM para Borboleta Eletronica (ETB) com dead-time (PE5 CH1, AF4)
// ----------------------------------------------------------------------------

void timer_etb_pwm_init(void) {
    RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN;

    gpio_set_af(&GPIOE_MODER, &GPIOE_AFRL, &GPIOE_AFRH, &GPIOE_OSPEEDR, 5u, GPIO_AF4);

    TIM15_CR1 = 0u;
    TIM15_PSC = 0u;
    TIM15_ARR = 12499u;
    TIM15_EGR = TIM_EGR_UG;

    TIM15_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;

    TIM15_BDTR = TIM_BDTR_MOE
              | (50u << 0u);

    TIM15_CCER = TIM_CCER_CC1E;

    TIM15_CCR1 = 0u;

    TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void timer_etb_set_duty(uint16_t duty) {
    if (duty > 1023u) { duty = 1023u; }
    uint32_t ccr1_val = ((uint32_t)duty * 12499u) / 1023u;
    TIM15_CCR1 = (uint16_t)ccr1_val;
}

#else  // EMS_HOST_TEST -------------------------------------------------------

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

void timer_etb_pwm_init(void) {}
void timer_etb_set_duty(uint16_t duty) { (void)duty; }

#endif  // EMS_HOST_TEST
