#pragma once
/**
 * @file hal/stm32h562/regs.h
 * @brief Definições de registradores do STM32H562RGT6 (ARM Cortex-M33 @ 250 MHz)
 *
 * Cobertura: RCC, GPIO, TIM1/TIM3/TIM4/TIM5/TIM6, ADC1/ADC2, FDCAN1,
 *            USART1, IWDG, SysTick (via CMSIS), Flash OPTCR/SR, NVIC helpers.
 *
 * Todos os endereços baseados em:
 *   RM0481 — STM32H562/H563/H573 Reference Manual Rev 2 (ST Microelectronics)
 */

#include <cstdint>

// ─── Macro de acesso a registrador ────────────────────────────────────────────
#define STM32_REG32(addr)  (*reinterpret_cast<volatile uint32_t*>(addr))
#define STM32_REG16(addr)  (*reinterpret_cast<volatile uint16_t*>(addr))

// ─── Bases dos periféricos (RM0481 §3.3, Memory Map) ─────────────────────────
// AHB1
#define GPIOA_BASE   0x42020000UL
#define GPIOB_BASE   0x42020400UL
#define GPIOC_BASE   0x42020800UL
#define GPIOD_BASE   0x42020C00UL
#define GPIOE_BASE   0x42021000UL

// APB2
#define TIM1_BASE    0x40012C00UL
#define USART1_BASE  0x40013800UL

// APB1
#define TIM3_BASE    0x40000400UL
#define TIM4_BASE    0x40000800UL
#define TIM5_BASE    0x40000C00UL
#define TIM6_BASE    0x40001000UL
#define USART2_BASE  0x40004400UL

// AHB1
#define ADC1_BASE    0x42028000UL
#define ADC2_BASE    0x42028100UL
#define ADC12_COMMON 0x42028300UL

// APB1 — FDCAN
#define FDCAN1_BASE  0x4000A400UL
#define FDCAN_SRAM   0x4000AC00UL   // Message RAM

// APB1 — IWDG
#define IWDG_BASE    0x40003000UL

// RCC
#define RCC_BASE     0x44020C00UL

// Flash
#define FLASH_BASE   0x40022000UL

// ─── RCC (Reset and Clock Control) ───────────────────────────────────────────
#define RCC_CR          STM32_REG32(RCC_BASE + 0x000)
#define RCC_CFGR1       STM32_REG32(RCC_BASE + 0x01C)
#define RCC_CFGR2       STM32_REG32(RCC_BASE + 0x020)
#define RCC_PLL1CFGR    STM32_REG32(RCC_BASE + 0x028)
#define RCC_PLL1DIVR    STM32_REG32(RCC_BASE + 0x034)
#define RCC_AHB1ENR     STM32_REG32(RCC_BASE + 0x0E8)
#define RCC_AHB2ENR1    STM32_REG32(RCC_BASE + 0x0EC)
#define RCC_AHB2ENR2    STM32_REG32(RCC_BASE + 0x0F0)
#define RCC_APB1LENR    STM32_REG32(RCC_BASE + 0x0F4)
#define RCC_APB2ENR     STM32_REG32(RCC_BASE + 0x100)

// RCC_CR bits
#define RCC_CR_HSEON    (1u << 16)
#define RCC_CR_HSERDY   (1u << 17)
#define RCC_CR_PLL1ON   (1u << 24)
#define RCC_CR_PLL1RDY  (1u << 25)

// RCC_CFGR1 bits (SW[2:0] / SWS[2:0])
#define RCC_CFGR1_SW_PLL1  (3u << 0)
#define RCC_CFGR1_SWS_PLL1 (3u << 3)

// Clock enables
#define RCC_AHB2ENR1_GPIOAEN  (1u << 0)
#define RCC_AHB2ENR1_GPIOBEN  (1u << 1)
#define RCC_AHB2ENR1_GPIOCEN  (1u << 2)
#define RCC_AHB2ENR1_GPIODEN  (1u << 3)
#define RCC_AHB2ENR1_GPIOEEN  (1u << 4)
#define RCC_AHB1ENR_ADC12EN   (1u << 5)
#define RCC_APB1LENR_TIM3EN   (1u << 1)
#define RCC_APB1LENR_TIM4EN   (1u << 2)
#define RCC_APB1LENR_TIM5EN   (1u << 3)
#define RCC_APB1LENR_TIM6EN   (1u << 4)
#define RCC_APB1LENR_FDCAN1EN (1u << 9)
#define RCC_APB1LENR_USART2EN (1u << 17)
#define RCC_APB1LENR_IWDGEN   (1u << 12)
#define RCC_APB2ENR_TIM1EN    (1u << 11)
#define RCC_APB2ENR_USART1EN  (1u << 14)

// ─── GPIO (RM0481 §17) ────────────────────────────────────────────────────────
// Offsets comuns a GPIOA..GPIOE
#define GPIO_MODER_OFF   0x00UL
#define GPIO_OTYPER_OFF  0x04UL
#define GPIO_OSPEEDR_OFF 0x08UL
#define GPIO_PUPDR_OFF   0x0CUL
#define GPIO_IDR_OFF     0x10UL
#define GPIO_ODR_OFF     0x14UL
#define GPIO_BSRR_OFF    0x18UL
#define GPIO_AFRL_OFF    0x20UL
#define GPIO_AFRH_OFF    0x24UL

#define GPIOA_MODER   STM32_REG32(GPIOA_BASE + GPIO_MODER_OFF)
#define GPIOA_OSPEEDR STM32_REG32(GPIOA_BASE + GPIO_OSPEEDR_OFF)
#define GPIOA_AFRL    STM32_REG32(GPIOA_BASE + GPIO_AFRL_OFF)
#define GPIOA_AFRH    STM32_REG32(GPIOA_BASE + GPIO_AFRH_OFF)
#define GPIOA_IDR     STM32_REG32(GPIOA_BASE + GPIO_IDR_OFF)

#define GPIOB_MODER   STM32_REG32(GPIOB_BASE + GPIO_MODER_OFF)
#define GPIOB_OSPEEDR STM32_REG32(GPIOB_BASE + GPIO_OSPEEDR_OFF)
#define GPIOB_AFRL    STM32_REG32(GPIOB_BASE + GPIO_AFRL_OFF)
#define GPIOB_AFRH    STM32_REG32(GPIOB_BASE + GPIO_AFRH_OFF)

#define GPIOE_MODER   STM32_REG32(GPIOE_BASE + GPIO_MODER_OFF)
#define GPIOE_OSPEEDR STM32_REG32(GPIOE_BASE + GPIO_OSPEEDR_OFF)
#define GPIOE_AFRH    STM32_REG32(GPIOE_BASE + GPIO_AFRH_OFF)

// GPIO MODER values (2 bits por pino)
#define GPIO_MODER_INPUT  0x0u
#define GPIO_MODER_OUTPUT 0x1u
#define GPIO_MODER_AF     0x2u
#define GPIO_MODER_ANALOG 0x3u

// GPIO OSPEEDR values
#define GPIO_OSPEED_LOW    0x0u
#define GPIO_OSPEED_MEDIUM 0x1u
#define GPIO_OSPEED_HIGH   0x2u
#define GPIO_OSPEED_VHIGH  0x3u

// Alternate Functions relevantes:
//   AF1  = TIM1/TIM2          AF2  = TIM3/TIM4/TIM5
//   AF7  = USART1/USART2/USART3
//   AF9  = FDCAN1/FDCAN2
#define GPIO_AF1   1u
#define GPIO_AF2   2u
#define GPIO_AF7   7u
#define GPIO_AF9   9u

// Inline: configura pino como AF
static inline void gpio_set_af(volatile uint32_t* moder,
                                volatile uint32_t* afr_l,
                                volatile uint32_t* afr_h,
                                volatile uint32_t* ospeedr,
                                uint8_t pin, uint8_t af) noexcept {
    // MODER = AF (10b)
    *moder = (*moder & ~(3u << (pin * 2u))) | (GPIO_MODER_AF << (pin * 2u));
    // OSPEEDR = very high
    *ospeedr = (*ospeedr & ~(3u << (pin * 2u))) | (GPIO_OSPEED_VHIGH << (pin * 2u));
    // AFR
    if (pin < 8u) {
        *afr_l = (*afr_l & ~(0xFu << (pin * 4u))) | (static_cast<uint32_t>(af) << (pin * 4u));
    } else {
        uint8_t p = pin - 8u;
        *afr_h = (*afr_h & ~(0xFu << (p * 4u))) | (static_cast<uint32_t>(af) << (p * 4u));
    }
}

// Inline: configura pino como ANALOG (ADC)
static inline void gpio_set_analog(volatile uint32_t* moder, uint8_t pin) noexcept {
    *moder = (*moder & ~(3u << (pin * 2u))) | (GPIO_MODER_ANALOG << (pin * 2u));
}

// ─── TIM genérico (offsets, RM0481 §33) ──────────────────────────────────────
#define TIM_CR1_OFF   0x00UL   // Control register 1
#define TIM_CR2_OFF   0x04UL   // Control register 2
#define TIM_SMCR_OFF  0x08UL   // Slave mode control
#define TIM_DIER_OFF  0x0CUL   // DMA/interrupt enable
#define TIM_SR_OFF    0x10UL   // Status register
#define TIM_EGR_OFF   0x14UL   // Event generation
#define TIM_CCMR1_OFF 0x18UL   // Capture/compare mode 1
#define TIM_CCMR2_OFF 0x1CUL   // Capture/compare mode 2
#define TIM_CCER_OFF  0x20UL   // Capture/compare enable
#define TIM_CNT_OFF   0x24UL   // Counter
#define TIM_PSC_OFF   0x28UL   // Prescaler
#define TIM_ARR_OFF   0x2CUL   // Auto-reload
#define TIM_CCR1_OFF  0x34UL   // Capture/compare 1
#define TIM_CCR2_OFF  0x38UL   // Capture/compare 2
#define TIM_CCR3_OFF  0x3CUL   // Capture/compare 3
#define TIM_CCR4_OFF  0x40UL   // Capture/compare 4
#define TIM_BDTR_OFF  0x44UL   // Break and dead-time (TIM1 only)
#define TIM_CR2_MMS_UPDATE (2u << 4)   // MMS = 010 → Update event → TRGO

// TIM_CR1 bits
#define TIM_CR1_CEN   (1u << 0)   // Counter enable
#define TIM_CR1_UDIS  (1u << 1)   // Update disable
#define TIM_CR1_URS   (1u << 2)   // Update request source
#define TIM_CR1_ARPE  (1u << 7)   // Auto-reload preload enable

// TIM_SR bits
#define TIM_SR_UIF    (1u << 0)   // Update interrupt flag
#define TIM_SR_CC1IF  (1u << 1)   // Capture/compare 1
#define TIM_SR_CC2IF  (1u << 2)   // Capture/compare 2
#define TIM_SR_CC3IF  (1u << 3)   // Capture/compare 3
#define TIM_SR_CC4IF  (1u << 4)   // Capture/compare 4
#define TIM_SR_CC1OF  (1u << 9)   // Overcapture 1

// TIM_DIER bits
#define TIM_DIER_UIE  (1u << 0)
#define TIM_DIER_CC1IE (1u << 1)
#define TIM_DIER_CC2IE (1u << 2)
#define TIM_DIER_CC3IE (1u << 3)
#define TIM_DIER_CC4IE (1u << 4)

// TIM_CCMR1 — Capture/Compare Mode (CH1 e CH2)
// Input Capture
#define TIM_CCMR1_CC1S_TI1  (1u << 0)   // CH1 → IC, mapeado em TI1
#define TIM_CCMR1_CC2S_TI2  (1u << 8)   // CH2 → IC, mapeado em TI2
#define TIM_CCMR1_IC1F_NONE (0u << 4)   // sem filtro
#define TIM_CCMR1_IC2F_NONE (0u << 12)
// Output Compare
#define TIM_CCMR1_OC1M_FROZEN  (0u << 4)
#define TIM_CCMR1_OC1M_ACTIVE  (1u << 4)
#define TIM_CCMR1_OC1M_INACTIVE (2u << 4)
#define TIM_CCMR1_OC1M_PWM1    (6u << 4)
#define TIM_CCMR1_OC2M_PWM1    (6u << 12)
#define TIM_CCMR1_OC1PE        (1u << 3)   // preload enable CH1
#define TIM_CCMR1_OC2PE        (1u << 11)

// TIM_CCMR2 — CH3 e CH4
#define TIM_CCMR2_OC3M_PWM1   (6u << 4)
#define TIM_CCMR2_OC4M_PWM1   (6u << 12)
#define TIM_CCMR2_OC3PE       (1u << 3)
#define TIM_CCMR2_OC4PE       (1u << 11)

// TIM_CCER bits (enable + polarity per channel)
#define TIM_CCER_CC1E   (1u << 0)
#define TIM_CCER_CC1P   (1u << 1)   // polarity (0=active high, 1=active low)
#define TIM_CCER_CC1NP  (1u << 3)   // input capture: 0=rising, 1=falling, 11=both
#define TIM_CCER_CC2E   (1u << 4)
#define TIM_CCER_CC2P   (1u << 5)
#define TIM_CCER_CC3E   (1u << 8)
#define TIM_CCER_CC4E   (1u << 12)

// TIM5 — 32-bit free-running counter (CKP input capture)
#define TIM5_CR1   STM32_REG32(TIM5_BASE + TIM_CR1_OFF)
#define TIM5_DIER  STM32_REG32(TIM5_BASE + TIM_DIER_OFF)
#define TIM5_SR    STM32_REG32(TIM5_BASE + TIM_SR_OFF)
#define TIM5_CCMR1 STM32_REG32(TIM5_BASE + TIM_CCMR1_OFF)
#define TIM5_CCER  STM32_REG32(TIM5_BASE + TIM_CCER_OFF)
#define TIM5_CNT   STM32_REG32(TIM5_BASE + TIM_CNT_OFF)
#define TIM5_PSC   STM32_REG32(TIM5_BASE + TIM_PSC_OFF)
#define TIM5_ARR   STM32_REG32(TIM5_BASE + TIM_ARR_OFF)
#define TIM5_CCR1  STM32_REG32(TIM5_BASE + TIM_CCR1_OFF)  // CKP timestamp travado
#define TIM5_CCR2  STM32_REG32(TIM5_BASE + TIM_CCR2_OFF)  // CMP timestamp travado
#define TIM5_EGR   STM32_REG32(TIM5_BASE + TIM_EGR_OFF)

// TIM1 — advanced timer (output compare ignição/injeção)
#define TIM1_CR1   STM32_REG32(TIM1_BASE + TIM_CR1_OFF)
#define TIM1_DIER  STM32_REG32(TIM1_BASE + TIM_DIER_OFF)
#define TIM1_SR    STM32_REG32(TIM1_BASE + TIM_SR_OFF)
#define TIM1_EGR   STM32_REG32(TIM1_BASE + TIM_EGR_OFF)
#define TIM1_CCMR1 STM32_REG32(TIM1_BASE + TIM_CCMR1_OFF)
#define TIM1_CCMR2 STM32_REG32(TIM1_BASE + TIM_CCMR2_OFF)
#define TIM1_CCER  STM32_REG32(TIM1_BASE + TIM_CCER_OFF)
#define TIM1_CNT   STM32_REG32(TIM1_BASE + TIM_CNT_OFF)
#define TIM1_PSC   STM32_REG32(TIM1_BASE + TIM_PSC_OFF)
#define TIM1_ARR   STM32_REG32(TIM1_BASE + TIM_ARR_OFF)
#define TIM1_CCR1  STM32_REG32(TIM1_BASE + TIM_CCR1_OFF)
#define TIM1_CCR2  STM32_REG32(TIM1_BASE + TIM_CCR2_OFF)
#define TIM1_CCR3  STM32_REG32(TIM1_BASE + TIM_CCR3_OFF)
#define TIM1_CCR4  STM32_REG32(TIM1_BASE + TIM_CCR4_OFF)
#define TIM1_BDTR  STM32_REG32(TIM1_BASE + TIM_BDTR_OFF)

// TIM3 — PWM (IACV CH1 + Wastegate CH2)
#define TIM3_CR1   STM32_REG32(TIM3_BASE + TIM_CR1_OFF)
#define TIM3_CR2   STM32_REG32(TIM3_BASE + TIM_CR2_OFF)
#define TIM3_SR    STM32_REG32(TIM3_BASE + TIM_SR_OFF)
#define TIM3_CCMR1 STM32_REG32(TIM3_BASE + TIM_CCMR1_OFF)
#define TIM3_CCER  STM32_REG32(TIM3_BASE + TIM_CCER_OFF)
#define TIM3_CNT   STM32_REG32(TIM3_BASE + TIM_CNT_OFF)
#define TIM3_PSC   STM32_REG32(TIM3_BASE + TIM_PSC_OFF)
#define TIM3_ARR   STM32_REG32(TIM3_BASE + TIM_ARR_OFF)
#define TIM3_CCR1  STM32_REG32(TIM3_BASE + TIM_CCR1_OFF)
#define TIM3_CCR2  STM32_REG32(TIM3_BASE + TIM_CCR2_OFF)
#define TIM3_EGR   STM32_REG32(TIM3_BASE + TIM_EGR_OFF)

// TIM4 — PWM (VVT Exhaust CH1 + VVT Intake CH2)
#define TIM4_CR1   STM32_REG32(TIM4_BASE + TIM_CR1_OFF)
#define TIM4_CR2   STM32_REG32(TIM4_BASE + TIM_CR2_OFF)
#define TIM4_SR    STM32_REG32(TIM4_BASE + TIM_SR_OFF)
#define TIM4_CCMR1 STM32_REG32(TIM4_BASE + TIM_CCMR1_OFF)
#define TIM4_CCER  STM32_REG32(TIM4_BASE + TIM_CCER_OFF)
#define TIM4_CNT   STM32_REG32(TIM4_BASE + TIM_CNT_OFF)
#define TIM4_PSC   STM32_REG32(TIM4_BASE + TIM_PSC_OFF)
#define TIM4_ARR   STM32_REG32(TIM4_BASE + TIM_ARR_OFF)
#define TIM4_CCR1  STM32_REG32(TIM4_BASE + TIM_CCR1_OFF)
#define TIM4_CCR2  STM32_REG32(TIM4_BASE + TIM_CCR2_OFF)
#define TIM4_EGR   STM32_REG32(TIM4_BASE + TIM_EGR_OFF)

// TIM6 — basic timer (ADC TRGO trigger)
#define TIM6_CR1   STM32_REG32(TIM6_BASE + TIM_CR1_OFF)
#define TIM6_CR2   STM32_REG32(TIM6_BASE + TIM_CR2_OFF)
#define TIM6_SR    STM32_REG32(TIM6_BASE + TIM_SR_OFF)
#define TIM6_PSC   STM32_REG32(TIM6_BASE + TIM_PSC_OFF)
#define TIM6_ARR   STM32_REG32(TIM6_BASE + TIM_ARR_OFF)
#define TIM6_EGR   STM32_REG32(TIM6_BASE + TIM_EGR_OFF)

// ─── ADC (RM0481 §25) ─────────────────────────────────────────────────────────
#define ADC_ISR_OFF    0x00UL
#define ADC_IER_OFF    0x04UL
#define ADC_CR_OFF     0x08UL
#define ADC_CFGR1_OFF  0x0CUL
#define ADC_CFGR2_OFF  0x10UL
#define ADC_SMPR1_OFF  0x14UL
#define ADC_SMPR2_OFF  0x18UL
#define ADC_SQR1_OFF   0x30UL
#define ADC_SQR2_OFF   0x34UL
#define ADC_SQR3_OFF   0x38UL
#define ADC_SQR4_OFF   0x3CUL
#define ADC_DR_OFF     0x40UL
#define ADC_OFR1_OFF   0x60UL

// Common registers (ADC1+ADC2 shared)
#define ADC12_CCR_OFF  0x08UL

#define ADC1_ISR   STM32_REG32(ADC1_BASE + ADC_ISR_OFF)
#define ADC1_CR    STM32_REG32(ADC1_BASE + ADC_CR_OFF)
#define ADC1_CFGR1 STM32_REG32(ADC1_BASE + ADC_CFGR1_OFF)
#define ADC1_CFGR2 STM32_REG32(ADC1_BASE + ADC_CFGR2_OFF)
#define ADC1_SMPR1 STM32_REG32(ADC1_BASE + ADC_SMPR1_OFF)
#define ADC1_SMPR2 STM32_REG32(ADC1_BASE + ADC_SMPR2_OFF)
#define ADC1_SQR1  STM32_REG32(ADC1_BASE + ADC_SQR1_OFF)
#define ADC1_SQR2  STM32_REG32(ADC1_BASE + ADC_SQR2_OFF)
#define ADC1_DR    STM32_REG32(ADC1_BASE + ADC_DR_OFF)

#define ADC2_ISR   STM32_REG32(ADC2_BASE + ADC_ISR_OFF)
#define ADC2_CR    STM32_REG32(ADC2_BASE + ADC_CR_OFF)
#define ADC2_CFGR1 STM32_REG32(ADC2_BASE + ADC_CFGR1_OFF)
#define ADC2_CFGR2 STM32_REG32(ADC2_BASE + ADC_CFGR2_OFF)
#define ADC2_SMPR1 STM32_REG32(ADC2_BASE + ADC_SMPR1_OFF)
#define ADC2_SMPR2 STM32_REG32(ADC2_BASE + ADC_SMPR2_OFF)
#define ADC2_SQR1  STM32_REG32(ADC2_BASE + ADC_SQR1_OFF)
#define ADC2_DR    STM32_REG32(ADC2_BASE + ADC_DR_OFF)

#define ADC12_CCR  STM32_REG32(ADC12_COMMON + ADC12_CCR_OFF)

// ADC_CR bits
#define ADC_CR_ADEN    (1u << 0)
#define ADC_CR_ADDIS   (1u << 1)
#define ADC_CR_ADSTART (1u << 2)
#define ADC_CR_ADSTP   (1u << 4)
#define ADC_CR_ADCAL   (1u << 31)
#define ADC_CR_ADCALDIF (1u << 30)
#define ADC_CR_DEEPPWD (1u << 29)
#define ADC_CR_ADVREGEN (1u << 28)

// ADC_ISR bits
#define ADC_ISR_ADRDY  (1u << 0)
#define ADC_ISR_EOC    (1u << 2)
#define ADC_ISR_EOS    (1u << 3)

// ADC_IER — interrupt enable register (offset 0x04)
#define ADC_IER_OFF    0x04UL
#define ADC_IER_EOCIE  (1u << 2)   // End of conversion interrupt enable
#define ADC_IER_EOSIE  (1u << 3)   // End of sequence interrupt enable
#define ADC1_IER  STM32_REG32(ADC1_BASE + ADC_IER_OFF)
#define ADC2_IER  STM32_REG32(ADC2_BASE + ADC_IER_OFF)

// ADC_CFGR1 bits
#define ADC_CFGR1_RES_12BIT  (0u << 2)
#define ADC_CFGR1_RES_10BIT  (1u << 2)
#define ADC_CFGR1_EXTEN_RISING (1u << 10)
// EXTSEL[4:0] = 0b01101 → TIM6_TRGO (ver RM0481 Tab.125)
#define ADC_CFGR1_EXTSEL_TIM6_TRGO (13u << 5)
#define ADC_CFGR1_CONT   (1u << 13)
#define ADC_CFGR1_DISCEN (1u << 16)

// ADC12_CCR bits
#define ADC12_CCR_PRESC_DIV4 (2u << 18)  // PRESC[3:0] = 0010 → /4

// ─── FDCAN1 (RM0481 §51) ──────────────────────────────────────────────────────
#define FDCAN_CCCR_OFF   0x018UL   // CC Control Register
#define FDCAN_NBTP_OFF   0x01CUL   // Nominal Bit Timing
#define FDCAN_DBTP_OFF   0x00CUL   // Data Bit Timing
#define FDCAN_RXGFC_OFF  0x080UL   // Global Filter Config
#define FDCAN_TXBC_OFF   0x0C4UL   // TX Buffer Config
#define FDCAN_TXFQS_OFF  0x0C8UL   // TX FIFO/Queue Status
#define FDCAN_TXBAR_OFF  0x0D0UL   // TX Buffer Add Request
#define FDCAN_RXF0C_OFF  0x0A0UL   // RX FIFO0 Config
#define FDCAN_RXF0S_OFF  0x0A4UL   // RX FIFO0 Status
#define FDCAN_RXF0A_OFF  0x0A8UL   // RX FIFO0 Acknowledge
#define FDCAN_IR_OFF     0x050UL   // Interrupt Register
#define FDCAN_IE_OFF     0x054UL   // Interrupt Enable
#define FDCAN_ILE_OFF    0x05CUL   // Interrupt Line Enable
#define FDCAN_SHPAM_OFF  0x094UL   // Standard ID filter area

#define FDCAN1_CCCR  STM32_REG32(FDCAN1_BASE + FDCAN_CCCR_OFF)
#define FDCAN1_NBTP  STM32_REG32(FDCAN1_BASE + FDCAN_NBTP_OFF)
#define FDCAN1_RXGFC STM32_REG32(FDCAN1_BASE + FDCAN_RXGFC_OFF)
#define FDCAN1_TXBC  STM32_REG32(FDCAN1_BASE + FDCAN_TXBC_OFF)
#define FDCAN1_TXFQS STM32_REG32(FDCAN1_BASE + FDCAN_TXFQS_OFF)
#define FDCAN1_TXBAR STM32_REG32(FDCAN1_BASE + FDCAN_TXBAR_OFF)
#define FDCAN1_RXF0C STM32_REG32(FDCAN1_BASE + FDCAN_RXF0C_OFF)
#define FDCAN1_RXF0S STM32_REG32(FDCAN1_BASE + FDCAN_RXF0S_OFF)
#define FDCAN1_RXF0A STM32_REG32(FDCAN1_BASE + FDCAN_RXF0A_OFF)
#define FDCAN1_IR    STM32_REG32(FDCAN1_BASE + FDCAN_IR_OFF)
#define FDCAN1_IE    STM32_REG32(FDCAN1_BASE + FDCAN_IE_OFF)

// FDCAN_CCCR bits
#define FDCAN_CCCR_INIT  (1u << 0)
#define FDCAN_CCCR_CCE   (1u << 1)    // Config Change Enable
#define FDCAN_CCCR_ASM   (1u << 2)    // Restricted Operation Mode
#define FDCAN_CCCR_CSA   (1u << 3)    // Clock Stop Acknowledge
#define FDCAN_CCCR_CSR   (1u << 4)    // Clock Stop Request
#define FDCAN_CCCR_MON   (1u << 5)    // Bus Monitoring Mode
#define FDCAN_CCCR_DAR   (1u << 6)    // Disable Automatic Retransmission
#define FDCAN_CCCR_TEST  (1u << 7)    // Test Mode Enable
#define FDCAN_CCCR_FDOE  (1u << 8)    // FD Operation Enable
#define FDCAN_CCCR_BRSE  (1u << 9)    // Bit Rate Switch Enable

// ─── USART1 / USART2 (RM0481 §49) ────────────────────────────────────────────
#define USART_CR1_OFF  0x00UL
#define USART_CR2_OFF  0x04UL
#define USART_CR3_OFF  0x08UL
#define USART_BRR_OFF  0x0CUL
#define USART_ISR_OFF  0x1CUL
#define USART_ICR_OFF  0x20UL
#define USART_RDR_OFF  0x24UL
#define USART_TDR_OFF  0x28UL

#define USART1_CR1 STM32_REG32(USART1_BASE + USART_CR1_OFF)
#define USART1_CR2 STM32_REG32(USART1_BASE + USART_CR2_OFF)
#define USART1_CR3 STM32_REG32(USART1_BASE + USART_CR3_OFF)
#define USART1_BRR STM32_REG32(USART1_BASE + USART_BRR_OFF)
#define USART1_ISR STM32_REG32(USART1_BASE + USART_ISR_OFF)
#define USART1_ICR STM32_REG32(USART1_BASE + USART_ICR_OFF)
#define USART1_RDR STM32_REG32(USART1_BASE + USART_RDR_OFF)
#define USART1_TDR STM32_REG32(USART1_BASE + USART_TDR_OFF)

// USART2 — registradores (reservado, não usado para TunerStudio — PA2/PA3 são ADC)
#define USART2_CR1 STM32_REG32(USART2_BASE + USART_CR1_OFF)
#define USART2_CR2 STM32_REG32(USART2_BASE + USART_CR2_OFF)
#define USART2_CR3 STM32_REG32(USART2_BASE + USART_CR3_OFF)
#define USART2_BRR STM32_REG32(USART2_BASE + USART_BRR_OFF)
#define USART2_ISR STM32_REG32(USART2_BASE + USART_ISR_OFF)
#define USART2_ICR STM32_REG32(USART2_BASE + USART_ICR_OFF)
#define USART2_RDR STM32_REG32(USART2_BASE + USART_RDR_OFF)
#define USART2_TDR STM32_REG32(USART2_BASE + USART_TDR_OFF)

// USART3 — TunerStudio UART (PB10=TX, PB11=RX, AF7) — PA2/PA3 livres para ADC1 MAP/MAF
#define USART3_BASE  0x40004800UL
#define RCC_APB1LENR_USART3EN  (1u << 18)
#define USART3_CR1 STM32_REG32(USART3_BASE + USART_CR1_OFF)
#define USART3_CR2 STM32_REG32(USART3_BASE + USART_CR2_OFF)
#define USART3_CR3 STM32_REG32(USART3_BASE + USART_CR3_OFF)
#define USART3_BRR STM32_REG32(USART3_BASE + USART_BRR_OFF)
#define USART3_ISR STM32_REG32(USART3_BASE + USART_ISR_OFF)
#define USART3_ICR STM32_REG32(USART3_BASE + USART_ICR_OFF)
#define USART3_RDR STM32_REG32(USART3_BASE + USART_RDR_OFF)
#define USART3_TDR STM32_REG32(USART3_BASE + USART_TDR_OFF)

// USART_CR1 bits
#define USART_CR1_UE    (1u << 0)
#define USART_CR1_RE    (1u << 2)
#define USART_CR1_TE    (1u << 3)
#define USART_CR1_FIFOEN (1u << 29)

// USART_ISR bits
#define USART_ISR_RXNE  (1u << 5)   // RX not empty (data available)
#define USART_ISR_TC    (1u << 6)   // Transmission complete
#define USART_ISR_TXE   (1u << 7)   // TX register empty

// ─── IWDG (Independent Watchdog, RM0481 §35) ─────────────────────────────────
#define IWDG_KR_OFF   0x00UL
#define IWDG_PR_OFF   0x04UL
#define IWDG_RLR_OFF  0x08UL
#define IWDG_SR_OFF   0x0CUL

#define IWDG_KR  STM32_REG32(IWDG_BASE + IWDG_KR_OFF)
#define IWDG_PR  STM32_REG32(IWDG_BASE + IWDG_PR_OFF)
#define IWDG_RLR STM32_REG32(IWDG_BASE + IWDG_RLR_OFF)
#define IWDG_SR  STM32_REG32(IWDG_BASE + IWDG_SR_OFF)

#define IWDG_KR_REFRESH  0xAAAAu
#define IWDG_KR_ACCESS   0x5555u
#define IWDG_KR_START    0xCCCCu

// IWDG_PR: prescaler para ~100ms a 32 kHz LSI
// Prescaler /32 → 32000/32 = 1000 Hz → RLR = 100 → 100 ms
#define IWDG_PR_DIV32    0x3u
#define IWDG_RLR_100MS   99u

// ─── Flash (RM0481 §7) ───────────────────────────────────────────────────────
#define FLASH_ACR_OFF     0x000UL
#define FLASH_OPTCR_OFF   0x004UL
#define FLASH_SR1_OFF     0x020UL   // Flash Bank1 status
#define FLASH_SR2_OFF     0x120UL   // Flash Bank2 status
#define FLASH_CR1_OFF     0x028UL
#define FLASH_CR2_OFF     0x128UL
#define FLASH_CCR1_OFF    0x030UL
#define FLASH_CCR2_OFF    0x130UL
#define FLASH_KEYR1_OFF   0x004UL
#define FLASH_KEYR2_OFF   0x104UL

#define FLASH_ACR  STM32_REG32(FLASH_BASE + FLASH_ACR_OFF)
#define FLASH_SR2  STM32_REG32(FLASH_BASE + FLASH_SR2_OFF)
#define FLASH_CR2  STM32_REG32(FLASH_BASE + FLASH_CR2_OFF)
#define FLASH_CCR2 STM32_REG32(FLASH_BASE + FLASH_CCR2_OFF)
#define FLASH_KEYR2 STM32_REG32(FLASH_BASE + FLASH_KEYR2_OFF)

// Flash_SR bits
#define FLASH_SR_BSY   (1u << 0)
#define FLASH_SR_WBNE  (1u << 1)   // Write buffer not empty
#define FLASH_SR_DBNE  (1u << 3)   // Data buffer not empty
#define FLASH_SR_EOP   (1u << 16)  // End of program
#define FLASH_SR_PGSERR (1u << 18) // Program sequence error
#define FLASH_SR_WRPERR (1u << 17) // Write protection error

// Flash_CR bits
#define FLASH_CR_LOCK  (1u << 0)
#define FLASH_CR_PG    (1u << 1)   // Programming enable
#define FLASH_CR_SER   (1u << 2)   // Sector erase
#define FLASH_CR_BER   (1u << 3)   // Bank erase
#define FLASH_CR_STRT  (1u << 5)   // Erase start
#define FLASH_CR_SNB_SHIFT 6       // Sector number [3:0] @ bits [9:6]

// Flash_ACR: latency / prefetch
#define FLASH_ACR_LATENCY_5WS (5u << 0)
#define FLASH_ACR_PRFTEN      (1u << 8)
#define FLASH_ACR_ICEN        (1u << 9)
#define FLASH_ACR_DCEN        (1u << 10)

// Flash Bank2 base address
#define FLASH_BANK2_BASE  0x08100000UL
#define FLASH_SECTOR_SIZE 0x00002000UL  // 8 KB por setor no H562

// ─── NVIC helpers (ARM Cortex-M33 NVIC — idêntico ao M4) ────────────────────
#define NVIC_ISER_BASE  0xE000E100UL
#define NVIC_IPR_BASE   0xE000E400UL
#define NVIC_ICPR_BASE  0xE000E280UL

static inline void nvic_enable_irq(uint8_t irq) noexcept {
    STM32_REG32(NVIC_ISER_BASE + (irq / 32u) * 4u) = (1u << (irq % 32u));
}

static inline void nvic_set_priority(uint8_t irq, uint8_t prio) noexcept {
    // Prioridade nos 4 bits superiores do byte IPR (como no Kinetis)
    volatile uint32_t* ipr = reinterpret_cast<volatile uint32_t*>(
        NVIC_IPR_BASE + (irq / 4u) * 4u);
    uint32_t sh = (irq % 4u) * 8u;
    *ipr = (*ipr & ~(0xFFu << sh)) | (static_cast<uint32_t>(prio) << (sh + 4u));
}

// IRQ numbers — STM32H562 (RM0481 §Table 87 / cmsis-device-h5 stm32h562xx.h)
// Valores confirmados contra TIM5_IRQn=48, TIM1_CC_IRQn=44, etc.
#define IRQ_TIM5         48u   // TIM5 global (CKP input capture)
#define IRQ_TIM1_CC      44u   // TIM1 capture/compare (ignição/injeção)
#define IRQ_TIM3         46u   // TIM3 (PWM IACV/wastegate — não usa IRQ)
#define IRQ_TIM4         47u   // TIM4 (PWM VVT — não usa IRQ)
#define IRQ_ADC1         37u   // ADC1 (IRQ separado do ADC2)
#define IRQ_ADC2         69u   // ADC2 (IRQ separado do ADC1)
#define IRQ_FDCAN1_IT0   39u   // FDCAN1 interrupt line 0
// SysTick não usa NVIC — configurado via SCB->SHP diretamente (ARM core)
