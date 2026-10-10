/**
 * @file hal/out_pins.cpp
 * INJ/IGN GPIO init + safe de-assert (relocated from ecu_sched_outputs_safe_early).
 */
#include "hal/out_pins.h"
#include "hal/coil_oc.h"

#if defined(EMS_HOST_TEST)

namespace ems::hal::out_pins_host {
uint32_t rcc_ahb2enr1 = 0u;
uint32_t gpioa_moder = 0u, gpiob_moder = 0u, gpioc_moder = 0u, gpioe_moder = 0u;
uint32_t gpioa_otyper = 0u, gpiob_otyper = 0u, gpioc_otyper = 0u, gpioe_otyper = 0u;
uint32_t gpioa_pupdr = 0u, gpiob_pupdr = 0u, gpioc_pupdr = 0u, gpioe_pupdr = 0u;
uint32_t gpioa_afrh = 0u;
uint32_t gpioa_bsrr = 0u, gpiob_bsrr = 0u, gpioc_bsrr = 0u, gpioe_bsrr = 0u;
void (*write_hook)(uint8_t, uint8_t) = nullptr;
}  // namespace ems::hal::out_pins_host

namespace ems::hal::coil_oc_host {
uint8_t model = 0U;
uint8_t armed[4] = {0U, 0U, 0U, 0U};
uint8_t arm_high[4] = {0U, 0U, 0U, 0U};
uint32_t arm_ts[4] = {0U, 0U, 0U, 0U};
uint8_t ref[4] = {0U, 0U, 0U, 0U};
void (*edge_hook)(uint8_t, uint8_t, uint32_t) = nullptr;

void advance(uint32_t from, uint32_t to) noexcept {
    for (;;) {
        int8_t best = -1;
        for (uint8_t c = 0U; c < 4U; ++c) {
            if (armed[c] == 0U) { continue; }
            const int32_t d_from = static_cast<int32_t>(arm_ts[c] - from);
            const int32_t d_to = static_cast<int32_t>(arm_ts[c] - to);
            if (d_from <= 0 || d_to > 0) { continue; }
            if (best < 0 || static_cast<int32_t>(arm_ts[c] - arm_ts[best]) < 0) {
                best = static_cast<int8_t>(c);
            }
        }
        if (best < 0) { return; }
        const uint8_t c = static_cast<uint8_t>(best);
        armed[c] = 0U;
        ref[c] = arm_high[c];
        if (edge_hook != nullptr) {
            edge_hook(static_cast<uint8_t>(7U - c), arm_high[c], arm_ts[c]);
        }
    }
}

void reset() noexcept {
    model = 0U;
    for (uint8_t c = 0U; c < 4U; ++c) { armed[c] = 0U; arm_high[c] = 0U; arm_ts[c] = 0U; ref[c] = 0U; }
    edge_hook = nullptr;
    g_coil_oc_enabled = 0U;
    g_coil_oc_offset = 0U;
}
}  // namespace ems::hal::coil_oc_host

#else
#include "hal/regs.h"
#endif

namespace ems::hal {

volatile uint8_t g_coil_oc_enabled = 0U;
volatile uint16_t g_coil_oc_offset = 0U;

void out_pins_hw_init() noexcept {
    // GPIO mode below takes the coil pins back from the timer: commits then
    // drive them through BSRR (software timing) until coil_oc_hw_init runs.
    g_coil_oc_enabled = 0U;
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}

#if EMS_BOARD_IS_VGT6
    // VGT6: all INJ/IGN on GPIOE — push-pull LOW (active-high actuators).
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN;
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}
    // 8 canais INJ/IGN: push-pull, sem pull, LOW = desligado.
    static const uint8_t pe_pins[] = {0U, 2U, 4U, 6U, 9U, 11U, 13U, 15U};
    for (uint8_t i = 0U; i < (sizeof(pe_pins) / sizeof(pe_pins[0])); ++i) {
        const uint8_t pin = pe_pins[i];
        GPIOE_OTYPER &= ~(1U << pin);
        GPIOE_PUPDR  = (GPIOE_PUPDR & ~(3U << (pin * 2U)));
        GPIOE_MODER  = (GPIOE_MODER & ~(3U << (pin * 2U))) | (1U << (pin * 2U));
    }
    // Escrita ÚNICA e atómica: tudo ao estado seguro de uma vez. Não separar em
    // duas escritas — além de deixar uma janela entre elas, o mock de host só
    // regista a última e o teste de cobertura VGT6 deixaria de ver os canais.
    GPIOE_BSRR = (1U << (0U + 16U)) | (1U << (2U + 16U)) | (1U << (4U + 16U))
               | (1U << (6U + 16U)) | (1U << (9U + 16U)) | (1U << (11U + 16U))
               | (1U << (13U + 16U)) | (1U << (15U + 16U));
#else
    // RGT6: INJ PA15/PB3/PC10/PC11 · IGN PC6–9
    // PA15 after reset is often JTDI with pull-up → HIGH until here.
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOBEN | RCC_AHB2ENR1_GPIOCEN;
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}

    GPIOA_AFRH = (GPIOA_AFRH & ~(0xFu << ((15U - 8U) * 4U)));
    GPIOA_OTYPER &= ~(1U << 15U);
    GPIOA_PUPDR  = (GPIOA_PUPDR & ~(3U << (15U * 2U)));
    GPIOB_OTYPER &= ~(1U << 3U);
    GPIOB_PUPDR  = (GPIOB_PUPDR & ~(3U << (3U * 2U)));
    for (uint8_t pin = 6U; pin <= 11U; ++pin) {
        GPIOC_OTYPER &= ~(1U << pin);
        GPIOC_PUPDR  = (GPIOC_PUPDR & ~(3U << (pin * 2U)));
    }

    GPIOA_MODER = (GPIOA_MODER & ~(3U << (15U * 2U))) | (1U << (15U * 2U));
    GPIOB_MODER = (GPIOB_MODER & ~(3U << (3U * 2U))) | (1U << (3U * 2U));
    for (uint8_t pin = 6U; pin <= 11U; ++pin) {
        GPIOC_MODER = (GPIOC_MODER & ~(3U << (pin * 2U))) | (1U << (pin * 2U));
    }

    GPIOA_BSRR = (1U << (15U + 16U));
    GPIOB_BSRR = (1U << (3U + 16U));
    GPIOC_BSRR = (1U << (6U + 16U)) | (1U << (7U + 16U))
               | (1U << (8U + 16U)) | (1U << (9U + 16U))
               | (1U << (10U + 16U)) | (1U << (11U + 16U));
#endif
}

#if defined(EMS_HOST_TEST)
bool coil_oc_hw_init() noexcept {
    // Host: the simulator opts into the compare model explicitly.
    g_coil_oc_enabled = coil_oc_host::model;
    g_coil_oc_offset = 0U;
    return g_coil_oc_enabled != 0U;
}
#else
bool coil_oc_hw_init() noexcept {
#if !EMS_IGN_HW_OC
    return false;
#else
    // Same 250 MHz kernel clock and PSC as TIM5 (system.cpp: PPRE1 = PPRE2 = /2).
#if EMS_BOARD_IS_VGT6
#define COIL_TIM(reg) TIM1_##reg
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;
#else
#define COIL_TIM(reg) TIM8_##reg
    RCC_APB2ENR |= RCC_APB2ENR_TIM8EN;
#endif
    (void)RCC_APB2ENR;
    COIL_TIM(CR1) = 0U;
    COIL_TIM(PSC) = 3U;
    COIL_TIM(ARR) = 0xFFFFU;
    COIL_TIM(RCR) = 0U;
    // Every channel starts forced inactive (coil off), output enabled, active high.
    COIL_TIM(CCMR1) = TIM_CCMR1_OC1M_FORCE_INACTIVE | TIM_CCMR1_OC2M_FORCE_INACTIVE;
    COIL_TIM(CCMR2) = TIM_CCMR2_OC3M_FORCE_INACTIVE | TIM_CCMR2_OC4M_FORCE_INACTIVE;
    COIL_TIM(CCR1) = 0U; COIL_TIM(CCR2) = 0U; COIL_TIM(CCR3) = 0U; COIL_TIM(CCR4) = 0U;
    COIL_TIM(CCER) = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    COIL_TIM(BDTR) = TIM_BDTR_MOE;   // advanced timer: outputs gated by MOE
    COIL_TIM(EGR) = TIM_EGR_UG;
    COIL_TIM(CR1) = TIM_CR1_CEN;

    // Offset TIMx − TIM5: bracket a TIMx read between two TIM5 reads and keep
    // the tightest sample. Both counters tick together, so the result is a
    // constant (error ≤ half the bracket, typically ≤ 1 tick = 16 ns).
    {
        CriticalSectionGuard guard;
        uint32_t best_span = 0xFFFFFFFFU;
        uint16_t best_off = 0U;
        for (uint8_t k = 0U; k < 8U; ++k) {
            const uint32_t a = TIM5_CNT;
            const uint32_t b = COIL_TIM(CNT);
            const uint32_t c = TIM5_CNT;
            const uint32_t span = c - a;
            if (span < best_span) {
                best_span = span;
                best_off = static_cast<uint16_t>(b - (a + span / 2U));
            }
        }
        g_coil_oc_offset = best_off;
    }

    // Hand the pins to the timer: ODR is LOW and OCxREF is forced LOW, so the
    // MODER switch is glitch-free.
#if EMS_BOARD_IS_VGT6
    static const uint8_t pins[4] = {9U, 11U, 13U, 15U};
    for (uint8_t i = 0U; i < 4U; ++i) {
        gpio_set_af(&GPIOE_MODER, &GPIOE_AFRL, &GPIOE_AFRH, &GPIOE_OSPEEDR, pins[i], GPIO_AF1);
    }
#else
    for (uint8_t pin = 6U; pin <= 9U; ++pin) {
        gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, pin, GPIO_AF3);
    }
#endif
#undef COIL_TIM
    g_coil_oc_enabled = 1U;
    return true;
#endif
}
#endif

#if defined(EMS_HOST_TEST)
uint32_t out_pins_test_bsrr_snapshot(uint8_t port) noexcept {
    using namespace out_pins_host;
    switch (port) {
    case 0: return gpioa_bsrr;
    case 1: return gpiob_bsrr;
    case 2: return gpioc_bsrr;
    case 3: return gpioe_bsrr;
    default: return 0u;
    }
}

void out_pins_test_reset_stubs() noexcept {
    using namespace out_pins_host;
    rcc_ahb2enr1 = 0u;
    gpioa_moder = gpiob_moder = gpioc_moder = gpioe_moder = 0u;
    gpioa_otyper = gpiob_otyper = gpioc_otyper = gpioe_otyper = 0u;
    gpioa_pupdr = gpiob_pupdr = gpioc_pupdr = gpioe_pupdr = 0u;
    gpioa_afrh = 0u;
    gpioa_bsrr = gpiob_bsrr = gpioc_bsrr = gpioe_bsrr = 0u;
}
#endif

}  // namespace ems::hal
