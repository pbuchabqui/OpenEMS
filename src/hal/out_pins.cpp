/**
 * @file hal/out_pins.cpp
 * INJ/IGN GPIO init + safe de-assert (relocated from ecu_sched_outputs_safe_early).
 */
#include "hal/out_pins.h"

#if defined(EMS_HOST_TEST)

namespace ems::hal::out_pins_host {
uint32_t rcc_ahb2enr1 = 0u;
uint32_t gpioa_moder = 0u, gpiob_moder = 0u, gpioc_moder = 0u, gpioe_moder = 0u;
uint32_t gpioa_otyper = 0u, gpiob_otyper = 0u, gpioc_otyper = 0u, gpioe_otyper = 0u;
uint32_t gpioa_pupdr = 0u, gpiob_pupdr = 0u, gpioc_pupdr = 0u, gpioe_pupdr = 0u;
uint32_t gpioa_afrh = 0u;
uint32_t gpioa_bsrr = 0u, gpiob_bsrr = 0u, gpioc_bsrr = 0u, gpioe_bsrr = 0u;
}  // namespace ems::hal::out_pins_host

#else
#include "hal/regs.h"
#endif

namespace ems::hal {

void out_pins_hw_init() noexcept {
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}

#if EMS_BOARD_IS_VGT6
    // VGT6: all INJ/IGN on GPIOE — push-pull LOW (active-high actuators).
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN;
    for (volatile uint32_t d = 0u; d < 8u; ++d) {}
    static const uint8_t pe_pins[] = {0U, 2U, 4U, 6U, 9U, 11U, 13U, 15U};
    for (uint8_t i = 0U; i < 8U; ++i) {
        const uint8_t pin = pe_pins[i];
        GPIOE_OTYPER &= ~(1U << pin);
        GPIOE_PUPDR  = (GPIOE_PUPDR & ~(3U << (pin * 2U)));
        GPIOE_MODER  = (GPIOE_MODER & ~(3U << (pin * 2U))) | (1U << (pin * 2U));
    }
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
