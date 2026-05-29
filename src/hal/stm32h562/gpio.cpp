#include "hal/stm32h562/gpio.h"
#include <cstdint>

void gpio_set_af(volatile uint32_t* moder, volatile uint32_t* afrl, volatile uint32_t* afrh,
                volatile uint32_t* ospeedr, uint8_t pin, uint8_t af) noexcept
{
    if (ospeedr) {
        const uint32_t shift = static_cast<uint32_t>(pin) * 2u;
        *ospeedr = (*ospeedr & ~(3u << shift)) | (3u << shift);
    }
    if (pin < 8u) {
        if (afrl) {
            uint32_t shift = (pin & 7u) * 4u;
            *afrl = (*afrl & ~(0xFu << shift)) | (af << shift);
        }
    } else {
        if (afrh) {
            uint32_t shift = (pin - 8u) * 4u;
            *afrh = (*afrh & ~(0xFu << shift)) | (af << shift);
        }
    }
    if (moder) {
        uint32_t shift = pin * 2u;
        *moder = (*moder & ~(3u << shift)) | (2u << shift);
    }
}

void gpio_set_output_pushpull(volatile uint32_t* moder, volatile uint32_t* otyper,
                              volatile uint32_t* ospeedr, uint8_t pin) noexcept
{
    if (ospeedr) {
        uint32_t shift = pin * 2u;
        *ospeedr = (*ospeedr & ~(3u << shift)) | (3u << shift);
    }
    if (otyper) {
        *otyper &= ~(1u << pin);
    }
    if (moder) {
        uint32_t shift = pin * 2u;
        *moder = (*moder & ~(3u << shift)) | (1u << shift);
    }
}

void gpio_write_pin(volatile uint32_t* bsrr_reg, uint8_t pin, uint8_t value) noexcept {
    if (value) {
        *bsrr_reg = (1u << pin);
    } else {
        *bsrr_reg = (1u << (pin + 16u));
    }
}
