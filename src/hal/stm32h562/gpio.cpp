#include "hal/stm32h562/gpio.h"
#include "hal/regs.h"

void gpio_set_af(volatile uint32_t* moder, volatile uint32_t* afrl, volatile uint32_t* afrh,
                volatile uint32_t* ospeedr, uint8_t pin, uint8_t af) noexcept
{
    // Speed: high speed (11b) if ospeedr provided
    if (ospeedr) {
        const uint32_t shift = static_cast<uint32_t>(pin) * 2u;
        *ospeedr = (*ospeedr & ~(3u << shift)) | (3u << shift);
    }

    // Alternate function selection
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

    // Configure mode to alternate function (10b)
    if (moder) {
        uint32_t shift = pin * 2u;
        *moder = (*moder & ~(3u << shift)) | (2u << shift);
    }
}

void gpio_set_output_pushpull(volatile uint32_t* moder, volatile uint32_t* otyper, 
                              volatile uint32_t* ospeedr, uint8_t pin) noexcept
{
    // Speed: high speed (11b)
    if (ospeedr) {
        uint32_t shift = pin * 2u;
        *ospeedr = (*ospeedr & ~(3u << shift)) | (3u << shift);
    }
    
    // Output type: push-pull (0)
    if (otyper) {
        *otyper &= ~(1u << pin);
    }
    
    // Mode: general purpose output (01b)
    if (moder) {
        uint32_t shift = pin * 2u;
        *moder = (*moder & ~(3u << shift)) | (1u << shift);
    }
}

void gpio_write_pin(volatile uint32_t* bsrr_reg, uint8_t pin, uint8_t value) noexcept {
    // BSRR: write 1 to lower 16 bits sets pin, write 1 to upper 16 bits resets pin
    if (value) {
        *bsrr_reg = (1u << pin);          // Set pin
    } else {
        *bsrr_reg = (1u << (pin + 16u));  // Reset pin
    }
}
