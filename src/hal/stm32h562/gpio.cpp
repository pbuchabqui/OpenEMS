#include "hal/stm32h562/gpio.h"
#include "hal/regs.h"

void gpio_set_af(volatile uint32_t* moder, volatile uint32_t* afrl, volatile uint32_t* afrh,
                volatile uint32_t* ospeedr, uint8_t pin, uint8_t af) noexcept
{
    // Speed: high speed (11b) if ospeedr provided
    if (ospeedr) {
        if (pin < 8u) {
            *ospeedr |= (3u << (pin * 4u));  // OSPEEDR pin bits
        } else {
            *ospeedr |= (3u << ((pin - 8u) * 4u));
        }
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
