#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure alternate function for a GPIO pin.
 *
 * @param moder   Pointer to GPIOx_MODER register.
 * @param afrl    Pointer to GPIOx_AFRL register (pins 0-7) or nullptr if not needed.
 * @param afrh    Pointer to GPIOx_AFRH register (pins 8-15) or nullptr if not needed.
 * @param ospeedr Pointer to GPIOx_OSPEEDR register or nullptr if not needed.
 * @param pin     Pin number (0-15).
 * @param af      Alternate function number (0-15).
 */
void gpio_set_af(volatile uint32_t* moder, volatile uint32_t* afrl, volatile uint32_t* afrh,
                volatile uint32_t* ospeedr, uint8_t pin, uint8_t af) noexcept;

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
