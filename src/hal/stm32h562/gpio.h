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

/**
 * @brief Configure GPIO pin as output push-pull with high speed.
 *
 * @param moder   Pointer to GPIOx_MODER register.
 * @param otyper  Pointer to GPIOx_OTYPER register or nullptr if not needed.
 * @param ospeedr Pointer to GPIOx_OSPEEDR register or nullptr if not needed.
 * @param pin     Pin number (0-15).
 */
void gpio_set_output_pushpull(volatile uint32_t* moder, volatile uint32_t* otyper,
                              volatile uint32_t* ospeedr, uint8_t pin) noexcept;

/**
 * @brief Write logic level to GPIO pin using BSRR register (atomic operation).
 *
 * @param bsrr_reg Pointer to GPIOx_BSRR register.
 * @param pin      Pin number (0-15).
 * @param value    Logic level: 0 = reset, 1 = set.
 */
void gpio_write_pin(volatile uint32_t* bsrr_reg, uint8_t pin, uint8_t value) noexcept;

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
