// gpio_set_af() is implemented as inline in hal/stm32h562/regs.h
// No out-of-line GPIO functions remain; this header kept for documentation.
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

void gpio_set_af(volatile uint32_t* moder, volatile uint32_t* afrl, volatile uint32_t* afrh,
                volatile uint32_t* ospeedr, uint8_t pin, uint8_t af) noexcept;

void gpio_set_output_pushpull(volatile uint32_t* moder, volatile uint32_t* otyper,
                              volatile uint32_t* ospeedr, uint8_t pin) noexcept;

void gpio_write_pin(volatile uint32_t* bsrr_reg, uint8_t pin, uint8_t value) noexcept;

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
