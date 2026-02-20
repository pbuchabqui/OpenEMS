/**
 * @file hal_gpio.h
 * @brief GPIO HAL — zero-latency inline wrappers
 *
 * All functions are __attribute__((always_inline)) inline.
 * With -O2 the compiler generates direct register writes —
 * identical assembly to bare-metal code, zero call overhead.
 *
 * Safe to use in ISR context (Core 0, time-critical path).
 */

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "hal_pins.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set GPIO high (direct register write, no API overhead)
 * @param gpio_num GPIO number (0-39 for pins 0-31 use OUT_W1TS, 32+ use OUT1_W1TS)
 */
__attribute__((always_inline)) static inline void HAL_GPIO_High(uint32_t gpio_num) {
    if (gpio_num < 32) {
        REG_WRITE(GPIO_OUT_W1TS_REG, 1U << gpio_num);
    } else {
        REG_WRITE(GPIO_OUT1_W1TS_REG, 1U << (gpio_num - 32));
    }
}

/**
 * @brief Set GPIO low (direct register write)
 */
__attribute__((always_inline)) static inline void HAL_GPIO_Low(uint32_t gpio_num) {
    if (gpio_num < 32) {
        REG_WRITE(GPIO_OUT_W1TC_REG, 1U << gpio_num);
    } else {
        REG_WRITE(GPIO_OUT1_W1TC_REG, 1U << (gpio_num - 32));
    }
}

/**
 * @brief Write injector state (active HIGH, low-side driver)
 * @param channel Injector channel 0-3
 * @param active  true = inject, false = off
 * H16 fix: Added bounds checking to prevent out-of-bounds access
 */
__attribute__((always_inline)) static inline void HAL_Injector_Set(uint8_t channel, bool active) {
    // H16 fix: bounds check - ignore invalid channel
    if (channel >= 4) {
        return;
    }
    static const uint32_t pins[4] = {
        HAL_PIN_INJ_1, HAL_PIN_INJ_2, HAL_PIN_INJ_3, HAL_PIN_INJ_4
    };
    if (active) HAL_GPIO_High(pins[channel]);
    else        HAL_GPIO_Low(pins[channel]);
}

/**
 * @brief Write ignition coil state (active HIGH = charge / dwell)
 * @param channel Ignition channel 0-3
 * @param charge  true = charging coil (dwell), false = fire (spark)
 * H16 fix: Added bounds checking to prevent out-of-bounds access
 */
__attribute__((always_inline)) static inline void HAL_Ignition_Set(uint8_t channel, bool charge) {
    // H16 fix: bounds check - ignore invalid channel
    if (channel >= 4) {
        return;
    }
    static const uint32_t pins[4] = {
        HAL_PIN_IGN_1, HAL_PIN_IGN_2, HAL_PIN_IGN_3, HAL_PIN_IGN_4
    };
    if (charge) HAL_GPIO_High(pins[channel]);
    else        HAL_GPIO_Low(pins[channel]);
}

/**
 * @brief Read digital input with direct register read
 */
__attribute__((always_inline)) static inline bool HAL_GPIO_Read(uint32_t gpio_num) {
    if (gpio_num < 32) {
        return (REG_READ(GPIO_IN_REG) >> gpio_num) & 1U;
    } else {
        return (REG_READ(GPIO_IN1_REG) >> (gpio_num - 32)) & 1U;
    }
}

/* Convenience macros for named signals */
#define HAL_CEL_On()         HAL_GPIO_High(HAL_PIN_CEL)
#define HAL_CEL_Off()        HAL_GPIO_Low(HAL_PIN_CEL)
#define HAL_FuelPump_On()    HAL_GPIO_High(HAL_PIN_FUEL_PUMP)
#define HAL_FuelPump_Off()   HAL_GPIO_Low(HAL_PIN_FUEL_PUMP)
#define HAL_Fan_On()         HAL_GPIO_High(HAL_PIN_FAN)
#define HAL_Fan_Off()        HAL_GPIO_Low(HAL_PIN_FAN)
#define HAL_Clutch_Read()    HAL_GPIO_Read(HAL_PIN_CLUTCH)
#define HAL_Brake_Read()     HAL_GPIO_Read(HAL_PIN_BRAKE)

#ifdef __cplusplus
}
#endif

#endif // HAL_GPIO_H
