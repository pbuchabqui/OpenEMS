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
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief S1-02: Safe GPIO initialisation — must be called BEFORE any MCPWM driver init.
 *
 * Configures all high-side actuator outputs (injectors, ignition coils, relays)
 * as outputs driven LOW with internal pull-down resistors enabled.
 * This guarantees that, during the window between power-on and the MCPWM peripheral
 * taking ownership of the pins, no injector or coil can be inadvertently energised
 * by a floating output.
 *
 * Call sequence in engine_control_init():
 *   1. hal_gpio_safe_init()       ← sets all actuators LOW first
 *   2. mcpwm_ignition_hp_init()   ← MCPWM claims the ign pins
 *   3. mcpwm_injection_hp_init()  ← MCPWM claims the inj pins
 *   4. (MCPWM overrides pull-down with its own drive — that is fine)
 */
static inline esp_err_t hal_gpio_safe_init(void) {
    // Pins that must be forced LOW at boot (active-HIGH actuators).
    static const gpio_num_t actuator_pins[] = {
        HAL_PIN_INJ_1, HAL_PIN_INJ_2, HAL_PIN_INJ_3, HAL_PIN_INJ_4,
        HAL_PIN_IGN_1, HAL_PIN_IGN_2, HAL_PIN_IGN_3, HAL_PIN_IGN_4,
        HAL_PIN_FUEL_PUMP, HAL_PIN_FAN, HAL_PIN_CEL,
        HAL_PIN_AUX_1, HAL_PIN_AUX_2,
    };

    gpio_config_t io_conf = {
        .mode       = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type  = GPIO_INTR_DISABLE,
        .pin_bit_mask = 0ULL,
    };

    for (int i = 0; i < (int)(sizeof(actuator_pins) / sizeof(actuator_pins[0])); i++) {
        io_conf.pin_bit_mask |= (1ULL << actuator_pins[i]);
    }

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // Drive all pins explicitly LOW (pull-down alone is not enough if the
    // output register holds a 1 from a previous power cycle).
    for (int i = 0; i < (int)(sizeof(actuator_pins) / sizeof(actuator_pins[0])); i++) {
        gpio_set_level(actuator_pins[i], 0);
    }

    return ESP_OK;
}

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
