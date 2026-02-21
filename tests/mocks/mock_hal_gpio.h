/**
 * @file mock_hal_gpio.h
 * @brief Mock implementation for HAL GPIO functions
 * 
 * Provides controllable GPIO functions for unit testing
 * without hardware dependencies.
 */

#ifndef MOCK_HAL_GPIO_H
#define MOCK_HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "unity.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mock GPIO state structure
typedef struct {
    uint32_t gpio_states[40];  // GPIO 0-39
    uint32_t call_counts[40];  // Call count per GPIO
    uint32_t total_call_count;
    bool capture_mode;
    uint32_t last_gpio;
    bool last_state;
} mock_hal_gpio_state_t;

// Global mock state
extern mock_hal_gpio_state_t g_mock_hal_gpio;

// Mock control functions
void mock_hal_gpio_reset(void);
void mock_hal_gpio_set_state(uint32_t gpio_num, bool state);
bool mock_hal_gpio_get_state(uint32_t gpio_num);
void mock_hal_gpio_set_capture_mode(bool enable);
uint32_t mock_hal_gpio_get_call_count(uint32_t gpio_num);
void mock_hal_gpio_get_last_operation(uint32_t* gpio_num, bool* state);

// Mocked HAL functions
void HAL_GPIO_High(uint32_t gpio_num);
void HAL_GPIO_Low(uint32_t gpio_num);
void HAL_Injector_Set(uint8_t channel, bool active);
void HAL_Ignition_Set(uint8_t channel, bool charge);
bool HAL_GPIO_Read(uint32_t gpio_num);

// Convenience macros
#define HAL_CEL_On()         HAL_GPIO_High(HAL_PIN_CEL)
#define HAL_CEL_Off()        HAL_GPIO_Low(HAL_PIN_CEL)
#define HAL_FuelPump_On()    HAL_GPIO_High(HAL_PIN_FUEL_PUMP)
#define HAL_FuelPump_Off()   HAL_GPIO_Low(HAL_PIN_FUEL_PUMP)
#define HAL_Fan_On()         HAL_GPIO_High(HAL_PIN_FAN)
#define HAL_Fan_Off()        HAL_GPIO_Low(HAL_PIN_FAN)
#define HAL_Clutch_Read()    HAL_GPIO_Read(HAL_PIN_CLUTCH)
#define HAL_Brake_Read()     HAL_GPIO_Read(HAL_PIN_BRAKE)

// Test helper macros
#define MOCK_HAL_GPIO_ASSERT_STATE(gpio, expected) \
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected ? 1 : 0, mock_hal_gpio_get_state(gpio), \
        "GPIO state mismatch for GPIO " #gpio)

#define MOCK_HAL_GPIO_ASSERT_CALL_COUNT(gpio, expected) \
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected, mock_hal_gpio_get_call_count(gpio), \
        "GPIO call count mismatch for GPIO " #gpio)

#ifdef __cplusplus
}
#endif

#endif // MOCK_HAL_GPIO_H
