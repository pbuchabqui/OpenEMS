/**
 * @file mock_hal_gpio.c
 * @brief Mock implementation for HAL GPIO functions
 */

#include "mock_hal_gpio.h"
#include "hal_pins.h"
#include <string.h>

// Global mock state
mock_hal_gpio_state_t g_mock_hal_gpio = {0};

void mock_hal_gpio_reset(void) {
    memset(&g_mock_hal_gpio, 0, sizeof(g_mock_hal_gpio));
}

void mock_hal_gpio_set_state(uint32_t gpio_num, bool state) {
    if (gpio_num < 40) {
        g_mock_hal_gpio.gpio_states[gpio_num] = state ? 1 : 0;
    }
}

bool mock_hal_gpio_get_state(uint32_t gpio_num) {
    if (gpio_num < 40) {
        return g_mock_hal_gpio.gpio_states[gpio_num] != 0;
    }
    return false;
}

void mock_hal_gpio_set_capture_mode(bool enable) {
    g_mock_hal_gpio.capture_mode = enable;
}

uint32_t mock_hal_gpio_get_call_count(uint32_t gpio_num) {
    if (gpio_num < 40) {
        return g_mock_hal_gpio.call_counts[gpio_num];
    }
    return 0;
}

void mock_hal_gpio_get_last_operation(uint32_t* gpio_num, bool* state) {
    if (gpio_num) *gpio_num = g_mock_hal_gpio.last_gpio;
    if (state) *state = g_mock_hal_gpio.last_state;
}

// Mocked HAL function implementations
void HAL_GPIO_High(uint32_t gpio_num) {
    g_mock_hal_gpio.total_call_count++;
    if (gpio_num < 40) {
        g_mock_hal_gpio.call_counts[gpio_num]++;
        g_mock_hal_gpio.gpio_states[gpio_num] = 1;
        
        if (g_mock_hal_gpio.capture_mode) {
            g_mock_hal_gpio.last_gpio = gpio_num;
            g_mock_hal_gpio.last_state = true;
        }
    }
}

void HAL_GPIO_Low(uint32_t gpio_num) {
    g_mock_hal_gpio.total_call_count++;
    if (gpio_num < 40) {
        g_mock_hal_gpio.call_counts[gpio_num]++;
        g_mock_hal_gpio.gpio_states[gpio_num] = 0;
        
        if (g_mock_hal_gpio.capture_mode) {
            g_mock_hal_gpio.last_gpio = gpio_num;
            g_mock_hal_gpio.last_state = false;
        }
    }
}

void HAL_Injector_Set(uint8_t channel, bool active) {
    if (channel >= 4) {
        return; // Bounds check - ignore invalid channel
    }
    
    static const uint32_t pins[4] = {
        HAL_PIN_INJ_1, HAL_PIN_INJ_2, HAL_PIN_INJ_3, HAL_PIN_INJ_4
    };
    
    if (active) {
        HAL_GPIO_High(pins[channel]);
    } else {
        HAL_GPIO_Low(pins[channel]);
    }
}

void HAL_Ignition_Set(uint8_t channel, bool charge) {
    if (channel >= 4) {
        return; // Bounds check - ignore invalid channel
    }
    
    static const uint32_t pins[4] = {
        HAL_PIN_IGN_1, HAL_PIN_IGN_2, HAL_PIN_IGN_3, HAL_PIN_IGN_4
    };
    
    if (charge) {
        HAL_GPIO_High(pins[channel]);
    } else {
        HAL_GPIO_Low(pins[channel]);
    }
}

bool HAL_GPIO_Read(uint32_t gpio_num) {
    g_mock_hal_gpio.total_call_count++;
    if (gpio_num < 40) {
        g_mock_hal_gpio.call_counts[gpio_num]++;
        return g_mock_hal_gpio.gpio_states[gpio_num] != 0;
    }
    return false;
}
