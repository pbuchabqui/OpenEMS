#ifndef OPENEMS_TEST_DEFS_H
#define OPENEMS_TEST_DEFS_H

// Tipos básicos
#include <stdint.h>
#include <stdbool.h>

// ESP-IDF types
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1

// HAL Timer types
typedef uint64_t hal_time_t;

// HAL GPIO types
typedef enum {
    HAL_GPIO_LOW = 0,
    HAL_GPIO_HIGH = 1
} hal_gpio_level_t;

// Sync config types
typedef struct {
    int teeth_count;
    int missing_teeth;
    float wheel_diameter;
} sync_config_t;

typedef struct {
    uint32_t rpm;
    uint32_t tooth_time;
    bool sync_state;
} sync_data_t;

// Event scheduler types
typedef struct {
    uint32_t angle;
    uint32_t time_us;
    void (*callback)(void);
    uint8_t priority;
} event_t;

typedef struct {
    event_t events[32];
    uint8_t count;
    uint32_t base_time;
} event_scheduler_t;

// MCPWM types
typedef struct {
    uint32_t pulse_width_us;
    uint32_t period_us;
    uint32_t deadtime_us;
    bool enabled;
} mcpwm_config_t;

// Atomic buffer types
typedef struct {
    volatile uint32_t write_index;
    volatile uint32_t read_index;
    uint8_t buffer[2][256];
    bool ready[2];
} atomic_buf_t;

#endif // OPENEMS_TEST_DEFS_H
