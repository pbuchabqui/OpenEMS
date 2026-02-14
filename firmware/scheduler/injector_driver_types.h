/**
 * @file mcpwm_injection.h
 * @brief MCPWM injection driver base definitions
 */

#ifndef MCPWM_INJECTION_H
#define MCPWM_INJECTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Injection channel configuration
 */
typedef struct {
    uint32_t base_frequency_hz;      // Base timer frequency
    uint32_t timer_resolution_bits;  // Timer resolution in bits
    uint32_t min_pulsewidth_us;      // Minimum pulsewidth in microseconds
    uint32_t max_pulsewidth_us;      // Maximum pulsewidth in microseconds
    int gpio_nums[4];                // GPIO numbers for each injector
} mcpwm_injection_config_t;

/**
 * @brief Injector channel status
 */
typedef struct {
    bool is_active;                  // Channel is currently active
    uint32_t last_pulsewidth_us;     // Last pulsewidth used
    uint32_t last_delay_us;          // Last delay used
    uint32_t total_pulses;           // Total pulses fired
    uint32_t error_count;            // Error count
} mcpwm_injector_channel_t;

#ifdef __cplusplus
}
#endif

#endif // MCPWM_INJECTION_H
