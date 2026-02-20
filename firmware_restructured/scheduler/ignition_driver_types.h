/**
 * @file mcpwm_ignition.h
 * @brief MCPWM ignition driver base definitions
 */

#ifndef MCPWM_IGNITION_H
#define MCPWM_IGNITION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ignition channel configuration
 */
typedef struct {
    uint32_t base_frequency_hz;      // Base timer frequency
    uint32_t timer_resolution_bits;  // Timer resolution in bits
    uint32_t min_dwell_us;           // Minimum dwell in microseconds
    uint32_t max_dwell_us;           // Maximum dwell in microseconds
    int gpio_nums[4];                // GPIO numbers for each coil
} mcpwm_ignition_config_t;

/**
 * @brief Ignition channel status
 */
typedef struct {
    bool is_active;                  // Channel is currently active
    uint32_t last_dwell_us;          // Last dwell used
    uint32_t last_timing_us;         // Last timing used
    uint32_t total_fires;            // Total fires
    uint32_t error_count;            // Error count
} mcpwm_ignition_status_t;

#ifdef __cplusplus
}
#endif

#endif // MCPWM_IGNITION_H
