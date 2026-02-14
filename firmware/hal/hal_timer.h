/**
 * @file hal_timer.h
 * @brief Timer HAL — inline counter reads for absolute compare scheduling
 *
 * Provides zero-overhead access to the MCPWM timer counter values
 * used by injector_driver and ignition_driver for absolute-compare
 * scheduling (no jitter from timer restart).
 *
 * Also provides esp_timer_get_time() alias for use in ISR context.
 *
 * Safe to use in ISR context (IRAM-safe, no API call overhead).
 */

#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdint.h>
#include "esp_attr.h"
#include "esp_timer.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get current system time in microseconds (64-bit, monotonic)
 *
 * Wraps esp_timer_get_time() — safe in ISR, IRAM-resident.
 * Overflow occurs every ~584,000 years.
 */
__attribute__((always_inline)) static inline uint64_t HAL_Time_us(void) {
    return (uint64_t)esp_timer_get_time();
}

/**
 * @brief Get current time in microseconds as 32-bit value
 *
 * Overflows every ~71.6 minutes. Use only for short interval measurements
 * (tooth period, dwell time, injection pulse width).
 * For absolute timestamps use HAL_Time_us().
 */
__attribute__((always_inline)) static inline uint32_t HAL_Time_us32(void) {
    return (uint32_t)esp_timer_get_time();
}

/**
 * @brief Compute elapsed microseconds handling 32-bit rollover
 * @param start  Timestamp captured earlier via HAL_Time_us32()
 * @param now    Current time via HAL_Time_us32()
 * @return       Elapsed time in microseconds (correct across rollover)
 */
__attribute__((always_inline)) static inline uint32_t HAL_Elapsed_us(uint32_t start, uint32_t now) {
    return now - start;  // unsigned subtraction handles rollover correctly
}

/**
 * @brief Busy-wait for N microseconds (use only in init, never in runtime)
 * @param us  Duration in microseconds
 */
__attribute__((always_inline)) static inline void HAL_Delay_us(uint32_t us) {
    uint32_t start = HAL_Time_us32();
    while (HAL_Elapsed_us(start, HAL_Time_us32()) < us) {
        /* spin */
    }
}

#ifdef __cplusplus
}
#endif

#endif // HAL_TIMER_H
