/**
 * @file math_utils.h
 * @brief Shared mathematical utility functions for engine control
 * 
 * This header provides common math functions used across multiple modules.
 * All functions are marked IRAM_ATTR for safe use in ISR context.
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "esp_attr.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Wrap angle to 0-360 degree range
 * 
 * @note IRAM_ATTR - safe for ISR context
 * 
 * @param angle_deg Input angle in degrees
 * @return Angle wrapped to [0, 360) range
 */
IRAM_ATTR static inline float wrap_angle_360(float angle_deg) {
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/**
 * @brief Wrap angle to 0-720 degree range (for 4-stroke cycle)
 * 
 * @note IRAM_ATTR - safe for ISR context
 * 
 * @param angle_deg Input angle in degrees
 * @return Angle wrapped to [0, 720) range
 */
IRAM_ATTR static inline float wrap_angle_720(float angle_deg) {
    while (angle_deg >= 720.0f) {
        angle_deg -= 720.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 720.0f;
    }
    return angle_deg;
}

/**
 * @brief Clamp a float value to a range
 * 
 * @note IRAM_ATTR - safe for ISR context
 * 
 * @param v Input value
 * @param min_v Minimum value
 * @param max_v Maximum value
 * @return Clamped value
 */
IRAM_ATTR static inline float clamp_float(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

/**
 * @brief Clamp an integer value to a range
 * 
 * @note IRAM_ATTR - safe for ISR context
 * 
 * @param v Input value
 * @param min_v Minimum value
 * @param max_v Maximum value
 * @return Clamped value
 */
IRAM_ATTR static inline uint32_t clamp_u32(uint32_t v, uint32_t min_v, uint32_t max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

/**
 * @brief Calculate absolute difference between two unsigned 16-bit values
 * 
 * @note IRAM_ATTR - safe for ISR context
 * 
 * @param a First value
 * @param b Second value
 * @return Absolute difference |a - b|
 */
IRAM_ATTR static inline uint16_t abs_u16_delta(uint16_t a, uint16_t b) {
    return (a > b) ? (a - b) : (b - a);
}

#ifdef __cplusplus
}
#endif

#endif // MATH_UTILS_H
