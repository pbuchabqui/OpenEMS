/**
 * @file mock_hal_timer.h
 * @brief Mock implementation for HAL Timer functions
 * 
 * Provides controllable timing functions for unit testing
 * without hardware dependencies.
 */

#ifndef MOCK_HAL_TIMER_H
#define MOCK_HAL_TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "unity.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mock state structure
typedef struct {
    uint64_t current_time_us;
    uint32_t time_increment_us;
    bool auto_increment;
    uint32_t call_count_time_us;
    uint32_t call_count_time_us32;
    uint32_t call_count_delay_us;
} mock_hal_timer_state_t;

// Global mock state
extern mock_hal_timer_state_t g_mock_hal_timer;

// Mock control functions
void mock_hal_timer_reset(void);
void mock_hal_timer_set_time(uint64_t time_us);
void mock_hal_timer_increment(uint32_t increment_us);
void mock_hal_timer_set_auto_increment(bool enable, uint32_t increment_us);
uint64_t mock_hal_timer_get_time(void);
uint32_t mock_hal_timer_get_call_count(const char* function_name);

// Mocked HAL functions (replace inline versions during testing)
uint64_t HAL_Time_us(void);
uint32_t HAL_Time_us32(void);
uint32_t HAL_Elapsed_us(uint32_t start, uint32_t now);
void HAL_Delay_us(uint32_t us);

// Test helper macros
#define MOCK_HAL_TIMER_ASSERT_CALL_COUNT(func, expected) \
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected, mock_hal_timer_get_call_count(#func), \
        "Unexpected call count for " #func)

#define MOCK_HAL_TIMER_ASSERT_TIME(expected) \
    TEST_ASSERT_EQUAL_UINT64_MESSAGE(expected, mock_hal_timer_get_time(), \
        "Mock timer time mismatch")

#ifdef __cplusplus
}
#endif

#endif // MOCK_HAL_TIMER_H
