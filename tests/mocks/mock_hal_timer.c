/**
 * @file mock_hal_timer.c
 * @brief Mock implementation for HAL Timer functions
 */

#include "mock_hal_timer.h"
#include <string.h>

// Global mock state
mock_hal_timer_state_t g_mock_hal_timer = {0};

void mock_hal_timer_reset(void) {
    memset(&g_mock_hal_timer, 0, sizeof(g_mock_hal_timer));
    g_mock_hal_timer.time_increment_us = 1; // Default increment
}

void mock_hal_timer_set_time(uint64_t time_us) {
    g_mock_hal_timer.current_time_us = time_us;
}

void mock_hal_timer_increment(uint32_t increment_us) {
    g_mock_hal_timer.current_time_us += increment_us;
}

void mock_hal_timer_set_auto_increment(bool enable, uint32_t increment_us) {
    g_mock_hal_timer.auto_increment = enable;
    g_mock_hal_timer.time_increment_us = increment_us;
}

uint64_t mock_hal_timer_get_time(void) {
    return g_mock_hal_timer.current_time_us;
}

uint32_t mock_hal_timer_get_call_count(const char* function_name) {
    // Simple implementation - could be enhanced with hash map
    if (strcmp(function_name, "HAL_Time_us") == 0) {
        return g_mock_hal_timer.call_count_time_us;
    } else if (strcmp(function_name, "HAL_Time_us32") == 0) {
        return g_mock_hal_timer.call_count_time_us32;
    } else if (strcmp(function_name, "HAL_Delay_us") == 0) {
        return g_mock_hal_timer.call_count_delay_us;
    }
    return 0;
}

// Mocked HAL function implementations
uint64_t HAL_Time_us(void) {
    g_mock_hal_timer.call_count_time_us++;
    
    if (g_mock_hal_timer.auto_increment) {
        g_mock_hal_timer.current_time_us += g_mock_hal_timer.time_increment_us;
    }
    
    return g_mock_hal_timer.current_time_us;
}

uint32_t HAL_Time_us32(void) {
    g_mock_hal_timer.call_count_time_us32++;
    
    if (g_mock_hal_timer.auto_increment) {
        g_mock_hal_timer.current_time_us += g_mock_hal_timer.time_increment_us;
    }
    
    return (uint32_t)g_mock_hal_timer.current_time_us;
}

uint32_t HAL_Elapsed_us(uint32_t start, uint32_t now) {
    return now - start;  // unsigned subtraction handles rollover
}

void HAL_Delay_us(uint32_t us) {
    g_mock_hal_timer.call_count_delay_us++;
    g_mock_hal_timer.current_time_us += us;
}
