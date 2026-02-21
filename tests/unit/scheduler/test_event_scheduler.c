/**
 * @file test_event_scheduler.c
 * @brief Unit tests for angle-based event scheduler
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "unity.h"
#include "esp_err.h"
#include "scheduler/event_scheduler.h"
#include "mocks/mock_hal_timer.h"
#include "mocks/mock_hal_gpio.h"
#include "fixtures/engine_test_data.h"

// Test state
static scheduler_config_t test_config;
static scheduler_state_t test_state;
static bool callback_called = false;
static uint32_t last_callback_time = 0;

// Test callback function
static void test_event_callback(uint32_t scheduled_time_us, void* ctx) {
    callback_called = true;
    last_callback_time = mock_hal_timer_get_time();
}

void setUp(void) {
    // Reset all mocks
    mock_hal_timer_reset();
    mock_hal_gpio_reset();
    
    // Reset test state
    callback_called = false;
    last_callback_time = 0;
    memset(&test_config, 0, sizeof(test_config));
    memset(&test_state, 0, sizeof(test_state));
    
    // Setup default test configuration
    test_config.max_events = 100;
    test_config.time_resolution_us = 1;
    test_config.enable_priority = true;
}

void tearDown(void) {
    scheduler_deinit();
}

void test_scheduler_init_default_config(void) {
    esp_err_t ret = scheduler_init(&test_config);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify scheduler is ready
    ret = scheduler_get_state(&test_state);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(true, test_state.initialized);
    TEST_ASSERT_EQUAL(false, test_state.running);
}

void test_scheduler_init_invalid_config(void) {
    // Test with invalid configuration
    test_config.max_events = 0;  // Invalid
    
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
}

void test_scheduler_start_stop(void) {
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify running state
    ret = scheduler_get_state(&test_state);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(true, test_state.running);
    
    ret = scheduler_stop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify stopped state
    ret = scheduler_get_state(&test_state);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(false, test_state.running);
}

void test_scheduler_schedule_single_event(void) {
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Schedule event for 1000us from now
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t event_time = current_time + 1000;
    
    ret = scheduler_schedule_event(event_time, test_event_callback, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Advance time to trigger event
    mock_hal_timer_set_time(event_time);
    
    // Process events (this would normally be done in ISR)
    ret = scheduler_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify callback was called
    TEST_ASSERT_TRUE(callback_called);
    TEST_ASSERT_EQUAL_UINT32(event_time, last_callback_time);
}

void test_scheduler_schedule_multiple_events(void) {
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t event_times[5];
    
    // Schedule multiple events
    for (int i = 0; i < 5; i++) {
        event_times[i] = current_time + (i + 1) * 1000;  // 1000, 2000, 3000, 4000, 5000
        ret = scheduler_schedule_event(event_times[i], test_event_callback, NULL);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }
    
    // Process events in order
    for (int i = 0; i < 5; i++) {
        callback_called = false;
        mock_hal_timer_set_time(event_times[i]);
        
        ret = scheduler_process_events();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        TEST_ASSERT_TRUE(callback_called);
        TEST_ASSERT_EQUAL_UINT32(event_times[i], last_callback_time);
    }
}

void test_scheduler_angle_to_time_conversion(void) {
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Test at 1000 RPM (60-2 trigger = 58 teeth)
    uint32_t rpm = 1000;
    uint32_t teeth_per_rev = 58;
    uint32_t us_per_rev = 60000000 / rpm;  // 60,000 us at 1000 RPM
    uint32_t us_per_degree = us_per_rev / 720;  // 83 us per degree at 1000 RPM
    
    // Update scheduler RPM
    ret = scheduler_update_rpm(rpm);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Test angle to time conversion
    uint32_t angle_deg = 180;  // Half revolution
    uint32_t expected_time = us_per_degree * angle_deg;  // 15,000 us
    
    uint32_t actual_time;
    ret = scheduler_angle_to_time(angle_deg, &actual_time);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Allow 5% tolerance
    uint32_t tolerance = expected_time / 20;
    TEST_ASSERT_UINT32_WITHIN(tolerance, expected_time, actual_time);
}

void test_scheduler_high_rpm_timing(void) {
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Test at 6000 RPM
    uint32_t rpm = 6000;
    uint32_t us_per_rev = 60000000 / rpm;  // 10,000 us at 6000 RPM
    uint32_t us_per_degree = us_per_rev / 720;  // 14 us per degree at 6000 RPM
    
    ret = scheduler_update_rpm(rpm);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Test precise timing at high RPM
    uint32_t angle_deg = 10;  // Small angle
    uint32_t expected_time = us_per_degree * angle_deg;  // 140 us
    
    uint32_t actual_time;
    ret = scheduler_angle_to_time(angle_deg, &actual_time);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // High precision required at high RPM
    uint32_t tolerance = expected_time / 50;  // 2% tolerance
    TEST_ASSERT_UINT32_WITHIN(tolerance, expected_time, actual_time);
}

void test_scheduler_event_priority(void) {
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint32_t current_time = mock_hal_timer_get_time();
    
    // Schedule events with different priorities
    ret = scheduler_schedule_event_priority(current_time + 2000, test_event_callback, NULL, SCHED_PRIORITY_LOW);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_schedule_event_priority(current_time + 1000, test_event_callback, NULL, SCHED_PRIORITY_HIGH);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_schedule_event_priority(current_time + 1500, test_event_callback, NULL, SCHED_PRIORITY_NORMAL);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // High priority event should execute first even if scheduled later
    mock_hal_timer_set_time(current_time + 1000);
    ret = scheduler_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(callback_called);
}

void test_scheduler_overflow_handling(void) {
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Test with time near overflow (32-bit)
    mock_hal_timer_set_time(0xFFFFFFF0);
    
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t event_time = current_time + 1000;  // This will overflow
    
    ret = scheduler_schedule_event(event_time, test_event_callback, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Advance time past overflow
    mock_hal_timer_set_time(event_time);
    
    ret = scheduler_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    TEST_ASSERT_TRUE(callback_called);
}

void test_scheduler_performance(void) {
    const performance_test_t* perf_test = &PERFORMANCE_TESTS[2];  // 6000 RPM test
    
    esp_err_t ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_update_rpm(perf_test->rpm);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Measure scheduling performance
    uint32_t start_time = mock_hal_timer_get_time();
    
    // Schedule maximum events
    for (int i = 0; i < test_config.max_events; i++) {
        uint32_t event_time = start_time + (i + 1) * perf_test->expected_tooth_period_us;
        ret = scheduler_schedule_event(event_time, test_event_callback, NULL);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }
    
    uint32_t schedule_time = mock_hal_timer_get_time() - start_time;
    
    // Verify scheduling performance is within limits
    TEST_ASSERT_UINT32_WITHIN(perf_test->max_latency_us, 0, schedule_time);
    
    // Test processing performance
    start_time = mock_hal_timer_get_time();
    
    ret = scheduler_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint32_t process_time = mock_hal_timer_get_time() - start_time;
    
    // Processing should be very fast
    TEST_ASSERT_UINT32_WITHIN(perf_test->max_jitter_us, 0, process_time);
}

void test_scheduler_error_handling(void) {
    esp_err_t ret;
    
    // Test operations without initialization
    ret = scheduler_start();
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_schedule_event(1000, test_event_callback, NULL);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_get_state(&test_state);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    // Test double initialization
    ret = scheduler_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = scheduler_init(&test_config);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);  // Should fail if already initialized
    
    scheduler_deinit();
}

// Test runner
int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_scheduler_init_default_config);
    RUN_TEST(test_scheduler_init_invalid_config);
    RUN_TEST(test_scheduler_start_stop);
    RUN_TEST(test_scheduler_schedule_single_event);
    RUN_TEST(test_scheduler_schedule_multiple_events);
    RUN_TEST(test_scheduler_angle_to_time_conversion);
    RUN_TEST(test_scheduler_high_rpm_timing);
    RUN_TEST(test_scheduler_event_priority);
    RUN_TEST(test_scheduler_overflow_handling);
    RUN_TEST(test_scheduler_performance);
    RUN_TEST(test_scheduler_error_handling);
    
    return UNITY_END();
}
