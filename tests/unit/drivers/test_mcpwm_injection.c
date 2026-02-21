/**
 * @file test_mcpwm_injection.c
 * @brief Unit tests for MCPWM high-precision injection driver
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "unity.h"
#include "esp_err.h"
#include "drivers/mcpwm_injection_hp.h"
#include "mocks/mock_hal_timer.h"
#include "mocks/mock_hal_gpio.h"
#include "fixtures/engine_test_data.h"

// Test state
static mcpwm_injection_config_t test_config;
static mcpwm_injection_state_t test_state;
static bool injection_callback_called = false;
static uint32_t last_injection_time = 0;
static uint8_t last_injection_channel = 0;

// Test callback function
static void test_injection_callback(uint8_t channel, uint32_t scheduled_time, void* ctx) {
    injection_callback_called = true;
    last_injection_time = mock_hal_timer_get_time();
    last_injection_channel = channel;
}

void setUp(void) {
    // Reset all mocks
    mock_hal_timer_reset();
    mock_hal_gpio_reset();
    
    // Reset test state
    injection_callback_called = false;
    last_injection_time = 0;
    last_injection_channel = 0;
    memset(&test_config, 0, sizeof(test_config));
    memset(&test_state, 0, sizeof(test_state));
    
    // Setup default test configuration
    test_config.num_channels = 4;
    test_config.timer_resolution_hz = 1000000;  // 1MHz = 1us resolution
    test_config.max_pulse_width_us = 25000;    // 25ms max pulse
    test_config.min_pulse_width_us = 100;       // 100us min pulse
    test_config.enable_deadtime = true;
    test_config.deadtime_us = 50;
}

void tearDown(void) {
    mcpwm_injection_deinit();
}

void test_mcpwm_injection_init_default_config(void) {
    esp_err_t ret = mcpwm_injection_init(&test_config);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify driver is ready
    ret = mcpwm_injection_get_state(&test_state);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(true, test_state.initialized);
    TEST_ASSERT_EQUAL(false, test_state.running);
}

void test_mcpwm_injection_init_invalid_config(void) {
    // Test with invalid configuration
    test_config.num_channels = 0;  // Invalid
    
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
}

void test_mcpwm_injection_start_stop(void) {
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify running state
    ret = mcpwm_injection_get_state(&test_state);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(true, test_state.running);
    
    ret = mcpwm_injection_stop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify stopped state
    ret = mcpwm_injection_get_state(&test_state);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(false, test_state.running);
}

void test_mcpwm_injection_single_pulse(void) {
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Schedule single injection pulse
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t pulse_width_us = 5000;  // 5ms pulse
    uint32_t start_time = current_time + 1000;  // Start in 1ms
    
    ret = mcpwm_injection_schedule_pulse(0, start_time, pulse_width_us);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Advance time to start of pulse
    mock_hal_timer_set_time(start_time);
    
    // Process injection events
    ret = mcpwm_injection_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify injector turned on
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_1, true);
    
    // Advance time to end of pulse
    mock_hal_timer_set_time(start_time + pulse_width_us);
    
    ret = mcpwm_injection_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify injector turned off
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_1, false);
}

void test_mcpwm_injection_multiple_channels(void) {
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint32_t current_time = mock_hal_timer_get_time();
    
    // Schedule pulses on all channels
    for (int channel = 0; channel < 4; channel++) {
        uint32_t start_time = current_time + (channel + 1) * 2000;  // Staggered starts
        uint32_t pulse_width = 3000 + channel * 500;  // Different widths
        
        ret = mcpwm_injection_schedule_pulse(channel, start_time, pulse_width);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }
    
    // Process events for each channel
    for (int channel = 0; channel < 4; channel++) {
        uint32_t start_time = current_time + (channel + 1) * 2000;
        uint32_t pulse_width = 3000 + channel * 500;
        
        // Start pulse
        mock_hal_timer_set_time(start_time);
        ret = mcpwm_injection_process_events();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        // Verify correct injector is on
        uint32_t expected_pin;
        switch (channel) {
            case 0: expected_pin = HAL_PIN_INJ_1; break;
            case 1: expected_pin = HAL_PIN_INJ_2; break;
            case 2: expected_pin = HAL_PIN_INJ_3; break;
            case 3: expected_pin = HAL_PIN_INJ_4; break;
        }
        MOCK_HAL_GPIO_ASSERT_STATE(expected_pin, true);
        
        // End pulse
        mock_hal_timer_set_time(start_time + pulse_width);
        ret = mcpwm_injection_process_events();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        
        // Verify injector is off
        MOCK_HAL_GPIO_ASSERT_STATE(expected_pin, false);
    }
}

void test_mcpwm_injection_timing_precision(void) {
    const performance_test_t* perf_test = &PERFORMANCE_TESTS[2];  // 6000 RPM test
    
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Test high-precision timing at 6000 RPM
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t pulse_width_us = 2000;  // 2ms pulse
    uint32_t start_time = current_time + 100;  // Start in 100us
    
    ret = mcpwm_injection_schedule_pulse(0, start_time, pulse_width_us);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Measure timing precision
    uint32_t actual_start_time = 0;
    uint32_t actual_end_time = 0;
    
    mock_hal_timer_set_auto_increment(true, 1);  // 1us increments
    
    // Process until pulse starts
    while (!actual_start_time) {
        mock_hal_timer_increment(1);
        ret = mcpwm_injection_process_events();
        if (mock_hal_gpio_get_state(HAL_PIN_INJ_1)) {
            actual_start_time = mock_hal_timer_get_time();
        }
    }
    
    // Process until pulse ends
    while (mock_hal_gpio_get_state(HAL_PIN_INJ_1)) {
        mock_hal_timer_increment(1);
        ret = mcpwm_injection_process_events();
        if (!mock_hal_gpio_get_state(HAL_PIN_INJ_1)) {
            actual_end_time = mock_hal_timer_get_time();
        }
    }
    
    // Verify timing precision
    uint32_t start_error = (actual_start_time > start_time) ? 
                          (actual_start_time - start_time) : 
                          (start_time - actual_start_time);
    uint32_t actual_pulse_width = actual_end_time - actual_start_time;
    uint32_t pulse_error = (actual_pulse_width > pulse_width_us) ? 
                           (actual_pulse_width - pulse_width_us) : 
                           (pulse_width_us - actual_pulse_width);
    
    TEST_ASSERT_UINT32_WITHIN(perf_test->max_jitter_us, 0, start_error);
    TEST_ASSERT_UINT32_WITHIN(perf_test->max_jitter_us, 0, pulse_error);
}

void test_mcpwm_injection_deadtime(void) {
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t pulse_width_us = 3000;
    uint32_t start_time = current_time + 1000;
    
    // Schedule overlapping pulses on adjacent channels
    ret = mcpwm_injection_schedule_pulse(0, start_time, pulse_width_us);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_schedule_pulse(1, start_time + 1000, pulse_width_us);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Process first pulse
    mock_hal_timer_set_time(start_time);
    ret = mcpwm_injection_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify first injector is on
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_1, true);
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_2, false);
    
    // Try to start second pulse during first pulse
    mock_hal_timer_set_time(start_time + 1000);
    ret = mcpwm_injection_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Second injector should wait for deadtime after first ends
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_1, true);
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_2, false);
    
    // End first pulse
    mock_hal_timer_set_time(start_time + pulse_width_us);
    ret = mcpwm_injection_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // First injector should be off
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_1, false);
    
    // Wait for deadtime
    mock_hal_timer_set_time(start_time + pulse_width_us + test_config.deadtime_us);
    ret = mcpwm_injection_process_events();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Second injector should now be on
    MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_2, true);
}

void test_mcpwm_injection_pulse_width_limits(void) {
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t start_time = current_time + 1000;
    
    // Test pulse width too small
    ret = mcpwm_injection_schedule_pulse(0, start_time, test_config.min_pulse_width_us - 1);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    // Test pulse width too large
    ret = mcpwm_injection_schedule_pulse(0, start_time, test_config.max_pulse_width_us + 1);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    // Test valid pulse widths
    ret = mcpwm_injection_schedule_pulse(0, start_time, test_config.min_pulse_width_us);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_schedule_pulse(1, start_time, test_config.max_pulse_width_us);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_mcpwm_injection_high_rpm_performance(void) {
    const performance_test_t* perf_test = &PERFORMANCE_TESTS[2];  // 6000 RPM test
    
    esp_err_t ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Simulate high RPM operation with many pulses
    uint32_t current_time = mock_hal_timer_get_time();
    uint32_t pulse_interval = perf_test->expected_tooth_period_us;  // 172us at 6000 RPM
    
    // Schedule pulses for multiple revolutions
    for (int rev = 0; rev < 10; rev++) {
        for (int cyl = 0; cyl < 4; cyl++) {
            uint32_t start_time = current_time + rev * 720 * pulse_interval + cyl * 180 * pulse_interval;
            uint32_t pulse_width = 2000;  // 2ms pulse
            
            ret = mcpwm_injection_schedule_pulse(cyl, start_time, pulse_width);
            TEST_ASSERT_EQUAL(ESP_OK, ret);
        }
    }
    
    // Measure processing performance
    uint32_t start_process_time = mock_hal_timer_get_time();
    
    // Process all events
    for (int i = 0; i < 40; i++) {  // 10 revs * 4 cylinders
        mock_hal_timer_set_time(current_time + i * pulse_interval);
        ret = mcpwm_injection_process_events();
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }
    
    uint32_t process_time = mock_hal_timer_get_time() - start_process_time;
    
    // Verify performance is within limits
    TEST_ASSERT_UINT32_WITHIN(perf_test->max_latency_us, 0, process_time);
}

void test_mcpwm_injection_error_handling(void) {
    esp_err_t ret;
    
    // Test operations without initialization
    ret = mcpwm_injection_start();
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_schedule_pulse(0, 1000, 2000);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_get_state(&test_state);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    // Test invalid channel
    ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = mcpwm_injection_schedule_pulse(4, 1000, 2000);  // Channel 4 doesn't exist
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    // Test double initialization
    ret = mcpwm_injection_init(&test_config);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);  // Should fail if already initialized
    
    mcpwm_injection_deinit();
}

// Test runner
int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_mcpwm_injection_init_default_config);
    RUN_TEST(test_mcpwm_injection_init_invalid_config);
    RUN_TEST(test_mcpwm_injection_start_stop);
    RUN_TEST(test_mcpwm_injection_single_pulse);
    RUN_TEST(test_mcpwm_injection_multiple_channels);
    RUN_TEST(test_mcpwm_injection_timing_precision);
    RUN_TEST(test_mcpwm_injection_deadtime);
    RUN_TEST(test_mcpwm_injection_pulse_width_limits);
    RUN_TEST(test_mcpwm_injection_high_rpm_performance);
    RUN_TEST(test_mcpwm_injection_error_handling);
    
    return UNITY_END();
}
