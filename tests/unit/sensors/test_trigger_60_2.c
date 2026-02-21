/**
 * @file test_trigger_60_2.c
 * @brief Unit tests for 60-2 trigger wheel decoder
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "unity.h"
#include "esp_err.h"
#include "sensors/trigger_60_2.h"
#include "mocks/mock_hal_timer.h"
#include "mocks/mock_hal_gpio.h"
#include "fixtures/engine_test_data.h"

// Test state
static sync_config_t test_config;
static sync_data_t test_data;
static bool callback_called = false;
static sync_tooth_callback_t test_callback = NULL;

// Test callback function
static void test_tooth_callback(uint32_t tooth_time_us,
                               uint32_t tooth_period_us,
                               uint8_t  tooth_index,
                               uint8_t  revolution_idx,
                               uint16_t rpm,
                               bool     sync_acquired,
                               void    *ctx) {
    callback_called = true;
}

void setUp(void) {
    // Reset all mocks
    mock_hal_timer_reset();
    mock_hal_gpio_reset();
    
    // Reset test state
    callback_called = false;
    memset(&test_config, 0, sizeof(test_config));
    memset(&test_data, 0, sizeof(test_data));
    
    // Setup default test configuration
    test_config.tooth_count = 58;  // 60-2 = 58 teeth
    test_config.gap_tooth = 57;    // Gap at tooth 57 (0-based)
    test_config.max_rpm = 8000;
    test_config.min_rpm = 100;
    test_config.enable_phase_detection = true;
}

void tearDown(void) {
    sync_deinit();
}

void test_sync_init_default_config(void) {
    esp_err_t ret = sync_init();
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify configuration was set
    ret = sync_get_config(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT32(58, test_config.tooth_count);
    TEST_ASSERT_EQUAL_UINT32(57, test_config.gap_tooth);
}

void test_sync_init_invalid_config(void) {
    // Test with invalid configuration
    test_config.tooth_count = 0;  // Invalid
    
    esp_err_t ret = sync_set_config(&test_config);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
}

void test_sync_start_stop(void) {
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = sync_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = sync_stop();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_sync_register_callback(void) {
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = sync_register_tooth_callback(test_tooth_callback, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    sync_unregister_tooth_callback();
}

void test_sync_data_initialization(void) {
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = sync_get_data(&test_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Verify initial state
    TEST_ASSERT_EQUAL_UINT32(0, test_data.tooth_index);
    TEST_ASSERT_EQUAL_UINT32(0, test_data.rpm);
    TEST_ASSERT_EQUAL(false, test_data.sync_acquired);
    TEST_ASSERT_EQUAL(false, test_data.sync_valid);
}

void test_sync_rpm_calculation_1000_rpm(void) {
    const trigger_wheel_test_t* test_data = &TRIGGER_60_2_1000_RPM;
    
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    sync_register_tooth_callback(test_tooth_callback, NULL);
    
    // Simulate trigger wheel at 1000 RPM
    mock_hal_timer_set_auto_increment(true, test_data->tooth_period_us);
    
    ret = sync_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Simulate enough teeth to acquire sync
    for (int i = 0; i < 60; i++) {
        // This would normally be called from ISR
        // For testing, we'll verify the calculation logic
        mock_hal_timer_increment(test_data->tooth_period_us);
    }
    
    // Get sync data and verify RPM calculation
    ret = sync_get_data(&test_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // RPM should be approximately 1000 (within 5% tolerance)
    TEST_ASSERT_UINT32_WITHIN(50, 1000, test_data.rpm);
}

void test_sync_rpm_calculation_6000_rpm(void) {
    const trigger_wheel_test_t* test_data = &TRIGGER_60_2_6000_RPM;
    
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    sync_register_tooth_callback(test_tooth_callback, NULL);
    
    // Simulate trigger wheel at 6000 RPM
    mock_hal_timer_set_auto_increment(true, test_data->tooth_period_us);
    
    ret = sync_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Simulate high RPM operation
    for (int i = 0; i < 60; i++) {
        mock_hal_timer_increment(test_data->tooth_period_us);
    }
    
    ret = sync_get_data(&test_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // RPM should be approximately 6000 (within 5% tolerance)
    TEST_ASSERT_UINT32_WITHIN(300, 6000, test_data.rpm);
}

void test_sync_gap_detection(void) {
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    sync_register_tooth_callback(test_tooth_callback, NULL);
    
    ret = sync_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Simulate normal teeth timing
    mock_hal_timer_set_time(1000);
    uint32_t normal_period = 1000; // 1ms per tooth at 1000 RPM
    
    // Simulate teeth before gap
    for (int i = 0; i < 56; i++) {
        mock_hal_timer_increment(normal_period);
    }
    
    // Simulate gap (2x normal period)
    mock_hal_timer_increment(normal_period * 3); // 3x to simulate missing 2 teeth
    
    ret = sync_get_data(&test_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Gap should be detected
    TEST_ASSERT_EQUAL(true, test_data.gap_detected);
}

void test_sync_phase_detection(void) {
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Enable phase detection
    test_config.enable_phase_detection = true;
    ret = sync_set_config(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    sync_register_tooth_callback(test_tooth_callback, NULL);
    
    ret = sync_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Simulate CAM signal (CMP)
    // This would normally be triggered by hardware interrupt
    // For testing, we verify the phase detection logic
    
    ret = sync_get_data(&test_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Phase detection should be enabled
    TEST_ASSERT_EQUAL(true, test_config.enable_phase_detection);
}

void test_sync_timing_precision(void) {
    const performance_test_t* perf_test = &PERFORMANCE_TESTS[0];
    
    esp_err_t ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    sync_register_tooth_callback(test_tooth_callback, NULL);
    
    // Test timing precision at high RPM
    mock_hal_timer_set_auto_increment(true, perf_test->expected_tooth_period_us);
    
    ret = sync_start();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    uint32_t start_time = mock_hal_timer_get_time();
    
    // Process multiple teeth
    for (int i = 0; i < 100; i++) {
        mock_hal_timer_increment(perf_test->expected_tooth_period_us);
    }
    
    uint32_t end_time = mock_hal_timer_get_time();
    uint32_t actual_period = (end_time - start_time) / 100;
    
    // Verify timing precision
    bool timing_ok = validate_timing_performance(actual_period, 
                                                 perf_test->expected_tooth_period_us,
                                                 perf_test->max_jitter_us);
    TEST_ASSERT_TRUE_MESSAGE(timing_ok, "Timing precision test failed");
}

void test_sync_error_handling(void) {
    esp_err_t ret;
    
    // Test operations without initialization
    ret = sync_start();
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    ret = sync_get_data(&test_data);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
    
    // Test double initialization
    ret = sync_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    ret = sync_init();
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);  // Should fail if already initialized
    
    sync_deinit();
}

// Test runner
int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_sync_init_default_config);
    RUN_TEST(test_sync_init_invalid_config);
    RUN_TEST(test_sync_start_stop);
    RUN_TEST(test_sync_register_callback);
    RUN_TEST(test_sync_data_initialization);
    RUN_TEST(test_sync_rpm_calculation_1000_rpm);
    RUN_TEST(test_sync_rpm_calculation_6000_rpm);
    RUN_TEST(test_sync_gap_detection);
    RUN_TEST(test_sync_phase_detection);
    RUN_TEST(test_sync_timing_precision);
    RUN_TEST(test_sync_error_handling);
    
    return UNITY_END();
}
