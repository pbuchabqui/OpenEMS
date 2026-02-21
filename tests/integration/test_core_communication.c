/**
 * @file test_core_communication.c
 * @brief Integration tests for Core 0 ↔ Core 1 communication
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "unity.h"
#include "esp_err.h"
#include "utils/atomic_buffer.h"
#include "mocks/mock_hal_timer.h"
#include "fixtures/engine_test_data.h"

// Test data structures
typedef struct {
    uint32_t rpm;
    float map_kpa;
    float tps_percent;
    uint32_t tooth_index;
    bool sync_acquired;
    uint32_t timestamp_us;
} core0_to_core1_data_t;

typedef struct {
    float fuel_pulse_us;
    float ignition_timing_deg;
    uint8_t injection_channel;
    uint8_t ignition_channel;
    uint32_t timestamp_us;
} core1_to_core0_data_t;

// Test state
static atomic_buf_t g_test_buffer_core0_to_core1;
static atomic_buf_t g_test_buffer_core1_to_core0;
static core0_to_core1_data_t test_core0_data;
static core1_to_core0_data_t test_core1_data;
static uint32_t write_count = 0;
static uint32_t read_count = 0;

void setUp(void) {
    // Reset all mocks
    mock_hal_timer_reset();
    
    // Reset test state
    write_count = 0;
    read_count = 0;
    memset(&test_core0_data, 0, sizeof(test_core0_data));
    memset(&test_core1_data, 0, sizeof(test_core1_data));
    memset(&g_test_buffer_core0_to_core1, 0, sizeof(g_test_buffer_core0_to_core1));
    memset(&g_test_buffer_core1_to_core0, 0, sizeof(g_test_buffer_core1_to_core0));
    
    // Initialize test data
    test_core0_data.rpm = 2500;
    test_core0_data.map_kpa = 70.0f;
    test_core0_data.tps_percent = 25.0f;
    test_core0_data.tooth_index = 15;
    test_core0_data.sync_acquired = true;
    test_core0_data.timestamp_us = mock_hal_timer_get_time();
    
    test_core1_data.fuel_pulse_us = 5000.0f;
    test_core1_data.ignition_timing_deg = 25.0f;
    test_core1_data.injection_channel = 0;
    test_core1_data.ignition_channel = 0;
    test_core1_data.timestamp_us = mock_hal_timer_get_time();
}

void tearDown(void) {
    // Cleanup
}

void test_atomic_buffer_basic_write_read(void) {
    // Write data from Core 0
    atomic_buf_write(&g_test_buffer_core0_to_core1, 
                     &test_core0_data, 
                     sizeof(test_core0_data));
    write_count++;
    
    // Read data from Core 1
    core0_to_core1_data_t read_data;
    atomic_buf_read(&g_test_buffer_core0_to_core1, 
                   &read_data, 
                   sizeof(read_data));
    read_count++;
    
    // Verify data integrity
    TEST_ASSERT_EQUAL_UINT32(test_core0_data.rpm, read_data.rpm);
    TEST_ASSERT_EQUAL_FLOAT(test_core0_data.map_kpa, read_data.map_kpa);
    TEST_ASSERT_EQUAL_FLOAT(test_core0_data.tps_percent, read_data.tps_percent);
    TEST_ASSERT_EQUAL_UINT32(test_core0_data.tooth_index, read_data.tooth_index);
    TEST_ASSERT_EQUAL(test_core0_data.sync_acquired, read_data.sync_acquired);
    TEST_ASSERT_EQUAL_UINT32(test_core0_data.timestamp_us, read_data.timestamp_us);
}

void test_atomic_buffer_concurrent_access(void) {
    // Simulate concurrent writes from Core 0
    for (int i = 0; i < 100; i++) {
        test_core0_data.rpm = 1000 + i * 50;
        test_core0_data.tooth_index = i % 58;
        test_core0_data.timestamp_us = mock_hal_timer_get_time();
        
        atomic_buf_write(&g_test_buffer_core0_to_core1, 
                         &test_core0_data, 
                         sizeof(test_core0_data));
        write_count++;
        
        mock_hal_timer_increment(100);  // Simulate time passing
    }
    
    // Simulate concurrent reads from Core 1
    core0_to_core1_data_t read_data;
    uint32_t successful_reads = 0;
    
    for (int i = 0; i < 100; i++) {
        atomic_buf_read(&g_test_buffer_core0_to_core1, 
                       &read_data, 
                       sizeof(read_data));
        
        // Verify data consistency
        if (read_data.rpm >= 1000 && read_data.rpm <= 6000) {
            successful_reads++;
            read_count++;
        }
        
        mock_hal_timer_increment(10);
    }
    
    // Should have successful reads
    TEST_ASSERT_GREATER_THAN(50, successful_reads);
    TEST_ASSERT_EQUAL_UINT32(write_count, 100);
}

void test_atomic_buffer_bidirectional_communication(void) {
    // Core 0 writes sensor data
    atomic_buf_write(&g_test_buffer_core0_to_core1, 
                     &test_core0_data, 
                     sizeof(test_core0_data));
    
    // Core 1 reads sensor data and writes control data
    core0_to_core1_data_t sensor_data;
    atomic_buf_read(&g_test_buffer_core0_to_core1, 
                   &sensor_data, 
                   sizeof(sensor_data));
    
    // Core 1 processes data and writes response
    test_core1_data.fuel_pulse_us = sensor_data.rpm * 2.0f;  // Simple calculation
    test_core1_data.ignition_timing_deg = 25.0f + (sensor_data.rpm / 1000.0f);
    test_core1_data.timestamp_us = mock_hal_timer_get_time();
    
    atomic_buf_write(&g_test_buffer_core1_to_core0, 
                     &test_core1_data, 
                     sizeof(test_core1_data));
    
    // Core 0 reads control data
    core1_to_core0_data_t control_data;
    atomic_buf_read(&g_test_buffer_core1_to_core0, 
                   &control_data, 
                   sizeof(control_data));
    
    // Verify bidirectional communication
    TEST_ASSERT_EQUAL_FLOAT(test_core1_data.fuel_pulse_us, control_data.fuel_pulse_us);
    TEST_ASSERT_EQUAL_FLOAT(test_core1_data.ignition_timing_deg, control_data.ignition_timing_deg);
    TEST_ASSERT_EQUAL_UINT8(test_core1_data.injection_channel, control_data.injection_channel);
    TEST_ASSERT_EQUAL_UINT8(test_core1_data.ignition_channel, control_data.ignition_channel);
}

void test_atomic_buffer_high_frequency_updates(void) {
    const performance_test_t* perf_test = &PERFORMANCE_TESTS[2];  // 6000 RPM test
    
    // Simulate high-frequency updates at 6000 RPM
    uint32_t update_interval = perf_test->expected_tooth_period_us;  // 172us
    uint32_t num_updates = 1000;
    
    uint32_t start_time = mock_hal_timer_get_time();
    
    // Core 0: High-frequency sensor updates
    for (uint32_t i = 0; i < num_updates; i++) {
        test_core0_data.rpm = 6000;
        test_core0_data.tooth_index = i % 58;
        test_core0_data.timestamp_us = start_time + i * update_interval;
        
        atomic_buf_write(&g_test_buffer_core0_to_core1, 
                         &test_core0_data, 
                         sizeof(test_core0_data));
        
        mock_hal_timer_set_time(start_time + i * update_interval);
    }
    
    uint32_t write_time = mock_hal_timer_get_time() - start_time;
    
    // Core 1: High-frequency reads
    uint32_t successful_reads = 0;
    uint32_t read_start_time = mock_hal_timer_get_time();
    
    for (uint32_t i = 0; i < num_updates; i++) {
        core0_to_core1_data_t read_data;
        atomic_buf_read(&g_test_buffer_core0_to_core1, 
                       &read_data, 
                       sizeof(read_data));
        
        // Verify data consistency
        if (read_data.rpm == 6000 && read_data.sync_acquired) {
            successful_reads++;
        }
        
        mock_hal_timer_increment(update_interval / 2);  // Read at twice the write frequency
    }
    
    uint32_t read_time = mock_hal_timer_get_time() - read_start_time;
    
    // Verify performance
    TEST_ASSERT_GREATER_THAN(num_updates * 0.9, successful_reads);  // 90% success rate
    TEST_ASSERT_UINT32_WITHIN(perf_test->max_latency_us, 0, write_time / num_updates);
    TEST_ASSERT_UINT32_WITHIN(perf_test->max_latency_us, 0, read_time / num_updates);
}

void test_atomic_buffer_memory_consistency(void) {
    // Test with various data patterns to catch memory consistency issues
    core0_to_core1_data_t test_patterns[] = {
        {.rpm = 0, .map_kpa = 0.0f, .tps_percent = 0.0f, .tooth_index = 0, .sync_acquired = false, .timestamp_us = 0},
        {.rpm = UINT32_MAX, .map_kpa = 200.0f, .tps_percent = 100.0f, .tooth_index = 57, .sync_acquired = true, .timestamp_us = UINT32_MAX},
        {.rpm = 8000, .map_kpa = 101.3f, .tps_percent = 50.5f, .tooth_index = 29, .sync_acquired = true, .timestamp_us = 1234567890},
        {.rpm = 500, .map_kpa = 20.5f, .tps_percent = 2.1f, .tooth_index = 1, .sync_acquired = false, .timestamp_us = 987654321}
    };
    
    for (int i = 0; i < 4; i++) {
        // Write test pattern
        atomic_buf_write(&g_test_buffer_core0_to_core1, 
                         &test_patterns[i], 
                         sizeof(test_patterns[i]));
        
        // Read back immediately
        core0_to_core1_data_t read_data;
        atomic_buf_read(&g_test_buffer_core0_to_core1, 
                       &read_data, 
                       sizeof(read_data));
        
        // Verify exact match
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(test_patterns[i].rpm, read_data.rpm, "RPM mismatch");
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(test_patterns[i].map_kpa, read_data.map_kpa, "MAP mismatch");
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(test_patterns[i].tps_percent, read_data.tps_percent, "TPS mismatch");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(test_patterns[i].tooth_index, read_data.tooth_index, "Tooth index mismatch");
        TEST_ASSERT_EQUAL_MESSAGE(test_patterns[i].sync_acquired, read_data.sync_acquired, "Sync acquired mismatch");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(test_patterns[i].timestamp_us, read_data.timestamp_us, "Timestamp mismatch");
    }
}

void test_atomic_buffer_rollover_handling(void) {
    // Test with 32-bit timestamp rollover
    mock_hal_timer_set_time(0xFFFFFFF0);  // Near rollover
    
    // Write data before rollover
    test_core0_data.timestamp_us = 0xFFFFFFFF;
    atomic_buf_write(&g_test_buffer_core0_to_core1, 
                     &test_core0_data, 
                     sizeof(test_core0_data));
    
    // Read data before rollover
    core0_to_core1_data_t read_data;
    atomic_buf_read(&g_test_buffer_core0_to_core1, 
                   &read_data, 
                   sizeof(read_data));
    
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFF, read_data.timestamp_us);
    
    // Advance time past rollover
    mock_hal_timer_set_time(0x00001000);
    
    // Write data after rollover
    test_core0_data.timestamp_us = 0x00001000;
    atomic_buf_write(&g_test_buffer_core0_to_core1, 
                     &test_core0_data, 
                     sizeof(test_core0_data));
    
    // Read data after rollover
    atomic_buf_read(&g_test_buffer_core0_to_core1, 
                   &read_data, 
                   sizeof(read_data));
    
    TEST_ASSERT_EQUAL_UINT32(0x00001000, read_data.timestamp_us);
}

void test_atomic_buffer_size_validation(void) {
    // Test with data that's too large for buffer
    char large_data[300];  // Larger than buffer (256 bytes)
    memset(large_data, 0xAA, sizeof(large_data));
    
    // This should handle gracefully (implementation dependent)
    // For now, we just verify it doesn't crash
    atomic_buf_write(&g_test_buffer_core0_to_core1, 
                     large_data, 
                     sizeof(large_data));
    
    char read_data[300];
    atomic_buf_read(&g_test_buffer_core0_to_core1, 
                   read_data, 
                   sizeof(read_data));
    
    // The test passes if we get here without crashing
    TEST_PASS();
}

// Test runner
int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_atomic_buffer_basic_write_read);
    RUN_TEST(test_atomic_buffer_concurrent_access);
    RUN_TEST(test_atomic_buffer_bidirectional_communication);
    RUN_TEST(test_atomic_buffer_high_frequency_updates);
    RUN_TEST(test_atomic_buffer_memory_consistency);
    RUN_TEST(test_atomic_buffer_rollover_handling);
    RUN_TEST(test_atomic_buffer_size_validation);
    
    return UNITY_END();
}
