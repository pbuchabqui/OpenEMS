/**
 * @file system_load_tests.c
 * @brief System load and stress tests for OpenEMS
 * 
 * Tests system performance under various load conditions:
 * - High-frequency event processing
 * - Memory usage under load
 * - Core 0/Core 1 communication performance
 * - Long-duration stability
 */

#include "unity.h"
#include "../fixtures/engine_test_data.h"
#include "../../firmware_restructured/scheduler/precision_integration.h"
#include "../../firmware_restructured/utils/atomic_buffer.h"
#include <stdio.h>
#include <string.h>

// Test configuration
#define STRESS_TEST_DURATION_MS 10000    // 10 seconds
#define HIGH_FREQ_EVENTS 10000         // 10k events
#define MEMORY_TEST_SIZE 1024          // 1KB test blocks
#define COMM_TEST_ITERATIONS 5000       // Core communication tests

// Performance metrics
typedef struct {
    uint32_t events_processed;
    uint32_t events_dropped;
    uint32_t max_latency_us;
    uint32_t avg_latency_us;
    uint32_t memory_peak_kb;
    uint32_t comm_errors;
    float cpu_usage_percent;
} system_load_metrics_t;

static system_load_metrics_t g_load_metrics = {0};
static atomic_buffer_t* g_test_buffer = NULL;

//=============================================================================
// SETUP AND TEARDOWN
//=============================================================================

void setUp(void) {
    // Initialize precision system
    precision_integration_config_t config = {
        .enable_precision_manager = true,
        .enable_adaptive_timer = true,
        .enable_automatic_updates = true,
        .update_interval_ms = 1,  // High frequency for stress test
        .validation_tolerance = 0.05f
    };
    
    TEST_ASSERT_TRUE(precision_integration_init(&config));
    precision_integration_set_enabled(true);
    
    // Initialize test buffer for Core 0/Core 1 communication
    g_test_buffer = atomic_buffer_create(1024);
    TEST_ASSERT_NOT_NULL(g_test_buffer);
    
    // Reset metrics
    memset(&g_load_metrics, 0, sizeof(g_load_metrics));
}

void tearDown(void) {
    precision_integration_set_enabled(false);
    if (g_test_buffer) {
        atomic_buffer_destroy(g_test_buffer);
        g_test_buffer = NULL;
    }
}

//=============================================================================
// HIGH-FREQUENCY EVENT PROCESSING TESTS
//=============================================================================

void test_high_frequency_event_processing(void) {
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t events_processed = 0;
    uint32_t max_latency = 0;
    uint64_t total_latency = 0;
    
    printf("Starting high-frequency event processing test...\n");
    
    // Process events at high frequency for specified duration
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time < STRESS_TEST_DURATION_MS) {
        uint32_t event_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Simulate high-frequency engine events
        for (int cylinder = 0; cylinder < 4; cylinder++) {
            uint16_t rpm = 2000 + (events_processed % 4000); // 2000-6000 RPM
            
            // Update precision system
            precision_integration_update(rpm);
            
            // Get timing parameters
            float angular_tolerance = precision_integration_get_angular_tolerance(rpm);
            uint32_t timer_resolution = precision_integration_get_timer_resolution(rpm);
            
            // Validate parameters
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.0f, angular_tolerance, 
                                                  "Invalid angular tolerance");
            TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(0, timer_resolution, 
                                                   "Invalid timer resolution");
            
            events_processed++;
        }
        
        uint32_t event_latency = (xTaskGetTickCount() * portTICK_PERIOD_MS) - event_start;
        
        if (event_latency > max_latency) {
            max_latency = event_latency;
        }
        
        total_latency += event_latency;
        
        // Small delay to prevent CPU overload
        if (events_processed % 1000 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    uint32_t test_duration = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
    
    g_load_metrics.events_processed = events_processed;
    g_load_metrics.max_latency_us = max_latency * 1000; // Convert to µs
    g_load_metrics.avg_latency_us = (uint32_t)(total_latency / events_processed) * 1000;
    
    printf("High-Frequency Event Processing Results:\n");
    printf("  Test duration: %u ms\n", test_duration);
    printf("  Events processed: %u\n", events_processed);
    printf("  Events/second: %.0f\n", (float)events_processed * 1000.0f / test_duration);
    printf("  Max latency: %u µs\n", g_load_metrics.max_latency_us);
    printf("  Avg latency: %u µs\n", g_load_metrics.avg_latency_us);
    
    // Verify performance requirements
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(1000, events_processed, 
                                           "Too few events processed");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(100, g_load_metrics.max_latency_us, 
                                        "Max latency too high");
}

//=============================================================================
// MEMORY USAGE TESTS
//=============================================================================

void test_memory_usage_under_load(void) {
    size_t initial_free = esp_get_free_heap_size();
    uint32_t allocations = 0;
    uint32_t allocations_failed = 0;
    void* test_blocks[100]; // Track up to 100 allocations
    
    memset(test_blocks, 0, sizeof(test_blocks));
    
    printf("Memory Usage Test - Initial free: %u bytes\n", initial_free);
    
    // Simulate memory allocation patterns under load
    for (int cycle = 0; cycle < 10; cycle++) {
        // Allocate memory blocks
        for (int i = 0; i < 100; i++) {
            if (!test_blocks[i]) {
                test_blocks[i] = malloc(MEMORY_TEST_SIZE);
                if (test_blocks[i]) {
                    memset(test_blocks[i], 0xAA, MEMORY_TEST_SIZE);
                    allocations++;
                } else {
                    allocations_failed++;
                }
            }
        }
        
        // Process some precision system updates
        for (int i = 0; i < 1000; i++) {
            uint16_t rpm = 1000 + (i % 5000);
            precision_integration_update(rpm);
        }
        
        // Free some memory blocks
        for (int i = 0; i < 50; i++) {
            if (test_blocks[i] && (cycle % 2 == 0)) {
                free(test_blocks[i]);
                test_blocks[i] = NULL;
            }
        }
        
        // Check memory usage
        size_t current_free = esp_get_free_heap_size();
        if (current_free < initial_free - (initial_free * 0.1)) { // More than 10% memory loss
            printf("Warning: High memory usage detected at cycle %d\n", cycle);
        }
    }
    
    // Clean up remaining allocations
    for (int i = 0; i < 100; i++) {
        if (test_blocks[i]) {
            free(test_blocks[i]);
            test_blocks[i] = NULL;
        }
    }
    
    size_t final_free = esp_get_free_heap_size();
    size_t memory_lost = initial_free - final_free;
    float memory_loss_percent = (float)memory_lost / (float)initial_free * 100.0f;
    
    g_load_metrics.memory_peak_kb = (initial_free - final_free) / 1024;
    
    printf("Memory Usage Results:\n");
    printf("  Initial free: %u bytes\n", initial_free);
    printf("  Final free: %u bytes\n", final_free);
    printf("  Memory lost: %u bytes (%.2f%%)\n", memory_lost, memory_loss_percent);
    printf("  Allocations: %u\n", allocations);
    printf("  Failed allocations: %u\n", allocations_failed);
    
    // Verify memory requirements
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(5.0f, memory_loss_percent, 
                                       "Too much memory lost");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(10, allocations_failed, 
                                        "Too many allocation failures");
}

//=============================================================================
// CORE COMMUNICATION TESTS
//=============================================================================

void test_core_communication_performance(void) {
    uint32_t comm_errors = 0;
    uint32_t successful_writes = 0;
    uint32_t successful_reads = 0;
    
    // Test data structure
    typedef struct {
        uint16_t rpm;
        float timing_advance;
        uint32_t injection_pulse;
        uint8_t cylinder_id;
        uint32_t timestamp;
    } core_data_t;
    
    printf("Core Communication Performance Test...\n");
    
    for (uint32_t i = 0; i < COMM_TEST_ITERATIONS; i++) {
        core_data_t write_data = {
            .rpm = 800 + (i % 5200), // 800-6000 RPM
            .timing_advance = 10.0f + (i % 30), // 10-40°
            .injection_pulse = 2000 + (i % 10000), // 2-12ms
            .cylinder_id = i % 4,
            .timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS
        };
        
        // Write to atomic buffer (simulating Core 0 -> Core 1)
        bool write_success = atomic_buffer_write(g_test_buffer, &write_data, sizeof(write_data));
        if (write_success) {
            successful_writes++;
        } else {
            comm_errors++;
            continue;
        }
        
        // Read from atomic buffer (simulating Core 1 <- Core 0)
        core_data_t read_data;
        bool read_success = atomic_buffer_read(g_test_buffer, &read_data, sizeof(read_data));
        if (read_success) {
            successful_reads++;
            
            // Validate data integrity
            if (read_data.rpm != write_data.rpm ||
                read_data.cylinder_id != write_data.cylinder_id) {
                comm_errors++;
            }
        } else {
            comm_errors++;
        }
        
        // Update precision system during communication test
        precision_integration_update(write_data.rpm);
        
        // Small delay every 1000 iterations
        if (i % 1000 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    float success_rate = (float)successful_reads / (float)COMM_TEST_ITERATIONS * 100.0f;
    g_load_metrics.comm_errors = comm_errors;
    
    printf("Core Communication Results:\n");
    printf("  Test iterations: %u\n", COMM_TEST_ITERATIONS);
    printf("  Successful writes: %u\n", successful_writes);
    printf("  Successful reads: %u\n", successful_reads);
    printf("  Communication errors: %u\n", comm_errors);
    printf("  Success rate: %.2f%%\n", success_rate);
    
    // Verify communication requirements
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(95.0f, success_rate, 
                                           "Communication success rate too low");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(COMM_TEST_ITERATIONS * 0.05f, comm_errors, 
                                         "Too many communication errors");
}

//=============================================================================
// LONG-DURATION STABILITY TEST
//=============================================================================

void test_long_duration_stability(void) {
    const uint32_t stability_duration_ms = 5000; // 5 seconds for demo
    uint32_t precision_updates = 0;
    uint32_t precision_violations = 0;
    
    printf("Long Duration Stability Test (%u ms)...\n", stability_duration_ms);
    
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t last_report_time = start_time;
    
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time < stability_duration_ms) {
        // Simulate varying engine conditions
        uint16_t rpm = 1000 + (precision_updates % 5000); // 1000-6000 RPM
        float load_factor = 0.1f + (precision_updates % 100) / 100.0f; // 0.1-1.0
        
        // Update precision system
        precision_integration_update(rpm);
        precision_updates++;
        
        // Check for precision violations
        float angular_tolerance = precision_integration_get_angular_tolerance(rpm);
        if (angular_tolerance > 2.0f) { // Should never exceed 2.0°
            precision_violations++;
        }
        
        // Report progress every second
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_report_time >= 1000) {
            printf("  Stability: %u updates, %u violations\n", 
                   precision_updates, precision_violations);
            last_report_time = current_time;
        }
        
        // Small delay to prevent CPU overload
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    float violation_rate = (float)precision_violations / (float)precision_updates * 100.0f;
    
    printf("Long Duration Stability Results:\n");
    printf("  Test duration: %u ms\n", stability_duration_ms);
    printf("  Precision updates: %u\n", precision_updates);
    printf("  Precision violations: %u\n", precision_violations);
    printf("  Violation rate: %.3f%%\n", violation_rate);
    printf("  Updates/second: %.0f\n", (float)precision_updates * 1000.0f / stability_duration_ms);
    
    // Verify stability requirements
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.1f, violation_rate, 
                                       "Precision violation rate too high");
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(1000, precision_updates, 
                                            "Too few precision updates");
}

//=============================================================================
// SYSTEM LOAD SUMMARY
//=============================================================================

void test_system_load_summary(void) {
    printf("\n=== SYSTEM LOAD PERFORMANCE SUMMARY ===\n");
    printf("Events Processed: %u\n", g_load_metrics.events_processed);
    printf("Max Event Latency: %u µs\n", g_load_metrics.max_latency_us);
    printf("Avg Event Latency: %u µs\n", g_load_metrics.avg_latency_us);
    printf("Memory Peak Usage: %u KB\n", g_load_metrics.memory_peak_kb);
    printf("Communication Errors: %u\n", g_load_metrics.comm_errors);
    
    // Overall system validation
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(1000, g_load_metrics.events_processed, 
                                           "Insufficient event processing");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(100, g_load_metrics.max_latency_us, 
                                        "Event latency too high");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(100, g_load_metrics.memory_peak_kb, 
                                         "Memory usage too high");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(250, g_load_metrics.comm_errors, 
                                         "Too many communication errors");
}

//=============================================================================
// TEST MAIN
//=============================================================================

int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_high_frequency_event_processing);
    RUN_TEST(test_memory_usage_under_load);
    RUN_TEST(test_core_communication_performance);
    RUN_TEST(test_long_duration_stability);
    RUN_TEST(test_system_load_summary);
    
    return UNITY_END();
}
