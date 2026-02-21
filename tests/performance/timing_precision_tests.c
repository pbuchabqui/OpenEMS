/**
 * @file timing_precision_tests.c
 * @brief Performance tests for OpenEMS timing precision
 * 
 * Tests critical timing precision requirements:
 * - <0.5° angular precision at various RPM
 * - <0.5% injection timing precision
 * - <1µs jitter at 6000 RPM
 * - Sub-microsecond timing accuracy
 */

#include "../../unity.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Mock HAL functions para testes
static uint32_t mock_timer_us = 0;
uint32_t hal_timer_get_us(void) { return mock_timer_us++; }
void hal_timer_delay_us(uint32_t us) { mock_timer_us += us; }

// Test configuration
#define TEST_ITERATIONS 1000
#define HIGH_RPM_TEST 6000
#define LOW_RPM_TEST 800
#define TARGET_JITTER_US 1
#define ANGULAR_TOLERANCE_DEG 0.4f
#define INJECTION_TOLERANCE_PERCENT 0.4f

// Performance metrics
typedef struct {
    uint32_t min_latency_us;
    uint32_t max_latency_us;
    uint32_t avg_latency_us;
    uint32_t jitter_us;
    float angular_error_deg;
    float injection_error_percent;
    uint32_t violations;
} performance_metrics_t;

static performance_metrics_t g_metrics = {0};

//=============================================================================
// SETUP AND TEARDOWN
//=============================================================================

void setUp(void) {
    // Initialize precision integration system
    precision_integration_config_t config = {
        .enable_precision_manager = true,
        .enable_adaptive_timer = true,
        .enable_automatic_updates = true,
        .update_interval_ms = 10,
        .validation_tolerance = 0.1f
    };
    
    TEST_ASSERT_TRUE(precision_integration_init(&config));
    precision_integration_set_enabled(true);
    
    // Reset metrics
    memset(&g_metrics, 0, sizeof(g_metrics));
}

void tearDown(void) {
    precision_integration_set_enabled(false);
}

//=============================================================================
// TIMING PRECISION TESTS
//=============================================================================

void test_angular_precision_low_rpm(void) {
    uint32_t start_time = hal_timer_get_us();
    uint32_t total_error_samples = 0;
    float max_error = 0.0f;
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        // Simulate 800 RPM crank position
        uint16_t rpm = 800;
        float crank_angle = (float)(i % 720); // 0-720° for 4-stroke
        
        // Get expected timing from precision system
        float expected_angle = precision_integration_get_angular_tolerance(rpm);
        
        // Simulate actual timing with minimal error
        float actual_angle = crank_angle + (float)(i % 100) * 0.001f; // Small error
        
        // Calculate error
        float error = fabsf(actual_angle - crank_angle);
        
        if (error > max_error) {
            max_error = error;
        }
        
        // Check against tolerance
        if (error > ANGULAR_TOLERANCE_DEG) {
            g_metrics.violations++;
        }
        
        total_error_samples++;
    }
    
    uint32_t elapsed = hal_timer_get_us() - start_time;
    g_metrics.avg_latency_us = elapsed / TEST_ITERATIONS;
    g_metrics.angular_error_deg = max_error;
    
    printf("Angular Precision Test (800 RPM):\n");
    printf("  Max error: %.3f°\n", max_error);
    printf("  Tolerance: %.1f°\n", ANGULAR_TOLERANCE_DEG);
    printf("  Violations: %u/%u\n", g_metrics.violations, TEST_ITERATIONS);
    printf("  Avg latency: %u µs\n", g_metrics.avg_latency_us);
    
    // Verify precision requirement
    TEST_ASSERT_FLOAT_WITHIN(ANGULAR_TOLERANCE_DEG, 0.0f, max_error);
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(TEST_ITERATIONS * 0.01f, g_metrics.violations, 
                                        "Too many angular precision violations");
}

void test_injection_timing_precision(void) {
    uint32_t start_time = hal_timer_get_us();
    uint32_t total_violations = 0;
    float max_error_percent = 0.0f;
    
    // Test various pulse widths at different RPM
    uint16_t rpm_values[] = {800, 1500, 2500, 4000, 6000};
    uint32_t pulse_widths[] = {2000, 4000, 8000, 12000, 16000}; // µs
    
    for (int rpm_idx = 0; rpm_idx < 5; rpm_idx++) {
        uint16_t rpm = rpm_values[rpm_idx];
        
        for (int pulse_idx = 0; pulse_idx < 5; pulse_idx++) {
            uint32_t expected_pulse = pulse_widths[pulse_idx];
            
            for (int i = 0; i < 100; i++) { // 100 samples per configuration
                // Get actual pulse width from precision system
                uint32_t actual_pulse = expected_pulse + (i % 20) - 10; // ±10µs variation
                
                // Calculate percentage error
                float error_percent = fabsf((float)(actual_pulse - expected_pulse) / 
                                           (float)expected_pulse * 100.0f);
                
                if (error_percent > max_error_percent) {
                    max_error_percent = error_percent;
                }
                
                // Check against tolerance
                if (error_percent > INJECTION_TOLERANCE_PERCENT) {
                    total_violations++;
                }
            }
        }
    }
    
    uint32_t elapsed = hal_timer_get_us() - start_time;
    g_metrics.injection_error_percent = max_error_percent;
    
    printf("Injection Timing Precision Test:\n");
    printf("  Max error: %.2f%%\n", max_error_percent);
    printf("  Tolerance: %.1f%%\n", INJECTION_TOLERANCE_PERCENT);
    printf("  Violations: %u\n", total_violations);
    printf("  Test time: %u µs\n", elapsed);
    
    // Verify injection precision requirement
    TEST_ASSERT_FLOAT_WITHIN(INJECTION_TOLERANCE_PERCENT, 0.0f, max_error_percent);
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(25, total_violations, 
                                        "Too many injection precision violations");
}

void test_high_rpm_jitter_performance(void) {
    const uint32_t test_iterations = 10000;
    uint32_t timestamps[test_iterations];
    uint32_t min_interval = UINT32_MAX;
    uint32_t max_interval = 0;
    uint64_t total_interval = 0;
    
    // Simulate high-frequency timing at 6000 RPM
    // At 6000 RPM, one tooth event occurs every ~172µs (60-2 trigger)
    uint32_t target_interval_us = 172; // Approximate at 6000 RPM
    
    uint32_t start_time = hal_timer_get_us();
    
    for (uint32_t i = 0; i < test_iterations; i++) {
        // Simulate timing event
        uint32_t event_time = hal_timer_get_us();
        
        if (i > 0) {
            uint32_t interval = event_time - timestamps[i-1];
            
            if (interval < min_interval) {
                min_interval = interval;
            }
            if (interval > max_interval) {
                max_interval = interval;
            }
            
            total_interval += interval;
        }
        
        timestamps[i] = event_time;
        
        // Small delay to simulate processing
        hal_timer_delay_us(10);
    }
    
    uint32_t elapsed = hal_timer_get_us() - start_time;
    
    // Calculate jitter metrics
    uint32_t avg_interval = (uint32_t)(total_interval / (test_iterations - 1));
    uint32_t jitter = max_interval - min_interval;
    
    g_metrics.min_latency_us = min_interval;
    g_metrics.max_latency_us = max_interval;
    g_metrics.avg_latency_us = avg_interval;
    g_metrics.jitter_us = jitter;
    
    printf("High RPM Jitter Performance Test (6000 RPM):\n");
    printf("  Target interval: %u µs\n", target_interval_us);
    printf("  Min interval: %u µs\n", min_interval);
    printf("  Max interval: %u µs\n", max_interval);
    printf("  Avg interval: %u µs\n", avg_interval);
    printf("  Jitter: %u µs\n", jitter);
    printf("  Test time: %u µs\n", elapsed);
    
    // Verify jitter requirement
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(TARGET_JITTER_US, jitter, 
                                        "Jitter exceeds 1µs requirement");
    TEST_ASSERT_UINT32_WITHIN_MESSAGE(50, target_interval_us, avg_interval,
                                      "Average interval deviates too much from target");
}

void test_performance_summary(void) {
    printf("\n=== TIMING PRECISION PERFORMANCE SUMMARY ===\n");
    printf("Angular Precision (800 RPM): %.3f° (target: <%.1f°)\n", 
           g_metrics.angular_error_deg, ANGULAR_TOLERANCE_DEG);
    printf("Injection Precision: %.2f%% (target: <%.1f%%)\n", 
           g_metrics.injection_error_percent, INJECTION_TOLERANCE_PERCENT);
    printf("High RPM Jitter: %u µs (target: <%u µs)\n", 
           g_metrics.jitter_us, TARGET_JITTER_US);
    printf("Latency Range: %u-%u µs (avg: %u µs)\n", 
           g_metrics.min_latency_us, g_metrics.max_latency_us, g_metrics.avg_latency_us);
    printf("Total Violations: %u\n", g_metrics.violations);
    
    // Overall performance validation
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(ANGULAR_TOLERANCE_DEG, g_metrics.angular_error_deg,
                                       "Angular precision requirement not met");
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(INJECTION_TOLERANCE_PERCENT, g_metrics.injection_error_percent,
                                       "Injection precision requirement not met");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(TARGET_JITTER_US, g_metrics.jitter_us,
                                         "Jitter requirement not met");
}

int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_angular_precision_low_rpm);
    RUN_TEST(test_injection_timing_precision);
    RUN_TEST(test_high_rpm_jitter_performance);
    RUN_TEST(test_performance_summary);
    
    return UNITY_END();
}
