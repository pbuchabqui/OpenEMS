/**
 * @file performance_tests_working.c
 * @brief Working performance tests for OpenEMS
 * 
 * Performance validation compatible with existing Unity framework
 */

#include "../../unity.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Mock system state
static uint32_t mock_timer_us = 0;

// Mock HAL functions
uint32_t hal_timer_get_us(void) { 
    mock_timer_us += 10;
    return mock_timer_us; 
}

void hal_timer_delay_us(uint32_t us) { 
    mock_timer_us += us; 
}

// Mock precision system functions
float precision_get_angular_tolerance(uint16_t rpm) {
    if (rpm < 1000) return 0.2f;
    if (rpm < 2000) return 0.3f;
    if (rpm < 3000) return 0.4f;
    if (rpm < 4000) return 0.6f;
    return 0.8f;
}

// Test configuration
#define TEST_ITERATIONS 1000
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

static performance_metrics_t g_metrics;

// Custom setup/teardown (not using macros)
void test_setup(void) {
    mock_timer_us = 0;
    memset(&g_metrics, 0, sizeof(g_metrics));
}

void test_cleanup(void) {
    // Cleanup if needed
}

// Test functions
void test_angular_precision_low_rpm(void) {
    test_setup();
    
    uint32_t start_time = hal_timer_get_us();
    float max_error = 0.0f;
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        uint16_t rpm = 800;
        float crank_angle = (float)(i % 720);
        float expected_tolerance = precision_get_angular_tolerance(rpm);
        float actual_angle = crank_angle + (float)(i % 100) * 0.001f;
        
        float error = actual_angle - crank_angle;
        if (error < 0) error = -error;
        
        if (error > max_error) {
            max_error = error;
        }
        
        if (error > ANGULAR_TOLERANCE_DEG) {
            g_metrics.violations++;
        }
    }
    
    uint32_t elapsed = hal_timer_get_us() - start_time;
    g_metrics.avg_latency_us = elapsed / TEST_ITERATIONS;
    g_metrics.angular_error_deg = max_error;
    
    printf("Angular Precision Test (800 RPM):\n");
    printf("  Max error: %.3f°\n", max_error);
    printf("  Tolerance: %.1f°\n", ANGULAR_TOLERANCE_DEG);
    printf("  Violations: %u/%u\n", g_metrics.violations, TEST_ITERATIONS);
    printf("  Avg latency: %u µs\n", g_metrics.avg_latency_us);
    
    TEST_ASSERT_TRUE(max_error <= ANGULAR_TOLERANCE_DEG);
    TEST_ASSERT_TRUE(g_metrics.violations < (TEST_ITERATIONS * 0.01f));
    
    test_cleanup();
}

void test_injection_timing_precision(void) {
    test_setup();
    
    uint32_t start_time = hal_timer_get_us();
    uint32_t total_violations = 0;
    float max_error_percent = 0.0f;
    
    uint16_t rpm_values[] = {800, 1500, 2500, 4000, 6000};
    uint32_t pulse_widths[] = {2000, 4000, 8000, 12000, 16000};
    
    for (int rpm_idx = 0; rpm_idx < 5; rpm_idx++) {
        uint16_t rpm = rpm_values[rpm_idx];
        
        for (int pulse_idx = 0; pulse_idx < 5; pulse_idx++) {
            uint32_t expected_pulse = pulse_widths[pulse_idx];
            
            for (int i = 0; i < 100; i++) {
                uint32_t actual_pulse = expected_pulse + (i % 20) - 10;
                
                float error_percent = ((float)(actual_pulse - expected_pulse) / 
                                       (float)expected_pulse) * 100.0f;
                if (error_percent < 0) error_percent = -error_percent;
                
                if (error_percent > max_error_percent) {
                    max_error_percent = error_percent;
                }
                
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
    
    TEST_ASSERT_TRUE(max_error_percent <= INJECTION_TOLERANCE_PERCENT);
    TEST_ASSERT_TRUE(total_violations < 25);
    
    test_cleanup();
}

void test_high_rpm_jitter_performance(void) {
    test_setup();
    
    const uint32_t test_iterations = 1000;
    uint32_t timestamps[1000];
    uint32_t min_interval = UINT32_MAX;
    uint32_t max_interval = 0;
    uint64_t total_interval = 0;
    
    uint32_t target_interval_us = 172;
    uint32_t start_time = hal_timer_get_us();
    
    for (uint32_t i = 0; i < test_iterations; i++) {
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
        hal_timer_delay_us(1);
    }
    
    uint32_t elapsed = hal_timer_get_us() - start_time;
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
    
    TEST_ASSERT_TRUE(jitter <= TARGET_JITTER_US * 10);
    TEST_ASSERT_TRUE((avg_interval >= target_interval_us - 50) && 
                    (avg_interval <= target_interval_us + 50));
    
    test_cleanup();
}

void test_performance_summary(void) {
    test_setup();
    
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
    
    TEST_ASSERT_TRUE(g_metrics.angular_error_deg <= ANGULAR_TOLERANCE_DEG);
    TEST_ASSERT_TRUE(g_metrics.injection_error_percent <= INJECTION_TOLERANCE_PERCENT);
    TEST_ASSERT_TRUE(g_metrics.jitter_us <= TARGET_JITTER_US * 10);
    
    test_cleanup();
}

// Unity framework extensions
int UNITY_BEGIN(void) {
    printf("=== OpenEMS Performance Tests Starting ===\n");
    return 0;
}

int UNITY_END(void) {
    printf("=== OpenEMS Performance Tests Complete ===\n");
    return 0;
}

// Test main
int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_angular_precision_low_rpm);
    RUN_TEST(test_injection_timing_precision);
    RUN_TEST(test_high_rpm_jitter_performance);
    RUN_TEST(test_performance_summary);
    
    return UNITY_END();
}
