/**
 * @file latency_benchmark.h
 * @brief Latency measurement and benchmarking utilities for OpenEMS
 * 
 * Provides high-precision timing measurements for critical paths:
 * - ISR trigger processing time
 * - Event scheduling latency  
 * - Cross-core communication overhead
 * - MCPWM timing accuracy
 */

#ifndef LATENCY_BENCHMARK_H
#define LATENCY_BENCHMARK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Statistics structure for latency measurements
typedef struct {
    uint32_t min_us;          // Minimum latency observed
    uint32_t max_us;          // Maximum latency observed
    uint32_t avg_us;          // Running average latency
    uint32_t p95_us;         // 95th percentile latency
    uint32_t p99_us;         // 99th percentile latency
    uint32_t total_us;        // Cumulative total (for averaging)
    uint32_t sample_count;     // Number of samples collected
    uint32_t last_start_us;   // Last measurement start time
} latency_stats_t;

// Initialize benchmark system
void latency_benchmark_init(void);

// Enable/disable benchmarking
void latency_benchmark_enable(bool enable);

// ISR timing measurements
void latency_benchmark_start_isr(void);
void latency_benchmark_end_isr(void);

// Event scheduler timing measurements
void latency_benchmark_start_scheduler(void);
void latency_benchmark_end_scheduler(void);

// Cross-core communication timing measurements
void latency_benchmark_start_crosscore(void);
void latency_benchmark_end_crosscore(void);

// MCPWM timing measurements
void latency_benchmark_start_mcpwm(void);
void latency_benchmark_end_mcpwm(void);

// Get statistics
void latency_benchmark_get_isr_stats(latency_stats_t *stats);
void latency_benchmark_get_scheduler_stats(latency_stats_t *stats);
void latency_benchmark_get_crosscore_stats(latency_stats_t *stats);
void latency_benchmark_get_mcpwm_stats(latency_stats_t *stats);

// Print comprehensive summary
void latency_benchmark_print_summary(void);

// Reset all statistics
void latency_benchmark_reset(void);

// Convenience macros for automatic timing
#define BENCHMARK_ISR_START() latency_benchmark_start_isr()
#define BENCHMARK_ISR_END() latency_benchmark_end_isr()

#define BENCHMARK_SCHEDULER_START() latency_benchmark_start_scheduler()
#define BENCHMARK_SCHEDULER_END() latency_benchmark_end_scheduler()

#define BENCHMARK_CROSSCORE_START() latency_benchmark_start_crosscore()
#define BENCHMARK_CROSSCORE_END() latency_benchmark_end_crosscore()

#define BENCHMARK_MCPWM_START() latency_benchmark_start_mcpwm()
#define BENCHMARK_MCPWM_END() latency_benchmark_end_mcpwm()

// Target performance metrics (in microseconds)
#define ISR_TARGET_AVG_US      5
#define ISR_TARGET_P99_US     10

#define SCHEDULER_TARGET_AVG_US  10
#define SCHEDULER_TARGET_P99_US 20

#define CROSSCORE_TARGET_AVG_US  20
#define CROSSCORE_TARGET_P99_US  50

#define MCPWM_TARGET_AVG_US     2
#define MCPWM_TARGET_P99_US     5

#ifdef __cplusplus
}
#endif

#endif // LATENCY_BENCHMARK_H
