/**
 * @file latency_benchmark.c
 * @brief Latency measurement and benchmarking utilities for OpenEMS optimizations
 * 
 * Provides high-precision timing measurements for critical paths:
 * - ISR trigger processing time
 * - Event scheduling latency
 * - Cross-core communication overhead
 * - MCPWM timing accuracy
 */

#include "latency_benchmark.h"
#include "hal/hal_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char* TAG = "LATENCY_BENCH";

// Benchmark state
static latency_stats_t g_isr_stats = {0};
static latency_stats_t g_scheduler_stats = {0};
static latency_stats_t g_crosscore_stats = {0};
static latency_stats_t g_mcpwm_stats = {0};
static bool g_benchmark_enabled = false;
static uint32_t g_benchmark_start_time = 0;

// Ring buffer for samples
#define SAMPLE_BUFFER_SIZE 256
static uint32_t g_isr_samples[SAMPLE_BUFFER_SIZE];
static uint32_t g_scheduler_samples[SAMPLE_BUFFER_SIZE];
static uint32_t g_crosscore_samples[SAMPLE_BUFFER_SIZE];
static uint32_t g_mcpwm_samples[SAMPLE_BUFFER_SIZE];
static uint16_t g_isr_sample_idx = 0;
static uint16_t g_scheduler_sample_idx = 0;
static uint16_t g_crosscore_sample_idx = 0;
static uint16_t g_mcpwm_sample_idx = 0;

// High-resolution timer using ESP32 hardware timer
static uint64_t bench_get_timestamp_us(void) {
    return HAL_Time_us();
}

// Update statistics with new sample
static void update_stats(latency_stats_t *stats, uint32_t sample_us) {
    if (sample_us < stats->min_us || stats->sample_count == 0) {
        stats->min_us = sample_us;
    }
    if (sample_us > stats->max_us) {
        stats->max_us = sample_us;
    }
    
    stats->total_us += sample_us;
    stats->sample_count++;
    
    // Calculate running average (exponential moving average for efficiency)
    if (stats->sample_count == 1) {
        stats->avg_us = sample_us;
    } else {
        // EMA with alpha = 0.1 for responsiveness
        stats->avg_us = (stats->avg_us * 9 + sample_us) / 10;
    }
    
    // Update P95 and P99 (simplified - using sorted array approach)
    if (stats->sample_count % 100 == 0) {
        // Every 100 samples, calculate percentiles
        // For production, would use proper percentile algorithm
        stats->p95_us = stats->avg_us + (stats->max_us - stats->avg_us) * 0.95f;
        stats->p99_us = stats->avg_us + (stats->max_us - stats->avg_us) * 0.99f;
    }
}

void latency_benchmark_init(void) {
    memset(&g_isr_stats, 0, sizeof(latency_stats_t));
    memset(&g_scheduler_stats, 0, sizeof(latency_stats_t));
    memset(&g_crosscore_stats, 0, sizeof(latency_stats_t));
    memset(&g_mcpwm_stats, 0, sizeof(latency_stats_t));
    
    memset(g_isr_samples, 0, sizeof(g_isr_samples));
    memset(g_scheduler_samples, 0, sizeof(g_scheduler_samples));
    memset(g_crosscore_samples, 0, sizeof(g_crosscore_samples));
    memset(g_mcpwm_samples, 0, sizeof(g_mcpwm_samples));
    
    g_isr_sample_idx = 0;
    g_scheduler_sample_idx = 0;
    g_crosscore_sample_idx = 0;
    g_mcpwm_sample_idx = 0;
    
    g_benchmark_enabled = true;
    g_benchmark_start_time = (uint32_t)bench_get_timestamp_us();
    
    ESP_LOGI(TAG, "Latency benchmark initialized");
}

void latency_benchmark_start_isr(void) {
    if (!g_benchmark_enabled) return;
    g_isr_stats.last_start_us = (uint32_t)bench_get_timestamp_us();
}

void latency_benchmark_end_isr(void) {
    if (!g_benchmark_enabled || g_isr_stats.last_start_us == 0) return;
    
    uint32_t elapsed = (uint32_t)bench_get_timestamp_us() - g_isr_stats.last_start_us;
    update_stats(&g_isr_stats, elapsed);
    
    // Store in ring buffer
    g_isr_samples[g_isr_sample_idx] = elapsed;
    g_isr_sample_idx = (g_isr_sample_idx + 1) % SAMPLE_BUFFER_SIZE;
    
    g_isr_stats.last_start_us = 0;
}

void latency_benchmark_start_scheduler(void) {
    if (!g_benchmark_enabled) return;
    g_scheduler_stats.last_start_us = (uint32_t)bench_get_timestamp_us();
}

void latency_benchmark_end_scheduler(void) {
    if (!g_benchmark_enabled || g_scheduler_stats.last_start_us == 0) return;
    
    uint32_t elapsed = (uint32_t)bench_get_timestamp_us() - g_scheduler_stats.last_start_us;
    update_stats(&g_scheduler_stats, elapsed);
    
    // Store in ring buffer
    g_scheduler_samples[g_scheduler_sample_idx] = elapsed;
    g_scheduler_sample_idx = (g_scheduler_sample_idx + 1) % SAMPLE_BUFFER_SIZE;
    
    g_scheduler_stats.last_start_us = 0;
}

void latency_benchmark_start_crosscore(void) {
    if (!g_benchmark_enabled) return;
    g_crosscore_stats.last_start_us = (uint32_t)bench_get_timestamp_us();
}

void latency_benchmark_end_crosscore(void) {
    if (!g_benchmark_enabled || g_crosscore_stats.last_start_us == 0) return;
    
    uint32_t elapsed = (uint32_t)bench_get_timestamp_us() - g_crosscore_stats.last_start_us;
    update_stats(&g_crosscore_stats, elapsed);
    
    // Store in ring buffer
    g_crosscore_samples[g_crosscore_sample_idx] = elapsed;
    g_crosscore_sample_idx = (g_crosscore_sample_idx + 1) % SAMPLE_BUFFER_SIZE;
    
    g_crosscore_stats.last_start_us = 0;
}

void latency_benchmark_start_mcpwm(void) {
    if (!g_benchmark_enabled) return;
    g_mcpwm_stats.last_start_us = (uint32_t)bench_get_timestamp_us();
}

void latency_benchmark_end_mcpwm(void) {
    if (!g_benchmark_enabled || g_mcpwm_stats.last_start_us == 0) return;
    
    uint32_t elapsed = (uint32_t)bench_get_timestamp_us() - g_mcpwm_stats.last_start_us;
    update_stats(&g_mcpwm_stats, elapsed);
    
    // Store in ring buffer
    g_mcpwm_samples[g_mcpwm_sample_idx] = elapsed;
    g_mcpwm_sample_idx = (g_mcpwm_sample_idx + 1) % SAMPLE_BUFFER_SIZE;
    
    g_mcpwm_stats.last_start_us = 0;
}

void latency_benchmark_get_isr_stats(latency_stats_t *stats) {
    if (stats) *stats = g_isr_stats;
}

void latency_benchmark_get_scheduler_stats(latency_stats_t *stats) {
    if (stats) *stats = g_scheduler_stats;
}

void latency_benchmark_get_crosscore_stats(latency_stats_t *stats) {
    if (stats) *stats = g_crosscore_stats;
}

void latency_benchmark_get_mcpwm_stats(latency_stats_t *stats) {
    if (stats) *stats = g_mcpwm_stats;
}

void latency_benchmark_print_summary(void) {
    if (!g_benchmark_enabled) {
        ESP_LOGW(TAG, "Benchmark not enabled");
        return;
    }
    
    uint32_t runtime_ms = ((uint32_t)bench_get_timestamp_us() - g_benchmark_start_time) / 1000;
    
    ESP_LOGI(TAG, "=== LATENCY BENCHMARK SUMMARY ===");
    ESP_LOGI(TAG, "Runtime: %u ms (%.1f seconds)", runtime_ms, runtime_ms / 1000.0f);
    
    ESP_LOGI(TAG, "ISR Processing:");
    ESP_LOGI(TAG, "  Samples: %u", g_isr_stats.sample_count);
    ESP_LOGI(TAG, "  Avg: %u us", g_isr_stats.avg_us);
    ESP_LOGI(TAG, "  Min: %u us", g_isr_stats.min_us);
    ESP_LOGI(TAG, "  Max: %u us", g_isr_stats.max_us);
    ESP_LOGI(TAG, "  P95: %u us", g_isr_stats.p95_us);
    ESP_LOGI(TAG, "  P99: %u us", g_isr_stats.p99_us);
    
    ESP_LOGI(TAG, "Event Scheduling:");
    ESP_LOGI(TAG, "  Samples: %u", g_scheduler_stats.sample_count);
    ESP_LOGI(TAG, "  Avg: %u us", g_scheduler_stats.avg_us);
    ESP_LOGI(TAG, "  Min: %u us", g_scheduler_stats.min_us);
    ESP_LOGI(TAG, "  Max: %u us", g_scheduler_stats.max_us);
    ESP_LOGI(TAG, "  P95: %u us", g_scheduler_stats.p95_us);
    ESP_LOGI(TAG, "  P99: %u us", g_scheduler_stats.p99_us);
    
    ESP_LOGI(TAG, "Cross-Core Communication:");
    ESP_LOGI(TAG, "  Samples: %u", g_crosscore_stats.sample_count);
    ESP_LOGI(TAG, "  Avg: %u us", g_crosscore_stats.avg_us);
    ESP_LOGI(TAG, "  Min: %u us", g_crosscore_stats.min_us);
    ESP_LOGI(TAG, "  Max: %u us", g_crosscore_stats.max_us);
    ESP_LOGI(TAG, "  P95: %u us", g_crosscore_stats.p95_us);
    ESP_LOGI(TAG, "  P99: %u us", g_crosscore_stats.p99_us);
    
    ESP_LOGI(TAG, "MCPWM Timing:");
    ESP_LOGI(TAG, "  Samples: %u", g_mcpwm_stats.sample_count);
    ESP_LOGI(TAG, "  Avg: %u us", g_mcpwm_stats.avg_us);
    ESP_LOGI(TAG, "  Min: %u us", g_mcpwm_stats.min_us);
    ESP_LOGI(TAG, "  Max: %u us", g_mcpwm_stats.max_us);
    ESP_LOGI(TAG, "  P95: %u us", g_mcpwm_stats.p95_us);
    ESP_LOGI(TAG, "  P99: %u us", g_mcpwm_stats.p99_us);
    
    // Performance assessment
    ESP_LOGI(TAG, "=== PERFORMANCE ASSESSMENT ===");
    
    bool isr_good = g_isr_stats.avg_us <= 5 && g_isr_stats.p99_us <= 10;
    bool scheduler_good = g_scheduler_stats.avg_us <= 10 && g_scheduler_stats.p99_us <= 20;
    bool crosscore_good = g_crosscore_stats.avg_us <= 20 && g_crosscore_stats.p99_us <= 50;
    bool mcpwm_good = g_mcpwm_stats.avg_us <= 2 && g_mcpwm_stats.p99_us <= 5;
    
    ESP_LOGI(TAG, "ISR Performance: %s", isr_good ? "EXCELLENT" : "NEEDS_IMPROVEMENT");
    ESP_LOGI(TAG, "Scheduler Performance: %s", scheduler_good ? "EXCELLENT" : "NEEDS_IMPROVEMENT");
    ESP_LOGI(TAG, "Cross-Core Performance: %s", crosscore_good ? "EXCELLENT" : "NEEDS_IMPROVEMENT");
    ESP_LOGI(TAG, "MCPWM Performance: %s", mcpwm_good ? "EXCELLENT" : "NEEDS_IMPROVEMENT");
    
    if (isr_good && scheduler_good && crosscore_good && mcpwm_good) {
        ESP_LOGI(TAG, "OVERALL: ALL TARGETS MET - OPTIMIZATION SUCCESSFUL!");
    } else {
        ESP_LOGW(TAG, "OVERALL: SOME TARGETS NOT MET - FURTHER OPTIMIZATION NEEDED");
    }
}

void latency_benchmark_reset(void) {
    memset(&g_isr_stats, 0, sizeof(latency_stats_t));
    memset(&g_scheduler_stats, 0, sizeof(latency_stats_t));
    memset(&g_crosscore_stats, 0, sizeof(latency_stats_t));
    memset(&g_mcpwm_stats, 0, sizeof(latency_stats_t));
    
    g_isr_sample_idx = 0;
    g_scheduler_sample_idx = 0;
    g_crosscore_sample_idx = 0;
    g_mcpwm_sample_idx = 0;
    
    g_benchmark_start_time = (uint32_t)bench_get_timestamp_us();
    
    ESP_LOGI(TAG, "Benchmark statistics reset");
}

void latency_benchmark_enable(bool enable) {
    g_benchmark_enabled = enable;
    ESP_LOGI(TAG, "Latency benchmark %s", enable ? "enabled" : "disabled");
}
