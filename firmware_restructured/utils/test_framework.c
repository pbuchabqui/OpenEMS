/**
 * @file test_framework.c
 * @brief Testing Framework Implementation
 * 
 * This module provides a lightweight testing framework for unit tests,
 * integration tests, and performance validation.
 */

#include "test_framework.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

/*============================================================================
 * Constants
 *============================================================================*/

static const char *TAG = "test";

/*============================================================================
 * Module State
 *============================================================================*/

typedef struct {
    test_case_t     tests[TEST_MAX_CASES];
    uint32_t        count;
    
    test_config_t   config;
    
    // Current test state
    char            current_message[128];
    const char     *current_file;
    int             current_line;
    bool            skipped;
    
    // Performance measurement
    uint32_t        perf_start_us;
    uint32_t        perf_start_cycles;
    
    // Memory tracking
    uint32_t        initial_heap;
} test_framework_t;

static test_framework_t g_test = {
    .count = 0,
    .config = {
        .stop_on_fail = false,
        .verbose = true,
        .measure_memory = true,
        .default_timeout_ms = TEST_DEFAULT_TIMEOUT,
    },
};

/*============================================================================
 * Test Registration
 *============================================================================*/

esp_err_t test_register(const test_case_t *test)
{
    if (test == NULL || test->run == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_test.count >= TEST_MAX_CASES) {
        ESP_LOGE(TAG, "Test registry full");
        return ESP_ERR_NO_MEM;
    }
    
    test_case_t *slot = &g_test.tests[g_test.count];
    
    strncpy(slot->name, test->name, TEST_NAME_LEN - 1);
    slot->name[TEST_NAME_LEN - 1] = '\0';
    
    strncpy(slot->module, test->module, TEST_MODULE_LEN - 1);
    slot->module[TEST_MODULE_LEN - 1] = '\0';
    
    slot->setup = test->setup;
    slot->run = test->run;
    slot->teardown = test->teardown;
    slot->timeout_ms = test->timeout_ms ? test->timeout_ms : g_test.config.default_timeout_ms;
    slot->flags = test->flags;
    
    g_test.count++;
    
    return ESP_OK;
}

int test_register_all(const test_case_t *tests)
{
    int count = 0;
    
    if (tests == NULL) {
        return 0;
    }
    
    while (tests->run != NULL) {
        if (test_register(tests) == ESP_OK) {
            count++;
        }
        tests++;
    }
    
    return count;
}

void test_clear_all(void)
{
    g_test.count = 0;
}

uint32_t test_count(void)
{
    return g_test.count;
}

/*============================================================================
 * Test Execution
 *============================================================================*/

static void run_single_test(test_case_t *test, test_result_t *result)
{
    uint32_t start_us = (uint32_t)esp_timer_get_time();
    uint32_t heap_before = 0;
    
    // Initialize result
    memset(result, 0, sizeof(test_result_t));
    strncpy(result->name, test->name, TEST_NAME_LEN - 1);
    result->status = TEST_STATUS_PASS;
    
    // Reset state
    g_test.current_message[0] = '\0';
    g_test.current_file = NULL;
    g_test.current_line = 0;
    g_test.skipped = false;
    
    // Track memory
    if (g_test.config.measure_memory) {
        heap_before = esp_get_free_heap_size();
    }
    
    // Run setup
    if (test->setup != NULL) {
        if (!test->setup()) {
            result->status = TEST_STATUS_ERROR;
            strncpy(result->message, "Setup failed", sizeof(result->message) - 1);
            goto cleanup;
        }
    }
    
    // Run test
    if (!test->run()) {
        if (g_test.skipped) {
            result->status = TEST_STATUS_SKIP;
        } else {
            result->status = TEST_STATUS_FAIL;
        }
    }
    
    // Run teardown
    if (test->teardown != NULL) {
        test->teardown();
    }
    
cleanup:
    // Calculate duration
    uint32_t end_us = (uint32_t)esp_timer_get_time();
    result->duration_us = end_us - start_us;
    
    // Copy message
    if (g_test.current_message[0] != '\0') {
        strncpy(result->message, g_test.current_message, sizeof(result->message) - 1);
    }
    
    // Copy failure location
    result->file = g_test.current_file;
    result->line = g_test.current_line;
    
    // Check memory leak
    if (g_test.config.measure_memory && result->status == TEST_STATUS_PASS) {
        uint32_t heap_after = esp_get_free_heap_size();
        if (heap_after < heap_before - 64) {  // Allow 64 bytes tolerance
            ESP_LOGW(TAG, "Possible memory leak in %s: %lu bytes", 
                     test->name, heap_before - heap_after);
        }
    }
    
    // Log result
    if (g_test.config.verbose) {
        const char *status_str;
        switch (result->status) {
            case TEST_STATUS_PASS:  status_str = "PASS"; break;
            case TEST_STATUS_FAIL:  status_str = "FAIL"; break;
            case TEST_STATUS_SKIP:  status_str = "SKIP"; break;
            case TEST_STATUS_TIMEOUT: status_str = "TIMEOUT"; break;
            default: status_str = "ERROR"; break;
        }
        
        ESP_LOGI(TAG, "[%s] %s (%lu us)", status_str, test->name, result->duration_us);
        
        if (result->status == TEST_STATUS_FAIL && result->file != NULL) {
            ESP_LOGI(TAG, "  at %s:%d", result->file, result->line);
        }
    }
}

uint32_t test_run_all(test_summary_t *summary)
{
    if (summary != NULL) {
        memset(summary, 0, sizeof(test_summary_t));
    }
    
    g_test.initial_heap = esp_get_free_heap_size();
    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    
    ESP_LOGI(TAG, "Running %lu tests...", g_test.count);
    ESP_LOGI(TAG, "========================================");
    
    for (uint32_t i = 0; i < g_test.count; i++) {
        test_result_t result;
        run_single_test(&g_test.tests[i], &result);
        
        if (summary != NULL) {
            summary->total++;
            switch (result.status) {
                case TEST_STATUS_PASS:
                    summary->passed++;
                    break;
                case TEST_STATUS_FAIL:
                    summary->failed++;
                    break;
                case TEST_STATUS_SKIP:
                    summary->skipped++;
                    break;
                case TEST_STATUS_TIMEOUT:
                    summary->timeout++;
                    break;
                default:
                    summary->failed++;
                    break;
            }
        }
        
        // Stop on fail if configured
        if (g_test.config.stop_on_fail && result.status == TEST_STATUS_FAIL) {
            ESP_LOGE(TAG, "Stopping on first failure");
            break;
        }
    }
    
    uint32_t end_ms = (uint32_t)(esp_timer_get_time() / 1000);
    
    if (summary != NULL) {
        summary->duration_ms = end_ms - start_ms;
        summary->memory_used = g_test.initial_heap - esp_get_free_heap_size();
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Tests complete: %lu passed, %lu failed, %lu skipped",
             summary ? summary->passed : 0,
             summary ? summary->failed : 0,
             summary ? summary->skipped : 0);
    
    return summary ? summary->failed : 0;
}

uint32_t test_run_module(const char *module, test_summary_t *summary)
{
    if (module == NULL) {
        return test_run_all(summary);
    }
    
    if (summary != NULL) {
        memset(summary, 0, sizeof(test_summary_t));
    }
    
    ESP_LOGI(TAG, "Running tests for module: %s", module);
    
    for (uint32_t i = 0; i < g_test.count; i++) {
        if (strcasecmp(g_test.tests[i].module, module) != 0) {
            continue;
        }
        
        test_result_t result;
        run_single_test(&g_test.tests[i], &result);
        
        if (summary != NULL) {
            summary->total++;
            switch (result.status) {
                case TEST_STATUS_PASS:
                    summary->passed++;
                    break;
                case TEST_STATUS_FAIL:
                    summary->failed++;
                    break;
                case TEST_STATUS_SKIP:
                    summary->skipped++;
                    break;
                default:
                    summary->failed++;
                    break;
            }
        }
        
        if (g_test.config.stop_on_fail && result.status == TEST_STATUS_FAIL) {
            break;
        }
    }
    
    return summary ? summary->failed : 0;
}

bool test_run_single(const char *name, test_result_t *result)
{
    if (name == NULL) {
        return false;
    }
    
    for (uint32_t i = 0; i < g_test.count; i++) {
        if (strcasecmp(g_test.tests[i].name, name) == 0) {
            test_result_t local_result;
            run_single_test(&g_test.tests[i], &local_result);
            
            if (result != NULL) {
                *result = local_result;
            }
            
            return local_result.status == TEST_STATUS_PASS;
        }
    }
    
    ESP_LOGE(TAG, "Test not found: %s", name);
    return false;
}

/*============================================================================
 * Test Configuration
 *============================================================================*/

void test_set_config(const test_config_t *config)
{
    if (config != NULL) {
        g_test.config = *config;
    }
}

void test_get_config(test_config_t *config)
{
    if (config != NULL) {
        *config = g_test.config;
    }
}

/*============================================================================
 * Test Reporting
 *============================================================================*/

void test_print_results(const test_summary_t *summary)
{
    if (summary == NULL) {
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Test Results Summary");
    ESP_LOGI(TAG, "====================");
    ESP_LOGI(TAG, "Total:      %lu", summary->total);
    ESP_LOGI(TAG, "Passed:     %lu", summary->passed);
    ESP_LOGI(TAG, "Failed:     %lu", summary->failed);
    ESP_LOGI(TAG, "Skipped:    %lu", summary->skipped);
    ESP_LOGI(TAG, "Timeout:    %lu", summary->timeout);
    ESP_LOGI(TAG, "Duration:   %lu ms", summary->duration_ms);
    ESP_LOGI(TAG, "Memory:     %lu bytes", summary->memory_used);
    
    if (summary->failed == 0 && summary->total > 0) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "*** ALL TESTS PASSED ***");
    } else if (summary->failed > 0) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "*** %lu TEST(S) FAILED ***", summary->failed);
    }
}

int test_get_results_json(char *buffer, size_t len)
{
    if (buffer == NULL || len == 0) {
        return 0;
    }
    
    return snprintf(buffer, len,
        "{\"total\":%lu,\"passed\":%lu,\"failed\":%lu,\"skipped\":%lu}",
        g_test.count,  // This should be from summary
        0UL,  // passed
        0UL,  // failed
        0UL   // skipped
    );
}

/*============================================================================
 * Test Assertion Helpers
 *============================================================================*/

void test_fail_assertion(const char *cond, const char *file, int line)
{
    snprintf(g_test.current_message, sizeof(g_test.current_message),
             "Assertion failed: %s", cond);
    g_test.current_file = file;
    g_test.current_line = line;
    
    if (g_test.config.verbose) {
        ESP_LOGE(TAG, "FAIL: %s", g_test.current_message);
    }
}

void test_fail_eq(const char *expected, const char *actual,
                  long exp_val, long act_val, const char *file, int line)
{
    snprintf(g_test.current_message, sizeof(g_test.current_message),
             "%s != %s (expected %ld, got %ld)", expected, actual, exp_val, act_val);
    g_test.current_file = file;
    g_test.current_line = line;
    
    if (g_test.config.verbose) {
        ESP_LOGE(TAG, "FAIL: %s", g_test.current_message);
    }
}

void test_fail_near(const char *expected, const char *actual,
                    long exp_val, long act_val, long tolerance,
                    const char *file, int line)
{
    snprintf(g_test.current_message, sizeof(g_test.current_message),
             "%s != %s (expected %ld, got %ld, tolerance %ld)", 
             expected, actual, exp_val, act_val, tolerance);
    g_test.current_file = file;
    g_test.current_line = line;
    
    if (g_test.config.verbose) {
        ESP_LOGE(TAG, "FAIL: %s", g_test.current_message);
    }
}

void test_skip_msg(const char *message)
{
    snprintf(g_test.current_message, sizeof(g_test.current_message),
             "Skipped: %s", message);
    g_test.skipped = true;
}

/*============================================================================
 * Performance Testing
 *============================================================================*/

void test_perf_start(void)
{
    g_test.perf_start_us = (uint32_t)esp_timer_get_time();
    // CPU cycles would need CCOUNT register
    g_test.perf_start_cycles = 0;
}

uint32_t test_perf_end(void)
{
    uint32_t end_us = (uint32_t)esp_timer_get_time();
    return end_us - g_test.perf_start_us;
}

void test_perf_get(uint32_t *cycles, uint32_t *us)
{
    if (cycles != NULL) {
        *cycles = g_test.perf_start_cycles;
    }
    if (us != NULL) {
        *us = test_perf_end();
    }
}

/*============================================================================
 * Memory Testing
 *============================================================================*/

uint32_t test_get_free_heap(void)
{
    return esp_get_free_heap_size();
}

uint32_t test_get_min_free_heap(void)
{
    return esp_get_minimum_free_heap_size();
}

bool test_check_memory_leak(uint32_t baseline)
{
    uint32_t current = esp_get_free_heap_size();
    return (current < baseline - 64);  // 64 bytes tolerance
}
