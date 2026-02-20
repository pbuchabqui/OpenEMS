/**
 * @file test_framework.h
 * @brief Testing Framework for ESP32-S3 EFI
 * 
 * This module provides a lightweight testing framework for unit tests,
 * integration tests, and performance validation.
 * 
 * Features:
 * - Test case registration and execution
 * - Test result reporting
 * - Performance measurement
 * - Memory usage tracking
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants and Configuration
 *============================================================================*/

/** @brief Maximum test name length */
#define TEST_NAME_LEN           64

/** @brief Maximum module name length */
#define TEST_MODULE_LEN         32

/** @brief Maximum registered tests */
#define TEST_MAX_CASES          64

/** @brief Default test timeout (ms) */
#define TEST_DEFAULT_TIMEOUT    5000

/*============================================================================
 * Test Macros
 *============================================================================*/

/** @brief Assert condition is true */
#define TEST_ASSERT(cond) \
    do { \
        if (!(cond)) { \
            test_fail_assertion(#cond, __FILE__, __LINE__); \
            return false; \
        } \
    } while (0)

/** @brief Assert two values are equal */
#define TEST_ASSERT_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            test_fail_eq(#expected, #actual, (long)(expected), (long)(actual), __FILE__, __LINE__); \
            return false; \
        } \
    } while (0)

/** @brief Assert two values are approximately equal */
#define TEST_ASSERT_NEAR(expected, actual, tolerance) \
    do { \
        long _exp = (expected); \
        long _act = (actual); \
        long _tol = (tolerance); \
        if (_exp - _act > _tol || _act - _exp > _tol) { \
            test_fail_near(#expected, #actual, _exp, _act, _tol, __FILE__, __LINE__); \
            return false; \
        } \
    } while (0)

/** @brief Assert pointer is not NULL */
#define TEST_ASSERT_NOT_NULL(ptr) \
    TEST_ASSERT((ptr) != NULL)

/** @brief Assert pointer is NULL */
#define TEST_ASSERT_NULL(ptr) \
    TEST_ASSERT((ptr) == NULL)

/** @brief Skip test with message */
#define TEST_SKIP(msg) \
    do { \
        test_skip_msg(msg); \
        return false; \
    } while (0)

/*============================================================================
 * Types and Structures
 *============================================================================*/

/**
 * @brief Test result status
 */
typedef enum {
    TEST_STATUS_PASS,
    TEST_STATUS_FAIL,
    TEST_STATUS_SKIP,
    TEST_STATUS_TIMEOUT,
    TEST_STATUS_ERROR,
} test_status_t;

/**
 * @brief Test case structure
 */
typedef struct {
    char        name[TEST_NAME_LEN];      /**< Test name */
    char        module[TEST_MODULE_LEN];  /**< Module name */
    bool        (*setup)(void);           /**< Setup function (can be NULL) */
    bool        (*run)(void);             /**< Test function */
    void        (*teardown)(void);        /**< Teardown function (can be NULL) */
    uint32_t    timeout_ms;               /**< Test timeout */
    uint32_t    flags;                    /**< Test flags */
} test_case_t;

/**
 * @brief Test result structure
 */
typedef struct {
    char        name[TEST_NAME_LEN];      /**< Test name */
    test_status_t status;                 /**< Test status */
    uint32_t    duration_us;              /**< Test duration */
    char        message[128];             /**< Status message */
    const char *file;                     /**< Failure file */
    int         line;                     /**< Failure line */
} test_result_t;

/**
 * @brief Test suite results
 */
typedef struct {
    uint32_t    total;                    /**< Total tests */
    uint32_t    passed;                   /**< Passed tests */
    uint32_t    failed;                   /**< Failed tests */
    uint32_t    skipped;                  /**< Skipped tests */
    uint32_t    timeout;                  /**< Timed out tests */
    uint32_t    duration_ms;              /**< Total duration */
    uint32_t    memory_used;              /**< Peak memory used */
} test_summary_t;

/**
 * @brief Test configuration
 */
typedef struct {
    bool        stop_on_fail;             /**< Stop on first failure */
    bool        verbose;                  /**< Verbose output */
    bool        measure_memory;           /**< Track memory usage */
    uint32_t    default_timeout_ms;       /**< Default timeout */
} test_config_t;

/*============================================================================
 * Test Registration
 *============================================================================*/

/**
 * @brief Register a test case
 * 
 * @param test Test case structure
 * @return ESP_OK on success
 */
esp_err_t test_register(const test_case_t *test);

/**
 * @brief Register multiple test cases
 * 
 * @param tests Array of test cases (NULL terminated)
 * @return Number of tests registered
 */
int test_register_all(const test_case_t *tests);

/**
 * @brief Clear all registered tests
 */
void test_clear_all(void);

/**
 * @brief Get number of registered tests
 * 
 * @return Number of tests
 */
uint32_t test_count(void);

/*============================================================================
 * Test Execution
 *============================================================================*/

/**
 * @brief Run all registered tests
 * 
 * @param summary Output summary (can be NULL)
 * @return Number of failed tests
 */
uint32_t test_run_all(test_summary_t *summary);

/**
 * @brief Run tests for a specific module
 * 
 * @param module Module name
 * @param summary Output summary (can be NULL)
 * @return Number of failed tests
 */
uint32_t test_run_module(const char *module, test_summary_t *summary);

/**
 * @brief Run a single test by name
 * 
 * @param name Test name
 * @param result Output result (can be NULL)
 * @return true if test passed
 */
bool test_run_single(const char *name, test_result_t *result);

/*============================================================================
 * Test Configuration
 *============================================================================*/

/**
 * @brief Set test configuration
 * 
 * @param config Configuration structure
 */
void test_set_config(const test_config_t *config);

/**
 * @brief Get current test configuration
 * 
 * @param config Output configuration
 */
void test_get_config(test_config_t *config);

/*============================================================================
 * Test Reporting
 *============================================================================*/

/**
 * @brief Print test results to console
 * 
 * @param summary Summary to print
 */
void test_print_results(const test_summary_t *summary);

/**
 * @brief Get test results as JSON string
 * 
 * @param buffer Output buffer
 * @param len Buffer length
 * @return Number of characters written
 */
int test_get_results_json(char *buffer, size_t len);

/*============================================================================
 * Test Assertion Helpers (internal use)
 *============================================================================*/

/**
 * @brief Record assertion failure
 * 
 * @param cond Condition string
 * @param file Source file
 * @param line Source line
 */
void test_fail_assertion(const char *cond, const char *file, int line);

/**
 * @brief Record equality failure
 * 
 * @param expected Expected expression
 * @param actual Actual expression
 * @param exp_val Expected value
 * @param act_val Actual value
 * @param file Source file
 * @param line Source line
 */
void test_fail_eq(const char *expected, const char *actual,
                  long exp_val, long act_val, const char *file, int line);

/**
 * @brief Record near-equality failure
 * 
 * @param expected Expected expression
 * @param actual Actual expression
 * @param exp_val Expected value
 * @param act_val Actual value
 * @param tolerance Tolerance value
 * @param file Source file
 * @param line Source line
 */
void test_fail_near(const char *expected, const char *actual,
                    long exp_val, long act_val, long tolerance,
                    const char *file, int line);

/**
 * @brief Record test skip
 * 
 * @param message Skip message
 */
void test_skip_msg(const char *message);

/*============================================================================
 * Performance Testing
 *============================================================================*/

/**
 * @brief Start performance measurement
 */
void test_perf_start(void);

/**
 * @brief End performance measurement
 * 
 * @return Elapsed time in microseconds
 */
uint32_t test_perf_end(void);

/**
 * @brief Get current test performance data
 * 
 * @param cycles Output CPU cycles
 * @param us Output microseconds
 */
void test_perf_get(uint32_t *cycles, uint32_t *us);

/*============================================================================
 * Memory Testing
 *============================================================================*/

/**
 * @brief Get free heap memory
 * 
 * @return Free heap in bytes
 */
uint32_t test_get_free_heap(void);

/**
 * @brief Get minimum ever free heap
 * 
 * @return Minimum free heap in bytes
 */
uint32_t test_get_min_free_heap(void);

/**
 * @brief Check for memory leaks
 * 
 * @param baseline Baseline free heap
 * @return true if leak detected
 */
bool test_check_memory_leak(uint32_t baseline);

#ifdef __cplusplus
}
#endif

#endif /* TEST_FRAMEWORK_H */
