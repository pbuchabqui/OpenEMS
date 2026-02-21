#ifndef UNITY_H
#define UNITY_H

// Unity Test Framework - Minimal Implementation
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef void (*UnityTestFunction)(void);

// Test macros
#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            printf("FAIL: Expected %d, got %d\n", (expected), (actual)); \
            return; \
        } \
    } while(0)

#define TEST_ASSERT_NOT_NULL(pointer) \
    do { \
        if ((pointer) == NULL) { \
            printf("FAIL: Expected non-NULL pointer\n"); \
            return; \
        } \
    } while(0)

#define TEST_ASSERT_NULL(pointer) \
    do { \
        if ((pointer) != NULL) { \
            printf("FAIL: Expected NULL pointer\n"); \
            return; \
        } \
    } while(0)

#define TEST_ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            printf("FAIL: Expected TRUE condition\n"); \
            return; \
        } \
    } while(0)

#define TEST_ASSERT_FALSE(condition) \
    do { \
        if (condition) { \
            printf("FAIL: Expected FALSE condition\n"); \
            return; \
        } \
    } while(0)

// Test setup/teardown
#define setUp() do {} while(0)
#define tearDown() do {} while(0)

// Test runner
#define RUN_TEST(test_func) \
    do { \
        printf("RUNNING: %s\n", #test_func); \
        test_func(); \
        printf("PASS: %s\n\n", #test_func); \
    } while(0)

#endif // UNITY_H
