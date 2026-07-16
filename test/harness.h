#pragma once
/**
 * @file test/harness.h
 * Shared host-test assertion macros and counters (OpenEMS hygiene PR-06).
 */
#include <cstdio>
#include <cmath>
#include <cstdint>

extern int g_pass;
extern int g_fail;

void section(const char* name);

#define CHECK(cond, name) do { \
    if (cond) { \
        ++g_pass; \
        printf("  PASS  %s\n", name); \
    } else { \
        ++g_fail; \
        printf("  FAIL  %s  (line %d)\n", name, __LINE__); \
    } \
} while (0)

#define CHECK_TRUE(cond,   name) CHECK(!!(cond), name)
#define CHECK_FALSE(cond,  name) CHECK(!(cond),  name)
#define CHECK_EQ(a, b,     name) CHECK((a) == (b), name)
#define CHECK_NEAR(a, b, eps, name) \
    CHECK(fabsf(static_cast<float>(a) - static_cast<float>(b)) <= (eps), name)
