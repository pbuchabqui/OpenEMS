#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/ign_calc.h"

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_I32(exp, act) do { \
    ++g_tests_run; \
    const int32_t _e = static_cast<int32_t>(exp); \
    const int32_t _a = static_cast<int32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %ld got %ld\n", __FILE__, __LINE__, (long)_e, (long)_a); \
    } \
} while (0)

void test_dwell_start_after_spark() {
    const int32_t spark_x10 = 100;
    const int32_t dwell_start = ems::engine::calc_dwell_start_deg_x10(100, 30u, 3000u);
    TEST_ASSERT_TRUE(dwell_start > spark_x10);
}

void test_clamp_high_advance() {
    // clamp_advance_deg limita a 40° BTDC (protecção contra tabelas excessivas).
    TEST_ASSERT_EQ_I32(40, ems::engine::clamp_advance_deg(60));
}

void test_clamp_low_advance() {
    TEST_ASSERT_EQ_I32(-10, ems::engine::clamp_advance_deg(-15));
}

}  // namespace

int main() {
    test_dwell_start_after_spark();
    test_clamp_high_advance();
    test_clamp_low_advance();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
