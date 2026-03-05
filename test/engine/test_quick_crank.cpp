#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/quick_crank.h"

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

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

#define TEST_ASSERT_EQ_I32(exp, act) do { \
    ++g_tests_run; \
    const int32_t _e = static_cast<int32_t>(exp); \
    const int32_t _a = static_cast<int32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %d got %d\n", __FILE__, __LINE__, (int)_e, (int)_a); \
    } \
} while (0)

void test_cranking_enrichment_and_spark_override() {
    ems::engine::quick_crank_reset();
    const auto out = ems::engine::quick_crank_update(
        100u, 3000u, true, 200, 20);
    TEST_ASSERT_TRUE(out.cranking);
    TEST_ASSERT_TRUE(!out.afterstart_active);
    TEST_ASSERT_TRUE(out.fuel_mult_x256 > 256u);
    TEST_ASSERT_EQ_I32(8, out.spark_deg);
    TEST_ASSERT_TRUE(out.min_pw_us >= 2000u);
}

void test_afterstart_decay_after_crank_exit() {
    ems::engine::quick_crank_reset();
    (void)ems::engine::quick_crank_update(0u, 2500u, true, 0, 15);     // cranking
    const auto out0 = ems::engine::quick_crank_update(100u, 7500u, true, 0, 15);  // exit
    const auto out1 = ems::engine::quick_crank_update(400u, 8000u, true, 0, 15);
    const auto out2 = ems::engine::quick_crank_update(2600u, 8000u, true, 0, 15);

    TEST_ASSERT_TRUE(!out0.cranking);
    TEST_ASSERT_TRUE(out0.afterstart_active);
    TEST_ASSERT_TRUE(out0.fuel_mult_x256 > 256u);
    TEST_ASSERT_TRUE(out1.fuel_mult_x256 <= out0.fuel_mult_x256);
    TEST_ASSERT_EQ_U32(256u, out2.fuel_mult_x256);
    TEST_ASSERT_TRUE(!out2.afterstart_active);
}

void test_pw_application_with_floor_and_clamp() {
    const uint32_t boosted = ems::engine::quick_crank_apply_pw_us(1000u, 512u, 2500u);
    const uint32_t saturated = ems::engine::quick_crank_apply_pw_us(90000u, 512u, 0u);
    TEST_ASSERT_EQ_U32(2500u, boosted);   // floor to minimum crank pulse
    TEST_ASSERT_EQ_U32(100000u, saturated);  // hard cap
}

}  // namespace

int main() {
    test_cranking_enrichment_and_spark_override();
    test_afterstart_decay_after_crank_exit();
    test_pw_application_with_floor_and_clamp();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
