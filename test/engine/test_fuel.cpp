#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/fuel_calc.h"

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

void test_base_pw_nominal() {
    const uint32_t pw = ems::engine::calc_base_pw_us(8000u, 128u, 100u, 100u);
    TEST_ASSERT_TRUE(pw >= 4010u && pw <= 4016u);
}

void test_base_pw_limits() {
    TEST_ASSERT_EQ_U32(0u, ems::engine::calc_base_pw_us(8000u, 0u, 100u, 100u));
    TEST_ASSERT_EQ_U32(8000u, ems::engine::calc_base_pw_us(8000u, 255u, 100u, 100u));
}

void test_ae_positive_on_tps_step() {
    ems::engine::fuel_ae_set_threshold(5u);
    ems::engine::fuel_ae_set_taper(8u);
    const int32_t ae = ems::engine::calc_ae_pw_us(500u, 0u, 10u, 900);
    TEST_ASSERT_TRUE(ae > 0);
}

void test_stft_grows_positive_with_lean_feedback() {
    ems::engine::fuel_reset_adaptives();

    const int16_t stft1 = ems::engine::fuel_update_stft(
        3000u,
        100u,
        1000,
        950,
        800,
        true,
        false,
        false);

    const int16_t stft2 = ems::engine::fuel_update_stft(
        3000u,
        100u,
        1000,
        950,
        800,
        true,
        false,
        false);

    TEST_ASSERT_TRUE(stft1 > 0);
    TEST_ASSERT_TRUE(stft2 >= stft1);
}

}  // namespace

int main() {
    test_base_pw_nominal();
    test_base_pw_limits();
    test_ae_positive_on_tps_step();
    test_stft_grows_positive_with_lean_feedback();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
