#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/knock.h"

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

void inject_knock_events(uint8_t cyl, uint8_t n) {
    ems::engine::knock_window_open(cyl);
    for (uint8_t i = 0u; i < n; ++i) {
        ems::engine::knock_cmp0_isr();
    }
    ems::engine::knock_window_close(cyl);
}

void test_retard_step_and_vosel_drop_on_knock() {
    ems::engine::knock_init();
    const uint8_t vosel0 = ems::engine::knock_get_vosel();

    inject_knock_events(0u, 4u);  // threshold default = 3
    ems::engine::knock_cycle_complete(0u);

    TEST_ASSERT_EQ_U32(20u, ems::engine::knock_get_retard_x10(0u));
    TEST_ASSERT_EQ_U32(static_cast<uint32_t>(vosel0 - 2u), ems::engine::knock_get_vosel());
}

void test_retard_clamps_at_10deg() {
    ems::engine::knock_init();

    for (uint8_t i = 0u; i < 8u; ++i) {
        inject_knock_events(1u, 6u);
        ems::engine::knock_cycle_complete(1u);
    }

    TEST_ASSERT_EQ_U32(100u, ems::engine::knock_get_retard_x10(1u));
}

void test_recovery_after_10_clean_cycles() {
    ems::engine::knock_init();
    inject_knock_events(2u, 5u);
    ems::engine::knock_cycle_complete(2u);
    TEST_ASSERT_EQ_U32(20u, ems::engine::knock_get_retard_x10(2u));

    for (uint8_t i = 0u; i < 9u; ++i) {
        ems::engine::knock_cycle_complete(2u);
    }
    TEST_ASSERT_EQ_U32(20u, ems::engine::knock_get_retard_x10(2u));

    ems::engine::knock_cycle_complete(2u);
    TEST_ASSERT_EQ_U32(19u, ems::engine::knock_get_retard_x10(2u));
}

void test_vosel_rises_after_100_clean_cycles() {
    ems::engine::knock_init();
    const uint8_t vosel0 = ems::engine::knock_get_vosel();

    for (uint8_t i = 0u; i < 100u; ++i) {
        ems::engine::knock_cycle_complete(3u);
    }

    TEST_ASSERT_EQ_U32(static_cast<uint32_t>(vosel0 + 1u), ems::engine::knock_get_vosel());
}

void test_cmp_events_count_only_inside_window() {
    ems::engine::knock_init();

    ems::engine::knock_cmp0_isr();
    TEST_ASSERT_EQ_U32(0u, ems::engine::knock_test_get_knock_count(0u));

    ems::engine::knock_window_open(0u);
    TEST_ASSERT_TRUE(ems::engine::knock_test_window_active());
    ems::engine::knock_cmp0_isr();
    ems::engine::knock_cmp0_isr();
    ems::engine::knock_window_close(0u);
    TEST_ASSERT_EQ_U32(2u, ems::engine::knock_test_get_knock_count(0u));
}

}  // namespace

int main() {
    test_retard_step_and_vosel_drop_on_knock();
    test_retard_clamps_at_10deg();
    test_recovery_after_10_clean_cycles();
    test_vosel_rises_after_100_clean_cycles();
    test_cmp_events_count_only_inside_window();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
