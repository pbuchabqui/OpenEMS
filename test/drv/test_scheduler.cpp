#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "drv/ckp.h"
#include "drv/scheduler.h"

extern volatile uint32_t ems_test_fgpio_c_psor;
extern volatile uint32_t ems_test_fgpio_c_pcor;
extern volatile uint32_t ems_test_fgpio_d_psor;
extern volatile uint32_t ems_test_fgpio_d_pcor;

namespace ems::hal {
static uint16_t g_ftm0_count = 0u;
static uint8_t g_last_compare_ch = 0u;
static uint16_t g_last_compare_ticks = 0u;
static uint8_t g_clear_chf_last = 0xFFu;

uint16_t ftm0_count() noexcept {
    return g_ftm0_count;
}

void ftm0_set_compare(uint8_t ch, uint16_t ticks) noexcept {
    g_last_compare_ch = ch;
    g_last_compare_ticks = ticks;
}

void ftm0_clear_chf(uint8_t ch) noexcept {
    g_clear_chf_last = ch;
}

void ftm0_arm_ignition(uint8_t ch, uint16_t ticks) noexcept {
    // Mock: reutiliza set_compare para capturar o tick alvo em testes
    g_last_compare_ch = ch;
    g_last_compare_ticks = ticks;
}
}  // namespace ems::hal

namespace ems::drv {
CkpSnapshot ckp_snapshot() noexcept {
    return CkpSnapshot{0u, 0u, 0u, 0u, SyncState::WAIT_GAP, false};
}
}  // namespace ems::drv

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

void test_reset() {
    ems::drv::sched_test_reset();
    ems::hal::g_ftm0_count = 0u;
    ems::hal::g_last_compare_ch = 0u;
    ems::hal::g_last_compare_ticks = 0u;
    ems::hal::g_clear_chf_last = 0xFFu;
}

void test_insert_out_of_order_sorts_queue() {
    test_reset();
    ems::hal::g_ftm0_count = 100u;

    const uint16_t ticks_in[8] = {600u, 500u, 900u, 300u, 450u, 700u, 200u, 800u};
    const ems::drv::Channel channels[8] = {
        ems::drv::Channel::INJ3,
        ems::drv::Channel::INJ4,
        ems::drv::Channel::INJ1,
        ems::drv::Channel::INJ2,
        ems::drv::Channel::IGN4,
        ems::drv::Channel::IGN3,
        ems::drv::Channel::IGN2,
        ems::drv::Channel::IGN1,
    };

    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_TRUE(ems::drv::sched_event(channels[i], ticks_in[i], ems::drv::Action::SET));
    }

    TEST_ASSERT_EQ_U32(8u, ems::drv::sched_test_size());

    uint16_t prev_ticks = 0u;
    for (uint8_t i = 0u; i < ems::drv::sched_test_size(); ++i) {
        uint16_t ticks = 0u;
        ems::drv::Channel ch = ems::drv::Channel::INJ3;
        ems::drv::Action act = ems::drv::Action::SET;
        bool valid = false;
        TEST_ASSERT_TRUE(ems::drv::sched_test_event(i, ticks, ch, act, valid));
        TEST_ASSERT_TRUE(valid);
        if (i > 0u) {
            TEST_ASSERT_TRUE(prev_ticks <= ticks);
        }
        prev_ticks = ticks;
    }

    TEST_ASSERT_EQ_U32(6u, ems::hal::g_last_compare_ch);
    TEST_ASSERT_EQ_U32(200u, ems::hal::g_last_compare_ticks);
}

void test_cancel_marks_channel_invalid() {
    test_reset();
    ems::hal::g_ftm0_count = 100u;

    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::IGN1, 300u, ems::drv::Action::SET));
    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::IGN1, 400u, ems::drv::Action::CLEAR));
    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::INJ1, 500u, ems::drv::Action::SET));

    ems::drv::sched_cancel(ems::drv::Channel::IGN1);

    TEST_ASSERT_EQ_U32(1u, ems::drv::sched_test_size());
    uint16_t ticks = 0u;
    ems::drv::Channel ch = ems::drv::Channel::INJ3;
    ems::drv::Action act = ems::drv::Action::SET;
    bool valid = false;
    TEST_ASSERT_TRUE(ems::drv::sched_test_event(0u, ticks, ch, act, valid));
    TEST_ASSERT_TRUE(valid);
    TEST_ASSERT_TRUE(ch == ems::drv::Channel::INJ1);
}

void test_reject_old_event_delta_64800() {
    test_reset();
    ems::hal::g_ftm0_count = 200u;
    TEST_ASSERT_TRUE(!ems::drv::sched_event(ems::drv::Channel::INJ3, 65000u, ems::drv::Action::SET));
}

void test_accept_future_event_delta_100() {
    test_reset();
    ems::hal::g_ftm0_count = 200u;
    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::INJ3, 300u, ems::drv::Action::SET));
}

void test_sched_isr_drives_correct_gpio() {
    test_reset();
    ems::hal::g_ftm0_count = 100u;

    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::IGN1, 300u, ems::drv::Action::SET));
    ems::hal::g_ftm0_count = 300u;
    ems::drv::sched_isr();

    TEST_ASSERT_EQ_U32((1u << 7u), ems_test_fgpio_d_psor);
    TEST_ASSERT_EQ_U32(7u, ems::hal::g_clear_chf_last);

    TEST_ASSERT_TRUE(ems::drv::sched_event(ems::drv::Channel::INJ2, 350u, ems::drv::Action::CLEAR));
    ems::hal::g_ftm0_count = 350u;
    ems::drv::sched_isr();

    TEST_ASSERT_EQ_U32((1u << 4u), ems_test_fgpio_c_pcor);
    TEST_ASSERT_EQ_U32(3u, ems::hal::g_clear_chf_last);
}

}  // namespace

int main() {
    test_insert_out_of_order_sorts_queue();
    test_cancel_marks_channel_invalid();
    test_reject_old_event_delta_64800();
    test_accept_future_event_delta_100();
    test_sched_isr_drives_correct_gpio();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
