#include <cassert>
#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "drv/ckp.h"

extern volatile uint32_t ems_test_ftm3_c0v;
extern volatile uint32_t ems_test_ftm3_c1v;
extern volatile uint32_t ems_test_gpiod_pdir;

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;
uint16_t g_capture = 0u;

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
    ems::drv::ckp_test_reset();
    g_capture = 0u;
}

void feed_ckp(uint16_t period_ticks) {
    ems_test_gpiod_pdir = (1u << 0u);
    g_capture = static_cast<uint16_t>(g_capture + period_ticks);
    ems_test_ftm3_c0v = g_capture;
    ems::drv::ckp_ftm3_ch0_isr();
}

void sync_with_two_gaps() {
    for (int i = 0; i < 58; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);

    for (int i = 0; i < 58; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);
}

void sync_with_one_gap() {
    for (int i = 0; i < 58; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);
}

void test_sync_after_two_gaps() {
    test_reset();
    sync_with_two_gaps();
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::FULL_SYNC);
}

void test_false_gap_ignored_before_tooth55() {
    test_reset();
    for (int i = 0; i < 20; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);

    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::WAIT_GAP);
}

void test_rpm_formula() {
    test_reset();
    const uint32_t rpm_x10 = ems::drv::ckp_test_rpm_x10_from_period_ns(1293103u);
    TEST_ASSERT_TRUE(rpm_x10 >= 7990u && rpm_x10 <= 8010u);
}

void test_circular_subtraction() {
    test_reset();
    ems_test_gpiod_pdir = (1u << 0u);
    ems_test_ftm3_c0v = 65530u;
    ems::drv::ckp_ftm3_ch0_isr();

    ems_test_ftm3_c0v = 10u;
    ems::drv::ckp_ftm3_ch0_isr();

    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_EQ_U32((16u * 16667u) / 1000u, snap.tooth_period_ns);
}

void test_tooth_count_over_60_goes_syncing() {
    test_reset();
    sync_with_two_gaps();

    auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::FULL_SYNC);

    for (int i = 0; i < 61; ++i) {
        feed_ckp(1000u);
    }

    snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::LOSS_OF_SYNC);
}

// P7: RPM deve ser zero antes de qualquer dente
void test_rpm_zero_before_sync() {
    test_reset();
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_EQ_U32(0u, snap.rpm_x10);
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::WAIT_GAP);
}

// P7: phase_A deve alternar a cada disparo de CH1 (cam sensor)
void test_phase_a_toggles_on_ch1() {
    test_reset();
    sync_with_two_gaps();

    const auto snap0 = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap0.state == ems::drv::SyncState::FULL_SYNC);
    const bool phase_initial = snap0.phase_A;

    // Simular rising edge no cam sensor (CH1, bit 1 de GPIOD)
    ems_test_gpiod_pdir = (1u << 1u);
    ems::drv::ckp_ftm3_ch1_isr();

    const auto snap1 = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap1.phase_A != phase_initial);

    // Segundo disparo: deve voltar ao valor original
    ems::drv::ckp_ftm3_ch1_isr();
    const auto snap2 = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap2.phase_A == phase_initial);
}

// P7: tooth_period_ns deve ser não-zero após ao menos um par de dentes
void test_tooth_period_nonzero_after_two_teeth() {
    test_reset();
    feed_ckp(1000u);  // primeiro dente — sem período ainda
    feed_ckp(1000u);  // segundo dente — período calculado

    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.tooth_period_ns > 0u);
}

void test_seeded_fast_reacquire_promotes_on_first_gap() {
    test_reset();
    ems::drv::ckp_seed_arm(true);
    sync_with_one_gap();
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::FULL_SYNC);
    TEST_ASSERT_TRUE(snap.phase_A == true);
}

}  // namespace

int main() {
    test_sync_after_two_gaps();
    test_false_gap_ignored_before_tooth55();
    test_rpm_formula();
    test_circular_subtraction();
    test_tooth_count_over_60_goes_syncing();
    test_rpm_zero_before_sync();
    test_phase_a_toggles_on_ch1();
    test_tooth_period_nonzero_after_two_teeth();
    test_seeded_fast_reacquire_promotes_on_first_gap();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
