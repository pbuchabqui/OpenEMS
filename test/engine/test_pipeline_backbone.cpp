#define EMS_HOST_TEST 1

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "drv/ckp.h"
#include "engine/cycle_sched.h"
#include "engine/ecu_sched.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"

extern volatile uint32_t ems_test_ftm3_c0v;
extern volatile uint32_t ems_test_ftm3_c1v;
extern volatile uint32_t ems_test_gpiod_pdir;

FTM_Type g_mock_ftm0;
PDB_Type g_mock_pdb0;
ADC_Type g_mock_adc0;

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
    std::memset(&g_mock_ftm0, 0, sizeof(g_mock_ftm0));
    std::memset(&g_mock_pdb0, 0, sizeof(g_mock_pdb0));
    std::memset(&g_mock_adc0, 0, sizeof(g_mock_adc0));
    g_capture = 0u;
    ems::drv::ckp_test_reset();
    ems::engine::cycle_sched_init();
    ecu_sched_test_reset();
    ECU_Hardware_Init();
}

void feed_tooth(uint16_t period_ticks) {
    ems_test_gpiod_pdir = (1u << 0u);
    g_capture = static_cast<uint16_t>(g_capture + period_ticks);
    ems_test_ftm3_c0v = g_capture;
    ems::drv::ckp_ftm3_ch0_isr();
}

void feed_revolution_pattern(uint16_t tooth_ticks, uint16_t gap_ticks) {
    for (int i = 0; i < 58; ++i) {
        feed_tooth(tooth_ticks);
    }
    feed_tooth(gap_ticks);
}

bool queue_contains_action(uint8_t action) {
    const uint8_t n = ecu_sched_test_angle_table_size();
    for (uint8_t i = 0u; i < n; ++i) {
        uint8_t tooth = 0u, sf = 0u, ch = 0u, act = 0u, phase = 0u;
        if (ecu_sched_test_get_angle_event(i, &tooth, &sf, &ch, &act, &phase) != 0u
            && act == action) {
            return true;
        }
    }
    return false;
}

void test_backbone_decode_sync_calc_schedule() {
    test_reset();

    // Keep decoded RPM inside current fuel assertion envelope (<= 20000 rpm_x10)
    // while preserving a detectable gap ratio (>1.5x).
    constexpr uint16_t kToothTicks = 40000u;
    constexpr uint16_t kGapTicks = 64000u;
    constexpr uint16_t kMapKpa = 100u;
    constexpr uint16_t kReqFuelUs = 8000u;
    constexpr uint16_t kMapRefKpa = 100u;
    constexpr uint32_t kSoiLeadDeg = 62u;
    constexpr uint16_t kVbattMv = 13800u;

    // Decode + sync: two 60-2 revolutions are required to reach FULL_SYNC.
    feed_revolution_pattern(kToothTicks, kGapTicks);
    feed_revolution_pattern(kToothTicks, kGapTicks);

    const ems::drv::CkpSnapshot sync_snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(sync_snap.state == ems::drv::SyncState::FULL_SYNC);
    TEST_ASSERT_TRUE(sync_snap.rpm_x10 > 0u);

    // Strategy calculations + table lookups.
    const uint8_t ve = ems::engine::get_ve(
        static_cast<uint16_t>(sync_snap.rpm_x10), kMapKpa);
    const uint32_t base_pw_us = ems::engine::calc_base_pw_us(
        kReqFuelUs, ve, kMapKpa, kMapRefKpa);
    const int16_t base_adv = ems::engine::get_advance(
        static_cast<uint16_t>(sync_snap.rpm_x10), kMapKpa);
    const int16_t clamped_adv = ems::engine::clamp_advance_deg(base_adv);

    TEST_ASSERT_TRUE(ve > 0u);
    TEST_ASSERT_TRUE(base_pw_us > 0u);

    const uint32_t advance_deg = (clamped_adv > 0) ? static_cast<uint32_t>(clamped_adv) : 0u;
    const uint16_t dwell_ms_x10 = ems::engine::dwell_ms_x10_from_vbatt(kVbattMv);
    const uint32_t dwell_ticks =
        (static_cast<uint32_t>(dwell_ms_x10) * ECU_FTM0_TICKS_PER_MS) / 100u;
    const uint32_t inj_pw_ticks = (base_pw_us * ECU_FTM0_TICKS_PER_MS) / 1000u;

    /* ticks_per_rev removed: calculated internally from tooth_period_ns in angle domain */
    ecu_sched_commit_calibration(advance_deg, dwell_ticks, inj_pw_ticks, kSoiLeadDeg);

    TEST_ASSERT_EQ_U32(inj_pw_ticks, ecu_sched_test_get_inj_pw_ticks());
    TEST_ASSERT_EQ_U32(dwell_ticks, ecu_sched_test_get_dwell_ticks());

    // Scheduling through CKP hook path (decode/sync -> schedule_on_tooth -> ecu_sched).
    feed_revolution_pattern(kToothTicks, kGapTicks);
    feed_revolution_pattern(kToothTicks, kGapTicks);
    feed_revolution_pattern(kToothTicks, kGapTicks);
    feed_revolution_pattern(kToothTicks, kGapTicks);
    const uint8_t q_after = ecu_sched_test_angle_table_size();

    TEST_ASSERT_TRUE(q_after > 0u);
    TEST_ASSERT_TRUE(queue_contains_action(ECU_ACT_INJ_ON));
    TEST_ASSERT_TRUE(queue_contains_action(ECU_ACT_INJ_OFF));
    TEST_ASSERT_TRUE(queue_contains_action(ECU_ACT_DWELL_START));
    TEST_ASSERT_TRUE(queue_contains_action(ECU_ACT_SPARK));
}

}  // namespace

int main() {
    test_backbone_decode_sync_calc_schedule();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
