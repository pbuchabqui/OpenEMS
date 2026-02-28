#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "hal/adc.h"

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

void reset_all() {
    ems::hal::adc_init();
    ems::drv::sensors_init();
    ems::drv::sensors_test_reset();
}

ems::drv::CkpSnapshot mk_snap() {
    return ems::drv::CkpSnapshot{1000000u, 0u, 0u, 10000u, ems::drv::SyncState::SYNCED, false};
}

void feed_fast_constant(uint16_t raw, int cycles) {
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAP_SE10, raw);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAF_V_SE11, raw);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::TPS_SE12, raw);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::O2_SE4B, raw);

    auto snap = mk_snap();
    for (int i = 0; i < cycles; ++i) {
        ems::drv::sensors_on_tooth(snap);
        snap.tooth_index = static_cast<uint16_t>((snap.tooth_index + 1u) % 58u);
    }
}

void test_iir_constant_converges_to_4095() {
    reset_all();
    feed_fast_constant(4095u, 20 * 5);
    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE(s.map_kpa_x10 > 2450u);
}

void test_iir_fall_to_zero_in_10_cycles() {
    reset_all();
    ems::drv::sensors_set_range(ems::drv::SensorId::MAP, ems::drv::SensorRange{0u, 4095u});
    feed_fast_constant(4095u, 200 * 5);
    feed_fast_constant(0u, 10 * 5);
    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE(s.map_kpa_x10 < 100u);
}

void test_clt_fault_after_3_out_of_range() {
    reset_all();
    ems::drv::sensors_set_range(ems::drv::SensorId::CLT, ems::drv::SensorRange{100u, 3800u});
    ems::hal::adc_test_set_raw_adc1(ems::hal::Adc1Channel::CLT_SE14, 4095u);

    ems::drv::sensors_tick_100ms();
    ems::drv::sensors_tick_100ms();
    ems::drv::sensors_tick_100ms();

    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE((s.fault_bits & (1u << 3u)) != 0u);
}

void test_map_linearization_full_scale() {
    reset_all();
    feed_fast_constant(4095u, 40 * 5);
    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_EQ_U32(2500u, s.map_kpa_x10);
}

void test_tps_calibration_midpoint() {
    reset_all();
    ems::drv::sensors_set_tps_cal(200u, 3895u);

    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAP_SE10, 1000u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAF_V_SE11, 1000u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::TPS_SE12, 2047u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::O2_SE4B, 1000u);

    auto snap = mk_snap();
    for (int i = 0; i < 80; ++i) {
        ems::drv::sensors_on_tooth(snap);
    }

    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE(s.tps_pct_x10 >= 495u && s.tps_pct_x10 <= 505u);
}

}  // namespace

int main() {
    test_iir_constant_converges_to_4095();
    test_iir_fall_to_zero_in_10_cycles();
    test_clt_fault_after_3_out_of_range();
    test_map_linearization_full_scale();
    test_tps_calibration_midpoint();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
