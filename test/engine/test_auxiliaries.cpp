#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/auxiliaries.h"
#include "drv/ckp.h"
#include "drv/sensors.h"

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

ems::drv::CkpSnapshot g_snap = {1000000u, 0u, 0u, 3000u, ems::drv::SyncState::FULL_SYNC, true};
ems::drv::SensorData g_sensors = {
    1000u,   // map_kpa_x10
    0u,      // maf_gps_x100
    700u,    // tps_pct_x10
    900,     // clt_degc_x10
    250,     // iat_degc_x10
    // o2_mv REMOVIDO
    3000u,   // fuel_press_kpa_x10
    2500u,   // oil_press_kpa_x10
    13500u,  // vbatt_mv
    0u,      // fault_bits
};

uint32_t g_ftm1_init_hz = 0u;
uint32_t g_ftm2_init_hz = 0u;
uint16_t g_ftm1_duty[2] = {0u, 0u};
uint16_t g_ftm2_duty[2] = {0u, 0u};

void reset_fixture() {
    g_snap = ems::drv::CkpSnapshot{1000000u, 8u, 0u, 3000u, ems::drv::SyncState::FULL_SYNC, true};
    g_sensors = ems::drv::SensorData{
        1000u, 0u, 700u, 900, 250, 3000u, 2500u, 13500u, 0u,
    };
    g_ftm1_init_hz = 0u;
    g_ftm2_init_hz = 0u;
    g_ftm1_duty[0] = 0u;
    g_ftm1_duty[1] = 0u;
    g_ftm2_duty[0] = 0u;
    g_ftm2_duty[1] = 0u;
}

}  // namespace

namespace ems::drv {
CkpSnapshot ckp_snapshot() noexcept {
    return g_snap;
}

SensorData sensors_get() noexcept {
    return g_sensors;
}
}  // namespace ems::drv

namespace ems::hal {
void ftm1_pwm_init(uint32_t freq_hz) {
    g_ftm1_init_hz = freq_hz;
}

void ftm2_pwm_init(uint32_t freq_hz) {
    g_ftm2_init_hz = freq_hz;
}

void ftm1_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    if (ch < 2u) {
        g_ftm1_duty[ch] = duty_pct_x10;
    }
}

void ftm2_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    if (ch < 2u) {
        g_ftm2_duty[ch] = duty_pct_x10;
    }
}
}  // namespace ems::hal

namespace {

void test_pwm_init_strategy() {
    reset_fixture();
    ems::engine::auxiliaries_init();

    TEST_ASSERT_TRUE(g_ftm1_init_hz == 15u);
    TEST_ASSERT_TRUE(g_ftm2_init_hz == 15u);
}

void test_wastegate_overboost_failsafe_after_500ms() {
    reset_fixture();
    ems::engine::auxiliaries_init();

    g_sensors.tps_pct_x10 = 1000u;
    g_snap.rpm_x10 = 5000u;
    g_sensors.map_kpa_x10 = 2200u;

    for (uint8_t i = 0u; i < 25u; ++i) {
        ems::engine::auxiliaries_tick_20ms();
    }

    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_wg_failsafe());
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_wg_duty() == 0u);

    g_sensors.map_kpa_x10 = 1200u;
    ems::engine::auxiliaries_tick_20ms();
    TEST_ASSERT_TRUE(!ems::engine::auxiliaries_test_get_wg_failsafe());
}

void test_vvt_failsafe_when_no_phase_confirm_for_200ms() {
    reset_fixture();
    ems::engine::auxiliaries_init();

    g_sensors.map_kpa_x10 = 1000u;
    g_snap.rpm_x10 = 3500u;
    g_snap.state = ems::drv::SyncState::FULL_SYNC;
    g_snap.phase_A = true;

    ems::engine::auxiliaries_tick_10ms();
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_vvt_esc_duty() > 0u);
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_vvt_adm_duty() > 0u);

    for (uint8_t i = 0u; i < 21u; ++i) {
        ems::engine::auxiliaries_tick_10ms();
    }

    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_vvt_esc_duty() == 0u);
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_vvt_adm_duty() == 0u);
}

void test_fan_hysteresis_and_pump_prime_logic() {
    reset_fixture();
    ems::engine::auxiliaries_init();

    g_sensors.clt_degc_x10 = 960;
    ems::engine::auxiliaries_tick_10ms();
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_fan_state());

    g_sensors.clt_degc_x10 = 930;
    ems::engine::auxiliaries_tick_10ms();
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_fan_state());

    g_sensors.clt_degc_x10 = 890;
    ems::engine::auxiliaries_tick_10ms();
    TEST_ASSERT_TRUE(!ems::engine::auxiliaries_test_get_fan_state());

    ems::engine::auxiliaries_set_key_on(true);
    g_snap.rpm_x10 = 0u;

    for (uint16_t i = 0u; i < 200u; ++i) {
        ems::engine::auxiliaries_tick_10ms();
    }
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_pump_state());

    for (uint16_t i = 0u; i < 300u; ++i) {
        ems::engine::auxiliaries_tick_10ms();
    }
    TEST_ASSERT_TRUE(!ems::engine::auxiliaries_test_get_pump_state());

    g_snap.rpm_x10 = 1200u;
    ems::engine::auxiliaries_tick_10ms();
    TEST_ASSERT_TRUE(ems::engine::auxiliaries_test_get_pump_state());
}

}  // namespace

int main() {
    test_pwm_init_strategy();
    test_wastegate_overboost_failsafe_after_500ms();
    test_vvt_failsafe_when_no_phase_confirm_for_200ms();
    test_fan_hysteresis_and_pump_prime_logic();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
