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

ems::drv::CkpSnapshot g_snap = {1000000u, 0u, 0u, 8000u, ems::drv::SyncState::SYNCED, true};
ems::drv::SensorData g_sensors = {
    1000u,
    0u,
    250u,
    900,
    250,
    450u,
    3000u,
    2500u,
    13500u,
    0u,
};

uint16_t g_last_iac_duty = 0u;

}  // namespace

namespace ems::drv {
CkpSnapshot ckp_snapshot() noexcept {
    return g_snap;
}

const SensorData& sensors_get() noexcept {
    return g_sensors;
}
}  // namespace ems::drv

namespace ems::hal {
void ftm1_pwm_init(uint32_t) {}
void ftm2_pwm_init(uint32_t) {}

void ftm1_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    if (ch == 0u) {
        g_last_iac_duty = duty_pct_x10;
    }
}

void ftm2_set_duty(uint8_t, uint16_t) noexcept {}
}  // namespace ems::hal

namespace {

void run_closed_loop_steps(uint32_t steps) {
    for (uint32_t i = 0u; i < steps; ++i) {
        ems::engine::auxiliaries_tick_20ms();

        const int32_t rpm = static_cast<int32_t>(g_snap.rpm_x10);
        const int32_t duty = static_cast<int32_t>(g_last_iac_duty);

        // Planta de 1a ordem: RPM de equilíbrio cresce com duty de IACV.
        const int32_t rpm_ss = 5000 + (duty * 6);
        int32_t next_rpm = rpm + ((rpm_ss - rpm) / 16);
        if (next_rpm < 0) {
            next_rpm = 0;
        }
        if (next_rpm > 20000) {
            next_rpm = 20000;
        }
        g_snap.rpm_x10 = static_cast<uint32_t>(next_rpm);
    }
}

void test_iacv_pid_converges_after_step() {
    ems::engine::auxiliaries_init();
    ems::engine::auxiliaries_set_key_on(true);

    // Motor quente -> PID ativo.
    g_sensors.clt_degc_x10 = 900;
    g_snap.rpm_x10 = 7800u;

    run_closed_loop_steps(120u);

    const uint32_t rpm_pre_step = g_snap.rpm_x10;
    const int32_t err_pre = (rpm_pre_step > 8200u)
        ? static_cast<int32_t>(rpm_pre_step - 8200u)
        : static_cast<int32_t>(8200u - rpm_pre_step);
    TEST_ASSERT_TRUE(err_pre < 2000);

    // Step de carga: derruba RPM abruptamente.
    g_snap.rpm_x10 = 7600u;
    const int32_t err_step = 600;
    run_closed_loop_steps(500u);

    const uint32_t rpm_after = g_snap.rpm_x10;
    const int32_t err_after = (rpm_after > 8200u)
        ? static_cast<int32_t>(rpm_after - 8200u)
        : static_cast<int32_t>(8200u - rpm_after);
    TEST_ASSERT_TRUE(err_after < err_step);
    TEST_ASSERT_TRUE(g_last_iac_duty > 0u);

    // Duty deve permanecer dentro do range válido.
    TEST_ASSERT_TRUE(g_last_iac_duty <= 1000u);
}

}  // namespace

int main() {
    test_iacv_pid_converges_after_step();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
