#include "test/harness.h"
#include "test/fixtures.h"
#include "test/ui_helpers.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "engine/etb_control.h"
#include "hal/etb_driver.h"
#include "engine/torque_manager.h"
#include "engine/calibration.h"
#include "app/can_rx_map.h"
#include "hal/adc.h"
#include "hal/system.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/auxiliaries.h"
#include "engine/knock.h"
#include "engine/table3d.h"
#include "engine/ecu_sched.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "engine/output_test.h"
#include "engine/engine_config.h"
#include "hal/timer.h"
#include "hal/flash.h"
#include "app/ui_protocol.h"
#include "app/status_bits.h"
#include "hal/crc32.h"

namespace ems::engine {
    int16_t etb_get_idle_spark_trim() noexcept;
}

extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

using namespace ems::drv;
using namespace ems::engine;
using namespace ems::app;
using namespace ems::hal;

void test_etb_driver_adc_to_percent(void) {
    section("etb_driver_adc_to_percent");

    // Below ETB_TPS_NORMAL_MIN (400) → 0 %
    CHECK_NEAR(etb_driver_adc_to_percent(0u),   0.0f, 0.01f, "adc=0 → 0%");
    CHECK_NEAR(etb_driver_adc_to_percent(200u), 0.0f, 0.01f, "adc=ADC_MIN(200) → 0%");
    CHECK_NEAR(etb_driver_adc_to_percent(399u), 0.0f, 0.01f, "adc=399 < NORMAL_MIN → 0%");
    CHECK_NEAR(etb_driver_adc_to_percent(400u), 0.0f, 0.01f, "adc=NORMAL_MIN(400) → 0%");

    // Above ETB_TPS_NORMAL_MAX (3700) → 100 %
    CHECK_NEAR(etb_driver_adc_to_percent(3700u), 100.0f, 0.01f, "adc=NORMAL_MAX(3700) → 100%");
    CHECK_NEAR(etb_driver_adc_to_percent(3900u), 100.0f, 0.01f, "adc=ADC_MAX(3900) → 100%");
    CHECK_NEAR(etb_driver_adc_to_percent(4095u), 100.0f, 0.01f, "adc=4095 → 100%");

    // Mid-point: (400+3700)/2 = 2050 → 50 %
    CHECK_NEAR(etb_driver_adc_to_percent(2050u), 50.0f, 0.1f, "adc=2050 → ~50%");

    // Just above NORMAL_MIN → tiny positive value
    CHECK_TRUE(etb_driver_adc_to_percent(401u) > 0.0f, "adc=401 > NORMAL_MIN → >0%");
}

void test_etb_driver_init_and_state(void) {
    section("etb_driver_init / get_state");

    drv_setup();
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_OFF, "state=OFF before init");

    const bool ok = etb_driver_init();
    CHECK_TRUE(ok, "init returns true with valid ADC");
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_READY, "state=READY after init");
}

void test_etb_driver_init_fault_tps1_open(void) {
    section("etb_driver_init fault: TPS1 open (ADC below min)");

    etb_driver_test_reset();
    using namespace ems::hal;
    // t1 = 100 < ETB_TPS_ADC_MIN(200) → TPS1_OPEN
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 100u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 2050u);

    const bool ok = etb_driver_init();
    CHECK_FALSE(ok, "init fails when TPS1 open");
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_FAULT, "state=FAULT on TPS1 open");
}

void test_etb_driver_init_fault_tps2_short(void) {
    section("etb_driver_init fault: TPS2 short (ADC above max)");

    etb_driver_test_reset();
    using namespace ems::hal;
    // t2 = 4000 > ETB_TPS_ADC_MAX(3900) → TPS2_SHORT
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 2050u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 4000u);

    const bool ok = etb_driver_init();
    CHECK_FALSE(ok, "init fails when TPS2 short");
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_FAULT, "state=FAULT on TPS2 short");
}

void test_etb_driver_read_sensors_valid(void) {
    section("etb_driver_read_sensors: valid reading");

    drv_setup();
    etb_driver_init();

    etb_driver_data_t data{};
    const etb_driver_fault_t fault = etb_driver_read_sensors(&data);
    CHECK_EQ(fault, ETB_DRV_OK, "read_sensors → OK");
    CHECK_EQ(data.tps1_raw, 2050u, "tps1_raw captured");
    CHECK_EQ(data.tps2_raw, 2050u, "tps2_raw captured");
    CHECK_NEAR(data.tps_validated, 50.0f, 0.5f, "tps_validated ≈ 50%");
}

void test_etb_driver_read_sensors_mismatch(void) {
    section("etb_driver_read_sensors: TPS mismatch (diff > 12%)");

    drv_setup();
    etb_driver_init();  // init with matching values

    // Now inject mismatched values: TPS1=0%, TPS2=50% → diff=50% > 12%
    using namespace ems::hal;
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 400u);   // 0%
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 2050u);  // 50%

    etb_driver_data_t data{};
    const etb_driver_fault_t fault = etb_driver_read_sensors(&data);
    CHECK_EQ(fault, ETB_DRV_FAULT_TPS_MISMATCH, "large TPS diff → MISMATCH fault");
}

void test_etb_driver_read_sensors_null(void) {
    section("etb_driver_read_sensors: null pointer guard");

    drv_setup();
    etb_driver_init();

    const etb_driver_fault_t fault = etb_driver_read_sensors(nullptr);
    CHECK_EQ(fault, ETB_DRV_FAULT_NOT_INITIALIZED, "nullptr → NOT_INITIALIZED");
}

void test_etb_driver_set_motor_pwm(void) {
    section("etb_driver_set_motor_pwm");

    etb_driver_test_reset();
    // Not READY → rejected
    CHECK_FALSE(etb_driver_set_motor_pwm(500), "set_motor_pwm false when not READY");

    drv_setup();
    etb_driver_init();

    CHECK_TRUE(etb_driver_set_motor_pwm(0),    "pwm=0 accepted");
    CHECK_TRUE(etb_driver_set_motor_pwm(512),  "pwm=+512 accepted");
    CHECK_TRUE(etb_driver_set_motor_pwm(1023), "pwm=+1023 (max) accepted");
    CHECK_TRUE(etb_driver_set_motor_pwm(-512), "pwm=-512 accepted");
    CHECK_TRUE(etb_driver_set_motor_pwm(-1023),"pwm=-1023 (min) accepted");
    // Values beyond ±1023 are clamped but still accepted
    CHECK_TRUE(etb_driver_set_motor_pwm(2000), "pwm=2000 clamped+accepted");
    CHECK_TRUE(etb_driver_set_motor_pwm(-2000),"pwm=-2000 clamped+accepted");
}

void test_etb_driver_shutdown(void) {
    section("etb_driver_shutdown");

    drv_setup();
    etb_driver_init();
    etb_driver_set_motor_pwm(800);

    etb_driver_shutdown();
    // Driver must remain usable (state not changed to FAULT by shutdown)
    CHECK_TRUE(etb_driver_set_motor_pwm(0), "driver still usable after shutdown");
}

void test_etb_driver_clear_fault(void) {
    section("etb_driver_clear_fault");

    // Trigger fault on init
    etb_driver_test_reset();
    using namespace ems::hal;
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 100u);  // TPS1 open
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 2050u);
    etb_driver_init();
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_FAULT, "pre-cond: FAULT state");

    // Fix ADC, then clear fault (clear_fault calls etb_driver_init internally)
    drv_set_valid_adc();
    etb_driver_clear_fault();
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_READY, "state=READY after clear_fault");
}

// ═══════════════════════════════════════════════════════════════════════════════
// ETB CONTROL TESTS
// ═══════════════════════════════════════════════════════════════════════════════

void test_etb_set_get_drive_mode(void) {
    section("etb_set_drive_mode / etb_get_drive_mode");

    etb_ctrl_setup();
    etb_control_init();

    etb_set_drive_mode(ETB_MODE_ECO);
    CHECK_EQ(etb_get_drive_mode(), ETB_MODE_ECO, "mode=ECO");

    etb_set_drive_mode(ETB_MODE_SPORT);
    CHECK_EQ(etb_get_drive_mode(), ETB_MODE_SPORT, "mode=SPORT");

    etb_set_drive_mode(ETB_MODE_RAIN);
    CHECK_EQ(etb_get_drive_mode(), ETB_MODE_RAIN, "mode=RAIN");

    etb_set_drive_mode(ETB_MODE_NORMAL);
    CHECK_EQ(etb_get_drive_mode(), ETB_MODE_NORMAL, "mode=NORMAL");

    // Out-of-range mode must be rejected; current mode unchanged
    etb_set_drive_mode(static_cast<etb_drive_mode_t>(99));
    CHECK_EQ(etb_get_drive_mode(), ETB_MODE_NORMAL, "invalid mode rejected");
}

void test_etb_is_ready(void) {
    section("etb_is_ready");

    etb_ctrl_setup();
    etb_control_init();
    CHECK_TRUE(etb_is_ready(), "ready after successful init");
}

void test_etb_enter_limp_mode(void) {
    section("etb_enter_limp_mode");

    etb_ctrl_setup();
    etb_control_init();
    CHECK_TRUE(etb_is_ready(), "pre-cond: ready");

    etb_enter_limp_mode();
    CHECK_FALSE(etb_is_ready(), "not ready after limp");
}

void test_etb_set_idle_control_and_spark_trim(void) {
    section("etb_set_idle_control / etb_get_idle_spark_trim");

    etb_ctrl_setup();
    etb_control_init();

    // Activate idle at 850 RPM target
    etb_set_idle_control(true, 850.0f);

    // rpm=800 → idle_error=50 < 100 → trim = (int16_t)(50 * 0.5) = 25
    etb_control_loop(0.0f, 800.0f, 10.0f);
    const int16_t trim_active = ems::engine::etb_get_idle_spark_trim();
    CHECK_NEAR(static_cast<float>(trim_active), 25.0f, 5.0f,
               "idle trim ≈ 25 at 50 RPM under target");

    // Deactivate idle → trim must return to 0
    etb_set_idle_control(false, 0.0f);
    etb_control_loop(0.0f, 800.0f, 10.0f);
    const int16_t trim_off = ems::engine::etb_get_idle_spark_trim();
    CHECK_EQ(trim_off, 0, "trim=0 when idle disabled");
}

void test_etb_get_throttle_position(void) {
    section("etb_get_throttle_position");

    etb_ctrl_setup();
    // ADC at 2050 → tps_validated ≈ 50%
    etb_control_init();
    etb_control_loop(50.0f, 3000.0f, 10.0f);

    const float pos = etb_get_throttle_position();
    CHECK_NEAR(pos, 50.0f, 1.0f, "throttle_position ≈ 50% at mid ADC");
}

void test_etb_control_loop_rpm_cutoff(void) {
    section("etb_control_loop: RPM over-rev cutoff");

    etb_ctrl_setup();
    etb_control_init();

    // RPM way above cutoff (7000) — must not crash, position remains valid
    etb_control_loop(100.0f, 8000.0f, 10.0f);
    const float pos = etb_get_throttle_position();
    CHECK_TRUE(pos >= 0.0f && pos <= 100.0f, "throttle_position in [0,100] during RPM cut");
}

void test_etb_control_loop_sensor_fault_triggers_limp(void) {
    section("etb_control_loop: sensor fault during loop triggers limp");

    etb_ctrl_setup();
    etb_control_init();
    CHECK_TRUE(etb_is_ready(), "pre-cond: ready");

    // Inject TPS1 fault AFTER successful init
    using namespace ems::hal;
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 100u);  // < ADC_MIN → open

    etb_control_loop(50.0f, 3000.0f, 10.0f);
    CHECK_FALSE(etb_is_ready(), "limp mode engaged after TPS fault in loop");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TORQUE MANAGER TESTS
// ═══════════════════════════════════════════════════════════════════════════════

void test_etb_cpp_update(void) {
    section("etb_control (C++ ns): reset / update / test_get_integrator");

    // etb_control_update requires etb_cal_valid != 0
    etb_cal_valid = 1u;
    etb_kp_x10  = 120u;  // kp = 12.0
    etb_ki_x10  = 8u;
    etb_kd_x10  = 40u;

    ems::engine::etb_control_reset();
    CHECK_EQ(ems::engine::etb_control_test_get_integrator(), 0, "integrator=0 after reset");

    // Disabled: enable_request=false → output inactive, integrator stays 0
    auto s = ems::engine::etb_control_update(500u, 0u, false, 10u);
    CHECK_FALSE(s.active, "enable=false → not active");
    CHECK_EQ(s.output_pct_x10, 0, "disabled → output=0");
    CHECK_EQ(ems::engine::etb_control_test_get_integrator(), 0, "integrator=0 when disabled");

    // Invalid calibration: etb_cal_valid=0 → disabled
    etb_cal_valid = 0u;
    s = ems::engine::etb_control_update(500u, 0u, true, 10u);
    CHECK_FALSE(s.active, "cal_invalid → not active");
    etb_cal_valid = 1u;

    // Enabled with positive error: output should be positive (opening)
    ems::engine::etb_control_reset();
    s = ems::engine::etb_control_update(500u, 0u, true, 10u);  // target=50%, measured=0%
    CHECK_TRUE(s.active, "enabled + cal_valid → active");
    CHECK_TRUE(s.output_pct_x10 > 0, "positive error → positive output");
    CHECK_EQ(s.position_error_x10, 500, "error = target - measured = 500");

    // Integrator accumulates with small error (P must stay below saturation limit).
    // kp_x10=120, error=50 (5%): P = (120*50)/10 = 600 < 1000 → not saturating.
    ems::engine::etb_control_reset();
    for (int i = 0; i < 5; ++i) {
        ems::engine::etb_control_update(50u, 0u, true, 10u);  // 5% error, small P
    }
    CHECK_TRUE(ems::engine::etb_control_test_get_integrator() > 0,
               "integrator grows with sustained small error");

    // No error (target == measured): integrator should stop growing
    ems::engine::etb_control_reset();
    ems::engine::etb_control_update(300u, 300u, true, 10u);
    CHECK_EQ(ems::engine::etb_control_test_get_integrator(), 0, "no error → integrator stays 0");
}

// ═══════════════════════════════════════════════════════════════════════════
// TORQUE MANAGER — C++ namespace (ems::engine)
// ═══════════════════════════════════════════════════════════════════════════

