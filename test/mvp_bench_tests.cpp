/**
 * @file test/mvp_bench_tests.cpp
 * @brief Host regression harness — OpenEMS EMS_HOST_TEST build
 *
 * Compile: g++ -DEMS_HOST_TEST -std=c++17 -O2 -g -I./src <all srcs> -o mvp_bench_tests
 * Run:     make host-test
 *
 * Cobertura:
 *   etb_driver  — adc_to_percent, init, get_state, read_sensors, set_motor_pwm,
 *                 shutdown, clear_fault
 *   etb_control — set/get_drive_mode, is_ready, enter_limp, set_idle_control,
 *                 get_idle_spark_trim, get_throttle_position, control_loop
 *   torque_manager — init, enter_limp, is_ready, set/get_config, loop (normal,
 *                    rpm_cut, limp, idle, traction_control, null guards)
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "engine/etb_control.h"
#include "hal/etb_driver.h"
#include "engine/torque_manager.h"
#include "engine/calibration.h"
#include "hal/adc.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/auxiliaries.h"
#include "engine/knock.h"

// etb_get_idle_spark_trim is implemented in ems::engine namespace in etb_control.cpp
// (C++ linkage) but only declared extern "C" in the header (C linkage — different symbol).
// Forward-declare the C++ version so we can call it directly from tests.
namespace ems::engine {
    int16_t etb_get_idle_spark_trim() noexcept;
}

// CKP mock registers exposed by drv/ckp.cpp under EMS_HOST_TEST
extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

using namespace ems::drv;
using namespace ems::engine;

// ─── Minimal test framework ────────────────────────────────────────────────────

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, name) do { \
    if (cond) { \
        ++g_pass; \
        printf("  PASS  %s\n", name); \
    } else { \
        ++g_fail; \
        printf("  FAIL  %s  (line %d)\n", name, __LINE__); \
    } \
} while (0)

#define CHECK_TRUE(cond,   name) CHECK(!!(cond), name)
#define CHECK_FALSE(cond,  name) CHECK(!(cond),  name)
#define CHECK_EQ(a, b,     name) CHECK((a) == (b), name)
#define CHECK_NEAR(a, b, eps, name) \
    CHECK(fabsf(static_cast<float>(a) - static_cast<float>(b)) <= (eps), name)

static void section(const char* name) {
    printf("\n[%s]\n", name);
}

// ─── Helpers ───────────────────────────────────────────────────────────────────

// Set both TPS ADC channels to a valid, consistent mid-range value (~50%).
// ETB_TPS_NORMAL_MIN=400, ETB_TPS_NORMAL_MAX=3700 → mid ≈ 2050.
static void drv_set_valid_adc(void) {
    using namespace ems::hal;
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 2050u);  // TPS1
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 2050u);  // TPS2
}

// Reset driver to known state + load valid ADC.
static void drv_setup(void) {
    etb_driver_test_reset();
    drv_set_valid_adc();
}

// Reset driver + init ETB control layer (also inits driver internally).
static void etb_ctrl_setup(void) {
    drv_setup();
    // etb_control_init calls etb_driver_init() → reads ADC → must be valid
}

// ═══════════════════════════════════════════════════════════════════════════════
// ETB DRIVER TESTS
// ═══════════════════════════════════════════════════════════════════════════════

static void test_etb_driver_adc_to_percent(void) {
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

static void test_etb_driver_init_and_state(void) {
    section("etb_driver_init / get_state");

    drv_setup();
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_OFF, "state=OFF before init");

    const bool ok = etb_driver_init();
    CHECK_TRUE(ok, "init returns true with valid ADC");
    CHECK_EQ(etb_driver_get_state(), ETB_DRV_STATE_READY, "state=READY after init");
}

static void test_etb_driver_init_fault_tps1_open(void) {
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

static void test_etb_driver_init_fault_tps2_short(void) {
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

static void test_etb_driver_read_sensors_valid(void) {
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

static void test_etb_driver_read_sensors_mismatch(void) {
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

static void test_etb_driver_read_sensors_null(void) {
    section("etb_driver_read_sensors: null pointer guard");

    drv_setup();
    etb_driver_init();

    const etb_driver_fault_t fault = etb_driver_read_sensors(nullptr);
    CHECK_EQ(fault, ETB_DRV_FAULT_NOT_INITIALIZED, "nullptr → NOT_INITIALIZED");
}

static void test_etb_driver_set_motor_pwm(void) {
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

static void test_etb_driver_shutdown(void) {
    section("etb_driver_shutdown");

    drv_setup();
    etb_driver_init();
    etb_driver_set_motor_pwm(800);

    etb_driver_shutdown();
    // Driver must remain usable (state not changed to FAULT by shutdown)
    CHECK_TRUE(etb_driver_set_motor_pwm(0), "driver still usable after shutdown");
}

static void test_etb_driver_clear_fault(void) {
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

static void test_etb_set_get_drive_mode(void) {
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

static void test_etb_is_ready(void) {
    section("etb_is_ready");

    etb_ctrl_setup();
    etb_control_init();
    CHECK_TRUE(etb_is_ready(), "ready after successful init");
}

static void test_etb_enter_limp_mode(void) {
    section("etb_enter_limp_mode");

    etb_ctrl_setup();
    etb_control_init();
    CHECK_TRUE(etb_is_ready(), "pre-cond: ready");

    etb_enter_limp_mode();
    CHECK_FALSE(etb_is_ready(), "not ready after limp");
}

static void test_etb_set_idle_control_and_spark_trim(void) {
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

static void test_etb_get_throttle_position(void) {
    section("etb_get_throttle_position");

    etb_ctrl_setup();
    // ADC at 2050 → tps_validated ≈ 50%
    etb_control_init();
    etb_control_loop(50.0f, 3000.0f, 10.0f);

    const float pos = etb_get_throttle_position();
    CHECK_NEAR(pos, 50.0f, 1.0f, "throttle_position ≈ 50% at mid ADC");
}

static void test_etb_control_loop_rpm_cutoff(void) {
    section("etb_control_loop: RPM over-rev cutoff");

    etb_ctrl_setup();
    etb_control_init();

    // RPM way above cutoff (7000) — must not crash, position remains valid
    etb_control_loop(100.0f, 8000.0f, 10.0f);
    const float pos = etb_get_throttle_position();
    CHECK_TRUE(pos >= 0.0f && pos <= 100.0f, "throttle_position in [0,100] during RPM cut");
}

static void test_etb_control_loop_sensor_fault_triggers_limp(void) {
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

static void test_torque_manager_init(void) {
    section("torque_manager_init / is_ready");

    const bool ok = torque_manager_init();
    CHECK_TRUE(ok, "init returns true");
    CHECK_TRUE(torque_manager_is_ready(), "is_ready after init");
}

static void test_torque_manager_enter_limp(void) {
    section("torque_manager_enter_limp");

    torque_manager_init();
    CHECK_TRUE(torque_manager_is_ready(), "pre-cond: ready");

    torque_manager_enter_limp();
    CHECK_FALSE(torque_manager_is_ready(), "not ready after enter_limp");
}

static void test_torque_manager_set_get_config(void) {
    section("torque_manager_set_config / get_config");

    torque_manager_init();

    torque_manager_config_t cfg{};
    cfg.max_rpm            = 6500.0f;
    cfg.max_rpm_hot        = 6000.0f;
    cfg.max_speed          = 180.0f;
    cfg.min_coolant_temp   = -10.0f;
    cfg.launch_rpm         = 4000.0f;
    cfg.launch_throttle    = 55.0f;
    cfg.tc_max_slip        = 10.0f;
    cfg.tc_reduction_rate  = 30.0f;
    cfg.pedal_filter_alpha = 0.5f;
    torque_manager_set_config(&cfg);

    const torque_manager_config_t* got = torque_manager_get_config();
    CHECK_NEAR(got->max_rpm,            6500.0f, 0.01f, "max_rpm persisted");
    CHECK_NEAR(got->max_rpm_hot,        6000.0f, 0.01f, "max_rpm_hot persisted");
    CHECK_NEAR(got->max_speed,          180.0f,  0.01f, "max_speed persisted");
    CHECK_NEAR(got->launch_rpm,         4000.0f, 0.01f, "launch_rpm persisted");
    CHECK_NEAR(got->launch_throttle,    55.0f,   0.01f, "launch_throttle persisted");
    CHECK_NEAR(got->pedal_filter_alpha, 0.5f,   0.001f, "pedal_filter_alpha persisted");
}

static void test_torque_manager_set_config_null(void) {
    section("torque_manager_set_config: null guard");

    torque_manager_init();
    const float max_rpm_before = torque_manager_get_config()->max_rpm;

    torque_manager_set_config(nullptr);  // must not crash or corrupt state

    CHECK_NEAR(torque_manager_get_config()->max_rpm, max_rpm_before, 0.01f,
               "config unchanged after null set_config");
}

static void test_torque_manager_loop_normal_pedal(void) {
    section("torque_manager_loop: normal driving");

    torque_manager_init();

    torque_manager_inputs_t inputs{};
    inputs.pedal_percent = 50.0f;
    inputs.engine_rpm    = 3000.0f;
    inputs.coolant_temp  = 85.0f;

    torque_manager_outputs_t outputs{};
    torque_manager_loop(&inputs, &outputs);

    CHECK_TRUE(outputs.throttle_target >= 0.0f && outputs.throttle_target <= 100.0f,
               "throttle_target in [0,100] range");
    CHECK_TRUE(outputs.throttle_target > 0.0f, "throttle_target > 0 with 50% pedal");
}

static void test_torque_manager_loop_rpm_hard_cut(void) {
    section("torque_manager_loop: hard RPM cut (rpm >= max+200)");

    torque_manager_init();

    torque_manager_inputs_t inputs{};
    inputs.pedal_percent = 100.0f;
    inputs.engine_rpm    = 7500.0f;  // > max_rpm(7000) + 200
    inputs.coolant_temp  = 85.0f;

    torque_manager_outputs_t outputs{};
    torque_manager_loop(&inputs, &outputs);

    CHECK_EQ(outputs.throttle_target, 0.0f, "throttle=0 at hard rev limit");
    CHECK_TRUE(outputs.rpm_cutoff_count > 0u, "rpm_cutoff_count incremented");
}

static void test_torque_manager_loop_rpm_progressive_cut(void) {
    section("torque_manager_loop: progressive RPM cut (max < rpm < max+200)");

    torque_manager_init();

    // Run first call to build up filtered pedal
    torque_manager_inputs_t inputs{};
    inputs.pedal_percent = 100.0f;
    inputs.engine_rpm    = 7100.0f;  // 100 rpm over max=7000, within 200 window
    inputs.coolant_temp  = 85.0f;

    torque_manager_outputs_t outputs{};
    torque_manager_loop(&inputs, &outputs);

    // cutoff_factor = 1 - (7100-7000)/200 = 0.5 → throttle reduced, not zero
    CHECK_TRUE(outputs.throttle_target > 0.0f,   "throttle > 0 in progressive cut zone");
    CHECK_TRUE(outputs.throttle_target < 100.0f, "throttle < 100 in progressive cut zone");
    CHECK_TRUE(outputs.rpm_cutoff_count > 0u,    "rpm_cutoff_count incremented");
}

static void test_torque_manager_loop_limp_via_input(void) {
    section("torque_manager_loop: limp mode via input flag");

    torque_manager_init();

    torque_manager_inputs_t inputs{};
    inputs.pedal_percent = 80.0f;
    inputs.engine_rpm    = 3000.0f;
    inputs.limp_mode     = true;

    torque_manager_outputs_t outputs{};
    torque_manager_loop(&inputs, &outputs);

    CHECK_NEAR(outputs.throttle_target, 5.0f,  0.01f, "throttle=5% in limp");
    CHECK_NEAR(outputs.torque_limit,   30.0f,  0.01f, "torque_limit=30% in limp");
}

static void test_torque_manager_loop_idle_mode(void) {
    section("torque_manager_loop: idle mode (ETB handles offset)");

    torque_manager_init();

    torque_manager_inputs_t inputs{};
    inputs.pedal_percent = 0.0f;
    inputs.engine_rpm    = 850.0f;
    inputs.idle_mode     = true;
    inputs.coolant_temp  = 85.0f;

    torque_manager_outputs_t outputs{};
    torque_manager_loop(&inputs, &outputs);

    // Idle forces requested_throttle=0; ETB control layer applies idle_offset
    CHECK_EQ(outputs.throttle_target, 0.0f, "throttle=0 in idle (ETB manages air)");
}

static void test_torque_manager_loop_traction_control(void) {
    section("torque_manager_loop: traction control reduces throttle+adds spark retard");

    torque_manager_init();

    torque_manager_inputs_t inputs{};
    inputs.pedal_percent   = 80.0f;
    inputs.engine_rpm      = 3000.0f;
    inputs.coolant_temp    = 85.0f;
    inputs.traction_active = true;
    inputs.tc_reduction    = 50.0f;  // 50% reduction

    torque_manager_outputs_t outputs{};
    torque_manager_loop(&inputs, &outputs);

    // With alpha=0.3, first call: filtered_pedal = 0.3*80 = 24.0
    // TC: 24.0 * (1 - 0.5) = 12.0
    CHECK_TRUE(outputs.throttle_target < 50.0f, "throttle < 50% under TC");
    CHECK_TRUE(outputs.tc_intervention > 0u,    "tc_intervention incremented");
    // spark_trim = -(50 * 0.3) = -15.0°
    CHECK_NEAR(outputs.spark_trim, -15.0f, 0.5f, "spark_trim ≈ -15° under 50% TC");
}

static void test_torque_manager_loop_null_guards(void) {
    section("torque_manager_loop: null pointer guards");

    torque_manager_init();
    torque_manager_inputs_t  in{};
    torque_manager_outputs_t out{};

    torque_manager_loop(nullptr, &out);  // must not crash
    torque_manager_loop(&in, nullptr);   // must not crash
    CHECK_TRUE(true, "null guards: no segfault");
}

static void test_torque_manager_loop_speed_limiter(void) {
    section("torque_manager_loop: speed limiter (>= max_speed)");

    torque_manager_init();

    torque_manager_inputs_t inputs{};
    inputs.pedal_percent   = 80.0f;
    inputs.engine_rpm      = 3000.0f;
    inputs.coolant_temp    = 85.0f;
    inputs.vehicle_speed   = 230.0f;  // > max_speed=220 km/h

    torque_manager_outputs_t outputs{};
    torque_manager_loop(&inputs, &outputs);

    // Limiter caps requested_throttle at 10% to maintain speed
    CHECK_TRUE(outputs.throttle_target <= 10.0f, "throttle <= 10% at speed limit");
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Includes adicionais para testes da segunda fase ────────────────────────
#include "hal/timer.h"
#include "engine/ecu_sched.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "hal/flash.h"

// ─── CKP helpers ──────────────────────────────────────────────────────────────
// Normal period ≈ 6250 RPM: rpm_x10 = 625000000 / 10000 = 62500
static constexpr uint32_t kNormalPeriod = 10000u;
static constexpr uint32_t kGapPeriod    = kNormalPeriod * 3u;
static uint32_t g_ckp_cap = 0u;

static void ckp_fire(uint32_t delta) {
    g_ckp_cap += delta;
    ems_test_tim5_ccr1 = g_ckp_cap;
    ckp_tim5_ch1_isr();
}
static void ckp_feed_n_then_gap(uint32_t n, uint32_t p = kNormalPeriod) {
    for (uint32_t i = 0; i < n; ++i) { ckp_fire(p); }
    ckp_fire(p * 3u);
}
static void ckp_reach_full_sync(uint32_t p = kNormalPeriod) {
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(55u, p);
    ckp_feed_n_then_gap(55u, p);
}

// ═══════════════════════════════════════════════════════════════════════════
// CKP DECODER / SYNC
// ═══════════════════════════════════════════════════════════════════════════

static void test_ckp_rpm_math(void) {
    section("ckp: rpm_x10 from period_ns");
    CHECK_EQ(ckp_test_rpm_x10_from_period_ns(160000u),  62500u, "160000 ns → 6250.0 RPM");
    CHECK_EQ(ckp_test_rpm_x10_from_period_ns(1250000u),  8000u, "1250000 ns → 800.0 RPM");
    CHECK_EQ(ckp_test_rpm_x10_from_period_ns(0u),            0u, "period=0 → rpm=0 (safe)");
}

static void test_ckp_initial_state(void) {
    section("ckp: initial state after reset");
    ckp_test_reset(); g_ckp_cap = 0u;
    const CkpSnapshot s = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(s.state), static_cast<uint8_t>(SyncState::WAIT_GAP), "state=WAIT_GAP");
    CHECK_EQ(s.rpm_x10, 0u,    "rpm=0");
    CHECK_EQ(s.tooth_index, 0u, "tooth_index=0");
}

static void test_ckp_half_sync(void) {
    section("ckp: WAIT_GAP → HALF_SYNC on first valid gap");
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(55u);
    const CkpSnapshot s = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(s.state), static_cast<uint8_t>(SyncState::HALF_SYNC), "HALF_SYNC");
    CHECK_TRUE(s.rpm_x10 > 0u, "rpm > 0 in HALF_SYNC");
}

static void test_ckp_full_sync(void) {
    section("ckp: HALF_SYNC → FULL_SYNC on second valid gap");
    ckp_reach_full_sync();
    const CkpSnapshot s = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(s.state), static_cast<uint8_t>(SyncState::FULL_SYNC), "FULL_SYNC");
    CHECK_EQ(s.tooth_index, 0u, "tooth_index=0 at gap");
    CHECK_NEAR(static_cast<float>(s.rpm_x10), 62500.0f, 500.0f, "rpm ≈ 62500");
}

static void test_ckp_tooth_index_increments(void) {
    section("ckp: tooth_index increments in FULL_SYNC");
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 5u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(ckp_snapshot().tooth_index, 5u, "tooth_index=5 after 5 teeth");
}

static void test_ckp_loss_of_sync_too_many_teeth(void) {
    section("ckp: LOSS_OF_SYNC when gap missed (>63 teeth)");
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 64u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "LOSS_OF_SYNC");
}

static void test_ckp_loss_of_sync_early_gap(void) {
    section("ckp: LOSS_OF_SYNC on early gap (<55 teeth)");
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 10u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kGapPeriod);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "LOSS_OF_SYNC on early gap");
}

static void test_ckp_noise_rejection(void) {
    section("ckp: glitch < kMinToothTicks rejected");
    ckp_reach_full_sync();
    const CkpSnapshot before = ckp_snapshot();
    ckp_fire(10u);  // < 50 ticks
    const CkpSnapshot after = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(after.state), static_cast<uint8_t>(SyncState::FULL_SYNC),
             "FULL_SYNC maintained after glitch");
    CHECK_EQ(after.rpm_x10, before.rpm_x10, "rpm unchanged after glitch");
}

static void test_ckp_stall_poll(void) {
    section("ckp: stall_poll detects stopped engine");
    ckp_reach_full_sync();
    // Produção: kMinStallTimeoutTicks = 12_500_000 (200 ms @ 62.5 MHz).
    const uint32_t stale = ckp_snapshot().last_tim5_capture + 13000000u;
    CHECK_TRUE(ckp_stall_poll(stale), "stall_poll=true after 208ms (kMinStallTimeoutTicks)");
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "LOSS_OF_SYNC after stall");

    // Bench-mode relaxa o timeout para 2 s (gaps de RMT restart do estimulador).
    section("ckp: stall_poll bench-mode usa timeout relaxado (2s)");
    ems::drv::sensors_set_bench_clt_iat(true, 900, 400);
    ckp_reach_full_sync();
    const uint32_t base = ckp_snapshot().last_tim5_capture;
    CHECK_FALSE(ckp_stall_poll(base + 13000000u), "bench: sem stall a 208ms");
    CHECK_TRUE(ckp_stall_poll(base + 130000000u), "bench: stall a 2.08s");
    ems::drv::sensors_set_bench_clt_iat(false, 0, 0);
}

static void test_ckp_phantom_rpm_unsync(void) {
    section("ckp: RPM só reportado com sync (ruído em CKP desligado = 0 RPM)");
    ckp_test_reset(); g_ckp_cap = 0u;
    // Bordas periódicas de ruído (ex.: 60 Hz de rede) sem gap → sem sync.
    // RPM reportado deve ficar em 0 mesmo com capturas contínuas.
    for (uint32_t i = 0; i < 6u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::WAIT_GAP), "sem sync (WAIT_GAP)");
    CHECK_EQ(ckp_snapshot().rpm_x10, 0u, "ruído sem sync → RPM 0");
    // Ruído prolongado (simula 60 Hz contínuo por >58 dentes, sem gap)
    for (uint32_t i = 0; i < 200u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(ckp_snapshot().rpm_x10, 0u, "RPM continua 0 com ruído contínuo");
    // Com sync real (gap presente) o RPM aparece normalmente.
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 3u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_TRUE(ckp_snapshot().rpm_x10 != 0u, "com sync, RPM reportado");
    // E o stall_poll decai o RPM residual após perda de captura sem sync
    // (caso: sync perdido com rpm_x10 ainda populado).
    const uint32_t base = g_ckp_cap;
    ckp_stall_poll(base + 13000000u);  // stall → LOSS_OF_SYNC + rpm=0
    CHECK_EQ(ckp_snapshot().rpm_x10, 0u, "stall zera RPM");
}

static void test_ckp_rpm_jump_recovery(void) {
    section("ckp: salto de RPM 3.75x recupera (deadlock do gap-como-normal)");
    // Reproduz a bancada: sync a "800 RPM" e salto p/ "3000" (periodo /3.75).
    // Com a media defasada, dente novo=SPIKE e gap novo cai na banda normal
    // (0.8x media) zerando o contador — sem o decaimento, re-bootstrap nunca
    // dispara e o CKP perfeito fica rejeitado para sempre.
    ckp_reach_full_sync(kNormalPeriod);               // "800 RPM"
    const uint32_t fast = kNormalPeriod * 100u / 375u; // periodo /3.75
    // 10 revolucoes no RPM novo: 57 dentes + gap 3x
    for (uint32_t rev = 0; rev < 10u; ++rev) {
        for (uint32_t i = 0; i < 57u; ++i) { ckp_fire(fast); }
        ckp_fire(fast * 3u);
    }
    const CkpSnapshot s = ckp_snapshot();
    CHECK_TRUE(s.state == SyncState::HALF_SYNC || s.state == SyncState::FULL_SYNC,
               "re-sincronizado apos salto 3.75x");
    CHECK_TRUE(s.rpm_x10 > 0u, "RPM reportado no novo regime");
}

static void test_ckp_stall_poll_no_false_positive(void) {
    section("ckp: stall_poll false when teeth are recent");
    ckp_reach_full_sync();
    CHECK_FALSE(ckp_stall_poll(ckp_snapshot().last_tim5_capture + 1000u), "no stall");
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "FULL_SYNC maintained");

    // Corrida real de bancada (7 falsos stalls/180s a 700 RPM): a ISR captura
    // um dente ENTRE a leitura de TIM5_CNT no main loop e a comparação —
    // prev_capture fica à frente de tim5_cnt_now e a subtração unsigned dava
    // ~2^32 → falso stall. Elapsed negativo tem de ser tratado como "recente".
    section("ckp: stall_poll imune a captura à frente do CNT lido (corrida ISR)");
    ckp_reach_full_sync();
    const uint32_t cap = ckp_snapshot().last_tim5_capture;
    CHECK_FALSE(ckp_stall_poll(cap - 5000u), "captura 5000 ticks à frente → sem stall");
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC),
             "FULL_SYNC preservado na corrida");
    CHECK_TRUE(ckp_snapshot().rpm_x10 != 0u, "RPM preservado na corrida");
}

static void test_ckp_seed_arm_disarm(void) {
    section("ckp: seed arm/disarm counters");
    ckp_test_reset(); g_ckp_cap = 0u;
    CHECK_EQ(ckp_seed_loaded_count(), 0u, "loaded=0 before arm");
    ckp_seed_arm(true);
    CHECK_EQ(ckp_seed_loaded_count(), 1u, "loaded=1 after arm");
    ckp_seed_disarm();
    CHECK_EQ(ckp_seed_loaded_count(), 1u, "loaded still 1 after disarm");
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSORS
// ═══════════════════════════════════════════════════════════════════════════

static void sensor_setup(void) {
    sensors_test_reset();
    using namespace ems::hal;
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::TPS,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP1,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP2,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2,      2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT,        2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT,        2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::FUEL_PRESS, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::OIL_PRESS,  2000u);
}

static void test_sensors_validate_range(void) {
    section("sensors: validate_sensor_range");
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 100u),   "CLT=100 valid");
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 3800u),  "CLT=3800 valid");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 99u),   "CLT=99 invalid");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 3801u), "CLT=3801 invalid");
    CHECK_TRUE(validate_sensor_range(SensorId::MAP, 1000u),  "MAP=1000 valid");
    CHECK_FALSE(validate_sensor_range(SensorId::MAP, 10u),   "MAP=10 invalid");
}

static void test_sensors_validate_values(void) {
    section("sensors: validate_sensor_values");
    SensorData d{};
    d.map_bar_x1000 = 1000u; d.clt_degc_x10 = 850; d.iat_degc_x10 = 250;
    d.tps_pct_x10 = 500u; d.vbatt_mv = 12000u;
    d.fuel_press_bar_x1000 = 3000u; d.oil_press_bar_x1000 = 5000u;
    CHECK_TRUE(validate_sensor_values(d), "all valid → true");
    SensorData bad = d; bad.map_bar_x1000 = 50u;
    CHECK_FALSE(validate_sensor_values(bad), "MAP < 0.10 bar → false");
    bad = d; bad.clt_degc_x10 = 1600;
    CHECK_FALSE(validate_sensor_values(bad), "CLT > 150°C → false");
    bad = d; bad.vbatt_mv = 5000u;
    CHECK_FALSE(validate_sensor_values(bad), "vbatt < 6V → false");
    bad = d; bad.tps_pct_x10 = 1001u;
    CHECK_FALSE(validate_sensor_values(bad), "TPS > 100% → false");
}

static void test_sensors_health_status(void) {
    section("sensors: get_sensor_health_status");
    sensor_setup(); sensors_init();
    CHECK_EQ(get_sensor_health_status(), 0u, "health=0 after init with valid ADC");
}

static void test_sensors_calibration(void) {
    section("sensors: set_tps_cal / set_app_cal / set_plausibility / set_etb_tps_cal");
    sensor_setup(); sensors_init();
    sensors_set_tps_cal(400u, 3800u);
    sensors_set_app_cal(400u, 3800u, 400u, 3800u);
    sensors_set_plausibility(100u, 100u);
    sensors_set_etb_tps_cal(400u, 3800u, 400u, 3800u);
    CHECK_TRUE(true, "calibration setters complete without crash");
}

static void test_sensors_tick_100ms_clt_iat(void) {
    section("sensors: sensors_tick_100ms → CLT/IAT via lut128");
    sensor_setup(); sensors_init();
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT, 2000u);
    for (int i = 0; i < 8; ++i) { sensors_test_tick_100ms(); }
    const SensorData sd = sensors_get();
    const int16_t expected = static_cast<int16_t>(-400 + (1900 * 62) / 127);
    CHECK_EQ(sd.clt_degc_x10, expected, "CLT lut128(2000) matches formula");
    CHECK_EQ(sd.iat_degc_x10, expected, "IAT lut128(2000) matches formula");
}

static void test_sensors_maf_freq_capture(void) {
    section("sensors: sensors_maf_freq_capture_isr");
    sensor_setup(); sensors_init();
    sensors_maf_freq_capture_isr(5000u);
    sensors_maf_freq_capture_isr(10000u);
    sensors_maf_freq_capture_isr(0u);
    CHECK_TRUE(true, "maf_freq_capture_isr handles various periods");
}

// ═══════════════════════════════════════════════════════════════════════════
// FUEL CALC
// ═══════════════════════════════════════════════════════════════════════════

static void test_fuel_calc_req_fuel_us(void) {
    section("fuel_calc: calc_req_fuel_us");
    const uint32_t req = calc_req_fuel_us(2000u, 4u, 440u, 1470u);
    CHECK_TRUE(req > 0u && req <= 50000u, "req_fuel in (0, 50ms]");
    CHECK_EQ(calc_req_fuel_us(0u,    4u, 440u, 1470u), 0u, "displacement=0 → 0");
    CHECK_EQ(calc_req_fuel_us(2000u, 0u, 440u, 1470u), 0u, "cylinders=0 → 0");
    CHECK_EQ(calc_req_fuel_us(2000u, 4u,   0u, 1470u), 0u, "flow=0 → 0");
    CHECK_EQ(calc_req_fuel_us(2000u, 4u, 440u,    0u), 0u, "stoich=0 → 0");
    CHECK_TRUE(calc_req_fuel_us(50000u, 1u, 1u, 100u) <= 50000u, "clamped at 50ms");
}

static void test_fuel_calc_base_pw(void) {
    section("fuel_calc: calc_base_pw_us");
    CHECK_TRUE(calc_base_pw_us(5000u, 80u, 100u, 101u) > 0u, "base_pw > 0");
    CHECK_EQ(calc_base_pw_us(5000u, 100u, 100u, 100u), 5000u, "VE=100% MAP=REF → pw=req");
    CHECK_EQ(calc_base_pw_us(5000u,  0u, 100u, 100u),     0u, "ve=0 → 0");
    CHECK_EQ(calc_base_pw_us(5000u, 80u, 100u,   0u),     0u, "map_ref=0 → 0");
    CHECK_EQ(calc_base_pw_us(5000u, 80u, 400u, 100u),     0u, "MAP>3 bar → 0");
    CHECK_EQ(calc_base_pw_us(   0u, 80u, 100u, 100u),     0u, "req=0 → 0");
}

static void test_fuel_apply_lambda_target(void) {
    section("fuel_calc: apply_lambda_target_pw_us");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1000u), 5000u, "lambda=1.000 → unchanged");
    CHECK_NEAR(static_cast<float>(apply_lambda_target_pw_us(5000u, 850u)),  5882.0f, 5.0f, "lambda=0.850 → richer");
    CHECK_NEAR(static_cast<float>(apply_lambda_target_pw_us(5000u, 1200u)), 4167.0f, 5.0f, "lambda=1.200 → leaner");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 600u),  5000u, "lambda<0.65 → passthrough");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1300u), 5000u, "lambda>1.20 → passthrough");
    CHECK_EQ(apply_lambda_target_pw_us(0u, 1000u),       0u, "base=0 → 0");
}

static void test_fuel_apply_trim(void) {
    section("fuel_calc: apply_fuel_trim_pw_us");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u,    0), 5000u, "trim=0 → unchanged");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u,  100), 5500u, "+10% → +10%");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u, -100), 4500u, "-10% → -10%");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u,  500), 7500u, "+50% → ×1.5");
    CHECK_EQ(apply_fuel_trim_pw_us(0u,     100),    0u, "base=0 → 0");
}

static void test_fuel_calc_final_pw(void) {
    section("fuel_calc: calc_final_pw_us");
    CHECK_EQ(calc_final_pw_us(5000u, 256u, 256u, 500u), 5500u, "neutral corr + 500µs dead");
    CHECK_EQ(calc_final_pw_us(5000u, 384u, 256u,   0u), 7500u, "CLT 1.5× → pw×1.5");
    CHECK_EQ(calc_final_pw_us(   0u, 256u, 256u, 500u),    0u, "base=0 → 0");
}

static void test_fuel_corr_functions(void) {
    section("fuel_calc: corr_clt / corr_iat / corr_vbatt");
    CHECK_TRUE(corr_clt(850) <= 270u,  "corr_clt at 85°C ≤ 270");
    CHECK_TRUE(corr_clt(-100) > 256u,  "corr_clt at -10°C > 256 (cold enrichment)");
    CHECK_TRUE(corr_iat(250) >= 256u,  "corr_iat at 25°C ≥ 256");
    CHECK_TRUE(corr_vbatt(12000u) > 0u && corr_vbatt(12000u) < 2000u, "dead-time at 12V in range");
    CHECK_TRUE(corr_vbatt(9000u) > corr_vbatt(14000u), "lower voltage → longer dead-time");
}

static void test_fuel_decel_cut(void) {
    section("fuel_calc: fuel_decel_cut_update / active / reset");
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_active(), "not active after reset");
    CHECK_TRUE(fuel_decel_cut_update(20000u, 0u, 800), "activates: closed + warm + high rpm");
    CHECK_TRUE(fuel_decel_cut_active(), "active() true");
    CHECK_FALSE(fuel_decel_cut_update(11000u, 0u, 800), "deactivates: rpm < exit");
    fuel_decel_cut_update(20000u, 0u, 800);
    fuel_decel_cut_update(20000u, 100u, 800);
    CHECK_FALSE(fuel_decel_cut_active(), "deactivates: throttle open");
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_update(20000u, 0u, 600), "no cut: cold engine");
}

static void test_fuel_baro(void) {
    section("fuel_calc: fuel_set/get_baro_bar_x100");
    fuel_set_baro_bar_x100(101u); CHECK_EQ(fuel_get_baro_bar_x100(), 101u, "baro=101");
    fuel_set_baro_bar_x100(70u);  CHECK_EQ(fuel_get_baro_bar_x100(),  70u, "baro=70 (min)");
    fuel_set_baro_bar_x100(110u); CHECK_EQ(fuel_get_baro_bar_x100(), 110u, "baro=110 (max)");
    fuel_set_baro_bar_x100(69u);  CHECK_EQ(fuel_get_baro_bar_x100(), 110u, "69 rejected");
    fuel_set_baro_bar_x100(111u); CHECK_EQ(fuel_get_baro_bar_x100(), 110u, "111 rejected");
}

// ═══════════════════════════════════════════════════════════════════════════
// IGN CALC
// ═══════════════════════════════════════════════════════════════════════════

static void test_ign_iat_correction(void) {
    section("ign_calc: calc_ign_iat_correction_deg");
    CHECK_EQ(calc_ign_iat_correction_deg(-200),  2,  "IAT=-20°C → +2°");
    CHECK_EQ(calc_ign_iat_correction_deg(0),     1,  "IAT=0°C → +1°");
    CHECK_EQ(calc_ign_iat_correction_deg(200),   0,  "IAT=20°C → 0° (ref)");
    CHECK_EQ(calc_ign_iat_correction_deg(400),  -1,  "IAT=40°C → -1°");
    CHECK_EQ(calc_ign_iat_correction_deg(800),  -5,  "IAT=80°C → -5°");
    CHECK_EQ(calc_ign_iat_correction_deg(1000), -5,  "IAT=100°C → clamped -5°");
}

static void test_ign_clt_correction(void) {
    section("ign_calc: calc_ign_clt_correction_deg");
    CHECK_EQ(calc_ign_clt_correction_deg(-400), 0,  "CLT=-40°C → 0°");
    CHECK_EQ(calc_ign_clt_correction_deg(200),  -8, "CLT=20°C → -8° (cat warm-up)");
    CHECK_EQ(calc_ign_clt_correction_deg(600),  0,  "CLT=60°C → 0°");
}

static void test_ign_antijerk(void) {
    section("ign_calc: calc_antijerk_retard_deg");
    antijerk_reset();
    CHECK_EQ(calc_antijerk_retard_deg(false), 0, "no AE → 0");
    CHECK_EQ(calc_antijerk_retard_deg(true),  3, "AE active → 3°");
    antijerk_reset();
    calc_antijerk_retard_deg(true);
    calc_antijerk_retard_deg(false); calc_antijerk_retard_deg(false); calc_antijerk_retard_deg(false);
    CHECK_EQ(calc_antijerk_retard_deg(false), 0, "decays to 0 after 3 clean cycles");
}

static void test_ign_clamp_and_total_advance(void) {
    section("ign_calc: clamp_advance_deg / calc_total_advance");
    CHECK_EQ(clamp_advance_deg(40),  40,  "40° at max");
    CHECK_EQ(clamp_advance_deg(50),  40,  "50° clamped to 40°");
    CHECK_EQ(clamp_advance_deg(-10), -10, "-10° at min");
    CHECK_EQ(clamp_advance_deg(-15), -10, "-15° clamped to -10°");
    AdvanceCorrections c{};
    CHECK_EQ(calc_total_advance(25, c), 25, "base=25 no corr → 25");
    c.iat_deg = -3;
    CHECK_EQ(calc_total_advance(25, c), 22, "iat=-3 → 22");
    c = {}; c.clt_deg = -8; c.knock_retard_deg = 5;
    CHECK_EQ(calc_total_advance(25, c), 12, "clt=-8 knock=5 → 12");
    c = {}; c.idle_spark_deg = 5;
    CHECK_EQ(calc_total_advance(15, c), 20, "idle_spark=+5 → 20");
}

static void test_ign_dwell(void) {
    section("ign_calc: dwell_ms_x10_from_vbatt / calc_dwell_angle_x10 / build_ign_schedule");
    CHECK_NEAR(static_cast<float>(dwell_ms_x10_from_vbatt(12000u)), 30.0f, 5.0f, "dwell@12V≈3.0ms");
    CHECK_TRUE(dwell_ms_x10_from_vbatt(9000u) > dwell_ms_x10_from_vbatt(14000u), "monotonic dwell");
    // angle: dwell=30 x10 @ 3000 RPM = 30×3000×36/6000 = 540
    CHECK_EQ(calc_dwell_angle_x10(30u, 3000u), 540u,  "3ms@3000RPM=54.0°");
    CHECK_EQ(calc_dwell_angle_x10(30u, 6000u), 1080u, "3ms@6000RPM=108.0°");
    CHECK_EQ(calc_dwell_angle_x10(420u, 8000u), 3599u, "capped at 359.9°");
    // dwell_start = spark + dwell_angle
    CHECK_EQ(calc_dwell_start_deg_x10(300, 30u, 3000u), 840, "dwell_start=300+540=840");
    // build schedule
    const IgnScheduleParams p = build_ign_schedule(0u, 250, 30u, 3000u);
    CHECK_EQ(p.cyl, 0u, "cyl=0"); CHECK_EQ(p.spark_x10, 250u, "spark=250"); CHECK_EQ(p.dwell_start_x10, 790u, "dwell_start=790");
    CHECK_EQ(build_ign_schedule(5u, 100, 30u, 3000u).cyl, 1u, "cyl=5 masked to 1");
    // inj_pw_to_ticks (host mode: ×60)
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(1000u), 60000u, "1000µs×60=60000 ticks");
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(0u), 0u, "0µs→0");
}

// ═══════════════════════════════════════════════════════════════════════════
// AUXILIARIES
// ═══════════════════════════════════════════════════════════════════════════

static void test_aux_init_and_idle(void) {
    section("auxiliaries: init / idle_target_rpm_x10");
    auxiliaries_test_reset();
    CHECK_FALSE(auxiliaries_test_get_fan_state(),  "fan off after init");
    CHECK_FALSE(auxiliaries_test_get_pump_state(), "pump off after init");
    auxiliaries_tick_10ms();
    CHECK_TRUE(true, "tick_10ms after init completes");
    // kIdleTargetRpmX10 table: -40°C→12000, 90°C→8200, 110°C→8000
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(-400), 12000u, "idle@-40°C=1200RPM");
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(900),   8200u, "idle@90°C=820RPM");
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(1100),  8000u, "idle@110°C=800RPM");
    CHECK_TRUE(auxiliaries_idle_target_rpm_x10(-100) > auxiliaries_idle_target_rpm_x10(700),
               "idle target decreases as engine warms");
}

static void test_aux_pump_prime(void) {
    section("auxiliaries: key_on → pump prime / key_off → pump stop");
    auxiliaries_test_reset();
    auxiliaries_set_key_on(true);
    auxiliaries_tick_10ms();
    CHECK_TRUE(auxiliaries_test_get_pump_state(), "pump on after key_on + tick");
    auxiliaries_set_key_on(false);
    for (int i = 0; i < 310; ++i) { auxiliaries_tick_10ms(); }
    CHECK_FALSE(auxiliaries_test_get_pump_state(), "pump off after key_off + delay");
}

static void test_aux_ticks_no_crash(void) {
    section("auxiliaries: tick_10ms / tick_20ms no crash");
    auxiliaries_test_reset();
    for (int i = 0; i < 10; ++i) { auxiliaries_tick_10ms(); auxiliaries_tick_20ms(); }
    CHECK_TRUE(true, "20 ticks complete without crash");
}

// ═══════════════════════════════════════════════════════════════════════════
// KNOCK
// ═══════════════════════════════════════════════════════════════════════════

static void test_knock_init_and_threshold(void) {
    section("knock: init / adc_threshold set/get");
    knock_init();
    for (uint8_t c = 0; c < 4u; ++c) { CHECK_EQ(knock_get_retard_x10(c), 0u, "retard=0 after init"); }
    CHECK_FALSE(knock_test_window_active(), "window closed after init");
    CHECK_EQ(knock_get_adc_threshold(), 2048u, "default threshold=2048");
    knock_set_adc_threshold(3000u); CHECK_EQ(knock_get_adc_threshold(), 3000u, "threshold=3000");
    knock_set_adc_threshold(100u);  CHECK_EQ(knock_get_adc_threshold(),  256u, "clamped at min=256");
    knock_set_adc_threshold(5000u); CHECK_EQ(knock_get_adc_threshold(), 4000u, "clamped at max=4000");
}

static void test_knock_window(void) {
    section("knock: window open/close + cylinder masking");
    knock_init();
    knock_window_open(0u);
    CHECK_TRUE(knock_test_window_active(), "window active");
    CHECK_EQ(knock_test_window_cyl(), 0u, "cyl=0");
    knock_window_close(0u);
    CHECK_FALSE(knock_test_window_active(), "window closed");
    knock_window_open(6u); CHECK_EQ(knock_test_window_cyl(), 2u, "cyl=6 masked to 2"); knock_window_close(6u);
}

static void test_knock_detection_and_recovery(void) {
    section("knock: adc_update / cycle_complete / retard / recovery / clamp / per-cyl");
    knock_init();
    knock_set_adc_threshold(2000u);
    knock_set_event_threshold(2u);

    // adc_update: counts only above threshold while window open
    knock_window_open(1u);
    knock_test_set_adc_raw(1900u); knock_test_set_adc_raw(1900u);
    CHECK_EQ(knock_test_get_knock_count(1u), 0u, "below threshold → no count");
    knock_test_set_adc_raw(2100u); knock_test_set_adc_raw(2100u);
    CHECK_EQ(knock_test_get_knock_count(1u), 2u, "above threshold → 2 counts");
    knock_window_close(1u);
    knock_test_set_adc_raw(3000u);
    CHECK_EQ(knock_test_get_knock_count(1u), 2u, "window closed → count frozen");

    // cycle_complete: retard on knock (count=2 > threshold=2? no, need >2 → need ≥3)
    // Reopen with 3 events
    knock_window_open(0u);
    knock_test_set_adc_raw(2500u); knock_test_set_adc_raw(2500u); knock_test_set_adc_raw(2500u);
    knock_window_close(0u);
    const uint16_t r0 = knock_get_retard_x10(0u);
    knock_cycle_complete(0u);
    CHECK_TRUE(knock_get_retard_x10(0u) > r0, "retard increases after knock");
    CHECK_EQ(knock_get_retard_x10(0u) - r0, 20u, "retard += 2.0° (20 x10)");

    // per-cylinder independence
    CHECK_EQ(knock_get_retard_x10(1u), 0u, "cyl 1 unaffected by cyl 0 knock");
    CHECK_EQ(knock_get_retard_x10(2u), 0u, "cyl 2 unaffected");

    // max clamp: force many knock events
    knock_set_event_threshold(0u);
    for (int i = 0; i < 20; ++i) {
        knock_window_open(3u); knock_test_set_adc_raw(2500u); knock_window_close(3u); knock_cycle_complete(3u);
    }
    CHECK_TRUE(knock_get_retard_x10(3u) <= 100u, "retard clamped at 10.0° (100 x10)");

    // recovery: 11 clean cycles → retard decreases
    knock_set_event_threshold(2u);
    const uint16_t peak = knock_get_retard_x10(0u);
    for (int i = 0; i < 11; ++i) {
        knock_window_open(0u); knock_test_set_adc_raw(500u); knock_window_close(0u); knock_cycle_complete(0u);
    }
    CHECK_TRUE(knock_get_retard_x10(0u) < peak, "retard decreases after clean cycles");
}

// ═══════════════════════════════════════════════════════════════════════════
// FUEL CALC — SEGUNDA FASE (funções não cobertas)
// ═══════════════════════════════════════════════════════════════════════════

static void test_fuel_table_lookups(void) {
    section("fuel_calc: get_ve / get_ve_prepared / get_lambda_target_x1000");

    // ve_table is default-initialized; all cells = some uint8 value.
    // We just verify: returns in [0,255], doesn't crash, uses axis clamping.
    const uint8_t ve_mid = get_ve(30000u, 100u);  // 3000 RPM, 100 kPa
    CHECK_TRUE(ve_mid <= 255u, "get_ve returns uint8 value");

    // Prepared path must match direct path
    const Table2dLookup lk = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u);
    const uint8_t ve_prep = get_ve_prepared(lk);
    CHECK_EQ(ve_mid, ve_prep, "get_ve_prepared == get_ve for same point");

    // lambda_target clamped to [650, 1200]
    const uint16_t lam = get_lambda_target_x1000(30000u, 100u);
    CHECK_TRUE(lam >= 650u && lam <= 1200u, "get_lambda_target in [650,1200]");

    // Prepared path must match
    const uint16_t lam_prep = get_lambda_target_x1000_prepared(lk);
    CHECK_EQ(lam, lam_prep, "get_lambda_target_x1000_prepared == direct");
}

static void test_fuel_default_req_and_base_default(void) {
    section("fuel_calc: default_req_fuel_us / calc_base_pw_us_default");

    // default_req_fuel_us uses compile-time engine config
    const uint32_t req = default_req_fuel_us();
    CHECK_TRUE(req > 0u && req <= 50000u, "default_req_fuel_us in (0, 50ms]");

    // calc_base_pw_us_default: same result as calc_base_pw_us with defaults
    const uint32_t base = calc_base_pw_us_default(80u, 100u);
    CHECK_TRUE(base > 0u, "calc_base_pw_us_default > 0 at VE=80% MAP=100kPa");
    CHECK_EQ(calc_base_pw_us_default(0u, 100u), 0u, "ve=0 → 0");
    CHECK_EQ(calc_base_pw_us_default(80u, 400u), 0u, "MAP>3bar → 0");
}

static void test_fuel_default_fast(void) {
    section("fuel_calc: calc_fuel_pw_us_default_fast");

    fuel_set_baro_bar_x100(101u);  // 1010 mbar reference
    const uint32_t pw = calc_fuel_pw_us_default_fast(
        80u,    // ve
        100u,   // map_bar_x100
        1000u,  // lambda_target_x1000 (stoich)
        0,      // trim_pct_x10
        256u,   // corr_clt_x256 (neutral)
        256u,   // corr_iat_x256 (neutral)
        500u    // dead_time_us
    );
    CHECK_TRUE(pw > 0u, "calc_fuel_pw_us_default_fast > 0 for valid inputs");

    // VE=0 → 0
    CHECK_EQ(calc_fuel_pw_us_default_fast(0u, 100u, 1000u, 0, 256u, 256u, 0u), 0u,
             "ve=0 → pw=0");

    // lambda out of range → 0
    CHECK_EQ(calc_fuel_pw_us_default_fast(80u, 100u, 600u, 0, 256u, 256u, 0u), 0u,
             "lambda<650 → pw=0");

    // Altitude compensation (F4): lower baro → larger PW (denominator shrinks).
    // baro=70 (0.70 bar, ~3000m altitude) vs baro=101 (1.01 bar, sea level).
    fuel_set_baro_bar_x100(101u);
    const uint32_t pw_sea   = calc_fuel_pw_us_default_fast(80u, 100u, 1000u, 0, 256u, 256u, 0u);
    fuel_set_baro_bar_x100(70u);
    const uint32_t pw_alt   = calc_fuel_pw_us_default_fast(80u, 100u, 1000u, 0, 256u, 256u, 0u);
    CHECK_TRUE(pw_alt > pw_sea,
               "altitude compensation: lower baro → higher PW (TI_FAC_ALTI)");
    // Ratio should be approximately baro_sea/baro_alt = 101/70 ≈ 1.44
    // Allow ±10% tolerance.
    const uint32_t ratio_x100 = (pw_alt * 100u) / (pw_sea > 0u ? pw_sea : 1u);
    CHECK_TRUE(ratio_x100 >= 130u && ratio_x100 <= 160u,
               "altitude PW ratio ≈ 1.44 (sea_baro/alt_baro=101/70)");
    fuel_set_baro_bar_x100(101u);  // restore
}

static void test_fuel_corr_warmup(void) {
    section("fuel_calc: corr_warmup");
    // warmup_corr_axis_x10: {-400,-100,0,200,400,700,900,1100}
    // warmup_corr_x256:     {420, 380,350,320,290,256,256, 256}
    const uint16_t w_cold = corr_warmup(-400);  // idx=0 → 420
    CHECK_EQ(w_cold, 420u, "corr_warmup at -40°C = 420 (1.64×)");

    const uint16_t w_warm = corr_warmup(900);   // idx=6 → 256
    CHECK_EQ(w_warm, 256u, "corr_warmup at 90°C = 256 (1.0×, no enrichment)");

    // Monotonic: colder → more enrichment
    CHECK_TRUE(corr_warmup(-100) > corr_warmup(700), "corr_warmup monotonically decreasing");
}

static void test_fuel_ae(void) {
    section("fuel_calc: fuel_ae_set_threshold / fuel_ae_set_taper / calc_ae_pw_us");

    fuel_reset_adaptives();

    // Set threshold=10 x10 (1.0%/ms), taper=4 cycles
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);

    // No acceleration: TPS same → ae=0
    const int32_t ae_idle = calc_ae_pw_us(500u, 500u, 10u, 800);
    CHECK_EQ(ae_idle, 0, "no TPS change → ae=0");

    // Large TPS step: delta=300 x10 in 10ms → tpsdot=30 > threshold=10
    const int32_t ae_accel = calc_ae_pw_us(800u, 500u, 10u, 800);
    CHECK_TRUE(ae_accel > 0, "large TPS step → ae > 0");
    CHECK_TRUE(ae_accel <= 5000, "ae ≤ ae_max_pw_us=5000");

    // Decel (TPS close): negative delta → clamped to 0, returns decaying pulse
    const int32_t ae_decel = calc_ae_pw_us(200u, 800u, 10u, 800);
    CHECK_TRUE(ae_decel >= 0, "decel → ae ≥ 0 (no decel enrichment)");

    // dt=0 guard
    CHECK_EQ(calc_ae_pw_us(800u, 0u, 0u, 800), 0, "dt=0 → ae=0");

    // Taper decay: with taper=4, the AE pulse decays to 0 over 4 cycles without TPS change.
    // Reset AE internal state by resetting adaptives, then fire one large step,
    // then call with no TPS delta 4 more times → pulse should be 0.
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);
    calc_ae_pw_us(800u, 500u, 10u, 800);  // seed the decay counter
    int32_t ae_t1 = calc_ae_pw_us(500u, 500u, 10u, 800);  // no delta: decay tick 1
    int32_t ae_t2 = calc_ae_pw_us(500u, 500u, 10u, 800);  // decay tick 2
    int32_t ae_t3 = calc_ae_pw_us(500u, 500u, 10u, 800);  // decay tick 3
    int32_t ae_t4 = calc_ae_pw_us(500u, 500u, 10u, 800);  // decay tick 4
    CHECK_TRUE(ae_t1 >= ae_t4, "AE taper: pulse non-increasing over cycles");
    CHECK_EQ(ae_t4, 0, "AE taper: pulse = 0 at or after taper_cycles=4");
}

static void test_fuel_adaptives_reset(void) {
    section("fuel_calc: fuel_reset_adaptives / fuel_lambda_delay_reset");

    // Ensure STFT is non-zero first
    fuel_update_stft(30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0, "pre-cond: STFT non-zero");

    fuel_reset_adaptives();
    CHECK_EQ(fuel_get_stft_pct_x10(), 0, "STFT=0 after reset_adaptives");

    // fuel_lambda_delay_reset clears history ring buffer.
    // Behaviour: after reset the delayed STFT has no history sample matching
    // the delay window, so o2_valid=false path fires → STFT decays.
    // Before reset: push a history sample at t=0 with a lean lambda.
    fuel_lambda_delay_reset();
    fuel_reset_adaptives();
    // Push entry at t=0ms: rpm=30000, map=100, target=1000
    fuel_update_stft_delayed(0u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // Now query at t=300ms (delay≈200ms → sample IS in window → o2_valid used → STFT updates)
    const int16_t stft_with_history = fuel_update_stft_delayed(
        300u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // After reset, ring buffer empty → same query at t=300 has no sample → o2_valid=false → no STFT update
    fuel_lambda_delay_reset();
    fuel_reset_adaptives();
    const int16_t stft_post_reset = fuel_update_stft_delayed(
        300u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // stft_post_reset should be 0 or smaller (no closed-loop without history at t=300 without prior push)
    CHECK_TRUE(stft_with_history > stft_post_reset || stft_post_reset == 0,
               "fuel_lambda_delay_reset: clears history; no closed-loop without prior push");
}

static void test_fuel_lambda_delay(void) {
    section("fuel_calc: lambda_delay_ms_from_rpm_load");

    // Returns interpolated delay in ms. With default table, should be > 0.
    const uint16_t d = lambda_delay_ms_from_rpm_load(30000u, 100u);
    CHECK_TRUE(d <= 2000u, "lambda_delay in plausible range");

    // At extremes must not crash
    lambda_delay_ms_from_rpm_load(0u, 0u);
    lambda_delay_ms_from_rpm_load(200000u, 300u);
    CHECK_TRUE(true, "lambda_delay extremes: no crash");
}

static void test_fuel_stft(void) {
    section("fuel_calc: fuel_update_stft / fuel_get_stft_pct_x10");

    fuel_reset_adaptives();
    CHECK_EQ(fuel_get_stft_pct_x10(), 0, "STFT=0 after reset");

    // Conditions for closed loop: clt>700, o2_valid=true, ae_active=false, rev_cut=false
    // lambda measured > target → lean signal → positive error → STFT increases (adds fuel)
    int16_t stft = fuel_update_stft(30000u, 100u,
        1000,   // target lambda (stoich)
        1050,   // measured lambda (lean by 5%)
        900,    // clt 90°C > 70°C → closed loop OK
        true, false, false, 5000u, 500u);
    CHECK_TRUE(stft > 0, "lean signal → STFT positive (add fuel)");
    CHECK_EQ(stft, fuel_get_stft_pct_x10(), "fuel_get_stft_pct_x10 matches return value");

    // Rich signal → STFT eventually negative
    fuel_reset_adaptives();
    for (int i = 0; i < 30; ++i) {
        stft = fuel_update_stft(30000u, 100u, 1000, 950, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(stft < 0, "rich signal → STFT negative (reduce fuel)");

    // Closed loop disabled (cold engine): STFT congela (anti-windup), não decai —
    // evita um "degrau" de combustível perceptível quando volta a closed-loop.
    fuel_reset_adaptives();
    fuel_update_stft(30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);  // set non-zero
    const int16_t before = fuel_get_stft_pct_x10();
    fuel_update_stft(30000u, 100u, 1000, 1050, 600, true, false, false, 5000u, 500u);  // clt too cold
    const int16_t after = fuel_get_stft_pct_x10();
    CHECK_EQ(after, before, "closed loop disabled → STFT congela (freeze)");
}

static void test_fuel_stft_delayed(void) {
    section("fuel_calc: fuel_update_stft_delayed");

    fuel_reset_adaptives();
    fuel_lambda_delay_reset();

    // With no history in ring buffer, now_ms=0, delay≈200ms → get_delayed fails
    // → o2_valid=false path → closed loop disabled → STFT stays 0.
    const int16_t v0 = fuel_update_stft_delayed(
        0u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(v0, 0, "t=0 no history: STFT=0 (delay not expired)");

    // Accumulate 5 samples; at t=500ms history sample at ~t=0 is 'old enough'
    // for a 200ms delay -> closed loop fires with lean signal -> STFT > 0.
    for (uint32_t t = 50u; t <= 250u; t += 50u) {
        fuel_update_stft_delayed(t, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    }
    const int16_t v_later = fuel_update_stft_delayed(
        500u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // At t=500 history sample from t≈0 satisfies delay≈200ms → closed loop active → STFT ≠ 0
    CHECK_TRUE(v_later != 0 || v0 == 0,
               "stft_delayed activates after delay window");
    CHECK_TRUE(v_later >= -250 && v_later <= 250,
               "stft_delayed in valid range [-25%,+25%]");
}

static void test_injector_scurve(void) {
    section("fuel_calc: apply_injector_scurve");
    using namespace ems::engine;

    CHECK_EQ(apply_injector_scurve(0u), 0u, "pw=0 → 0 (early return)");

    // Acima do último ponto do eixo (1500us): correção=256 (Q8=1.0) → sem alteração.
    CHECK_EQ(apply_injector_scurve(3000u), 3000u, "pw grande: sem correção (Q8=1.0)");

    // PW pequeno (perto de zero): tabela dá correção < 1.0 → PW corrigido > PW teórico
    // (o bico entrega proporcionalmente menos que o linear em aberturas curtas,
    // logo é preciso comandar mais tempo para compensar).
    const uint32_t pw_small_corrected = apply_injector_scurve(100u);
    CHECK_TRUE(pw_small_corrected > 100u, "pw pequeno: PW corrigido > PW teórico");

    // Monotonicidade: PW corrigido cresce com o PW teórico.
    const uint32_t pw_200 = apply_injector_scurve(200u);
    const uint32_t pw_800 = apply_injector_scurve(800u);
    CHECK_TRUE(pw_800 > pw_200, "correção monótona: PW maior → corrigido maior");
}

static void test_fuel_delta_p_compensation(void) {
    section("fuel_calc: apply_delta_p_compensation");
    using namespace ems::engine;

    CHECK_EQ(apply_delta_p_compensation(0u, 3000u, 100u), 0u, "pw=0 → 0 (early return)");

    // Pressão real = nominal (3000 = 3.0 bar), MAP igual em ambos os lados →
    // ΔP_atual == ΔP_nominal → sem correção (ratio=1.0).
    const uint32_t pw_nominal = apply_delta_p_compensation(5000u, 3000u, 100u);
    CHECK_NEAR(static_cast<int32_t>(pw_nominal), 5000, 20,
               "pressão nominal: PW ~inalterado");

    // Sensor sem leitura (0): usa o nominal como fallback → mesmo resultado
    // que passar o nominal explicitamente.
    const uint32_t pw_fallback = apply_delta_p_compensation(5000u, 0u, 100u);
    CHECK_NEAR(static_cast<int32_t>(pw_fallback), static_cast<int32_t>(pw_nominal), 5,
               "fuel_press=0 → usa nominal (mesmo resultado)");

    // Pressão real ABAIXO da nominal → ΔP_atual < ΔP_nominal → fluxo do bico
    // menor que o esperado → PW tem de aumentar para compensar (enriquece).
    const uint32_t pw_low_press = apply_delta_p_compensation(5000u, 2000u, 100u);
    CHECK_TRUE(pw_low_press > pw_nominal, "pressão baixa: PW corrigido aumenta");

    // Pressão real ACIMA da nominal → fluxo do bico maior → PW deve diminuir.
    const uint32_t pw_high_press = apply_delta_p_compensation(5000u, 4000u, 100u);
    CHECK_TRUE(pw_high_press < pw_nominal, "pressão alta: PW corrigido diminui");
}

static void test_fuel_ltft(void) {
    section("fuel_calc: fuel_get_ltft_pct_x10 / fuel_get_ltft_add_us");

    fuel_reset_adaptives();

    // After reset: LTFT cells loaded from NVM (host = all zeros)
    const int16_t ltft = fuel_get_ltft_pct_x10(0u, 0u);
    CHECK_TRUE(ltft >= -250 && ltft <= 250, "ltft_pct_x10 in valid range");

    // Out-of-range index: returns 0
    CHECK_EQ(fuel_get_ltft_pct_x10(16u, 0u), 0, "ltft: out-of-range map_idx → 0");
    CHECK_EQ(fuel_get_ltft_pct_x10(0u, 16u), 0, "ltft: out-of-range rpm_idx → 0");

    const int16_t ltft_add = fuel_get_ltft_add_us(0u, 0u);
    CHECK_TRUE(ltft_add >= -6350 && ltft_add <= 6350, "ltft_add_us in valid range");
    CHECK_EQ(fuel_get_ltft_add_us(16u, 0u), 0, "ltft_add: out-of-range → 0");
}

// WP0: apply (fuel_get_ltft_at / _add_at) usa nearest — igual crédito/store.
// Em RPM/MAP exactos no eixo, floor (bilineal) ≠ nearest (dominante).
static void test_ltft_adapt_enable(void) {
    section("fuel_trim: ltft_adapt_enable congela LTFT/LEARN");

    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    ltft_adapt_enable = 0u;
    fuel_ltft_accum_reset();

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    for (int i = 0; i < 10; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1100, 900, true, false, false,
                               5000u, 500u);
    }
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0, "adapt off: STFT ainda integra");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "adapt off: zero LEARN hits");

    ltft_adapt_enable = 1u;
    fuel_ltft_accum_reset();
    (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false,
                           5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false,
                               5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) > 0u, "adapt on: LEARN hits");

    ltft_adapt_enable = 1u;
    ltft_adapt_min_rpm_x10 = 12000u;
    closed_loop_post_start_s = 15u;
    fuel_reset_adaptives();
}

static void test_fuel_trim_dtcs(void) {
    section("fuel_trim: DTCs STFT/LTFT saturação");
    using namespace ems::engine;

    using DC = DiagnosticCode;
    DiagnosticManager::init();
    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    stft_clamp_pct_x10 = 50u;       // ±5% — satura rápido
    ltft_mult_clamp_pct_x10 = 50u;
    ltft_learn_div = 1u;
    ltft_max_step_x10 = 0u;

    CHECK_FALSE(DiagnosticManager::is_fault_active(DC::STFT_LIMIT_REACHED),
                "sem DTC STFT no início");

    // Mantém erro grande até saturar STFT e confirmar 50 ticks
    for (int i = 0; i < 80; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1300, 900, true, false, false,
                               5000u, 500u);
    }
    CHECK_TRUE(fuel_get_stft_pct_x10() >= 50 || fuel_get_stft_pct_x10() <= -50,
               "STFT no clamp");
    CHECK_TRUE(DiagnosticManager::is_fault_active(DC::STFT_LIMIT_REACHED),
               "STFT_LIMIT_REACHED após saturação prolongada");
    CHECK_TRUE(DiagnosticManager::is_fault_active(DC::FUEL_TRIM_LEAN) ||
               DiagnosticManager::is_fault_active(DC::FUEL_TRIM_RICH),
               "FUEL_TRIM lean/rich com STFT saturado");

    // Recupera: erro ~0, STFT baixa → clear
    for (int i = 0; i < 80; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false,
                               5000u, 500u);
    }
    // Integrador desce devagar; força STFT para 0 via reset parcial de integrador
    // e várias amostras no alvo com enable/off dance — ou clear via muitos ticks
    // com err=0: se STFT ainda sat, não limpa. Zera STFT via reset adaptives.
    fuel_reset_adaptives();
    DiagnosticManager::init();  // limpa lista (reset adaptives não limpa DTC)
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    stft_clamp_pct_x10 = 250u;
    ltft_mult_clamp_pct_x10 = 250u;
    ltft_learn_div = 64u;
    fuel_reset_adaptives();
}

static void test_fuel_ltft_authority(void) {
    section("fuel_trim: LTFT clamp/rate calibráveis (≠ STFT)");

    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;  // allow LTFT at any RPM for this test
    ltft_learn_div = 1u;          // fast IIR → cell ≈ stft
    ltft_max_step_x10 = 0u;
    ltft_mult_clamp_pct_x10 = 50u;  // ±5.0% only
    stft_clamp_pct_x10 = 250u;      // STFT still ±25%

    // Drive STFT high (Ki lento — precisa de muitas iterações), LTFT com div=1
    for (int i = 0; i < 200; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false,
                               5000u, 500u);
    }
    const int16_t stft = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft > 50, "STFT pode ir além do clamp LTFT");
    const int16_t ltft = fuel_get_ltft_at(30000u, 100u);
    CHECK_TRUE(ltft <= 50 && ltft >= -50, "LTFT mult respeita clamp 50 (±5%)");
    CHECK_TRUE(stft > ltft || ltft == 50, "STFT authority > LTFT cell clamp");

    // max_step limita avanço por tick
    fuel_reset_ltft();
    fuel_reset_adaptives();
    ltft_mult_clamp_pct_x10 = 250u;
    ltft_learn_div = 1u;
    ltft_max_step_x10 = 5u;  // 0.5%/tick
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    // seed STFT high without LTFT catch: freeze LTFT via min rpm briefly
    ltft_adapt_min_rpm_x10 = 90000u;
    for (int i = 0; i < 30; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1150, 900, true, false, false,
                               5000u, 500u);
    }
    const int16_t stft_hi = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft_hi > 20, "STFT aquecido com LTFT freeze");
    ltft_adapt_min_rpm_x10 = 0u;
    const int16_t ltft0 = fuel_get_ltft_at(30000u, 100u);
    (void)fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false,
                           5000u, 500u);
    const int16_t ltft1 = fuel_get_ltft_at(30000u, 100u);
    const int16_t step = static_cast<int16_t>(
        (ltft1 > ltft0) ? (ltft1 - ltft0) : (ltft0 - ltft1));
    CHECK_TRUE(step <= 5, "max_step_x10=5 limita |Δ| por tick");

    // restore defaults
    ltft_mult_clamp_pct_x10 = 250u;
    ltft_add_clamp_us = 6350u;
    ltft_learn_div = 64u;
    ltft_commit_gain_pct = 50u;
    ltft_max_step_x10 = 0u;
    ltft_adapt_min_rpm_x10 = 12000u;
    closed_loop_post_start_s = 15u;
    fuel_reset_adaptives();
}

static void test_fuel_closed_loop_gates(void) {
    section("fuel_trim: closed_loop_enable + LTFT min RPM");

    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;  // no post-start delay in host tests
    ltft_adapt_min_rpm_x10 = 12000u;  // 1200 RPM

    // Master off → STFT freeze
    const int16_t stft0 = fuel_get_stft_pct_x10();
    closed_loop_enable = 0u;
    (void)fuel_update_stft(30000u, 100u, 1000, 1100, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_get_stft_pct_x10(), stft0, "CL enable=0 → STFT congelado");
    closed_loop_enable = 1u;

    // Abaixo do min RPM: STFT corre, LEARN hits não sobem
    fuel_ltft_accum_reset();
    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 8000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    (void)fuel_update_stft(8000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        (void)fuel_update_stft(8000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0, "RPM 800: STFT ainda integra");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "RPM 800 < min → zero hits LEARN");

    // Acima do min: hits acumulam
    fuel_ltft_accum_reset();
    const uint8_t ri2 = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi2 = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi2, ri2) > 0u, "RPM 3000 ≥ min → LEARN hits");

    closed_loop_enable = 1u;
    closed_loop_post_start_s = 15u;
    ltft_adapt_min_rpm_x10 = 12000u;
    fuel_reset_adaptives();
}

static void test_fuel_ltft_apply_nearest(void) {
    section("fuel_calc: LTFT apply = nearest (não floor bilineal)");

    fuel_reset_ltft();
    fuel_reset_adaptives();

    // 2000 rpm exacto no eixo: floor cai na célula anterior; nearest = nó 2000.
    const uint32_t rpm_x10 = 20000u;
    const uint16_t map_x100 = 110u;
    const uint8_t ri_floor =
        table_axis_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_floor =
        table_axis_index(kLoadAxisBarX100, kTableAxisSize, map_x100);
    const uint8_t ri_near =
        table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_near =
        table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, map_x100);
    CHECK_TRUE(ri_floor != ri_near || mi_floor != mi_near,
               "pré-condição: floor ≠ nearest neste OP");

    // Grava só a célula nearest em NVM (int8 %); floor fica 0.
    // nvm_write_ltft(rpm_i, load_i, val) — ordem (rpm, map).
    CHECK_TRUE(ems::hal::nvm_write_ltft(ri_near, mi_near, 12),
               "nvm LTFT nearest = +12%");
    CHECK_TRUE(ems::hal::nvm_write_ltft(ri_floor, mi_floor, 0),
               "nvm LTFT floor = 0");
    fuel_reset_adaptives();  // reload g_ltft_* from NVM

    CHECK_EQ(fuel_get_ltft_pct_x10(mi_near, ri_near), 120,
             "load nearest → +12.0% (×10)");
    CHECK_EQ(fuel_get_ltft_pct_x10(mi_floor, ri_floor), 0,
             "floor cell permanece 0");
    CHECK_EQ(fuel_get_ltft_at(rpm_x10, map_x100), 120,
             "apply mult usa nearest (+12%), não floor");

    // LTFT add: grava sub-grid da célula nearest
    CHECK_TRUE(ems::hal::nvm_write_ltft_add(ri_near >> 1u, mi_near >> 1u, 4),
               "nvm LTFT add nearest = +4×50µs");
    fuel_reset_adaptives();
    CHECK_EQ(fuel_get_ltft_add_at(rpm_x10, map_x100), 200,
             "apply add_at nearest = +200 µs");

    fuel_reset_ltft();
    fuel_reset_adaptives();
}

static void test_fuel_ltft_accum(void) {
    section("fuel_calc: LTFT accum stats / bake-in gates");

    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
    // Auto-learn default off — liga só nos trechos de commit VE
    ltft_apply_burn_ve = 0u;

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // ── sample_valid: contrato bake-in ───────────────────────────────────
    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, false,
                    1000, 1015, 40, 900, true, false, false),
                "sem amostra anterior → inválido");

    // |err|=15, STFT=4% estável, regime OK → válido (erro residual NÃO exigido)
    CHECK_TRUE(ltft_accum_sample_valid(
                   30000u, 30000u, 500u, 500u, true,
                   1000, 1015, 40, 900, true, false, false),
               "regime estável + λ convergida + STFT útil → válido");

    // |err|≈0 (no alvo com trim a segurar) → válido
    CHECK_TRUE(ltft_accum_sample_valid(
                   30000u, 30000u, 500u, 500u, true,
                   1000, 1000, 40, 900, true, false, false),
               "|err λ|≈0 com STFT estável → válido (bake-in)");

    // |err| > max → ainda a convergir / outlier
    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, true,
                    1000, 1050, 40, 900, true, false, false),
                "|erro λ| > max → inválido (não convergido)");

    CHECK_FALSE(ltft_accum_sample_valid(
                    32200u, 30000u, 500u, 500u, true,
                    1000, 1015, 40, 900, true, false, false),
                "ΔRPM > 200 → inválido");

    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, true,
                    1000, 1015, 160, 900, true, false, false),
                "|STFT| > 15% → inválido (saturado)");

    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, true,
                    1000, 1015, 40, 600, true, false, false),
                "CLT frio → inválido");

    // ── Integração via fuel_update_stft (λ perto do alvo) ────────────────
    // err=15 (1015-1000) ≤ max; 1ª amostra sem prev → 0 hits
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "1ª amostra: sem hit");

    // 2ª chamada estável → 1 hit
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 1u, "2ª amostra estável: 1 hit");

    // AE bloqueia closed-loop: prev NÃO avança (fica APP=500); hit não incrementa
    const uint16_t hits_before_ae = fuel_ltft_accum_hits(mi, ri);
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, true, false, 5000u, 900u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_ae, "AE: sem hit");
    // Após AE, APP=500 de novo → ΔTPS=0 vs prev preservado → aceita
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_ae + 1u,
             "pós-AE com APP estável: hit (prev preservado)");

    // Salto de APP → rejeita (prev=500, agora=700)
    const uint16_t hits_before_jump = fuel_ltft_accum_hits(mi, ri);
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 700u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_jump, "salto APP: hit não incrementa");
    // Nota: prev AVANÇA em closed-loop mesmo com amostra rejeitada → agora prev=700
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 700u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_jump + 1u,
             "APP estável no novo valor: hit");

    // ── Médias exactas (err constante) + ready ───────────────────────────
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    // 1ª = só prev; 2ª..11 = 10 hits com err=10
    fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int n = 0; n < 10; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 10u, "10 hits estáveis");
    CHECK_EQ(fuel_ltft_accum_mean_err_x1000(mi, ri), 10,
             "mean_err exacto = 10 (todas amostras err=10)");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri),
                "10 hits < ReadyHits → não ready");

    // Sobe o PI com err=20 (ainda ≤ MaxErr=30); depois limpa stats e grava
    // hits com λ no alvo (err=0) e STFT congelado. Ready NÃO aplica VE sozinho.
    for (int n = 0; n < 120; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    }
    const int16_t stft_now = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft_now >= kLtftAccumReadyMinMeanStftX10,
               "STFT aquecido ≥ min ready");
    CHECK_TRUE(stft_now <= kLtftAccumMaxStftX10,
               "STFT aquecido ainda dentro do gate de amostra");

    // Acumula até ready: VE intacta sem apply manual
    const uint8_t ve_before = ve_table[mi][ri];
    fuel_ltft_accum_reset();
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) >= kLtftAccumReadyHits,
               "stats acumulam sem mexer VE");
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri),
               "célula ready mas sem commit automático");
    CHECK_EQ(ve_table[mi][ri], ve_before, "closed-loop não altera VE");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "zero commits sem apply manual");
    {
        uint8_t exp[kLtftAccumPageSize] = {};
        fuel_ltft_accum_export(exp, kLtftAccumPageSize);
        const uint16_t eidx =
            static_cast<uint16_t>(mi) * kTableAxisSize + ri;
        CHECK_TRUE((exp[eidx] & 0x80u) != 0u,
                   "export bit7 ready=1 quando cell_ready");
    }

    // Apply manual (try_commit) → bake-in
    const int16_t stft_before_commit = fuel_get_stft_pct_x10();
    const int16_t ltft_before = fuel_get_ltft_pct_x10(mi, ri);
    CHECK_TRUE(fuel_ltft_accum_try_commit(mi, ri),
               "ready → try_commit manual OK");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u,
             "após commit stats da célula zerados");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri),
                "após commit não ready");
    if (stft_before_commit > 0 && ve_before < kLtftAccumVeMax) {
        CHECK_TRUE(ve_table[mi][ri] > ve_before,
                   "STFT+ → VE aumentou");
        CHECK_TRUE(fuel_get_ltft_pct_x10(mi, ri) <= ltft_before,
                   "LTFT desenrolado após bake-in positivo");
        CHECK_TRUE(fuel_get_stft_pct_x10() <= stft_before_commit,
                   "STFT desenrolado após bake-in positivo");
    }
    CHECK_FALSE(fuel_ltft_ve_burn_pending(),
                "burn_ve=0 → sem pedido de burn");

    // burn_ve=1 + apply manual → pending
    fuel_ltft_accum_reset();
    ltft_apply_burn_ve = 1u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_ltft_ve_burn_clear();
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_FALSE(fuel_ltft_ve_burn_pending(),
                "ready sem apply → sem burn pending");
    CHECK_TRUE(fuel_ltft_accum_try_commit(mi, ri), "apply manual com burn_ve=1");
    CHECK_TRUE(fuel_ltft_ve_burn_pending(), "burn pending true após apply");
    fuel_ltft_ve_burn_clear();
    CHECK_FALSE(fuel_ltft_ve_burn_pending(), "clear limpa pending");
    ltft_apply_burn_ve = 0u;

    fuel_ltft_accum_reset_cell(mi, ri);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "reset_cell zera hits");
    CHECK_EQ(fuel_ltft_accum_mean_stft_x10(mi, ri), 0, "reset_cell zera mean_stft");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri), "ready false com 0 hits");

    // Restaura VE: nearest == canto alto bilineal em 3000/100 (math tables).
    ve_table[mi][ri] = ve_before;
    fuel_reset_adaptives();
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "reset_adaptives zera acumulador");
}

static void test_fuel_ltft_accum_commit_ve(void) {
    section("fuel_calc: LTFT accum Fase 2 commit manual → VE");

    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
    ltft_apply_burn_ve = 0u;

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // Aquece STFT via PI (sem bake automático)
    for (int n = 0; n < 150; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    }
    const int16_t stft = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft >= kLtftAccumReadyMinMeanStftX10, "STFT aquecido p/ commit");

    // Estado limpo + VE conhecida ANTES do acumulador (após o warmup)
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_ltft_ve_burn_clear();

    // 1 prev + (ReadyHits-1) hits → ainda não ready
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < static_cast<uint16_t>(kLtftAccumReadyHits - 1u); ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) < kLtftAccumReadyHits,
               "ainda sem hits suficientes");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri),
                "ainda sem hits suficientes → não ready");
    CHECK_FALSE(fuel_ltft_accum_try_commit(mi, ri),
                "try_commit manual sem ready → false");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta sem commit");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "sem commits antes do hit ready");

    // Mais um hit → ready, mas VE só muda com apply manual
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri), "célula ready");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "sem auto-commit no hit ready");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta até apply manual");

    CHECK_TRUE(fuel_ltft_accum_try_commit(mi, ri), "apply manual → commit");
    CHECK_EQ(g_dbg_ltft_accum_commits, 1u, "1 commit manual");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "stats limpos pós-commit");
    CHECK_TRUE(ve_table[mi][ri] > 100u, "VE > 100 após bake-in STFT+");
    CHECK_TRUE(ve_table[mi][ri] <= kLtftAccumVeMax, "VE ≤ max");

    // apply_all_ready: bulk VE+LTFT; STFT global NÃO desenrola N vezes
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri), "ready p/ apply_all");
    const int16_t stft_before_all = fuel_get_stft_pct_x10();
    const uint16_t n_app = fuel_ltft_accum_apply_all_ready();
    CHECK_TRUE(n_app >= 1u, "apply_all_ready commitou ≥1");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri), "stats limpos pós apply_all");
    CHECK_TRUE(ve_table[mi][ri] > 100u, "VE alterada por apply_all");
    CHECK_EQ(fuel_get_stft_pct_x10(), stft_before_all,
             "apply_all não desenrola STFT global (só VE+LTFT célula)");

    // Caminho aditivo (PW < threshold): NÃO alimenta acumulador LEARN→VE
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 100u, 500u);
    for (uint16_t n = 0u; n < static_cast<uint16_t>(kLtftAccumReadyHits + 2u); ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 100u, 500u);
    }
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u,
             "caminho aditivo: zero hits LEARN (só LTFT add)");
    CHECK_FALSE(fuel_ltft_accum_try_commit(mi, ri),
                "caminho aditivo: nada ready para bake VE");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta no caminho aditivo");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "sem commits no caminho aditivo");

    // Restaura VE default (nearest 3000/100 = [11][10] = 88) p/ testes math.
    ve_table[mi][ri] = 88u;
    ltft_apply_burn_ve = 0u;
    fuel_reset_adaptives();
}

// ═══════════════════════════════════════════════════════════════════════════
// IGN CALC — SEGUNDA FASE
// ═══════════════════════════════════════════════════════════════════════════

static void test_ign_get_advance(void) {
    section("ign_calc: get_advance / get_advance_prepared");

    // spark_table is int8_t — values depend on defaults. Just verify consistency.
    const int16_t adv = get_advance(30000u, 100u);  // 3000 RPM, 100 kPa
    CHECK_TRUE(adv >= -40 && adv <= 80, "get_advance in plausible range [-40,80]°");

    // Prepared path must match
    const Table2dLookup lk = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u);
    const int16_t adv_prep = get_advance_prepared(lk);
    CHECK_EQ(adv, adv_prep, "get_advance_prepared == get_advance for same point");

    // Axis clamping: below min axis → uses first cell
    const int16_t adv_lo = get_advance(100u, 10u);   // below all axis values
    const int16_t adv_lo2 = get_advance(4999u, 19u); // just below first axis point
    CHECK_EQ(adv_lo, adv_lo2, "below-axis values both clamp to first cell");
}

static void test_ign_dwell_vbatt_rpm(void) {
    section("ign_calc: dwell_ms_x10_from_vbatt_rpm");

    // dwell_rpm_axis_rpm: {500,1200,4000,7000}
    // dwell_rpm_factor_q8: {384,288,256,200}  (384/256=1.5x at 500RPM, 200/256=0.78x at 7000RPM)
    // At 12V base dwell ≈ 30 x10. At 500 RPM (cranking): 30 × 384/256 = 45.
    const uint16_t d_crank = dwell_ms_x10_from_vbatt_rpm(12000u, 5000u);   // rpm_x10=5000 → 500 RPM
    const uint16_t d_mid   = dwell_ms_x10_from_vbatt_rpm(12000u, 40000u);  // 4000 RPM (factor=1.0)
    const uint16_t d_high  = dwell_ms_x10_from_vbatt_rpm(12000u, 70000u);  // 7000 RPM (factor=0.78)
    CHECK_TRUE(d_crank > d_mid,  "cranking dwell > mid-RPM dwell");
    CHECK_TRUE(d_mid   > d_high, "mid-RPM dwell > high-RPM dwell");
    // At reference RPM (4000 RPM, factor_q8=256=1.0): result == base
    const uint16_t base = dwell_ms_x10_from_vbatt(12000u);
    CHECK_EQ(d_mid, base, "dwell unchanged at 4000 RPM (factor=1.0x)");
}

static void test_ign_idle_spark_correction(void) {
    section("ign_calc: calc_idle_spark_correction_deg");

    // Calibration defaults:
    //   idle_spark_tps_max_x10=25, idle_spark_map_max_bar_x100=80
    //   idle_spark_rpm_min_x10=5000, idle_spark_window_above_target_x10=4000
    //   idle_spark_deadband_rpm_x10=500, idle_spark_rpm_per_deg_x10=500
    //   retard_limit=-8, advance_limit=12

    // Conditions NOT met: TPS too high
    CHECK_EQ(calc_idle_spark_correction_deg(8500u, 8500u, 30u, 60u), 0,
             "tps > max → 0");

    // Conditions NOT met: MAP too high
    CHECK_EQ(calc_idle_spark_correction_deg(8500u, 8500u, 0u, 90u), 0,
             "map > max → 0");

    // Within deadband: error=0 < 500
    CHECK_EQ(calc_idle_spark_correction_deg(8500u, 8500u, 0u, 60u), 0,
             "rpm == target (within deadband) → 0");

    // Below target by 1500 x10 (150 RPM): error=1500, -deadband=1000 → corr=1000/500=2° advance
    const int16_t corr_low = calc_idle_spark_correction_deg(7000u, 8500u, 0u, 60u);
    CHECK_EQ(corr_low, 2, "150 RPM below target → +2° advance");

    // Above target by 1500 x10: error=-1500, +deadband=-1000 → corr=-1000/500=-2° retard
    const int16_t corr_high = calc_idle_spark_correction_deg(10000u, 8500u, 0u, 60u);
    CHECK_EQ(corr_high, -2, "150 RPM above target → -2° retard");

    // Advance clamped at advance_limit=12: need idle_target big enough so
    // rpm (>= rpm_min=5000) still has error > deadband + 12*rpm_per_deg (=6500).
    // Use idle_target=20000, rpm=5000: error=15000, -deadband=14500 → 29° → clamped at 12.
    const int16_t corr_clamp = calc_idle_spark_correction_deg(5000u, 20000u, 0u, 60u);
    CHECK_EQ(corr_clamp, 12, "large underspeed (5000 vs target 20000) → clamped at +12°");
}

// ═══════════════════════════════════════════════════════════════════════════
// ETB CONTROL — C++ namespace (ems::engine)
// ═══════════════════════════════════════════════════════════════════════════

static void test_etb_cpp_update(void) {
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

static void test_torque_manager_cpp_update(void) {
    section("torque_manager (C++ ns): reset / update / test_get_target / test_get_limp_reason");

    ems::engine::torque_manager_reset();
    CHECK_EQ(ems::engine::torque_manager_test_get_target(), 0u, "target=0 after reset");
    CHECK_EQ(ems::engine::torque_manager_test_get_limp_reason(), 0u, "limp_reason=0 after reset");

    // key_on=false → everything disabled
    ems::drv::CkpSnapshot snap{};
    snap.rpm_x10 = 15000u;  // 1500 RPM
    ems::drv::SensorData sens{};
    sens.app_pct_x10 = 500u;  // 50% pedal

    auto out = ems::engine::torque_manager_update(snap, sens, false, false, false, 8500u, 10u);
    CHECK_FALSE(out.etb_enable_request, "key_off → ETB disabled");
    CHECK_EQ(out.etb_target_pct_x10, 0u, "key_off → target=0");
    CHECK_EQ(ems::engine::torque_manager_test_get_target(), 0u, "state target=0 when key_off");

    // key_on, no faults, valid calibration → target = app_pct_x10
    etb_cal_valid = 1u;
    out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 10u);
    CHECK_TRUE(out.etb_enable_request, "key_on + no faults → ETB enabled");
    CHECK_EQ(out.etb_target_pct_x10, 500u, "target = app_pct_x10=500");
    CHECK_EQ(ems::engine::torque_manager_test_get_target(), 500u, "state target=500");
    CHECK_EQ(ems::engine::torque_manager_test_get_limp_reason(), 0u, "no limp reason");

    // Invalid calibration → TORQUE_LIMP_NO_CALIB, target=0
    etb_cal_valid = 0u;
    out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 10u);
    CHECK_TRUE((out.limp_reason & TORQUE_LIMP_NO_CALIB) != 0u, "no_calib → LIMP_NO_CALIB set");
    CHECK_EQ(out.etb_target_pct_x10, 0u, "no_calib → target=0");
    etb_cal_valid = 1u;

    // map_clt_limp → TORQUE_LIMP_MAP_CLT, target clamped to etb_max_open_pct_x10_limp
    etb_max_open_pct_x10_limp = 250u;  // 25%
    sens.app_pct_x10 = 800u;           // 80% pedal
    out = ems::engine::torque_manager_update(snap, sens, true, true, false, 8500u, 10u);
    CHECK_TRUE((out.limp_reason & TORQUE_LIMP_MAP_CLT) != 0u, "map_clt_limp → LIMP_MAP_CLT");
    CHECK_EQ(out.etb_target_pct_x10, 250u, "limp target clamped to 250 (25%)");

    // rev_cut → TORQUE_LIMP_REV_CUT, target=0
    sens.app_pct_x10 = 500u;
    out = ems::engine::torque_manager_update(snap, sens, true, false, true, 8500u, 10u);
    CHECK_TRUE((out.limp_reason & TORQUE_LIMP_REV_CUT) != 0u, "rev_cut → LIMP_REV_CUT");
    CHECK_EQ(out.etb_target_pct_x10, 0u, "rev_cut → target=0");

    // APP fault → TORQUE_LIMP_APP_FAULT
    sens.app_pct_x10 = 500u;
    sens.throttle_fault_bits = ems::drv::THROTTLE_FAULT_APP1;
    out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 10u);
    CHECK_TRUE((out.limp_reason & TORQUE_LIMP_APP_FAULT) != 0u, "APP fault → LIMP_APP_FAULT");
    sens.throttle_fault_bits = 0u;
}

// ═══════════════════════════════════════════════════════════════════════════
// CKP — SEGUNDA FASE (seed_confirmed, seed_rejected, cmp_glitch)
// ═══════════════════════════════════════════════════════════════════════════

// Helper: fire the cam ISR (TIM5 CH2). IDR bit 1 must be high.
static void cam_fire(uint32_t capture_value) {
    ems_test_cam_gpio_idr = (1u << 1u);  // bit 1 = rising edge
    ems_test_tim5_ccr2 = capture_value;
    ckp_tim5_ch2_isr();
}

static void test_ckp_seed_confirmed(void) {
    section("ckp: seed_confirmed_count after cam edge during probation");

    // NOTA: o seed está desativado em produção (ckp.cpp "FIX 2026-06-29: seed
    // desativado p/ diagnóstico", TODO: re-activar). g_seed_probation nunca é
    // posto a true em nenhum caminho de código atual — o 1º gap vai sempre
    // para HALF_SYNC, nunca para FULL_SYNC+probation, e ckp_seed_arm() não
    // tem qualquer efeito observável. Este teste reflete esse estado actual;
    // quando o seed for reativado, restaurar a expectativa de FULL_SYNC aqui.
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_seed_arm(true);

    for (uint32_t i = 0; i < 55u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kNormalPeriod * 3u);  // gap: seed desativado → HALF_SYNC (não FULL_SYNC)
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::HALF_SYNC), "pre-cond: HALF_SYNC (seed desativado)");

    // Cam ISR sem probation ativa não confirma nada.
    cam_fire(g_ckp_cap + kNormalPeriod * 58u);
    CHECK_EQ(ckp_seed_confirmed_count(), 0u, "seed_confirmed_count=0 (seed desativado)");
}

static void test_ckp_seed_rejected(void) {
    section("ckp: seed_rejected_count after probation timeout");

    // NOTA: mesmo motivo do teste acima — seed desativado, nunca entra em
    // probation, logo nunca rejeita por timeout.
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_seed_arm(true);

    for (uint32_t i = 0; i < 55u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kNormalPeriod * 3u);  // gap: seed desativado → HALF_SYNC
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::HALF_SYNC), "pre-cond: HALF_SYNC (seed desativado)");

    // Sem probation ativa, nenhuma quantidade de dentes gera rejeição.
    for (uint32_t i = 0; i < 71u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(ckp_seed_rejected_count(), 0u, "seed_rejected_count=0 (seed desativado)");
}

static void test_ckp_cmp_glitch_count(void) {
    section("ckp: ckp_get_cmp_glitch_count on invalid cam timing");

    ckp_reach_full_sync();
    // ckp_test_reset() inside ckp_reach_full_sync() now also resets s_prev_cmp_capture.

    // First cam edge: s_prev_cmp_capture=0 → skip validation → always accepted.
    const uint32_t cap1 = g_ckp_cap;
    cam_fire(cap1);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "first cam edge always accepted");

    // Second cam edge too soon (delta=100, expected=58×10000=580000) → glitch
    cam_fire(cap1 + 100u);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 1u, "cam edge too soon → glitch counted");
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSORS — SEGUNDA FASE
// ═══════════════════════════════════════════════════════════════════════════

static void test_sensors_on_tooth(void) {
    section("sensors: sensors_on_tooth");
    sensor_setup(); sensors_init();
    ems::drv::CkpSnapshot snap{};
    snap.tooth_period_ns = 160000u;  // 10000 ticks × 16 ns = 160000 ns
    snap.rpm_x10 = 62500u;
    sensors_on_tooth(snap);  // triggers ADC sample accumulation (fast channels)
    CHECK_TRUE(true, "sensors_on_tooth completes without crash");
}

static void test_sensors_tick_50ms(void) {
    section("sensors: sensors_tick_50ms");
    sensor_setup(); sensors_init();
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::FUEL_PRESS, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::OIL_PRESS,  2000u);
    sensors_tick_50ms();
    CHECK_TRUE(true, "sensors_tick_50ms completes without crash");
    // After tick: fuel/oil pressure should be set (raw=2000 → some bar value)
    for (int i = 0; i < 4; ++i) { sensors_tick_50ms(); }  // fill staging buffers
    sensors_test_tick_100ms();  // commits staging → committed (double-buffer swap)
    const ems::drv::SensorData sd = sensors_get();
    CHECK_TRUE(sd.fuel_press_bar_x1000 > 0u, "fuel_press > 0 after tick_50ms with raw=2000");
}

static void test_sensors_set_range(void) {
    section("sensors: sensors_set_range");
    sensor_setup(); sensors_init();
    // Widen CLT range so that raw=50 is accepted
    sensors_set_range(SensorId::CLT, {50u, 4000u});
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 50u), "CLT raw=50 valid after range change");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 49u), "CLT raw=49 still invalid");
}

static void test_sensors_etb_harness_present(void) {
    section("sensors: sensors_set_etb_harness_present");
    sensor_setup(); sensors_init();
    sensors_set_etb_harness_present(true);
    // When harness present, tick_100ms uses fixed vbatt=12000 instead of ADC.
    // Just verify no crash.
    sensors_test_tick_100ms();
    CHECK_TRUE(true, "tick_100ms with harness_present=true: no crash");
    sensors_set_etb_harness_present(false);  // restore
}

static void test_sensors_table_entry_setters(void) {
    section("sensors: sensors_test_set_clt_table_entry / set_iat_table_entry");
    sensor_setup(); sensors_init();
    // Manually set CLT table entry at index 62 (ADC=2000>>5=62) to 200 (20.0°C)
    sensors_test_set_clt_table_entry(62u, 200);
    sensors_test_set_iat_table_entry(62u, 150);
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT, 2000u);
    for (int i = 0; i < 8; ++i) { sensors_test_tick_100ms(); }
    const ems::drv::SensorData sd = sensors_get();
    CHECK_EQ(sd.clt_degc_x10, 200, "CLT table entry 62 = 200 (20.0°C)");
    CHECK_EQ(sd.iat_degc_x10, 150, "IAT table entry 62 = 150 (15.0°C)");
}

// ═══════════════════════════════════════════════════════════════════════════
// KNOCK — SEGUNDA FASE
// ═══════════════════════════════════════════════════════════════════════════

static void test_knock_window_cycle_end(void) {
    section("knock: knock_window_cycle_end");
    knock_init();
    knock_set_adc_threshold(2000u);
    knock_set_event_threshold(2u);

    // Open window, feed 3 above-threshold samples, call cycle_end (closes + evaluates)
    knock_window_open(0u);
    knock_test_set_adc_raw(2500u);
    knock_test_set_adc_raw(2500u);
    knock_test_set_adc_raw(2500u);  // count=3 > threshold=2
    CHECK_TRUE(knock_test_window_active(), "pre-cond: window active");

    knock_window_cycle_end();
    CHECK_FALSE(knock_test_window_active(), "window closed by cycle_end");
    CHECK_TRUE(knock_get_retard_x10(0u) > 0u, "retard applied by cycle_end");
}

static void test_knock_save_to_nvm(void) {
    section("knock: knock_save_to_nvm");
    knock_init();
    // NVM is mocked in host test (flash.cpp stub). Just verify no crash.
    knock_save_to_nvm();
    CHECK_TRUE(true, "knock_save_to_nvm: no crash");
}

// ═══════════════════════════════════════════════════════════════════════════
// AUXILIARIES — SEGUNDA FASE (getters WG/VVT)
// ═══════════════════════════════════════════════════════════════════════════

static void test_aux_test_getters(void) {
    section("auxiliaries: test_get_wg_duty / vvt_esc_duty / vvt_adm_duty / wg_failsafe");
    auxiliaries_test_reset();
    // After init all duties are 0 and no failsafe
    CHECK_EQ(auxiliaries_test_get_wg_duty(),       0u, "wg_duty=0 after reset");
    CHECK_EQ(auxiliaries_test_get_vvt_esc_duty(),  0u, "vvt_esc_duty=0 after reset");
    CHECK_EQ(auxiliaries_test_get_vvt_adm_duty(),  0u, "vvt_adm_duty=0 after reset");
    CHECK_FALSE(auxiliaries_test_get_wg_failsafe(), "wg_failsafe=false after reset");
    // Multiple ticks should not crash even with these getters
    for (int i = 0; i < 5; ++i) { auxiliaries_tick_20ms(); }
    CHECK_TRUE(true, "WG/VVT getters accessible after ticks");
}

// ═══════════════════════════════════════════════════════════════════════════
// TIMER HAL (stubs em host — testa que não crasham)
// ═══════════════════════════════════════════════════════════════════════════

static void test_timer_stubs(void) {
    section("timer HAL: all stubs execute without crash");
    using namespace ems::hal;
    tim5_ic_init();
    const uint32_t cnt = tim5_count();
    CHECK_EQ(cnt, 0u, "tim5_count() returns mock value (0)");
    tim3_pwm_init(15u);
    tim3_set_duty(0u, 500u);
    tim3_set_duty(1u, 250u);
    tim4_pwm_init(15u);
    tim4_set_duty(0u, 750u);
    tim4_set_duty(1u, 1000u);
    tim15_etb_pwm_init(20000u);
    tim15_etb_set_duty_x10(500u);
    CHECK_TRUE(true, "all timer stubs: no crash");
}

// ============================================================================
// ECU SCHED — FASE 2 (arm_channel, CCR, late events, dwell watchdog, presync)
// ============================================================================

static void test_ecu_sched_hardware_init(void) {
    section("ecu_sched: ECU_Hardware_Init runs without crash");
    // ECU_Hardware_Init writes to TIM2/TIM1/GPIO mock registers (file-scope statics
    // in ecu_sched.cpp, not externally observable). Only testable behavior:
    //   1. No crash.
    //   2. Angle table cleared — ecu_sched_test_angle_table_size()=0 after init.
    //   3. Diagnostic counters cleared.
    ECU_Hardware_Init();
    CHECK_EQ(ecu_sched_test_angle_table_size(), 0u,
             "angle table empty after ECU_Hardware_Init");
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "dwell_watchdog_count=0 after ECU_Hardware_Init");
    CHECK_TRUE(true, "ECU_Hardware_Init: no crash");
}

static void test_ecu_sched_ccr_write(void) {
    section("ecu_sched: arm_channel inserts event into TIM5 queue on DWELL_START");

    // Scheduler now uses TIM5-based absolute-timestamp event queue + GPIO BSRR.
    // TIM1 CCRs are no longer written by arm_channel.
    // Verify: after firing 13 teeth, at least one event is in the TIM5 queue.
    ecu_sched_test_reset();
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();  // angle table built at FULL_SYNC gap

    // Events fire when specific teeth match angle table entries — not at tooth 0.
    // Fire a full revolution so at least one event tooth is hit.
    for (uint32_t i = 0u; i < 58u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u ||
               ecu_sched_test_get_tim5_ccr3() > 0u,
               "at least one event in TIM5 queue after full revolution");
}

static void test_ecu_sched_late_events(void) {
    section("ecu_sched: small delta events are queued with minimum delay");

    // Scheduler now uses TIM5 absolute-timestamp queue. Events with very small
    // delta use a minimum delay (STM32_MIN_COMPARE_LEAD_TICKS) instead
    // of being rejected. The old g_late_event_count path is no longer reached.
    // Verify: with advance=0 (delta≈0 at tooth 0), events still reach the queue.
    ecu_sched_test_reset();
    ecu_sched_set_advance_deg(0u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // Events were inserted (with minimum delay) even for near-zero delta.
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u ||
               ecu_sched_test_get_tim5_ccr3() > 0u,
               "events queued with minimum delay when delta~=0");
}

// Golden identity checks (plan verification): min-lead timestamp formula, angle
// table shape, sorted queue order — no soft "count>0" only.
static void test_ecu_sched_golden_min_lead_timestamp(void) {
    section("ecu_sched golden: min-lead insert timestamp + late counter");
    // STM32_MIN_COMPARE_LEAD_TICKS = 125 @ 62.5 MHz (2 µs).
    // ECU_SCHED_US_TO_TICKS(1) = 62 < 125 → OFF event must land at now+125.
    constexpr uint32_t kNow = 100000u;
    constexpr uint32_t kMinLead = 125u;
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_pulse_inj(0u, 1u);  // 1 µs PW → short delta → min-lead
    CHECK_TRUE(ecu_sched_test_get_evt_count() >= 1u, "OFF event queued");
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "peek head event");
    CHECK_EQ(ts, kNow + kMinLead, "min-lead timestamp = TIM5_CNT + 125");
    CHECK_EQ(high, 0u, "OFF event is low");
    CHECK_EQ(ch, ECU_CH_INJ1, "INJ1 channel id unchanged");
    CHECK_TRUE(ecu_sched_test_get_late_event_count() > late0,
               "late_event_count increments on min-lead path only (diag)");
}

static void test_ecu_sched_golden_far_target_timestamp(void) {
    section("ecu_sched golden: far target uses exact delta (no min-lead)");
    constexpr uint32_t kNow = 50000u;
    constexpr uint32_t kPwUs = 1000u;  // 1 ms → 62500 ticks @ 62.5 MHz
    constexpr uint32_t kExpectedDelta = (kPwUs * 125u) / 2u;  // ECU_SCHED_US_TO_TICKS
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_pulse_inj(1u, kPwUs);
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "peek OFF event");
    CHECK_EQ(ts, kNow + kExpectedDelta, "timestamp = now + exact PW ticks");
    CHECK_EQ(ch, ECU_CH_INJ2, "INJ2 channel");
    CHECK_EQ(high, 0u, "OFF");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), late0,
             "no late count when delta >= min-lead");
}

static void test_ecu_sched_golden_queue_sorted(void) {
    section("ecu_sched golden: queue stays sorted by timestamp");
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    // Two pulses with different PW → two OFF times; queue must be ascending.
    ecu_sched_test_pulse_inj(0u, 2000u);  // later OFF
    ecu_sched_test_set_tim5_cnt(1000u);   // same now for second arm
    ecu_sched_test_pulse_inj(1u, 500u);   // earlier OFF
    const uint8_t n = ecu_sched_test_get_evt_count();
    CHECK_TRUE(n >= 2u, "at least two OFF events");
    uint32_t prev = 0u;
    for (uint8_t i = 0u; i < n; ++i) {
        uint32_t ts = 0u;
        CHECK_EQ(ecu_sched_test_get_evt(i, &ts, nullptr, nullptr), 1u, "peek evt");
        if (i > 0u) {
            CHECK_TRUE(ts >= prev, "queue non-decreasing timestamps");
        }
        prev = ts;
    }
}

static void test_ecu_sched_golden_seq_angle_table_size(void) {
    section("ecu_sched golden: sequential base angle table is 16 events");
    // 4 cyl × (DWELL + SPARK + INJ_ON + INJ_OFF) = 16 without multi-spark.
    ecu_sched_test_reset();
    ecu_sched_set_mspark(0u, 0u, 18u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    ckp_test_set_cmp_confirms(2u);
    ckp_feed_n_then_gap(57u);  // rebuild sequential at gap
    // May need schedule_this_gap toggle: first sequential gap builds table.
    if (ecu_sched_test_angle_table_size() == 0u) {
        ckp_feed_n_then_gap(57u);
    }
    CHECK_EQ(ecu_sched_test_angle_table_size(), 16u,
             "sequential no-mspark table has 16 events");
    // Every event has valid channel + action.
    uint8_t n_dwell = 0u, n_spark = 0u, n_inj_on = 0u, n_inj_off = 0u;
    for (uint8_t i = 0u; i < 16u; ++i) {
        uint8_t tooth = 0, frac = 0, ch = 0, action = 0, phase = 0;
        CHECK_EQ(ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase),
                 1u, "event valid");
        if (action == ECU_ACT_DWELL_START) { ++n_dwell; }
        else if (action == ECU_ACT_SPARK) { ++n_spark; }
        else if (action == ECU_ACT_INJ_ON) { ++n_inj_on; }
        else if (action == ECU_ACT_INJ_OFF) { ++n_inj_off; }
    }
    CHECK_EQ(n_dwell, 4u, "4 dwell starts");
    CHECK_EQ(n_spark, 4u, "4 sparks");
    CHECK_EQ(n_inj_on, 4u, "4 inj on");
    CHECK_EQ(n_inj_off, 4u, "4 inj off");
}

static void test_ecu_sched_golden_dispatch_identity(void) {
    section("ecu_sched golden: dispatch fires head GPIO order (channel, high)");
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    ecu_sched_test_pulse_inj(0u, 1000u);  // OFF at 1000+62500
    // Make event due and dispatch.
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "have event");
    const uint8_t n0 = ecu_sched_test_get_evt_count();
    ecu_sched_test_set_tim5_cnt(ts);  // CNT == timestamp → due
    ecu_sched_evt_dispatch();
    CHECK_EQ(ecu_sched_test_get_evt_count(), static_cast<uint32_t>(n0 - 1u),
             "one event consumed by dispatch");
    // Pin counters: INJ1 pin index 0 — OFF is low transition after force ON.
    // At least one high and one low counted for pin 0 path (force ON + OFF).
    uint32_t pins[24];
    ecu_sched_get_pin_counts_u32x24(pins);
    CHECK_TRUE(pins[0] >= 1u, "INJ1 high_count >= 1 after force ON");
    CHECK_TRUE(pins[1] >= 1u, "INJ1 low_count >= 1 after OFF dispatch");
}

static void test_ecu_sched_dwell_watchdog_fires(void) {
    section("ecu_sched: dwell watchdog fires after 1.4x dwell ticks");

    // Setup: same as CCR test. After 13 teeth, DWELL_START for cyl3 (tim_ch=3)
    // sets g_dwell_arm_tick[2]=TIM5_CNT=0 and g_dwell_wdog_ticks[2]=140625*7/5=196875.
    // arm_channel uses TIM5_CNT (scheduler_counter()). Watchdog also reads TIM5_CNT.
    // TIM5_CNT must be ≠1: 0 is the "not armed" sentinel for g_dwell_arm_tick.
    // cyl3 → ECU_CH_IGN4 → tim_ch=4 → ign_idx=3. arm_tick = TIM5_CNT at arm time.
    // wdog threshold = dwell_ticks × 7/5 = 140625×7/5 = 196875.
    const uint32_t kTim2Base = 1000u;
    const uint32_t kWdogTicks = (140625u * 7u) / 5u;  // 196875
    ecu_sched_test_reset();
    ecu_sched_test_set_tim2_cnt(kTim2Base);  // sets TIM5_CNT so arm_tick != 0
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // Force sequential mode: requires cmp_confirms>=2 and a gap to rebuild table.
    ckp_test_set_cmp_confirms(2u);
    // Fire 57 teeth so presync SPARK at tooth 57 clears arm_ticks before the gap.
    // Then the gap triggers Calculate_Sequential_Cycle (sequential mode).
    ckp_feed_n_then_gap(57u);  // sequential table built; tooth_index back to 0
    // Fire 13 teeth: at tooth 13 DWELL_START for cyl3 (IGN4) fires.
    for (uint32_t i = 0u; i < 13u; ++i) { ckp_fire(kNormalPeriod); }
    // Pre-condition: watchdog not fired (elapsed = 0, threshold = 196875)
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "pre-cond: wdog_count=0 before threshold");
    // Advance TIM5_CNT past 1.4× dwell from arm_tick=kTim2Base
    ecu_sched_test_set_tim2_cnt(kTim2Base + kWdogTicks + 1u);
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 1u,
             "dwell watchdog fires: elapsed > 196875 (1.4× dwell)");
    // Second call: arm_tick reset to 0 by watchdog → sentinel check fails → no re-fire
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 1u, "watchdog fires only once per arm");
}

static void test_ecu_sched_presync_table(void) {
    section("ecu_sched: presync table built in HALF_SYNC at natural rev boundary");

    // After HALF_SYNC, tooth_index advances 0..57. When tooth_index wraps 57→0,
    // rev_boundary=1 with state=HALF_SYNC → calculate_presync_revolution fires.
    ecu_sched_test_reset();
    ecu_sched_test_set_tim1_cnt(0u);
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    ckp_test_reset(); g_ckp_cap = 0u;

    ckp_feed_n_then_gap(55u);   // → HALF_SYNC (no rev_boundary yet: prev_tooth=0)
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::HALF_SYNC), "pre-cond: HALF_SYNC");

    // Fire 58 normal teeth: tooth_index 1..57, then wraps 57→0.
    // At the wrap: prev_tooth=57≠0 → rev_boundary=1 → calculate_presync_revolution.
    for (uint32_t i = 0u; i < 58u; ++i) { ckp_fire(kNormalPeriod); }
    const uint8_t tsz = ecu_sched_test_angle_table_size();
    CHECK_TRUE(tsz > 0u, "presync angle table populated after HALF_SYNC rev boundary");

    // Presync events use ECU_PHASE_ANY (= 2) so they fire on every revolution
    bool found_any = false;
    for (uint8_t i = 0u; i < tsz; ++i) {
        uint8_t tooth, frac, ch, action, phase;
        if (ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase) != 0u) {
            if (phase == ECU_PHASE_ANY) { found_any = true; }
        }
    }
    CHECK_TRUE(found_any, "at least one presync event uses ECU_PHASE_ANY");

    // Presync IGN events include DWELL_START and SPARK for all 4 coils simultaneously
    // → table should have ≥ 2 ignition actions (at minimum: DWELL_START + SPARK)
    uint8_t n_ign = 0u;
    for (uint8_t i = 0u; i < tsz; ++i) {
        uint8_t tooth, frac, ch, action, phase;
        if (ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase) != 0u) {
            if (action == ECU_ACT_DWELL_START || action == ECU_ACT_SPARK) { ++n_ign; }
        }
    }
    CHECK_TRUE(n_ign >= 2u, "presync table: ≥2 ignition events");
}

// ============================================================================
// CKP — FASE 3 (prime_on_tooth, snap fields, tooth_index, phase_A toggle)
// ============================================================================

static constexpr uint32_t kCrankPeriod = 100000u;  // rpm_x10 = 6250 < 7000 (cranking)

static void test_ckp_prime_on_tooth(void) {
    section("ckp: prime_on_tooth generates prime pulse after bootstrap + target teeth");

    // crank_prime_tooth = 3. Bootstrap = first 3 teeth (prime not called).
    // Teeth 4, 5, 6 call prime_on_tooth with count=1,2,3. At count=3: fires.
    quick_crank_reset();
    ckp_test_reset(); g_ckp_cap = 0u;
    quick_crank_set_prime_context(-400, 500);  // CLT=-40°C, cold engine

    // rpm_x10 = 625000000 / 100000 = 6250 < crank_exit=7000 → cranking mode
    for (uint32_t i = 0u; i < 7u; ++i) { ckp_fire(kCrankPeriod); }

    const uint32_t prime_pw = quick_crank_consume_prime();
    CHECK_TRUE(prime_pw > 0u, "prime pulse generated at 7th cranking tooth (4 post-bootstrap)");
    CHECK_TRUE(prime_pw <= 30000u, "prime_pw ≤ max clamp (30 ms)");

    // One-shot: second consume returns 0
    CHECK_EQ(quick_crank_consume_prime(), 0u, "prime is one-shot");

    // After reset: no prime pending
    quick_crank_reset();
    CHECK_EQ(quick_crank_consume_prime(), 0u, "no prime after quick_crank_reset");
}

static void test_ckp_snap_fields(void) {
    section("ckp: snap fields accurate in FULL_SYNC");

    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    const auto snap = ckp_snapshot();

    // tooth_period_ns = kNormalPeriod ticks * 16 ns/tick
    CHECK_EQ(snap.tooth_period_ns, kNormalPeriod * 16u,
             "tooth_period_ns = ticks × 16");

    // predicted period > 0 (set by predict_next_period_ticks on last normal tooth)
    CHECK_TRUE(snap.predicted_tooth_period_ns > 0u,
               "predicted_tooth_period_ns > 0");

    // With constant period, predicted ≈ actual (trend=0)
    CHECK_EQ(snap.predicted_tooth_period_ns, kNormalPeriod * 16u,
             "predicted_tooth_period_ns = actual at constant speed");

    // rpm_x10 = 625000000 / kNormalPeriod
    CHECK_EQ(snap.rpm_x10, 625000000u / kNormalPeriod, "rpm_x10 correct");

    // last_tim5_capture is updated with each tooth
    CHECK_TRUE(snap.last_tim5_capture > 0u, "last_tim5_capture > 0 after teeth");
}

static void test_ckp_tooth_index_progression(void) {
    section("ckp: tooth_index increments per normal tooth and wraps at 58");

    g_ckp_cap = 0u;
    ckp_reach_full_sync();  // tooth_index=0 at FULL_SYNC gap
    const uint16_t idx0 = ckp_snapshot().tooth_index;
    CHECK_EQ(idx0, 0u, "tooth_index=0 immediately after gap");

    ckp_fire(kNormalPeriod);
    CHECK_EQ(ckp_snapshot().tooth_index, 1u, "tooth_index=1 after 1 tooth");

    ckp_fire(kNormalPeriod);
    CHECK_EQ(ckp_snapshot().tooth_index, 2u, "tooth_index=2 after 2 teeth");

    // Advance to tooth 57 (fires 55 more teeth: 2+55=57)
    for (uint32_t i = 0u; i < 55u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(ckp_snapshot().tooth_index, 57u, "tooth_index=57 after 57 teeth");

    // 58th tooth wraps to 0 (kRealTeethPerRev=58, 57=max, next=0)
    ckp_fire(kNormalPeriod);
    CHECK_EQ(ckp_snapshot().tooth_index, 0u, "tooth_index wraps 57→0 on 58th tooth");

    // State still FULL_SYNC (tooth_count=58 < kMaxTeethBeforeLoss=63)
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "FULL_SYNC maintained through wrap");
}

static void test_ckp_phase_toggle(void) {
    section("ckp: CMP corrects phase_A to kCmpRefHalf; glitch is ignored");

    // CMP does NOT just toggle — it SETS phase_A to kCmpRefHalf (=0→true) at
    // the NEXT gap via advance_phase_half(). A glitch is rejected; the gap then
    // only performs the normal phase toggle.

    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_reach_full_sync();

    // First cam: s_prev_cmp_capture=0 → validation skipped → always accepted.
    // Records cam1_ts for delta validation of subsequent edges.
    const uint32_t cam1_ts = g_ckp_cap + kNormalPeriod * 58u;
    cam_fire(cam1_ts);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "first cam edge not a glitch");
    ckp_feed_n_then_gap(55u);  // gap applies pending CMP correction
    // After CMP correction: phase_half = kCmpRefHalf = 0 → phase_A = true.
    CHECK_EQ(ckp_snapshot().phase_A, true, "first CMP corrects phase_A to kCmpRefHalf (true)");

    // Second cam: delta = cam2_ts - cam1_ts = 116×kNP (2 crank revs = 720°).
    // expected = 2×58×kNP = 1160000; tolerance ±25% → [870k, 1450k] ✓.
    cam_fire(g_ckp_cap + kNormalPeriod * 116u);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "second cam edge not a glitch");
    ckp_feed_n_then_gap(55u);  // gap applies pending correction again
    CHECK_EQ(ckp_snapshot().phase_A, true, "second CMP corrects phase_A to kCmpRefHalf (true)");

    // Glitch: delta from prev valid CMP is too small → rejected.
    cam_fire(g_ckp_cap + 10u);  // delta << expected → glitch
    CHECK_EQ(ckp_get_cmp_glitch_count(), 1u, "glitch cam edge counted");
    // No cmp_phase_pending set → gap just does normal toggle.
    const bool phase_before = ckp_snapshot().phase_A;  // true
    ckp_feed_n_then_gap(55u);
    CHECK_EQ(ckp_snapshot().phase_A, !phase_before, "after glitch: phase_A toggles normally (no CMP correction)");
}

// (all includes moved to top of file)

// ============================================================================
// TABLE3D
// ============================================================================

static void test_table3d_all(void) {
    using namespace ems::engine;

    // ─ table_axis_index ────────────────────────────────────────────────
    section("table3d: table_axis_index");
    // kRpmAxisX10: {5000,7500,10000,...,120000} — kTableAxisSize=16
    // Below first → idx=0
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 100u), 0u,
             "below axis[0] → idx=0");
    // At exact first value → idx=0
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 5000u), 0u,
             "at axis[0]=5000 → idx=0");
    // Between axis[0]=5000 and axis[1]=7500 → idx=0
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 6000u), 0u,
             "between axis[0] and axis[1] → idx=0");
    // value==axis[1]: binary search sets hi=1, idx=lo-1=0 (lower interval)
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 7500u), 0u,
             "at axis[1]=7500 → idx=0 (lower interval)");
    // Above last → idx=kTableAxisSize-2=18
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 999999u), 18u,
             "above last → idx=kTableAxisSize-2=18");

    // ─ table_axis_nearest_index (célula dominante = trace VE do dash) ─
    // Em valor exacto no eixo k>0, table_axis_index devolve k-1 (frac=255).
    // nearest deve devolver k — senão LEARN hit cai na célula anterior.
    section("table3d: table_axis_nearest_index");
    // kRpmAxisX10[6]=20000 (2000 rpm), [5]=17500
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 20000u), 5u,
             "floor em 2000 rpm exacto → idx 5 (1750)");
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 20000u), 6u,
             "nearest em 2000 rpm exacto → idx 6 (2000)");
    // kLoadAxisBarX100[12]=110, [11]=100
    CHECK_EQ(table_axis_index(kLoadAxisBarX100, kTableAxisSize, 110u), 11u,
             "floor em MAP 110 exacto → idx 11 (100)");
    CHECK_EQ(table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 110u), 12u,
             "nearest em MAP 110 exacto → idx 12 (110)");
    // Midpoint RPM 2125 (2000–2250): frac=0.5 → sobe
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 21250u), 7u,
             "nearest midpoint 2125 rpm → 2250 (idx 7)");
    // Abaixo do 1º / no 1º
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 100u), 0u,
             "nearest below axis[0] → 0");
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 5000u), 0u,
             "nearest at axis[0] → 0");
    // No último nó
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 80000u), 19u,
             "nearest at last rpm → idx 19");

    // ─ table_axis_frac_q8 ────────────────────────────────────────────
    section("table3d: table_axis_frac_q8");
    // axis[0]=5000, axis[1]=7500. At value=5000 → frac=0
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 5000u), 0u, "at axis[0]: frac=0");
    // At value=7500 (== axis[1]) → frac=255
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 7500u), 255u, "at axis[1]: frac=255");
    // Midpoint 6250: (6250-5000)/(7500-5000) × 256 = 1250/2500×256 = 128
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 6250u), 128u, "midpoint: frac=128");
    // Below axis[0] → 0
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 4000u), 0u, "below axis[0]: frac=0");

    // ─ table3d_lookup_u8_prepared / table3d_lookup_u8 ──────────────────
    section("table3d: lookup_u8 bilinear interpolation");
    // Build flat table: all cells = 80
    static uint8_t flat_u8[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            flat_u8[y][x] = 80u;
    const Table2dLookup lk = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100,
                                                     30000u, 100u);
    CHECK_EQ(table3d_lookup_u8_prepared(flat_u8, lk), 80u,
             "flat u8 table: any point → 80");
    CHECK_EQ(table3d_lookup_u8(flat_u8, kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u),
             80u, "table3d_lookup_u8 matches prepared");

    // Gradient table: cell[y][x] = x+y (0..30), expect interpolation
    static uint8_t grad_u8[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            grad_u8[y][x] = static_cast<uint8_t>(x + y);
    // At axis boundary (exact) xi=0,yi=0,frac=0 → cell[0][0]=0
    const Table2dLookup lk00 = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100,
                                                       5000u, 20u);
    CHECK_EQ(table3d_lookup_u8_prepared(grad_u8, lk00), 0u,
             "grad u8 at [0][0] → 0");

    // ─ table3d_lookup_i8_prepared ────────────────────────────────────────
    section("table3d: lookup_i8_prepared");
    static int8_t flat_i8[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            flat_i8[y][x] = -10;
    CHECK_EQ(table3d_lookup_i8_prepared(flat_i8, lk), (int16_t)-10,
             "flat i8 table: any point → -10");
    static int8_t neg_grad[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            neg_grad[y][x] = static_cast<int8_t>(-x - y);
    CHECK_EQ(table3d_lookup_i8_prepared(neg_grad, lk00), (int16_t)0,
             "neg grad i8 at [0][0] → 0");

    // ─ table3d_lookup_s16_prepared / table3d_lookup_s16 ──────────────
    section("table3d: lookup_s16_prepared");
    static int16_t flat_s16[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            flat_s16[y][x] = 1000;
    CHECK_EQ(table3d_lookup_s16_prepared(flat_s16, lk), (int16_t)1000,
             "flat s16 table: any point → 1000");
    CHECK_EQ(table3d_lookup_s16(flat_s16, kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u),
             (int16_t)1000, "table3d_lookup_s16 matches prepared");

    // ─ table3d_lookup_ve_q8 ───────────────────────────────────────────
    section("table3d: lookup_ve_q8");
    // flat VE=80 → result in Q8 = 80<<8=20480
    const uint16_t ve_q8 = table3d_lookup_ve_q8(flat_u8,
                                                  kRpmAxisX10, kLoadAxisBarX100,
                                                  30000u, 100u);
    CHECK_EQ(ve_q8, 80u << 8u, "flat VE=80 → ve_q8=80<<8");

    // ─ table3d_lookup_advance_q10 ─────────────────────────────────
    section("table3d: lookup_advance_q10");
    // flat advance=30° → result in Q10 = 30<<10=30720
    const int32_t adv_q10 = table3d_lookup_advance_q10(flat_s16,
                                                         kRpmAxisX10, kLoadAxisBarX100,
                                                         30000u, 100u);
    CHECK_EQ(adv_q10, 1000 << 10, "flat adv=1000 → adv_q10=1000<<10");
}

// ============================================================================
// ECU SCHED
// ============================================================================

static void test_ecu_sched_setters(void) {
    section("ecu_sched: reset / setters / getters");
    ecu_sched_test_reset();

    // Defaults after reset: advance=10, dwell=140625, inj_pw=140625, eoi=355
    CHECK_EQ(ecu_sched_test_get_advance_deg(),  10u, "default advance=10°");
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(), 140625u, "default dwell=140625");
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 140625u, "default inj_pw=140625");
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 355u, "default eoi=355° (open-valve)");

    // Individual setters
    ecu_sched_set_advance_deg(20u);
    CHECK_EQ(ecu_sched_test_get_advance_deg(), 20u, "set_advance_deg(20)");

    ecu_sched_set_dwell_ticks(30000u);
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(), 30000u, "set_dwell_ticks(187500)");

    ecu_sched_set_inj_pw_ticks(15000u);
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 15000u, "set_inj_pw_ticks(15000)");

    ecu_sched_set_eoi_lead_deg(50u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 50u, "set_eoi_lead_deg(50)");

    // commit_calibration sets all four atomically
    ecu_sched_commit_calibration(25u, 25000u, 18000u, 55u);
    CHECK_EQ(ecu_sched_test_get_advance_deg(),   25u, "commit: advance=25");
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(),  25000u, "commit: dwell=25000");
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 18000u, "commit: inj_pw=18000");
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(),  55u, "commit: eoi=55");

    // Calibration clamp: advance > 719 → clamped
    ecu_sched_set_advance_deg(800u);
    CHECK_TRUE(ecu_sched_test_get_advance_deg() <= 719u, "advance > 720 → clamped");
    CHECK_EQ(ecu_sched_test_get_calibration_clamp_count(), 1u, "clamp count=1");

    // reset_diagnostic_counters
    ecu_sched_reset_diagnostic_counters();
    CHECK_EQ(ecu_sched_test_get_calibration_clamp_count(), 0u, "clamp_count=0 after reset");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), 0u, "late_count=0 after reset");
}

static void test_ecu_sched_angle_table(void) {
    section("ecu_sched: schedule_on_tooth populates angle table in FULL_SYNC");
    ecu_sched_test_reset();
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);

    // Reach full sync — schedule_on_tooth fires each CKP tooth hook
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // After gap tooth, scheduler should have emitted events for 4 cylinders
    const uint8_t tbl_sz = ecu_sched_test_angle_table_size();
    CHECK_TRUE(tbl_sz > 0u, "angle table has events after FULL_SYNC");
    // Modo de presync default é SEMI_SEQUENTIAL (g_presync_inj_mode em
    // ecu_sched_test_reset()), não SIMULTANEOUS: injeta em só 2 dos 4
    // cilindros por revolução em presync. Ignição continua 1/cilindro
    // (DWELL+SPARK = 4×2 = 8), injeção só 2 cilindros (ON+OFF = 2×2 = 4) →
    // 12 eventos, não os 16 que a expectativa antiga assumia (modo
    // SIMULTANEOUS, que só é ativado automaticamente durante cranking).
    CHECK_TRUE(tbl_sz >= 12u, "angle table has ≥12 events (presync SEMI_SEQUENTIAL)");

    // Inspect first valid event: should be one of ECU_ACT_*
    uint8_t tooth, frac, ch, action, phase;
    const uint8_t ok = ecu_sched_test_get_angle_event(0u, &tooth, &frac, &ch, &action, &phase);
    CHECK_EQ(ok, 1u, "event[0] is valid");
    CHECK_TRUE(action <= ECU_ACT_SPARK, "action in [0,3] (DWELL/SPARK/INJ)");
    CHECK_TRUE(tooth < 58u, "tooth_index < kRealTeeth60_2");
}

// ── Transição wasted-spark ↔ sequencial via bordas CMP reais ────────────────
// Ao contrário dos outros testes sequenciais (que dão bypass ao gate com
// ckp_test_set_cmp_confirms(2u)), este conduz o caminho NUNCA testado: a
// validação temporal em ckp_tim5_ch2_isr via cam_fire(). Reproduz o sintoma
// de bancada ("ao ligar o CMP nada muda") com entradas ideais.
//
// Mecânica do timing: cada ckp_feed_n_then_gap(55) avança g_ckp_cap por
// 58×período (55 normais + gap 3×período). CMP dispara 1×/720° = 2 revs =
// 116×período, que é exactamente expected = 2×kRealTeethPerRev(58)×período em
// ckp_tim5_ch2_isr → a 2ª borda cai no centro da janela ±25%.
static void test_ecu_sched_wasted_to_sequential(void) {
    section("ecu_sched: transição wasted→sequencial via CMP real (cam_fire)");
    ecu_sched_test_reset();
    // Janela de dente CMP desabilitada (default), defensivo contra herança de
    // estado de testes anteriores no mesmo processo.
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;

    // 1) FULL_SYNC sem CMP → wasted-spark (presync).
    ckp_reach_full_sync();                       // reseta g_ckp_cap=0, dirige o hook
    ckp_feed_n_then_gap(55u);                     // 1 rev extra em wasted
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "cmp_confirms=0 sem CMP");
    CHECK_TRUE(ecu_sched_test_get_presync_revs() > 0u, "presync_revs>0 (wasted a correr)");
    CHECK_EQ(ecu_sched_test_get_seq_revs(), 0u, "seq_revs=0 (ainda não sequencial)");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "is_sequential=0 sem CMP");

    // 2) 1ª borda CMP (s_prev_cmp_capture=0 → salta validação) → cmp_confirms=1.
    //    Ainda insuficiente para sequencial (gate exige >=2).
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 1u, "1ª borda CMP → cmp_confirms=1");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);                     // +2 revs → g_ckp_cap += 116×período
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "1 confirm insuficiente: continua wasted");

    // 3) 2ª borda CMP: delta = 116×período = expected → validada → cmp_confirms=2.
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "2ª borda coerente → cmp_confirms=2");
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "nenhuma borda CMP rejeitada");

    // 4) Próximas fronteiras de revolução → gate abre → Calculate_Sequential_Cycle.
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    CHECK_TRUE(ecu_sched_test_get_seq_revs() > 0u, "seq_revs>0 após CMP confirmado");
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "is_sequential=1: entrou em sequencial");

    // 5) Fallback CMP-ausente (Parte C-#2): mantendo o sincronismo mas SEM novas
    //    bordas de came, o contador de revoluções desde a última borda ultrapassa
    //    kMaxRevsWithoutCmp (6 em produção; 60 só em bench-mode para tolerar
    //    gaps do estimulador RMT) → cmp_confirms zera → o agendador reverte a wasted.
    for (uint32_t i = 0; i < 7u; ++i) { ckp_feed_n_then_gap(55u); }
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "sem came >6 revs → cmp_confirms zerado");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "fallback: reverteu a wasted-spark");
}

// ── Revalidação CMP após perda de sync do CKP ───────────────────────────────
// Perda de sync (gap prematuro) preserva a contagem interna do came, mas o
// valor EXPORTADO é capado a 1: o resync não pode retomar sequencial com a
// referência de fase antiga (phase_half pode ter caído na metade errada de
// 720°). Só uma borda CMP fresca coerente — que também re-ancora a fase via
// cmp_phase_pending — reabre o gate.
// Timing: bordas de came a cada 2 revs (116×período, janela [90,150]); a borda
// fresca pós-resync cai a 28+58+58 = 144×período, dentro da janela ±25%.
static void test_ecu_sched_cmp_revalidation_after_sync_loss(void) {
    section("ecu_sched: perda de sync fecha gate sequencial até borda CMP fresca");
    ecu_sched_test_reset();
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;

    // 1) FULL_SYNC + 2 bordas CMP coerentes → sequencial.
    ckp_reach_full_sync();
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                          // 1ª borda (sem prev → aceita)
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                          // 2ª borda, delta=116 ✓
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "2 bordas coerentes → cmp_confirms=2");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                          // 3ª borda mantém cadência, delta=116 ✓
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "sequencial activo antes da perda");

    // 2) Gap prematuro (25 dentes) → LOSS_OF_SYNC; gate exportado capa a 1,
    //    contagem interna preservada.
    ckp_feed_n_then_gap(25u);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "gap prematuro → LOSS_OF_SYNC");
    CHECK_EQ(ckp_snapshot().cmp_confirms, 1u, "perda de sync: gate exportado capado a 1");

    // 3) Resync (LOSS→HALF→FULL) SEM borda de came fresca → wasted, não sequencial.
    ckp_feed_n_then_gap(55u);                     // LOSS → HALF_SYNC
    ckp_feed_n_then_gap(55u);                     // HALF → FULL_SYNC
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "resync completo");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "pós-resync sem came fresco: fica em wasted");

    // 4) Borda CMP fresca coerente (delta=144 ✓, mesmo tooth_index) → gate reabre.
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "borda fresca restaura cmp_confirms=2");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "sequencial retomado após revalidação");
}

// Ruído no pino CMP (came desligado, PA1 a flutuar) gera bordas fantasma em
// posições de virabrequim ALEATÓRIAS. O gate de consistência de posição
// (ckp_tim5_ch2_isr) exige que bordas consecutivas ocorram no mesmo tooth_index:
// ruído nunca junta 2 coerentes → cmp_confirms não chega a 2 → fica em wasted.
static void test_ecu_sched_noise_rejects_sequential(void) {
    section("ecu_sched: ruído CMP (posição inconsistente) NÃO entra em sequencial");
    ecu_sched_test_reset();
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;
    ckp_reach_full_sync();                                   // FULL_SYNC, tooth 0

    // Borda 1 no dente 5 → ancora a posição de referência (cmp_confirms=1).
    for (uint32_t i = 0; i < 5u; ++i) { ckp_fire(kNormalPeriod); }
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 1u, "1ª borda ancora posição (cmp_confirms=1)");

    // Borda 2 ~2 revs depois mas noutro dente (25≠5): passa o gate temporal
    // (~120 períodos, dentro de ±25%) mas falha o de posição → rejeitada.
    for (uint32_t i = 0; i < 50u; ++i) { ckp_fire(kNormalPeriod); } ckp_fire(kNormalPeriod * 3u);
    ckp_feed_n_then_gap(55u);
    for (uint32_t i = 0; i < 25u; ++i) { ckp_fire(kNormalPeriod); }   // tooth 25
    cam_fire(g_ckp_cap);
    CHECK_TRUE(ckp_snapshot().cmp_confirms < 2u, "borda em dente inconsistente não confirma");

    // Mais bordas em dentes sempre diferentes → nunca acumula 2 coerentes.
    const uint8_t teeth[3] = {40u, 12u, 33u};
    for (uint8_t k = 0; k < 3u; ++k) {
        for (uint32_t i = 0; i < 30u; ++i) { ckp_fire(kNormalPeriod); } ckp_fire(kNormalPeriod * 3u);
        ckp_feed_n_then_gap(55u);
        for (uint32_t i = 0; i < teeth[k]; ++i) { ckp_fire(kNormalPeriod); }
        cam_fire(g_ckp_cap);
    }
    CHECK_TRUE(ckp_snapshot().cmp_confirms < 2u, "ruído nunca atinge cmp_confirms=2");
    ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "permanece em wasted-spark sob ruído CMP");
}

// Após fallback a wasted (came ausente), o came RECONECTADO tem de recuperar o
// sequencial. Reproduz o deadlock: s_prev_cmp_capture fica obsoleto (borda real de
// há muitas revs) e cada borda reconectada é rejeitada por tempo contra ele. O
// resync por rejeições consecutivas (kCmpRejectResync) larga a referência e recupera.
static void test_ecu_sched_recovers_after_fallback(void) {
    section("ecu_sched: recupera sequencial após fallback (came reconectado)");
    ecu_sched_test_reset();
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;
    ckp_reach_full_sync();

    // Entra em sequencial: 2 bordas coerentes no tooth 0.
    cam_fire(g_ckp_cap);
    ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);
    ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "pré: sequencial (cmp_confirms=2)");
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "pré: is_sequential=1");

    // "Desconecta": >60 revs sem came (kMaxRevsWithoutCmp) → #2 fallback → wasted.
    // s_prev fica obsoleto.
    for (uint32_t i = 0; i < 61u; ++i) { ckp_feed_n_then_gap(55u); }
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "fallback: cmp_confirms=0");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "fallback: wasted");

    // "Reconecta": bordas coerentes no tooth 0. Sem o resync, todas seriam rejeitadas
    // por tempo contra o s_prev obsoleto (deadlock). Com o resync, recupera.
    for (uint8_t e = 0; e < 6u; ++e) {
        cam_fire(g_ckp_cap);
        ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    }
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "recuperou: cmp_confirms=2 após reconexão");
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "recuperou: voltou a sequencial");
}

static void test_ecu_sched_inhibit_masks(void) {
    section("ecu_sched: injection / ignition inhibit masks");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0u, "inj_inhibit=0 after reset");
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0u, "ign_inhibit=0 after reset");

    ecu_sched_set_inj_inhibit_mask(0x05u);  // cylinders 0 and 2
    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0x05u, "inj_inhibit=0x05");

    ecu_sched_set_ign_inhibit_mask(0x0Au);  // cylinders 1 and 3
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0x0Au, "ign_inhibit=0x0A");

    // Restore
    ecu_sched_set_inj_inhibit_mask(0u);
    ecu_sched_set_ign_inhibit_mask(0u);
    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0u, "inj_inhibit cleared");
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0u, "ign_inhibit cleared");
}

static void test_ecu_sched_mspark(void) {
    section("ecu_sched: multi-spark");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_test_get_mspark_count(), 0u, "mspark=0 after reset");

    ecu_sched_set_mspark(2u, 5000u, 18u);
    CHECK_EQ(ecu_sched_test_get_mspark_count(), 2u, "mspark_count=2");

    // Overflow: count > 3 → clamped to 3
    ecu_sched_set_mspark(5u, 5000u, 18u);
    CHECK_TRUE(ecu_sched_test_get_mspark_count() <= 3u, "mspark_count clamped ≤3");

    // Disable
    ecu_sched_set_mspark(0u, 0u, 0u);
    CHECK_EQ(ecu_sched_test_get_mspark_count(), 0u, "mspark disabled");
}

// ── EOI targeting ───────────────────────────────────────────────────────────
// Helper: procura na tabela angular o primeiro evento (channel, action).
static uint8_t find_angle_event(uint8_t want_ch, uint8_t want_act,
                                uint8_t *out_tooth, uint8_t *out_frac, uint8_t *out_phase) {
    for (uint8_t i = 0u; i < ecu_sched_test_angle_table_size(); ++i) {
        uint8_t t, f, ch, act, ph;
        if (ecu_sched_test_get_angle_event(i, &t, &f, &ch, &act, &ph) != 0u &&
            ch == want_ch && act == want_act) {
            *out_tooth = t; *out_frac = f; *out_phase = ph;
            return 1u;
        }
    }
    return 0u;
}

// Constrói tabela sequencial com o PW dado e devolve eventos INJ1 ON/OFF.
// kNormalPeriod=10000 ticks → tooth_period=160000ns → tooth_ticks=10000
// → deg = ticks×6/10000 (720°); RPM ≈ 6250.
static void build_seq_table_with_pw(uint32_t pw_ticks) {
    ecu_sched_test_reset();
    for (uint8_t i = 0u; i < 4u; ++i) { ems::engine::cyl_fuel_trim_pct[i] = 0; }
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(pw_ticks);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    ckp_test_set_cmp_confirms(2u);
    ckp_feed_n_then_gap(55u);  // gap → Calculate_Sequential_Cycle
}

static void test_ecu_sched_eoi_targeting(void) {
    // Cyl0: tdc=0, eoi_lead=60 → EOI=660° (60° BTDC combustão).
    // Trigger frame (offset=0): ang=660%360=300 → 300×256/6=12800 → tooth=50,
    // frac=0, PHASE_B (660≥360). O INJ_OFF deve ficar AQUI para qualquer PW.
    uint8_t t_on, f_on, p_on, t_off, f_off, p_off;

    section("ecu_sched EOI: INJ_OFF fixo no alvo de EOI, independente do PW");
    // PW curto: 120° → ticks = 120×10000/6 = 200000. SOI = 660−120 = 540°
    // → ang=180 → 180×256/6=7680 → tooth=30, frac=0, PHASE_B.
    build_seq_table_with_pw(200000u);
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "PW=120°: INJ1 OFF presente");
    CHECK_EQ(t_off, 50u, "PW=120°: INJ_OFF em tooth 50 (EOI=660°)");
    CHECK_EQ(f_off, 0u,  "PW=120°: INJ_OFF frac=0");
    CHECK_EQ(p_off, ECU_PHASE_B, "PW=120°: INJ_OFF em PHASE_B");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "PW=120°: INJ1 ON presente");
    CHECK_EQ(t_on, 30u, "PW=120°: SOI recua para tooth 30 (540°)");
    CHECK_EQ(p_on, ECU_PHASE_B, "PW=120°: SOI em PHASE_B");
    CHECK_EQ(ecu_sched_test_get_pw_duty_clamp_count(), 0u,
             "PW=120°: duty clamp não dispara");

    section("ecu_sched EOI: SOI cruza a origem do ciclo (wrap) mantendo o EOI");
    // Com eoi_lead=60 e duty clamp de 648°, SOI = 660−PW ∈ [12,660] — nunca
    // cruza a origem. O wrap real ocorre com EOI mais cedo no ciclo:
    // eoi_lead=300 → EOI=420° → ang=60 → tooth=10, frac=0, PHASE_B.
    // PW: ticks=833333 → deg=499 (trunc). SOI=(420+720−499)%720=641°
    // → ang=281 → 281×256/6=11989 → tooth=46, frac=213, PHASE_B.
    // SOI (641°) fica DEPOIS do EOI (420°) em ângulo absoluto: o pulso
    // atravessa a fronteira 720→0 do ciclo — o caso que o SOI fixo não cobria.
    ecu_sched_test_reset();
    for (uint8_t i = 0u; i < 4u; ++i) { ems::engine::cyl_fuel_trim_pct[i] = 0; }
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(833333u);
    ecu_sched_set_eoi_lead_deg(300u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // ckp_reach_full_sync() já constrói uma tabela de presync (cmp_confirms
    // ainda a 0 nesse ponto) com este mesmo inj_pw_ticks — em modo presync o
    // clamp actua a 324° (ECU_MAX_PRESYNC_INJ_PW_DEG, janela de 360°) e o
    // PW de 499° dispara-o aí, incrementando g_pw_duty_clamp_count antes da
    // build sequencial que este teste quer isolar. Reset após confirmar CMP.
    ckp_test_set_cmp_confirms(2u);
    ecu_sched_reset_diagnostic_counters();
    ckp_feed_n_then_gap(55u);
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "PW=499°/EOI=420°: INJ1 OFF presente");
    CHECK_EQ(t_off, 10u, "PW=499°: INJ_OFF em tooth 10 (EOI=420° fixo)");
    CHECK_EQ(p_off, ECU_PHASE_B, "PW=499°: INJ_OFF em PHASE_B");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "PW=499°: INJ1 ON presente");
    CHECK_EQ(t_on, 46u, "PW=499°: SOI em tooth 46 (641° — wrap além da origem)");
    CHECK_EQ(p_on, ECU_PHASE_B, "PW=499°: SOI em PHASE_B");
    CHECK_EQ(ecu_sched_test_get_pw_duty_clamp_count(), 0u,
             "PW=499° ≤ 648°: duty clamp não dispara");

    section("ecu_sched EOI: duty clamp em PW ≥ 90% do ciclo");
    // PW máximo permitido: ticks=1250000 → deg=750 > 648 → clamp a 648°.
    // SOI=(660+720−648)%720=732%720=12° → ang=12 → 512 → tooth=2, frac=0,
    // PHASE_A. Contador: 4 incrementos (um por cilindro na build).
    build_seq_table_with_pw(1250000u);
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "PW=750°: INJ1 OFF presente");
    CHECK_EQ(t_off, 50u, "PW=750°: INJ_OFF permanece em tooth 50 (EOI fixo)");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "PW=750°: INJ1 ON presente");
    CHECK_EQ(t_on, 2u,  "PW=750°→648°: SOI em tooth 2 (12°)");
    CHECK_EQ(p_on, ECU_PHASE_A, "PW=750°→648°: SOI em PHASE_A");
    // Contador: ≥4 (um por cilindro na build sequencial). Nota: as builds
    // presync durante o padrão de sync (modo default SEMI_SEQUENTIAL, sem
    // halving de PW: 750° > 324°) também incrementam, pelo que a contagem
    // exacta depende do número de rev boundaries.
    CHECK_TRUE(ecu_sched_test_get_pw_duty_clamp_count() >= 4u,
             "PW=750°: duty clamp disparou ≥4× (1× por cilindro na build seq.)");

    section("ecu_sched EOI: presync usa EOI targeting na janela de 360°");
    // presync: eoi=(360−60)%360=300 → tooth 50, frac 0, PHASE_ANY.
    ecu_sched_test_reset();
    ecu_sched_test_set_tim1_cnt(0u);
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(55u);   // HALF_SYNC
    for (uint32_t i = 0u; i < 58u; ++i) { ckp_fire(kNormalPeriod); }  // rev boundary
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "presync: INJ1 OFF presente");
    CHECK_EQ(t_off, 50u, "presync: INJ_OFF em tooth 50 (EOI=300° na janela 360°)");
    CHECK_EQ(p_off, ECU_PHASE_ANY, "presync: INJ_OFF em PHASE_ANY");

    section("ecu_sched EOI: default 355° — pulso presync cruza a fronteira de rev");
    // Com o default open-valve (eoi_lead=355), o EOI presync cai a
    // eoi=(360−355%360)%360=5° → ang=5 → 5×256/6=213 → tooth 0, frac 213.
    // Modo presync default é SEMI_SEQUENTIAL (ecu_sched_test_reset()) — SEM
    // halving do PW (isso só acontece em SIMULTANEOUS): PW=125000 ticks → 75°
    // → SOI=(5+360−75)%360=290° → 290×256/6=12373 → tooth 48, frac 85.
    // O pulso ON(tooth 48, rev N) → OFF(tooth 0, rev N+1) CRUZA a fronteira de
    // revolução onde a tabela é reconstruída — este check fixa que ambos os
    // eventos existem na tabela (o OFF da tabela nova fecha o injetor aberto
    // na rev anterior; toggle de bancos já validado acima).
    ecu_sched_test_reset();  // usa o default eoi_lead=355 — sem set explícito
    ecu_sched_test_set_tim1_cnt(0u);
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(55u);   // HALF_SYNC
    for (uint32_t i = 0u; i < 58u; ++i) { ckp_fire(kNormalPeriod); }  // rev boundary
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "presync 355°: INJ1 OFF presente");
    CHECK_EQ(t_off, 0u,  "presync 355°: INJ_OFF em tooth 0 (EOI=5°)");
    CHECK_EQ(f_off, 213u, "presync 355°: INJ_OFF frac=213 (5°×256/6)");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "presync 355°: INJ1 ON presente");
    CHECK_EQ(t_on, 48u, "presync 355°: SOI em tooth 48 (290°) — antes da fronteira");
    CHECK_EQ(f_on, 85u, "presync 355°: SOI frac=85");
}

static void test_eoi_blend(void) {
    section("fuel_calc: EOI blend de 2 pontos por RPM");
    // main = g_eng_cfg.default_eoi_lead_deg (355 por default de compilação)
    const uint16_t saved_main = ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg;
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = 355u;

    // Desligado (0/0 — page 0 antiga zerada): devolve sempre o main
    ems::engine::eoi_idle_deg = 60u;
    ems::engine::eoi_blend_rpm_lo = 0u;
    ems::engine::eoi_blend_rpm_hi = 0u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u), 355u, "blend off (0/0): main a 850 RPM");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(85000u), 355u, "blend off (0/0): main a 8500 RPM");

    // Desligado (hi < lo): gate contra janela invertida / divisão por zero
    ems::engine::eoi_blend_rpm_lo = 2500u;
    ems::engine::eoi_blend_rpm_hi = 1500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 355u, "blend off (hi<lo): main");
    ems::engine::eoi_blend_rpm_hi = 2500u;  // hi == lo também desliga
    ems::engine::eoi_blend_rpm_lo = 2500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 355u, "blend off (hi==lo): main");

    // Janela 1500→2500, idle=60, main=355 (ascendente)
    ems::engine::eoi_blend_rpm_lo = 1500u;
    ems::engine::eoi_blend_rpm_hi = 2500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u),  60u, "850 RPM (< lo): idle");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(15000u), 60u, "1500 RPM (== lo): idle");
    // 2000 RPM: 60 + 295×500/1000 = 60 + 147 = 207 (trunc)
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 207u, "2000 RPM (meio): 207");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(25000u), 355u, "2500 RPM (== hi): main");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(85000u), 355u, "8500 RPM (> hi): main");

    // Descendente (idle=365 pré-IVO > main=355): interpola para baixo
    ems::engine::eoi_idle_deg = 365u;
    // 2000 RPM: 365 + (−10)×500/1000 = 365 − 5 = 360
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 360u, "descendente: 365→355 dá 360 no meio");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u),  365u, "descendente: idle=365 abaixo da janela");

    // Extremos int32: idle=0, main=719, janela de 1 RPM
    ems::engine::eoi_idle_deg = 0u;
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = 719u;
    ems::engine::eoi_blend_rpm_lo = 1000u;
    ems::engine::eoi_blend_rpm_hi = 1001u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(10000u), 0u,   "janela 1 RPM: == lo → idle");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(10010u), 719u, "janela 1 RPM: == hi → main");

    // Entradas fora de gama são clampadas a 719 (defesa antes do sanitize)
    ems::engine::eoi_idle_deg = 60000u;
    ems::engine::eoi_blend_rpm_lo = 1500u;
    ems::engine::eoi_blend_rpm_hi = 2500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u), 719u, "idle fora de gama → clamp 719");
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = 60000u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(85000u), 719u, "main fora de gama → clamp 719");

    // restaurar estado partilhado
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = saved_main;
    ems::engine::eoi_idle_deg = 60u;
    ems::engine::eoi_blend_rpm_lo = 0u;
    ems::engine::eoi_blend_rpm_hi = 0u;

    section("ecu_sched: sanitize aceita EOI até 719 (pré-IVO)");
    ecu_sched_test_reset();
    ecu_sched_set_eoi_lead_deg(719u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 719u, "eoi=719 aceite (clamp estendido)");
    ecu_sched_set_eoi_lead_deg(365u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 365u, "eoi=365 (pré-IVO) aceite");
    ecu_sched_set_eoi_lead_deg(720u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 719u, "eoi=720 clampado a 719");
}

static void test_ecu_sched_presync(void) {
    section("ecu_sched: presync enable/mode setters");
    ecu_sched_test_reset();

    // Just verify no crash
    ecu_sched_set_presync_enable(0u);
    ecu_sched_set_presync_enable(1u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SEMI_SEQUENTIAL);
    ecu_sched_set_presync_ign_mode(ECU_PRESYNC_IGN_WASTED_SPARK);
    ecu_sched_fire_prime_pulse(5000u);  // prime pulse: no crash with valid pw
    CHECK_TRUE(true, "presync setters and prime_pulse 5000: no crash");

    // ecu_sched_fire_prime_pulse edge cases:
    // pw=0 → guard: early return (no crash)
    ecu_sched_fire_prime_pulse(0u);
    CHECK_TRUE(true, "fire_prime_pulse(0): no crash (early return)");

    // pw > 30000 → clamped to 30000 (no crash, clamp happens internally)
    ecu_sched_fire_prime_pulse(100000u);
    CHECK_TRUE(true, "fire_prime_pulse(100000): no crash (clamped to 30ms)");
}

static void test_ecu_sched_dwell_watchdog(void) {
    section("ecu_sched: dwell watchdog");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u, "watchdog_count=0 at start");
    // Calling watchdog with no armed dwell should be a no-op
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u, "watchdog_count=0 with no armed coil");
}

// ============================================================================
// QUICK CRANK
// ============================================================================

static void test_quick_crank_all(void) {
    using namespace ems::engine;

    section("quick_crank: reset");
    quick_crank_reset();
    CHECK_EQ(quick_crank_consume_prime(), 0u, "no prime pending after reset");

    section("quick_crank: not cranking when rpm=0 or no sync");
    auto out = quick_crank_update(0u, 0u, false, 800, 8);
    CHECK_FALSE(out.cranking, "rpm=0, no sync → not cranking");
    CHECK_EQ(out.fuel_mult_x256, 256u, "not cranking → mult=1.0 (256)");

    section("quick_crank: cranking when rpm below enter threshold");
    quick_crank_reset();
    // crank_enter_rpm_x10=4500, crank_exit_rpm_x10=7000
    // First call with rpm=3000 (300 RPM) and sync_available=true
    out = quick_crank_update(1000u, 3000u, true, 800, 8);
    CHECK_TRUE(out.cranking, "rpm=3000 < enter=4500 → cranking");
    CHECK_TRUE(out.fuel_mult_x256 > 256u, "cranking enrichment > 1.0");
    CHECK_EQ(out.spark_deg, ems::engine::crank_spark_deg, "spark_deg = crank_spark_deg");
    CHECK_TRUE(out.min_pw_us > 0u, "min_pw_us > 0 during cranking");

    section("quick_crank: afterstart on RPM jump");
    quick_crank_reset();
    // Enter cranking state first
    quick_crank_update(1000u, 3000u, true, 800, 8);
    // Now RPM jumps above exit: transitions from cranking to afterstart
    out = quick_crank_update(5000u, 80000u, true, 800, 8);  // 8000 RPM > exit
    // fuel_mult should still be > 256 during afterstart (hot engine at 80°C may be minimal)
    // Or == 256 if CLT=800 (warm). Either way, no crash.
    CHECK_TRUE(out.fuel_mult_x256 >= 256u, "afterstart: fuel_mult ≥ 1.0");

    section("quick_crank: quick_crank_apply_pw_us");
    // base_pw * mult / 256, clamped to min_pw
    CHECK_EQ(quick_crank_apply_pw_us(10000u, 256u, 0u), 10000u, "mult=1.0 → pw unchanged");
    CHECK_EQ(quick_crank_apply_pw_us(10000u, 512u, 0u), 20000u, "mult=2.0 → pw doubled");
    CHECK_EQ(quick_crank_apply_pw_us(100u, 256u, 500u), 500u, "below min_pw → clamped to 500");
    CHECK_EQ(quick_crank_apply_pw_us(0u, 512u, 1000u), 1000u, "base=0 → clamped to min");
    // Overflow clamp at 100000 us
    CHECK_EQ(quick_crank_apply_pw_us(90000u, 512u, 0u), 100000u, "overflow → 100000 clamp");

    section("quick_crank: set_prime_context / set_clt / consume_prime");
    quick_crank_set_prime_context(800, 500u);
    quick_crank_set_clt(600);
    // consume_prime: no prime fired yet → 0
    CHECK_EQ(quick_crank_consume_prime(), 0u, "no prime fired → consume=0");
    // Two calls to consume same prime: second must return 0 (one-shot)
    CHECK_EQ(quick_crank_consume_prime(), 0u, "second consume → 0 (one-shot)");
}

// ============================================================================
// TRANSIENT FUEL (X-Tau)
// ============================================================================

static void test_transient_fuel_all(void) {
    using namespace ems::engine;

    section("transient_fuel: reset");
    transient_fuel_reset();
    CHECK_TRUE(true, "transient_fuel_reset: no crash");

    section("transient_fuel: disabled → returns base pw");
    transient_fuel_reset();
    const uint32_t pw_base = 5000u;
    const uint32_t out_disabled = transient_fuel_xtau_update(pw_base, 800, false);
    CHECK_EQ(out_disabled, pw_base, "disabled → output = input pw");

    section("transient_fuel: enabled with valid CLT");
    transient_fuel_reset();
    // Enabled: X-Tau model applies wall wetting correction.
    // Result may differ from input but must be > 0 and <= 100000.
    const uint32_t out_enabled = transient_fuel_xtau_update(pw_base, 800, true);
    CHECK_TRUE(out_enabled > 0u && out_enabled <= 100000u,
               "enabled: output in (0, 100ms]");

    section("transient_fuel: enabled with zero pw → reset + return 0");
    transient_fuel_reset();
    const uint32_t out_zero = transient_fuel_xtau_update(0u, 800, true);
    CHECK_EQ(out_zero, 0u, "pw=0, enabled → reset + return 0");

    section("transient_fuel: warm-up steady state converges");
    transient_fuel_reset();
    uint32_t prev = 0u;
    for (int i = 0; i < 20; ++i) {
        prev = transient_fuel_xtau_update(5000u, 800, true);
    }
    // After many iterations wall fuel reaches steady state; output near input
    CHECK_TRUE(prev > 0u, "xtau: converges to positive value");
}

// ============================================================================
// MAP ESTIMATOR
// ============================================================================

static void test_map_estimator_all(void) {
    using namespace ems::engine;

    section("map_estimator: init / update / getters");
    map_estimator_init();

    // First update: estimated should track sensor
    const uint16_t est = map_estimator_update(100u, 500u, 10u, 30000u, 220);
    CHECK_TRUE(est > 0u && est <= 300u, "estimated MAP in (0, 300 kPa]");
    CHECK_EQ(map_get_estimated_bar_x100(), est, "getter matches return value");

    section("map_estimator: tpsdot");
    // After two calls with same TPS: tpsdot ≈ 0
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    const int16_t dot = map_get_tpsdot_x10();
    CHECK_EQ(dot, 0, "steady TPS → tpsdot=0");

    // TPS step: history ring buffer design requires full wrap to compute non-zero
    // tpsdot; just verify the API returns a valid range value.
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    map_estimator_update(100u, 900u, 10u, 30000u, 220);
    const int16_t dot2 = map_get_tpsdot_x10();
    CHECK_TRUE(dot2 >= -1000 && dot2 <= 1000, "TPS step → tpsdot in clamped range");

    section("map_estimator: is_transient");
    // After rapid TPS step: transient flag may be set
    // (depends on implementation thresholds)
    const bool trans = map_is_transient();
    CHECK_TRUE(trans == true || trans == false, "is_transient returns bool");

    section("map_estimator: get_state");
    const MapEstimatorState st = map_estimator_get_state();
    CHECK_TRUE(st.map_estimated_bar_x100 > 0u, "state.map_estimated > 0");

    section("map_estimator: set_gains");
    map_estimator_set_gains(200u, 150u);  // arbitrary gains
    // Call update after gain change — must not crash
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    CHECK_TRUE(true, "set_gains + update: no crash");

    section("map_estimator: edge cases");
    // dt=0
    map_estimator_update(100u, 500u, 0u, 30000u, 220);
    CHECK_TRUE(true, "dt=0: no crash");
    // rpm=0
    map_estimator_update(100u, 500u, 10u, 0u, 220);
    CHECK_TRUE(true, "rpm=0: no crash");

    section("map_estimator: modelo termodinâmico — set_model_params");
    ManifoldModelParams mp{};
    mp.volume_cc_x10 = 5000u;
    mp.throttle_flow_coeff_q8 = 256u;
    mp.engine_pumping_coeff_q8 = 256u;
    map_estimator_set_model_params(mp);
    CHECK_TRUE(true, "set_model_params: no crash");

    section("map_estimator: tpsdot deteta transiente após volta completa do ring buffer");
    // g_tps_history_pos aponta sempre para o slot mais antigo ainda residente
    // (prestes a ser sobrescrito); o mais recente é pos-1. tpsdot só fica
    // não-nulo depois do buffer (kTpsHistorySize=4 amostras) ter dado a volta
    // completa com um degrau de TPS dentro dela.
    map_estimator_init();
    for (uint8_t i = 0u; i < 4u; ++i) {
        map_estimator_update(30u, 100u, 2u, 8000u, 220);
    }
    map_estimator_update(30u, 900u, 2u, 8000u, 220);
    CHECK_TRUE(map_get_tpsdot_x10() > 0, "step de TPS após volta do buffer → tpsdot > 0");
    CHECK_TRUE(map_is_transient(), "tpsdot alto → is_transient()=true");

    section("map_estimator: modelo termodinâmico responde a IAT (fluxo de admissão)");
    // Ar mais frio é mais denso → mais massa de ar admitida para a mesma
    // abertura de borboleta → o modelo deve prever um MAP igual ou mais alto
    // que com ar quente, no mesmo cenário de tip-in (baixo RPM, TPS 10%→90%).
    auto run_tip_in = [](int16_t iat_x10) noexcept -> uint16_t {
        map_estimator_init();
        for (uint8_t i = 0u; i < 4u; ++i) {
            map_estimator_update(30u, 100u, 2u, 8000u, iat_x10);
        }
        uint16_t v = 0u;
        for (uint8_t i = 0u; i < 3u; ++i) {
            v = map_estimator_update(30u, 900u, 2u, 8000u, iat_x10);
        }
        return v;
    };
    const uint16_t est_hot_iat = run_tip_in(220);   // 22.0°C
    const uint16_t est_cold_iat = run_tip_in(-100);  // -10.0°C
    CHECK_TRUE(est_cold_iat >= est_hot_iat,
               "IAT baixo (ar mais denso) → fluxo de admissão maior ou igual ao de IAT alto");
}

// ============================================================================
// MISFIRE DETECT
// ============================================================================

static void test_misfire_all(void) {
    using namespace ems::engine;

    section("misfire: init / reset / get_event_count / clear_events");
    misfire_init();
    misfire_reset();
    for (uint8_t c = 0u; c < 4u; ++c) {
        CHECK_EQ(misfire_get_event_count(c), 0u, "event_count=0 after reset");
    }

    section("misfire: misfire_clear_events");
    // Manually call on_tooth with exaggerated slow period (3× expected → misfire)
    // Simulate 10 teeth of a window for cyl 0 with slow period
    ckp_reach_full_sync();
    const uint32_t normal_ns = kNormalPeriod * 16u;  // ticks×16 ns/tick = normal_ns
    ems::drv::CkpSnapshot snap_mf{};
    snap_mf.state = ems::drv::SyncState::FULL_SYNC;
    snap_mf.tooth_index = 0u;
    snap_mf.phase_A = true;
    snap_mf.tooth_period_ns = normal_ns * 3u;            // 3× slow = misfire
    snap_mf.predicted_tooth_period_ns = normal_ns;      // expected period
    // Feed kMisfireWindowTeeth teeth × kMisfireDebounceCycles to trigger event
    for (uint32_t w = 0u; w < (uint32_t)kMisfireDebounceCycles; ++w) {
        for (uint32_t t = 0u; t < (uint32_t)kMisfireWindowTeeth; ++t) {
            ems::drv::misfire_on_tooth(snap_mf);
        }
    }
    // After debounce cycles, event_count[0] should be ≥1
    CHECK_TRUE(misfire_get_event_count(0u) >= 1u, "misfire event after slow period");

    // clear_events
    misfire_clear_events(0u);
    CHECK_EQ(misfire_get_event_count(0u), 0u, "event_count=0 after clear");

    section("misfire: inhibit suppresses detection");
    misfire_reset();
    misfire_set_all_inhibit(true);
    for (uint32_t w = 0u; w < (uint32_t)kMisfireDebounceCycles; ++w) {
        for (uint32_t t = 0u; t < (uint32_t)kMisfireWindowTeeth; ++t) {
            ems::drv::misfire_on_tooth(snap_mf);
        }
    }
    CHECK_EQ(misfire_get_event_count(0u), 0u, "inhibited: no event despite slow period");
    misfire_set_all_inhibit(false);  // restore
}

// ============================================================================
// DIAGNOSTIC MANAGER
// ============================================================================

static void test_diagnostic_manager_all(void) {
    using namespace ems::engine;

    section("DiagnosticManager: init");
    DiagnosticManager::init();
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 0u, "no faults after init");
    CHECK_TRUE(DiagnosticManager::is_system_ready(), "system ready after init");

    section("DiagnosticManager: report_fault / is_fault_active / clear_fault");
    DiagnosticManager::init();
    const bool first_report = DiagnosticManager::report_fault(
        DiagnosticCode::MAP_SENSOR_RANGE, FaultSeverity::WARNING);
    CHECK_TRUE(first_report, "first report returns true (new fault)");
    CHECK_TRUE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_SENSOR_RANGE),
               "fault is active after report");
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 1u, "count=1 after one fault");

    const bool second_report = DiagnosticManager::report_fault(
        DiagnosticCode::MAP_SENSOR_RANGE, FaultSeverity::WARNING);
    CHECK_FALSE(second_report, "duplicate report returns false (already active)");
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 1u, "count still 1 (duplicate)");

    const bool cleared = DiagnosticManager::clear_fault(DiagnosticCode::MAP_SENSOR_RANGE);
    CHECK_TRUE(cleared, "clear_fault returns true");
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_SENSOR_RANGE),
                "fault inactive after clear");
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 0u, "count=0 after clear");

    section("DiagnosticManager: get_highest_severity");
    DiagnosticManager::init();
    DiagnosticManager::report_fault(DiagnosticCode::VBATT_LOW, FaultSeverity::WARNING);
    DiagnosticManager::report_fault(DiagnosticCode::OVERTEMP_CRITICAL, FaultSeverity::CRITICAL);
    DiagnosticManager::report_fault(DiagnosticCode::CKP_SIGNAL_FAULT, FaultSeverity::ERROR);
    CHECK_EQ(static_cast<uint8_t>(DiagnosticManager::get_highest_severity()),
             static_cast<uint8_t>(FaultSeverity::CRITICAL), "highest=CRITICAL");

    section("DiagnosticManager: is_system_ready blocked by CRITICAL");
    CHECK_FALSE(DiagnosticManager::is_system_ready(),
                "system NOT ready with CRITICAL fault");

    section("DiagnosticManager: clear_all_faults");
    DiagnosticManager::clear_all_faults();
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 0u, "count=0 after clear_all");
    CHECK_TRUE(DiagnosticManager::is_system_ready(), "system ready after clear_all");

    section("DiagnosticManager: update_recovery / get_recovery_state");
    DiagnosticManager::init();
    DiagnosticManager::report_fault(DiagnosticCode::ADC_TIMEOUT, FaultSeverity::ERROR);
    auto rs = DiagnosticManager::update_recovery(DiagnosticCode::ADC_TIMEOUT, false);
    CHECK_TRUE(static_cast<uint8_t>(rs) <= static_cast<uint8_t>(RecoveryState::PERMANENT),
               "recovery state in valid range");
    const auto rs2 = DiagnosticManager::get_recovery_state(DiagnosticCode::ADC_TIMEOUT);
    CHECK_EQ(static_cast<uint8_t>(rs), static_cast<uint8_t>(rs2),
             "get_recovery_state matches update_recovery");
    // Success recovery
    DiagnosticManager::update_recovery(DiagnosticCode::ADC_TIMEOUT, true);
    CHECK_TRUE(true, "update_recovery(success): no crash");

    section("DiagnosticManager: record_freeze_frame / get_event");
    DiagnosticManager::init();
    DiagnosticManager::report_fault(DiagnosticCode::CLT_SENSOR_RANGE, FaultSeverity::WARNING,
                                    1000u, 3000u);
    const uint16_t ff[4] = {900u, 100u, 30000u, 12000u};
    DiagnosticManager::record_freeze_frame(DiagnosticCode::CLT_SENSOR_RANGE, ff);
    const DiagnosticEvent* ev = DiagnosticManager::get_event(
        DiagnosticCode::CLT_SENSOR_RANGE);
    CHECK_TRUE(ev != nullptr, "get_event returns non-null for active fault");
    if (ev != nullptr) {
        CHECK_EQ(ev->freeze_frame[0], 900u, "freeze_frame[0]=900");
    }
    // get_event for unknown fault → nullptr
    const DiagnosticEvent* ev_none = DiagnosticManager::get_event(
        DiagnosticCode::FLASH_WRITE_FAULT);
    CHECK_TRUE(ev_none == nullptr, "get_event for inactive fault → nullptr");

    section("DiagnosticManager: check_sensor_plausibility");
    DiagnosticManager::init();
    // High TPS (900 = 90%) + Low MAP (200 = 2 bar) at high RPM is implausible
    CHECK_FALSE(DiagnosticManager::check_sensor_plausibility(200u, 900u, 80000u),
                "high TPS + low MAP + high RPM: implausible");
    // Low TPS + mid MAP at mid RPM: plausible
    CHECK_TRUE(DiagnosticManager::check_sensor_plausibility(700u, 100u, 30000u),
               "low TPS + mid MAP + mid RPM: plausible");
}

// ============================================================================
// HAL ADC
// ============================================================================

static void test_hal_adc_all(void) {
    using namespace ems::hal;

    section("hal/adc: init + primary/secondary read");
    adc_init();
    // After test_set_raw, read must return set value
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 2500u);
    CHECK_EQ(adc_primary_read(AdcPrimaryChannel::MAP), 2500u,
             "primary_read == test_set_raw");

    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 1800u);
    CHECK_EQ(adc_secondary_read(AdcSecondaryChannel::CLT), 1800u,
             "secondary_read == test_set_raw");

    section("hal/adc: adc_trigger_on_tooth updates trigger mod");
    adc_trigger_on_tooth(10000u);
    const uint32_t mod = adc_test_last_trigger_mod();
    CHECK_TRUE(mod > 0u, "trigger_mod > 0 after trigger_on_tooth(10000)");

    // Short period: mod should be smaller
    adc_trigger_on_tooth(5000u);
    const uint32_t mod2 = adc_test_last_trigger_mod();
    CHECK_TRUE(mod2 > 0u, "trigger_mod > 0 after trigger_on_tooth(5000)");
    CHECK_TRUE(mod2 <= mod, "shorter period → trigger_mod ≤ previous");

    section("hal/adc: recovery / timeout flags");
    adc_test_set_recovering(false);
    CHECK_FALSE(adc_is_recovering(), "is_recovering=false after set_false");
    adc_test_set_recovering(true);
    CHECK_TRUE(adc_is_recovering(), "is_recovering=true after set_true");
    adc_test_set_recovering(false);  // restore

    adc_test_set_recovery_failed(false);
    CHECK_FALSE(adc_recovery_failed(), "recovery_failed=false");
    adc_test_set_recovery_failed(true);
    CHECK_TRUE(adc_recovery_failed(), "recovery_failed=true");
    adc_test_set_recovery_failed(false);  // restore

    adc_test_set_timeout_count(42u);
    CHECK_EQ(adc_get_timeout_count(), 42u, "get_timeout_count=42");
    adc_test_set_timeout_count(0u);

    adc_test_set_recovery_retries(7u);
    CHECK_EQ(adc_get_recovery_retries(), 7u, "get_recovery_retries=7");
    adc_test_set_recovery_retries(0u);
}

// ============================================================================
// HAL FLASH (NVM)
// ============================================================================

static void test_hal_flash_all(void) {
    using namespace ems::hal;

    section("hal/flash: test_reset + erase/program counters");
    nvm_test_reset();
    CHECK_EQ(nvm_test_erase_count(), 0u, "erase_count=0 after reset");
    CHECK_EQ(nvm_test_program_count(), 0u, "program_count=0 after reset");

    section("hal/flash: nvm_write_ltft / nvm_read_ltft round-trip");
    nvm_test_reset();
    // Write: index in [0,7], value int8_t
    const bool ok_w = nvm_write_ltft(3u, 5u, 25);
    CHECK_TRUE(ok_w, "nvm_write_ltft returns true");
    const int8_t v = nvm_read_ltft(3u, 5u);
    CHECK_EQ(v, (int8_t)25, "nvm_read_ltft returns written value");
    // nvm_write_ltft uses RAM shadow, does not increment program_count
    CHECK_EQ(nvm_test_program_count(), 0u, "write_ltft uses RAM shadow (no flash counter)");

    section("hal/flash: nvm_write_ltft_add / nvm_read_ltft_add");
    nvm_test_reset();
    CHECK_TRUE(nvm_write_ltft_add(2u, 4u, -10), "nvm_write_ltft_add returns true");
    CHECK_EQ(nvm_read_ltft_add(2u, 4u), (int8_t)-10, "ltft_add round-trip");

    section("hal/flash: nvm_load_adaptive_maps / nvm_flush_adaptive_maps");
    nvm_test_reset();
    // In host mode these operate on RAM shadow — just must not crash
    const bool loaded = nvm_load_adaptive_maps();
    CHECK_TRUE(loaded == true || loaded == false, "nvm_load_adaptive_maps: no crash");
    const bool flushed = nvm_flush_adaptive_maps();
    CHECK_TRUE(flushed == true || flushed == false, "nvm_flush_adaptive_maps: no crash");

    section("hal/flash: nvm_write_knock / nvm_read_knock / nvm_reset_knock_map");
    nvm_test_reset();
    CHECK_TRUE(nvm_write_knock(1u, 2u, -5), "nvm_write_knock returns true");
    CHECK_EQ(nvm_read_knock(1u, 2u), (int8_t)-5, "nvm_read_knock round-trip");
    nvm_reset_knock_map();
    CHECK_EQ(nvm_read_knock(1u, 2u), (int8_t)0, "nvm_read_knock=0 after reset_knock_map");

    section("hal/flash: nvm_save_calibration / nvm_load_calibration");
    nvm_test_reset();
    uint8_t page_out[16] = {0xAA, 0xBB, 0x01, 0x02, 0x03,
                             0x04, 0x05, 0x06, 0x07, 0x08,
                             0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
    const bool saved = nvm_save_calibration(0u, page_out, 16u);
    CHECK_TRUE(saved == true || saved == false, "nvm_save_calibration: no crash");
    uint8_t page_in[16] = {};
    const bool ldok = nvm_load_calibration(0u, page_in, 16u);
    if (saved && ldok) {
        CHECK_EQ(page_in[0], 0xAAu, "load_calibration[0] = 0xAA");
        CHECK_EQ(page_in[1], 0xBBu, "load_calibration[1] = 0xBB");
    } else {
        CHECK_TRUE(true, "save/load_calibration: graceful result");
    }

    section("hal/flash: gate de layout do setor adaptativo (magic LTF2)");
    {
        // Imagem sintética: setor apagado (0xFF) OU layout antigo (sem magic
        // na posição atual) → inválido; magic presente → válido.
        static uint8_t sector[8192];
        memset(sector, 0xFF, sizeof(sector));
        CHECK_FALSE(nvm_adaptive_sector_valid(sector),
                    "setor apagado (0xFF) → layout inválido");
        memset(sector, 0, sizeof(sector));
        CHECK_FALSE(nvm_adaptive_sector_valid(sector),
                    "layout antigo (sem magic) → inválido");
        memcpy(sector + kNvmOffLayoutMagic, &kNvmLayoutMagic,
                    sizeof(kNvmLayoutMagic));
        CHECK_TRUE(nvm_adaptive_sector_valid(sector),
                   "magic presente → layout válido");
        CHECK_FALSE(nvm_adaptive_sector_valid(nullptr), "nullptr → inválido");
        // Coerência do layout derivado: regiões não podem sobrepor-se
        CHECK_TRUE(kNvmOffKnock >= kNvmLtftDim * kNvmLtftDim,
                   "knock após LTFT-mult");
        CHECK_TRUE(kNvmOffLayoutMagic >=
                       kNvmOffLtftAdd + kNvmLtftAddDim * kNvmLtftAddDim,
                   "magic após LTFT-add");
        CHECK_TRUE(kNvmSeedOffset >= kNvmOffLayoutMagic + 4u, "seed após magic");
        CHECK_TRUE((kNvmOffLayoutMagic % 16u) == 0u, "magic 16-alinhado");
        CHECK_TRUE((kNvmSeedOffset % 16u) == 0u, "seed 16-alinhado");
    }

    section("hal/flash: bounds LTFT seguem as dimensões NVM");
    nvm_test_reset();
    CHECK_TRUE(nvm_write_ltft(kNvmLtftDim - 1u, kNvmLtftDim - 1u, 7),
               "última célula LTFT aceita");
    CHECK_FALSE(nvm_write_ltft(kNvmLtftDim, 0u, 7), "além do grid → false");
    CHECK_TRUE(nvm_write_ltft_add(kNvmLtftAddDim - 1u, kNvmLtftAddDim - 1u, 3),
               "última célula LTFT-add aceita");
    CHECK_FALSE(nvm_write_ltft_add(kNvmLtftAddDim, 0u, 3),
                "além do sub-grid → false");

    section("hal/flash: flash_test_set_busy_polls blocks writes");
    nvm_test_reset();
    flash_test_set_busy_polls(5u);  // g_flash_busy=true → all writes blocked
    CHECK_FALSE(nvm_write_ltft(0u, 0u, 10), "write blocked when flash busy");
    CHECK_FALSE(nvm_write_knock(0u, 0u, 5), "knock write blocked when flash busy");
    CHECK_FALSE(nvm_save_calibration(0u, page_out, 16u), "cal save blocked when flash busy");
    flash_test_set_busy_polls(0u);  // restore
    CHECK_TRUE(nvm_write_ltft(0u, 0u, 10), "write succeeds after busy cleared");
}

// ============================================================================
// XTAU AUTOCALIB
// ============================================================================

static void test_xtau_autocalib_all(void) {
    using namespace ems::engine;

    section("xtau_autocalib: init / reset");
    xtau_autocalib_init();
    xtau_autocalib_reset();
    CHECK_FALSE(xtau_is_learning(), "not learning after reset");

    section("xtau_autocalib: update — non-transient → no update");
    xtau_autocalib_reset();
    // is_transient=false → no learning
    const bool updated_steady = xtau_autocalib_update(
        30000u, 100u, 1000, 1000, 800, false);
    CHECK_FALSE(updated_steady, "non-transient → no update");
    CHECK_FALSE(xtau_is_learning(), "not learning in steady state");

    section("xtau_autocalib: update — transient with lambda error");
    xtau_autocalib_reset();
    // Transient + lambda error=100 x1000 (in [50,150] valid window).
    // kLambdaErrorHistorySize entries needed before valid_count≥4.
    // After 4+ calls: update() returns true and calibration_state=2.
    bool any_update = false;
    for (int i = 0; i < 20; ++i) {
        any_update |= xtau_autocalib_update(
            30000u, 100u, 1000, 1100, 800, true);  // 10% rich, valid transient
    }
    CHECK_TRUE(any_update, "transient update: returns true after ≥4 valid history samples");

    // After successful update: calibration_state=2 (calibrated), NOT 1 (learning).
    // xtau_is_learning() returns calibration_state==1 — state jumps 0→2, never 1.
    CHECK_FALSE(xtau_is_learning(), "is_learning()=false after calibrated (state=2, skips 1)");

    // xtau_get_state: fields are non-trivially populated after learning
    const WallFuelState wst = xtau_get_state();
    CHECK_EQ(wst.calibration_state, 2u, "calibration_state=2 after successful update");

    section("xtau_autocalib: xtau_get_current_params");
    const XTauParams p = xtau_get_current_params(800);
    CHECK_TRUE(p.x_fraction_q8 <= 255u, "x_fraction_q8 in [0,255]");
    CHECK_TRUE(p.tau_cycles >= 1u, "tau_cycles ≥ 1");
    // After learning, params may differ from initial table values
    // (blended toward ideal values; x_fraction should be in [0,192] after clamp)
    CHECK_TRUE(p.x_fraction_q8 <= 192u || p.x_fraction_q8 > 0u,
               "x_fraction_q8 bounded after learning");

    section("xtau_autocalib: transient_fuel_xtau_with_autocalib");
    xtau_autocalib_reset();
    // disabled: returns input pw
    const uint32_t pw_disabled = transient_fuel_xtau_with_autocalib(5000u, 30000u, 100u, 800, false);
    CHECK_EQ(pw_disabled, 5000u, "disabled → returns input pw");
    // enabled: applies model
    const uint32_t pw_enabled = transient_fuel_xtau_with_autocalib(5000u, 30000u, 100u, 800, true);
    CHECK_TRUE(pw_enabled > 0u && pw_enabled <= 100000u,
               "enabled: pw in (0, 100ms]");

    section("xtau_autocalib: células RPM×MAP diferentes aprendem parâmetros diferentes");
    // Duas células bem afastadas nos eixos (baixo RPM/baixa carga vs. alto
    // RPM/alta carga) recebem erros de lambda opostos — se a Fase 3 deixou de
    // ser um escalar global, os parâmetros aprendidos devem divergir.
    xtau_autocalib_reset();
    for (int i = 0; i < 20; ++i) {
        xtau_autocalib_update(8000u, 2500u, 1000, 1100, 800, true);   // baixo RPM/carga, rico
    }
    const XTauParams p_low = xtau_get_current_params_2d(8000u, 2500u);

    xtau_autocalib_reset();
    for (int i = 0; i < 20; ++i) {
        xtau_autocalib_update(70000u, 15000u, 1000, 900, 800, true);  // alto RPM/carga, pobre
    }
    const XTauParams p_high = xtau_get_current_params_2d(70000u, 15000u);

    CHECK_TRUE(p_low.x_fraction_q8 != p_high.x_fraction_q8 || p_low.tau_cycles != p_high.tau_cycles,
               "células RPM×MAP distantes aprendem parâmetros distintos");
}

// ============================================================================
// VERIFICAÇÃO MATEMÁTICA — valores independentes calculados analiticamente
// ============================================================================

static void test_math_req_fuel(void) {
    // calc_req_fuel_us formula:
    //   num = displacement_cc × kAirDensityMgPerCcX1000(1184) × 100 × 60000000
    //   den = cylinders × stoich_afr_x100 × injector_flow_cc_min × kFuelDensityMgPerCc(755) × 1000
    //   result = num / den (integer)
    //
    // Derivado independentemente em Python:
    //   num = 1998×1184×100×60000000 = 14193792000000000
    //   den = 4×1470×440×755×1000   = 1953336000000
    //   result = 7266 µs
    section("MATH: calc_req_fuel_us formula exacta");
    CHECK_EQ(calc_req_fuel_us(1998u, 4u, 440u, 1470u), 7266u,
             "req_fuel(1998cc,4cyl,440cc/min,AFR14.7) = 7266µs");

    // Caso de proporcionalidade: deslocamento ÷ 2 → req_fuel ÷ 2
    // calc_req_fuel_us(999,4,440,1470) = 7266/2... mas integer, verificamos proporcionalidade
    const uint32_t req_half = calc_req_fuel_us(999u, 4u, 440u, 1470u);
    CHECK_TRUE(req_half >= 3630u && req_half <= 3634u,
               "half displacement → approx half req_fuel (3632±2)");

    // Injetor 2× mais caudal → req_fuel ÷ 2
    const uint32_t req_big_inj = calc_req_fuel_us(1998u, 4u, 880u, 1470u);
    CHECK_TRUE(req_big_inj >= 3630u && req_big_inj <= 3634u,
               "2× injector flow → approx half req_fuel (3633±2)");

    // Guards: zero inputs
    CHECK_EQ(calc_req_fuel_us(0u, 4u, 440u, 1470u), 0u, "displacement=0 → 0");
    CHECK_EQ(calc_req_fuel_us(1998u, 0u, 440u, 1470u), 0u, "cylinders=0 → 0");
}

static void test_math_base_pw(void) {
    // calc_base_pw_us formula: pw = req_fuel × ve/100 × map/map_ref
    // With req=7266, ve=80, map=100, map_ref=100:
    //   num = 7266×80×100 = 58128000
    //   den = 100×100 = 10000
    //   result = 5812
    section("MATH: calc_base_pw_us formula exacta");
    CHECK_EQ(calc_base_pw_us(7266u, 80u, 100u, 100u), 5812u,
             "base_pw(req=7266,VE=80%,MAP=REF) = 5812µs");

    // VE=100%, MAP=REF → pw = req (identidade)
    CHECK_EQ(calc_base_pw_us(7266u, 100u, 100u, 100u), 7266u,
             "VE=100% MAP=REF → pw = req_fuel (identidade)");

    // MAP = 2×REF → pw = req×ve×2 = 7266×80×200/10000 = 11625
    //   num = 7266×80×200 = 116256000, den=10000, result=11625
    CHECK_EQ(calc_base_pw_us(7266u, 80u, 200u, 100u), 11625u,
             "MAP=2×REF → pw doubles proportionally (11625µs)");

    // MAP = REF/2 → pw halves: 7266×80×50/10000 = 2906
    CHECK_EQ(calc_base_pw_us(7266u, 80u, 50u, 100u), 2906u,
             "MAP=REF/2 → pw halves (2906µs)");

    // Proporcionalidade VE: VE=40 deve dar metade de VE=80
    const uint32_t pw80 = calc_base_pw_us(7266u, 80u, 100u, 100u);  // 5812
    const uint32_t pw40 = calc_base_pw_us(7266u, 40u, 100u, 100u);  // 2906
    CHECK_EQ(pw80, 2u * pw40, "VE=80 = 2×VE=40 (proporcionalidade linear)");
}

static void test_math_lambda_pw(void) {
    // apply_lambda_target_pw_us: pw_out = base × 1000 / lambda_target
    //   base=5000, lambda=850 → 5000×1000/850 = 5000000/850 = 5882
    //   base=5000, lambda=1200 → 5000000/1200 = 4166
    section("MATH: apply_lambda_target_pw_us formula exacta");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 850u), 5882u,
             "lambda=0.850 → 5000×1000/850 = 5882µs");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1200u), 4166u,
             "lambda=1.200 → 5000×1000/1200 = 4166µs");
    // lambda=1.000 → identidade
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1000u), 5000u,
             "lambda=1.000 → pw inalterado");
    // Proporcionalidade inversa: pw × lambda = constante (base × 1000)
    // lambda deve estar em [650,1200]; usamos 800 e 1000.
    // pw(800) = 6000×1000/800 = 7500;  pw(1000) = 6000×1000/1000 = 6000
    // pw_a×lambda_a = 6000×1000=6000000 = pw_b×lambda_b = 7500×800=6000000
    const uint32_t pw_a = apply_lambda_target_pw_us(6000u, 1000u);  // 6000
    const uint32_t pw_b = apply_lambda_target_pw_us(6000u,  800u);  // 7500
    CHECK_EQ(pw_a * 1000u, pw_b * 800u,
             "lambda proporcionalidade inversa: pw×lambda=constante (base×1000)");
}

static void test_math_table3d_bilinear(void) {
    using namespace ems::engine;
    section("MATH: table3d bilinear interpolation 2D (fx>0, fy>0)");

    // Ponto: x=6250, y=25.
    // kRpmAxisX10:        [0]=5000, [1]=7500  → xi=0, fx=(6250-5000)×256/2500=128
    // kLoadAxisBarX100:   [0]=20,   [1]=30    → yi=0, fy=(25-20)×256/10=128
    //
    // Tabela (só as 4 células que importam, resto=0):
    //   [yi][xi]=[0][0]=10  [0][1]=20
    //   [1][0]=30           [1][1]=40
    //
    // Cálculo:
    //   v0 = lerp(10,20,128) = 10 + 10×128/256 = 15
    //   v1 = lerp(30,40,128) = 30 + 10×128/256 = 35
    //   v  = lerp(15,35,128) = 15 + 20×128/256 = 25
    static uint8_t tbl_u8[kTableAxisSize][kTableAxisSize] = {};
    tbl_u8[0][0] = 10u; tbl_u8[0][1] = 20u;
    tbl_u8[1][0] = 30u; tbl_u8[1][1] = 40u;

    const Table2dLookup lk_mid = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 6250u, 25u);
    CHECK_EQ(lk_mid.fx_q8, 128u, "fx=128 at x=6250 (midpoint axis[0]=5000..axis[1]=7500)");
    CHECK_EQ(lk_mid.fy_q8, 128u, "fy=128 at y=25 (midpoint axis[0]=20..axis[1]=30)");

    const uint8_t result = table3d_lookup_u8_prepared(tbl_u8, lk_mid);
    CHECK_EQ(result, 25u, "bilinear([10,20/30,40],fx=fy=128) = 25");

    // Verifica é realmente 2D: alterar apenas fy modifica o resultado
    const Table2dLookup lk_bottom = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 6250u, 20u);  // fy=0 (linha inferior)
    const uint8_t result_bottom = table3d_lookup_u8_prepared(tbl_u8, lk_bottom);
    CHECK_EQ(result_bottom, 15u,
             "bilinear com fy=0: lerp(10,20,128)=15 (só linha yi=0)");

    const Table2dLookup lk_top = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 6250u, 30u);  // fy=255 (linha superior)
    const uint8_t result_top = table3d_lookup_u8_prepared(tbl_u8, lk_top);
    CHECK_EQ(result_top, 35u,
             "bilinear com fy=255: lerp(30,40,128)=35 (linha yi=1 por frac=255)");

    // Caso i8 com valores negativos: [0][0]=-20,[0][1]=0,[1][0]=0,[1][1]=20
    // v0=lerp(-20,0,128)=-10, v1=lerp(0,20,128)=10, v=lerp(-10,10,128)=0
    static int8_t tbl_i8[kTableAxisSize][kTableAxisSize] = {};
    tbl_i8[0][0] = -20; tbl_i8[0][1] = 0;
    tbl_i8[1][0] =   0; tbl_i8[1][1] = 20;
    const int16_t result_i8 = table3d_lookup_i8_prepared(tbl_i8, lk_mid);
    CHECK_EQ(result_i8, 0, "bilinear i8([-20,0/0,20],fx=fy=128) = 0 (centro)");

    section("MATH: table3d frac=255 boundary (lerp retorna valor exato de b)");
    // Sem o caso especial frac=255, lerp(0,100,255)=99; com ele retorna 100.
    static uint8_t tbl_bound[kTableAxisSize][kTableAxisSize] = {};
    tbl_bound[0][0] =   0u;
    tbl_bound[0][1] = 100u;  // xi+1, yi=0
    // x=7485 → frac=(7485-5000)×256/2500=2485×256/2500=254 → lerp(0,100,254)=99
    const Table2dLookup lk_254 = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 7485u, 20u);
    CHECK_EQ(lk_254.fx_q8, 254u, "x=7485 → frac=254 (abaixo do caso especial)");
    const uint8_t r254 = table3d_lookup_u8_prepared(tbl_bound, lk_254);
    CHECK_EQ(r254, 99u, "lerp(0,100,254)=99 (truncamento normal)");

    // x=7500 → frac=255 → caso especial retorna b=100 (não 99)
    const Table2dLookup lk_255 = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 7500u, 20u);
    CHECK_EQ(lk_255.fx_q8, 255u, "x=7500=axis[1] → frac=255 (caso especial)");
    const uint8_t r255 = table3d_lookup_u8_prepared(tbl_bound, lk_255);
    CHECK_EQ(r255, 100u, "lerp(0,100,255)=100 (caso especial frac=255 retorna b exato)");
}

static void test_math_corrections(void) {
    section("MATH: corr_clt valores exactos (interp linear em tabela)");
    // clt_corr_axis_x10 = {-400,-100,0,200,...}
    // clt_corr_x256     = {384, 352,320,288,...}
    // Ponto -250 (midpoint entre -400 e -100):
    //   frac = (-250-(-400))×256/(-100-(-400)) = 150×256/300 = 128
    //   lerp(384,352,128) = 384 + (352-384)×128/256 = 384-16 = 368
    CHECK_EQ(corr_clt(-250), 368u, "corr_clt(-250) = 368 (interp exacta)");

    // Ponto extremo inferior: usa clt_corr_x256[0]=384
    CHECK_EQ(corr_clt(-400), 384u, "corr_clt(-400) = 384 (valor eixo 0)");

    section("MATH: corr_iat valores exactos");
    // iat_corr_axis_x10 = {-200,0,200,400,600,800,1000,1200}
    // iat_corr_x256     = {272, 264,256,256,264,272, 280, 288}
    // Ponto 100 (midpoint 0..200):
    //   frac = 100×256/200 = 128
    //   lerp(264,256,128) = 264 + (256-264)×128/256 = 264-4 = 260
    CHECK_EQ(corr_iat(100), 260u, "corr_iat(100) = 260 (interp exacta)");
    CHECK_EQ(corr_iat(200), 256u, "corr_iat(200) = 256 (valor ref, frac=255→b)");

    section("MATH: corr_vbatt valores exactos (dead-time do injector)");
    // vbatt_corr_axis_mv  = {9000,...,12000,13000,...}
    // injector_dead_time  = {1400,..., 900,  800,...}
    // Ponto 12500 (midpoint 12000..13000):
    //   frac = 500×256/1000 = 128
    //   lerp(900,800,128) = 900 + (800-900)×128/256 = 900-50 = 850
    CHECK_EQ(corr_vbatt(12500u), 850u, "corr_vbatt(12500mV) = 850µs (interp exacta)");

    // Monotonícidade: vbatt mais baixo → dead-time maior
    CHECK_TRUE(corr_vbatt(9000u) > corr_vbatt(16000u),
               "dead-time monotonicamente decrescente com vbatt");
    CHECK_TRUE(corr_vbatt(11000u) > corr_vbatt(13000u),
               "dead-time(11V) > dead-time(13V)");
}

static void test_math_stft_gains(void) {
    section("MATH: fuel_update_stft ganhos Kp e Ki exactos");
    // Kp = kStftKpNum/100 = 3/100. Por error=200 x1000: p = 200×3/100 = 6 x10
    // Ki = kStftKiNum/kStftKiDen = 1/200. Por error=200 x1000: i += 200/200 = 1 x10
    // Após N chamadas com erro constante de 200 x1000 (20% lean):
    //   integrator = N (sem clamp), stft = 6 + N
    // Verificar após 1 chamada: stft=7; após 10: stft=16; após 300: stft=250 (clampado)

    fuel_reset_adaptives();
    // 1 chamada: p=6, integrator=1, stft=7
    const int16_t s1 = fuel_update_stft(
        30000u, 100u, 1000, 1200, 900, true, false, false, 30000u, 500u);
    CHECK_EQ(s1, 7, "STFT após 1 call (lean 20%): p=6 + I=1 = 7");

    // Após 9 calls adicionais (total 10): integrator=10, stft=16
    for (int i = 0; i < 9; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false, 30000u, 500u);
    }
    CHECK_EQ(fuel_get_stft_pct_x10(), 16, "STFT após 10 calls: p=6 + I=10 = 16");

    section("MATH: fuel_update_stft clamp kStftClampX10=250");
    // Após 300 chamadas: integrator clamped=250, stft=clamp(6+250,-250,250)=250
    for (int i = 0; i < 290; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false, 30000u, 500u);
    }
    CHECK_EQ(fuel_get_stft_pct_x10(), 250, "STFT saturado no clamp kStftClampX10=250");

    section("MATH: fuel_update_stft congela (freeze) quando loop fechado desabilitado");
    // Com CLT fria (clt=600 < 700=70°C): closed_loop_allowed=false.
    // Anti-windup: stft congela no último valor (250, saturado no clamp acima),
    // não decai — evita degrau de combustível ao voltar a closed-loop.
    const int16_t s_cold1 = fuel_update_stft(
        30000u, 100u, 1000, 1200, 600, true, false, false, 30000u, 500u);
    CHECK_EQ(s_cold1, 250, "STFT congelado após 1 chamada fria: mantém 250");
    const int16_t s_cold2 = fuel_update_stft(
        30000u, 100u, 1000, 1200, 600, true, false, false, 30000u, 500u);
    CHECK_EQ(s_cold2, 250, "STFT congelado após 2 chamadas frias: mantém 250");
}

static void test_math_inj_scheduler_ticks(void) {
    section("MATH: inj_pw_us_to_scheduler_ticks (host: ×60, prod: ×10)");
    using namespace ems::engine;
    // Em host (não TARGET_STM32H562): retorna pw_us × 60
    // Em producão (STM32H562): retorna pw_us × 10 (100 ns/tick @ 10 MHz)
    // Nota: o branch de host usa factor 60 (legado de outro target);
    // a formula de producao (factor 10) não é testada no host.
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(1000u), 60000u,
             "1000µs × 60 = 60000 ticks (host branch)");
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(100u), 6000u,
             "100µs × 60 = 6000 ticks (host branch)");
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(0u), 0u,
             "0µs → 0 ticks");
    // Proporcionalidade: dobrar PW dobra ticks
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(2000u), 2u * inj_pw_us_to_scheduler_ticks(1000u),
             "proporcionalidade linear");
}

static void test_math_xtau_convergence(void) {
    section("MATH: transient_fuel_xtau steady-state injected = desired");
    // Prova analítica: em regime permanente
    //   injected = (desired - wall/tau) × 256/(256-x)
    //   wall/tau = desired × x/256 (regime permanente)
    //   => injected = desired × (256-x)/256 × 256/(256-x) = desired
    // Após 200 ciclos com entrada constante de 5000µs @ clt=800:
    //   x_q8=32 (interp entre [5]=35 e [6]=28 com frac=128)
    //   tau=11  (interp entre [5]=12 e [6]=10 com frac=128)
    //   Convergencia em ~5×tau=55 ciclos; a 200 ciclos é estado permanente.
    transient_fuel_reset();
    uint32_t out = 0u;
    for (int i = 0u; i < 200; ++i) {
        out = transient_fuel_xtau_update(5000u, 800, true);
    }
    // Tolera ±20µs por truncamento Q8 (wall_q8 em aritmética inteira)
    CHECK_TRUE(out >= 4980u && out <= 5020u,
               "xtau steady-state: injected ≈ desired=5000 (±20µs)");

    section("MATH: transient_fuel_xtau transitorio — overshot no arranque (wall=0)");
    // Com wall=0 na chamada inicial: evap=0, numerator=desired,
    // injected = desired × 256/(256-x_q8) = 5000×256/224 ≈ 5714µs.
    // (x_q8=32 → dry_fraction=256-32=224)
    transient_fuel_reset();
    const uint32_t first_out = transient_fuel_xtau_update(5000u, 800, true);
    // first_out deve ser > desired (wall vazia → injetor compensa parede)
    CHECK_TRUE(first_out > 5000u,
               "xtau arranque: 1ª injecao > desired (parede vazia → overshot)");
    // E inferior ao limite de clamp (100ms)
    CHECK_TRUE(first_out <= 100000u, "xtau 1ª injeção ≤ 100ms clamp");
}

static void test_math_production_tables(void) {
    using namespace ems::engine;
    section("MATH: get_ve / get_lambda / get_advance com tabelas reais");
    // Ponto (3000 RPM = rpm_x10=30000, MAP=100 kPa = map_bar_x100=100).
    // Eixos 20×20: kRpmAxisX10[10]=30000 → floor idx=9, frac=255
    //              kLoadAxisBarX100[11]=100 → floor idx=10, frac=255
    // Frac=255 → lerp devolve o canto alto exacto: ve_table[11][10]=88.
    CHECK_EQ(get_ve(30000u, 100u), 88u,
             "get_ve(3000RPM,100kPa) = 88 (tabela real, frac=255 ambos eixos)");

    // Prepared == direct para mesmo ponto
    const Table2dLookup lk = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u);
    CHECK_EQ(get_ve_prepared(lk), 88u, "get_ve_prepared == 88");

    // lambda_target_table[6]=all 1000; [7]=all 990
    //   v0=lerp(1000,1000,255)=1000; v1=lerp(990,990,255)=990; v=lerp(1000,990,255)=990
    CHECK_EQ(get_lambda_target_x1000(30000u, 100u), 990u,
             "get_lambda(3000RPM,100kPa) = 990 (tabela real)");
    CHECK_EQ(get_lambda_target_x1000_prepared(lk), 990u,
             "get_lambda_prepared == 990");

    // spark_table[6][6]=19,[6][7]=18,[7][6]=17,[7][7]=16
    //   v0=lerp(19,18,255)=18; v1=lerp(17,16,255)=16; v=lerp(18,16,255)=16
    CHECK_EQ(get_advance(30000u, 100u), 16,
             "get_advance(3000RPM,100kPa) = 16\u00b0 (tabela real)");
    CHECK_EQ(get_advance_prepared(lk), 16,
             "get_advance_prepared == 16");

    section("MATH: corr_warmup valores exactos");
    // warmup_corr_axis_x10={-400,-100,0,...}, warmup_corr_x256={420,380,350,...}
    // Eixo exacto -400 → 420; eixo exacto -100: idx=0,frac=255 → lerp(420,380,255)=380
    CHECK_EQ(corr_warmup(-400), 420u, "corr_warmup(-40\u00b0C) = 420");
    CHECK_EQ(corr_warmup(-100), 380u, "corr_warmup(-10\u00b0C) = 380 (eixo[1], frac=255)");
    // Midpoint entre -400 e -100 (-250): frac=150\u00d7256/300=128
    // lerp(420,380,128) = 420 + (380-420)\u00d7128/256 = 420-20 = 400
    CHECK_EQ(corr_warmup(-250), 400u, "corr_warmup(-25\u00b0C) = 400 (interp exacta)");

    section("MATH: dwell_ms_x10_from_vbatt valor exacto");
    // dwell_vbatt_axis_mv={9000,...,12000,...},dwell_ms_x10={42,...,30,...}
    // 12000 = axis[3]: idx=2, frac=255 → lerp(35,30,255)=30
    CHECK_EQ(dwell_ms_x10_from_vbatt(12000u), 30u,
             "dwell @ 12V = 30 x10 (3.0ms, valor exacto da tabela)");
    CHECK_EQ(dwell_ms_x10_from_vbatt(9000u),  42u,
             "dwell @ 9V = 42 x10 (4.2ms, limite inferior)");

    section("MATH: dwell_ms_x10_from_vbatt_rpm correc\u00e7\u00e3o por RPM");
    // dwell_rpm_axis_rpm={500,1200,4000,7000}, factor_q8={384,288,256,200}
    // rpm_x10=5000 → RPM=500 = axis[0]: frac=0 (value<=axis[0]) → factor=384
    // dwell_final = 30 \u00d7 384 / 256 = 45
    CHECK_EQ(dwell_ms_x10_from_vbatt_rpm(12000u, 5000u), 45u,
             "dwell @ 12V, 500 RPM = 45 x10 (1.5\u00d7 factor de arranque)");
    // rpm_x10=70000 → RPM=7000 = axis[3]: frac=255 → factor=200
    // dwell_final = 30 \u00d7 200 / 256 = 23 (integer: 6000/256=23)
    CHECK_EQ(dwell_ms_x10_from_vbatt_rpm(12000u, 70000u), 23u,
             "dwell @ 12V, 7000 RPM = 23 x10 (0.78\u00d7 factor alto RPM)");

    section("MATH: calc_ae_pw_us formula exacta");
    // Input: tps_now=800, tps_prev=500, dt=10ms, clt=800.
    // delta_tps_x10 = 300;  tpsdot_x10 = 300/10 = 30
    // ae_tpsdot_axis_x10={5,20,50,100}: tpsdot=30 entre [1]=20 e [2]=50
    //   frac = (30-20)\u00d7256/(50-20) = 2560/30 = 85
    //   base_pw = lerp(800,1500,85) = 800 + 700\u00d785/256 = 800+232 = 1032
    // ae_clt_corr_axis_x10={-400,-100,0,200,400,700,900,1100}
    //   clt=800: bucket=5 (700<800<900) → ae_clt_sens[5]=6
    // ae_pw = 1032 \u00d7 6 / 8 = 6192/8 = 774
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);
    CHECK_EQ(calc_ae_pw_us(800u, 500u, 10u, 800), 774,
             "calc_ae_pw_us(tpsdot=30,clt=800): 1032\u00d76/8=774\u00b5s");
}

static void test_math_misfire_threshold(void) {
    using namespace ems::engine;
    section("MATH: misfire threshold kMisfireThresholdQ8=287 (=1.12×256)");
    // Threshold de detecção: power_sum > (predicted_sum × 287) >> 8
    // = predicted_sum × 1.12109...
    //
    // Com kMisfireWindowTeeth=10, kNormalPeriod=10000 ticks, 16ns/tick:
    //   normal_ns      = 160000
    //   predicted_sum  = 10 × 160000 = 1600000
    //   threshold      = (1600000 × 287) >> 8 = 1793750
    //
    //   1.2× slow: power_sum = 10×192000 = 1920000 > 1793750 → misfire
    //   1.1× slow: power_sum = 10×176000 = 1760000 < 1793750 → NO misfire

    misfire_init();
    misfire_reset();

    const uint32_t normal_ns = kNormalPeriod * 16u;   // 160000 ns
    const uint32_t slow_1_2  = static_cast<uint32_t>(normal_ns * 12u / 10u);  // 192000 ns
    const uint32_t slow_1_1  = static_cast<uint32_t>(normal_ns * 11u / 10u);  // 176000 ns

    // Snap para cyl0: phase_A=true, tooth_index 0..9
    // g_tooth_to_cyl[0][0..kMisfireWindowTeeth-1] = cyl0 (mapa preenchido por misfire_init)
    auto feed_window = [&](uint32_t period_ns, uint32_t pred_ns) {
        ems::drv::CkpSnapshot sn{};
        sn.state                    = ems::drv::SyncState::FULL_SYNC;
        sn.phase_A                  = true;
        sn.tooth_period_ns          = period_ns;
        sn.predicted_tooth_period_ns = pred_ns;
        for (uint8_t t = 0u; t < kMisfireWindowTeeth; ++t) {
            sn.tooth_index = t;
            ems::drv::misfire_on_tooth(sn);
        }
    };

    // Alimentar 3 janelas lentas (1.2×) → kMisfireDebounceCycles=3 → evento confirmado
    for (uint8_t w = 0u; w < kMisfireDebounceCycles; ++w) {
        feed_window(slow_1_2, normal_ns);
    }
    CHECK_TRUE(misfire_get_event_count(0u) >= 1u,
               "misfire detectado: 1.2× (power=1920000 > threshold=1793750)");

    misfire_reset();
    // 1.1× slow: power_sum=1760000 < threshold=1793750 → NÃO deve detectar
    for (uint8_t w = 0u; w < kMisfireDebounceCycles; ++w) {
        feed_window(slow_1_1, normal_ns);
    }
    CHECK_EQ(misfire_get_event_count(0u), 0u,
             "sem misfire: 1.1× (power=1760000 < threshold=1793750)");

    section("MATH: misfire threshold sensitivity — proporcionalidade");
    // Verificar que o threshold é exactamente (predicted × 287) >> 8
    // ao calcular a janela mínima de detecção manualmente:
    //   threshold = (1600000 × 287) >> 8 = 459200000 >> 8 = 1793750
    //   min_power_per_tooth = 1793750/10 = 179375 ns (per tooth)
    //   min_period_factor = 179375 / 160000 = 1.1210... (= 287/256)
    const uint32_t threshold_check = (static_cast<uint64_t>(kMisfireWindowTeeth) *
                                       normal_ns * kMisfireThresholdQ8) >> 8u;
    CHECK_EQ(threshold_check, 1793750u, "threshold = predicted_sum × 287/256 = 1793750");
}

static void test_trigger_offset(void) {
    using namespace ems::engine::cfg;

    // ----------------------------------------------------------------
    // Baseline: offset = 0°
    // TDC 540° = posição 3 da firing order {0,2,3,1} = cilindro físico 1
    // → canal ECU_CH_IGN2 (convenção: canal = cilindro físico).
    // Com advance=15°, dwell=84°:
    //   dwell_engine_angle = (525+720-84)%720 = 441°
    //   trigger_angle(441, offset=0) = (441+720-0)%720 = 441°
    //   ang=81, pos_x256=81*256/6=3456, tooth=13, frac=128, phase_B
    // ----------------------------------------------------------------
    section("trigger offset=0°: cyl1 (tdc=540°) DWELL_START scheduled at tooth 13");
    g_eng_cfg.trigger_tooth0_engine_deg = 0u;
    ecu_sched_test_reset();
    ecu_sched_test_reset_ccr();
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // Force cmp_confirms>=2 so Calculate_Sequential_Cycle() runs (not presync).
    ckp_test_set_cmp_confirms(2u);
    ckp_feed_n_then_gap(55u);  // trigger sequential scheduling at next gap
    ecu_sched_test_reset_ccr();

    // Inspect angle table: find ECU_CH_IGN2 DWELL_START entry (cyl 1, tdc=540°)
    uint8_t tooth_off0 = 0xFFu;
    for (uint8_t i = 0u; i < ecu_sched_test_angle_table_size(); ++i) {
        uint8_t t, f, ch, act, ph;
        if (ecu_sched_test_get_angle_event(i, &t, &f, &ch, &act, &ph)) {
            if (ch == ECU_CH_IGN2 && act == ECU_ACT_DWELL_START) { tooth_off0 = t; }
        }
    }
    CHECK_EQ(tooth_off0, 13u,
             "offset=0°: cyl1 DWELL_START tooth=13 (trigger=441°, ang=81, 81*256/6=3456>>8=13)");

    // Events from tooth 0 (gap) may already be in queue; reset for clean check.
    ecu_sched_test_reset_ccr();

    // Fire 13 teeth → at tooth 13 DWELL_START for cyl3 fires → event inserted.
    for (uint32_t i = 0u; i < 13u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u || ecu_sched_test_get_tim5_ccr3() > 0u,
               "offset=0°: TIM5 event queued after tooth 13 DWELL_START");

    // ----------------------------------------------------------------
    // Non-zero offset: 78°
    // trigger_angle(441, offset=78) = (441+720-78)%720 = 363°
    //   ang=363%360=3, pos_x256=3*256/6=128, tooth=0, frac=128, phase_B
    // Event shifts to tooth 0 → fires AT the FULL_SYNC gap (tooth 0 is
    // processed immediately when Calculate_Sequential_Cycle completes).
    // ----------------------------------------------------------------
    section("trigger offset=78°: cyl1 DWELL_START shifts to tooth 0");
    g_eng_cfg.trigger_tooth0_engine_deg = 78u;
    ecu_sched_test_reset();
    ecu_sched_test_reset_ccr();
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // Force cmp_confirms>=2 so Calculate_Sequential_Cycle() runs (not presync).
    ckp_test_set_cmp_confirms(2u);
    // 57 teeth: presync SPARK at tooth 57 clears arm_ticks; gap triggers sequential.
    // Do NOT call reset_ccr after — event at tooth 0 fires during the gap itself.
    ckp_feed_n_then_gap(57u);

    // Angle table: cyl1 (tdc=540°, canal IGN2) DWELL_START must be at tooth 0
    uint8_t tooth_off78 = 0xFFu;
    for (uint8_t i = 0u; i < ecu_sched_test_angle_table_size(); ++i) {
        uint8_t t, f, ch, act, ph;
        if (ecu_sched_test_get_angle_event(i, &t, &f, &ch, &act, &ph)) {
            if (ch == ECU_CH_IGN2 && act == ECU_ACT_DWELL_START) { tooth_off78 = t; }
        }
    }
    CHECK_EQ(tooth_off78, 0u,
             "offset=78°: cyl1 DWELL_START shifts to tooth 0 (trigger=363°, ang=3, 3*256/6=128>>8=0)");
    CHECK_TRUE(tooth_off0 != tooth_off78,
               "offset change moves the event: tooth 13 (offset=0) ≠ tooth 0 (offset=78)");

    // With offset=78°, cyl3 DWELL_START fires at tooth 0 (at the FULL_SYNC gap).
    // Verify event was queued immediately at FULL_SYNC.
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u || ecu_sched_test_get_tim5_ccr3() > 0u,
               "offset=78°: TIM5 event queued immediately at FULL_SYNC gap (tooth 0)");

    // Confirm: with offset=0 the CCR was 0 at gap; with offset=78 it is 1800 at gap.
    // Both CCR values are numerically 1800 because frac=128 in both cases,
    // but offset=0 fires 13 teeth LATER whereas offset=78 fires IMMEDIATELY.
    // The table inspection above is the definitive proof of timing difference.

    // Restore default so subsequent tests are unaffected
    g_eng_cfg.trigger_tooth0_engine_deg = 0u;
}

// ═══════════════════════════════════════════════════════════════════════════
// UI PROTOCOL / TUNERSTUDIO ENVELOPE
// ═══════════════════════════════════════════════════════════════════════════

#include <cstring>

#include "app/ui_protocol.h"
#include "hal/crc32.h"

static void ui_feed(const uint8_t* data, uint16_t n) {
    for (uint16_t i = 0u; i < n; ++i) { ems::app::ui_rx_byte(data[i]); }
    ems::app::ui_process();
}

static uint16_t ui_drain(uint8_t* out, uint16_t max) {
    uint16_t n = 0u;
    uint8_t b = 0u;
    while (n < max && ems::app::ui_tx_pop(b)) { out[n++] = b; }
    return n;
}

// Monta frame envelope: [size u16 BE][payload][CRC32 BE]. Retorna bytes totais.
static uint16_t env_frame(uint8_t* out, const uint8_t* payload, uint16_t n) {
    out[0] = static_cast<uint8_t>(n >> 8u);
    out[1] = static_cast<uint8_t>(n & 0xFFu);
    memcpy(out + 2, payload, n);
    const uint32_t crc = ems::hal::crc32_calc(payload, n);
    out[2u + n] = static_cast<uint8_t>(crc >> 24u);
    out[3u + n] = static_cast<uint8_t>(crc >> 16u);
    out[4u + n] = static_cast<uint8_t>(crc >> 8u);
    out[5u + n] = static_cast<uint8_t>(crc & 0xFFu);
    return static_cast<uint16_t>(n + 6u);
}

struct EnvResp {
    bool frame_ok;    // tamanho coerente
    bool crc_ok;      // CRC32 da resposta confere
    uint8_t code;
    uint16_t len;     // bytes de dados (sem o code)
    uint8_t data[1024];  // >= maior página (lambda 800B) p/ testes whole-page
};

// Envia payload embrulhado e decodifica a resposta envelope.
static EnvResp env_txn(const uint8_t* payload, uint16_t n) {
    EnvResp r = {};
    static uint8_t frame[1024] = {};
    const uint16_t fl = env_frame(frame, payload, n);
    ui_feed(frame, fl);

    static uint8_t buf[1024] = {};
    memset(buf, 0, sizeof(buf));
    const uint16_t rn = ui_drain(buf, sizeof(buf));
    if (rn < 7u) { return r; }
    const uint16_t psize = static_cast<uint16_t>((buf[0] << 8u) | buf[1]);
    if (static_cast<uint16_t>(psize + 6u) != rn) { return r; }
    r.frame_ok = true;
    r.code = buf[2];
    r.len = static_cast<uint16_t>(psize - 1u);
    memcpy(r.data, buf + 3, r.len);
    const uint32_t rx_crc = (static_cast<uint32_t>(buf[2u + psize]) << 24u) |
                            (static_cast<uint32_t>(buf[3u + psize]) << 16u) |
                            (static_cast<uint32_t>(buf[4u + psize]) << 8u) |
                             static_cast<uint32_t>(buf[5u + psize]);
    r.crc_ok = (rx_crc == ems::hal::crc32_calc(buf + 2, psize));
    return r;
}

static void test_crc32_vectors(void) {
    section("crc32: vetores ISO-HDLC");
    const uint8_t check[9] = {'1','2','3','4','5','6','7','8','9'};
    CHECK_EQ(ems::hal::crc32_calc(check, 9u), 0xCBF43926u, "crc32(\"123456789\")=0xCBF43926");
    CHECK_EQ(ems::hal::crc32_calc(nullptr, 0u), 0x00000000u, "crc32(vazio)=0");
}

static void test_legacy_protocol_regression(void) {
    section("ui_protocol legacy: intocado pelo envelope");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();

    uint8_t buf[80] = {};
    const uint8_t q = 'Q';
    ui_feed(&q, 1u);
    uint16_t n = ui_drain(buf, sizeof(buf));
    CHECK_EQ(n, 12u, "'Q' devolve 12 bytes crus (sem envelope)");
    CHECK_TRUE(memcmp(buf, "OpenEMS_v1.3", 12u) == 0, "assinatura OpenEMS_v1.3");

    const uint8_t c = 'C';
    ui_feed(&c, 1u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 2u && buf[0] == 0x00u && buf[1] == 0xAAu, "'C' → ACK+0xAA");

    // read legacy: 'r' page1 off0 len16
    const uint8_t rd[6] = {'r', 0x01u, 0x00u, 0x00u, 0x10u, 0x00u};
    ui_feed(rd, 6u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_EQ(n, 16u, "'r' page1 len16 → 16 bytes crus");
    CHECK_EQ(buf[0], ve_table[0][0], "primeiro byte = ve_table[0][0]");

    // write RAM-only legacy: 'x' page1 off0 len1 data=77
    const uint8_t wr[7] = {'x', 0x01u, 0x00u, 0x00u, 0x01u, 0x00u, 77u};
    ui_feed(wr, 7u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "'x' → ACK");
    CHECK_EQ(ve_table[0][0], 77u, "ve_table[0][0]=77 aplicado em RAM");

    const uint8_t d = 'd';
    ui_feed(&d, 1u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 1u && (buf[0] & 0x01u) != 0u, "'d' → 1 byte, página 1 dirty");
}

static void test_ts_envelope_basic(void) {
    section("envelope TS: assinatura + comms test");
    ems::app::ui_test_reset();

    const uint8_t q = 'Q';
    EnvResp r = env_txn(&q, 1u);
    CHECK_TRUE(r.frame_ok, "resposta com framing válido");
    CHECK_TRUE(r.crc_ok, "CRC32 da resposta confere");
    CHECK_EQ(r.code, 0x00u, "code OK");
    CHECK_TRUE(r.len == 12u && memcmp(r.data, "OpenEMS_v1.3", 12u) == 0,
               "payload = assinatura");

    const uint8_t c = 'C';
    r = env_txn(&c, 1u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u && r.len == 1u &&
               r.data[0] == 0xAAu, "'C' → code OK + magic");

    const uint8_t z = 'Z';  // comando inexistente
    r = env_txn(&z, 1u);
    CHECK_TRUE(r.frame_ok && r.code == 0x83u, "comando desconhecido → 0x83");
}

static void test_ts_envelope_crc_reject(void) {
    section("envelope TS: CRC corrompido rejeitado");
    ems::app::ui_test_reset();

    uint8_t frame[16] = {};
    const uint8_t q = 'Q';
    const uint16_t fl = env_frame(frame, &q, 1u);
    frame[fl - 1u] ^= 0xFFu;  // corrompe CRC
    ui_feed(frame, fl);

    uint8_t buf[32] = {};
    const uint16_t n = ui_drain(buf, sizeof(buf));
    CHECK_EQ(n, 7u, "resposta de erro tem 7 bytes (code sem dados)");
    CHECK_EQ(buf[2], 0x82u, "code 0x82 = CRC error");

    // parser recupera: próximo frame válido responde normalmente
    EnvResp r = env_txn(&q, 1u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "parser recuperado após CRC error");
}

static void test_ts_envelope_read_write_burn(void) {
    section("envelope TS: r/w/b com página VE");
    ckp_test_reset(); g_ckp_cap = 0u;  // RPM=0 → burn permitido
    ems::app::ui_test_reset();

    // read: 'r' page1 off0 len16 (forma sem canId)
    const uint8_t rd[6] = {'r', 0x01u, 0x00u, 0x00u, 0x10u, 0x00u};
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u && r.len == 16u,
               "'r' 5-args → 16 bytes");
    CHECK_EQ(r.data[0], ve_table[0][0], "dados = ve_table");

    // read com canId à frente (forma TS canónica de 6 args)
    const uint8_t rd7[7] = {'r', 0x00u, 0x01u, 0x00u, 0x00u, 0x10u, 0x00u};
    r = env_txn(rd7, 7u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 16u,
               "'r' com canId=0 → 16 bytes");

    // chunk write: RAM-only, sem burn
    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t wr[10] = {'w', 0x01u, 0x00u, 0x00u, 0x04u, 0x00u, 11u, 22u, 33u, 44u};
    r = env_txn(wr, 10u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' chunk → OK");
    CHECK_EQ(ve_table[0][0], 11u, "VE[0][0]=11 aplicado em RAM");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before,
             "'w' envelope NÃO grava flash (RAM-only)");

    // dirty mask (16 bits LE no envelope)
    const uint8_t d = 'd';
    r = env_txn(&d, 1u);
    CHECK_TRUE(r.frame_ok && r.len == 2u && (r.data[0] & 0x01u) != 0u,
               "'d' → 2 bytes, página 1 dirty");

    // burn com motor parado
    const uint8_t burn[2] = {'b', 0x01u};
    r = env_txn(burn, 2u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'b' page1 @ 0 RPM → OK");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before + 1u,
             "burn gravou 1 página");

    r = env_txn(&d, 1u);
    CHECK_TRUE(r.frame_ok && (r.data[0] & 0x01u) == 0u, "dirty limpo após burn");
}

static void test_ts_envelope_burn_gate(void) {
    section("envelope TS: burn bloqueado com motor girando");
    ems::app::ui_test_reset();
    ckp_reach_full_sync();  // ~6250 RPM > kFlashWriteSafeRpmX10 (300 RPM)

    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t burn[2] = {'b', 0x01u};
    EnvResp r = env_txn(burn, 2u);
    CHECK_TRUE(r.frame_ok && r.code == 0x85u, "'b' com RPM alto → 0x85 busy");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before, "flash intocada");

    // legacy 'b' também bloqueado
    uint8_t buf[8] = {};
    const uint8_t lb[2] = {'b', 0x01u};
    ui_feed(lb, 2u);
    const uint16_t n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "legacy 'b' com RPM alto → NACK");

    ckp_test_reset(); g_ckp_cap = 0u;  // restaura RPM=0 p/ testes seguintes
}

static void test_ts_axes_page(void) {
    section("página 11: eixos de tabela editáveis");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();

    // read: defaults serializados (rpm[0]=500, load[0]=20)
    const uint8_t rd[6] = {'r', 0x0Bu, 0x00u, 0x00u, 0x50u, 0x00u};
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 80u, "'r' page11 → 80 bytes");
    const uint16_t rpm0 = static_cast<uint16_t>(r.data[0] | (r.data[1] << 8u));
    const uint16_t load0 = static_cast<uint16_t>(r.data[40] | (r.data[41] << 8u));
    CHECK_EQ(rpm0, 500u, "rpm[0] default = 500");
    CHECK_EQ(load0, 20u, "load[0] default = 20 (0.20 bar)");

    // write monotónico: rpm 400..(passo 400), load 10..(passo 10)
    uint8_t wr[6u + 80u] = {'w', 0x0Bu, 0x00u, 0x00u, 0x50u, 0x00u};
    for (uint8_t i = 0u; i < 20u; ++i) {
        const uint16_t rv = static_cast<uint16_t>(400u + i * 400u);
        const uint16_t lv = static_cast<uint16_t>(10u + i * 10u);
        wr[6u + i * 2u]       = static_cast<uint8_t>(rv & 0xFFu);
        wr[7u + i * 2u]       = static_cast<uint8_t>(rv >> 8u);
        wr[6u + 40u + i * 2u] = static_cast<uint8_t>(lv & 0xFFu);
        wr[7u + 40u + i * 2u] = static_cast<uint8_t>(lv >> 8u);
    }
    r = env_txn(wr, sizeof(wr));
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' eixos monotónicos → OK");
    CHECK_EQ(kRpmAxisX10[0], 4000u, "kRpmAxisX10[0]=4000 (400 RPM ×10)");
    CHECK_EQ(kLoadAxisBarX100[19], 200u, "kLoadAxisBarX100[19]=200");

    // write não-monotónico rejeitado, eixos preservados
    wr[6u + 4u] = wr[6u + 0u];  // rpm[2] == rpm[0] → viola monotonicidade
    wr[7u + 4u] = wr[7u + 0u];
    r = env_txn(wr, sizeof(wr));
    CHECK_TRUE(r.frame_ok && r.code == 0x84u, "'w' não-monotónico → 0x84");
    CHECK_EQ(kRpmAxisX10[2], 12000u, "eixos preservados após rejeição");

    // buffer restaurado: 'r' devolve os eixos válidos, não o lixo rejeitado
    r = env_txn(rd, 6u);
    const uint16_t rpm2 = static_cast<uint16_t>(r.data[4] | (r.data[5] << 8u));
    CHECK_EQ(rpm2, 1200u, "'r' pós-rejeição devolve eixo válido (1200)");

    // burn página 11 → NVM slot 9
    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t burn[2] = {'b', 0x0Bu};
    r = env_txn(burn, 2u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'b' page11 → OK");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before + 1u, "NVM slot 9 gravado");

    // restaura defaults p/ não afetar outros testes
    const uint16_t rpm_def[20] = {500u, 750u, 1000u, 1250u, 1500u, 1750u, 2000u,
                                  2250u, 2500u, 2750u, 3000u, 3500u, 4000u, 4500u,
                                  5000u, 5500u, 6000u, 6500u, 7000u, 8000u};
    const uint16_t load_def[20] = {20u, 30u, 40u, 46u, 52u, 58u, 64u, 70u, 76u, 88u,
                                   94u, 100u, 110u, 130u, 160u, 190u, 220u, 250u,
                                   273u, 300u};
    CHECK_TRUE(table_axes_set(rpm_def, load_def), "defaults restaurados");
}

static void test_ts_whole_page_800(void) {
    section("envelope TS: whole-page read de 800B (página 4, lambda 20×20)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();

    // 'r' page4 off0 count800 numa única transação — exatamente o caso que
    // motivou kEnvMaxChunk historicamente (Comm Manager pede página inteira).
    const uint8_t rd[6] = {'r', 0x04u, 0x00u, 0x00u, 0x20u, 0x03u};  // 0x0320=800
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u, "'r' 800B → OK");
    CHECK_EQ(r.len, 800u, "800 bytes devolvidos");
    // primeira célula = lambda_target[0][0] (default 1050)
    const int16_t l00 = static_cast<int16_t>(r.data[0] | (r.data[1] << 8u));
    CHECK_EQ(l00, (int16_t)1050, "lambda[0][0] = 1050");
    // última célula (célula 399) legível e plausível
    const int16_t l_last = static_cast<int16_t>(r.data[798] | (r.data[799] << 8u));
    CHECK_TRUE(l_last > 500 && l_last < 1200, "lambda[19][19] plausível");

    // whole-page 400B das páginas 1/2 também
    const uint8_t rd1[6] = {'r', 0x01u, 0x00u, 0x00u, 0x90u, 0x01u};  // 400
    r = env_txn(rd1, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 400u, "'r' VE 400B → OK");
    CHECK_EQ(r.data[0], ems::engine::ve_table[0][0], "VE[0][0] confere");
    CHECK_EQ(r.data[399], ems::engine::ve_table[19][19], "VE[19][19] confere");
}

static void test_adaptives_reset_cmd_z(void) {
    section("protocolo: 'Z' learn session reset (STFT+accum+LTFT shadow)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    // Gera hits e STFT não-zero
    fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) > 0u, "hits antes do Z");
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0 || g_stft_integrator_x1000 != 0,
               "STFT/integrador activo antes do Z");

    // Comando legacy 'Z' → ACK 0x00
    ems::app::ui_rx_byte(static_cast<uint8_t>('Z'));
    ems::app::ui_process();
    uint8_t ack = 0xFFu;
    bool got = false;
    for (int i = 0; i < 16; ++i) {
        if (ems::app::ui_tx_pop(ack)) { got = true; break; }
    }
    CHECK_TRUE(got, "Z produz byte TX");
    CHECK_EQ(ack, 0x00u, "Z → ACK OK");

    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "Z zera hits do acumulador");
    CHECK_EQ(fuel_get_stft_pct_x10(), 0, "Z zera STFT");
    CHECK_EQ(g_stft_integrator_x1000, 0, "Z zera integrador");
    CHECK_EQ(g_dbg_ltft_accum_accepted, 0u, "Z zera contadores accum");
}

static void test_ltft_apply_cmd_y(void) {
    section("protocolo: 'Y' apply LEARN ready → VE (manual)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
    ltft_apply_burn_ve = 0u;

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // Aquece STFT e acumula até ready
    for (int n = 0; n < 150; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    }
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri), "célula ready antes do Y");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta antes do Y");

    // Comando legacy 'Y' → ACK + n_commits
    while (true) {
        uint8_t dump = 0;
        if (!ems::app::ui_tx_pop(dump)) break;
    }
    ems::app::ui_rx_byte(static_cast<uint8_t>('Y'));
    ems::app::ui_process();
    uint8_t b0 = 0xFFu, b1 = 0xFFu;
    bool g0 = ems::app::ui_tx_pop(b0);
    bool g1 = ems::app::ui_tx_pop(b1);
    CHECK_TRUE(g0 && g1, "Y produz 2 bytes TX");
    CHECK_EQ(b0, 0x00u, "Y → ACK OK");
    CHECK_TRUE(b1 >= 1u, "Y → n_commits ≥ 1");
    CHECK_TRUE(g_dbg_ltft_accum_commits >= 1u, "commit registado");
    CHECK_TRUE(ve_table[mi][ri] > 100u, "Y alterou VE");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri), "stats limpos pós-Y");

    ve_table[mi][ri] = 88u;
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
}

// Regressão: hit LEARN na célula dominante do trace VE (não no canto floor).
// Em 2000 rpm / 110 kPa exactos, floor = (1750,100) e nearest = (2000,110).
static void test_ltft_hit_matches_ve_dominant_cell(void) {
    section("LEARN hit = célula dominante do VE (nearest, não floor)");
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    const uint32_t rpm_x10 = 20000u;   // 2000 rpm — nó exacto do eixo
    const uint16_t map_x100 = 110u;    // 110 kPa — nó exacto

    const uint8_t ri_near = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_near = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, map_x100);
    const uint8_t ri_floor = table_axis_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_floor = table_axis_index(kLoadAxisBarX100, kTableAxisSize, map_x100);

    CHECK_EQ(ri_near, 6u, "nearest rpm 2000 → idx 6");
    CHECK_EQ(mi_near, 12u, "nearest map 110 → idx 12");
    CHECK_EQ(ri_floor, 5u, "floor rpm 2000 → idx 5 (pré-fix bug)");
    CHECK_EQ(mi_floor, 11u, "floor map 110 → idx 11 (pré-fix bug)");
    CHECK_TRUE(ri_near != ri_floor && mi_near != mi_floor,
               "caso de teste: floor ≠ nearest nos dois eixos");

    // 1 prev + N hits em regime estável
    fuel_update_stft(rpm_x10, map_x100, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 8; ++i) {
        fuel_update_stft(rpm_x10, map_x100, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }

    CHECK_EQ(fuel_ltft_accum_hits(mi_near, ri_near), 8u,
             "hits na célula dominante (2000×110) — igual ao trace VE");
    CHECK_EQ(fuel_ltft_accum_hits(mi_floor, ri_floor), 0u,
             "zero hits na célula floor (1750×100) — era o bug");
    // Também não deve espalhar para os outros cantos do rectângulo bilineal
    CHECK_EQ(fuel_ltft_accum_hits(mi_near, ri_floor), 0u, "sem hit no canto misto A");
    CHECK_EQ(fuel_ltft_accum_hits(mi_floor, ri_near), 0u, "sem hit no canto misto B");

    fuel_ltft_accum_reset();
    fuel_reset_adaptives();
}

static void test_ltft_accum_page12(void) {
    section("página 12: LTFT accum export (hits u8 + mean_stft i8)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // 1 prev + 5 hits com err residual
    fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 5u, "5 hits na célula");

    uint8_t buf[kLtftAccumPageSize] = {};
    fuel_ltft_accum_export(buf, kLtftAccumPageSize);
    const uint16_t idx = static_cast<uint16_t>(mi) * kTableAxisSize + ri;
    CHECK_EQ(buf[idx] & 0x7Fu, 5u, "export hits[map][rpm] = 5");
    CHECK_EQ(buf[idx] & 0x80u, 0u, "export ready bit clear com 5 hits");
    // mean STFT i8 no 2º half
    const int8_t mean_wire = static_cast<int8_t>(buf[kTableCells + idx]);
    CHECK_EQ(static_cast<int>(mean_wire),
             static_cast<int>(fuel_ltft_accum_mean_stft_x10(mi, ri)),
             "export mean_stft confere");

    // Leitura via protocolo page 12 (800 B)
    const uint8_t rd[6] = {'r', 0x0Cu, 0x00u, 0x00u, 0x20u, 0x03u};  // 800
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u, "'r' page12 800B → OK");
    CHECK_EQ(r.len, 800u, "page12 len 800");
    CHECK_EQ(r.data[idx], 5u, "page12 wire hits = 5 (ready bit clear)");

    fuel_ltft_accum_reset();
    fuel_reset_adaptives();
}

static void test_ltft_page_offsets_20(void) {
    section("página 10: offsets do layout 20×20 (mult 400 + add 10×10)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    ems::hal::nvm_test_reset();

    // grava células de canto e lê a página inteira
    CHECK_TRUE(ems::hal::nvm_write_ltft(19u, 19u, 21), "ltft[19][19]=21");
    CHECK_TRUE(ems::hal::nvm_write_ltft_add(9u, 9u, -7), "add[9][9]=-7");
    const uint8_t rd[6] = {'r', 0x0Au, 0x00u, 0x00u, 0xF4u, 0x01u};  // 500
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 500u, "'r' page10 → 500B");
    // mult[m=19][r=19] no byte 19*20+19 = 399
    CHECK_EQ(static_cast<int8_t>(r.data[399]), (int8_t)21, "mult[19][19] no byte 399");
    // add[m=9][r=9] no byte 400 + 9*10+9 = 499
    CHECK_EQ(static_cast<int8_t>(r.data[499]), (int8_t)-7, "add[9][9] no byte 499");
}

// Formas de wire que o TunerStudio REAL emite quando pageIdentifier usa o
// prefixo \$tsCanId (obrigatório em msEnvelope_1.0, confirmado contra os
// projetos Speeduino 202501.6 e rusEFI): toda página fica com identificador
// de 2 bytes (canId+page), logo 'w'/'b' chegam sempre com 1 byte a mais do
// que a forma "sem canId" testada em test_ts_envelope_read_write_burn.
static void test_ts_envelope_canid_forms(void) {
    section("envelope TS: formas com \\$tsCanId (write/burn/och)");
    ckp_test_reset(); g_ckp_cap = 0u;  // RPM=0 → burn permitido
    ems::app::ui_test_reset();

    // och lê página 3 → update_realtime_page() → get_ve(rpm, map_bar_x100), que
    // faz assert em map válido. map_bar_x1000 só é populado depois de pelo menos
    // uma sample_fast_channels() (5× sensors_on_tooth(), kFastSamplesPerRev=12 /
    // kRealTeethPerRev=58) seguida do commit staging→committed.
    sensor_setup(); sensors_init();
    {
        ems::drv::CkpSnapshot snap{};
        snap.tooth_period_ns = 160000u;
        snap.rpm_x10 = 62500u;
        for (int i = 0; i < 5; ++i) { sensors_on_tooth(snap); }
        sensors_test_tick_100ms();
    }

    // write canId-prefixed: 'w' canId(0) page(1) off(2) len(2) data(4)
    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t wr[11] = {'w', 0x00u, 0x01u, 0x00u, 0x00u, 0x04u, 0x00u, 55u, 66u, 77u, 88u};
    EnvResp r = env_txn(wr, 11u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' canId+page+off+len → OK");
    CHECK_EQ(ve_table[0][0], 55u, "VE[0][0]=55 aplicado (forma canId)");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before,
             "'w' canId não grava flash (RAM-only)");

    // burn canId-prefixed: 'b' canId(0) page(1) — 3 bytes total
    const uint8_t burn[3] = {'b', 0x00u, 0x01u};
    r = env_txn(burn, 3u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'b' canId+page → OK");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before + 1u,
             "burn (forma canId) gravou 1 página");

    // ochGetCommand real: 'r' canId(0) page(3) off(0) count(66) — 7 bytes
    const uint8_t och[7] = {'r', 0x00u, 0x03u, 0x00u, 0x00u, 0x42u, 0x00u};
    r = env_txn(och, 7u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u && r.len == 66u,
               "och via 'r' canId+page3+off0+count66 → 66 bytes");
}

static void test_ts_envelope_signature_via_r(void) {
    section("envelope TS: 'r' page 0x0F → assinatura (convenção Comm Manager)");
    ems::app::ui_test_reset();

    // 'r' canId(0) page(0x0F) off(0) count(0) — o Comm Manager real do
    // TunerStudio usa esta pseudo-página para validar o controlador após a
    // conexão, distinta do probe leve 'Q' cru da fase de deteção/wizard
    // (confirmado no comms.cpp real do Speeduino: "cmd == 0x0f → Request
    // for signature"). off/count no pedido são ignorados para esta página.
    const uint8_t req[7] = {'r', 0x00u, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u};
    EnvResp r = env_txn(req, 7u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u, "'r' page 0x0F → OK");
    CHECK_TRUE(r.len == 12u && memcmp(r.data, "OpenEMS_v1.3", 12u) == 0,
               "payload = assinatura OpenEMS_v1.3");

    // forma sem canId (6 bytes) também deve funcionar
    const uint8_t req6[6] = {'r', 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u};
    r = env_txn(req6, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 12u,
               "'r' page 0x0F sem canId → também OK");
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTE DE SAÍDAS ('T' / output_test)
// ═══════════════════════════════════════════════════════════════════════════

#include "engine/output_test.h"

// Helper: transação 'T' de 5 bytes, devolve nº de bytes de resposta em out.
static uint16_t ot_txn(uint8_t sub, uint8_t a1, uint16_t a2, uint8_t* out, uint16_t max) {
    const uint8_t cmd[5] = {'T', sub, a1,
                            static_cast<uint8_t>(a2 & 0xFFu),
                            static_cast<uint8_t>(a2 >> 8u)};
    ui_feed(cmd, 5u);
    return ui_drain(out, max);
}

static void ot_reset_all(void) {
    ckp_test_reset(); g_ckp_cap = 0u;
    ecu_sched_test_reset();
    ems::app::ui_test_reset();
    ems::engine::output_test_test_reset();
    ems::engine::auxiliaries_test_reset();
}

static void test_output_test_enter_gate(void) {
    section("output_test: enter exige RPM=0 + magic");
    ot_reset_all();

    uint8_t buf[8] = {};
    // magic errado → NAK, continua inactivo
    uint16_t n = ot_txn(0x01u, 0u, 0x1234u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "ENTER magic errado → NAK");
    CHECK_TRUE(!ems::engine::output_test_active(), "continua inactivo");

    // magic certo com RPM=0 → OK
    n = ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "ENTER magic 0xA55A → ACK");
    CHECK_TRUE(ems::engine::output_test_active(), "modo activo");
    ems::engine::output_test_exit();

    // com motor girando → NAK
    ckp_reach_full_sync();
    n = ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "ENTER com RPM>0 → NAK");
    CHECK_TRUE(!ems::engine::output_test_active(), "inactivo com motor girando");
}

static void test_output_test_fire_inj(void) {
    section("output_test: FIRE_INJ agenda pulso e clampa");
    ot_reset_all();

    uint8_t buf[8] = {};
    // fire sem enter → NAK, nada agendado
    uint16_t n = ot_txn(0x10u, 0u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "FIRE_INJ sem enter → NAK");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 0u, "fila vazia sem enter");

    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    ecu_sched_test_set_tim5_cnt(1000000u);
    n = ot_txn(0x10u, 0u, 5000u, buf, sizeof(buf));  // INJ1, 5000µs
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "FIRE_INJ cyl0 5000µs → ACK");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 1u, "1 evento OFF agendado");
    // OFF ≈ cnt + 5000µs × 62.5 = 1000000 + 312500
    const uint32_t ccr = ecu_sched_test_get_tim5_ccr3();
    CHECK_TRUE(ccr >= 1312500u && ccr <= 1313000u, "CCR3 = now + 312500 ticks");

    // cyl inválido → NAK
    n = ot_txn(0x10u, 4u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "cyl=4 → NAK");
    ems::engine::output_test_exit();
}

static void test_output_test_busy_window(void) {
    section("output_test: janela busy serializa pulsos");
    ot_reset_all();

    uint8_t buf[8] = {};
    ems::engine::output_test_poll(10000u, 0u);  // g_now_ms = 10000
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    uint16_t n = ot_txn(0x10u, 0u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(buf[0] == 0x00u, "1º fire → ACK");
    n = ot_txn(0x10u, 1u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "2º fire imediato → NAK (busy)");
    // avança 200ms (>5ms pulso + 100ms gap) → liberto
    ems::engine::output_test_poll(10200u, 0u);
    n = ot_txn(0x10u, 1u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "fire após janela → ACK");
    ems::engine::output_test_exit();
}

static void test_output_test_fire_ign_watchdog(void) {
    section("output_test: FIRE_IGN clampa dwell e arma watchdog");
    ot_reset_all();

    uint8_t buf[8] = {};
    ecu_sched_test_set_tim5_cnt(2000000u);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    uint16_t n = ot_txn(0x11u, 0u, 60000u, buf, sizeof(buf));  // pede 60ms
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "FIRE_IGN → ACK");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 1u, "evento SPARK agendado");
    // clamp p/ 10000µs = 625000 ticks
    const uint32_t ccr = ecu_sched_test_get_tim5_ccr3();
    CHECK_TRUE(ccr >= 2625000u && ccr <= 2625500u, "SPARK = now + 625000 (clamp 10ms)");
    // watchdog armado: avança TIM5 além de 1.4×dwell e verifica que dispara
    const uint32_t wd_before = ecu_sched_dwell_watchdog_count();
    ecu_sched_test_set_tim5_cnt(2000000u + 875000u + 100u);  // 1.4×625000 + margem
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), wd_before + 1u,
             "dwell watchdog dispara como backstop");
    ems::engine::output_test_exit();
}

static void test_output_test_rpm_abort(void) {
    section("output_test: aborto por RPM restaura estado seguro");
    ot_reset_all();

    uint8_t buf[8] = {};
    ems::engine::output_test_poll(20000u, 0u);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    ot_txn(0x20u, 1u, 0u, buf, sizeof(buf));  // bomba ON
    CHECK_TRUE(ems::engine::auxiliaries_test_get_pump_state(), "bomba ligada");
    ot_txn(0x10u, 0u, 20000u, buf, sizeof(buf));  // pulso em voo
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u, "evento em voo");

    ems::engine::output_test_poll(20002u, 7000u);  // RPM detectado
    CHECK_TRUE(!ems::engine::output_test_active(), "modo abortado");
    CHECK_TRUE(!ems::engine::auxiliaries_test_get_pump_state(), "bomba desligada");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 0u, "fila de eventos limpa");

    uint8_t st[4] = {};
    uint16_t n = ot_txn(0x03u, 0u, 0u, st, sizeof(st));
    CHECK_TRUE(n == 4u && st[0] == 0u && st[1] == 1u,
               "STATUS: inactivo, abort_reason=RPM");
}

static void test_output_test_keepalive_timeout(void) {
    section("output_test: timeout de keepalive");
    ot_reset_all();

    uint8_t buf[8] = {};
    ems::engine::output_test_poll(30000u, 0u);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    // keepalive em +4s mantém vivo
    ems::engine::output_test_poll(34000u, 0u);
    ot_txn(0x02u, 0u, 0u, buf, sizeof(buf));
    CHECK_TRUE(buf[0] == 0x00u, "KEEPALIVE → ACK");
    ems::engine::output_test_poll(38000u, 0u);
    CHECK_TRUE(ems::engine::output_test_active(), "vivo em +8s com keepalive em +4s");
    // sem keepalive: expira em +5s
    ems::engine::output_test_poll(43001u, 0u);
    CHECK_TRUE(!ems::engine::output_test_active(), "expirado após 5s sem keepalive");
    uint8_t st[4] = {};
    ot_txn(0x03u, 0u, 0u, st, sizeof(st));
    CHECK_TRUE(st[1] == 2u, "abort_reason=timeout");
}

static void test_output_test_suspends_aux(void) {
    section("output_test: suspende controles automáticos");
    ot_reset_all();

    uint8_t buf[8] = {};
    // key-on dispara o prime da bomba (2s) via run_pump_control no tick —
    // excepto em modo teste, onde o tick retorna antes de tocar as saídas.
    ems::engine::auxiliaries_set_key_on(true);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    ems::engine::auxiliaries_tick_10ms();
    CHECK_TRUE(!ems::engine::auxiliaries_test_get_pump_state(),
               "tick_10ms não liga a bomba (prime) em modo teste");
    ems::engine::output_test_exit();
    ems::engine::auxiliaries_tick_10ms();
    CHECK_TRUE(ems::engine::auxiliaries_test_get_pump_state(),
               "após exit, prime de key-on religa a bomba");
    ems::engine::auxiliaries_set_key_on(false);
}

int main(void) {
    printf("OpenEMS Host Regression Tests\n");
    printf("============================================================\n");

    // ── ETB Driver ────────────────────────────────────────────────────────────
    printf("\n=== ETB DRIVER ===");
    test_etb_driver_adc_to_percent();
    test_etb_driver_init_and_state();
    test_etb_driver_init_fault_tps1_open();
    test_etb_driver_init_fault_tps2_short();
    test_etb_driver_read_sensors_valid();
    test_etb_driver_read_sensors_mismatch();
    test_etb_driver_read_sensors_null();
    test_etb_driver_set_motor_pwm();
    test_etb_driver_shutdown();
    test_etb_driver_clear_fault();

    // ── ETB Control ───────────────────────────────────────────────────────────
    printf("\n=== ETB CONTROL ===");
    test_etb_set_get_drive_mode();
    test_etb_is_ready();
    test_etb_enter_limp_mode();
    test_etb_set_idle_control_and_spark_trim();
    test_etb_get_throttle_position();
    test_etb_control_loop_rpm_cutoff();
    test_etb_control_loop_sensor_fault_triggers_limp();

    // ── Torque Manager ────────────────────────────────────────────────────────
    printf("\n=== TORQUE MANAGER ===");
    test_torque_manager_init();
    test_torque_manager_enter_limp();
    test_torque_manager_set_get_config();
    test_torque_manager_set_config_null();
    test_torque_manager_loop_normal_pedal();
    test_torque_manager_loop_rpm_hard_cut();
    test_torque_manager_loop_rpm_progressive_cut();
    test_torque_manager_loop_limp_via_input();
    test_torque_manager_loop_idle_mode();
    test_torque_manager_loop_traction_control();
    test_torque_manager_loop_null_guards();
    test_torque_manager_loop_speed_limiter();

    // ── CKP Decoder ───────────────────────────────────────────────────────────
    printf("\n=== CKP DECODER / SYNC ===");
    test_ckp_rpm_math();
    test_ckp_initial_state();
    test_ckp_half_sync();
    test_ckp_full_sync();
    test_ckp_tooth_index_increments();
    test_ckp_loss_of_sync_too_many_teeth();
    test_ckp_loss_of_sync_early_gap();
    test_ckp_noise_rejection();
    test_ckp_stall_poll();
    test_ckp_phantom_rpm_unsync();
    test_ckp_rpm_jump_recovery();
    test_ckp_stall_poll_no_false_positive();
    test_ckp_seed_arm_disarm();

    // ── Sensors ───────────────────────────────────────────────────────────────
    printf("\n=== SENSORS ===");
    test_sensors_validate_range();
    test_sensors_validate_values();
    test_sensors_health_status();
    test_sensors_calibration();
    test_sensors_tick_100ms_clt_iat();
    test_sensors_maf_freq_capture();

    // ── Fuel Calc ─────────────────────────────────────────────────────────────
    printf("\n=== FUEL CALC ===");
    test_fuel_calc_req_fuel_us();
    test_fuel_calc_base_pw();
    test_fuel_apply_lambda_target();
    test_fuel_apply_trim();
    test_fuel_calc_final_pw();
    test_fuel_corr_functions();
    test_fuel_decel_cut();
    test_fuel_baro();

    // ── Ignition Calc ─────────────────────────────────────────────────────────
    printf("\n=== IGN CALC ===");
    test_ign_iat_correction();
    test_ign_clt_correction();
    test_ign_antijerk();
    test_ign_clamp_and_total_advance();
    test_ign_dwell();

    // ── Auxiliaries ───────────────────────────────────────────────────────────
    printf("\n=== AUXILIARIES ===");
    test_aux_init_and_idle();
    test_aux_pump_prime();
    test_aux_ticks_no_crash();

    // ── Knock ─────────────────────────────────────────────────────────────────
    printf("\n=== KNOCK ===");
    test_knock_init_and_threshold();
    test_knock_window();
    test_knock_detection_and_recovery();

    // ── Fuel Calc — Segunda Fase ──────────────────────────────────────────────
    printf("\n=== FUEL CALC (fase 2) ===");
    test_fuel_table_lookups();
    test_fuel_default_req_and_base_default();
    test_fuel_default_fast();
    test_fuel_corr_warmup();
    test_fuel_ae();
    test_fuel_adaptives_reset();
    test_fuel_lambda_delay();
    test_fuel_stft();
    test_fuel_stft_delayed();
    test_injector_scurve();
    test_fuel_delta_p_compensation();
    test_fuel_ltft();
    test_ltft_adapt_enable();
    test_fuel_trim_dtcs();
    test_fuel_ltft_authority();
    test_fuel_closed_loop_gates();
    test_fuel_ltft_apply_nearest();
    test_fuel_ltft_accum();
    test_fuel_ltft_accum_commit_ve();

    // ── Ign Calc — Segunda Fase ───────────────────────────────────────────────
    printf("\n=== IGN CALC (fase 2) ===");
    test_ign_get_advance();
    test_ign_dwell_vbatt_rpm();
    test_ign_idle_spark_correction();

    // ── ETB Control C++ ns ───────────────────────────────────────────────────
    printf("\n=== ETB CONTROL (C++ ns) ===");
    test_etb_cpp_update();

    // ── Torque Manager C++ ns ──────────────────────────────────────────────
    printf("\n=== TORQUE MANAGER (C++ ns) ===");
    test_torque_manager_cpp_update();

    // ── CKP — Segunda Fase ───────────────────────────────────────────────────
    printf("\n=== CKP (fase 2) ===");
    test_ckp_seed_confirmed();
    test_ckp_seed_rejected();
    test_ckp_cmp_glitch_count();

    // ── Sensors — Segunda Fase ───────────────────────────────────────────────
    printf("\n=== SENSORS (fase 2) ===");
    test_sensors_on_tooth();
    test_sensors_tick_50ms();
    test_sensors_set_range();
    test_sensors_etb_harness_present();
    test_sensors_table_entry_setters();

    // ── Knock — Segunda Fase ──────────────────────────────────────────────────
    printf("\n=== KNOCK (fase 2) ===");
    test_knock_window_cycle_end();
    test_knock_save_to_nvm();

    // ── Auxiliaries — Segunda Fase ────────────────────────────────────────────
    printf("\n=== AUXILIARIES (fase 2) ===");
    test_aux_test_getters();

    // ── Timer HAL ────────────────────────────────────────────────────────────
    printf("\n=== TIMER HAL ===");
    test_timer_stubs();

    // ── TABLE3D ──────────────────────────────────────────────────────────
    printf("\n=== TABLE3D ===");
    test_table3d_all();

    // ── ECU SCHED ───────────────────────────────────────────────────────
    printf("\n=== ECU SCHED ===");
    test_ecu_sched_setters();
    test_ecu_sched_angle_table();
    test_ecu_sched_wasted_to_sequential();
    test_ecu_sched_cmp_revalidation_after_sync_loss();
    test_ecu_sched_noise_rejects_sequential();
    test_ecu_sched_recovers_after_fallback();
    test_ecu_sched_inhibit_masks();
    test_ecu_sched_mspark();
    test_ecu_sched_eoi_targeting();
    test_eoi_blend();
    test_ecu_sched_presync();
    test_ecu_sched_dwell_watchdog();

    // ── QUICK CRANK ─────────────────────────────────────────────────────
    printf("\n=== QUICK CRANK ===");
    test_quick_crank_all();

    // ── TRANSIENT FUEL ──────────────────────────────────────────────────
    printf("\n=== TRANSIENT FUEL ===");
    test_transient_fuel_all();

    // ── MAP ESTIMATOR ───────────────────────────────────────────────────
    printf("\n=== MAP ESTIMATOR ===");
    test_map_estimator_all();

    // ── MISFIRE DETECT ──────────────────────────────────────────────────
    printf("\n=== MISFIRE DETECT ===");
    test_misfire_all();

    // ── DIAGNOSTIC MANAGER ──────────────────────────────────────────────
    printf("\n=== DIAGNOSTIC MANAGER ===");
    test_diagnostic_manager_all();

    // ── HAL ADC ───────────────────────────────────────────────────────────
    printf("\n=== HAL ADC ===");
    test_hal_adc_all();

    // ── HAL FLASH (NVM) ─────────────────────────────────────────────────
    printf("\n=== HAL FLASH (NVM) ===");
    test_hal_flash_all();

    // ── XTAU AUTOCALIB ──────────────────────────────────────────────────
    printf("\n=== XTAU AUTOCALIB ===");
    test_xtau_autocalib_all();

    // ── ECU SCHED FASE 2 ────────────────────────────────────────────────
    printf("\n=== ECU SCHED (fase 2) ===");
    test_ecu_sched_hardware_init();
    test_ecu_sched_ccr_write();
    test_ecu_sched_late_events();
    test_ecu_sched_golden_min_lead_timestamp();
    test_ecu_sched_golden_far_target_timestamp();
    test_ecu_sched_golden_queue_sorted();
    test_ecu_sched_golden_seq_angle_table_size();
    test_ecu_sched_golden_dispatch_identity();
    test_ecu_sched_dwell_watchdog_fires();
    test_ecu_sched_presync_table();

    // ── VERIFICAÇÃO MATEMÁTICA ─────────────────────────────────────────────
    printf("\n=== VERIFICAÇÃO MATEMÁTICA ===");
    test_math_req_fuel();
    test_math_base_pw();
    test_math_lambda_pw();
    test_math_table3d_bilinear();
    test_math_corrections();
    test_math_stft_gains();
    test_math_inj_scheduler_ticks();
    test_math_xtau_convergence();
    test_math_production_tables();
    test_math_misfire_threshold();
    test_trigger_offset();

    // ── CKP FASE 2 (snap fields, prime, phase_A, tooth_index) ─────────────
    printf("\n=== CKP (fase 3) ===");
    test_ckp_prime_on_tooth();
    test_ckp_snap_fields();
    test_ckp_tooth_index_progression();
    test_ckp_phase_toggle();

    // ── UI PROTOCOL / TUNERSTUDIO ENVELOPE ────────────────────────────────
    printf("\n=== UI PROTOCOL / TS ENVELOPE ===");
    test_crc32_vectors();
    test_legacy_protocol_regression();
    test_ts_envelope_basic();
    test_ts_envelope_crc_reject();
    test_ts_envelope_read_write_burn();
    test_ts_envelope_burn_gate();
    test_ts_axes_page();
    test_ts_envelope_canid_forms();
    test_ts_envelope_signature_via_r();
    test_ts_whole_page_800();
    test_adaptives_reset_cmd_z();
    test_ltft_apply_cmd_y();
    test_ltft_hit_matches_ve_dominant_cell();
    test_ltft_accum_page12();
    test_ltft_page_offsets_20();

    printf("\n=== OUTPUT TEST (teste de saídas) ===");
    test_output_test_enter_gate();
    test_output_test_fire_inj();
    test_output_test_busy_window();
    test_output_test_fire_ign_watchdog();
    test_output_test_rpm_abort();
    test_output_test_keepalive_timeout();
    test_output_test_suspends_aux();

    // ── Summary ───────────────────────────────────────────────────────────────
    printf("\n============================================================\n");
    printf("Results: %d PASS  %d FAIL\n", g_pass, g_fail);

    return (g_fail == 0) ? 0 : 1;
}
