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

void test_torque_manager_init(void) {
    section("torque_manager_init / is_ready");

    const bool ok = torque_manager_init();
    CHECK_TRUE(ok, "init returns true");
    CHECK_TRUE(torque_manager_is_ready(), "is_ready after init");
}

void test_torque_manager_enter_limp(void) {
    section("torque_manager_enter_limp");

    torque_manager_init();
    CHECK_TRUE(torque_manager_is_ready(), "pre-cond: ready");

    torque_manager_enter_limp();
    CHECK_FALSE(torque_manager_is_ready(), "not ready after enter_limp");
}

void test_torque_manager_set_get_config(void) {
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

void test_torque_manager_set_config_null(void) {
    section("torque_manager_set_config: null guard");

    torque_manager_init();
    const float max_rpm_before = torque_manager_get_config()->max_rpm;

    torque_manager_set_config(nullptr);  // must not crash or corrupt state

    CHECK_NEAR(torque_manager_get_config()->max_rpm, max_rpm_before, 0.01f,
               "config unchanged after null set_config");
}

void test_torque_manager_loop_normal_pedal(void) {
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

void test_torque_manager_loop_rpm_hard_cut(void) {
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

void test_torque_manager_loop_rpm_progressive_cut(void) {
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

void test_torque_manager_loop_limp_via_input(void) {
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

void test_torque_manager_loop_idle_mode(void) {
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

void test_torque_manager_loop_traction_control(void) {
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

void test_torque_manager_loop_null_guards(void) {
    section("torque_manager_loop: null pointer guards");

    torque_manager_init();
    torque_manager_inputs_t  in{};
    torque_manager_outputs_t out{};

    torque_manager_loop(nullptr, &out);  // must not crash
    torque_manager_loop(&in, nullptr);   // must not crash
    CHECK_TRUE(true, "null guards: no segfault");
}

void test_torque_manager_loop_speed_limiter(void) {
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



void test_torque_manager_cpp_update(void) {
    section("torque_manager (C++ ns): reset / update / test_get_target / test_get_limp_reason");

    // Existing step-response checks assume unlimited blade rate; production
    // default (500 %/s) would slew over many ticks. 0 = unlimited.
    const uint16_t saved_rate = ems::engine::etb_max_rate_pct_per_s;
    ems::engine::etb_max_rate_pct_per_s = 0u;

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

    // key_on, no faults, valid calibration → target from NORMAL pedal map.
    // NORMAL map: 50% pedal (app=500) → between map[5]=500 and map[6]=600 → 500.
    etb_cal_valid = 1u;
    out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 10u);
    CHECK_TRUE(out.etb_enable_request, "key_on + no faults → ETB enabled");
    CHECK_EQ(out.etb_target_pct_x10, 500u, "NORMAL map @50% pedal → target 500");
    CHECK_EQ(ems::engine::torque_manager_test_get_target(), 500u, "state target=500");
    CHECK_EQ(ems::engine::torque_manager_test_get_limp_reason(), 0u, "no limp reason");

    // Pedal map OOB guard: app 90–99% must not read past map[9]
    sens.app_pct_x10 = 950u;
    out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 10u);
    CHECK_EQ(out.etb_target_pct_x10, 1000u, "app 95% → last map point (1000)");
    sens.app_pct_x10 = 500u;

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

    // External rev_cut → TORQUE_LIMP_REV_CUT, target=0
    sens.app_pct_x10 = 500u;
    snap.rpm_x10 = 15000u;
    out = ems::engine::torque_manager_update(snap, sens, true, false, true, 8500u, 10u);
    CHECK_TRUE((out.limp_reason & TORQUE_LIMP_REV_CUT) != 0u, "rev_cut → LIMP_REV_CUT");
    CHECK_EQ(out.etb_target_pct_x10, 0u, "rev_cut → target=0");

    // Progressive ETB rev pullback (rev_limit defaults 7000 / window 200 RPM)
    {
        const uint32_t saved_hard = ems::engine::rev_limit_rpm_x10;
        const uint32_t saved_win  = ems::engine::rev_limit_soft_window_x10;
        ems::engine::rev_limit_rpm_x10 = 70000u;          // 7000 RPM
        ems::engine::rev_limit_soft_window_x10 = 2000u;   // 200 RPM window
        etb_cal_valid = 1u;
        sens.app_pct_x10 = 1000u;  // WOT via map → 1000
        sens.clt_degc_x10 = 800;   // warm, not hot

        // Below soft_start (6800): full pedal map
        snap.rpm_x10 = 60000u;  // 6000
        out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 1000u, "below soft band → full target");
        CHECK_EQ((out.limp_reason & TORQUE_LIMP_REV_CUT), 0u, "no REV_CUT below soft");

        // Mid soft band: 6900 = soft_start+100 of 200 → keep 50%
        // soft_start=68000, hard=70000, rpm=69000 → keep=(2000-1000)/2000 = 50%
        snap.rpm_x10 = 69000u;
        out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_TRUE((out.limp_reason & TORQUE_LIMP_REV_CUT) != 0u, "soft band → REV_CUT flag");
        CHECK_EQ(out.etb_target_pct_x10, 500u, "mid soft band → 50% of WOT target");

        // At hard limit: closed
        snap.rpm_x10 = 70000u;
        out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 0u, "at hard rev limit → ETB target 0");

        ems::engine::rev_limit_rpm_x10 = saved_hard;
        ems::engine::rev_limit_soft_window_x10 = saved_win;
        snap.rpm_x10 = 15000u;
        sens.app_pct_x10 = 500u;
    }

    // APP fault → TORQUE_LIMP_APP_FAULT
    sens.app_pct_x10 = 500u;
    sens.throttle_fault_bits = ems::drv::THROTTLE_FAULT_APP1;
    out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 10u);
    CHECK_TRUE((out.limp_reason & TORQUE_LIMP_APP_FAULT) != 0u, "APP fault → LIMP_APP_FAULT");
    sens.throttle_fault_bits = 0u;

    // ── Crank open-loop air + idle hysteresis ────────────────────────────
    section("torque_manager: crank open-loop ETB floor + idle RPM exit");
    {
        ems::engine::torque_manager_reset();
        ems::engine::quick_crank_reset();
        etb_cal_valid = 1u;
        ems::engine::etb_idle_min_opening_x10 = 30u;   // 3%
        ems::engine::etb_idle_max_opening_x10 = 80u;   // 8% crank open
        ems::engine::etb_idle_open_pct_x10 = 80u;      // APP idle threshold 8%
        ems::engine::idle_spark_window_above_target_x10 = 4000u;  // 400 RPM

        // Enter cranking via quick_crank so is_cranking() is true
        ems::engine::quick_crank_update(0u, 3000u, true, 800, 8);
        CHECK_TRUE(ems::engine::is_cranking(), "precondition: cranking");

        snap.rpm_x10 = 3000u;
        sens.app_pct_x10 = 0u;
        out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_TRUE(out.etb_target_pct_x10 >= 80u,
                   "cranking → ETB floor ≥ crank open (max idle opening)");

        // Exit cranking → taper/idle; force high RPM to leave idle phase (no I windup)
        ems::engine::quick_crank_update(100u, 8000u, true, 800, 8);
        CHECK_FALSE(ems::engine::is_cranking(), "exited crank");
        snap.rpm_x10 = 20000u;  // 2000 RPM >> target+1.5×upper
        sens.app_pct_x10 = 0u;
        // Several ticks: integrator should not climb toward max at high RPM
        for (int i = 0; i < 50; ++i) {
            out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        }
        CHECK_TRUE(out.etb_target_pct_x10 <= 80u,
                   "high RPM closed APP: idle phase exited (no forced high floor)");

        // True idle: RPM near target → floor at least min opening after I accumulates
        snap.rpm_x10 = 7000u;  // 700 RPM, target 850
        sens.app_pct_x10 = 0u;
        ems::engine::torque_manager_reset();
        ems::engine::quick_crank_reset();
        for (int i = 0; i < 200; ++i) {
            out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        }
        CHECK_TRUE(out.etb_target_pct_x10 >= 30u,
                   "idle under target: floor ≥ min opening after I");
    }

    // ── Launch control ──────────────────────────────────────────────────
    section("torque_manager: launch control holds ETB / RPM");
    {
        ems::engine::torque_manager_reset();
        etb_cal_valid = 1u;
        ems::engine::torque_launch_force_enable(1u);  // force enable
        ems::engine::launch_rpm_x10 = 45000u;
        ems::engine::launch_etb_pct_x10 = 600u;
        ems::engine::launch_app_arm_x10 = 200u;
        ems::engine::launch_app_disarm_x10 = 50u;
        snap.rpm_x10 = 50000u;  // 5000 > 4500 → cut
        sens.app_pct_x10 = 800u;  // armed
        out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.launch_active, 1u, "launch active when armed + APP high");
        CHECK_TRUE((out.limp_reason & TORQUE_ACTIVE_LAUNCH) != 0u, "ACTIVE_LAUNCH flag");
        CHECK_TRUE(out.etb_target_pct_x10 <= 600u, "launch caps ETB ≤ launch_etb");
        // Disarm
        sens.app_pct_x10 = 20u;
        out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.launch_active, 0u, "launch disarms when APP low");
        ems::engine::torque_launch_force_enable(0u);
        snap.rpm_x10 = 15000u;
        sens.app_pct_x10 = 500u;
    }

    // ── Traction control (external slip) ────────────────────────────────
    section("torque_manager: TC external slip reduces ETB + spark retard");
    {
        ems::engine::torque_manager_reset();
        etb_cal_valid = 1u;
        ems::engine::tc_enable = 1u;
        ems::engine::tc_max_reduction_pct_x10 = 800u;
        ems::engine::tc_spark_retard_max_deg = 12u;
        ems::engine::tc_reduction_rate_x10 = 10000u;  // fast slew for test
        sens.app_pct_x10 = 1000u;
        snap.rpm_x10 = 30000u;
        // Seed prev rpm then apply slip
        (void)ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        ems::engine::torque_tc_set_external_slip_pct_x10(500u);  // 50% slip
        // Several ticks to slew reduction up
        for (int i = 0; i < 20; ++i) {
            out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        }
        CHECK_TRUE(out.tc_active != 0u, "TC active with external slip");
        CHECK_TRUE(out.tc_reduction_pct_x10 > 0u, "TC reduction > 0");
        CHECK_TRUE(out.etb_target_pct_x10 < 1000u, "TC reduced ETB below WOT");
        CHECK_TRUE(out.spark_retard_deg > 0, "TC applies spark retard");
        ems::engine::torque_tc_clear_external_slip();
        ems::engine::tc_enable = 0u;
        for (int i = 0; i < 50; ++i) {
            out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        }
        CHECK_EQ(out.tc_reduction_pct_x10, 0u, "TC clears after slip removed");
        snap.rpm_x10 = 15000u;
        sens.app_pct_x10 = 500u;
    }

    // ── Traction control from CAN wheel vs vehicle speed ────────────────
    section("torque_manager: TC slip from CAN wheel speed");
    {
        ems::engine::torque_manager_reset();
        etb_cal_valid = 1u;
        ems::engine::tc_enable = 1u;
        ems::engine::tc_max_reduction_pct_x10 = 800u;
        ems::engine::tc_spark_retard_max_deg = 12u;
        ems::engine::tc_reduction_rate_x10 = 10000u;
        ems::engine::tc_app_min_x10 = 100u;
        ems::engine::tc_rpm_min_x10 = 10000u;

        // Frame 0x250: vehicle km/h LE u16 @0, driven wheel LE u16 @2
        ems::app::CanSignalDef veh = {};
        veh.id = 0x250u;
        veh.byte_lo = 0u;
        veh.byte_hi = 1u;
        veh.shift_right = 0u;
        veh.mask = 0xFFFFu;
        veh.timeout_ms = 0u;  // no timeout (host millis is static)
        ems::app::CanSignalDef whl = veh;
        whl.byte_lo = 2u;
        whl.byte_hi = 3u;
        ems::app::can_rx_map_set(ems::app::CanRxSignal::SPEED_KMH, veh);
        ems::app::can_rx_map_set(ems::app::CanRxSignal::WHEEL_SPEED_KMH, whl);

        // vehicle=50, wheel=80 → slip = 30/50 = 60%
        const uint8_t payload[8] = {50u, 0u, 80u, 0u, 0u, 0u, 0u, 0u};
        ems::app::can_rx_map_process(0x250u, payload, 8u, 1000u);

        uint16_t v = 0u, w = 0u;
        CHECK_TRUE(ems::app::can_rx_speed_kmh(v, 1000u), "CAN vehicle speed valid");
        CHECK_TRUE(ems::app::can_rx_wheel_speed_kmh(w, 1000u), "CAN wheel speed valid");
        CHECK_EQ(v, 50u, "vehicle=50 km/h");
        CHECK_EQ(w, 80u, "wheel=80 km/h");

        ems::drv::CkpSnapshot snap{};
        snap.rpm_x10 = 30000u;
        ems::drv::SensorData sens{};
        sens.app_pct_x10 = 800u;
        for (int i = 0; i < 20; ++i) {
            out = ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        }
        CHECK_EQ(ems::engine::torque_manager_test_get_vehicle_kmh(), 50u, "TM latched vehicle");
        CHECK_EQ(ems::engine::torque_manager_test_get_wheel_kmh(), 80u, "TM latched wheel");
        CHECK_TRUE(out.tc_active != 0u, "TC active on CAN slip");
        CHECK_TRUE(out.tc_reduction_pct_x10 > 0u, "TC reduction from CAN slip");
        CHECK_TRUE(out.etb_target_pct_x10 < 1000u, "ETB cut under CAN slip");

        // Clear map (disable signals)
        ems::app::CanSignalDef off = {};
        ems::app::can_rx_map_set(ems::app::CanRxSignal::SPEED_KMH, off);
        ems::app::can_rx_map_set(ems::app::CanRxSignal::WHEEL_SPEED_KMH, off);
        ems::engine::tc_enable = 0u;
        ems::engine::torque_manager_reset();
    }

    // page0 can_rx_map round-trip
    section("page0: can_rx_map serialize/apply");
    {
        ems::app::CanSignalDef def = {};
        def.id = 0x321u;
        def.byte_lo = 2u;
        def.byte_hi = 3u;
        def.shift_right = 0u;
        def.mask = 0xFFFFu;
        def.offset = 0;
        def.timeout_ms = 400u;
        ems::app::can_rx_map_set(ems::app::CanRxSignal::WHEEL_SPEED_KMH, def);
        uint8_t page[512] = {};
        ems::app::can_rx_map_serialize_to_page0(page, sizeof(page));
        ems::app::CanSignalDef off = {};
        ems::app::can_rx_map_set(ems::app::CanRxSignal::WHEEL_SPEED_KMH, off);
        ems::app::can_rx_map_apply_from_page0(page, sizeof(page));
        auto got = ems::app::can_rx_map_get(ems::app::CanRxSignal::WHEEL_SPEED_KMH);
        CHECK_EQ(got.id, 0x321u, "apply wheel id");
        CHECK_EQ(got.byte_lo, 2u, "apply wheel byte_lo");
        CHECK_EQ(got.mask, 0xFFFFu, "apply wheel mask");
        CHECK_EQ(got.timeout_ms, 400u, "apply wheel timeout");
        ems::app::can_rx_map_set(ems::app::CanRxSignal::WHEEL_SPEED_KMH, off);
    }

    // ── ETB target rate-limit (dirigibilidade) ────────────────────────────
    section("torque_manager: etb_max_rate_pct_per_s slews target");
    {
        const uint16_t saved_idle_open = etb_idle_open_pct_x10;
        const uint16_t saved_idle_min  = etb_idle_min_opening_x10;
        // Disable idle/dashpot so the rate test sees pure pedal→target slew.
        etb_idle_open_pct_x10 = 0u;   // app_idle never true (app ≥ 0)
        etb_idle_min_opening_x10 = 0u;

        ems::engine::torque_manager_reset();
        etb_cal_valid = 1u;
        ems::engine::etb_max_rate_pct_per_s = 500u;  // 500 %/s
        ems::drv::CkpSnapshot s{};
        s.rpm_x10 = 15000u;
        ems::drv::SensorData sn{};
        sn.app_pct_x10 = 0u;
        (void)ems::engine::torque_manager_update(s, sn, true, false, false, 8500u, 2u);
        CHECK_EQ(ems::engine::torque_manager_test_get_target(), 0u, "rate: start at 0");

        // APP step to 100% → map 1000. period 2ms × 500%/s → max_delta_x10 = 10
        sn.app_pct_x10 = 1000u;
        out = ems::engine::torque_manager_update(s, sn, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 10u, "rate: first 2ms step → +1.0% (10 x10)");
        CHECK_EQ(out.etb_max_rate_pct_per_s, 500u, "rate: exposes cal rate");

        // After many ticks reaches map target
        for (int i = 0; i < 120; ++i) {
            out = ems::engine::torque_manager_update(s, sn, true, false, false, 8500u, 2u);
        }
        CHECK_EQ(out.etb_target_pct_x10, 1000u, "rate: eventually reaches WOT map");

        // Rev cut bypasses slew → immediate 0
        out = ems::engine::torque_manager_update(s, sn, true, false, true, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 0u, "rate: rev_cut bypasses slew → 0");

        etb_idle_open_pct_x10 = saved_idle_open;
        etb_idle_min_opening_x10 = saved_idle_min;
    }

    ems::engine::etb_max_rate_pct_per_s = saved_rate;
}

// page0 layout v5: launch/TC serialize ↔ apply round-trip + clamps
void test_launch_tc_page0_roundtrip(void) {
    section("page0 v5: launch_tc serialize/apply round-trip");

    // Save defaults so other tests are not polluted
    const uint8_t  s_le  = launch_enable;
    const uint16_t s_lr  = launch_rpm_x10;
    const uint16_t s_let = launch_etb_pct_x10;
    const uint16_t s_la  = launch_app_arm_x10;
    const uint16_t s_ld  = launch_app_disarm_x10;
    const uint16_t s_lh  = launch_rpm_hyst_x10;
    const uint8_t  s_te  = tc_enable;
    const uint16_t s_ta  = tc_app_min_x10;
    const uint16_t s_tr  = tc_rpm_min_x10;
    const uint16_t s_td  = tc_rpm_dot_thresh;
    const uint16_t s_tm  = tc_max_reduction_pct_x10;
    const uint16_t s_ts  = tc_spark_retard_max_deg;
    const uint16_t s_tt  = tc_reduction_rate_x10;

    // Distinct non-default values
    launch_enable           = 1u;
    launch_rpm_x10          = 38000u;  // 3800 RPM
    launch_etb_pct_x10      = 450u;
    launch_app_arm_x10      = 350u;
    launch_app_disarm_x10   = 80u;
    launch_rpm_hyst_x10     = 150u;
    tc_enable               = 1u;
    tc_app_min_x10          = 400u;
    tc_rpm_min_x10          = 25000u;
    tc_rpm_dot_thresh       = 9000u;
    tc_max_reduction_pct_x10 = 700u;
    tc_spark_retard_max_deg = 8u;
    tc_reduction_rate_x10   = 600u;

    uint8_t page[512] = {};
    launch_tc_serialize_to_page0(page, sizeof(page));
    CHECK_EQ(page[191], 1u, "wire: launch_enable @191");
    uint16_t wire_rpm = 0u;
    std::memcpy(&wire_rpm, page + 192, 2u);
    CHECK_EQ(wire_rpm, 38000u, "wire: launch_rpm @192");
    CHECK_EQ(page[202], 1u, "wire: tc_enable @202");
    CHECK_EQ(page[203], 0u, "wire: pad @203");
    uint16_t wire_rate = 0u;
    std::memcpy(&wire_rate, page + 214, 2u);
    CHECK_EQ(wire_rate, 600u, "wire: tc_reduction_rate @214");

    // Mutate RAM then re-apply from wire → restore
    launch_enable = 0u;
    launch_rpm_x10 = 1000u;
    tc_enable = 0u;
    tc_spark_retard_max_deg = 1u;
    launch_tc_apply_from_page0(page, sizeof(page));
    CHECK_EQ(launch_enable, 1u, "apply: launch_enable");
    CHECK_EQ(launch_rpm_x10, 38000u, "apply: launch_rpm");
    CHECK_EQ(launch_etb_pct_x10, 450u, "apply: launch_etb");
    CHECK_EQ(launch_app_arm_x10, 350u, "apply: arm");
    CHECK_EQ(launch_app_disarm_x10, 80u, "apply: disarm");
    CHECK_EQ(launch_rpm_hyst_x10, 150u, "apply: hyst");
    CHECK_EQ(tc_enable, 1u, "apply: tc_enable");
    CHECK_EQ(tc_app_min_x10, 400u, "apply: tc_app_min");
    CHECK_EQ(tc_rpm_min_x10, 25000u, "apply: tc_rpm_min");
    CHECK_EQ(tc_rpm_dot_thresh, 9000u, "apply: rpm_dot");
    CHECK_EQ(tc_max_reduction_pct_x10, 700u, "apply: max_red");
    CHECK_EQ(tc_spark_retard_max_deg, 8u, "apply: spark");
    CHECK_EQ(tc_reduction_rate_x10, 600u, "apply: rate");

    // Short buffer → no-op
    launch_enable = 0u;
    launch_tc_apply_from_page0(page, 200u);  // < 191+25
    CHECK_EQ(launch_enable, 0u, "short buffer: apply no-op");
    launch_tc_serialize_to_page0(nullptr, 512u);  // null guard

    // Clamps: inverted arm/disarm, OOB etb, OOB spark
    std::memset(page, 0, sizeof(page));
    page[191] = 1u;
    uint16_t v = 50000u;  // 5000 RPM ok
    std::memcpy(page + 192, &v, 2u);
    v = 2000u;  // etb 200% → clamp 1000
    std::memcpy(page + 194, &v, 2u);
    v = 50u;    // arm 5%
    std::memcpy(page + 196, &v, 2u);
    v = 300u;   // disarm 30% > arm → swap
    std::memcpy(page + 198, &v, 2u);
    v = 100u;
    std::memcpy(page + 200, &v, 2u);
    page[202] = 1u;
    v = 100u;
    std::memcpy(page + 204, &v, 2u);
    v = 20000u;
    std::memcpy(page + 206, &v, 2u);
    v = 8000u;
    std::memcpy(page + 208, &v, 2u);
    v = 1500u;  // max_red > 1000
    std::memcpy(page + 210, &v, 2u);
    v = 99u;    // spark > 30
    std::memcpy(page + 212, &v, 2u);
    v = 400u;
    std::memcpy(page + 214, &v, 2u);
    launch_tc_apply_from_page0(page, sizeof(page));
    CHECK_EQ(launch_etb_pct_x10, 1000u, "clamp: etb ≤ 100%");
    CHECK_TRUE(launch_app_arm_x10 >= launch_app_disarm_x10, "clamp: arm ≥ disarm");
    CHECK_EQ(launch_app_arm_x10, 300u, "clamp: swapped arm=300");
    CHECK_EQ(launch_app_disarm_x10, 50u, "clamp: swapped disarm=50");
    CHECK_EQ(tc_max_reduction_pct_x10, 1000u, "clamp: max_red ≤ 100%");
    CHECK_EQ(tc_spark_retard_max_deg, 30u, "clamp: spark ≤ 30°");

    // Layout constants
    CHECK_EQ(kLaunchTcPage0Off, 191u, "kLaunchTcPage0Off");
    CHECK_EQ(kLaunchTcPage0Len, 25u, "kLaunchTcPage0Len");
    CHECK_EQ(kCalLayoutVersion, 5u, "layout version 5");

    // Restore
    launch_enable = s_le;
    launch_rpm_x10 = s_lr;
    launch_etb_pct_x10 = s_let;
    launch_app_arm_x10 = s_la;
    launch_app_disarm_x10 = s_ld;
    launch_rpm_hyst_x10 = s_lh;
    tc_enable = s_te;
    tc_app_min_x10 = s_ta;
    tc_rpm_min_x10 = s_tr;
    tc_rpm_dot_thresh = s_td;
    tc_max_reduction_pct_x10 = s_tm;
    tc_spark_retard_max_deg = s_ts;
    tc_reduction_rate_x10 = s_tt;
}

// ═══════════════════════════════════════════════════════════════════════════
// CKP — SEGUNDA FASE (seed_confirmed, seed_rejected, cmp_glitch)
// ═══════════════════════════════════════════════════════════════════════════



