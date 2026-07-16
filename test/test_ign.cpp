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

void test_ign_iat_correction(void) {
    section("ign_calc: calc_ign_iat_correction_deg");
    CHECK_EQ(calc_ign_iat_correction_deg(-200),  2,  "IAT=-20°C → +2°");
    CHECK_EQ(calc_ign_iat_correction_deg(0),     1,  "IAT=0°C → +1°");
    CHECK_EQ(calc_ign_iat_correction_deg(200),   0,  "IAT=20°C → 0° (ref)");
    CHECK_EQ(calc_ign_iat_correction_deg(400),  -1,  "IAT=40°C → -1°");
    CHECK_EQ(calc_ign_iat_correction_deg(800),  -5,  "IAT=80°C → -5°");
    CHECK_EQ(calc_ign_iat_correction_deg(1000), -5,  "IAT=100°C → clamped -5°");
}

void test_ign_clt_correction(void) {
    section("ign_calc: calc_ign_clt_correction_deg");
    CHECK_EQ(calc_ign_clt_correction_deg(-400), 0,  "CLT=-40°C → 0°");
    CHECK_EQ(calc_ign_clt_correction_deg(200),  -8, "CLT=20°C → -8° (cat warm-up)");
    CHECK_EQ(calc_ign_clt_correction_deg(600),  0,  "CLT=60°C → 0°");
}

void test_ign_antijerk(void) {
    section("ign_calc: calc_antijerk_retard_deg");
    antijerk_reset();
    const uint16_t saved_thr = antijerk_tpsdot_threshold_x10;
    const int16_t saved_ret = antijerk_retard_deg;
    const uint8_t saved_dec = antijerk_decay_cycles;
    antijerk_tpsdot_threshold_x10 = 30u;
    antijerk_retard_deg = 5;
    antijerk_decay_cycles = 4u;

    CHECK_EQ(calc_antijerk_retard_deg(static_cast<int16_t>(0)), 0, "tpsdot=0 → 0");
    CHECK_EQ(calc_antijerk_retard_deg(static_cast<int16_t>(20)), 0, "below threshold → 0");
    // 1000 ×5 /1000 = 5°
    CHECK_EQ(calc_antijerk_retard_deg(static_cast<int16_t>(1000)), 5, "full tip-in → max 5°");
    antijerk_reset();
    // 200 ×5 /1000 = 1°
    CHECK_EQ(calc_antijerk_retard_deg(static_cast<int16_t>(200)), 1, "mild tip-in → 1°");
    // Decay: armed for 4 cycles (including arm tick), then zeros
    calc_antijerk_retard_deg(static_cast<int16_t>(0));
    calc_antijerk_retard_deg(static_cast<int16_t>(0));
    calc_antijerk_retard_deg(static_cast<int16_t>(0));
    CHECK_EQ(calc_antijerk_retard_deg(static_cast<int16_t>(0)), 0, "decays to 0 after decay_cycles");

    antijerk_tpsdot_threshold_x10 = saved_thr;
    antijerk_retard_deg = saved_ret;
    antijerk_decay_cycles = saved_dec;
    antijerk_reset();
}

void test_ign_clamp_and_total_advance(void) {
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

void test_ign_dwell(void) {
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

void test_ign_get_advance(void) {
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

void test_ign_dwell_vbatt_rpm(void) {
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

void test_ign_idle_spark_correction(void) {
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

