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
    section("ign_calc: calc_ign_iat_correction_x10");
    CHECK_EQ(calc_ign_iat_correction_x10(-200),  20, "IAT=-20°C → +2.0°");
    CHECK_EQ(calc_ign_iat_correction_x10(0),     10, "IAT=0°C → +1.0°");
    CHECK_EQ(calc_ign_iat_correction_x10(100),    5, "IAT=10°C → +0.5° (between nodes, no truncation)");
    CHECK_EQ(calc_ign_iat_correction_x10(200),    0, "IAT=20°C → 0 (ref)");
    CHECK_EQ(calc_ign_iat_correction_x10(400),  -10, "IAT=40°C → -1.0°");
    CHECK_EQ(calc_ign_iat_correction_x10(800),  -50, "IAT=80°C → -5.0°");
    CHECK_EQ(calc_ign_iat_correction_x10(1000), -50, "IAT=100°C → clamped -5.0°");
}

void test_ign_clt_correction(void) {
    section("ign_calc: calc_ign_clt_correction_x10");
    CHECK_EQ(calc_ign_clt_correction_x10(-400), 0,   "CLT=-40°C → 0");
    CHECK_EQ(calc_ign_clt_correction_x10(200),  -80, "CLT=20°C → -8.0° (cat warm-up)");
    CHECK_EQ(calc_ign_clt_correction_x10(600),  0,   "CLT=60°C → 0");
}

void test_ign_antijerk(void) {
    section("ign_calc: calc_antijerk_retard_x10");
    antijerk_reset();
    const uint16_t saved_thr = antijerk_tpsdot_threshold_x10;
    const int16_t saved_ret = antijerk_retard_deg;
    const uint8_t saved_dec = antijerk_decay_cycles;
    antijerk_tpsdot_threshold_x10 = 30u;
    antijerk_retard_deg = 5;
    antijerk_decay_cycles = 4u;

    CHECK_EQ(calc_antijerk_retard_x10(static_cast<int16_t>(0)), 0, "tpsdot=0 → 0");
    CHECK_EQ(calc_antijerk_retard_x10(static_cast<int16_t>(20)), 0, "below threshold → 0");
    CHECK_EQ(calc_antijerk_retard_x10(static_cast<int16_t>(1000)), 50, "full tip-in → max 5.0°");
    antijerk_reset();
    // 300 %/s×10 of 1000 → 30 % of 5° = 1.5° (whole degrees gave 1°)
    CHECK_EQ(calc_antijerk_retard_x10(static_cast<int16_t>(300)), 15, "mild tip-in → 1.5°");
    // Decay: armed for 4 cycles (including arm tick), then zeros
    calc_antijerk_retard_x10(static_cast<int16_t>(0));
    calc_antijerk_retard_x10(static_cast<int16_t>(0));
    calc_antijerk_retard_x10(static_cast<int16_t>(0));
    CHECK_EQ(calc_antijerk_retard_x10(static_cast<int16_t>(0)), 0, "decays to 0 after decay_cycles");

    antijerk_tpsdot_threshold_x10 = saved_thr;
    antijerk_retard_deg = saved_ret;
    antijerk_decay_cycles = saved_dec;
    antijerk_reset();
}

void test_ign_clamp_and_total_advance(void) {
    section("ign_calc: clamp_advance_x10 / calc_total_advance_x10");
    CHECK_EQ(clamp_advance_x10(600),  600,  "60.0° at max");
    CHECK_EQ(clamp_advance_x10(615),  600,  "61.5° clamped to 60.0°");
    CHECK_EQ(clamp_advance_x10(-200), -200, "-20.0° at min");
    CHECK_EQ(clamp_advance_x10(-215), -200, "-21.5° clamped to -20.0°");
    AdvanceCorrectionsX10 c{};
    CHECK_EQ(calc_total_advance_x10(253, c), 253, "base=25.3 no corr → 25.3");
    c.iat = -35;
    CHECK_EQ(calc_total_advance_x10(253, c), 218, "iat=-3.5 → 21.8");
    c = {}; c.clt = -80; c.antijerk_retard = 15;
    CHECK_EQ(calc_total_advance_x10(250, c), 155, "clt=-8 antijerk=1.5 → 15.5");
    c = {}; c.idle = 50; c.torque_retard = 20;
    CHECK_EQ(calc_total_advance_x10(150, c), 180, "idle=+5 torque=2 → 18.0");
    c = {}; c.clt = -400;
    CHECK_EQ(calc_total_advance_x10(50, c), -200, "after-TDC result clamped at -20.0");
}

void test_ign_dwell(void) {
    section("ign_calc: dwell_ms_x10_from_vbatt / inj_pw_us_to_scheduler_ticks");
    CHECK_NEAR(static_cast<float>(dwell_ms_x10_from_vbatt(12000u)), 30.0f, 5.0f, "dwell@12V≈3.0ms");
    CHECK_TRUE(dwell_ms_x10_from_vbatt(9000u) > dwell_ms_x10_from_vbatt(14000u), "monotonic dwell");
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(1000u), 62500u, "1000µs = 62500 ticks (16 ns)");
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(0u), 0u, "0µs→0");
}

// ═══════════════════════════════════════════════════════════════════════════
// AUXILIARIES
// ═══════════════════════════════════════════════════════════════════════════

void test_ign_get_advance(void) {
    section("ign_calc: get_advance_x10 interpolates the table at 0.1°");
    static int8_t saved[kTableAxisSize][kTableAxisSize];
    std::memcpy(saved, spark_table, sizeof(saved));
    // Two adjacent RPM cells 20° and 25°: halfway must be 22.5° (whole-degree
    // pipeline gave 22°).
    for (uint8_t y = 0u; y < kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < kTableAxisSize; ++x) {
            spark_table[y][x] = static_cast<int8_t>(x < 1u ? 20 : 25);
        }
    }
    const uint32_t mid = (kRpmAxisX10[0] + kRpmAxisX10[1]) / 2u;
    CHECK_EQ(get_advance_x10(mid, 100u), 225, "midpoint 20..25° → 22.5°");
    const Table2dLookup lk = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100, mid, 100u);
    CHECK_EQ(get_advance_x10_prepared(lk), 225, "prepared == direct");
    CHECK_EQ(get_advance_x10(100u, 10u), 200, "below axis → first cell 20.0°");
    // Negative (after TDC) cells survive.
    spark_table[0][0] = -5; spark_table[1][0] = -5;
    CHECK_EQ(get_advance_x10(kRpmAxisX10[0], kLoadAxisBarX100[0]), -50, "cell -5° → -5.0°");
    std::memcpy(spark_table, saved, sizeof(saved));
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
    section("ign_calc: calc_idle_spark_correction_x10");

    // Calibration defaults:
    //   idle_spark_tps_max_x10=25, idle_spark_map_max_bar_x100=80
    //   idle_spark_rpm_min_x10=5000, idle_spark_window_above_target_x10=4000
    //   idle_spark_deadband_rpm_x10=500, idle_spark_rpm_per_deg_x10=500
    //   retard_limit=-8, advance_limit=12

    // Conditions NOT met: TPS too high
    CHECK_EQ(calc_idle_spark_correction_x10(8500u, 8500u, 30u, 60u), 0,
             "tps > max → 0");

    // Conditions NOT met: MAP too high
    CHECK_EQ(calc_idle_spark_correction_x10(8500u, 8500u, 0u, 90u), 0,
             "map > max → 0");

    // Within deadband: error=0 < 500
    CHECK_EQ(calc_idle_spark_correction_x10(8500u, 8500u, 0u, 60u), 0,
             "rpm == target (within deadband) → 0");

    // Below target by 1500 x10 (150 RPM): error=1500, -deadband=1000 → corr=1000/500=2° advance
    const int16_t corr_low = calc_idle_spark_correction_x10(7000u, 8500u, 0u, 60u);
    CHECK_EQ(corr_low, 20, "150 RPM below target → +2.0° advance");

    // Above target by 1500 x10: error=-1500, +deadband=-1000 → corr=-1000/500=-2° retard
    const int16_t corr_high = calc_idle_spark_correction_x10(10000u, 8500u, 0u, 60u);
    CHECK_EQ(corr_high, -20, "150 RPM above target → -2.0° retard");

    // Advance clamped at advance_limit=12: need idle_target big enough so
    // rpm (>= rpm_min=5000) still has error > deadband + 12*rpm_per_deg (=6500).
    // Use idle_target=20000, rpm=5000: error=15000, -deadband=14500 → 29° → clamped at 12.
    const int16_t corr_clamp = calc_idle_spark_correction_x10(5000u, 20000u, 0u, 60u);
    CHECK_EQ(corr_clamp, 120, "large underspeed (5000 vs target 20000) → clamped at +12.0°");
}

// ═══════════════════════════════════════════════════════════════════════════
// ETB CONTROL — C++ namespace (ems::engine)
// ═══════════════════════════════════════════════════════════════════════════

