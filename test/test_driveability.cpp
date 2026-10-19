// Driveability helpers: DFCO ramp fraction, PW fall limiter, overrun spark retard, and the
// page-0 block that carries their calibration.
#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "engine/calibration.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"

using namespace ems::engine;

void test_dfco_ramp_fraction(void) {
    section("fuel_calc: DFCO exit ramp fraction (q8)");
    const uint16_t saved_ramp = decel_cut_ramp_ms;
    decel_cut_ramp_ms = 200u;
    fuel_decel_cut_reset();
    CHECK_EQ(fuel_decel_cut_ramp_frac_q8(), 255u, "inactive ramp fraction is 255");
    CHECK_TRUE(fuel_decel_cut_update(20000u, 0u, 800), "ramp test enters DFCO");
    CHECK_FALSE(fuel_decel_cut_update(20000u, 100u, 800), "opening the blade starts the ramp");
    CHECK_EQ(fuel_decel_cut_ramp_frac_q8(), 0u, "ramp starts at fraction 0");
    CHECK_EQ(fuel_decel_cut_ramp_pw(1000u, 100u), 500u, "halfway ramp returns half the pulse");
    CHECK_EQ(fuel_decel_cut_ramp_frac_q8(), 127u, "100/200 ms -> fraction 127");
    CHECK_EQ(fuel_decel_cut_ramp_pw(1000u, 200u), 1000u, "finished ramp returns the full pulse");
    CHECK_EQ(fuel_decel_cut_ramp_frac_q8(), 255u, "finished ramp fraction is 255");
    decel_cut_ramp_ms = saved_ramp;
    fuel_decel_cut_reset();
}

void test_fuel_pw_limit_drop(void) {
    section("fuel_calc: PW fall limiter");
    const uint16_t saved = fuel_pw_drop_max_us;
    fuel_pw_limit_drop_reset();
    fuel_pw_drop_max_us = 0u;
    CHECK_EQ(fuel_pw_limit_drop(8000u), 8000u, "limit off passes the pulse");
    CHECK_EQ(fuel_pw_limit_drop(100u), 100u, "limit off allows a free fall");
    fuel_pw_limit_drop_reset();
    fuel_pw_drop_max_us = 500u;
    CHECK_EQ(fuel_pw_limit_drop(8000u), 8000u, "first sample is stored");
    CHECK_EQ(fuel_pw_limit_drop(100u), 7500u, "fall is capped at 500 us");
    CHECK_EQ(fuel_pw_limit_drop(100u), 7000u, "keeps falling 500 us per step");
    CHECK_EQ(fuel_pw_limit_drop(9000u), 9000u, "a rise is not limited");
    fuel_pw_limit_drop_reset();
    CHECK_EQ(fuel_pw_limit_drop(100u), 100u, "reset forgets the previous pulse");
    fuel_pw_drop_max_us = saved;
    fuel_pw_limit_drop_reset();
}

void test_overrun_spark_retard(void) {
    section("ign_calc: overrun spark retard");
    const uint8_t saved_ov = overrun_spark_retard_deg;
    const uint16_t saved_map_gate = decel_cut_map_max_bar_x100;
    const uint16_t saved_ramp = decel_cut_ramp_ms;
    const uint8_t saved_inh = decel_cut_gear_inhibit_ms10;
    overrun_spark_retard_deg = 0u;
    decel_cut_gear_inhibit_ms10 = 0u;
    fuel_decel_cut_reset();
    CHECK_EQ(calc_overrun_spark_retard_x10(20000u, 0u, 30u, 8500u), 0, "deg 0 stays off");
    overrun_spark_retard_deg = 5u;
    decel_cut_map_max_bar_x100 = 40u;
    CHECK_EQ(calc_overrun_spark_retard_x10(20000u, 0u, 30u, 8500u), 50,
             "trailing closed throttle -> 5.0 deg");
    CHECK_EQ(calc_overrun_spark_retard_x10(9000u, 0u, 30u, 8500u), 0, "inside the idle window -> 0");
    CHECK_EQ(calc_overrun_spark_retard_x10(20000u, 0u, 90u, 8500u), 0, "MAP above the gate -> 0");
    fuel_decel_cut_reset();
    CHECK_TRUE(fuel_decel_cut_update(20000u, 0u, 800), "DFCO on for overrun");
    CHECK_EQ(calc_overrun_spark_retard_x10(20000u, 100u, 90u, 8500u), 50,
             "active cut keeps full retard");
    decel_cut_ramp_ms = 255u;
    CHECK_FALSE(fuel_decel_cut_update(20000u, 100u, 800), "DFCO exit starts the ramp");
    CHECK_EQ(calc_overrun_spark_retard_x10(20000u, 100u, 90u, 8500u), 50,
             "ramp fraction 0 still full retard");
    (void)fuel_decel_cut_ramp_pw(1000u, 127u);
    CHECK_EQ(calc_overrun_spark_retard_x10(20000u, 100u, 90u, 8500u), 25, "half ramp -> 2.5 deg");
    (void)fuel_decel_cut_ramp_pw(1000u, 200u);
    CHECK_EQ(calc_overrun_spark_retard_x10(20000u, 100u, 90u, 8500u), 0, "ramp done -> 0");
    overrun_spark_retard_deg = saved_ov;
    decel_cut_map_max_bar_x100 = saved_map_gate;
    decel_cut_ramp_ms = saved_ramp;
    decel_cut_gear_inhibit_ms10 = saved_inh;
    fuel_decel_cut_reset();

    section("ign_calc: overrun retard is subtracted from the total advance");
    AdvanceCorrectionsX10 corr{};
    corr.overrun_retard = 50;
    CHECK_EQ(calc_total_advance_x10(200, corr), 150, "20.0 deg base - 5.0 deg overrun = 15.0 deg");
}

void test_driveability_page0(void) {
    section("calibration: driveability page-0 block (276-347)");
    const uint16_t saved_drop = fuel_pw_drop_max_us;
    const uint8_t saved_ov = overrun_spark_retard_deg;
    static uint8_t page[512];

    fuel_pw_drop_max_us = 1234u;
    overrun_spark_retard_deg = 7u;
    std::memset(page, 0, sizeof(page));
    driveability_serialize_to_page0(page, sizeof(page));
    CHECK_EQ(page[kDriveabilityPage0Off + 55], 1234u & 0xFFu, "fuel_pw_drop_max_us low byte");
    CHECK_EQ(page[kDriveabilityPage0Off + 56], 1234u >> 8, "fuel_pw_drop_max_us high byte");
    CHECK_EQ(page[kDriveabilityPage0Off + 57], 7u, "overrun_spark_retard_deg");

    fuel_pw_drop_max_us = 0u;
    overrun_spark_retard_deg = 0u;
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(fuel_pw_drop_max_us, 1234u, "round trip: fuel_pw_drop_max_us");
    CHECK_EQ(overrun_spark_retard_deg, 7u, "round trip: overrun_spark_retard_deg");

    page[kDriveabilityPage0Off + 55] = 0xFFu;
    page[kDriveabilityPage0Off + 56] = 0xFFu;
    page[kDriveabilityPage0Off + 57] = 200u;
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(fuel_pw_drop_max_us, 20000u, "PW drop limit clamped to 20 ms");
    CHECK_EQ(overrun_spark_retard_deg, 30u, "overrun retard clamped to 30 deg");

    fuel_pw_drop_max_us = 321u;
    overrun_spark_retard_deg = 9u;
    std::memset(page, 0xFF, sizeof(page));   // erased flash
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(fuel_pw_drop_max_us, 321u, "blank block keeps the defaults (PW drop)");
    CHECK_EQ(overrun_spark_retard_deg, 9u, "blank block keeps the defaults (overrun)");

    fuel_pw_drop_max_us = saved_drop;
    overrun_spark_retard_deg = saved_ov;
}
