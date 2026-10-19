// Driveability helpers: DFCO ramp fraction, PW fall limiter, overrun spark retard, and the
// page-0 block that carries their calibration.
#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "app/can_rx_map.h"
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

void test_ae_de_table(void) {
    section("fuel_calc: tip-out (DE) PW table");
    fuel_reset_adaptives();
    const uint16_t saved_thr = ae_tpsdot_threshold_x10;
    const uint16_t saved_taper = ae_taper_ms;
    ae_tpsdot_threshold_x10 = 10u;
    ae_taper_ms = 8u;
    uint16_t saved[kAeRateTableSize];
    std::memcpy(saved, ae_de_pw_us, sizeof(saved));
    for (uint8_t i = 0u; i < kAeRateTableSize; ++i) { ae_de_pw_us[i] = 0u; }
    fuel_reset_adaptives();
    const int32_t tip = calc_ae_pw_from_tpsdot(100, 800);
    CHECK_TRUE(tip > 0, "pre-condition: tip-in adds fuel");
    fuel_reset_adaptives();
    CHECK_EQ(calc_ae_pw_from_tpsdot(static_cast<int16_t>(-100), 800), -(tip / 2),
             "all-zero DE table keeps half the tip-in adder");
    for (uint8_t i = 0u; i < kAeRateTableSize; ++i) { ae_de_pw_us[i] = ae_pw_adder_us[i]; }
    fuel_reset_adaptives();
    CHECK_EQ(calc_ae_pw_from_tpsdot(static_cast<int16_t>(-100), 800), -tip,
             "a DE table equal to the AE table uses the full adder, negative");
    for (uint8_t i = 0u; i < kAeRateTableSize; ++i) { ae_de_pw_us[i] = ae_pw_adder_us[i] / 4u; }
    fuel_reset_adaptives();
    const int32_t quarter = calc_ae_pw_from_tpsdot(static_cast<int16_t>(-100), 800);
    CHECK_TRUE(quarter < 0 && -quarter < tip / 2, "a smaller DE table removes less fuel than the default");
    fuel_reset_adaptives();
    CHECK_EQ(calc_ae_pw_from_tpsdot(100, 800), tip, "tip-in is unaffected by the DE table");

    section("calibration: DE table in the page-0 block (323-330)");
    static uint8_t page[512];
    ae_de_pw_us[0] = 100u; ae_de_pw_us[1] = 200u; ae_de_pw_us[2] = 300u; ae_de_pw_us[3] = 400u;
    std::memset(page, 0, sizeof(page));
    driveability_serialize_to_page0(page, sizeof(page));
    CHECK_EQ(page[kDriveabilityPage0Off + 47], 100u, "entry 0 low byte");
    CHECK_EQ(page[kDriveabilityPage0Off + 54], 400u >> 8, "entry 3 high byte");
    for (uint8_t i = 0u; i < kAeRateTableSize; ++i) { ae_de_pw_us[i] = 9u; }
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(ae_de_pw_us[2], 300u, "round trip: entry 2");
    page[kDriveabilityPage0Off + 47] = 0xFFu;
    page[kDriveabilityPage0Off + 48] = 0xFFu;
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(ae_de_pw_us[0], 20000u, "entry clamped to 20 ms");
    std::memset(page + kDriveabilityPage0Off, 0, 72u);   // written block of zeros
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(ae_de_pw_us[0], 0u, "a block of zeros clears the DE table (zero is not a sentinel)");
    std::memcpy(ae_de_pw_us, saved, sizeof(saved));
    ae_tpsdot_threshold_x10 = saved_thr;
    ae_taper_ms = saved_taper;
    fuel_reset_adaptives();
}

void test_driveline_retard(void) {
    section("ign_calc: driveline surge retard");
    const uint8_t saved_dl_en = driveline_enable;
    const uint8_t saved_dl_deg = driveline_max_retard_deg;
    const uint16_t saved_dd = driveline_dd_thresh_rpm_x10;
    const uint8_t saved_sp_min = driveline_speed_min_kmh;
    const uint8_t saved_sp_max = driveline_speed_max_kmh;
    const ems::app::CanSignalDef saved_spd =
        ems::app::can_rx_map_get(ems::app::CanRxSignal::SPEED_KMH);
    driveline_enable = 0u;
    driveline_max_retard_deg = 8u;
    driveline_reset();
    CHECK_EQ(calc_driveline_retard_x10(10000u, 200, 80u), 0, "driveline off → 0");
    driveline_enable = 1u;
    driveline_max_retard_deg = 0u;
    CHECK_EQ(calc_driveline_retard_x10(10100u, 200, 80u), 0, "max retard 0 → 0");
    driveline_max_retard_deg = 8u;
    driveline_dd_thresh_rpm_x10 = 50u;
    driveline_reset();
    CHECK_EQ(calc_driveline_retard_x10(10000u, 200, 80u), 0, "first sample has no second difference");
    CHECK_EQ(calc_driveline_retard_x10(10000u, 200, 80u), 0, "second sample has no second difference");
    CHECK_EQ(calc_driveline_retard_x10(10050u, 200, 80u), 20, "second difference arms, slew 2.0°");
    CHECK_EQ(calc_driveline_retard_x10(10150u, 200, 80u), 40, "slew continues to 4.0°");
    CHECK_EQ(calc_driveline_retard_x10(10300u, 200, 80u), 60, "slew continues to 6.0°");
    CHECK_EQ(calc_driveline_retard_x10(10500u, 200, 80u), 80, "slew reaches the 8.0° cap");
    CHECK_EQ(calc_driveline_retard_x10(10750u, 0, 80u), 60, "cold CLT slews back toward 0");
    driveline_dd_thresh_rpm_x10 = 0u;
    driveline_reset();
    (void)calc_driveline_retard_x10(10000u, 200, 80u);
    (void)calc_driveline_retard_x10(10000u, 200, 80u);
    CHECK_EQ(calc_driveline_retard_x10(10050u, 200, 80u), 20,
             "threshold 0 uses the 5.0 rpm×10 default");
    driveline_speed_min_kmh = 30u;
    driveline_speed_max_kmh = 0u;
    ems::app::CanSignalDef spd{};
    spd.id = 0x123u;
    spd.byte_lo = 0u;
    spd.byte_hi = 0xFFu;
    spd.mask = 0xFFu;
    spd.timeout_ms = 0u;
    ems::app::can_rx_map_set(ems::app::CanRxSignal::SPEED_KMH, spd);
    const uint8_t slow = 10u;
    ems::app::can_rx_map_process(0x123u, &slow, 1u, 1u);
    driveline_reset();
    (void)calc_driveline_retard_x10(10000u, 200, 80u);
    (void)calc_driveline_retard_x10(10000u, 200, 80u);
    CHECK_EQ(calc_driveline_retard_x10(10050u, 200, 80u), 0, "speed below minimum blocks retard");
    const uint8_t fast = 80u;
    ems::app::can_rx_map_process(0x123u, &fast, 1u, 2u);
    CHECK_EQ(calc_driveline_retard_x10(10150u, 200, 80u), 20, "speed inside the window arms");
    ems::app::can_rx_map_set(ems::app::CanRxSignal::SPEED_KMH, saved_spd);
    driveline_enable = saved_dl_en;
    driveline_max_retard_deg = saved_dl_deg;
    driveline_dd_thresh_rpm_x10 = saved_dd;
    driveline_speed_min_kmh = saved_sp_min;
    driveline_speed_max_kmh = saved_sp_max;
    driveline_reset();
}

void test_driveline_page0(void) {
    section("calibration: driveline fields in the page-0 block (334-347)");
    const uint8_t s_en = driveline_enable, s_deg = driveline_max_retard_deg;
    const uint16_t s_dd = driveline_dd_thresh_rpm_x10, s_rmin = driveline_rpm_min_x10;
    const uint16_t s_rmax = driveline_rpm_max_x10, s_map = driveline_map_min_bar_x100;
    const int16_t s_clt = driveline_clt_min_x10;
    const uint8_t s_smin = driveline_speed_min_kmh, s_smax = driveline_speed_max_kmh;
    static uint8_t page[512];
    driveline_enable = 1u; driveline_dd_thresh_rpm_x10 = 77u; driveline_max_retard_deg = 6u;
    driveline_rpm_min_x10 = 9000u; driveline_rpm_max_x10 = 40000u; driveline_clt_min_x10 = -50;
    driveline_map_min_bar_x100 = 55u; driveline_speed_min_kmh = 20u; driveline_speed_max_kmh = 120u;
    std::memset(page, 0, sizeof(page));
    driveability_serialize_to_page0(page, sizeof(page));
    CHECK_EQ(page[kDriveabilityPage0Off + 58], 1u, "enable byte");
    CHECK_EQ(page[kDriveabilityPage0Off + 61], 6u, "max retard byte");
    driveline_enable = 0u; driveline_dd_thresh_rpm_x10 = 1u; driveline_max_retard_deg = 0u;
    driveline_rpm_min_x10 = 1u; driveline_rpm_max_x10 = 1u; driveline_clt_min_x10 = 1;
    driveline_map_min_bar_x100 = 1u; driveline_speed_min_kmh = 0u; driveline_speed_max_kmh = 0u;
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(driveline_enable, 1u, "round trip: enable");
    CHECK_EQ(driveline_dd_thresh_rpm_x10, 77u, "round trip: threshold");
    CHECK_EQ(driveline_max_retard_deg, 6u, "round trip: max retard");
    CHECK_EQ(driveline_rpm_min_x10, 9000u, "round trip: rpm min");
    CHECK_EQ(driveline_rpm_max_x10, 40000u, "round trip: rpm max");
    CHECK_EQ(driveline_clt_min_x10, -50, "round trip: CLT min (signed)");
    CHECK_EQ(driveline_map_min_bar_x100, 55u, "round trip: MAP min");
    CHECK_EQ(driveline_speed_min_kmh, 20u, "round trip: speed min");
    CHECK_EQ(driveline_speed_max_kmh, 120u, "round trip: speed max");
    // Zero in a "keep the default" field leaves the value alone; the retard is clamped.
    driveline_dd_thresh_rpm_x10 = 321u;
    page[kDriveabilityPage0Off + 59] = 0u; page[kDriveabilityPage0Off + 60] = 0u;
    page[kDriveabilityPage0Off + 61] = 99u;
    driveability_apply_page0(page, sizeof(page));
    CHECK_EQ(driveline_dd_thresh_rpm_x10, 321u, "zero threshold keeps the current value");
    CHECK_EQ(driveline_max_retard_deg, 30u, "max retard clamped to 30 deg");
    driveline_enable = s_en; driveline_max_retard_deg = s_deg; driveline_dd_thresh_rpm_x10 = s_dd;
    driveline_rpm_min_x10 = s_rmin; driveline_rpm_max_x10 = s_rmax; driveline_clt_min_x10 = s_clt;
    driveline_map_min_bar_x100 = s_map; driveline_speed_min_kmh = s_smin; driveline_speed_max_kmh = s_smax;
}
