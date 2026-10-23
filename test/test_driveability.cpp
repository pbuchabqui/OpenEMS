// Driveability helpers: DFCO ramp fraction, PW fall limiter, overrun spark retard, and the
// page-0 block that carries their calibration.
#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "app/can_rx_map.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/etb_control.h"
#include "engine/quick_crank.h"
#include "engine/torque_manager.h"
#include "engine/calibration.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"

using namespace ems::engine;
using namespace ems::app;

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
    {
        const uint16_t s_ref = stft_gain_ref_delay_ms;
        stft_gain_ref_delay_ms = 640u;
        std::memset(page, 0, sizeof(page));
        driveability_serialize_to_page0(page, sizeof(page));
        CHECK_EQ(page[348], 640u & 0xFFu, "STFT gain ref delay low byte (348)");
        CHECK_EQ(page[349], 640u >> 8, "STFT gain ref delay high byte (349)");
        stft_gain_ref_delay_ms = 0u;
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(stft_gain_ref_delay_ms, 640u, "round trip: STFT gain ref delay");
        page[348] = 0xFFu; page[349] = 0xFFu;
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(stft_gain_ref_delay_ms, 2000u, "STFT gain ref delay limitado a 2000 ms");
        stft_gain_ref_delay_ms = s_ref;
    }
    {
        const uint16_t s_rise = tc_flare_app_rise_x10;
        const uint16_t s_hold = tc_flare_hold_ms;
        tc_flare_app_rise_x10 = 300u;
        tc_flare_hold_ms = 250u;
        std::memset(page, 0, sizeof(page));
        driveability_serialize_to_page0(page, sizeof(page));
        CHECK_EQ(page[350], 300u & 0xFFu, "TC flare APP rise low byte (350)");
        CHECK_EQ(page[351], 300u >> 8, "TC flare APP rise high byte (351)");
        CHECK_EQ(page[352], 250u, "TC flare hold low byte (352)");
        tc_flare_app_rise_x10 = 0u;
        tc_flare_hold_ms = 0u;
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(tc_flare_app_rise_x10, 300u, "round trip: TC flare APP rise");
        CHECK_EQ(tc_flare_hold_ms, 250u, "round trip: TC flare hold");
        page[350] = 0u; page[351] = 0u; page[352] = 0xFFu; page[353] = 0xFFu;
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(tc_flare_app_rise_x10, 300u, "TC flare APP rise 0 = mantém");
        CHECK_EQ(tc_flare_hold_ms, 1000u, "TC flare hold limitado a 1000 ms");
        tc_flare_app_rise_x10 = s_rise;
        tc_flare_hold_ms = s_hold;
    }
    {
        const uint16_t s_taper = etb_taper_ms;
        etb_taper_ms = 2500u;
        std::memset(page, 0, sizeof(page));
        driveability_serialize_to_page0(page, sizeof(page));
        CHECK_EQ(page[354], 2500u & 0xFFu, "ETB taper low byte (354)");
        etb_taper_ms = 0u;
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(etb_taper_ms, 2500u, "round trip: ETB taper");
        page[354] = 0xFFu; page[355] = 0xFFu;
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(etb_taper_ms, 10000u, "ETB taper limitado a 10000 ms");
        page[354] = 0u; page[355] = 0u;
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(etb_taper_ms, 0u, "ETB taper 0 = segue o afterstart");
        etb_taper_ms = s_taper;
    }
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

void test_idle_air_and_dashpot(void) {
    section("torque_manager: idle I steps, load air feed-forward, dashpot (fan / gear / A-C)");
    ems::drv::CkpSnapshot snap{};
    snap.rpm_x10 = 30000u;
    ems::drv::SensorData sens{};
    sens.app_pct_x10 = 800u;
    TorqueOutput out{};
    torque_manager_init();   // clears a limp left behind by other suites
    tc_enable = 0u;
    const etb_drive_mode_t saved_mode = etb_get_drive_mode();
    etb_set_drive_mode(static_cast<etb_drive_mode_t>(0));   // pedal map of the default mode
    const uint16_t saved_rate = etb_max_rate_pct_per_s;
    etb_max_rate_pct_per_s = 0u;                            // no slew limit: assert the raw target
    torque_manager_reset();
    {
        const uint16_t saved_min = etb_idle_min_opening_x10;
        const uint16_t saved_max = etb_idle_max_opening_x10;
        const uint16_t saved_open = etb_idle_open_pct_x10;
        const uint16_t saved_caps[4] = {
            dashpot_cap_x10[0], dashpot_cap_x10[1],
            dashpot_cap_x10[2], dashpot_cap_x10[3]};
        const uint16_t saved_taus[4] = {
            dashpot_tau_ms[0], dashpot_tau_ms[1],
            dashpot_tau_ms[2], dashpot_tau_ms[3]};
        const uint16_t saved_fan = idle_fan_air_x10;
        const uint16_t saved_gear = idle_gear_air_x10;
        const uint16_t saved_ac = idle_ac_air_x10;
        const CanSignalDef saved_gear_def = can_rx_map_get(CanRxSignal::GEAR);

        etb_cal_valid = 1u;
        etb_idle_open_pct_x10 = 80u;
        etb_idle_min_opening_x10 = 0u;
        etb_idle_max_opening_x10 = 200u;
        for (uint8_t i = 0u; i < 4u; ++i) {
            dashpot_cap_x10[i] = 1u;
        }
        idle_fan_air_x10 = 0u;
        idle_gear_air_x10 = 0u;
        idle_ac_air_x10 = 0u;
        torque_idle_set_ac_load(0u);
        auxiliaries_force_fan(false);
        quick_crank_reset();
        torque_manager_reset();

        snap.rpm_x10 = 7000u;
        sens.app_pct_x10 = 0u;
        for (int i = 0; i < 5; ++i) {
            out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        }
        CHECK_EQ(out.etb_target_pct_x10, 10u, "five under-target ticks open by 2");
        snap.rpm_x10 = 10000u;
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 9u, "one over-target tick closes by 1");

        torque_manager_reset();
        snap.rpm_x10 = 7000u;
        idle_fan_air_x10 = 50u;
        auxiliaries_force_fan(true);
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 52u, "fan air adds to the idle floor");
        auxiliaries_force_fan(false);
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 4u, "fan air is not stored in the integrator");

        idle_ac_air_x10 = 10u;
        torque_idle_set_ac_load(1u);
        idle_gear_air_x10 = 15u;
        CanSignalDef gear_def{};
        gear_def.id = 0x111u;
        gear_def.byte_lo = 0u;
        gear_def.byte_hi = 0xFFu;
        gear_def.mask = 0xFFu;
        gear_def.timeout_ms = 0u;
        can_rx_map_set(CanRxSignal::GEAR, gear_def);
        const uint8_t gear_byte = 3u;
        can_rx_map_process(0x111u, &gear_byte, 1u, 1u);
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 31u, "AC and engaged gear add air");
        can_rx_map_set(CanRxSignal::GEAR, saved_gear_def);
        torque_idle_set_ac_load(0u);
        idle_fan_air_x10 = 0u;
        idle_gear_air_x10 = 0u;
        idle_ac_air_x10 = 0u;

        for (uint8_t i = 0u; i < 4u; ++i) {
            dashpot_cap_x10[i] = 350u;
            dashpot_tau_ms[i] = 400u;
        }
        torque_manager_reset();
        snap.rpm_x10 = 15000u;
        sens.app_pct_x10 = 1000u;
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 1000u, "wide open pedal is not idle");
        sens.app_pct_x10 = 0u;
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 350u, "tip-out seeds the 35% dashpot cap");
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 349u, "tau 400 ms decays one count per 2 ms");

        for (uint8_t i = 0u; i < 4u; ++i) {
            dashpot_tau_ms[i] = 50u;
        }
        torque_manager_reset();
        sens.app_pct_x10 = 1000u;
        (void)torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        sens.app_pct_x10 = 0u;
        (void)torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 336u, "tau 50 ms decays 14 counts");

        for (uint8_t i = 0u; i < 4u; ++i) {
            dashpot_tau_ms[i] = 400u;
        }
        torque_manager_reset();
        snap.rpm_x10 = 26000u;
        sens.app_pct_x10 = 1000u;
        (void)torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        sens.app_pct_x10 = 0u;
        (void)torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(out.etb_target_pct_x10, 347u, "rpm above 2500 halves tau to 200 ms");

        etb_idle_min_opening_x10 = saved_min;
        etb_idle_max_opening_x10 = saved_max;
        etb_idle_open_pct_x10 = saved_open;
        for (uint8_t i = 0u; i < 4u; ++i) {
            dashpot_cap_x10[i] = saved_caps[i];
            dashpot_tau_ms[i] = saved_taus[i];
        }
        idle_fan_air_x10 = saved_fan;
        idle_gear_air_x10 = saved_gear;
        idle_ac_air_x10 = saved_ac;
        torque_idle_set_ac_load(0u);
        auxiliaries_force_fan(false);
        torque_manager_reset();
    }
    etb_set_drive_mode(saved_mode);
    etb_max_rate_pct_per_s = saved_rate;
}

void test_driveability_wire_rules(void) {
    section("calibration: wire rules (zero keeps the default, 255 = Q8 256, a zero block clears the DE table)");
    {
        const uint8_t saved_rec = idle_i_step_recover;
        const uint8_t saved_cls = idle_i_step_close;
        const uint16_t saved_tau0 = dashpot_tau_ms[0];
        const uint16_t saved_tau1 = dashpot_tau_ms[1];
        const uint16_t saved_fac = dashpot_fast_factor_q8;
        const uint16_t saved_de = ae_de_pw_us[0];
        const int16_t saved_spk = idle_spark_err_corr_x10[0];
        uint8_t blank[512] = {};
        driveability_apply_page0(blank, sizeof(blank));
        CHECK_EQ(idle_i_step_recover, 2u, "zero wire keeps recover step 2");
        CHECK_EQ(idle_i_step_close, 1u, "zero wire keeps close step 1");
        CHECK_EQ(dashpot_tau_ms[0], 400u, "zero wire keeps dashpot tau 400");
        CHECK_EQ(dashpot_cap_x10[0], 350u, "zero wire keeps dashpot cap 35%");
        CHECK_EQ(dashpot_fast_rpm_x10, 25000u, "zero wire keeps fast-rpm gate");
        CHECK_EQ(dashpot_fast_factor_q8, 128u, "zero wire keeps Q8 128");
        CHECK_EQ(ae_de_pw_us[0], 0u, "zero DE table stays the half-adder fallback");
        CHECK_EQ(idle_spark_err_corr_x10[0], static_cast<int16_t>(0),
                 "zero idle-spark curve stays linear");
        CHECK_EQ(idle_fan_air_x10, 0u, "zero fan air is a real zero");
        CHECK_EQ(driveline_dd_thresh_rpm_x10, 50u, "zero threshold keeps 5.0 rpm×10");

        uint8_t page[512] = {};
        page[kDriveabilityPage0Off + 8] = 5u;
        const uint16_t tau1 = 200u;
        std::memcpy(page + kDriveabilityPage0Off + 18, &tau1, 2u);
        page[kDriveabilityPage0Off + 34] = 255u;
        const uint16_t de0 = 100u;
        std::memcpy(page + kDriveabilityPage0Off + 47, &de0, 2u);
        driveability_apply_page0(page, sizeof(page));
        CHECK_EQ(idle_i_step_recover, 5u, "non-zero recover step is copied");
        CHECK_EQ(dashpot_tau_ms[0], 400u, "a zero tau point stays 400");
        CHECK_EQ(dashpot_tau_ms[1], 200u, "a non-zero tau point is copied");
        CHECK_EQ(dashpot_fast_factor_q8, 256u, "wire 255 means Q8 256");
        CHECK_EQ(ae_de_pw_us[0], 100u, "DE table zero is not a sentinel");
        uint8_t again[512] = {};
        driveability_apply_page0(again, sizeof(again));
        CHECK_EQ(idle_i_step_recover, 5u, "a later zero block does not wipe recover");
        CHECK_EQ(dashpot_tau_ms[1], 200u, "a later zero block does not wipe tau");
        CHECK_EQ(ae_de_pw_us[0], 0u, "a later zero block clears the DE table");
        uint8_t outp[512] = {};
        driveability_serialize_to_page0(outp, sizeof(outp));
        CHECK_EQ(outp[kDriveabilityPage0Off + 8], 5u, "serialize writes the resolved recover step");
        CHECK_EQ(outp[kDriveabilityPage0Off + 16], static_cast<uint8_t>(400u & 0xFFu), "serialize writes tau 400");
        CHECK_EQ(outp[kDriveabilityPage0Off + 34], 255u, "serialize writes Q8 256 as 255");
        CHECK_EQ(outp[kDriveabilityPage0Off + 47], 0u, "serialize keeps a zero DE table");
        CHECK_EQ(outp[kDriveabilityPage0Off + 35], 0u, "serialize does not invent the linear idle curve");
        driveability_apply_page0(again, 100u);
        CHECK_EQ(idle_i_step_recover, 5u, "short page is a no-op");

        idle_i_step_recover = saved_rec;
        idle_i_step_close = saved_cls;
        dashpot_tau_ms[0] = saved_tau0;
        dashpot_tau_ms[1] = saved_tau1;
        dashpot_fast_factor_q8 = saved_fac;
        ae_de_pw_us[0] = saved_de;
        idle_spark_err_corr_x10[0] = saved_spk;
    }}

void test_dfco_entry_shaped_by_clt(void) {
    section("fuel_calc: DFCO entry rpm shaped by CLT");
    const uint32_t s_entry = decel_cut_entry_rpm_x10;
    const uint32_t s_exit = decel_cut_exit_rpm_x10;
    int16_t s_d[kDashpotCurvePts];
    std::memcpy(s_d, decel_entry_delta_rpm_x10, sizeof(s_d));
    for (uint8_t i = 0u; i < kDashpotCurvePts; ++i) { decel_entry_delta_rpm_x10[i] = 0; }
    decel_cut_entry_rpm_x10 = 15000u;
    decel_cut_exit_rpm_x10 = 12000u;
    decel_entry_delta_rpm_x10[3] = 3000;
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_update(16000u, 0u, 1000), "CLT curve raises entry above 1600 rpm");
    CHECK_TRUE(fuel_decel_cut_update(19000u, 0u, 1000), "1900 rpm still enters the raised gate");
    decel_entry_delta_rpm_x10[3] = -20000;
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_update(11000u, 0u, 1000), "entry clamped up to the exit rpm");
    CHECK_TRUE(fuel_decel_cut_update(12000u, 0u, 1000), "clamped entry equals the exit rpm");
    decel_entry_delta_rpm_x10[3] = 0;
    fuel_decel_cut_reset();
    CHECK_TRUE(fuel_decel_cut_update(16000u, 0u, 1000), "flat deltas use the scalar entry rpm");
    decel_cut_entry_rpm_x10 = s_entry;
    decel_cut_exit_rpm_x10 = s_exit;
    std::memcpy(decel_entry_delta_rpm_x10, s_d, sizeof(s_d));
    fuel_decel_cut_reset();
}

void test_idle_spark_error_curve(void) {
    section("ign_calc: idle spark error curve replaces the linear law");
    int16_t s_c[kIdleSparkCurvePts];
    std::memcpy(s_c, idle_spark_err_corr_x10, sizeof(s_c));
    for (uint8_t i = 0u; i < kIdleSparkCurvePts; ++i) { idle_spark_err_corr_x10[i] = 0; }
    CHECK_EQ(calc_idle_spark_correction_x10(7000u, 8500u, 0u, 60u), 20,
             "all-zero curve: linear law, 150 rpm below -> +2.0 deg");
    idle_spark_err_corr_x10[3] = 40;
    idle_spark_err_corr_x10[4] = 100;
    CHECK_EQ(calc_idle_spark_correction_x10(7000u, 8500u, 0u, 60u), 80, "curve at error 1500 -> 8.0 deg");
    CHECK_EQ(calc_idle_spark_correction_x10(8500u, 8500u, 0u, 60u), 0, "curve still honors the deadband");
    CHECK_EQ(calc_idle_spark_correction_x10(7000u, 8500u, 30u, 60u), 0, "curve still honors the TPS window");
    std::memcpy(idle_spark_err_corr_x10, s_c, sizeof(s_c));
}
