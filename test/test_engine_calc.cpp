// End-to-end tests of one 2 ms engine-control step (engine_calc_step): from
// sensors and calibration tables to what reaches the scheduler. The expected
// values come from physics written out here, not from the code under test.
#include "test/harness.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/calibration.h"
#include "engine/ecu_sched.h"
#include "engine/engine_calc.h"
#include "engine/engine_config.h"
#include "engine/fuel_calc.h"
#include "engine/fuel_trim.h"
#include "engine/limp_gating.h"
#include "engine/map_estimator.h"
#include "engine/quick_crank.h"
#include "engine/table3d.h"
#include "engine/transient_fuel.h"

using namespace ems::engine;

namespace {

struct Saved {
    uint8_t ve[kTableAxisSize][kTableAxisSize];
    int16_t lam[kTableAxisSize][kTableAxisSize];
    int8_t spk[kTableAxisSize][kTableAxisSize];
};
Saved g_saved;

void tables_flat(uint8_t ve, int16_t lambda_x1000, int8_t spark) {
    std::memcpy(g_saved.ve, ve_table, sizeof(ve_table));
    std::memcpy(g_saved.lam, lambda_target_table_x1000, sizeof(lambda_target_table_x1000));
    std::memcpy(g_saved.spk, spark_table, sizeof(spark_table));
    for (uint8_t y = 0u; y < kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < kTableAxisSize; ++x) {
            ve_table[y][x] = ve;
            lambda_target_table_x1000[y][x] = lambda_x1000;
            spark_table[y][x] = spark;
        }
    }
}

void tables_restore() {
    std::memcpy(ve_table, g_saved.ve, sizeof(ve_table));
    std::memcpy(lambda_target_table_x1000, g_saved.lam, sizeof(lambda_target_table_x1000));
    std::memcpy(spark_table, g_saved.spk, sizeof(spark_table));
}

void reset_all() {
    ecu_sched_test_reset();
    engine_calc_reset();
    quick_crank_reset();
    transient_fuel_reset();
    fuel_reset_adaptives();
    map_estimator_init();
    timing_light_enable = 0u;
}

// Warm engine: CLT 90 C, IAT 25 C, 14 V, oil 3 bar, MAP-referenced regulator
// (rail = rated dP + MAP - baro, so the dP correction is exactly 1).
ems::drv::SensorData sensors_at(uint16_t map_kpa, uint16_t app_pct_x10) {
    ems::drv::SensorData s{};
    s.map_bar_x1000 = static_cast<uint16_t>(map_kpa * 10u);
    s.clt_degc_x10 = 900;
    s.iat_degc_x10 = 250;
    s.vbatt_mv = 14000u;
    s.oil_press_bar_x1000 = 3000u;
    s.fuel_press_bar_x1000 = static_cast<uint16_t>(
        fuel_press_nominal_bar_x1000 - (fuel_get_baro_bar_x100() - map_kpa) * 10);
    s.app_pct_x10 = app_pct_x10;
    s.tps_pct_x10 = app_pct_x10;
    return s;
}

EngineCalcIn input_at(uint32_t now_ms, uint32_t rpm, uint16_t map_kpa, uint16_t app_x10,
                      ems::drv::SyncState st = ems::drv::SyncState::FULL_SYNC) {
    EngineCalcIn in{};
    in.now_ms = now_ms;
    in.snap.rpm_x10 = rpm * 10u;
    in.snap.state = st;
    in.snap.tooth_period_ns = static_cast<uint32_t>(60e9 / (rpm * 60.0));
    in.snap.cmp_confirms = 2u;
    in.sensors = sensors_at(map_kpa, app_x10);
    return in;
}

// Runs `seconds` of 2 ms steps at a fixed point; returns the last output.
EngineCalcOut run(uint32_t& t, double seconds, uint32_t rpm, uint16_t map_kpa,
                  uint16_t app_x10,
                  ems::drv::SyncState st = ems::drv::SyncState::FULL_SYNC) {
    EngineCalcOut out{};
    const uint32_t n = static_cast<uint32_t>(seconds * 500.0);
    for (uint32_t i = 0u; i < n; ++i) {
        t += 2u;
        out = engine_calc_step(input_at(t, rpm, map_kpa, app_x10, st));
    }
    return out;
}

// Physics: injected mass per cylinder per cycle at the injector rating.
//   flow_us = REQ_FUEL x VE x (MAP / 101.325 kPa) x (T_ref / T_iat) / lambda
// times the coolant / IAT correction curves (calibration inputs). It is split
// over the 2 openings (semi-sequential, no cam on the host); each opening is
// lengthened so that opening x efficiency(opening) delivers its share (the
// injector small-pulse curve), then adds one dead time.
double eff(double t_us) {
    const uint16_t* ax = injector_scurve_pw_axis_us;
    const uint16_t* q = injector_scurve_corr_q8;
    if (t_us <= ax[0]) { return q[0] / 256.0; }
    for (int i = 1; i < 8; ++i) {
        if (t_us <= ax[i]) {
            const double f = (t_us - ax[i - 1]) / double(ax[i] - ax[i - 1]);
            return (q[i - 1] + f * (q[i] - q[i - 1])) / 256.0;
        }
    }
    return q[7] / 256.0;
}

double expected_pulse_us(double ve_pct, double map_kpa) {
    const double req = default_req_fuel_us();
    const double t_ref = 298.0, t_iat = 273.0 + 25.0;
    double flow = req * (ve_pct / 100.0) * (map_kpa / 101.325) * (t_ref / t_iat);
    flow *= corr_clt(900) / 256.0;
    flow *= corr_iat(250) / 256.0;
    const double dead = corr_vbatt(14000u);
    const double share = flow / 2.0;
    double open = share;
    for (int i = 0; i < 50; ++i) { open = share / eff(open); }  // exact fixed point
    return open + dead;
}

}  // namespace

void test_engine_calc_pw_matches_physics(void) {
    section("engine_calc: injector pulse at the scheduler = physics (idle / part / full load)");
    struct Pt { uint32_t rpm; uint16_t map_kpa; uint8_t ve; uint16_t app; };
    const Pt pts[] = {{900u, 35u, 45u, 0u}, {3000u, 60u, 75u, 250u}, {5500u, 100u, 95u, 900u}};
    for (const Pt& p : pts) {
        reset_all();
        tables_flat(p.ve, 1000, 15);
        uint32_t t = 1000u;
        const EngineCalcOut out = run(t, 3.0, p.rpm, p.map_kpa, p.app);
        const double got_us = out.inj_pw_ticks / 62.5;
        const double exp_us = expected_pulse_us(p.ve, p.map_kpa);
        const double err = 100.0 * (got_us - exp_us) / exp_us;
        std::printf("    %4u rpm MAP %3u kPa VE %2u: pulse %.0f us, physics %.0f us (%+.2f %%)\n",
                    p.rpm, p.map_kpa, p.ve, got_us, exp_us, err);
        CHECK_TRUE(out.committed, "committed to the scheduler");
        CHECK_TRUE(std::fabs(err) < 0.5, "pulse within 0.5 % of physics");
        tables_restore();
    }
}

void test_engine_calc_advance_and_timing_light(void) {
    section("engine_calc: advance from the table; timing light fixes it");
    reset_all();
    tables_flat(60, 1000, 22);
    uint32_t t = 1000u;
    EngineCalcOut out = run(t, 1.0, 3000u, 60u, 250u);
    // 22 deg table, IAT 25 C / CLT 90 C corrections are small (< 1 deg).
    CHECK_TRUE(out.spark_x10 >= 210 && out.spark_x10 <= 230,
               "running advance ~ table 22 deg (+ small corrections)");
    CHECK_EQ(ecu_sched_test_get_advance_x10(), static_cast<int32_t>(out.spark_x10),
             "scheduler holds the same 0.1 deg advance");
    timing_light_enable = 1u;
    timing_light_advance_x10 = 100;
    out = run(t, 0.1, 3000u, 60u, 250u);
    CHECK_EQ(out.spark_x10, 100, "timing light: 10.0 deg exactly, no corrections");
    timing_light_enable = 0u;
    tables_restore();
}

void test_engine_calc_cranking(void) {
    section("engine_calc: HALF sync cranking = REQ x crank mult(CLT), crank spark");
    reset_all();
    tables_flat(60, 1000, 22);
    uint32_t t = 1000u;
    const EngineCalcOut out = run(t, 0.2, 250u, 95u, 0u, ems::drv::SyncState::HALF_SYNC);
    CHECK_TRUE(out.committed, "committed while cranking");
    CHECK_EQ(out.spark_x10, static_cast<int16_t>(crank_spark_deg * 10), "crank spark");
    const QuickCrankOutput qc = quick_crank_update(t, 2500u, true, 900, 0);
    const double flow = default_req_fuel_us() * qc.fuel_mult_x256 / 256.0;
    const double exp_us = (flow + 2.0 * corr_vbatt(14000u)) / 2.0;
    const double got_us = out.inj_pw_ticks / 62.5;
    std::printf("    crank pulse %.0f us, expected %.0f us\n", got_us, exp_us);
    CHECK_TRUE(std::fabs(got_us - exp_us) / exp_us < 0.01, "crank pulse within 1 %");
    tables_restore();
}

void test_engine_calc_cuts(void) {
    section("engine_calc: DFCO by pedal, rev limit keeps spark");
    reset_all();
    tables_flat(60, 1000, 22);
    uint32_t t = 1000u;
    // Closed pedal at 3000 rpm, warm engine: fuel cut after the entry delay.
    EngineCalcOut out = run(t, 3.0, 3000u, 25u, 0u);
    CHECK_EQ(out.net_pw_us, 0u, "pedal closed at 3000 rpm -> DFCO, no fuel");
    // Pedal pressed: fuel back.
    out = run(t, 1.0, 3000u, 60u, 200u);
    CHECK_TRUE(out.net_pw_us > 0u, "pedal 20 % -> fuel back");

    reset_all();
    t = 1000u;
    out = run(t, 0.5, (rev_limit_rpm_x10 / 10u) + 200u, 90u, 900u);
    CHECK_TRUE(g_rev_limit_active, "above the rev limit -> limiter active");
    CHECK_EQ(out.pw_ms_x10, 0u, "fuel shown as cut");
    CHECK_TRUE(out.committed && out.spark_x10 > 0, "spark still committed");
    tables_restore();
}
