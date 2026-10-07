/**
 * Fuel chain against an independent physical oracle.
 *
 * Expected values are computed here in double precision from first
 * principles (ideal gas, injector mass flow) — never from the firmware's
 * fixed-point formulas. Tolerances are the fixed-point resolution only.
 */
#include "test/harness.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "engine/engine_config.h"
#include "engine/fuel_calc.h"
#include "engine/transient_fuel.h"
#include "engine/xtau_autocalib.h"
#include "engine/fuel_trim.h"

using namespace ems::engine;

namespace {

constexpr double kPrefKpa = 101.325;        // reference of kAirDensityMgPerCcX1000
constexpr double kTrefK = 298.15;           // 25 °C
constexpr double kRair = 287.05;            // J/(kg·K)
constexpr double kFuelDensityGcc = 0.755;   // gasoline

// Air density at the reference state, mg/cc.
double rho_air_ref_mg_cc() { return kPrefKpa * 1000.0 / (kRair * kTrefK); }

// Injector open time (flow part, µs) for one cylinder filled at 100 % VE at
// the reference state, lambda 1.
double req_fuel_us(double disp_cc, int cyl, double flow_cc_min, double afr)
{
    const double air_mg = disp_cc / cyl * rho_air_ref_mg_cc();
    const double fuel_mg = air_mg / afr;
    const double flow_mg_us = flow_cc_min * kFuelDensityGcc * 1000.0 / 60e6;
    return fuel_mg / flow_mg_us;
}

// Speed-density flow time for an operating point.
double flow_us(double req_us, double ve_pct, double map_kpa, double iat_c, double lambda)
{
    return req_us * (ve_pct / 100.0) * (map_kpa / kPrefKpa) *
           (kTrefK / (iat_c + 273.15)) / lambda;
}

double rel_err(double a, double b) { return std::fabs(a - b) / b; }

struct EngCfgGuard {
    cfg::EngineConfigRam saved = cfg::g_eng_cfg;
    ~EngCfgGuard() { cfg::g_eng_cfg = saved; fuel_set_baro_bar_x100(cfg::kMapRefBarX100); }
};

}  // namespace

void test_fuel_physics_req_fuel(void) {
    section("fuel physics: REQ_FUEL vs ideal-gas oracle");
    struct { uint16_t cc, flow, afr_x100; } cases[] = {
        {2000u, 450u, 1300u}, {1600u, 240u, 1470u}, {2000u, 1000u, 1470u}, {1000u, 190u, 1470u},
    };
    for (const auto& c : cases) {
        const double exp = req_fuel_us(c.cc, 4, c.flow, c.afr_x100 / 100.0);
        const uint32_t got = calc_req_fuel_us(c.cc, 4u, c.flow, c.afr_x100);
        CHECK_TRUE(rel_err(got, exp) < 0.002, "REQ_FUEL within 0.2 % of physics");
    }
    // Hand-checked anchor from the review: 2.0 L, 450 cc/min, AFR 13.0 → 8042 µs.
    CHECK_TRUE(std::fabs(req_fuel_us(2000, 4, 450, 13.0) - 8042.0) < 2.0, "oracle anchor 8042 us");
}

void test_fuel_physics_pw_points(void) {
    section("fuel physics: speed-density PW at operating points");
    EngCfgGuard guard;
    cfg::g_eng_cfg.displacement_cc = 2000u;
    cfg::g_eng_cfg.injector_flow_cc_min = 450u;
    cfg::g_eng_cfg.stoich_afr_x100 = 1300u;
    fuel_set_baro_bar_x100(100u);
    const double req = req_fuel_us(2000, 4, 450, 13.0);
    struct { uint8_t ve; uint16_t map; int16_t iat_x10; uint16_t lambda; } pts[] = {
        {45u, 30u, 250, 1000u},    // idle
        {80u, 60u, 250, 1000u},    // cruise
        {95u, 100u, 250, 870u},    // WOT NA, rich
        {90u, 200u, 400, 800u},    // boost, hot air
        {70u, 45u, -100, 1000u},   // cold air
        {85u, 300u, 600, 780u},    // top of the MAP range
    };
    for (const auto& p : pts) {
        const double exp = flow_us(req, p.ve, p.map, p.iat_x10 / 10.0, p.lambda / 1000.0);
        const uint32_t got = calc_fuel_pw_us_default_fast(
            p.ve, p.map, corr_iat_density_q8(p.iat_x10), p.lambda, 0, 256u, 256u, 0u);
        std::printf("    VE %3u MAP %3u kPa IAT %5.1f C lambda %.3f: expected %7.1f us, got %6u us (%+.2f %%)\n",
                    p.ve, p.map, p.iat_x10 / 10.0, p.lambda / 1000.0, exp, got,
                    (got - exp) / exp * 100.0);
        // Q8 IAT density (1/256) + integer steps: 0.6 %.
        CHECK_TRUE(rel_err(got, exp) < 0.006, "PW within 0.6 % of speed-density physics");
    }
}

void test_fuel_physics_altitude_and_limits(void) {
    section("fuel physics: barometric pressure does not change cylinder air at a given MAP");
    EngCfgGuard guard;
    fuel_set_baro_bar_x100(100u);
    const uint32_t sea = calc_fuel_pw_us_default_fast(80u, 50u, 256u, 1000u, 0, 256u, 256u, 0u);
    fuel_set_baro_bar_x100(85u);
    const uint32_t alt = calc_fuel_pw_us_default_fast(80u, 50u, 256u, 1000u, 0, 256u, 256u, 0u);
    CHECK_EQ(alt, sea, "same MAP, VE, IAT → same fuel at 0.85 bar baro (no +18 %)");

    section("fuel physics: MAP above the sensor range never cuts fuel");
    fuel_set_baro_bar_x100(100u);
    const uint32_t at300 = calc_fuel_pw_us_default_fast(85u, 300u, 256u, 800u, 0, 256u, 256u, 0u);
    const uint32_t at320 = calc_fuel_pw_us_default_fast(85u, 320u, 256u, 800u, 0, 256u, 256u, 0u);
    CHECK_TRUE(at300 > 0u, "MAP 300 kPa fuelled");
    CHECK_TRUE(at320 >= at300, "MAP 320 kPa fuelled at least as much as 300 kPa (was 0)");

    section("fuel physics: dead time is added once per opening, never scaled");
    CHECK_EQ(inj_pulse_pw_us(2000u, 800u, 1u), 2800u, "sequential: flow + dead");
    CHECK_EQ(inj_pulse_pw_us(2000u, 800u, 2u), 1800u, "2 openings: flow/2 + dead");
    CHECK_EQ(inj_cycle_pw_us(2000u, 800u, 2u), 3600u, "cycle on-time: flow + 2 dead");
}

// Wall-film (X-tau, Aquino) oracle: per engine cycle the cylinder receives
// (1-X)·m_inj + M/tau and the film gains X·m_inj − M/tau. With compensation
// m_inj = (m_des − M/tau)/(1−X) the cylinder gets exactly m_des every cycle,
// so in steady state injected == desired. Driven by the real 2 ms loop rate.
static uint32_t xtau_run(uint32_t desired_us, uint32_t rpm_x10, int16_t clt_x10, double seconds)
{
    uint32_t out = 0u;
    const int ticks = static_cast<int>(seconds * 500.0);
    for (int i = 0; i < ticks; ++i) {
        out = transient_fuel_xtau_with_autocalib(desired_us, rpm_x10, 35u, clt_x10, true, 2u);
    }
    return out;
}

void test_fuel_physics_xtau_steady_state(void) {
    section("fuel physics: X-tau wall film — steady state injects exactly the demand");
    struct { uint32_t rpm_x10; int16_t clt_x10; } pts[] = {
        {8000u, -100}, {8000u, 200}, {8000u, 900}, {30000u, 200}, {60000u, 900},
    };
    for (const auto& p : pts) {
        transient_fuel_reset();
        const uint32_t out = xtau_run(3000u, p.rpm_x10, p.clt_x10, 20.0);
        std::printf("    %5u rpm CLT %5.1f C: demand 3000 us -> injected %u us (%+.1f %%)\n",
                    p.rpm_x10 / 10u, p.clt_x10 / 10.0, out, (out - 3000.0) / 30.0);
        CHECK_TRUE(std::fabs(static_cast<double>(out) - 3000.0) <= 30.0,
                   "steady state within 1 % of demand (no film saturation)");
    }

    section("fuel physics: X-tau tip-in adds fuel, then converges");
    transient_fuel_reset();
    (void)xtau_run(3000u, 20000u, 200, 20.0);
    const uint32_t first = transient_fuel_xtau_with_autocalib(6000u, 20000u, 35u, 200, true, 2u);
    CHECK_TRUE(first > 6000u, "step up: first pulse above the new demand (film loading)");
    const uint32_t later = xtau_run(6000u, 20000u, 200, 20.0);
    CHECK_TRUE(std::fabs(static_cast<double>(later) - 6000.0) <= 60.0, "converges to the new demand");
}

void test_fuel_physics_delta_p(void) {
    section("fuel physics: injector dP compensation = sqrt(dP_rated / dP_actual)");
    fuel_set_baro_bar_x100(100u);
    // Rated 3.0 bar. Rail gauge + baro − MAP = actual ΔP.
    struct { uint16_t rail_gauge_x1000, map; } pts[] = {
        {3000u, 100u}, {3000u, 30u}, {3000u, 200u}, {3700u, 30u}, {4000u, 200u}, {2500u, 100u},
    };
    for (const auto& p : pts) {
        const double dp = p.rail_gauge_x1000 / 1000.0 + (100.0 - p.map) / 100.0;
        const double exp = 5000.0 * std::sqrt(3.0 / dp);
        const uint32_t got = apply_delta_p_compensation(5000u, p.rail_gauge_x1000, p.map);
        std::printf("    rail %.2f bar(g) MAP %3u kPa: dP %.2f bar, expected %.0f us, got %u us\n",
                    p.rail_gauge_x1000 / 1000.0, p.map, dp, exp, got);
        CHECK_TRUE(rel_err(got, exp) < 0.004, "within 0.4 % of sqrt(dP) physics");
    }
    // MAP-referenced regulator: rail tracks MAP → constant ΔP → no change.
    for (uint16_t map = 30u; map <= 250u; map = static_cast<uint16_t>(map + 55u)) {
        const uint16_t rail = static_cast<uint16_t>(3000 + (map - 100) * 10);
        CHECK_NEAR(apply_delta_p_compensation(5000u, rail, map), 5000, 5,
                   "MAP-referenced regulator: factor 1 at any MAP");
    }
    CHECK_EQ(apply_delta_p_compensation(5000u, 0u, 50u), 5000u, "no valid reading → no correction");
}

// Closed loop against a plant that needs 10 % more fuel than the base map.
// Total trim c = (STFT + LTFT)/1000; measured lambda = target × 1.10 / (1 + c).
// Correct adaptation: LTFT holds the whole +10 % and STFT returns to ~0, so
// the correction survives open loop (cold start, DFCO, sensor fault).
void test_fuel_physics_ltft_learns_whole_error(void) {
    section("fuel physics: LTFT learns the whole steady error (STFT -> 0)");
    fuel_reset_adaptives();
    const uint32_t rpm = 30000u;
    const uint16_t map = 50u;
    const int16_t target = 1000;
    uint32_t now = 0u;
    for (int i = 0; i < 36000; ++i) {   // 1 h of 100 ms closed-loop updates
        now += 100u;
        const double c = (fuel_get_stft_pct_x10() + fuel_get_ltft_at(rpm, map)) / 1000.0;
        const int16_t measured = static_cast<int16_t>(std::lround(target * 1.10 / (1.0 + c)));
        (void)fuel_update_stft(rpm, map, target, measured, 900, true, false, false, 5000u, 200u, now);
        if (std::getenv("LTFT_TRACE") != nullptr && i % 600 == 0) {
            std::printf("      t=%5.0f s LTFT %+.1f STFT %+.1f meas %d\n", now / 1000.0,
                        fuel_get_ltft_at(rpm, map) / 10.0, fuel_get_stft_pct_x10() / 10.0, measured);
        }
    }
    const int16_t ltft = fuel_get_ltft_at(rpm, map);
    const int16_t stft = fuel_get_stft_pct_x10();
    std::printf("    after 1 h: LTFT %+.1f %%, STFT %+.1f %% (need +10.0 %%)\n", ltft / 10.0, stft / 10.0);
    CHECK_TRUE(ltft >= 90 && ltft <= 110, "LTFT within 1 % of the +10 % the engine needs");
    CHECK_TRUE(stft >= -15 && stft <= 15, "STFT back near 0");
    fuel_reset_adaptives();
}
