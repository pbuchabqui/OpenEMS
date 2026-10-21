/**
 * make precision-test — virtual-engine timing accuracy suite.
 *
 * Every number checked here is what the engine would physically receive
 * (spark angle, dwell time, injected fuel time, EOI angle), measured against
 * the kinematic oracle in test/sim/engine_sim.cpp — never against firmware
 * formulas.
 *
 * Known defects are tracked as XFAIL with the review stage that must fix them.
 * An XFAIL that starts passing (XPASS) fails the run so the table is kept
 * honest; bump kStage / the tag when a stage lands.
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>

#include "engine/calibration.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/ecu_sched.h"
#include "engine/engine_calc.h"
#include "engine/fuel_calc.h"
#include "engine/fuel_trim.h"
#include "engine/map_estimator.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/ign_calc.h"
#include "engine/limp_gating.h"
#include "engine/table3d.h"
#include "hal/host/tim5_host.h"
#include "test/sim/engine_sim.h"

using namespace sim;

namespace {

constexpr int kStage = 5;  // review stage this tree is at

int g_pass = 0, g_fail = 0, g_xfail = 0, g_xpass = 0;

// value <= limit passes. xfail_stage > kStage: known defect fixed in that stage.
void check(const char* scen, const char* what, double value, double limit, int xfail_stage = 0)
{
    const bool ok = value <= limit;
    const bool expected_fail = xfail_stage > kStage;
    const char* tag;
    if (ok && !expected_fail)      { tag = "PASS "; ++g_pass; }
    else if (ok && expected_fail)  { tag = "XPASS"; ++g_xpass; }
    else if (!ok && expected_fail) { tag = "XFAIL"; ++g_xfail; }
    else                           { tag = "FAIL "; ++g_fail; }
    std::printf("  %s %-24s %-26s %9.3f <= %-7.3f%s\n", tag, scen, what, value, limit,
                expected_fail ? (xfail_stage == 1 ? "  [stage 1]" : xfail_stage == 2 ? "  [stage 2]" : "  [stage 3+]") : "");
}

Metrics run_and_print(const char* name, const Config& c, double t_from, bool wasted, Result* out = nullptr)
{
    Result r = run(c);
    const Metrics m = analyze(r, t_from, wasted);
    std::printf("%-24s spk=%4d miss=%3d spur=%3d | err max %6.2f mean %6.2f rms %5.2f deg | dwell %5.1f%% short %d long %d wdog %u | "
                "inj=%4d fuel max %5.1f%% mean %5.1f%% | eoi %5.2f deg\n",
                name, m.sparks, m.missing, m.spurious, m.spark_err_max, m.spark_err_mean, m.spark_err_rms,
                m.dwell_err_max_pct, m.short_dwell, m.long_dwell, r.dwell_wdog,
                m.inj_pulses, m.fuel_err_max_pct, m.fuel_err_mean_pct, m.eoi_err_max);
    if (out != nullptr) { *out = r; }
    return m;
}

// Common check set for a fully synced (sequential) scenario.
struct Limits {
    double spark = 0.1, dwell_pct = 5.0, fuel_pct = 0.5, eoi = 0.5;
    int xf_spark = 0, xf_dwell = 0, xf_fuel = 0, xf_eoi = 0, xf_count = 0;
};

void check_seq(const char* n, const Metrics& m, const Limits& l)
{
    check(n, "spark error max (deg)", m.spark_err_max, l.spark, l.xf_spark);
    check(n, "missing+spurious sparks", m.missing + m.spurious, 0, l.xf_count);
    check(n, "dwell error max (%)", m.dwell_err_max_pct, l.dwell_pct, l.xf_dwell);
    check(n, "fuel error max (%)", m.fuel_err_max_pct, l.fuel_pct, l.xf_fuel);
    check(n, "EOI error max (deg)", m.eoi_err_max, l.eoi, l.xf_eoi);
}

Config base(double rpm)
{
    Config c;
    c.rpm = {{0.0, rpm}};
    c.duration_s = 1.5;
    c.trigger_offset_deg = 636;  // tooth 0 at 84° BTDC cyl 0 (README example)
    return c;
}

// Bug A: purge re-arms CCR3 with an already-due head and clears CC3IF →
// no compare match until the 32-bit counter wraps (~68.7 s).
void test_purge_does_not_freeze_queue()
{
    ecu_sched_test_reset();
    ems_test_tim5_cnt = 1000u;
    ecu_sched_test_pulse_inj(0U, 1000U);           // INJ1 ON now, OFF queued +1 ms
    uint32_t off_ts = 0;
    ecu_sched_test_get_evt(0U, &off_ts, nullptr, nullptr);
    ems_test_tim5_cnt = off_ts + 10u;              // OFF is now due…
    ems_test_tim5_sr |= (1u << 3);                 // …and the compare matched
    ecu_sched_set_ign_inhibit_mask(0x2U);          // main loop purges IGN cyl 1
    if ((ems_test_tim5_sr & (1u << 3)) != 0u) {     // TIM5_IRQHandler
        ems_test_tim5_sr &= ~(1u << 3);
        ecu_sched_evt_dispatch();
    }
    check("purge/rearm", "events stuck after purge", ecu_sched_test_get_evt_count(), 0, 1);
    ecu_sched_set_ign_inhibit_mask(0U);
    ecu_sched_test_reset();
}


// ── Pin-level helpers for the protection / supply scenarios ─────────────
bool is_inj_ch(uint8_t ch) { return ch <= 3u; }

// Opening pulses (high -> low) of injector channels with the opening at t >= t_from.
struct PulseStats { int n = 0; double mean_us = 0.0; };

PulseStats inj_pulses(const Result& r, double t_from, double t_to)
{
    PulseStats st;
    double t_on[8] = {};
    bool open[8] = {};
    double sum = 0.0;
    for (const PinEdge& e : r.edges) {
        if (!is_inj_ch(e.ch)) { continue; }
        if (e.high != 0u) { open[e.ch] = true; t_on[e.ch] = e.t_s; continue; }
        if (!open[e.ch]) { continue; }
        open[e.ch] = false;
        if (t_on[e.ch] >= t_from && t_on[e.ch] < t_to) {
            ++st.n;
            sum += (e.t_s - t_on[e.ch]) * 1e6;
        }
    }
    st.mean_us = (st.n > 0) ? sum / st.n : 0.0;
    return st;
}

// Mean coil dwell (ON -> spark) in microseconds for coils whose ON is at t >= t_from.
double mean_dwell_us(const Result& r, double t_from)
{
    double t_on[8] = {};
    bool open[8] = {};
    double sum = 0.0;
    int n = 0;
    for (const PinEdge& e : r.edges) {
        if (is_inj_ch(e.ch)) { continue; }
        if (e.high != 0u) { open[e.ch] = true; t_on[e.ch] = e.t_s; continue; }
        if (!open[e.ch]) { continue; }
        open[e.ch] = false;
        if (t_on[e.ch] >= t_from) { sum += (e.t_s - t_on[e.ch]) * 1e6; ++n; }
    }
    return (n > 0) ? sum / n : 0.0;
}

// Spark events (coil ON -> OFF) on any ignition channel with the ON edge in [t_from, t_to).
int coil_sparks(const Result& r, double t_from, double t_to)
{
    double t_on[8] = {};
    bool open[8] = {};
    int n = 0;
    for (const PinEdge& e : r.edges) {
        if (is_inj_ch(e.ch)) { continue; }
        if (e.high != 0u) { open[e.ch] = true; t_on[e.ch] = e.t_s; continue; }
        if (!open[e.ch]) { continue; }
        open[e.ch] = false;
        if (t_on[e.ch] >= t_from && t_on[e.ch] < t_to) { ++n; }
    }
    return n;
}

// Number of rising edges (any of the 8 outputs) at t >= t_from, plus 1 if any output
// is still HIGH at the end of the log. 0 = the engine receives nothing from t_from on.
int activity_after(const Result& r, double t_from)
{
    bool level[8] = {};
    int rising = 0;
    for (const PinEdge& e : r.edges) {
        level[e.ch] = (e.high != 0u);
        if (e.high != 0u && e.t_s >= t_from) { ++rising; }
    }
    for (bool l : level) { if (l) { ++rising; } }
    return rising;
}

// Warm engine, flat tables, 2 ms loop = engine_calc_step with fixed sensors; `mod` edits the
// sensors per scenario. The firmware commits what it computes; nothing is injected by the sim.
Result run_engine_calc(const char* name, Config c, uint16_t map_kpa,
                       std::function<void(ems::drv::SensorData&)> mod,
                       std::function<void(ems::engine::EngineCalcIn&)> in_mod = nullptr)
{
    namespace en = ems::engine;
    static uint8_t sv_ve[en::kTableAxisSize][en::kTableAxisSize];
    static int16_t sv_lam[en::kTableAxisSize][en::kTableAxisSize];
    static int8_t sv_spk[en::kTableAxisSize][en::kTableAxisSize];
    std::memcpy(sv_ve, en::ve_table, sizeof(sv_ve));
    std::memcpy(sv_lam, en::lambda_target_table_x1000, sizeof(sv_lam));
    std::memcpy(sv_spk, en::spark_table, sizeof(sv_spk));
    for (uint8_t y = 0u; y < en::kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < en::kTableAxisSize; ++x) {
            en::ve_table[y][x] = 70u;
            en::lambda_target_table_x1000[y][x] = 1000;
            en::spark_table[y][x] = 24;
        }
    }
    en::engine_calc_reset();
    en::limp_gating_test_reset();
    en::fuel_inj_duty_reset();
    en::quick_crank_reset();
    en::transient_fuel_reset();
    en::fuel_reset_adaptives();
    en::map_estimator_init();
    ems::drv::SensorData s{};
    s.map_bar_x1000 = static_cast<uint16_t>(map_kpa * 10u);
    s.clt_degc_x10 = 900;
    s.iat_degc_x10 = 200;
    s.vbatt_mv = 14000u;
    s.oil_press_bar_x1000 = 3000u;
    s.fuel_press_bar_x1000 = static_cast<uint16_t>(
        en::fuel_press_nominal_bar_x1000 - (en::fuel_get_baro_bar_x100() - map_kpa) * 10);
    s.app_pct_x10 = 250u;
    s.tps_pct_x10 = 250u;
    if (mod != nullptr) { mod(s); }
    c.on_main = [&](double t_s) {
        en::EngineCalcIn in{};
        in.now_ms = 1000u + static_cast<uint32_t>(t_s * 1000.0);
        in.snap = ems::drv::ckp_snapshot();
        in.sensors = s;
        if (in_mod) { in_mod(in); }
        en::engine_calc_step(in);
    };
    Result r = run(c);
    std::memcpy(en::ve_table, sv_ve, sizeof(sv_ve));
    std::memcpy(en::lambda_target_table_x1000, sv_lam, sizeof(sv_lam));
    std::memcpy(en::spark_table, sv_spk, sizeof(sv_spk));
    const PulseStats p = inj_pulses(r, 0.8, 1e9);
    std::printf("%-24s inj pulses after 0.8 s: %4d (mean %.0f us) | dwell mean %.0f us\n", name, p.n,
                p.mean_us, mean_dwell_us(r, 0.8));
    return r;
}

}  // namespace

int main()
{
    std::printf("== OpenEMS precision suite (stage %d) ==\n", kStage);
    std::printf("ISR model: entry 0.1 us, scheduler reads CNT after 2+-1 us, CKP ISR 4 us\n\n");

    // ── Steady state, sequential (cam) ──────────────────────────────────
    {
        Limits l;
        check_seq("800 rpm", run_and_print("800 rpm", base(800), 0.8, false), l);
        Config c = base(800);
        c.cmd.fuel_us = 1000.0;  // short idle pulse: 4.8 deg at 800 rpm
        Limits lq;
        lq.xf_fuel = 2;          // PW floored to whole degrees
        check_seq("800 rpm PW 1 ms", run_and_print("800 rpm PW 1 ms", c, 0.8, false), lq);
        check_seq("3000 rpm", run_and_print("3000 rpm", base(3000), 0.8, false), l);
        Limits l65;
        l65.xf_spark = 2;        // anchor on TIM5_CNT read in hook, not on capture
        l65.xf_fuel = 2;
        check_seq("6500 rpm", run_and_print("6500 rpm", base(6500), 0.8, false), l65);
    }

    // ── EOI far from TDC + long PW: the pulse crosses the 720° cycle origin
    {
        Config c = base(3000);
        c.cmd.eoi_deg = 300.0;
        c.cmd.fuel_us = 12000.0;   // 216° at 3000 rpm
        check_seq("EOI 300 long PW", run_and_print("EOI 300, PW 12 ms", c, 0.8, false), Limits{});
    }

    // ── PW above 90 % of the cycle → clamped, pulses never merge ────────
    {
        Config c = base(3000);     // cycle = 40 ms
        c.cmd.fuel_us = 39000.0;
        const Metrics m = run_and_print("PW 39 ms (> 90 % duty)", c, 0.8, false);
        check("duty clamp", "fuel delivered vs 90% cycle (%)",
              std::fabs((1.0 - m.fuel_err_max_pct / 100.0) * 39000.0 - 36000.0) / 360.0, 0.5);
        check("duty clamp", "missing+spurious sparks", m.missing + m.spurious, 0);
    }

    // ── Trigger gap region (348..360 trigger deg) ───────────────────────
    {
        Config c = base(800);
        c.trigger_offset_deg = 0;
        c.cmd.advance_deg = 2.0;    // spark at trigger 358 deg → inside the gap
        Limits l;
        l.xf_spark = 2; l.xf_dwell = 2; l.xf_fuel = 2;
        check_seq("gap region", run_and_print("gap region (off 0)", c, 0.8, false), l);
    }

    // ── Advance resolution / ATDC ───────────────────────────────────────
    {
        Config c = base(3000);
        c.cmd.advance_deg = 12.5;
        Limits l;
        l.xf_spark = 2;   // integer degrees
        check_seq("advance 12.5", run_and_print("advance 12.5 deg", c, 0.8, false), l);
        c.cmd.advance_deg = -4.0;
        check_seq("advance -4 (ATDC)", run_and_print("advance -4 deg ATDC", c, 0.8, false), l);
    }

    // ── Ignition chain end to end: table -> 0.1° advance -> spark ───────
    {
        static int8_t saved[ems::engine::kTableAxisSize][ems::engine::kTableAxisSize];
        std::memcpy(saved, ems::engine::spark_table, sizeof(saved));
        for (auto& row : ems::engine::spark_table) {
            for (uint8_t x = 0u; x < ems::engine::kTableAxisSize; ++x) {
                row[x] = static_cast<int8_t>(x < 10u ? 20 : 25);   // 20° up to 3000 rpm, then 25°
            }
        }
        const uint32_t mid = (ems::engine::kRpmAxisX10[9] + ems::engine::kRpmAxisX10[10]) / 2u;
        const int16_t adv_x10 = ems::engine::get_advance_x10(mid, 100u);
        check("table advance", "table midpoint 20..25 deg -> |adv - 22.5|",
              std::fabs(adv_x10 / 10.0 - 22.5), 0.0);
        Config c = base(mid / 10.0);
        c.cmd.advance_deg = adv_x10 / 10.0;
        check_seq("table 22.5 deg", run_and_print("advance from table (22.5 deg)", c, 0.8, false),
                  Limits{});
        std::memcpy(ems::engine::spark_table, saved, sizeof(saved));
    }

    // ── Trigger offset set in 0.1° (timing-light fine trim) ─────────────
    {
        Config c = base(3000);
        c.trigger_offset_deg = 100;
        c.wheel_extra_deg = 0.4;           // real wheel is 0.4° off the whole-degree value
        const Metrics off = analyze(run(c), 0.8, false);
        check("trigger fine", "error with whole-degree offset only (deg)",
              std::fabs(std::fabs(off.spark_err_mean) - 0.4), 0.05);
        ems::engine::trigger_fine_x10 = 4;  // timing light: +0.4°
        check_seq("trigger fine +0.4", run_and_print("trigger offset 100 + fine 0.4 deg", c, 0.8, false),
                  Limits{});
        ems::engine::trigger_fine_x10 = 0;
    }

    // ── Whole chain: sensors + tables -> engine_calc (2 ms loop) -> pins ─
    // The main-loop step is the firmware's engine_calc_step fed with fixed
    // sensors; the expected spark / dwell / fuel come from the tables and
    // physics (speed density, injector small-pulse curve, dead time).
    {
        namespace en = ems::engine;
        static uint8_t sv_ve[en::kTableAxisSize][en::kTableAxisSize];
        static int16_t sv_lam[en::kTableAxisSize][en::kTableAxisSize];
        static int8_t sv_spk[en::kTableAxisSize][en::kTableAxisSize];
        std::memcpy(sv_ve, en::ve_table, sizeof(sv_ve));
        std::memcpy(sv_lam, en::lambda_target_table_x1000, sizeof(sv_lam));
        std::memcpy(sv_spk, en::spark_table, sizeof(sv_spk));
        for (uint8_t y = 0u; y < en::kTableAxisSize; ++y) {
            for (uint8_t x = 0u; x < en::kTableAxisSize; ++x) {
                en::ve_table[y][x] = 70u;
                en::lambda_target_table_x1000[y][x] = 1000;
                en::spark_table[y][x] = 24;
            }
        }
        en::engine_calc_reset();
        en::quick_crank_reset();
        en::transient_fuel_reset();
        en::fuel_reset_adaptives();
        en::map_estimator_init();
        const uint16_t map_kpa = 60u;
        ems::drv::SensorData s{};
        s.map_bar_x1000 = map_kpa * 10u;
        s.clt_degc_x10 = 900;
        s.iat_degc_x10 = 200;   // 20 C: IAT spark correction 0
        s.vbatt_mv = 14000u;
        s.oil_press_bar_x1000 = 3000u;
        s.fuel_press_bar_x1000 = static_cast<uint16_t>(
            en::fuel_press_nominal_bar_x1000 - (en::fuel_get_baro_bar_x100() - map_kpa) * 10);
        s.app_pct_x10 = 250u;
        s.tps_pct_x10 = 250u;

        const double rpm = 3000.0;
        // Expected: sequential (cam), one opening per cycle.
        auto eff = [](double t) {
            const uint16_t* ax = en::injector_scurve_pw_axis_us;
            const uint16_t* q = en::injector_scurve_corr_q8;
            if (t <= ax[0]) { return q[0] / 256.0; }
            for (int i = 1; i < 8; ++i) {
                if (t <= ax[i]) {
                    return (q[i - 1] + (t - ax[i - 1]) / double(ax[i] - ax[i - 1]) * (q[i] - q[i - 1])) / 256.0;
                }
            }
            return q[7] / 256.0;
        };
        double flow = en::default_req_fuel_us() * 0.70 * (map_kpa / 101.325) * (298.0 / 293.0);
        flow *= en::corr_clt(900) / 256.0;
        flow *= en::corr_iat(200) / 256.0;
        double open = flow;
        for (int i = 0; i < 50; ++i) { open = flow / eff(open); }

        Config c = base(rpm);
        c.cmd.advance_deg = 24.0;
        c.cmd.dwell_ms = en::dwell_ms_x10_from_vbatt_rpm(14000u, static_cast<uint32_t>(rpm * 10)) / 10.0;
        c.cmd.fuel_us = open;
        c.cmd.dead_us = en::corr_vbatt(14000u);
        c.cmd.eoi_deg = en::calc_eoi_lead_deg(static_cast<uint32_t>(rpm * 10), 900);
        int16_t last_spark = 0;
        c.on_main = [&](double t_s) {
            en::EngineCalcIn in{};
            in.now_ms = 1000u + static_cast<uint32_t>(t_s * 1000.0);
            in.snap = ems::drv::ckp_snapshot();
            in.sensors = s;
            last_spark = en::engine_calc_step(in).spark_x10;
        };
        c.duration_s = 4.0;   // X-tau wall film loads after start: measure at steady state
        const Metrics m = run_and_print("2 ms loop: tables -> pins", c, 3.0, false);
        std::printf("    engine_calc advance %.1f deg, expected 24.0\n", last_spark / 10.0);
        check_seq("engine_calc chain", m, Limits{});
        check("engine_calc chain", "advance from table (deg)", std::fabs(last_spark / 10.0 - 24.0), 0.05);
        std::memcpy(en::ve_table, sv_ve, sizeof(sv_ve));
        std::memcpy(en::lambda_target_table_x1000, sv_lam, sizeof(sv_lam));
        std::memcpy(en::spark_table, sv_spk, sizeof(sv_spk));
    }

    // ── Knock retard is per cylinder: only the knocking coil moves ──────
    {
        Config c = base(3000);
        c.cmd.advance_deg = 20.0;
        c.cmd.cyl_retard_deg[2] = 3.7;   // cylinder 2 knocks
        const Metrics m = run_and_print("knock retard 3.7 deg on cyl 2 only", c, 0.8, false);
        check_seq("knock per cylinder", m, Limits{});
    }

    // ── Per-cylinder advance stays inside -20..+60 deg after trim and knock ──
    {
        namespace en = ems::engine;
        // Trim +15 on cyl 0 on top of +60: the oracle (no trim) expects 60 deg, so a clamped
        // scheduler matches it; unclamped it would fire 15 deg early.
        const int8_t sv_trim = en::cyl_ign_trim_deg[0];
        en::cyl_ign_trim_deg[0] = 15;
        Config c = base(3000);
        c.cmd.advance_deg = 60.0;
        const Metrics m = run_and_print("advance 60 + trim 15 (clamp)", c, 0.8, false);
        en::cyl_ign_trim_deg[0] = sv_trim;
        check("advance clamp +60", "spark error max (deg)", m.spark_err_max, 0.1);

        // Knock 30 deg on top of +5: the oracle expects -25 deg, the scheduler stops at -20.
        Config ck = base(3000);
        ck.cmd.advance_deg = 5.0;
        ck.cmd.cyl_retard_deg[0] = 30.0;
        const Metrics mk = run_and_print("advance 5 + knock 30 (clamp)", ck, 0.8, false);
        check("advance clamp -20", "spark 5 deg earlier than unclamped: |err-5|", std::fabs(mk.spark_err_max - 5.0), 0.2);
    }

    // ── Transients ───────────────────────────────────────────────────────
    {
        Config c = base(1500);
        c.rpm = {{0.0, 1500.0}, {0.5, 1500.0}, {2.0, 5500.0}};  // +2667 rpm/s
        c.duration_s = 2.0;
        Limits l;
        l.spark = 0.5;
        // EOI is a soft target: SOI = EOI − PW angle estimated at the last
        // table build; the injected quantity itself is exact (PW in time).
        l.eoi = 2.0;
        check_seq("accel", run_and_print("accel +2667 rpm/s", c, 0.6, false), l);
        c.rpm = {{0.0, 5500.0}, {0.5, 5500.0}, {2.0, 1500.0}};
        check_seq("decel", run_and_print("decel -2667 rpm/s", c, 0.6, false), l);
    }

    // ── Cranking with compression ripple ────────────────────────────────
    {
        Config c = base(250);
        c.ripple = 0.3;
        c.duration_s = 4.0;
        Result r;
        const Metrics m = run_and_print("crank 250 rpm +-30%", c, 1.5, false, &r);
        check("crank", "spark error max (deg)", m.spark_err_max, 1.0);
        check("crank", "missing+spurious sparks", m.missing + m.spurious, 0);
        check("crank", "dwell error max (%)", m.dwell_err_max_pct, 15.0, 2);
        check("crank", "dwell watchdog trips", r.dwell_wdog, 0, 2);
        check("crank", "fuel error max (%)", m.fuel_err_max_pct, 1.0, 2);
    }

    // ── Edge jitter / wheel machining error ─────────────────────────────
    {
        Config c = base(3000);
        c.edge_jitter_us = 1.0;
        Limits l;
        l.spark = 0.2;
        l.xf_fuel = 2;      // PW in whole degrees from one jittered tooth
        check_seq("jitter 1us", run_and_print("edge jitter 1 us", c, 0.8, false), l);
        c.edge_jitter_us = 0.0;
        c.tooth_error_deg = 0.2;
        Limits lm;
        lm.spark = 0.3;     // 0.2 deg wheel error itself + prediction

        check_seq("tooth err 0.2", run_and_print("tooth error +-0.2 deg", c, 0.8, false), lm);
    }

    // ── No cam: wasted spark / batch fuel ───────────────────────────────
    {
        Config c = base(3000);
        c.cam = false;
        Limits l;
        l.xf_count = 1;   // all four coils fired at the cyl 0/3 angle (fixed stage 1)
        l.xf_fuel = 2;    // PW in whole degrees (was -51 %: opening count, fixed stage 1)
        l.eoi = 1e9;      // not meaningful in batch mode
        check_seq("no cam", run_and_print("no cam (wasted)", c, 0.8, true), l);
        c.presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
        check_seq("no cam simult", run_and_print("no cam simultaneous", c, 0.8, true), l);
    }

    // ── Start-up: presync → sequential transition (whole run) ───────────
    {
        // Tooth 0 (where presync→sequential switches) inside cyl 0's dwell
        // (695.6..710 deg) while the 12 deg gap before it holds no event.
        Config c = base(800);
        c.trigger_offset_deg = 709;
        c.duration_s = 1.0;
        const Metrics m = run_and_print("startup transition", c, 0.0, false);
        check("startup", "sparks with dwell <50%/>150%", m.short_dwell + m.long_dwell, 0);
    }

    // ── Noise pulse mid-tooth (lost tooth) ──────────────────────────────
    {
        Config c = base(3000);
        c.duration_s = 1.5;
        c.noise_s = {0.9001, 1.1003};
        Result r;
        const Metrics m = run_and_print("noise mid-tooth", c, 0.8, false, &r);
        check("noise", "spark error max (deg)", m.spark_err_max, 0.2, 3);
        check("noise", "sparks with dwell < 50%", m.short_dwell, 0);
    }

    // ── Noise pulse at every fraction of a tooth period ─────────────────
    {
        // 3000 rpm: θ(t) = 18000·t deg; tooth n edge at engine 636 + 6n deg.
        // One pulse per revolution at fraction f = 0.1 … 0.9 of the period.
        Config c = base(3000);
        c.duration_s = 1.6;
        for (int k = 1; k <= 9; ++k) {
            // Revolution 42+2k (t ≈ 0.88…1.36 s, inside the measured window), tooth 7k.
            const double tooth_deg = 636.0 + 360.0 * (42 + 2 * k) + 6.0 * (7 * k);
            c.noise_s.push_back((tooth_deg + 6.0 * 0.1 * k) / 18000.0);
        }
        Result r;
        const Metrics m = run_and_print("noise f=0.1..0.9", c, 0.8, false, &r);
        // f < 0.5: edge ignored. f >= 0.5: the noise stands in for the real
        // edge of that one tooth (indistinguishable at that instant): events
        // armed from it are early by at most (1 - f) × 6 deg = 3 deg.
        check("noise sweep", "spark error max (deg)", m.spark_err_max, 3.0);
        check("noise sweep", "missing+spurious sparks", m.missing + m.spurious, 0);
        check("noise sweep", "sparks with dwell <50%/>120%", m.short_dwell + m.long_dwell, 0);
    }

    // ── Noise above the 0.9 "substitute" threshold ──────────────────────
    {
        // f = 0.95 / 0.99: the pulse is taken as the tooth itself and the real edge that
        // follows is dropped, so events armed from it are early by (1 - f) × 6 deg <= 0.3 deg.
        Config c = base(3000);
        c.duration_s = 1.6;
        const double fr[2] = {0.95, 0.99};
        for (int k = 1; k <= 2; ++k) {
            const double tooth_deg = 636.0 + 360.0 * (42 + 4 * k) + 6.0 * (7 * k);
            c.noise_s.push_back((tooth_deg + 6.0 * fr[k - 1]) / 18000.0);
        }
        const Metrics m = run_and_print("noise f=0.95,0.99", c, 0.8, false);
        check("noise f>=0.9", "spark error max (deg)", m.spark_err_max, 0.6);
        check("noise f>=0.9", "missing+spurious sparks", m.missing + m.spurious, 0);
        check("noise f>=0.9", "sparks with dwell <50%/>120%", m.short_dwell + m.long_dwell, 0);
    }

    // ── One tooth edge lost (sensor glitch) → explicit sync loss ────────
    {
        Config c = base(3000);
        c.duration_s = 1.6;
        // Lose only tooth 30 of revolution 45 (t ≈ 0.95 s, inside the window).
        const double th = 636.0 + 360.0 * 45.0 + 6.0 * 30.0;
        c.dropout_s = {{(th - 1.0) / 18000.0, (th + 1.0) / 18000.0}};
        const Metrics m = run_and_print("one tooth lost", c, 0.8, false);
        // Never fire off-angle on the miscounted teeth; sparks lost while
        // re-syncing are acceptable (≤ 2 revolutions ≈ 8 sparks).
        check("tooth lost", "spark error max (deg)", m.spark_err_max, 0.2);
        check("tooth lost", "missing sparks while re-syncing", m.missing, 8);
    }

    // ── Hard cranking: ±45 % compression ripple at 200 rpm ──────────────
    {
        Config c = base(200);
        c.ripple = 0.45;
        c.duration_s = 5.0;
        Result r;
        const Metrics m = run_and_print("crank 200 rpm +-45%", c, 2.0, false, &r);
        check("crank 45%", "spark error max (deg)", m.spark_err_max, 1.0);
        check("crank 45%", "missing+spurious sparks", m.missing + m.spurious, 0);
        check("crank 45%", "dwell watchdog trips", r.dwell_wdog, 0);
    }

    // ── CKP dropout during a dwell → sync loss (outputs must end on time) ─
    {
        // 3000 rpm = 18 deg/ms, θ(t) = 18000·t. Cyl 0 sparks at 710 deg
        // (dwell from 656); its spark tooth is 12 (engine 708 with offset
        // 636). Dropping engine 700..709.5 loses teeth 11-12: the coil is ON
        // when sync is lost and no tooth will ever arm its SPARK.
        Config c = base(3000);
        c.duration_s = 1.6;
        for (int n : {20, 30}) {
            c.dropout_s.push_back({(700.0 + 720.0 * n) / 18000.0, (709.5 + 720.0 * n) / 18000.0});
        }
        Result r;
        const Metrics m = run_and_print("CKP dropout in dwell", c, 0.0, false, &r);
        check("dropout", "dwell watchdog trips", r.dwell_wdog, 0, 1);
        check("dropout", "coil held >120% dwell", m.long_dwell, 0, 1);
        check("dropout", "injector watchdog trips", r.inj_wdog, 0, 1);
    }

    // ── TIM5 32-bit wrap during run ─────────────────────────────────────
    {
        Config c = base(3000);
        c.t0_ticks = 0xFFFFFFFFu - static_cast<uint32_t>(0.9 * kTickHz);
        Limits l;
        check_seq("tim5 wrap", run_and_print("TIM5 wrap at 0.9 s", c, 0.8, false), l);
    }

    // ── Protections and supply seen at the injector / coil pins ─────────
    // What the engine physically receives when the firmware cuts or compensates.
    {
        namespace en = ems::engine;
        constexpr uint16_t kMapKpa = 60u;
        auto healthy = [](ems::drv::SensorData&) {};
        auto map_fault = [](ems::drv::SensorData& s) { s.fault_bits |= 0x01u; };   // SensorId::MAP
        auto overtemp = [](ems::drv::SensorData& s) { s.clt_degc_x10 = 1200; };    // >= 115 C
        auto batt_9v = [](ems::drv::SensorData& s) { s.vbatt_mv = 9000u; };

        Config c = base(4000);
        c.duration_s = 3.0;
        const Result ok = run_engine_calc("4000 rpm healthy", c, kMapKpa, healthy);
        check("healthy 4000 rpm", "injector pulses missing (0=ok)", inj_pulses(ok, 0.8, 1e9).n > 0 ? 0.0 : 1.0, 0);

        const Result mf = run_engine_calc("4000 rpm MAP fault", c, kMapKpa, map_fault);
        check("MAP fault >limp rpm", "injector pulses at the pins", inj_pulses(mf, 0.8, 1e9).n, 0);
        check("MAP fault >limp rpm", "sparks at the pins", coil_sparks(mf, 0.8, 1e9), 0);

        const Result ot = run_engine_calc("4000 rpm CLT 120 C", c, kMapKpa, overtemp);
        check("critical overtemp", "injector pulses at the pins", inj_pulses(ot, 0.8, 1e9).n, 0);
        check("critical overtemp", "sparks at the pins", coil_sparks(ot, 0.8, 1e9), 0);

        // Rev limit: over the limit the injectors stay shut; back below it they resume.
        Config cr = base(3000);
        const double over = en::rev_limit_rpm_x10 / 10.0 + 1000.0;
        cr.rpm = {{0.0, over}, {1.5, over}, {1.7, 3000.0}};
        cr.duration_s = 4.0;
        const Result rl = run_engine_calc("rev limit then 3000 rpm", cr, kMapKpa, healthy);
        check("rev limit", "injector pulses while over", inj_pulses(rl, 0.8, 1.5).n, 0);
        check("rev limit", "sparks kept while over (0=ok)", coil_sparks(rl, 0.8, 1.5) > 0 ? 0.0 : 1.0, 0);
        check("rev limit", "injector pulses missing after release (0=ok)",
              inj_pulses(rl, 3.0, 1e9).n > 0 ? 0.0 : 1.0, 0);

        // Battery: low voltage lengthens both the coil charge and the injector dead time.
        Config cb = base(3000);
        cb.duration_s = 3.0;
        const Result b14 = run_engine_calc("3000 rpm 14 V", cb, kMapKpa, healthy);
        const Result b9 = run_engine_calc("3000 rpm 9 V", cb, kMapKpa, batt_9v);
        check("battery 9 V vs 14 V", "dwell not longer at 9 V (0=ok)",
              mean_dwell_us(b9, 1.0) > mean_dwell_us(b14, 1.0) ? 0.0 : 1.0, 0);
        check("battery 9 V vs 14 V", "injector pulse not longer at 9 V (0=ok)",
              inj_pulses(b9, 1.0, 1e9).mean_us > inj_pulses(b14, 1.0, 1e9).mean_us ? 0.0 : 1.0, 0);
    }

    // ── Every cut seen at the injector AND coil pins ────────────────────
    // limp_gating has no unit test: here each cut is judged only by what leaves the pins.
    // 2500 rpm keeps the sensor-limp rpm cut (3000) out of the way, so each case isolates one cut.
    {
        namespace en = ems::engine;
        constexpr uint16_t kMapKpa = 60u;
        constexpr double kFrom = 1.5;   // window start: cut timers (<= 500 ms) have run
        auto none = [](ems::drv::SensorData&) {};
        auto inj_n = [&](const Result& r) { return inj_pulses(r, kFrom, 1e9).n; };
        auto sp_n = [&](const Result& r) { return coil_sparks(r, kFrom, 1e9); };
        auto flag = [](bool bad) { return bad ? 1.0 : 0.0; };

        Config c = base(2500);
        c.duration_s = 3.5;
        const Result hl = run_engine_calc("cuts: healthy 2500", c, kMapKpa, none);
        const int base_inj = inj_n(hl);
        const int base_sp = sp_n(hl);
        check("cuts healthy", "injector pulses present (0=ok)", flag(base_inj < 50), 0);
        check("cuts healthy", "sparks present (0=ok)", flag(base_sp < 50), 0);
        auto fuel_only = [&](const char* name, const Result& r) {
            check(name, "injector pulses at the pins", inj_n(r), 0);
            check(name, "sparks kept (>= 90% of healthy, 0=ok)", flag(sp_n(r) < base_sp * 9 / 10), 0);
        };
        auto fuel_and_spark = [&](const char* name, const Result& r) {
            check(name, "injector pulses at the pins", inj_n(r), 0);
            check(name, "sparks at the pins", sp_n(r), 0);
        };

        fuel_and_spark("oil sensor fault", run_engine_calc("cuts: oil sensor fault", c, kMapKpa,
            [](ems::drv::SensorData& s) { s.fault_bits |= 0x80u; }));         // SensorId::OIL_PRESS
        fuel_only("fuel rail fault", run_engine_calc("cuts: fuel rail fault", c, kMapKpa,
            [](ems::drv::SensorData& s) { s.fault_bits |= 0x40u; }));         // SensorId::FUEL_PRESS
        fuel_and_spark("critical overtemp 2500", run_engine_calc("cuts: CLT 120 C", c, kMapKpa,
            [](ems::drv::SensorData& s) { s.clt_degc_x10 = 1200; }));

        const uint16_t sv_boost = en::boost_cut_map_bar_x100;
        en::boost_cut_map_bar_x100 = 40u;                                      // MAP 60 kPa > 0.40 bar
        fuel_only("boost cut", run_engine_calc("cuts: boost", c, kMapKpa, none));
        en::boost_cut_map_bar_x100 = sv_boost;

        const uint16_t sv_oil = en::oil_min_after_start_bar_x1000;
        en::oil_min_after_start_bar_x1000 = 1500u;
        fuel_only("oil pressure low", run_engine_calc("cuts: oil pressure low", c, kMapKpa,
            [](ems::drv::SensorData& s) { s.oil_press_bar_x1000 = 500u; }));
        en::oil_min_after_start_bar_x1000 = sv_oil;

        const uint16_t sv_lt = en::lambda_protect_timeout_ms, sv_ld = en::lambda_protect_dev_x1000,
                       sv_ll = en::lambda_protect_min_load_bar_x100;
        const uint32_t sv_lr = en::lambda_protect_min_rpm_x10;
        en::lambda_protect_timeout_ms = 200u;
        en::lambda_protect_dev_x1000 = 100u;
        en::lambda_protect_min_rpm_x10 = 10000u;
        en::lambda_protect_min_load_bar_x100 = 20u;
        fuel_only("lambda lean protect", run_engine_calc("cuts: lambda lean", c, kMapKpa, none,
            [](en::EngineCalcIn& in) { in.lambda_x1000 = 1600u; in.lambda_valid = true; }));
        en::lambda_protect_timeout_ms = sv_lt;
        en::lambda_protect_dev_x1000 = sv_ld;
        en::lambda_protect_min_rpm_x10 = sv_lr;
        en::lambda_protect_min_load_bar_x100 = sv_ll;

        const uint8_t sv_harness = en::etb_harness_present;
        en::etb_harness_present = 1u;
        fuel_only("ETB fault > 1500 rpm", run_engine_calc("cuts: ETB fault", c, kMapKpa,
            [](ems::drv::SensorData& s) { s.throttle_fault_bits |= ems::drv::THROTTLE_FAULT_ETB_TPS1; }));
        en::etb_harness_present = sv_harness;
        // Harness absent (project default): the same TPS fault must not cut anything.
        {
            const Result nh = run_engine_calc("cuts: ETB fault, no harness", c, kMapKpa,
                [](ems::drv::SensorData& s) { s.throttle_fault_bits |= ems::drv::THROTTLE_FAULT_ETB_TPS1; });
            check("ETB fault, harness 0", "injector pulses kept (0=ok)", flag(inj_n(nh) < base_inj * 9 / 10), 0);
        }

        // Flood clear: cranking with the pedal floored → no fuel, crank spark continues.
        {
            Config cf = base(250);
            cf.duration_s = 3.0;
            const Result fl = run_engine_calc("cuts: flood clear", cf, kMapKpa,
                [](ems::drv::SensorData& s) { s.app_pct_x10 = 900u; });
            check("flood clear", "injector pulses at the pins", inj_pulses(fl, 1.0, 1e9).n, 0);
            check("flood clear", "sparks kept (0=ok)", flag(coil_sparks(fl, 1.0, 1e9) < 10), 0);
        }

        // Soft limiter: spark skip near the hard limit removes about half of the sparks.
        {
            const uint16_t sv_w = en::spark_skip_window_rpm_x10;
            const uint8_t sv_m = en::spark_skip_max_q8;
            Config cs = base(en::rev_limit_rpm_x10 / 10 - 5);
            cs.duration_s = 3.0;
            const Result ref = run_engine_calc("cuts: soft limit off", cs, kMapKpa, none);
            en::spark_skip_window_rpm_x10 = 500u;
            en::spark_skip_max_q8 = 128u;
            const Result sk = run_engine_calc("cuts: soft limit skip", cs, kMapKpa, none);
            en::spark_skip_window_rpm_x10 = sv_w;
            en::spark_skip_max_q8 = sv_m;
            const double ratio = static_cast<double>(coil_sparks(sk, 1.0, 1e9)) /
                                 std::max(1, coil_sparks(ref, 1.0, 1e9));
            // Ratio at 6995 rpm: 128*450/500 = 115/256 -> 4*115/256 = 1.8 of 4 sparks skipped -> 55 % kept.
            check("spark skip", "sparks kept vs window off: |ratio-0.55|", std::fabs(ratio - 0.55), 0.03);
            check("spark skip", "injector pulses kept (0=ok)",
                  flag(inj_pulses(sk, 1.0, 1e9).n < inj_pulses(ref, 1.0, 1e9).n * 9 / 10), 0);
        }

        // diag_critical alone: the overtemp cut is disabled, so only the diagnostic manager is left.
        fuel_and_spark("diag critical only", run_engine_calc("cuts: diag critical only", c, kMapKpa,
            [](ems::drv::SensorData& s) { s.clt_degc_x10 = 1200; }, 
            [](en::EngineCalcIn&) { en::limp_gating_set_protect_disable(en::kProtectDisOvertemp); }));

        // Control for the line above: with overtemp AND diag both disabled nothing is cut, so the
        // cut really came from the diagnostic manager and not from the overtemp rule.
        {
            const Result ctl = run_engine_calc("cuts: overtemp+diag disabled", c, kMapKpa,
                [](ems::drv::SensorData& s) { s.clt_degc_x10 = 1200; },
                [](en::EngineCalcIn&) {
                    en::limp_gating_set_protect_disable(en::kProtectDisOvertemp | en::kProtectDisDiag);
                });
            check("diag control", "injector pulses kept (0=ok)", flag(inj_n(ctl) < base_inj * 9 / 10), 0);
        }

        // Engine_calc sees HALF_SYNC while running (Hall wheel: the cycle phase is unknown):
        // running fuel is locked out, the spark keeps going on the table advance.
        {
            const Result hs = run_engine_calc("cuts: HALF_SYNC running", c, kMapKpa, none,
                [](en::EngineCalcIn& in) { in.snap.state = ems::drv::SyncState::HALF_SYNC; });
            check("HALF_SYNC running", "injector pulses at the pins", inj_n(hs), 0);
            check("HALF_SYNC running", "sparks kept (0=ok)", flag(sp_n(hs) < base_sp * 9 / 10), 0);
        }

        // Injector duty protection: > 15 % duty for 10 ms cuts fuel; it only resumes below 20 %.
        // Sequential duty = one opening (~4.1 ms) per 720 deg cycle: 22 % at 6500 rpm.
        {
            const uint8_t sv_d = en::inj_duty_max_pct, sv_t = en::inj_duty_tol_ms10;
            Config cd2 = base(6500);
            cd2.duration_s = 3.5;
            const Result d0 = run_engine_calc("cuts: 6500 rpm duty off", cd2, kMapKpa, none);
            en::inj_duty_max_pct = 15u;
            en::inj_duty_tol_ms10 = 1u;
            const Result d1 = run_engine_calc("cuts: injector duty", cd2, kMapKpa, none);
            en::inj_duty_max_pct = sv_d;
            en::inj_duty_tol_ms10 = sv_t;
            check("injector duty cut", "reference run injects (0=ok)", flag(inj_pulses(d0, kFrom, 1e9).n < 50), 0);
            check("injector duty cut", "injector pulses at the pins", inj_pulses(d1, kFrom, 1e9).n, 0);
            check("injector duty cut", "sparks kept (>= 90% of duty off, 0=ok)",
                  flag(coil_sparks(d1, kFrom, 1e9) < coil_sparks(d0, kFrom, 1e9) * 9 / 10), 0);
        }

        // CKP lost for good while running: nothing may fire and no coil may stay charged.
        {
            Config cd = base(2500);
            cd.duration_s = 3.5;
            cd.dropout_s.push_back({1.5, 10.0});
            const Result lost = run_engine_calc("cuts: CKP lost", cd, kMapKpa, none);
            check("CKP lost", "pin activity after 0.3 s (0=all LOW)", activity_after(lost, 1.8), 0);
        }
    }

    test_purge_does_not_freeze_queue();

    std::printf("\nResults: %d PASS  %d FAIL  %d XFAIL  %d XPASS\n", g_pass, g_fail, g_xfail, g_xpass);
    return (g_fail == 0 && g_xpass == 0) ? 0 : 1;
}
