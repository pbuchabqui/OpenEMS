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
#include <cmath>
#include <cstdio>
#include <cstring>

#include "engine/ecu_sched.h"
#include "hal/tim5_host.h"
#include "test/sim/engine_sim.h"

using namespace sim;

namespace {

constexpr int kStage = 3;  // review stage this tree is at

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
                expected_fail ? (xfail_stage == 1 ? "  [stage 1]" : xfail_stage == 2 ? "  [stage 2]" : "  [stage 3]") : "");
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

    test_purge_does_not_freeze_queue();

    std::printf("\nResults: %d PASS  %d FAIL  %d XFAIL  %d XPASS\n", g_pass, g_fail, g_xfail, g_xpass);
    return (g_fail == 0 && g_xpass == 0) ? 0 : 1;
}
