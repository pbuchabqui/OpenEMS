#include "test/sim/engine_sim.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <cstdio>
#include <cstdlib>

#include "drv/ckp.h"
#include "engine/ecu_sched.h"
#include "engine/engine_config.h"
#include "hal/out_pins.h"
#include "hal/tim5_host.h"

namespace sim {
namespace {

constexpr double kDThetaDeg = 0.1;  // trajectory resolution (linear interp between)
constexpr uint32_t kCc3if = 1u << 3;
constexpr uint32_t kCc3ie = 1u << 3;
constexpr uint64_t kInf = std::numeric_limits<uint64_t>::max();

// ── Simulated TIM5 time base (64-bit; TIM5_CNT = low 32 bits) ─────────────
uint64_t g_now = 0;
std::vector<PinEdge>* g_log = nullptr;
uint64_t g_t0 = 0;

void advance(uint64_t to)
{
    if (to <= g_now) { return; }
    const uint32_t ccr3 = ems_test_tim5_ccr3;
    const int32_t d_old = static_cast<int32_t>(ccr3 - static_cast<uint32_t>(g_now));
    const int32_t d_new = static_cast<int32_t>(ccr3 - static_cast<uint32_t>(to));
    if (d_old > 0 && d_new <= 0) { ems_test_tim5_sr |= kCc3if; }  // compare match
    g_now = to;
    ems_test_tim5_cnt = static_cast<uint32_t>(to);
}

uint8_t g_level[8];

void pin_hook(uint8_t ch, uint8_t high)
{
    // Log real level changes only (a BSRR write to the current level is no edge).
    if (ch >= 8U || g_level[ch] == high) { return; }
    g_level[ch] = high;
    if (g_log != nullptr) {
        g_log->push_back(PinEdge{ch, high,
            static_cast<double>(g_now - g_t0) / kTickHz, 0.0});
    }
}

uint64_t us_ticks(double us) { return static_cast<uint64_t>(std::llround(us * kTickHz * 1e-6)); }

double rpm_at(const Config& c, double t)
{
    const auto& p = c.rpm;
    if (t <= p.front().first) { return p.front().second; }
    for (size_t i = 1; i < p.size(); ++i) {
        if (t <= p[i].first) {
            const double f = (t - p[i - 1].first) / (p[i].first - p[i - 1].first);
            return p[i - 1].second + f * (p[i].second - p[i - 1].second);
        }
    }
    return p.back().second;
}

// Single adapter between engine-level Command and the firmware API.
// Mirrors the main-loop contract: PW per opening = (fuel + dead·n)/n with
// n = 1 squirt/cycle sequential, 2 otherwise (fuel_calc inj_pulse_pw_us).
void commit(const Command& c)
{
    const uint32_t squirts = (ecu_sched_is_sequential() != 0U) ? 1U : 2U;
    const double pw_us = (c.fuel_us + c.dead_us * squirts) / squirts;
    ecu_sched_commit_calibration_x10(
        static_cast<int32_t>(std::lround(c.advance_deg * 10.0)),
        static_cast<uint32_t>(std::llround(c.dwell_ms * 62500.0)),
        static_cast<uint32_t>(std::llround(pw_us * 62.5)),
        static_cast<uint32_t>(std::lround(c.eoi_deg)));
}

}  // namespace

int channel_cyl(uint8_t ch, bool* is_ign)
{
    switch (ch) {
    case ECU_CH_INJ1: *is_ign = false; return 0;
    case ECU_CH_INJ2: *is_ign = false; return 1;
    case ECU_CH_INJ3: *is_ign = false; return 2;
    case ECU_CH_INJ4: *is_ign = false; return 3;
    case ECU_CH_IGN1: *is_ign = true; return 0;
    case ECU_CH_IGN2: *is_ign = true; return 1;
    case ECU_CH_IGN3: *is_ign = true; return 2;
    case ECU_CH_IGN4: *is_ign = true; return 3;
    default: *is_ign = false; return -1;
    }
}

Command Result::cmd_at(double t_s) const { return cmd_fn(t_s); }

double Result::theta_at(double ts) const
{
    if (ts <= t.front()) { return theta.front(); }
    if (ts >= t.back()) { return theta.back(); }
    const size_t i = static_cast<size_t>(std::upper_bound(t.begin(), t.end(), ts) - t.begin());
    const double f = (ts - t[i - 1]) / (t[i] - t[i - 1]);
    return theta[i - 1] + f * (theta[i] - theta[i - 1]);
}

double Result::time_at(double th) const
{
    if (th <= theta.front()) { return t.front(); }
    if (th >= theta.back()) { return t.back(); }
    const double fi = (th - theta.front()) / kDThetaDeg;
    const size_t i = static_cast<size_t>(fi);
    const double f = fi - static_cast<double>(i);
    return t[i] + f * (t[i + 1] - t[i]);
}

Result run(const Config& cfg)
{
    Result r;
    r.cmd_fn = cfg.cmd_at ? cfg.cmd_at : [c = cfg.cmd](double) { return c; };

    // ── 1. Kinematics: integrate dt = dθ/ω(θ,t) ─────────────────────────
    {
        double tt = 0.0, th = 0.0;
        r.t.push_back(tt);
        r.theta.push_back(th);
        while (tt < cfg.duration_s + 0.05) {
            const double thm = th + 0.5 * kDThetaDeg;
            const double ripple = 1.0 - cfg.ripple * std::cos(2.0 * M_PI * thm / 180.0);
            const double w = rpm_at(cfg, tt) * 6.0 * ripple;  // deg/s
            tt += kDThetaDeg / w;
            th += kDThetaDeg;
            r.t.push_back(tt);
            r.theta.push_back(th);
        }
    }

    // ── 2. Edge generation (TIM5 ticks, 64-bit) ─────────────────────────
    std::mt19937_64 rng(cfg.seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    double tooth_err[60];
    for (double& e : tooth_err) { e = cfg.tooth_error_deg * uni(rng); }

    g_t0 = cfg.t0_ticks;
    auto to_tick = [&](double ts) { return g_t0 + static_cast<uint64_t>(std::llround(ts * kTickHz)); };
    const double off = cfg.trigger_offset_deg;
    const double th_max = r.theta.back();

    std::vector<uint64_t> ckp, cam;
    for (long n = static_cast<long>(std::ceil(-off / 6.0)); ; ++n) {
        const long k = ((n % 60) + 60) % 60;
        if (k >= 58) { continue; }
        const double th = off + 6.0 * static_cast<double>(n) + tooth_err[k];
        if (th < 0.0) { continue; }
        if (th > th_max) { break; }
        double ts = r.time_at(th);
        if (cfg.edge_jitter_us > 0.0) { ts += cfg.edge_jitter_us * 1e-6 * gauss(rng); }
        bool lost = false;
        for (const auto& d : cfg.dropout_s) { lost = lost || (ts >= d.first && ts < d.second); }
        if (!lost) { ckp.push_back(to_tick(ts)); }
    }
    for (double ts : cfg.noise_s) { ckp.push_back(to_tick(ts)); }
    std::sort(ckp.begin(), ckp.end());
    if (cfg.cam) {
        for (long m = static_cast<long>(std::ceil((-off - cfg.cam_trigger_deg) / 720.0)); ; ++m) {
            const double th = off + cfg.cam_trigger_deg + 720.0 * static_cast<double>(m);
            if (th < 0.0) { continue; }
            if (th > th_max) { break; }
            cam.push_back(to_tick(r.time_at(th)));
        }
    }

    // ── 3. Reset firmware + simulated TIM5 ──────────────────────────────
    ems::drv::ckp_test_reset();
    ecu_sched_test_reset();
    const uint16_t saved_off = ems::engine::cfg::g_eng_cfg.trigger_tooth0_engine_deg;
    ems::engine::cfg::g_eng_cfg.trigger_tooth0_engine_deg = cfg.trigger_offset_deg;
    g_now = g_t0;
    ems_test_tim5_cnt = static_cast<uint32_t>(g_t0);
    ems_test_tim5_ccr3 = 0u;
    ems_test_tim5_sr = 0u;
    ems_test_tim5_dier = 0u;
    for (uint8_t& lv : g_level) { lv = 0U; }
    g_log = &r.edges;
    ems::hal::out_pins_host::write_hook = &pin_hook;
    if (cfg.presync_inj_mode >= 0) {
        ecu_sched_set_presync_inj_mode(static_cast<uint8_t>(cfg.presync_inj_mode));
    }
    const uint32_t wd0 = ecu_sched_dwell_watchdog_count();
    const uint32_t wi0 = ecu_sched_inj_watchdog_count();
    commit(r.cmd_fn(0.0));

    // ── 4. Event loop ────────────────────────────────────────────────────
    const uint64_t t_end = to_tick(cfg.duration_s);
    const uint64_t main_period = static_cast<uint64_t>(std::llround(cfg.main_period_s * kTickHz));
    uint64_t next_main = g_t0 + main_period;
    size_t ic = 0, im = 0;
    bool pend_cc1 = false, pend_cc2 = false;

    while (true) {
        uint64_t t_cc3 = kInf;
        if ((ems_test_tim5_dier & kCc3ie) != 0u) {
            if ((ems_test_tim5_sr & kCc3if) != 0u) {
                t_cc3 = g_now;
            } else {
                const int32_t d = static_cast<int32_t>(ems_test_tim5_ccr3 - static_cast<uint32_t>(g_now));
                if (d > 0) { t_cc3 = g_now + static_cast<uint64_t>(d); }
                // d <= 0: hardware only matches after a full 32-bit wrap.
            }
        }
        const uint64_t t_ckp = (ic < ckp.size()) ? ckp[ic] : kInf;
        const uint64_t t_cam = (im < cam.size()) ? cam[im] : kInf;
        uint64_t t = std::min({t_cc3, t_ckp, t_cam, next_main});
        if (pend_cc1 || pend_cc2) { t = g_now; }
        if (t >= t_end) { break; }
        advance(t);
        // Latch captures (overcapture: a later edge overwrites, like hardware).
        while (ic < ckp.size() && ckp[ic] <= g_now) {
            ems_test_tim5_ccr1 = static_cast<uint32_t>(ckp[ic++]);
            pend_cc1 = true;
        }
        while (im < cam.size() && cam[im] <= g_now) {
            ems_test_tim5_ccr2 = static_cast<uint32_t>(cam[im++]);
            pend_cc2 = true;
        }
        const bool cc3 = ((ems_test_tim5_sr & kCc3if) != 0u) && ((ems_test_tim5_dier & kCc3ie) != 0u);
        if (pend_cc1 || pend_cc2 || cc3) {
            // TIM5_IRQHandler: CH1, CH2, CH3 in order (timer.cpp).
            const uint64_t t_irq = g_now;
            advance(g_now + us_ticks(cfg.isr_entry_us));
            if (pend_cc1) {
                pend_cc1 = false;
                const double pre = cfg.isr_pre_sched_us + cfg.isr_pre_sched_jitter_us * uni(rng);
                advance(g_now + us_ticks(pre > 0.0 ? pre : 0.0));
                ems::drv::ckp_tim5_ch1_isr();
                advance(std::max(g_now, t_irq + us_ticks(cfg.isr_ckp_total_us)));
            }
            if (pend_cc2) {
                pend_cc2 = false;
                ems_test_cam_gpio_idr = (1u << 1u);
                ems::drv::ckp_tim5_ch2_isr();
                advance(g_now + us_ticks(1.0));
            }
            if ((ems_test_tim5_sr & kCc3if) != 0u) {
                ems_test_tim5_sr &= ~kCc3if;
                ecu_sched_evt_dispatch();
                advance(g_now + us_ticks(cfg.dispatch_us));
            }
            continue;
        }
        if (g_now >= next_main) {
            const double ts = static_cast<double>(g_now - g_t0) / kTickHz;
            commit(r.cmd_fn(ts));
            if (cfg.on_main) { cfg.on_main(ts); }
            ecu_sched_dwell_watchdog();
            ecu_sched_inj_watchdog();
            ems::drv::ckp_stall_poll(ems_test_tim5_cnt);
            next_main += main_period;
        }
    }

    ems::hal::out_pins_host::write_hook = nullptr;
    g_log = nullptr;
    ems::engine::cfg::g_eng_cfg.trigger_tooth0_engine_deg = saved_off;
    r.tim5_end = ems_test_tim5_cnt;
    r.t_end_s = cfg.duration_s;
    r.dwell_wdog = ecu_sched_dwell_watchdog_count() - wd0;
    r.inj_wdog = ecu_sched_inj_watchdog_count() - wi0;
    for (PinEdge& e : r.edges) { e.theta = r.theta_at(e.t_s); }
    return r;
}

Metrics analyze(const Result& r, double t_from, bool wasted)
{
    Metrics m;
    const double th_from = r.theta_at(t_from);
    const double th_to = r.theta_at(r.t_end_s) - 45.0;
    const double t_to = r.time_at(th_to);
    const double cycle_step = wasted ? 360.0 : 720.0;

    // ── Sparks: falling IGN edges vs ideal angle TDC − advance ─────────
    std::vector<char> used(r.edges.size(), 0);
    double sum = 0, sum2 = 0;
    for (int cyl = 0; cyl < 4; ++cyl) {
        const double tdc = ems::engine::cfg::cyl_tdc_deg(static_cast<uint8_t>(cyl));
        for (double base = std::floor(th_from / 720.0) * 720.0 - 720.0; base < th_to + 720.0; base += cycle_step) {
            const double t_guess = r.time_at(base + tdc);
            const double expect = base + tdc - r.cmd_at(t_guess).advance_deg;
            if (expect < th_from || expect > th_to) { continue; }
            int best = -1;
            double best_err = 1e9;
            for (size_t i = 0; i < r.edges.size(); ++i) {
                const PinEdge& e = r.edges[i];
                bool ign = false;
                if (used[i] || e.high || channel_cyl(e.ch, &ign) != cyl || !ign) { continue; }
                const double err = e.theta - expect;
                if (std::fabs(err) < 60.0 && std::fabs(err) < std::fabs(best_err)) { best = static_cast<int>(i); best_err = err; }
            }
            if (best < 0) {
                ++m.missing;
                if (std::getenv("SIM_VERBOSE") != nullptr) {
                    std::printf("    missing spark cyl %d at theta %.2f (t %.5f)\n", cyl, expect, r.time_at(expect));
                }
                continue;
            }
            used[static_cast<size_t>(best)] = 1;
            ++m.sparks;
            sum += best_err;
            sum2 += best_err * best_err;
            m.spark_err_max = std::max(m.spark_err_max, std::fabs(best_err));
        }
    }
    for (size_t i = 0; i < r.edges.size(); ++i) {
        const PinEdge& e = r.edges[i];
        bool ign = false;
        if (channel_cyl(e.ch, &ign) < 0 || !ign || e.high || used[i]) { continue; }
        if (e.theta >= th_from && e.theta <= th_to) { ++m.spurious; }
    }
    if (m.sparks > 0) {
        m.spark_err_mean = sum / m.sparks;
        m.spark_err_rms = std::sqrt(sum2 / m.sparks);
    }

    // ── Dwell and injector pulses (rising→falling per channel) ──────────
    double rise[8];
    for (double& x : rise) { x = -1.0; }
    std::vector<std::pair<double, double>> inj[4];  // (t_on, t_off) per cylinder
    for (const PinEdge& e : r.edges) {
        bool ign = false;
        const int cyl = channel_cyl(e.ch, &ign);
        if (cyl < 0) { continue; }
        if (e.high) { rise[e.ch] = e.t_s; continue; }
        if (rise[e.ch] < 0.0) { continue; }
        const double on = rise[e.ch];
        rise[e.ch] = -1.0;
        if (on < t_from || e.t_s > t_to) { continue; }
        if (ign) {
            const double cmd = r.cmd_at(e.t_s).dwell_ms;
            const double act = (e.t_s - on) * 1e3;
            const double err = std::fabs(act - cmd) / cmd * 100.0;
            m.dwell_err_max_pct = std::max(m.dwell_err_max_pct, err);
            if (act < 0.5 * cmd) { ++m.short_dwell; }
            if (act > 1.2 * cmd) { ++m.long_dwell; }
        } else {
            inj[cyl].push_back({on, e.t_s});
            ++m.inj_pulses;
            if (!wasted) {
                const double tdc = ems::engine::cfg::cyl_tdc_deg(static_cast<uint8_t>(cyl));
                const double eoi_exp_mod = std::fmod(tdc - r.cmd_at(e.t_s).eoi_deg + 1440.0, 720.0);
                double d = std::fmod(e.theta, 720.0) - eoi_exp_mod;
                while (d > 360.0) { d -= 720.0; }
                while (d < -360.0) { d += 720.0; }
                m.eoi_err_max = std::max(m.eoi_err_max, std::fabs(d));
            }
        }
    }

    // ── Fuel per cylinder per 720° window (sum of effective open time) ──
    double fsum = 0;
    int fn = 0;
    for (int cyl = 0; cyl < 4; ++cyl) {
        const auto& p = inj[cyl];
        if (p.empty()) { ++m.inj_missing; continue; }
        // Window edges 90 deg before a pulse: never on a pulse (360/720 spacing).
        const double th0 = r.theta_at(p.front().first) - 90.0;
        for (double w = th0; w + 720.0 <= r.theta_at(p.back().first) + 90.0; w += 720.0) {
            double eff = 0;
            const double cmd_fuel = r.cmd_at(r.time_at(w)).fuel_us;
            const double dead = r.cmd_at(r.time_at(w)).dead_us;
            for (const auto& pr : p) {
                const double th = r.theta_at(pr.first);
                if (th >= w - 1e-6 && th < w + 720.0 - 1e-6) { eff += (pr.second - pr.first) * 1e6 - dead; }
            }
            const double err = (eff - cmd_fuel) / cmd_fuel * 100.0;
            m.fuel_err_max_pct = std::max(m.fuel_err_max_pct, std::fabs(err));
            fsum += err;
            ++fn;
        }
    }
    if (fn > 0) { m.fuel_err_mean_pct = fsum / fn; }
    return m;
}

}  // namespace sim
