#pragma once
/**
 * Virtual engine for closed-loop timing tests (host only).
 *
 * Drives the REAL firmware capture + scheduler code through the single
 * simulated TIM5 (hal/tim5_host.h):
 *   crank kinematics θ(t)  →  60-2 / cam edges (with jitter, machining error,
 *   noise)  →  ckp_tim5_ch1/ch2_isr  →  ecu_sched hook  →  TIM5_CH3 compare
 *   →  ecu_sched_evt_dispatch  →  out_pin_write  →  timestamped pin log.
 *
 * The oracle is the kinematic model itself: every output edge is converted
 * back to the true engine angle at that instant, independent of firmware math.
 *
 * Angle convention (matches firmware):
 *   engine angle 0° = TDC of cylinder 0 (compression), cycle = 720°.
 *   trigger angle = engine angle − trigger_offset; tooth 0 (first tooth after
 *   the gap) is at trigger angle 0°, tooth k at 6k°, k = 0..57, gap 348..360°.
 */
#include <cstdint>
#include <functional>
#include <vector>

namespace sim {

constexpr double kTickHz = 62.5e6;  // TIM5

/** What the main loop commands (engine-level units, firmware independent). */
struct Command {
    double advance_deg = 10.0;  // spark, deg BTDC
    double dwell_ms    = 3.0;
    double fuel_us     = 2500.0;  // effective (flow) injector open time per cycle per cylinder
    double dead_us     = 0.0;     // injector dead time added per opening
    double eoi_deg     = 355.0;   // end of injection, deg BTDC of combustion TDC
    double cyl_retard_deg[4] = {0.0, 0.0, 0.0, 0.0};  // knock retard per cylinder
};

struct Config {
    // rpm profile: piecewise linear (t_s, rpm); held constant after last point.
    std::vector<std::pair<double, double>> rpm = {{0.0, 1000.0}};
    double duration_s = 1.0;
    double ripple = 0.0;  // compression speed ripple (fraction), slow at each TDC

    uint16_t trigger_offset_deg = 0;  // engine angle of tooth 0 (firmware cfg)
    double wheel_extra_deg = 0.0;     // physical tooth 0 is this much further (not in cfg)
    bool cam = true;
    double cam_trigger_deg = 600.0;   // cam edge, trigger-cycle angle [360,720) = rev before PHASE_A

    double edge_jitter_us = 0.0;      // gaussian σ on each CKP edge
    double tooth_error_deg = 0.0;     // ± uniform machining error per tooth (fixed)
    std::vector<double> noise_s;      // extra (spurious) CKP edges at these times
    std::vector<std::pair<double, double>> dropout_s;  // CKP edges lost in [a,b)

    // ISR timing model (µs). pre_sched = work before the scheduler reads TIM5_CNT.
    double isr_entry_us = 0.1;
    double isr_pre_sched_us = 2.0;
    double isr_pre_sched_jitter_us = 1.0;  // uniform ±
    double isr_ckp_total_us = 4.0;
    double dispatch_us = 0.3;
    bool hw_coil_oc = true;           // coils on TIM1/TIM8 compare (firmware default)

    int presync_inj_mode = -1;        // -1 keep firmware default, else ECU_PRESYNC_INJ_*
    uint32_t t0_ticks = 0;            // TIM5 start value (wrap tests)
    double main_period_s = 0.002;
    uint64_t seed = 1;

    Command cmd;                                       // constant command, or:
    std::function<Command(double t_s)> cmd_at;          // time-varying command
    std::function<void(double t_s)> on_main;            // extra main-loop work
};

struct PinEdge {
    uint8_t ch;      // ECU_CH_*
    uint8_t high;
    double t_s;
    double theta;    // true cumulative engine angle at edge (deg)
};

struct Metrics {
    int    sparks = 0, missing = 0, spurious = 0;
    double spark_err_max = 0, spark_err_rms = 0, spark_err_mean = 0;  // deg, +late
    double dwell_err_max_pct = 0;    // |actual-cmd|/cmd
    int    short_dwell = 0;          // sparks after < 50 % of commanded dwell
    int    long_dwell = 0;           // coil held > 120 % of commanded dwell
    int    inj_pulses = 0, inj_missing = 0;
    double fuel_err_max_pct = 0;     // per-cylinder per-cycle effective fuel
    double fuel_err_mean_pct = 0;
    double eoi_err_max = 0;          // deg
};

struct Result {
    std::vector<PinEdge> edges;
    std::vector<double> t, theta;    // kinematic trajectory (cumulative θ)
    uint32_t tim5_end = 0;
    double t_end_s = 0;
    uint32_t dwell_wdog = 0, inj_wdog = 0;   // firmware watchdog trips
    Command cmd_at(double t_s) const;
    std::function<Command(double)> cmd_fn;
    double theta_at(double t_s) const;   // cumulative
    double time_at(double theta) const;  // inverse
};

Result run(const Config& cfg);

/**
 * Compare outputs with the ideal (from the kinematic oracle) for t ≥ t_from.
 * wasted = coils fire every 360° (presync) → match either TDC.
 */
Metrics analyze(const Result& r, double t_from, bool wasted);

/** Engine cylinder (0..3) of an ECU channel, or -1. */
int channel_cyl(uint8_t ch, bool* is_ign);

}  // namespace sim
