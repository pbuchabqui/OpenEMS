/**
 * @file torque_manager.cpp
 * @brief Gerenciador de Torque
 *
 * Produção: ems::engine::torque_manager_update (inteiro) — main 2 ms.
 * Host:     torque_manager_loop (float) — só EMS_HOST_TEST.
 */

#include "torque_manager.h"
#include "engine/quick_crank.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"
#include "etb_control.h"
#include "app/can_rx_map.h"
#include "hal/system.h"

#include <string.h>
#if defined(EMS_HOST_TEST)
#include <math.h>
#endif

static bool g_initialized = false;
static bool g_limp_active = false;

#if defined(EMS_HOST_TEST)
static torque_manager_config_t g_config = {};
static torque_manager_outputs_t g_outputs = {};
static float g_filtered_pedal = 0.0f;
#endif

bool torque_manager_init(void) {
    g_initialized = true;
    g_limp_active = false;
#if defined(EMS_HOST_TEST)
    memset(&g_config, 0, sizeof(g_config));
    memset(&g_outputs, 0, sizeof(g_outputs));
    g_config.max_rpm = 7000.0f;
    g_config.max_rpm_hot = 7200.0f;
    g_config.max_speed = 220.0f;
    g_config.min_coolant_temp = 0.0f;
    g_config.launch_rpm = 4500.0f;
    g_config.launch_throttle = 60.0f;
    g_config.tc_max_slip = 15.0f;
    g_config.tc_reduction_rate = 50.0f;
    g_config.pedal_filter_alpha = 0.3f;
    g_filtered_pedal = 0.0f;
#endif
    return true;
}

void torque_manager_enter_limp(void) {
    g_limp_active = true;
}

bool torque_manager_is_ready(void) {
    return g_initialized && !g_limp_active;
}

#if defined(EMS_HOST_TEST)

void torque_manager_loop(const torque_manager_inputs_t* inputs,
                         torque_manager_outputs_t* outputs) {
    if (!g_initialized || inputs == NULL || outputs == NULL) {
        return;
    }

    g_outputs.last_update_ms += 1.0f;

    g_filtered_pedal = g_config.pedal_filter_alpha * inputs->pedal_percent +
                       (1.0f - g_config.pedal_filter_alpha) * g_filtered_pedal;

    float requested_throttle = g_filtered_pedal;

    if (g_limp_active || inputs->limp_mode) {
        outputs->throttle_target = 5.0f;
        outputs->torque_limit = 30.0f;
        outputs->spark_trim = 0.0f;
        g_outputs.intervention_count++;
        return;
    }

    if (inputs->idle_mode) {
        requested_throttle = 0.0f;
    }

    if (inputs->launch_control) {
        if (inputs->engine_rpm > g_config.launch_rpm) {
            requested_throttle = g_config.launch_throttle;
            g_outputs.intervention_count++;
        }
    }

    if (inputs->engine_rpm >= g_config.max_rpm) {
        if (inputs->engine_rpm >= g_config.max_rpm + 200.0f) {
            requested_throttle = 0.0f;
        } else {
            float cutoff_factor = 1.0f - ((inputs->engine_rpm - g_config.max_rpm) / 200.0f);
            requested_throttle *= cutoff_factor;
        }
        g_outputs.rpm_cutoff_count++;
    }

    if (inputs->coolant_temp > 105.0f && inputs->engine_rpm >= g_config.max_rpm_hot) {
        requested_throttle = 0.0f;
        g_outputs.rpm_cutoff_count++;
    }

    if (inputs->vehicle_speed > 0.1f && inputs->vehicle_speed >= g_config.max_speed) {
        if (requested_throttle > 10.0f) {
            requested_throttle = 10.0f;
        }
        g_outputs.intervention_count++;
    }

    if (inputs->traction_active) {
        float reduction = inputs->tc_reduction;
        if (reduction > 80.0f) reduction = 80.0f;
        requested_throttle *= (1.0f - (reduction / 100.0f));
        g_outputs.tc_intervention++;
    }

    if (inputs->cruise_target > 0.1f && inputs->pedal_percent < 5.0f) {
        requested_throttle = inputs->cruise_target;
    }

    if (inputs->coolant_temp < g_config.min_coolant_temp) {
        if (requested_throttle > 30.0f) {
            requested_throttle = 30.0f;
        }
    }

    if (requested_throttle < 0.0f) requested_throttle = 0.0f;
    if (requested_throttle > 100.0f) requested_throttle = 100.0f;

    outputs->torque_limit = requested_throttle;

    if (inputs->traction_active) {
        outputs->spark_trim = -(inputs->tc_reduction * 0.3f);
        if (outputs->spark_trim < -150.0f) outputs->spark_trim = -150.0f;
        if (outputs->spark_trim > 50.0f) outputs->spark_trim = 50.0f;
    } else {
        outputs->spark_trim = 0.0f;
    }

    outputs->throttle_target = requested_throttle;
    outputs->intervention_count = g_outputs.intervention_count;
    outputs->rpm_cutoff_count = g_outputs.rpm_cutoff_count;
    outputs->tc_intervention = g_outputs.tc_intervention;
}

void torque_manager_set_config(const torque_manager_config_t* config) {
    if (config == NULL) return;
    memcpy(&g_config, config, sizeof(torque_manager_config_t));
}

const torque_manager_config_t* torque_manager_get_config(void) {
    return &g_config;
}

#endif  // EMS_HOST_TEST

// ── API inteira de produção ────────────────────────────────────────────────

namespace ems::engine {

static uint16_t g_etb_target_x10   = 0u;
static uint8_t  g_limp_reason      = 0u;
// Integrador de ar para marcha lenta (pct×10). Acumula abertura necessária
// para manter RPM no target; limitado entre etb_idle_min/max_opening_x10.
static int32_t  g_idle_air_int_x10 = 0;
// Crank→idle taper: hold elevated blade after first fire, then blend to idle I.
static bool     g_prev_idle_cranking = false;
static bool     g_crank_taper_active = false;
static uint32_t g_crank_taper_start_ms = 0u;
static int32_t  g_crank_taper_from_x10 = 0;
// Idle phase latch (RPM upper hysteresis — avoid flapping idle↔coasting).
static bool     g_idle_phase = false;
// Dashpot: hold blade after tip-out (APP→closed) then decay toward idle.
static int32_t  g_dashpot_x10 = 0;
static bool     g_prev_app_idle = false;

// Open-loop crank air (pct×10). Prefer max idle opening; never below min.
static uint16_t crank_open_pct_x10() noexcept {
    uint16_t lo = etb_idle_min_opening_x10;
    uint16_t hi = etb_idle_max_opening_x10;
    if (hi < lo) {
        const uint16_t t = lo;
        lo = hi;
        hi = t;
    }
    if (hi < 10u) {
        hi = 80u;  // 8% fallback
    }
    return hi;
}

// Idle RPM upper band (rpm×10). Reuse idle-spark window cal when sane.
static uint32_t idle_rpm_upper_x10() noexcept {
    uint32_t u = idle_spark_window_above_target_x10;
    if (u < 500u) {
        u = 4000u;  // 400 RPM
    }
    if (u > 20000u) {
        u = 20000u;
    }
    return u;
}

constexpr uint32_t kCrankToIdleTaperMs = 1000u;

// Launch state machine
enum class LaunchState : uint8_t { Idle = 0, Armed = 1, Active = 2 };
static LaunchState g_launch_state = LaunchState::Idle;
static uint8_t     g_launch_force = 0u;  // 0=cal, 1=force on, 2=force off

// TC state
static uint16_t g_tc_reduction_x10 = 0u;     // current applied cut 0–1000
static uint32_t g_tc_prev_rpm_x10  = 0u;
static uint16_t g_tc_ext_slip_x10  = 0u;     // external slip; 0 = none
static uint8_t  g_tc_ext_slip_valid = 0u;
static uint8_t  g_launch_active_latched = 0u;
static uint16_t g_tc_reduction_latched = 0u;
static int16_t  g_spark_retard_latched = 0;
// Last CAN speeds seen by TC (0 if invalid/timeout)
static uint16_t g_can_vehicle_kmh = 0u;
static uint16_t g_can_wheel_kmh   = 0u;

// Interpolação inteira do pedal map: app_x10 (0-1000) → throttle_x10 (0-1000).
// 10 pontos em pedal 0%,10%,…,90% (índices 0..9); 100% pedal → ponto [9].
// Segmentos interpolados: [i]→[i+1] para i=0..8 (app 0..899).
// app 900..999: flat no ponto [9] (antes lia [10] → OOB).
static uint16_t pedal_map_lookup(uint16_t app_x10) noexcept {
    const uint8_t mode = static_cast<uint8_t>(etb_get_drive_mode());
    if (mode >= 4u) { return 0u; }
    if (app_x10 >= 900u) { return etb_pedal_map[mode][9]; }
    const uint16_t idx  = app_x10 / 100u;           // 0..8
    const uint16_t frac = app_x10 - idx * 100u;     // 0..99
    const uint16_t lo   = etb_pedal_map[mode][idx];
    const uint16_t hi   = etb_pedal_map[mode][idx + 1u];
    return static_cast<uint16_t>(
        lo + (static_cast<uint32_t>(hi - lo) * frac) / 100u);
}

static bool launch_is_enabled() noexcept {
    if (g_launch_force == 1u) { return true; }
    if (g_launch_force == 2u) { return false; }
    return launch_enable != 0u;
}

void torque_manager_reset() noexcept {
    g_etb_target_x10   = 0u;
    g_limp_reason      = 0u;
    g_idle_air_int_x10 = 0;
    g_prev_idle_cranking = false;
    g_crank_taper_active = false;
    g_crank_taper_start_ms = 0u;
    g_crank_taper_from_x10 = 0;
    g_idle_phase = false;
    g_dashpot_x10 = 0;
    g_prev_app_idle = false;
    g_launch_state     = LaunchState::Idle;
    g_tc_reduction_x10 = 0u;
    g_tc_prev_rpm_x10  = 0u;
    g_tc_ext_slip_x10  = 0u;
    g_tc_ext_slip_valid = 0u;
    g_launch_active_latched = 0u;
    g_tc_reduction_latched = 0u;
    g_spark_retard_latched = 0;
    g_can_vehicle_kmh = 0u;
    g_can_wheel_kmh   = 0u;
}

void torque_tc_set_external_slip_pct_x10(uint16_t slip_pct_x10) noexcept {
    g_tc_ext_slip_x10 = (slip_pct_x10 > 1000u) ? 1000u : slip_pct_x10;
    g_tc_ext_slip_valid = 1u;
}

void torque_tc_clear_external_slip() noexcept {
    g_tc_ext_slip_x10 = 0u;
    g_tc_ext_slip_valid = 0u;
}

void torque_launch_force_enable(uint8_t on) noexcept {
    g_launch_force = (on > 2u) ? 0u : on;
}

TorqueOutput torque_manager_update(
    const ems::drv::CkpSnapshot& snap,
    const ems::drv::SensorData&  sensors,
    bool key_on, bool map_clt_limp, bool rev_cut,
    uint16_t idle_target_rpm_x10, uint16_t period_ms) noexcept
{
    TorqueOutput out{};
    out.limp_reason = 0u;

    if (!key_on) {
        out.etb_enable_request = false;
        g_etb_target_x10 = 0u;
        g_limp_reason    = 0u;
        return out;
    }

    if (etb_cal_valid == 0u) {
        out.limp_reason |= TORQUE_LIMP_NO_CALIB;
    }
    if (map_clt_limp) {
        out.limp_reason |= TORQUE_LIMP_MAP_CLT;
    }
    if (rev_cut) {
        out.limp_reason |= TORQUE_LIMP_REV_CUT;
    }
    if ((sensors.throttle_fault_bits & (ems::drv::THROTTLE_FAULT_APP1 |
                                         ems::drv::THROTTLE_FAULT_APP2 |
                                         ems::drv::THROTTLE_FAULT_APP_PLAUS)) != 0u) {
        out.limp_reason |= TORQUE_LIMP_APP_FAULT;
    }
    if ((sensors.throttle_fault_bits & (ems::drv::THROTTLE_FAULT_ETB_TPS1 |
                                         ems::drv::THROTTLE_FAULT_ETB_TPS2 |
                                         ems::drv::THROTTLE_FAULT_ETB_PLAUS)) != 0u) {
        out.limp_reason |= TORQUE_LIMP_ETB_FAULT;
    }

    // Base target from driver pedal — apply response map
    uint16_t target_x10 = pedal_map_lookup(sensors.app_pct_x10);

    // Limp mode: clamp to etb_max_open_pct_x10_limp
    if ((out.limp_reason & (TORQUE_LIMP_MAP_CLT | TORQUE_LIMP_APP_FAULT)) != 0u) {
        if (target_x10 > etb_max_open_pct_x10_limp) {
            target_x10 = etb_max_open_pct_x10_limp;
        }
    }
    if ((out.limp_reason & TORQUE_LIMP_NO_CALIB) != 0u) {
        target_x10 = 0u;
    }

    // ── Progressive ETB rev pullback (production; ported from float API) ──
    // Starts closing the blade *before* the fuel rev-limit so air demand falls
    // first. Uses rev_limit_rpm_x10 / rev_limit_soft_window_x10 (same knobs as
    // fuel cut). Does NOT replace fuel cut in main — only ETB target.
    //
    //   soft_start ──────── hard (fuel cut) ──── hard+margin
    //   full open           target→0            hold 0
    //
    // Hot CLT (>105°C): soft_start and hard pull earlier by half the window.
    {
        uint32_t hard = rev_limit_rpm_x10;
        uint32_t window = rev_limit_soft_window_x10;
        if (hard < 10000u) { hard = 70000u; }           // 7000 RPM fallback
        if (window < 500u || window > hard) { window = 2000u; }  // 200 RPM

        if (sensors.clt_degc_x10 > 1050) {              // > 105°C
            const uint32_t pull = window / 2u;
            hard = (hard > pull) ? (hard - pull) : hard;
        }

        const uint32_t soft_start = (hard > window) ? (hard - window) : 0u;
        const uint32_t rpm = snap.rpm_x10;

        if (rpm >= hard) {
            // At/above fuel-cut threshold: blade closed (air path safe).
            target_x10 = 0u;
            out.limp_reason |= TORQUE_LIMP_REV_CUT;
        } else if (rpm > soft_start && hard > soft_start) {
            // Linear fade: full at soft_start → 0 at hard.
            const uint32_t span = hard - soft_start;
            const uint32_t over = rpm - soft_start;
            const uint32_t keep = span - over;  // span..0
            target_x10 = static_cast<uint16_t>(
                (static_cast<uint32_t>(target_x10) * keep) / span);
            // Soft band: flag limp for telemetry without implying fuel cut.
            out.limp_reason |= TORQUE_LIMP_REV_CUT;
        }
    }

    // External rev_cut (main limp @ high RPM): force closed regardless.
    if (rev_cut) {
        out.limp_reason |= TORQUE_LIMP_REV_CUT;
        target_x10 = 0u;
    }

    const uint16_t dt = (period_ms == 0u) ? 2u : period_ms;
    int32_t i_step = static_cast<int32_t>(dt) / 2;  // 2ms → 1
    if (i_step < 1) { i_step = 1; }

    // ── Launch control ────────────────────────────────────────────────────
    // Street-lite (no clutch): arm when enable && APP ≥ arm; active while APP
    // stays high; disarm when APP < disarm. While active, hold RPM near
    // launch_rpm by capping and cutting ETB.
    int16_t spark_retard = 0;
    uint8_t launch_active = 0u;
    {
        const bool en = launch_is_enabled();
        const uint16_t app = sensors.app_pct_x10;
        const uint32_t rpm = snap.rpm_x10;
        const uint16_t arm_app = (launch_app_arm_x10 > 1000u) ? 200u : launch_app_arm_x10;
        const uint16_t disarm_app = (launch_app_disarm_x10 >= arm_app)
            ? static_cast<uint16_t>(arm_app / 2u) : launch_app_disarm_x10;
        const uint32_t l_rpm = (launch_rpm_x10 < 1000u) ? 45000u : launch_rpm_x10;
        const uint16_t l_etb = (launch_etb_pct_x10 > 1000u) ? 600u : launch_etb_pct_x10;
        const uint32_t hyst = (launch_rpm_hyst_x10 == 0u) ? 100u : launch_rpm_hyst_x10;

        if (!en || (out.limp_reason & (TORQUE_LIMP_NO_CALIB | TORQUE_LIMP_REV_CUT)) != 0u) {
            g_launch_state = LaunchState::Idle;
        } else {
            switch (g_launch_state) {
            case LaunchState::Idle:
                if (app >= arm_app) { g_launch_state = LaunchState::Armed; }
                break;
            case LaunchState::Armed:
                if (app < disarm_app) {
                    g_launch_state = LaunchState::Idle;
                } else {
                    g_launch_state = LaunchState::Active;  // control immediately when armed
                }
                break;
            case LaunchState::Active:
                if (app < disarm_app) { g_launch_state = LaunchState::Idle; }
                break;
            }
        }

        if (g_launch_state == LaunchState::Active || g_launch_state == LaunchState::Armed) {
            launch_active = 1u;
            out.limp_reason |= TORQUE_ACTIVE_LAUNCH;
            // Cap blade
            if (target_x10 > l_etb) { target_x10 = l_etb; }
            // Over target RPM: progressive extra cut
            if (rpm > l_rpm + hyst) {
                const uint32_t over = rpm - l_rpm;
                // 500 rpm_x10 over → full cut of remaining
                uint32_t cut = (over * 1000u) / 5000u;
                if (cut > 1000u) { cut = 1000u; }
                target_x10 = static_cast<uint16_t>(
                    (static_cast<uint32_t>(target_x10) * (1000u - cut)) / 1000u);
                // Mild spark retard when far over (up to 8°)
                const int16_t retard = static_cast<int16_t>((over * 8u) / 5000u);
                spark_retard = (retard > 8) ? 8 : retard;
            } else if (rpm > l_rpm) {
                // Inside hyst band above target: hold cap only
            }
            // Below target: leave pedal (capped) so RPM can climb to launch rpm
        }
    }

    // ── Traction control ──────────────────────────────────────────────────
    // Slip priority:
    //   1) external API (tests / override)
    //   2) CAN wheel vs vehicle (can_rx_map SPEED_KMH + WHEEL_SPEED_KMH)
    //   3) RPM-flare proxy when APP/RPM gates pass
    uint16_t tc_red = 0u;
    {
        const bool en = (tc_enable != 0u);
        const uint16_t app = sensors.app_pct_x10;
        const uint32_t rpm = snap.rpm_x10;
        const uint16_t app_min = (tc_app_min_x10 > 1000u) ? 300u : tc_app_min_x10;
        const uint32_t rpm_min = (tc_rpm_min_x10 < 1000u) ? 20000u : tc_rpm_min_x10;
        const uint32_t now_ms = ::millis();

        // rpm_dot (rpm_x10 per second)
        int32_t rpm_dot = 0;
        if (g_tc_prev_rpm_x10 != 0u && dt > 0u) {
            rpm_dot = (static_cast<int32_t>(rpm) - static_cast<int32_t>(g_tc_prev_rpm_x10))
                      * 1000 / static_cast<int32_t>(dt);
        }
        g_tc_prev_rpm_x10 = rpm;

        // Latch CAN speeds for diagnostics / speed limiter
        uint16_t veh_kmh = 0u;
        uint16_t whl_kmh = 0u;
        const bool have_veh = ems::app::can_rx_speed_kmh(veh_kmh, now_ms);
        const bool have_whl = ems::app::can_rx_wheel_speed_kmh(whl_kmh, now_ms);
        g_can_vehicle_kmh = have_veh ? veh_kmh : 0u;
        g_can_wheel_kmh   = have_whl ? whl_kmh : 0u;

        uint16_t slip_x10 = 0u;
        if (g_tc_ext_slip_valid != 0u) {
            slip_x10 = g_tc_ext_slip_x10;
        } else if (en && have_whl && have_veh && app >= app_min && rpm >= rpm_min) {
            // True slip: (driven − vehicle) / vehicle. Deadband 3%.
            // If only one speed is mapped, the other branch falls through to rpm_dot.
            if (whl_kmh > veh_kmh && veh_kmh > 0u) {
                const uint32_t num = static_cast<uint32_t>(whl_kmh - veh_kmh) * 1000u;
                uint32_t slip = num / static_cast<uint32_t>(veh_kmh);
                if (slip > 1000u) { slip = 1000u; }
                if (slip >= 30u) {  // ignore <3% noise
                    slip_x10 = static_cast<uint16_t>(slip);
                }
            } else if (whl_kmh > veh_kmh && veh_kmh == 0u && whl_kmh >= 5u) {
                // Vehicle reports 0 but driven wheel spinning (launch / ABS freeze):
                // treat as high slip scaled by wheel speed (cap at 100%).
                uint32_t slip = static_cast<uint32_t>(whl_kmh) * 20u;  // 50 km/h → 100%
                if (slip > 1000u) { slip = 1000u; }
                slip_x10 = static_cast<uint16_t>(slip);
            }
        } else if (en && have_whl && !have_veh && app >= app_min && rpm >= rpm_min) {
            // Driven wheel only: no body speed — use rpm_dot still, but also
            // treat very high wheel speed under high APP as mild slip proxy is weak;
            // fall through to rpm_dot below when slip still 0.
        }

        if (slip_x10 == 0u && en && app >= app_min && rpm >= rpm_min && rpm_dot > 0
            && g_tc_ext_slip_valid == 0u
            && !(have_whl && have_veh)) {
            // RPM-flare proxy only when CAN dual-speed slip path is not active
            const int32_t thresh = static_cast<int32_t>(
                (tc_rpm_dot_thresh < 500u) ? 8000u : tc_rpm_dot_thresh);
            if (rpm_dot > thresh) {
                const int32_t excess = rpm_dot - thresh;
                int32_t slip = (excess * 1000) / thresh;
                if (slip > 1000) { slip = 1000; }
                slip_x10 = static_cast<uint16_t>(slip);
            }
        }

        // Vehicle speed limiter (CAN body speed)
        if (have_veh && veh_kmh >= 220u) {
            if (target_x10 > 100u) { target_x10 = 100u; }  // ~10% hold
        }

        // Desired reduction from slip (0 slip → 0 cut; 100% slip → max cut)
        uint16_t desired = 0u;
        if (en && slip_x10 > 0u) {
            const uint16_t max_cut = (tc_max_reduction_pct_x10 > 1000u)
                ? 800u : tc_max_reduction_pct_x10;
            desired = static_cast<uint16_t>(
                (static_cast<uint32_t>(slip_x10) * max_cut) / 1000u);
        }

        // Slew reduction toward desired (rate in %×10 per second)
        const uint16_t rate = (tc_reduction_rate_x10 == 0u) ? 500u : tc_reduction_rate_x10;
        const uint32_t step = (static_cast<uint32_t>(rate) * dt) / 1000u;
        const uint32_t step_i = (step < 1u) ? 1u : step;
        if (desired > g_tc_reduction_x10) {
            const uint32_t next = static_cast<uint32_t>(g_tc_reduction_x10) + step_i;
            g_tc_reduction_x10 = static_cast<uint16_t>(
                next > desired ? desired : next);
        } else if (desired < g_tc_reduction_x10) {
            if (g_tc_reduction_x10 > step_i) {
                g_tc_reduction_x10 = static_cast<uint16_t>(g_tc_reduction_x10 - step_i);
            } else {
                g_tc_reduction_x10 = desired;
            }
        }

        // Don't fight launch hard-close / rev cut zeros
        if ((out.limp_reason & TORQUE_LIMP_REV_CUT) != 0u && target_x10 == 0u) {
            g_tc_reduction_x10 = 0u;
        }

        tc_red = g_tc_reduction_x10;
        if (tc_red > 0u) {
            out.limp_reason |= TORQUE_ACTIVE_TC;
            target_x10 = static_cast<uint16_t>(
                (static_cast<uint32_t>(target_x10) * (1000u - tc_red)) / 1000u);
            // Spark retard proportional to reduction (up to tc_spark_retard_max_deg)
            const uint16_t max_ret = (tc_spark_retard_max_deg > 30u)
                ? 12u : tc_spark_retard_max_deg;
            const int16_t ret = static_cast<int16_t>(
                (static_cast<uint32_t>(tc_red) * max_ret) / 1000u);
            if (ret > spark_retard) { spark_retard = ret; }
        }
    }

    // Idle air / crank open-loop / crank→idle taper.
    // Skip while rev-limiting / launch / heavy TC.
    const bool rev_limiting = (out.limp_reason & TORQUE_LIMP_REV_CUT) != 0u;
    const bool app_idle = (sensors.app_pct_x10 < etb_idle_open_pct_x10);
    const bool cranking_now = is_cranking();
    const int32_t idle_min = static_cast<int32_t>(etb_idle_min_opening_x10);
    const int32_t idle_max = static_cast<int32_t>(etb_idle_max_opening_x10);
    const uint16_t crank_open = crank_open_pct_x10();
    const uint32_t now_ms = ::millis();

    // Crank edge → start taper from crank open toward idle I.
    if (cranking_now) {
        g_idle_air_int_x10 = static_cast<int32_t>(crank_open);
        g_crank_taper_active = false;
        g_crank_taper_from_x10 = static_cast<int32_t>(crank_open);
        g_idle_phase = true;
    } else if (g_prev_idle_cranking) {
        g_crank_taper_active = true;
        g_crank_taper_start_ms = now_ms;
        g_crank_taper_from_x10 = static_cast<int32_t>(crank_open);
        if (g_idle_air_int_x10 < idle_min) {
            g_idle_air_int_x10 = idle_min;
        }
    }
    g_prev_idle_cranking = cranking_now;

    // RPM hysteresis for idle phase (closed APP only).
    const uint32_t upper = idle_rpm_upper_x10();
    const uint32_t exit_rpm = static_cast<uint32_t>(idle_target_rpm_x10)
                            + (upper + upper / 2u);  // target + 1.5×upper
    const uint32_t enter_rpm = static_cast<uint32_t>(idle_target_rpm_x10) + upper;
    if (!app_idle || rev_limiting || launch_active != 0u || tc_red >= 100u) {
        g_idle_phase = false;
    } else if (snap.rpm_x10 > 0u) {
        if (g_idle_phase) {
            if (snap.rpm_x10 > exit_rpm) {
                g_idle_phase = false;
            }
        } else if (snap.rpm_x10 < enter_rpm) {
            g_idle_phase = true;
        }
    }

    const bool allow_idle_air = !rev_limiting && launch_active == 0u && tc_red < 100u;

    if (allow_idle_air && cranking_now) {
        // Open-loop crank air: guarantee blade for first fire.
        if (target_x10 < crank_open) {
            target_x10 = crank_open;
        }
    } else if (allow_idle_air && g_crank_taper_active) {
        const uint32_t elapsed = now_ms - g_crank_taper_start_ms;
        if (elapsed >= kCrankToIdleTaperMs) {
            g_crank_taper_active = false;
        } else {
            // Linear blend crank_open → current idle I floor.
            const int32_t to = (g_idle_air_int_x10 < idle_min)
                ? idle_min
                : (g_idle_air_int_x10 > idle_max ? idle_max : g_idle_air_int_x10);
            const int32_t from = g_crank_taper_from_x10;
            const int32_t blended = from
                + ((to - from) * static_cast<int32_t>(elapsed))
                  / static_cast<int32_t>(kCrankToIdleTaperMs);
            const uint16_t floor = static_cast<uint16_t>(
                blended < 0 ? 0 : (blended > 1000 ? 1000 : blended));
            if (target_x10 < floor) {
                target_x10 = floor;
            }
        }
    }

    if (allow_idle_air && g_idle_phase && !cranking_now && snap.rpm_x10 > 0u) {
        // Closed-loop idle I (after crank open / under taper floor).
        const int32_t rpm_error = static_cast<int32_t>(idle_target_rpm_x10)
                                  - static_cast<int32_t>(snap.rpm_x10);
        if (rpm_error > 50) {          // deadband ±5 RPM (×10)
            g_idle_air_int_x10 += i_step;
        } else if (rpm_error < -50) {
            g_idle_air_int_x10 -= i_step;
        }
        if (g_idle_air_int_x10 < idle_min) { g_idle_air_int_x10 = idle_min; }
        if (g_idle_air_int_x10 > idle_max) { g_idle_air_int_x10 = idle_max; }
        if (!g_crank_taper_active) {
            const uint16_t idle_floor = static_cast<uint16_t>(g_idle_air_int_x10);
            if (target_x10 < idle_floor) {
                target_x10 = idle_floor;
            }
        }
    } else if (!cranking_now &&
               (sensors.app_pct_x10 >= etb_idle_open_pct_x10 || rev_limiting ||
                launch_active != 0u || !g_idle_phase)) {
        if (g_idle_air_int_x10 > 0) {
            g_idle_air_int_x10 -= i_step;
            if (g_idle_air_int_x10 < 0) { g_idle_air_int_x10 = 0; }
        }
    }

    // ── Dashpot (return-to-idle) ─────────────────────────────────────────
    // Ao soltar o pedal (drive → app_idle), segura a lâmina no último target
    // e decai com τ≈400 ms até o floor de idle — anti-stall / anti-solavanco.
    {
        if (!app_idle || rev_limiting || launch_active != 0u || cranking_now) {
            g_dashpot_x10 = 0;
        } else if (!g_prev_app_idle) {
            // Entrada em idle: seed com o target actual (ainda o do drive).
            int32_t seed = static_cast<int32_t>(target_x10);
            if (seed < static_cast<int32_t>(g_etb_target_x10)) {
                seed = static_cast<int32_t>(g_etb_target_x10);
            }
            if (seed < idle_min) {
                seed = idle_min;
            }
            // Cap dashpot — não segurar WOT no tip-out.
            constexpr int32_t kDashpotMaxX10 = 350;  // 35%
            if (seed > kDashpotMaxX10) {
                seed = kDashpotMaxX10;
            }
            g_dashpot_x10 = seed;
        } else if (g_dashpot_x10 > 0) {
            // Decay: ~400 ms time constant → step = dashpot * dt / 400
            int32_t step = (g_dashpot_x10 * static_cast<int32_t>(dt)) / 400;
            if (step < 1) {
                step = 1;
            }
            g_dashpot_x10 -= step;
            if (g_dashpot_x10 <= idle_min) {
                g_dashpot_x10 = 0;  // hand-off ao idle I
            }
        }
        g_prev_app_idle = app_idle;

        if (g_dashpot_x10 > 0 && allow_idle_air) {
            const uint16_t floor = static_cast<uint16_t>(
                g_dashpot_x10 > 1000 ? 1000 : g_dashpot_x10);
            if (target_x10 < floor) {
                target_x10 = floor;
            }
        }
    }

    // Rate-limit do alvo de lâmina (calibrável etb_max_rate_pct_per_s).
    // Unidades: rate em %/s; target em pct×10 (0–1000).
    // max_delta_x10 = rate × period_ms / 100  (porque %×10 = % × 10).
    // rate=0 → sem limite. Bypass em rev-cut / ETB fault / no-cal (fecho seguro
    // imediato — não atrasar proteção por slew de dirigibilidade).
    {
        const bool safety_immediate =
            (out.limp_reason & (TORQUE_LIMP_REV_CUT | TORQUE_LIMP_ETB_FAULT |
                                TORQUE_LIMP_NO_CALIB)) != 0u;
        const uint16_t rate = etb_max_rate_pct_per_s;
        if (!safety_immediate && rate > 0u) {
            const uint32_t max_delta =
                (static_cast<uint32_t>(rate) * static_cast<uint32_t>(dt)) / 100u;
            const uint32_t step = (max_delta < 1u) ? 1u : max_delta;
            const uint16_t prev = g_etb_target_x10;
            if (target_x10 > prev) {
                const uint32_t next = static_cast<uint32_t>(prev) + step;
                target_x10 = static_cast<uint16_t>(
                    next > target_x10 ? target_x10 : next);
            } else if (target_x10 < prev) {
                if (static_cast<uint32_t>(prev) - static_cast<uint32_t>(target_x10) > step) {
                    target_x10 = static_cast<uint16_t>(
                        static_cast<uint32_t>(prev) - step);
                }
                // else: step cobre o restante → target_x10 já é o pedido final
            }
        }
    }

    g_etb_target_x10 = target_x10;
    g_limp_reason    = out.limp_reason;
    g_launch_active_latched = launch_active;
    g_tc_reduction_latched = tc_red;
    g_spark_retard_latched = spark_retard;

    out.etb_target_pct_x10     = target_x10;
    out.etb_max_rate_pct_per_s = etb_max_rate_pct_per_s;
    out.etb_enable_request     = (out.limp_reason & TORQUE_LIMP_ETB_FAULT) == 0u;
    out.spark_retard_deg       = spark_retard;
    out.tc_reduction_pct_x10   = tc_red;
    out.launch_active          = launch_active;
    out.tc_active              = (tc_red > 0u) ? 1u : 0u;
    return out;
}

uint16_t torque_manager_get_target()      noexcept { return g_etb_target_x10; }
uint8_t  torque_manager_get_limp_reason() noexcept { return g_limp_reason; }
uint8_t  torque_manager_get_launch_active() noexcept { return g_launch_active_latched; }
uint16_t torque_manager_get_tc_reduction() noexcept { return g_tc_reduction_latched; }
int16_t  torque_manager_get_spark_retard() noexcept { return g_spark_retard_latched; }
uint16_t torque_manager_get_vehicle_kmh() noexcept { return g_can_vehicle_kmh; }
uint16_t torque_manager_get_wheel_kmh() noexcept { return g_can_wheel_kmh; }

}  // namespace ems::engine
