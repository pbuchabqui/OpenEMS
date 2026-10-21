/**
 * @file etb_control.cpp
 * @brief ETB: init/drive-mode (prod) + PID inteiro + float loop (host only)
 */

#include "etb_control.h"
#include "engine/calibration.h"
#include "engine/limp_gating.h"
#include "engine/math_utils.h"
#include "hal/etb_driver.h"

#include <string.h>
#include <algorithm>
#if defined(EMS_HOST_TEST)
#include <math.h>
#endif

// ── Estado partilhado produção / host ───────────────────────────────────────
static etb_drive_mode_t g_drive_mode = ETB_MODE_NORMAL;
static bool g_initialized = false;
static bool g_limp_mode = false;

#if defined(EMS_HOST_TEST)
// ── Estado float (só host tests) ────────────────────────────────────────────
static const etb_pid_config_t g_default_pid_config = {
    .kp_pos = 2.5f,
    .ki_pos = 0.1f,
    .kd_pos = 0.05f,
    .kp_vel = 1.8f,
    .ki_vel = 0.05f,
    .kd_vel = 0.02f,
    .ff_friction = 8.0f,
    .ff_inertia = 0.3f,
    .max_opening = 95.0f,
    .min_closing = 3.0f,
    .ramp_rate = 300.0f,
    .deadband_center = 5.0f,
    .deadband_width = 2.0f
};

static etb_system_config_t g_config = {};
static etb_control_data_t g_data = {};
static etb_driver_data_t g_driver_data = {};

static float interpolate_response(const uint16_t* map, float pedal) {
    if (pedal < 0.0f) pedal = 0.0f;
    if (pedal > 100.0f) pedal = 100.0f;
    float index = (pedal / 100.0f) * 9.0f;
    int idx_low = (int)index;
    int idx_high = idx_low + 1;
    if (idx_high >= 10) {
        return map[9] / 10.0f;
    }
    float frac = index - (float)idx_low;
    return (map[idx_low] + (map[idx_high] - map[idx_low]) * frac) / 10.0f;
}

static float apply_ramp_limit(float current, float target, float max_rate, float dt_ms) {
    float max_change = (max_rate * dt_ms) / 1000.0f;
    if (target > current + max_change) {
        return current + max_change;
    }
    if (target < current - max_change) {
        return current - max_change;
    }
    return target;
}

static float calculate_pid(float error, float* integral, float* prev_error,
                           float* filtered_deriv,
                           float kp, float ki, float kd, float dt_ms,
                           float max_integral, float output_limit) {
    float p = kp * error;
    const float candidate_integral = *integral + error * (dt_ms / 1000.0f);
    const float candidate_out = p + ki * candidate_integral;
    const bool saturated_open  = (candidate_out >  output_limit) && (error > 0.0f);
    const bool saturated_close = (candidate_out < -output_limit) && (error < 0.0f);
    if (!saturated_open && !saturated_close) {
        *integral = candidate_integral;
    }
    if (*integral >  max_integral) { *integral =  max_integral; }
    if (*integral < -max_integral) { *integral = -max_integral; }
    float i = ki * (*integral);
    float deriv = (error - (*prev_error)) / (dt_ms / 1000.0f);
    *prev_error = error;
    *filtered_deriv = (*filtered_deriv) * 0.8f + deriv * 0.2f;
    float d = kd * (*filtered_deriv);
    return p + i + d;
}
#endif  // EMS_HOST_TEST

bool etb_control_init(void) {
    if (!etb_driver_init()) {
        return false;
    }
    g_drive_mode = ETB_MODE_NORMAL;
    g_initialized = true;
    g_limp_mode = false;

#if defined(EMS_HOST_TEST)
    memset(&g_config, 0, sizeof(g_config));
    memset(&g_data, 0, sizeof(g_data));
    g_config.current_mode = ETB_MODE_NORMAL;
    for (int i = 0; i < ETB_MODE_COUNT; i++) {
        memcpy(&g_config.pid_configs[i], &g_default_pid_config, sizeof(etb_pid_config_t));
    }
    g_config.idle_rpm_target  = static_cast<float>(ems::engine::etb_idle_rpm_target);
    g_config.idle_min_opening = static_cast<float>(ems::engine::etb_idle_min_opening_x10) / 10.0f;
    g_config.idle_max_opening = static_cast<float>(ems::engine::etb_idle_max_opening_x10) / 10.0f;
    g_config.idle_spark_advance = 15.0f;
    g_config.idle_spark_retard = 10.0f;
    g_config.rpm_cutoff = 7000.0f;
    g_config.tps_rate_limit = 500.0f;
    g_config.limp_opening = 5.0f;
    g_data.homing_count = 1;
    etb_driver_read_sensors(&g_driver_data);
#endif
    return true;
}

void etb_set_drive_mode(etb_drive_mode_t mode) {
    if (mode >= ETB_MODE_COUNT) return;
    g_drive_mode = mode;
#if defined(EMS_HOST_TEST)
    g_config.current_mode = mode;
#endif
}

etb_drive_mode_t etb_get_drive_mode(void) {
    return g_drive_mode;
}

void etb_apply_idle_calibration(void) {
#if defined(EMS_HOST_TEST)
    g_config.idle_rpm_target  = static_cast<float>(ems::engine::etb_idle_rpm_target);
    g_config.idle_min_opening = static_cast<float>(ems::engine::etb_idle_min_opening_x10) / 10.0f;
    g_config.idle_max_opening = static_cast<float>(ems::engine::etb_idle_max_opening_x10) / 10.0f;
#endif
    // Produção: torque_manager / idle spark leem globals de calibration directamente.
}

bool etb_is_ready(void) {
    return g_initialized && !g_limp_mode &&
           (etb_driver_get_state() == ETB_DRV_STATE_READY);
}

void etb_enter_limp_mode(void) {
    g_limp_mode = true;
    ems::engine::limp_gating_report_etb_problem();
#if defined(EMS_HOST_TEST)
    g_data.fault_count++;
    g_data.throttle_target = g_config.limp_opening;
#endif
}

#if defined(EMS_HOST_TEST)

void etb_control_loop(float pedal, float rpm, float dt) {
    if (!g_initialized) return;

    g_data.last_update_ms += dt;

    etb_driver_fault_t fault = etb_driver_read_sensors(&g_driver_data);
    if (fault != ETB_DRV_OK) {
        g_data.fault_count++;
        etb_enter_limp_mode();
        return;
    }

    g_data.throttle_actual = g_driver_data.tps_validated;
    g_data.pedal_percent = pedal;

    if (rpm > g_config.rpm_cutoff) {
        g_data.throttle_target = g_config.idle_min_opening;
    } else if (g_limp_mode) {
        g_data.throttle_target = g_config.limp_opening;
    } else {
        float raw_target = interpolate_response(
            ems::engine::etb_pedal_map[g_config.current_mode],
            pedal
        );

        if (g_data.idle_active && rpm < (g_config.idle_rpm_target + 200.0f)) {
            float idle_error = g_config.idle_rpm_target - rpm;
            float idle_add = idle_error * 0.05f;
            if (idle_add < 0.0f) idle_add = 0.0f;
            if (idle_add > (g_config.idle_max_opening - g_config.idle_min_opening)) {
                idle_add = g_config.idle_max_opening - g_config.idle_min_opening;
            }
            g_data.idle_offset = g_config.idle_min_opening + idle_add;
            if (raw_target < g_data.idle_offset) {
                raw_target = g_data.idle_offset;
            }
            if (fabsf(idle_error) < 100.0f) {
                g_data.idle_spark_trim = (int16_t)(idle_error * 0.5f);
                if (g_data.idle_spark_trim > 100) g_data.idle_spark_trim = 100;
                if (g_data.idle_spark_trim < -100) g_data.idle_spark_trim = -100;
            } else {
                g_data.idle_spark_trim = 0;
            }
        } else {
            g_data.idle_offset = 0.0f;
            g_data.idle_spark_trim = 0;
            g_data.idle_active = false;
        }

        g_data.throttle_target = apply_ramp_limit(
            g_data.throttle_target,
            raw_target,
            g_config.tps_rate_limit,
            dt
        );
    }

    g_data.pos_error = g_data.throttle_target - g_data.throttle_actual;

    float pos_output = calculate_pid(
        g_data.pos_error,
        &g_data.pos_integral,
        &g_data.pos_derivative,
        &g_data.pos_filtered_deriv,
        g_config.pid_configs[g_config.current_mode].kp_pos,
        g_config.pid_configs[g_config.current_mode].ki_pos,
        g_config.pid_configs[g_config.current_mode].kd_pos,
        dt,
        50.0f,
        100.0f
    );

    float ff_output = g_config.pid_configs[g_config.current_mode].ff_friction;
    if (g_data.pos_error > 2.0f) {
        ff_output += g_config.pid_configs[g_config.current_mode].ff_inertia *
                     (g_data.throttle_target - g_data.throttle_actual);
    }

    g_data.throttle_command = pos_output + ff_output;
    int16_t motor_pwm = (int16_t)(g_data.throttle_command * 10.23f);
    if (motor_pwm > 1023) motor_pwm = 1023;
    if (motor_pwm < -1023) motor_pwm = -1023;
    etb_driver_set_motor_pwm(motor_pwm);
}

void etb_set_idle_control(bool active, float target_rpm) {
    g_data.idle_active = active;
    if (active) {
        g_config.idle_rpm_target = target_rpm;
    }
}

float etb_get_throttle_position(void) {
    return g_data.throttle_actual;
}

namespace ems::engine {
int16_t etb_get_idle_spark_trim() noexcept {
    return static_cast<int16_t>(g_data.idle_spark_trim);
}
}  // namespace ems::engine

// C linkage symbol used by older test paths (header declares extern "C")
extern "C" int16_t etb_get_idle_spark_trim(void) {
    return static_cast<int16_t>(g_data.idle_spark_trim);
}

#endif  // EMS_HOST_TEST

// ── PID inteiro de produção ─────────────────────────────────────────────────
namespace ems::engine {

static int32_t g_integrator_x10 = 0;
static int16_t g_prev_error_x10 = 0;
// Watchdog de seguimento: borboleta que não segue o alvo (travada, ponte com polaridade
// invertida, realimentação trocada) deixava o PID saturado em ±100% para sempre.
constexpr int16_t  kFollowErrX10 = 200;    // erro >= 20%
constexpr int16_t  kFollowSatX10 = 900;    // com saída >= 90%
constexpr uint16_t kFollowTripMs = 1500u;  // por 1,5 s seguido (partida com tensão caída não deve disparar)
static uint16_t g_follow_ms = 0u;
static bool     g_prev_error_valid = false;   // 1ª amostra após reset não gera "chute" de derivativo
static bool     g_follow_fault = false;
static uint16_t g_follow_tps_at_trip = 0u;
static bool     g_follow_tps_valid = false;
static bool     g_follow_rearm_used = false;
static bool     g_follow_stop_timing = false;
static uint32_t g_follow_stopped_since_ms = 0u;
static bool     g_follow_awaiting_good = false;
static uint16_t g_follow_good_ms = 0u;
static bool     g_follow_dtc_clear_ready = false;
constexpr uint32_t kFollowRearmStopMs = 2000u;

void etb_control_reset() noexcept {
    g_integrator_x10 = 0;
    g_prev_error_x10 = 0;
    g_prev_error_valid = false;
    g_follow_ms = 0u;
}

bool etb_control_follow_fault() noexcept { return g_follow_fault; }

void etb_control_follow_fault_clear() noexcept {
    g_follow_fault = false;
    g_follow_ms = 0u;
    g_follow_tps_valid = false;
    g_follow_rearm_used = false;
    g_follow_stop_timing = false;
    g_follow_stopped_since_ms = 0u;
    g_follow_awaiting_good = false;
    g_follow_good_ms = 0u;
    g_follow_dtc_clear_ready = false;
}

EtbFollowAction etb_follow_recovery_tick(uint32_t now_ms, uint16_t rpm_x10,
                                         uint16_t app_pct_x10,
                                         uint16_t measured_pct_x10) noexcept {
    if (g_follow_dtc_clear_ready) {
        g_follow_dtc_clear_ready = false;
        return EtbFollowAction::ClearDtc;
    }
    if (!g_follow_fault || g_follow_rearm_used) {
        g_follow_stop_timing = false;
        return EtbFollowAction::Hold;
    }
    if (rpm_x10 != 0u) {
        g_follow_stop_timing = false;
        return EtbFollowAction::Hold;
    }
    if (!g_follow_stop_timing) {
        g_follow_stop_timing = true;
        g_follow_stopped_since_ms = now_ms;
        return EtbFollowAction::Hold;
    }
    if ((now_ms - g_follow_stopped_since_ms) < kFollowRearmStopMs) {
        return EtbFollowAction::Hold;
    }
    if (app_pct_x10 >= etb_idle_open_pct_x10) { return EtbFollowAction::Hold; }
    if (!g_follow_tps_valid || measured_pct_x10 == g_follow_tps_at_trip) {
        return EtbFollowAction::Hold;
    }
    g_follow_fault = false;
    g_follow_ms = 0u;
    g_follow_rearm_used = true;
    g_follow_awaiting_good = true;
    g_follow_good_ms = 0u;
    g_follow_stop_timing = false;
    limp_gating_clear_etb_problem();
    return EtbFollowAction::Rearm;
}

EtbControlState etb_control_update(uint16_t target_pct_x10,
                                   uint16_t measured_pct_x10,
                                   bool     enable_request,
                                   uint16_t period_ms) noexcept
{
    EtbControlState out{};

    if (!enable_request || etb_cal_valid == 0u || g_follow_fault) {
        etb_control_reset();
        return out;
    }

    out.active = true;

    const int16_t error_x10 = clamp_i16(
        static_cast<int32_t>(target_pct_x10) - static_cast<int32_t>(measured_pct_x10),
        -1000, 1000);

    const int32_t kp_x10 = static_cast<int32_t>(etb_kp_x10);
    const int32_t ki_x10 = static_cast<int32_t>(etb_ki_x10);
    const int32_t kd_x10 = static_cast<int32_t>(etb_kd_x10);

    int32_t output_x10 = (kp_x10 * static_cast<int32_t>(error_x10)) / 10;

    // Anti-windup: saturação em P+I (não só P).
    if (period_ms > 0u) {
        const int32_t ki_inc = (ki_x10 * static_cast<int32_t>(error_x10)
                                * static_cast<int32_t>(period_ms)) / 1000;
        const int32_t raw_i = g_integrator_x10 + ki_inc;
        const int32_t candidate_i = (raw_i > 2000) ? 2000 : (raw_i < -2000 ? -2000 : raw_i);
        const int32_t candidate_out = output_x10 + candidate_i;
        const bool saturating = (candidate_out >  1000 && error_x10 > 0) ||
                                (candidate_out < -1000 && error_x10 < 0);
        if (!saturating) {
            g_integrator_x10 = static_cast<int16_t>(candidate_i);
        }
    }
    g_integrator_x10 = clamp_i16(static_cast<int32_t>(g_integrator_x10), -2000, 2000);
    output_x10 += g_integrator_x10;

    if (!g_prev_error_valid) {
        g_prev_error_x10 = error_x10;
        g_prev_error_valid = true;
    }
    if (period_ms > 0u) {
        const int32_t deriv = ((static_cast<int32_t>(error_x10) - g_prev_error_x10)
                               * 1000) / static_cast<int32_t>(period_ms);
        // int64: kd × deriv passava de int32 com ganho alto e erro em degrau.
        const int64_t d_term = (static_cast<int64_t>(kd_x10) * deriv) / 10;
        output_x10 = static_cast<int32_t>(
            std::max<int64_t>(-100000, std::min<int64_t>(100000, static_cast<int64_t>(output_x10) + d_term)));
    }
    g_prev_error_x10 = error_x10;

    out.output_pct_x10     = clamp_i16(output_x10, -1000, 1000);
    out.position_error_x10 = error_x10;

    const int16_t abs_err = static_cast<int16_t>(error_x10 < 0 ? -error_x10 : error_x10);
    const int16_t abs_out = static_cast<int16_t>(out.output_pct_x10 < 0 ? -out.output_pct_x10
                                                                          : out.output_pct_x10);
    if (abs_err >= kFollowErrX10 && abs_out >= kFollowSatX10) {
        g_follow_ms = static_cast<uint16_t>(g_follow_ms + period_ms);
        if (g_follow_ms >= kFollowTripMs) {
            if (!g_follow_fault) {
                g_follow_tps_at_trip = measured_pct_x10;
                g_follow_tps_valid = true;
                g_follow_awaiting_good = false;
                g_follow_good_ms = 0u;
                g_follow_dtc_clear_ready = false;
            }
            g_follow_fault = true;
            out.active = false;
            out.output_pct_x10 = 0;
        }
    } else {
        g_follow_ms = 0u;
    }
    if (g_follow_awaiting_good && !g_follow_fault) {
        if (abs_err < kFollowErrX10) {
            const uint32_t sum = static_cast<uint32_t>(g_follow_good_ms) + period_ms;
            g_follow_good_ms = static_cast<uint16_t>(sum > 60000u ? 60000u : sum);
            if (g_follow_good_ms >= kFollowTripMs) {
                g_follow_dtc_clear_ready = true;
                g_follow_awaiting_good = false;
                g_follow_rearm_used = false;
            }
        } else {
            g_follow_good_ms = 0u;
        }
    }
    return out;
}

int32_t etb_control_get_integrator() noexcept {
    return g_integrator_x10;
}

bool etb_rev_cut_request(bool rev_limit_active, bool limp_rpm_cut_active) noexcept {
    return rev_limit_active || limp_rpm_cut_active;
}

void etb_apply_output(bool enable_request, bool active, bool initialized,
                      int16_t output_pct_x10) noexcept {
    if (!enable_request || !active || !initialized) {
        ::etb_driver_shutdown();
        return;
    }
    // output_pct_x10 ∈ [-1000, 1000] → driver PWM ∈ [-1023, 1023]
    int32_t pwm = (static_cast<int32_t>(output_pct_x10) * 1023) / 1000;
    if (pwm > 1023) { pwm = 1023; }
    if (pwm < -1023) { pwm = -1023; }
    if (!::etb_driver_set_motor_pwm(static_cast<int16_t>(pwm))) {
        ::etb_driver_shutdown();
    }
}

}  // namespace ems::engine
