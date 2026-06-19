#include "engine/ewg_control.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"
#include "hal/ewg_driver.h"

namespace {

int32_t g_integrator_x10 = 0;
int16_t g_prev_error_x10 = 0;
int16_t g_output_x10 = 0;

}  // namespace

namespace ems::engine {

void ewg_control_init() noexcept {
    ems::hal::ewg_driver_init();
    ewg_control_reset();
}

void ewg_control_reset() noexcept {
    g_integrator_x10 = 0;
    g_prev_error_x10 = 0;
    g_output_x10 = 0;
    ems::hal::ewg_driver_shutdown();
}

int16_t ewg_control_update(uint16_t target_pct_x10, uint16_t measured_pct_x10) noexcept {
    const int16_t error_x10 = static_cast<int16_t>(
        static_cast<int32_t>(target_pct_x10) - static_cast<int32_t>(measured_pct_x10));

    const int32_t p = (static_cast<int32_t>(ewg_kp_x10) * error_x10) / 10;

    const int32_t candidate_i = g_integrator_x10
        + (static_cast<int32_t>(ewg_ki_x10) * error_x10) / 10;
    const int32_t candidate_out = p + candidate_i;
    const bool saturating = (candidate_out > 1000 && error_x10 > 0)
                         || (candidate_out < -1000 && error_x10 < 0);
    if (!saturating) {
        g_integrator_x10 = clamp_i16(static_cast<int16_t>(candidate_i),
                                     static_cast<int16_t>(-1000), static_cast<int16_t>(1000));
    }

    const int16_t deriv_x10 = static_cast<int16_t>(error_x10 - g_prev_error_x10);
    g_prev_error_x10 = error_x10;

    int32_t output = p + g_integrator_x10
        + (static_cast<int32_t>(ewg_kd_x10) * deriv_x10) / 10;

    g_output_x10 = clamp_i16(static_cast<int16_t>(output),
                             static_cast<int16_t>(-1000), static_cast<int16_t>(1000));

    ems::hal::ewg_driver_set_motor_pwm(g_output_x10);
    return g_output_x10;
}

uint16_t ewg_read_position_pct_x10() noexcept {
    const uint16_t raw = ems::hal::ewg_driver_read_position_raw();
    if (ewg_pos_max_raw <= ewg_pos_min_raw) { return 0u; }
    if (raw <= ewg_pos_min_raw) { return 0u; }
    if (raw >= ewg_pos_max_raw) { return 1000u; }
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(raw - ewg_pos_min_raw) * 1000u)
        / (ewg_pos_max_raw - ewg_pos_min_raw));
}

}  // namespace ems::engine
