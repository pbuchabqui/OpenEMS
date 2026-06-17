/**
 * @file etb_control.h
 * @brief Controle de Borboleta Eletrônica (ETB) com PID em Cascata
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "hal/etb_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ETB_MODE_ECO = 0,
    ETB_MODE_NORMAL,
    ETB_MODE_SPORT,
    ETB_MODE_RAIN,
    ETB_MODE_COUNT
} etb_drive_mode_t;

typedef struct {
    float kp_pos, ki_pos, kd_pos;
    float kp_vel, ki_vel, kd_vel;
    float ff_friction, ff_inertia;
    float max_opening, min_closing, ramp_rate;
    float deadband_center, deadband_width;
} etb_pid_config_t;

typedef struct {
    float pedal_percent, throttle_target, throttle_actual, throttle_command;
    float pos_error, pos_integral, pos_derivative, pos_filtered_deriv;
    float vel_error, vel_integral;
    float idle_offset, idle_spark_trim;
    bool  idle_active;
    uint32_t homing_count, fault_count;
    float last_update_ms;
} etb_control_data_t;

typedef struct {
    etb_drive_mode_t current_mode;
    etb_pid_config_t pid_configs[ETB_MODE_COUNT];
    float idle_rpm_target, idle_min_opening, idle_max_opening;
    float idle_spark_advance, idle_spark_retard;
    float rpm_cutoff, tps_rate_limit, limp_opening;
} etb_system_config_t;

bool             etb_control_init(void);
void             etb_control_loop(float pedal, float rpm, float dt);
void             etb_set_drive_mode(etb_drive_mode_t mode);
etb_drive_mode_t etb_get_drive_mode(void);
void             etb_set_idle_control(bool active, float target_rpm);
int16_t          etb_get_idle_spark_trim(void);
float            etb_get_throttle_position(void);
bool             etb_is_ready(void);
void             etb_enter_limp_mode(void);

#ifdef __cplusplus
}

namespace ems::engine {

struct EtbControlState {
    bool    active;
    int16_t output_pct_x10;
    int16_t position_error_x10;
};

void            etb_control_reset() noexcept;
EtbControlState etb_control_update(uint16_t target_pct_x10,
                                   uint16_t measured_pct_x10,
                                   bool     enable_request,
                                   uint16_t period_ms) noexcept;
int32_t         etb_control_test_get_integrator() noexcept;

}  // namespace ems::engine

#endif  // __cplusplus
