#include lambda_pid.h"

void lambda_pid_init(lambda_pid_t *pid, float kp, float ki, float kd, float output_min, float output_max) {
    if (!pid) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integrator_min = output_min;
    pid->integrator_max = output_max;
}

float lambda_pid_update(lambda_pid_t *pid, float target, float measured, float dt_s) {
    if (!pid || dt_s <= 0.0f) {
        return 0.0f;
    }

    float error = target - measured;
    float p = pid->kp * error;

    pid->integrator += pid->ki * error * dt_s;
    if (pid->integrator > pid->integrator_max) {
        pid->integrator = pid->integrator_max;
    } else if (pid->integrator < pid->integrator_min) {
        pid->integrator = pid->integrator_min;
    }

    float d = pid->kd * (error - pid->prev_error) / dt_s;
    pid->prev_error = error;

    float output = p + pid->integrator + d;
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}
