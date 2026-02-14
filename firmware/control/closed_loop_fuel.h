#ifndef LAMBDA_PID_H
#define LAMBDA_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float output_min;
    float output_max;
    float integrator_min;
    float integrator_max;
} lambda_pid_t;

void lambda_pid_init(lambda_pid_t *pid, float kp, float ki, float kd, float output_min, float output_max);

float lambda_pid_update(lambda_pid_t *pid, float target, float measured, float dt_s);

#ifdef __cplusplus
}
#endif

#endif // LAMBDA_PID_H
