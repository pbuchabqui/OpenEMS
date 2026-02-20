/**
 * @file hal_pwm.h
 * @brief PWM HAL — inline duty control for slow actuators
 *
 * Used for: VVT solenoids, IAC valve, boost wastegate solenoid.
 * These operate at 10–200 Hz — not time-critical.
 * Implemented as normal ledc driver calls (not bare-metal register writes)
 * because the latency here is irrelevant (millisecond-scale actuators).
 *
 * Core 1 use only.
 */

#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>
#include "driver/ledc.h"
#include "hal_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

// LEDC channel assignments
#define HAL_PWM_CH_VVT_INTAKE   LEDC_CHANNEL_0
#define HAL_PWM_CH_VVT_EXHAUST  LEDC_CHANNEL_1
#define HAL_PWM_CH_IAC          LEDC_CHANNEL_2
#define HAL_PWM_CH_BOOST        LEDC_CHANNEL_3
#define HAL_PWM_CH_AUX1         LEDC_CHANNEL_4
#define HAL_PWM_CH_AUX2         LEDC_CHANNEL_5

#define HAL_PWM_FREQ_VVT_HZ     100     // VVT solenoid
#define HAL_PWM_FREQ_IAC_HZ     100     // IAC valve
#define HAL_PWM_FREQ_BOOST_HZ   50      // Boost solenoid
#define HAL_PWM_RESOLUTION      LEDC_TIMER_10_BIT  // 0–1023

/**
 * @brief Initialize all PWM channels. Call once at startup (Core 1).
 */
esp_err_t hal_pwm_init(void);

/**
 * @brief Set PWM duty cycle (0–1000 = 0–100.0%)
 */
__attribute__((always_inline)) static inline void HAL_PWM_SetDuty(ledc_channel_t ch, uint16_t duty_per_mille) {
    uint32_t duty = ((uint32_t)duty_per_mille * 1023U) / 1000U;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* Named convenience wrappers */
#define HAL_VVT_Intake_SetDuty(d)   HAL_PWM_SetDuty(HAL_PWM_CH_VVT_INTAKE,  (d))
#define HAL_VVT_Exhaust_SetDuty(d)  HAL_PWM_SetDuty(HAL_PWM_CH_VVT_EXHAUST, (d))
#define HAL_IAC_SetDuty(d)          HAL_PWM_SetDuty(HAL_PWM_CH_IAC,          (d))
#define HAL_Boost_SetDuty(d)        HAL_PWM_SetDuty(HAL_PWM_CH_BOOST,        (d))

#ifdef __cplusplus
}
#endif

#endif // HAL_PWM_H
