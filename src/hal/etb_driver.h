#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ADC range limits for TPS sensors (12-bit)
#define ETB_TPS_ADC_MIN       200u
#define ETB_TPS_ADC_MAX      3900u
#define ETB_TPS_NORMAL_MIN    400u
#define ETB_TPS_NORMAL_MAX   3700u

typedef enum {
    ETB_DRV_OK = 0,
    ETB_DRV_FAULT_TPS1_OPEN,
    ETB_DRV_FAULT_TPS1_SHORT,
    ETB_DRV_FAULT_TPS2_OPEN,
    ETB_DRV_FAULT_TPS2_SHORT,
    ETB_DRV_FAULT_TPS_MISMATCH,
    ETB_DRV_FAULT_OVERCURRENT,
    ETB_DRV_FAULT_NOT_INITIALIZED
} etb_driver_fault_t;

typedef enum {
    ETB_DRV_STATE_OFF = 0,
    ETB_DRV_STATE_INIT,
    ETB_DRV_STATE_READY,
    ETB_DRV_STATE_FAULT
} etb_driver_state_t;

typedef struct {
    uint16_t tps1_raw;
    uint16_t tps2_raw;
    float    tps1_percent;
    float    tps2_percent;
    float    tps_validated;
    int16_t  motor_pwm;
    etb_driver_state_t state;
    etb_driver_fault_t fault;
    uint32_t fault_count;
    uint32_t last_update_us;
} etb_driver_data_t;

// Public API
bool              etb_driver_init(void);
etb_driver_fault_t etb_driver_read_sensors(etb_driver_data_t* data);
bool              etb_driver_set_motor_pwm(int16_t pwm);
void              etb_driver_shutdown(void);
void              etb_driver_clear_fault(void);
etb_driver_state_t etb_driver_get_state(void);
float             etb_driver_adc_to_percent(uint16_t adc_raw);

// Test hook
void etb_driver_test_reset(void);

#ifdef __cplusplus
}
#endif
