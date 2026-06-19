#include "etb_driver.h"
#ifndef EMS_HOST_TEST
#include "stm32h562/regs.h"
#else
// Host test: mock GPIO registers as writable variables instead of
// raw memory-mapped register dereferences (which would segfault on x86).
static volatile uint32_t ems_etb_gpioa_moder;
static volatile uint32_t ems_etb_gpiob_moder;
static volatile uint32_t ems_etb_gpioa_bsrr;
static volatile uint32_t ems_etb_gpiob_bsrr;
#define GPIOA_MODER ems_etb_gpioa_moder
#define GPIOB_MODER ems_etb_gpiob_moder
#define GPIOA_BSRR  ems_etb_gpioa_bsrr
#define GPIOB_BSRR  ems_etb_gpiob_bsrr
#endif
#include "timer.h"
#include "adc.h"

#include <cstring>

using ems::hal::tim15_etb_pwm_init;
using ems::hal::tim15_etb_set_duty_x10;
using ems::hal::adc_primary_read;
using ems::hal::AdcPrimaryChannel;

namespace {

etb_driver_data_t  g_etb_data  = {};
etb_driver_state_t g_state     = ETB_DRV_STATE_OFF;
etb_driver_fault_t g_fault     = ETB_DRV_OK;
uint32_t           g_fault_count = 0u;

// Direction pins: PA10 (IN1 open), PB2 (IN2 close)
constexpr uint8_t kIn1Pin = 10u;
constexpr uint8_t kIn2Pin = 2u;

}  // namespace

float etb_driver_adc_to_percent(uint16_t adc_raw) {
    if (adc_raw <= ETB_TPS_NORMAL_MIN) { return 0.0f; }
    if (adc_raw >= ETB_TPS_NORMAL_MAX) { return 100.0f; }
    return static_cast<float>(adc_raw - ETB_TPS_NORMAL_MIN)
         / static_cast<float>(ETB_TPS_NORMAL_MAX - ETB_TPS_NORMAL_MIN) * 100.0f;
}

bool etb_driver_init(void) {
    std::memset(&g_etb_data, 0, sizeof(g_etb_data));
    g_state  = ETB_DRV_STATE_INIT;
    g_fault  = ETB_DRV_OK;
    g_fault_count = 0u;

    // PA10 direction 1 (open), PB2 direction 2 (close) — push-pull outputs
    // MODER bits: 01b = general purpose output
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (kIn1Pin * 2u))) | (1u << (kIn1Pin * 2u));
    GPIOB_MODER = (GPIOB_MODER & ~(3u << (kIn2Pin * 2u))) | (1u << (kIn2Pin * 2u));

    // TIM15 PWM @ 20 kHz for ETB motor drive
    tim15_etb_pwm_init(20000u);

    etb_driver_shutdown();

    etb_driver_fault_t fault = etb_driver_read_sensors(&g_etb_data);
    if (fault != ETB_DRV_OK) {
        g_state = ETB_DRV_STATE_FAULT;
        g_fault = fault;
        ++g_fault_count;
        return false;
    }

    g_state = ETB_DRV_STATE_READY;
    return true;
}

etb_driver_fault_t etb_driver_read_sensors(etb_driver_data_t* data) {
    if (data == nullptr) { return ETB_DRV_FAULT_NOT_INITIALIZED; }
    if (g_state == ETB_DRV_STATE_FAULT) { return g_fault; }

    const uint16_t t1 = adc_primary_read(AdcPrimaryChannel::ETB_TPS1);
    const uint16_t t2 = adc_primary_read(AdcPrimaryChannel::ETB_TPS2);

    if (t1 < ETB_TPS_ADC_MIN) { return ETB_DRV_FAULT_TPS1_OPEN;  }
    if (t1 > ETB_TPS_ADC_MAX) { return ETB_DRV_FAULT_TPS1_SHORT; }
    if (t2 < ETB_TPS_ADC_MIN) { return ETB_DRV_FAULT_TPS2_OPEN;  }
    if (t2 > ETB_TPS_ADC_MAX) { return ETB_DRV_FAULT_TPS2_SHORT; }

    data->tps1_raw     = t1;
    data->tps2_raw     = t2;
    data->tps1_percent = etb_driver_adc_to_percent(t1);
    data->tps2_percent = etb_driver_adc_to_percent(t2);

    const float diff = (data->tps1_percent > data->tps2_percent)
                     ? (data->tps1_percent - data->tps2_percent)
                     : (data->tps2_percent - data->tps1_percent);
    if (diff > 12.0f) { return ETB_DRV_FAULT_TPS_MISMATCH; }

    data->tps_validated = (data->tps1_percent + data->tps2_percent) * 0.5f;
    return ETB_DRV_OK;
}

bool etb_driver_set_motor_pwm(int16_t pwm) {
    if (g_state != ETB_DRV_STATE_READY) { return false; }
    if (pwm >  1023) { pwm =  1023; }
    if (pwm < -1023) { pwm = -1023; }

    g_etb_data.motor_pwm = pwm;
    const uint16_t duty = static_cast<uint16_t>((pwm >= 0) ? pwm : -pwm);

    if (pwm > 0) {
        GPIOA_BSRR = (1u << kIn1Pin);
        GPIOB_BSRR = (1u << (kIn2Pin + 16u));
    } else if (pwm < 0) {
        GPIOA_BSRR = (1u << (kIn1Pin + 16u));
        GPIOB_BSRR = (1u << kIn2Pin);
    } else {
        GPIOA_BSRR = (1u << (kIn1Pin + 16u));
        GPIOB_BSRR = (1u << (kIn2Pin + 16u));
    }
    tim15_etb_set_duty_x10(static_cast<uint16_t>((static_cast<uint32_t>(duty) * 1000u) / 1023u));
    return true;
}

void etb_driver_shutdown(void) {
    GPIOA_BSRR = (1u << (kIn1Pin + 16u));
    GPIOB_BSRR = (1u << (kIn2Pin + 16u));
    tim15_etb_set_duty_x10(0u);
    g_etb_data.motor_pwm = 0;
}

void etb_driver_clear_fault(void) {
    if (g_state != ETB_DRV_STATE_FAULT) { return; }
    g_fault  = ETB_DRV_OK;
    g_state  = ETB_DRV_STATE_INIT;
    etb_driver_init();
}

etb_driver_state_t etb_driver_get_state(void) {
    return g_state;
}

void etb_driver_test_reset(void) {
    std::memset(&g_etb_data, 0, sizeof(g_etb_data));
    g_state      = ETB_DRV_STATE_OFF;
    g_fault      = ETB_DRV_OK;
    g_fault_count = 0u;
}
