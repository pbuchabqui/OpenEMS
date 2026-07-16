#include "test/fixtures.h"
#include "hal/etb_driver.h"
#include "hal/adc.h"
#include "drv/ckp.h"
#include "drv/sensors.h"

using namespace ems::drv;
using namespace ems::hal;

extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

void drv_set_valid_adc(void) {
    using namespace ems::hal;
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 2050u);  // TPS1
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 2050u);  // TPS2
}

// Reset driver to known state + load valid ADC.
void drv_setup(void) {
    etb_driver_test_reset();
    drv_set_valid_adc();
}

// Reset driver + init ETB control layer (also inits driver internally).
void etb_ctrl_setup(void) {
    drv_setup();
    // etb_control_init calls etb_driver_init() → reads ADC → must be valid
}

const uint32_t kNormalPeriod = 10000u;
const uint32_t kGapPeriod    = kNormalPeriod * 3u;
uint32_t g_ckp_cap = 0u;

void ckp_fire(uint32_t delta) {
    g_ckp_cap += delta;
    ems_test_tim5_ccr1 = g_ckp_cap;
    ckp_tim5_ch1_isr();
}
void ckp_feed_n_then_gap(uint32_t n, uint32_t p) {
    for (uint32_t i = 0; i < n; ++i) { ckp_fire(p); }
    ckp_fire(p * 3u);
}
void ckp_reach_full_sync(uint32_t p) {
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(55u, p);
    ckp_feed_n_then_gap(55u, p);
}

// ═══════════════════════════════════════════════════════════════════════════
// CKP DECODER / SYNC
// ═══════════════════════════════════════════════════════════════════════════

void cam_fire(uint32_t capture_value) {
    ems_test_cam_gpio_idr = (1u << 1u);  // bit 1 = rising edge
    ems_test_tim5_ccr2 = capture_value;
    ckp_tim5_ch2_isr();
}

void sensor_setup(void) {
    sensors_test_reset();
    using namespace ems::hal;
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::TPS,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP1,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP2,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1,      2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2,      2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT,        2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT,        2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::FUEL_PRESS, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::OIL_PRESS,  2000u);
}

