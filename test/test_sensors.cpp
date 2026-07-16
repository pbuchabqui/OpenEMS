#include "test/harness.h"
#include "test/fixtures.h"
#include "test/ui_helpers.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "engine/etb_control.h"
#include "hal/etb_driver.h"
#include "engine/torque_manager.h"
#include "engine/calibration.h"
#include "app/can_rx_map.h"
#include "hal/adc.h"
#include "hal/system.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/auxiliaries.h"
#include "engine/knock.h"
#include "engine/table3d.h"
#include "engine/ecu_sched.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "engine/output_test.h"
#include "engine/engine_config.h"
#include "hal/timer.h"
#include "hal/flash.h"
#include "app/ui_protocol.h"
#include "app/status_bits.h"
#include "hal/crc32.h"

namespace ems::engine {
    int16_t etb_get_idle_spark_trim() noexcept;
}

extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

using namespace ems::drv;
using namespace ems::engine;
using namespace ems::app;
using namespace ems::hal;

void test_sensors_validate_range(void) {
    section("sensors: validate_sensor_range");
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 100u),   "CLT=100 valid");
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 3800u),  "CLT=3800 valid");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 99u),   "CLT=99 invalid");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 3801u), "CLT=3801 invalid");
    CHECK_TRUE(validate_sensor_range(SensorId::MAP, 1000u),  "MAP=1000 valid");
    CHECK_FALSE(validate_sensor_range(SensorId::MAP, 10u),   "MAP=10 invalid");
}

void test_sensors_validate_values(void) {
    section("sensors: validate_sensor_values");
    SensorData d{};
    d.map_bar_x1000 = 1000u; d.clt_degc_x10 = 850; d.iat_degc_x10 = 250;
    d.tps_pct_x10 = 500u; d.vbatt_mv = 12000u;
    d.fuel_press_bar_x1000 = 3000u; d.oil_press_bar_x1000 = 5000u;
    CHECK_TRUE(validate_sensor_values(d), "all valid → true");
    SensorData bad = d; bad.map_bar_x1000 = 50u;
    CHECK_FALSE(validate_sensor_values(bad), "MAP < 0.10 bar → false");
    bad = d; bad.clt_degc_x10 = 1600;
    CHECK_FALSE(validate_sensor_values(bad), "CLT > 150°C → false");
    bad = d; bad.vbatt_mv = 5000u;
    CHECK_FALSE(validate_sensor_values(bad), "vbatt < 6V → false");
    bad = d; bad.tps_pct_x10 = 1001u;
    CHECK_FALSE(validate_sensor_values(bad), "TPS > 100% → false");
}

void test_sensors_health_status(void) {
    section("sensors: get_sensor_health_status");
    sensor_setup(); sensors_init();
    CHECK_EQ(get_sensor_health_status(), 0u, "health=0 after init with valid ADC");
}

void test_sensors_calibration(void) {
    section("sensors: set_tps_cal / set_app_cal / set_plausibility / set_etb_tps_cal");
    sensor_setup(); sensors_init();
    sensors_set_tps_cal(400u, 3800u);
    sensors_set_app_cal(400u, 3800u, 400u, 3800u);
    sensors_set_plausibility(100u, 100u);
    sensors_set_etb_tps_cal(400u, 3800u, 400u, 3800u);
    CHECK_TRUE(true, "calibration setters complete without crash");
}

void test_sensors_tick_100ms_clt_iat(void) {
    section("sensors: sensors_tick_100ms → CLT/IAT via lut128");
    sensor_setup(); sensors_init();
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT, 2000u);
    for (int i = 0; i < 8; ++i) { sensors_test_tick_100ms(); }
    const SensorData sd = sensors_get();
    const int16_t expected = static_cast<int16_t>(-400 + (1900 * 62) / 127);
    CHECK_EQ(sd.clt_degc_x10, expected, "CLT lut128(2000) matches formula");
    CHECK_EQ(sd.iat_degc_x10, expected, "IAT lut128(2000) matches formula");
}

void test_sensors_maf_freq_capture(void) {
    section("sensors: sensors_maf_freq_capture_isr");
    sensor_setup(); sensors_init();
    sensors_maf_freq_capture_isr(5000u);
    sensors_maf_freq_capture_isr(10000u);
    sensors_maf_freq_capture_isr(0u);
    CHECK_TRUE(true, "maf_freq_capture_isr handles various periods");
}

// ═══════════════════════════════════════════════════════════════════════════
// FUEL CALC
// ═══════════════════════════════════════════════════════════════════════════

void test_sensors_on_tooth(void) {
    section("sensors: sensors_on_tooth");
    sensor_setup(); sensors_init();
    ems::drv::CkpSnapshot snap{};
    snap.tooth_period_ns = 160000u;  // 10000 ticks × 16 ns = 160000 ns
    snap.rpm_x10 = 62500u;
    sensors_on_tooth(snap);  // triggers ADC sample accumulation (fast channels)
    CHECK_TRUE(true, "sensors_on_tooth completes without crash");
}

void test_sensors_tick_50ms(void) {
    section("sensors: sensors_tick_50ms");
    sensor_setup(); sensors_init();
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::FUEL_PRESS, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::OIL_PRESS,  2000u);
    sensors_tick_50ms();
    CHECK_TRUE(true, "sensors_tick_50ms completes without crash");
    // After tick: fuel/oil pressure should be set (raw=2000 → some bar value)
    for (int i = 0; i < 4; ++i) { sensors_tick_50ms(); }  // fill staging buffers
    sensors_test_tick_100ms();  // commits staging → committed (double-buffer swap)
    const ems::drv::SensorData sd = sensors_get();
    CHECK_TRUE(sd.fuel_press_bar_x1000 > 0u, "fuel_press > 0 after tick_50ms with raw=2000");
}

void test_sensors_set_range(void) {
    section("sensors: sensors_set_range");
    sensor_setup(); sensors_init();
    // Widen CLT range so that raw=50 is accepted
    sensors_set_range(SensorId::CLT, {50u, 4000u});
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 50u), "CLT raw=50 valid after range change");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 49u), "CLT raw=49 still invalid");
}

void test_sensors_etb_harness_present(void) {
    section("sensors: sensors_set_etb_harness_present");
    sensor_setup(); sensors_init();
    sensors_set_etb_harness_present(true);
    // When harness present, tick_100ms uses fixed vbatt=12000 instead of ADC.
    // Just verify no crash.
    sensors_test_tick_100ms();
    CHECK_TRUE(true, "tick_100ms with harness_present=true: no crash");
    sensors_set_etb_harness_present(false);  // restore
}

void test_sensors_table_entry_setters(void) {
    section("sensors: sensors_test_set_clt_table_entry / set_iat_table_entry");
    sensor_setup(); sensors_init();
    // Manually set CLT table entry at index 62 (ADC=2000>>5=62) to 200 (20.0°C)
    sensors_test_set_clt_table_entry(62u, 200);
    sensors_test_set_iat_table_entry(62u, 150);
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT, 2000u);
    for (int i = 0; i < 8; ++i) { sensors_test_tick_100ms(); }
    const ems::drv::SensorData sd = sensors_get();
    CHECK_EQ(sd.clt_degc_x10, 200, "CLT table entry 62 = 200 (20.0°C)");
    CHECK_EQ(sd.iat_degc_x10, 150, "IAT table entry 62 = 150 (15.0°C)");
}

// ═══════════════════════════════════════════════════════════════════════════
// KNOCK — SEGUNDA FASE
// ═══════════════════════════════════════════════════════════════════════════

