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

void test_aux_init_and_idle(void) {
    section("auxiliaries: init / idle_target_rpm_x10");
    auxiliaries_test_reset();
    CHECK_FALSE(auxiliaries_test_get_fan_state(),  "fan off after init");
    CHECK_FALSE(auxiliaries_test_get_pump_state(), "pump off after init");
    auxiliaries_tick_10ms();
    CHECK_TRUE(true, "tick_10ms after init completes");
    // kIdleTargetRpmX10 table: -40°C→12000, 90°C→8200, 110°C→8000
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(-400), 12000u, "idle@-40°C=1200RPM");
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(900),   8200u, "idle@90°C=820RPM");
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(1100),  8000u, "idle@110°C=800RPM");
    CHECK_TRUE(auxiliaries_idle_target_rpm_x10(-100) > auxiliaries_idle_target_rpm_x10(700),
               "idle target decreases as engine warms");
}

void test_aux_pump_prime(void) {
    section("auxiliaries: key_on → pump prime / key_off → pump stop");
    auxiliaries_test_reset();
    auxiliaries_set_key_on(true);
    auxiliaries_tick_10ms();
    CHECK_TRUE(auxiliaries_test_get_pump_state(), "pump on after key_on + tick");
    auxiliaries_set_key_on(false);
    for (int i = 0; i < 310; ++i) { auxiliaries_tick_10ms(); }
    CHECK_FALSE(auxiliaries_test_get_pump_state(), "pump off after key_off + delay");
}

void test_aux_ticks_no_crash(void) {
    section("auxiliaries: tick_10ms / tick_20ms no crash");
    auxiliaries_test_reset();
    for (int i = 0; i < 10; ++i) { auxiliaries_tick_10ms(); auxiliaries_tick_20ms(); }
    CHECK_TRUE(true, "20 ticks complete without crash");
}

// ═══════════════════════════════════════════════════════════════════════════
// KNOCK
// ═══════════════════════════════════════════════════════════════════════════

void test_knock_init_and_threshold(void) {
    section("knock: init / adc_threshold set/get");
    knock_init();
    for (uint8_t c = 0; c < 4u; ++c) { CHECK_EQ(knock_get_retard_x10(c), 0u, "retard=0 after init"); }
    CHECK_FALSE(knock_test_window_active(), "window closed after init");
    CHECK_EQ(knock_get_adc_threshold(), 2048u, "default threshold=2048");
    knock_set_adc_threshold(3000u); CHECK_EQ(knock_get_adc_threshold(), 3000u, "threshold=3000");
    knock_set_adc_threshold(100u);  CHECK_EQ(knock_get_adc_threshold(),  256u, "clamped at min=256");
    knock_set_adc_threshold(5000u); CHECK_EQ(knock_get_adc_threshold(), 4000u, "clamped at max=4000");
}

void test_knock_window(void) {
    section("knock: window open/close + cylinder masking");
    knock_init();
    knock_window_open(0u);
    CHECK_TRUE(knock_test_window_active(), "window active");
    CHECK_EQ(knock_test_window_cyl(), 0u, "cyl=0");
    knock_window_close(0u);
    CHECK_FALSE(knock_test_window_active(), "window closed");
    knock_window_open(6u); CHECK_EQ(knock_test_window_cyl(), 2u, "cyl=6 masked to 2"); knock_window_close(6u);
}

void test_knock_detection_and_recovery(void) {
    section("knock: adc_update / cycle_complete / retard / recovery / clamp / per-cyl");
    knock_init();
    knock_set_adc_threshold(2000u);
    knock_set_event_threshold(2u);

    // adc_update: counts only above threshold while window open
    knock_window_open(1u);
    knock_test_set_adc_raw(1900u); knock_test_set_adc_raw(1900u);
    CHECK_EQ(knock_test_get_knock_count(1u), 0u, "below threshold → no count");
    knock_test_set_adc_raw(2100u); knock_test_set_adc_raw(2100u);
    CHECK_EQ(knock_test_get_knock_count(1u), 2u, "above threshold → 2 counts");
    knock_window_close(1u);
    knock_test_set_adc_raw(3000u);
    CHECK_EQ(knock_test_get_knock_count(1u), 2u, "window closed → count frozen");

    // cycle_complete: retard on knock (count=2 > threshold=2? no, need >2 → need ≥3)
    // Reopen with 3 events
    knock_window_open(0u);
    knock_test_set_adc_raw(2500u); knock_test_set_adc_raw(2500u); knock_test_set_adc_raw(2500u);
    knock_window_close(0u);
    const uint16_t r0 = knock_get_retard_x10(0u);
    knock_cycle_complete(0u);
    CHECK_TRUE(knock_get_retard_x10(0u) > r0, "retard increases after knock");
    CHECK_EQ(knock_get_retard_x10(0u) - r0, 20u, "retard += 2.0° (20 x10)");

    // per-cylinder independence
    CHECK_EQ(knock_get_retard_x10(1u), 0u, "cyl 1 unaffected by cyl 0 knock");
    CHECK_EQ(knock_get_retard_x10(2u), 0u, "cyl 2 unaffected");

    // max clamp: force many knock events
    knock_set_event_threshold(0u);
    for (int i = 0; i < 20; ++i) {
        knock_window_open(3u); knock_test_set_adc_raw(2500u); knock_window_close(3u); knock_cycle_complete(3u);
    }
    CHECK_TRUE(knock_get_retard_x10(3u) <= 100u, "retard clamped at 10.0° (100 x10)");

    // recovery: 11 clean cycles → retard decreases
    knock_set_event_threshold(2u);
    const uint16_t peak = knock_get_retard_x10(0u);
    for (int i = 0; i < 11; ++i) {
        knock_window_open(0u); knock_test_set_adc_raw(500u); knock_window_close(0u); knock_cycle_complete(0u);
    }
    CHECK_TRUE(knock_get_retard_x10(0u) < peak, "retard decreases after clean cycles");
}

// ═══════════════════════════════════════════════════════════════════════════
// FUEL CALC — SEGUNDA FASE (funções não cobertas)
// ═══════════════════════════════════════════════════════════════════════════

void test_knock_window_cycle_end(void) {
    section("knock: knock_window_cycle_end");
    knock_init();
    knock_set_adc_threshold(2000u);
    knock_set_event_threshold(2u);

    // Open window, feed 3 above-threshold samples, call cycle_end (closes + evaluates)
    knock_window_open(0u);
    knock_test_set_adc_raw(2500u);
    knock_test_set_adc_raw(2500u);
    knock_test_set_adc_raw(2500u);  // count=3 > threshold=2
    CHECK_TRUE(knock_test_window_active(), "pre-cond: window active");

    knock_window_cycle_end();
    CHECK_FALSE(knock_test_window_active(), "window closed by cycle_end");
    CHECK_TRUE(knock_get_retard_x10(0u) > 0u, "retard applied by cycle_end");
}

void test_knock_save_to_nvm(void) {
    section("knock: knock_save_to_nvm");
    knock_init();
    // NVM is mocked in host test (flash.cpp stub). Just verify no crash.
    knock_save_to_nvm();
    CHECK_TRUE(true, "knock_save_to_nvm: no crash");
}

// ═══════════════════════════════════════════════════════════════════════════
// AUXILIARIES — SEGUNDA FASE (getters WG/VVT)
// ═══════════════════════════════════════════════════════════════════════════

void test_aux_test_getters(void) {
    section("auxiliaries: test_get_wg_duty / vvt_esc_duty / vvt_adm_duty / wg_failsafe");
    auxiliaries_test_reset();
    // After init all duties are 0 and no failsafe
    CHECK_EQ(auxiliaries_test_get_wg_duty(),       0u, "wg_duty=0 after reset");
    CHECK_EQ(auxiliaries_test_get_vvt_esc_duty(),  0u, "vvt_esc_duty=0 after reset");
    CHECK_EQ(auxiliaries_test_get_vvt_adm_duty(),  0u, "vvt_adm_duty=0 after reset");
    CHECK_FALSE(auxiliaries_test_get_wg_failsafe(), "wg_failsafe=false after reset");
    // Multiple ticks should not crash even with these getters
    for (int i = 0; i < 5; ++i) { auxiliaries_tick_20ms(); }
    CHECK_TRUE(true, "WG/VVT getters accessible after ticks");
}

// ═══════════════════════════════════════════════════════════════════════════
// TIMER HAL (stubs em host — testa que não crasham)
// ═══════════════════════════════════════════════════════════════════════════

