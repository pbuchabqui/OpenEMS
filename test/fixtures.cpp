#include "test/fixtures.h"
#include "hal/etb_driver.h"
#include "hal/adc.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/ecu_sched.h"

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
}

void ckp_feed_n_then_gap(uint32_t n, uint32_t p) {
    for (uint32_t i = 0; i < n; ++i) { ckp_fire(p); }
    ckp_fire(p * 3u);
}

void ckp_reach_full_sync(uint32_t p) {
    ckp_test_reset();
    g_ckp_cap = 0u;
    CkpSnapshot s{};
    s.state = SyncState::FULL_SYNC;
    s.cmp_confirms = 2u;
    s.phase_A = true;
    s.last_tim5_capture = 1u;
    // p is TIM5 ticks of a 60-2 tooth (legacy fixture arg). One rev = 60 teeth.
    const uint32_t rev_ns = (p == 0u) ? 0u : (p * 16u * 60u);
    s.rpm_x10 = ckp_test_rpm_x10_from_period_ns(rev_ns);
    ckp_publish_encoder_snapshot(s);
}

void cam_fire(uint32_t capture_value) {
    ems_test_cam_gpio_idr = (1u << 1u);
    ems_test_tim5_ccr2 = capture_value;
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

void encoder_seq_seed_omega(void) {
    // omega=0.5 (d_tim2=500, d_tim5=1000). End at tim2=1400 so the next
    // sequential heartbeat (typ. 1500) does not repeat tim2_now (Δ=0 ⇒ ω=0).
    ecu_sched_encoder_test_set_tim2_cnt(900u);
    ecu_sched_encoder_heartbeat_tick(900u, 1000u, 0u, 0u);
    ecu_sched_encoder_test_set_tim2_cnt(1400u);
    ecu_sched_encoder_heartbeat_tick(1400u, 2000u, 0u, 0u);
}

uint32_t encoder_seq_arm_cyl_in_window(uint8_t cyl, uint32_t tim5_now,
                                       uint32_t cmp_angle, uint32_t cmp_edges) {
    const uint32_t window = ecu_sched_encoder_test_arm_window_counts();
    // Predict with the currently seeded ω (typ. 0.5 from encoder_seq_seed_omega).
    uint32_t now = 0u;
    for (int i = 0; i < 3; ++i) {
        const uint32_t arm = ecu_sched_encoder_test_predict_arm_at(cyl, now);
        now = arm - (window / 2u);
    }
    // Re-seed ω at the destination with the same 0.5 ratio. A raw heartbeat
    // jump from the seed tip (~1400) to `now` would spike ω and push dwell/PW
    // spans outside the 60° arm window.
    constexpr uint32_t kD2 = 500u;
    constexpr uint32_t kD5 = 1000u;
    ecu_sched_encoder_omega_test_reset();
    const uint32_t t5_prev = (tim5_now > kD5) ? (tim5_now - kD5) : 0u;
    ecu_sched_encoder_omega_sample(now - kD2, t5_prev);
    ecu_sched_encoder_omega_sample(now, tim5_now);
    ecu_sched_encoder_test_set_tim2_cnt(now);
    // Same (tim2,tim5) as last omega sample ⇒ d_tim5==0 ⇒ ω unchanged.
    ecu_sched_encoder_heartbeat_tick(now, tim5_now, cmp_angle, cmp_edges);
    return now;
}

