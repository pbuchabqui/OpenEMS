#pragma once
#include <cstdint>

void drv_set_valid_adc(void);
void drv_setup(void);
void etb_ctrl_setup(void);

extern uint32_t g_ckp_cap;
extern const uint32_t kNormalPeriod;
extern const uint32_t kGapPeriod;

void ckp_fire(uint32_t delta);
void ckp_feed_n_then_gap(uint32_t n, uint32_t p = kNormalPeriod);
void ckp_reach_full_sync(uint32_t p = kNormalPeriod);
void cam_fire(uint32_t capture_value);

void sensor_setup(void);

// Seed encoder ω ≈ 0.5 (d_tim2=500, d_tim5=1000); ends at tim2=1400 so the
// next heartbeat can use a different tim2_now (Δ=0 would force ω=0).
void encoder_seq_seed_omega(void);

// Place TIM2 within the ≤60° arm window of `cyl` (dwell/inj_on earliest) and
// run a sequential heartbeat. Requires phase_valid + omega already seeded.
// Returns the now_raw used. dwell/inj spans from current g_* are included.
uint32_t encoder_seq_arm_cyl_in_window(uint8_t cyl, uint32_t tim5_now,
                                       uint32_t cmp_angle, uint32_t cmp_edges);
