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
