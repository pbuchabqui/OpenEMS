#pragma once

#include <cstdint>

#include "engine/table3d.h"

namespace ems::engine {

constexpr uint8_t kCorrectionTableSize = 8u;
constexpr uint8_t kIgnitionDwellTableSize = 8u;
constexpr uint8_t kLambdaDelayTableSize = 3u;
constexpr uint8_t kAeRateTableSize = 4u;

extern uint8_t ve_table[kTableAxisSize][kTableAxisSize];
extern int16_t lambda_target_table_x1000[kTableAxisSize][kTableAxisSize];
extern int8_t spark_table[kTableAxisSize][kTableAxisSize];

extern int16_t clt_corr_axis_x10[kCorrectionTableSize];
extern uint16_t clt_corr_x256[kCorrectionTableSize];

extern int16_t iat_corr_axis_x10[kCorrectionTableSize];
extern uint16_t iat_corr_x256[kCorrectionTableSize];

extern int16_t warmup_corr_axis_x10[kCorrectionTableSize];
extern uint16_t warmup_corr_x256[kCorrectionTableSize];

extern uint16_t vbatt_corr_axis_mv[kCorrectionTableSize];
extern uint16_t injector_dead_time_us[kCorrectionTableSize];

extern int16_t ae_clt_corr_axis_x10[kCorrectionTableSize];
extern uint16_t ae_clt_sens[kCorrectionTableSize];
extern uint16_t ae_tpsdot_threshold_x10;
extern uint16_t ae_taper_cycles;
extern uint16_t ae_max_pw_us;
extern uint16_t ae_tpsdot_axis_x10[kAeRateTableSize];
extern uint16_t ae_pw_adder_us[kAeRateTableSize];

extern int16_t xtau_clt_axis_x10[kCorrectionTableSize];
extern uint16_t xtau_x_fraction_q8[kCorrectionTableSize];
extern uint16_t xtau_tau_cycles[kCorrectionTableSize];

extern uint16_t crank_enter_rpm_x10;
extern uint16_t crank_exit_rpm_x10;
extern int16_t crank_spark_deg;
extern uint16_t crank_min_pw_us;
extern uint16_t crank_prime_tooth;
extern uint16_t crank_prime_max_pw_us;

extern uint16_t dwell_vbatt_axis_mv[kIgnitionDwellTableSize];
extern uint16_t dwell_ms_x10_table[kIgnitionDwellTableSize];

extern uint32_t lambda_delay_rpm_axis_x10[kLambdaDelayTableSize];
extern uint32_t lambda_delay_load_axis_bar_x100[kLambdaDelayTableSize];
extern uint16_t lambda_delay_ms_table[kLambdaDelayTableSize][kLambdaDelayTableSize];

extern uint16_t idle_spark_tps_max_x10;
extern uint16_t idle_spark_map_max_bar_x100;
extern uint16_t idle_spark_rpm_min_x10;
extern uint16_t idle_spark_window_above_target_x10;
extern uint16_t idle_spark_deadband_rpm_x10;
extern uint16_t idle_spark_rpm_per_deg_x10;
extern int16_t idle_spark_retard_limit_deg;
extern int16_t idle_spark_advance_limit_deg;

extern uint16_t app1_raw_min;
extern uint16_t app1_raw_max;
extern uint16_t app2_raw_min;
extern uint16_t app2_raw_max;
extern uint16_t etb_tps1_raw_min;
extern uint16_t etb_tps1_raw_max;
extern uint16_t etb_tps2_raw_min;
extern uint16_t etb_tps2_raw_max;
extern uint16_t app_max_delta_pct_x10;
extern uint16_t etb_max_delta_pct_x10;
extern uint16_t etb_max_open_pct_x10_limp;
extern uint16_t etb_max_rate_pct_per_s;
extern uint16_t etb_idle_open_pct_x10;
extern uint16_t etb_kp_x10;
extern uint16_t etb_ki_x10;
extern uint16_t etb_kd_x10;
extern uint8_t  etb_cal_valid;
extern uint8_t  etb_harness_present;

extern uint8_t  xtau_autocal_enabled;
extern uint8_t  xtau_autocal_active;
extern int8_t   xtau_autocal_tau_delta[kCorrectionTableSize];

void apply_etb_calibration_from_page(const uint8_t* page, uint16_t len) noexcept;
void sync_etb_calibration_to_page(uint8_t* page, uint16_t len) noexcept;
void apply_xtau_autocal_from_page(const uint8_t* page, uint16_t len) noexcept;
void sync_xtau_autocal_to_page(uint8_t* page, uint16_t len) noexcept;

}  // namespace ems::engine
