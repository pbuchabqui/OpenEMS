#pragma once

#include <cstdint>

#include "engine/table3d.h"
#include "engine/engine_config.h"

namespace ems::engine {

constexpr uint8_t kCorrectionTableSize    = 8u;
constexpr uint8_t kIgnitionDwellTableSize = 8u;
constexpr uint8_t kDwellRpmCorrSize       = 4u;  // pontos do eixo RPM para correcção de dwell
constexpr uint8_t kLambdaDelayTableSize   = 3u;
constexpr uint8_t kAeRateTableSize        = 4u;

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

// Correcção de dwell por RPM (MS42 §2.2.2.2.1 — IP_TD__VB__N_32).
// dwell_final = dwell_1d(V) × dwell_rpm_factor_q8(N) / 256
// Eixo em RPM (não ×10) para caber em uint16_t.
// Factor Q8: 256 = 1.0× (referência a ~4000 RPM).
//   Cranking (~500 RPM): 384 = 1.5× (mais tempo disponível → mais energia na bobina)
//   High RPM (~7000 RPM): 200 = 0.78× (evitar sobreposição entre ciclos)
extern uint16_t dwell_rpm_axis_rpm[kDwellRpmCorrSize];   // eixo em RPM (500, 1200, 4000, 7000)
extern uint16_t dwell_rpm_factor_q8[kDwellRpmCorrSize];  // factor multiplicativo Q8

// Multi-spark (MS42 §2.2.3) — sparks adicionais a baixo RPM para melhorar ignição.
// mspark_max_rpm_x10: RPM (×10) acima do qual multi-spark é desabilitado.
// mspark_count: número de sparks adicionais por ciclo (0=desabilitado, máx 3).
// mspark_inter_dwell_ms_x10: dwell entre sparks consecutivos (ms ×10, ex: 18 = 1.8ms).
extern uint16_t mspark_max_rpm_x10;
extern uint8_t  mspark_count;
extern uint16_t mspark_inter_dwell_ms_x10;

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
// Pedal-to-throttle response maps: 4 modes × 10 points, units pct×10 (0–1000).
// Axis is fixed: pedal 0%,10%,...,90%,100%. Points[0]=0 and [9]=1000 are enforced.
extern uint16_t etb_pedal_map[4][10];
extern uint16_t tps_raw_min;   // TPS legado (cabo, PA4) — raw ADC em 0%/100%
extern uint16_t tps_raw_max;

// Boost target: 7 marchas (0=neutro/desconhecido, 1-6) × 8 pontos de RPM.
// Unidade: bar × 1000. Eixo RPM fixo (mesmo que kBoostRpmAxisX10 em auxiliaries).
extern uint16_t boost_target_bar_x1000[7][8];

// Trim por cilindro — índice = cilindro físico (0-based, kCylinderCount elementos).
// Fuel: ±% sobre o PW calculado.  Ign: ±° sobre o avanço calculado.
// Defaults zero = sem correção.
extern int8_t cyl_fuel_trim_pct[::ems::engine::cfg::kCylinderCount];
extern int8_t cyl_ign_trim_deg[::ems::engine::cfg::kCylinderCount];

// CMP dente único: janela de dentes do virabrequim onde a borda do sensor CMP
// é esperada. open=close=0 → validação desabilitada (usa só timing).
// Unidade: índice de dente (0 = primeiro após gap).
extern uint8_t cmp_window_open_tooth;
extern uint8_t cmp_window_close_tooth;

extern uint8_t  xtau_autocal_enabled;
extern uint8_t  xtau_autocal_active;
extern int8_t   xtau_autocal_tau_delta[kCorrectionTableSize];

// IAT ignition correction (MS42 §2.2.4 — retardo por temperatura do ar)
extern int16_t iat_spark_axis_x10[kCorrectionTableSize];
extern int16_t iat_spark_corr_deg[kCorrectionTableSize];

// CLT ignition correction — aquecimento do catalisador (MS42 §2.2.4.2 / TI_CH)
extern int16_t clt_spark_axis_x10[kCorrectionTableSize];
extern int16_t clt_spark_corr_deg[kCorrectionTableSize];

// Anti-jerk ignition retard (MS42 §2.2.4 conforto na transmissão)
extern uint16_t antijerk_tpsdot_threshold_x10;
extern int16_t  antijerk_retard_deg;
extern uint8_t  antijerk_decay_cycles;

// Limitador de RPM progressivo (MS42 §2.2.5 / PAT_INH_IV)
extern uint32_t rev_limit_rpm_x10;
extern uint32_t rev_limit_soft_window_x10;   // janela do corte de injeção (menor, ~200 RPM)

// Soft rev limiter por spark retard (complementar ao corte de injeção)
// Janela mais larga (ex. 500 RPM) — retarda progressivamente antes do corte de injeção
extern uint32_t rev_limit_spark_window_x10;  // janela do retardo de ignição
extern int16_t  rev_limit_max_retard_deg;    // retardo máximo no limite (ex. 15°)

// LTFT aditivo (MS42 TI_AD_ADD_MMV)
extern uint16_t ltft_add_pw_threshold_us;

// Corte de combustível na desaceleração (MS42 TI_PUR)
extern uint16_t decel_cut_tps_threshold_x10;
extern uint32_t decel_cut_entry_rpm_x10;
extern uint32_t decel_cut_exit_rpm_x10;
extern int16_t  decel_cut_min_clt_x10;

// Marcha lenta ETB — posição mínima/máxima da borboleta no idle (unidade: %×10)
// idle_rpm_target: RPM alvo (uint16, RPM direto, ex. 850)
// idle_min_opening_x10: abertura mínima (%×10, ex. 30 = 3.0%)
// idle_max_opening_x10: abertura máxima (%×10, ex. 80 = 8.0%)
extern uint16_t etb_idle_rpm_target;
extern uint16_t etb_idle_min_opening_x10;
extern uint16_t etb_idle_max_opening_x10;

// IAC PID gains — ganhos do controlador P+I+D da válvula de ar de marcha lenta
// Kp e Kd são numerador inteiro (denominador fixo = 1000 e 2 respectivamente).
// I-clamp: limite do integrador em unidades de duty×10.
// CLT enable: temperatura mínima (°C×10) para ativar o PID (abaixo usa warmup duty).
extern int16_t iac_kp_num;
extern int16_t iac_kd_num;
extern int16_t iac_kd_den;
extern int16_t iac_i_clamp_x10;
extern int16_t iac_clt_pid_enable_x10;

void apply_etb_calibration_from_page(const uint8_t* page, uint16_t len) noexcept;
// Empurra a calibração de sensores (APP/ETB/TPS/plausibilidade) p/ drv::sensors.
void push_sensor_calibration_to_drivers() noexcept;
void sync_etb_calibration_to_page(uint8_t* page, uint16_t len) noexcept;
void apply_xtau_autocal_from_page(const uint8_t* page, uint16_t len) noexcept;
void sync_xtau_autocal_to_page(uint8_t* page, uint16_t len) noexcept;

}  // namespace ems::engine
