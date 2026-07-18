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

// S-curve do injetor: correção não-linear de PW pequeno (bico não abre
// linearmente perto do tempo de dead-time). Eixo em µs de PW teórico,
// correção em Q8 (256 = 1.0×, sem correção — acima do último ponto do eixo).
extern uint16_t injector_scurve_pw_axis_us[kCorrectionTableSize];
extern uint16_t injector_scurve_corr_q8[kCorrectionTableSize];

// Compensação Δ-P: pressão de combustível nominal de calibração (bar × 1000).
// Fluxo do bico ∝ sqrt(ΔP); usada como referência quando o sensor de pressão
// real (sensors.fuel_press_bar_x1000) diverge desse nominal.
extern uint16_t fuel_press_nominal_bar_x1000;

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

// Injector 2-slope: breakpoint (µs de PW líquido) + razão small/main em Q8.
// 0 = desligado (linear puro). Ver calc_final_pw_us.
extern uint16_t inj_small_pulse_break_us;
extern uint8_t  inj_small_pulse_rate_q8;

// Spark-skip soft limiter: janela (RPM×10) antes do hard cut + ratio máximo
// Q8 (clampa a 128 = 50%). 0 = off. Ver engine/spark_skip.h.
extern uint16_t spark_skip_window_rpm_x10;
extern uint8_t  spark_skip_max_q8;

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
// Hard ceiling 1500 RPM — above this the dwell window is too short for extra sparks.
// mspark_count: número de sparks adicionais por ciclo (0=desabilitado, máx 3).
// mspark_inter_dwell_ms_x10: dwell entre sparks consecutivos (ms ×10, ex: 18 = 1.8ms).
constexpr uint16_t kMsparkRpmCeilingX10 = 15000u;  // 1500 RPM absolute max gate
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

// ── Launch control (torque_manager) — page0 191-201 (layout v5+) ─────────────
// enable=0 off. When armed (APP ≥ arm): hold engine near launch_rpm via ETB cap.
// Disarm when APP drops below disarm. No clutch switch required (street-lite).
// Offsets: 191 en, 192-193 rpm_x10, 194-195 etb, 196-197 app_arm, 198-199 disarm,
//          200-201 hyst.
extern uint8_t  launch_enable;
extern uint16_t launch_rpm_x10;           // target hold RPM ×10 (default 4500)
extern uint16_t launch_etb_pct_x10;       // max ETB while active (default 60%)
extern uint16_t launch_app_arm_x10;       // APP to arm (default 20%)
extern uint16_t launch_app_disarm_x10;    // APP to disarm (default 5%)
extern uint16_t launch_rpm_hyst_x10;      // deadband around target (default 100=10 RPM)

// ── Traction control (torque_manager) — page0 202-215 (layout v5+) ───────────
// 202 en, 203 pad, 204-205 app_min, 206-207 rpm_min, 208-209 rpm_dot_thresh,
// 210-211 max_red, 212-213 spark_max, 214-215 rate.
// Slip priority: external API → CAN wheel vs vehicle (vehicle_inputs) → RPM-dot proxy.
extern uint8_t  tc_enable;
extern uint16_t tc_app_min_x10;           // min APP to intervene (default 30%)
extern uint16_t tc_rpm_min_x10;           // min RPM (default 2000)
// rpm_dot in rpm_x10 per second: 8000 ≈ +800 RPM/s flare
extern uint16_t tc_rpm_dot_thresh;
extern uint16_t tc_max_reduction_pct_x10; // max throttle cut 0-1000 (default 800=80%)
extern uint16_t tc_spark_retard_max_deg;  // max spark retard ° (default 12)
extern uint16_t tc_reduction_rate_x10;    // %×10 per second slew of reduction (default 500=50%/s)

// page0 wire helpers (layout v5 block 191-215). Safe no-ops on short buffers.
constexpr uint16_t kLaunchTcPage0Off = 191u;
constexpr uint16_t kLaunchTcPage0Len = 25u;  // 191..215 inclusive
void launch_tc_serialize_to_page0(uint8_t* page0, uint16_t len) noexcept;
void launch_tc_apply_from_page0(const uint8_t* page0, uint16_t len) noexcept;

// Rev limiter: retardo progressivo de faísca removido em b565491 (rusEFI-style:
// corte só de combustível, faísca nunca cortada). Offsets 80-85 da page 0
// ficam reservados para não partir o layout do protocolo.

// LTFT aditivo (MS42 TI_AD_ADD_MMV)
extern uint16_t ltft_add_pw_threshold_us;

// Closed-loop / LEARN — page0 offsets 80-85.
// 80: closed_loop_enable (0=open-loop freeze STFT+LTFT, 1=on). Default 1.
// 81: ltft_apply_burn_ve — após APPLY manual ('Y'), 0=VE só RAM; 1=burn page1 se RPM seguro.
// 82-83: closed_loop_post_start_s — segundos após CLT+O2 OK antes de integrar (default 15).
// 84-85: ltft_adapt_min_rpm_x10 — abaixo disto STFT corre, LTFT IIR+LEARN congelam (default 1200 RPM).
extern uint8_t  closed_loop_enable;
extern uint8_t  ltft_apply_burn_ve;
extern uint16_t closed_loop_post_start_s;
extern uint16_t ltft_adapt_min_rpm_x10;

// LTFT authority / rates — page0 offsets 176-183 (após layout version @175).
// mult_clamp: teto |LTFT %×10| (default 250 = ±25%; separado do STFT).
// add_clamp_us: teto |LTFT add| em µs (default 6350).
// learn_div: IIR cell += (stft−cell)/div (default 64; min 1).
// commit_gain_pct: fracção do mean STFT no bake VE (default 50).
// max_step_x10: |ΔLTFT %×10| por tick (0 = sem limite extra além do IIR).
extern uint16_t ltft_mult_clamp_pct_x10;
extern uint16_t ltft_add_clamp_us;
extern uint8_t  ltft_learn_div;
extern uint8_t  ltft_commit_gain_pct;
extern uint16_t ltft_max_step_x10;
// page0[184]: 0 = STFT only (LTFT IIR+LEARN frozen); 1 = adapt LTFT (default).
extern uint8_t  ltft_adapt_enable;

// LEARN thresholds — page0 185-190 (0 no blob → defaults constexpr em fuel_trim.h).
extern uint16_t ltft_learn_ready_hits;           // default 30
extern uint8_t  ltft_learn_max_err_x1000;        // sample |err| max, default 30
extern uint8_t  ltft_learn_ready_max_mean_err;   // default 25
extern uint8_t  ltft_learn_ready_min_stft_x10;   // default 5
extern uint8_t  ltft_learn_ready_max_stft_x10;   // default 150

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

// Idle RPM target vs CLT — 8 pontos, eixo CLT (°C×10); usado pelo ETB idle spark
constexpr uint8_t kIacWarmupPts = 8u;
extern int16_t  iac_clt_axis_x10[kIacWarmupPts];
extern uint16_t iac_idle_target_rpm_x10[kIacWarmupPts];

// ID CAN 11-bit do sensor wideband lambda (WBO2). Default 0x180.
// Gravado em NVM (página 0, offset 138). Aplicado via can_stack_init().
extern uint16_t wbo2_can_id;

// STFT closed-loop tuning (página 0, offsets 140-145)
extern uint16_t stft_kp_x100;        // Kp × 100, default 3 (= 0.03)
// ── EOI blend por RPM (fase de injeção) ─────────────────────────────────
// EOI efetivo interpolado linearmente entre eoi_idle_deg (rpm ≤ lo) e
// g_eng_cfg.default_eoi_lead_deg (rpm ≥ hi). hi ≤ lo (incl. 0/0) = DESLIGADO
// → usa sempre default_eoi_lead_deg (comportamento pré-blend). RPM em
// unidades planas (não ×10): u16 ×10 saturaria a 6553 RPM.
extern uint16_t eoi_idle_deg;        // ° BTDC combustão, default 60 (closed-valve)
extern uint16_t eoi_blend_rpm_lo;    // RPM início do blend, default 0 (off)
extern uint16_t eoi_blend_rpm_hi;    // RPM fim do blend,    default 0 (off)

extern uint16_t stft_ki_x1000;       // Ki × 1000, default 5 (= 0.005)
extern uint16_t stft_clamp_pct_x10;  // clamp ±%, default 250 (= 25.0%)

// X-τ auto-calibration limits (página 0, offsets 146-153)
extern uint16_t xtau_x_min_q8;       // X min Q8, default 64 (= 0.25)
extern uint16_t xtau_x_max_q8;       // X max Q8, default 192 (= 0.75)
extern uint16_t xtau_tau_min;         // τ min cycles, default 10
extern uint16_t xtau_tau_max;         // τ max cycles, default 255

// EWG position PID (página 0, offsets 154-163)
extern uint16_t ewg_kp_x10;          // Kp × 10, default 80
extern uint16_t ewg_ki_x10;          // Ki × 10, default 5
extern uint16_t ewg_kd_x10;          // Kd × 10, default 20
extern uint16_t ewg_pos_min_raw;     // ADC raw at fully closed
extern uint16_t ewg_pos_max_raw;     // ADC raw at fully open

void apply_etb_calibration_from_page(const uint8_t* page, uint16_t len) noexcept;
// Empurra a calibração de sensores (APP/ETB/TPS/plausibilidade) p/ drv::sensors.
void push_sensor_calibration_to_drivers() noexcept;
void sync_etb_calibration_to_page(uint8_t* page, uint16_t len) noexcept;
void apply_xtau_autocal_from_page(const uint8_t* page, uint16_t len) noexcept;
void sync_xtau_autocal_to_page(uint8_t* page, uint16_t len) noexcept;

}  // namespace ems::engine
