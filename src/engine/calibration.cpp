#include "engine/calibration.h"

#include <cstring>

#include "drv/sensors.h"

namespace ems::engine {

uint8_t ve_table[kTableAxisSize][kTableAxisSize] = {
    {45u, 47u, 50u, 52u, 53u, 54u, 54u, 54u, 55u, 56u, 56u, 58u, 60u, 62u, 63u, 64u, 66u, 67u, 68u, 70u},
    {48u, 52u, 55u, 57u, 58u, 58u, 59u, 60u, 60u, 60u, 61u, 63u, 65u, 67u, 68u, 70u, 72u, 73u, 74u, 76u},
    {52u, 56u, 60u, 62u, 63u, 64u, 64u, 64u, 65u, 66u, 66u, 68u, 70u, 72u, 74u, 76u, 78u, 79u, 80u, 82u},
    {54u, 58u, 62u, 64u, 65u, 66u, 66u, 66u, 67u, 68u, 68u, 70u, 72u, 74u, 76u, 78u, 80u, 81u, 82u, 84u},
    {56u, 61u, 64u, 66u, 67u, 68u, 68u, 68u, 69u, 70u, 70u, 72u, 74u, 76u, 78u, 80u, 82u, 83u, 84u, 86u},
    {58u, 63u, 66u, 68u, 70u, 70u, 70u, 71u, 72u, 72u, 72u, 74u, 76u, 78u, 80u, 82u, 84u, 86u, 86u, 88u},
    {60u, 65u, 68u, 70u, 72u, 72u, 73u, 74u, 74u, 74u, 75u, 77u, 79u, 81u, 83u, 85u, 87u, 88u, 89u, 91u},
    {62u, 66u, 70u, 72u, 74u, 75u, 76u, 76u, 77u, 78u, 78u, 80u, 82u, 84u, 86u, 88u, 90u, 91u, 92u, 94u},
    {64u, 68u, 72u, 75u, 77u, 78u, 79u, 80u, 80u, 80u, 81u, 83u, 85u, 87u, 89u, 91u, 93u, 94u, 95u, 96u},
    {68u, 72u, 76u, 79u, 81u, 82u, 83u, 84u, 84u, 84u, 85u, 87u, 89u, 91u, 93u, 95u, 97u, 98u, 99u, 100u},
    {69u, 74u, 78u, 80u, 82u, 84u, 84u, 85u, 86u, 86u, 86u, 88u, 90u, 92u, 94u, 96u, 98u, 100u, 100u, 102u},
    {70u, 75u, 79u, 82u, 84u, 85u, 86u, 86u, 87u, 88u, 88u, 90u, 92u, 94u, 96u, 98u, 100u, 101u, 102u, 103u},
    {73u, 78u, 82u, 85u, 88u, 89u, 90u, 90u, 91u, 92u, 92u, 94u, 96u, 98u, 100u, 102u, 104u, 105u, 106u, 107u},
    {90u, 96u, 102u, 107u, 111u, 112u, 114u, 115u, 116u, 117u, 118u, 121u, 124u, 127u, 130u, 133u, 136u, 138u, 139u, 142u},
    {105u, 113u, 120u, 126u, 131u, 133u, 135u, 136u, 138u, 140u, 141u, 145u, 149u, 153u, 157u, 161u, 165u, 167u, 169u, 173u},
    {120u, 130u, 138u, 146u, 152u, 154u, 157u, 159u, 161u, 163u, 165u, 170u, 175u, 180u, 185u, 190u, 195u, 198u, 200u, 205u},
    {135u, 146u, 156u, 165u, 173u, 176u, 179u, 182u, 184u, 186u, 189u, 195u, 201u, 207u, 213u, 219u, 225u, 228u, 231u, 237u},
    {150u, 163u, 175u, 186u, 195u, 199u, 203u, 206u, 209u, 212u, 215u, 222u, 229u, 236u, 243u, 250u, 252u, 252u, 253u, 254u},
    {165u, 180u, 194u, 207u, 218u, 222u, 227u, 231u, 235u, 238u, 242u, 250u, 252u, 253u, 254u, 254u, 254u, 254u, 254u, 254u},
    {180u, 197u, 213u, 228u, 241u, 246u, 250u, 251u, 252u, 252u, 253u, 254u, 254u, 254u, 254u, 254u, 254u, 254u, 254u, 254u},
};

int16_t lambda_target_table_x1000[kTableAxisSize][kTableAxisSize] = {
    {1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050},
    {1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030},
    {1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010},
    {1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005, 1005},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995, 995},
    {990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990},
    {970, 970, 970, 970, 960, 955, 950, 945, 940, 935, 930, 920, 920, 920, 920, 920, 930, 935, 940, 950},
    {930, 930, 925, 920, 915, 912, 910, 905, 900, 898, 895, 890, 890, 890, 895, 900, 905, 908, 910, 920},
    {900, 900, 895, 890, 885, 882, 880, 875, 870, 868, 865, 860, 860, 865, 870, 875, 885, 890, 895, 905},
    {880, 880, 875, 870, 865, 860, 855, 850, 845, 842, 840, 835, 835, 840, 845, 855, 865, 870, 875, 890},
    {860, 860, 855, 850, 845, 840, 835, 830, 825, 822, 820, 815, 815, 820, 830, 840, 850, 858, 865, 880},
    {845, 845, 840, 835, 830, 825, 820, 815, 810, 808, 805, 800, 800, 805, 815, 825, 840, 848, 855, 875},
    {825, 825, 820, 815, 810, 805, 800, 795, 790, 788, 785, 780, 780, 785, 795, 810, 830, 840, 850, 870},
    {810, 810, 805, 800, 795, 790, 785, 780, 775, 772, 770, 765, 765, 775, 790, 805, 825, 838, 850, 870},
};

int8_t spark_table[kTableAxisSize][kTableAxisSize] = {
    {12, 15, 18, 22, 26, 28, 30, 32, 33, 32, 32, 34, 35, 38, 40, 40, 39, 38, 37, 35},
    {10, 13, 16, 20, 24, 26, 28, 30, 31, 30, 30, 32, 33, 36, 38, 38, 37, 36, 35, 33},
    {8, 11, 14, 18, 22, 24, 26, 28, 29, 28, 28, 30, 31, 34, 36, 36, 35, 34, 33, 31},
    {7, 10, 13, 17, 21, 23, 24, 26, 28, 27, 26, 28, 30, 32, 34, 34, 34, 32, 32, 30},
    {6, 9, 12, 16, 20, 22, 23, 24, 26, 26, 25, 27, 28, 31, 33, 33, 32, 31, 30, 28},
    {5, 8, 11, 15, 19, 20, 22, 24, 25, 24, 24, 26, 27, 30, 32, 32, 31, 30, 29, 27},
    {4, 7, 10, 14, 18, 20, 21, 22, 24, 24, 23, 25, 26, 29, 31, 31, 30, 29, 28, 26},
    {3, 6, 9, 13, 16, 18, 20, 21, 22, 22, 22, 24, 24, 28, 30, 30, 28, 28, 26, 24},
    {2, 5, 8, 12, 15, 16, 18, 20, 21, 20, 20, 22, 23, 26, 28, 28, 27, 26, 25, 23},
    {1, 3, 6, 10, 13, 14, 16, 18, 19, 18, 18, 20, 21, 24, 26, 26, 25, 24, 23, 21},
    {0, 2, 5, 9, 12, 14, 15, 16, 18, 18, 17, 19, 20, 23, 25, 25, 24, 23, 22, 20},
    {0, 2, 4, 8, 11, 12, 14, 16, 17, 16, 16, 18, 19, 22, 24, 24, 23, 22, 21, 19},
    {0, 1, 3, 6, 9, 10, 12, 14, 15, 14, 14, 16, 17, 20, 22, 22, 21, 20, 19, 17},
    {0, 0, 2, 5, 8, 10, 11, 12, 13, 12, 12, 14, 15, 18, 20, 20, 19, 18, 17, 15},
    {0, 0, 1, 4, 7, 8, 9, 10, 11, 10, 10, 12, 13, 16, 18, 18, 17, 16, 15, 13},
    {0, 0, 0, 3, 5, 6, 7, 8, 9, 8, 8, 10, 11, 14, 16, 16, 15, 14, 13, 11},
    {0, 0, 0, 2, 4, 5, 6, 7, 8, 8, 7, 9, 10, 13, 15, 15, 14, 13, 12, 10},
    {2, 2, 2, 4, 6, 7, 8, 8, 9, 8, 8, 10, 11, 14, 16, 16, 15, 14, 13, 11},
    {1, 1, 2, 3, 5, 6, 7, 8, 8, 8, 7, 9, 10, 13, 15, 15, 14, 13, 12, 10},
    {0, 1, 1, 2, 4, 5, 6, 6, 7, 6, 6, 8, 9, 12, 14, 14, 13, 12, 11, 9},
};

int16_t clt_corr_axis_x10[kCorrectionTableSize] = {-400, -100, 0, 200, 400, 700, 900, 1100};
uint16_t clt_corr_x256[kCorrectionTableSize] = {384u, 352u, 320u, 288u, 272u, 256u, 256u, 256u};

int16_t iat_corr_axis_x10[kCorrectionTableSize] = {-200, 0, 200, 400, 600, 800, 1000, 1200};
uint16_t iat_corr_x256[kCorrectionTableSize] = {272u, 264u, 256u, 256u, 264u, 272u, 280u, 288u};

int16_t warmup_corr_axis_x10[kCorrectionTableSize] = {-400, -100, 0, 200, 400, 700, 900, 1100};
uint16_t warmup_corr_x256[kCorrectionTableSize] = {420u, 380u, 350u, 320u, 290u, 256u, 256u, 256u};

uint16_t vbatt_corr_axis_mv[kCorrectionTableSize] = {9000u, 10000u, 11000u, 12000u, 13000u, 14000u, 15000u, 16000u};
uint16_t injector_dead_time_us[kCorrectionTableSize] = {1400u, 1200u, 1050u, 900u, 800u, 700u, 650u, 600u};

uint16_t injector_scurve_pw_axis_us[kCorrectionTableSize] = {0u, 200u, 400u, 600u, 800u, 1000u, 1200u, 1500u};
uint16_t injector_scurve_corr_q8[kCorrectionTableSize] = {90u, 179u, 205u, 220u, 235u, 245u, 252u, 256u};

uint16_t fuel_press_nominal_bar_x1000 = 3000u;  // 3.0 bar

int16_t ae_clt_corr_axis_x10[kCorrectionTableSize] = {-400, -100, 0, 200, 400, 700, 900, 1100};
uint16_t ae_clt_sens[kCorrectionTableSize] = {11u, 10u, 9u, 8u, 7u, 6u, 5u, 4u};
// Limiar 3 %/s (×10) — evita AE em ruído de TPS; tip-in real fica acima de light-transient.
uint16_t ae_tpsdot_threshold_x10 = 30u;
uint16_t ae_taper_cycles = 8u;
uint16_t ae_max_pw_us = 5000u;
// Eixo de taxa tip-in/tip-out (%/s ×10). Começa no limiar default.
uint16_t ae_tpsdot_axis_x10[kAeRateTableSize] = {30u, 80u, 200u, 500u};
uint16_t ae_pw_adder_us[kAeRateTableSize] = {300u, 800u, 1500u, 2500u};

int16_t xtau_clt_axis_x10[kCorrectionTableSize] = {-400, -100, 0, 200, 400, 700, 900, 1100};
uint16_t xtau_x_fraction_q8[kCorrectionTableSize] = {77u, 70u, 64u, 54u, 45u, 35u, 28u, 24u};
uint16_t xtau_tau_cycles[kCorrectionTableSize] = {32u, 28u, 24u, 20u, 16u, 12u, 10u, 8u};

uint16_t crank_enter_rpm_x10 = 4500u;
uint16_t crank_exit_rpm_x10 = 7000u;
int16_t crank_spark_deg = 8;
uint16_t crank_min_pw_us = 2500u;
uint16_t crank_prime_tooth = 3u;
uint16_t crank_prime_max_pw_us = 30000u;

// Injector 2-slope (modelo rusEFI): abaixo do breakpoint o bico entrega menos
// combustível que o modelo linear ("kink") — o tempo comandado cresce mais
// rápido nessa região. rate_q8 = slope_small/slope_main em Q8 (ex.: 128 = o
// bico rende metade no pulso pequeno). 0 (ou ≥256 impossível em u8) = OFF —
// default seguro: comportamento idêntico ao modelo linear atual.
uint16_t inj_small_pulse_break_us = 0u;
uint8_t  inj_small_pulse_rate_q8  = 0u;


uint16_t dwell_vbatt_axis_mv[kIgnitionDwellTableSize] = {9000u, 10000u, 11000u, 12000u, 13000u, 14000u, 15000u, 16000u};
uint16_t dwell_ms_x10_table[kIgnitionDwellTableSize] = {42u, 38u, 35u, 30u, 28u, 25u, 23u, 22u};

// Eixo RPM para correcção de dwell (em RPM, não ×10)
uint16_t dwell_rpm_axis_rpm[kDwellRpmCorrSize] = {500u, 1200u, 4000u, 7000u};

// Factor multiplicativo Q8 (÷256): ajusta dwell por zona de RPM.
// 300 cranking: 1.5× — mais energia na bobina a baixa rotação
// 1200 idle:    1.125×
// 4000 cruise:  1.0×  — ponto de referência (tabela de tensão calibrada aqui)
// 7000 high:    0.78× — reduz dwell para evitar sobreposição de ciclos
uint16_t dwell_rpm_factor_q8[kDwellRpmCorrSize] = {384u, 288u, 256u, 200u};

// Multi-spark: activo abaixo de 1200 RPM, 2 sparks adicionais, dwell inter-spark 1.8ms
uint16_t mspark_max_rpm_x10        = kMsparkRpmCeilingX10;  // 1500 RPM — gate (hard max)
uint8_t  mspark_count              = 2u;      // sparks adicionais por ciclo
uint16_t mspark_inter_dwell_ms_x10 = 18u;     // 1.8ms entre sparks

uint32_t lambda_delay_rpm_axis_x10[kLambdaDelayTableSize] = {10000u, 25500u, 80000u};
uint32_t lambda_delay_load_axis_bar_x100[kLambdaDelayTableSize] = {10u, 91u, 300u};
uint16_t lambda_delay_ms_table[kLambdaDelayTableSize][kLambdaDelayTableSize] = {
    {1100u, 550u, 200u},
    {600u, 400u, 150u},
    {300u, 150u, 80u},
};

uint16_t idle_spark_tps_max_x10 = 25u;
uint16_t idle_spark_map_max_bar_x100 = 80u;
uint16_t idle_spark_rpm_min_x10 = 5000u;
uint16_t idle_spark_window_above_target_x10 = 4000u;
uint16_t idle_spark_deadband_rpm_x10 = 500u;
uint16_t idle_spark_rpm_per_deg_x10 = 500u;
int16_t idle_spark_retard_limit_deg = -8;
int16_t idle_spark_advance_limit_deg = 12;

uint16_t app1_raw_min = 200u;
uint16_t app1_raw_max = 3895u;
uint16_t app2_raw_min = 200u;
uint16_t app2_raw_max = 3895u;
uint16_t etb_tps1_raw_min = 200u;
uint16_t etb_tps1_raw_max = 3895u;
uint16_t etb_tps2_raw_min = 200u;
uint16_t etb_tps2_raw_max = 3895u;
uint16_t app_max_delta_pct_x10 = 120u;
uint16_t etb_max_delta_pct_x10 = 120u;
uint16_t etb_max_open_pct_x10_limp = 250u;
uint16_t etb_max_rate_pct_per_s = 500u;
uint16_t etb_idle_open_pct_x10 = 80u;
uint16_t etb_kp_x10 = 120u;
uint16_t etb_ki_x10 = 8u;
uint16_t etb_kd_x10 = 40u;
uint8_t etb_cal_valid = 0u;
uint8_t etb_harness_present = 0u;
uint16_t etb_pedal_map[4][10] = {
    {   0,  80, 150, 220, 300, 400, 520, 650, 800, 1000},  // ECO
    {   0, 100, 200, 300, 400, 500, 600, 700, 800, 1000},  // NORMAL
    {   0, 180, 350, 500, 600, 700, 780, 850, 920, 1000},  // SPORT
    {   0,  50, 100, 150, 220, 300, 400, 520, 650, 1000},  // RAIN
};
uint16_t tps_raw_min = 200u;
uint16_t tps_raw_max = 3895u;

int8_t cyl_fuel_trim_pct[cfg::kCylinderCount] = {};  // 0 = sem correção
int8_t cyl_ign_trim_deg[cfg::kCylinderCount]  = {};  // 0 = sem correção
uint8_t cmp_window_open_tooth  = 0u;  // 0/0 = desabilitado
uint8_t cmp_window_close_tooth = 0u;

uint16_t boost_target_bar_x1000[7][8] = {
    {1000u, 1020u, 1050u, 1080u, 1100u, 1120u, 1150u, 1180u},  // 0: neutro
    {1000u, 1050u, 1100u, 1150u, 1200u, 1250u, 1280u, 1300u},  // 1ª marcha
    {1000u, 1080u, 1150u, 1220u, 1280u, 1340u, 1380u, 1420u},  // 2ª marcha
    {1000u, 1100u, 1180u, 1260u, 1330u, 1400u, 1450u, 1500u},  // 3ª marcha
    {1000u, 1120u, 1210u, 1300u, 1380u, 1460u, 1520u, 1580u},  // 4ª marcha
    {1000u, 1140u, 1240u, 1340u, 1430u, 1520u, 1600u, 1680u},  // 5ª marcha
    {1000u, 1150u, 1260u, 1370u, 1470u, 1570u, 1660u, 1750u},  // 6ª marcha
};

uint8_t xtau_autocal_enabled = 0u;
uint8_t xtau_autocal_active = 0u;
int8_t xtau_autocal_tau_delta[kCorrectionTableSize] = {};

// IAT spark correction: retarda 0° a 20°C, até -5° a 80°C+
int16_t iat_spark_axis_x10[kCorrectionTableSize] = {-200, 0, 200, 400, 500, 600, 700, 800};
int16_t iat_spark_corr_deg[kCorrectionTableSize]  = {2, 1, 0, -1, -2, -3, -4, -5};

// CLT spark correction: retarda até -8° no aquecimento inicial para catalisador
int16_t clt_spark_axis_x10[kCorrectionTableSize] = {-400, -100, 200, 400, 600, 700, 900, 1100};
int16_t clt_spark_corr_deg[kCorrectionTableSize]  = {0, -4, -8, -4, 0, 0, 0, 0};

// Anti-jerk: limiar alinhado ao AE (3 %/s); retardo escala com tpsdot até max.
uint16_t antijerk_tpsdot_threshold_x10 = 30u;
int16_t  antijerk_retard_deg           = 5;   // máximo ° a 100 %/s tip-in
uint8_t  antijerk_decay_cycles         = 4u;

// Rev limiter: limite duro 7000 RPM
uint8_t  launch_enable           = 0u;
uint16_t launch_rpm_x10          = 45000u;  // 4500 RPM
uint16_t launch_etb_pct_x10      = 600u;    // 60%
uint16_t launch_app_arm_x10      = 200u;    // 20%
uint16_t launch_app_disarm_x10   = 50u;     // 5%
uint16_t launch_rpm_hyst_x10     = 100u;    // 10 RPM

uint8_t  tc_enable               = 0u;
uint16_t tc_app_min_x10          = 300u;    // 30%
uint16_t tc_rpm_min_x10          = 20000u;  // 2000 RPM
uint16_t tc_rpm_dot_thresh       = 8000u;   // +800 rpm_x10/s ≈ 800 RPM/s flare
uint16_t tc_max_reduction_pct_x10 = 800u;   // 80%
uint16_t tc_spark_retard_max_deg = 12u;
uint16_t tc_reduction_rate_x10   = 500u;    // 50 %/s slew

uint32_t rev_limit_rpm_x10           = 70000u;
// Corte de injeção: janela 200 RPM (6800–7000 RPM)
uint32_t rev_limit_soft_window_x10   = 2000u;
// LTFT aditivo: ativa quando PW < 2.5ms (regime de marcha lenta / carga baixa)
uint16_t ltft_add_pw_threshold_us = 2500u;
// Malha fechada: default on; post-start 15 s; LTFT só acima de 1200 RPM.
uint8_t  closed_loop_enable         = 1u;
uint8_t  ltft_apply_burn_ve         = 0u;   // burn VE após APPLY manual (default off)
uint16_t closed_loop_post_start_s   = 15u;
uint16_t ltft_adapt_min_rpm_x10     = 12000u;  // 1200 RPM
// LTFT authority / rates (page0 176-183)
uint16_t ltft_mult_clamp_pct_x10    = 250u;    // ±25.0 %
uint16_t ltft_add_clamp_us          = 6350u;
uint8_t  ltft_learn_div             = 64u;
uint8_t  ltft_commit_gain_pct       = 50u;
uint16_t ltft_max_step_x10          = 0u;      // 0 = sem cap de passo
uint8_t  ltft_adapt_enable          = 1u;      // 0 = freeze LTFT maps
uint16_t ltft_learn_ready_hits         = 30u;
uint8_t  ltft_learn_max_err_x1000      = 30u;
uint8_t  ltft_learn_ready_max_mean_err = 25u;
uint8_t  ltft_learn_ready_min_stft_x10 = 5u;
uint8_t  ltft_learn_ready_max_stft_x10 = 150u;

// Corte de combustível na desaceleração (MS42 TI_PUR)
// Entrada: TPS < 0.5% + RPM > 1500 + CLT > 70°C
// Saída: RPM < 1200 (histerese 300 RPM) OU TPS abre
uint16_t decel_cut_tps_threshold_x10 = 5u;      // 0.5% TPS
uint32_t decel_cut_entry_rpm_x10     = 15000u;   // 1500 RPM
uint32_t decel_cut_exit_rpm_x10      = 12000u;   // 1200 RPM (histerese)
int16_t  decel_cut_min_clt_x10       = 700;      // 70°C

// Marcha lenta ETB
uint16_t etb_idle_rpm_target      = 850u;
uint16_t etb_idle_min_opening_x10 = 30u;   // 3.0%
uint16_t etb_idle_max_opening_x10 = 80u;   // 8.0%

// Idle RPM target vs CLT — usado para idle spark correction (ETB)
int16_t  iac_clt_axis_x10[kIacWarmupPts]        = {-400, -100, 100, 300, 500, 700, 900, 1100};
uint16_t iac_idle_target_rpm_x10[kIacWarmupPts]  = {12000u, 11500u, 10800u, 10000u, 9200u, 8500u, 8200u, 8000u};
uint16_t wbo2_can_id = 0x180u;

uint16_t stft_kp_x100       = 3u;    // 0.03
uint16_t eoi_idle_deg      = 60u;   // closed-valve (fim na compressão)
uint16_t eoi_blend_rpm_lo  = 2000u; // abaixo: closed-valve (60°)
uint16_t eoi_blend_rpm_hi  = 4000u; // acima: open-valve (355°)

uint16_t stft_ki_x1000      = 5u;    // 0.005
uint16_t stft_clamp_pct_x10 = 250u;  // 25.0%

uint16_t xtau_x_min_q8  = 64u;   // 0.25
uint16_t xtau_x_max_q8  = 192u;  // 0.75
uint16_t xtau_tau_min   = 10u;
uint16_t xtau_tau_max   = 255u;

uint16_t ewg_kp_x10       = 80u;   // 8.0
uint16_t ewg_ki_x10       = 5u;    // 0.5
uint16_t ewg_kd_x10       = 20u;   // 2.0
uint16_t ewg_pos_min_raw  = 200u;
uint16_t ewg_pos_max_raw  = 3800u;

void apply_etb_calibration_from_page(const uint8_t* page, uint16_t len) noexcept {
    if (page == nullptr || len < 36u) {
        return;
    }
    // Validação: flash apagada lê 0xFF — sem isto o boot carregava 65535 como
    // calibração de pedal/ETB. Raws são ADC 12-bit (≤4095), pares min<max,
    // flags 0/1. Bloco inválido → mantém defaults de compilação.
    for (uint16_t i = 0u; i < 4u; ++i) {
        uint16_t mn = 0u;
        uint16_t mx = 0u;
        std::memcpy(&mn, page + (i * 4u), 2u);
        std::memcpy(&mx, page + (i * 4u) + 2u, 2u);
        if (mx > 4095u || mn >= mx) {
            return;
        }
    }
    if (page[26] > 1u || page[27] > 1u) {
        return;
    }
    std::memcpy(&app1_raw_min, page + 0, 2u);
    std::memcpy(&app1_raw_max, page + 2, 2u);
    std::memcpy(&app2_raw_min, page + 4, 2u);
    std::memcpy(&app2_raw_max, page + 6, 2u);
    std::memcpy(&etb_tps1_raw_min, page + 8, 2u);
    std::memcpy(&etb_tps1_raw_max, page + 10, 2u);
    std::memcpy(&etb_tps2_raw_min, page + 12, 2u);
    std::memcpy(&etb_tps2_raw_max, page + 14, 2u);
    std::memcpy(&app_max_delta_pct_x10, page + 16, 2u);
    std::memcpy(&etb_max_delta_pct_x10, page + 18, 2u);
    std::memcpy(&etb_max_open_pct_x10_limp, page + 20, 2u);
    std::memcpy(&etb_max_rate_pct_per_s, page + 22, 2u);
    std::memcpy(&etb_idle_open_pct_x10, page + 24, 2u);
    etb_cal_valid = page[26];
    etb_harness_present = page[27];
    std::memcpy(&etb_kp_x10, page + 28, 2u);
    std::memcpy(&etb_ki_x10, page + 30, 2u);
    std::memcpy(&etb_kd_x10, page + 32, 2u);
    if (len >= 40u) {
        uint16_t mn = 0u;
        uint16_t mx = 0u;
        std::memcpy(&mn, page + 36, 2u);
        std::memcpy(&mx, page + 38, 2u);
        if (mx <= 4095u && mn < mx) {
            tps_raw_min = mn;
            tps_raw_max = mx;
        }
    }
}

void sync_etb_calibration_to_page(uint8_t* page, uint16_t len) noexcept {
    if (page == nullptr || len < 36u) {
        return;
    }
    std::memcpy(page + 0, &app1_raw_min, 2u);
    std::memcpy(page + 2, &app1_raw_max, 2u);
    std::memcpy(page + 4, &app2_raw_min, 2u);
    std::memcpy(page + 6, &app2_raw_max, 2u);
    std::memcpy(page + 8, &etb_tps1_raw_min, 2u);
    std::memcpy(page + 10, &etb_tps1_raw_max, 2u);
    std::memcpy(page + 12, &etb_tps2_raw_min, 2u);
    std::memcpy(page + 14, &etb_tps2_raw_max, 2u);
    std::memcpy(page + 16, &app_max_delta_pct_x10, 2u);
    std::memcpy(page + 18, &etb_max_delta_pct_x10, 2u);
    std::memcpy(page + 20, &etb_max_open_pct_x10_limp, 2u);
    std::memcpy(page + 22, &etb_max_rate_pct_per_s, 2u);
    std::memcpy(page + 24, &etb_idle_open_pct_x10, 2u);
    page[26] = etb_cal_valid;
    page[27] = etb_harness_present;
    std::memcpy(page + 28, &etb_kp_x10, 2u);
    std::memcpy(page + 30, &etb_ki_x10, 2u);
    std::memcpy(page + 32, &etb_kd_x10, 2u);
    if (len >= 40u) {
        std::memcpy(page + 36, &tps_raw_min, 2u);
        std::memcpy(page + 38, &tps_raw_max, 2u);
    }
}

void push_sensor_calibration_to_drivers() noexcept {
    ems::drv::sensors_set_app_cal(app1_raw_min, app1_raw_max,
                                  app2_raw_min, app2_raw_max);
    ems::drv::sensors_set_etb_tps_cal(etb_tps1_raw_min, etb_tps1_raw_max,
                                      etb_tps2_raw_min, etb_tps2_raw_max);
    ems::drv::sensors_set_plausibility(app_max_delta_pct_x10, etb_max_delta_pct_x10);
    ems::drv::sensors_set_etb_harness_present(etb_harness_present != 0u);
    if (tps_raw_min < tps_raw_max && tps_raw_max <= 4095u) {
        ems::drv::sensors_set_tps_cal(tps_raw_min, tps_raw_max);
    }
}

void apply_xtau_autocal_from_page(const uint8_t* page, uint16_t len) noexcept {
    if (page == nullptr || len < 86u) {
        return;
    }
    xtau_autocal_enabled = page[76];
    std::memcpy(xtau_autocal_tau_delta, page + 78, 8u);
}

void sync_xtau_autocal_to_page(uint8_t* page, uint16_t len) noexcept {
    if (page == nullptr || len < 86u) {
        return;
    }
    page[76] = xtau_autocal_enabled;
    page[77] = 1u;
    std::memcpy(page + 78, xtau_autocal_tau_delta, 8u);
}

// page0 layout v5: launch 191-201, TC 202-215 (see calibration.h).
void launch_tc_serialize_to_page0(uint8_t* page0, uint16_t len) noexcept {
    if (page0 == nullptr || len < (kLaunchTcPage0Off + kLaunchTcPage0Len)) {
        return;
    }
    uint8_t* const p = page0 + kLaunchTcPage0Off;
    p[0] = (launch_enable != 0u) ? 1u : 0u;
    std::memcpy(p + 1,  &launch_rpm_x10,        2u);
    std::memcpy(p + 3,  &launch_etb_pct_x10,    2u);
    std::memcpy(p + 5,  &launch_app_arm_x10,    2u);
    std::memcpy(p + 7,  &launch_app_disarm_x10, 2u);
    std::memcpy(p + 9,  &launch_rpm_hyst_x10,   2u);
    p[11] = (tc_enable != 0u) ? 1u : 0u;
    p[12] = 0u;  // pad @203
    std::memcpy(p + 13, &tc_app_min_x10,           2u);
    std::memcpy(p + 15, &tc_rpm_min_x10,           2u);
    std::memcpy(p + 17, &tc_rpm_dot_thresh,        2u);
    std::memcpy(p + 19, &tc_max_reduction_pct_x10, 2u);
    std::memcpy(p + 21, &tc_spark_retard_max_deg,  2u);
    std::memcpy(p + 23, &tc_reduction_rate_x10,    2u);
}

void launch_tc_apply_from_page0(const uint8_t* page0, uint16_t len) noexcept {
    if (page0 == nullptr || len < (kLaunchTcPage0Off + kLaunchTcPage0Len)) {
        return;
    }
    const uint8_t* const p = page0 + kLaunchTcPage0Off;

    launch_enable = (p[0] != 0u) ? 1u : 0u;
    uint16_t rpm = 0u, etb = 0u, arm = 0u, disarm = 0u, hyst = 0u;
    std::memcpy(&rpm,    p + 1, 2u);
    std::memcpy(&etb,    p + 3, 2u);
    std::memcpy(&arm,    p + 5, 2u);
    std::memcpy(&disarm, p + 7, 2u);
    std::memcpy(&hyst,   p + 9, 2u);
    // RPM hold: 500–6500 RPM (uint16 ×10). 0 in blob keeps prior (flash blank/legacy).
    if (rpm != 0u) {
        if (rpm < 5000u) {
            rpm = 5000u;
        } else if (rpm > 65000u) {
            rpm = 65000u;
        }
        launch_rpm_x10 = rpm;
    }
    if (etb > 1000u) {
        etb = 1000u;
    }
    launch_etb_pct_x10 = etb;
    if (arm > 1000u) {
        arm = 1000u;
    }
    if (disarm > 1000u) {
        disarm = 1000u;
    }
    // Arm must be ≥ disarm (hysteresis). Swap if inverted.
    if (arm < disarm) {
        const uint16_t tmp = arm;
        arm = disarm;
        disarm = tmp;
    }
    launch_app_arm_x10    = arm;
    launch_app_disarm_x10 = disarm;
    if (hyst > 5000u) {
        hyst = 5000u;  // 500 RPM max deadband
    }
    launch_rpm_hyst_x10 = hyst;

    tc_enable = (p[11] != 0u) ? 1u : 0u;
    uint16_t app_min = 0u, rpm_min = 0u, dot = 0u, max_red = 0u, spark = 0u, rate = 0u;
    std::memcpy(&app_min, p + 13, 2u);
    std::memcpy(&rpm_min, p + 15, 2u);
    std::memcpy(&dot,     p + 17, 2u);
    std::memcpy(&max_red, p + 19, 2u);
    std::memcpy(&spark,   p + 21, 2u);
    std::memcpy(&rate,    p + 23, 2u);
    if (app_min > 1000u) {
        app_min = 1000u;
    }
    tc_app_min_x10 = app_min;
    if (rpm_min > 65000u) {
        rpm_min = 65000u;  // uint16 ×10 ceiling
    }
    tc_rpm_min_x10 = rpm_min;
    tc_rpm_dot_thresh = dot;  // 0 = very sensitive; allow
    if (max_red > 1000u) {
        max_red = 1000u;
    }
    tc_max_reduction_pct_x10 = max_red;
    if (spark > 30u) {
        spark = 30u;
    }
    tc_spark_retard_max_deg = spark;
    // rate 0 would freeze slew — floor at 1 if non-zero wire; 0 keeps prior default
    if (rate != 0u) {
        tc_reduction_rate_x10 = rate;
    }
}

}  // namespace ems::engine
