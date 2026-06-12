#include "engine/calibration.h"

#include <cstring>

#include "drv/sensors.h"

namespace ems::engine {

uint8_t ve_table[kTableAxisSize][kTableAxisSize] = {
    {45u, 47u, 50u, 52u, 53u, 54u, 55u, 56u, 58u, 60u, 62u, 63u, 64u, 66u, 68u, 70u},
    {48u, 52u, 55u, 57u, 58u, 59u, 60u, 61u, 63u, 65u, 67u, 68u, 70u, 72u, 74u, 76u},
    {52u, 56u, 60u, 62u, 63u, 64u, 65u, 66u, 68u, 70u, 72u, 74u, 76u, 78u, 80u, 82u},
    {56u, 61u, 64u, 66u, 67u, 68u, 69u, 70u, 72u, 74u, 76u, 78u, 80u, 82u, 84u, 86u},
    {60u, 65u, 68u, 70u, 72u, 73u, 74u, 75u, 77u, 79u, 81u, 83u, 85u, 87u, 89u, 91u},
    {64u, 68u, 72u, 75u, 77u, 79u, 80u, 81u, 83u, 85u, 87u, 89u, 91u, 93u, 95u, 96u},
    {68u, 72u, 76u, 79u, 81u, 83u, 84u, 85u, 87u, 89u, 91u, 93u, 95u, 97u, 99u, 100u},
    {70u, 75u, 79u, 82u, 84u, 86u, 87u, 88u, 90u, 92u, 94u, 96u, 98u, 100u, 102u, 103u},
    {73u, 78u, 82u, 85u, 88u, 90u, 91u, 92u, 94u, 96u, 98u, 100u, 102u, 104u, 106u, 107u},
    {90u, 96u, 102u, 107u, 111u, 114u, 116u, 118u, 121u, 124u, 127u, 130u, 133u, 136u, 139u, 142u},
    {105u, 113u, 120u, 126u, 131u, 135u, 138u, 141u, 145u, 149u, 153u, 157u, 161u, 165u, 169u, 173u},
    {120u, 130u, 138u, 146u, 152u, 157u, 161u, 165u, 170u, 175u, 180u, 185u, 190u, 195u, 200u, 205u},
    {135u, 146u, 156u, 165u, 173u, 179u, 184u, 189u, 195u, 201u, 207u, 213u, 219u, 225u, 231u, 237u},
    {150u, 163u, 175u, 186u, 195u, 203u, 209u, 215u, 222u, 229u, 236u, 243u, 250u, 252u, 253u, 254u},
    {165u, 180u, 194u, 207u, 218u, 227u, 235u, 242u, 250u, 252u, 253u, 254u, 254u, 254u, 254u, 254u},
    {180u, 197u, 213u, 228u, 241u, 250u, 252u, 253u, 254u, 254u, 254u, 254u, 254u, 254u, 254u, 254u},
};

int16_t lambda_target_table_x1000[kTableAxisSize][kTableAxisSize] = {
    {1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050, 1050},
    {1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030, 1030},
    {1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010, 1010},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},
    {990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990, 990},
    {970, 970, 970, 970, 960, 950, 940, 930, 920, 920, 920, 920, 920, 930, 940, 950},
    {930, 930, 925, 920, 915, 910, 900, 895, 890, 890, 890, 895, 900, 905, 910, 920},
    {900, 900, 895, 890, 885, 880, 870, 865, 860, 860, 865, 870, 875, 885, 895, 905},
    {880, 880, 875, 870, 865, 855, 845, 840, 835, 835, 840, 845, 855, 865, 875, 890},
    {860, 860, 855, 850, 845, 835, 825, 820, 815, 815, 820, 830, 840, 850, 865, 880},
    {845, 845, 840, 835, 830, 820, 810, 805, 800, 800, 805, 815, 825, 840, 855, 875},
    {825, 825, 820, 815, 810, 800, 790, 785, 780, 780, 785, 795, 810, 830, 850, 870},
    {810, 810, 805, 800, 795, 785, 775, 770, 765, 765, 775, 790, 805, 825, 850, 870},
};

int8_t spark_table[kTableAxisSize][kTableAxisSize] = {
    {12, 15, 18, 22, 26, 30, 33, 32, 34, 35, 38, 40, 40, 39, 37, 35},
    {10, 13, 16, 20, 24, 28, 31, 30, 32, 33, 36, 38, 38, 37, 35, 33},
    {8, 11, 14, 18, 22, 26, 29, 28, 30, 31, 34, 36, 36, 35, 33, 31},
    {6, 9, 12, 16, 20, 23, 26, 25, 27, 28, 31, 33, 33, 32, 30, 28},
    {4, 7, 10, 14, 18, 21, 24, 23, 25, 26, 29, 31, 31, 30, 28, 26},
    {2, 5, 8, 12, 15, 18, 21, 20, 22, 23, 26, 28, 28, 27, 25, 23},
    {1, 3, 6, 10, 13, 16, 19, 18, 20, 21, 24, 26, 26, 25, 23, 21},
    {0, 2, 4, 8, 11, 14, 17, 16, 18, 19, 22, 24, 24, 23, 21, 19},
    {0, 1, 3, 6, 9, 12, 15, 14, 16, 17, 20, 22, 22, 21, 19, 17},
    {0, 0, 2, 5, 8, 11, 13, 12, 14, 15, 18, 20, 20, 19, 17, 15},
    {0, 0, 1, 4, 7, 9, 11, 10, 12, 13, 16, 18, 18, 17, 15, 13},
    {0, 0, 0, 3, 5, 7, 9, 8, 10, 11, 14, 16, 16, 15, 13, 11},
    {0, 0, 0, 2, 4, 6, 8, 7, 9, 10, 13, 15, 15, 14, 12, 10},
    {2, 2, 2, 4, 6, 8, 9, 8, 10, 11, 14, 16, 16, 15, 13, 11},
    {1, 1, 2, 3, 5, 7, 8, 7, 9, 10, 13, 15, 15, 14, 12, 10},
    {0, 1, 1, 2, 4, 6, 7, 6, 8, 9, 12, 14, 14, 13, 11, 9},
};

int16_t clt_corr_axis_x10[kCorrectionTableSize] = {-400, -100, 0, 200, 400, 700, 900, 1100};
uint16_t clt_corr_x256[kCorrectionTableSize] = {384u, 352u, 320u, 288u, 272u, 256u, 256u, 256u};

int16_t iat_corr_axis_x10[kCorrectionTableSize] = {-200, 0, 200, 400, 600, 800, 1000, 1200};
uint16_t iat_corr_x256[kCorrectionTableSize] = {272u, 264u, 256u, 256u, 264u, 272u, 280u, 288u};

int16_t warmup_corr_axis_x10[kCorrectionTableSize] = {-400, -100, 0, 200, 400, 700, 900, 1100};
uint16_t warmup_corr_x256[kCorrectionTableSize] = {420u, 380u, 350u, 320u, 290u, 256u, 256u, 256u};

uint16_t vbatt_corr_axis_mv[kCorrectionTableSize] = {9000u, 10000u, 11000u, 12000u, 13000u, 14000u, 15000u, 16000u};
uint16_t injector_dead_time_us[kCorrectionTableSize] = {1400u, 1200u, 1050u, 900u, 800u, 700u, 650u, 600u};

int16_t ae_clt_corr_axis_x10[kCorrectionTableSize] = {-400, -100, 0, 200, 400, 700, 900, 1100};
uint16_t ae_clt_sens[kCorrectionTableSize] = {11u, 10u, 9u, 8u, 7u, 6u, 5u, 4u};
uint16_t ae_tpsdot_threshold_x10 = 5u;
uint16_t ae_taper_cycles = 8u;
uint16_t ae_max_pw_us = 5000u;
uint16_t ae_tpsdot_axis_x10[kAeRateTableSize] = {5u, 20u, 50u, 100u};
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
uint16_t mspark_max_rpm_x10        = 12000u;  // 1200 RPM — gate RPM
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
uint16_t tps_raw_min = 200u;
uint16_t tps_raw_max = 3895u;

uint8_t xtau_autocal_enabled = 0u;
uint8_t xtau_autocal_active = 0u;
int8_t xtau_autocal_tau_delta[kCorrectionTableSize] = {};

// IAT spark correction: retarda 0° a 20°C, até -5° a 80°C+
int16_t iat_spark_axis_x10[kCorrectionTableSize] = {-200, 0, 200, 400, 500, 600, 700, 800};
int16_t iat_spark_corr_deg[kCorrectionTableSize]  = {2, 1, 0, -1, -2, -3, -4, -5};

// CLT spark correction: retarda até -8° no aquecimento inicial para catalisador
int16_t clt_spark_axis_x10[kCorrectionTableSize] = {-400, -100, 200, 400, 600, 700, 900, 1100};
int16_t clt_spark_corr_deg[kCorrectionTableSize]  = {0, -4, -8, -4, 0, 0, 0, 0};

// Anti-jerk: mesmo limiar de tpsdot do AE; 3° retardo por 3 ciclos
uint16_t antijerk_tpsdot_threshold_x10 = 5u;   // mesmo que ae_tpsdot_threshold_x10
int16_t  antijerk_retard_deg           = 3;
uint8_t  antijerk_decay_cycles         = 3u;

// Rev limiter: limite duro 7000 RPM
uint32_t rev_limit_rpm_x10           = 70000u;
// Corte de injeção: janela 200 RPM (6800–7000 RPM)
uint32_t rev_limit_soft_window_x10   = 2000u;
// Retardo de ignição: janela 500 RPM (6500–7000 RPM), máx 15° retardo
uint32_t rev_limit_spark_window_x10  = 5000u;
int16_t  rev_limit_max_retard_deg    = 15;

// LTFT aditivo: ativa quando PW < 2.5ms (regime de marcha lenta / carga baixa)
uint16_t ltft_add_pw_threshold_us = 2500u;

// Corte de combustível na desaceleração (MS42 TI_PUR)
// Entrada: TPS < 0.5% + RPM > 1500 + CLT > 70°C
// Saída: RPM < 1200 (histerese 300 RPM) OU TPS abre
uint16_t decel_cut_tps_threshold_x10 = 5u;      // 0.5% TPS
uint32_t decel_cut_entry_rpm_x10     = 15000u;   // 1500 RPM
uint32_t decel_cut_exit_rpm_x10      = 12000u;   // 1200 RPM (histerese)
int16_t  decel_cut_min_clt_x10       = 700;      // 70°C

void apply_etb_calibration_from_page(const uint8_t* page, uint16_t len) noexcept {
    if (page == nullptr || len < 36u) {
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
        std::memcpy(&tps_raw_min, page + 36, 2u);
        std::memcpy(&tps_raw_max, page + 38, 2u);
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

}  // namespace ems::engine
