#pragma once

#include <cstdint>

#include "engine/table3d.h"

namespace ems::engine {

extern uint8_t ve_table[kTableAxisSize][kTableAxisSize];

uint8_t get_ve(uint16_t rpm_x10, uint16_t map_kpa) noexcept;

uint32_t calc_base_pw_us(uint16_t req_fuel_us,
                         uint8_t ve,
                         uint16_t map_kpa,
                         uint16_t map_ref_kpa) noexcept;

uint16_t corr_clt(int16_t clt_x10) noexcept;
uint16_t corr_iat(int16_t iat_x10) noexcept;
uint16_t corr_vbatt(uint16_t vbatt_mv) noexcept;
uint16_t corr_warmup(int16_t clt_x10) noexcept;

uint32_t calc_final_pw_us(uint32_t base_pw_us,
                          uint16_t corr_clt_x256,
                          uint16_t corr_iat_x256,
                          uint16_t dead_time_us) noexcept;

void fuel_ae_set_threshold(uint16_t threshold_tpsdot_x10) noexcept;
void fuel_ae_set_taper(uint8_t taper_cycles) noexcept;

int32_t calc_ae_pw_us(uint16_t tps_now_x10,
                      uint16_t tps_prev_x10,
                      uint16_t dt_ms,
                      int16_t clt_x10) noexcept;

void fuel_reset_adaptives() noexcept;

int16_t fuel_update_stft(uint16_t rpm_x10,
                         uint16_t map_kpa,
                         int16_t lambda_target_x1000,
                         int16_t lambda_measured_x1000,
                         int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut) noexcept;

int16_t fuel_get_stft_pct_x10() noexcept;
int16_t fuel_get_ltft_pct_x10(uint8_t map_idx, uint8_t rpm_idx) noexcept;

}  // namespace ems::engine
