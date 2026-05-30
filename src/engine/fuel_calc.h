#pragma once

#include <cstdint>

#include "engine/engine_config.h"
#include "engine/table3d.h"

namespace ems::engine {

constexpr uint32_t calc_req_fuel_us_constexpr(uint16_t displacement_cc,
                                              uint8_t cylinders,
                                              uint16_t injector_flow_cc_min,
                                              uint16_t stoich_afr_x100) noexcept {
    if (displacement_cc == 0u || cylinders == 0u ||
        injector_flow_cc_min == 0u || stoich_afr_x100 == 0u) {
        return 0u;
    }

    const uint64_t num = static_cast<uint64_t>(displacement_cc) *
                         cfg::kAirDensityMgPerCcX1000 *
                         100u *
                         60000000u;
    const uint64_t den = static_cast<uint64_t>(cylinders) *
                         stoich_afr_x100 *
                         injector_flow_cc_min *
                         cfg::kFuelDensityMgPerCc *
                         1000u;
    const uint32_t req = static_cast<uint32_t>(num / den);
    return (req > 50000u) ? 50000u : req;
}

inline constexpr uint32_t kDefaultReqFuelUs =
    calc_req_fuel_us_constexpr(cfg::kDisplacementCc,
                               cfg::kCylinderCount,
                               cfg::kInjectorFlowCcMin,
                               cfg::kStoichAfrX100);

uint8_t get_ve(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;
uint8_t get_ve_prepared(const Table2dLookup& lookup) noexcept;
uint16_t get_lambda_target_x1000(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;
uint16_t get_lambda_target_x1000_prepared(const Table2dLookup& lookup) noexcept;

uint32_t calc_req_fuel_us(uint16_t displacement_cc,
                          uint8_t cylinders,
                          uint16_t injector_flow_cc_min,
                          uint16_t stoich_afr_x100) noexcept;
uint32_t default_req_fuel_us() noexcept;

uint32_t calc_base_pw_us(uint16_t req_fuel_us,
                         uint8_t ve,
                         uint16_t map_bar_x100,
                         uint16_t map_ref_bar_x100) noexcept;
uint32_t calc_base_pw_us_default(uint8_t ve,
                                 uint16_t map_bar_x100) noexcept;

uint32_t apply_lambda_target_pw_us(uint32_t base_pw_us,
                                   uint16_t lambda_target_x1000) noexcept;

uint32_t apply_fuel_trim_pw_us(uint32_t base_pw_us,
                               int16_t trim_pct_x10) noexcept;

uint16_t corr_clt(int16_t clt_x10) noexcept;
uint16_t corr_iat(int16_t iat_x10) noexcept;
uint16_t corr_vbatt(uint16_t vbatt_mv) noexcept;
uint16_t corr_warmup(int16_t clt_x10) noexcept;

uint32_t calc_final_pw_us(uint32_t base_pw_us,
                          uint16_t corr_clt_x256,
                          uint16_t corr_iat_x256,
                          uint16_t dead_time_us) noexcept;
uint32_t calc_fuel_pw_us_default_fast(uint8_t ve,
                                      uint16_t map_bar_x100,
                                      uint16_t lambda_target_x1000,
                                      int16_t trim_pct_x10,
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
void fuel_lambda_delay_reset() noexcept;

uint16_t lambda_delay_ms_from_rpm_load(uint32_t rpm_x10,
                                       uint16_t map_bar_x100) noexcept;

int16_t fuel_update_stft(uint32_t rpm_x10,
                         uint16_t map_bar_x100,
                         int16_t lambda_target_x1000,
                         int16_t lambda_measured_x1000,
                         int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut,
                         uint32_t net_pw_us) noexcept;

int16_t fuel_update_stft_delayed(uint32_t now_ms,
                                 uint32_t rpm_x10,
                                 uint16_t map_bar_x100,
                                 int16_t lambda_target_x1000,
                                 int16_t lambda_measured_x1000,
                                 int16_t clt_x10,
                                 bool o2_valid,
                                 bool ae_active,
                                 bool rev_cut,
                                 uint32_t net_pw_us) noexcept;

int16_t fuel_get_stft_pct_x10() noexcept;
int16_t fuel_get_ltft_pct_x10(uint8_t map_idx, uint8_t rpm_idx) noexcept;
// LTFT aditivo: retorna offset em µs (negativo = redução de PW)
int16_t fuel_get_ltft_add_us(uint8_t map_idx, uint8_t rpm_idx) noexcept;

// Corte de combustível na desaceleração (MS42 TI_PUR).
// Chama a cada ciclo de injeção (2ms); atualiza estado interno com histerese.
// Retorna true enquanto o corte estiver ativo.
bool fuel_decel_cut_update(uint32_t rpm_x10,
                           uint16_t tps_pct_x10,
                           int16_t clt_x10) noexcept;
bool fuel_decel_cut_active() noexcept;
void fuel_decel_cut_reset() noexcept;

// Compensação barométrica (MS42 TI_FAC_ALTI).
// Leitura do MAP com motor parado → referência dinâmica de pressão atmosférica.
// Clamp de segurança: [70, 110] centibares (700–1100 mbar).
void     fuel_set_baro_bar_x100(uint16_t baro) noexcept;
uint16_t fuel_get_baro_bar_x100() noexcept;

}  // namespace ems::engine
