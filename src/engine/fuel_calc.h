#pragma once

#include <cstdint>

#include "engine/engine_config.h"
#include "engine/table3d.h"
#include "engine/fuel_trim.h"

namespace ems::engine {

uint8_t get_ve(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;
uint8_t get_ve_prepared(const Table2dLookup& lookup) noexcept;
uint16_t get_lambda_target_x1000(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;

// EOI (° BTDC combustão, fim da injeção) por RPM×CLT — bilinear 3×3, ver
// calibration.h (eoi_rpm_axis_x10/eoi_clt_axis_x10/eoi_table_deg).
uint16_t calc_eoi_lead_deg(uint32_t rpm_x10, int16_t clt_x10) noexcept;
uint16_t get_lambda_target_x1000_prepared(const Table2dLookup& lookup) noexcept;

uint32_t calc_req_fuel_us(uint16_t displacement_cc,
                          uint8_t cylinders,
                          uint16_t injector_flow_cc_min,
                          uint16_t stoich_afr_x100) noexcept;
uint32_t default_req_fuel_us() noexcept;
// Flex fuel: runtime stoich AFR ×100 (0 = configured value). Not persisted.
void fuel_set_stoich_override_x100(uint16_t afr_x100) noexcept;

uint16_t corr_clt(int16_t clt_x10) noexcept;
uint16_t corr_iat(int16_t iat_x10) noexcept;
uint16_t corr_vbatt(uint16_t vbatt_mv) noexcept;

// Densidade do ar (lei dos gases ideais, T_ref/T_iat em Q8) — física pura,
// não calibrável. T_ref = 298.0K (25°C), mesma referência de
// cfg::kAirDensityMgPerCcX1000. Ver comentário em fuel_calc.cpp.
uint16_t corr_iat_density_q8(int16_t iat_x10) noexcept;

uint32_t apply_injector_scurve(uint32_t pw_us) noexcept;

uint32_t apply_delta_p_compensation(uint32_t pw_us,
                                    uint16_t fuel_press_bar_x1000,
                                    uint16_t map_bar_x100) noexcept;

uint32_t calc_final_pw_us(uint32_t base_pw_us,
                          uint16_t corr_clt_x256,
                          uint16_t corr_iat_x256,
                          uint16_t dead_time_us) noexcept;

// Cycle electrical formula: flow + dead × openings.
// Sequential (1) = flow+dead; semi/sim (2) = flow+2×dead.
// Dead is never folded into flow and then halved. flow=0 → 0.
uint32_t inj_cycle_pw_us(uint32_t flow_us, uint16_t dead_time_us,
                         uint8_t squirts) noexcept;
// One opening so the squirts sum to inj_cycle_pw_us (pin width).
uint32_t inj_pulse_pw_us(uint32_t flow_us, uint16_t dead_time_us,
                         uint8_t squirts) noexcept;
uint32_t calc_fuel_pw_us_default_fast(uint8_t ve,
                                      uint16_t map_bar_x100,
                                      uint16_t iat_density_q8,
                                      uint16_t lambda_target_x1000,
                                      int16_t trim_pct_x10,
                                      uint16_t corr_clt_x256,
                                      uint16_t corr_iat_x256,
                                      uint16_t dead_time_us) noexcept;

// taper: ticks legados (≤64) ou ms (>64).
void fuel_ae_apply_taper_raw(uint16_t raw) noexcept;
void fuel_ae_reset() noexcept;

// STFT / X-τ learn freeze enquanto o pulso AE tip-in (µs > 0) estiver activo.
// Chamar a cada tick 2 ms com o ae_pw_us calculado (também quando 0).
void fuel_ae_notify_pulse(int32_t ae_pw_us) noexcept;
bool fuel_ae_stft_freeze_active() noexcept;
void fuel_ae_stft_freeze_clear() noexcept;

// AE/DE from precomputed TPSdot (map fusion ring).
// tpsdot > +threshold → tip-in enrichment (µs > 0);
// tpsdot < −threshold → tip-out enleanment (µs < 0, 50% authority).
int32_t calc_ae_pw_from_tpsdot(int16_t tpsdot_x10, int16_t clt_x10) noexcept;

// Corte de combustível na desaceleração (MS42 TI_PUR).
bool fuel_decel_cut_update(uint32_t rpm_x10,
                           uint16_t tps_pct_x10,
                           int16_t clt_x10) noexcept;
bool fuel_decel_cut_active() noexcept;
void fuel_decel_cut_reset() noexcept;
// Rising edge do DFCO (válido até ao próximo update) — reset de filme só na entrada.
bool fuel_decel_cut_just_entered() noexcept;
// Soft ramp-in linear 0→1 após exit (decel_cut_ramp_ms; 0 = passthrough).
uint32_t fuel_decel_cut_ramp_pw(uint32_t flow_us, uint16_t dt_ms) noexcept;
// Contexto extra do DFCO (estilo FOME #485/#487), fornecido antes do update:
// MAP corrente p/ o gate de vácuo (decel_cut_map_max_bar_x100 > 0) e marcha
// p/ a inibição pós-troca (decel_cut_gear_inhibit_ms10 > 0 — troca também
// derruba um corte activo, evitando jerk na transmissão).
// Exit por MAP: MAP > gate + 5 bar×100 mesmo com TPS fechado.
void fuel_decel_cut_notify_map(uint16_t map_bar_x100) noexcept;
// Relógio (ms) do atraso de entrada do DFCO (ms42.dfco_entry_delay_ms).
void fuel_decel_cut_notify_time(uint32_t now_ms) noexcept;
void fuel_decel_cut_notify_gear(uint8_t gear, uint32_t now_ms) noexcept;

// Protecção de duty do injector (FOME #215): chamar 1×/tick de 2 ms com o
// PW final comandado. duty% (ciclo 720°) acima de inj_duty_max_pct por mais
// de inj_duty_tol_ms10×10 ms → corte; retoma só abaixo de 20% (FOME).
// inj_duty_max_pct = 0 → inerte.
bool     fuel_inj_duty_update(uint32_t pw_us, uint32_t rpm_x10,
                              uint16_t dt_ms) noexcept;
bool     fuel_inj_duty_cut_active() noexcept;
uint16_t fuel_inj_duty_pct_x10() noexcept;
void     fuel_inj_duty_reset() noexcept;

// Compensação barométrica (MS42 TI_FAC_ALTI).
void     fuel_set_baro_bar_x100(uint16_t baro) noexcept;
uint16_t fuel_get_baro_bar_x100() noexcept;

}  // namespace ems::engine
