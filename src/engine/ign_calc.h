#pragma once

#include <cstdint>

#include "engine/engine_config.h"
#include "engine/table3d.h"

namespace ems::engine {

// Avanço em décimos de grau BTDC (positivo = avanço, negativo = após o PMS).
// Faixa única: tabela no TunerStudio, cálculo e agendador.
constexpr int16_t kAdvanceMinX10 = -200;
constexpr int16_t kAdvanceMaxX10 = 600;

// Todas as correções em décimos de grau. O retardo de knock não entra aqui:
// é por cilindro e vai direto ao agendador (ecu_sched_set_cyl_retard_x10).
struct AdvanceCorrectionsX10 {
    int16_t iat;             // + avança / − atrasa (ar quente)
    int16_t clt;             // + avança / − atrasa (aquecimento do catalisador)
    int16_t idle;            // correção de marcha lenta (+ avança)
    int16_t antijerk_retard; // subtraído: retardo anti-jerk no tip-in
    int16_t torque_retard;   // subtraído: TC / launch
};

int16_t calc_ign_iat_correction_x10(int16_t iat_x10) noexcept;
int16_t calc_ign_clt_correction_x10(int16_t clt_x10) noexcept;
// Retardo anti-jerk no tip-in. Dispara quando tpsdot > antijerk_tpsdot_threshold_x10;
// magnitude proporcional a tpsdot até antijerk_retard_deg (full scale @ 100 %/s).
int16_t calc_antijerk_retard_x10(int16_t tpsdot_x10) noexcept;
void    antijerk_reset() noexcept;

// Tabela de avanço (graus inteiros) interpolada com resolução de 0,1°.
int16_t get_advance_x10(uint32_t rpm_x10, uint16_t load_bar_x100) noexcept;
int16_t get_advance_x10_prepared(const Table2dLookup& lookup) noexcept;
int16_t clamp_advance_x10(int32_t advance_x10) noexcept;

int16_t calc_total_advance_x10(int16_t base_x10, AdvanceCorrectionsX10 corr) noexcept;
int16_t calc_idle_spark_correction_x10(uint32_t rpm_x10,
                                       uint16_t idle_target_rpm_x10,
                                       uint16_t tps_pct_x10,
                                       uint16_t map_bar_x100) noexcept;

uint16_t dwell_ms_x10_from_vbatt(uint16_t vbatt_mv) noexcept;

// Dwell com correcção 2D tensão × RPM (MS42 §2.2.2.2.1 IP_TD__VB__N_32).
// Usa dwell_ms_x10_from_vbatt() como base e aplica o factor RPM Q8.
uint16_t dwell_ms_x10_from_vbatt_rpm(uint16_t vbatt_mv, uint32_t rpm_x10) noexcept;

uint32_t inj_pw_us_to_scheduler_ticks(uint32_t pw_us) noexcept;

}  // namespace ems::engine
