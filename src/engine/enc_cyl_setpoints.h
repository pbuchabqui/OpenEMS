/**
 * @file enc_cyl_setpoints.h
 * @brief Prep 2 ms + finalize por cilindro no arm sequencial encoder.
 *
 * O loop 2 ms publica EncFuelIgnPrep (sensores, VE/λ/tabelas, AE/X-τ, corrs).
 * try_arm_sequential_due() finaliza PW/avanço do cilindro que entra na janela
 * ≤60° — sem reexecutar o pipeline pesado na ISR.
 */
#pragma once

#include <cstdint>

namespace ems::engine {

struct EncFuelIgnPrep {
    uint8_t  valid;                 // 1 = snapshot publicado
    uint8_t  fuel_cut;              // força PW=0
    uint8_t  cranking;              // usa crank_spark_deg
    uint16_t map_bar_x100;
    uint16_t fuel_press_bar_x1000;
    uint16_t dead_time_us;
    uint32_t flow_pw_us;            // pós AE/X-τ/quick_crank, pré ΔP/S-curve/dead
    int16_t  base_advance_deg;      // tabela (sem knock)
    int16_t  crank_spark_deg;
    int16_t  iat_spark_deg;
    int16_t  clt_spark_deg;
    int16_t  idle_spark_deg;
    int16_t  antijerk_retard_deg;
    int16_t  torque_retard_deg;
    uint32_t dwell_ticks;
    uint32_t eoi_lead_deg;
};

struct CylArmSetpoints {
    uint32_t advance_deg;
    uint32_t dwell_ticks;
    uint32_t inj_pw_ticks;
    uint32_t eoi_lead_deg;
};

void enc_fuel_ign_prep_publish(const EncFuelIgnPrep& prep) noexcept;
EncFuelIgnPrep enc_fuel_ign_prep_read(void) noexcept;
uint8_t enc_fuel_ign_prep_valid(void) noexcept;

// Finalize leve: knock do cyl + trims + ΔP/S-curve/dead a partir do prep.
// Se prep.valid==0, deriva de g_* (sem reaplicar knock — já no commit) + trims.
CylArmSetpoints finalize_cyl_setpoints(uint8_t cyl) noexcept;

void enc_cyl_setpoints_reset(void) noexcept;

#if defined(EMS_HOST_TEST)
void enc_fuel_ign_prep_test_publish(const EncFuelIgnPrep& prep) noexcept;
#endif

}  // namespace ems::engine
