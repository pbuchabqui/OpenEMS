/**
 * @file enc_cyl_setpoints.h
 * @brief Prep 2 ms + finalize por cilindro no arm sequencial encoder.
 *
 * O loop 2 ms publica EncFuelIgnPrep (sensores, VE/λ/tabelas, AE, corrs).
 * try_arm_sequential_due() finaliza PW/avanço do cilindro que entra na janela
 * ≤60° — VE bilineal opcional (map_window), X-τ/AE por evento de spray.
 */
#pragma once

#include <cstdint>

namespace ems::engine {

struct EncFuelIgnPrep {
    uint8_t  valid;                 // 1 = snapshot publicado
    uint8_t  fuel_cut;              // força PW=0
    uint8_t  cranking;              // usa crank_spark_deg
    uint8_t  xtau_event_enable;     // 1 = X-τ por spray no finalize (encoder)
    uint16_t map_bar_x100;          // MAP fundido (2 ms)
    uint16_t fuel_press_bar_x1000;
    uint16_t dead_time_us;
    uint32_t rpm_x10;
    uint16_t corr_clt_x256;
    uint16_t corr_iat_x256;
    int16_t  fuel_trim_pct_x10;     // STFT+LTFT do tick 2 ms
    int16_t  clt_x10;
    uint32_t base_flow_pw_us;       // fluxo sem AE (pré evento X-τ)
    int32_t  ae_pw_us;              // residual tip-in/out do 2 ms
    uint32_t flow_pw_us;            // telemetria: base+AE (legado / fallback)
    int16_t  base_advance_deg;      // tabela (sem knock)
    int16_t  crank_spark_deg;
    int16_t  iat_spark_deg;
    int16_t  clt_spark_deg;
    int16_t  idle_spark_deg;
    int16_t  antijerk_retard_deg;
    int16_t  torque_retard_deg;
    uint32_t dwell_ticks;
    uint32_t eoi_lead_deg;
    // !=0 = 'P' bench lock ativo. Só aplicado com motor parado (rpm_x10==0
    // e omega inválido/zero). Em marcha o sequencial ignora o lock.
    // Colapsa o tri-state de ecu_sched_bench_pw_override_state()
    // (0=livre/1=trancado/2=armado-a-trancar-no-próximo-commit) num
    // booleano — finalize só testa "!=0", nunca distingue 1 de 2.
    uint8_t  bench_pw_locked;
    uint32_t bench_pw_lock_ticks;    // valor a usar quando locked e parado
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

// Finalize leve: knock do cyl + trims + (opcional VE bilineal) + X-τ evento +
// ΔP/S-curve/dead a partir do prep.
// Se prep.valid==0, deriva de g_* (sem reaplicar knock — já no commit) + trims.
// commit_fuel=false: propaga para transient_fuel_xtau_event() como peek (não
// persiste o filme de parede) — usado por try_arm_sequential_due() enquanto
// avalia candidatos fora da janela de armamento (ver xtau_autocalib.h).
CylArmSetpoints finalize_cyl_setpoints(uint8_t cyl, bool commit_fuel = true) noexcept;

void enc_cyl_setpoints_reset(void) noexcept;

#if defined(EMS_HOST_TEST)
void enc_fuel_ign_prep_test_publish(const EncFuelIgnPrep& prep) noexcept;
#endif

}  // namespace ems::engine
