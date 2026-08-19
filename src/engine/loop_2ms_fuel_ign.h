/**
 * @file loop_2ms_fuel_ign.h
 * @brief Slot 2 ms: protect + combustível/ignição + EncFuelIgnPrep + commit.
 *
 * Chamado do main depois dos watchdogs e da amostra de sensores.
 * Watchdogs TIM5 / CMP ficam no main. finalize_cyl_setpoints() (pulso
 * por cilindro) não vive aqui.
 */
#pragma once

#include "drv/ckp.h"
#include "drv/sensors.h"

#include <cstdint>

inline constexpr uint32_t kLimpRpmLimit_x10 = 30000u;
inline constexpr uint16_t kMapMinBarX100    = 10u;
inline constexpr uint16_t kMapMaxBarX100    = 300u;
inline constexpr uint16_t kLambdaMinMilli   = 700u;
inline constexpr uint16_t kLambdaMaxMilli   = 1400u;

inline int8_t clamp_i8(int16_t v, int8_t lo, int8_t hi) noexcept
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return static_cast<int8_t>(v);
}

void loop_2ms_fuel_ign(uint32_t now,
                       const ems::drv::CkpSnapshot& snap,
                       const ems::drv::SensorData& sensors) noexcept;

// Telemetria / estado lido pelos slots 20 ms, 100 ms e ETB.
extern int8_t   g_last_advance_deg;
extern int16_t  g_torque_spark_retard_deg;
extern uint8_t  g_last_pw_ms_x10;
extern int8_t   g_last_stft_pct;
extern uint8_t  g_last_lambda_target_d4;
extern uint16_t g_last_map_fused_x100;
extern int8_t   g_last_ltft_pct;
extern uint32_t g_last_net_pw_us;
extern bool     g_limp_active;
extern bool     g_rev_limit_active;
