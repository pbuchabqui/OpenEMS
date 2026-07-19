#pragma once

#include <cstdint>

namespace ems::engine {

// ── Razões tipadas de corte fuel/spark (estilo FOME ClearReason) ─────────────
// Bitmask em vez de enum único: vários cortes podem estar activos em
// simultâneo (ex.: limp + overtemp) e o dash mostra todos. Escritas apenas
// pelo main loop de 2 ms (single-writer); lidas pela telemetria ('D' [51],
// hi16 = spark, lo16 = fuel). 0 = sem corte.

// fuel (injecção)
inline constexpr uint16_t kFuelCutRevLimit = 1u << 0;  // limitador hard de RPM
inline constexpr uint16_t kFuelCutLimpRpm  = 1u << 1;  // limp acima do tecto de RPM
inline constexpr uint16_t kFuelCutMapFault = 1u << 2;  // MAP em fault (carga não fiável)
inline constexpr uint16_t kFuelCutOilPress = 1u << 3;  // protecção pressão de óleo
inline constexpr uint16_t kFuelCutFuelRail = 1u << 4;  // fault pressão de combustível
inline constexpr uint16_t kFuelCutOvertemp = 1u << 5;  // sobreaquecimento crítico
inline constexpr uint16_t kFuelCutDiagCrit = 1u << 6;  // diagnóstico crítico
inline constexpr uint16_t kFuelCutNoSync   = 1u << 7;  // HALF lockout / sem sync
inline constexpr uint16_t kFuelCutDfco     = 1u << 8;  // decel fuel cut (TI_PUR)
inline constexpr uint16_t kFuelCutFlood    = 1u << 9;  // flood clear no cranking
inline constexpr uint16_t kFuelCutInjDuty  = 1u << 10; // duty do injector acima do limite

// spark (ignição)
inline constexpr uint16_t kSparkCutLimpRpm  = 1u << 0;  // limp acima do tecto de RPM
inline constexpr uint16_t kSparkCutOilPress = 1u << 1;  // protecção pressão de óleo
inline constexpr uint16_t kSparkCutOvertemp = 1u << 2;  // sobreaquecimento crítico
inline constexpr uint16_t kSparkCutDiagCrit = 1u << 3;  // diagnóstico crítico
inline constexpr uint16_t kSparkSkipActive  = 1u << 4;  // soft limiter (corte parcial)

inline volatile uint16_t g_fuel_cut_reasons  = 0u;
inline volatile uint16_t g_spark_cut_reasons = 0u;

}  // namespace ems::engine
