/**
 * @file limp_gating.h
 * @brief Single writer of fuel/spark/ETB allow (FOME LimpManager, OpenEMS style).
 *
 * limp_gating_update() is the only path that computes allow flags, writes
 * g_fuel_cut_reasons / g_spark_cut_reasons, and commits inj/ign inhibit masks.
 */
#pragma once

#include <cstdint>

extern bool g_rev_limit_active;

namespace ems::engine {

struct LimpGatingInputs {
    uint32_t rpm_x10;
    uint16_t map_bar_x100;
    int16_t  clt_degc_x10;
    uint16_t oil_press_bar_x1000;
    bool     oil_fault;
    bool     fuel_press_fault;
    uint16_t lambda_x1000;
    bool     lambda_valid;
    uint16_t lambda_target_x1000;
    uint16_t tps_pct_x10;
    bool     cranking;
    bool     full_sync;
    bool     half_sync;
    bool     etb_fault;
    uint8_t  inj_duty_pct;
    bool     inj_duty_cut;
    uint32_t now_ms;
    bool     overtemp_crit;
    bool     diag_critical;
    bool     flood_clear;
    bool     limp_rpm_cut;
    // Encoder TIM2: HALF_SYNC already has 360° — running fuel allowed.
    // Hall 60-2: HALF_SYNC after crank is unknown cycle — lock fuel.
    bool     half_sync_allows_fuel;
};

struct LimpGatingResult {
    uint8_t  inj_inhibit_mask;
    uint8_t  ign_inhibit_mask;
    bool     allow_injection;
    bool     allow_ignition;
    bool     rev_limit_active;
    bool     fuel_protect_cut;
    bool     half_fuel_lockout;
};

// Runtime knobs (0 = disabled so benches do not trip).
extern uint16_t boost_cut_map_bar_x100;
extern uint16_t oil_min_after_start_bar_x1000;
extern uint16_t lambda_protect_timeout_ms;
extern uint16_t lambda_protect_dev_x1000;
extern uint32_t lambda_protect_min_rpm_x10;
extern uint16_t lambda_protect_min_load_bar_x100;

LimpGatingResult limp_gating_update(const LimpGatingInputs& in) noexcept;

void limp_gating_report_etb_problem() noexcept;
// Libera o latch do problema de ETB (o limite de giro de falha de ETB). Se a causa persistir, o
// próximo limp_gating_update / limp_gating_report_etb_problem o trava de novo.
void limp_gating_clear_etb_problem() noexcept;
// DFCO is decided later in the fuel path; still a single writer of the bit.
void limp_gating_or_fuel_reason(uint16_t bit) noexcept;

// Per-cut disable mask (RAM, lost on reset). Bit=1 → that cut is ignored.
// Never covers rev-limit, flood, fatal, stall, watchdogs, or no-sync/phase.
inline constexpr uint16_t kProtectDisOil      = 1u << 0;
inline constexpr uint16_t kProtectDisRail     = 1u << 2;
inline constexpr uint16_t kProtectDisOvertemp = 1u << 3;
inline constexpr uint16_t kProtectDisDiag     = 1u << 4;
inline constexpr uint16_t kProtectDisLambda   = 1u << 5;
inline constexpr uint16_t kProtectDisEtbLimp  = 1u << 6;
inline constexpr uint16_t kProtectDisLimpRpm  = 1u << 7;
inline constexpr uint16_t kProtectDisBoost    = 1u << 8;
inline constexpr uint16_t kProtectDisInjDuty  = 1u << 9;
inline constexpr uint16_t kProtectDisSensorGroup = 0x00FDu;  // bits 0–7, sem o bit 1 (MAP, removido)
inline constexpr uint16_t kProtectDisWritable    = 0x03FDu;  // bits 0–9, sem o bit 1 (MAP, removido)

void limp_gating_set_protect_disable(uint16_t mask) noexcept;
// true se o corte de limp a 3000 rpm está desligado pelo PROTECT (kProtectDisLimpRpm).
bool limp_gating_limp_rpm_disabled() noexcept;

#if defined(EMS_HOST_TEST)
// Zera a memória dos cortes (histerese, latches, tempos) entre cenários de teste.
void limp_gating_test_reset() noexcept;
#endif
}  // namespace ems::engine
