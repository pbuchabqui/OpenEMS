/**
 * @file loop_2ms_fuel_ign.cpp
 * @brief Protect + fuel/ign 2 ms + EncFuelIgnPrep. Sem watchdogs, sem EWG.
 */
#include "engine/loop_2ms_fuel_ign.h"

#include "app/ui_protocol.h"
#include "engine/auxiliaries.h"
#include "engine/calibration.h"
#include "engine/cut_reason.h"
#include "engine/diagnostic_manager.h"
#include "engine/etb_control.h"
#include "engine/ecu_sched.h"
#include "engine/enc_cyl_setpoints.h"
#include "engine/engine_config.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/map_estimator.h"
#include "engine/math_utils.h"
#include "engine/misfire_encoder.h"
#include "engine/output_test.h"
#include "engine/quick_crank.h"
#include "engine/spark_skip.h"
#include "engine/table3d.h"
#include "engine/transient_fuel.h"
#include "engine/vehicle_inputs.h"
#include "hal/timer.h"

#include <cstdint>

using ems::engine::clamp_u16;
using ems::engine::clamp_i16;

extern uint16_t g_dbg_dead_time_us;
extern uint32_t g_dbg_pulse_pw_us;
extern uint32_t g_dbg_cycle_pw_us;
extern uint8_t  g_dbg_squirts;
extern uint32_t g_dbg_rev_limit_trips;
extern uint32_t g_dbg_rev_limit_rpm_x10;
extern uint32_t g_dbg_rev_limit_rpm_max;

int8_t   g_last_advance_deg = 0;
int16_t  g_torque_spark_retard_deg = 0;
uint8_t  g_last_pw_ms_x10 = 0u;
int8_t   g_last_stft_pct = 0;
uint8_t  g_last_lambda_target_d4 = 0u;
uint16_t g_last_map_fused_x100 = 0u;
int8_t   g_last_ltft_pct = 0;
uint32_t g_last_net_pw_us = 0u;
bool     g_limp_active = false;
bool     g_rev_limit_active = false;

namespace {

constexpr uint8_t  kFaultBitMap  = (1u << 0u);
constexpr uint8_t  kFaultBitClt  = (1u << 3u);
constexpr uint8_t  kFaultBitFuel = (1u << 6u);
constexpr uint8_t  kFaultBitOil  = (1u << 7u);
constexpr uint32_t kOilProtectRpmX10  = 15000u;
constexpr uint32_t kFuelRailMinRpmX10 = 5000u;
constexpr int16_t  kOvertempWarnX10   = 1050;
constexpr int16_t  kOvertempCritX10   = 1150;
constexpr uint32_t kSchedulerTicksPerMs = 62500u;
constexpr uint16_t kAePeriodMs = 2u;
uint16_t g_prev_tps_pct_x10 = 0u;

struct CachedFuelCorrections {
    bool valid;
    int16_t clt_x10;
    int16_t iat_x10;
    uint16_t vbatt_mv;
    uint16_t corr_clt_x256;
    uint16_t corr_iat_x256;
    uint16_t dead_time_us;
};

CachedFuelCorrections g_fuel_corr_cache = {};

const CachedFuelCorrections& fuel_corrections_for(
    const ems::drv::SensorData& sensors) noexcept
{
    if (!g_fuel_corr_cache.valid ||
        g_fuel_corr_cache.clt_x10 != sensors.clt_degc_x10 ||
        g_fuel_corr_cache.iat_x10 != sensors.iat_degc_x10 ||
        g_fuel_corr_cache.vbatt_mv != sensors.vbatt_mv) {
        g_fuel_corr_cache.valid = true;
        g_fuel_corr_cache.clt_x10 = sensors.clt_degc_x10;
        g_fuel_corr_cache.iat_x10 = sensors.iat_degc_x10;
        g_fuel_corr_cache.vbatt_mv = sensors.vbatt_mv;
        g_fuel_corr_cache.corr_clt_x256 = ems::engine::corr_clt(sensors.clt_degc_x10);
        g_fuel_corr_cache.corr_iat_x256 = ems::engine::corr_iat(sensors.iat_degc_x10);
        g_fuel_corr_cache.dead_time_us = ems::engine::corr_vbatt(sensors.vbatt_mv);
    }
    return g_fuel_corr_cache;
}

}  // namespace

void loop_2ms_fuel_ign(uint32_t now,
                       const ems::drv::CkpSnapshot& snap,
                       const ems::drv::SensorData& sensors) noexcept
{
    const bool full_sync = (snap.state == ems::drv::SyncState::FULL_SYNC);
    const bool sched_sync = (snap.state == ems::drv::SyncState::HALF_SYNC || full_sync);

    const bool map_fault = (sensors.fault_bits & kFaultBitMap) != 0u;
    const uint16_t map_bar_x100_raw = static_cast<uint16_t>(sensors.map_bar_x1000 / 10u);
    const uint16_t map_bar_x100_sensor = clamp_u16(map_bar_x100_raw, kMapMinBarX100, kMapMaxBarX100);
    // Throttle signal for manifold model: ETB blade if harness present, else APP.
    const uint16_t tps_for_map = (ems::engine::etb_harness_present != 0u)
        ? sensors.etb_tps_pct_x10
        : sensors.app_pct_x10;
    // Fusion: sensor_valid=false on MAP fault so fallback 1 bar is not trusted.
    const uint16_t map_bar_x100 = ems::engine::map_estimator_update(
        map_bar_x100_sensor,
        tps_for_map,
        kAePeriodMs,
        snap.rpm_x10,
        sensors.iat_degc_x10,
        !map_fault);
    const bool clt_fault = (sensors.fault_bits & kFaultBitClt) != 0u;
    const bool oil_fault = (sensors.fault_bits & kFaultBitOil) != 0u;
    const bool fuel_press_fault = (sensors.fault_bits & kFaultBitFuel) != 0u;
    // Overtemp from CLT value (open/short already covered by clt_fault).
    const bool overtemp_warn = sensors.clt_degc_x10 >= kOvertempWarnX10;
    const bool overtemp_crit = sensors.clt_degc_x10 >= kOvertempCritX10;
    if (overtemp_crit) {
        ems::engine::DiagnosticManager::report_fault(
            ems::engine::DiagnosticCode::OVERTEMP_CRITICAL,
            ems::engine::FaultSeverity::CRITICAL,
            static_cast<uint16_t>(sensors.clt_degc_x10), 0u);
    } else if (overtemp_warn) {
        ems::engine::DiagnosticManager::report_fault(
            ems::engine::DiagnosticCode::OVERTEMP_WARNING,
            ems::engine::FaultSeverity::WARNING,
            static_cast<uint16_t>(sensors.clt_degc_x10), 0u);
    } else {
        ems::engine::DiagnosticManager::clear_fault(
            ems::engine::DiagnosticCode::OVERTEMP_CRITICAL);
        ems::engine::DiagnosticManager::clear_fault(
            ems::engine::DiagnosticCode::OVERTEMP_WARNING);
    }
    const bool diag_critical =
        !ems::engine::DiagnosticManager::is_system_ready();
    g_limp_active = map_fault || clt_fault || oil_fault || overtemp_warn;
    // CLT limp: fuel+ign cut only above kLimpRpmLimit (reduced performance below).
    // MAP fault: always cut fuel — fallback MAP≈1 bar is unsafe load for PW at any RPM.
    // Oil range fault while spinning: cut fuel+ign (bearing protection).
    // Fuel-rail range fault after crank: cut fuel only (lean/dry risk).
    // Overtemp critical: fuel+ign cut while spinning (same floor as oil).
    // Fuel angular policy:
    //   (1) FULL_SYNC → running fuel (VE / ASE / semi-seq / sequential)
    //   (2) HALF_SYNC + is_cranking → batch only (simultaneous, crank PW)
    //   (3) else (exit crank, flood, protect, anomaly/no-sync) → inj cut
    const bool rev_cut = g_limp_active &&
        (snap.rpm_x10 > kLimpRpmLimit_x10);
    const bool map_fuel_cut = map_fault;
    const bool oil_protect_cut =
        oil_fault && (snap.rpm_x10 > kOilProtectRpmX10);
    const bool fuel_rail_cut =
        fuel_press_fault && (snap.rpm_x10 > kFuelRailMinRpmX10);
    const bool overtemp_cut =
        overtemp_crit && (snap.rpm_x10 > kOilProtectRpmX10);
    const bool fuel_protect_cut =
        rev_cut || map_fuel_cut || oil_protect_cut || fuel_rail_cut ||
        overtemp_cut || diag_critical;
    const CachedFuelCorrections& fuel_corr = fuel_corrections_for(sensors);
    // Dwell 2D: tensão × RPM (MS42 §2.2.2.2.1).
    // Calculado fora do cache porque depende de RPM que varia a cada dente.
    const uint16_t dwell_ms_x10 = ems::engine::dwell_ms_x10_from_vbatt_rpm(
        sensors.vbatt_mv, snap.rpm_x10);
    const uint32_t dwell_ticks =
        (static_cast<uint32_t>(dwell_ms_x10) * kSchedulerTicksPerMs) / 10u;

    // Multi-spark (MS42 §2.2.3): habilita/desabilita conforme RPM gate.
    // Hard ceiling 1500 RPM (kMsparkRpmCeilingX10) — window too short above.
    // O dwell inter-spark é mais curto (tabela dedicada mspark_inter_dwell_ms_x10).
    // Limite 18°ATDC garante que o último spark contribui para a combustão.
    {
        uint16_t ms_gate = ems::engine::mspark_max_rpm_x10;
        if (ms_gate == 0u || ms_gate > ems::engine::kMsparkRpmCeilingX10) {
            ms_gate = ems::engine::kMsparkRpmCeilingX10;
        }
        if (snap.rpm_x10 < ms_gate && ems::engine::mspark_count > 0u) {
            const uint32_t inter_dwell_ticks =
                (static_cast<uint32_t>(ems::engine::mspark_inter_dwell_ms_x10)
                 * kSchedulerTicksPerMs) / 10u;
            ::ecu_sched_set_mspark(ems::engine::mspark_count, inter_dwell_ticks, 18u);
        } else {
            ::ecu_sched_set_mspark(0u, 0u, 18u);
        }
    }
    // Quick-crank state once per 2 ms tick (HALF + FULL + stopped).
    // Must not be gated on FULL_SYNC fuel — is_cranking() drives presync
    // SIMULTANEOUS, ETB crank open-loop, and HALF batch fuel.
    ems::engine::quick_crank_set_prime_context(sensors.clt_degc_x10,
                                               fuel_corr.dead_time_us);
    // Sem hook por-dente: RPM cru de posição/tempo
    // (quick_crank_encoder_poll).
    ems::engine::quick_crank_encoder_poll(ems::hal::tim2_encoder_count(), now);
    const auto qc = ems::engine::quick_crank_update(
        now, snap.rpm_x10, sched_sync, sensors.clt_degc_x10, 0);
    // Gate closed-loop enrichments during crank + afterstart (not raw RPM).
    const bool crank_or_ase = qc.cranking || qc.afterstart_active;
    const bool flood_clear =
        ems::engine::crank_flood_clear_active(sensors.app_pct_x10);
    const bool half_sync = sched_sync && !full_sync;
    // HALF batch: cranking only, no flood/protect. Auto presync → SIMULTANEOUS.
    const bool allow_half_crank_batch =
        half_sync && qc.cranking && !flood_clear && !fuel_protect_cut;
    // HALF_SYNC não é "posição incerta" — é TIM2 absoluto, sem fase
    // CMP. O auto-select por cranking no tooth hook nunca corre aqui;
    // o modo tem de ser mantido neste slot (achado 2026-08-15).
    ::ecu_sched_set_presync_inj_mode(allow_half_crank_batch
        ? ECU_PRESYNC_INJ_SIMULTANEOUS
        : ECU_PRESYNC_INJ_SEMI_SEQUENTIAL);
    const bool allow_half_running =
        half_sync && !qc.cranking && !flood_clear && !fuel_protect_cut;
    // Lock out angular fuel in HALF unless batch- or running-allowed.
    const bool half_fuel_lockout =
        half_sync && !allow_half_crank_batch && !allow_half_running;

    // Limitador de RPM — rusEFI-style: fuel cut only, total cut + hysteresis.
    // Corta 100% injecção ao atingir hard limit; reativa ao descer
    // abaixo de (hard - hysteresis). IGN nunca é cortada (só limp mode).
    {
        const uint32_t hard      = ems::engine::rev_limit_rpm_x10;
        const uint32_t hyst      = ems::engine::rev_limit_soft_window_x10;
        const uint32_t resume    = (hard > hyst) ? hard - hyst : 0u;

        if (snap.rpm_x10 > g_dbg_rev_limit_rpm_max) {
            g_dbg_rev_limit_rpm_max = snap.rpm_x10;  // pico global (apanha glitch)
        }
        if (snap.rpm_x10 >= hard) {
            if (!g_rev_limit_active) {           // borda de subida = 1 trip
                ++g_dbg_rev_limit_trips;
                g_dbg_rev_limit_rpm_x10 = snap.rpm_x10;  // rpm que disparou
            }
            g_rev_limit_active = true;
        } else if (snap.rpm_x10 <= resume) {
            g_rev_limit_active = false;
        }
        ems::app::ui_set_rev_limit_active(g_rev_limit_active);

        // Spark-skip soft limiter: na janela [hard−window, hard) o
        // ratio rampa 0→max — torque cai progressivamente ANTES do
        // corte duro de fuel (que permanece inalterado no hard).
        {
            const uint16_t win = ems::engine::spark_skip_window_rpm_x10;
            const uint8_t  mx  = ems::engine::spark_skip_max_q8;
            uint8_t ratio = 0u;
            if (win != 0u && mx != 0u && !g_rev_limit_active &&
                hard > win && snap.rpm_x10 >= (hard - win)) {
                const uint32_t into = snap.rpm_x10 - (hard - win);
                ratio = static_cast<uint8_t>(
                    (static_cast<uint32_t>(mx) * into) / win);
            }
            ems::engine::spark_skip_set_ratio_q8(ratio);
            // spark_skip_on_rev() corre no heavy tick do encoder
            // (1×/volta), não inferido de wrap de dente.
        }

        // Duty do injector: estado do tick anterior (o PW final só é
        // conhecido mais abaixo) — 2 ms de latência, irrelevante vs
        // a tolerância de centenas de ms da protecção.
        const bool inj_duty_cut = ems::engine::fuel_inj_duty_cut_active();
        const uint8_t inj_mask =
            (fuel_protect_cut || g_rev_limit_active ||
             half_fuel_lockout || inj_duty_cut) ? 0x0Fu : 0u;
        const uint8_t ign_mask_cut =
            (rev_cut || oil_protect_cut || overtemp_cut ||
             diag_critical) ? 0x0Fu : 0u;
        const uint8_t ign_mask = static_cast<uint8_t>(
            ign_mask_cut | ems::engine::spark_skip_mask());
        ::ecu_sched_set_inj_inhibit_mask(inj_mask);
        ::ecu_sched_set_ign_inhibit_mask(ign_mask);

        // Razões tipadas de corte (cut_reason.h) — telemetria 'D' [51].
        // DFCO é decidido mais abaixo no caminho FULL_SYNC (OR posterior).
        uint16_t fr = 0u;
        if (g_rev_limit_active) fr |= ems::engine::kFuelCutRevLimit;
        if (rev_cut)            fr |= ems::engine::kFuelCutLimpRpm;
        if (map_fuel_cut)       fr |= ems::engine::kFuelCutMapFault;
        if (oil_protect_cut)    fr |= ems::engine::kFuelCutOilPress;
        if (fuel_rail_cut)      fr |= ems::engine::kFuelCutFuelRail;
        if (overtemp_cut)       fr |= ems::engine::kFuelCutOvertemp;
        if (diag_critical)      fr |= ems::engine::kFuelCutDiagCrit;
        if (half_fuel_lockout)  fr |= ems::engine::kFuelCutNoSync;
        if (flood_clear)        fr |= ems::engine::kFuelCutFlood;
        if (inj_duty_cut)       fr |= ems::engine::kFuelCutInjDuty;
        uint16_t sr = 0u;
        if (rev_cut)          sr |= ems::engine::kSparkCutLimpRpm;
        if (oil_protect_cut)  sr |= ems::engine::kSparkCutOilPress;
        if (overtemp_cut)     sr |= ems::engine::kSparkCutOvertemp;
        if (diag_critical)    sr |= ems::engine::kSparkCutDiagCrit;
        if (ems::engine::spark_skip_mask() != 0u) {
            sr |= ems::engine::kSparkSkipActive;
        }
        ems::engine::g_fuel_cut_reasons  = fr;
        ems::engine::g_spark_cut_reasons = sr;
    }

    // Telemetry PW must match actuators: only when injectors are actually cut.
    const bool fuel_cut_active =
        g_rev_limit_active || fuel_protect_cut || half_fuel_lockout ||
        ems::engine::fuel_inj_duty_cut_active();

    // (1) FULL_SYNC (ou HALF_SYNC fora de cranking em encoder, ver
    // allow_half_running acima): running fuel path (VE / trims / AE /
    // X-τ when not crank-ASE).
    if ((full_sync || allow_half_running) && !fuel_protect_cut) {
        const ems::engine::Table2dLookup fuel_lookup =
            ems::engine::table3d_prepare_lookup(ems::engine::kRpmAxisX10,
                                                ems::engine::kLoadAxisBarX100,
                                                snap.rpm_x10,
                                                map_bar_x100);
        const uint8_t  ve = ems::engine::get_ve_prepared(fuel_lookup);
        const uint16_t lambda_target_x1000 =
            ems::engine::get_lambda_target_x1000_prepared(fuel_lookup);
        // LTFT apply = nearest cell (mesma política que crédito/store LEARN).
        // fuel_lookup.yi/xi são floor da bilineal VE — mid-bin errava a célula.
        const int16_t fuel_trim_pct_x10 = crank_or_ase ? 0 : clamp_i16(
            static_cast<int16_t>(ems::engine::fuel_get_stft_pct_x10() +
                                 ems::engine::fuel_get_ltft_at(snap.rpm_x10, map_bar_x100)),
            -500, 500);
        // AE/DE from map-fusion TPSdot (signed: tip-in >0, tip-out <0).
        const int16_t ae_tpsdot = ems::engine::map_get_tpsdot_x10();
        int32_t ae_pw_us = crank_or_ase ? 0
            : ems::engine::calc_ae_pw_from_tpsdot(ae_tpsdot, sensors.clt_degc_x10);
        uint32_t final_pw_us_base =
            ems::engine::calc_fuel_pw_us_default_fast(ve,
                                                       map_bar_x100,
                                                       lambda_target_x1000,
                                                       fuel_trim_pct_x10,
                                                       fuel_corr.corr_clt_x256,
                                                       fuel_corr.corr_iat_x256,
                                                       fuel_corr.dead_time_us);
        // Corte de combustível na desaceleração (MS42 TI_PUR).
        // Avaliado ANTES do X-Tau: evita alimentar o modelo de parede com PW
        // real e depois descartar o resultado, contaminando a auto-calibração.
        // Contexto DFCO: MAP p/ o gate de vácuo e marcha p/ inibição
        // pós-troca (ambos inertes com as respectivas cals a 0).
        ems::engine::fuel_decel_cut_notify_map(map_bar_x100);
        {
            uint8_t gr = 0u;
            if (ems::engine::vehicle_gear(gr, now)) {
                ems::engine::fuel_decel_cut_notify_gear(gr, now);
            }
        }
        const bool decel_cut_active = !crank_or_ase &&
            ems::engine::fuel_decel_cut_update(
                snap.rpm_x10, sensors.etb_tps_pct_x10, sensors.clt_degc_x10);
        ems::engine::misfire_encoder_set_all_inhibit(
            decel_cut_active || crank_or_ase || flood_clear);
        // X-τ desde !cranking (inclui afterstart frio — pior wall-wetting).
        // AE residual a 50% quando X-τ activo (evita empilhar enrich).
        const bool xtau_enabled = !qc.cranking;
        if (xtau_enabled && ae_pw_us > 0) {
            ae_pw_us /= 2;
        }
        if (decel_cut_active) {
            ems::engine::g_fuel_cut_reasons = static_cast<uint16_t>(
                ems::engine::g_fuel_cut_reasons | ems::engine::kFuelCutDfco);
            g_last_net_pw_us = 0u;
            ems::engine::fuel_ae_notify_pulse(0);
            // Filme: reset só na entrada (não a cada tick do cut).
            if (ems::engine::fuel_decel_cut_just_entered()) {
                ems::engine::transient_fuel_reset();
            }
        } else if (final_pw_us_base > fuel_corr.dead_time_us) {
            uint32_t fuel_pw_us =
                final_pw_us_base - static_cast<uint32_t>(fuel_corr.dead_time_us);

            // LTFT aditivo (MS42 TI_AD_ADD_MMV): offset no PW líquido, célula nearest
            if (!crank_or_ase) {
                const int16_t ltft_add =
                    ems::engine::fuel_get_ltft_add_at(snap.rpm_x10, map_bar_x100);
                const int32_t pw_adj = static_cast<int32_t>(fuel_pw_us) + ltft_add;
                fuel_pw_us = (pw_adj <= 0) ? 0u
                           : (pw_adj > 100000) ? 100000u
                           : static_cast<uint32_t>(pw_adj);
            }
            g_last_net_pw_us = fuel_pw_us;

            // X-τ de produção avança por evento de spray no finalize
            // (por cilindro). Loop 2 ms só mantém telemetria/learn.
            final_pw_us_base = fuel_pw_us;
        } else {
            ems::engine::transient_fuel_reset();
            final_pw_us_base = 0u;  // fluxo ≈ 0 (base ≤ dead-time)
        }
        // Fluxo base (antes do AE) para o prep por cilindro.
        const uint32_t base_flow_before_ae = final_pw_us_base;
        // AE tip-in (add) ou DE tip-out (subtract), clamp a [0, 100ms].
        // Encoder: AE também vai no prep p/ depósito no evento de spray;
        // no commit 2 ms ainda soma p/ telemetria/duty.
        if (!decel_cut_active && ae_pw_us != 0) {
            const int64_t adj = static_cast<int64_t>(final_pw_us_base) + ae_pw_us;
            if (adj <= 0) {
                final_pw_us_base = 0u;
            } else if (adj > 100000) {
                final_pw_us_base = 100000u;
            } else {
                final_pw_us_base = static_cast<uint32_t>(adj);
            }
        }
        // Soft ramp-in pós-DFCO (antes de quick_crank / ΔP).
        if (!decel_cut_active) {
            final_pw_us_base =
                ems::engine::fuel_decel_cut_ramp_pw(final_pw_us_base, 2u);
        }
        // Sempre: tip-in (µs>0) freezes STFT; pulse==0 limpa no próprio 2 ms
        // (antes ficava sticky até ao slot 100 ms).
        if (!decel_cut_active) {
            ems::engine::fuel_ae_notify_pulse(ae_pw_us);
        }
        const int16_t base_advance_deg = ems::engine::get_advance_prepared(fuel_lookup);
        // Máximo entre os 4 cilindros, não só o cilindro 0 (FIX:
        // knock_retard_x10[] é genuinamente por cilindro, mas este
        // valor é aplicado como escalar único e partilhado a todos
        // os cilindros abaixo — máximo é a escolha conservadora,
        // nunca sub-retarda o cilindro que mais precisa. Retard
        // verdadeiramente por cilindro precisa de infra-estrutura
        // nova que não existe hoje — ver AdvanceCorrections/
        // calc_total_advance, escalar único, fora de escopo aqui).
        uint16_t knock_retard_x10 = 0u;
        for (uint8_t kc = 0u; kc < 4u; ++kc) {
            const uint16_t r = ems::engine::knock_get_retard_x10(kc);
            if (r > knock_retard_x10) { knock_retard_x10 = r; }
        }
        const uint16_t idle_target_rpm_x10 =
            ems::engine::auxiliaries_idle_target_rpm_x10(sensors.clt_degc_x10);
        // Idle spark OK during afterstart (helps settle); suppressed only while cranking.
        const int16_t idle_spark_corr_deg = qc.cranking ? 0 :
            ems::engine::calc_idle_spark_correction_deg(snap.rpm_x10,
                                                        idle_target_rpm_x10,
                                                        sensors.etb_tps_pct_x10,
                                                        map_bar_x100);
        const int16_t iat_spark_deg = qc.cranking ? 0 :
            ems::engine::calc_ign_iat_correction_deg(sensors.iat_degc_x10);
        const int16_t clt_spark_deg = qc.cranking ? 0 :
            ems::engine::calc_ign_clt_correction_deg(sensors.clt_degc_x10);
        const int16_t antijerk_retard = crank_or_ase ? 0 :
            ems::engine::calc_antijerk_retard_deg(ae_tpsdot);
        const int16_t advance_deg = ems::engine::calc_total_advance(
            base_advance_deg,
            {iat_spark_deg, clt_spark_deg,
             static_cast<int16_t>(knock_retard_x10 / 10u),
             idle_spark_corr_deg, antijerk_retard,
             g_torque_spark_retard_deg});
        // Cranking spark from qc (base was 0 at update); else table+corr.
        const int16_t sched_spark_deg = qc.cranking
            ? ems::engine::crank_spark_deg
            : advance_deg;
        // Decel / flood: force PW=0 (do not apply min_pw floor).
        const uint32_t quick_crank_pw_us =
            (decel_cut_active || flood_clear) ? 0u :
            ems::engine::quick_crank_apply_pw_us(final_pw_us_base,
                                                 qc.fuel_mult_x256,
                                                 qc.min_pw_us);
        // Correções físicas finais do bico: pressão diferencial de combustível
        // (real, via sensor) e não-linearidade de abertura em PW pequeno.
        // Aplicam-se apenas ao fluxo; o dead-time entra DEPOIS, sem escalar,
        // e só quando há fluxo (PW=0 em corte não ganha dead-time).
        const uint32_t delta_p_pw_us = ems::engine::apply_delta_p_compensation(
            quick_crank_pw_us, sensors.fuel_press_bar_x1000, map_bar_x100);
        const uint32_t scurve_pw_us = ems::engine::apply_injector_scurve(delta_p_pw_us);
        // Formula is flow + dead×openings (seq: +1 dead, semi: +2).
        // The pin splits that total across the openings; the gauge
        // shows the formula, not one opening.
        const uint8_t squirts = ::ecu_sched_is_sequential() ? 1u : 2u;
        const uint32_t cycle_on_us = ems::engine::inj_cycle_pw_us(
            scurve_pw_us, fuel_corr.dead_time_us, squirts);
        const uint32_t pulse_pw_us = ems::engine::inj_pulse_pw_us(
            scurve_pw_us, fuel_corr.dead_time_us, squirts);
        g_dbg_dead_time_us = fuel_corr.dead_time_us;
        g_dbg_pulse_pw_us  = pulse_pw_us;
        g_dbg_cycle_pw_us  = cycle_on_us;
        g_dbg_squirts      = squirts;
        // Com fuel cut (rev limiter/limp) os injectores estão inibidos pela
        // mask — a telemetria (dash/CAN) tem de mostrar 0, não o PW calculado
        // que continua a ser comitado para retoma suave.
        const uint32_t pw_100 = cycle_on_us / 100u;
        g_last_pw_ms_x10 = fuel_cut_active ? 0u
            : static_cast<uint8_t>(pw_100 > 255u ? 255u : pw_100);
        g_last_advance_deg = clamp_i8(sched_spark_deg, -10, 40);

        // Protecção de duty (FOME #215): alimenta com o PW final
        // comandado; o corte em si entra na mask do próximo tick.
        //
        // LIMITE CONHECIDO: cycle_on_us usa o MAP
        // FUNDIDO do motor inteiro (map_bar_x100). Quando o slot de MAP
        // por-cilindro está válido (map_window_slot_valid_for_cyl(),
        // ver enc_cyl_setpoints.cpp:finalize_cyl_setpoints()), o PW
        // REALMENTE armado no injetor usa esse MAP por-cilindro, que
        // pode ser maior num motor com pulsação de admissão real — a
        // proteção fica PERMISSIVA nesse caso (sub-reporta o duty
        // real), nunca conservadora. Documentado, não corrigido nesta
        // revisão — mudar a fonte do valor vigiado é uma alteração de
        // superfície de proteção de hardware, precisa de desenho e
        // validação em bancada à parte.
        ems::engine::fuel_inj_duty_update(cycle_on_us, snap.rpm_x10, 2u);

        const uint32_t inj_pw_ticks = ems::engine::inj_pw_us_to_scheduler_ticks(pulse_pw_us);
        const uint32_t eoi_lead =
            static_cast<uint32_t>(ems::engine::calc_eoi_lead_deg(
                snap.rpm_x10, sensors.clt_degc_x10));

        {
            ems::engine::EncFuelIgnPrep prep{};
            prep.valid = 1u;
            prep.fuel_cut = static_cast<uint8_t>(
                fuel_cut_active || decel_cut_active || flood_clear);
            prep.cranking = static_cast<uint8_t>(qc.cranking);
            prep.xtau_event_enable = static_cast<uint8_t>(xtau_enabled ? 1u : 0u);
            prep.map_bar_x100 = map_bar_x100;
            prep.fuel_press_bar_x1000 = sensors.fuel_press_bar_x1000;
            prep.dead_time_us = fuel_corr.dead_time_us;
            prep.rpm_x10 = snap.rpm_x10;
            prep.corr_clt_x256 = fuel_corr.corr_clt_x256;
            prep.corr_iat_x256 = fuel_corr.corr_iat_x256;
            prep.fuel_trim_pct_x10 = fuel_trim_pct_x10;
            prep.clt_x10 = sensors.clt_degc_x10;
            // Base sem AE (quick_crank sobre fluxo pré-AE); AE residual
            // separado p/ depósito no filme do cyl no finalize.
            const uint32_t base_qc = (decel_cut_active || flood_clear) ? 0u
                : ems::engine::quick_crank_apply_pw_us(base_flow_before_ae,
                                                       qc.fuel_mult_x256,
                                                       qc.min_pw_us);
            prep.base_flow_pw_us = base_qc;
            prep.ae_pw_us = (decel_cut_active || flood_clear)
                ? 0 : ae_pw_us;
            prep.flow_pw_us = quick_crank_pw_us;
            prep.base_advance_deg = base_advance_deg;
            prep.crank_spark_deg = ems::engine::crank_spark_deg;
            prep.iat_spark_deg = iat_spark_deg;
            prep.clt_spark_deg = clt_spark_deg;
            prep.idle_spark_deg = idle_spark_corr_deg;
            prep.antijerk_retard_deg = antijerk_retard;
            prep.torque_retard_deg = g_torque_spark_retard_deg;
            prep.dwell_ticks = dwell_ticks;
            prep.eoi_lead_deg = eoi_lead;
            ems::engine::enc_fuel_ign_prep_publish(prep);
        }

        ::ecu_sched_commit_calibration(
            static_cast<uint32_t>(sched_spark_deg < 0 ? 0 : sched_spark_deg),
            dwell_ticks,
            inj_pw_ticks,
            eoi_lead);
    } else if (allow_half_crank_batch) {
        // (2) HALF_SYNC + cranking: simultaneous batch, crank PW only (no VE/STFT/AE).
        // Presync auto already selects SIMULTANEOUS while is_cranking().
        // Force mode in case auto was off or race with tooth ISR.
        ::ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
        ems::engine::misfire_encoder_set_all_inhibit(true);
        ems::engine::fuel_ae_notify_pulse(0);
        ems::engine::transient_fuel_reset();

        const uint32_t req_us = ems::engine::default_req_fuel_us();
        const uint32_t crank_flow_us = ems::engine::quick_crank_apply_pw_us(
            req_us, qc.fuel_mult_x256, qc.min_pw_us);
        g_last_net_pw_us = crank_flow_us;
        const uint32_t delta_p_pw_us = ems::engine::apply_delta_p_compensation(
            crank_flow_us, sensors.fuel_press_bar_x1000, map_bar_x100);
        const uint32_t scurve_pw_us = ems::engine::apply_injector_scurve(delta_p_pw_us);
        // Cranking batch is simultaneous: formula flow+2×dead.
        const uint32_t cycle_on_us = ems::engine::inj_cycle_pw_us(
            scurve_pw_us, fuel_corr.dead_time_us, 2u);
        const uint32_t pulse_pw_us = ems::engine::inj_pulse_pw_us(
            scurve_pw_us, fuel_corr.dead_time_us, 2u);
        g_dbg_dead_time_us = fuel_corr.dead_time_us;
        g_dbg_pulse_pw_us  = pulse_pw_us;
        g_dbg_cycle_pw_us  = cycle_on_us;
        g_dbg_squirts      = 2u;
        const uint32_t pw_100 = cycle_on_us / 100u;
        g_last_pw_ms_x10 = fuel_cut_active ? 0u
            : static_cast<uint8_t>(pw_100 > 255u ? 255u : pw_100);
        const int16_t sched_spark_deg = ems::engine::crank_spark_deg;
        g_last_advance_deg = clamp_i8(sched_spark_deg, -10, 40);
        const uint32_t inj_pw_ticks =
            ems::engine::inj_pw_us_to_scheduler_ticks(pulse_pw_us);
        ::ecu_sched_commit_calibration(
            static_cast<uint32_t>(sched_spark_deg < 0 ? 0 : sched_spark_deg),
            dwell_ticks,
            inj_pw_ticks,
            static_cast<uint32_t>(ems::engine::calc_eoi_lead_deg(
                snap.rpm_x10, sensors.clt_degc_x10)));
    } else if (sched_sync &&
               (fuel_protect_cut || half_fuel_lockout || g_rev_limit_active)) {
        // (3) Spark-only: exit-crank HALF, flood, protect, rev-limit, anomaly path.
        // qc already updated — use crank spark only while still latched cranking.
        const int16_t base_advance_deg = ems::engine::get_advance(snap.rpm_x10, map_bar_x100);
        const int16_t sched_spark_deg = qc.cranking
            ? ems::engine::crank_spark_deg
            : base_advance_deg;
        ::ecu_sched_commit_calibration(
            static_cast<uint32_t>(sched_spark_deg < 0 ? 0 : sched_spark_deg),
            dwell_ticks,
            0u,
            static_cast<uint32_t>(ems::engine::calc_eoi_lead_deg(
                snap.rpm_x10, sensors.clt_degc_x10)));
        g_last_pw_ms_x10 = 0u;
        g_last_net_pw_us = 0u;
        g_last_advance_deg = clamp_i8(sched_spark_deg, -10, 40);
        ems::engine::fuel_ae_notify_pulse(0);
    }
    // Sempre que o branch (1) acima NÃO publicou (fuel_protect_cut,
    // HALF sem allow_half_running, ou sync perdido de todo) —
    // fora do if/else-if de propósito, não só dentro do branch (3) —
    // republica o prep. Sem isto, o dispatcher sequencial
    // (try_arm_sequential_due, ecu_sched_encoder_builders.cpp) continua
    // a ler o prep congelado de ANTES do corte e arma injeção real
    // indefinidamente: nem g_inj_inhibit_mask nem fuel_protect_cut
    // são consultados no caminho de disparo do encoder
    // (enc_evt_execute_head chama out_pin_write diretamente), e
    // ecu_sched_encoder_omega_valid() não depende de sched_sync —
    // fica "válido" mesmo com sync total perdido, então o branch (3)
    // sozinho (gated em sched_sync) não bastava. Achado 2026-08-15:
    // LEDs INJ a piscar continuamente com sensor_fault_bits!=0 e
    // pw_ms=0 na telemetria (outro global, não o que este
    // dispatcher realmente arma).
    //
    // Recalcula avanço/dwell/eoi_lead/cranking aqui também (achado
    // #2 da revisão 2026-08-15, mesma classe de bug do fuel_cut,
    // desta vez para a faísca): deixar esses campos apenas
    // "preservados" do último branch (1) parecia inofensivo para um
    // corte breve, mas com fuel_protect_cut/half_fuel_lockout
    // sustentados por segundos o dwell ficava errado para o
    // RPM/tensão atuais e o avanço não refletia mais IAT/CLT/idle
    // reais. Mesmos termos que o branch (3) usa (spark-only) — só
    // avanço base + crank spark, sem as correções finas de IAT/CLT/
    // idle/antijerk/torque (essas só existem dentro do pipeline
    // completo do branch (1); zeradas aqui tal como o branch (3) já
    // as omite do commit legado, não é uma redução de segurança
    // nova).
    if (!((full_sync || allow_half_running) && !fuel_protect_cut)) {
        const int16_t base_advance_deg =
            ems::engine::get_advance(snap.rpm_x10, map_bar_x100);
        ems::engine::EncFuelIgnPrep prep = ems::engine::enc_fuel_ign_prep_read();
        prep.valid = 1u;
        prep.fuel_cut = 1u;
        prep.cranking = static_cast<uint8_t>(qc.cranking);
        prep.base_advance_deg = base_advance_deg;
        prep.crank_spark_deg = ems::engine::crank_spark_deg;
        prep.iat_spark_deg = 0;
        prep.clt_spark_deg = 0;
        prep.idle_spark_deg = 0;
        prep.antijerk_retard_deg = 0;
        prep.torque_retard_deg = 0;
        prep.dwell_ticks = dwell_ticks;
        prep.eoi_lead_deg =
            static_cast<uint32_t>(ems::engine::calc_eoi_lead_deg(
                snap.rpm_x10, sensors.clt_degc_x10));
        ems::engine::enc_fuel_ign_prep_publish(prep);
    }
    g_prev_tps_pct_x10 = sensors.etb_tps_pct_x10;
    ems::app::ui_update_rt_map_fuel(map_bar_x100, g_last_net_pw_us);
    g_last_map_fused_x100 = map_bar_x100;

    // Prime one-shot: suppressed on flood-clear, fuel-protect (MAP/oil/rail/
    // overtemp/diag/rev limp), and whenever inj mask already locks all cyls.
    // force_output also honors the mask (defense in depth vs bypass).
    const bool prime_blocked =
        flood_clear || fuel_protect_cut || half_fuel_lockout ||
        g_rev_limit_active;
    const uint32_t prime_pw = prime_blocked
        ? 0u
        : ems::engine::quick_crank_consume_prime();
    if (prime_pw != 0u && !ems::engine::output_test_active()) {
        ::ecu_sched_fire_prime_pulse(prime_pw);
    } else if (prime_blocked) {
        // Drop pending prime so protect/flood cannot fire after condition clears mid-tooth.
        static_cast<void>(ems::engine::quick_crank_consume_prime());
    }
}
