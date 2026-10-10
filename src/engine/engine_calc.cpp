#include "engine/engine_calc.h"

#include "engine/auxiliaries.h"
#include "engine/calibration.h"
#include "engine/cut_reason.h"
#include "engine/diagnostic_manager.h"
#include "engine/ecu_sched.h"
#include "engine/fuel_calc.h"
#include "engine/fuel_trim.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/limp_gating.h"
#include "engine/map_estimator.h"
#include "engine/map_window.h"
#include "engine/math_utils.h"
#include "engine/misfire_detect.h"
#include "engine/output_test.h"
#include "engine/quick_crank.h"
#include "engine/spark_skip.h"
#include "engine/table3d.h"
#include "engine/transient_fuel.h"
#include "engine/vehicle_inputs.h"
#include "engine/xtau_autocalib.h"

namespace ems::engine {
namespace {

constexpr uint32_t kLimpRpmLimit_x10 = 30000u;
constexpr uint8_t  kFaultBitMap  = (1u << 0u);  // SensorId::MAP
constexpr uint8_t  kFaultBitClt  = (1u << 3u);  // SensorId::CLT
constexpr uint8_t  kFaultBitFuel = (1u << 6u);  // SensorId::FUEL_PRESS
constexpr uint8_t  kFaultBitOil  = (1u << 7u);  // SensorId::OIL_PRESS
// Coolant protection (value-based, not just open/short fault_bits).
constexpr int16_t  kOvertempWarnX10 = 1050;    // 105 °C
constexpr int16_t  kOvertempCritX10 = 1150;    // 115 °C
constexpr uint32_t kSchedulerTicksPerMs = 62500u;  // TIM5_CNT @ 62.5 MHz
constexpr uint16_t kMapMinBarX100 = 10u;
constexpr uint16_t kMapMaxBarX100 = 300u;
constexpr uint16_t kAePeriodMs = 2u;

EngineCalcOut s_out = {};

struct CachedFuelCorrections {
    bool valid;
    int16_t clt_x10;
    int16_t iat_x10;
    uint16_t vbatt_mv;
    uint16_t corr_clt_x256;
    uint16_t corr_iat_x256;
    uint16_t iat_density_q8;
    uint16_t dead_time_us;
};

CachedFuelCorrections g_fuel_corr_cache = {};

const CachedFuelCorrections& fuel_corrections_for(const ems::drv::SensorData& sensors) noexcept {
    if (!g_fuel_corr_cache.valid ||
        g_fuel_corr_cache.clt_x10 != sensors.clt_degc_x10 ||
        g_fuel_corr_cache.iat_x10 != sensors.iat_degc_x10 ||
        g_fuel_corr_cache.vbatt_mv != sensors.vbatt_mv) {
        g_fuel_corr_cache.valid = true;
        g_fuel_corr_cache.clt_x10 = sensors.clt_degc_x10;
        g_fuel_corr_cache.iat_x10 = sensors.iat_degc_x10;
        g_fuel_corr_cache.vbatt_mv = sensors.vbatt_mv;
        g_fuel_corr_cache.corr_clt_x256 = corr_clt(sensors.clt_degc_x10);
        g_fuel_corr_cache.corr_iat_x256 = corr_iat(sensors.iat_degc_x10);
        g_fuel_corr_cache.iat_density_q8 = corr_iat_density_q8(sensors.iat_degc_x10);
        g_fuel_corr_cache.dead_time_us = corr_vbatt(sensors.vbatt_mv);
    }
    return g_fuel_corr_cache;
}

// Final injector PW: dP + S-curve act on flow only; dead-time is added after
// (flow + dead x squirts) and never scaled. Telemetry shows 0 under fuel cut
// (mask inhibits the injectors; PW keeps being committed for smooth resume).
// Returns the per-opening pulse in scheduler ticks.
uint32_t inj_finish_pw(uint32_t flow_us, uint8_t squirts, uint16_t dead_us,
                       const ems::drv::SensorData& sensors,
                       uint16_t map_bar_x100, bool fuel_cut_active,
                       uint32_t& cycle_on_us) noexcept {
    // dP is linear in time (whole cycle); the injector non-linearity acts on
    // each opening, which is what the injector actually sees.
    const uint8_t n = (squirts < 1u) ? 1u : squirts;
    const uint32_t dp_flow_us = apply_delta_p_compensation(
        flow_us,
        ((sensors.fault_bits & kFaultBitFuel) != 0u) ? 0u : sensors.fuel_press_bar_x1000,
        map_bar_x100);
    const uint32_t scurve_pw_us = apply_injector_scurve(dp_flow_us / n) * n;
    cycle_on_us = inj_cycle_pw_us(scurve_pw_us, dead_us, squirts);
    const uint32_t pw_100 = cycle_on_us / 100u;
    s_out.pw_ms_x10 = fuel_cut_active ? 0u
        : static_cast<uint8_t>(pw_100 > 255u ? 255u : pw_100);
    return inj_pw_us_to_scheduler_ticks(inj_pulse_pw_us(scurve_pw_us, dead_us, squirts));
}

void commit(int16_t spark_x10, uint32_t dwell_ticks, uint32_t inj_pw_ticks,
            const ems::drv::CkpSnapshot& snap,
            const ems::drv::SensorData& sensors) noexcept {
    // Knock retard is per cylinder (each coil retards on its own); none in
    // timing-light mode, the strobe must see the fixed advance.
    uint16_t knock_x10[4];
    for (uint8_t c = 0u; c < 4u; ++c) {
        knock_x10[c] = (timing_light_enable != 0u) ? 0u : knock_get_retard_x10(c);
    }
    ::ecu_sched_set_cyl_retard_x10(knock_x10);
    ::ecu_sched_commit_calibration_x10(
        spark_x10, dwell_ticks, inj_pw_ticks,
        static_cast<uint32_t>(calc_eoi_lead_deg(snap.rpm_x10, sensors.clt_degc_x10)));
    s_out.committed = true;
    s_out.spark_x10 = spark_x10;
    s_out.dwell_ticks = dwell_ticks;
    s_out.inj_pw_ticks = inj_pw_ticks;
}

}  // namespace

void engine_calc_reset() noexcept {
    s_out = {};
    g_fuel_corr_cache = {};
}

const EngineCalcOut& engine_calc_step(const EngineCalcIn& in) noexcept {
    const ems::drv::CkpSnapshot& snap = in.snap;
    const ems::drv::SensorData& sensors = in.sensors;
    s_out.committed = false;

    const bool full_sync = (snap.state == ems::drv::SyncState::FULL_SYNC);
    const bool sched_sync = (snap.state == ems::drv::SyncState::HALF_SYNC || full_sync);

    const bool map_fault = (sensors.fault_bits & kFaultBitMap) != 0u;
    // MAP "sensor": por padrão o IIR ao vivo de sensors.cpp (suaviza
    // no tempo, não sincronizado ao ângulo de admissão). Quando
    // map_window_use_for_fuel=1 (opt-in separado, exige calibração
    // prévia de open_deg/len_deg — ver AVISO em map_window.h) E sync
    // pleno/cam confirmada NO INSTANTE ATUAL (não só quando a janela
    // fechou — evita servir média congelada após perda de sync) E
    // já há pelo menos 1 ciclo medido, usa a média das 4 janelas
    // angulares por cilindro em vez do IIR — livre de ripple de
    // pulso síncrono ao motor, ao custo de até ~1 ciclo de lag
    // (aceitável: map_estimator já reduz o peso do "sensor" durante
    // transientes detectados via TPSdot, a favor do modelo).
    uint16_t map_bar_x100_raw = static_cast<uint16_t>(sensors.map_bar_x1000 / 10u);
    if (map_window_use_for_fuel != 0u &&
        map_window_enable != 0u &&
        snap.state == ems::drv::SyncState::FULL_SYNC &&
        snap.cmp_confirms >= 2u &&
        map_window_cycles() > 0u) {
        map_bar_x100_raw = static_cast<uint16_t>(
            map_window_mean_bar_x1000() / 10u);
    }
    const uint16_t map_bar_x100_sensor = clamp_u16(map_bar_x100_raw, kMapMinBarX100, kMapMaxBarX100);
    // Throttle signal for manifold model: ETB blade if harness present, else APP.
    const uint16_t tps_for_map = (etb_harness_present != 0u)
        ? sensors.etb_tps_pct_x10
        : sensors.app_pct_x10;
    // Fusion: sensor_valid=false on MAP fault so fallback 1 bar is not trusted.
    const uint16_t map_bar_x100 = map_estimator_update(
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
        DiagnosticManager::report_fault(
            DiagnosticCode::OVERTEMP_CRITICAL,
            FaultSeverity::CRITICAL,
            static_cast<uint16_t>(sensors.clt_degc_x10), 0u);
    } else if (overtemp_warn) {
        DiagnosticManager::report_fault(
            DiagnosticCode::OVERTEMP_WARNING,
            FaultSeverity::WARNING,
            static_cast<uint16_t>(sensors.clt_degc_x10), 0u);
    } else {
        DiagnosticManager::clear_fault(
            DiagnosticCode::OVERTEMP_CRITICAL);
        DiagnosticManager::clear_fault(
            DiagnosticCode::OVERTEMP_WARNING);
    }
    const bool diag_critical =
        !DiagnosticManager::is_system_ready();
    s_out.limp_active = map_fault || clt_fault || oil_fault || overtemp_warn;
    // Fuel angular policy (cuts live in limp_gating):
    //   (1) FULL_SYNC → running fuel
    //   (2) HALF_SYNC + cranking → batch; Hall does NOT allow HALF running fuel
    //   (3) flood / protect / LOSS_OF_SYNC → inj cut
    const bool limp_rpm_cut = s_out.limp_active &&
        (snap.rpm_x10 > kLimpRpmLimit_x10);
    const CachedFuelCorrections& fuel_corr = fuel_corrections_for(sensors);
    // Dwell 2D: tensão × RPM (MS42 §2.2.2.2.1).
    // Calculado fora do cache porque depende de RPM que varia a cada dente.
    const uint16_t dwell_ms_x10 = dwell_ms_x10_from_vbatt_rpm(
        sensors.vbatt_mv, snap.rpm_x10);
    const uint32_t dwell_ticks =
        (static_cast<uint32_t>(dwell_ms_x10) * kSchedulerTicksPerMs) / 10u;

    // Multi-spark (MS42 §2.2.3): habilita/desabilita conforme RPM gate.
    // Hard ceiling 1500 RPM (kMsparkRpmCeilingX10) — window too short above.
    // O dwell inter-spark é mais curto (tabela dedicada mspark_inter_dwell_ms_x10).
    // Limite 18°ATDC garante que o último spark contribui para a combustão.
    {
        uint16_t ms_gate = mspark_max_rpm_x10;
        if (ms_gate == 0u || ms_gate > kMsparkRpmCeilingX10) {
            ms_gate = kMsparkRpmCeilingX10;
        }
        const bool ms_on = snap.rpm_x10 < ms_gate && mspark_count > 0u;
        ::ecu_sched_set_mspark(
            ms_on ? mspark_count : 0u,
            ms_on ? (static_cast<uint32_t>(mspark_inter_dwell_ms_x10)
                     * kSchedulerTicksPerMs) / 10u : 0u,
            18u);
    }
    // Quick-crank state once per 2 ms tick (HALF + FULL + stopped).
    // Must not be gated on FULL_SYNC fuel — is_cranking() drives presync
    // SIMULTANEOUS, ETB crank open-loop, and HALF batch fuel.
    quick_crank_set_prime_context(sensors.clt_degc_x10,
                                               fuel_corr.dead_time_us);
    const auto qc = quick_crank_update(
        in.now_ms, snap.rpm_x10, sched_sync, sensors.clt_degc_x10, 0);
    // Gate closed-loop enrichments during crank + afterstart (not raw RPM).
    const bool crank_or_ase = qc.cranking || qc.afterstart_active;
    const bool flood_clear =
        crank_flood_clear_active(sensors.app_pct_x10);
    const bool half_sync = sched_sync && !full_sync;

    if (snap.rpm_x10 > s_out.rpm_max_x10) {
        s_out.rpm_max_x10 = snap.rpm_x10;
    }
    {
        const uint32_t hard = rev_limit_rpm_x10;
        const uint16_t win = spark_skip_window_rpm_x10;
        const uint8_t  mx  = spark_skip_max_q8;
        uint8_t ratio = 0u;
        if (win != 0u && mx != 0u && hard > win &&
            snap.rpm_x10 >= (hard - win) && snap.rpm_x10 < hard) {
            const uint32_t into = snap.rpm_x10 - (hard - win);
            ratio = static_cast<uint8_t>(
                (static_cast<uint32_t>(mx) * into) / win);
        }
        spark_skip_set_ratio_q8(ratio);
        static uint16_t s_prev_tooth = 0u;
        if (snap.tooth_index < s_prev_tooth) {
            spark_skip_on_rev();
        }
        s_prev_tooth = snap.tooth_index;
    }

    const bool etb_fault =
        (etb_harness_present != 0u) &&
        ((sensors.throttle_fault_bits &
          (ems::drv::THROTTLE_FAULT_ETB_TPS1 |
           ems::drv::THROTTLE_FAULT_ETB_TPS2 |
           ems::drv::THROTTLE_FAULT_ETB_PLAUS)) != 0u);
    const uint16_t lambda_x1000 = in.lambda_x1000;
    const bool lambda_valid = in.lambda_valid;
    const uint16_t lambda_target_x1000_gate =
        get_lambda_target_x1000(snap.rpm_x10, map_bar_x100);

    LimpGatingInputs gate_in{};
    gate_in.rpm_x10 = snap.rpm_x10;
    gate_in.map_bar_x100 = map_bar_x100;
    gate_in.clt_degc_x10 = sensors.clt_degc_x10;
    gate_in.oil_press_bar_x1000 = sensors.oil_press_bar_x1000;
    gate_in.oil_fault = oil_fault;
    gate_in.fuel_press_fault = fuel_press_fault;
    gate_in.lambda_x1000 = lambda_x1000;
    gate_in.lambda_valid = lambda_valid;
    gate_in.lambda_target_x1000 = lambda_target_x1000_gate;
    gate_in.tps_pct_x10 = sensors.app_pct_x10;
    gate_in.cranking = qc.cranking;
    gate_in.full_sync = full_sync;
    gate_in.half_sync = half_sync;
    gate_in.phase_valid = true;
    gate_in.sequential = ::ecu_sched_is_sequential() != 0u;
    gate_in.etb_fault = etb_fault;
    gate_in.inj_duty_pct = static_cast<uint8_t>(
        fuel_inj_duty_pct_x10() / 10u);
    gate_in.inj_duty_cut = fuel_inj_duty_cut_active();
    gate_in.now_ms = in.now_ms;
    gate_in.map_fault = map_fault;
    gate_in.overtemp_crit = overtemp_crit;
    gate_in.diag_critical = diag_critical;
    gate_in.flood_clear = flood_clear;
    gate_in.limp_rpm_cut = limp_rpm_cut;
    gate_in.half_sync_allows_fuel = false;
    const LimpGatingResult gate =
        limp_gating_update(gate_in);
    
    const bool fuel_protect_cut = gate.fuel_protect_cut;
    const bool half_fuel_lockout = gate.half_fuel_lockout;
    const bool allow_half_crank_batch =
        half_sync && qc.cranking && !flood_clear && !fuel_protect_cut;

    // Telemetry PW must match actuators: only when injectors are actually cut.
    const bool fuel_cut_active =
        g_rev_limit_active || fuel_protect_cut || half_fuel_lockout ||
        fuel_inj_duty_cut_active();

    // (1) FULL_SYNC: running fuel path (VE / trims / AE / X-τ when not crank-ASE).
    if (full_sync && !fuel_protect_cut) {
        const Table2dLookup fuel_lookup =
            table3d_prepare_lookup(kRpmAxisX10,
                                                kLoadAxisBarX100,
                                                snap.rpm_x10,
                                                map_bar_x100);
        const uint8_t  ve = get_ve_prepared(fuel_lookup);
        const uint16_t lambda_target_x1000 =
            get_lambda_target_x1000_prepared(fuel_lookup);
        // LTFT apply = nearest cell (mesma política que crédito/store LEARN).
        // fuel_lookup.yi/xi são floor da bilineal VE — mid-bin errava a célula.
        const int16_t fuel_trim_pct_x10 = crank_or_ase ? 0 : clamp_i16(
            static_cast<int16_t>(fuel_get_stft_pct_x10() +
                                 fuel_get_ltft_at(snap.rpm_x10, map_bar_x100)),
            -500, 500);
        // AE/DE from map-fusion TPSdot (signed: tip-in >0, tip-out <0).
        const int16_t ae_tpsdot = map_get_tpsdot_x10();
        int32_t ae_pw_us = crank_or_ase ? 0
            : calc_ae_pw_from_tpsdot(ae_tpsdot, sensors.clt_degc_x10);
        uint32_t final_pw_us_base =
            calc_fuel_pw_us_default_fast(ve,
                                                       map_bar_x100,
                                                       fuel_corr.iat_density_q8,
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
        fuel_decel_cut_notify_map(map_bar_x100);
        fuel_decel_cut_notify_time(in.now_ms);
        {
            uint8_t gr = 0u;
            if (vehicle_gear(gr, in.now_ms)) {
                fuel_decel_cut_notify_gear(gr, in.now_ms);
            }
        }
        // Pedal (APP) = driver intent, valid with or without ETB (the
        // ETB blade opens by itself for idle air; without ETB the
        // ETB TPS input reads 0 and would allow a cut under load).
        const bool decel_cut_active = !crank_or_ase &&
            fuel_decel_cut_update(
                snap.rpm_x10, sensors.app_pct_x10, sensors.clt_degc_x10);
        misfire_set_all_inhibit(
            decel_cut_active || crank_or_ase || flood_clear);
        // X-τ desde !cranking (inclui afterstart frio — pior wall-wetting).
        // AE e X-τ são fenómenos distintos (AE = ar previsto pelo TPSdot,
        // X-τ = filme de parede); ambos entram inteiros.
        const bool xtau_enabled = !qc.cranking;
        if (decel_cut_active) {
            limp_gating_or_fuel_reason(kFuelCutDfco);
            s_out.net_pw_us = 0u;
            fuel_ae_notify_pulse(0);
            // Filme: reset só na entrada (não a cada tick do cut).
            if (fuel_decel_cut_just_entered()) {
                transient_fuel_reset();
            }
        } else if (final_pw_us_base > fuel_corr.dead_time_us) {
            uint32_t fuel_pw_us =
                final_pw_us_base - static_cast<uint32_t>(fuel_corr.dead_time_us);

            // LTFT aditivo (MS42 TI_AD_ADD_MMV): offset no PW líquido, célula nearest
            if (!crank_or_ase) {
                const int16_t ltft_add =
                    fuel_get_ltft_add_at(snap.rpm_x10, map_bar_x100);
                const int32_t pw_adj = static_cast<int32_t>(fuel_pw_us) + ltft_add;
                fuel_pw_us = (pw_adj <= 0) ? 0u
                           : (pw_adj > 100000) ? 100000u
                           : static_cast<uint32_t>(pw_adj);
            }
            s_out.net_pw_us = fuel_pw_us;

            // Learn X-τ: apenas no slot 100ms (λ + STFT + tpsdot gates).
            // Modelo de parede com τ escalado a wall-clock (period_ms).
            const uint32_t xtau_fuel_pw_us =
                transient_fuel_xtau_with_autocalib(fuel_pw_us,
                                                                snap.rpm_x10,
                                                                map_bar_x100,
                                                                sensors.clt_degc_x10,
                                                                xtau_enabled,
                                                                kAePeriodMs);
            // Só a parcela de FLUXO segue no pipeline; o dead-time
            // eléctrico é somado no fim, depois de ΔP/S-curve
            // (convenção de calc_final_pw_us — dead-time nunca escala).
            final_pw_us_base = xtau_fuel_pw_us;
        } else {
            transient_fuel_reset();
            final_pw_us_base = 0u;  // fluxo ≈ 0 (base ≤ dead-time)
        }
        // AE tip-in (add) ou DE tip-out (subtract), clamp a [0, 100ms].
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
                fuel_decel_cut_ramp_pw(final_pw_us_base, 2u);
        }
        // Sempre: tip-in (µs>0) freezes STFT; pulse==0 limpa no próprio 2 ms.
        if (!decel_cut_active) {
            fuel_ae_notify_pulse(ae_pw_us);
        }
        const int16_t base_advance_x10 =
            get_advance_x10_prepared(fuel_lookup);
        const uint16_t idle_target_rpm_x10 =
            auxiliaries_idle_target_rpm_x10(sensors.clt_degc_x10);
        // Cranking: fixed crank spark, no corrections.
        AdvanceCorrectionsX10 corr{};
        if (!qc.cranking) {
            // Idle spark OK during afterstart (helps settle).
            corr.idle = calc_idle_spark_correction_x10(
                snap.rpm_x10, idle_target_rpm_x10, sensors.app_pct_x10, map_bar_x100);
            corr.iat = calc_ign_iat_correction_x10(sensors.iat_degc_x10);
            corr.clt = calc_ign_clt_correction_x10(sensors.clt_degc_x10);
            corr.torque_retard = static_cast<int16_t>(in.torque_spark_retard_deg * 10);
        }
        if (!crank_or_ase) {
            corr.antijerk_retard = calc_antijerk_retard_x10(ae_tpsdot);
        }
        const int16_t sched_spark_x10 = qc.cranking
            ? static_cast<int16_t>(crank_spark_deg * 10)
            : ign_running_advance_x10(
                  calc_total_advance_x10(base_advance_x10, corr));
        // Decel / flood: force PW=0 (do not apply min_pw floor).
        const uint32_t quick_crank_pw_us =
            (decel_cut_active || flood_clear) ? 0u :
            quick_crank_flow_us(qc, final_pw_us_base);
        // Formula is flow + dead×openings (seq: +1 dead, semi: +2).
        // The pin splits that total across the openings; the gauge
        // shows the formula, not one opening.
        const uint8_t squirts = ::ecu_sched_is_sequential() ? 1u : 2u;
        uint32_t cycle_on_us = 0u;
        const uint32_t inj_pw_ticks = inj_finish_pw(
            quick_crank_pw_us, squirts, fuel_corr.dead_time_us, sensors,
            map_bar_x100, fuel_cut_active, cycle_on_us);

        // Protecção de duty (FOME #215): alimenta com o PW final
        // comandado; o corte em si entra na mask do próximo tick.
        fuel_inj_duty_update(cycle_on_us, snap.rpm_x10, 2u);

        commit(sched_spark_x10, dwell_ticks, inj_pw_ticks, snap, sensors);
    } else if (allow_half_crank_batch) {
        // (2) HALF_SYNC + cranking: simultaneous batch, crank PW only (no VE/STFT/AE).
        // Presync auto already selects SIMULTANEOUS while is_cranking().
        // Force mode in case auto was off or race with tooth ISR.
        ::ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
        misfire_set_all_inhibit(true);
        fuel_ae_notify_pulse(0);
        transient_fuel_reset();

        const uint32_t crank_flow_us = quick_crank_flow_us(qc, 0u);
        s_out.net_pw_us = crank_flow_us;
        // Cranking batch is simultaneous: formula flow+2×dead.
        uint32_t cycle_on_us = 0u;
        const uint32_t inj_pw_ticks = inj_finish_pw(
            crank_flow_us, 2u, fuel_corr.dead_time_us, sensors,
            map_bar_x100, fuel_cut_active, cycle_on_us);
        commit(static_cast<int16_t>(crank_spark_deg * 10),
                     dwell_ticks, inj_pw_ticks, snap, sensors);
    } else if (sched_sync &&
               (fuel_protect_cut || half_fuel_lockout || g_rev_limit_active)) {
        // (3) Spark-only: exit-crank HALF, flood, protect, rev-limit, anomaly path.
        // qc already updated — use crank spark only while still latched cranking.
        const int16_t sched_spark_x10 = qc.cranking
            ? static_cast<int16_t>(crank_spark_deg * 10)
            : ign_running_advance_x10(
                  get_advance_x10(snap.rpm_x10, map_bar_x100));
        commit(sched_spark_x10, dwell_ticks, 0u, snap, sensors);
        s_out.pw_ms_x10 = 0u;
        s_out.net_pw_us = 0u;
        fuel_ae_notify_pulse(0);
    }
    s_out.map_fused_x100 = map_bar_x100;

    // Prime one-shot: suppressed on flood-clear, fuel-protect (MAP/oil/rail/
    // overtemp/diag/rev limp), and whenever inj mask already locks all cyls.
    // force_output also honors the mask (defense in depth vs bypass).
    const bool prime_blocked =
        flood_clear || fuel_protect_cut || half_fuel_lockout ||
        g_rev_limit_active;
    const uint32_t prime_pw = prime_blocked
        ? 0u
        : quick_crank_consume_prime();
    if (prime_pw != 0u && !output_test_active()) {
        ::ecu_sched_fire_prime_pulse(prime_pw);
    } else if (prime_blocked) {
        // Drop pending prime so protect/flood cannot fire after condition clears mid-tooth.
        static_cast<void>(quick_crank_consume_prime());
    }


    return s_out;
}

}  // namespace ems::engine
