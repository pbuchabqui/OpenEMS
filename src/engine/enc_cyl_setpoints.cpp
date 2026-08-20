#include "engine/enc_cyl_setpoints.h"

#include "engine/calibration.h"
#include "engine/ecu_sched_internal.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/map_window.h"
#include "engine/xtau_autocalib.h"
#include "hal/critical_section.h"

#include <stdint.h>

namespace ems::engine {

namespace {

EncFuelIgnPrep g_prep{};
volatile uint8_t g_prep_seq = 0U;

uint16_t map_bar_x100_for_cyl(uint8_t cyl, uint16_t fused_map_x100) noexcept
{
    if (map_window_enable == 0U) { return fused_map_x100; }
    const uint8_t slot = map_window_slot_for_cyl(cyl);
    const uint16_t slot_x1000 = map_window_slot_bar_x1000(slot);
    if (slot_x1000 == 0U) { return fused_map_x100; }
    return static_cast<uint16_t>(slot_x1000 / 10U);
}

bool map_window_slot_valid_for_cyl(uint8_t cyl) noexcept
{
    if (map_window_enable == 0U) { return false; }
    const uint8_t slot = map_window_slot_for_cyl(cyl);
    return map_window_slot_bar_x1000(slot) != 0U;
}

uint32_t estimate_spray_dt_ms(uint32_t rpm_x10) noexcept
{
    uint32_t rpm = rpm_x10 / 10U;
    if (rpm < 200U) { rpm = 200U; }
    if (rpm > 15000U) { rpm = 15000U; }
    // 4 sprays / ciclo 720° → cycle_ms / 4
    const uint32_t cycle_ms = 120000U / rpm;
    uint32_t dt = cycle_ms / 4U;
    if (dt < 1U) { dt = 1U; }
    if (dt > 200U) { dt = 200U; }
    return dt;
}

}  // namespace

void enc_fuel_ign_prep_publish(const EncFuelIgnPrep& prep) noexcept
{
    ems::hal::CriticalSectionGuard guard;
    ++g_prep_seq;
    g_prep = prep;
    g_prep.valid = 1U;
    ++g_prep_seq;
}

EncFuelIgnPrep enc_fuel_ign_prep_read(void) noexcept
{
    EncFuelIgnPrep out{};
    for (;;) {
        const uint8_t s0 = g_prep_seq;
        if ((s0 & 1U) != 0U) { continue; }  // write in progress
        out = g_prep;
        const uint8_t s1 = g_prep_seq;
        if (s0 == s1) { break; }
    }
    return out;
}

uint8_t enc_fuel_ign_prep_valid(void) noexcept
{
    return enc_fuel_ign_prep_read().valid;
}

CylArmSetpoints finalize_cyl_setpoints(uint8_t cyl, bool commit_fuel) noexcept
{
    namespace si = sched_internal;
    CylArmSetpoints out{};
    if (cyl >= cfg::kCylinderCount) {
        return out;
    }

    const int8_t ign_trim = cyl_ign_trim_deg[cyl];
    const int8_t fuel_trim = cyl_fuel_trim_pct[cyl];
    const EncFuelIgnPrep prep = enc_fuel_ign_prep_read();

    if (prep.valid != 0U) {
        int16_t advance = 0;
        if (prep.cranking != 0U) {
            advance = prep.crank_spark_deg;
        } else {
            const uint16_t knock_x10 = knock_get_retard_x10(cyl);
            advance = calc_total_advance(
                prep.base_advance_deg,
                {prep.iat_spark_deg, prep.clt_spark_deg,
                 static_cast<int16_t>(knock_x10 / 10u),
                 prep.idle_spark_deg, prep.antijerk_retard_deg,
                 prep.torque_retard_deg});
        }
        advance = static_cast<int16_t>(advance + ign_trim);
        if (advance < 0) { advance = 0; }

        uint32_t pw_us = 0U;
        if (prep.fuel_cut == 0U) {
            const uint16_t map_cyl =
                map_bar_x100_for_cyl(cyl, prep.map_bar_x100);

            uint32_t flow_u = 0U;
            // Fase 4: com slot MAP válido, re-lookup VE/λ + base PW (não scale).
            //
            // LIMITE CONHECIDO: o pw_us resultante deste ramo (MAP por-cilindro,
            // map_cyl) é o que fica efetivamente armado no injetor via
            // arm_sequential_cyl() — mas fuel_inj_duty_update() (main_stm32.cpp)
            // continua a ser alimentado com o PW calculado sobre o MAP FUNDIDO
            // do motor. Se map_cyl > MAP fundido (pulsação de admissão real), a
            // proteção de duty sub-reporta o duty real. Documentado no ponto de
            // chamada de fuel_inj_duty_update(); não corrigido nesta revisão.
            if (map_window_slot_valid_for_cyl(cyl) && prep.rpm_x10 != 0U) {
                const uint8_t ve = get_ve(prep.rpm_x10, map_cyl);
                // VE=0 nesta célula (tabela NVM vazia no canto idle, ou
                // ASE/min_pw só no loop 2 ms): o re-lookup zerava o fluxo
                // → inj_on==EOI → pulso de ns, IGN visível e INJ "mudo".
                if (ve != 0U) {
                    const uint16_t lambda =
                        get_lambda_target_x1000(prep.rpm_x10, map_cyl);
                    const uint16_t corr_clt =
                        (prep.corr_clt_x256 != 0U) ? prep.corr_clt_x256 : 256U;
                    const uint16_t corr_iat =
                        (prep.corr_iat_x256 != 0U) ? prep.corr_iat_x256 : 256U;
                    const uint32_t full = calc_fuel_pw_us_default_fast(
                        ve, map_cyl, lambda, prep.fuel_trim_pct_x10,
                        corr_clt, corr_iat, prep.dead_time_us);
                    flow_u = (full > prep.dead_time_us)
                        ? (full - static_cast<uint32_t>(prep.dead_time_us))
                        : 0U;
                }
            }
            if (flow_u == 0U) {
                // Prefer base_flow (sem AE); fallback a flow_pw_us legado.
                flow_u = (prep.base_flow_pw_us != 0U || prep.ae_pw_us != 0)
                    ? prep.base_flow_pw_us
                    : prep.flow_pw_us;
            }

            // Trim por cilindro no fluxo.
            {
                int32_t flow = static_cast<int32_t>(flow_u)
                    * (100 + static_cast<int32_t>(fuel_trim)) / 100;
                if (flow < 0) { flow = 0; }
                flow_u = static_cast<uint32_t>(flow);
            }

            // Fase 3: AE tip-in no commanded + X-τ por evento de spray.
            if (prep.xtau_event_enable != 0U && prep.cranking == 0U) {
                int64_t commanded = static_cast<int64_t>(flow_u);
                if (prep.ae_pw_us > 0) {
                    commanded += prep.ae_pw_us;
                } else if (prep.ae_pw_us < 0) {
                    commanded += prep.ae_pw_us;  // tip-out enlean
                }
                if (commanded < 0) { commanded = 0; }
                if (commanded > 100000) { commanded = 100000; }
                const uint16_t dt =
                    static_cast<uint16_t>(estimate_spray_dt_ms(prep.rpm_x10));
                flow_u = transient_fuel_xtau_event(
                    cyl, static_cast<uint32_t>(commanded),
                    prep.rpm_x10, map_cyl, prep.clt_x10, dt, commit_fuel);
            } else if (prep.ae_pw_us != 0 &&
                       prep.base_flow_pw_us != 0U) {
                // Sem event X-τ: AE aditivo no fluxo (path legado de testes).
                const int64_t adj =
                    static_cast<int64_t>(flow_u) + prep.ae_pw_us;
                if (adj <= 0) {
                    flow_u = 0U;
                } else if (adj > 100000) {
                    flow_u = 100000U;
                } else {
                    flow_u = static_cast<uint32_t>(adj);
                }
            }

            flow_u = apply_delta_p_compensation(
                flow_u, prep.fuel_press_bar_x1000, map_cyl);
            flow_u = apply_injector_scurve(flow_u);
            pw_us = (flow_u > 0U)
                ? (flow_u + static_cast<uint32_t>(prep.dead_time_us))
                : 0U;
        }

        out.advance_deg = static_cast<uint32_t>(advance);
        out.dwell_ticks = prep.dwell_ticks;
        out.inj_pw_ticks = inj_pw_us_to_scheduler_ticks(pw_us);
        // 'P' / lock de bancada: o sequencial não pode ignorar o PW
        // comitado (finalize ia pela VE e voltava a 0). Publicado em
        // EncFuelIgnPrep pelo loop de 2ms (achado #7 da revisão
        // 2fa1513..bc30ca6, 2026-08-19) — antes lia si::g_inj_pw_ticks
        // diretamente, bypassando o contrato do prep.
        if (prep.fuel_cut == 0U && prep.bench_pw_locked != 0U) {
            out.inj_pw_ticks = prep.bench_pw_lock_ticks;
        }
        out.eoi_lead_deg = prep.eoi_lead_deg;
        return out;
    }

    // Fallback: globals do commit (testes / antes do 1º publish).
    int32_t adv = static_cast<int32_t>(si::g_advance_deg) + ign_trim;
    if (adv < 0) { adv = 0; }
    int32_t pw = static_cast<int32_t>(si::g_inj_pw_ticks)
        * (100 + static_cast<int32_t>(fuel_trim)) / 100;
    if (pw < 0) { pw = 0; }
    out.advance_deg = static_cast<uint32_t>(adv);
    out.dwell_ticks = si::g_dwell_ticks;
    out.inj_pw_ticks = static_cast<uint32_t>(pw);
    out.eoi_lead_deg = si::g_eoi_lead_deg;
    return out;
}

void enc_cyl_setpoints_reset(void) noexcept
{
    ems::hal::CriticalSectionGuard guard;
    g_prep = EncFuelIgnPrep{};
    g_prep_seq = 0U;
}

#if defined(EMS_HOST_TEST)
void enc_fuel_ign_prep_test_publish(const EncFuelIgnPrep& prep) noexcept
{
    enc_fuel_ign_prep_publish(prep);
}
#endif

}  // namespace ems::engine
