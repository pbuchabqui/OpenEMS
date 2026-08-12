#include "engine/enc_cyl_setpoints.h"

#include "engine/calibration.h"
#include "engine/ecu_sched_internal.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/map_window.h"
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

uint32_t scale_flow_by_map(uint32_t flow_us, uint16_t map_cyl_x100,
                           uint16_t map_fused_x100) noexcept
{
    if (map_fused_x100 == 0U || map_cyl_x100 == map_fused_x100) {
        return flow_us;
    }
    // Clamp ratio 50%..150% to avoid wild PW from a bad window sample.
    uint32_t num = static_cast<uint32_t>(map_cyl_x100);
    uint32_t den = static_cast<uint32_t>(map_fused_x100);
    if (num * 2U < den) { num = den / 2U; }
    if (num > (den * 3U) / 2U) { num = (den * 3U) / 2U; }
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(flow_us) * num) / den);
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

CylArmSetpoints finalize_cyl_setpoints(uint8_t cyl) noexcept
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
        if (prep.fuel_cut == 0U && prep.flow_pw_us > 0U) {
            int32_t flow = static_cast<int32_t>(prep.flow_pw_us)
                * (100 + static_cast<int32_t>(fuel_trim)) / 100;
            if (flow < 0) { flow = 0; }
            uint32_t flow_u = static_cast<uint32_t>(flow);
            const uint16_t map_cyl =
                map_bar_x100_for_cyl(cyl, prep.map_bar_x100);
            flow_u = scale_flow_by_map(flow_u, map_cyl, prep.map_bar_x100);
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
