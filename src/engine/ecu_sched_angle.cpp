/**
 * @file ecu_sched_angle.cpp
 * @brief Cold-path angle table builders for the event scheduler.
 *
 * Called only at crank rev boundary (gap), never from TIM5 compare ISR.
 * Hot path (queue insert/dispatch/GPIO arm) remains in ecu_sched.cpp.
 */
#include "engine/ecu_sched_internal.h"

#include "engine/calibration.h"
#include "engine/engine_config.h"

#include <stdint.h>

namespace ems::engine::sched_internal {

AngleEvent_t g_angle_table[ECU_ANGLE_TABLE_SIZE];
uint8_t g_angle_table_count = 0U;
uint32_t g_angle_tooth_mask_lo = 0U;
uint32_t g_angle_tooth_mask_hi = 0U;

void clear_angle_table(void)
{
    g_angle_table_count = 0U;
    g_angle_tooth_mask_lo = 0U;
    g_angle_tooth_mask_hi = 0U;
}

static void angle_to_tooth_event(uint32_t angle_deg,
                                 uint8_t *out_tooth,
                                 uint8_t *out_sub_frac,
                                 uint8_t *out_phase_A)
{
    const uint32_t ang = angle_deg % 360U;
    uint32_t pos_x256 = (ang * 256U) / 6U;
    uint8_t tooth = static_cast<uint8_t>(pos_x256 >> 8U);
    uint8_t frac = static_cast<uint8_t>(pos_x256 & 0xFFU);
    if (tooth > 57U) {
        tooth = 57U;
        frac = 255U;
    }
    *out_phase_A = (angle_deg < 360U) ? ECU_PHASE_A : ECU_PHASE_B;
    *out_tooth = tooth;
    *out_sub_frac = frac;
}

static uint32_t engine_angle_to_trigger_angle(uint32_t engine_angle_deg,
                                             uint32_t cycle_deg)
{
    const uint32_t trigger_offset =
        static_cast<uint32_t>(cfg::g_eng_cfg.trigger_tooth0_engine_deg) % cycle_deg;
    return (engine_angle_deg + cycle_deg - trigger_offset) % cycle_deg;
}

static void table_add(uint8_t tooth,
                      uint8_t sub_frac,
                      uint8_t phase_A,
                      uint8_t channel,
                      uint8_t action)
{
    if (g_angle_table_count >= ECU_ANGLE_TABLE_SIZE) {
        ++g_cycle_schedule_drop_count;
        return;
    }
    AngleEvent_t *e = &g_angle_table[g_angle_table_count++];
    e->tooth_index = tooth;
    e->sub_frac_x256 = sub_frac;
    e->phase_A = phase_A;
    e->channel = channel;
    e->action = action;
    e->valid = 1U;
    if (tooth < 32U) {
        g_angle_tooth_mask_lo |= (1UL << tooth);
    } else {
        g_angle_tooth_mask_hi |= (1UL << (tooth - 32U));
    }
}

static uint32_t ticks_to_cycle_degrees(uint32_t ticks,
                                       uint32_t tooth_period_ns,
                                       uint32_t cycle_deg)
{
    const uint64_t tooth_ticks =
        static_cast<uint64_t>(TOOTH_NS_TO_SCHED_INTERNAL(tooth_period_ns));
    const uint64_t factor = (cycle_deg == kCycleDeg) ? 120ULL : 60ULL;
    const uint64_t denom = tooth_ticks * factor;
    return (denom > 0ULL)
        ? static_cast<uint32_t>((static_cast<uint64_t>(ticks) * cycle_deg) / denom)
        : 0U;
}

// Multi-spark timing MS42 — single site for sequential and presync.
template <typename EmitFn>
static inline void emit_multispark(uint32_t spark_ang,
                                   uint32_t cycle_deg,
                                   uint32_t tooth_period_ns,
                                   EmitFn emit)
{
    const uint8_t ms_count = g_mspark_count;
    if (ms_count == 0U || tooth_period_ns == 0U) {
        return;
    }
    const uint32_t inter_deg =
        ticks_to_cycle_degrees(g_mspark_inter_dwell_ticks, tooth_period_ns, cycle_deg);
    const uint32_t step = inter_deg + 1U;
    const uint32_t window = g_advance_deg + g_mspark_atdc_limit_deg;
    for (uint8_t n = 1U; n <= ms_count; ++n) {
        const uint32_t add_spark_off = static_cast<uint32_t>(n) * step;
        if (add_spark_off >= window) {
            break;
        }
        const uint32_t add_dwell_off = static_cast<uint32_t>(n - 1U) * step + 1U;
        emit((spark_ang + add_dwell_off) % cycle_deg,
             (spark_ang + add_spark_off) % cycle_deg);
    }
}

void rebuild_sequential_cycle(const ems::drv::CkpSnapshot& snap)
{
    static_assert(cfg::kCylinderCount == 4u, "ign/inj channel tables are 4-cyl");
    const uint8_t* const ign_ch = kIgnCh;
    const uint8_t* const inj_ch = kInjCh;

    g_knock_sequential = 1U;
    clear_angle_table();

    const uint32_t dwell_deg =
        ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, kCycleDeg);
    const uint32_t base_inj_pw_deg =
        ticks_to_cycle_degrees(g_inj_pw_ticks, snap.tooth_period_ns, kCycleDeg);

    for (uint8_t seq = 0U; seq < cfg::kCylinderCount; ++seq) {
        const uint8_t cyl = cfg::kFiringOrder[seq];
        const uint32_t tdc = cfg::cyl_tdc_deg(cyl);

        const int32_t ign_trim = static_cast<int32_t>(cyl_ign_trim_deg[cyl]);
        const int32_t trimmed_advance = static_cast<int32_t>(g_advance_deg) + ign_trim;
        const uint32_t eff_advance = (trimmed_advance < 0)
            ? 0u
            : static_cast<uint32_t>(trimmed_advance);

        const uint32_t spark = (tdc + kCycleDeg - eff_advance) % kCycleDeg;
        const uint32_t dwell = (spark + kCycleDeg - dwell_deg) % kCycleDeg;
        const uint32_t eoi = (tdc + kCycleDeg - g_eoi_lead_deg) % kCycleDeg;

        const int32_t fuel_trim = static_cast<int32_t>(cyl_fuel_trim_pct[cyl]);
        const int32_t pw_trimmed =
            static_cast<int32_t>(base_inj_pw_deg) * (100 + fuel_trim) / 100;
        uint32_t inj_pw = (pw_trimmed < 0) ? 0u : static_cast<uint32_t>(pw_trimmed);

        if (inj_pw > kMaxSeqInjPwDeg) {
            inj_pw = kMaxSeqInjPwDeg;
            ++g_pw_duty_clamp_count;
        }

        const uint32_t inj_on = (eoi + kCycleDeg - inj_pw) % kCycleDeg;
        const uint32_t inj_off = eoi;
        uint8_t tooth = 0U, frac = 0U, phase = 0U;

        angle_to_tooth_event(engine_angle_to_trigger_angle(dwell, kCycleDeg),
                             &tooth, &frac, &phase);
        table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_DWELL_START);
        angle_to_tooth_event(engine_angle_to_trigger_angle(spark, kCycleDeg),
                             &tooth, &frac, &phase);
        table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_SPARK);

        emit_multispark(spark, kCycleDeg, snap.tooth_period_ns,
            [&](uint32_t add_dwell_ang, uint32_t add_spark_ang) {
                angle_to_tooth_event(
                    engine_angle_to_trigger_angle(add_dwell_ang, kCycleDeg),
                    &tooth, &frac, &phase);
                table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_DWELL_START);
                angle_to_tooth_event(
                    engine_angle_to_trigger_angle(add_spark_ang, kCycleDeg),
                    &tooth, &frac, &phase);
                table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_SPARK);
            });

        angle_to_tooth_event(engine_angle_to_trigger_angle(inj_on, kCycleDeg),
                             &tooth, &frac, &phase);
        table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_ON);
        angle_to_tooth_event(engine_angle_to_trigger_angle(inj_off, kCycleDeg),
                             &tooth, &frac, &phase);
        table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_OFF);
    }
}

void rebuild_presync_revolution(const ems::drv::CkpSnapshot& snap)
{
    static const uint8_t inj_a[2U] = {ECU_CH_INJ1, ECU_CH_INJ4};
    static const uint8_t inj_b[2U] = {ECU_CH_INJ2, ECU_CH_INJ3};
    const uint8_t* const inj_all = kInjCh;
    const uint8_t* const ign = kIgnCh;
    uint8_t tooth = 0U, frac = 0U, phase = 0U;

    g_knock_sequential = 0U;
    clear_angle_table();

    const uint32_t dwell_deg =
        ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, 360U);
    uint32_t inj_pw_deg = ticks_to_cycle_degrees(
        (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS)
            ? (g_inj_pw_ticks / 2U)
            : g_inj_pw_ticks,
        snap.tooth_period_ns, 360U);
    if (inj_pw_deg > kMaxPresyncInjPwDeg) {
        inj_pw_deg = kMaxPresyncInjPwDeg;
        ++g_pw_duty_clamp_count;
    }
    const uint32_t spark = (360U - (g_advance_deg % 360U)) % 360U;
    const uint32_t dwell = (spark + 360U - dwell_deg) % 360U;
    const uint32_t eoi = (360U - (g_eoi_lead_deg % 360U)) % 360U;
    const uint32_t inj_on = (eoi + 360U - inj_pw_deg) % 360U;
    const uint32_t inj_off = eoi;

    angle_to_tooth_event(engine_angle_to_trigger_angle(dwell, 360U),
                         &tooth, &frac, &phase);
    for (uint8_t i = 0U; i < 4U; ++i) {
        table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_DWELL_START);
    }
    angle_to_tooth_event(engine_angle_to_trigger_angle(spark, 360U),
                         &tooth, &frac, &phase);
    for (uint8_t i = 0U; i < 4U; ++i) {
        table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_SPARK);
    }

    emit_multispark(spark, 360U, snap.tooth_period_ns,
        [&](uint32_t add_dwell_ang, uint32_t add_spark_ang) {
            angle_to_tooth_event(engine_angle_to_trigger_angle(add_dwell_ang, 360U),
                                 &tooth, &frac, &phase);
            for (uint8_t i = 0U; i < 4U; ++i) {
                table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_DWELL_START);
            }
            angle_to_tooth_event(engine_angle_to_trigger_angle(add_spark_ang, 360U),
                                 &tooth, &frac, &phase);
            for (uint8_t i = 0U; i < 4U; ++i) {
                table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_SPARK);
            }
        });

    angle_to_tooth_event(engine_angle_to_trigger_angle(inj_on, 360U),
                         &tooth, &frac, &phase);
    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
        for (uint8_t i = 0U; i < 4U; ++i) {
            table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_ON);
        }
    } else {
        const uint8_t *bank = (g_presync_bank_toggle == 0U) ? inj_a : inj_b;
        for (uint8_t i = 0U; i < 2U; ++i) {
            table_add(tooth, frac, ECU_PHASE_ANY, bank[i], ECU_ACT_INJ_ON);
        }
        g_presync_bank_toggle ^= 1U;
    }

    angle_to_tooth_event(engine_angle_to_trigger_angle(inj_off, 360U),
                         &tooth, &frac, &phase);
    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
        for (uint8_t i = 0U; i < 4U; ++i) {
            table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_OFF);
        }
    } else {
        const uint8_t *bank = (g_presync_bank_toggle == 1U) ? inj_a : inj_b;
        for (uint8_t i = 0U; i < 2U; ++i) {
            table_add(tooth, frac, ECU_PHASE_ANY, bank[i], ECU_ACT_INJ_OFF);
        }
    }
}

}  // namespace ems::engine::sched_internal
