/**
 * @file ecu_sched_angle.cpp
 * @brief Cold-path angle table builders for the event scheduler.
 *
 * Called only at a crank rev boundary (gap), never from the TIM5 compare ISR.
 * Hot path (queue insert/dispatch/GPIO arm) remains in ecu_sched.cpp.
 *
 * Each table entry says WHERE an event is armed (a real tooth + phase) and
 * the angle from that tooth edge to the event, in 1/256 tooth. Only angles
 * live here; durations are applied in time when the event is armed:
 *   SPARK        fires at the angle.
 *   DWELL_START  angle = its SPARK; coil ON at (spark time − dwell ticks).
 *                Armed early enough (estimated dwell + margin) to start on time.
 *   INJ_ON       opens at the angle (SOI = EOI − estimated PW angle);
 *                INJ_OFF is queued at ON + PW ticks — the pulse is exact in
 *                time whatever the speed does meanwhile.
 * Angles in the 12° missing-tooth gap are armed from tooth 57 with an
 * offset up to 3 tooth periods.
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

namespace {

constexpr int32_t kToothX10 = 60;        // 6.0° per tooth position
constexpr int32_t kRevX10 = 3600;
constexpr int32_t kMaxDwellSpanX10 = 3000;
// Point events (SPARK, INJ_ON) are armed at least this far before their
// target. Armed on the very tooth they fall on, they would fire after the
// tooth ISR has run (~2-4 us late, 0.08 deg at 3000 rpm); 1.0 deg is >= 24 us
// up to 7000 rpm, well above the ISR latency.
constexpr int32_t kArmLeadX10 = 10;

int32_t wrap(int32_t a, int32_t cycle) { return ((a % cycle) + cycle) % cycle; }

// Duration (ticks) → crank angle ×10 at the given tooth period (estimate).
int32_t ticks_to_x10(uint32_t ticks, uint32_t tooth_ticks)
{
    if (tooth_ticks == 0U) { return 0; }
    return static_cast<int32_t>((static_cast<uint64_t>(ticks) * kToothX10) / tooth_ticks);
}

// One table entry: armed at trigger angle `arm`, fires at `target` (both ×10,
// cycle-relative). The arming tooth is the last real tooth at or before
// `arm`; the offset is measured from that tooth edge.
void add_entry(int32_t arm, int32_t target, int32_t cycle, uint8_t ch, uint8_t action)
{
    if (g_angle_table_count >= ECU_ANGLE_TABLE_SIZE) {
        ++g_cycle_schedule_drop_count;
        return;
    }
    arm = wrap(arm, cycle);
    const int32_t in_rev = arm % kRevX10;
    int32_t tooth = in_rev / kToothX10;
    if (tooth > 57) { tooth = 57; }                      // gap: arm from tooth 57
    const int32_t tooth_ang = arm - in_rev + tooth * kToothX10;
    const int32_t delta = wrap(target - tooth_ang, cycle);

    AngleEvent_t* e = &g_angle_table[g_angle_table_count++];
    e->tooth_index = static_cast<uint8_t>(tooth);
    e->offset_x256 = static_cast<uint16_t>((delta * 256 + kToothX10 / 2) / kToothX10);
    e->phase_A = (cycle == static_cast<int32_t>(kCycleDeg * 10U))
        ? ((arm < kRevX10) ? ECU_PHASE_A : ECU_PHASE_B)
        : ECU_PHASE_ANY;
    e->channel = ch;
    e->action = action;
    if (tooth < 32) {
        g_angle_tooth_mask_lo |= (1UL << tooth);
    } else {
        g_angle_tooth_mask_hi |= (1UL << (tooth - 32));
    }
}

// `now` = trigger angle of the tooth 0 being processed (the rebuild point).
// An event armed ahead of its target (dwell) whose arming window spans `now`
// is listed twice: at its natural arming point (serves the next cycle) and
// at `now` for this cycle's target — the latter matters when the window
// grew past the rebuild point (accelerating) and the previous table never
// armed it. The hook drops the duplicate when it had.
void table_add(int32_t arm, int32_t target, int32_t cycle, int32_t now,
               uint8_t ch, uint8_t action)
{
    add_entry(arm, target, cycle, ch, action);
    if (wrap(target - now, cycle) < wrap(target - arm, cycle)) {
        add_entry(now, target, cycle, ch, action);
    }
}

// One coil + one injector for a cylinder whose TDC is at `tdc` (engine ×10).
void add_cylinder(uint8_t cyl, int32_t tdc, int32_t cycle, int32_t now, int32_t inj_ref,
                  int32_t dwell_span, int32_t pw_x10, int32_t trig_off)
{
    const int32_t adv = g_advance_x10 + static_cast<int32_t>(cyl_ign_trim_deg[cyl]) * 10 -
                        g_cyl_retard_x10[cyl];
    const int32_t spark = wrap(tdc - adv - trig_off, cycle);
    table_add(spark - dwell_span, spark, cycle, now, kIgnCh[cyl], ECU_ACT_DWELL_START);
    table_add(spark - kArmLeadX10, spark, cycle, now, kIgnCh[cyl], ECU_ACT_SPARK);

    const int32_t eoi = inj_ref - static_cast<int32_t>(g_eoi_lead_deg) * 10 - trig_off;
    const int32_t soi = wrap(eoi - pw_x10, cycle);
    table_add(soi - kArmLeadX10, soi, cycle, now, kInjCh[cyl], ECU_ACT_INJ_ON);
}

void build(const ems::drv::CkpSnapshot& snap, int32_t cycle, bool presync)
{
    const int32_t now = (!presync && !snap.phase_A) ? kRevX10 : 0;  // this tooth 0
    clear_angle_table();
    const uint32_t tooth_ticks = TOOTH_NS_TO_SCHED_INTERNAL(snap.tooth_period_ns);
    const int32_t dwell_x10 = ticks_to_x10(g_dwell_ticks, tooth_ticks);
    // Arm one tooth + 25 % early: the exact start is computed in time.
    int32_t dwell_span = dwell_x10 + kToothX10 + dwell_x10 / 4;
    if (dwell_span > kMaxDwellSpanX10) { dwell_span = kMaxDwellSpanX10; }
    int32_t pw_x10 = ticks_to_x10(g_inj_pw_ticks, tooth_ticks);
    if (pw_x10 > cycle * 9 / 10) { pw_x10 = cycle * 9 / 10; }
    const int32_t trig_off =
        static_cast<int32_t>(cfg::g_eng_cfg.trigger_tooth0_engine_deg) * 10 + trigger_fine_x10;
    const bool simultaneous = (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS);

    for (uint8_t cyl = 0U; cyl < cfg::kCylinderCount; ++cyl) {
        int32_t tdc = static_cast<int32_t>(cfg::cyl_tdc_deg(cyl)) * 10;
        if (presync) { tdc %= kRevX10; }
        // Presync: wasted spark (each coil at its own TDC mod 360) and two
        // openings per cycle; simultaneous mode times all injectors to cyl 0.
        const int32_t inj_ref = (presync && simultaneous) ? 0 : tdc;
        add_cylinder(cyl, tdc, cycle, now, inj_ref, dwell_span, pw_x10, trig_off);
    }
}

}  // namespace

void rebuild_sequential_cycle(const ems::drv::CkpSnapshot& snap)
{
    static_assert(cfg::kCylinderCount == 4u, "ign/inj channel tables are 4-cyl");
    g_knock_sequential = 1U;
    build(snap, static_cast<int32_t>(kCycleDeg * 10U), false);
}

void rebuild_presync_revolution(const ems::drv::CkpSnapshot& snap)
{
    g_knock_sequential = 0U;
    build(snap, kRevX10, true);
}

}  // namespace ems::engine::sched_internal
