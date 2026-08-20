#include "test/harness.h"
#include "test/fixtures.h"
#include "test/ui_helpers.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "engine/etb_control.h"
#include "hal/etb_driver.h"
#include "engine/torque_manager.h"
#include "engine/calibration.h"
#include "app/can_rx_map.h"
#include "hal/adc.h"
#include "hal/system.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/auxiliaries.h"
#include "engine/knock.h"
#include "engine/table3d.h"
#include "engine/ecu_sched.h"
#include "hal/out_pins.h"
#include "engine/enc_cyl_setpoints.h"
#include "engine/map_window.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/misfire_encoder.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "engine/output_test.h"
#include "engine/engine_config.h"
#include "hal/timer.h"
#include "hal/flash.h"
#include "drv/encoder_sync.h"
#include "app/ui_protocol.h"
#include "app/status_bits.h"
#include "hal/crc32.h"

namespace ems::engine {
    int16_t etb_get_idle_spark_trim() noexcept;
}

extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

using namespace ems::drv;
using namespace ems::drv::encoder_sync;
using namespace ems::engine;
using namespace ems::app;
using namespace ems::hal;

void test_ecu_sched_setters(void) {
    section("ecu_sched: reset / setters / getters");
    ecu_sched_test_reset();

    // Defaults after reset: advance=10, dwell=140625, inj_pw=140625, eoi=355
    CHECK_EQ(ecu_sched_test_get_advance_deg(),  10u, "default advance=10°");
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(), 140625u, "default dwell=140625");
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 140625u, "default inj_pw=140625");
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 355u, "default eoi=355° (open-valve)");

    // Individual setters
    ecu_sched_set_advance_deg(20u);
    CHECK_EQ(ecu_sched_test_get_advance_deg(), 20u, "set_advance_deg(20)");

    ecu_sched_set_dwell_ticks(30000u);
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(), 30000u, "set_dwell_ticks(187500)");

    ecu_sched_set_inj_pw_ticks(15000u);
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 15000u, "set_inj_pw_ticks(15000)");

    ecu_sched_set_eoi_lead_deg(50u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 50u, "set_eoi_lead_deg(50)");

    // commit_calibration sets all four atomically
    ecu_sched_commit_calibration(25u, 25000u, 18000u, 55u);
    CHECK_EQ(ecu_sched_test_get_advance_deg(),   25u, "commit: advance=25");
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(),  25000u, "commit: dwell=25000");
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 18000u, "commit: inj_pw=18000");
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(),  55u, "commit: eoi=55");

    // Calibration clamp: advance > 719 → clamped
    ecu_sched_set_advance_deg(800u);
    CHECK_TRUE(ecu_sched_test_get_advance_deg() <= 719u, "advance > 720 → clamped");
    CHECK_EQ(ecu_sched_test_get_calibration_clamp_count(), 1u, "clamp count=1");

    // reset_diagnostic_counters
    ecu_sched_reset_diagnostic_counters();
    CHECK_EQ(ecu_sched_test_get_calibration_clamp_count(), 0u, "clamp_count=0 after reset");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), 0u, "late_count=0 after reset");
}

void test_ecu_sched_inhibit_masks(void) {
    section("ecu_sched: injection / ignition inhibit masks");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0u, "inj_inhibit=0 after reset");
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0u, "ign_inhibit=0 after reset");

    ecu_sched_set_inj_inhibit_mask(0x05u);  // cylinders 0 and 2
    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0x05u, "inj_inhibit=0x05");

    ecu_sched_set_ign_inhibit_mask(0x0Au);  // cylinders 1 and 3
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0x0Au, "ign_inhibit=0x0A");

    // Restore
    ecu_sched_set_inj_inhibit_mask(0u);
    ecu_sched_set_ign_inhibit_mask(0u);
    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0u, "inj_inhibit cleared");
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0u, "ign_inhibit cleared");

    section("ecu_sched: prime cannot bypass inj inhibit mask");
    {
        ecu_sched_test_reset();
        uint32_t pins[24] = {};
        ecu_sched_get_pin_counts_u32x24(pins);
        const uint32_t h0 = pins[0];   // INJ1 high count
        const uint32_t h1 = pins[3];   // INJ2 (idx 1 → offset 3)
        const uint32_t h2 = pins[6];
        const uint32_t h3 = pins[9];

        ecu_sched_set_inj_inhibit_mask(0x0Fu);
        ecu_sched_fire_prime_pulse(5000u);
        ecu_sched_get_pin_counts_u32x24(pins);
        CHECK_EQ(pins[0], h0, "prime+mask: INJ1 high count unchanged");
        CHECK_EQ(pins[3], h1, "prime+mask: INJ2 high count unchanged");
        CHECK_EQ(pins[6], h2, "prime+mask: INJ3 high count unchanged");
        CHECK_EQ(pins[9], h3, "prime+mask: INJ4 high count unchanged");

        ecu_sched_set_inj_inhibit_mask(0u);
        ecu_sched_fire_prime_pulse(5000u);
        ecu_sched_get_pin_counts_u32x24(pins);
        CHECK_TRUE(pins[0] > h0, "prime unmasked: INJ1 high count increases");
        CHECK_TRUE(pins[3] > h1, "prime unmasked: INJ2 high count increases");
        CHECK_TRUE(pins[6] > h2, "prime unmasked: INJ3 high count increases");
        CHECK_TRUE(pins[9] > h3, "prime unmasked: INJ4 high count increases");
    }
}

void test_ecu_sched_mspark(void) {
    section("ecu_sched: multi-spark");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_test_get_mspark_count(), 0u, "mspark=0 after reset");

    ecu_sched_set_mspark(2u, 5000u, 18u);
    CHECK_EQ(ecu_sched_test_get_mspark_count(), 2u, "mspark_count=2");

    // Overflow: count > 3 → clamped to 3
    ecu_sched_set_mspark(5u, 5000u, 18u);
    CHECK_TRUE(ecu_sched_test_get_mspark_count() <= 3u, "mspark_count clamped ≤3");

    // Disable
    ecu_sched_set_mspark(0u, 0u, 0u);
    CHECK_EQ(ecu_sched_test_get_mspark_count(), 0u, "mspark disabled");

    // Hard RPM ceiling for multi-spark gate (firmware policy)
    CHECK_EQ(ems::engine::kMsparkRpmCeilingX10, 15000u, "mspark ceiling = 1500 RPM");
    CHECK_TRUE(ems::engine::mspark_max_rpm_x10 <= ems::engine::kMsparkRpmCeilingX10,
               "default mspark gate ≤ 1500 RPM");
}

// O blend 1D só-RPM (eoi_idle_deg/eoi_blend_rpm_lo/hi/default_eoi_lead_deg)
// foi removido 2026-08-16, substituído pela tabela EOI 2D RPM×CLT — ver
// test_fuel_eoi_2d (test_fuel.cpp). A parte de sanitize (independente do
// blend, testa o setter direto do scheduler legacy) fica retida aqui.
void test_ecu_sched_eoi_lead_deg_sanitize(void) {
    section("ecu_sched: sanitize aceita EOI até 719 (pré-IVO)");
    ecu_sched_test_reset();
    ecu_sched_set_eoi_lead_deg(719u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 719u, "eoi=719 aceite (clamp estendido)");
    ecu_sched_set_eoi_lead_deg(365u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 365u, "eoi=365 (pré-IVO) aceite");
    ecu_sched_set_eoi_lead_deg(720u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 719u, "eoi=720 clampado a 719");
}

void test_ecu_sched_presync(void) {
    section("ecu_sched: presync enable/mode setters");
    ecu_sched_test_reset();

    // Just verify no crash
    ecu_sched_set_presync_enable(0u);
    ecu_sched_set_presync_enable(1u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SEMI_SEQUENTIAL);
    ecu_sched_fire_prime_pulse(5000u);  // prime pulse: no crash with valid pw
    CHECK_TRUE(true, "presync setters and prime_pulse 5000: no crash");

    // ecu_sched_fire_prime_pulse edge cases:
    // pw=0 → guard: early return (no crash)
    ecu_sched_fire_prime_pulse(0u);
    CHECK_TRUE(true, "fire_prime_pulse(0): no crash (early return)");

    // pw > 30000 → clamped to 30000 (no crash, clamp happens internally)
    ecu_sched_fire_prime_pulse(100000u);
    CHECK_TRUE(true, "fire_prime_pulse(100000): no crash (clamped to 30ms)");
}

void test_ecu_sched_dwell_watchdog(void) {
    section("ecu_sched: dwell watchdog");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u, "watchdog_count=0 at start");
    // Calling watchdog with no armed dwell should be a no-op
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u, "watchdog_count=0 with no armed coil");
}

// ============================================================================
// MT6835/TIM2 ENCODER — estimador de ω (ecu_sched_encoder_omega.cpp)
// ============================================================================

void test_ecu_sched_encoder_omega(void) {
    section("ecu_sched: encoder omega estimator");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "invalid before first sample");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 0, "x65536=0 before first sample");

    // First sample only seeds prev — still no rate to compute.
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "invalid after single sample");

    // d_tim2=1000, d_tim5=1000 -> ω=1.0 exact -> x65536=65536.
    ecu_sched_encoder_omega_sample(2000u, 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "valid after second sample");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega=1.0 -> x65536=65536");

    // d_tim2=500, d_tim5=1000 -> ω=0.5 -> x65536=32768.
    ecu_sched_encoder_omega_sample(2500u, 3000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 32768, "omega=0.5 -> x65536=32768");

    // Reverse rotation (kick-back): TIM2 decrements, TIM5 keeps advancing —
    // sign must survive, not be clamped to zero/positive.
    ecu_sched_encoder_omega_sample(2300u, 4000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), -13107,
             "reverse rotation: negative x65536 (d_tim2=-200/d_tim5=1000)");

    // d_tim5<=0 (stale/out-of-order sample): estimate must hold, not update
    // (division-by-zero / sign-inversion guard).
    const int32_t before = ecu_sched_encoder_omega_x65536();
    ecu_sched_encoder_omega_sample(9999u, 4000u);  // same tim5_now as previous
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), before,
             "d_tim5<=0: estimate unchanged, no divide-by-zero");
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "still valid after stale sample");

    // ISR bunched (Δt de poucos ticks, Δposição grande): não adoptar o pico.
    ecu_sched_encoder_omega_sample(9999u + 256u, 4010u);  // d_tim5=10, d_tim2=256
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), before,
             "d_tim5 curto: estimaçao anterior mantida");

    // Pico absurdo (d_tim2/d_tim5 ⇒ x65536 > tecto de host 2e6).
    // No firmware o tecto é 6000 (~21 kRPM); a suite host semeia ω=1.0.
    ecu_sched_encoder_omega_sample(10255u + 100000u, 4010u + 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), before,
             "omega acima do tecto: pico rejeitado, estimaçao anterior mantida");

    // Regression guard for the x256 truncation-to-zero bug: a realistic
    // ~200 rpm cranking rate (d_tim2=874 counts / d_tim5=1e6 ticks, the
    // same ratio as 200 rpm @ 16384 counts/rev, 62.5 MHz TIM5) must NOT
    // read as zero rotation. Under the old ×256 scale this rounded to 0
    // (0.224 truncated); at ×65536 it must land at 57.
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(0u, 0u);
    ecu_sched_encoder_omega_sample(874u, 1000000u);
    CHECK_TRUE(ecu_sched_encoder_omega_x65536() != 0,
               "200rpm-equivalent rate must not truncate to zero (x256 bug regression)");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 57,
             "200rpm-equivalent rate -> x65536=57");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "test_reset() clears omega state");
}

void test_ecu_sched_encoder_phase(void) {
    section("ecu_sched: encoder phase tracker");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "invalid before first anchor");

    // Anchor: raw=100000 marks entering ECU_PHASE_B.
    ecu_sched_encoder_phase_set_anchor(100000u, ECU_PHASE_B);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u, "valid after set_anchor");
    CHECK_EQ(ecu_sched_encoder_phase_at(100000u), ECU_PHASE_B,
             "at anchor: same phase (revs=0, even)");

    // +1 full rev (16384 counts) -> phase flips.
    CHECK_EQ(ecu_sched_encoder_phase_at(116384u), ECU_PHASE_A,
             "+1 rev: flipped (revs=1, odd)");
    // +2 full revs -> back to anchor phase.
    CHECK_EQ(ecu_sched_encoder_phase_at(132768u), ECU_PHASE_B,
             "+2 revs: same phase again (revs=2, even)");

    // Boundary just BEFORE the anchor: still belongs to the previous
    // (flipped) revolution — this is the floor-division correctness case
    // (truncated division would wrongly give revs=0/unflipped here).
    CHECK_EQ(ecu_sched_encoder_phase_at(99999u), ECU_PHASE_A,
             "1 count before anchor: flipped (floor(-1/16384)=-1, odd)");
    // Exactly 1 rev before the anchor -> flipped (revs=-1 exact).
    CHECK_EQ(ecu_sched_encoder_phase_at(83616u), ECU_PHASE_A,
             "-1 rev exact: flipped (revs=-1, odd)");
    // 1 count further back crosses into the next-older revolution -> unflipped.
    CHECK_EQ(ecu_sched_encoder_phase_at(83615u), ECU_PHASE_B,
             "-1 rev -1 count: unflipped (revs=-2, even)");

    // Re-anchoring is absolute, not incremental — a second set_anchor with a
    // different phase overrides the previous state entirely (no toggle).
    ecu_sched_encoder_phase_set_anchor(500000u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_phase_at(500000u), ECU_PHASE_A,
             "re-anchor: absolute, reflects new anchor immediately");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "test_reset() clears phase anchor");
}

void test_ecu_sched_encoder_min_lead(void) {
    section("ecu_sched: encoder arm — min-lead floor via omega (task #8)");
    ecu_sched_test_reset();

    // Omega invalid (never sampled): floor is 0, so a target at CNT is
    // already due. TIM2 is a position counter — insert executes it inline
    // (late path). Parking CCR3==CNT and clearing CC3IF would stall the
    // dispatcher until the 32-bit wrap (pulses stop).
    ecu_sched_encoder_test_set_tim2_cnt(1000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 1000u, ECU_ACT_INJ_ON);
    uint32_t ts = 0u; uint8_t ch = 0xFFu; uint8_t high = 0xFFu;
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "omega invalid, target==CNT: executed inline (no stuck CCR3)");
    CHECK_TRUE(ecu_sched_encoder_enc_evt_execute_count() >= 1u,
               "due insert executed (on-time path of dispatch)");
    ecu_sched_test_reset();

    // Seed omega=1.0 exact (d_tim2=1000/d_tim5=1000, same recipe as
    // test_ecu_sched_encoder_omega) -> x65536=65536. Floor = 2us worth of
    // counts at this rate = ECU_SCHED_US_TO_TICKS_INTERNAL(2)=125 ticks *
    // 65536/65536 = 125 counts.
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega seeded to 1.0");

    // Target exactly at "now": lead=0 < floor(125) -> clamped to now+125.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5125u, "lead=0 < floor: clamped to now+125");
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);

    // Target just short of the floor (lead=124): still clamped.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5124u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5125u, "lead=124 < floor=125: still clamped to now+125");
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);

    // Target exactly at the floor (lead=125): passes through unchanged.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5125u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5125u, "lead=125 == floor: passes through unchanged");
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);

    // Comfortably far target: unaffected by the floor.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 9000u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 9000u, "far target: unaffected by floor");

    // Half-rate omega (0.5, x65536=32768): floor scales down proportionally
    // -> 125*32768/65536 = 62 (integer truncation).
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(1500u, 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 32768, "omega seeded to 0.5");
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5062u, "omega=0.5: floor scales down to 62 counts");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_basic(void) {
    section("ecu_sched: encoder queue (TIM2/CH3) — insert order + CCR3 arm");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "empty at start");
    CHECK_EQ(ecu_sched_encoder_test_get_dier(), 0u, "CC3IE off at start");

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 500u, ECU_ACT_INJ_ON);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "count=1 after first arm");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 500u, "CCR3=500 (only/earliest event)");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() != 0u,
               "CC3IE on after first insert");

    uint32_t ts = 0u; uint8_t ch = 0xFFu; uint8_t high = 0xFFu;
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 500u, "evt[0].ts=500");
    CHECK_EQ(ch, (uint8_t)ECU_CH_INJ1, "evt[0].channel=INJ1");
    CHECK_EQ(high, 1u, "evt[0].high=1 (ON)");

    // Earlier target becomes the new head — CCR3 rearms to it, not appended.
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 300u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 2u, "count=2 after second arm");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 300u, "CCR3 rearmed to earlier target");
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 300u, "evt[0]=IGN1/300 (sorted ahead of INJ1/500)");
    CHECK_EQ(ch, (uint8_t)ECU_CH_IGN1, "evt[0].channel=IGN1");
    CHECK_EQ(high, 0u, "evt[0].high=0 (SPARK)");
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(1, &ts, &ch, &high) != 0u, "get_evt(1) ok");
    CHECK_EQ(ts, 500u, "evt[1]=INJ1/500 (unchanged, now second)");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "test_reset() clears encoder queue");
}

void test_ecu_sched_encoder_queue_dispatch(void) {
    section("ecu_sched: encoder queue — dispatch + CC3IE dynamic disable");
    ecu_sched_test_reset();

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 200u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 2u, "2 events armed");

    ecu_sched_encoder_test_set_tim2_cnt(100u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "1 event fired at ts=100");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 200u, "CCR3 rearmed to remaining event");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() != 0u,
               "CC3IE still on — queue not empty");

    ecu_sched_encoder_test_set_tim2_cnt(200u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "queue empty after second fire");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() == 0u,
               "CC3IE off — queue emptied (dynamic disable, mirrors TIM5)");

    // Simultaneous-at-dispatch: two events due in the same ISR entry both fire.
    ecu_sched_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 50u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ2, 50u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(50u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "both simultaneous events fire in one dispatch entry");
    CHECK_EQ(ecu_sched_encoder_test_get_late_event_count(), 0u,
             "on-time dispatch is not counted as late");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_dispatch_margin_domain(void) {
    section("ecu_sched: dispatch — margem TIM2 = 3 µs via ω (fallback 16 sem ω)");

    // ω inválido (estado limpo de reset): fallback ao literal 16 counts.
    // Alvo a now+17 (>16): CCR3 reprogramado, não dispara já.
    ecu_sched_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 17u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u,
             "ω inválido, alvo a +17: ainda pendente (margem=16 fallback)");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 17u,
             "ω inválido, alvo a +17: CCR3 reprogramado, não disparado como late");

    // Alvo a now+16 (não > 16): dispara já como late — fronteira do fallback.
    ecu_sched_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 16u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "ω inválido, alvo a +16: dispara já (fronteira do fallback=16)");
    CHECK_EQ(ecu_sched_encoder_test_get_late_event_count(), 1u,
             "ω inválido, alvo a +16: contado como late");
    CHECK_EQ(ecu_sched_encoder_late_event_count(), 1u,
             "getter de produção late TIM2 ≠0 após late forçado");
    CHECK_EQ(ecu_sched_encoder_evt_overflow(), 0u,
             "getter de produção overflow TIM2 ainda 0");

    // ω válido, ratio=2.0 (omega_x65536=131072) → margem =
    //   ticks(3µs)=3*125/2=187 → 187*131072/65536 = 374 counts.
    // Alvo absoluto bem distante (100000) para não ser clampado pelo piso
    // min_lead_counts() de arm_channel.
    ecu_sched_test_reset();
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(2000u, 1000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 131072, "ω sintético seeded a ratio=2.0");

    ecu_sched_encoder_queue_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100000u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(100000u - 375u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u,
             "ω=2.0×, alvo a +375: ainda pendente (margem=374 = 3 µs via ω)");

    ecu_sched_encoder_queue_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100000u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(100000u - 374u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "ω=2.0×, alvo a +374: dispara já (fronteira da margem 3 µs)");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_overflow(void) {
    section("ecu_sched: encoder queue — overflow policy (never drop a pending OFF)");
    ecu_sched_test_reset();

    // Fill the queue with 48 ON events (alternating channels so none collide
    // as the "same channel" preference in the drop policy).
    for (uint32_t i = 0u; i < 48u; ++i) {
        const uint8_t ch = (i % 2u == 0u) ? ECU_CH_INJ1 : ECU_CH_INJ2;
        ecu_sched_encoder_arm_channel(ch, 1000u + i, ECU_ACT_INJ_ON);
    }
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 48u, "queue full at 48");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_overflow(), 0u, "no overflow yet — exactly full");

    // 49th ON with a full queue: dropped (queue keeps de-asserts already
    // queued in preference over a new assert).
    ecu_sched_encoder_arm_channel(ECU_CH_INJ3, 2000u, ECU_ACT_INJ_ON);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 48u, "still 48 — new ON dropped");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_overflow(), 1u, "overflow counted");

    // A de-assert (OFF/SPARK) on a full queue evicts one ON to make room —
    // never silently dropped itself (an open injector must be closable).
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 3000u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 48u,
             "still 48 — OFF evicted an ON to fit");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_purge_via_inhibit_mask(void) {
    section("ecu_sched: encoder queue — purge sweep via inj inhibit mask");
    ecu_sched_test_reset();

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_ON);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "INJ1 event armed on encoder queue");

    // INJ1 = cyl 0 -> inhibit mask bit0. purge_events_for_cyl_mask()
    // (ecu_sched.cpp) must sweep BOTH queues — this is the real wiring
    // path, not a direct call into the internal sweep function.
    ecu_sched_set_inj_inhibit_mask(0x01u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "encoder queue purged by inj inhibit mask (cross-queue sweep)");

    ecu_sched_set_inj_inhibit_mask(0x00u);
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_clear_via_outputs_safe(void) {
    section("ecu_sched: encoder queue — cleared by ecu_sched_test_all_outputs_safe()");
    ecu_sched_test_reset();

    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 7000u, ECU_ACT_DWELL_START);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "IGN1 event armed on encoder queue");

    ecu_sched_test_all_outputs_safe();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "encoder queue cleared by clear_all_events_and_drive_safe_outputs()");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() == 0u,
               "CC3IE off after clear-all");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_handoff_force_closes_pins(void) {
    section("ecu_sched: handoff presync->sequencial força fecho de pinos (fix bug 4)");
    ecu_sched_test_reset();
    ems::hal::out_pins_test_reset_stubs();

    // INJ1 (canal 2 = PA15 na RGT6 default) é o único canal no port A — a
    // fila de force-close (0x0F) reescreve GPIOC várias vezes (IGN1-4/INJ3-4
    // partilham port C), por isso o snapshot final desse port não prova nada
    // sobre um canal específico. PA15 nunca é tocado por outro canal, o que
    // torna o snapshot final determinístico para esta asserção.
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 100u, ECU_ACT_DWELL_START);
    ecu_sched_encoder_test_set_tim2_cnt(100u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "INJ_ON+DWELL_START despachados");
    CHECK_TRUE((ems::hal::out_pins_test_bsrr_snapshot(0u) & (1u << 15u)) != 0u,
               "INJ1 (PA15) HIGH após dispatch do INJ_ON");
    CHECK_TRUE(ecu_sched_test_get_dwell_arm_tick(0u) != 0u,
               "g_dwell_arm_tick[0] armado após DWELL_START");

    // Arma as contrapartes de-assert para alvos futuros — ficam PENDENTES na
    // fila TIM2/CH3 quando o handoff acontecer a seguir.
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_OFF);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 5000u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 2u, "INJ_OFF+SPARK pendentes na fila");

    // Handoff presync→sequencial: fase passa a válida com
    // g_enc_last_builder_was_sequential==0 (estado limpo do reset acima) —
    // dispara o purge da fila + (fix) o force-close dos pinos.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    ecu_sched_encoder_heartbeat_tick(6000u, 6000u, 0u, 0u);

    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "fila TIM2 purgada no handoff (eventos pendentes descartados)");
    CHECK_TRUE((ems::hal::out_pins_test_bsrr_snapshot(0u) & (1u << (15u + 16u))) != 0u,
               "INJ1 (PA15) forçado LOW no handoff — antes do fix ficava HIGH até o watchdog");
    CHECK_EQ(ecu_sched_test_get_dwell_arm_tick(0u), 0u,
             "g_dwell_arm_tick[0] limpo no handoff — sem isto o watchdog ainda achava a bobina em carga");

    ecu_sched_test_reset();
    ems::hal::out_pins_test_reset_stubs();
}

void test_ecu_sched_encoder_heartbeat(void) {
    section("ecu_sched: encoder heartbeat tick — feeds omega estimator");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "omega invalid before any heartbeat tick");

    // First tick only seeds the omega estimator's previous sample.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "still invalid after single tick");

    // Second tick: d_tim2=1000, d_tim5=1000 -> omega=1.0 -> x65536=65536.
    // Confirms the heartbeat really calls ecu_sched_encoder_omega_sample()
    // with the values it was handed (HAL reads them, this just verifies the
    // wiring, not the estimator's own math — that's covered separately).
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "valid after second tick");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega fed correctly through the heartbeat");

    // cmp_edge_count delta is tracked/validated (drv/encoder_sync.cpp, task
    // #13) — but the actual phase_set_anchor() call stays gated behind
    // EMS_MT6835_CMP_PHASE_CALIBRATED (0 by default: calibration constant
    // not yet measured on a bench, see plan), so phase_valid() stays 0 even
    // though the edge itself was accepted as a valid first reference. See
    // test_ecu_sched_encoder_heartbeat_cmp_tracking() for direct coverage of
    // the tracking/validation logic itself.
    ecu_sched_encoder_heartbeat_tick(2500u, 3000u, 12345u, 1u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "phase anchor NOT set by the heartbeat yet (calibration constant pending)");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "test_reset() clears heartbeat-fed state");
}

void test_ecu_sched_encoder_heartbeat_cmp_tracking(void) {
    section("ecu_sched: encoder heartbeat — CMP edge tracking/validation (task #13)");
    ecu_sched_test_reset();
    ckp_test_reset();

    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "reject count=0 at start");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 0u, "missed-edge count=0 at start");

    // A: first edge (cmp_edge_count 0->1) — arms reference, no validation.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 1000u, 1u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "A: first edge — no reject");

    // B: normal span (delta=32768 exact) — accepted, multiple=1.
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 1000u + kCmpSpanCounts, 2u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "B: normal span — no reject");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 0u, "B: multiple=1 — not counted as missed");

    // C: implausible span (delta=16384, half a revolution) — rejected.
    ecu_sched_encoder_heartbeat_tick(3000u, 3000u, 1000u + kCmpSpanCounts + 16384u, 3u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 1u, "C: implausible span — rejected");

    // D: missed edge (delta=2x32768 from B's angle, C's reject didn't move
    // the reference) — accepted as multiple=2.
    ecu_sched_encoder_heartbeat_tick(4000u, 4000u, 1000u + kCmpSpanCounts + 2u * kCmpSpanCounts, 4u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 1u, "D: accepted — reject count unchanged");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 1u, "D: multiple=2 — missed edge counted");

    // E/F/G: 3 consecutive bad edges (delta=100, nowhere near a multiple) ->
    // reject-streak reaches the resync threshold on the 3rd.
    const uint32_t ref = 1000u + kCmpSpanCounts + 2u * kCmpSpanCounts;  // D's accepted angle
    ecu_sched_encoder_heartbeat_tick(5000u, 5000u, ref + 100u, 5u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 2u, "E: 1st consecutive reject");
    ecu_sched_encoder_heartbeat_tick(6000u, 6000u, ref + 100u, 6u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 3u, "F: 2nd consecutive reject");
    ecu_sched_encoder_heartbeat_tick(7000u, 7000u, ref + 100u, 7u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 4u, "G: 3rd consecutive reject (streak resync)");

    // H: reference was dropped by the resync — next edge is treated as a
    // fresh "first edge" again, no reject regardless of its angle.
    ecu_sched_encoder_heartbeat_tick(8000u, 8000u, 999999u, 8u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 4u, "H: post-resync first edge — no new reject");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 1u, "H: missed-edge count unchanged");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "test_reset() clears CMP tracking state");
}

// Regressão: ckp_get_cmp_glitch_count() ficava sempre preso em stub (0),
// mascarando ruído real do CMP no dash (achado #4 da revisão
// 2fa1513..bc30ca6, 2026-08-19). Agora é publicado em CkpSnapshot a cada
// heavy tick, mesmo padrão de cmp_confirms.
void test_ckp_get_cmp_glitch_count_wired(void) {
    section("ckp: ckp_get_cmp_glitch_count() reflete rejeições reais (não fica preso em stub)");
    ecu_sched_test_reset();
    ckp_test_reset();

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 1000u, 1u);  // A: arma referência
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "glitch count=0 antes de qualquer rejeição");

    // Span implausível — rejeitado.
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 1000u + 16384u, 2u);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 1u,
             "glitch count sobe para 1 depois da 1ª rejeição — publicado, não stub");

    ecu_sched_encoder_heartbeat_tick(3000u, 3000u, 1000u + 16384u + 50u, 3u);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 2u, "2ª rejeição consecutiva: glitch count=2");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_cmp_watchdog_presync(void) {
    section("ecu_sched: encoder heartbeat — CMP watchdog fires without phase_valid() (presync)");
    ecu_sched_test_reset();
    ckp_test_reset();
    // Força threshold de produção (kMaxHeartbeatsWithoutCmp=6), independente
    // do que testes anteriores tenham deixado em g_bench_clt_iat (não é
    // resetado por sensors_test_reset() — é config de bancada persistente).
    sensors_set_bench_clt_iat(false, 0, 0);

    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_request_count(), 0u, "request count=0 at start");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_poll_and_clear(), 0u, "nothing pending at start");

    // cmp_phase_state nunca calibrado nesta bancada -> phase_valid() fica 0
    // o tempo todo (mesmo cenário do bug de campo: presync, CMP morto desde
    // o boot). O bloco staleness_exceeded() existente (linhas ~588-592)
    // nunca roda aqui — é exatamente o caso que ele NÃO cobre.
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "phase never valid — uncalibrated bench");

    // cmp_edge_count constante (0) em toda chamada: g_hb_last_cmp_edge_count
    // também começa em 0 (test_reset), então o bloco de aceitação nunca
    // roda — heartbeats_since_ok só incrementa.
    for (uint32_t i = 1u; i < kMaxHeartbeatsWithoutCmp; ++i) {
        ecu_sched_encoder_heartbeat_tick(i * 1000u, i * 1000u, 0u, 0u);
        CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), i,
                 "heartbeats_since_ok tracks tick count before threshold");
        CHECK_EQ(ecu_sched_encoder_cmp_watchdog_poll_and_clear(), 0u,
                 "no rearm requested before threshold");
    }

    // Tick que cruza o limiar (== kMaxHeartbeatsWithoutCmp): dispara.
    ecu_sched_encoder_heartbeat_tick(kMaxHeartbeatsWithoutCmp * 1000u,
                                     kMaxHeartbeatsWithoutCmp * 1000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), kMaxHeartbeatsWithoutCmp,
             "heartbeats_since_ok == limit on the triggering tick");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_request_count(), 1u, "watchdog fired exactly once");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_poll_and_clear(), 1u, "poll returns the pending rearm");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_poll_and_clear(), 0u, "poll clears — second call sees nothing");

    // Continua sem flanco aceite por mais um bocado: "==" não retrigger.
    for (uint32_t i = 1u; i <= kMaxHeartbeatsWithoutCmp; ++i) {
        ecu_sched_encoder_heartbeat_tick((kMaxHeartbeatsWithoutCmp + i) * 1000u,
                                         (kMaxHeartbeatsWithoutCmp + i) * 1000u, 0u, 0u);
    }
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_request_count(), 1u,
             "no re-trigger while heartbeats_since_ok stays above the threshold");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_poll_and_clear(), 0u, "still nothing pending");

    // Um flanco aceite (primeira referência, mesmo padrão do teste A acima)
    // zera heartbeats_since_ok — prova que não é um latch permanente.
    ecu_sched_encoder_heartbeat_tick(90000u, 90000u, 12345u, 1u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 1u,
             "accepted edge resets heartbeats_since_ok (then this same tick re-increments once)");

    // Novo episódio de silêncio (mesmo cmp_edge_count=1 dali em diante) volta
    // a cruzar o limiar e pede rearm de novo — não é latch, é por episódio.
    for (uint32_t i = 2u; i < kMaxHeartbeatsWithoutCmp; ++i) {
        ecu_sched_encoder_heartbeat_tick((90000u + i * 1000u), (90000u + i * 1000u), 12345u, 1u);
    }
    ecu_sched_encoder_heartbeat_tick(90000u + kMaxHeartbeatsWithoutCmp * 1000u,
                                     90000u + kMaxHeartbeatsWithoutCmp * 1000u, 12345u, 1u);
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_request_count(), 2u,
             "second silence episode requests a second rearm");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_poll_and_clear(), 1u, "second rearm is pending");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_request_count(), 0u, "test_reset() clears watchdog state");
}

void test_ecu_sched_encoder_cmp_watchdog_alongside_staleness(void) {
    section("ecu_sched: encoder heartbeat — CMP watchdog fires alongside phase_invalidate() (calibrated)");
    ecu_sched_test_reset();
    ckp_test_reset();
    sensors_set_bench_clt_iat(false, 0, 0);
    ems::engine::cfg::g_eng_cfg.cmp_phase_state = ems::engine::cfg::kCmpPhaseCalibratedA;

    // Primeiro flanco: aceite como referência, mas com o Fix B (exige 2
    // flancos consecutivos confirmados antes de ancorar) ainda NÃO ancora.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 1000u, 1u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "1 flanco sozinho não ancora (Fix B: exige confirmação)");

    // Segundo flanco, consistente com o primeiro (delta = kCmpSpanCounts) —
    // confirma e ancora a fase.
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 1000u + kCmpSpanCounts, 2u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u,
             "phase anchored — 2º flanco consecutivo confirma (Fix B)");

    // Sem novo flanco (cmp_edge_count constante=2) até cruzar o limiar: o
    // bloco de staleness_exceeded() existente e o watchdog novo disparam
    // no mesmo tick, sem interferir um no outro.
    uint32_t heartbeats_since_ok = ecu_sched_encoder_test_get_cmp_heartbeats_since_ok();
    uint32_t t = 3000u;
    while (heartbeats_since_ok < kMaxHeartbeatsWithoutCmp) {
        ecu_sched_encoder_heartbeat_tick(t, t, 1000u + kCmpSpanCounts, 2u);
        heartbeats_since_ok = ecu_sched_encoder_test_get_cmp_heartbeats_since_ok();
        t += 1000u;
    }
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "phase_invalidate() still fires on staleness (existing behavior unchanged)");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 0u,
             "phase_invalidate zera confirm_count — 1 flanco só não re-ancora");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_request_count(), 1u,
             "watchdog also requested a rearm on the same tick");
    CHECK_EQ(ecu_sched_encoder_cmp_watchdog_poll_and_clear(), 1u, "rearm is pending");

    // Um flanco aceite depois da perda (span válido contra o último real)
    // sobe confirm a 1 mas NÃO re-ancora — o caso "desliguei o CMP e o
    // rearm do TIM3 / ruído no pino gerou um flanco fantasma".
    ecu_sched_encoder_heartbeat_tick(t, t, 1000u + 2u * kCmpSpanCounts, 3u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 1u,
             "1º flanco após perda: confirm=1");
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "1 flanco após perda de CMP não re-ancora (fica em presync)");

    ecu_sched_test_reset();
}

// Regressão: o silêncio do CMP tem de descartar TAMBÉM a referência angular,
// não só phase_valid/confirm_count. Cenário: silêncio longo do CMP com a
// cambota a rodar, e os flancos DEPOIS voltam a chegar — o 1º flanco de volta
// vem dezenas de spans depois do último aceite. Comparado contra a referência
// pré-silêncio, evaluate_cmp_edge() dá n ≫ kCmpMaxAcceptedMultiple ⇒
// REJEITADO, e a recuperação só acontecia pela via lenta (3 rejeições até
// streak_resync + 2 flancos de confirm).
void test_ecu_sched_encoder_cmp_ref_dropped_on_staleness(void) {
    section("ecu_sched: encoder heartbeat — staleness descarta a referência angular do CMP");
    ecu_sched_test_reset();
    ckp_test_reset();
    sensors_set_bench_clt_iat(false, 0, 0);
    ems::engine::cfg::g_eng_cfg.cmp_phase_state = ems::engine::cfg::kCmpPhaseCalibratedA;

    // Ancorar com 2 flancos consistentes.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 1000u, 1u);
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 1000u + kCmpSpanCounts, 2u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u, "pré-cond: fase ancorada");

    // Silêncio do CMP até cruzar o limiar de staleness.
    uint32_t heartbeats_since_ok = ecu_sched_encoder_test_get_cmp_heartbeats_since_ok();
    uint32_t t = 3000u;
    while (heartbeats_since_ok < kMaxHeartbeatsWithoutCmp) {
        ecu_sched_encoder_heartbeat_tick(t, t, 1000u + kCmpSpanCounts, 2u);
        heartbeats_since_ok = ecu_sched_encoder_test_get_cmp_heartbeats_since_ok();
        t += 1000u;
    }
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "staleness invalidou a fase");

    const uint32_t rejects_before = ecu_sched_encoder_test_get_cmp_reject_count();

    // 1º flanco de volta, MUITO além de kCmpMaxAcceptedMultiple spans.
    const uint32_t far_angle = 1000u + 40u * kCmpSpanCounts;
    t += 1000u;
    ecu_sched_encoder_heartbeat_tick(t, t, far_angle, 3u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), rejects_before,
             "1º flanco após silêncio NÃO é rejeitado (referência velha descartada)");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 1u,
             "1º flanco após silêncio só arma a referência: confirm=1");
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "gate de 2 flancos preservado: 1 flanco não re-ancora");

    // 2º flanco, um span depois: re-ancora — recuperação em 2 flancos.
    t += 1000u;
    ecu_sched_encoder_heartbeat_tick(t, t, far_angle + kCmpSpanCounts, 4u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u,
             "2º flanco re-ancora (2 flancos, não 3 rejeições + 2)");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_cmp_confirm_gate(void) {
    section("ecu_sched: encoder heartbeat — re-anchor exige 2 flancos consecutivos após streak_resync (Fix B)");
    ecu_sched_test_reset();
    ckp_test_reset();
    ems::engine::cfg::g_eng_cfg.cmp_phase_state = ems::engine::cfg::kCmpPhaseCalibratedA;

    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 0u, "confirm_count=0 no início");

    // 1º flanco: confirm_count sobe a 1, ainda não ancora.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 1000u, 1u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 1u, "1º flanco: confirm_count=1");
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "1º flanco sozinho não ancora (Fix B)");

    // 2º flanco consistente (delta=kCmpSpanCounts): confirm_count satura em
    // 2, agora ancora.
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 1000u + kCmpSpanCounts, 2u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 2u, "2º flanco: confirm_count=2 (satura)");
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u, "2º flanco confirma e ancora");

    // 3 flancos ruins seguidos (delta=100, longe de qualquer múltiplo
    // plausível) -> streak_resync na 3ª rejeição — descarta referência E
    // confirm_count (mesmo padrão E/F/G de test_ecu_sched_encoder_heartbeat_cmp_tracking).
    const uint32_t ref = 1000u + kCmpSpanCounts;
    ecu_sched_encoder_heartbeat_tick(3000u, 3000u, ref + 100u, 3u);
    ecu_sched_encoder_heartbeat_tick(4000u, 4000u, ref + 100u, 4u);
    ecu_sched_encoder_heartbeat_tick(5000u, 5000u, ref + 100u, 5u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 0u,
             "streak_resync zera confirm_count (3ª rejeição consecutiva)");

    // Flanco isolado pós-resync (ângulo arbitrário/espúrio, ex.: ruído num
    // pino agora flutuante) — confirm_count sobe só a 1;
    // phase_set_anchor() NÃO é chamado (gate ainda fechado), phase_valid()
    // não muda por causa deste flanco isolado.
    const uint8_t prev_valid = ecu_sched_encoder_phase_valid();
    ecu_sched_encoder_heartbeat_tick(6000u, 6000u, 999999u, 6u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 1u,
             "flanco isolado pós-resync: confirm_count=1, ainda não confirma");
    CHECK_EQ(ecu_sched_encoder_phase_valid(), prev_valid,
             "gate fechado: phase_valid() não muda por causa do flanco isolado");

    // Segundo flanco consistente com o isolado — agora confirma e reancora.
    ecu_sched_encoder_heartbeat_tick(7000u, 7000u, 999999u + kCmpSpanCounts, 7u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 2u,
             "2º flanco pós-resync: confirm_count=2");
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u,
             "2º flanco consistente reancora depois do resync");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 0u, "test_reset() zera confirm_count");
}

void test_ecu_sched_encoder_seq_arm_stall_watchdog(void) {
    section("ecu_sched: encoder heartbeat — watchdog do builder sequencial cai p/ presync se nada arma (Fix C)");
    ecu_sched_test_reset();
    ckp_test_reset();
    // bench_mode (limiar de staleness=60, não 6): sem novos flancos CMP,
    // o watchdog de staleness pré-existente (mesmo padrão "6 heartbeats")
    // não pode invalidar a fase antes do loop abaixo terminar — isolando
    // este teste ao mecanismo do Fix C especificamente.
    sensors_set_bench_clt_iat(true, 900, 250);
    ems::engine::cfg::g_eng_cfg.cmp_phase_state = ems::engine::cfg::kCmpPhaseCalibratedA;

    // Ancora diretamente (bypassa o gate de confirmação do Fix B de
    // propósito — este teste é sobre o Fix C isoladamente).
    ecu_sched_encoder_phase_set_anchor(500000u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u, "âncora marcada, phase_valid()=1");
    CHECK_EQ(ecu_sched_encoder_seq_arm_stall_count(), 0u, "stall count=0 no início");

    // Garante zero armamentos de forma determinística: tim5_now CONSTANTE
    // (nunca avança) faz ecu_sched_encoder_omega_sample() descartar toda
    // atualização (delta_tim5≤0 é tratado como "relógio não avançou" —
    // ver comentário da função), então ω nunca fica válido e
    // try_arm_sequential_due() retorna cedo sempre (omega_valid()==0) —
    // mesmo efeito prático de uma âncora corrompida que nunca encontra
    // janela, mas sem depender da geometria exata de fuel_calc/janela
    // ≤60° coincidir ou não com os números sintéticos deste teste.
    uint32_t t = 1000u;
    for (uint32_t i = 0; i < ECU_SEQ_ARM_STALL_HEAVY_TICKS; ++i) {
        ecu_sched_encoder_heartbeat_tick(t, 1000u, 0u, 0u);
        t += 1000u;
    }
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "watchdog de stall forçou fallback para presync");
    CHECK_EQ(ecu_sched_encoder_seq_arm_stall_count(), 1u,
             "contador de diagnóstico incrementou 1×");

    sensors_set_bench_clt_iat(false, 0, 0);
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_due_head_dispatches_inline(void) {
    section("ecu_sched: encoder queue — alvo já devido/passado não trava CCR3 (TIM2 posição)");
    ecu_sched_test_reset();

    // Alvo no passado com ω inválido: arm_channel_with_lead empurra para
    // now+min_lead=CNT e, sem o despacho inline, CCR3==CNT + CC3IF limpo
    // deixava a fila muda até o wrap de 32 bits.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 1000u, ECU_ACT_INJ_ON);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "alvo no passado executado inline — fila não fica presa");
    CHECK_TRUE(ecu_sched_encoder_enc_evt_execute_count() >= 1u,
               "passado despachado pelo caminho due (ts<=CNT)");
    CHECK_EQ(ecu_sched_encoder_test_get_dier(), 0u,
             "CC3IE off — fila vazia depois do despacho inline");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_seq_to_presync_force_closes_pins(void) {
    section("ecu_sched: queda sequencial→presync (CMP off) força fecho de pinos");
    ecu_sched_test_reset();
    ems::hal::out_pins_test_reset_stubs();

    // Primeiro heartbeat com fase válida e ω ainda inválido: marca
    // last_builder=sequencial (handoff) sem armar cilindros.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    ecu_sched_encoder_heartbeat_tick(50u, 50u, 0u, 0u);

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 100u, ECU_ACT_DWELL_START);
    ecu_sched_encoder_test_set_tim2_cnt(100u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_TRUE((ems::hal::out_pins_test_bsrr_snapshot(0u) & (1u << 15u)) != 0u,
               "INJ1 HIGH após INJ_ON sequencial");
    CHECK_TRUE(ecu_sched_test_get_dwell_arm_tick(0u) != 0u,
               "dwell armado após DWELL_START sequencial");

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_OFF);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 5000u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 2u, "OFF/SPARK ainda pendentes");

    // Queda para presync: o heartbeat vê last_builder=seq e tem de fechar
    // os pinos cujo OFF/SPARK vai ser purgado pelo rebuild.
    ecu_sched_encoder_phase_invalidate();
    ecu_sched_encoder_test_set_tim2_cnt(2000u);
    ecu_sched_encoder_heartbeat_tick(2000u, 4000u, 0u, 0u);

    CHECK_EQ(ecu_sched_is_sequential(), 0u, "caiu para presync");
    CHECK_TRUE((ems::hal::out_pins_test_bsrr_snapshot(0u) & (1u << (15u + 16u))) != 0u,
               "INJ1 forçado LOW na queda para presync");
    CHECK_EQ(ecu_sched_test_get_dwell_arm_tick(0u), 0u,
             "dwell watchdog libertado na queda para presync");

    ecu_sched_test_reset();
    ems::hal::out_pins_test_reset_stubs();
}

void test_ecu_sched_encoder_presync_after_cmp_loss_keeps_dispatcher(void) {
    section("ecu_sched: presync após perda de CMP — CCR3 fica à frente de CNT");
    ecu_sched_test_reset();
    ckp_test_reset();
    sensors_set_bench_clt_iat(false, 0, 0);
    ems::engine::cfg::g_eng_cfg.cmp_phase_state = ems::engine::cfg::kCmpPhaseCalibratedA;

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 1000u, 1u);
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 1000u + kCmpSpanCounts, 2u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u, "fase ancorada antes da perda");

    uint32_t t = 3000u;
    uint32_t heartbeats_since_ok = ecu_sched_encoder_test_get_cmp_heartbeats_since_ok();
    while (heartbeats_since_ok < kMaxHeartbeatsWithoutCmp) {
        ecu_sched_encoder_heartbeat_tick(t, t, 1000u + kCmpSpanCounts, 2u);
        heartbeats_since_ok = ecu_sched_encoder_test_get_cmp_heartbeats_since_ok();
        t += 1000u;
    }
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "staleness caiu para presync");

    // Hardware-like: CNT == now_raw do heartbeat. Dwell/PW default são
    // longos → vários alvos caem no passado e, sem o despacho inline,
    // a cabeça da fila armava CCR3 <= CNT e os pulsos paravam.
    const uint32_t now = t;
    ecu_sched_encoder_test_set_tim2_cnt(now);
    ecu_sched_encoder_heartbeat_tick(now, now + 1000u, 1000u + kCmpSpanCounts, 2u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "continua em presync");
    CHECK_TRUE(ecu_sched_encoder_presync_call_count() > 0u,
               "builder presync correu depois da perda de CMP");

    const uint8_t n = ecu_sched_encoder_test_get_evt_count();
    if (n == 0u) {
        CHECK_EQ(ecu_sched_encoder_test_get_dier(), 0u,
                 "fila vazia (alvos devidos já despachados) — CC3IE off");
    } else {
        CHECK_TRUE((int32_t)(ecu_sched_encoder_test_get_ccr3() - now) > 0,
                   "cabeça da fila à frente de CNT — dispatcher vivo");
        CHECK_TRUE(ecu_sched_encoder_test_get_dier() != 0u,
                   "CC3IE on enquanto há eventos futuros");
    }

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    sensors_set_bench_clt_iat(false, 0, 0);
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_heartbeat_publish_snapshot(void) {
    section("ecu_sched: encoder heartbeat — publishes ckp_snapshot() (task #13)");
    ecu_sched_test_reset();
    ckp_test_reset();

    // Default: phase invalid (uncalibrated), health ok -> HALF_SYNC.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    CkpSnapshot snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::HALF_SYNC,
               "phase invalid + health ok -> HALF_SYNC published");
    CHECK_EQ(snap.cmp_confirms, 0u, "HALF_SYNC: cmp_confirms=0");

    // rpm_x10: seed omega to a known ratio (874/1e6, the same 200rpm-
    // equivalent recipe used by the omega estimator's own regression test)
    // and hand-verify the conversion: omega=57 -> rpm_x10=1990 (600e9*57 /
    // (16*16384*65536), integer truncation).
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(874u, 1000000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 57, "omega seeded to 57 (200rpm-equivalent)");
    snap = ckp_snapshot();
    CHECK_EQ(snap.rpm_x10, 1990u, "rpm_x10 derived correctly from omega_x65536");

    // Health fault (encoder_sync::set_health_ok(false), simulating task #14's
    // mt6835_ok() poll having detected a failure) -> LOSS_OF_SYNC overrides
    // everything else, regardless of phase state.
    ems::drv::encoder_sync::set_health_ok(false);
    ecu_sched_encoder_heartbeat_tick(875u, 1000001u, 0u, 0u);
    snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::LOSS_OF_SYNC,
               "health_ok()=false -> LOSS_OF_SYNC published");
    ems::drv::encoder_sync::set_health_ok(true);

    // Phase valid (seeded directly — bypasses the EMS_MT6835_CMP_PHASE_CALIBRATED
    // gate, which only guards the call site inside the heartbeat, not the
    // underlying phase tracker itself) -> FULL_SYNC, cmp_confirms=2.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    ecu_sched_encoder_heartbeat_tick(876u, 1000002u, 0u, 0u);
    snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::FULL_SYNC,
               "phase valid + health ok -> FULL_SYNC published");
    CHECK_EQ(snap.cmp_confirms, 2u, "FULL_SYNC: cmp_confirms=2");
    CHECK_TRUE(snap.phase_A, "phase_A reflects ecu_sched_encoder_phase_at() at publish time");

    // Staleness: many heartbeats with no new CMP edge eventually invalidate
    // the phase (fallback FULL_SYNC->HALF_SYNC) — exact threshold already
    // covered by test_encoder_sync_staleness(); here just confirm the
    // integration actually fires within a safe margin above it.
    for (uint32_t i = 0u; i < 10u; ++i) {
        ecu_sched_encoder_heartbeat_tick(877u + i, 1000003u + i, 0u, 0u);
    }
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "10 heartbeats without a new CMP edge (> prod limit=6): phase invalidated");
    snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::HALF_SYNC,
               "staleness fallback published as HALF_SYNC, not stuck at FULL_SYNC");

    ecu_sched_test_reset();
    ckp_test_reset();
}

void test_ecu_sched_encoder_heartbeat_publish_crank_deg(void) {
    section("ecu_sched: encoder heartbeat — crank_deg derivado de tim2_now");
    ecu_sched_test_reset();
    ckp_test_reset();

    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().crank_deg, 0u, "tim2_now=0 -> crank_deg=0");
    CHECK_EQ(ckp_snapshot().tim2_cnt, 0u, "tim2_now=0 -> tim2_cnt=0");

    ecu_sched_encoder_heartbeat_tick(4096u, 100u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().crank_deg, 90u, "tim2_now=4096 -> crank_deg=90");
    CHECK_EQ(ckp_snapshot().tim2_cnt, 4096u, "tim2_now=4096 -> tim2_cnt");

    ecu_sched_encoder_heartbeat_tick(8192u, 200u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().crank_deg, 180u, "tim2_now=8192 -> crank_deg=180");

    ecu_sched_encoder_heartbeat_tick(16384u + 4096u, 300u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().crank_deg, 90u, "wrap de revolução: crank_deg correcto");
    CHECK_EQ(ckp_snapshot().tim2_cnt, 16384u + 4096u, "tim2_cnt raw preserved");

    ecu_sched_test_reset();
    ckp_test_reset();
}

// ── Split light/heavy do heartbeat TIM2_CH4 ─────────────────────────────────
// ecu_sched_encoder_heartbeat_subtick() é chamada a CADA CC4IF (256 counts,
// ~64×/volta), não só 1×/volta como ecu_sched_encoder_heartbeat_tick()
// (testada directamente nos 3 testes acima, que continuam a passar
// inalterados — não passam por este wrapper). Verifica só a cadência do
// split em si: o caminho pesado (aqui detectado via
// cmp_heartbeats_since_ok, que só o caminho pesado incrementa) dispara
// exactamente 1×/64 chamadas, nunca nas outras 63.
void test_ecu_sched_encoder_heartbeat_subtick_cadence(void) {
    section("ecu_sched: heartbeat_subtick — split light/heavy, cadência 1/64");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_test_get_subtick_count(), 0u, "pré-condição: subtick_count=0");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 0u,
             "pré-condição: caminho pesado nunca correu");

    // cmp_edge_count constante (0) em todas as chamadas — a avaliação de
    // flanco CMP fica sempre inactiva (delta=0), isolando o teste só à
    // cadência do split, sem interferência da lógica de validação de CMP.
    for (uint32_t i = 1u; i <= 63u; ++i) {
        ecu_sched_encoder_heartbeat_subtick(i * 256u, i * 100u, 0u, 0u);
        CHECK_EQ(ecu_sched_encoder_test_get_subtick_count(), static_cast<uint8_t>(i),
                 "subtick_count incrementa a cada sub-tick");
    }
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 0u,
             "63 sub-ticks: caminho pesado ainda não correu nenhuma vez");

    // 64ª chamada: dispara o caminho pesado, reseta subtick_count.
    ecu_sched_encoder_heartbeat_subtick(64u * 256u, 64u * 100u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_subtick_count(), 0u,
             "64º sub-tick: subtick_count reseta a 0");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 1u,
             "64º sub-tick: caminho pesado correu exactamente 1×");

    // Segundo ciclo completo de 64 — confirma que a cadência se repete, não
    // é um efeito de arranque único.
    for (uint32_t i = 1u; i <= 64u; ++i) {
        ecu_sched_encoder_heartbeat_subtick((64u + i) * 256u, (64u + i) * 100u, 0u, 0u);
    }
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 2u,
             "2º ciclo de 64: caminho pesado corre de novo exactamente 1×");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_omega_only_on_heavy_tick(void) {
    section("ecu_sched: refresh de dwell só no tick pesado, não a cada sub-tick");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(4000u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    const uint32_t now = encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);
    const uint32_t refreshes0 = ecu_sched_encoder_test_get_omega_refresh_count();

    // ω sobe >2% mas só via sub-tick (não completa 64): refresh NÃO corre.
    ecu_sched_encoder_omega_test_reset();
    ecu_sched_encoder_omega_sample(0u, 0u);
    ecu_sched_encoder_omega_sample(700u, 1000u);
    ecu_sched_encoder_test_set_tim2_cnt(now);
    for (uint32_t i = 1u; i <= 10u; ++i) {
        ecu_sched_encoder_heartbeat_subtick(now + i, 1000u + i, 0u, 1u);
    }
    CHECK_EQ(ecu_sched_encoder_test_get_omega_refresh_count(), refreshes0,
             "sub-tick não reposiciona dwell (refresh só no tick pesado)");

    ecu_sched_encoder_heartbeat_tick(now, 1000u, 0u, 1u);
    CHECK_TRUE(ecu_sched_encoder_test_get_omega_refresh_count() > refreshes0,
               "tick pesado faz o refresh de dwell");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

// Prova a LIGAÇÃO (não a lógica — essa está coberta em detalhe em
// test_misfire_encoder.cpp): heartbeat_subtick() alimenta mesmo
// misfire_encoder_on_sample(), não só chama o caminho pesado. Mesma
// sequência numérica de test_misfire_encoder_threshold_debounce_and_inertness
// (1 ciclo lento só, o suficiente para confirmar a ligação), mas conduzida
// através de ecu_sched_encoder_heartbeat_subtick() em vez de chamar
// misfire_encoder_on_sample() directamente.
void test_ecu_sched_encoder_heartbeat_subtick_feeds_misfire(void) {
    section("ecu_sched: heartbeat_subtick alimenta misfire_encoder (wiring)");
    ecu_sched_test_reset();
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    misfire_encoder_init();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);

    ecu_sched_encoder_heartbeat_subtick(15360u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(15616u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(15872u, 2000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(16128u, 3000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(32768u, 4200u, 0u, 0u);  // entra cyl0, Δ=1200 (lento)
    ecu_sched_encoder_heartbeat_subtick(35840u, 5200u, 0u, 0u);  // sai → avalia

    CHECK_EQ(misfire_encoder_test_get_debounce(0u), 1u,
             "heartbeat_subtick alimenta misfire_encoder_on_sample via wiring real");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_conversion(void) {
    section("ecu_sched: encoder degrees<->counts conversion (pure math)");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(0u), 0u, "0 deg -> 0 counts");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(90u), 4096u, "90 deg -> 1/4 rev (4096)");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(270u), 12288u, "270 deg -> 3/4 rev (12288)");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(360u), 0u, "360 deg wraps to 0 (mod 360 domain)");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(359u), 16338u,
             "359 deg -> 16338 (359*16384/360, truncated)");

    // Origin residue property: a calibrator writing 400 or 40 must produce
    // IDENTICAL encoder-mode timing (only encoder_tdc1_origin_deg % 360 is
    // load-bearing here — TIM2 wraps every 360, not 720 like
    // trigger_tooth0_engine_deg's tooth-wheel domain, the Hall-path field
    // this one used to share before it got its own dedicated field). If
    // this ever diverges, the field is silently carrying phase information
    // again (the exact bug class this session has been avoiding).
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 40u;
    const uint32_t with_40 = ecu_sched_encoder_test_engine_deg_to_counts(90u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 400u;
    const uint32_t with_400 = ecu_sched_encoder_test_engine_deg_to_counts(90u);
    CHECK_EQ(with_400, with_40, "origin=400 and origin=40 (400%360) give identical counts");
    CHECK_EQ(with_40, 2275u, "90 deg, origin=40 -> crank_deg=410%360=50 -> 50*16384/360=2275");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;

    // rev_target_to_absolute: half-open window (now_raw, now_raw+16384].
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(100u, 50u), 100u,
             "forward within same image: target ahead of now, no wrap");
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(100u, 100u), 16484u,
             "boundary: target==now must land at now+16384 (next rev), not now+0");
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(10u, 16380u), 16394u,
             "wrap forward: target just past the 16384 boundary");
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(100u, 0x10000032u), 0x10000064u,
             "upper bits of a 32-bit raw count preserved across the addition");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_tdc1_calibrate(void) {
    section("ecu_sched: encoder TDC1 calibration (pure math, round-trip with conversion)");

    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(0u), 0u,
             "TIM2==0 at TDC1 -> origin=0 (raw zero already IS TDC1)");
    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(8192u), 180u,
             "TIM2==8192 (half a rev) at TDC1 -> origin=180");
    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(4096u), 270u,
             "TIM2==4096 (1/4 rev = 90 deg) at TDC1 -> origin=270 (360-90)");
    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(0x10001000u), 270u,
             "upper bits of a 32-bit raw count ignored — only mod 16384 matters");

    // Round-trip: whatever calibrate_from_raw() returns, feeding it back as
    // the origin must make engine_deg_to_counts_in_rev(0) reproduce the
    // ORIGINAL raw reading (mod 16384) — this is the actual contract the
    // field exists to satisfy (TDC1 in engine-degree space maps back to the
    // exact TIM2 position it was measured at).
    ecu_sched_test_reset();
    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    const uint32_t raws[] = {0u, 1u, 4095u, 4096u, 8191u, 8192u, 12288u, 16383u};
    for (uint8_t i = 0u; i < sizeof(raws) / sizeof(raws[0]); ++i) {
        const uint32_t raw = raws[i];
        const uint16_t origin = ecu_sched_encoder_tdc1_calibrate_from_raw(raw);
        ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = origin;
        const uint32_t back = ecu_sched_encoder_test_engine_deg_to_counts(0u);
        // origin_deg é um INTEIRO 0-359 — 360 não divide 16384 exactamente
        // (1 grau ≈ 45,5 counts), então o round-trip nunca é exacto ao
        // count: a tolerância real é ~meio grau (±23 counts), não ±1 (esse
        // seria só o caso onde os dois lados arredondam para o mesmo
        // valor por coincidência). Distância circular (mod 16384): raw=16383
        // e back=0 são POSIÇÕES ADJACENTES (a 1 count uma da outra através
        // do wrap), não uma diferença de ~16384 — diff ingénuo confundia as
        // duas.
        int32_t diff = static_cast<int32_t>(back) - static_cast<int32_t>(raw);
        if (diff > 8192) { diff -= 16384; }
        if (diff < -8192) { diff += 16384; }
        CHECK_TRUE(diff >= -46 && diff <= 46,
                   "round-trip TIM2 raw -> origin_deg -> counts_in_rev, dentro de meio grau (distância circular)");
    }
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_cmp_phase_calibrate(void) {
    section("ecu_sched: encoder CMP phase calibration (pure math, comando 'M')");

    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(0u, 0u),
             ems::engine::cfg::kCmpPhaseCalibratedA,
             "flanco CMP no mesmo raw do PMS1 (delta=0, par) -> fase A");
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(16384u, 0u),
             ems::engine::cfg::kCmpPhaseCalibratedB,
             "flanco CMP 1 volta antes do PMS1 (delta=16384, ímpar) -> fase B");
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(32768u, 0u),
             ems::engine::cfg::kCmpPhaseCalibratedA,
             "flanco CMP 2 voltas antes do PMS1 (delta=32768, par) -> fase A");
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(0u, 100u),
             ems::engine::cfg::kCmpPhaseCalibratedB,
             "flanco CMP 'à frente' do PMS1 (delta=-100, floor_div arredonda "
             "p/ -infinito -> quociente -1, ímpar) -> fase B");
    // Wraparound perto do limite de uint32_t: tim2_raw_at_cmp_edge=0xFFFFFFFF
    // com tim2_raw_at_tdc1=100 — subtração sem sinal envolve (100 - 0xFFFFFFFF
    // mod 2^32 = 101), caso físico real de "o flanco foi capturado mesmo
    // antes do contador dar a volta" — confirma que o cast p/ int32_t não
    // quebra perto do limite.
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(100u, 0xFFFFFFFFu),
             ems::engine::cfg::kCmpPhaseCalibratedA,
             "flanco CMP a 101 counts do PMS1 através do wrap de uint32_t -> fase A");
}

// Gap identificado na revisão 2026-08-15: nunca existiu teste de
// engine_config_valid()/serialize()/load() (nem para encoder_tdc1_origin_deg,
// nem agora para cmp_phase_state) — não há test_engine_config.cpp dedicado.
// Cobre aqui o suficiente para o campo novo: round-trip válido, rejeição de
// valor fora do domínio (cai nos defaults, não aplica parcial) e rejeição
// por magic errado.
void test_engine_config_cmp_phase_state_roundtrip(void) {
    section("engine_config: cmp_phase_state — valid()/serialize()/load() round-trip");

    using namespace ems::engine::cfg;
    const EngineConfigRam saved = g_eng_cfg;

    // Round-trip válido.
    g_eng_cfg.cmp_phase_state = kCmpPhaseCalibratedB;
    uint8_t buf[256] = {};
    engine_config_serialize(buf, kEngineConfigMinPageLen);
    g_eng_cfg.cmp_phase_state = kCmpPhaseUncalibrated;  // adultera antes do load
    engine_config_load(buf, kEngineConfigMinPageLen);
    CHECK_EQ(g_eng_cfg.cmp_phase_state, static_cast<uint8_t>(kCmpPhaseCalibratedB),
             "serialize(B) -> load() reproduz cmp_phase_state=B");

    // Valor fora do domínio (byte 12 corrompido diretamente, simula NVM
    // antigo/lixo) — engine_config_valid() deve rejeitar o struct INTEIRO,
    // não só o campo mau: g_eng_cfg fica intocado, não parcialmente aplicado.
    g_eng_cfg.cmp_phase_state = kCmpPhaseCalibratedA;
    const uint8_t sentinel_before = g_eng_cfg.cmp_phase_state;
    buf[12] = 3u;  // > kCmpPhaseCalibratedB — inválido
    engine_config_load(buf, kEngineConfigMinPageLen);
    CHECK_EQ(g_eng_cfg.cmp_phase_state, sentinel_before,
             "cmp_phase_state=3 (fora do domínio) rejeitado — g_eng_cfg não muda");

    EngineConfigRam probe = saved;
    probe.cmp_phase_state = 3u;
    CHECK_FALSE(engine_config_valid(probe), "engine_config_valid() rejeita cmp_phase_state>2 diretamente");

    // Magic errado — load() não deve tocar g_eng_cfg mesmo com bytes 0-13
    // válidos (mesma disciplina já usada por encoder_tdc1_origin_deg).
    g_eng_cfg.cmp_phase_state = kCmpPhaseCalibratedB;
    const uint8_t sentinel2 = g_eng_cfg.cmp_phase_state;
    engine_config_serialize(buf, kEngineConfigMinPageLen);
    buf[12] = static_cast<uint8_t>(kCmpPhaseCalibratedA);  // válido, mas...
    buf[14] = 0xFFu; buf[15] = 0xFFu;                        // ...magic errado
    engine_config_load(buf, kEngineConfigMinPageLen);
    CHECK_EQ(g_eng_cfg.cmp_phase_state, sentinel2,
             "magic errado -> load() não aplica nada, mesmo com cmp_phase_state válido no buffer");

    g_eng_cfg = saved;
}

// Shared by encoder presync + sequential tests (find first matching queue evt).
static uint8_t encoder_evt_find_ch(uint8_t want_ch, uint8_t want_high,
                                   uint32_t *out_ts) {
    for (uint8_t i = 0u; i < ecu_sched_encoder_test_get_evt_count(); ++i) {
        uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
        ecu_sched_encoder_test_get_evt(i, &ts, &ch, &high);
        if (ch == want_ch && high == want_high) {
            if (out_ts != nullptr) { *out_ts = ts; }
            return 1u;
        }
    }
    return 0u;
}

void test_ecu_sched_encoder_recompute_presync(void) {
    section("ecu_sched: encoder heartbeat recompute — presync wasted pairs @ 180°");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_advance_deg(10u);            // spark_a=350, spark_b=170
    ecu_sched_set_eoi_lead_deg(355u);          // eoi_deg=5
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(2000u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);   // omega=0.5
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 32768, "omega seeded to 0.5 for deterministic spans");

    // Hand-computed (origin=0, now_raw=1500, dwell_span=1000):
    //   pair A (IGN1/IGN4): spark_a=350 → 15928, dwell=14928
    //   pair B (IGN3/IGN2): spark_b=170 → 7736,  dwell=6736
    //   inj: eoi=5 → 16611, inj_on=15611 (g_inj_pw_ticks is per-opening, no /2)
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u,
             "2 pairs × 2 coils × (dwell+spark) + 4 INJ (on+off)");
    CHECK_EQ(ecu_sched_is_sequential(), 0u,
             "presync builder clears g_knock_sequential (wasted-spark)");

    uint32_t spark_a = 0u, spark_b = 0u, dwell_a = 0u, dwell_b = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark_a), 1u, "pair A IGN1 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 1u, "pair A IGN4 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark_b), 1u, "pair B IGN3 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 1u, "pair B IGN2 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell_a), 1u, "pair A IGN1 DWELL");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 1u, &dwell_b), 1u, "pair B IGN3 DWELL");

    CHECK_EQ(spark_a, 15928u, "pair A spark @ 350°");
    CHECK_EQ(dwell_a, 14928u, "pair A dwell = spark − span");
    CHECK_EQ(spark_b, 7736u, "pair B spark @ 170°");
    CHECK_EQ(dwell_b, 6736u, "pair B dwell = spark − span");
    CHECK_TRUE(spark_a != spark_b, "pairs at distinct crank angles (180° apart)");

    uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
    // Chronological: pair-B dwell first in queue.
    ecu_sched_encoder_test_get_evt(0u, &ts, &ch, &high);
    CHECK_EQ(ts, 6736u, "evt0: earliest = pair B dwell");
    CHECK_EQ(high, 1u, "evt0: DWELL high=1");

    ecu_sched_encoder_heartbeat_tick(1500u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u,
             "repeated heartbeat: still exactly 16, no duplication from purge+rebuild");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_recompute_presync_next_rev_if_on_past(void) {
    section("ecu_sched: encoder presync — ON no passado empurra o par p/ a próxima volta");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_advance_deg(10u);            // spark_a=350 → 15928
    ecu_sched_set_eoi_lead_deg(355u);          // eoi=5 → 227
    ecu_sched_set_dwell_ticks(2000u);          // span=2000 @ ω=1
    ecu_sched_set_inj_pw_ticks(2000u);         // per-opening PW; span=2000 @ ω=1
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);

    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega=1.0");

    // now=15000: spark_a=15928 nesta volta, dwell=13928 já passou;
    // eoi=16611, inj_on=14611 já passou. Sem o shift o ON ia para "agora"
    // e o pulso ficava com o resto até ao spark/EOI.
    const uint32_t now = 15000u;
    ecu_sched_encoder_test_set_tim2_cnt(now);
    ecu_sched_encoder_heartbeat_tick(now, now, 0u, 0u);

    uint32_t spark_a = 0u, dwell_a = 0u, inj_on = 0u, inj_off = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark_a), 1u, "pair A SPARK presente");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell_a), 1u, "pair A DWELL presente");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on), 1u, "INJ_ON presente");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off), 1u, "INJ_OFF presente");

    CHECK_EQ(spark_a, 15928u + 16384u, "spark_a adiado 1 volta (dwell estava no passado)");
    CHECK_EQ(dwell_a, 13928u + 16384u, "dwell_a adiado 1 volta");
    CHECK_EQ(spark_a - dwell_a, 2000u, "dwell completo preservado (não truncado)");
    CHECK_TRUE((int32_t)(dwell_a - now) > 0, "dwell agora está à frente de now");

    CHECK_EQ(inj_off, 16611u + 16384u, "EOI adiado 1 volta (inj_on estava no passado)");
    CHECK_EQ(inj_on, 14611u + 16384u, "inj_on adiado 1 volta");
    CHECK_EQ(inj_off - inj_on, 2000u, "PW completo preservado (não truncado)");
    CHECK_TRUE((int32_t)(inj_on - now) > 0, "inj_on agora está à frente de now");

    // pair B: spark_b=170° já passou esta volta → próxima ocorrência nesta
    // geometria já fica à frente; dwell também — sem shift extra.
    uint32_t spark_b = 0u, dwell_b = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark_b), 1u, "pair B SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 1u, &dwell_b), 1u, "pair B DWELL");
    CHECK_EQ(spark_b, 24120u, "pair B spark = próxima ocorrência (sem +16384 extra)");
    CHECK_EQ(dwell_b, 22120u, "pair B dwell = spark − 2000, já à frente");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_recompute_presync_ign_inhibit_mask_gate(void) {
    section("ecu_sched: encoder heartbeat recompute — máscara de ignição corta metade de um par "
            "wasted-spark (fix 2026-08-15, achado #3: recompute_presync() é o caminho REALMENTE "
            "ativo hoje, EMS_MT6835_CMP_PHASE_CALIBRATED=0 ⇒ try_arm_sequential_due nunca corre "
            "em produção — corrigir só o caminho sequencial não teria efeito nenhum na config atual)");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(355u);
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(2000u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);

    // kWastedIgnPairA = {IGN1, IGN4}. IGN1=ECU_CH_IGN1(7)→k_ign_ch_to_bit=bit0;
    // IGN4=ECU_CH_IGN4(4)→bit3. Máscara 0x01 corta só IGN1 — prova que o gate
    // é por canal, não pelo par inteiro (a companion IGN4 continua a armar).
    ecu_sched_set_ign_inhibit_mask(0x01u);

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);   // omega=0.5

    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, nullptr), 0u,
             "IGN1 DWELL_START NÃO armado — bit0 mascarado");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 0u,
             "IGN1 SPARK também não armado");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 1u, nullptr), 1u,
             "IGN4 (companion do mesmo par) continua armado — gate por canal, não por par");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 1u, nullptr), 1u,
             "pair B (IGN3/IGN2) intocado pela máscara de pair A");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, nullptr), 1u,
             "INJ continua a armar — máscara de ignição não corta fuel");

    ecu_sched_set_ign_inhibit_mask(0u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_recompute_presync_bank_toggle(void) {
    section("ecu_sched: encoder heartbeat recompute — presync semi-sequential both banks @ 180°");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SEMI_SEQUENTIAL);
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(355u);          // eoi=5°, bank B = 185°
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(2000u);         // /2 → 1000 ticks; ω=0.5 → span=1000

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u,
             "4 IGN (dwell+spark) + 4 INJ (on+off), semi: both banks every rev");

    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, nullptr), 1u, "bank A INJ1");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ4, 1u, nullptr), 1u, "bank A INJ4");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ2, 1u, nullptr), 1u, "bank B INJ2");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ3, 1u, nullptr), 1u, "bank B INJ3");

    uint32_t inj_on_a = 0u, inj_off_a = 0u, inj_on_b = 0u, inj_off_b = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on_a), 1u, "bank A INJ1 ON");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off_a), 1u, "bank A INJ1 OFF");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ2, 1u, &inj_on_b), 1u, "bank B INJ2 ON");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ2, 0u, &inj_off_b), 1u, "bank B INJ2 OFF");
    CHECK_EQ(inj_off_a - inj_on_a, 1000u, "semi uses committed per-opening PW (2000 ticks, ω=0.5 → span=1000)");
    CHECK_EQ(inj_off_b - inj_on_b, 1000u, "bank B same per-opening PW");
    CHECK_TRUE(inj_off_a != inj_off_b, "banks at distinct crank angles (180°)");

    ecu_sched_encoder_heartbeat_tick(2000u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u, "still 16 next rev (no bank drop)");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, nullptr), 1u, "INJ1 still armed next rev");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ2, 1u, nullptr), 1u, "INJ2 still armed next rev");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_recompute_presync_pw_clamp(void) {
    section("ecu_sched: encoder heartbeat recompute — presync PW duty clamp");
    ecu_sched_test_reset();

    const uint32_t before = ecu_sched_pw_duty_clamp_count();
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_inj_pw_ticks(2000000u);   // huge PW ticks, forces a span > 90% of a rev at omega=1.0
    ecu_sched_set_dwell_ticks(0u);

    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);  // omega=1.0 -> inj_pw_span way over a rev

    CHECK_TRUE(ecu_sched_pw_duty_clamp_count() > before,
               "oversized presync PW span clamped to 90% of a revolution");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_presync_multispark(void) {
    section("ecu_sched: encoder presync — multi-spark per wasted pair");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_advance_deg(20u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_mspark(1u, 100u, 18u);

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);

    CHECK_EQ(ecu_sched_is_sequential(), 0u, "presync stays wasted-spark");

    // Primary: 2 pairs × 2 coils × 2 = 8; multi: +8; inj sim: 8 → 24.
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 24u,
             "presync+1 mspark/pair: 16 IGN + 8 INJ");

    uint32_t spark_a = 0u, spark_b = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark_a), 1u, "pair A primary spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark_b), 1u, "pair B primary spark");
    CHECK_TRUE(spark_a != spark_b, "multi-spark still on distinct pair angles");

    uint8_t ign1_count = 0u;
    for (uint8_t i = 0u; i < ecu_sched_encoder_test_get_evt_count(); ++i) {
        uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
        ecu_sched_encoder_test_get_evt(i, &ts, &ch, &high);
        if (ch == ECU_CH_IGN1) { ++ign1_count; }
    }
    CHECK_EQ(ign1_count, 4u,
             "pair-A coil: primary + 1 multi-spark pair (4 events)");

    ecu_sched_encoder_heartbeat_tick(1500u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 24u,
             "repeated heartbeat: still 24, no duplication");

    ecu_sched_set_mspark(0u, 0u, 18u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

// ── Disparo sequencial encoder (fase-consciente) ───────────────────────────
// Helpers: o mock TIM2_CNT e o parâmetro tim2_now do heartbeat são
// independentes — testes multi-passagem têm de os sincronizar. Qualquer
// sequência com phase_valid tem de ficar abaixo de kMaxHeartbeatsWithoutCmp=6
// (ou reancorar via flanco CMP aceite) senão o staleness invalida a fase.
// omega seed: encoder_seq_seed_omega() em test/fixtures.h.
// encoder_evt_find_ch() está definido acima (antes dos testes de presync).

void test_ecu_sched_encoder_placement(void) {
    section("ecu_sched: encoder engine_deg720_to_absolute placement");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    // Discriminator vectors (hand-verified) — composition identical to
    // engine_deg720_to_absolute's body. Integer deg→counts cannot hit residue
    // 50/3000 exactly; these lock the +16384 correction oracle.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    uint32_t cand = ecu_sched_encoder_test_rev_target_to_absolute(50u, 100u);
    CHECK_EQ(cand, 16434u, "disc1: rev_target_to_absolute(50,100)=16434");
    CHECK_EQ(ecu_sched_encoder_phase_at(cand), ECU_PHASE_B,
             "disc1: candidate lands in wrong phase (B)");
    CHECK_EQ(cand + 16384u, 32818u, "disc1: +16384 correction -> 32818");

    ecu_sched_encoder_phase_set_anchor(5000u, ECU_PHASE_A);
    cand = ecu_sched_encoder_test_rev_target_to_absolute(3000u, 6000u);
    CHECK_EQ(cand, 19384u, "disc2: rev_target_to_absolute(3000,6000)=19384");
    CHECK_EQ(ecu_sched_encoder_phase_at(cand), ECU_PHASE_A,
             "disc2: candidate lands in wrong phase for B-target (A)");
    CHECK_EQ(cand + 16384u, 35768u, "disc2: +16384 correction -> 35768");

    // End-to-end deg720: target phase A at 0° with anchor=0/A, now=100.
    // counts_in_rev(0)=0 -> candidate=16384 (phase B) -> +16384 -> 32768.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_test_deg720_to_absolute(0u, 100u), 32768u,
             "deg720(0°, now=100, anchor=0/A) -> 32768 (phase A)");
    CHECK_EQ(ecu_sched_encoder_phase_at(32768u), ECU_PHASE_A,
             "result phase matches target A");
    // Target phase B at 360°: same counts, candidate=16384 already B -> no fix.
    CHECK_EQ(ecu_sched_encoder_test_deg720_to_absolute(360u, 100u), 16384u,
             "deg720(360°, now=100, anchor=0/A) -> 16384 (phase B, no +16384)");
    CHECK_EQ(ecu_sched_encoder_phase_at(16384u), ECU_PHASE_B,
             "result phase matches target B");

    // Misaligned anchor (disc2 geometry via deg720): 3000 counts ≈ 65.9° →
    // use 66° (66*16384/360=3003). Close enough to exercise the oracle path.
    ecu_sched_encoder_phase_set_anchor(5000u, ECU_PHASE_A);
    const uint32_t abs_b = ecu_sched_encoder_test_deg720_to_absolute(360u + 66u, 6000u);
    CHECK_EQ(ecu_sched_encoder_phase_at(abs_b), ECU_PHASE_B,
             "misaligned-anchor deg720 B-target lands in phase B");

    // Negative-delta / wrap cases reuse phase_at coverage (99999/83616 vs
    // anchor=100000) — phase oracle inherits wrap-safety; no new logic here.
    ecu_sched_encoder_phase_set_anchor(100000u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_phase_at(99999u), ECU_PHASE_B,
             "phase_at just before anchor: previous phase (neg delta)");
    CHECK_EQ(ecu_sched_encoder_phase_at(83616u), ECU_PHASE_B,
             "phase_at -1 rev from anchor=100000: phase flips");
    CHECK_EQ(ecu_sched_encoder_phase_at(83615u), ECU_PHASE_A,
             "phase_at just before -1 rev boundary");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_distinct_targets(void) {
    section("ecu_sched: encoder sequential — distinct per-cyl targets");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(2000u);

    encoder_seq_seed_omega();
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "pre: still presync");

    // Late-arm ≤60°: arm cyl0 then cyl2 in their windows (events accumulate).
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "phase valid -> sequential");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 1u, "cyl0 armed in window");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 4u,
             "only cyl0 (4 events) while cyl2 still outside window");

    encoder_seq_arm_cyl_in_window(2u, 4000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 8u,
             "cyl0+cyl2 = 8 events after second window arm");

    uint32_t spark0 = 0u, spark2 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "cyl0 SPARK present");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark2), 1u, "cyl2 SPARK present");
    CHECK_TRUE(spark0 != spark2, "cyl0 and cyl2 spark targets are distinct");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_trims(void) {
    section("ecu_sched: encoder sequential — per-cyl ign/fuel trims");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(2000u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 0u, 0u);
    encoder_seq_arm_cyl_in_window(2u, 4000u, 0u, 0u);

    uint32_t spark0_base = 0u, inj_on0_base = 0u, inj_off0_base = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0_base), 1u, "base cyl0 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on0_base), 1u, "base cyl0 inj_on");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off0_base), 1u, "base cyl0 inj_off");
    const uint32_t pw0_base = inj_off0_base - inj_on0_base;

    // +5° ign trim +50% fuel trim on cyl0 only.
    ecu_sched_test_reset();
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }
    ems::engine::cyl_ign_trim_deg[0] = 5;
    ems::engine::cyl_fuel_trim_pct[0] = 50;
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(2000u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 0u, 0u);
    encoder_seq_arm_cyl_in_window(2u, 4000u, 0u, 0u);

    uint32_t spark0_trim = 0u, spark2_trim = 0u;
    uint32_t inj_on0_trim = 0u, inj_off0_trim = 0u;
    uint32_t inj_on2_trim = 0u, inj_off2_trim = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0_trim), 1u, "trimmed cyl0 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark2_trim), 1u, "untrimmed cyl2 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on0_trim), 1u, "trimmed cyl0 inj_on");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off0_trim), 1u, "trimmed cyl0 inj_off");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ3, 1u, &inj_on2_trim), 1u, "untrimmed cyl2 inj_on");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ3, 0u, &inj_off2_trim), 1u, "untrimmed cyl2 inj_off");

    CHECK_TRUE(spark0_trim != spark0_base, "ign trim moved cyl0 spark vs untrimmed baseline");
    CHECK_TRUE(spark0_trim != spark2_trim, "ign trim applies only to cyl0, not cyl2");

    const uint32_t pw0_trim = inj_off0_trim - inj_on0_trim;
    const uint32_t pw2 = inj_off2_trim - inj_on2_trim;
    CHECK_TRUE(pw0_trim > pw0_base,
               "fuel trim +50% on cyl0 widens PW vs its own untrimmed baseline");
    CHECK_EQ(pw2, pw0_base,
             "untrimmed cyl2 PW matches cyl0 baseline (same inj_pw_ticks, omega)");

    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_phase_progression(void) {
    section("ecu_sched: encoder sequential — phase A/B cylinder partition");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);

    // Late-arm: only cylinders whose arm_at is inside ≤60° get events.
    const uint32_t now0 = encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 1u, "cyl0 armed in window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 0u, "cyl3 not yet in window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 0u, "cyl1 not yet in window");

    uint32_t spark0 = 0u;
    encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0);
    CHECK_TRUE(static_cast<int32_t>(spark0 - now0) <=
               static_cast<int32_t>(ecu_sched_encoder_test_arm_window_counts() + 5000u),
               "cyl0 spark lead is near the arm window (not ~1 rev early)");

    encoder_seq_arm_cyl_in_window(2u, 3500u, 1u, 1u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, nullptr), 1u, "cyl2 armed in its window");

    encoder_seq_arm_cyl_in_window(3u, 4000u, 2u, 2u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 1u, "cyl3 armed in its window");
    encoder_seq_arm_cyl_in_window(1u, 4500u, 2u, 2u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 1u, "cyl1 armed in its window");
    CHECK_TRUE(ecu_sched_encoder_test_get_evt_count() >= 16u,
               "all 4 cylinders armed across successive windows");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_presync_to_sequential_transition(void) {
    section("ecu_sched: encoder presync↔sequential handoff");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);

    encoder_seq_seed_omega();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u, "presync: 16 events");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "g_knock_sequential=0 in presync");
    CHECK_TRUE(ecu_sched_encoder_presync_call_count() > 0u,
               "presync_call_count sobe no builder PRESYNC");
    CHECK_EQ(ecu_sched_encoder_seq_call_count(), 0u,
             "seq_calls=0 enquanto o builder é presync");

    // Enter sequential: handoff purges presync 4-wide; late-arm may add 0..4
    // events depending on window (here arm cyl0 only).
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);
    CHECK_TRUE(ecu_sched_encoder_seq_call_count() > 0u,
               "seq_calls sobe no builder SEQ (dump 'D' / dash)");
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "g_knock_sequential=1 in sequential");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 1u,
             "cyl0 armed after handoff+window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 0u,
             "presync IGN2 shared-target gone after sequential handoff");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 0u,
             "presync IGN4 shared-target gone after sequential handoff");

    // Back to presync: invalidate phase → recompute_presync purges all +
    // clears flag.
    ecu_sched_encoder_phase_invalidate();
    ecu_sched_encoder_test_set_tim2_cnt(2000u);
    ecu_sched_encoder_heartbeat_tick(2000u, 4000u, 0u, 0u);
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "g_knock_sequential=0 after return to presync");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u,
             "presync rebuild restores 16-wide simultaneous schedule");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_min_lead_skip(void) {
    section("ecu_sched: encoder sequential — lead below min stays unarmed (window gate)");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    // Window is [min_lead, 60°]. cyl2 spark @ 120° = 5461; now=5451 → lead=10
    // which is below min_lead ⇒ outside the arm window ⇒ no insert, no skip
    // counter (gate never calls arm_pair). cyl0 remains far away.
    ecu_sched_set_advance_deg(60u);
    ecu_sched_set_eoi_lead_deg(355u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);

    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_test_set_tim2_cnt(1000u);
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);  // omega=1.0
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega=1.0 seeded");

    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    const uint32_t near_now = 5451u;
    CHECK_EQ(ecu_sched_encoder_test_deg720_to_absolute(120u, near_now), 5461u,
             "precondition: cyl2 spark_abs=5461, lead=10");

    ecu_sched_encoder_test_set_tim2_cnt(near_now);
    ecu_sched_encoder_heartbeat_tick(near_now, 1000u + (near_now - 1000u), 1u, 1u);

    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, nullptr), 0u,
             "cyl2 not armed — lead below window floor (min_lead)");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 0u,
             "cyl0 outside 60° window — not armed early");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_min_lead_dwell_behind(void) {
    section("ecu_sched: encoder sequential — min-lead skip when dwell behind now");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    // omega=1.0 → dwell_span == dwell_ticks. Spark/EOI for cyl2 @ 120° = 5461
    // (eoi_lead=60 ⇒ same angle as spark). now=4500 → dwell=2461 behind;
    // arm_at falls to inj_on=5461 with lead=961 ∈ [min_lead, 60°] → try arm
    // then IGN pair skipped (dwell behind).
    ecu_sched_set_advance_deg(60u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(3000u);
    ecu_sched_set_inj_pw_ticks(0u);

    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_test_set_tim2_cnt(1000u);
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);

    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    const uint32_t now = 4500u;
    const uint32_t skips_before = ecu_sched_encoder_test_get_seq_min_lead_skip_count();
    ecu_sched_encoder_test_set_tim2_cnt(now);
    ecu_sched_encoder_heartbeat_tick(now, 1000u + (now - 1000u), 1u, 1u);

    CHECK_TRUE(ecu_sched_encoder_test_get_seq_min_lead_skip_count() > skips_before,
               "dwell-behind spark increments skip counter");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, nullptr), 0u,
             "cyl2 IGN pair skipped when dwell is behind now");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 1u, nullptr), 0u,
             "cyl2 dwell also absent (whole pair skipped)");

    // Já sequencial: tick pesado com run_seq_arm=0 não duplica try_arm
    // (mesmo contrato do 64º subtick após o light path já ter armado).
    const uint32_t skips_after_arm = ecu_sched_encoder_seq_min_lead_skip_count();
    ecu_sched_encoder_heartbeat_tick(now, 1000u + (now - 1000u), 1u, 1u,
                                     /*run_seq_arm=*/0u);
    CHECK_EQ(ecu_sched_encoder_seq_min_lead_skip_count(), skips_after_arm,
             "run_seq_arm=0 — sem segundo try_arm (skip count estável)");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_long_lead_refresh(void) {
    section("ecu_sched: encoder sequential — no arm outside 60° window");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    // Far from all arm points (now=1500, next EOI/dwell many thousands of counts away).
    ecu_sched_encoder_test_set_tim2_cnt(1500u);
    ecu_sched_encoder_heartbeat_tick(1500u, 3000u, 1u, 1u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "sequential after phase");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "no events while all arm_at leads exceed 60°");

    // Enter cyl0 window → events appear with lead ≤ window.
    const uint32_t now0 = encoder_seq_arm_cyl_in_window(0u, 4000u, 1u, 1u);
    uint32_t spark0 = 0u, dwell0 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "cyl0 spark after window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell0), 1u, "cyl0 dwell after window");
    CHECK_TRUE(static_cast<int32_t>(dwell0 - now0) <=
               static_cast<int32_t>(ecu_sched_encoder_test_arm_window_counts()),
               "dwell lead ≤ 60° arm window");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_multispark(void) {
    section("ecu_sched: encoder sequential — multi-spark extra IGN events");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(20u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);
    // 1 multi-spark, tiny inter-dwell so offsets stay inside advance+atdc window.
    ecu_sched_set_mspark(1u, 100u, 18u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint8_t ign1_count = 0u;
    for (uint8_t i = 0u; i < ecu_sched_encoder_test_get_evt_count(); ++i) {
        uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
        ecu_sched_encoder_test_get_evt(i, &ts, &ch, &high);
        if (ch == ECU_CH_IGN1) { ++ign1_count; }
    }
    // Primary dwell+spark (2) + multi-spark dwell+spark (2) = 4 on IGN1.
    CHECK_EQ(ign1_count, 4u,
             "cyl0 has primary + 1 multi-spark pair (4 IGN1 events)");

    ecu_sched_set_mspark(0u, 0u, 18u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_prep_pw_overrides_global(void) {
    section("ecu_sched: encoder sequential — prep PW overrides g_inj_pw_ticks");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    // Global PW deliberately different from prep flow.
    ecu_sched_set_inj_pw_ticks(999999u);

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.flow_pw_us = 1000u;  // host ticks = 1000*60 = 60000
    prep.dwell_ticks = 0u;
    prep.eoi_lead_deg = 60u;
    prep.base_advance_deg = 10;
    prep.map_bar_x100 = 100u;
    prep.fuel_press_bar_x1000 = 3000u;
    ems::engine::enc_fuel_ign_prep_test_publish(prep);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint32_t inj_on = 0u, inj_off = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on), 1u, "inj_on present");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off), 1u, "inj_off present");
    const uint32_t span = inj_off - inj_on;
    // omega=0.5 → span_counts = ticks/2 = 30000 (plus scurve/delta_p ≈ identity at MAP=1bar)
    CHECK_TRUE(span < 50000u,
               "armed PW span from prep, not huge g_inj_pw_ticks");
    CHECK_TRUE(span > 1000u, "armed PW span non-trivial from prep flow");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_enc_cyl_setpoints_fuel_cut_read_modify_write_preserves_spark(void) {
    section("enc_cyl_setpoints: fuel_cut read-modify-write zera PW mas preserva dwell/advance "
            "(fix 2026-08-15: main_stm32.cpp branch (3) republica prep com fuel_cut=1 quando "
            "fuel_protect_cut trava o branch (1) que publicava)");
    ecu_sched_test_reset();
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.fuel_cut = 0u;
    prep.flow_pw_us = 1000u;
    prep.dwell_ticks = 1234u;
    prep.eoi_lead_deg = 60u;
    prep.base_advance_deg = 10;
    prep.map_bar_x100 = 100u;
    prep.fuel_press_bar_x1000 = 3000u;
    ems::engine::enc_fuel_ign_prep_test_publish(prep);

    const ems::engine::CylArmSetpoints running =
        ems::engine::finalize_cyl_setpoints(0u, false);
    CHECK_TRUE(running.inj_pw_ticks != 0u, "PW real com fuel_cut=0");
    CHECK_EQ(running.dwell_ticks, 1234u, "dwell publicado tal qual");

    // Mesmo padrão do fix: read-modify-write, só fuel_cut muda.
    ems::engine::EncFuelIgnPrep cut = ems::engine::enc_fuel_ign_prep_read();
    cut.fuel_cut = 1u;
    cut.valid = 1u;
    ems::engine::enc_fuel_ign_prep_test_publish(cut);

    const ems::engine::CylArmSetpoints cutout =
        ems::engine::finalize_cyl_setpoints(0u, false);
    CHECK_EQ(cutout.inj_pw_ticks, 0u, "PW zerado por fuel_cut=1 (sem isto, injeção real continuava)");
    CHECK_EQ(cutout.dwell_ticks, 1234u, "dwell/spark preservado — map_fault não corta ignição por desenho");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_ign_inhibit_mask_gate(void) {
    section("ecu_sched: encoder sequential — máscara de ignição corta dwell/spark de forma sustentada "
            "(fix 2026-08-15, achado #3: antes só purgava a fila UMA VEZ na borda de subida, "
            "try_arm_sequential_due voltava a armar no ciclo seguinte sem nunca consultar a máscara)");

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }
    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.flow_pw_us = 1000u;
    prep.dwell_ticks = 100u;
    prep.eoi_lead_deg = 60u;
    prep.base_advance_deg = 10;
    prep.map_bar_x100 = 100u;
    prep.fuel_press_bar_x1000 = 3000u;

    // kIgnCh[0] = ECU_CH_IGN1 (7); k_ign_ch_to_bit[7] = bit0 — mesma tabela
    // usada por force_output() (ecu_sched.cpp) e agora também por
    // arm_sequential_cyl() (ecu_sched_encoder_builders.cpp).
    {
        ecu_sched_test_reset();
        ems::engine::enc_fuel_ign_prep_test_publish(prep);
        encoder_seq_seed_omega();
        ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
        ecu_sched_set_ign_inhibit_mask(0x01u);
        encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

        CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, nullptr), 0u,
                 "IGN1 DWELL_START NÃO armado com máscara a cortar cyl0");
        CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, nullptr), 1u,
                 "INJ1 continua a armar — máscara de ignição não corta fuel (gates independentes)");
        ecu_sched_set_ign_inhibit_mask(0u);
    }

    // Máscara limpa: mesma sequência deve armar dwell/spark normalmente.
    {
        ecu_sched_test_reset();
        ems::engine::enc_fuel_ign_prep_test_publish(prep);
        encoder_seq_seed_omega();
        ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
        ecu_sched_set_ign_inhibit_mask(0x00u);
        encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

        CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, nullptr), 1u,
                 "IGN1 DWELL_START armado normalmente com máscara limpa");
    }

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_prep_knock_per_cyl(void) {
    section("ecu_sched: encoder sequential — knock retard only on armed cyl");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
        ems::engine::knock_retard_x10[i] = 0u;
    }
    ems::engine::knock_retard_x10[0] = 50u;  // 5°

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.base_advance_deg = 20;
    prep.eoi_lead_deg = 60u;
    prep.dwell_ticks = 0u;
    prep.flow_pw_us = 0u;
    ems::engine::enc_fuel_ign_prep_test_publish(prep);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);
    encoder_seq_arm_cyl_in_window(2u, 4000u, 1u, 1u);

    uint32_t spark0 = 0u, spark2 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "cyl0 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark2), 1u, "cyl2 spark");

    // cyl0 advance 15°, cyl2 advance 20° → distinct absolute targets.
    CHECK_TRUE(spark0 != spark2, "knock on cyl0 moves its spark vs unknocked cyl2");

    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::knock_retard_x10[i] = 0u;
    }
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_omega_refresh(void) {
    section("ecu_sched: encoder sequential — Δω refreshes dwell, spark fixed");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(4000u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    const uint32_t now = encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint32_t dwell0 = 0u, spark0 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell0), 1u, "dwell before refresh");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "spark before refresh");
    const uint32_t refreshes_before = ecu_sched_encoder_test_get_omega_refresh_count();

    // Mild ω rise (~0.5 → 0.7, >2% threshold) so new dwell stays ahead of now.
    ecu_sched_encoder_omega_test_reset();
    ecu_sched_encoder_omega_sample(0u, 0u);
    ecu_sched_encoder_omega_sample(700u, 1000u);  // ω=0.7
    ecu_sched_encoder_test_set_tim2_cnt(now);
    ecu_sched_encoder_heartbeat_tick(now, 1000u, 1u, 1u);

    uint32_t dwell1 = 0u, spark1 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell1), 1u, "dwell after refresh");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark1), 1u, "spark after refresh");
    CHECK_EQ(spark1, spark0, "spark absolute unchanged after ω refresh");
    CHECK_TRUE(dwell1 < dwell0,
               "dwell moves earlier (more angle) when ω rises");
    CHECK_TRUE(ecu_sched_encoder_test_get_omega_refresh_count() > refreshes_before,
               "omega refresh counter increments");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_omega_refresh_lock(void) {
    section("ecu_sched: encoder sequential — no refresh past limit angle");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(4000u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint32_t dwell0 = 0u, spark0 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell0), 1u, "dwell armed");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "spark armed");

    // Park just before spark (inside lock / min_lead zone).
    const uint32_t near = spark0 - 2u;
    ecu_sched_encoder_omega_sample(near + 100u, 4100u);
    ecu_sched_encoder_omega_sample(near + 2100u, 4200u);
    const uint32_t refreshes_before = ecu_sched_encoder_test_get_omega_refresh_count();
    ecu_sched_encoder_test_set_tim2_cnt(near);
    ecu_sched_encoder_heartbeat_tick(near, 4200u, 1u, 1u);

    uint32_t dwell1 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell1), 1u, "dwell still queued");
    CHECK_EQ(dwell1, dwell0, "dwell frozen after limit-angle lock");
    CHECK_EQ(ecu_sched_encoder_test_get_omega_refresh_count(), refreshes_before,
             "no refresh after lock");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_enc_finalize_xtau_peek_no_commit(void) {
    section("enc_cyl_setpoints: finalize peek + commit_last_peek (filme 1×, PW do peek)");
    ecu_sched_test_reset();
    map_window_reset();
    enc_cyl_setpoints_reset();
    ems::engine::xtau_wall_fuel_reset();

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.cranking = 0u;
    prep.xtau_event_enable = 1u;   // caminho X-τ por evento (encoder sequencial)
    prep.flow_pw_us = 5000u;
    prep.base_flow_pw_us = 5000u;
    prep.map_bar_x100 = 100u;
    prep.dead_time_us = 0u;
    prep.rpm_x10 = 30000u;         // 3000 RPM
    prep.corr_clt_x256 = 256u;
    prep.corr_iat_x256 = 256u;
    prep.clt_x10 = 800;
    prep.base_advance_deg = 10;
    prep.eoi_lead_deg = 60u;
    enc_fuel_ign_prep_test_publish(prep);

    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), 0,
             "filme de parede começa em 0");

    // 1ª chamada COMITADA (equivalente à finalização vencedora de uma
    // tentativa de arme dentro da janela) — estabelece um filme não-nulo.
    (void)finalize_cyl_setpoints(0u, /*commit_fuel=*/true);
    const int32_t wall_after_commit =
        ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u);
    CHECK_TRUE(wall_after_commit != 0,
               "commit_fuel=true integra o modelo — filme passa a não-nulo");

    // Simula ~5 tentativas especulativas de try_arm_sequential_due() para um
    // cilindro fora da janela de 60° (sub-tick chamado ~65×/volta) — ANTES
    // do fix, cada uma destas chamadas mutava g_cyl_wall_us_q8 mesmo sendo
    // depois descartada por lead_in_arm_window(); o filme sobre-integrava
    // dezenas de vezes por spray real e neutralizava o AE.
    for (uint8_t i = 0u; i < 5u; ++i) {
        (void)finalize_cyl_setpoints(0u, /*commit_fuel=*/false);
    }
    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), wall_after_commit,
             "5× peek (commit_fuel=false) — filme inalterado (fix bug 3)");

    // Caminho do arm vencedor: reutiliza sp do peek + commit_last_peek
    // (só o filme; sem segundo finalize completo).
    const ems::engine::CylArmSetpoints sp_peek =
        finalize_cyl_setpoints(0u, /*commit_fuel=*/false);
    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), wall_after_commit,
             "peek pré-commit — filme ainda inalterado");
    CHECK_TRUE(ems::engine::transient_fuel_xtau_commit_last_peek(0u),
               "commit_last_peek aplica o passo do peek ao filme real");
    const int32_t wall_after_peek_commit =
        ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u);
    CHECK_TRUE(wall_after_peek_commit != wall_after_commit,
               "após peek+commit API, filme muda exactamente 1×");
    CHECK_TRUE(!ems::engine::transient_fuel_xtau_commit_last_peek(0u),
               "segundo commit_last_peek é no-op (peek já consumido)");
    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), wall_after_peek_commit,
             "no-op não volta a integrar o filme");

    // PW que o arm usaria é o do peek — não o de um segundo finalize
    // (que veria o filme já mutado e produziria PW diferente).
    const ems::engine::CylArmSetpoints sp_if_refinalize =
        finalize_cyl_setpoints(0u, /*commit_fuel=*/false);
    CHECK_TRUE(sp_peek.inj_pw_ticks != sp_if_refinalize.inj_pw_ticks,
               "re-finalize após commit veria filme novo — PW mudaria");
    // O contrato do arm: sp_peek.inj_pw_ticks é o que fica armado.

    ecu_sched_test_reset();
    ems::engine::xtau_wall_fuel_reset();
}

void test_enc_finalize_map_window_per_cyl(void) {
    section("enc_cyl_setpoints: map_window VE bilineal + ΔP no finalize");
    ecu_sched_test_reset();
    map_window_reset();
    enc_cyl_setpoints_reset();

    CHECK_EQ(map_window_slot_for_cyl(0u), 0u, "cyl0 TDC 0° → slot 0");
    CHECK_EQ(map_window_slot_for_cyl(2u), 1u, "cyl2 TDC 180° → slot 1");

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.flow_pw_us = 5000u;
    prep.base_flow_pw_us = 5000u;
    prep.map_bar_x100 = 100u;           // fused 1.00 bar
    prep.fuel_press_bar_x1000 = 3000u;  // 3.0 bar abs → ΔP vs MAP
    prep.dead_time_us = 0u;
    prep.rpm_x10 = 30000u;              // 3000 RPM — VE re-lookup
    prep.corr_clt_x256 = 256u;
    prep.corr_iat_x256 = 256u;
    prep.fuel_trim_pct_x10 = 0;
    prep.clt_x10 = 800;
    prep.base_advance_deg = 10;
    prep.eoi_lead_deg = 60u;
    enc_fuel_ign_prep_test_publish(prep);

    map_window_enable = 0u;
    const CylArmSetpoints base0 = finalize_cyl_setpoints(0u);
    const CylArmSetpoints base2 = finalize_cyl_setpoints(2u);
    CHECK_EQ(base0.inj_pw_ticks, base2.inj_pw_ticks,
             "enable=0: same PW for cyl0 and cyl2");

    map_window_enable = 1u;
    map_window_test_set_slot_bar_x1000(0u, 800u);   // 0.80 bar
    map_window_test_set_slot_bar_x1000(1u, 1200u);  // 1.20 bar
    const CylArmSetpoints win0 = finalize_cyl_setpoints(0u);
    const CylArmSetpoints win2 = finalize_cyl_setpoints(2u);
    CHECK_TRUE(win0.inj_pw_ticks != win2.inj_pw_ticks,
               "enable=1 + distinct slots → different PW per cyl");
    CHECK_TRUE(win2.inj_pw_ticks > win0.inj_pw_ticks,
               "higher MAP slot → higher PW (VE×MAP, não só scale linear)");

    // Coerência: PW do slot alto ≈ calc_fuel_pw_us_default_fast(ve, map_cyl, …)
    {
        const uint8_t ve_hi = get_ve(30000u, 120u);
        const uint16_t lam_hi = get_lambda_target_x1000(30000u, 120u);
        uint32_t expect_hi = calc_fuel_pw_us_default_fast(
            ve_hi, 120u, lam_hi, 0, 256u, 256u, 0u);
        expect_hi = apply_delta_p_compensation(expect_hi, 3000u, 120u);
        expect_hi = apply_injector_scurve(expect_hi);
        const uint32_t expect_ticks = inj_pw_us_to_scheduler_ticks(expect_hi);
        // VE path + ΔP/S-curve — tolera ±5% por aritmética inteira.
        CHECK_TRUE(win2.inj_pw_ticks > (expect_ticks * 95u) / 100u &&
                   win2.inj_pw_ticks < (expect_ticks * 105u) / 100u,
                   "slot MAP alto → PW coerente com tabela VE/λ");
    }

    map_window_enable = 0u;
    map_window_reset();
    enc_cyl_setpoints_reset();
    ecu_sched_test_reset();
}

void test_enc_finalize_map_window_ve0_keeps_prep_flow(void) {
    section("enc_cyl_setpoints: VE=0 no slot MAP não zera o PW do prep (INJ visível)");
    ecu_sched_test_reset();
    map_window_reset();
    enc_cyl_setpoints_reset();

    uint8_t ve_saved[kTableAxisSize][kTableAxisSize];
    std::memcpy(ve_saved, ve_table, sizeof(ve_table));
    std::memset(ve_table, 0, sizeof(ve_table));

    EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.flow_pw_us = 4000u;
    prep.base_flow_pw_us = 4000u;
    prep.map_bar_x100 = 50u;
    prep.fuel_press_bar_x1000 = 3000u;
    prep.dead_time_us = 0u;
    prep.rpm_x10 = 7000u;
    prep.corr_clt_x256 = 256u;
    prep.corr_iat_x256 = 256u;
    prep.base_advance_deg = 10;
    prep.eoi_lead_deg = 60u;
    enc_fuel_ign_prep_test_publish(prep);

    map_window_enable = 1u;
    map_window_test_set_slot_bar_x1000(0u, 500u);
    const CylArmSetpoints sp = finalize_cyl_setpoints(0u, false);
    CHECK_TRUE(sp.inj_pw_ticks != 0u,
               "VE=0 + janela MAP válida: PW cai no fluxo do prep, não no span 0");

    std::memcpy(ve_table, ve_saved, sizeof(ve_table));
    map_window_enable = 0u;
    map_window_reset();
    enc_cyl_setpoints_reset();
    ecu_sched_test_reset();
}

// ============================================================================
// QUICK CRANK
// ============================================================================

void test_ecu_sched_hardware_init(void) {
    section("ecu_sched: ECU_Hardware_Init runs without crash");
    // ECU_Hardware_Init clears angle table + TIM5 queue mocks. Only testable:
    //   1. No crash.
    //   2. Angle table cleared — ecu_sched_test_angle_table_size()=0 after init.
    //   3. Diagnostic counters cleared.
    ECU_Hardware_Init();
    CHECK_EQ(ecu_sched_test_angle_table_size(), 0u,
             "angle table empty after ECU_Hardware_Init");
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "dwell_watchdog_count=0 after ECU_Hardware_Init");
    CHECK_TRUE(true, "ECU_Hardware_Init: no crash");
}

// Golden identity checks (plan verification): min-lead timestamp formula, angle
// table shape, sorted queue order — no soft "count>0" only.
void test_ecu_sched_golden_min_lead_timestamp(void) {
    section("ecu_sched golden: min-lead insert timestamp (not STATUS late)");
    // STM32_MIN_COMPARE_LEAD_TICKS = 125 @ 62.5 MHz (2 µs).
    // ECU_SCHED_US_TO_TICKS(1) = 62 < 125 → OFF event must land at now+125.
    // Min-lead is a schedule safety policy — does NOT increment g_late_event_count
    // (that bit is reserved for dispatch path-2 true misses).
    constexpr uint32_t kNow = 100000u;
    constexpr uint32_t kMinLead = 125u;
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_pulse_inj(0u, 1u);  // 1 µs PW → short delta → min-lead
    CHECK_TRUE(ecu_sched_test_get_evt_count() >= 1u, "OFF event queued");
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "peek head event");
    CHECK_EQ(ts, kNow + kMinLead, "min-lead timestamp = TIM5_CNT + 125");
    CHECK_EQ(high, 0u, "OFF event is low");
    CHECK_EQ(ch, ECU_CH_INJ1, "INJ1 channel id unchanged");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), late0,
             "min-lead does not sticky-inflate late_event_count");
}

void test_ecu_sched_golden_dispatch_past_counts_late(void) {
    section("ecu_sched golden: path-2 tight re-arm increments late_event_count");
    // path-2: after due-loop, next event is already past or ≤16 ticks ahead.
    // Queue OFF far out, then set CNT to ts-5 so due-loop skips (still future)
    // and re-arm loop takes path-2 (5 ≤ 16).
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    ecu_sched_test_pulse_inj(0u, 1000u);  // OFF ~ now+62500
    uint32_t ts = 0u;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, nullptr, nullptr), 1u, "have event");
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_set_tim5_cnt(ts - 5u);  // 5 ticks before → path-2
    ecu_sched_evt_dispatch();
    CHECK_TRUE(ecu_sched_test_get_late_event_count() > late0,
               "path-2 tight re-arm increments late_event_count");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 0u, "event consumed");
}

void test_ecu_sched_golden_far_target_timestamp(void) {
    section("ecu_sched golden: far target uses exact delta (no min-lead)");
    constexpr uint32_t kNow = 50000u;
    constexpr uint32_t kPwUs = 1000u;  // 1 ms → 62500 ticks @ 62.5 MHz
    constexpr uint32_t kExpectedDelta = (kPwUs * 125u) / 2u;  // ECU_SCHED_US_TO_TICKS
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_pulse_inj(1u, kPwUs);
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "peek OFF event");
    CHECK_EQ(ts, kNow + kExpectedDelta, "timestamp = now + exact PW ticks");
    CHECK_EQ(ch, ECU_CH_INJ2, "INJ2 channel");
    CHECK_EQ(high, 0u, "OFF");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), late0,
             "no late count when delta >= min-lead");
}

void test_ecu_sched_golden_queue_sorted(void) {
    section("ecu_sched golden: queue stays sorted by timestamp");
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    // Two pulses with different PW → two OFF times; queue must be ascending.
    ecu_sched_test_pulse_inj(0u, 2000u);  // later OFF
    ecu_sched_test_set_tim5_cnt(1000u);   // same now for second arm
    ecu_sched_test_pulse_inj(1u, 500u);   // earlier OFF
    const uint8_t n = ecu_sched_test_get_evt_count();
    CHECK_TRUE(n >= 2u, "at least two OFF events");
    uint32_t prev = 0u;
    for (uint8_t i = 0u; i < n; ++i) {
        uint32_t ts = 0u;
        CHECK_EQ(ecu_sched_test_get_evt(i, &ts, nullptr, nullptr), 1u, "peek evt");
        if (i > 0u) {
            CHECK_TRUE(ts >= prev, "queue non-decreasing timestamps");
        }
        prev = ts;
    }
}

void test_ecu_sched_golden_dispatch_identity(void) {
    section("ecu_sched golden: dispatch fires head GPIO order (channel, high)");
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    ecu_sched_test_pulse_inj(0u, 1000u);  // OFF at 1000+62500
    // Make event due and dispatch.
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "have event");
    const uint8_t n0 = ecu_sched_test_get_evt_count();
    ecu_sched_test_set_tim5_cnt(ts);  // CNT == timestamp → due
    ecu_sched_evt_dispatch();
    CHECK_EQ(ecu_sched_test_get_evt_count(), static_cast<uint32_t>(n0 - 1u),
             "one event consumed by dispatch");
    // Pin counters: INJ1 pin index 0 — OFF is low transition after force ON.
    // At least one high and one low counted for pin 0 path (force ON + OFF).
    uint32_t pins[24];
    ecu_sched_get_pin_counts_u32x24(pins);
    CHECK_TRUE(pins[0] >= 1u, "INJ1 high_count >= 1 after force ON");
    CHECK_TRUE(pins[1] >= 1u, "INJ1 low_count >= 1 after OFF dispatch");
}

void test_ecu_sched_dwell_watchdog_fires(void) {
    section("ecu_sched: dwell watchdog fires after 1.4x dwell ticks");

    // Direct path: force dwell HIGH + queue SPARK without dispatching SPARK.
    // Watchdog must stay armed across SPARK *arm* (only pin LOW / trip release it)
    // so a lost SPARK cannot leave the coil charged indefinitely.
    const uint32_t kNow = 1000u;
    const uint32_t kDwellUs = 3000u;  // 3 ms → 187500 ticks @ 62.5 MHz
    const uint32_t kDwellTicks = (kDwellUs * 125u) / 2u;
    const uint32_t kWdogTicks = (kDwellTicks * 7u) / 5u;  // 1.4×
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    ecu_sched_test_pulse_ign(0u, kDwellUs);  // cyl0 HIGH + SPARK queued, wdog armed

    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "pre-cond: wdog_count=0 before threshold");
    // Still within window — no trip
    ecu_sched_test_set_tim5_cnt(kNow + kWdogTicks - 1u);
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "watchdog silent inside 1.4× dwell");
    // Past 1.4× dwell with SPARK still only queued → force pin LOW
    ecu_sched_test_set_tim5_cnt(kNow + kWdogTicks + 1u);
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 1u,
             "dwell watchdog fires: elapsed > 1.4× dwell (lost-SPARK backstop)");
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 1u, "watchdog fires only once per arm");
}

void test_ecu_sched_inj_watchdog_fires(void) {
    section("ecu_sched: injector open watchdog fires after timeout (lost INJ_OFF)");

    // Force INJ HIGH without OFF (simulate queue drop of OFF). force_output path
    // uses hard 36 ms timeout.
    const uint32_t kNow = 5000u;
    const uint32_t kHardTicks = (36000u * 125u) / 2u;  // 36 ms @ 62.5 MHz
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    // test_pulse schedules OFF — fire raw force path via test pulse then clear OFF
    // by advancing past OFF without dispatch: use pulse then wipe queue.
    ecu_sched_test_pulse_inj(0u, 3000u);  // ON + OFF queued
    // Drop OFF from queue by resetting event queue only, keep pin state via
    // another open: re-force through pulse then zero events after arm.
    // Simpler: pulse with PW, discard OFF by resetting CCR/queue after ON.
    {
        // Re-open: force via second pulse; then clear queue so OFF never runs.
        ecu_sched_test_reset_ccr();
        // Pin may be LOW after reset_ccr — re-open with pulse and immediately
        // drop all events (lost OFF).
        ecu_sched_test_set_tim5_cnt(kNow);
        ecu_sched_test_pulse_inj(0u, 5000u);
        // Wipe queue: OFF is gone, pin still HIGH from force_output.
        ecu_sched_test_reset_ccr();
    }
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 0u, "pre: inj wdog count=0");
    ecu_sched_test_set_tim5_cnt(kNow + kHardTicks - 1u);
    ecu_sched_inj_watchdog();
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 0u, "silent inside hard timeout");
    ecu_sched_test_set_tim5_cnt(kNow + kHardTicks + 1u);
    ecu_sched_inj_watchdog();
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 1u,
             "inj watchdog fires: pin HIGH past hard timeout without OFF");
    ecu_sched_inj_watchdog();
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 1u, "inj wdog fires once per open");
}

