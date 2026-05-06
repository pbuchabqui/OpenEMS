#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "app/can_stack.h"
#include "app/ui_protocol.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/calibration.h"
#include "engine/ecu_sched.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/quick_crank.h"
#include "engine/table3d.h"
#include "engine/transient_fuel.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/flash.h"

extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

namespace {

uint32_t g_failures = 0u;

void expect(bool cond, const char* msg) {
    if (!cond) {
        std::printf("FAIL: %s\n", msg);
        ++g_failures;
    }
}

void ckp_edge(uint32_t delta_ticks) {
    ems_test_tim5_ccr1 += delta_ticks;
    ems::drv::ckp_tim5_ch1_isr();
}

void cam_edge() {
    ems_test_cam_gpio_idr |= (1u << 1u);
    ++ems_test_tim5_ccr2;
    ems::drv::ckp_tim5_ch2_isr();
}

void feed_60_2_revolution(uint32_t normal_ticks) {
    for (uint8_t i = 0u; i < 58u; ++i) {
        ckp_edge(normal_ticks);
    }
    ckp_edge(normal_ticks * 3u);
}

void test_ckp_sync_range() {
    constexpr uint32_t kTicksAt200Rpm = 312500u;
    constexpr uint32_t kTicksAt8500Rpm = 7352u;

    ems::drv::ckp_test_reset();
    feed_60_2_revolution(kTicksAt200Rpm);
    expect(ems::drv::ckp_snapshot().state == ems::drv::SyncState::HALF_SYNC,
           "CKP reaches HALF_SYNC after first valid 60-2 gap at 200 rpm");
    feed_60_2_revolution(kTicksAt200Rpm);
    auto snap = ems::drv::ckp_snapshot();
    expect(snap.state == ems::drv::SyncState::FULL_SYNC,
           "CKP reaches FULL_SYNC after second valid 60-2 gap at 200 rpm");
    expect(snap.rpm_x10 >= 1990u && snap.rpm_x10 <= 2010u,
           "CKP reports roughly 200 rpm");

    ems::drv::ckp_test_reset();
    feed_60_2_revolution(kTicksAt8500Rpm);
    feed_60_2_revolution(kTicksAt8500Rpm);
    snap = ems::drv::ckp_snapshot();
    expect(snap.state == ems::drv::SyncState::FULL_SYNC,
           "CKP reaches FULL_SYNC at 8500 rpm");
    expect(snap.rpm_x10 >= 84900u && snap.rpm_x10 <= 85150u,
           "CKP reports roughly 8500 rpm");

    cam_edge();
    expect(ems::drv::ckp_snapshot().phase_A,
           "CMP edge toggles phase_A for bench phase visibility");
}

void test_quick_crank() {
    ems::engine::quick_crank_reset();
    ems::engine::crank_enter_rpm_x10 = 4500u;
    ems::engine::crank_exit_rpm_x10 = 7000u;
    ems::engine::crank_spark_deg = 8;
    ems::engine::crank_min_pw_us = 2500u;
    ems::engine::crank_prime_tooth = 3u;
    ems::engine::crank_prime_max_pw_us = 30000u;

    const auto crank = ems::engine::quick_crank_update(10u, 3000u, true, 200, 20);
    expect(crank.cranking, "quick crank detects cranking window");
    expect(crank.spark_deg == 8, "quick crank applies cranking spark");
    expect(crank.fuel_mult_x256 > 256u, "quick crank enriches fuel");
    expect(ems::engine::quick_crank_apply_pw_us(1000u, crank.fuel_mult_x256,
                                                crank.min_pw_us) >= 2500u,
           "quick crank applies minimum pulse width");

    const auto afterstart = ems::engine::quick_crank_update(20u, 8000u, true, 200, 20);
    expect(!afterstart.cranking && afterstart.afterstart_active,
           "quick crank enters afterstart after exit rpm");

    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_prime_context(200, 900u);
    ems::drv::CkpSnapshot snap{};
    snap.rpm_x10 = 3000u;
    for (uint8_t i = 0u; i < 3u; ++i) {
        ems::drv::prime_on_tooth(snap);
    }
    const uint32_t prime = ems::engine::quick_crank_consume_prime();
    expect(prime > 0u && prime <= 30000u, "prime pulse is generated once during cranking");
    expect(ems::engine::quick_crank_consume_prime() == 0u,
           "prime pulse consume is one-shot");
}

void test_scheduler_host_regression() {
    ECU_Hardware_Init();
    ecu_sched_test_reset();
    ecu_sched_test_set_advance_deg(999u);
    ecu_sched_test_set_dwell_ticks(999999u);
    ecu_sched_test_set_inj_pw_ticks(999999u);
    ecu_sched_test_set_soi_lead_deg(999u);

    expect(ecu_sched_test_get_advance_deg() == 60u,
           "scheduler clamps excessive advance");
    expect(ecu_sched_test_get_dwell_ticks() == 100000u,
           "scheduler clamps excessive dwell");
    expect(ecu_sched_test_get_inj_pw_ticks() == 200000u,
           "scheduler clamps excessive injection pulse");
    expect(ecu_sched_test_get_soi_lead_deg() == 719u,
           "scheduler clamps SOI lead to cycle range");
    expect(ecu_sched_test_get_calibration_clamp_count() != 0u,
           "scheduler exposes calibration clamp diagnostics");

    ecu_sched_test_reset();
    ems::drv::CkpSnapshot snap{};
    snap.state = ems::drv::SyncState::FULL_SYNC;
    snap.tooth_index = 1u;
    snap.tooth_period_ns = 1000000u;
    snap.predicted_tooth_period_ns = 1000000u;
    ems::drv::schedule_on_tooth(snap);
    snap.tooth_index = 0u;
    ems::drv::schedule_on_tooth(snap);
    expect(ecu_sched_test_angle_table_size() == 16u,
           "scheduler builds a complete 4-cylinder sequential event table");

    ecu_sched_test_reset();
    ecu_sched_test_set_advance_deg(0u);
    ecu_sched_test_set_dwell_ticks(0u);
    ecu_sched_test_set_inj_pw_ticks(0u);
    ecu_sched_test_set_soi_lead_deg(0u);
    snap = {};
    snap.state = ems::drv::SyncState::FULL_SYNC;
    snap.phase_A = true;
    snap.tooth_index = 1u;
    snap.tooth_period_ns = 1000000u;
    snap.predicted_tooth_period_ns = 1000000u;
    ems::drv::schedule_on_tooth(snap);
    snap.tooth_index = 0u;
    ems::drv::schedule_on_tooth(snap);
    expect(ecu_sched_test_get_late_event_count() != 0u,
           "scheduler applies rebuilt tooth-0 table before arming events");
}

void test_tables_and_ui_protocol() {
    const auto lookup = ems::engine::table3d_prepare_lookup(
        ems::engine::kRpmAxisX10,
        ems::engine::kLoadAxisKpa,
        5000u,
        20u);
    expect(lookup.xi == 0u && lookup.yi == 0u &&
           lookup.fx_q8 == 0u && lookup.fy_q8 == 0u,
           "table lookup clamps to first cell");
    expect(ems::engine::table3d_lookup_u8_prepared(ems::engine::ve_table, lookup) ==
               ems::engine::ve_table[0][0],
           "table lookup returns exact first VE cell");

    ems::hal::nvm_test_reset();
    ems::app::ui_test_reset();
    ems::app::ui_rx_byte(static_cast<uint8_t>('H'));
    ems::app::ui_process();
    const char signature[] = "OpenEMS_v1.1";
    for (uint8_t i = 0u; i < sizeof(signature) - 1u; ++i) {
        uint8_t b = 0u;
        expect(ems::app::ui_tx_pop(b), "UI signature byte available");
        expect(b == static_cast<uint8_t>(signature[i]), "UI signature matches");
    }

    const uint8_t cmd[] = {
        static_cast<uint8_t>('r'), static_cast<uint8_t>('3'),
        47u, 0u, 8u, 0u,
    };
    for (uint8_t b : cmd) {
        ems::app::ui_rx_byte(b);
    }
    ems::app::ui_process();
    for (uint8_t i = 0u; i < 8u; ++i) {
        uint8_t b = 0u;
        expect(ems::app::ui_tx_pop(b), "UI realtime diag byte available");
    }
}

void test_transient_fuel() {
    using ems::engine::transient_fuel_reset;
    using ems::engine::transient_fuel_xtau_update;

    // disabled: passthrough, state stays reset
    transient_fuel_reset();
    expect(transient_fuel_xtau_update(3000u, -400, false) == 3000u,
           "transient fuel disabled returns input unchanged");

    // zero pulse: returns 0
    transient_fuel_reset();
    expect(transient_fuel_xtau_update(0u, -400, true) == 0u,
           "transient fuel zero pulse returns 0");

    // cold (-40°C) enriches more than warm (90°C) on first pulse
    transient_fuel_reset();
    const uint32_t cold = transient_fuel_xtau_update(3000u, -400, true);
    transient_fuel_reset();
    const uint32_t warm = transient_fuel_xtau_update(3000u, 900, true);
    expect(cold > warm, "cold CLT produces more wall-wetting enrichment than warm CLT");
    expect(warm >= 3000u, "warm CLT still enriches on first pulse");

    // wall-wetting state accumulates: second call differs from first
    transient_fuel_reset();
    const uint32_t first = transient_fuel_xtau_update(3000u, -400, true);
    const uint32_t second = transient_fuel_xtau_update(3000u, -400, true);
    expect(first != second, "wall-wetting state accumulates between calls");
}

void test_ign_calc() {
    using namespace ems::engine;

    // dwell table lookup at exact axis points
    expect(dwell_ms_x10_from_vbatt(12000u) == 30u, "dwell at 12V matches table");
    expect(dwell_ms_x10_from_vbatt(14000u) == 25u, "dwell at 14V matches table");

    // dwell angle: 3ms @ 3000rpm = 30*3000*36/6000 = 540 (54.0 degrees)
    expect(calc_dwell_angle_x10(30u, 3000u) == 540u,
           "dwell angle at 3000rpm/3ms = 54.0 deg");
    expect(calc_dwell_angle_x10(30u, 0u) == 0u, "dwell angle at 0rpm is zero");

    // calc_total_advance: no corrections pass through base
    expect(calc_total_advance(20, {0, 0, 0}) == 20,
           "calc_total_advance with no corrections returns base");
    // heavy knock retard clamps to lower limit (-10)
    expect(calc_total_advance(5, {0, 0, 20}) == -10,
           "heavy knock retard clamps to -10");
    // excessive advance clamps to 40
    expect(calc_total_advance(38, {5, 0, 0}) == 40,
           "excessive advance clamps to 40");

    // idle spark: outside TPS window → 0
    expect(calc_idle_spark_correction_deg(7000u, 8000u, 1000u, 50u) == 0,
           "idle spark correction returns 0 outside TPS window");
    // idle spark: rpm below target within window → advance
    // rpm=7000, target=8000, error=1000, deadband=500 → corrected_error=500
    // corr = 500 / idle_spark_rpm_per_deg_x10(500) = 1
    expect(calc_idle_spark_correction_deg(7000u, 8000u, 5u, 50u) > 0,
           "idle spark advances when rpm is below target");
}

void test_fuel_calc_chain() {
    using namespace ems::engine;

    // calc_req_fuel_us: zero args → 0 (division-by-zero guard)
    expect(calc_req_fuel_us(0u, 0u, 0u, 0u) == 0u,
           "calc_req_fuel_us with zero inputs returns 0");

    // nominal engine: result should be in a plausible range
    const uint32_t req = calc_req_fuel_us(
        cfg::kDisplacementCc, cfg::kCylinderCount,
        cfg::kInjectorFlowCcMin, cfg::kStoichAfrX100);
    expect(req > 5000u && req < 15000u,
           "calc_req_fuel_us nominal result is in plausible range");
    // runtime result must match the compile-time constant
    expect(req == kDefaultReqFuelUs,
           "calc_req_fuel_us matches kDefaultReqFuelUs");

    // calc_base_pw_us: zero ref → 0
    expect(calc_base_pw_us(8000u, 80u, 100u, 0u) == 0u,
           "calc_base_pw_us returns 0 for zero map_ref_kpa");

    // corr_clt: cold temperature enriches (factor > 256)
    expect(corr_clt(-400) > 256u,
           "CLT correction at -40C is greater than 1.0 (enrichment)");
    // corr_clt: warm temperature is neutral (factor == 256)
    expect(corr_clt(900) == 256u,
           "CLT correction at 90C is exactly 1.0 (neutral)");

    // corr_vbatt: returns injector dead time from table
    expect(corr_vbatt(12000u) == 900u,
           "vbatt correction at 12V returns correct dead time (900us)");
}

}  // namespace

int main() {
    test_ckp_sync_range();
    test_quick_crank();
    test_scheduler_host_regression();
    test_tables_and_ui_protocol();
    test_transient_fuel();
    test_ign_calc();
    test_fuel_calc_chain();

    if (g_failures != 0u) {
        std::printf("%u MVP bench host test(s) failed\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("MVP bench host tests passed");
    return EXIT_SUCCESS;
}
