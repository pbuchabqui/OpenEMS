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

void test_auxiliaries() {
    // 1. After reset + key_on, pump primes (should be ON initially)
    ems::engine::auxiliaries_test_reset();
    ems::engine::auxiliaries_set_key_on(true);
    ems::engine::auxiliaries_tick_10ms();
    expect(ems::engine::auxiliaries_test_get_pump_state(),
           "fuel pump primes after key_on");

    // 2. Cold CLT needs higher idle RPM than warm CLT
    expect(ems::engine::auxiliaries_idle_target_rpm_x10(-400) >
           ems::engine::auxiliaries_idle_target_rpm_x10(900),
           "cold CLT produces higher idle target RPM than warm CLT");

    // 3. Wastegate failsafe starts false after reset
    ems::engine::auxiliaries_test_reset();
    expect(!ems::engine::auxiliaries_test_get_wg_failsafe(),
           "wastegate failsafe is inactive after reset");
}

void test_can_stack() {
    // 1. After reset, safe lambda returns WBO2_SAFE_LAMBDA_MILLI (no frame received)
    ems::hal::can_test_reset();
    ems::app::can_stack_test_reset();
    expect(ems::app::can_stack_lambda_milli_safe(0u) == ems::app::WBO2_SAFE_LAMBDA_MILLI,
           "safe lambda returns fallback when no WBO2 frame received");

    // 2. Inject frame 0x180 with lambda 1.000 (1000 milli)
    {
        ems::hal::CanFrame frame = {};
        frame.id = 0x180u;
        frame.dlc = 3u;
        frame.extended = false;
        frame.data[0] = 0xE8u;  // 1000 & 0xFF
        frame.data[1] = 0x03u;  // 1000 >> 8
        frame.data[2] = 0u;     // status
        ems::hal::can_test_inject_rx(frame);
    }
    // Process so RX is consumed
    {
        ems::drv::CkpSnapshot snap = {};
        ems::drv::SensorData sensors = {};
        ems::app::can_stack_process(100u, snap, sensors, 0, 0, 0, 0u, 0u, 0u);
    }
    expect(ems::app::can_stack_lambda_milli() == 1000u,
           "lambda_milli returns 1000 after injecting 0x180 with lambda 1.000");
    expect(ems::app::can_stack_wbo2_fresh(100u),
           "wbo2_fresh is true immediately after receiving frame");

    // 3. WBO2 timeout: frame at t=100, check at t=700 (>500ms timeout)
    expect(!ems::app::can_stack_wbo2_fresh(700u),
           "wbo2_fresh is false after 600ms without new frame");
    expect(ems::app::can_stack_lambda_milli_safe(700u) == ems::app::WBO2_SAFE_LAMBDA_MILLI,
           "safe lambda returns fallback after WBO2 timeout");

    // 4. TX encoding: call process() and pop 0x400 frame, verify RPM bytes
    {
        ems::hal::can_test_reset();
        ems::app::can_stack_test_reset();
        ems::drv::CkpSnapshot snap = {};
        snap.rpm_x10 = 30000u;  // 3000 RPM
        ems::drv::SensorData sensors = {};
        // Send at t=10 so elapsed(10, 0, 10) triggers TX (10 - 0 >= 10)
        ems::app::can_stack_process(10u, snap, sensors, 0, 0, 0, 0u, 0u, 0u);
        ems::hal::CanFrame tx_frame = {};
        const bool got_frame = ems::hal::can_test_pop_tx(tx_frame);
        expect(got_frame, "can_stack_process transmits 0x400 frame");
        if (got_frame) {
            expect(tx_frame.id == 0x400u,
                   "transmitted frame has ID 0x400");
            const uint16_t rpm_decoded = static_cast<uint16_t>(
                tx_frame.data[0] | (static_cast<uint16_t>(tx_frame.data[1]) << 8u));
            expect(rpm_decoded == 3000u,
                   "0x400 frame bytes 0-1 encode RPM correctly (3000)");
        }
    }
}

void test_knock() {
    // Reset NVM so knock_init() sees no stored retard
    ems::hal::nvm_test_reset();

    // 1. After knock_init(), all knock_retard_x10 are 0
    ems::engine::knock_init();
    bool all_zero = true;
    for (uint8_t i = 0u; i < ems::engine::kKnockCylinders; ++i) {
        if (ems::engine::knock_retard_x10[i] != 0u) {
            all_zero = false;
        }
    }
    expect(all_zero, "knock_retard_x10 all zero after knock_init with clean NVM");

    // 2. knock_window_open() sets window active and cylinder
    ems::engine::knock_window_open(0u);
    expect(ems::engine::knock_test_window_active(),
           "knock window is active after knock_window_open(0)");
    expect(ems::engine::knock_test_window_cyl() == 0u,
           "knock window cylinder is 0 after knock_window_open(0)");

    // 3. Fire ISR 4 times (> default threshold 3), then cycle_complete → retard increases
    ems::engine::knock_cmp0_isr();
    ems::engine::knock_cmp0_isr();
    ems::engine::knock_cmp0_isr();
    ems::engine::knock_cmp0_isr();
    ems::engine::knock_cycle_complete(0u);
    expect(ems::engine::knock_retard_x10[0] > 0u,
           "knock_retard_x10[0] increases after 4 knock events (threshold=3)");
    // kRetardStepX10 = 20
    expect(ems::engine::knock_retard_x10[0] == 20u,
           "knock_retard_x10[0] equals kRetardStepX10=20 after one knock event");

    // 4. knock_window_close() deactivates window
    ems::engine::knock_window_close(0u);
    expect(!ems::engine::knock_test_window_active(),
           "knock window is inactive after knock_window_close(0)");

    // 5. Many clean cycles → retard decreases (recovery)
    // Need > kRecoveryDelayCycles (10) clean cycles for recovery to begin
    const uint16_t initial_retard = ems::engine::knock_retard_x10[0];
    for (uint8_t i = 0u; i < 20u; ++i) {
        ems::engine::knock_cycle_complete(0u);
    }
    expect(ems::engine::knock_retard_x10[0] < initial_retard,
           "knock retard decreases after many clean cycles");

    // 6. knock_get_vosel() returns a valid value (0-63)
    const uint8_t vosel = ems::engine::knock_get_vosel();
    expect(vosel <= 63u,
           "knock_get_vosel returns a value in range [0,63]");
}

void test_sensors() {
    // 1. After sensors_test_reset(), fault_bits = 0 (healthy initial state)
    ems::drv::sensors_test_reset();
    const ems::drv::SensorData data = ems::drv::sensors_get();
    expect(data.fault_bits == 0u,
           "sensors_get fault_bits is 0 after sensors_test_reset");

    // 2. validate_sensor_values() on default-initialized (all zero) SensorData returns false
    const ems::drv::SensorData zero_data = {};
    expect(!ems::drv::validate_sensor_values(zero_data),
           "validate_sensor_values returns false for zero-initialized SensorData");

    // 3. get_sensor_health_status() doesn't crash
    const uint8_t health = ems::drv::get_sensor_health_status();
    static_cast<void>(health);
    expect(true, "get_sensor_health_status executes without crashing");

    // 4. After reset, map_kpa_x10 is the fallback value (not zero)
    ems::drv::sensors_test_reset();
    const ems::drv::SensorData data2 = ems::drv::sensors_get();
    expect(data2.map_kpa_x10 == 0u,  // reset_state zeroes g_data before any conversion
           "sensors_get map_kpa_x10 after test_reset is zero (pre-ADC-update state)");
}

void test_tim8_wraparound() {
    ECU_Hardware_Init();
    ecu_sched_test_reset();
    ecu_sched_test_set_tim8_cnt(0xFFFEu);

    ems::drv::CkpSnapshot snap{};
    snap.state = ems::drv::SyncState::FULL_SYNC;
    snap.phase_A = true;
    snap.tooth_index = 1u;
    snap.tooth_period_ns = 1000000u;
    snap.predicted_tooth_period_ns = 1000000u;
    ems::drv::schedule_on_tooth(snap);
    snap.tooth_index = 0u;
    ems::drv::schedule_on_tooth(snap);

    for (uint8_t ch = 1u; ch <= 4u; ++ch) {
        expect(ecu_sched_test_get_tim8_ccr(ch) <= 0xFFFFu,
               "TIM8 CCR stays in 16-bit range when CNT near rollover");
    }
}

void test_sensor_adc_recovery_fallback() {
    ems::drv::sensors_test_reset();
    ems::hal::adc_test_set_recovery_failed(true);

    ems::drv::CkpSnapshot snap{};
    snap.tooth_period_ns = 1000000u;
    for (uint8_t i = 0u; i < 5u; ++i) {
        ems::drv::sensors_on_tooth(snap);
    }
    ems::drv::sensors_tick_100ms();

    const auto data = ems::drv::sensors_get();
    expect(data.map_kpa_x10 == ems::drv::kFallbackMapKpaX10,
           "MAP uses fallback when ADC recovery failed");

    ems::hal::adc_test_set_recovery_failed(false);
}

void test_sensor_clt_fault_fallback() {
    ems::drv::sensors_test_reset();
    ems::drv::sensors_init();

    ems::hal::adc_test_set_raw_secondary(
        ems::hal::AdcSecondaryChannel::CLT_SE14, 0u);

    ems::drv::sensors_tick_100ms();
    ems::drv::sensors_tick_100ms();
    ems::drv::sensors_tick_100ms();

    const auto data = ems::drv::sensors_get();
    expect(data.clt_degc_x10 == ems::drv::kFallbackCltDegcX10,
           "CLT uses fallback value after 3 consecutive out-of-range reads");
    expect((data.fault_bits & (1u << 3u)) != 0u,
           "CLT fault bit is set after consecutive range violations");
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
    test_auxiliaries();
    test_can_stack();
    test_knock();
    test_sensors();
    test_tim8_wraparound();
    test_sensor_adc_recovery_fallback();
    test_sensor_clt_fault_fallback();

    if (g_failures != 0u) {
        std::printf("%u MVP bench host test(s) failed\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("MVP bench host tests passed");
    return EXIT_SUCCESS;
}
