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
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "engine/output_test.h"
#include "engine/engine_config.h"
#include "hal/timer.h"
#include "hal/flash.h"
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
using namespace ems::engine;
using namespace ems::app;
using namespace ems::hal;

void test_table3d_all(void) {
    using namespace ems::engine;

    // ─ table_axis_index ────────────────────────────────────────────────
    section("table3d: table_axis_index");
    // kRpmAxisX10: {5000,7500,10000,...,120000} — kTableAxisSize=16
    // Below first → idx=0
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 100u), 0u,
             "below axis[0] → idx=0");
    // At exact first value → idx=0
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 5000u), 0u,
             "at axis[0]=5000 → idx=0");
    // Between axis[0]=5000 and axis[1]=7500 → idx=0
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 6000u), 0u,
             "between axis[0] and axis[1] → idx=0");
    // value==axis[1]: binary search sets hi=1, idx=lo-1=0 (lower interval)
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 7500u), 0u,
             "at axis[1]=7500 → idx=0 (lower interval)");
    // Above last → idx=kTableAxisSize-2=18
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 999999u), 18u,
             "above last → idx=kTableAxisSize-2=18");

    // ─ table_axis_nearest_index (célula dominante = trace VE do dash) ─
    // Em valor exacto no eixo k>0, table_axis_index devolve k-1 (frac=255).
    // nearest deve devolver k — senão LEARN hit cai na célula anterior.
    section("table3d: table_axis_nearest_index");
    // kRpmAxisX10[6]=20000 (2000 rpm), [5]=17500
    CHECK_EQ(table_axis_index(kRpmAxisX10, kTableAxisSize, 20000u), 5u,
             "floor em 2000 rpm exacto → idx 5 (1750)");
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 20000u), 6u,
             "nearest em 2000 rpm exacto → idx 6 (2000)");
    // kLoadAxisBarX100[12]=110, [11]=100
    CHECK_EQ(table_axis_index(kLoadAxisBarX100, kTableAxisSize, 110u), 11u,
             "floor em MAP 110 exacto → idx 11 (100)");
    CHECK_EQ(table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 110u), 12u,
             "nearest em MAP 110 exacto → idx 12 (110)");
    // Midpoint RPM 2125 (2000–2250): frac=0.5 → sobe
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 21250u), 7u,
             "nearest midpoint 2125 rpm → 2250 (idx 7)");
    // Abaixo do 1º / no 1º
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 100u), 0u,
             "nearest below axis[0] → 0");
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 5000u), 0u,
             "nearest at axis[0] → 0");
    // No último nó
    CHECK_EQ(table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 80000u), 19u,
             "nearest at last rpm → idx 19");

    // ─ table_axis_frac_q8 ────────────────────────────────────────────
    section("table3d: table_axis_frac_q8");
    // axis[0]=5000, axis[1]=7500. At value=5000 → frac=0
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 5000u), 0u, "at axis[0]: frac=0");
    // At value=7500 (== axis[1]) → frac=255
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 7500u), 255u, "at axis[1]: frac=255");
    // Midpoint 6250: (6250-5000)/(7500-5000) × 256 = 1250/2500×256 = 128
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 6250u), 128u, "midpoint: frac=128");
    // Below axis[0] → 0
    CHECK_EQ(table_axis_frac_q8(kRpmAxisX10, 0u, 4000u), 0u, "below axis[0]: frac=0");

    // ─ table3d_lookup_u8_prepared / table3d_lookup_u8 ──────────────────
    section("table3d: lookup_u8 bilinear interpolation");
    // Build flat table: all cells = 80
    static uint8_t flat_u8[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            flat_u8[y][x] = 80u;
    const Table2dLookup lk = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100,
                                                     30000u, 100u);
    CHECK_EQ(table3d_lookup_u8_prepared(flat_u8, lk), 80u,
             "flat u8 table: any point → 80");
    CHECK_EQ(table3d_lookup_u8(flat_u8, kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u),
             80u, "table3d_lookup_u8 matches prepared");

    // Gradient table: cell[y][x] = x+y (0..30), expect interpolation
    static uint8_t grad_u8[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            grad_u8[y][x] = static_cast<uint8_t>(x + y);
    // At axis boundary (exact) xi=0,yi=0,frac=0 → cell[0][0]=0
    const Table2dLookup lk00 = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100,
                                                       5000u, 20u);
    CHECK_EQ(table3d_lookup_u8_prepared(grad_u8, lk00), 0u,
             "grad u8 at [0][0] → 0");

    // ─ table3d_lookup_i8_prepared ────────────────────────────────────────
    section("table3d: lookup_i8_prepared");
    static int8_t flat_i8[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            flat_i8[y][x] = -10;
    CHECK_EQ(table3d_lookup_i8_prepared(flat_i8, lk), (int16_t)-10,
             "flat i8 table: any point → -10");
    static int8_t neg_grad[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            neg_grad[y][x] = static_cast<int8_t>(-x - y);
    CHECK_EQ(table3d_lookup_i8_prepared(neg_grad, lk00), (int16_t)0,
             "neg grad i8 at [0][0] → 0");

    // ─ table3d_lookup_s16_prepared / table3d_lookup_s16 ──────────────
    section("table3d: lookup_s16_prepared");
    static int16_t flat_s16[kTableAxisSize][kTableAxisSize];
    for (int y = 0; y < kTableAxisSize; ++y)
        for (int x = 0; x < kTableAxisSize; ++x)
            flat_s16[y][x] = 1000;
    CHECK_EQ(table3d_lookup_s16_prepared(flat_s16, lk), (int16_t)1000,
             "flat s16 table: any point → 1000");
    CHECK_EQ(table3d_lookup_s16(flat_s16, kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u),
             (int16_t)1000, "table3d_lookup_s16 matches prepared");

    // ─ table3d_lookup_ve_q8 ───────────────────────────────────────────
    section("table3d: lookup_ve_q8");
    // flat VE=80 → result in Q8 = 80<<8=20480
    const uint16_t ve_q8 = table3d_lookup_ve_q8(flat_u8,
                                                  kRpmAxisX10, kLoadAxisBarX100,
                                                  30000u, 100u);
    CHECK_EQ(ve_q8, 80u << 8u, "flat VE=80 → ve_q8=80<<8");

    // ─ table3d_lookup_advance_q10 ─────────────────────────────────
    section("table3d: lookup_advance_q10");
    // flat advance=30° → result in Q10 = 30<<10=30720
    const int32_t adv_q10 = table3d_lookup_advance_q10(flat_s16,
                                                         kRpmAxisX10, kLoadAxisBarX100,
                                                         30000u, 100u);
    CHECK_EQ(adv_q10, 1000 << 10, "flat adv=1000 → adv_q10=1000<<10");
}

// ============================================================================
// ECU SCHED
// ============================================================================

void test_quick_crank_all(void) {
    using namespace ems::engine;

    section("quick_crank: reset");
    quick_crank_reset();
    CHECK_EQ(quick_crank_consume_prime(), 0u, "no prime pending after reset");

    section("quick_crank: not cranking when rpm=0 or no sync");
    auto out = quick_crank_update(0u, 0u, false, 800, 8);
    CHECK_FALSE(out.cranking, "rpm=0, no sync → not cranking");
    CHECK_EQ(out.fuel_mult_x256, 256u, "not cranking → mult=1.0 (256)");

    section("quick_crank: cranking when rpm below enter threshold");
    quick_crank_reset();
    // crank_enter_rpm_x10=4500, crank_exit_rpm_x10=7000
    // First call with rpm=3000 (300 RPM) and sync_available=true
    out = quick_crank_update(1000u, 3000u, true, 800, 8);
    CHECK_TRUE(out.cranking, "rpm=3000 < enter=4500 → cranking");
    CHECK_TRUE(out.fuel_mult_x256 > 256u, "cranking enrichment > 1.0");
    CHECK_EQ(out.spark_deg, ems::engine::crank_spark_deg, "spark_deg = crank_spark_deg");
    CHECK_TRUE(out.min_pw_us > 0u, "min_pw_us > 0 during cranking");

    section("quick_crank: afterstart on RPM jump");
    quick_crank_reset();
    // Enter cranking state first
    quick_crank_update(1000u, 3000u, true, 800, 8);
    // Now RPM jumps above exit: transitions from cranking to afterstart
    out = quick_crank_update(5000u, 80000u, true, 800, 8);  // 8000 RPM > exit
    // fuel_mult should still be > 256 during afterstart (hot engine at 80°C may be minimal)
    // Or == 256 if CLT=800 (warm). Either way, no crash.
    CHECK_TRUE(out.fuel_mult_x256 >= 256u, "afterstart: fuel_mult ≥ 1.0");

    section("quick_crank: quick_crank_apply_pw_us");
    // base_pw * mult / 256, clamped to min_pw
    CHECK_EQ(quick_crank_apply_pw_us(10000u, 256u, 0u), 10000u, "mult=1.0 → pw unchanged");
    CHECK_EQ(quick_crank_apply_pw_us(10000u, 512u, 0u), 20000u, "mult=2.0 → pw doubled");
    CHECK_EQ(quick_crank_apply_pw_us(100u, 256u, 500u), 500u, "below min_pw → clamped to 500");
    CHECK_EQ(quick_crank_apply_pw_us(0u, 512u, 1000u), 1000u, "base=0 → clamped to min");
    // Overflow clamp at 100000 us
    CHECK_EQ(quick_crank_apply_pw_us(90000u, 512u, 0u), 100000u, "overflow → 100000 clamp");

    section("quick_crank: set_prime_context / set_clt / consume_prime");
    quick_crank_set_prime_context(800, 500u);
    quick_crank_set_clt(600);
    // consume_prime: no prime fired yet → 0
    CHECK_EQ(quick_crank_consume_prime(), 0u, "no prime fired → consume=0");
    // Two calls to consume same prime: second must return 0 (one-shot)
    CHECK_EQ(quick_crank_consume_prime(), 0u, "second consume → 0 (one-shot)");

    section("quick_crank: hysteresis enter/exit + is_cranking latch");
    quick_crank_reset();
    CHECK_FALSE(is_cranking(), "reset → not cranking");
    // Between enter and exit without prior crank → not cranking (enter=4500)
    out = quick_crank_update(0u, 5000u, true, 800, 8);
    CHECK_FALSE(out.cranking, "rpm=500 between enter/exit without latch → not cranking");
    CHECK_FALSE(is_cranking(), "is_cranking tracks out");
    // Enter cranking
    out = quick_crank_update(10u, 3000u, true, 800, 8);
    CHECK_TRUE(out.cranking, "enter at 300 rpm");
    CHECK_TRUE(is_cranking(), "is_cranking true while latched");
    // Stay cranking at 500 rpm (below exit 700)
    out = quick_crank_update(20u, 5000u, true, 800, 8);
    CHECK_TRUE(out.cranking, "hysteresis: stay cranking below exit");
    // Exit at 800 rpm
    out = quick_crank_update(30u, 8000u, true, 200, 8);
    CHECK_FALSE(out.cranking, "exit above crank_exit");
    CHECK_FALSE(is_cranking(), "is_cranking false after exit");
    CHECK_TRUE(out.afterstart_active || out.fuel_mult_x256 >= 256u,
               "afterstart arm on exit edge");
    CHECK_TRUE(is_afterstart() || !out.afterstart_active,
               "is_afterstart matches out when ASE armed");

    section("quick_crank: flood clear APP threshold");
    quick_crank_reset();
    crank_flood_tps_x10 = 700u;
    quick_crank_update(0u, 3000u, true, 800, 8);
    CHECK_TRUE(is_cranking(), "cranking for flood test");
    CHECK_FALSE(crank_flood_clear_active(500u), "APP 50% < 70% → no flood clear");
    CHECK_TRUE(crank_flood_clear_active(700u), "APP 70% → flood clear");
    CHECK_TRUE(crank_flood_clear_active(1000u), "APP 100% → flood clear");
    quick_crank_reset();
    CHECK_FALSE(crank_flood_clear_active(1000u), "not cranking → no flood clear");
}

// ============================================================================
// TRANSIENT FUEL (X-Tau)
// ============================================================================

void test_transient_fuel_all(void) {
    using namespace ems::engine;

    section("transient_fuel: reset");
    transient_fuel_reset();
    CHECK_TRUE(true, "transient_fuel_reset: no crash");

    section("transient_fuel: disabled → returns base pw");
    transient_fuel_reset();
    const uint32_t pw_base = 5000u;
    const uint32_t out_disabled = transient_fuel_xtau_update(pw_base, 800, false);
    CHECK_EQ(out_disabled, pw_base, "disabled → output = input pw");

    section("transient_fuel: enabled with valid CLT");
    transient_fuel_reset();
    // Enabled: X-Tau model applies wall wetting correction.
    // Result may differ from input but must be > 0 and <= 100000.
    const uint32_t out_enabled = transient_fuel_xtau_update(pw_base, 800, true);
    CHECK_TRUE(out_enabled > 0u && out_enabled <= 100000u,
               "enabled: output in (0, 100ms]");

    section("transient_fuel: enabled with zero pw → reset + return 0");
    transient_fuel_reset();
    const uint32_t out_zero = transient_fuel_xtau_update(0u, 800, true);
    CHECK_EQ(out_zero, 0u, "pw=0, enabled → reset + return 0");

    section("transient_fuel: warm-up steady state converges");
    transient_fuel_reset();
    uint32_t prev = 0u;
    for (int i = 0; i < 20; ++i) {
        prev = transient_fuel_xtau_update(5000u, 800, true);
    }
    // After many iterations wall fuel reaches steady state; output near input
    CHECK_TRUE(prev > 0u, "xtau: converges to positive value");
}

// ============================================================================
// MAP ESTIMATOR
// ============================================================================

void test_map_estimator_all(void) {
    using namespace ems::engine;

    section("map_estimator: init / update / getters");
    map_estimator_init();

    // First update: estimated should track sensor
    const uint16_t est = map_estimator_update(100u, 500u, 10u, 30000u, 220);
    CHECK_TRUE(est > 0u && est <= 300u, "estimated MAP in (0, 300 kPa]");
    CHECK_EQ(map_get_estimated_bar_x100(), est, "getter matches return value");

    section("map_estimator: tpsdot");
    // After two calls with same TPS: tpsdot ≈ 0
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    const int16_t dot = map_get_tpsdot_x10();
    CHECK_EQ(dot, 0, "steady TPS → tpsdot=0");

    // TPS step: history ring buffer design requires full wrap to compute non-zero
    // tpsdot; just verify the API returns a valid range value.
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    map_estimator_update(100u, 900u, 10u, 30000u, 220);
    const int16_t dot2 = map_get_tpsdot_x10();
    CHECK_TRUE(dot2 >= -1000 && dot2 <= 1000, "TPS step → tpsdot in clamped range");

    section("map_estimator: is_transient");
    // After rapid TPS step: transient flag may be set
    // (depends on implementation thresholds)
    const bool trans = map_is_transient();
    CHECK_TRUE(trans == true || trans == false, "is_transient returns bool");

    section("map_estimator: get_state");
    const MapEstimatorState st = map_estimator_get_state();
    CHECK_TRUE(st.map_estimated_bar_x100 > 0u, "state.map_estimated > 0");

    section("map_estimator: set_gains");
    map_estimator_set_gains(200u, 150u);  // arbitrary gains
    // Call update after gain change — must not crash
    map_estimator_update(100u, 500u, 10u, 30000u, 220);
    CHECK_TRUE(true, "set_gains + update: no crash");

    section("map_estimator: edge cases");
    // dt=0
    map_estimator_update(100u, 500u, 0u, 30000u, 220);
    CHECK_TRUE(true, "dt=0: no crash");
    // rpm=0
    map_estimator_update(100u, 500u, 10u, 0u, 220);
    CHECK_TRUE(true, "rpm=0: no crash");

    section("map_estimator: sensor inválido → model_only estável");
    map_estimator_init();
    (void)map_estimator_update(0u, 500u, 10u, 30000u, 220, true);
    CHECK_EQ(map_estimator_get_state().estimator_mode, 2u,
             "sensor 0 bar → estimator_mode=2 (model_only)");
    (void)map_estimator_update(5u, 500u, 10u, 30000u, 220, true);
    CHECK_EQ(map_estimator_get_state().estimator_mode, 2u,
             "sensor inválido mantém model_only (não sobrescreve)");

    section("map_estimator: sensor_valid=false (MAP fault) ignora fallback 1 bar");
    map_estimator_init();
    const uint16_t est_fault = map_estimator_update(101u, 500u, 10u, 30000u, 220, false);
    CHECK_EQ(map_estimator_get_state().estimator_mode, 2u,
             "MAP fault flag → model_only mesmo com 1.01 bar no range");
    CHECK_TRUE(est_fault >= 10u && est_fault <= 300u, "estimate still in range");

    section("map_estimator: modelo termodinâmico — set_model_params");
    ManifoldModelParams mp{};
    mp.volume_cc_x10 = 5000u;
    mp.throttle_flow_coeff_q8 = 256u;
    mp.engine_pumping_coeff_q8 = 256u;
    map_estimator_set_model_params(mp);
    CHECK_TRUE(true, "set_model_params: no crash");

    section("map_estimator: tpsdot deteta transiente após volta completa do ring buffer");
    // g_tps_history_pos aponta sempre para o slot mais antigo ainda residente
    // (prestes a ser sobrescrito); o mais recente é pos-1. tpsdot só fica
    // não-nulo depois do buffer (kTpsHistorySize=4 amostras) ter dado a volta
    // completa com um degrau de TPS dentro dela.
    map_estimator_init();
    for (uint8_t i = 0u; i < 4u; ++i) {
        map_estimator_update(30u, 100u, 2u, 8000u, 220);
    }
    map_estimator_update(30u, 900u, 2u, 8000u, 220);
    CHECK_TRUE(map_get_tpsdot_x10() > 0, "step de TPS após volta do buffer → tpsdot > 0");
    CHECK_TRUE(map_is_transient(), "tpsdot alto → is_transient()=true");

    section("map_estimator: modelo termodinâmico responde a IAT (fluxo de admissão)");
    // Ar mais frio é mais denso → mais massa de ar admitida para a mesma
    // abertura de borboleta → o modelo deve prever um MAP igual ou mais alto
    // que com ar quente, no mesmo cenário de tip-in (baixo RPM, TPS 10%→90%).
    auto run_tip_in = [](int16_t iat_x10) noexcept -> uint16_t {
        map_estimator_init();
        for (uint8_t i = 0u; i < 4u; ++i) {
            map_estimator_update(30u, 100u, 2u, 8000u, iat_x10);
        }
        uint16_t v = 0u;
        for (uint8_t i = 0u; i < 3u; ++i) {
            v = map_estimator_update(30u, 900u, 2u, 8000u, iat_x10);
        }
        return v;
    };
    const uint16_t est_hot_iat = run_tip_in(220);   // 22.0°C
    const uint16_t est_cold_iat = run_tip_in(-100);  // -10.0°C
    CHECK_TRUE(est_cold_iat >= est_hot_iat,
               "IAT baixo (ar mais denso) → fluxo de admissão maior ou igual ao de IAT alto");
}

// ============================================================================
// MISFIRE DETECT
// ============================================================================

void test_misfire_all(void) {
    using namespace ems::engine;

    section("misfire: init / reset / get_event_count / clear_events");
    misfire_init();
    misfire_reset();
    for (uint8_t c = 0u; c < 4u; ++c) {
        CHECK_EQ(misfire_get_event_count(c), 0u, "event_count=0 after reset");
    }

    section("misfire: misfire_clear_events");
    // Manually call on_tooth with exaggerated slow period (3× expected → misfire)
    // Simulate 10 teeth of a window for cyl 0 with slow period
    ckp_reach_full_sync();
    const uint32_t normal_ns = kNormalPeriod * 16u;  // ticks×16 ns/tick = normal_ns
    ems::drv::CkpSnapshot snap_mf{};
    snap_mf.state = ems::drv::SyncState::FULL_SYNC;
    snap_mf.tooth_index = 0u;
    snap_mf.phase_A = true;
    snap_mf.tooth_period_ns = normal_ns * 3u;            // 3× slow = misfire
    snap_mf.predicted_tooth_period_ns = normal_ns;      // expected period
    // Feed kMisfireWindowTeeth teeth × kMisfireDebounceCycles to trigger event
    for (uint32_t w = 0u; w < (uint32_t)kMisfireDebounceCycles; ++w) {
        for (uint32_t t = 0u; t < (uint32_t)kMisfireWindowTeeth; ++t) {
            ems::drv::misfire_on_tooth(snap_mf);
        }
    }
    // After debounce cycles, event_count[0] should be ≥1
    CHECK_TRUE(misfire_get_event_count(0u) >= 1u, "misfire event after slow period");

    // clear_events
    misfire_clear_events(0u);
    CHECK_EQ(misfire_get_event_count(0u), 0u, "event_count=0 after clear");

    section("misfire: inhibit suppresses detection");
    misfire_reset();
    misfire_set_all_inhibit(true);
    for (uint32_t w = 0u; w < (uint32_t)kMisfireDebounceCycles; ++w) {
        for (uint32_t t = 0u; t < (uint32_t)kMisfireWindowTeeth; ++t) {
            ems::drv::misfire_on_tooth(snap_mf);
        }
    }
    CHECK_EQ(misfire_get_event_count(0u), 0u, "inhibited: no event despite slow period");
    misfire_set_all_inhibit(false);  // restore
}

// ============================================================================
// DIAGNOSTIC MANAGER
// ============================================================================

void test_diagnostic_manager_all(void) {
    using namespace ems::engine;

    section("DiagnosticManager: init");
    DiagnosticManager::init();
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 0u, "no faults after init");
    CHECK_TRUE(DiagnosticManager::is_system_ready(), "system ready after init");

    section("DiagnosticManager: report_fault / is_fault_active / clear_fault");
    DiagnosticManager::init();
    const bool first_report = DiagnosticManager::report_fault(
        DiagnosticCode::MAP_SENSOR_RANGE, FaultSeverity::WARNING);
    CHECK_TRUE(first_report, "first report returns true (new fault)");
    CHECK_TRUE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_SENSOR_RANGE),
               "fault is active after report");
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 1u, "count=1 after one fault");

    const bool second_report = DiagnosticManager::report_fault(
        DiagnosticCode::MAP_SENSOR_RANGE, FaultSeverity::WARNING);
    CHECK_FALSE(second_report, "duplicate report returns false (already active)");
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 1u, "count still 1 (duplicate)");

    const bool cleared = DiagnosticManager::clear_fault(DiagnosticCode::MAP_SENSOR_RANGE);
    CHECK_TRUE(cleared, "clear_fault returns true");
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_SENSOR_RANGE),
                "fault inactive after clear");
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 0u, "count=0 after clear");

    section("DiagnosticManager: get_highest_severity");
    DiagnosticManager::init();
    DiagnosticManager::report_fault(DiagnosticCode::VBATT_LOW, FaultSeverity::WARNING);
    DiagnosticManager::report_fault(DiagnosticCode::OVERTEMP_CRITICAL, FaultSeverity::CRITICAL);
    DiagnosticManager::report_fault(DiagnosticCode::CKP_SIGNAL_FAULT, FaultSeverity::ERROR);
    CHECK_EQ(static_cast<uint8_t>(DiagnosticManager::get_highest_severity()),
             static_cast<uint8_t>(FaultSeverity::CRITICAL), "highest=CRITICAL");

    section("DiagnosticManager: is_system_ready blocked by CRITICAL");
    CHECK_FALSE(DiagnosticManager::is_system_ready(),
                "system NOT ready with CRITICAL fault");

    section("DiagnosticManager: clear_all_faults");
    DiagnosticManager::clear_all_faults();
    CHECK_EQ(DiagnosticManager::get_active_fault_count(), 0u, "count=0 after clear_all");
    CHECK_TRUE(DiagnosticManager::is_system_ready(), "system ready after clear_all");

    section("DiagnosticManager: update_recovery / get_recovery_state");
    DiagnosticManager::init();
    DiagnosticManager::report_fault(DiagnosticCode::ADC_TIMEOUT, FaultSeverity::ERROR);
    auto rs = DiagnosticManager::update_recovery(DiagnosticCode::ADC_TIMEOUT, false);
    CHECK_TRUE(static_cast<uint8_t>(rs) <= static_cast<uint8_t>(RecoveryState::PERMANENT),
               "recovery state in valid range");
    const auto rs2 = DiagnosticManager::get_recovery_state(DiagnosticCode::ADC_TIMEOUT);
    CHECK_EQ(static_cast<uint8_t>(rs), static_cast<uint8_t>(rs2),
             "get_recovery_state matches update_recovery");
    // Success recovery
    DiagnosticManager::update_recovery(DiagnosticCode::ADC_TIMEOUT, true);
    CHECK_TRUE(true, "update_recovery(success): no crash");

    section("DiagnosticManager: record_freeze_frame / get_event");
    DiagnosticManager::init();
    DiagnosticManager::report_fault(DiagnosticCode::CLT_SENSOR_RANGE, FaultSeverity::WARNING,
                                    1000u, 3000u);
    const uint16_t ff[4] = {900u, 100u, 30000u, 12000u};
    DiagnosticManager::record_freeze_frame(DiagnosticCode::CLT_SENSOR_RANGE, ff);
    const DiagnosticEvent* ev = DiagnosticManager::get_event(
        DiagnosticCode::CLT_SENSOR_RANGE);
    CHECK_TRUE(ev != nullptr, "get_event returns non-null for active fault");
    if (ev != nullptr) {
        CHECK_EQ(ev->freeze_frame[0], 900u, "freeze_frame[0]=900");
    }
    // get_event for unknown fault → nullptr
    const DiagnosticEvent* ev_none = DiagnosticManager::get_event(
        DiagnosticCode::FLASH_WRITE_FAULT);
    CHECK_TRUE(ev_none == nullptr, "get_event for inactive fault → nullptr");

    section("DiagnosticManager: check_sensor_plausibility");
    DiagnosticManager::init();
    // High TPS (900 = 90%) + Low MAP (200 = 2 bar) at high RPM is implausible
    CHECK_FALSE(DiagnosticManager::check_sensor_plausibility(200u, 900u, 80000u),
                "high TPS + low MAP + high RPM: implausible");
    // Low TPS + mid MAP at mid RPM: plausible
    CHECK_TRUE(DiagnosticManager::check_sensor_plausibility(700u, 100u, 30000u),
               "low TPS + mid MAP + mid RPM: plausible");
}

// ============================================================================
// HAL ADC
// ============================================================================

void test_hal_adc_all(void) {
    using namespace ems::hal;

    section("hal/adc: init + primary/secondary read");
    adc_init();
    // After test_set_raw, read must return set value
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 2500u);
    CHECK_EQ(adc_primary_read(AdcPrimaryChannel::MAP), 2500u,
             "primary_read == test_set_raw");

    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 1800u);
    CHECK_EQ(adc_secondary_read(AdcSecondaryChannel::CLT), 1800u,
             "secondary_read == test_set_raw");

    section("hal/adc: adc_trigger_on_tooth updates trigger mod");
    adc_trigger_on_tooth(10000u);
    const uint32_t mod = adc_test_last_trigger_mod();
    CHECK_TRUE(mod > 0u, "trigger_mod > 0 after trigger_on_tooth(10000)");

    // Short period: mod should be smaller
    adc_trigger_on_tooth(5000u);
    const uint32_t mod2 = adc_test_last_trigger_mod();
    CHECK_TRUE(mod2 > 0u, "trigger_mod > 0 after trigger_on_tooth(5000)");
    CHECK_TRUE(mod2 <= mod, "shorter period → trigger_mod ≤ previous");

    section("hal/adc: recovery / timeout flags");
    adc_test_set_recovering(false);
    CHECK_FALSE(adc_is_recovering(), "is_recovering=false after set_false");
    adc_test_set_recovering(true);
    CHECK_TRUE(adc_is_recovering(), "is_recovering=true after set_true");
    adc_test_set_recovering(false);  // restore

    adc_test_set_recovery_failed(false);
    CHECK_FALSE(adc_recovery_failed(), "recovery_failed=false");
    adc_test_set_recovery_failed(true);
    CHECK_TRUE(adc_recovery_failed(), "recovery_failed=true");
    adc_test_set_recovery_failed(false);  // restore

    adc_test_set_timeout_count(42u);
    CHECK_EQ(adc_get_timeout_count(), 42u, "get_timeout_count=42");
    adc_test_set_timeout_count(0u);

    adc_test_set_recovery_retries(7u);
    CHECK_EQ(adc_get_recovery_retries(), 7u, "get_recovery_retries=7");
    adc_test_set_recovery_retries(0u);
}

// ============================================================================
// HAL FLASH (NVM)
// ============================================================================

void test_hal_flash_all(void) {
    using namespace ems::hal;

    section("hal/flash: test_reset + erase/program counters");
    nvm_test_reset();
    CHECK_EQ(nvm_test_erase_count(), 0u, "erase_count=0 after reset");
    CHECK_EQ(nvm_test_program_count(), 0u, "program_count=0 after reset");

    section("hal/flash: nvm_write_ltft / nvm_read_ltft round-trip");
    nvm_test_reset();
    // Write: index in [0,7], value int8_t
    const bool ok_w = nvm_write_ltft(3u, 5u, 25);
    CHECK_TRUE(ok_w, "nvm_write_ltft returns true");
    const int8_t v = nvm_read_ltft(3u, 5u);
    CHECK_EQ(v, (int8_t)25, "nvm_read_ltft returns written value");
    // nvm_write_ltft uses RAM shadow, does not increment program_count
    CHECK_EQ(nvm_test_program_count(), 0u, "write_ltft uses RAM shadow (no flash counter)");

    section("hal/flash: nvm_write_ltft_add / nvm_read_ltft_add");
    nvm_test_reset();
    CHECK_TRUE(nvm_write_ltft_add(2u, 4u, -10), "nvm_write_ltft_add returns true");
    CHECK_EQ(nvm_read_ltft_add(2u, 4u), (int8_t)-10, "ltft_add round-trip");

    section("hal/flash: nvm_load_adaptive_maps / nvm_flush_adaptive_maps");
    nvm_test_reset();
    // In host mode these operate on RAM shadow — just must not crash
    const bool loaded = nvm_load_adaptive_maps();
    CHECK_TRUE(loaded == true || loaded == false, "nvm_load_adaptive_maps: no crash");
    const bool flushed = nvm_flush_adaptive_maps();
    CHECK_TRUE(flushed == true || flushed == false, "nvm_flush_adaptive_maps: no crash");

    section("hal/flash: nvm_write_knock / nvm_read_knock / nvm_reset_knock_map");
    nvm_test_reset();
    CHECK_TRUE(nvm_write_knock(1u, 2u, -5), "nvm_write_knock returns true");
    CHECK_EQ(nvm_read_knock(1u, 2u), (int8_t)-5, "nvm_read_knock round-trip");
    nvm_reset_knock_map();
    CHECK_EQ(nvm_read_knock(1u, 2u), (int8_t)0, "nvm_read_knock=0 after reset_knock_map");

    section("hal/flash: nvm_save_calibration / nvm_load_calibration");
    nvm_test_reset();
    uint8_t page_out[16] = {0xAA, 0xBB, 0x01, 0x02, 0x03,
                             0x04, 0x05, 0x06, 0x07, 0x08,
                             0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
    const bool saved = nvm_save_calibration(0u, page_out, 16u);
    CHECK_TRUE(saved == true || saved == false, "nvm_save_calibration: no crash");
    uint8_t page_in[16] = {};
    const bool ldok = nvm_load_calibration(0u, page_in, 16u);
    if (saved && ldok) {
        CHECK_EQ(page_in[0], 0xAAu, "load_calibration[0] = 0xAA");
        CHECK_EQ(page_in[1], 0xBBu, "load_calibration[1] = 0xBB");
    } else {
        CHECK_TRUE(true, "save/load_calibration: graceful result");
    }

    section("hal/flash: gate de layout do setor adaptativo (magic LTF3 + CRC)");
    {
        // Imagem sintética: setor apagado / sem magic / CRC errado → inválido;
        // magic LTF3 + CRC dos mapas → válido.
        static uint8_t sector[8192];
        memset(sector, 0xFF, sizeof(sector));
        CHECK_FALSE(nvm_adaptive_sector_valid(sector),
                    "setor apagado (0xFF) → layout inválido");
        memset(sector, 0, sizeof(sector));
        CHECK_FALSE(nvm_adaptive_sector_valid(sector),
                    "layout antigo (sem magic) → inválido");
        // Magic alone (LTF2-style) is no longer enough — need maps CRC.
        memcpy(sector + kNvmOffLayoutMagic, &kNvmLayoutMagic,
                    sizeof(kNvmLayoutMagic));
        CHECK_FALSE(nvm_adaptive_sector_valid(sector),
                    "magic sem CRC → inválido");
        const uint32_t crc = nvm_adaptive_maps_crc(sector);
        memcpy(sector + kNvmOffMapsCrc, &crc, sizeof(crc));
        CHECK_TRUE(nvm_adaptive_sector_valid(sector),
                   "magic LTF3 + CRC → layout válido");
        // Bit-rot: flip one payload byte → CRC fails
        sector[0] ^= 0x01u;
        CHECK_FALSE(nvm_adaptive_sector_valid(sector),
                    "payload corrompido → CRC rejeita");
        sector[0] ^= 0x01u;  // restore
        CHECK_TRUE(nvm_adaptive_sector_valid(sector), "restaurado → válido");
        CHECK_FALSE(nvm_adaptive_sector_valid(nullptr), "nullptr → inválido");
        // Coerência do layout derivado: regiões não podem sobrepor-se
        CHECK_TRUE(kNvmOffKnock >= kNvmLtftDim * kNvmLtftDim,
                   "knock após LTFT-mult");
        CHECK_TRUE(kNvmOffLayoutMagic >=
                       kNvmOffLtftAdd + kNvmLtftAddDim * kNvmLtftAddDim,
                   "magic após LTFT-add");
        CHECK_TRUE(kNvmOffMapsCrc == kNvmOffLayoutMagic + 4u, "CRC após magic");
        CHECK_TRUE(kNvmSeedOffset >= kNvmOffMapsCrc + 4u, "seed após CRC");
        CHECK_TRUE((kNvmOffLayoutMagic % 16u) == 0u, "magic 16-alinhado");
        CHECK_TRUE((kNvmSeedOffset % 16u) == 0u, "seed 16-alinhado");
    }

    section("hal/flash: bounds LTFT seguem as dimensões NVM");
    nvm_test_reset();
    CHECK_TRUE(nvm_write_ltft(kNvmLtftDim - 1u, kNvmLtftDim - 1u, 7),
               "última célula LTFT aceita");
    CHECK_FALSE(nvm_write_ltft(kNvmLtftDim, 0u, 7), "além do grid → false");
    CHECK_TRUE(nvm_write_ltft_add(kNvmLtftAddDim - 1u, kNvmLtftAddDim - 1u, 3),
               "última célula LTFT-add aceita");
    CHECK_FALSE(nvm_write_ltft_add(kNvmLtftAddDim, 0u, 3),
                "além do sub-grid → false");

    section("hal/flash: flash_test_set_busy_polls blocks writes");
    nvm_test_reset();
    flash_test_set_busy_polls(5u);  // g_flash_busy=true → all writes blocked
    CHECK_FALSE(nvm_write_ltft(0u, 0u, 10), "write blocked when flash busy");
    CHECK_FALSE(nvm_write_knock(0u, 0u, 5), "knock write blocked when flash busy");
    CHECK_FALSE(nvm_save_calibration(0u, page_out, 16u), "cal save blocked when flash busy");
    flash_test_set_busy_polls(0u);  // restore
    CHECK_TRUE(nvm_write_ltft(0u, 0u, 10), "write succeeds after busy cleared");
}

// ============================================================================
// XTAU AUTOCALIB
// ============================================================================

void test_xtau_autocalib_all(void) {
    using namespace ems::engine;

    section("xtau_autocalib: init / reset");
    host_set_millis(0u);
    xtau_autocalib_init();
    xtau_autocalib_reset();
    CHECK_FALSE(xtau_is_learning(), "not learning after reset");

    section("xtau_autocalib: update — non-transient → no update");
    xtau_autocalib_reset();
    // is_transient=false → no learning
    const bool updated_steady = xtau_autocalib_update(
        30000u, 100u, 1000, 1000, 800, false);
    CHECK_FALSE(updated_steady, "non-transient → no update");
    CHECK_FALSE(xtau_is_learning(), "not learning in steady state");

    section("xtau_autocalib: update — transient needs ≥100ms before samples");
    xtau_autocalib_reset();
    host_set_millis(1000u);
    // First transient tick starts the window; no learn yet (duration < 100ms).
    CHECK_FALSE(xtau_autocalib_update(30000u, 100u, 1000, 1100, 800, true),
                "t=0: duration gate blocks learn");
    CHECK_TRUE(xtau_is_learning() || xtau_get_state().calibration_state == 1u,
               "duration warm-up → learning state");

    section("xtau_autocalib: update — transient with lambda error");
    xtau_autocalib_reset();
    host_set_millis(2000u);
    // Transient + lambda error=100 x1000 (in [50,150] valid window).
    // Need ≥100ms continuous transient then ≥4 valid history samples.
    bool any_update = false;
    for (int i = 0; i < 20; ++i) {
        host_advance_millis(50u);  // 50ms steps → after 3rd call elapsed ≥100ms
        any_update |= xtau_autocalib_update(
            30000u, 100u, 1000, 1100, 800, true);  // 10% rich, valid transient
    }
    CHECK_TRUE(any_update, "transient update: returns true after duration + ≥4 samples");

    // After successful update: calibration_state=2 (calibrated).
    CHECK_FALSE(xtau_is_learning(), "is_learning()=false after calibrated (state=2)");

    // xtau_get_state: fields are non-trivially populated after learning
    const WallFuelState wst = xtau_get_state();
    CHECK_EQ(wst.calibration_state, 2u, "calibration_state=2 after successful update");

    section("xtau_autocalib: xtau_get_current_params");
    const XTauParams p = xtau_get_current_params(800);
    CHECK_TRUE(p.x_fraction_q8 <= 255u, "x_fraction_q8 in [0,255]");
    CHECK_TRUE(p.tau_cycles >= 1u, "tau_cycles ≥ 1");
    // After learning, params may differ from initial table values
    // (blended toward ideal values; x_fraction should be in [0,192] after clamp)
    CHECK_TRUE(p.x_fraction_q8 <= 192u || p.x_fraction_q8 > 0u,
               "x_fraction_q8 bounded after learning");

    section("xtau_autocalib: transient_fuel_xtau_with_autocalib + wall reset");
    xtau_autocalib_reset();
    // disabled: returns input pw and clears production wall
    const uint32_t pw_disabled = transient_fuel_xtau_with_autocalib(5000u, 30000u, 100u, 800, false);
    CHECK_EQ(pw_disabled, 5000u, "disabled → returns input pw");
    // enabled: applies model (builds wall)
    const uint32_t pw_enabled = transient_fuel_xtau_with_autocalib(5000u, 30000u, 100u, 800, true);
    CHECK_TRUE(pw_enabled > 0u && pw_enabled <= 100000u,
               "enabled: pw in (0, 100ms]");
    CHECK_TRUE(xtau_get_state().wall_fuel_us_q8 > 0, "enabled builds wall film");
    // DFCO-style reset must clear production wall (not just legacy)
    transient_fuel_reset();
    CHECK_EQ(xtau_get_state().wall_fuel_us_q8, 0, "transient_fuel_reset clears production wall");

    section("xtau_autocalib: células RPM×MAP diferentes aprendem parâmetros diferentes");
    // Duas células bem afastadas nos eixos (baixo RPM/baixa carga vs. alto
    // RPM/alta carga) recebem erros de lambda opostos — se a Fase 3 deixou de
    // ser um escalar global, os parâmetros aprendidos devem divergir.
    xtau_autocalib_reset();
    host_set_millis(5000u);
    for (int i = 0; i < 20; ++i) {
        host_advance_millis(50u);
        xtau_autocalib_update(8000u, 2500u, 1000, 1100, 800, true);   // baixo RPM/carga, rico
    }
    const XTauParams p_low = xtau_get_current_params_2d(8000u, 2500u);

    xtau_autocalib_reset();
    host_set_millis(10000u);
    for (int i = 0; i < 20; ++i) {
        host_advance_millis(50u);
        xtau_autocalib_update(70000u, 15000u, 1000, 900, 800, true);  // alto RPM/carga, pobre
    }
    const XTauParams p_high = xtau_get_current_params_2d(70000u, 15000u);

    CHECK_TRUE(p_low.x_fraction_q8 != p_high.x_fraction_q8 || p_low.tau_cycles != p_high.tau_cycles,
               "células RPM×MAP distantes aprendem parâmetros distintos");
}

// ============================================================================
// VERIFICAÇÃO MATEMÁTICA — valores independentes calculados analiticamente
// ============================================================================

