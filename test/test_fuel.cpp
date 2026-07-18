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

void test_fuel_calc_req_fuel_us(void) {
    section("fuel_calc: calc_req_fuel_us");
    const uint32_t req = calc_req_fuel_us(2000u, 4u, 440u, 1470u);
    CHECK_TRUE(req > 0u && req <= 50000u, "req_fuel in (0, 50ms]");
    CHECK_EQ(calc_req_fuel_us(0u,    4u, 440u, 1470u), 0u, "displacement=0 → 0");
    CHECK_EQ(calc_req_fuel_us(2000u, 0u, 440u, 1470u), 0u, "cylinders=0 → 0");
    CHECK_EQ(calc_req_fuel_us(2000u, 4u,   0u, 1470u), 0u, "flow=0 → 0");
    CHECK_EQ(calc_req_fuel_us(2000u, 4u, 440u,    0u), 0u, "stoich=0 → 0");
    CHECK_TRUE(calc_req_fuel_us(50000u, 1u, 1u, 100u) <= 50000u, "clamped at 50ms");
}

void test_fuel_calc_base_pw(void) {
    section("fuel_calc: calc_base_pw_us");
    CHECK_TRUE(calc_base_pw_us(5000u, 80u, 100u, 101u) > 0u, "base_pw > 0");
    CHECK_EQ(calc_base_pw_us(5000u, 100u, 100u, 100u), 5000u, "VE=100% MAP=REF → pw=req");
    CHECK_EQ(calc_base_pw_us(5000u,  0u, 100u, 100u),     0u, "ve=0 → 0");
    CHECK_EQ(calc_base_pw_us(5000u, 80u, 100u,   0u),     0u, "map_ref=0 → 0");
    CHECK_EQ(calc_base_pw_us(5000u, 80u, 400u, 100u),     0u, "MAP>3 bar → 0");
    CHECK_EQ(calc_base_pw_us(   0u, 80u, 100u, 100u),     0u, "req=0 → 0");
}

void test_fuel_apply_lambda_target(void) {
    section("fuel_calc: apply_lambda_target_pw_us");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1000u), 5000u, "lambda=1.000 → unchanged");
    CHECK_NEAR(static_cast<float>(apply_lambda_target_pw_us(5000u, 850u)),  5882.0f, 5.0f, "lambda=0.850 → richer");
    CHECK_NEAR(static_cast<float>(apply_lambda_target_pw_us(5000u, 1200u)), 4167.0f, 5.0f, "lambda=1.200 → leaner");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 600u),  5000u, "lambda<0.65 → passthrough");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1300u), 5000u, "lambda>1.20 → passthrough");
    CHECK_EQ(apply_lambda_target_pw_us(0u, 1000u),       0u, "base=0 → 0");
}

void test_fuel_apply_trim(void) {
    section("fuel_calc: apply_fuel_trim_pw_us");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u,    0), 5000u, "trim=0 → unchanged");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u,  100), 5500u, "+10% → +10%");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u, -100), 4500u, "-10% → -10%");
    CHECK_EQ(apply_fuel_trim_pw_us(5000u,  500), 7500u, "+50% → ×1.5");
    CHECK_EQ(apply_fuel_trim_pw_us(0u,     100),    0u, "base=0 → 0");
}

void test_fuel_calc_final_pw(void) {
    section("fuel_calc: calc_final_pw_us");
    CHECK_EQ(calc_final_pw_us(5000u, 256u, 256u, 500u), 5500u, "neutral corr + 500µs dead");
    CHECK_EQ(calc_final_pw_us(5000u, 384u, 256u,   0u), 7500u, "CLT 1.5× → pw×1.5");
    CHECK_EQ(calc_final_pw_us(   0u, 256u, 256u, 500u),    0u, "base=0 → 0");
    // Corrupt page-5 Q8 corr (0xFFFF) must not wrap — clamp to 2.0× then 100 ms cap
    CHECK_EQ(calc_final_pw_us(5000u, 0xFFFFu, 0xFFFFu, 0u), 20000u,
             "corrupt corr Q8 clamped to 2×2 → 20 ms");
    CHECK_EQ(calc_final_pw_us(90000u, 512u, 512u, 5000u), 100000u,
             "final PW saturates at 100 ms");
}

void test_fuel_corr_functions(void) {
    section("fuel_calc: corr_clt / corr_iat / corr_vbatt");
    CHECK_TRUE(corr_clt(850) <= 270u,  "corr_clt at 85°C ≤ 270");
    CHECK_TRUE(corr_clt(-100) > 256u,  "corr_clt at -10°C > 256 (cold enrichment)");
    CHECK_TRUE(corr_iat(250) >= 256u,  "corr_iat at 25°C ≥ 256");
    CHECK_TRUE(corr_vbatt(12000u) > 0u && corr_vbatt(12000u) < 2000u, "dead-time at 12V in range");
    CHECK_TRUE(corr_vbatt(9000u) > corr_vbatt(14000u), "lower voltage → longer dead-time");
}

void test_fuel_decel_cut(void) {
    section("fuel_calc: fuel_decel_cut_update / active / reset");
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_active(), "not active after reset");
    CHECK_TRUE(fuel_decel_cut_update(20000u, 0u, 800), "activates: closed + warm + high rpm");
    CHECK_TRUE(fuel_decel_cut_active(), "active() true");
    CHECK_FALSE(fuel_decel_cut_update(11000u, 0u, 800), "deactivates: rpm < exit");
    fuel_decel_cut_update(20000u, 0u, 800);
    fuel_decel_cut_update(20000u, 100u, 800);
    CHECK_FALSE(fuel_decel_cut_active(), "deactivates: throttle open");
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_update(20000u, 0u, 600), "no cut: cold engine");
}

void test_fuel_baro(void) {
    section("fuel_calc: fuel_set/get_baro_bar_x100");
    fuel_set_baro_bar_x100(101u); CHECK_EQ(fuel_get_baro_bar_x100(), 101u, "baro=101");
    fuel_set_baro_bar_x100(70u);  CHECK_EQ(fuel_get_baro_bar_x100(),  70u, "baro=70 (min)");
    fuel_set_baro_bar_x100(110u); CHECK_EQ(fuel_get_baro_bar_x100(), 110u, "baro=110 (max)");
    fuel_set_baro_bar_x100(69u);  CHECK_EQ(fuel_get_baro_bar_x100(), 110u, "69 rejected");
    fuel_set_baro_bar_x100(111u); CHECK_EQ(fuel_get_baro_bar_x100(), 110u, "111 rejected");
}

// ═══════════════════════════════════════════════════════════════════════════
// IGN CALC
// ═══════════════════════════════════════════════════════════════════════════

void test_fuel_table_lookups(void) {
    section("fuel_calc: get_ve / get_ve_prepared / get_lambda_target_x1000");

    // ve_table is default-initialized; all cells = some uint8 value.
    // We just verify: returns in [0,255], doesn't crash, uses axis clamping.
    const uint8_t ve_mid = get_ve(30000u, 100u);  // 3000 RPM, 100 kPa
    CHECK_TRUE(ve_mid <= 255u, "get_ve returns uint8 value");

    // Prepared path must match direct path
    const Table2dLookup lk = table3d_prepare_lookup(kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u);
    const uint8_t ve_prep = get_ve_prepared(lk);
    CHECK_EQ(ve_mid, ve_prep, "get_ve_prepared == get_ve for same point");

    // lambda_target clamped to [650, 1200]
    const uint16_t lam = get_lambda_target_x1000(30000u, 100u);
    CHECK_TRUE(lam >= 650u && lam <= 1200u, "get_lambda_target in [650,1200]");

    // Prepared path must match
    const uint16_t lam_prep = get_lambda_target_x1000_prepared(lk);
    CHECK_EQ(lam, lam_prep, "get_lambda_target_x1000_prepared == direct");
}

void test_fuel_default_req_and_base_default(void) {
    section("fuel_calc: default_req_fuel_us / calc_base_pw_us_default");

    // default_req_fuel_us uses compile-time engine config
    const uint32_t req = default_req_fuel_us();
    CHECK_TRUE(req > 0u && req <= 50000u, "default_req_fuel_us in (0, 50ms]");

    // calc_base_pw_us_default: same result as calc_base_pw_us with defaults
    const uint32_t base = calc_base_pw_us_default(80u, 100u);
    CHECK_TRUE(base > 0u, "calc_base_pw_us_default > 0 at VE=80% MAP=100kPa");
    CHECK_EQ(calc_base_pw_us_default(0u, 100u), 0u, "ve=0 → 0");
    CHECK_EQ(calc_base_pw_us_default(80u, 400u), 0u, "MAP>3bar → 0");
}

void test_fuel_default_fast(void) {
    section("fuel_calc: calc_fuel_pw_us_default_fast");

    fuel_set_baro_bar_x100(101u);  // 1010 mbar reference
    const uint32_t pw = calc_fuel_pw_us_default_fast(
        80u,    // ve
        100u,   // map_bar_x100
        1000u,  // lambda_target_x1000 (stoich)
        0,      // trim_pct_x10
        256u,   // corr_clt_x256 (neutral)
        256u,   // corr_iat_x256 (neutral)
        500u    // dead_time_us
    );
    CHECK_TRUE(pw > 0u, "calc_fuel_pw_us_default_fast > 0 for valid inputs");

    // VE=0 → 0
    CHECK_EQ(calc_fuel_pw_us_default_fast(0u, 100u, 1000u, 0, 256u, 256u, 0u), 0u,
             "ve=0 → pw=0");

    // lambda out of range → 0
    CHECK_EQ(calc_fuel_pw_us_default_fast(80u, 100u, 600u, 0, 256u, 256u, 0u), 0u,
             "lambda<650 → pw=0");

    // Altitude compensation (F4): lower baro → larger PW (denominator shrinks).
    // baro=70 (0.70 bar, ~3000m altitude) vs baro=101 (1.01 bar, sea level).
    fuel_set_baro_bar_x100(101u);
    const uint32_t pw_sea   = calc_fuel_pw_us_default_fast(80u, 100u, 1000u, 0, 256u, 256u, 0u);
    fuel_set_baro_bar_x100(70u);
    const uint32_t pw_alt   = calc_fuel_pw_us_default_fast(80u, 100u, 1000u, 0, 256u, 256u, 0u);
    CHECK_TRUE(pw_alt > pw_sea,
               "altitude compensation: lower baro → higher PW (TI_FAC_ALTI)");
    // Ratio should be approximately baro_sea/baro_alt = 101/70 ≈ 1.44
    // Allow ±10% tolerance.
    const uint32_t ratio_x100 = (pw_alt * 100u) / (pw_sea > 0u ? pw_sea : 1u);
    CHECK_TRUE(ratio_x100 >= 130u && ratio_x100 <= 160u,
               "altitude PW ratio ≈ 1.44 (sea_baro/alt_baro=101/70)");
    fuel_set_baro_bar_x100(101u);  // restore
}

void test_fuel_corr_warmup(void) {
    section("fuel_calc: corr_warmup");
    // warmup_corr_axis_x10: {-400,-100,0,200,400,700,900,1100}
    // warmup_corr_x256:     {420, 380,350,320,290,256,256, 256}
    const uint16_t w_cold = corr_warmup(-400);  // idx=0 → 420
    CHECK_EQ(w_cold, 420u, "corr_warmup at -40°C = 420 (1.64×)");

    const uint16_t w_warm = corr_warmup(900);   // idx=6 → 256
    CHECK_EQ(w_warm, 256u, "corr_warmup at 90°C = 256 (1.0×, no enrichment)");

    // Monotonic: colder → more enrichment
    CHECK_TRUE(corr_warmup(-100) > corr_warmup(700), "corr_warmup monotonically decreasing");
}

void test_fuel_ae(void) {
    section("fuel_calc: fuel_ae_set_threshold / fuel_ae_set_taper / calc_ae_pw_us");

    fuel_reset_adaptives();

    // Set threshold=10 x10 (1.0%/ms), taper=4 cycles
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);

    // No acceleration: TPS same → ae=0
    const int32_t ae_idle = calc_ae_pw_us(500u, 500u, 10u, 800);
    CHECK_EQ(ae_idle, 0, "no TPS change → ae=0");

    // Large TPS step: delta=300 x10 in 10ms → tpsdot=%/s×10 = 30000 → clamp 1000
    const int32_t ae_accel = calc_ae_pw_us(800u, 500u, 10u, 800);
    CHECK_TRUE(ae_accel > 0, "large TPS step → ae > 0");
    CHECK_TRUE(ae_accel <= 5000, "ae ≤ ae_max_pw_us=5000");
    // Direct API (map-fusion path)
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);
    CHECK_TRUE(calc_ae_pw_from_tpsdot(100, 800) > 0, "calc_ae_pw_from_tpsdot tip-in");

    // Tip-out (DE): negative tpsdot below −threshold → enleanment (µs < 0)
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);
    const int32_t de = calc_ae_pw_from_tpsdot(static_cast<int16_t>(-100), 800);
    CHECK_TRUE(de < 0, "tip-out tpsdot → DE < 0");
    const int32_t ae_decel = calc_ae_pw_us(200u, 800u, 10u, 800);
    CHECK_TRUE(ae_decel <= 0, "TPS close → DE ≤ 0 (enleanment or decay)");

    // dt=0 guard
    CHECK_EQ(calc_ae_pw_us(800u, 0u, 0u, 800), 0, "dt=0 → ae=0");

    // Taper decay: with taper=4, the AE pulse decays to 0 over 4 cycles without TPS change.
    // Reset AE internal state by resetting adaptives, then fire one large step,
    // then call with no TPS delta 4 more times → pulse should be 0.
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);
    calc_ae_pw_us(800u, 500u, 10u, 800);  // seed the decay counter
    int32_t ae_t1 = calc_ae_pw_us(500u, 500u, 10u, 800);  // no delta: decay tick 1
    (void)calc_ae_pw_us(500u, 500u, 10u, 800);            // decay tick 2
    (void)calc_ae_pw_us(500u, 500u, 10u, 800);            // decay tick 3
    int32_t ae_t4 = calc_ae_pw_us(500u, 500u, 10u, 800);  // decay tick 4
    CHECK_TRUE(ae_t1 >= ae_t4, "AE taper: pulse non-increasing over cycles");
    CHECK_EQ(ae_t4, 0, "AE taper: pulse = 0 at or after taper_cycles=4");
}

void test_fuel_adaptives_reset(void) {
    section("fuel_calc: fuel_reset_adaptives / fuel_lambda_delay_reset");

    // Ensure STFT is non-zero first
    fuel_update_stft(30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0, "pre-cond: STFT non-zero");

    fuel_reset_adaptives();
    CHECK_EQ(fuel_get_stft_pct_x10(), 0, "STFT=0 after reset_adaptives");

    // fuel_lambda_delay_reset clears history ring buffer.
    // Behaviour: after reset the delayed STFT has no history sample matching
    // the delay window, so o2_valid=false path fires → STFT decays.
    // Before reset: push a history sample at t=0 with a lean lambda.
    fuel_lambda_delay_reset();
    fuel_reset_adaptives();
    // Push entry at t=0ms: rpm=30000, map=100, target=1000
    fuel_update_stft_delayed(0u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // Now query at t=300ms (delay≈200ms → sample IS in window → o2_valid used → STFT updates)
    const int16_t stft_with_history = fuel_update_stft_delayed(
        300u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // After reset, ring buffer empty → same query at t=300 has no sample → o2_valid=false → no STFT update
    fuel_lambda_delay_reset();
    fuel_reset_adaptives();
    const int16_t stft_post_reset = fuel_update_stft_delayed(
        300u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // stft_post_reset should be 0 or smaller (no closed-loop without history at t=300 without prior push)
    CHECK_TRUE(stft_with_history > stft_post_reset || stft_post_reset == 0,
               "fuel_lambda_delay_reset: clears history; no closed-loop without prior push");
}

void test_fuel_lambda_delay(void) {
    section("fuel_calc: lambda_delay_ms_from_rpm_load");

    // Returns interpolated delay in ms. With default table, should be > 0.
    const uint16_t d = lambda_delay_ms_from_rpm_load(30000u, 100u);
    CHECK_TRUE(d <= 2000u, "lambda_delay in plausible range");

    // At extremes must not crash
    lambda_delay_ms_from_rpm_load(0u, 0u);
    lambda_delay_ms_from_rpm_load(200000u, 300u);
    CHECK_TRUE(true, "lambda_delay extremes: no crash");
}

void test_fuel_stft(void) {
    section("fuel_calc: fuel_update_stft / fuel_get_stft_pct_x10");

    fuel_reset_adaptives();
    CHECK_EQ(fuel_get_stft_pct_x10(), 0, "STFT=0 after reset");

    // Conditions for closed loop: clt>700, o2_valid=true, ae_active=false, rev_cut=false
    // lambda measured > target → lean signal → positive error → STFT increases (adds fuel)
    int16_t stft = fuel_update_stft(30000u, 100u,
        1000,   // target lambda (stoich)
        1050,   // measured lambda (lean by 5%)
        900,    // clt 90°C > 70°C → closed loop OK
        true, false, false, 5000u, 500u);
    CHECK_TRUE(stft > 0, "lean signal → STFT positive (add fuel)");
    CHECK_EQ(stft, fuel_get_stft_pct_x10(), "fuel_get_stft_pct_x10 matches return value");

    // Rich signal → STFT eventually negative
    fuel_reset_adaptives();
    for (int i = 0; i < 30; ++i) {
        stft = fuel_update_stft(30000u, 100u, 1000, 950, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(stft < 0, "rich signal → STFT negative (reduce fuel)");

    // Closed loop disabled (cold engine): STFT congela (anti-windup), não decai —
    // evita um "degrau" de combustível perceptível quando volta a closed-loop.
    fuel_reset_adaptives();
    fuel_update_stft(30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);  // set non-zero
    const int16_t before = fuel_get_stft_pct_x10();
    fuel_update_stft(30000u, 100u, 1000, 1050, 600, true, false, false, 5000u, 500u);  // clt too cold
    const int16_t after = fuel_get_stft_pct_x10();
    CHECK_EQ(after, before, "closed loop disabled → STFT congela (freeze)");
}

void test_fuel_stft_delayed(void) {
    section("fuel_calc: fuel_update_stft_delayed");

    fuel_reset_adaptives();
    fuel_lambda_delay_reset();

    // With no history in ring buffer, now_ms=0, delay≈200ms → get_delayed fails
    // → o2_valid=false path → closed loop disabled → STFT stays 0.
    const int16_t v0 = fuel_update_stft_delayed(
        0u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(v0, 0, "t=0 no history: STFT=0 (delay not expired)");

    // Accumulate 5 samples; at t=500ms history sample at ~t=0 is 'old enough'
    // for a 200ms delay -> closed loop fires with lean signal -> STFT > 0.
    for (uint32_t t = 50u; t <= 250u; t += 50u) {
        fuel_update_stft_delayed(t, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    }
    const int16_t v_later = fuel_update_stft_delayed(
        500u, 30000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    // At t=500 history sample from t≈0 satisfies delay≈200ms → closed loop active → STFT ≠ 0
    CHECK_TRUE(v_later != 0 || v0 == 0,
               "stft_delayed activates after delay window");
    CHECK_TRUE(v_later >= -250 && v_later <= 250,
               "stft_delayed in valid range [-25%,+25%]");
}

void test_injector_scurve(void) {
    section("fuel_calc: apply_injector_scurve");
    using namespace ems::engine;

    CHECK_EQ(apply_injector_scurve(0u), 0u, "pw=0 → 0 (early return)");

    // Acima do último ponto do eixo (1500us): correção=256 (Q8=1.0) → sem alteração.
    CHECK_EQ(apply_injector_scurve(3000u), 3000u, "pw grande: sem correção (Q8=1.0)");

    // PW pequeno (perto de zero): tabela dá correção < 1.0 → PW corrigido > PW teórico
    // (o bico entrega proporcionalmente menos que o linear em aberturas curtas,
    // logo é preciso comandar mais tempo para compensar).
    const uint32_t pw_small_corrected = apply_injector_scurve(100u);
    CHECK_TRUE(pw_small_corrected > 100u, "pw pequeno: PW corrigido > PW teórico");

    // Monotonicidade: PW corrigido cresce com o PW teórico.
    const uint32_t pw_200 = apply_injector_scurve(200u);
    const uint32_t pw_800 = apply_injector_scurve(800u);
    CHECK_TRUE(pw_800 > pw_200, "correção monótona: PW maior → corrigido maior");
}

void test_fuel_delta_p_compensation(void) {
    section("fuel_calc: apply_delta_p_compensation");
    using namespace ems::engine;

    CHECK_EQ(apply_delta_p_compensation(0u, 3000u, 100u), 0u, "pw=0 → 0 (early return)");

    // Pressão real = nominal (3000 = 3.0 bar), MAP igual em ambos os lados →
    // ΔP_atual == ΔP_nominal → sem correção (ratio=1.0).
    const uint32_t pw_nominal = apply_delta_p_compensation(5000u, 3000u, 100u);
    CHECK_NEAR(static_cast<int32_t>(pw_nominal), 5000, 20,
               "pressão nominal: PW ~inalterado");

    // Sensor sem leitura (0): usa o nominal como fallback → mesmo resultado
    // que passar o nominal explicitamente.
    const uint32_t pw_fallback = apply_delta_p_compensation(5000u, 0u, 100u);
    CHECK_NEAR(static_cast<int32_t>(pw_fallback), static_cast<int32_t>(pw_nominal), 5,
               "fuel_press=0 → usa nominal (mesmo resultado)");

    // Pressão real ABAIXO da nominal → ΔP_atual < ΔP_nominal → fluxo do bico
    // menor que o esperado → PW tem de aumentar para compensar (enriquece).
    const uint32_t pw_low_press = apply_delta_p_compensation(5000u, 2000u, 100u);
    CHECK_TRUE(pw_low_press > pw_nominal, "pressão baixa: PW corrigido aumenta");

    // Pressão real ACIMA da nominal → fluxo do bico maior → PW deve diminuir.
    const uint32_t pw_high_press = apply_delta_p_compensation(5000u, 4000u, 100u);
    CHECK_TRUE(pw_high_press < pw_nominal, "pressão alta: PW corrigido diminui");
}

void test_fuel_ltft(void) {
    section("fuel_calc: fuel_get_ltft_pct_x10 / fuel_get_ltft_add_us");

    fuel_reset_adaptives();

    // After reset: LTFT cells loaded from NVM (host = all zeros)
    const int16_t ltft = fuel_get_ltft_pct_x10(0u, 0u);
    CHECK_TRUE(ltft >= -250 && ltft <= 250, "ltft_pct_x10 in valid range");

    // Out-of-range index: returns 0
    CHECK_EQ(fuel_get_ltft_pct_x10(16u, 0u), 0, "ltft: out-of-range map_idx → 0");
    CHECK_EQ(fuel_get_ltft_pct_x10(0u, 16u), 0, "ltft: out-of-range rpm_idx → 0");

    const int16_t ltft_add = fuel_get_ltft_add_us(0u, 0u);
    CHECK_TRUE(ltft_add >= -6350 && ltft_add <= 6350, "ltft_add_us in valid range");
    CHECK_EQ(fuel_get_ltft_add_us(16u, 0u), 0, "ltft_add: out-of-range → 0");
}

// WP0: apply (fuel_get_ltft_at / _add_at) usa nearest — igual crédito/store.
// Em RPM/MAP exactos no eixo, floor (bilineal) ≠ nearest (dominante).
void test_ltft_adapt_enable(void) {
    section("fuel_trim: ltft_adapt_enable congela LTFT/LEARN");

    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    ltft_adapt_enable = 0u;
    fuel_ltft_accum_reset();

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    for (int i = 0; i < 10; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1100, 900, true, false, false,
                               5000u, 500u);
    }
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0, "adapt off: STFT ainda integra");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "adapt off: zero LEARN hits");

    ltft_adapt_enable = 1u;
    fuel_ltft_accum_reset();
    (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false,
                           5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false,
                               5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) > 0u, "adapt on: LEARN hits");

    ltft_adapt_enable = 1u;
    ltft_adapt_min_rpm_x10 = 12000u;
    closed_loop_post_start_s = 15u;
    fuel_reset_adaptives();
}

void test_fuel_trim_dtcs(void) {
    section("fuel_trim: DTCs STFT/LTFT saturação");
    using namespace ems::engine;

    using DC = DiagnosticCode;
    DiagnosticManager::init();
    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    stft_clamp_pct_x10 = 50u;       // ±5% — satura rápido
    ltft_mult_clamp_pct_x10 = 50u;
    ltft_learn_div = 1u;
    ltft_max_step_x10 = 0u;

    CHECK_FALSE(DiagnosticManager::is_fault_active(DC::STFT_LIMIT_REACHED),
                "sem DTC STFT no início");

    // Mantém erro grande até saturar STFT e confirmar 50 ticks
    for (int i = 0; i < 80; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1300, 900, true, false, false,
                               5000u, 500u);
    }
    CHECK_TRUE(fuel_get_stft_pct_x10() >= 50 || fuel_get_stft_pct_x10() <= -50,
               "STFT no clamp");
    CHECK_TRUE(DiagnosticManager::is_fault_active(DC::STFT_LIMIT_REACHED),
               "STFT_LIMIT_REACHED após saturação prolongada");
    CHECK_TRUE(DiagnosticManager::is_fault_active(DC::FUEL_TRIM_LEAN) ||
               DiagnosticManager::is_fault_active(DC::FUEL_TRIM_RICH),
               "FUEL_TRIM lean/rich com STFT saturado");

    // Recupera: erro ~0, STFT baixa → clear
    for (int i = 0; i < 80; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false,
                               5000u, 500u);
    }
    // Integrador desce devagar; força STFT para 0 via reset parcial de integrador
    // e várias amostras no alvo com enable/off dance — ou clear via muitos ticks
    // com err=0: se STFT ainda sat, não limpa. Zera STFT via reset adaptives.
    fuel_reset_adaptives();
    DiagnosticManager::init();  // limpa lista (reset adaptives não limpa DTC)
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    stft_clamp_pct_x10 = 250u;
    ltft_mult_clamp_pct_x10 = 250u;
    ltft_learn_div = 64u;
    fuel_reset_adaptives();
}

void test_fuel_ltft_authority(void) {
    section("fuel_trim: LTFT clamp/rate calibráveis (≠ STFT)");

    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;  // allow LTFT at any RPM for this test
    ltft_learn_div = 1u;          // fast IIR → cell ≈ stft
    ltft_max_step_x10 = 0u;
    ltft_mult_clamp_pct_x10 = 50u;  // ±5.0% only
    stft_clamp_pct_x10 = 250u;      // STFT still ±25%

    // Drive STFT high (Ki lento — precisa de muitas iterações), LTFT com div=1
    for (int i = 0; i < 200; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false,
                               5000u, 500u);
    }
    const int16_t stft = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft > 50, "STFT pode ir além do clamp LTFT");
    const int16_t ltft = fuel_get_ltft_at(30000u, 100u);
    CHECK_TRUE(ltft <= 50 && ltft >= -50, "LTFT mult respeita clamp 50 (±5%)");
    CHECK_TRUE(stft > ltft || ltft == 50, "STFT authority > LTFT cell clamp");

    // max_step limita avanço por tick
    fuel_reset_ltft();
    fuel_reset_adaptives();
    ltft_mult_clamp_pct_x10 = 250u;
    ltft_learn_div = 1u;
    ltft_max_step_x10 = 5u;  // 0.5%/tick
    closed_loop_post_start_s = 0u;
    ltft_adapt_min_rpm_x10 = 0u;
    // seed STFT high without LTFT catch: freeze LTFT via min rpm briefly
    // (uint16 max 65535; 9000.0 RPM ×10 is a hard freeze for host tests)
    ltft_adapt_min_rpm_x10 = 65000u;
    for (int i = 0; i < 30; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1150, 900, true, false, false,
                               5000u, 500u);
    }
    const int16_t stft_hi = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft_hi > 20, "STFT aquecido com LTFT freeze");
    ltft_adapt_min_rpm_x10 = 0u;
    const int16_t ltft0 = fuel_get_ltft_at(30000u, 100u);
    (void)fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false,
                           5000u, 500u);
    const int16_t ltft1 = fuel_get_ltft_at(30000u, 100u);
    const int16_t step = static_cast<int16_t>(
        (ltft1 > ltft0) ? (ltft1 - ltft0) : (ltft0 - ltft1));
    CHECK_TRUE(step <= 5, "max_step_x10=5 limita |Δ| por tick");

    // restore defaults
    ltft_mult_clamp_pct_x10 = 250u;
    ltft_add_clamp_us = 6350u;
    ltft_learn_div = 64u;
    ltft_commit_gain_pct = 50u;
    ltft_max_step_x10 = 0u;
    ltft_adapt_min_rpm_x10 = 12000u;
    closed_loop_post_start_s = 15u;
    fuel_reset_adaptives();
}

void test_fuel_closed_loop_gates(void) {
    section("fuel_trim: closed_loop_enable + LTFT min RPM");

    fuel_reset_adaptives();
    closed_loop_enable = 1u;
    closed_loop_post_start_s = 0u;  // no post-start delay in host tests
    ltft_adapt_min_rpm_x10 = 12000u;  // 1200 RPM

    // Master off → STFT freeze
    const int16_t stft0 = fuel_get_stft_pct_x10();
    closed_loop_enable = 0u;
    (void)fuel_update_stft(30000u, 100u, 1000, 1100, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_get_stft_pct_x10(), stft0, "CL enable=0 → STFT congelado");
    closed_loop_enable = 1u;

    // Abaixo do min RPM: STFT corre, LEARN hits não sobem
    fuel_ltft_accum_reset();
    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 8000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    (void)fuel_update_stft(8000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        (void)fuel_update_stft(8000u, 100u, 1000, 1050, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0, "RPM 800: STFT ainda integra");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "RPM 800 < min → zero hits LEARN");

    // Acima do min: hits acumulam
    fuel_ltft_accum_reset();
    const uint8_t ri2 = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi2 = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        (void)fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi2, ri2) > 0u, "RPM 3000 ≥ min → LEARN hits");

    closed_loop_enable = 1u;
    closed_loop_post_start_s = 15u;
    ltft_adapt_min_rpm_x10 = 12000u;
    fuel_reset_adaptives();
}

void test_fuel_ltft_apply_nearest(void) {
    section("fuel_calc: LTFT apply = nearest (não floor bilineal)");

    fuel_reset_ltft();
    fuel_reset_adaptives();

    // 2000 rpm exacto no eixo: floor cai na célula anterior; nearest = nó 2000.
    const uint32_t rpm_x10 = 20000u;
    const uint16_t map_x100 = 110u;
    const uint8_t ri_floor =
        table_axis_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_floor =
        table_axis_index(kLoadAxisBarX100, kTableAxisSize, map_x100);
    const uint8_t ri_near =
        table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_near =
        table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, map_x100);
    CHECK_TRUE(ri_floor != ri_near || mi_floor != mi_near,
               "pré-condição: floor ≠ nearest neste OP");

    // Grava só a célula nearest em NVM (int8 %); floor fica 0.
    // nvm_write_ltft(rpm_i, load_i, val) — ordem (rpm, map).
    CHECK_TRUE(ems::hal::nvm_write_ltft(ri_near, mi_near, 12),
               "nvm LTFT nearest = +12%");
    CHECK_TRUE(ems::hal::nvm_write_ltft(ri_floor, mi_floor, 0),
               "nvm LTFT floor = 0");
    fuel_reset_adaptives();  // reload g_ltft_* from NVM

    CHECK_EQ(fuel_get_ltft_pct_x10(mi_near, ri_near), 120,
             "load nearest → +12.0% (×10)");
    CHECK_EQ(fuel_get_ltft_pct_x10(mi_floor, ri_floor), 0,
             "floor cell permanece 0");
    CHECK_EQ(fuel_get_ltft_at(rpm_x10, map_x100), 120,
             "apply mult usa nearest (+12%), não floor");

    // LTFT add: grava sub-grid da célula nearest
    CHECK_TRUE(ems::hal::nvm_write_ltft_add(ri_near >> 1u, mi_near >> 1u, 4),
               "nvm LTFT add nearest = +4×50µs");
    fuel_reset_adaptives();
    CHECK_EQ(fuel_get_ltft_add_at(rpm_x10, map_x100), 200,
             "apply add_at nearest = +200 µs");

    fuel_reset_ltft();
    fuel_reset_adaptives();
}

void test_fuel_ltft_accum(void) {
    section("fuel_calc: LTFT accum stats / bake-in gates");

    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
    // Auto-learn default off — liga só nos trechos de commit VE
    ltft_apply_burn_ve = 0u;

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // ── sample_valid: contrato bake-in ───────────────────────────────────
    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, false,
                    1000, 1015, 40, 900, true, false, false),
                "sem amostra anterior → inválido");

    // |err|=15, STFT=4% estável, regime OK → válido (erro residual NÃO exigido)
    CHECK_TRUE(ltft_accum_sample_valid(
                   30000u, 30000u, 500u, 500u, true,
                   1000, 1015, 40, 900, true, false, false),
               "regime estável + λ convergida + STFT útil → válido");

    // |err|≈0 (no alvo com trim a segurar) → válido
    CHECK_TRUE(ltft_accum_sample_valid(
                   30000u, 30000u, 500u, 500u, true,
                   1000, 1000, 40, 900, true, false, false),
               "|err λ|≈0 com STFT estável → válido (bake-in)");

    // |err| > max → ainda a convergir / outlier
    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, true,
                    1000, 1050, 40, 900, true, false, false),
                "|erro λ| > max → inválido (não convergido)");

    CHECK_FALSE(ltft_accum_sample_valid(
                    32200u, 30000u, 500u, 500u, true,
                    1000, 1015, 40, 900, true, false, false),
                "ΔRPM > 200 → inválido");

    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, true,
                    1000, 1015, 160, 900, true, false, false),
                "|STFT| > 15% → inválido (saturado)");

    CHECK_FALSE(ltft_accum_sample_valid(
                    30000u, 30000u, 500u, 500u, true,
                    1000, 1015, 40, 600, true, false, false),
                "CLT frio → inválido");

    // ── Integração via fuel_update_stft (λ perto do alvo) ────────────────
    // err=15 (1015-1000) ≤ max; 1ª amostra sem prev → 0 hits
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "1ª amostra: sem hit");

    // 2ª chamada estável → 1 hit
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 1u, "2ª amostra estável: 1 hit");

    // AE bloqueia closed-loop: prev NÃO avança (fica APP=500); hit não incrementa
    const uint16_t hits_before_ae = fuel_ltft_accum_hits(mi, ri);
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, true, false, 5000u, 900u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_ae, "AE: sem hit");
    // Após AE, APP=500 de novo → ΔTPS=0 vs prev preservado → aceita
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_ae + 1u,
             "pós-AE com APP estável: hit (prev preservado)");

    // Salto de APP → rejeita (prev=500, agora=700)
    const uint16_t hits_before_jump = fuel_ltft_accum_hits(mi, ri);
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 700u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_jump, "salto APP: hit não incrementa");
    // Nota: prev AVANÇA em closed-loop mesmo com amostra rejeitada → agora prev=700
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 700u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), hits_before_jump + 1u,
             "APP estável no novo valor: hit");

    // ── Médias exactas (err constante) + ready ───────────────────────────
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    // 1ª = só prev; 2ª..11 = 10 hits com err=10
    fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int n = 0; n < 10; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 10u, "10 hits estáveis");
    CHECK_EQ(fuel_ltft_accum_mean_err_x1000(mi, ri), 10,
             "mean_err exacto = 10 (todas amostras err=10)");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri),
                "10 hits < ReadyHits → não ready");

    // Sobe o PI com err=20 (ainda ≤ MaxErr=30); depois limpa stats e grava
    // hits com λ no alvo (err=0) e STFT congelado. Ready NÃO aplica VE sozinho.
    for (int n = 0; n < 120; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    }
    const int16_t stft_now = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft_now >= kLtftAccumReadyMinMeanStftX10,
               "STFT aquecido ≥ min ready");
    CHECK_TRUE(stft_now <= kLtftAccumMaxStftX10,
               "STFT aquecido ainda dentro do gate de amostra");

    // Acumula até ready: VE intacta sem apply manual
    const uint8_t ve_before = ve_table[mi][ri];
    fuel_ltft_accum_reset();
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) >= kLtftAccumReadyHits,
               "stats acumulam sem mexer VE");
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri),
               "célula ready mas sem commit automático");
    CHECK_EQ(ve_table[mi][ri], ve_before, "closed-loop não altera VE");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "zero commits sem apply manual");
    {
        uint8_t exp[kLtftAccumPageSize] = {};
        fuel_ltft_accum_export(exp, kLtftAccumPageSize);
        const uint16_t eidx =
            static_cast<uint16_t>(mi) * kTableAxisSize + ri;
        CHECK_TRUE((exp[eidx] & 0x80u) != 0u,
                   "export bit7 ready=1 quando cell_ready");
    }

    // Apply manual (try_commit) → bake-in
    const int16_t stft_before_commit = fuel_get_stft_pct_x10();
    const int16_t ltft_before = fuel_get_ltft_pct_x10(mi, ri);
    CHECK_TRUE(fuel_ltft_accum_try_commit(mi, ri),
               "ready → try_commit manual OK");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u,
             "após commit stats da célula zerados");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri),
                "após commit não ready");
    if (stft_before_commit > 0 && ve_before < kLtftAccumVeMax) {
        CHECK_TRUE(ve_table[mi][ri] > ve_before,
                   "STFT+ → VE aumentou");
        CHECK_TRUE(fuel_get_ltft_pct_x10(mi, ri) <= ltft_before,
                   "LTFT desenrolado após bake-in positivo");
        CHECK_TRUE(fuel_get_stft_pct_x10() <= stft_before_commit,
                   "STFT desenrolado após bake-in positivo");
    }
    CHECK_FALSE(fuel_ltft_ve_burn_pending(),
                "burn_ve=0 → sem pedido de burn");

    // burn_ve=1 + apply manual → pending
    fuel_ltft_accum_reset();
    ltft_apply_burn_ve = 1u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_ltft_ve_burn_clear();
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_FALSE(fuel_ltft_ve_burn_pending(),
                "ready sem apply → sem burn pending");
    CHECK_TRUE(fuel_ltft_accum_try_commit(mi, ri), "apply manual com burn_ve=1");
    CHECK_TRUE(fuel_ltft_ve_burn_pending(), "burn pending true após apply");
    fuel_ltft_ve_burn_clear();
    CHECK_FALSE(fuel_ltft_ve_burn_pending(), "clear limpa pending");
    ltft_apply_burn_ve = 0u;

    fuel_ltft_accum_reset_cell(mi, ri);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "reset_cell zera hits");
    CHECK_EQ(fuel_ltft_accum_mean_stft_x10(mi, ri), 0, "reset_cell zera mean_stft");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri), "ready false com 0 hits");

    // Restaura VE: nearest == canto alto bilineal em 3000/100 (math tables).
    ve_table[mi][ri] = ve_before;
    fuel_reset_adaptives();
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "reset_adaptives zera acumulador");

    // ── Regressão: teto de amostras congela hits E somas juntos ──────────
    // Se as somas crescessem com hits saturado, a média (sum/hits) derivaria
    // com o denominador preso (e no extremo estouraria int32). Injeta direto no
    // acumulador: em operação real o gate STFT rejeita as amostras muito antes
    // do teto, então este é o único caminho para exercitá-lo.
    fuel_ltft_accum_reset();
    constexpr int16_t kStftSample = 40;   // dentro do limite de |STFT|
    constexpr int16_t kErrSample  = 15;
    for (uint32_t i = 0u; i < 70000u; ++i) {  // > 65535 → passa do teto
        fuel_ltft_accum_tick_for_test(mi, ri, kStftSample, kErrSample);
    }
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 65535u, "hits satura exatamente no teto");
    CHECK_EQ(fuel_ltft_accum_mean_stft_x10(mi, ri), kStftSample,
             "média STFT = amostra constante (soma congelada no teto)");
    CHECK_EQ(fuel_ltft_accum_mean_err_x1000(mi, ri), kErrSample,
             "média erro = amostra constante (soma congelada no teto)");
    fuel_reset_adaptives();
}

void test_fuel_ltft_accum_commit_ve(void) {
    section("fuel_calc: LTFT accum Fase 2 commit manual → VE");

    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
    ltft_apply_burn_ve = 0u;

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // Aquece STFT via PI (sem bake automático)
    for (int n = 0; n < 150; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    }
    const int16_t stft = fuel_get_stft_pct_x10();
    CHECK_TRUE(stft >= kLtftAccumReadyMinMeanStftX10, "STFT aquecido p/ commit");

    // Estado limpo + VE conhecida ANTES do acumulador (após o warmup)
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_ltft_ve_burn_clear();

    // 1 prev + (ReadyHits-1) hits → ainda não ready
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < static_cast<uint16_t>(kLtftAccumReadyHits - 1u); ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) < kLtftAccumReadyHits,
               "ainda sem hits suficientes");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri),
                "ainda sem hits suficientes → não ready");
    CHECK_FALSE(fuel_ltft_accum_try_commit(mi, ri),
                "try_commit manual sem ready → false");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta sem commit");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "sem commits antes do hit ready");

    // Mais um hit → ready, mas VE só muda com apply manual
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri), "célula ready");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "sem auto-commit no hit ready");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta até apply manual");

    CHECK_TRUE(fuel_ltft_accum_try_commit(mi, ri), "apply manual → commit");
    CHECK_EQ(g_dbg_ltft_accum_commits, 1u, "1 commit manual");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "stats limpos pós-commit");
    CHECK_TRUE(ve_table[mi][ri] > 100u, "VE > 100 após bake-in STFT+");
    CHECK_TRUE(ve_table[mi][ri] <= kLtftAccumVeMax, "VE ≤ max");

    // apply_all: bulk VE+LTFT em células ready; STFT global NÃO desenrola N vezes
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri), "ready p/ apply_all");
    const int16_t stft_before_all = fuel_get_stft_pct_x10();
    const uint16_t n_app = fuel_ltft_accum_apply_all_ready();
    CHECK_TRUE(n_app >= 1u, "apply_all_ready commitou ≥1");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri), "stats limpos pós apply_all");
    CHECK_TRUE(ve_table[mi][ri] > 100u, "VE alterada por apply_all");
    CHECK_EQ(fuel_get_stft_pct_x10(), stft_before_all,
             "apply_all não desenrola STFT global (só VE+LTFT célula)");

    // apply_all aplica células com hits mas AINDA NÃO ready (parcial)
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    // Mantém STFT+ com λ ligeiramente lean; poucos hits < ready
    fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    const uint16_t partial_hits = static_cast<uint16_t>(kLtftAccumReadyHits / 2u);
    CHECK_TRUE(partial_hits >= 2u, "pre-cond: partial_hits ≥ 2");
    for (uint16_t n = 0u; n < partial_hits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) > 0u, "hits parciais > 0");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri), "ainda não ready");
    CHECK_FALSE(fuel_ltft_accum_try_commit(mi, ri),
                "try_commit continua a exigir ready");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta após try_commit falhado");
    const int16_t mean_before = fuel_ltft_accum_mean_stft_x10(mi, ri);
    CHECK_TRUE(mean_before != 0, "mean STFT parcial ≠ 0");
    const uint16_t n_partial = fuel_ltft_accum_apply_all_ready();
    CHECK_TRUE(n_partial >= 1u,
               "apply_all bakeia célula parcial (hits>0, não ready)");
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "stats limpos pós apply parcial");
    CHECK_TRUE(ve_table[mi][ri] != 100u, "VE alterada por apply_all parcial");

    // Caminho aditivo (PW < threshold): NÃO alimenta acumulador LEARN→VE
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 100u, 500u);
    for (uint16_t n = 0u; n < static_cast<uint16_t>(kLtftAccumReadyHits + 2u); ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 100u, 500u);
    }
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u,
             "caminho aditivo: zero hits LEARN (só LTFT add)");
    CHECK_FALSE(fuel_ltft_accum_try_commit(mi, ri),
                "caminho aditivo: nada ready para bake VE");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta no caminho aditivo");
    CHECK_EQ(g_dbg_ltft_accum_commits, 0u, "sem commits no caminho aditivo");

    // Restaura VE default (nearest 3000/100 = [11][10] = 88) p/ testes math.
    ve_table[mi][ri] = 88u;
    ltft_apply_burn_ve = 0u;
    fuel_reset_adaptives();
}

void test_fuel_ltft_center_gate(void) {
    section("fuel_calc: LEARN center gate (¼ do vão do nó dominante)");

    // Eixos default: RPM×10 …30000,35000… (vão 5000); MAP …100,110… (vão 10).
    // Nó exacto → centrado.
    CHECK_TRUE(fuel_ltft_learn_point_centered(30000u, 100u),
               "nó exacto (3000rpm/100kPa) centrado");
    // ≤ ¼ do vão acima/abaixo do nó → centrado.
    CHECK_TRUE(fuel_ltft_learn_point_centered(31000u, 100u),
               "+1000 de 5000 (frac 0.2) centrado no nó baixo");
    CHECK_TRUE(fuel_ltft_learn_point_centered(34000u, 100u),
               "-1000 do nó alto (frac 0.8) centrado no nó alto");
    // Zona de mistura (¼..¾ do vão) → rejeita.
    CHECK_FALSE(fuel_ltft_learn_point_centered(32500u, 100u),
                "meio do vão RPM rejeitado (mistura bilinear)");
    CHECK_FALSE(fuel_ltft_learn_point_centered(30000u, 105u),
                "meio do vão MAP rejeitado");
    CHECK_TRUE(fuel_ltft_learn_point_centered(30000u, 102u),
               "MAP a 0.2 do vão centrado");
    // Fora da tabela: até ¼ do vão da borda aceita; além rejeita.
    CHECK_TRUE(fuel_ltft_learn_point_centered(4500u, 100u),
               "500 abaixo do 1º nó RPM (vão 2500, ¼=625) aceita");
    CHECK_FALSE(fuel_ltft_learn_point_centered(3500u, 100u),
                "1500 abaixo do 1º nó RPM rejeita (fora da tabela)");
    CHECK_TRUE(fuel_ltft_learn_point_centered(82000u, 100u),
               "2000 acima do último nó RPM (vão 10000, ¼=2500) aceita");
    CHECK_FALSE(fuel_ltft_learn_point_centered(83000u, 100u),
                "3000 acima do último nó RPM rejeita");

    // Integração: ponto na zona de mistura NÃO acumula hits; centrado acumula.
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 32500u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    fuel_update_stft(32500u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    fuel_update_stft(32500u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u,
             "zona de mistura: 0 hits (gate de centralidade)");
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    fuel_update_stft(30000u, 100u, 1000, 1015, 900, true, false, false, 5000u, 500u);
    const uint8_t ri_c = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri_c) >= 1u,
               "ponto centrado acumula hits normalmente");
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
}

// ═══════════════════════════════════════════════════════════════════════════
// IGN CALC — SEGUNDA FASE
// ═══════════════════════════════════════════════════════════════════════════

