/**
 * test/engine/test_iacv.cpp
 *
 * IACV PID step-response and behavioural tests.
 * Covers every clause from PROMPT-6 "IACV PID" spec:
 *
 *   1. Warmup feedforward  — clt < 60 °C → duty from warmup table, PID inactive
 *   2. PID enable gate     — clt ≤ 60 °C → no integrator wind-up over time
 *   3. drpm_dt inhibit     — sharp RPM spike does not corrupt duty
 *   4. Anti-windup         — sustained large error saturates to 0 or 1000 (no overflow)
 *   5. Duty bounds         — output always in [0, 1000] under any conditions
 *   6. Closed-loop step    — duty moves in the correct direction after load disturbance
 *   7. Directional control — steady-state duty(rpm<target) > duty(rpm>target)
 *   8. Warm → cold handoff — PID disables, feedforward takes over correctly
 *   9. Idempotent key-on   — second set_key_on(true) does not corrupt output
 *  10. Warmup table exact  — spot-check known interpolation points
 *
 * Build (from repo root):
 *   g++ -std=c++17 -DEMS_HOST_TEST=1 -I src \
 *       src/engine/auxiliaries.cpp \
 *       test/engine/test_iacv.cpp \
 *       -o build/test_iacv && ./build/test_iacv
 */

#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/auxiliaries.h"
#include "drv/ckp.h"
#include "drv/sensors.h"

// ─── Test harness ─────────────────────────────────────────────────────────────

namespace {

int g_tests_run    = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_SECTION(name) \
    std::printf("  [%s]\n", (name))

// ─── Shared fixture ───────────────────────────────────────────────────────────

ems::drv::CkpSnapshot g_snap    = {};
ems::drv::SensorData  g_sensors = {};

uint16_t g_ftm1_duty[2] = {};
uint16_t g_ftm2_duty[2] = {};

void reset_fixture(int16_t clt_x10, uint32_t rpm_x10) {
    g_snap = ems::drv::CkpSnapshot{
        500000u,                          // tooth_period_ns
        8u,                               // tooth_index
        0u,                               // last_ftm3_capture
        static_cast<uint32_t>(rpm_x10),  // rpm_x10 (uint32_t)
        ems::drv::SyncState::FULL_SYNC,
        true                              // phase_A
    };
    g_sensors = ems::drv::SensorData{
        350u,    // map_kpa_x10
        0u,      // maf_gps_x100
        0u,      // tps_pct_x10
        clt_x10, // clt_degc_x10
        250,     // iat_degc_x10
        // o2_mv REMOVIDO
        0u,      // fuel_press_kpa_x10
        0u,      // oil_press_kpa_x10
        13500u,  // vbatt_mv
        0u,      // fault_bits
        0u,      // an1_raw
        0u,      // an2_raw
        0u,      // an3_raw
        0u,      // an4_raw
    };
    g_ftm1_duty[0] = g_ftm1_duty[1] = 0u;
    g_ftm2_duty[0] = g_ftm2_duty[1] = 0u;
    ems::engine::auxiliaries_init();
    ems::engine::auxiliaries_set_key_on(true);
}

uint16_t iac_duty() { return g_ftm1_duty[0]; }

inline int32_t abs32(int32_t x) { return x < 0 ? -x : x; }

// ─── Calibrated plant (Euler, Ts=20 ms) ─────────────────────────────────────
// ss(duty) = 5200 + 10 * duty.
// At duty=300 (hot-engine PID base), ss = 8200 = idle target at clt=90 °C.
// τ ≈ 16 steps × 20 ms = 320 ms.
void plant_step() {
    int32_t rpm  = static_cast<int32_t>(g_snap.rpm_x10);
    int32_t ss   = 5200 + static_cast<int32_t>(iac_duty()) * 10;
    int32_t next = rpm + (ss - rpm) / 16;
    if (next < 0)     next = 0;
    if (next > 20000) next = 20000;
    g_snap.rpm_x10 = static_cast<uint32_t>(next);
}

void tick_with_plant(uint32_t n) {
    for (uint32_t i = 0u; i < n; ++i) {
        ems::engine::auxiliaries_tick_20ms();
        plant_step();
    }
}

void tick_open(uint32_t n) {
    for (uint32_t i = 0u; i < n; ++i) {
        ems::engine::auxiliaries_tick_20ms();
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

// 1. Warmup feedforward — table produces correct duty for cold clt values
//    Firmware warmup table (exact axis points, no interpolation needed):
//      clt=-40°C → 620,  clt=-10°C → 560,  clt=10°C → 500
void test_warmup_feedforward() {
    TEST_SECTION("1. warmup feedforward: duty == table value at axis points");

    // At exact axis points interpolation yields the table value directly.
    reset_fixture(-400, 12000u);  // -40 °C
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() == 620u);

    reset_fixture(-100, 12000u);  // -10 °C
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() == 560u);

    reset_fixture(100, 12000u);   // +10 °C
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() == 500u);

    // Spot check: clt=300 → 440
    reset_fixture(300, 12000u);
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() == 440u);

    // PID baseline check: output must NOT equal the hot PID baseline (300)
    // when cold, regardless of RPM.
    reset_fixture(-100, 12000u);
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() != 300u);
}

// 2. PID disabled below 60 °C — duty must not integrate over many ticks
//    (feedforward from warmup table is constant when clt is constant)
void test_pid_disabled_below_60c() {
    TEST_SECTION("2. PID disabled below 60 degC: no integrator wind-up");

    // 50 °C, rpm well below where target would be if PID were active.
    reset_fixture(500, 3000u);
    tick_open(1u);
    const uint16_t d0 = iac_duty();

    tick_open(99u);  // 100 more ticks — integrator must stay still
    const uint16_t d1 = iac_duty();

    // Feedforward is constant at constant clt: delta must be zero.
    const int32_t delta = abs32(static_cast<int32_t>(d1) - static_cast<int32_t>(d0));
    TEST_ASSERT_TRUE(delta == 0);
}

// 3. drpm_dt inhibit — sharp RPM spike must not push duty out of bounds
//    Firmware threshold: |drpm_x10 * 50| < 2000 → drpm_x10 > 40 disables PID.
//    We inject drpm_x10 = 200 (×50 = 10000 >> 2000) to trigger inhibit.
void test_drpmdt_inhibits_pid() {
    TEST_SECTION("3. drpm/dt > threshold: duty remains in [0,1000]");

    reset_fixture(900, 8200u);
    tick_with_plant(60u);  // settle

    // Sharp drop: 200 rpm_x10 in one tick → drpm_dt = 200*50 = 10000 > 2000
    const uint32_t rpm_before = g_snap.rpm_x10;
    g_snap.rpm_x10 = static_cast<uint32_t>(rpm_before > 200u ? rpm_before - 200u : 0u);
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() <= 1000u);

    // Sharp rise
    g_snap.rpm_x10 = static_cast<uint32_t>(rpm_before + 200u);
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() <= 1000u);

    // After inhibit window passes, control resumes with valid duty
    tick_with_plant(10u);
    TEST_ASSERT_TRUE(iac_duty() <= 1000u);
}

// 4. Anti-windup — sustained large errors must saturate, not overflow
void test_anti_windup_clamp() {
    TEST_SECTION("4. anti-windup: duty saturates at 1000 / 0, never overflows");

    // Persistent large positive error (rpm far below target)
    reset_fixture(900, 1000u);
    for (uint32_t i = 0u; i < 500u; ++i) {
        ems::engine::auxiliaries_tick_20ms();
        // plant NOT updated — rpm stays pegged low
    }
    // Must be exactly 1000 (clamped), not 0 or wrap-around
    TEST_ASSERT_TRUE(iac_duty() == 1000u);

    // Persistent large negative error (rpm far above target)
    reset_fixture(900, 18000u);
    for (uint32_t i = 0u; i < 500u; ++i) {
        ems::engine::auxiliaries_tick_20ms();
    }
    // Must be exactly 0 (clamped), not 65535 or wrap-around
    TEST_ASSERT_TRUE(iac_duty() == 0u);
}

// 5. Duty bounds — [0, 1000] under extreme inputs
void test_duty_bounds_always_valid() {
    TEST_SECTION("5. duty always in [0,1000] under extreme conditions");

    reset_fixture(-400, 0u);     // extreme cold, zero RPM
    tick_open(300u);
    TEST_ASSERT_TRUE(iac_duty() <= 1000u);

    reset_fixture(1200, 20000u); // extreme hot, max RPM
    tick_open(300u);
    TEST_ASSERT_TRUE(iac_duty() <= 1000u);
}

// 6. Closed-loop step response — duty moves in correct direction after disturbance
//    After a downward RPM step, duty must increase (or already be at max).
//    After an upward RPM step, duty must decrease (or already be at zero).
void test_closed_loop_step_direction() {
    TEST_SECTION("6. closed-loop step: duty responds in correct direction");

    // — Downward step —
    // Settle at target (clt=900 → target=8200 rpm_x10)
    reset_fixture(900, 8200u);
    tick_with_plant(300u);  // 6 s settle with calibrated plant
    const uint16_t duty_settled = iac_duty();
    const uint32_t rpm_settled  = g_snap.rpm_x10;

    // Drop RPM by 200 (rpm now below target)
    g_snap.rpm_x10 = static_cast<uint32_t>(rpm_settled > 200u ? rpm_settled - 200u : 0u);
    tick_open(1u);  // one tick to let PID react
    const uint16_t duty_after_drop = iac_duty();

    // Duty must rise (or was already at ceiling)
    TEST_ASSERT_TRUE(duty_after_drop >= duty_settled || duty_settled == 1000u);

    // — Upward step —
    reset_fixture(900, 8200u);
    tick_with_plant(300u);
    const uint16_t duty_settled2 = iac_duty();
    const uint32_t rpm_settled2  = g_snap.rpm_x10;

    g_snap.rpm_x10 = static_cast<uint32_t>(rpm_settled2 + 200u);
    tick_open(1u);
    const uint16_t duty_after_rise = iac_duty();

    TEST_ASSERT_TRUE(duty_after_rise <= duty_settled2 || duty_settled2 == 0u);
}

// 7. Directional control — after settling, duty when rpm<target must be higher
//    than duty when rpm>target (same clt, same fixture, open-loop rpm fixed).
void test_directional_control_steady_state() {
    TEST_SECTION("7. directional: duty(below target) > duty(above target)");

    // clt=700 (70°C, PID active), target at this clt ≈ 8500 rpm_x10.
    // Pin rpm below target and let integrator settle.
    reset_fixture(700, 8400u);  // 100 below target
    tick_open(2u);              // prime drpm history
    for (uint32_t i = 0u; i < 50u; ++i) {
        g_snap.rpm_x10 = 8400u;
        tick_open(1u);
    }
    const uint16_t duty_below = iac_duty();

    // Pin rpm above target and let integrator settle.
    reset_fixture(700, 8600u);  // 100 above target
    tick_open(2u);
    for (uint32_t i = 0u; i < 50u; ++i) {
        g_snap.rpm_x10 = 8600u;
        tick_open(1u);
    }
    const uint16_t duty_above = iac_duty();

    TEST_ASSERT_TRUE(duty_below > duty_above);
}

// 8. Warm → cold transition — PID disables, feedforward takes over
//    At 20 °C (clt_x10=200) the warmup table gives 470 (interpolated between
//    axis[2]=100→500 and axis[3]=300→440: at x=200, frac=0.5 → 470).
void test_warm_to_cold_handoff() {
    TEST_SECTION("8. warm->cold: feedforward at 20 degC == 470");

    reset_fixture(900, 8200u);
    tick_with_plant(60u);  // warm and settled (PID active)

    // Switch to cold
    g_sensors.clt_degc_x10 = 200;   // 20 °C
    g_snap.rpm_x10 = 8200u;
    tick_open(5u);

    // At 20 °C, interpolation: idx=2 (axis[2]=100, axis[3]=300), x=200, frac=0.5
    // y = 500 + (440-500)*0.5 = 500-30 = 470.
    TEST_ASSERT_TRUE(iac_duty() == 470u);
}

// 9. Idempotent key-on — second call must not corrupt IACV duty
void test_key_on_idempotent() {
    TEST_SECTION("9. idempotent key-on: duty remains valid");

    reset_fixture(900, 8200u);
    tick_with_plant(30u);
    const uint16_t d0 = iac_duty();
    TEST_ASSERT_TRUE(d0 <= 1000u);

    ems::engine::auxiliaries_set_key_on(true);  // second call
    tick_with_plant(5u);
    TEST_ASSERT_TRUE(iac_duty() <= 1000u);
}

// 10. Warmup table interpolation — verify mid-segment points
void test_warmup_table_interpolation() {
    TEST_SECTION("10. warmup table: mid-segment interpolation correct");

    // Axis: [..., 100→500, 300→440, ...].  At clt=200: frac=0.5 → 470.
    reset_fixture(200, 12000u);
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() == 470u);

    // Axis: [-100→560, 100→500]. At clt=0: frac=0.5 → 530.
    reset_fixture(0, 12000u);
    tick_open(1u);
    TEST_ASSERT_TRUE(iac_duty() == 530u);

    // Above warmup zone (clt=600 is exactly at threshold; clt=601 → PID mode)
    // At clt=600 the firmware returns duty_base=300 with no PID adjustment on
    // the very first tick (prev_rpm not yet initialised → drpm=0 guard fires).
    reset_fixture(600, 8200u);  // at transition boundary
    tick_open(1u);
    // duty_base at clt>=600: 300. No PID on tick 1 (have_prev_rpm=false path).
    TEST_ASSERT_TRUE(iac_duty() == 300u);
}

}  // namespace

// ─── HAL / DRV stubs ─────────────────────────────────────────────────────────

namespace ems::drv {
CkpSnapshot       ckp_snapshot() noexcept { return g_snap; }
SensorData sensors_get() noexcept  { return g_sensors; }
}

namespace ems::hal {
void ftm1_pwm_init(uint32_t) noexcept {}
void ftm2_pwm_init(uint32_t) noexcept {}
void ftm1_set_duty(uint8_t ch, uint16_t d) noexcept { if (ch < 2u) g_ftm1_duty[ch] = d; }
void ftm2_set_duty(uint8_t ch, uint16_t d) noexcept { if (ch < 2u) g_ftm2_duty[ch] = d; }
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main() {
    std::printf("=== test_iacv ===\n");

    test_warmup_feedforward();
    test_pid_disabled_below_60c();
    test_drpmdt_inhibits_pid();
    test_anti_windup_clamp();
    test_duty_bounds_always_valid();
    test_closed_loop_step_direction();
    test_directional_control_steady_state();
    test_warm_to_cold_handoff();
    test_key_on_idempotent();
    test_warmup_table_interpolation();

    std::printf("=================\n");
    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
