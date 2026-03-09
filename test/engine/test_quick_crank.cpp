#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/quick_crank.h"
#include "drv/ckp.h"

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

#define TEST_ASSERT_EQ_I32(exp, act) do { \
    ++g_tests_run; \
    const int32_t _e = static_cast<int32_t>(exp); \
    const int32_t _a = static_cast<int32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %d got %d\n", __FILE__, __LINE__, (int)_e, (int)_a); \
    } \
} while (0)

void test_cranking_enrichment_and_spark_override() {
    ems::engine::quick_crank_reset();
    const auto out = ems::engine::quick_crank_update(
        100u, 3000u, true, 200, 20);
    TEST_ASSERT_TRUE(out.cranking);
    TEST_ASSERT_TRUE(!out.afterstart_active);
    TEST_ASSERT_TRUE(out.fuel_mult_x256 > 256u);
    TEST_ASSERT_EQ_I32(8, out.spark_deg);
    TEST_ASSERT_TRUE(out.min_pw_us >= 2000u);
}

void test_afterstart_decay_after_crank_exit() {
    ems::engine::quick_crank_reset();
    (void)ems::engine::quick_crank_update(0u, 2500u, true, 0, 15);     // cranking
    const auto out0 = ems::engine::quick_crank_update(100u, 7500u, true, 0, 15);  // exit
    const auto out1 = ems::engine::quick_crank_update(400u, 8000u, true, 0, 15);
    const auto out2 = ems::engine::quick_crank_update(2600u, 8000u, true, 0, 15);

    TEST_ASSERT_TRUE(!out0.cranking);
    TEST_ASSERT_TRUE(out0.afterstart_active);
    TEST_ASSERT_TRUE(out0.fuel_mult_x256 > 256u);
    TEST_ASSERT_TRUE(out1.fuel_mult_x256 <= out0.fuel_mult_x256);
    TEST_ASSERT_EQ_U32(256u, out2.fuel_mult_x256);
    TEST_ASSERT_TRUE(!out2.afterstart_active);
}

void test_pw_application_with_floor_and_clamp() {
    const uint32_t boosted = ems::engine::quick_crank_apply_pw_us(1000u, 512u, 2500u);
    const uint32_t saturated = ems::engine::quick_crank_apply_pw_us(90000u, 512u, 0u);
    TEST_ASSERT_EQ_U32(2500u, boosted);   // floor to minimum crank pulse
    TEST_ASSERT_EQ_U32(100000u, saturated);  // hard cap
}

// Helper: simula chegada de N dentes com RPM de cranking.
static void sim_teeth(uint32_t n, uint32_t rpm_x10) {
    ems::drv::CkpSnapshot snap{};
    snap.rpm_x10 = rpm_x10;
    snap.state   = ems::drv::SyncState::WAIT_GAP;
    for (uint32_t i = 0u; i < n; ++i) {
        ems::drv::prime_on_tooth(snap);
    }
}

void test_prime_pulse_fires_on_5th_tooth() {
    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_clt(200);  // 20°C

    // 4 primeiros dentes — sem prime ainda
    sim_teeth(4u, 3000u);
    TEST_ASSERT_EQ_U32(0u, ems::engine::quick_crank_consume_prime());

    // 5º dente — prime deve ser pendente
    sim_teeth(1u, 3000u);
    const uint32_t pw = ems::engine::quick_crank_consume_prime();
    TEST_ASSERT_TRUE(pw > 0u);

    // Segundo consume: one-shot
    TEST_ASSERT_EQ_U32(0u, ems::engine::quick_crank_consume_prime());
}

void test_prime_pulse_no_sync_required() {
    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_clt(0);  // 0°C

    // WAIT_GAP state (snap.state = WAIT_GAP) — deve disparar mesmo assim
    ems::drv::CkpSnapshot snap{};
    snap.rpm_x10 = 2000u;
    snap.state   = ems::drv::SyncState::WAIT_GAP;
    for (uint8_t i = 0u; i < 5u; ++i) {
        ems::drv::prime_on_tooth(snap);
    }
    TEST_ASSERT_TRUE(ems::engine::quick_crank_consume_prime() > 0u);
}

void test_prime_pulse_not_fired_outside_cranking() {
    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_clt(200);

    // RPM >= kCrankExitRpmX10 (7000) → não é cranking
    sim_teeth(10u, 8000u);
    TEST_ASSERT_EQ_U32(0u, ems::engine::quick_crank_consume_prime());
}

void test_prime_pulse_reset_allows_refiring() {
    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_clt(200);

    sim_teeth(5u, 3000u);
    TEST_ASSERT_TRUE(ems::engine::quick_crank_consume_prime() > 0u);

    // Após reset (key-off), prime deve poder disparar novamente
    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_clt(200);
    sim_teeth(5u, 3000u);
    TEST_ASSERT_TRUE(ems::engine::quick_crank_consume_prime() > 0u);
}

void test_prime_pw_scales_with_clt() {
    // Prime PW a -40°C deve ser maior do que a 80°C
    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_clt(-400);  // -40°C
    sim_teeth(5u, 3000u);
    const uint32_t pw_cold = ems::engine::quick_crank_consume_prime();

    ems::engine::quick_crank_reset();
    ems::engine::quick_crank_set_clt(800);  // 80°C
    sim_teeth(5u, 3000u);
    const uint32_t pw_hot = ems::engine::quick_crank_consume_prime();

    TEST_ASSERT_TRUE(pw_cold > pw_hot);
    TEST_ASSERT_TRUE(pw_hot > 0u);
}

}  // namespace

int main() {
    test_cranking_enrichment_and_spark_override();
    test_afterstart_decay_after_crank_exit();
    test_pw_application_with_floor_and_clamp();
    test_prime_pulse_fires_on_5th_tooth();
    test_prime_pulse_no_sync_required();
    test_prime_pulse_not_fired_outside_cranking();
    test_prime_pulse_reset_allows_refiring();
    test_prime_pw_scales_with_clt();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
