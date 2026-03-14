#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "hal/adc.h"

namespace {

int g_tests_run    = 0;
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
        std::printf("FAIL %s:%d: expected %u got %u\n", \
                    __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

// ─── helpers ─────────────────────────────────────────────────────────────────

void reset_all() {
    ems::hal::adc_init();
    ems::drv::sensors_init();
    ems::drv::sensors_test_reset();
}

// tooth_period_ns = 1_000_000 ns → ticks FTM3 = 1_000_000*60/1000 = 60_000
// last_ftm3_capture e rpm_x10 irrelevantes para os testes de sensor
ems::drv::CkpSnapshot mk_snap() {
    return ems::drv::CkpSnapshot{
        1000000u,                       // tooth_period_ns
        0u,                             // tooth_index
        10000u,                         // last_ftm3_capture
        0u,                             // rpm_x10
        ems::drv::SyncState::FULL_SYNC,
        false
    };
}

// Alimenta os 4 canais rápidos do ADC0 com valor constante por N dentes.
// kFastSamplesPerRev=12, kRealTeethPerRev=58 → amostragem a cada ~4.8 dentes.
// cycles=N * 5 garante N amostras efetivas.
void feed_fast_constant(uint16_t raw, int cycles) {
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAP_SE10,   raw);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAF_V_SE11, raw);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::TPS_SE12,   raw);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::O2_SE4B,    raw);

    auto snap = mk_snap();
    for (int i = 0; i < cycles; ++i) {
        ems::drv::sensors_on_tooth(snap);
        snap.tooth_index = static_cast<uint16_t>((snap.tooth_index + 1u) % 58u);
    }
}

// ─── Testes do PROMPT (obrigatórios) ─────────────────────────────────────────

// a) IIR α=0.3: entrada constante 4095 por 20 amostras efetivas → map > 2450
void test_iir_constant_converges_to_4095() {
    reset_all();
    feed_fast_constant(4095u, 20 * 5);
    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE(s.map_kpa_x10 > 2450u);
}

// b) IIR: após saturar em 4095, entrada 0 por 10 amostras efetivas → map < 100
void test_iir_fall_to_zero_in_10_cycles() {
    reset_all();
    ems::drv::sensors_set_range(ems::drv::SensorId::MAP,
                                ems::drv::SensorRange{0u, 4095u});
    feed_fast_constant(4095u, 200 * 5);
    feed_fast_constant(0u,     10 * 5);
    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE(s.map_kpa_x10 < 100u);
}

// c) CLT fault: 3 amostras raw=4095 fora de max=3800 → bit 3 setado
void test_clt_fault_after_3_out_of_range() {
    reset_all();
    ems::drv::sensors_set_range(ems::drv::SensorId::CLT,
                                ems::drv::SensorRange{100u, 3800u});
    ems::hal::adc_test_set_raw_adc1(ems::hal::Adc1Channel::CLT_SE14, 4095u);

    ems::drv::sensors_tick_100ms();
    ems::drv::sensors_tick_100ms();
    ems::drv::sensors_tick_100ms();

    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE((s.fault_bits & (1u << 3u)) != 0u);
}

// d) MAP linearização: raw=4095 → map_kpa_x10 = 2500
void test_map_linearization_full_scale() {
    reset_all();
    feed_fast_constant(4095u, 40 * 5);
    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_EQ_U32(2500u, s.map_kpa_x10);
}

// e) TPS calibração: min=200, max=3895, raw=2047 → tps_pct_x10 ≈ 500
void test_tps_calibration_midpoint() {
    reset_all();
    ems::drv::sensors_set_tps_cal(200u, 3895u);

    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAP_SE10,   1000u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAF_V_SE11, 1000u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::TPS_SE12,   2047u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::O2_SE4B,    1000u);

    auto snap = mk_snap();
    for (int i = 0; i < 80; ++i) {
        ems::drv::sensors_on_tooth(snap);
    }

    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_TRUE(s.tps_pct_x10 >= 495u && s.tps_pct_x10 <= 505u);
}

// ─── Novos testes — correções FIX-1, FIX-2, FIX-3 ───────────────────────────

// [FIX-1] ranges padrão idênticos após sensors_init() e após sensors_test_reset()
// Verifica que MAP aceita raw=4095 (max=4095) sem gerar fault em nenhum dos dois casos.
void test_fault_ranges_consistent_after_init_and_reset() {
    // Após sensors_init()
    ems::hal::adc_init();
    ems::drv::sensors_init();

    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAP_SE10, 4095u);
    // 3 amostras — não deve ser fault porque max_raw=4095
    for (int i = 0; i < 15; ++i) {
        auto snap = mk_snap();
        ems::drv::sensors_on_tooth(snap);
    }
    {
        const auto& s = ems::drv::sensors_get();
        TEST_ASSERT_TRUE((s.fault_bits & (1u << 0u)) == 0u);  // MAP sem fault
    }

    // Após sensors_test_reset() — deve ser idêntico
    ems::drv::sensors_test_reset();
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAP_SE10, 4095u);
    for (int i = 0; i < 15; ++i) {
        auto snap = mk_snap();
        ems::drv::sensors_on_tooth(snap);
    }
    {
        const auto& s = ems::drv::sensors_get();
        TEST_ASSERT_TRUE((s.fault_bits & (1u << 0u)) == 0u);  // MAP sem fault
    }
}

// [FIX-2 revisado] sensors_on_tooth converte tooth_period_ns → ticks (FTM3 60 MHz)
// e passa diretamente para adc_pdb_on_tooth (sem divisão — PDB também a 60 MHz).
// mk_snap: tooth_period_ns = 1_000_000 ns
//   ticks = 1_000_000 * 60 / 1000 = 60_000
//   pdb_mod = 60_000 (sem divisão por 2)
void test_pdb_mod_uses_ftm3_ticks_directly() {
    reset_all();
    auto snap = mk_snap();          // tooth_period_ns = 1_000_000
    ems::drv::sensors_on_tooth(snap);
    TEST_ASSERT_EQ_U32(60000u, ems::hal::adc_test_last_pdb_mod());
}

// [FIX-3] AN1-4 publicados em sensors_tick_100ms()
void test_an1_to_an4_passthrough() {
    reset_all();
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::AN1_SE6B, 1111u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::AN2_SE7B, 2222u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::AN3_SE8B, 3333u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::AN4_SE9B,  444u);

    ems::drv::sensors_tick_100ms();

    const auto& s = ems::drv::sensors_get();
    TEST_ASSERT_EQ_U32(1111u, s.an1_raw);
    TEST_ASSERT_EQ_U32(2222u, s.an2_raw);
    TEST_ASSERT_EQ_U32(3333u, s.an3_raw);
    TEST_ASSERT_EQ_U32( 444u, s.an4_raw);
}

}  // namespace

int main() {
    std::printf("=== Testes do PROMPT (obrigatórios) ===\n");
    test_iir_constant_converges_to_4095();
    test_iir_fall_to_zero_in_10_cycles();
    test_clt_fault_after_3_out_of_range();
    test_map_linearization_full_scale();
    test_tps_calibration_midpoint();

    std::printf("=== Testes das correções (FIX-1, FIX-2, FIX-3) ===\n");
    test_fault_ranges_consistent_after_init_and_reset();
    test_pdb_mod_uses_ftm3_ticks_directly();
    test_an1_to_an4_passthrough();

    std::printf("\ntests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
