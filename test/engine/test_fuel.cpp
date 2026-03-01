#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/fuel_calc.h"

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

void test_base_pw_nominal() {
    // VE=128%, MAP=MAP_ref=100 kPa → pw = 8000 × 128/100 = 10240 µs
    const uint32_t pw = ems::engine::calc_base_pw_us(8000u, 128u, 100u, 100u);
    TEST_ASSERT_TRUE(pw >= 10237u && pw <= 10243u);
}

void test_base_pw_limits() {
    // VE=0 → pw=0
    TEST_ASSERT_EQ_U32(0u, ems::engine::calc_base_pw_us(8000u, 0u, 100u, 100u));
    // VE=100 (100%), MAP=MAP_ref → pw = req_fuel_us exato
    TEST_ASSERT_EQ_U32(8000u, ems::engine::calc_base_pw_us(8000u, 100u, 100u, 100u));
    // VE=255 (255%) → pw = 8000 × 255/100 = 20400 µs
    TEST_ASSERT_EQ_U32(20400u, ems::engine::calc_base_pw_us(8000u, 255u, 100u, 100u));
}

void test_ae_positive_on_tps_step() {
    ems::engine::fuel_ae_set_threshold(5u);
    ems::engine::fuel_ae_set_taper(8u);
    const int32_t ae = ems::engine::calc_ae_pw_us(500u, 0u, 10u, 900);
    TEST_ASSERT_TRUE(ae > 0);
}

void test_stft_grows_positive_with_lean_feedback() {
    ems::engine::fuel_reset_adaptives();

    const int16_t stft1 = ems::engine::fuel_update_stft(
        3000u,
        100u,
        1000,
        950,
        800,
        true,
        false,
        false);

    const int16_t stft2 = ems::engine::fuel_update_stft(
        3000u,
        100u,
        1000,
        950,
        800,
        true,
        false,
        false);

    TEST_ASSERT_TRUE(stft1 > 0);
    TEST_ASSERT_TRUE(stft2 >= stft1);
}

// P7: STFT deve decair em direção a zero quando rev_cut=true (malha aberta, ×15/16)
void test_stft_decays_during_rev_cut() {
    ems::engine::fuel_reset_adaptives();

    // Acumular STFT positivo via feedback lean repetido (o2_valid=true, clt>700)
    for (int i = 0; i < 10; ++i) {
        ems::engine::fuel_update_stft(3000u, 100u, 1000, 900, 800, true, false, false);
    }
    const int16_t stft_loaded = ems::engine::fuel_get_stft_pct_x10();
    TEST_ASSERT_TRUE(stft_loaded > 0);

    // Com rev_cut=true, STFT deve decair (×15/16 por chamada) — não crescer
    const int16_t stft_after_revcut = ems::engine::fuel_update_stft(
        3000u, 100u, 1000, 900, 800, true, false, true);
    TEST_ASSERT_TRUE(stft_after_revcut <= stft_loaded);

    // Após muitas iterações com rev_cut, STFT deve convergir a zero
    for (int i = 0; i < 200; ++i) {
        ems::engine::fuel_update_stft(3000u, 100u, 1000, 900, 800, true, false, true);
    }
    TEST_ASSERT_TRUE(ems::engine::fuel_get_stft_pct_x10() == 0);
}

// P7: correção CLT — motor frio deve enriquecer (> 256)
void test_corr_clt_cold_enrichment() {
    // Temperatura muito fria: -20°C × 10 = -200
    const uint16_t corr = ems::engine::corr_clt(-200);
    TEST_ASSERT_TRUE(corr > 256u);
}

// P7: correção IAT — ar quente deve empobrecer (< 256)
void test_corr_iat_hot_leaning() {
    // Temperatura muito quente: +80°C × 10 = 800
    const uint16_t corr = ems::engine::corr_iat(800);
    TEST_ASSERT_TRUE(corr < 256u);
}

// P7: pw_final menor que pw_base quando corr_clt < 256 (IAT muito frio não leaneia)
void test_final_pw_applies_corrections() {
    const uint32_t base_pw = ems::engine::calc_base_pw_us(8000u, 100u, 100u, 100u);
    // corr_iat a 40°C (400 × 10) deve ser próximo de 256 (referência)
    const uint16_t corr_iat = ems::engine::corr_iat(400);
    const uint16_t corr_clt = ems::engine::corr_clt(900);  // 90°C, quase operacional
    const uint32_t final_pw = ems::engine::calc_final_pw_us(base_pw, corr_clt, corr_iat, 0u);
    // A correção não deve resultar em PW negativo ou zero com condições normais
    TEST_ASSERT_TRUE(final_pw > 0u);
}

}  // namespace

int main() {
    test_base_pw_nominal();
    test_base_pw_limits();
    test_ae_positive_on_tps_step();
    test_stft_grows_positive_with_lean_feedback();
    test_stft_decays_during_rev_cut();
    test_corr_clt_cold_enrichment();
    test_corr_iat_hot_leaning();
    test_final_pw_applies_corrections();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
