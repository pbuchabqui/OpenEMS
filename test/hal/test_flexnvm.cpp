#include <cstdint>
#include <cstdio>
#include <cstring>

#define EMS_HOST_TEST 1
#include "hal/flexnvm.h"

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

#define TEST_ASSERT_EQ_I32(exp, act) do { \
    ++g_tests_run; \
    const int32_t _e = static_cast<int32_t>(exp); \
    const int32_t _a = static_cast<int32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %ld got %ld\n", __FILE__, __LINE__, (long)_e, (long)_a); \
    } \
} while (0)

void test_reset() {
    ems::hal::nvm_test_reset();
}

void test_ltft_write_read_persists() {
    test_reset();
    TEST_ASSERT_TRUE(ems::hal::nvm_write_ltft(3u, 7u, static_cast<int8_t>(-12)));
    TEST_ASSERT_EQ_I32(-12, ems::hal::nvm_read_ltft(3u, 7u));
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_ltft(0u, 0u));
}

void test_ltft_bounds() {
    test_reset();
    TEST_ASSERT_TRUE(!ems::hal::nvm_write_ltft(16u, 0u, 1));
    TEST_ASSERT_TRUE(!ems::hal::nvm_write_ltft(0u, 16u, 1));
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_ltft(17u, 1u));
}

void test_calibration_save_load_roundtrip() {
    test_reset();
    uint8_t src[64] = {};
    uint8_t dst[64] = {};
    for (uint8_t i = 0u; i < 64u; ++i) {
        src[i] = static_cast<uint8_t>(i ^ 0x5Au);
    }

    TEST_ASSERT_TRUE(ems::hal::nvm_save_calibration(2u, src, sizeof(src)));
    TEST_ASSERT_TRUE(ems::hal::nvm_load_calibration(2u, dst, sizeof(dst)));
    TEST_ASSERT_TRUE(std::memcmp(src, dst, sizeof(src)) == 0);
    TEST_ASSERT_EQ_I32(1, ems::hal::nvm_test_erase_count());
    TEST_ASSERT_EQ_I32(1, ems::hal::nvm_test_program_count());
}

void test_calibration_rewrite_replaces_previous_data() {
    test_reset();
    uint8_t first[16] = {};
    uint8_t second[16] = {};
    uint8_t out[16] = {};
    std::memset(first, 0xAA, sizeof(first));
    std::memset(second, 0x11, sizeof(second));

    TEST_ASSERT_TRUE(ems::hal::nvm_save_calibration(0u, first, sizeof(first)));
    TEST_ASSERT_TRUE(ems::hal::nvm_save_calibration(0u, second, sizeof(second)));
    TEST_ASSERT_TRUE(ems::hal::nvm_load_calibration(0u, out, sizeof(out)));
    TEST_ASSERT_TRUE(std::memcmp(second, out, sizeof(out)) == 0);
    TEST_ASSERT_EQ_I32(2, ems::hal::nvm_test_erase_count());
    TEST_ASSERT_EQ_I32(2, ems::hal::nvm_test_program_count());
}

void test_timeout_when_ccif_busy() {
    test_reset();
    ems::hal::nvm_test_set_ccif_busy_polls(2'000'000u);
    TEST_ASSERT_TRUE(!ems::hal::nvm_write_ltft(1u, 1u, 9));

    uint8_t data[8] = {};
    ems::hal::nvm_test_set_ccif_busy_polls(2'000'000u);
    TEST_ASSERT_TRUE(!ems::hal::nvm_save_calibration(1u, data, sizeof(data)));
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_test_erase_count());
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_test_program_count());
}

void test_invalid_calibration_parameters() {
    test_reset();
    uint8_t data[4] = {1u, 2u, 3u, 4u};
    TEST_ASSERT_TRUE(!ems::hal::nvm_save_calibration(32u, data, sizeof(data)));
    TEST_ASSERT_TRUE(!ems::hal::nvm_save_calibration(1u, nullptr, sizeof(data)));
    TEST_ASSERT_TRUE(!ems::hal::nvm_save_calibration(1u, data, 0u));

    uint8_t out[4] = {};
    TEST_ASSERT_TRUE(!ems::hal::nvm_load_calibration(32u, out, sizeof(out)));
    TEST_ASSERT_TRUE(!ems::hal::nvm_load_calibration(1u, nullptr, sizeof(out)));
    TEST_ASSERT_TRUE(!ems::hal::nvm_load_calibration(1u, out, 0u));
}

// ---------------------------------------------------------------------------
// knock_map tests
// ---------------------------------------------------------------------------

void test_knock_write_read_persists() {
    test_reset();
    // Escreve retraso de –5° (–50 × 0,1°) na célula (3, 5)
    TEST_ASSERT_TRUE(ems::hal::nvm_write_knock(3u, 5u, static_cast<int8_t>(-50)));
    TEST_ASSERT_EQ_I32(-50, ems::hal::nvm_read_knock(3u, 5u));
    // Célula não escrita deve retornar 0
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_knock(0u, 0u));
}

void test_knock_write_read_all_cells() {
    test_reset();
    // Preenche todo o mapa 8×8 com valores distintos e verifica
    for (uint8_t r = 0u; r < 8u; ++r) {
        for (uint8_t c = 0u; c < 8u; ++c) {
            const int8_t val = static_cast<int8_t>((r * 8u + c) - 32);
            TEST_ASSERT_TRUE(ems::hal::nvm_write_knock(r, c, val));
        }
    }
    for (uint8_t r = 0u; r < 8u; ++r) {
        for (uint8_t c = 0u; c < 8u; ++c) {
            const int8_t expected = static_cast<int8_t>((r * 8u + c) - 32);
            TEST_ASSERT_EQ_I32(expected, ems::hal::nvm_read_knock(r, c));
        }
    }
}

void test_knock_bounds() {
    test_reset();
    // Índices fora de [0..7] devem rejeitar write e retornar 0 no read
    TEST_ASSERT_TRUE(!ems::hal::nvm_write_knock(8u, 0u, -10));
    TEST_ASSERT_TRUE(!ems::hal::nvm_write_knock(0u, 8u, -10));
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_knock(8u, 0u));
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_knock(0u, 8u));
}

void test_knock_isolated_from_ltft() {
    test_reset();
    // Gravar na knock_map não deve corromper LTFT e vice-versa
    TEST_ASSERT_TRUE(ems::hal::nvm_write_ltft(7u, 7u, static_cast<int8_t>(42)));
    TEST_ASSERT_TRUE(ems::hal::nvm_write_knock(7u, 7u, static_cast<int8_t>(-99)));
    TEST_ASSERT_EQ_I32(42, ems::hal::nvm_read_ltft(7u, 7u));
    TEST_ASSERT_EQ_I32(-99, ems::hal::nvm_read_knock(7u, 7u));
}

void test_knock_reset_clears_map() {
    test_reset();
    TEST_ASSERT_TRUE(ems::hal::nvm_write_knock(0u, 0u, static_cast<int8_t>(-127)));
    TEST_ASSERT_TRUE(ems::hal::nvm_write_knock(7u, 7u, static_cast<int8_t>(127)));
    ems::hal::nvm_reset_knock_map();
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_knock(0u, 0u));
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_knock(7u, 7u));
    // LTFT deve permanecer intacto após reset do knock
    TEST_ASSERT_EQ_I32(0, ems::hal::nvm_read_ltft(0u, 0u));
}

void test_knock_timeout_when_ccif_busy() {
    test_reset();
    ems::hal::nvm_test_set_ccif_busy_polls(2'000'000u);
    TEST_ASSERT_TRUE(!ems::hal::nvm_write_knock(1u, 1u, static_cast<int8_t>(-20)));
}

}  // namespace

int main() {
    test_ltft_write_read_persists();
    test_ltft_bounds();
    test_calibration_save_load_roundtrip();
    test_calibration_rewrite_replaces_previous_data();
    test_timeout_when_ccif_busy();
    test_invalid_calibration_parameters();
    test_knock_write_read_persists();
    test_knock_write_read_all_cells();
    test_knock_bounds();
    test_knock_isolated_from_ltft();
    test_knock_reset_clears_map();
    test_knock_timeout_when_ccif_busy();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
