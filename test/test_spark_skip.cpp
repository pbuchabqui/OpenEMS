#include "test/harness.h"

#include <cstdint>

#include "engine/spark_skip.h"

using namespace ems::engine;

static uint8_t popcount4(uint8_t m) {
    uint8_t n = 0u;
    for (uint8_t b = 0u; b < 4u; ++b) { n = static_cast<uint8_t>(n + ((m >> b) & 1u)); }
    return n;
}

void test_spark_skip(void) {
    section("spark_skip: Bresenham por revolução + rotação + clamp");

    spark_skip_reset();
    CHECK_EQ(spark_skip_mask(), 0u, "reset: máscara vazia");
    CHECK_EQ(spark_skip_get_ratio_q8(), 0u, "reset: ratio 0");

    // Ratio 0 → nunca inibe.
    for (int i = 0; i < 8; ++i) { spark_skip_on_rev(); }
    CHECK_EQ(spark_skip_mask(), 0u, "ratio 0: máscara sempre vazia");

    // Clamp: pedir 255 → aceita só 128 (50%).
    spark_skip_set_ratio_q8(255u);
    CHECK_EQ(spark_skip_get_ratio_q8(), 128u, "ratio clampa a 128 (50%)");

    // 50%: 2 fires/rev × 0.5 = 1 salto por volta, exacto.
    spark_skip_reset();
    spark_skip_set_ratio_q8(128u);
    uint32_t total = 0u;
    uint8_t seen_union = 0u;
    for (int i = 0; i < 8; ++i) {
        spark_skip_on_rev();
        total += popcount4(spark_skip_mask());
        seen_union |= spark_skip_mask();
    }
    CHECK_EQ(total, 8u, "50%: 1 salto/volta × 8 voltas = 8");
    CHECK_EQ(seen_union, 0x0Fu, "rotação cobre os 4 cilindros");

    // 25% (64 Q8): 0.5 salto/volta → 4 saltos em 8 voltas (Bresenham conserva).
    spark_skip_reset();
    spark_skip_set_ratio_q8(64u);
    total = 0u;
    for (int i = 0; i < 8; ++i) {
        spark_skip_on_rev();
        total += popcount4(spark_skip_mask());
    }
    CHECK_EQ(total, 4u, "25%: 4 saltos em 8 voltas");

    // set_ratio(0) limpa máscara imediatamente (saída do soft zone).
    spark_skip_set_ratio_q8(0u);
    CHECK_EQ(spark_skip_mask(), 0u, "ratio→0 limpa máscara");

    spark_skip_reset();
}
