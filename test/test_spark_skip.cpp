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
    section("spark_skip: Bresenham por ciclo (720°) + rotação + clamp");

    spark_skip_reset();
    CHECK_EQ(spark_skip_mask(), 0u, "reset: máscara vazia");
    CHECK_EQ(spark_skip_get_ratio_q8(), 0u, "reset: ratio 0");

    // Ratio 0 → nunca inibe.
    for (int i = 0; i < 8; ++i) { spark_skip_on_rev((i & 1) == 0); }
    CHECK_EQ(spark_skip_mask(), 0u, "ratio 0: máscara sempre vazia");

    // Clamp: pedir 255 → aceita só 128 (50%).
    spark_skip_set_ratio_q8(255u);
    CHECK_EQ(spark_skip_get_ratio_q8(), 128u, "ratio clampa a 128 (50%)");

    // 50%: 4 faíscas/ciclo × 0.5 = 2 cilindros mascarados por ciclo (2 voltas cada).
    spark_skip_reset();
    spark_skip_set_ratio_q8(128u);
    uint32_t total = 0u;
    uint8_t seen_union = 0u;
    for (int i = 0; i < 8; ++i) {
        spark_skip_on_rev((i & 1) == 0);
        total += popcount4(spark_skip_mask());
        seen_union |= spark_skip_mask();
    }
    CHECK_EQ(total, 16u, "50%: 2 cilindros mascarados × 8 voltas = 16");
    CHECK_EQ(seen_union, 0x0Fu, "rotação cobre os 4 cilindros");

    // 25% (64 Q8): 1 cilindro mascarado por ciclo → 8 voltas = 8.
    spark_skip_reset();
    spark_skip_set_ratio_q8(64u);
    total = 0u;
    for (int i = 0; i < 8; ++i) {
        spark_skip_on_rev((i & 1) == 0);
        total += popcount4(spark_skip_mask());
    }
    CHECK_EQ(total, 8u, "25%: 1 cilindro mascarado × 8 voltas = 8");

    // set_ratio(0) limpa máscara imediatamente (saída do soft zone).
    spark_skip_set_ratio_q8(0u);
    CHECK_EQ(spark_skip_mask(), 0u, "ratio→0 limpa máscara");

    spark_skip_reset();

    // Fase manda: revolução fora de cycle_start nunca recalcula, mesmo em sequência.
    spark_skip_reset();
    spark_skip_set_ratio_q8(128u);
    spark_skip_on_rev(true);
    const uint8_t held = spark_skip_mask();
    spark_skip_on_rev(false);
    CHECK_EQ(spark_skip_mask(), held, "2ª volta mantém a máscara");
    spark_skip_on_rev(false);
    CHECK_EQ(spark_skip_mask(), held, "sem cycle_start nunca recalcula");

    spark_skip_reset();

    // Fase congelada em true: recalcula uma vez (borda), não a cada volta.
    spark_skip_reset();
    spark_skip_set_ratio_q8(128u);
    spark_skip_on_rev(true);
    const uint8_t first = spark_skip_mask();
    spark_skip_on_rev(true);
    CHECK_EQ(spark_skip_mask(), first, "cycle_start congelado em true não recalcula");

    // Fase congelada em false: a máscara velha é zerada, não fica presa.
    spark_skip_reset();
    spark_skip_set_ratio_q8(128u);
    spark_skip_on_rev(true);
    CHECK_TRUE(spark_skip_mask() != 0u, "máscara ativa após borda");
    for (int i = 0; i < 4; ++i) { spark_skip_on_rev(false); }
    CHECK_EQ(spark_skip_mask(), 0u, "sem borda por 4 voltas: máscara zerada");

    // Perda de sync limpa a máscara.
    spark_skip_reset();
    spark_skip_set_ratio_q8(128u);
    spark_skip_on_rev(true);
    spark_skip_clear_mask();
    CHECK_EQ(spark_skip_mask(), 0u, "clear_mask zera");

    spark_skip_reset();
}
