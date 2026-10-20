#include "engine/spark_skip.h"

namespace ems::engine {

namespace {
constexpr uint8_t kMaxRatioQ8 = 128u;   // 50% — acima disso é caso p/ fuel cut
constexpr uint8_t kFiresPerCycle = 4u;  // 4-cil 4T: cada cilindro dispara 1x por ciclo de 720°

uint8_t  g_ratio_q8 = 0u;
uint16_t g_acc_q8   = 0u;   // acumulador Bresenham (Q8)
uint8_t  g_rot      = 0u;   // cilindro inicial da próxima inibição (rotação)
uint8_t  g_mask     = 0u;
bool     g_prev_start = false;   // cycle_start da chamada anterior: só a borda de subida abre o ciclo
uint8_t  g_since_start = 0u;     // chamadas desde o último recálculo
// Num ciclo de 720° normal há 2 chamadas por recálculo. Muito além disso a fase do
// came congelou ou o sync caiu: a máscara velha não pode ficar presa nos mesmos cilindros.
constexpr uint8_t kStaleCalls = 4u;
}  // namespace

void spark_skip_set_ratio_q8(uint8_t ratio_q8) noexcept {
    g_ratio_q8 = (ratio_q8 > kMaxRatioQ8) ? kMaxRatioQ8 : ratio_q8;
    if (g_ratio_q8 == 0u) {
        g_acc_q8 = 0u;
        g_mask   = 0u;
    }
}

uint8_t spark_skip_get_ratio_q8() noexcept {
    return g_ratio_q8;
}

void spark_skip_on_rev(bool cycle_start) noexcept {
    if (g_ratio_q8 == 0u) {
        g_mask = 0u;
        return;
    }
    // A máscara vale por 720°: assim cada cilindro mascarado perde exatamente uma
    // faísca por ciclo (por volta, só 2 dos 4 cilindros disparam e metade dos saltos
    // caía num cilindro que não ia disparar, entregando metade do ratio pedido).
    const bool rising = cycle_start && !g_prev_start;
    g_prev_start = cycle_start;
    if (!rising) {
        if (g_since_start < kStaleCalls && ++g_since_start >= kStaleCalls) { g_mask = 0u; }
        return;
    }
    g_since_start = 0u;
    // Orçamento do ciclo: ratio × eventos-por-ciclo, em Q8.
    g_acc_q8 = static_cast<uint16_t>(
        g_acc_q8 + static_cast<uint16_t>(g_ratio_q8) * kFiresPerCycle);
    uint8_t skips = static_cast<uint8_t>(g_acc_q8 >> 8u);
    g_acc_q8 &= 0xFFu;
    if (skips > 4u) {
        skips = 4u;
    }
    uint8_t mask = 0u;
    for (uint8_t i = 0u; i < skips; ++i) {
        mask |= static_cast<uint8_t>(1u << ((g_rot + i) & 3u));
    }
    g_rot  = static_cast<uint8_t>((g_rot + skips) & 3u);
    g_mask = mask;
}

uint8_t spark_skip_mask() noexcept {
    return g_mask;
}

void spark_skip_clear_mask() noexcept {
    g_mask         = 0u;
    g_prev_start   = false;
    g_since_start  = kStaleCalls;
}

void spark_skip_reset() noexcept {
    g_prev_start  = false;
    g_since_start = 0u;
    g_ratio_q8 = 0u;
    g_acc_q8   = 0u;
    g_rot      = 0u;
    g_mask     = 0u;
}

}  // namespace ems::engine
