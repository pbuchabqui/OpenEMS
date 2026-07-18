#include "engine/spark_skip.h"

namespace ems::engine {

namespace {
constexpr uint8_t kMaxRatioQ8 = 128u;   // 50% — acima disso é caso p/ fuel cut
constexpr uint8_t kFiresPerRev = 2u;    // 4-cil 4T: ~2 eventos IGN por volta

uint8_t  g_ratio_q8 = 0u;
uint16_t g_acc_q8   = 0u;   // acumulador Bresenham (Q8)
uint8_t  g_rot      = 0u;   // cilindro inicial da próxima inibição (rotação)
uint8_t  g_mask     = 0u;
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

void spark_skip_on_rev() noexcept {
    if (g_ratio_q8 == 0u) {
        g_mask = 0u;
        return;
    }
    // Orçamento da volta: ratio × eventos-por-volta, em Q8.
    g_acc_q8 = static_cast<uint16_t>(
        g_acc_q8 + static_cast<uint16_t>(g_ratio_q8) * kFiresPerRev);
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

void spark_skip_reset() noexcept {
    g_ratio_q8 = 0u;
    g_acc_q8   = 0u;
    g_rot      = 0u;
    g_mask     = 0u;
}

}  // namespace ems::engine
