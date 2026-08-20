#include "engine/spark_skip.h"

namespace ems::engine {

namespace {
constexpr uint8_t kMaxRatioQ8 = 128u;   // 50% — acima disso é caso p/ fuel cut
constexpr uint8_t kFiresPerRev = 2u;    // 4-cil 4T: ~2 eventos IGN por volta

// g_ratio_q8/g_acc_q8/g_mask atravessam contexto: spark_skip_on_rev() corre
// na ISR TIM2 CC4 (heavy tick do heartbeat do encoder), enquanto
// spark_skip_set_ratio_q8()/spark_skip_mask() correm no loop de 2ms
// (loop_2ms_fuel_ign.cpp) — mesmo padrão de g_inj_inhibit_mask
// (ecu_sched_internal.h), que já é volatile por esta razão. Sem isto, nada
// impede o compilador de assumir que o valor não muda entre duas leituras
// dentro da mesma função (nunca acontece hoje só porque o acesso é sempre
// via chamada a função fora da TU, e este Makefile não liga -flto —
// invariante frágil do build atual, não uma garantia da linguagem).
// g_rot fica de fora de propósito, não por suposição: o único outro
// escritor seria spark_skip_reset(), mas essa função não tem NENHUM
// chamador em src/ hoje — só em test/test_spark_skip.cpp (host-only). Se
// spark_skip_reset() alguma vez for chamada a partir do loop de 2ms em
// produção, g_rot passa a ter o mesmo problema e tem de ganhar volatile
// também.
volatile uint8_t  g_ratio_q8 = 0u;
volatile uint16_t g_acc_q8   = 0u;   // acumulador Bresenham (Q8)
uint8_t           g_rot      = 0u;   // cilindro inicial da próxima inibição (rotação)
volatile uint8_t  g_mask     = 0u;
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
