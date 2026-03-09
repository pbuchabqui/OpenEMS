#pragma once

#include <cstdint>

namespace ems::engine {

struct QuickCrankOutput {
    bool cranking;
    bool afterstart_active;
    uint16_t fuel_mult_x256;
    int16_t spark_deg;
    uint32_t min_pw_us;
    uint32_t prime_pw_us;  ///< Duração do prime pulse calculado (µs); informativo
};

void quick_crank_reset() noexcept;

QuickCrankOutput quick_crank_update(uint32_t now_ms,
                                    uint32_t rpm_x10,
                                    bool full_sync,
                                    int16_t clt_x10,
                                    int16_t base_spark_deg) noexcept;

uint32_t quick_crank_apply_pw_us(uint32_t base_pw_us,
                                 uint16_t fuel_mult_x256,
                                 uint32_t min_pw_us) noexcept;

/**
 * @brief Atualiza o valor de CLT usado pelo hook de dente para calcular o
 *        prime pulse. Chamar do loop de fundo (2 ms); seguro para ISR-side.
 */
void quick_crank_set_clt(int16_t clt_x10) noexcept;

/**
 * @brief Consome o prime pulse pendente (one-shot, atômico).
 *
 * @return Largura de pulso em µs se um prime pulse foi agendado pelo ISR de
 *         dente desde a última chamada; 0 caso contrário.
 */
uint32_t quick_crank_consume_prime() noexcept;

}  // namespace ems::engine
