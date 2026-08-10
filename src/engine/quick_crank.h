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

/**
 * @brief Prime pulse por posição absoluta (fork MT6835/TIM2, modo encoder).
 *
 * Sem hook por-dente disponível em modo encoder (não há wheel de dentes) —
 * corre a partir do slot de 2 ms do loop principal, ramo EMS_MT6835_ENCODER,
 * em vez de prime_on_tooth() (drv::ckp, caminho por-dente do CKP físico).
 * Deriva RPM cru de delta de posição TIM2 / delta de tempo entre polls —
 * NUNCA de CkpSnapshot::tooth_period_ns (fica sempre 0 em modo encoder, ver
 * ecu_sched_encoder_heartbeat_tick()). Alvo em counts TIM2:
 * sanitized_prime_tooth() × 6° (kDegPerTooth de map_window.cpp), convertido
 * via 16384 counts/volta. Mesma margem de overshoot-reset (+5 "dentes") do
 * caminho por-dente.
 *
 * @param tim2_now Posição absoluta actual (TIM2->CNT, contagem X4 do
 *                 encoder).
 * @param now_ms   Timestamp do loop principal (millis()).
 */
void quick_crank_encoder_poll(uint32_t tim2_now, uint32_t now_ms) noexcept;

QuickCrankOutput quick_crank_update(uint32_t now_ms,
                                    uint32_t rpm_x10,
                                    bool sync_available,
                                    int16_t clt_x10,
                                    int16_t base_spark_deg) noexcept;

uint32_t quick_crank_apply_pw_us(uint32_t base_pw_us,
                                 uint16_t fuel_mult_x256,
                                 uint32_t min_pw_us) noexcept;

/**
 * @brief Atualiza CLT e dead time usados pelo hook de dente para calcular o
 *        prime pulse. Chamar do loop de fundo (2 ms); seguro para ISR-side.
 */
void quick_crank_set_prime_context(int16_t clt_x10, uint16_t dead_time_us) noexcept;

/**
 * @brief Compatibilidade: atualiza apenas CLT do contexto de prime.
 */
void quick_crank_set_clt(int16_t clt_x10) noexcept;

/**
 * @brief Consome o prime pulse pendente (one-shot, atômico).
 *
 * @return Largura de pulso em µs se um prime pulse foi agendado pelo ISR de
 *         dente desde a última chamada; 0 caso contrário.
 */
uint32_t quick_crank_consume_prime() noexcept;

/// Returns true while the engine is in cranking mode (last quick_crank_update).
bool is_cranking() noexcept;

/// Returns true while afterstart enrichment is active (last quick_crank_update).
bool is_afterstart() noexcept;

/// True when cranking and APP is at/above flood-clear threshold (default 70%).
/// Used to cut scheduled fuel and suppress prime while clearing a flood.
bool crank_flood_clear_active(uint16_t app_pct_x10) noexcept;

/// Flood-clear APP threshold (pct×10). Default 700 = 70%.
extern uint16_t crank_flood_tps_x10;

}  // namespace ems::engine
