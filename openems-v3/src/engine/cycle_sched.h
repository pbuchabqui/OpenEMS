#pragma once

#include <cstdint>

namespace ems::engine {

// Inicializa o agendador de ciclo: pré-computa os dentes-gatilho por cilindro.
// Deve ser chamado após CKP sincronizar, antes de cycle_sched_enable(true).
void cycle_sched_init() noexcept;

// Habilita ou desabilita o agendamento (desabilitado por padrão).
void cycle_sched_enable(bool en) noexcept;

// Atualiza os parâmetros pré-calculados para todos os cilindros.
// Chamado pelo loop de background periódico com os valores mais recentes.
//
// pw_ticks     : largura de pulso de injeção em ticks FTM0
// dead_ticks   : dead-time do injetor em ticks FTM0
// soi_lead_x10 : avanço do SOI em relação ao TDC compressão (graus × 10)
// advance_x10  : avanço total de ignição (graus × 10, positivo = avanço BTDC)
// dwell_x10    : ângulo de dwell em graus × 10
//
// NOTA: advance_x10 e dwell_x10 são armazenados para uso futuro quando o
// agendamento de ignição for implementado com suporte adequado de hardware.
void cycle_sched_update(uint32_t pw_ticks,
                        uint16_t dead_ticks,
                        uint16_t soi_lead_x10,
                        int16_t  advance_x10,
                        uint16_t dwell_x10) noexcept;

#if defined(EMS_HOST_TEST)
void cycle_sched_test_reset() noexcept;
// Retorna o dente-gatilho de injeção para um slot de disparo (0..3).
bool cycle_sched_test_trigger(uint8_t slot, uint8_t& tooth, bool& phase) noexcept;
// Retorna o dente-gatilho de ignição SET (início de dwell) para um slot (0..3).
bool cycle_sched_test_ign_set_trigger(uint8_t slot, uint8_t& tooth, bool& phase) noexcept;
// Retorna o dente-gatilho de ignição CLR (faísca) para um slot (0..3).
bool cycle_sched_test_ign_clr_trigger(uint8_t slot, uint8_t& tooth, bool& phase) noexcept;
#endif

// ============================================================================
// Angular Execution Loop - Forced Update Support
// ============================================================================

/**
 * @brief Force immediate update of scheduling parameters for all cylinders.
 *
 * Deve ser chamada APENAS do loop background — não da ISR.
 * Ver CYC-01: chamar de ISR causa race condition com g_pending[].
 */
void cycle_sched_force_update() noexcept;

/**
 * @brief Check if scheduling is enabled (used by diagnostics).
 */
bool cycle_sched_should_update_on_gap() noexcept;

/**
 * @brief Update scheduling parameters with pre-calculated values.
 */
void cycle_sched_update_with_calculated(uint32_t pw_ticks,
                                        uint16_t dead_ticks,
                                        uint16_t soi_lead_x10,
                                        int16_t  advance_x10,
                                        uint16_t dwell_x10) noexcept;

}  // namespace ems::engine
