#pragma once

#include <cstdint>

namespace ems::engine {

/**
 * @file misfire_encoder.h
 * @brief Detector de misfire para o fork MT6835/TIM2 (modo encoder) —
 *        matemática nova (queda de velocidade angular por janela de
 *        cilindro), paralela a misfire_detect.h/.cpp (caminho CKP,
 *        intocado por este par de ficheiros). Ver
 *        docs/dev/mt6835_encoder_fork.md.
 *
 * Reutiliza os limiares de misfire_detect.h (kMisfireThresholdQ8,
 * kMisfireDebounceCycles, kMisfireFaultThreshold) — só o domínio muda
 * (counts/ticks TIM2 vs. dentes/ns), não os números de afinação.
 *
 * Diferença deliberada face ao caminho CKP: a avaliação da janela ocorre
 * na SAÍDA da janela (mudança de cilindro detectada), não numa contagem
 * fixa de amostras — sub-amostras (256 counts, ver
 * ecu_sched_angle_encoder.cpp "split light/heavy") não estão alinhadas à
 * janela de 62° da mesma forma que dentes discretos de 6° estão no
 * caminho CKP.
 *
 * Mapeamento de cilindro via cfg::cyl_tdc_deg() (engine_config.h, domínio
 * de graus, já partilhado) — não porta o cálculo de TDC tooth-domain
 * próprio de misfire_detect.cpp (nunca reconciliado com cyl_tdc_deg()).
 */

void misfire_encoder_init() noexcept;
void misfire_encoder_reset() noexcept;

// Inibe toda a detecção (decel cut, arranque) — mesmo contrato de
// misfire_set_all_inhibit() do caminho CKP.
void misfire_encoder_set_all_inhibit(bool inhibit) noexcept;

uint8_t misfire_encoder_get_event_count(uint8_t cyl) noexcept;
void    misfire_encoder_clear_events(uint8_t cyl) noexcept;

/**
 * @brief Amostra de posição/tempo — chamada a cada sub-tick do heartbeat
 *        TIM2_CH4 (256 counts, ~64×/volta). Função pura, sem MMIO.
 *        tim2_now/tim5_now: leituras já feitas pelo caller (mesmo padrão
 *        de ecu_sched_encoder_omega_sample()).
 */
void misfire_encoder_on_sample(uint32_t tim2_now, uint32_t tim5_now) noexcept;

#if defined(EMS_HOST_TEST)
// Estado de debounce por cilindro — corre sempre internamente,
// independente de EMS_MISFIRE_ENCODER_ENABLE (só a publicação em
// misfire_encoder_get_event_count() é que fica atrás dessa flag).
uint8_t misfire_encoder_test_get_debounce(uint8_t cyl) noexcept;
// Consulta directa da tabela cilindro×posição construída por init() — -1
// se nenhum cilindro tem janela nessa posição. phase_idx: 0=ECU_PHASE_A,
// 1=ECU_PHASE_B. tim2_pos: 0..16383 (posição dentro da volta).
int8_t misfire_encoder_test_cyl_at(uint8_t phase_idx, uint32_t tim2_pos) noexcept;
#endif

}  // namespace ems::engine
