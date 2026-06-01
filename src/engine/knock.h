#pragma once

#include <cstdint>

namespace ems::engine {

constexpr uint8_t kKnockCylinders = 4u;

// Retardo por cilindro em graus x10 (ex.: 25 = 2.5 deg).
// Contrato para leitura por engine/ign_calc.
extern volatile uint16_t knock_retard_x10[kKnockCylinders];

void knock_init() noexcept;
void knock_save_to_nvm() noexcept;
void knock_set_event_threshold(uint8_t threshold) noexcept;

// Threshold ADC para detecção de knock (0-4095, 12-bit).
// Substitui o VOSEL do comparador analógico que o STM32H562 não possui.
// Aumentar = menos sensível; reduzir = mais sensível.
void knock_set_adc_threshold(uint16_t threshold) noexcept;
uint16_t knock_get_adc_threshold() noexcept;

// Janela de knock por cilindro — abre no DWELL_START, fecha no próximo DWELL_START.
void knock_window_open(uint8_t cyl) noexcept;
void knock_window_close(uint8_t cyl) noexcept;

// Chamado por sample_fast_channels() (ISR de dente CKP) com a leitura ADC
// do canal do sensor de knock (PA5/ADC1_IN6). Conta amostras acima do
// threshold enquanto a janela estiver ativa.
void knock_adc_update(uint16_t raw) noexcept;

// Fecha o ciclo de combustao do cilindro e aplica algoritmo retard/recovery.
void knock_cycle_complete(uint8_t cyl) noexcept;

// Fecha a janela corrente (se houver) e avalia o cilindro que estava aberto.
// Chamado no ECU_ACT_DWELL_START do próximo cilindro (ISR-safe).
void knock_window_cycle_end() noexcept;

uint16_t knock_get_retard_x10(uint8_t cyl) noexcept;

#if defined(EMS_HOST_TEST)
uint8_t knock_test_get_knock_count(uint8_t cyl) noexcept;
bool knock_test_window_active() noexcept;
uint8_t knock_test_window_cyl() noexcept;
void knock_test_set_adc_raw(uint16_t raw) noexcept;
#endif

}  // namespace ems::engine
