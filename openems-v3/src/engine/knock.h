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

// Janela de knock por cilindro (tipicamente 10-90 ATDC, agendada por FTM0).
void knock_window_open(uint8_t cyl) noexcept;
void knock_window_close(uint8_t cyl) noexcept;

// ISR do comparador (CMP0): conta apenas quando janela estiver ativa.
void knock_cmp0_isr() noexcept;

// Fecha o ciclo de combustao do cilindro e aplica algoritmo.
void knock_cycle_complete(uint8_t cyl) noexcept;

uint16_t knock_get_retard_x10(uint8_t cyl) noexcept;
uint8_t knock_get_vosel() noexcept;

#if defined(EMS_HOST_TEST)
uint8_t knock_test_get_knock_count(uint8_t cyl) noexcept;
bool knock_test_window_active() noexcept;
uint8_t knock_test_window_cyl() noexcept;
#endif

}  // namespace ems::engine
