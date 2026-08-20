#pragma once

#include <cstdint>

namespace ems::engine {

// ── Amostragem de MAP em janela angular por cilindro + auto-balanceamento ────
// (estilo FOME modules/map_averaging, changelog #610)
//
// A cada amostra (poll 2 ms no encoder), o valor corrente do MAP é acumulado
// na janela angular activa. O ciclo de 720° é dividido em 4 slots de 180°;
// a janela do slot k abre em map_window_open_deg + k·180° e dura
// map_window_len_deg. Ao fechar cada janela guarda-se a média; quando as 4
// janelas de um ciclo fecham, actualiza-se o desvio EMA de cada slot face à
// média dos quatro (balanceamento — assimetria de colector por cilindro).
//
// Slot ≠ cilindro físico: o mapeamento depende do offset do trigger e da ordem
// de ignição — calibrar map_window_open_deg observando qual slot responde a
// qual cilindro. Helper map_window_slot_for_cyl() usa TDC/180 (convenção
// open_deg alinhado). Requer FULL_SYNC + fase de came confirmada (cmp_confirms ≥ 2);
// sem came a atribuição 720° é ambígua (slots emparelhados trocariam).
//
// Contexto: chamado da ISR do CKP (via sensors_on_tooth) — só inteiros; a
// única divisão ocorre no fecho de janela (~4×/ciclo). Com map_window_enable,
// o finalize encoder (enc_cyl_setpoints) usa o MAP do slot por cilindro
// para ΔP e escala de fluxo; senão continua telemetria-only no path CKP.

// Amostra angular (poll 2 ms no encoder). cycle_deg = 0..719.
// Gate interno: map_window_enable == 0 → no-op imediato.
// Sem FULL_SYNC + CMP confirmado aborta a janela parcial.
void map_window_on_sample(uint16_t cycle_deg, uint16_t map_bar_x1000,
                          bool full_sync, bool cmp_ok) noexcept;

// Slot 0..3 para o cilindro físico (TDC/180), assumindo open_deg calibrado.
uint8_t map_window_slot_for_cyl(uint8_t cyl) noexcept;

// Última média fechada do slot (bar × 1000; 0 = ainda sem janela fechada).
uint16_t map_window_slot_bar_x1000(uint8_t slot) noexcept;

// Desvio EMA (α = 1/8) do slot vs média dos 4 slots (bar × 1000, com sinal).
int16_t map_window_balance_x1000(uint8_t slot) noexcept;

// Nº de ciclos completos (4 janelas fechadas) — diagnóstico.
uint32_t map_window_cycles() noexcept;

// Nº de janelas inteiras saltadas (poll de 2ms mais largo que a janela, a
// RPM alto ou map_window_len_deg curto) — diagnóstico. Slot saltado mantém
// o último valor válido em map_window_slot_bar_x1000(); este contador é a
// forma de detetar essa staleness em vez de confiar cegamente no valor.
uint32_t map_window_skip_count() noexcept;

// Reset total (init / host tests / perda de sync prolongada).
void map_window_reset() noexcept;

#if defined(EMS_HOST_TEST)
void map_window_test_set_slot_bar_x1000(uint8_t slot, uint16_t bar_x1000) noexcept;
#endif

}  // namespace ems::engine
