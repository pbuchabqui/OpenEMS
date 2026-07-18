#pragma once

#include <cstdint>

#include "drv/ckp.h"

namespace ems::engine {

// ── Amostragem de MAP em janela angular por cilindro + auto-balanceamento ────
// (estilo FOME modules/map_averaging, changelog #610)
//
// A cada dente do CKP (6° de virabrequim), o valor corrente do MAP é acumulado
// na janela angular activa. O ciclo de 720° é dividido em 4 slots de 180°;
// a janela do slot k abre em map_window_open_deg + k·180° e dura
// map_window_len_deg. Ao fechar cada janela guarda-se a média; quando as 4
// janelas de um ciclo fecham, actualiza-se o desvio EMA de cada slot face à
// média dos quatro (balanceamento — assimetria de colector por cilindro).
//
// Slot ≠ cilindro físico: o mapeamento depende do offset do trigger e da ordem
// de ignição — calibrar map_window_open_deg observando qual slot responde a
// qual cilindro. Requer FULL_SYNC + fase de came confirmada (cmp_confirms ≥ 2);
// sem came a atribuição 720° é ambígua (slots emparelhados trocariam).
//
// Contexto: chamado da ISR do CKP (via sensors_on_tooth) — só inteiros; a
// única divisão ocorre no fecho de janela (~4×/ciclo). Nesta fase o resultado
// é medição/telemetria; aplicação ao fuel por cilindro é fase posterior.

// Chamada por dente. map_bar_x1000 = leitura instantânea já convertida.
// Gate interno: map_window_enable == 0 → no-op imediato.
void map_window_on_tooth(const ems::drv::CkpSnapshot& snap,
                         uint16_t map_bar_x1000) noexcept;

// Última média fechada do slot (bar × 1000; 0 = ainda sem janela fechada).
uint16_t map_window_slot_bar_x1000(uint8_t slot) noexcept;

// Desvio EMA (α = 1/8) do slot vs média dos 4 slots (bar × 1000, com sinal).
int16_t map_window_balance_x1000(uint8_t slot) noexcept;

// Nº de ciclos completos (4 janelas fechadas) — diagnóstico.
uint32_t map_window_cycles() noexcept;

// Reset total (init / host tests / perda de sync prolongada).
void map_window_reset() noexcept;

}  // namespace ems::engine
