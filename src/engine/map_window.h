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
// única divisão ocorre no fecho de janela (~4×/ciclo). Aplicação ao fuel POR
// CILINDRO (per-cylinder trim usando o slot individual) continua fase
// posterior — feature maior, exige resolver a ambiguidade slot↔cilindro
// acima. O que existe hoje (map_window_mean_bar_x1000, ver abaixo) é só a
// média escalar dos 4 slots, opcionalmente usada como MAP único para todos
// os cilindros — ver map_window_use_for_fuel em calibration.h.
//
// AVISO — precondição de calibração antes de map_window_use_for_fuel=1:
// com map_window_open_deg=0 (default) e map_window_len_deg=90° (default,
// "meia fase de admissão" — não os 180° do slot inteiro), a média dos 4
// slots amostra um arco de 90° arbitrário e NÃO calibrado por cilindro, não
// o ciclo completo. Isso não cancela ripple de admissão — pode introduzir
// um viés sistemático em vez de eliminar ruído, dependendo de onde esse
// arco cai em relação ao evento real de admissão de cada cilindro. Calibrar
// map_window_open_deg (observando qual slot responde a qual cilindro, ver
// acima) e revisar map_window_len_deg no motor real ANTES de ligar
// map_window_use_for_fuel — caso contrário deixar em telemetria (só
// map_window_enable=1) é mais seguro.

// Chamada por dente. map_bar_x1000 = leitura instantânea já convertida.
// Gate interno: map_window_enable == 0 → no-op imediato.
void map_window_on_tooth(const ems::drv::CkpSnapshot& snap,
                         uint16_t map_bar_x1000) noexcept;

// Última média fechada do slot (bar × 1000; 0 = ainda sem janela fechada).
uint16_t map_window_slot_bar_x1000(uint8_t slot) noexcept;

// Desvio EMA (α = 1/8) do slot vs média dos 4 slots (bar × 1000, com sinal).
int16_t map_window_balance_x1000(uint8_t slot) noexcept;

// Nº de ciclos completos (4 janelas fechadas) — diagnóstico e gate de
// validade (map_window_cycles() == 0 → nenhuma média viva, não confiar).
// Perda de FULL_SYNC / came zera este contador (e os slots) para o fuel
// não reutilizar MAP de pré-dropout no re-lock.
uint32_t map_window_cycles() noexcept;

// Média dos 4 slots do ciclo mais recente (bar × 1000). Não decide
// freshness/validade nem viés de calibração — caller (main_stm32.cpp) deve
// checar map_window_enable, FULL_SYNC+cmp_confirms≥2 NO INSTANTE DO USO
// (não só quando a janela fechou — evita servir média congelada após perda
// de sync) e map_window_cycles() > 0 antes de confiar neste valor. Ver
// AVISO de calibração acima antes de usar para combustível.
uint16_t map_window_mean_bar_x1000() noexcept;

// Reset total (init / host tests / perda de sync prolongada).
void map_window_reset() noexcept;

}  // namespace ems::engine
