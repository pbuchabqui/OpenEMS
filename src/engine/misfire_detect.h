#pragma once

#include "drv/ckp.h"
#include <cstdint>

namespace ems::engine {

constexpr uint8_t  kMisfireWindowTeeth    = 10u;   // dentes da janela de potência (~60°)
constexpr uint32_t kMisfireThresholdQ8    = 287u;  // 1.12 em Q8 (256 × 1.12): default da tabela
constexpr uint8_t  kMisfireDebounceCycles = 3u;    // ciclos consecutivos para confirmar
constexpr uint8_t  kMisfireFaultThreshold = 5u;    // eventos em período de 100ms → DTC

void misfire_init() noexcept;
void misfire_reset() noexcept;

// Inibe toda a detecção (ex.: decel cut, arranque) — sem combustão é esperado.
// Chame com true quando corte intencional estiver ativo; false para retomar.
void misfire_set_all_inhibit(bool inhibit) noexcept;

// Limiar por rpm × MAP (MS42: 12 mapas; aqui ms42x.misfire_excess_q8 4×4).
// Chamar do loop de fundo; a ISR do CKP só lê o valor resolvido.
void misfire_set_operating_point(uint32_t rpm_x10, uint16_t map_kpa) noexcept;
// Limiar em uso (Q8, 256 = 1,0× o período previsto).
uint32_t misfire_threshold_q8() noexcept;

// Retorna número de eventos de misfire acumulados desde o último clear.
// Seguro para leitura fora do ISR (uint8_t = atômico em ARM).
uint8_t misfire_get_event_count(uint8_t cyl) noexcept;
void    misfire_clear_events(uint8_t cyl) noexcept;

// Total de eventos confirmados por cilindro, saturado em 0xFFFF. Retido na
// NVM entre partidas (adapt_retention); só misfire_init/set_total o mudam.
uint16_t misfire_get_total(uint8_t cyl) noexcept;
void     misfire_set_total(uint8_t cyl, uint16_t total) noexcept;

}  // namespace ems::engine

// Hook chamado no ISR do CKP (ems::drv namespace, igual aos outros hooks).
// Implementação forte em misfire_detect.cpp substitui o símbolo fraco de ckp.cpp.
namespace ems::drv {
void misfire_on_tooth(const CkpSnapshot& snap) noexcept;
}  // namespace ems::drv
