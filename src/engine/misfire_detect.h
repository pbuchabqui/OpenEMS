#pragma once

#include "drv/ckp.h"
#include <cstdint>

namespace ems::engine {

constexpr uint8_t  kMisfireWindowTeeth    = 10u;   // dentes da janela de potência (~60°)
constexpr uint32_t kMisfireThresholdQ8    = 287u;  // 1.12 em Q8 (256 × 1.12)
constexpr uint8_t  kMisfireDebounceCycles = 3u;    // ciclos consecutivos para confirmar
constexpr uint8_t  kMisfireFaultThreshold = 5u;    // eventos em período de 100ms → DTC

void misfire_init() noexcept;
void misfire_reset() noexcept;

// Retorna número de eventos de misfire acumulados desde o último clear.
// Seguro para leitura fora do ISR (uint8_t = atômico em ARM).
uint8_t misfire_get_event_count(uint8_t cyl) noexcept;
void    misfire_clear_events(uint8_t cyl) noexcept;

}  // namespace ems::engine

// Hook chamado no ISR do CKP (ems::drv namespace, igual aos outros hooks).
// Implementação forte em misfire_detect.cpp substitui o símbolo fraco de ckp.cpp.
namespace ems::drv {
void misfire_on_tooth(const CkpSnapshot& snap) noexcept;
}  // namespace ems::drv
