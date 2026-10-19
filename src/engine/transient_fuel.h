#pragma once

#include <cstdint>

namespace ems::engine {

// Zera o filme de parede X-τ (xtau_wall_fuel_reset).
// Chamar em DFCO, fuel cut e qualquer path que descarte o modelo de parede.
// O modelo em si é transient_fuel_xtau_with_autocalib() em xtau_autocalib.h.
void transient_fuel_reset() noexcept;

}  // namespace ems::engine
