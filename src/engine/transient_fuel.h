#pragma once

#include <cstdint>

namespace ems::engine {

// Zera o filme legado e o filme de produção (via xtau_wall_fuel_reset).
// Chamar em DFCO, fuel cut e qualquer path que descarte o modelo de parede.
void transient_fuel_reset() noexcept;

uint32_t transient_fuel_xtau_update(uint32_t fuel_pw_us,
                                    int16_t clt_x10,
                                    bool enabled) noexcept;

}  // namespace ems::engine
