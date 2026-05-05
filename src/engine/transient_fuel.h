#pragma once

#include <cstdint>

namespace ems::engine {

void transient_fuel_reset() noexcept;

uint32_t transient_fuel_xtau_update(uint32_t fuel_pw_us,
                                    int16_t clt_x10,
                                    bool enabled) noexcept;

}  // namespace ems::engine
