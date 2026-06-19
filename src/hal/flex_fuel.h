#pragma once
#include <cstdint>

namespace ems::hal {

void flex_fuel_init() noexcept;
void flex_fuel_edge_isr() noexcept;
uint8_t flex_fuel_ethanol_pct() noexcept;
int16_t flex_fuel_temp_x10() noexcept;
bool flex_fuel_valid() noexcept;

}  // namespace ems::hal
