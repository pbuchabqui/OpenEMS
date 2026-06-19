#pragma once
#include <cstdint>

namespace ems::engine {

void ewg_control_init() noexcept;
void ewg_control_reset() noexcept;
int16_t ewg_control_update(uint16_t target_pct_x10, uint16_t measured_pct_x10) noexcept;
uint16_t ewg_read_position_pct_x10() noexcept;

}  // namespace ems::engine
