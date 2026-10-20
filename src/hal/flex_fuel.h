#pragma once
#include <cstdint>

namespace ems::hal {

void flex_fuel_init() noexcept;
void flex_fuel_edge_isr() noexcept;
uint8_t flex_fuel_ethanol_pct() noexcept;
int16_t flex_fuel_temp_x10() noexcept;
bool flex_fuel_valid() noexcept;

// 45–50 Hz → 0 %, 150–155 Hz → 100 %, 50–150 linear. Fora de 45–155 não é válido.
inline bool flex_fuel_freq_plausible(uint32_t freq_hz) noexcept {
    return freq_hz >= 45u && freq_hz <= 155u;
}
inline uint8_t flex_ethanol_pct_from_hz(uint32_t freq_hz) noexcept {
    if (freq_hz <= 50u) { return 0u; }
    if (freq_hz >= 150u) { return 100u; }
    return static_cast<uint8_t>(freq_hz - 50u);
}

}  // namespace ems::hal
