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


// Conversões puras (período/alto em µs medidos pela ISR). O .cpp do target só as chama;
// os testes de host exercitam estas mesmas funções.
inline uint8_t flex_ethanol_pct_from_period_us(uint32_t period_us) noexcept {
    if (period_us == 0u) { return 0u; }
    return flex_ethanol_pct_from_hz(1000000u / period_us);
}
// Duty 10–90 % → -40…125 °C (x10). Sem sinal ou sem tempo em alto → -40 °C.
inline int16_t flex_temp_x10_from_us(uint32_t period_us, uint32_t high_us) noexcept {
    if (period_us == 0u || high_us == 0u) { return -400; }
    const uint32_t duty_pct = (high_us * 100u) / period_us;
    if (duty_pct <= 10u) { return -400; }
    if (duty_pct >= 90u) { return 1250; }
    return static_cast<int16_t>(-400 + static_cast<int32_t>((duty_pct - 10u) * 16500u) / 800);
}
inline bool flex_signal_valid(uint32_t age_ms, uint32_t period_us, uint32_t timeout_ms) noexcept {
    if (age_ms >= timeout_ms || period_us == 0u) { return false; }
    return flex_fuel_freq_plausible(1000000u / period_us);
}

}  // namespace ems::hal
