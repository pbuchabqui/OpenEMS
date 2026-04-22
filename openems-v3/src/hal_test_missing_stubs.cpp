// Minimal stubs for host-test missing implementations
// Only defines functions that are truly missing

#ifdef EMS_HOST_TEST

#include <cstdint>
#include "hal/adc.h"

namespace ems::hal {

// These functions are referenced by tests but have no implementations
// Provide simple no-op stubs

__attribute__((weak))
void adc_pdb_on_tooth(uint16_t crank_angle_decideg) noexcept {}

__attribute__((weak))
uint16_t adc0_read(Adc0Channel ch) noexcept { return 512u; }

__attribute__((weak))
uint16_t adc1_read(Adc1Channel ch) noexcept { return 512u; }

__attribute__((weak))
void ftm0_set_oc(uint8_t channel, uint16_t ticks) noexcept {}

__attribute__((weak))
void ftm0_enable_interrupt() noexcept {}

__attribute__((weak))
uint32_t ftm0_get_counter() noexcept { return 0u; }

__attribute__((weak))
uint32_t ftm3_get_counter() noexcept { return 0u; }

__attribute__((weak))
uint32_t micros() noexcept { return 0u; }

__attribute__((weak))
uint32_t millis() noexcept { return 0u; }

__attribute__((weak))
void pit1_kick() noexcept {}

__attribute__((weak))
void uart_send_bytes(const uint8_t* data, uint16_t len) noexcept {}

__attribute__((weak))
bool can_rx_available() noexcept { return false; }

__attribute__((weak))
void can_rx_get_frame(uint16_t& id, uint8_t* data, uint8_t& dlc) noexcept {}

__attribute__((weak))
void can_tx_send_frame(uint16_t id, const uint8_t* data, uint8_t dlc) noexcept {}

} // namespace ems::hal

#endif
