#pragma once
/**
 * @file engine/vehicle_inputs.h
 * Vehicle signals for ENGINE consumers (hygiene PR-11).
 *
 * Implemented in APP (can_rx_map bridge). ENGINE must not include app headers.
 * Timeout semantics match can_rx_* (false if unconfigured id=0 or timed out).
 */
#include <cstdint>

namespace ems::engine {

bool vehicle_gear(uint8_t& out_gear, uint32_t now_ms) noexcept;
bool vehicle_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept;
bool vehicle_wheel_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept;

}  // namespace ems::engine
