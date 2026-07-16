/**
 * @file app/vehicle_inputs_bridge.cpp
 * APP-side implementation of engine::vehicle_* → can_rx_map (PR-11).
 */
#include "engine/vehicle_inputs.h"
#include "app/can_rx_map.h"

namespace ems::engine {

bool vehicle_gear(uint8_t& out_gear, uint32_t now_ms) noexcept {
    return ems::app::can_rx_gear(out_gear, now_ms);
}

bool vehicle_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept {
    return ems::app::can_rx_speed_kmh(out_kmh, now_ms);
}

bool vehicle_wheel_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept {
    return ems::app::can_rx_wheel_speed_kmh(out_kmh, now_ms);
}

}  // namespace ems::engine
