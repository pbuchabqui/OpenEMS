#pragma once

#include <cstdint>

// CAN FD Advanced Filtering (Phase 4 Feature)
// Implements priority-based message filtering and routing for FDCAN1

namespace ems::app {

// CAN Filter types
enum class CanFilterPriority : uint8_t {
    LOW    = 0,
    NORMAL = 1,
    HIGH   = 2,
    CRITICAL = 3
};

// CAN message routing
struct CanMessageRoute {
    uint16_t frame_id;
    CanFilterPriority priority;
    bool enabled;
};

// Advanced filtering functions
void can_filters_init() noexcept;
void can_filters_add(const CanMessageRoute& route) noexcept;
void can_filters_remove(uint16_t frame_id) noexcept;
bool can_filters_should_accept(uint16_t frame_id, uint8_t dlc) noexcept;

// Priority-based routing (determines which handler processes message)
uint8_t can_filter_get_priority(uint16_t frame_id) noexcept;

} // namespace ems::app
