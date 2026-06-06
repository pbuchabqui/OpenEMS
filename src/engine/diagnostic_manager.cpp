#include "engine/diagnostic_manager.h"

#include <cstdint>
#include <cstring>

namespace ems::engine {

// Static storage for diagnostic events
static constexpr uint8_t kMaxEvents = 16;
static DiagnosticEvent g_events[kMaxEvents];
static uint8_t g_event_count = 0;
static uint32_t g_system_tick_ms = 0;

void DiagnosticManager::init() noexcept {
    // Clear all events
    for (uint8_t i = 0; i < kMaxEvents; ++i) {
        g_events[i].code = DiagnosticCode::NONE;
        g_events[i].severity = FaultSeverity::INFO;
        g_events[i].timestamp_ms = 0;
        g_events[i].occurrence_count = 0;
        for (uint8_t j = 0; j < 4; ++j) {
            g_events[i].freeze_frame[j] = 0;
        }
    }
    g_event_count = 0;
    g_system_tick_ms = 0;
}

bool DiagnosticManager::report_fault(DiagnosticCode code,
                                     FaultSeverity severity,
                                     uint16_t param1,
                                     uint16_t param2) noexcept {
    if (code == DiagnosticCode::NONE) {
        return false;
    }
    
    // Check if fault already exists
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            // Update occurrence count and freeze frame
            if (g_events[i].occurrence_count < 65535) {
                ++g_events[i].occurrence_count;
            }
            g_events[i].freeze_frame[0] = param1;
            g_events[i].freeze_frame[1] = param2;
            g_events[i].timestamp_ms = g_system_tick_ms;
            return false;  // Not new
        }
    }
    
    // New fault - add to event list if space available
    if (g_event_count >= kMaxEvents) {
        // Overwrite oldest non-critical event
        for (uint8_t i = 0; i < kMaxEvents - 1; ++i) {
            if (g_events[i].severity != FaultSeverity::CRITICAL) {
                // Shift events
                g_events[i] = g_events[i + 1];
            }
        }
        g_event_count = kMaxEvents - 1;
    }
    
    // Add new event
    DiagnosticEvent& event = g_events[g_event_count++];
    event.code = code;
    event.severity = severity;
    event.timestamp_ms = g_system_tick_ms;
    event.occurrence_count = 1;
    event.freeze_frame[0] = param1;
    event.freeze_frame[1] = param2;
    event.freeze_frame[2] = 0;
    event.freeze_frame[3] = 0;
    
    return true;  // New fault
}

bool DiagnosticManager::clear_fault(DiagnosticCode code) noexcept {
    if (code == DiagnosticCode::NONE) {
        return false;
    }
    
    // Find and remove the fault
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            // Shift remaining events
            for (uint8_t j = i; j < g_event_count - 1; ++j) {
                g_events[j] = g_events[j + 1];
            }
            --g_event_count;
            // Clear the last slot
            g_events[g_event_count].code = DiagnosticCode::NONE;
            return true;
        }
    }
    return false;
}

bool DiagnosticManager::is_fault_active(DiagnosticCode code) noexcept {
    if (code == DiagnosticCode::NONE) {
        return false;
    }
    
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            return true;
        }
    }
    return false;
}

uint8_t DiagnosticManager::get_active_fault_count() noexcept {
    return g_event_count;
}

FaultSeverity DiagnosticManager::get_highest_severity() noexcept {
    if (g_event_count == 0) {
        return FaultSeverity::INFO;
    }
    
    FaultSeverity highest = FaultSeverity::INFO;
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (static_cast<uint8_t>(g_events[i].severity) > static_cast<uint8_t>(highest)) {
            highest = g_events[i].severity;
        }
    }
    return highest;
}

// Recovery state machine storage
static RecoveryState g_recovery_states[16];

RecoveryState DiagnosticManager::update_recovery(DiagnosticCode code,
                                                 bool success) noexcept {
    // Find associated event
    uint8_t event_idx = 0;
    bool found = false;
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            event_idx = i;
            found = true;
            break;
        }
    }
    
    if (!found) {
        return RecoveryState::IDLE;
    }
    
    RecoveryState& state = g_recovery_states[event_idx];
    
    switch (state) {
        case RecoveryState::IDLE:
            if (!success) {
                state = RecoveryState::DETECTED;
            }
            break;
            
        case RecoveryState::DETECTED:
            if (success) {
                state = RecoveryState::RECOVERED;
            } else {
                state = RecoveryState::RECOVERING;
            }
            break;
            
        case RecoveryState::RECOVERING:
            if (success) {
                state = RecoveryState::RECOVERED;
            } else {
                state = RecoveryState::FAILED;
            }
            break;
            
        case RecoveryState::RECOVERED:
            state = RecoveryState::IDLE;
            break;
            
        case RecoveryState::FAILED:
            // Stay in failed state until manually reset
            break;
            
        case RecoveryState::PERMANENT:
            // Permanent faults don't recover
            break;
    }
    
    return state;
}

RecoveryState DiagnosticManager::get_recovery_state(DiagnosticCode code) noexcept {
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            return g_recovery_states[i];
        }
    }
    return RecoveryState::IDLE;
}

void DiagnosticManager::record_freeze_frame(DiagnosticCode code,
                                           const uint16_t frame[4]) noexcept {
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            for (uint8_t j = 0; j < 4; ++j) {
                g_events[i].freeze_frame[j] = frame[j];
            }
            break;
        }
    }
}

const DiagnosticEvent* DiagnosticManager::get_event(DiagnosticCode code) noexcept {
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            return &g_events[i];
        }
    }
    return nullptr;
}

bool DiagnosticManager::check_sensor_plausibility(uint16_t map_bar_x1000,
                                                  uint16_t tps_pct_x10,
                                                  uint32_t rpm_x10) noexcept {
    // Plausibility checks based on physical constraints
    
    // At idle (low RPM), MAP should be low (high vacuum)
    if (rpm_x10 < 12000) {  // Below 1200 RPM
        if (map_bar_x1000 > 600) {  // MAP > 0.60 bar at idle is suspicious
            // Could indicate vacuum leak or sensor fault
            return false;
        }
    }
    
    // At wide open throttle (TPS > 90%), MAP should be high
    if (tps_pct_x10 > 900) {  // TPS > 90%
        if (map_bar_x1000 < 700) {  // MAP < 0.70 bar at WOT is suspicious
            // Could indicate boost leak or sensor fault
            return false;
        }
    }
    
    // At closed throttle (TPS < 5%), MAP should not be atmospheric
    if (tps_pct_x10 < 50) {  // TPS < 5%
        if (map_bar_x1000 > 950) {  // MAP > 0.95 bar at closed throttle
            // Could indicate stuck throttle or sensor fault
            return false;
        }
    }
    
    // High RPM with very low MAP is implausible (unless decelerating)
    if (rpm_x10 > 60000 && map_bar_x1000 < 300) {  // RPM > 6000, MAP < 0.30 bar
        // Very unlikely under normal operation
        return false;
    }
    
    return true;
}

void DiagnosticManager::clear_all_faults() noexcept {
    // Clear only non-permanent faults
    uint8_t write_idx = 0;
    for (uint8_t i = 0; i < g_event_count; ++i) {
        // Keep permanent faults
        if (g_recovery_states[i] == RecoveryState::PERMANENT) {
            if (write_idx != i) {
                g_events[write_idx] = g_events[i];
                g_recovery_states[write_idx] = g_recovery_states[i];
            }
            ++write_idx;
        }
    }
    
    // Fix count first, then clear remaining slots
    const uint8_t old_count = g_event_count;
    g_event_count = write_idx;
    for (uint8_t i = write_idx; i < old_count; ++i) {
        g_events[i].code = DiagnosticCode::NONE;
        g_recovery_states[i] = RecoveryState::IDLE;
    }
}

bool DiagnosticManager::is_system_ready() noexcept {
    FaultSeverity highest = get_highest_severity();
    return highest != FaultSeverity::CRITICAL;
}

// Tick function to update system time (call from main loop)
void diagnostic_tick(uint32_t elapsed_ms) noexcept {
    g_system_tick_ms += elapsed_ms;
}

}  // namespace ems::engine
