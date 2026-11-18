#include "engine/diagnostic_manager.h"

#include "hal/critical_section.h"

#include <cstdint>
#include <cstring>

namespace ems::engine {

// Static storage for diagnostic events
static constexpr uint8_t kMaxEvents = 16;
static DiagnosticEvent g_events[kMaxEvents];
static uint8_t g_event_count = 0;
static uint32_t g_system_tick_ms = 0;
// Estado de recuperação, paralelo a g_events (mesmo índice): é deslocado junto com ele.
static RecoveryState g_recovery_states[kMaxEvents];

void DiagnosticManager::init() noexcept {
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
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
    for (uint8_t i = 0; i < kMaxEvents; ++i) { g_recovery_states[i] = RecoveryState::IDLE; }
}

bool DiagnosticManager::report_fault(DiagnosticCode code,
                                     FaultSeverity severity,
                                     uint16_t param1,
                                     uint16_t param2) noexcept {
    // A tabela é alterada pela ISR do dente (sensors.cpp: falhas de MAP/TPS/ADC) e pelo laço
    // principal (sobretemperatura, trims, ETB). Sem a guarda, uma ISR no meio de um shift
    // duplicava ou perdia eventos, inclusive um CRITICAL (corte preso ou corte perdido).
    ems::hal::CriticalSectionGuard guard;
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
            // Escalada: a mesma falha reportada com severidade maior sobe (ex.: ADC em
            // recuperação WARNING → recuperação falhou CRITICAL). Nunca rebaixa por reporte.
            if (static_cast<uint8_t>(severity) > static_cast<uint8_t>(g_events[i].severity)) {
                g_events[i].severity = severity;
            }
            return false;  // Not new
        }
    }
    
    // New fault - add to event list if space available
    if (g_event_count >= kMaxEvents) {
        // Descarta o evento não crítico mais antigo (com o estado de recuperação junto).
        // Tudo crítico: o novo evento não tem lugar.
        uint8_t victim = kMaxEvents;
        for (uint8_t i = 0; i < kMaxEvents; ++i) {
            if (g_events[i].severity != FaultSeverity::CRITICAL) { victim = i; break; }
        }
        if (victim == kMaxEvents) {
            return false;
        }
        for (uint8_t j = victim; j < kMaxEvents - 1; ++j) {
            g_events[j] = g_events[j + 1];
            g_recovery_states[j] = g_recovery_states[j + 1];
        }
        g_event_count = kMaxEvents - 1;
    }
    
    // Add new event
    g_recovery_states[g_event_count] = RecoveryState::IDLE;
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

bool DiagnosticManager::clear_fault_if_not_critical(DiagnosticCode code) noexcept {
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            if (g_events[i].severity == FaultSeverity::CRITICAL) { return false; }
            break;
        }
    }
    return clear_fault(code);  // guarda aninhada: CriticalSectionGuard é nest-safe
}

bool DiagnosticManager::clear_fault(DiagnosticCode code) noexcept {
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
    if (code == DiagnosticCode::NONE) {
        return false;
    }
    
    // Find and remove the fault
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            // Shift remaining events
            for (uint8_t j = i; j < g_event_count - 1; ++j) {
                g_events[j] = g_events[j + 1];
                g_recovery_states[j] = g_recovery_states[j + 1];
            }
            --g_event_count;
            // Clear the last slot
            g_events[g_event_count].code = DiagnosticCode::NONE;
            g_recovery_states[g_event_count] = RecoveryState::IDLE;
            return true;
        }
    }
    return false;
}

bool DiagnosticManager::is_fault_active(DiagnosticCode code) noexcept {
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
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
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
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

RecoveryState DiagnosticManager::update_recovery(DiagnosticCode code,
                                                 bool success) noexcept {
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
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
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
    for (uint8_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].code == code) {
            return g_recovery_states[i];
        }
    }
    return RecoveryState::IDLE;
}

void DiagnosticManager::record_freeze_frame(DiagnosticCode code,
                                           const uint16_t frame[4]) noexcept {
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
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

// Limiares da plausibilidade MAP×TPS×RPM (fonte única; antes eram números soltos).
namespace {
constexpr uint32_t kPlausMinRunningRpmX10 = 4000u;    // 400 RPM: abaixo disso é partida/cranking ou motor parado
constexpr uint32_t kPlausIdleMaxRpmX10    = 12000u;   // 1200 RPM: faixa de marcha lenta
constexpr uint16_t kPlausIdleMapMaxBarX1000 = 600u;   // 0,60 bar: MAP acima disso em marcha lenta é suspeito
constexpr uint16_t kPlausWotTpsMinPctX10  = 900u;     // 90 %
constexpr uint16_t kPlausWotMapMinBarX1000 = 700u;    // 0,70 bar
constexpr uint16_t kPlausClosedTpsMaxPctX10 = 50u;    // 5 %
constexpr uint16_t kPlausClosedMapMaxBarX1000 = 950u; // 0,95 bar
constexpr uint32_t kPlausHighRpmX10       = 60000u;   // 6000 RPM
constexpr uint16_t kPlausHighRpmMapMinBarX1000 = 300u; // 0,30 bar
}  // namespace

bool DiagnosticManager::check_sensor_plausibility(uint16_t map_bar_x1000,
                                                  uint16_t tps_pct_x10,
                                                  uint32_t rpm_x10) noexcept {
    // As regras que assumem motor FUNCIONANDO (vácuo em marcha lenta, MAP não atmosférico com a
    // borboleta fechada) só valem com o motor rodando. Com a chave ligada e motor parado (rpm == 0,
    // inclusive sem sync) ou em cranking, o MAP é atmosférico (~1 bar) e isso é normal — antes gerava
    // MAP_TPS_CORRELATION espúrio a cada amostra.
    const bool engine_running = rpm_x10 >= kPlausMinRunningRpmX10;

    // At idle (low RPM), MAP should be low (high vacuum)
    if (engine_running && rpm_x10 < kPlausIdleMaxRpmX10) {
        if (map_bar_x1000 > kPlausIdleMapMaxBarX1000) {
            // Could indicate vacuum leak or sensor fault
            return false;
        }
    }

    // At wide open throttle (TPS > 90%), MAP should be high
    if (tps_pct_x10 > kPlausWotTpsMinPctX10) {
        if (map_bar_x1000 < kPlausWotMapMinBarX1000) {
            // Could indicate boost leak or sensor fault
            return false;
        }
    }

    // At closed throttle (TPS < 5%), MAP should not be atmospheric (motor rodando)
    if (engine_running && tps_pct_x10 < kPlausClosedTpsMaxPctX10) {
        if (map_bar_x1000 > kPlausClosedMapMaxBarX1000) {
            // Could indicate stuck throttle or sensor fault
            return false;
        }
    }

    // High RPM with very low MAP is implausible (unless decelerating)
    if (rpm_x10 > kPlausHighRpmX10 && map_bar_x1000 < kPlausHighRpmMapMinBarX1000) {
        // Very unlikely under normal operation
        return false;
    }

    return true;
}

void DiagnosticManager::clear_all_faults() noexcept {
    ems::hal::CriticalSectionGuard guard;   // ver report_fault
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
