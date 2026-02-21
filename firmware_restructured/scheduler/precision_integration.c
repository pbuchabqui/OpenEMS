/**
 * @file precision_integration.c
 * @brief Implementação do sistema de integração de precisão adaptativa
 */

#include "precision_integration.h"
#include "../utils/logger.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

static const char* TAG = "PRECISION_INTEGRATION";

//=============================================================================
// ESTADO GLOBAL
//=============================================================================

static precision_integration_state_t g_integration_state = {0};
static precision_integration_config_t g_integration_config = {0};
static precision_system_metrics_t g_system_metrics = {0};
static bool g_initialized = false;

//=============================================================================
// IMPLEMENTAÇÃO DA API GERAL
//=============================================================================

bool precision_integration_init(const precision_integration_config_t* config) {
    if (g_initialized) {
        return true;
    }

    // Inicializar subsistemas
    if (!precision_manager_init()) {
        LOG_SYSTEM_E("Failed to initialize precision manager");
        return false;
    }

    if (!adaptive_timer_init()) {
        LOG_SYSTEM_E("Failed to initialize adaptive timer");
        return false;
    }

    // Configurar integração
    if (config) {
        g_integration_config = *config;
    } else {
        // Configuração padrão
        g_integration_config.enable_precision_manager = true;
        g_integration_config.enable_adaptive_timer = true;
        g_integration_config.enable_automatic_updates = true;
        g_integration_config.enable_validation = true;
        g_integration_config.enable_statistics = true;
        g_integration_config.update_interval_ms = 100;  // 100ms
        g_integration_config.validation_tolerance = 0.1f; // 10%
    }

    // Inicializar estado
    g_integration_state.precision_config = precision_get_config();
    g_integration_state.precision_stats = precision_get_stats();
    g_integration_state.timer_config = adaptive_timer_get_config();
    g_integration_state.timer_stats = adaptive_timer_get_stats();
    
    g_integration_state.current_rpm = 0;
    g_integration_state.current_precision_tier = 3; // Normal
    g_integration_state.current_timer_resolution = 1000000U; // 1MHz
    g_integration_state.current_angular_tolerance = 0.8f;
    g_integration_state.current_injection_tolerance = 0.8f;
    g_integration_state.current_precision_us = 1.0f;
    
    g_integration_state.total_precision_gain = 1.0f;
    g_integration_state.total_jitter_reduction = 0.0f;
    g_integration_state.total_transitions = 0;
    
    g_integration_state.integration_enabled = true;
    g_integration_state.legacy_mode = false;
    g_integration_state.last_update_time = 0;

    // Resetar métricas
    memset(&g_system_metrics, 0, sizeof(g_system_metrics));

    g_initialized = true;

    LOG_SYSTEM_I("Precision integration initialized");
    LOG_SYSTEM_I("  Precision manager: %s", 
             g_integration_config.enable_precision_manager ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Adaptive timer: %s", 
             g_integration_config.enable_adaptive_timer ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Automatic updates: %s", 
             g_integration_config.enable_automatic_updates ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Validation: %s", 
             g_integration_config.enable_validation ? "enabled" : "disabled");
    
    return true;
}

precision_integration_state_t* precision_integration_get_state(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_integration_state;
}

precision_system_metrics_t* precision_integration_get_metrics(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_system_metrics;
}

bool precision_integration_update(uint16_t rpm, uint32_t timestamp_us) {
    if (!g_initialized || !g_integration_state.integration_enabled) {
        return false;
    }

    bool state_changed = false;

    // Atualizar precision manager
    if (g_integration_config.enable_precision_manager) {
        bool tier_changed = precision_update_tier(rpm);
        if (tier_changed) {
            state_changed = true;
            g_integration_state.total_transitions++;
        }
    }

    // Atualizar adaptive timer
    if (g_integration_config.enable_adaptive_timer) {
        bool timer_changed = adaptive_timer_update_tier(rpm);
        if (timer_changed) {
            state_changed = true;
            g_integration_state.total_transitions++;
        }
    }

    // Atualizar estado integrado
    g_integration_state.current_rpm = rpm;
    g_integration_state.last_update_time = timestamp_us;

    if (g_integration_config.enable_precision_manager && g_integration_state.precision_config) {
        g_integration_state.current_angular_tolerance = 
            precision_get_angular_tolerance(rpm);
        g_integration_state.current_injection_tolerance = 
            precision_get_injection_tolerance(rpm);
        g_integration_state.current_precision_tier = 
            precision_get_tier_for_rpm(rpm);
    }

    if (g_integration_config.enable_adaptive_timer && g_integration_state.timer_config) {
        g_integration_state.current_timer_resolution = 
            adaptive_timer_get_resolution(rpm);
        g_integration_state.current_precision_us = 
            adaptive_timer_get_precision_us(rpm);
    }

    // Calcular métricas combinadas
    float angular_gain = 1.0f;
    if (g_integration_state.current_angular_tolerance > 0) {
        angular_gain = 0.4f / g_integration_state.current_angular_tolerance; // vs 0.4° base
    }

    float temporal_gain = 1.0f;
    if (g_integration_state.current_precision_us > 0) {
        temporal_gain = 1.0f / g_integration_state.current_precision_us; // vs 1.0µs base
    }

    g_integration_state.total_precision_gain = angular_gain * temporal_gain;

    // Calcular redução de jitter
    float base_jitter = 20.0f; // 20µs base
    float current_jitter = base_jitter / g_integration_state.total_precision_gain;
    g_integration_state.total_jitter_reduction = 
        ((base_jitter - current_jitter) / base_jitter) * 100.0f;

    // Atualizar métricas do sistema
    if (g_integration_config.enable_statistics) {
        g_system_metrics.angular_precision_deg = g_integration_state.current_angular_tolerance;
        g_system_metrics.angular_tolerance_deg = g_integration_state.current_angular_tolerance;
        g_system_metrics.angular_gain_factor = angular_gain;
        
        g_system_metrics.temporal_precision_us = g_integration_state.current_precision_us;
        g_system_metrics.temporal_resolution_hz = g_integration_state.current_timer_resolution;
        g_system_metrics.temporal_gain_factor = temporal_gain;
        
        g_system_metrics.jitter_reduction_percent = g_integration_state.total_jitter_reduction;
        g_system_metrics.transition_count = g_integration_state.total_transitions;
        
        g_system_metrics.measurements_count++;
        
        // Média móvel de RPM
        if (g_system_metrics.measurements_count == 1) {
            g_system_metrics.average_rpm = (float)rpm;
        } else {
            float alpha = 0.01f;
            g_system_metrics.average_rpm = (alpha * (float)rpm) + 
                                        ((1.0f - alpha) * g_system_metrics.average_rpm);
        }
    }

    // Validação cruzada
    if (g_integration_config.enable_validation && state_changed) {
        bool validation_passed = true;
        
        // Validar consistência entre angular e temporal
        float angular_time_us = g_integration_state.current_angular_tolerance * 
                              (60000000.0f / (rpm * 360.0f)); // graus para microssegundos
        
        if (g_integration_state.current_precision_us > 0) {
            float ratio = angular_time_us / g_integration_state.current_precision_us;
            if (ratio < 0.5f || ratio > 2.0f) { // Fora da faixa razoável
                validation_passed = false;
                g_system_metrics.validation_failures++;
            }
        }
        
        g_system_metrics.validation_success_rate = 
            ((float)(g_system_metrics.measurements_count - g_system_metrics.validation_failures) / 
             (float)g_system_metrics.measurements_count) * 100.0f;
    }

    return state_changed;
}

void precision_integration_set_enabled(bool enabled) {
    if (!g_initialized) {
        return;
    }
    
    g_integration_state.integration_enabled = enabled;
    
    if (enabled) {
        precision_set_adaptive_mode(true);
        adaptive_timer_set_enabled(true);
    } else {
        precision_set_adaptive_mode(false);
        adaptive_timer_set_enabled(false);
    }
    
    LOG_DEBUG_I("Precision integration %s", enabled ? "enabled" : "disabled");
}

bool precision_integration_is_enabled(void) {
    return g_initialized ? g_integration_state.integration_enabled : false;
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE CONSULTA UNIFICADA
//=============================================================================

float precision_integration_get_angular_tolerance(uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_precision_manager) {
        return 0.8f; // Default legacy
    }
    return precision_get_angular_tolerance(rpm);
}

float precision_integration_get_injection_tolerance(uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_precision_manager) {
        return 0.8f; // Default legacy
    }
    return precision_get_injection_tolerance(rpm);
}

uint32_t precision_integration_get_timer_resolution(uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_adaptive_timer) {
        return 1000000U; // Default 1MHz
    }
    return adaptive_timer_get_resolution(rpm);
}

float precision_integration_get_temporal_precision(uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_adaptive_timer) {
        return 1.0f; // Default 1µs
    }
    return adaptive_timer_get_precision_us(rpm);
}

float precision_integration_get_total_gain(uint16_t rpm) {
    if (!g_initialized) {
        return 1.0f;
    }
    
    // Forçar atualização para o RPM específico
    precision_integration_update(rpm, 0);
    return g_integration_state.total_precision_gain;
}

float precision_integration_get_jitter_reduction(uint16_t rpm) {
    if (!g_initialized) {
        return 0.0f;
    }
    
    // Forçar atualização para o RPM específico
    precision_integration_update(rpm, 0);
    return g_integration_state.total_jitter_reduction;
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE CONFIGURAÇÃO
//=============================================================================

void precision_integration_set_legacy_mode(bool legacy_mode) {
    if (!g_initialized) {
        return;
    }
    
    g_integration_state.legacy_mode = legacy_mode;
    
    if (legacy_mode) {
        precision_set_adaptive_mode(false);
        adaptive_timer_set_enabled(false);
    } else {
        precision_set_adaptive_mode(true);
        adaptive_timer_set_enabled(true);
    }
    
    LOG_DEBUG_I("Legacy mode %s", legacy_mode ? "enabled" : "disabled");
}

void precision_integration_set_update_interval(uint32_t interval_ms) {
    if (!g_initialized) {
        return;
    }
    
    g_integration_config.update_interval_ms = interval_ms;
    LOG_DEBUG_D("Update interval set to %lu ms", (unsigned long)interval_ms);
}

void precision_integration_set_validation_tolerance(float tolerance) {
    if (!g_initialized) {
        return;
    }
    
    g_integration_config.validation_tolerance = tolerance;
    LOG_DEBUG_D("Validation tolerance set to %.2f", tolerance);
}

void precision_integration_reset_stats(void) {
    if (!g_initialized) {
        return;
    }
    
    precision_reset_stats();
    adaptive_timer_reset_stats();
    memset(&g_system_metrics, 0, sizeof(g_system_metrics));
    
    g_integration_state.total_transitions = 0;
    g_integration_state.last_update_time = 0;
    
    LOG_SYSTEM_I("Precision integration statistics reset");
}

bool precision_integration_recalculate(void) {
    if (!g_initialized) {
        return false;
    }
    
    uint16_t current_rpm = g_integration_state.current_rpm;
    return precision_integration_update(current_rpm, g_integration_state.last_update_time);
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE VALIDAÇÃO
//=============================================================================

bool precision_integration_validate_angular(float expected_angle, float actual_angle, uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_validation) {
        return true; // Skip validation if disabled
    }
    
    float tolerance = precision_integration_get_angular_tolerance(rpm);
    float error = fabsf(actual_angle - expected_angle);
    
    bool passed = (error <= tolerance);
    
    if (!passed) {
        g_system_metrics.validation_failures++;
        LOG_DEBUG_W("Angular validation failed: expected=%.2f, actual=%.2f, tolerance=%.2f, error=%.2f", 
                 expected_angle, actual_angle, tolerance, error);
    }
    
    return passed;
}

bool precision_integration_validate_temporal(uint32_t expected_time, uint32_t actual_time, uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_validation) {
        return true; // Skip validation if disabled
    }
    
    float precision_us = precision_integration_get_temporal_precision(rpm);
    float tolerance = precision_us * 10.0f; // 10x tolerance
    float error = fabsf((float)actual_time - (float)expected_time);
    
    bool passed = (error <= tolerance);
    
    if (!passed) {
        g_system_metrics.validation_failures++;
        LOG_DEBUG_W("Temporal validation failed: expected=%lu, actual=%lu, tolerance=%.1f, error=%.1f", 
                 (unsigned long)expected_time, (unsigned long)actual_time, tolerance, error);
    }
    
    return passed;
}

bool precision_integration_validate_injection(uint32_t expected_pulse, uint32_t actual_pulse, uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_validation) {
        return true; // Skip validation if disabled
    }
    
    float tolerance_percent = precision_integration_get_injection_tolerance(rpm);
    float error_percent = fabsf(((float)actual_pulse - (float)expected_pulse) / (float)expected_pulse * 100.0f);
    
    bool passed = (error_percent <= tolerance_percent);
    
    if (!passed) {
        g_system_metrics.validation_failures++;
        LOG_DEBUG_W("Injection validation failed: expected=%lu, actual=%lu, tolerance=%.1f%%, error=%.1f%%", 
                 (unsigned long)expected_pulse, (unsigned long)actual_pulse, tolerance_percent, error_percent);
    }
    
    return passed;
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE ESTATÍSTICAS
//=============================================================================

precision_stats_t* precision_integration_get_angular_stats(uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_precision_manager) {
        return NULL;
    }
    
    // Forçar atualização para obter estatísticas corretas
    precision_integration_update(rpm, 0);
    return g_integration_state.precision_stats;
}

adaptive_timer_stats_t* precision_integration_get_temporal_stats(uint16_t rpm) {
    if (!g_initialized || !g_integration_config.enable_adaptive_timer) {
        return NULL;
    }
    
    // Forçar atualização para obter estatísticas corretas
    precision_integration_update(rpm, 0);
    return g_integration_state.timer_stats;
}

precision_system_metrics_t* precision_integration_get_system_metrics(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_system_metrics;
}

float precision_integration_calculate_overhead(void) {
    if (!g_initialized) {
        return 0.0f;
    }
    
    // Estimar overhead baseado no número de operações
    float angular_overhead = g_integration_config.enable_precision_manager ? 1.0f : 0.0f;
    float temporal_overhead = g_integration_config.enable_adaptive_timer ? 1.5f : 0.0f;
    float validation_overhead = g_integration_config.enable_validation ? 0.5f : 0.0f;
    float statistics_overhead = g_integration_config.enable_statistics ? 0.5f : 0.0f;
    
    return angular_overhead + temporal_overhead + validation_overhead + statistics_overhead;
}

//=============================================================================
// IMPLEMENTAÇÃO DE UTILITÁRIOS
//=============================================================================

const char* precision_integration_tier_to_string(uint8_t tier) {
    if (tier >= 4) {
        return "Unknown";
    }
    
    static const char* tier_names[] = {
        "Ultra-High (0-1000 RPM)",
        "High (1000-2500 RPM)",
        "Medium (2500-4500 RPM)",
        "Normal (4500+ RPM)"
    };
    
    return tier_names[tier];
}

void precision_integration_print_config(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Precision integration not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Precision Integration Configuration ===");
    LOG_SYSTEM_I("Integration enabled: %s", 
             g_integration_state.integration_enabled ? "yes" : "no");
    LOG_SYSTEM_I("Legacy mode: %s", 
             g_integration_state.legacy_mode ? "yes" : "no");
    LOG_SYSTEM_I("Update interval: %lu ms", 
             (unsigned long)g_integration_config.update_interval_ms);
    LOG_SYSTEM_I("Validation tolerance: %.2f", 
             g_integration_config.validation_tolerance);
    
    LOG_SYSTEM_I("Subsystems:");
    LOG_SYSTEM_I("  Precision manager: %s", 
             g_integration_config.enable_precision_manager ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Adaptive timer: %s", 
             g_integration_config.enable_adaptive_timer ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Automatic updates: %s", 
             g_integration_config.enable_automatic_updates ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Validation: %s", 
             g_integration_config.enable_validation ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Statistics: %s", 
             g_integration_config.enable_statistics ? "enabled" : "disabled");
}

void precision_integration_print_state(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Precision integration not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Precision Integration State ===");
    LOG_SYSTEM_I("Current RPM: %d", g_integration_state.current_rpm);
    LOG_SYSTEM_I("Current tier: %d (%s)", 
             g_integration_state.current_precision_tier,
             precision_integration_tier_to_string(g_integration_state.current_precision_tier));
    LOG_SYSTEM_I("Timer resolution: %lu Hz (%.1fµs)", 
             (unsigned long)g_integration_state.current_timer_resolution,
             g_integration_state.current_precision_us);
    LOG_SYSTEM_I("Angular tolerance: %.2f°", g_integration_state.current_angular_tolerance);
    LOG_SYSTEM_I("Injection tolerance: %.2f%%", g_integration_state.current_injection_tolerance);
    
    LOG_SYSTEM_I("Combined metrics:");
    LOG_SYSTEM_I("  Total precision gain: %.1fx", g_integration_state.total_precision_gain);
    LOG_SYSTEM_I("  Jitter reduction: %.1f%%", g_integration_state.total_jitter_reduction);
    LOG_SYSTEM_I("  Total transitions: %lu", (unsigned long)g_integration_state.total_transitions);
}

void precision_integration_print_metrics(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Precision integration not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Precision System Metrics ===");
    LOG_SYSTEM_I("Angular precision:");
    LOG_SYSTEM_I("  Precision: %.2f°", g_system_metrics.angular_precision_deg);
    LOG_SYSTEM_I("  Tolerance: %.2f°", g_system_metrics.angular_tolerance_deg);
    LOG_SYSTEM_I("  Gain factor: %.1fx", g_system_metrics.angular_gain_factor);
    
    LOG_SYSTEM_I("Temporal precision:");
    LOG_SYSTEM_I("  Precision: %.1fµs", g_system_metrics.temporal_precision_us);
    LOG_SYSTEM_I("  Resolution: %lu Hz", (unsigned long)g_system_metrics.temporal_resolution_hz);
    LOG_SYSTEM_I("  Gain factor: %.1fx", g_system_metrics.temporal_gain_factor);
    
    LOG_SYSTEM_I("Performance:");
    LOG_SYSTEM_I("  Jitter reduction: %.1f%%", g_system_metrics.jitter_reduction_percent);
    LOG_SYSTEM_I("  System overhead: %.1f%%", precision_integration_calculate_overhead());
    LOG_SYSTEM_I("  Transitions: %lu", (unsigned long)g_system_metrics.transition_count);
    
    LOG_SYSTEM_I("Validation:");
    LOG_SYSTEM_I("  Failures: %lu", (unsigned long)g_system_metrics.validation_failures);
    LOG_SYSTEM_I("  Success rate: %.1f%%", g_system_metrics.validation_success_rate);
    
    LOG_SYSTEM_I("System:");
    LOG_SYSTEM_I("  Measurements: %lu", (unsigned long)g_system_metrics.measurements_count);
    LOG_SYSTEM_I("  Average RPM: %.1f", g_system_metrics.average_rpm);
}

void precision_integration_print_summary(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Precision integration not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Precision Integration Summary ===");
    LOG_SYSTEM_I("Status: %s", g_integration_state.integration_enabled ? "ACTIVE" : "INACTIVE");
    LOG_SYSTEM_I("Mode: %s", g_integration_state.legacy_mode ? "LEGACY" : "ADAPTIVE");
    LOG_SYSTEM_I("Current RPM: %d", g_integration_state.current_rpm);
    
    LOG_SYSTEM_I("Precision gains:");
    LOG_SYSTEM_I("  Angular: %.1fx", g_system_metrics.angular_gain_factor);
    LOG_SYSTEM_I("  Temporal: %.1fx", g_system_metrics.temporal_gain_factor);
    LOG_SYSTEM_I("  Combined: %.1fx", g_integration_state.total_precision_gain);
    
    LOG_SYSTEM_I("Performance:");
    LOG_SYSTEM_I("  Jitter reduction: %.1f%%", g_system_metrics.jitter_reduction_percent);
    LOG_SYSTEM_I("  System overhead: %.1f%%", precision_integration_calculate_overhead());
    LOG_SYSTEM_I("  Validation success: %.1f%%", g_system_metrics.validation_success_rate);
    
    LOG_SYSTEM_I("Current tolerances:");
    LOG_SYSTEM_I("  Angular: ±%.2f°", g_integration_state.current_angular_tolerance);
    LOG_SYSTEM_I("  Injection: ±%.2f%%", g_integration_state.current_injection_tolerance);
    LOG_SYSTEM_I("  Temporal: %.1fµs", g_integration_state.current_precision_us);
}

size_t precision_integration_generate_report(char* buffer, size_t buffer_size) {
    if (!g_initialized || !buffer || buffer_size == 0) {
        return 0;
    }
    
    int written = snprintf(buffer, buffer_size,
        "=== OpenEMS Precision Integration Report ===\n"
        "Generated: %lu\n"
        "Status: %s\n"
        "Mode: %s\n"
        "Current RPM: %d\n\n"
        "Precision Gains:\n"
        "  Angular: %.1fx\n"
        "  Temporal: %.1fx\n"
        "  Combined: %.1fx\n\n"
        "Current Tolerances:\n"
        "  Angular: ±%.2f°\n"
        "  Injection: ±%.2f%%\n"
        "  Temporal: %.1fµs\n\n"
        "Performance:\n"
        "  Jitter Reduction: %.1f%%\n"
        "  System Overhead: %.1f%%\n"
        "  Validation Success: %.1f%%\n"
        "  Total Transitions: %lu\n"
        "  Measurements: %lu\n"
        "  Average RPM: %.1f\n",
        (unsigned long)g_integration_state.last_update_time,
        g_integration_state.integration_enabled ? "ACTIVE" : "INACTIVE",
        g_integration_state.legacy_mode ? "LEGACY" : "ADAPTIVE",
        g_integration_state.current_rpm,
        g_system_metrics.angular_gain_factor,
        g_system_metrics.temporal_gain_factor,
        g_integration_state.total_precision_gain,
        g_integration_state.current_angular_tolerance,
        g_integration_state.current_injection_tolerance,
        g_integration_state.current_precision_us,
        g_system_metrics.jitter_reduction_percent,
        precision_integration_calculate_overhead(),
        g_system_metrics.validation_success_rate,
        (unsigned long)g_system_metrics.transition_count,
        (unsigned long)g_system_metrics.measurements_count,
        g_system_metrics.average_rpm
    );
    
    return (written > 0 && (size_t)written < buffer_size) ? (size_t)written : 0;
}
