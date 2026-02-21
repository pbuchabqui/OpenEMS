/**
 * @file precision_manager.c
 * @brief Implementação do sistema de precisão adaptativa por RPM
 */

#include "precision_manager.h"
#include "../utils/logger.h"
#include <string.h>

static const char* TAG = "PRECISION_MGR";

//=============================================================================
// ESTADO GLOBAL
//=============================================================================

static precision_config_t g_precision_config = {0};
static precision_stats_t g_precision_stats = {0};
static bool g_initialized = false;
static uint16_t g_last_rpm = 0;

//=============================================================================
// IMPLEMENTAÇÃO DA API GERAL
//=============================================================================

bool precision_manager_init(void) {
    if (g_initialized) {
        return true;
    }

    // Inicializar configuração com valores padrão
    g_precision_config.rpm_thresholds[0] = RPM_TIER_1_MAX;
    g_precision_config.rpm_thresholds[1] = RPM_TIER_2_MAX;
    g_precision_config.rpm_thresholds[2] = RPM_TIER_3_MAX;
    g_precision_config.rpm_thresholds[3] = RPM_TIER_4_MAX;

    g_precision_config.timer_resolutions[0] = TIMER_RES_TIER_1;
    g_precision_config.timer_resolutions[1] = TIMER_RES_TIER_2;
    g_precision_config.timer_resolutions[2] = TIMER_RES_TIER_3;
    g_precision_config.timer_resolutions[3] = TIMER_RES_TIER_4;

    g_precision_config.angular_tolerances[0] = ANGULAR_TOL_TIER_1;
    g_precision_config.angular_tolerances[1] = ANGULAR_TOL_TIER_2;
    g_precision_config.angular_tolerances[2] = ANGULAR_TOL_TIER_3;
    g_precision_config.angular_tolerances[3] = ANGULAR_TOL_TIER_4;

    g_precision_config.injection_tolerances[0] = INJECTION_TOL_TIER_1;
    g_precision_config.injection_tolerances[1] = INJECTION_TOL_TIER_2;
    g_precision_config.injection_tolerances[2] = INJECTION_TOL_TIER_3;
    g_precision_config.injection_tolerances[3] = INJECTION_TOL_TIER_4;

    g_precision_config.current_tier = 0;
    g_precision_config.adaptive_enabled = true;

    // Resetar estatísticas
    memset(&g_precision_stats, 0, sizeof(g_precision_stats));

    g_initialized = true;
    g_last_rpm = 0;

    LOG_SYSTEM_I("Precision manager initialized");
    LOG_SYSTEM_I("  Adaptive mode: %s", g_precision_config.adaptive_enabled ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Tiers: %d", PRECISION_TIERS);
    
    return true;
}

precision_config_t* precision_get_config(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_precision_config;
}

precision_stats_t* precision_get_stats(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_precision_stats;
}

void precision_set_adaptive_mode(bool enabled) {
    if (!g_initialized) {
        return;
    }
    
    bool was_enabled = g_precision_config.adaptive_enabled;
    g_precision_config.adaptive_enabled = enabled;
    
    if (was_enabled != enabled) {
        LOG_DEBUG_I("Adaptive mode %s", enabled ? "enabled" : "disabled");
    }
}

bool precision_is_adaptive_enabled(void) {
    return g_initialized ? g_precision_config.adaptive_enabled : false;
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE CONSULTA
//=============================================================================

uint32_t precision_get_timer_resolution(uint16_t rpm) {
    if (!g_initialized || !g_precision_config.adaptive_enabled) {
        return TIMER_RES_TIER_4; // Default para modo legacy
    }
    
    uint8_t tier = precision_get_tier_for_rpm(rpm);
    return g_precision_config.timer_resolutions[tier];
}

float precision_get_angular_tolerance(uint16_t rpm) {
    if (!g_initialized || !g_precision_config.adaptive_enabled) {
        return ANGULAR_TOL_TIER_4; // Default para modo legacy
    }
    
    uint8_t tier = precision_get_tier_for_rpm(rpm);
    return g_precision_config.angular_tolerances[tier];
}

float precision_get_injection_tolerance(uint16_t rpm) {
    if (!g_initialized || !g_precision_config.adaptive_enabled) {
        return INJECTION_TOL_TIER_4; // Default para modo legacy
    }
    
    uint8_t tier = precision_get_tier_for_rpm(rpm);
    return g_precision_config.injection_tolerances[tier];
}

uint8_t precision_get_tier_for_rpm(uint16_t rpm) {
    // Encontrar faixa apropriada para o RPM
    for (uint8_t i = 0; i < PRECISION_TIERS; i++) {
        if (rpm <= g_precision_config.rpm_thresholds[i]) {
            return i;
        }
    }
    return PRECISION_TIERS - 1; // Última faixa
}

bool precision_check_tier_transition(uint16_t new_rpm) {
    if (!g_initialized || !g_precision_config.adaptive_enabled) {
        return false;
    }
    
    uint8_t current_tier = g_precision_config.current_tier;
    uint8_t new_tier = precision_get_tier_for_rpm(new_rpm);
    
    return (current_tier != new_tier);
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE CONFIGURAÇÃO
//=============================================================================

bool precision_update_tier(uint16_t rpm) {
    if (!g_initialized || !g_precision_config.adaptive_enabled) {
        return false;
    }
    
    uint8_t new_tier = precision_get_tier_for_rpm(rpm);
    uint8_t current_tier = g_precision_config.current_tier;
    
    if (current_tier != new_tier) {
        g_precision_config.current_tier = new_tier;
        g_precision_stats.tier_transitions++;
        
        LOG_DEBUG_D("Precision tier changed: %d -> %d (RPM: %d)", 
                 current_tier, new_tier, rpm);
        LOG_DEBUG_D("  Timer resolution: %lu Hz", 
                 g_precision_config.timer_resolutions[new_tier]);
        LOG_DEBUG_D("  Angular tolerance: %.2f°", 
                 g_precision_config.angular_tolerances[new_tier]);
        LOG_DEBUG_D("  Injection tolerance: %.2f%%", 
                 g_precision_config.injection_tolerances[new_tier]);
        
        return true;
    }
    
    return false;
}

void precision_reset_stats(void) {
    if (!g_initialized) {
        return;
    }
    
    memset(&g_precision_stats, 0, sizeof(g_precision_stats));
    LOG_SYSTEM_I("Precision statistics reset");
}

void precision_record_violation(float expected, float actual, float tolerance) {
    if (!g_initialized) {
        return;
    }
    
    float error = (actual > expected) ? (actual - expected) : (expected - actual);
    if (error > tolerance) {
        g_precision_stats.precision_violations++;
        LOG_DEBUG_W("Precision violation: expected=%.3f, actual=%.3f, tolerance=%.3f, error=%.3f", 
                 expected, actual, tolerance, error);
    }
}

void precision_record_jitter(float jitter_us) {
    if (!g_initialized) {
        return;
    }
    
    g_precision_stats.measurements_count++;
    
    // Atualizar jitter médio (média móvel simples)
    if (g_precision_stats.measurements_count == 1) {
        g_precision_stats.avg_jitter_us = jitter_us;
        g_precision_stats.max_jitter_us = jitter_us;
    } else {
        // Média móvel com peso decrescente
        float alpha = 0.1f;
        g_precision_stats.avg_jitter_us = (alpha * jitter_us) + 
                                        ((1.0f - alpha) * g_precision_stats.avg_jitter_us);
        
        if (jitter_us > g_precision_stats.max_jitter_us) {
            g_precision_stats.max_jitter_us = jitter_us;
        }
    }
}

//=============================================================================
// IMPLEMENTAÇÃO DE UTILITÁRIOS
//=============================================================================

const char* precision_tier_to_string(uint8_t tier) {
    if (tier >= PRECISION_TIERS) {
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

void precision_print_config(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Precision manager not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Precision Manager Configuration ===");
    LOG_SYSTEM_I("Adaptive mode: %s", 
             g_precision_config.adaptive_enabled ? "enabled" : "disabled");
    LOG_SYSTEM_I("Current tier: %d (%s)", 
             g_precision_config.current_tier, 
             precision_tier_to_string(g_precision_config.current_tier));
    
    LOG_SYSTEM_I("Tier thresholds and configurations:");
    for (uint8_t i = 0; i < PRECISION_TIERS; i++) {
        LOG_SYSTEM_I("  Tier %d: 0-%d RPM", i, g_precision_config.rpm_thresholds[i]);
        LOG_SYSTEM_I("    Timer: %lu Hz (%.1fµs)", 
                 g_precision_config.timer_resolutions[i],
                 1000000.0f / (float)g_precision_config.timer_resolutions[i]);
        LOG_SYSTEM_I("    Angular: ±%.2f°", g_precision_config.angular_tolerances[i]);
        LOG_SYSTEM_I("    Injection: ±%.2f%%", g_precision_config.injection_tolerances[i]);
    }
}

void precision_print_stats(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Precision manager not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Precision Manager Statistics ===");
    LOG_SYSTEM_I("Tier transitions: %lu", g_precision_stats.tier_transitions);
    LOG_SYSTEM_I("Precision violations: %lu", g_precision_stats.precision_violations);
    LOG_SYSTEM_I("Measurements: %lu", g_precision_stats.measurements_count);
    
    if (g_precision_stats.measurements_count > 0) {
        LOG_SYSTEM_I("Average jitter: %.2fµs", g_precision_stats.avg_jitter_us);
        LOG_SYSTEM_I("Maximum jitter: %.2fµs", g_precision_stats.max_jitter_us);
        
        // Calcular taxa de violações
        float violation_rate = (float)g_precision_stats.precision_violations / 
                             (float)g_precision_stats.measurements_count * 100.0f;
        LOG_SYSTEM_I("Violation rate: %.2f%%", violation_rate);
    }
}
