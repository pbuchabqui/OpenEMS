/**
 * @file adaptive_timer.c
 * @brief Implementação do sistema de timer resolution adaptativa por RPM
 */

#include "adaptive_timer.h"
#include "../utils/logger.h"
#include <string.h>
#include <math.h>

static const char* TAG = "ADAPTIVE_TIMER";

//=============================================================================
// ESTADO GLOBAL
//=============================================================================

static adaptive_timer_config_t g_timer_config = {0};
static adaptive_timer_stats_t g_timer_stats = {0};
static timer_validation_t g_timer_validation = {0};
static bool g_initialized = false;

//=============================================================================
// IMPLEMENTAÇÃO DA API GERAL
//=============================================================================

bool adaptive_timer_init(void) {
    if (g_initialized) {
        return true;
    }

    // Inicializar configuração com valores padrão
    g_timer_config.rpm_thresholds[0] = TIMER_TIER_1_MAX;
    g_timer_config.rpm_thresholds[1] = TIMER_TIER_2_MAX;
    g_timer_config.rpm_thresholds[2] = TIMER_TIER_3_MAX;
    g_timer_config.rpm_thresholds[3] = TIMER_TIER_4_MAX;

    g_timer_config.timer_resolutions[0] = TIMER_RES_TIER_1;
    g_timer_config.timer_resolutions[1] = TIMER_RES_TIER_2;
    g_timer_config.timer_resolutions[2] = TIMER_RES_TIER_3;
    g_timer_config.timer_resolutions[3] = TIMER_RES_TIER_4;

    g_timer_config.current_resolution = TIMER_RES_TIER_4; // Default 1MHz
    g_timer_config.current_tier = 3; // Faixa normal
    g_timer_config.last_rpm = 0;
    g_timer_config.transition_count = 0;
    g_timer_config.adaptive_enabled = true;

    // Resetar estatísticas
    memset(&g_timer_stats, 0, sizeof(g_timer_stats));
    memset(&g_timer_validation, 0, sizeof(g_timer_validation));

    g_initialized = true;

    LOG_SYSTEM_I("Adaptive timer initialized");
    LOG_SYSTEM_I("  Adaptive mode: %s", g_timer_config.adaptive_enabled ? "enabled" : "disabled");
    LOG_SYSTEM_I("  Tiers: %d", TIMER_TIERS);
    LOG_SYSTEM_I("  Resolution range: %.1fMHz - %.1fMHz", 
             TIMER_RES_TIER_1 / 1000000.0f, TIMER_RES_TIER_4 / 1000000.0f);
    
    return true;
}

adaptive_timer_config_t* adaptive_timer_get_config(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_timer_config;
}

adaptive_timer_stats_t* adaptive_timer_get_stats(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_timer_stats;
}

void adaptive_timer_set_enabled(bool enabled) {
    if (!g_initialized) {
        return;
    }
    
    bool was_enabled = g_timer_config.adaptive_enabled;
    g_timer_config.adaptive_enabled = enabled;
    
    if (was_enabled != enabled) {
        LOG_DEBUG_I("Adaptive timer %s", enabled ? "enabled" : "disabled");
    }
}

bool adaptive_timer_is_enabled(void) {
    return g_initialized ? g_timer_config.adaptive_enabled : false;
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE RESOLUÇÃO
//=============================================================================

uint32_t adaptive_timer_get_resolution(uint16_t rpm) {
    if (!g_initialized || !g_timer_config.adaptive_enabled) {
        return TIMER_RES_TIER_4; // Default para modo legacy
    }
    
    uint8_t tier = adaptive_timer_get_tier_for_rpm(rpm);
    return g_timer_config.timer_resolutions[tier];
}

uint32_t adaptive_timer_get_period_ns(uint16_t rpm) {
    uint32_t resolution_hz = adaptive_timer_get_resolution(rpm);
    if (resolution_hz == 0) return 0;
    
    return 1000000000UL / resolution_hz; // Nanossegundos por tick
}

float adaptive_timer_get_precision_us(uint16_t rpm) {
    uint32_t resolution_hz = adaptive_timer_get_resolution(rpm);
    if (resolution_hz == 0) return 0.0f;
    
    return 1000000.0f / (float)resolution_hz; // Microssegundos por tick
}

uint8_t adaptive_timer_get_tier_for_rpm(uint16_t rpm) {
    // Encontrar faixa apropriada para o RPM
    for (uint8_t i = 0; i < TIMER_TIERS; i++) {
        if (rpm <= g_timer_config.rpm_thresholds[i]) {
            return i;
        }
    }
    return TIMER_TIERS - 1; // Última faixa
}

bool adaptive_timer_check_transition(uint16_t new_rpm) {
    if (!g_initialized || !g_timer_config.adaptive_enabled) {
        return false;
    }
    
    uint8_t current_tier = g_timer_config.current_tier;
    uint8_t new_tier = adaptive_timer_get_tier_for_rpm(new_rpm);
    
    return (current_tier != new_tier);
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE CONFIGURAÇÃO
//=============================================================================

bool adaptive_timer_update_tier(uint16_t rpm) {
    if (!g_initialized || !g_timer_config.adaptive_enabled) {
        return false;
    }
    
    uint8_t new_tier = adaptive_timer_get_tier_for_rpm(rpm);
    uint8_t current_tier = g_timer_config.current_tier;
    
    // Aplicar histerese para evitar transições rápidas
    if (current_tier != new_tier) {
        uint16_t hysteresis_rpm = TIMER_HYSTERESIS_RPM;
        
        // Verificar se a mudança é significativa
        if (rpm > g_timer_config.last_rpm) {
            // Aumentando RPM - verificar se cruzou o limiar + histerese
            if (current_tier < TIMER_TIERS - 1 && 
                rpm > (g_timer_config.rpm_thresholds[current_tier] + hysteresis_rpm)) {
                // Permitir transição para faixa superior
            } else if (current_tier > 0 && 
                       rpm < (g_timer_config.rpm_thresholds[current_tier - 1] - hysteresis_rpm)) {
                // Permitir transição para faixa inferior
            } else {
                return false; // Mudança não significativa
            }
        } else {
            // Diminuindo RPM - verificar se cruzou o limiar - histerese
            if (current_tier > 0 && 
                rpm < (g_timer_config.rpm_thresholds[current_tier - 1] - hysteresis_rpm)) {
                // Permitir transição para faixa inferior
            } else if (current_tier < TIMER_TIERS - 1 && 
                       rpm > (g_timer_config.rpm_thresholds[current_tier] + hysteresis_rpm)) {
                // Permitir transição para faixa superior
            } else {
                return false; // Mudança não significativa
            }
        }
    }
    
    if (current_tier != new_tier) {
        uint8_t old_tier = g_timer_config.current_tier;
        g_timer_config.current_tier = new_tier;
        g_timer_config.current_resolution = g_timer_config.timer_resolutions[new_tier];
        g_timer_config.last_rpm = rpm;
        g_timer_config.transition_count++;
        
        adaptive_timer_record_transition(old_tier, new_tier, rpm);
        
        LOG_DEBUG_D("Timer tier changed: %d -> %d (RPM: %d)", 
                 old_tier, new_tier, rpm);
        LOG_DEBUG_D("  Resolution: %lu Hz (%.1fµs)", 
                 g_timer_config.current_resolution,
                 adaptive_timer_get_precision_us(rpm));
        
        return true;
    }
    
    g_timer_config.last_rpm = rpm;
    return false;
}

bool adaptive_timer_set_resolution(uint32_t new_resolution) {
    if (!g_initialized) {
        return false;
    }
    
    // Validar resolução
    bool valid_resolution = false;
    for (uint8_t i = 0; i < TIMER_TIERS; i++) {
        if (g_timer_config.timer_resolutions[i] == new_resolution) {
            valid_resolution = true;
            break;
        }
    }
    
    if (!valid_resolution) {
        LOG_DEBUG_E("Invalid resolution: %lu Hz", new_resolution);
        return false;
    }
    
    uint32_t old_resolution = g_timer_config.current_resolution;
    g_timer_config.current_resolution = new_resolution;
    g_timer_stats.resolution_changes++;
    
    LOG_DEBUG_I("Timer resolution changed: %lu Hz -> %lu Hz", 
             old_resolution, new_resolution);
    
    return true;
}

void adaptive_timer_reset_stats(void) {
    if (!g_initialized) {
        return;
    }
    
    memset(&g_timer_stats, 0, sizeof(g_timer_stats));
    LOG_SYSTEM_I("Adaptive timer statistics reset");
}

void adaptive_timer_record_transition(uint8_t old_tier, uint8_t new_tier, uint16_t rpm) {
    if (!g_initialized) {
        return;
    }
    
    g_timer_stats.tier_transitions++;
    
    // Calcular ganho de precisão
    uint32_t old_resolution = g_timer_config.timer_resolutions[old_tier];
    uint32_t new_resolution = g_timer_config.timer_resolutions[new_tier];
    float gain = adaptive_timer_calculate_gain(new_resolution, old_resolution);
    
    if (gain > g_timer_stats.max_precision_gain) {
        g_timer_stats.max_precision_gain = gain;
    }
    
    // Atualizar resolução média
    g_timer_stats.measurements_count++;
    if (g_timer_stats.measurements_count == 1) {
        g_timer_stats.avg_resolution_hz = (float)new_resolution;
    } else {
        float alpha = 0.1f;
        g_timer_stats.avg_resolution_hz = (alpha * (float)new_resolution) + 
                                        ((1.0f - alpha) * g_timer_stats.avg_resolution_hz);
    }
}

//=============================================================================
// IMPLEMENTAÇÃO DA API DE VALIDAÇÃO
//=============================================================================

bool adaptive_timer_validate_timestamp(uint32_t timestamp_us, uint32_t expected_period_us) {
    if (!g_initialized || expected_period_us == 0) {
        return false;
    }
    
    g_timer_validation.last_timestamp_us = timestamp_us;
    g_timer_validation.expected_period_us = expected_period_us;
    
    // Calcular período medido (diferença do último timestamp)
    if (g_timer_validation.actual_period_us > 0) {
        uint32_t measured_period = timestamp_us - g_timer_validation.actual_period_us;
        g_timer_validation.actual_period_us = timestamp_us;
        
        // Calcular erro de validação
        float error = fabsf((float)measured_period - (float)expected_period_us);
        g_timer_validation.validation_error = error;
        
        // Validação passa se erro < 10% do período esperado
        float tolerance = (float)expected_period_us * 0.1f;
        g_timer_validation.validation_passed = (error <= tolerance);
        
        if (!g_timer_validation.validation_passed) {
            g_timer_stats.validation_failures++;
            LOG_DEBUG_W("Timer validation failed: expected=%lu, measured=%lu, error=%.1f", 
                     expected_period_us, measured_period, error);
        }
        
        return g_timer_validation.validation_passed;
    }
    
    g_timer_validation.actual_period_us = timestamp_us;
    g_timer_validation.validation_passed = true;
    return true;
}

timer_validation_t* adaptive_timer_get_validation(void) {
    if (!g_initialized) {
        return NULL;
    }
    return &g_timer_validation;
}

void adaptive_timer_reset_validation(void) {
    if (!g_initialized) {
        return;
    }
    
    memset(&g_timer_validation, 0, sizeof(g_timer_validation));
}

//=============================================================================
// IMPLEMENTAÇÃO DE UTILITÁRIOS
//=============================================================================

const char* adaptive_timer_tier_to_string(uint8_t tier) {
    if (tier >= TIMER_TIERS) {
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

float adaptive_timer_resolution_to_precision_us(uint32_t resolution_hz) {
    if (resolution_hz == 0) return 0.0f;
    return 1000000.0f / (float)resolution_hz;
}

float adaptive_timer_calculate_gain(uint32_t current_resolution, uint32_t base_resolution) {
    if (base_resolution == 0) return 1.0f;
    return (float)current_resolution / (float)base_resolution;
}

void adaptive_timer_print_config(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Adaptive timer not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Adaptive Timer Configuration ===");
    LOG_SYSTEM_I("Adaptive mode: %s", 
             g_timer_config.adaptive_enabled ? "enabled" : "disabled");
    LOG_SYSTEM_I("Current tier: %d (%s)", 
             g_timer_config.current_tier, 
             adaptive_timer_tier_to_string(g_timer_config.current_tier));
    LOG_SYSTEM_I("Current resolution: %lu Hz (%.1fµs)", 
             g_timer_config.current_resolution,
             adaptive_timer_resolution_to_precision_us(g_timer_config.current_resolution));
    
    LOG_SYSTEM_I("Tier thresholds and resolutions:");
    for (uint8_t i = 0; i < TIMER_TIERS; i++) {
        LOG_SYSTEM_I("  Tier %d: 0-%d RPM", i, g_timer_config.rpm_thresholds[i]);
        LOG_SYSTEM_I("    Resolution: %lu Hz (%.1fµs)", 
                 g_timer_config.timer_resolutions[i],
                 adaptive_timer_resolution_to_precision_us(g_timer_config.timer_resolutions[i]));
    }
}

void adaptive_timer_print_stats(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Adaptive timer not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Adaptive Timer Statistics ===");
    LOG_SYSTEM_I("Tier transitions: %lu", g_timer_stats.tier_transitions);
    LOG_SYSTEM_I("Resolution changes: %lu", g_timer_stats.resolution_changes);
    LOG_SYSTEM_I("Measurements: %lu", g_timer_stats.measurements_count);
    LOG_SYSTEM_I("Validation failures: %lu", g_timer_stats.validation_failures);
    
    if (g_timer_stats.measurements_count > 0) {
        LOG_SYSTEM_I("Average resolution: %.1f MHz", 
                 g_timer_stats.avg_resolution_hz / 1000000.0f);
        LOG_SYSTEM_I("Max precision gain: %.1fx", g_timer_stats.max_precision_gain);
        
        // Calcular taxa de falhas
        float failure_rate = (float)g_timer_stats.validation_failures / 
                            (float)g_timer_stats.measurements_count * 100.0f;
        LOG_SYSTEM_I("Validation failure rate: %.2f%%", failure_rate);
    }
}

void adaptive_timer_print_validation(void) {
    if (!g_initialized) {
        LOG_SYSTEM_E("Adaptive timer not initialized");
        return;
    }
    
    LOG_SYSTEM_I("=== Timer Validation Status ===");
    LOG_SYSTEM_I("Last timestamp: %lu µs", g_timer_validation.last_timestamp_us);
    LOG_SYSTEM_I("Expected period: %lu µs", g_timer_validation.expected_period_us);
    LOG_SYSTEM_I("Validation error: %.1f µs", g_timer_validation.validation_error);
    LOG_SYSTEM_I("Validation passed: %s", 
             g_timer_validation.validation_passed ? "YES" : "NO");
}
