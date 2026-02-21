/**
 * @file precision_integration.h
 * @brief Sistema de integração de precisão adaptativa completa
 * 
 * Este módulo integra o precision_manager e adaptive_timer em um sistema
 * unificado de precisão adaptativa por RPM, fornecendo API única
 * para todo o sistema OpenEMS.
 * 
 * Características:
 * - API unificada para precisão adaptativa
 * - Integração automática entre angular e temporal
 * - Configuração centralizada de tolerâncias
 * - Estatísticas consolidadas do sistema
 * - Interface simples para MCPWM e outros módulos
 */

#ifndef PRECISION_INTEGRATION_H
#define PRECISION_INTEGRATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "precision_manager.h"
#include "adaptive_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// ESTRUTURAS DE INTEGRAÇÃO
//=============================================================================

/**
 * @brief Estado consolidado de precisão adaptativa
 */
typedef struct {
    // Estado do precision manager
    precision_config_t* precision_config;
    precision_stats_t* precision_stats;
    
    // Estado do adaptive timer
    adaptive_timer_config_t* timer_config;
    adaptive_timer_stats_t* timer_stats;
    
    // Estado integrado
    uint16_t current_rpm;
    uint8_t current_precision_tier;
    uint32_t current_timer_resolution;
    float current_angular_tolerance;
    float current_injection_tolerance;
    float current_precision_us;
    
    // Métricas combinadas
    float total_precision_gain;
    float total_jitter_reduction;
    uint32_t total_transitions;
    
    // Sistema
    bool integration_enabled;
    bool legacy_mode;
    uint32_t last_update_time;
} precision_integration_state_t;

/**
 * @brief Configuração de integração
 */
typedef struct {
    bool enable_precision_manager;
    bool enable_adaptive_timer;
    bool enable_automatic_updates;
    bool enable_validation;
    bool enable_statistics;
    uint32_t update_interval_ms;
    float validation_tolerance;
} precision_integration_config_t;

/**
 * @brief Métricas consolidadas do sistema
 */
typedef struct {
    // Precisão angular
    float angular_precision_deg;
    float angular_tolerance_deg;
    float angular_gain_factor;
    
    // Precisão temporal
    float temporal_precision_us;
    float temporal_resolution_hz;
    float temporal_gain_factor;
    
    // Performance
    float jitter_reduction_percent;
    float total_overhead_percent;
    uint32_t transition_count;
    
    // Validação
    uint32_t validation_failures;
    float validation_success_rate;
    
    // Sistema
    uint32_t uptime_seconds;
    uint32_t measurements_count;
    float average_rpm;
} precision_system_metrics_t;

//=============================================================================
// API GERAL DE INTEGRAÇÃO
//=============================================================================

/**
 * @brief Inicializa o sistema de precisão integrada
 * @param config Configuração de integração
 * @return true se inicializado com sucesso
 */
bool precision_integration_init(const precision_integration_config_t* config);

/**
 * @brief Obtém estado atual da integração
 * @return Ponteiro para estrutura de estado
 */
precision_integration_state_t* precision_integration_get_state(void);

/**
 * @brief Obtém métricas consolidadas do sistema
 * @return Ponteiro para estrutura de métricas
 */
precision_system_metrics_t* precision_integration_get_metrics(void);

/**
 * @brief Atualiza estado integrado baseado no RPM
 * @param rpm RPM atual do motor
 * @param timestamp_us Timestamp atual em microssegundos
 * @return true se estado foi atualizado
 */
bool precision_integration_update(uint16_t rpm, uint32_t timestamp_us);

/**
 * @brief Ativa/desativa modo de integração
 * @param enabled true para ativar, false para desativar
 */
void precision_integration_set_enabled(bool enabled);

/**
 * @brief Verifica se integração está ativa
 * @return true se ativa
 */
bool precision_integration_is_enabled(void);

//=============================================================================
// API DE CONSULTA UNIFICADA
//=============================================================================

/**
 * @brief Obtém tolerância angular para RPM dado
 * @param rpm RPM do motor
 * @return Tolerância em graus
 */
float precision_integration_get_angular_tolerance(uint16_t rpm);

/**
 * @brief Obtém tolerância de injeção para RPM dado
 * @param rpm RPM do motor
 * @return Tolerância percentual
 */
float precision_integration_get_injection_tolerance(uint16_t rpm);

/**
 * @brief Obtém resolução de timer para RPM dado
 * @param rpm RPM do motor
 * @return Resolução em Hz
 */
uint32_t precision_integration_get_timer_resolution(uint16_t rpm);

/**
 * @brief Obtém precisão temporal para RPM dado
 * @param rpm RPM do motor
 * @return Precisão em microssegundos
 */
float precision_integration_get_temporal_precision(uint16_t rpm);

/**
 * @brief Obtém ganho total de precisão para RPM dado
 * @param rpm RPM do motor
 * @return Fator de ganho combinado
 */
float precision_integration_get_total_gain(uint16_t rpm);

/**
 * @brief Obtém redução de jitter para RPM dado
 * @param rpm RPM do motor
 * @return Redução percentual de jitter
 */
float precision_integration_get_jitter_reduction(uint16_t rpm);

//=============================================================================
// API DE CONFIGURAÇÃO
//=============================================================================

/**
 * @brief Configura modo legacy (compatibilidade)
 * @param legacy_mode true para modo legacy
 */
void precision_integration_set_legacy_mode(bool legacy_mode);

/**
 * @brief Configura intervalo de atualização automática
 * @param interval_ms Intervalo em milissegundos
 */
void precision_integration_set_update_interval(uint32_t interval_ms);

/**
 * @brief Configura tolerância de validação
 * @param tolerance Tolerância (0.0 a 1.0)
 */
void precision_integration_set_validation_tolerance(float tolerance);

/**
 * @brief Reseta estatísticas integradas
 */
void precision_integration_reset_stats(void);

/**
 * @brief Força recalculação do estado integrado
 * @return true se recalculado com sucesso
 */
bool precision_integration_recalculate(void);

//=============================================================================
// API DE VALIDAÇÃO
//=============================================================================

/**
 * @brief Valida evento angular
 * @param expected_angle Ângulo esperado
 * @param actual_angle Ângulo medido
 * @param rpm RPM do motor
 * @return true se validação passou
 */
bool precision_integration_validate_angular(float expected_angle, float actual_angle, uint16_t rpm);

/**
 * @brief Valida evento temporal
 * @param expected_time Tempo esperado
 * @param actual_time Tempo medido
 * @param rpm RPM do motor
 * @return true se validação passou
 */
bool precision_integration_validate_temporal(uint32_t expected_time, uint32_t actual_time, uint16_t rpm);

/**
 * @brief Valida evento de injeção
 * @param expected_pulse Pulso esperado
 * @param actual_pulse Pulso medido
 * @param rpm RPM do motor
 * @return true se validação passou
 */
bool precision_integration_validate_injection(uint32_t expected_pulse, uint32_t actual_pulse, uint16_t rpm);

//=============================================================================
// API DE ESTATÍSTICAS
//=============================================================================

/**
 * @brief Obtém estatísticas de precisão angular
 * @param rpm RPM do motor
 * @return Estatísticas específicas
 */
precision_stats_t* precision_integration_get_angular_stats(uint16_t rpm);

/**
 * @brief Obtém estatísticas de precisão temporal
 * @param rpm RPM do motor
 * @return Estatísticas específicas
 */
adaptive_timer_stats_t* precision_integration_get_temporal_stats(uint16_t rpm);

/**
 * @brief Obtém estatísticas consolidadas
 * @return Métricas do sistema
 */
precision_system_metrics_t* precision_integration_get_system_metrics(void);

/**
 * @brief Calcula overhead total do sistema
 * @return Overhead percentual
 */
float precision_integration_calculate_overhead(void);

//=============================================================================
// UTILITÁRIOS
//=============================================================================

/**
 * @brief Converte faixa de precisão para string
 * @param tier índice da faixa
 * @return string descritiva
 */
const char* precision_integration_tier_to_string(uint8_t tier);

/**
 * @brief Imprime configuração da integração
 */
void precision_integration_print_config(void);

/**
 * @brief Imprime estado atual da integração
 */
void precision_integration_print_state(void);

/**
 * @brief Imprime métricas do sistema
 */
void precision_integration_print_metrics(void);

/**
 * @brief Imprime resumo da precisão integrada
 */
void precision_integration_print_summary(void);

/**
 * @brief Gera relatório de performance
 * @param buffer Buffer para armazenar relatório
 * @param buffer_size Tamanho do buffer
 * @return Número de bytes escritos
 */
size_t precision_integration_generate_report(char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // PRECISION_INTEGRATION_H
