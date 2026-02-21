/**
 * @file adaptive_timer.h
 * @brief Sistema de timer resolution adaptativa por RPM
 * 
 * Este módulo implementa resolução de timer adaptativa que prioriza
 * alta precisão em baixas rotações onde é mais crítica.
 * 
 * Características:
 * - 4 faixas de resolução dinâmica
 * - Transições suaves entre faixas
 * - Validação cruzada de timestamps
 * - Compatibilidade com sistema MCPWM existente
 */

#ifndef ADAPTIVE_TIMER_H
#define ADAPTIVE_TIMER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// CONSTANTES DE RESOLUÇÃO ADAPTATIVA
//=============================================================================

// Número de faixas de resolução
#define TIMER_TIERS 4

// Limiares de RPM para cada faixa
#define TIMER_TIER_1_MAX 1000    // 0-1000 RPM: máxima precisão
#define TIMER_TIER_2_MAX 2500    // 1000-2500 RPM: alta precisão
#define TIMER_TIER_3_MAX 4500    // 2500-4500 RPM: média precisão
#define TIMER_TIER_4_MAX 8000    // 4500-8000 RPM: precisão normal

// Resoluções de timer por faixa (Hz)
#define TIMER_RES_TIER_1 10000000U  // 10MHz = 0.1µs
#define TIMER_RES_TIER_2 5000000U   // 5MHz = 0.2µs
#define TIMER_RES_TIER_3 2000000U   // 2MHz = 0.5µs
#define TIMER_RES_TIER_4 1000000U   // 1MHz = 1.0µs

// Histerese para evitar transições rápidas
#define TIMER_HYSTERESIS_RPM 100

// Fator de conversão entre resoluções
#define TIMER_CONVERSION_FACTOR 10.0f  // 10x melhoria em marcha lenta

//=============================================================================
// ESTRUTURAS DE DADOS
//=============================================================================

/**
 * @brief Configuração de timer adaptativo
 */
typedef struct {
    uint16_t rpm_thresholds[TIMER_TIERS];     // Limiares de RPM
    uint32_t timer_resolutions[TIMER_TIERS];   // Resoluções por faixa
    uint32_t current_resolution;               // Resolução atual
    uint8_t current_tier;                      // Faixa atual
    uint16_t last_rpm;                         // Último RPM medido
    uint32_t transition_count;                 // Contador de transições
    bool adaptive_enabled;                     // Sistema ativo
} adaptive_timer_config_t;

/**
 * @brief Estatísticas de timer adaptativo
 */
typedef struct {
    uint32_t tier_transitions;                 // Número de transições
    uint32_t resolution_changes;               // Mudanças de resolução
    float avg_resolution_hz;                  // Resolução média
    float max_precision_gain;                 // Ganho máximo de precisão
    uint32_t measurements_count;               // Contagem de medições
    uint32_t validation_failures;              // Falhas de validação
} adaptive_timer_stats_t;

/**
 * @brief Estado de validação cruzada
 */
typedef struct {
    uint32_t last_timestamp_us;                // Último timestamp
    uint32_t expected_period_us;               // Período esperado
    uint32_t actual_period_us;                 // Período medido
    float validation_error;                    // Erro de validação
    bool validation_passed;                    // Validação passou
} timer_validation_t;

//=============================================================================
// API GERAL
//=============================================================================

/**
 * @brief Inicializa o sistema de timer adaptativo
 * @return true se inicializado com sucesso
 */
bool adaptive_timer_init(void);

/**
 * @brief Obtém configuração atual de timer
 * @return Ponteiro para estrutura de configuração
 */
adaptive_timer_config_t* adaptive_timer_get_config(void);

/**
 * @brief Obtém estatísticas de timer
 * @return Ponteiro para estrutura de estatísticas
 */
adaptive_timer_stats_t* adaptive_timer_get_stats(void);

/**
 * @brief Ativa/desativa modo adaptativo
 * @param enabled true para ativar, false para desativar
 */
void adaptive_timer_set_enabled(bool enabled);

/**
 * @brief Verifica se modo adaptativo está ativo
 * @return true se ativo
 */
bool adaptive_timer_is_enabled(void);

//=============================================================================
// API DE RESOLUÇÃO
//=============================================================================

/**
 * @brief Obtém resolução de timer para RPM dado
 * @param rpm RPM do motor
 * @return Resolução em Hz
 */
uint32_t adaptive_timer_get_resolution(uint16_t rpm);

/**
 * @brief Obtém período de timer para RPM dado
 * @param rpm RPM do motor
 * @return Período em nanossegundos
 */
uint32_t adaptive_timer_get_period_ns(uint16_t rpm);

/**
 * @brief Obtém precisão em microssegundos para RPM dado
 * @param rpm RPM do motor
 * @return Precisão em microssegundos
 */
float adaptive_timer_get_precision_us(uint16_t rpm);

/**
 * @brief Obtém faixa atual para RPM dado
 * @param rpm RPM do motor
 * @return Índice da faixa (0-3)
 */
uint8_t adaptive_timer_get_tier_for_rpm(uint16_t rpm);

/**
 * @brief Verifica se houve transição de faixa
 * @param new_rpm Novo valor de RPM
 * @return true se houve transição
 */
bool adaptive_timer_check_transition(uint16_t new_rpm);

//=============================================================================
// API DE CONFIGURAÇÃO
//=============================================================================

/**
 * @brief Atualiza faixa atual baseado no RPM
 * @param rpm RPM atual do motor
 * @return true se faixa foi alterada
 */
bool adaptive_timer_update_tier(uint16_t rpm);

/**
 * @brief Força mudança de resolução
 * @param new_resolution Nova resolução em Hz
 * @return true se mudança foi bem sucedida
 */
bool adaptive_timer_set_resolution(uint32_t new_resolution);

/**
 * @brief Reseta estatísticas de timer
 */
void adaptive_timer_reset_stats(void);

/**
 * @brief Registra transição de faixa
 * @param old_tier Faixa anterior
 * @param new_tier Nova faixa
 * @param rpm RPM da transição
 */
void adaptive_timer_record_transition(uint8_t old_tier, uint8_t new_tier, uint16_t rpm);

//=============================================================================
// API DE VALIDAÇÃO
//=============================================================================

/**
 * @brief Valida timestamp cruzado
 * @param timestamp_us Timestamp atual em microssegundos
 * @param expected_period_us Período esperado
 * @return true se validação passou
 */
bool adaptive_timer_validate_timestamp(uint32_t timestamp_us, uint32_t expected_period_us);

/**
 * @brief Obtém estado de validação
 * @return Ponteiro para estrutura de validação
 */
timer_validation_t* adaptive_timer_get_validation(void);

/**
 * @brief Reseta estado de validação
 */
void adaptive_timer_reset_validation(void);

//=============================================================================
// UTILITÁRIOS
//=============================================================================

/**
 * @brief Converte faixa de timer para string
 * @param tier índice da faixa
 * @return string descritiva
 */
const char* adaptive_timer_tier_to_string(uint8_t tier);

/**
 * @brief Converte resolução para precisão em microssegundos
 * @param resolution_hz Resolução em Hz
 * @return Precisão em microssegundos
 */
float adaptive_timer_resolution_to_precision_us(uint32_t resolution_hz);

/**
 * @brief Calcula ganho de precisão relativo
 * @param current_resolution Resolução atual
 * @param base_resolution Resolução base (1MHz)
 * @return Fator de ganho (1.0 = sem ganho)
 */
float adaptive_timer_calculate_gain(uint32_t current_resolution, uint32_t base_resolution);

/**
 * @brief Imprime configuração atual de timer
 */
void adaptive_timer_print_config(void);

/**
 * @brief Imprime estatísticas de timer
 */
void adaptive_timer_print_stats(void);

/**
 * @brief Imprime estado de validação
 */
void adaptive_timer_print_validation(void);

#ifdef __cplusplus
}
#endif

#endif // ADAPTIVE_TIMER_H
