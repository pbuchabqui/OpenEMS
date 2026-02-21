/**
 * @file precision_manager.h
 * @brief Sistema de precisão adaptativa por RPM para OpenEMS
 * 
 * Este módulo implementa precisão adaptativa que prioriza alta precisão
 * em baixas rotações onde é mais crítica para o motor.
 * 
 * Características:
 * - Binning logarítmico de RPM (maior granularidade em baixa rotação)
 * - Timer resolution adaptativa (maior precisão onde importa)
 * - Tolerâncias dinâmicas baseadas em RPM
 * - Compatibilidade total com sistema atual
 */

#ifndef PRECISION_MANAGER_H
#define PRECISION_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// CONSTANTES DE PRECISÃO ADAPTATIVA
//=============================================================================

// Número de faixas de precisão
#define PRECISION_TIERS 4

// Limiares de RPM para cada faixa
#define RPM_TIER_1_MAX 1000    // 0-1000 RPM: máxima precisão
#define RPM_TIER_2_MAX 2500    // 1000-2500 RPM: alta precisão
#define RPM_TIER_3_MAX 4500    // 2500-4500 RPM: média precisão
#define RPM_TIER_4_MAX 8000    // 4500-8000 RPM: precisão normal

// Resoluções de timer por faixa (Hz)
#define TIMER_RES_TIER_1 10000000U  // 10MHz = 0.1µs
#define TIMER_RES_TIER_2 5000000U   // 5MHz = 0.2µs
#define TIMER_RES_TIER_3 2000000U   // 2MHz = 0.5µs
#define TIMER_RES_TIER_4 1000000U   // 1MHz = 1.0µs

// Tolerâncias angulares por faixa (graus)
#define ANGULAR_TOL_TIER_1 0.2f   // ±0.2° em marcha lenta
#define ANGULAR_TOL_TIER_2 0.3f   // ±0.3° em baixa rotação
#define ANGULAR_TOL_TIER_3 0.5f   // ±0.5° em média rotação
#define ANGULAR_TOL_TIER_4 0.8f   // ±0.8° em alta rotação

// Tolerâncias de injeção por faixa (percentual)
#define INJECTION_TOL_TIER_1 0.2f  // ±0.2% em marcha lenta
#define INJECTION_TOL_TIER_2 0.3f  // ±0.3% em baixa rotação
#define INJECTION_TOL_TIER_3 0.5f  // ±0.5% em média rotação
#define INJECTION_TOL_TIER_4 0.8f  // ±0.8% em alta rotação

//=============================================================================
// ESTRUTURAS DE DADOS
//=============================================================================

/**
 * @brief Configuração de precisão adaptativa
 */
typedef struct {
    uint16_t rpm_thresholds[PRECISION_TIERS];     // Limiares de RPM
    uint32_t timer_resolutions[PRECISION_TIERS];   // Resoluções por faixa
    float angular_tolerances[PRECISION_TIERS];      // Tolerâncias angulares
    float injection_tolerances[PRECISION_TIERS];    // Tolerâncias de injeção
    uint8_t current_tier;                         // Faixa atual
    bool adaptive_enabled;                          // Sistema ativo
} precision_config_t;

/**
 * @brief Estatísticas de precisão
 */
typedef struct {
    uint32_t tier_transitions;                     // Número de transições
    uint32_t precision_violations;                 // Violações de tolerância
    float avg_jitter_us;                          // Jitter médio
    float max_jitter_us;                          // Jitter máximo
    uint32_t measurements_count;                   // Contagem de medições
} precision_stats_t;

//=============================================================================
// API GERAL
//=============================================================================

/**
 * @brief Inicializa o gerenciador de precisão
 * @return true se inicializado com sucesso
 */
bool precision_manager_init(void);

/**
 * @brief Obtém configuração atual de precisão
 * @return Ponteiro para estrutura de configuração
 */
precision_config_t* precision_get_config(void);

/**
 * @brief Obtém estatísticas de precisão
 * @return Ponteiro para estrutura de estatísticas
 */
precision_stats_t* precision_get_stats(void);

/**
 * @brief Ativa/desativa modo adaptativo
 * @param enabled true para ativar, false para desativar
 */
void precision_set_adaptive_mode(bool enabled);

/**
 * @brief Verifica se modo adaptativo está ativo
 * @return true se ativo
 */
bool precision_is_adaptive_enabled(void);

//=============================================================================
// API DE CONSULTA
//=============================================================================

/**
 * @brief Obtém resolução de timer para RPM dado
 * @param rpm RPM do motor
 * @return Resolução em Hz
 */
uint32_t precision_get_timer_resolution(uint16_t rpm);

/**
 * @brief Obtém tolerância angular para RPM dado
 * @param rpm RPM do motor
 * @return Tolerância em graus
 */
float precision_get_angular_tolerance(uint16_t rpm);

/**
 * @brief Obtém tolerância de injeção para RPM dado
 * @param rpm RPM do motor
 * @return Tolerância percentual
 */
float precision_get_injection_tolerance(uint16_t rpm);

/**
 * @brief Obtém faixa atual para RPM dado
 * @param rpm RPM do motor
 * @return Índice da faixa (0-3)
 */
uint8_t precision_get_tier_for_rpm(uint16_t rpm);

/**
 * @brief Verifica se houve transição de faixa
 * @param new_rpm Novo valor de RPM
 * @return true se houve transição
 */
bool precision_check_tier_transition(uint16_t new_rpm);

//=============================================================================
// API DE CONFIGURAÇÃO
//=============================================================================

/**
 * @brief Atualiza faixa atual baseado no RPM
 * @param rpm RPM atual do motor
 * @return true se faixa foi alterada
 */
bool precision_update_tier(uint16_t rpm);

/**
 * @brief Reseta estatísticas de precisão
 */
void precision_reset_stats(void);

/**
 * @brief Registra violação de precisão
 * @param expected_valor esperado
 * @param actual valor medido
 * @param tolerance tolerância permitida
 */
void precision_record_violation(float expected, float actual, float tolerance);

/**
 * @brief Registra medição de jitter
 * @param jitter_us valor de jitter em microssegundos
 */
void precision_record_jitter(float jitter_us);

//=============================================================================
// UTILITÁRIOS
//=============================================================================

/**
 * @brief Converte faixa de precisão para string
 * @param tier índice da faixa
 * @return string descritiva
 */
const char* precision_tier_to_string(uint8_t tier);

/**
 * @brief Imprime configuração atual de precisão
 */
void precision_print_config(void);

/**
 * @brief Imprime estatísticas de precisão
 */
void precision_print_stats(void);

#ifdef __cplusplus
}
#endif

#endif // PRECISION_MANAGER_H
