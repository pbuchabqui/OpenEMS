/**
 * @file hp_state.h
 * @brief Módulo centralizado de estado de alta precisão
 * 
 * Este módulo gerencia o estado compartilhado entre todos os componentes
 * de timing de alta precisão, eliminando duplicação de estado.
 * 
 * Componentes gerenciados:
 * - Phase predictor: Predição de período do motor
 * - Hardware latency compensation: Compensação de latência de hardware
 * - Jitter measurer: Medição de jitter de timing
 */

#ifndef HP_STATE_H
#define HP_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "scheduler/hp_timing.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa o módulo de estado HP
 * 
 * Deve ser chamado uma única vez durante a inicialização do sistema.
 * Inicializa todas as estruturas de estado compartilhadas.
 * 
 * @param initial_period_us Período inicial estimado em microssegundos
 * @return true se inicializado com sucesso
 */
bool hp_state_init(float initial_period_us);

/**
 * @brief Obtém ponteiro para o phase predictor global
 * @return Ponteiro para a estrutura phase_predictor_t
 */
phase_predictor_t* hp_state_get_phase_predictor(void);

/**
 * @brief Obtém ponteiro para a estrutura de latência de hardware global
 * @return Ponteiro para a estrutura hardware_latency_comp_t
 */
hardware_latency_comp_t* hp_state_get_hw_latency(void);

/**
 * @brief Obtém ponteiro para o medidor de jitter global
 * @return Ponteiro para a estrutura jitter_measurer_t
 */
jitter_measurer_t* hp_state_get_jitter_measurer(void);

/**
 * @brief Atualiza o preditor de fase com nova medição
 * 
 * Função de conveniência que encapsula o acesso ao estado global.
 * 
 * @param measured_period_us Período medido em microssegundos
 * @param timestamp Timestamp em ciclos
 */
void hp_state_update_phase_predictor(float measured_period_us, uint32_t timestamp);

/**
 * @brief Prediz o próximo período do motor
 * 
 * Função de conveniência que encapsula o acesso ao estado global.
 * 
 * @param fallback_period Período de fallback se predição não disponível
 * @return Período predito em microssegundos
 */
float hp_state_predict_next_period(float fallback_period);

/**
 * @brief Obtém latência de hardware para condições dadas (coil/ignição)
 * 
 * @param battery_voltage Tensão da bateria em volts
 * @param temperature Temperatura em Celsius
 * @return Latência em microssegundos
 */
float hp_state_get_latency(float battery_voltage, float temperature);

/**
 * @brief Obtém latência de injetor para condições dadas
 * 
 * @param battery_voltage Tensão da bateria em volts
 * @param temperature Temperatura em Celsius
 * @return Latência em microssegundos
 */
float hp_state_get_injector_latency(float battery_voltage, float temperature);

/**
 * @brief Registra medição de jitter
 * 
 * @param expected_us Tempo esperado em microssegundos
 * @param actual_us Tempo real em microssegundos
 */
void hp_state_record_jitter(uint32_t expected_us, uint32_t actual_us);

/**
 * @brief Obtém estatísticas de jitter
 * 
 * @param[out] avg_us Jitter médio em microssegundos
 * @param[out] max_us Jitter máximo em microssegundos
 * @param[out] min_us Jitter mínimo em microssegundos
 */
void hp_state_get_jitter_stats(float *avg_us, float *max_us, float *min_us);

#ifdef __cplusplus
}
#endif

#endif // HP_STATE_H
