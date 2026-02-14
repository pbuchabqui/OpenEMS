/**
 * @file mcpwm_ignition_hp.h
 * @brief Header do driver MCPWM de ignição otimizado com compare absoluto
 * 
 * Este driver implementa ignição de alta precisão usando:
 * - Timer contínuo sem reinício por evento (elimina jitter)
 * - Compare absoluto em ticks (sem recalculação de delay)
 * - Leitura direta de contador do timer
 * - Core isolamento para timing crítico
 * - Funções críticas marcadas com IRAM_ATTR para execução em ISR
 */

#ifndef MCPWM_IGNITION_HP_H
#define MCPWM_IGNITION_HP_H

#include <stdbool.h>
#include <stdint.h>
#include "mcpwm_ignition.h"
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa o driver de ignição de alta precisão
 * @return true se bem-sucedido
 */
bool mcpwm_ignition_hp_init(void);

/**
 * @brief Agenda evento de ignição com compare absoluto
 * 
 * Esta função usa compare absoluto em vez de recalcular delays.
 * O timer roda continuamente sem reinício por evento.
 * 
 * @note IRAM_ATTR - função crítica de timing, pode ser chamada em ISR context
 * 
 * @param cylinder_id ID do cilindro (1-4)
 * @param target_us Tempo absoluto desde o início da janela (em microssegundos)
 * @param rpm RPM atual para cálculo de dwell
 * @param battery_voltage Tensão da bateria para cálculo de dwell
 * @param current_counter Valor atual do contador do timer
 * @return true se bem-sucedido
 */
IRAM_ATTR bool mcpwm_ignition_hp_schedule_one_shot_absolute(
    uint8_t cylinder_id,
    uint32_t target_us,
    uint16_t rpm,
    float battery_voltage,
    uint32_t current_counter);

/**
 * @brief Agenda múltiplos cilindros sequencialmente
 * @param rpm RPM atual
 * @param battery_voltage Tensão da bateria
 * @param base_target_us Tempo base em microssegundos
 * @param cylinder_offsets Array de offsets por cilindro
 * @return true se todos agendados com sucesso
 */
bool mcpwm_ignition_hp_schedule_sequential_absolute(
    uint16_t rpm,
    float battery_voltage,
    uint32_t base_target_us,
    uint32_t cylinder_offsets[4]);

/**
 * @brief Para cilindro específico
 * @param cylinder_id ID do cilindro (1-4)
 * @return true se bem-sucedido
 */
bool mcpwm_ignition_hp_stop_cylinder(uint8_t cylinder_id);

/**
 * @brief Obtém status do canal de ignição
 * @param cylinder_id ID do cilindro (1-4)
 * @param status Ponteiro para estrutura de status
 * @return true se bem-sucedido
 */
bool mcpwm_ignition_hp_get_status(uint8_t cylinder_id, mcpwm_ignition_status_t *status);

/**
 * @brief Atualiza preditor de fase com medição
 * @note IRAM_ATTR - função crítica de timing
 * @param measured_period_us Período medido em microssegundos
 * @param timestamp Timestamp em ciclos
 */
IRAM_ATTR void mcpwm_ignition_hp_update_phase_predictor(float measured_period_us, uint32_t timestamp);

/**
 * @brief Obtém estatísticas de jitter
 * @param[out] avg_us Jitter médio em microssegundos
 * @param[out] max_us Jitter máximo em microssegundos
 * @param[out] min_us Jitter mínimo em microssegundos
 */
void mcpwm_ignition_hp_get_jitter_stats(float *avg_us, float *max_us, float *min_us);

/**
 * @brief Aplica compensação de latência física
 * @param[in,out] timing_us Ponteiro para timing (será modificado)
 * @param battery_voltage Tensão da bateria em volts
 * @param temperature Temperatura em Celsius
 */
void mcpwm_ignition_hp_apply_latency_compensation(float *timing_us, float battery_voltage, float temperature);

/**
 * @brief Obtém valor atual do contador do timer MCPWM
 * 
 * @note IRAM_ATTR - função crítica de timing, pode ser chamada em ISR context
 * @note Esta função deve ser usada para obter o tempo atual real para cálculos de timing
 * 
 * @param cylinder_id ID do cilindro (0-3)
 * @return Valor atual do contador em microssegundos, ou 0 se inválido
 */
IRAM_ATTR uint32_t mcpwm_ignition_hp_get_counter(uint8_t cylinder_id);

/**
 * @brief Desinicializa o driver
 * @return true se bem-sucedido
 */
bool mcpwm_ignition_hp_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // MCPWM_IGNITION_HP_H
