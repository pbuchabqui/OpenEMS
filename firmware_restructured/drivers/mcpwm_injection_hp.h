/**
 * @file mcpwm_injection_hp.h
 * @brief Header do driver MCPWM de injeção otimizado com compare absoluto
 * 
 * Este driver implementa injeção de combustível de alta precisão usando:
 * - Timer contínuo sem reinício por evento (elimina jitter)
 * - Compare absoluto em ticks (sem recalculação de delay)
 * - Leitura direta de contador do timer
 * - Funções críticas marcadas com IRAM_ATTR para execução em ISR
 */

#ifndef MCPWM_INJECTION_HP_H
#define MCPWM_INJECTION_HP_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t timer_resolution_hz;
    uint32_t max_pulsewidth_us;
    uint8_t cylinder_count;
} mcpwm_injection_config_t;

/**
 * @brief Inicializa o driver de injeção de alta precisão
 * @return true se bem-sucedido
 */
bool mcpwm_injection_hp_init(void);

/**
 * @brief Configura parâmetros do driver
 * @param config Ponteiro para configuração
 * @return true se bem-sucedido
 */
bool mcpwm_injection_hp_configure(const mcpwm_injection_config_t *config);

/**
 * @brief Agenda evento de injeção com compare absoluto
 * 
 * Esta função usa compare absoluto em vez de recalcular delays.
 * O timer roda continuamente sem reinício por evento.
 * 
 * @note IRAM_ATTR - função crítica de timing, pode ser chamada em ISR context
 * 
 * @param cylinder_id ID do injetor (0-3)
 * @param target_us Tempo absoluto no contador MCPWM (em microssegundos)
 * @param pulsewidth_us Largura de pulso desejada
 * @param current_counter Valor atual do contador do timer
 * @return true se bem-sucedido
 */
IRAM_ATTR bool mcpwm_injection_hp_schedule_one_shot_absolute(
    uint8_t cylinder_id,
    uint32_t target_us,
    uint32_t pulsewidth_us,
    uint32_t current_counter);

/**
 * @brief Agenda múltiplos injetores sequencialmente
 * @note IRAM_ATTR - função crítica de timing
 * 
 * @param base_time_us Tempo base absoluto em microssegundos (contador MCPWM)
 * @param pulsewidth_us Largura de pulso em microssegundos
 * @param cylinder_offsets Array de offsets por cilindro
 * @param current_counter Valor atual do contador
 * @return true se todos agendados com sucesso
 */
IRAM_ATTR bool mcpwm_injection_hp_schedule_sequential_absolute(
    uint32_t base_time_us,
    uint32_t pulsewidth_us,
    uint32_t cylinder_offsets[4],
    uint32_t current_counter);

/**
 * @brief Para injetor específico
 * @param cylinder_id ID do injetor (0-3)
 * @return true se bem-sucedido
 */
bool mcpwm_injection_hp_stop(uint8_t cylinder_id);

/**
 * @brief Para todos os injetores
 * @return true se bem-sucedido
 */
bool mcpwm_injection_hp_stop_all(void);

/**
 * @brief Obtém status do canal de injeção
 * @param cylinder_id ID do injetor (0-3)
 * @param status Ponteiro para estrutura de status
 * @return true se bem-sucedido
 */
bool mcpwm_injection_hp_get_status(uint8_t cylinder_id, mcpwm_injector_channel_t *status);

/**
 * @brief Obtém estatísticas de jitter
 * @param[out] avg_us Jitter médio em microssegundos
 * @param[out] max_us Jitter máximo em microssegundos
 * @param[out] min_us Jitter mínimo em microssegundos
 */
void mcpwm_injection_hp_get_jitter_stats(float *avg_us, float *max_us, float *min_us);

/**
 * @brief Aplica compensação de latência física
 * @param[in,out] pulsewidth_us Ponteiro para largura de pulso (será modificado)
 * @param battery_voltage Tensão da bateria em volts
 * @param temperature Temperatura em Celsius
 */
void mcpwm_injection_hp_apply_latency_compensation(float *pulsewidth_us, float battery_voltage, float temperature);

/**
 * @brief Obtém valor atual do contador do timer MCPWM
 * 
 * @note IRAM_ATTR - função crítica de timing, pode ser chamada em ISR context
 * @note Esta função deve ser usada para obter o tempo atual real para cálculos de timing
 * 
 * @param cylinder_id ID do injetor (0-3)
 * @return Valor atual do contador em microssegundos, ou 0 se inválido
 */
IRAM_ATTR uint32_t mcpwm_injection_hp_get_counter(uint8_t cylinder_id);

/**
 * @brief Obtém configuração atual
 * @return Ponteiro para configuração (read-only)
 */
const mcpwm_injection_config_t* mcpwm_injection_hp_get_config(void);

/**
 * @brief Desinicializa o driver
 * @return true se bem-sucedido
 */
bool mcpwm_injection_hp_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // MCPWM_INJECTION_HP_H
