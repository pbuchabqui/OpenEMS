/**
 * @file high_precision_timing.h
 * @brief Sistema de alta precisão de timing para ESP32-S3 EFI
 * 
 * Implementa:
 * - Sistema de contagem de ciclos (CCOUNT)
 * - Core isolamento total
 * - Predição de fase adaptativa
 * - Compensação de latência física
 * 
 * Otimizações:
 * - Conversões us/cycles usando inteiros quando possível
 * - Funções inline para evitar overhead de chamada
 * - IRAM_ATTR para código crítico em ISR
 */

#ifndef HIGH_PRECISION_TIMING_H
#define HIGH_PRECISION_TIMING_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// CONSTANTES DE OTIMIZAÇÃO
//=============================================================================

// CPU frequency em MHz (compile-time constant para otimização)
#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#define HP_CPU_FREQ_MHZ CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#else
#define HP_CPU_FREQ_MHZ 240  // Default to 240MHz for ESP32-S3
#endif

//=============================================================================
// SISTEMA DE CONTAGEM DE CICLOS (CCOUNT)
//=============================================================================

/**
 * @brief Obtém a contagem atual de ciclos da CPU
 * @return Contagem de ciclos do CCOUNT register
 */
IRAM_ATTR static inline uint32_t hp_get_cycle_count(void) {
    uint32_t ccount;
    __asm__ volatile ("rsr %0, ccount" : "=r"(ccount));
    return ccount;
}

/**
 * @brief Configura alarme de comparação de ciclos
 * @param target_cycles Ciclo alvo para disparo
 * @note Usa CCOMPARE register para timing preciso
 */
IRAM_ATTR static inline void hp_set_cycle_alarm(uint32_t target_cycles) {
    __asm__ volatile ("wsr %0, ccompare0" :: "r"(target_cycles));
}

/**
 * @brief Converte microssegundos para ciclos de CPU (versão inteira otimizada)
 * @param us Tempo em microssegundos
 * @return Número de ciclos correspondente
 * @note Usa multiplicação inteira para melhor performance
 */
IRAM_ATTR static inline uint32_t hp_us_to_cycles(uint32_t us) {
    return us * HP_CPU_FREQ_MHZ;
}

/**
 * @brief Converte microssegundos para ciclos (versão float para precisão)
 * @param us Tempo em microssegundos (float)
 * @return Número de ciclos correspondente
 */
IRAM_ATTR static inline uint32_t hp_us_to_cycles_f(float us) {
    return (uint32_t)(us * (float)HP_CPU_FREQ_MHZ);
}

/**
 * @brief Converte ciclos para microssegundos (versão inteira otimizada)
 * @param cycles Número de ciclos
 * @return Tempo em microssegundos
 * @note Usa divisão inteira para melhor performance
 */
IRAM_ATTR static inline uint32_t hp_cycles_to_us_u32(uint32_t cycles) {
    return cycles / HP_CPU_FREQ_MHZ;
}

/**
 * @brief Converte ciclos para microssegundos (versão float)
 * @param cycles Número de ciclos
 * @return Tempo em microssegundos
 */
IRAM_ATTR static inline float hp_cycles_to_us(uint32_t cycles) {
    return (float)cycles / (float)HP_CPU_FREQ_MHZ;
}

//=============================================================================
// PREDITOR DE FASE ADAPTATIVO
//=============================================================================

/**
 * @brief Estrutura do preditor de fase adaptativo
 */
typedef struct {
    float alpha;              // Fator de adaptação dinâmico
    float predicted_period;   // Período predito em microssegundos
    float acceleration;      // Aceleração em us/us
    float last_period;        // Último período medido
    float last_dt;           // Último delta de tempo
    uint32_t last_timestamp;  // Timestamp da última medição (ciclos)
    uint32_t tooth_count;    // Contador de dentes
} phase_predictor_t;

/**
 * @brief Inicializa o preditor de fase
 * @param predictor Ponteiro para a estrutura do preditor
 * @param initial_period Período inicial em microssegundos
 */
void hp_init_phase_predictor(phase_predictor_t *predictor, float initial_period);

/**
 * @brief Atualiza o preditor com nova medição de período
 * @param predictor Ponteiro para a estrutura do preditor
 * @param current_period Período atual medido em microssegundos
 * @param timestamp Timestamp atual em ciclos
 * @note IRAM_ATTR - função crítica de timing chamada em ISR context
 */
IRAM_ATTR void hp_update_phase_predictor(phase_predictor_t *predictor, float current_period, uint32_t timestamp);

/**
 * @brief Prediz o próximo período
 * @param predictor Ponteiro para a estrutura do preditor
 * @param dt Delta de tempo desde última medição
 * @return Período predito em microssegundos
 */
IRAM_ATTR static inline float hp_predict_next_period(phase_predictor_t *predictor, float dt) {
    return predictor->predicted_period + (predictor->acceleration * dt * predictor->alpha);
}

/**
 * @brief Prediz o tempo até o próximo evento
 * @param predictor Ponteiro para a estrutura do preditor
 * @param teeth_ahead Número de dentes à frente
 * @return Tempo até o próximo evento em microssegundos
 */
IRAM_ATTR static inline float hp_predict_time_to_event(phase_predictor_t *predictor, uint32_t teeth_ahead) {
    return hp_predict_next_period(predictor, 0) * (float)teeth_ahead;
}

//=============================================================================
// COMPENSAÇÃO DE LATÊNCIA FÍSICA
//=============================================================================

/**
 * @brief Estrutura de compensação de latência de hardware
 */
typedef struct {
    float coil_delay_base;       // Latência base da bobina em us
    float injector_delay_base;   // Latência base do injetor em us
    float voltage_factor;        // Fator de compensação por volt
    float temp_factor;           // Fator de compensação por grau Celsius
    float temp_reference;        // Temperatura de referência
} hardware_latency_comp_t;

/**
 * @brief Inicializa a compensação de latência
 * @param comp Ponteiro para a estrutura de compensação
 */
void hp_init_hardware_latency(hardware_latency_comp_t *comp);

/**
 * @brief Calcula latência compensada da bobina
 * @param comp Ponteiro para a estrutura de compensação
 * @param voltage Tensão da bateria em volts
 * @param temperature Temperatura em Celsius
 * @return Latência compensada em microssegundos
 */
IRAM_ATTR static inline float hp_get_coil_latency(hardware_latency_comp_t *comp, 
                                                   float voltage, float temperature) {
    float voltage_comp = comp->coil_delay_base * (1.0f + (12.0f - voltage) * comp->voltage_factor);
    float temp_comp = voltage_comp * (1.0f + (temperature - comp->temp_reference) * comp->temp_factor);
    return temp_comp;
}

/**
 * @brief Calcula latência compensada do injetor
 * @param comp Ponteiro para a estrutura de compensação
 * @param voltage Tensão da bateria em volts
 * @param temperature Temperatura em Celsius
 * @return Latência compensada em microssegundos
 */
IRAM_ATTR static inline float hp_get_injector_latency(hardware_latency_comp_t *comp,
                                                       float voltage, float temperature) {
    float voltage_comp = comp->injector_delay_base * (1.0f + (12.0f - voltage) * comp->voltage_factor);
    float temp_comp = voltage_comp * (1.0f + (temperature - comp->temp_reference) * comp->temp_factor);
    return temp_comp;
}

//=============================================================================
// MEDIÇÃO DE JITTER
//=============================================================================

/**
 * @brief Estrutura para medição de jitter
 */
typedef struct {
    uint32_t sample_count;      // Número de amostras
    uint32_t max_jitter;       // Jitter máximo em ciclos
    uint32_t min_jitter;       // Jitter mínimo em ciclos
    uint64_t jitter_sum;       // Soma de todos os jitters
    uint32_t last_target;       // Último target programado
    uint32_t last_actual;       // Último valor real
    bool is_first_sample;       // Flag para primeira amostra
} jitter_measurer_t;

/**
 * @brief Inicializa o medidor de jitter
 * @param measurer Ponteiro para a estrutura de medição
 */
void hp_init_jitter_measurer(jitter_measurer_t *measurer);

/**
 * @brief Registra uma medição de jitter
 * @param measurer Ponteiro para a estrutura de medição
 * @param target_target Target programado em ciclos
 * @param actual_target Target real em ciclos
 * @note IRAM_ATTR - função crítica chamada em timing-critical context
 */
IRAM_ATTR void hp_record_jitter(jitter_measurer_t *measurer, uint32_t target_target, uint32_t actual_target);

/**
 * @brief Obtém estatísticas de jitter
 * @param measurer Ponteiro para a estrutura de medição
 * @param out_avg_jitter Ponteiro para receber jitter médio em microssegundos
 * @param out_max_jitter Ponteiro para receber jitter máximo em microssegundos
 * @param out_min_jitter Ponteiro para receber jitter mínimo em microssegundos
 */
void hp_get_jitter_stats(jitter_measurer_t *measurer, 
                         float *out_avg_jitter,
                         float *out_max_jitter,
                         float *out_min_jitter);

//=============================================================================
// CONFIGURAÇÃO DE CORE E PRIORIDADE
//=============================================================================

/**
 * @brief Configura afinidade de tarefa para core específico
 * @param task_handle Handle da tarefa
 * @param core_id ID do core (0 ou 1)
 * @return pdTRUE se bem-sucedido, pdFALSE caso contrário
 */
BaseType_t hp_set_task_core_affinity(TaskHandle_t task_handle, UBaseType_t core_id);

/**
 * @brief Configura prioridade máxima para tarefa
 * @param task_handle Handle da tarefa
 * @return pdTRUE se bem-sucedido, pdFALSE caso contrário
 */
BaseType_t hp_set_task_max_priority(TaskHandle_t task_handle);

/**
 * @brief Cria tarefa de timing crítico com configuração otimizada
 * @param pvTaskCode Função da tarefa
 * @param pcNameBuffer Nome da tarefa
 * @param usStackDepth Tamanho da pilha em palavras
 * @param pvParameters Parâmetros da tarefa
 * @param uxPriority Prioridade da tarefa
 * @param pvCreatedTask Handle da tarefa criada
 * @param core_id Core para alocação (0 ou 1)
 * @return pdTRUE se bem-sucedido, pdFALSE caso contrário
 */
BaseType_t hp_create_critical_task(TaskFunction_t pvTaskCode,
                                    const char * const pcName,
                                    const configSTACK_DEPTH_TYPE usStackDepth,
                                    void * const pvParameters,
                                    UBaseType_t uxPriority,
                                    TaskHandle_t * const pvCreatedTask,
                                    BaseType_t core_id);

//=============================================================================
// UTILITÁRIOS DE TIMING OTIMIZADOS
//=============================================================================

/**
 * @brief Calcula delay de scheduling (versão otimizada com inteiros)
 * @param target_us Tempo alvo em microssegundos
 * @param current_cycles Tempo atual em ciclos
 * @return Delay restante em ciclos
 */
IRAM_ATTR static inline uint32_t hp_calculate_schedule_delay(uint32_t target_us, uint32_t current_cycles) {
    uint32_t target_cycles = hp_us_to_cycles(target_us);
    int32_t delay_cycles = (int32_t)target_cycles - (int32_t)current_cycles;
    return (delay_cycles > 0) ? (uint32_t)delay_cycles : 0;
}

/**
 * @brief Verifica se há overflow de timer
 * @param last_count Último valor do contador
 * @param current_count Contador atual
 * @return true se houve overflow
 */
IRAM_ATTR static inline bool hp_check_timer_overflow(uint32_t last_count, uint32_t current_count) {
    return (current_count < last_count);
}

/**
 * @brief Calcula diferença de tempo entre dois timestamps com tratamento de overflow
 * @param start_time Timestamp inicial em ciclos
 * @param end_time Timestamp final em ciclos
 * @return Diferença em ciclos
 */
IRAM_ATTR static inline uint32_t hp_elapsed_cycles(uint32_t start_time, uint32_t end_time) {
    if (end_time >= start_time) {
        return end_time - start_time;
    }
    // Overflow occurred
    return (UINT32_MAX - start_time) + end_time + 1;
}

/**
 * @brief Calcula diferença de tempo em microssegundos
 * @param start_time Timestamp inicial em ciclos
 * @param end_time Timestamp final em ciclos
 * @return Diferença em microssegundos
 */
IRAM_ATTR static inline uint32_t hp_elapsed_us(uint32_t start_time, uint32_t end_time) {
    return hp_cycles_to_us_u32(hp_elapsed_cycles(start_time, end_time));
}

/**
 * @brief Verifica se um deadline foi atingido
 * @param start_time Timestamp inicial em ciclos
 * @param deadline_us Deadline em microssegundos
 * @return true se o deadline foi atingido
 */
IRAM_ATTR static inline bool hp_deadline_reached(uint32_t start_time, uint32_t deadline_us) {
    uint32_t now = hp_get_cycle_count();
    uint32_t elapsed = hp_elapsed_cycles(start_time, now);
    return (elapsed >= hp_us_to_cycles(deadline_us));
}

/**
 * @brief Aguarda ativamente por um número de microssegundos (busy wait)
 * @param us Tempo para aguardar em microssegundos
 * @note Usa busy-wait com CCOUNT para máxima precisão
 */
IRAM_ATTR static inline void hp_delay_us(uint32_t us) {
    uint32_t start = hp_get_cycle_count();
    uint32_t target = start + hp_us_to_cycles(us);
    while ((int32_t)(hp_get_cycle_count() - start) < (int32_t)hp_us_to_cycles(us)) {
        // Busy wait
    }
}

/**
 * @brief Calcula tempo por grau para RPM dado
 * @param rpm RPM do motor
 * @return Microssegundos por grau
 */
IRAM_ATTR static inline uint32_t hp_us_per_degree(uint16_t rpm) {
    if (rpm == 0) return 0;
    // (60 * 1000000) / (rpm * 360) = 166666 / rpm
    return 166666U / rpm;
}

/**
 * @brief Converte graus para microssegundos dado RPM
 * @param degrees Ângulo em graus
 * @param rpm RPM do motor
 * @return Tempo em microssegundos
 */
IRAM_ATTR static inline uint32_t hp_degrees_to_us(float degrees, uint16_t rpm) {
    if (rpm == 0) return 0;
    // degrees * (60 * 1000000) / (rpm * 360) = degrees * 166666 / rpm
    return (uint32_t)(degrees * 166666.0f / (float)rpm);
}

/**
 * @brief Converte microssegundos para graus dado RPM
 * @param us Tempo em microssegundos
 * @param rpm RPM do motor
 * @return Ângulo em graus
 */
IRAM_ATTR static inline float hp_us_to_degrees(uint32_t us, uint16_t rpm) {
    if (rpm == 0) return 0.0f;
    // us * (rpm * 360) / (60 * 1000000) = us * rpm * 0.000006
    return (float)us * (float)rpm * 0.000006f;
}

#ifdef __cplusplus
}
#endif

#endif // HIGH_PRECISION_TIMING_H
