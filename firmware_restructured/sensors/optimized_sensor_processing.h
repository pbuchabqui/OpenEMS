/**
 * @file optimized_sensor_processing.h
 * @brief Módulo de processamento de sensores otimizado com DSP e vetorial
 * 
 * Integra as capacidades DSP do ESP32-S3 com processamento vetorial para
 * máximo desempenho no processamento de sensores do sistema EFI.
 * 
 * Recursos:
 * - Processamento paralelo de múltiplos sensores
 * - Filtros adaptativos com cancelamento de ruído
 * - Detecção de falhas em tempo real
 * - Calibração automática de sensores
 */

#ifndef OPTIMIZED_SENSOR_PROCESSING_H
#define OPTIMIZED_SENSOR_PROCESSING_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_sensor_processing.h"
#include "vector_math.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Configurações e Constantes
//=============================================================================

/** @brief Número máximo de sensores processados em paralelo */
#define OPTIMIZED_MAX_SENSORS      8

/** @brief Taxa de amostragem para sensores rápidos (Hz) */
#define OPTIMIZED_FAST_SAMPLE_RATE 1000

/** @brief Taxa de amostragem para sensores lentos (Hz) */
#define OPTIMIZED_SLOW_SAMPLE_RATE 100

/** @brief Limiar para detecção de falha de sensor */
#define OPTIMIZED_FAULT_THRESHOLD  5.0f

/** @brief Número de amostras para calibração automática */
#define OPTIMIZED_CALIBRATION_SAMPLES 100

//=============================================================================
// Tipos de Sensores
//=============================================================================

typedef enum {
    OPTIMIZED_SENSOR_MAP = 0,      /**< Sensor MAP (rápido) */
    OPTIMIZED_SENSOR_TPS,          /**< Sensor TPS (rápido) */
    OPTIMIZED_SENSOR_CLT,          /**< Sensor CLT (lento) */
    OPTIMIZED_SENSOR_IAT,          /**< Sensor IAT (lento) */
    OPTIMIZED_SENSOR_O2,           /**< Sensor O2 (rápido) */
    OPTIMIZED_SENSOR_VBAT,         /**< Bateria (lento) */
    OPTIMIZED_SENSOR_KNOCK,        /**< Sensor knock (rápido) */
    OPTIMIZED_SENSOR_FLEX,         /**< Flex fuel (médio) */
    OPTIMIZED_SENSOR_COUNT
} optimized_sensor_type_t;

typedef enum {
    OPTIMIZED_RATE_FAST = 0,       /**< 1000 Hz */
    OPTIMIZED_RATE_MEDIUM,         /**< 500 Hz */
    OPTIMIZED_RATE_SLOW,           /**< 100 Hz */
    OPTIMIZED_RATE_COUNT
} optimized_sample_rate_t;

//=============================================================================
// Estruturas de Dados
//=============================================================================

/**
 * @brief Configuração de sensor otimizado
 */
typedef struct {
    optimized_sensor_type_t type;       /**< Tipo do sensor */
    optimized_sample_rate_t sample_rate; /**< Taxa de amostragem */
    float min_value;                    /**< Valor mínimo esperado */
    float max_value;                    /**< Valor máximo esperado */
    float noise_threshold;              /**< Limiar de ruído */
    bool enable_adaptive_filter;        /**< Habilitar filtro adaptativo */
    bool enable_fault_detection;        /**< Habilitar detecção de falhas */
    bool enable_calibration;            /**< Habilitar calibração automática */
} optimized_sensor_config_t;

/**
 * @brief Estado de sensor otimizado
 */
typedef struct {
    // Buffers de processamento
    float raw_buffer[OPTIMIZED_CALIBRATION_SAMPLES];
    float filtered_buffer[OPTIMIZED_CALIBRATION_SAMPLES];
    float calibration_buffer[OPTIMIZED_CALIBRATION_SAMPLES];
    
    // Estatísticas do sensor
    float current_value;                /**< Valor atual filtrado */
    float raw_value;                    /**< Valor bruto atual */
    float mean_value;                   /**< Média histórica */
    float std_dev;                      /**< Desvio padrão */
    float min_value;                    /**< Valor mínimo observado */
    float max_value;                    /**< Valor máximo observado */
    float snr_db;                       /**< Relação sinal/ruído */
    
    // Calibração
    float calibration_offset;           /**< Offset de calibração */
    float calibration_scale;            /**< Escala de calibração */
    uint16_t calibration_count;         /**< Contador de amostras para calibração */
    bool calibrated;                    /**< Estado de calibração */
    
    // Detecção de falhas
    uint16_t fault_count;               /**< Contador de falhas */
    bool fault_detected;                /**< Flag de falha ativa */
    uint32_t last_fault_time;           /**< Timestamp da última falha */
    
    // Configuração
    optimized_sensor_config_t config;   /**< Configuração do sensor */
    
    // Estado interno
    bool initialized;                   /**< Estado de inicialização */
    uint32_t sample_count;              /**< Contador total de amostras */
    uint32_t last_update_time;          /**< Timestamp da última atualização */
} optimized_sensor_state_t;

/**
 * @brief Contexto principal de processamento otimizado
 */
typedef struct {
    // Processadores DSP e vetorial
    dsp_sensor_processor_t dsp_processor;
    vector_context_t vector_ctx;
    
    // Estados dos sensores
    optimized_sensor_state_t sensors[OPTIMIZED_SENSOR_COUNT];
    
    // Buffers para processamento em lote
    float batch_input[OPTIMIZED_MAX_SENSORS][VECTOR_MAX_SIZE];
    float batch_output[OPTIMIZED_MAX_SENSORS][VECTOR_MAX_SIZE];
    float batch_angles[VECTOR_MAX_SIZE];
    float batch_timing[VECTOR_MAX_SIZE];
    
    // Configurações globais
    uint16_t fast_sample_rate;          /**< Taxa de amostragem rápida */
    uint16_t slow_sample_rate;          /**< Taxa de amostragem lenta */
    bool parallel_processing_enabled;   /**< Habilitar processamento paralelo */
    bool adaptive_filtering_enabled;    /**< Habilitar filtragem adaptativa */
    
    // Estatísticas globais
    uint32_t total_samples_processed;   /**< Total de amostras processadas */
    uint32_t processing_time_us;        /**< Tempo total de processamento */
    float cpu_usage_percent;            /**< Uso de CPU */
    
    // Estado
    bool initialized;                   /**< Estado de inicialização */
    uint32_t init_time;                 /**< Timestamp de inicialização */
} optimized_sensor_processor_t;

/**
 * @brief Estrutura de resultados do processamento em lote
 */
typedef struct {
    float sensor_values[OPTIMIZED_SENSOR_COUNT];
    bool sensor_faults[OPTIMIZED_SENSOR_COUNT];
    float sensor_snr[OPTIMIZED_SENSOR_COUNT];
    uint32_t processing_time_us;
    uint16_t sensors_processed;
} optimized_batch_results_t;

//=============================================================================
// Funções de Inicialização
//=============================================================================

/**
 * @brief Inicializa o processador de sensores otimizado
 * @param processor Ponteiro para o processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_sensor_processor_init(optimized_sensor_processor_t *processor);

/**
 * @brief Desinicializa o processador de sensores otimizado
 * @param processor Ponteiro para o processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_sensor_processor_deinit(optimized_sensor_processor_t *processor);

/**
 * @brief Configura um sensor específico
 * @param processor Ponteiro para o processador
 * @param sensor_type Tipo do sensor
 * @param config Configuração do sensor
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_sensor_configure(optimized_sensor_processor_t *processor,
                                     optimized_sensor_type_t sensor_type,
                                     const optimized_sensor_config_t *config);

//=============================================================================
// Funções de Processamento Individual
//=============================================================================

/**
 * @brief Processa amostra de um sensor específico
 * @param processor Ponteiro para o processador
 * @param sensor_type Tipo do sensor
 * @param raw_value Valor bruto do sensor
 * @param filtered_value Valor filtrado (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_process_sensor_sample(optimized_sensor_processor_t *processor,
                                         optimized_sensor_type_t sensor_type,
                                         float raw_value,
                                         float *filtered_value);

/**
 * @brief Obtém valor calibrado de um sensor
 * @param processor Ponteiro para o processador
 * @param sensor_type Tipo do sensor
 * @param calibrated_value Valor calibrado (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_get_calibrated_value(optimized_sensor_processor_t *processor,
                                         optimized_sensor_type_t sensor_type,
                                         float *calibrated_value);

/**
 * @brief Verifica se sensor tem falha ativa
 * @param processor Ponteiro para o processador
 * @param sensor_type Tipo do sensor
 * @param has_fault Flag de falha (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_check_sensor_fault(optimized_sensor_processor_t *processor,
                                       optimized_sensor_type_t sensor_type,
                                       bool *has_fault);

//=============================================================================
// Funções de Processamento em Lote (Otimizado ESP32-S3)
//=============================================================================

/**
 * @brief Processa múltiplos sensores em paralelo (vetorial)
 * @param processor Ponteiro para o processador
 * @param raw_values Array de valores brutos [sensor_count]
 * @param results Resultados do processamento
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_process_sensors_batch(optimized_sensor_processor_t *processor,
                                          const float *raw_values,
                                          optimized_batch_results_t *results);

/**
 * @brief Processa série temporal de múltiplos sensores
 * @param processor Ponteiro para o processador
 * @param sensor_data Array de dados [sensor_count][sample_count]
 * @param num_samples Número de amostras
 * @param results Resultados do processamento
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_process_time_series(optimized_sensor_processor_t *processor,
                                        const float (*sensor_data)[VECTOR_MAX_SIZE],
                                        uint16_t num_samples,
                                        optimized_batch_results_t *results);

/**
 * @brief Calcula timing de injeção vetorial para todos os cilindros
 * @param processor Ponteiro para o processador
 * @param rpm RPM atual
 * @param pulse_widths Larguras de pulso por cilindro
 * @param injection_times Tempos de injeção calculados
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_calculate_injection_timing_vectorized(optimized_sensor_processor_t *processor,
                                                          uint16_t rpm,
                                                          const float *pulse_widths,
                                                          uint32_t *injection_times);

/**
 * @brief Calcula timing de ignição vetorial para todos os cilindros
 * @param processor Ponteiro para o processador
 * @param rpm RPM atual
 * @param advance_angles Ângulos de avanço por cilindro
 * @param ignition_times Tempos de ignição calculados
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_calculate_ignition_timing_vectorized(optimized_sensor_processor_t *processor,
                                                         uint16_t rpm,
                                                         const float *advance_angles,
                                                         uint32_t *ignition_times);

//=============================================================================
// Funções de Calibração e Diagnóstico
//=============================================================================

/**
 * @brief Inicia calibração automática de sensores
 * @param processor Ponteiro para o processador
 * @param sensor_types Array de tipos de sensores para calibrar
 * @param num_sensors Número de sensores
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_start_calibration(optimized_sensor_processor_t *processor,
                                      const optimized_sensor_type_t *sensor_types,
                                      uint8_t num_sensors);

/**
 * @brief Finaliza calibração e aplica coeficientes
 * @param processor Ponteiro para o processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_finish_calibration(optimized_sensor_processor_t *processor);

/**
 * @brief Executa diagnóstico completo de sensores
 * @param processor Ponteiro para o processador
 * @param fault_bitmap Bitmap de falhas (saída)
 * @param health_score Pontuação de saúde geral (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_run_diagnostics(optimized_sensor_processor_t *processor,
                                    uint16_t *fault_bitmap,
                                    float *health_score);

/**
 * @brief Obtém estatísticas detalhadas de um sensor
 * @param processor Ponteiro para o processador
 * @param sensor_type Tipo do sensor
 * @param stats Array de estatísticas (saída)
 * @param num_stats Número de estatísticas retornadas
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_get_sensor_statistics(optimized_sensor_processor_t *processor,
                                          optimized_sensor_type_t sensor_type,
                                          float *stats,
                                          uint8_t *num_stats);

//=============================================================================
// Funções de Configuração e Controle
//=============================================================================

/**
 * @brief Habilita/desabilita processamento paralelo
 * @param processor Ponteiro para o processador
 * @param enabled Estado do processamento paralelo
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_set_parallel_processing(optimized_sensor_processor_t *processor,
                                           bool enabled);

/**
 * @brief Configura taxas de amostragem
 * @param processor Ponteiro para o processador
 * @param fast_rate Taxa rápida (Hz)
 * @param slow_rate Taxa lenta (Hz)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_set_sample_rates(optimized_sensor_processor_t *processor,
                                      uint16_t fast_rate,
                                      uint16_t slow_rate);

/**
 * @brief Obtém estatísticas de performance do processador
 * @param processor Ponteiro para o processador
 * @param cpu_usage Uso de CPU (%)
 * @param avg_processing_time Tempo médio de processamento (μs)
 * @param throughput Vazão de processamento (amostras/segundo)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_get_performance_stats(optimized_sensor_processor_t *processor,
                                           float *cpu_usage,
                                           uint32_t *avg_processing_time,
                                           uint32_t *throughput);

/**
 * @brief Reseta estatísticas e contadores
 * @param processor Ponteiro para o processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t optimized_reset_statistics(optimized_sensor_processor_t *processor);

#ifdef __cplusplus
}
#endif

#endif // OPTIMIZED_SENSOR_PROCESSING_H
