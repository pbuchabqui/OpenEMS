/**
 * @file esp32s3_integration.h
 * @brief Módulo de integração das melhorias competitivas ESP32-S3
 * 
 * Este módulo integra todos os componentes otimizados para ESP32-S3,
 * fornecendo uma interface unificada para o sistema principal.
 * 
 * Componentes integrados:
 * - Processamento DSP de sensores
 * - Filtros MAP/TPS otimizados
 * - Processamento vetorial de timing
 * - Monitoramento ULP contínuo
 * - Comunicação ESP-NOW com compressão
 */

#ifndef ESP32S3_INTEGRATION_H
#define ESP32S3_INTEGRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_sensor_processing.h"
#include "map_tps_filters.h"
#include "vector_math.h"
#include "ulp_monitor.h"
#include "espnow_compression.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Configurações e Constantes
//=============================================================================

/** @brief Versão da integração ESP32-S3 */
#define ESP32S3_INTEGRATION_VERSION    1

/** @brief Frequência principal de processamento (Hz) */
#define ESP32S3_MAIN_PROCESSING_FREQ   1000

/** @brief Frequência de telemetria (Hz) */
#define ESP32S3_TELEMETRY_FREQ         50

/** @brief Número máximo de cilindros suportados */
#define ESP32S3_MAX_CYLINDERS          8

/** @brief Timeout para inicialização (ms) */
#define ESP32S3_INIT_TIMEOUT_MS         5000

//=============================================================================
// Estruturas de Dados
//=============================================================================

/**
 * @brief Configuração principal da integração ESP32-S3
 */
typedef struct {
    // Configurações DSP
    bool enable_dsp_processing;         /**< Habilitar processamento DSP */
    uint16_t dsp_sample_rate;          /**< Taxa de amostragem DSP */
    bool enable_adaptive_filters;       /**< Habilitar filtros adaptativos */
    
    // Configurações de sensores
    bool enable_map_tps_optimization;  /**< Habilitar otimização MAP/TPS */
    float map_critical_temp;            /**< Temperatura crítica MAP */
    float oil_pressure_critical;         /**< Pressão crítica óleo */
    
    // Configurações ULP
    bool enable_ulp_monitoring;        /**< Habilitar monitoramento ULP */
    bool enable_deep_sleep;            /**< Habilitar deep sleep */
    uint32_t ulp_sample_interval;      /**< Intervalo amostragem ULP */
    
    // Configurações de comunicação
    bool enable_compression;           /**< Habilitar compressão ESP-NOW */
    espnow_compress_type_t compression_type; /**< Tipo de compressão */
    uint8_t compression_level;         /**< Nível de compressão */
    
    // Configurações de timing
    bool enable_vector_timing;         /**< Habilitar timing vetorial */
    uint8_t num_cylinders;            /**< Número de cilindros */
} esp32s3_integration_config_t;

/**
 * @brief Estado da integração ESP32-S3
 */
typedef struct {
    // Componentes DSP
    dsp_sensor_processor_t dsp_processor;
    map_tps_processor_t map_tps_processor;
    vector_context_t vector_context;
    
    // Componentes ULP
    ulp_monitor_context_t ulp_monitor;
    
    // Componentes de comunicação
    espnow_compress_context_t compression_context;
    
    // Dados integrados
    struct {
        // Sensores processados
        float map_filtered;              /**< MAP filtrado */
        float tps_filtered;              /**< TPS filtrado */
        float clt_filtered;              /**< CLT filtrado */
        float oil_temp_filtered;         /**< Temperatura óleo filtrada */
        float oil_pressure_filtered;      /**< Pressão óleo filtrada */
        float battery_voltage_filtered;   /**< Tensão bateria filtrada */
        
        // Timing calculado
        uint32_t injection_times[ESP32S3_MAX_CYLINDERS]; /**< Tempos injeção */
        uint32_t ignition_times[ESP32S3_MAX_CYLINDERS];  /**< Tempos ignição */
        float injection_pw[ESP32S3_MAX_CYLINDERS];       /**< Largura pulso injeção */
        float ignition_advance[ESP32S3_MAX_CYLINDERS];   /**< Avanço ignição */
        
        // Estado do motor
        uint16_t current_rpm;            /**< RPM atual */
        float engine_load;               /**< Carga do motor */
        float map_tps_correlation;       /**< Correlação MAP-TPS */
        bool acceleration_detected;       /**< Aceleração detectada */
        bool deceleration_detected;       /**< Desaceleração detectada */
        
        // Status ULP
        bool ulp_critical_condition;     /**< Condição crítica ULP */
        ulp_adc_channel_t critical_channel; /**< Canal crítico */
        float critical_value;             /**< Valor crítico */
        
        // Performance
        uint32_t processing_time_us;      /**< Tempo processamento */
        float cpu_usage_percent;          /**< Uso de CPU */
        float compression_ratio;          /**< Taxa compressão */
    } data;
    
    // Configuração
    esp32s3_integration_config_t config;
    
    // Estado
    bool initialized;                    /**< Estado de inicialização */
    bool running;                        /**< Estado de execução */
    uint32_t start_time;                 /**< Timestamp de início */
    uint32_t last_update_time;           /**< Última atualização */
    uint32_t update_count;               /**< Contador de atualizações */
} esp32s3_integration_t;

/**
 * @brief Resultado do processamento integrado
 */
typedef struct {
    bool success;                        /**< Sucesso do processamento */
    uint32_t processing_time_us;          /**< Tempo de processamento */
    float cpu_usage;                     /**< Uso de CPU */
    bool critical_condition;              /**< Condição crítica detectada */
    bool performance_warning;            /**< Aviso de performance */
    uint16_t sensors_processed;          /**< Sensores processados */
    uint16_t calculations_performed;      /**< Cálculos realizados */
} esp32s3_process_result_t;

//=============================================================================
// Funções de Inicialização
//=============================================================================

/**
 * @brief Inicializa a integração ESP32-S3
 * @param integration Ponteiro para a estrutura de integração
 * @param config Configuração da integração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_integration_init(esp32s3_integration_t *integration,
                                   const esp32s3_integration_config_t *config);

/**
 * @brief Desinicializa a integração ESP32-S3
 * @param integration Ponteiro para a estrutura de integração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_integration_deinit(esp32s3_integration_t *integration);

/**
 * @brief Inicia o processamento integrado
 * @param integration Ponteiro para a estrutura de integração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_integration_start(esp32s3_integration_t *integration);

/**
 * @brief Para o processamento integrado
 * @param integration Ponteiro para a estrutura de integração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_integration_stop(esp32s3_integration_t *integration);

//=============================================================================
// Funções de Processamento Principal
//=============================================================================

/**
 * @brief Processa ciclo completo de sensores e timing
 * @param integration Ponteiro para a estrutura de integração
 * @param raw_sensor_data Dados brutos dos sensores
 * @param result Resultado do processamento
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_process_engine_cycle(esp32s3_integration_t *integration,
                                        const float *raw_sensor_data,
                                        esp32s3_process_result_t *result);

/**
 * @brief Processa sensores com otimização DSP
 * @param integration Ponteiro para a estrutura de integração
 * @param raw_map MAP bruto
 * @param raw_tps TPS bruto
 * @param raw_clt CLT bruto
 * @param raw_oil_temp Temperatura óleo bruta
 * @param raw_oil_press Pressão óleo bruta
 * @param raw_vbat Tensão bateria bruta
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_process_sensors(esp32s3_integration_t *integration,
                                   float raw_map,
                                   float raw_tps,
                                   float raw_clt,
                                   float raw_oil_temp,
                                   float raw_oil_press,
                                   float raw_vbat);

/**
 * @brief Calcula timing de injeção e ignição otimizado
 * @param integration Ponteiro para a estrutura de integração
 * @param rpm RPM atual
 * @param engine_load Carga do motor
 * @param pulse_widths Larguras de pulso desejadas
 * @param advance_angles Ângulos de avanço desejados
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_calculate_timing(esp32s3_integration_t *integration,
                                    uint16_t rpm,
                                    float engine_load,
                                    const float *pulse_widths,
                                    const float *advance_angles);

/**
 * @brief Verifica status do monitoramento ULP
 * @param integration Ponteiro para a estrutura de integração
 * @param critical_condition Condição crítica detectada
 * @param critical_channel Canal crítico
 * @param critical_value Valor crítico
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_check_ulp_status(esp32s3_integration_t *integration,
                                      bool *critical_condition,
                                      ulp_adc_channel_t *critical_channel,
                                      float *critical_value);

//=============================================================================
// Funções de Comunicação e Telemetria
//=============================================================================

/**
 * @brief Prepara mensagem de telemetria com compressão
 * @param integration Ponteiro para a estrutura de integração
 * @param output_buffer Buffer de saída
 * @param buffer_size Tamanho do buffer
 * @param compressed_size Tamanho comprimido (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_prepare_telemetry(esp32s3_integration_t *integration,
                                      uint8_t *output_buffer,
                                      uint16_t buffer_size,
                                      uint16_t *compressed_size);

/**
 * @brief Envia telemetria via ESP-NOW otimizado
 * @param integration Ponteiro para a estrutura de integração
 * @param peer_mac MAC do peer (NULL para broadcast)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_send_telemetry(esp32s3_integration_t *integration,
                                   const uint8_t *peer_mac);

/**
 * @brief Recebe configuração via ESP-NOW
 * @param integration Ponteiro para a estrutura de integração
 * @param config_data Dados de configuração recebidos
 * @param data_size Tamanho dos dados
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_receive_config(esp32s3_integration_t *integration,
                                 const uint8_t *config_data,
                                 uint16_t data_size);

//=============================================================================
// Funções de Configuração Dinâmica
//=============================================================================

/**
 * @brief Atualiza configuração de sensores
 * @param integration Ponteiro para a estrutura de integração
 * @param sensor_type Tipo de sensor
 * @param threshold Novo limiar
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_update_sensor_threshold(esp32s3_integration_t *integration,
                                           optimized_sensor_type_t sensor_type,
                                           float threshold);

/**
 * @brief Ajusta nível de compressão dinamicamente
 * @param integration Ponteiro para a estrutura de integração
 * @param level Novo nível de compressão (1-9)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_adjust_compression_level(esp32s3_integration_t *integration,
                                            uint8_t level);

/**
 * @brief Habilita/desabilita modo de economia de energia
 * @param integration Ponteiro para a estrutura de integração
 * @param enable Habilitar modo economia
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_set_power_save_mode(esp32s3_integration_t *integration,
                                       bool enable);

//=============================================================================
// Funções de Diagnóstico e Estatísticas
//=============================================================================

/**
 * @brief Obtém estatísticas de performance
 * @param integration Ponteiro para a estrutura de integração
 * @param cpu_usage Uso de CPU (%)
 * @param processing_time Tempo médio processamento (μs)
 * @param compression_ratio Taxa compressão
 * @param ulp_wakeups Despertares ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_get_performance_stats(esp32s3_integration_t *integration,
                                         float *cpu_usage,
                                         uint32_t *processing_time,
                                         float *compression_ratio,
                                         uint32_t *ulp_wakeups);

/**
 * @brief Executa diagnóstico completo do sistema
 * @param integration Ponteiro para a estrutura de integração
 * @param health_score Pontuação de saúde geral (0-100)
 * @param issues Array de problemas detectados
 * @param num_issues Número de problemas
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_run_system_diagnostics(esp32s3_integration_t *integration,
                                           float *health_score,
                                           char *issues[],
                                           uint8_t *num_issues);

/**
 * @brief Reseta estatísticas de performance
 * @param integration Ponteiro para a estrutura de integração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_reset_performance_stats(esp32s3_integration_t *integration);

/**
 * @brief Verifica se todos os componentes estão operacionais
 * @param integration Ponteiro para a estrutura de integração
 * @param all_operational Todos componentes operacionais (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_check_system_health(esp32s3_integration_t *integration,
                                        bool *all_operational);

#ifdef __cplusplus
}
#endif

#endif // ESP32S3_INTEGRATION_H
