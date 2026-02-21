/**
 * @file ulp_monitor.h
 * @brief Módulo de monitoramento ULP para sensores críticos
 * 
 * Implementa monitoramento contínuo de temperatura e pressão usando o
 * Ultra Low Power (ULP) coprocessor do ESP32-S3, permitindo detecção
 * de condições críticas mesmo durante modo deep-sleep.
 * 
 * Recursos:
 * - Monitoramento de temperatura do motor e óleo
 * - Monitoramento de pressão de óleo
 * - Detecção de sobretemperatura e baixa pressão
 * - Operação independente do CPU principal
 * - Consumo de energia mínimo (~10μA)
 */

#ifndef ULP_MONITOR_H
#define ULP_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_sleep.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Configurações e Constantes
//=============================================================================

/** @brief Frequência de amostragem ULP (Hz) */
#define ULP_SAMPLE_RATE_HZ           1

/** @brief Número de amostras para média */
#define ULP_AVERAGE_SAMPLES         16

/** @brief Limiar de temperatura crítica (°C) */
#define ULP_CRITICAL_TEMP_C         105.0f

/** @brief Limiar de baixa pressão de óleo (kPa) */
#define ULP_LOW_OIL_PRESSURE_KPA    100.0f

/** @brief Tempo entre verificações (segundos) */
#define ULP_CHECK_INTERVAL_S        5

/** @brief Endereço base na RTC slow memory para dados ULP */
#define ULP_DATA_BASE_ADDR          0x50000000

//=============================================================================
// Canais ADC para ULP
//=============================================================================

typedef enum {
    ULP_ADC_CHANNEL_CLT = 3,       /**< Temperatura do líquido de arrefecimento */
    ULP_ADC_CHANNEL_OIL_TEMP = 4,   /**< Temperatura do óleo */
    ULP_ADC_CHANNEL_OIL_PRESS = 5,  /**< Pressão do óleo */
    ULP_ADC_CHANNEL_VBAT = 6,       /**< Tensão da bateria */
    ULP_ADC_CHANNEL_COUNT
} ulp_adc_channel_t;

//=============================================================================
// Estruturas de Dados
//=============================================================================

/**
 * @brief Configuração do sensor ULP
 */
typedef struct {
    ulp_adc_channel_t adc_channel;  /**< Canal ADC */
    float min_value;                /**< Valor mínimo esperado */
    float max_value;                /**< Valor máximo esperado */
    float critical_threshold;       /**< Limiar crítico */
    float warning_threshold;        /**< Limiar de aviso */
    bool enable_monitoring;        /**< Habilitar monitoramento */
    bool enable_wake_on_critical;  /**< Acordar CPU em condição crítica */
    uint32_t sample_interval_ms;   /**< Intervalo de amostragem */
} ulp_sensor_config_t;

/**
 * @brief Dados compartilhados entre ULP e CPU
 * 
 * Esta estrutura reside na RTC slow memory e é acessível
 * tanto pelo ULP quanto pelo CPU principal.
 */
typedef struct {
    // Dados brutos dos sensores
    uint16_t clt_raw;               /**< CLT ADC raw value */
    uint16_t oil_temp_raw;          /**< Oil temp ADC raw value */
    uint16_t oil_press_raw;         /**< Oil pressure ADC raw value */
    uint16_t vbat_raw;              /**< Battery ADC raw value */
    
    // Valores convertidos
    float clt_celsius;              /**< CLT em Celsius */
    float oil_temp_celsius;         /**< Oil temp em Celsius */
    float oil_pressure_kpa;         /**< Oil pressure em kPa */
    float battery_voltage;          /**< Battery voltage */
    
    // Médias móveis
    float clt_avg;                  /**< Média CLT */
    float oil_temp_avg;             /**< Média oil temp */
    float oil_pressure_avg;         /**< Média oil pressure */
    float battery_voltage_avg;      /**< Média battery voltage */
    
    // Contadores e estatísticas
    uint32_t sample_count;          /**< Total de amostras */
    uint32_t critical_events;       /**< Eventos críticos */
    uint32_t warning_events;        /**< Eventos de aviso */
    uint32_t last_critical_time;    /**< Timestamp último evento crítico */
    uint32_t last_warning_time;     /**< Timestamp último aviso */
    
    // Flags de estado
    bool clt_critical;              /**< CLT crítica */
    bool oil_temp_critical;         /**< Oil temp crítica */
    bool oil_pressure_critical;     /**< Oil pressure crítica */
    bool battery_critical;          /**< Battery crítica */
    bool any_warning;               /**< Qualquer aviso ativo */
    bool any_critical;              /**< Qualquer condição crítica */
    bool cpu_wake_requested;        /**< CPU acordada pelo ULP */
    
    // Configuração
    uint32_t sample_interval;       /**< Intervalo de amostragem (ms) */
    uint16_t clt_critical_raw;      /**< CLT limiar crítico (raw) */
    uint16_t oil_temp_critical_raw; /**< Oil temp limiar crítico (raw) */
    uint16_t oil_press_critical_raw;/**< Oil pressure limiar crítico (raw) */
    uint16_t vbat_critical_raw;     /**< Battery limiar crítico (raw) */
    
    // Reservado para expansão
    uint16_t reserved[8];
} ulp_shared_data_t;

/**
 * @brief Contexto principal do monitor ULP
 */
typedef struct {
    ulp_shared_data_t *shared_data; /**< Ponteiro para dados compartilhados */
    ulp_sensor_config_t sensors[ULP_ADC_CHANNEL_COUNT]; /**< Configurações */
    
    // Estado do monitoramento
    bool ulp_running;               /**< ULP em execução */
    bool deep_sleep_enabled;        /**< Deep sleep habilitado */
    uint32_t program_start_time;    /**< Timestamp de início */
    
    // Estatísticas
    uint32_t total_wakeups;         /**< Total de despertares */
    uint32_t critical_wakeups;      /**< Despertares por condição crítica */
    uint32_t scheduled_wakeups;     /**< Despertares agendados */
    float avg_sleep_duration;       /**< Duração média do sleep */
    
    // Callbacks
    void (*critical_callback)(ulp_adc_channel_t channel, float value);
    void (*warning_callback)(ulp_adc_channel_t channel, float value);
    void (*status_callback)(const ulp_shared_data_t *data);
    
    // Estado interno
    bool initialized;               /**< Estado de inicialização */
} ulp_monitor_context_t;

/**
 * @brief Resultado da verificação ULP
 */
typedef struct {
    bool monitoring_active;         /**< Monitoramento ativo */
    bool critical_condition;        /**< Condição crítica detectada */
    bool warning_condition;         /**< Condição de aviso detectada */
    ulp_adc_channel_t critical_channel; /**< Canal crítico */
    ulp_adc_channel_t warning_channel;  /**< Canal de aviso */
    float critical_value;           /**< Valor crítico */
    float warning_value;            /**< Valor de aviso */
    uint32_t uptime_seconds;        /**< Uptime em segundos */
    uint32_t samples_since_wakeup;  /**< Amostras desde último despertar */
} ulp_check_result_t;

//=============================================================================
// Funções de Inicialização
//=============================================================================

/**
 * @brief Inicializa o monitoramento ULP
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_init(ulp_monitor_context_t *ctx);

/**
 * @brief Desinicializa o monitoramento ULP
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_deinit(ulp_monitor_context_t *ctx);

/**
 * @brief Configura um sensor para monitoramento ULP
 * @param ctx Ponteiro para o contexto ULP
 * @param channel Canal do sensor
 * @param config Configuração do sensor
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_configure_sensor(ulp_monitor_context_t *ctx,
                                       ulp_adc_channel_t channel,
                                       const ulp_sensor_config_t *config);

/**
 * @brief Configura limiares críticos para todos os sensores
 * @param ctx Ponteiro para o contexto ULP
 * @param clt_critical Temperatura crítica do líquido (°C)
 * @param oil_temp_critical Temperatura crítica do óleo (°C)
 * @param oil_pressure_critical Pressão crítica do óleo (kPa)
 * @param vbat_critical Tensão crítica da bateria (V)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_set_critical_thresholds(ulp_monitor_context_t *ctx,
                                              float clt_critical,
                                              float oil_temp_critical,
                                              float oil_pressure_critical,
                                              float vbat_critical);

//=============================================================================
// Funções de Controle
//=============================================================================

/**
 * @brief Inicia o monitoramento ULP
 * @param ctx Ponteiro para o contexto ULP
 * @param enable_deep_sleep Habilitar deep sleep entre amostragens
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_start(ulp_monitor_context_t *ctx, bool enable_deep_sleep);

/**
 * @brief Para o monitoramento ULP
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_stop(ulp_monitor_context_t *ctx);

/**
 * @brief Pausa o monitoramento ULP temporariamente
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_pause(ulp_monitor_context_t *ctx);

/**
 * @brief Retoma o monitoramento ULP
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_resume(ulp_monitor_context_t *ctx);

//=============================================================================
// Funções de Verificação e Leitura
//=============================================================================

/**
 * @brief Verifica status do monitoramento ULP
 * @param ctx Ponteiro para o contexto ULP
 * @param result Resultado da verificação
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_check_status(ulp_monitor_context_t *ctx,
                                    ulp_check_result_t *result);

/**
 * @brief Obtém dados compartilhados do ULP
 * @param ctx Ponteiro para o contexto ULP
 * @param data Dados compartilhados (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_get_shared_data(ulp_monitor_context_t *ctx,
                                      ulp_shared_data_t *data);

/**
 * @brief Lê valor atual de um sensor
 * @param ctx Ponteiro para o contexto ULP
 * @param channel Canal do sensor
 * @param value Valor atual (saída)
 * @param average Valor médio (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_read_sensor(ulp_monitor_context_t *ctx,
                                  ulp_adc_channel_t channel,
                                  float *value,
                                  float *average);

/**
 * @brief Força amostragem imediata de todos os sensores
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_force_sample(ulp_monitor_context_t *ctx);

//=============================================================================
// Funções de Configuração Avançada
//=============================================================================

/**
 * @brief Configura intervalo de amostragem
 * @param ctx Ponteiro para o contexto ULP
 * @param interval_ms Intervalo em milissegundos
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_set_sample_interval(ulp_monitor_context_t *ctx,
                                          uint32_t interval_ms);

/**
 * @brief Configura número de amostras para média
 * @param ctx Ponteiro para o contexto ULP
 * @param num_samples Número de amostras
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_set_average_samples(ulp_monitor_context_t *ctx,
                                          uint16_t num_samples);

/**
 * @brief Habilita/desabilita despertar CPU em condição crítica
 * @param ctx Ponteiro para o contexto ULP
 * @param channel Canal do sensor
 * @param enable Habilitar despertar
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_set_wake_on_critical(ulp_monitor_context_t *ctx,
                                            ulp_adc_channel_t channel,
                                            bool enable);

/**
 * @brief Configura callback para eventos críticos
 * @param ctx Ponteiro para o contexto ULP
 * @param callback Função de callback
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_set_critical_callback(ulp_monitor_context_t *ctx,
                                             void (*callback)(ulp_adc_channel_t, float));

/**
 * @brief Configura callback para eventos de aviso
 * @param ctx Ponteiro para o contexto ULP
 * @param callback Função de callback
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_set_warning_callback(ulp_monitor_context_t *ctx,
                                            void (*callback)(ulp_adc_channel_t, float));

/**
 * @brief Configura callback de status periódico
 * @param ctx Ponteiro para o contexto ULP
 * @param callback Função de callback
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_set_status_callback(ulp_monitor_context_t *ctx,
                                          void (*callback)(const ulp_shared_data_t *));

//=============================================================================
// Funções de Diagnóstico e Estatísticas
//=============================================================================

/**
 * @brief Obtém estatísticas do monitoramento
 * @param ctx Ponteiro para o contexto ULP
 * @param total_wakeups Total de despertares
 * @param critical_wakeups Despertares críticos
 * @param avg_sleep_duration Duração média do sleep
 * @param cpu_usage_percent Uso de CPU (%)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_get_statistics(ulp_monitor_context_t *ctx,
                                      uint32_t *total_wakeups,
                                      uint32_t *critical_wakeups,
                                      float *avg_sleep_duration,
                                      float *cpu_usage_percent);

/**
 * @brief Executa diagnóstico completo do sistema ULP
 * @param ctx Ponteiro para o contexto ULP
 * @param health_score Pontuação de saúde (0-100)
 * @param issues Array de problemas detectados
 * @param num_issues Número de problemas
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_run_diagnostics(ulp_monitor_context_t *ctx,
                                       float *health_score,
                                       char *issues[],
                                       uint8_t *num_issues);

/**
 * @brief Reseta estatísticas e contadores
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_reset_statistics(ulp_monitor_context_t *ctx);

/**
 * @brief Verifica se ULP está operacional
 * @param ctx Ponteiro para o contexto ULP
 * @param operational ULP operacional (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_is_operational(ulp_monitor_context_t *ctx, bool *operational);

//=============================================================================
// Funções de Calibração
//=============================================================================

/**
 * @brief Inicia calibração automática de sensores
 * @param ctx Ponteiro para o contexto ULP
 * @param duration_s Duração da calibração (segundos)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_start_calibration(ulp_monitor_context_t *ctx, uint32_t duration_s);

/**
 * @brief Finaliza calibração e aplica coeficientes
 * @param ctx Ponteiro para o contexto ULP
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_finish_calibration(ulp_monitor_context_t *ctx);

/**
 * @brief Aplica coeficientes de calibração manualmente
 * @param ctx Ponteiro para o contexto ULP
 * @param channel Canal do sensor
 * @param offset Offset de calibração
 * @param scale Escala de calibração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t ulp_monitor_apply_calibration(ulp_monitor_context_t *ctx,
                                        ulp_adc_channel_t channel,
                                        float offset,
                                        float scale);

#ifdef __cplusplus
}
#endif

#endif // ULP_MONITOR_H
