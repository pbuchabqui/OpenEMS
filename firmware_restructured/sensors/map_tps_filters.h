/**
 * @file map_tps_filters.h
 * @brief Filtros digitais otimizados para sensores MAP e TPS usando ESP-DSP
 * 
 * Implementa filtros especializados para sensores críticos de MAP e TPS
 * aproveitando as capacidades DSP do ESP32-S3 para máxima precisão e
 * mínima latência.
 * 
 * Recursos:
 * - Filtros adaptativos para diferentes condições de operação
 * - Cancelamento de ruído específico para cada sensor
 * - Detecção de anomalias e transientes
 * - Modos de filtragem para diferentes regimes do motor
 */

#ifndef MAP_TPS_FILTERS_H
#define MAP_TPS_FILTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_sensor_processing.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Configurações e Constantes
//=============================================================================

/** @brief Taxa de amostragem padrão para MAP/TPS (Hz) */
#define MAP_TPS_SAMPLE_RATE_HZ        1000

/** @brief Tamanho do buffer para análise espectral */
#define MAP_TPS_FFT_SIZE              64

/** @brief Número de modos de operação do motor */
#define MAP_TPS_ENGINE_MODES          4

/** @brief Limiar para detecção de transiente TPS */
#define TPS_TRANSIENT_THRESHOLD       5.0f

/** @brief Limiar para detecção de pulso MAP */
#define MAP_PULSE_THRESHOLD           10.0f

/** @brief Tempo de settling para filtro (ms) */
#define MAP_TPS_SETTLING_TIME_MS      50

//=============================================================================
// Modos de Operação do Motor
//=============================================================================

typedef enum {
    MAP_TPS_MODE_IDLE = 0,        /**< Marcha lenta */
    MAP_TPS_MODE_CRUISE,          /**< Cruzeiro */
    MAP_TPS_MODE_ACCEL,           /**< Aceleração */
    MAP_TPS_MODE_DECEL,           /**< Desaceleração */
    MAP_TPS_MODE_TRANSIENT,       /**< Transiente */
    MAP_TPS_MODE_COUNT
} map_tps_engine_mode_t;

//=============================================================================
// Estruturas de Dados para MAP
//=============================================================================

/**
 * @brief Configuração do filtro MAP
 */
typedef struct {
    float cutoff_freq_idle;          /**< Frequência de corte em marcha lenta (Hz) */
    float cutoff_freq_cruise;        /**< Frequência de corte em cruzeiro (Hz) */
    float cutoff_freq_transient;     /**< Frequência de corte em transiente (Hz) */
    float noise_threshold;           /**< Limiar de ruído (kPa) */
    float pulse_detection_threshold;  /**< Limiar para detecção de pulsos */
    bool enable_pulse_detection;     /**< Habilitar detecção de pulsos */
    bool enable_adaptive_filter;     /**< Habilitar filtro adaptativo */
    bool enable_spectral_analysis;   /**< Habilitar análise espectral */
} map_filter_config_t;

/**
 * @brief Estado do filtro MAP
 */
typedef struct {
    // Filtros DSP para diferentes modos
    dsp_fir_filter_t fir_filters[MAP_TPS_ENGINE_MODES];
    dsp_iir_filter_t iir_noise_canceler;
    dsp_lms_filter_t adaptive_filter;
    
    // Buffers para processamento
    float input_buffer[MAP_TPS_FFT_SIZE];
    float output_buffer[MAP_TPS_FFT_SIZE];
    float fft_spectrum[MAP_TPS_FFT_SIZE / 2 + 1];
    float window_buffer[MAP_TPS_FFT_SIZE];
    
    // Detecção de pulsos e anomalias
    dsp_anomaly_detector_t pulse_detector;
    dsp_anomaly_detector_t anomaly_detector;
    
    // Estatísticas e estado
    float current_map;               /**< MAP atual filtrada */
    float raw_map;                   /**< MAP bruta */
    float map_rate;                  /**< Taxa de variação MAP (kPa/s) */
    float map_derivative;            /**< Derivada da MAP */
    float noise_level;               /**< Nível de ruído atual */
    float dominant_frequency;        /**< Frequência dominante no espectro */
    
    // Controle adaptativo
    map_tps_engine_mode_t current_mode;
    float adaptation_factor;
    uint32_t mode_change_time;
    bool transient_detected;
    
    // Configuração
    map_filter_config_t config;
    
    // Estado interno
    bool initialized;
    uint32_t sample_count;
    uint32_t last_update_time;
    uint32_t last_pulse_time;
} map_filter_state_t;

//=============================================================================
// Estruturas de Dados para TPS
//=============================================================================

/**
 * @brief Configuração do filtro TPS
 */
typedef struct {
    float cutoff_freq_slow;          /**< Frequência de corte lenta (Hz) */
    float cutoff_freq_fast;          /**< Frequência de corte rápida (Hz) */
    float transient_threshold;       /**< Limiar para detecção de transiente */
    float hysteresis_percent;        /**< Histerese para evitar chatter */
    float deadband_percent;          /**< Banda morta */
    bool enable_transient_detection; /**< Habilitar detecção de transientes */
    bool enable_rate_limiter;        /**< Habilitar limitador de taxa */
    bool enable_predictive_filter;   /**< Habilitar filtro preditivo */
} tps_filter_config_t;

/**
 * @brief Estado do filtro TPS
 */
typedef struct {
    // Filtros DSP
    dsp_fir_filter_t slow_filter;
    dsp_fir_filter_t fast_filter;
    dsp_iir_filter_t rate_limiter;
    dsp_lms_filter_t predictive_filter;
    
    // Buffers para processamento
    float input_buffer[MAP_TPS_FFT_SIZE];
    float output_buffer[MAP_TPS_FFT_SIZE];
    float rate_buffer[MAP_TPS_FFT_SIZE];
    
    // Detecção de transientes
    dsp_anomaly_detector_t transient_detector;
    
    // Estatísticas e estado
    float current_tps;               /**< TPS atual filtrada */
    float raw_tps;                   /**< TPS bruta */
    float tps_rate;                  /**< Taxa de variação TPS (%/s) */
    float tps_acceleration;          /**< Aceleração TPS (%/s²) */
    float predicted_tps;             /**< TPS predita */
    
    // Controle de transientes
    bool transient_active;
    uint32_t transient_start_time;
    float transient_magnitude;
    float last_stable_tps;
    
    // Controle de histerese
    float upper_threshold;
    float lower_threshold;
    
    // Configuração
    tps_filter_config_t config;
    
    // Estado interno
    bool initialized;
    uint32_t sample_count;
    uint32_t last_update_time;
    uint32_t last_transient_time;
} tps_filter_state_t;

//=============================================================================
// Estrutura Combinada
//=============================================================================

/**
 * @brief Processador combinado MAP/TPS
 */
typedef struct {
    map_filter_state_t map_filter;
    tps_filter_state_t tps_filter;
    
    // Correlação entre MAP e TPS
    float map_tps_correlation;
    float load_estimate;
    float engine_load_rate;
    
    // Detecção de padrões
    bool acceleration_pattern;
    bool deceleration_pattern;
    bool tip_in_detected;
    bool tip_out_detected;
    
    // Estatísticas combinadas
    uint32_t processing_time_us;
    float cpu_usage_percent;
    
    // Estado
    bool initialized;
    uint32_t init_time;
} map_tps_processor_t;

//=============================================================================
// Funções de Inicialização
//=============================================================================

/**
 * @brief Inicializa processador MAP/TPS
 * @param processor Ponteiro para o processador
 * @param map_config Configuração do filtro MAP
 * @param tps_config Configuração do filtro TPS
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_processor_init(map_tps_processor_t *processor,
                                 const map_filter_config_t *map_config,
                                 const tps_filter_config_t *tps_config);

/**
 * @brief Desinicializa processador MAP/TPS
 * @param processor Ponteiro para o processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_processor_deinit(map_tps_processor_t *processor);

//=============================================================================
// Funções de Processamento MAP
//=============================================================================

/**
 * @brief Processa amostra do sensor MAP
 * @param processor Ponteiro para o processador
 * @param raw_map Valor bruto do MAP (kPa)
 * @param filtered_map MAP filtrado (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_process_map(map_tps_processor_t *processor,
                              float raw_map,
                              float *filtered_map);

/**
 * @brief Detecta pulsos no sinal MAP
 * @param processor Ponteiro para o processador
 * @param pulse_detected Flag de pulso detectado (saída)
 * @param pulse_magnitude Magnitude do pulso (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_detect_map_pulse(map_tps_processor_t *processor,
                                   bool *pulse_detected,
                                   float *pulse_magnitude);

/**
 * @brief Realiza análise espectral do sinal MAP
 * @param processor Ponteiro para o processador
 * @param dominant_frequency Frequência dominante (saída)
 * @param noise_level Nível de ruído (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_analyze_map_spectrum(map_tps_processor_t *processor,
                                      float *dominant_frequency,
                                      float *noise_level);

/**
 * @brief Atualiza modo de operação do motor baseado no MAP
 * @param processor Ponteiro para o processador
 * @param rpm RPM atual
 * @param load Carga estimada
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_update_engine_mode(map_tps_processor_t *processor,
                                     uint16_t rpm,
                                     float load);

//=============================================================================
// Funções de Processamento TPS
//=============================================================================

/**
 * @brief Processa amostra do sensor TPS
 * @param processor Ponteiro para o processador
 * @param raw_tps Valor bruto do TPS (%)
 * @param filtered_tps TPS filtrado (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_process_tps(map_tps_processor_t *processor,
                              float raw_tps,
                              float *filtered_tps);

/**
 * @brief Detecta transientes no sinal TPS
 * @param processor Ponteiro para o processador
 * @param transient_active Flag de transiente ativo (saída)
 * @param transient_type Tipo de transiente (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_detect_tps_transient(map_tps_processor_t *processor,
                                        bool *transient_active,
                                        map_tps_engine_mode_t *transient_type);

/**
 * @brief Prediz próximo valor do TPS
 * @param processor Ponteiro para o processador
 * @param predicted_tps TPS predito (saída)
 * @param confidence Confiança da predição (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_predict_tps(map_tps_processor_t *processor,
                              float *predicted_tps,
                              float *confidence);

//=============================================================================
// Funções Combinadas
//=============================================================================

/**
 * @brief Processa ambos os sensores em paralelo (otimizado ESP32-S3)
 * @param processor Ponteiro para o processador
 * @param raw_map MAP bruto
 * @param raw_tps TPS bruto
 * @param filtered_map MAP filtrado (saída)
 * @param filtered_tps TPS filtrado (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_process_parallel(map_tps_processor_t *processor,
                                   float raw_map,
                                   float raw_tps,
                                   float *filtered_map,
                                   float *filtered_tps);

/**
 * @brief Calcula correlação entre MAP e TPS
 * @param processor Ponteiro para o processador
 * @param correlation Coeficiente de correlação (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_calculate_correlation(map_tps_processor_t *processor,
                                         float *correlation);

/**
 * @brief Estima carga do motor baseado em MAP e TPS
 * @param processor Ponteiro para o processador
 * @param rpm RPM atual
 * @param engine_load Carga estimada (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_estimate_engine_load(map_tps_processor_t *processor,
                                       uint16_t rpm,
                                       float *engine_load);

/**
 * @brief Detecta padrões de aceleração/desaceleração
 * @param processor Ponteiro para o processador
 * @param accel_pattern Padrão de aceleração detectado (saída)
 * @param decel_pattern Padrão de desaceleração detectado (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_detect_patterns(map_tps_processor_t *processor,
                                  bool *accel_pattern,
                                  bool *decel_pattern);

//=============================================================================
// Funções de Configuração e Diagnóstico
//=============================================================================

/**
 * @brief Configura parâmetros do filtro MAP
 * @param processor Ponteiro para o processador
 * @param config Nova configuração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_configure_map_filter(map_tps_processor_t *processor,
                                       const map_filter_config_t *config);

/**
 * @brief Configura parâmetros do filtro TPS
 * @param processor Ponteiro para o processador
 * @param config Nova configuração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_configure_tps_filter(map_tps_processor_t *processor,
                                       const tps_filter_config_t *config);

/**
 * @brief Obtém estatísticas de performance
 * @param processor Ponteiro para o processador
 * @param processing_time Tempo médio de processamento (μs)
 * @param cpu_usage Uso de CPU (%)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_get_performance_stats(map_tps_processor_t *processor,
                                         uint32_t *processing_time,
                                         float *cpu_usage);

/**
 * @brief Executa diagnóstico completo dos filtros
 * @param processor Ponteiro para o processador
 * @param map_health Saúde do filtro MAP (saída)
 * @param tps_health Saúde do filtro TPS (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_run_diagnostics(map_tps_processor_t *processor,
                                  float *map_health,
                                  float *tps_health);

/**
 * @brief Reseta estatísticas e filtros
 * @param processor Ponteiro para o processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t map_tps_reset_filters(map_tps_processor_t *processor);

#ifdef __cplusplus
}
#endif

#endif // MAP_TPS_FILTERS_H
