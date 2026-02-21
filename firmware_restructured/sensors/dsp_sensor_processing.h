/**
 * @file dsp_sensor_processing.h
 * @brief Módulo de processamento DSP otimizado para ESP32-S3
 * 
 * Aproveita as instruções vetoriais do ESP32-S3 para processamento
 * otimizado de sensores usando biblioteca ESP-DSP.
 * 
 * Recursos:
 * - Filtros digitais otimizados com DSP
 * - Processamento vetorial de múltiplos sensores
 * - Redução de ruído adaptativa
 * - Detecção de anomalias em tempo real
 */

#ifndef DSP_SENSOR_PROCESSING_H
#define DSP_SENSOR_PROCESSING_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_dsp.h"
#include "dsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Configurações e Constantes
//=============================================================================

/** @brief Número máximo de amostras para processamento vetorial */
#define DSP_MAX_SAMPLES           64

/** @brief Número de canais de sensores suportados */
#define DSP_MAX_CHANNELS          8

/** @brief Tamanho do buffer circular para cada sensor */
#define DSP_BUFFER_SIZE           32

/** @brief Frequência de amostragem padrão (Hz) */
#define DSP_SAMPLE_RATE_HZ        1000

/** @brief Taxa de aprendizado para filtros adaptativos */
#define DSP_ADAPTIVE_LEARNING_RATE  0.01f

//=============================================================================
// Estruturas de Dados
//=============================================================================

/**
 * @brief Estrutura para filtro FIR otimizado com DSP
 */
typedef struct {
    float *coeffs;              /**< Coeficientes do filtro */
    float *delay_line;          /**< Linha de atraso circular */
    uint16_t num_coeffs;        /**< Número de coeficientes */
    uint16_t delay_index;       /**< Índice atual na linha de atraso */
    bool initialized;           /**< Estado de inicialização */
} dsp_fir_filter_t;

/**
 * @brief Estrutura para filtro IIR otimizado com DSP
 */
typedef struct {
    float *b_coeffs;            /**< Coeficientes do numerador */
    float *a_coeffs;            /**< Coeficientes do denominador */
    float *x_history;           /**< História de entrada */
    float *y_history;           /**< História de saída */
    uint16_t num_b_coeffs;      /**< Número de coeficientes B */
    uint16_t num_a_coeffs;      /**< Número de coeficientes A */
    bool initialized;           /**< Estado de inicialização */
} dsp_iir_filter_t;

/**
 * @brief Estrutura para filtro adaptativo LMS
 */
typedef struct {
    float *weights;             /**< Pesos adaptativos */
    float *input_buffer;        /**< Buffer de entrada */
    float error;                /**< Erro atual */
    float learning_rate;        /**< Taxa de aprendizado */
    uint16_t filter_length;     /**< Comprimento do filtro */
    uint16_t buffer_index;      /**< Índice do buffer */
    bool initialized;           /**< Estado de inicialização */
} dsp_lms_filter_t;

/**
 * @brief Estrutura para processamento vetorial de sensores
 */
typedef struct {
    // Buffers para processamento vetorial
    float input_buffer[DSP_MAX_CHANNELS][DSP_MAX_SAMPLES];
    float output_buffer[DSP_MAX_CHANNELS][DSP_MAX_SAMPLES];
    float window_buffer[DSP_MAX_SAMPLES];
    
    // Filtros para cada canal
    dsp_fir_filter_t fir_filters[DSP_MAX_CHANNELS];
    dsp_iir_filter_t iir_filters[DSP_MAX_CHANNELS];
    dsp_lms_filter_t lms_filters[DSP_MAX_CHANNELS];
    
    // Configurações
    uint8_t num_channels;       /**< Número de canais ativos */
    uint16_t sample_rate;      /**< Frequência de amostragem */
    uint16_t buffer_size;       /**< Tamanho do buffer de processamento */
    
    // Estatísticas
    float signal_power[DSP_MAX_CHANNELS];
    float noise_power[DSP_MAX_CHANNELS];
    float snr_db[DSP_MAX_CHANNELS];
    uint32_t sample_count;
    
    // Estado
    bool initialized;
    bool processing_enabled;
} dsp_sensor_processor_t;

/**
 * @brief Estrutura para detecção de anomalias
 */
typedef struct {
    float threshold;            /**< Limiar de detecção */
    float window_mean;          /**< Média da janela */
    float window_std;           /**< Desvio padrão da janela */
    uint16_t window_size;       /**< Tamanho da janela de análise */
    uint16_t anomaly_count;     /**< Contador de anomalias */
    bool anomaly_detected;      /**< Flag de anomalia detectada */
} dsp_anomaly_detector_t;

//=============================================================================
// Tipos de Canais de Sensores
//=============================================================================

typedef enum {
    DSP_CHANNEL_MAP = 0,        /**< Sensor MAP */
    DSP_CHANNEL_TPS,            /**< Sensor TPS */
    DSP_CHANNEL_CLT,            /**< Sensor de temperatura do líquido de arrefecimento */
    DSP_CHANNEL_IAT,            /**< Sensor de temperatura do ar de admissão */
    DSP_CHANNEL_O2,             /**< Sensor O2 */
    DSP_CHANNEL_VBAT,           /**< Tensão da bateria */
    DSP_CHANNEL_KNOCK,         /**< Sensor knock */
    DSP_CHANNEL_FLEX,          /**< Sensor flex fuel */
    DSP_CHANNEL_COUNT
} dsp_sensor_channel_t;

//=============================================================================
// Funções de Inicialização
//=============================================================================

/**
 * @brief Inicializa o módulo de processamento DSP
 * @param processor Ponteiro para a estrutura do processador
 * @param num_channels Número de canais de sensores
 * @param sample_rate Frequência de amostragem em Hz
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_sensor_processor_init(dsp_sensor_processor_t *processor, 
                                   uint8_t num_channels, 
                                   uint16_t sample_rate);

/**
 * @brief Desinicializa o módulo de processamento DSP
 * @param processor Ponteiro para a estrutura do processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_sensor_processor_deinit(dsp_sensor_processor_t *processor);

/**
 * @brief Inicializa filtro FIR para um canal específico
 * @param filter Ponteiro para a estrutura do filtro
 * @param coeffs Array de coeficientes
 * @param num_coeffs Número de coeficientes
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_fir_filter_init(dsp_fir_filter_t *filter, 
                              const float *coeffs, 
                              uint16_t num_coeffs);

/**
 * @brief Inicializa filtro IIR para um canal específico
 * @param filter Ponteiro para a estrutura do filtro
 * @param b_coeffs Coeficientes do numerador
 * @param num_b_coeffs Número de coeficientes B
 * @param a_coeffs Coeficientes do denominador
 * @param num_a_coeffs Número de coeficientes A
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_iir_filter_init(dsp_iir_filter_t *filter,
                              const float *b_coeffs, uint16_t num_b_coeffs,
                              const float *a_coeffs, uint16_t num_a_coeffs);

/**
 * @brief Inicializa filtro adaptativo LMS
 * @param filter Ponteiro para a estrutura do filtro
 * @param filter_length Comprimento do filtro
 * @param learning_rate Taxa de aprendizado
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_lms_filter_init(dsp_lms_filter_t *filter,
                              uint16_t filter_length,
                              float learning_rate);

//=============================================================================
// Funções de Processamento
//=============================================================================

/**
 * @brief Processa amostra bruta de um sensor com filtragem DSP
 * @param processor Ponteiro para o processador
 * @param channel Canal do sensor
 * @param raw_value Valor bruto do sensor
 * @param filtered_value Valor filtrado de saída
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_process_sensor_sample(dsp_sensor_processor_t *processor,
                                   dsp_sensor_channel_t channel,
                                   float raw_value,
                                   float *filtered_value);

/**
 * @brief Processa múltiplas amostras vetorialmente (otimizado ESP32-S3)
 * @param processor Ponteiro para o processador
 * @param input_samples Array de amostras de entrada
 * @param output_samples Array de amostras de saída
 * @param num_samples Número de amostras
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_process_samples_vectorized(dsp_sensor_processor_t *processor,
                                         const float *input_samples,
                                         float *output_samples,
                                         uint16_t num_samples);

/**
 * @brief Aplica filtro FIR a uma amostra
 * @param filter Ponteiro para o filtro
 * @param input Amostra de entrada
 * @return Amostra filtrada
 */
float dsp_fir_filter_process(dsp_fir_filter_t *filter, float input);

/**
 * @brief Aplica filtro IIR a uma amostra
 * @param filter Ponteiro para o filtro
 * @param input Amostra de entrada
 * @return Amostra filtrada
 */
float dsp_iir_filter_process(dsp_iir_filter_t *filter, float input);

/**
 * @brief Aplica filtro adaptativo LMS
 * @param filter Ponteiro para o filtro
 * @param input Amostra de entrada
 * @param desired Sinal desejado
 * @return Amostra filtrada
 */
float dsp_lms_filter_process(dsp_lms_filter_t *filter, float input, float desired);

//=============================================================================
// Funções de Análise e Detecção
//=============================================================================

/**
 * @brief Calcula estatísticas do sinal para um canal
 * @param processor Ponteiro para o processador
 * @param channel Canal do sensor
 * @param mean Média do sinal
 * @param std_dev Desvio padrão
 * @param rms Valor RMS
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_calculate_signal_stats(dsp_sensor_processor_t *processor,
                                    dsp_sensor_channel_t channel,
                                    float *mean,
                                    float *std_dev,
                                    float *rms);

/**
 * @brief Detecta anomalias no sinal do sensor
 * @param detector Ponteiro para o detector de anomalias
 * @param sample Amostra atual
 * @return true se anomalia detectada
 */
bool dsp_detect_anomaly(dsp_anomaly_detector_t *detector, float sample);

/**
 * @brief Calcula relação sinal/ruído (SNR)
 * @param processor Ponteiro para o processador
 * @param channel Canal do sensor
 * @return SNR em dB
 */
float dsp_calculate_snr(dsp_sensor_processor_t *processor, dsp_sensor_channel_t channel);

//=============================================================================
// Funções de Otimização para ESP32-S3
//=============================================================================

/**
 * @brief Configura processamento vetorial otimizado para ESP32-S3
 * @param processor Ponteiro para o processador
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_enable_vectorized_processing(dsp_sensor_processor_t *processor);

/**
 * @brief Aplica janelamento de Hamming para análise espectral
 * @param processor Ponteiro para o processador
 * @param samples Array de amostras
 * @param num_samples Número de amostras
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_apply_hamming_window(dsp_sensor_processor_t *processor,
                                  float *samples,
                                  uint16_t num_samples);

/**
 * @brief Realiza FFT otimizada para análise de frequência
 * @param processor Ponteiro para o processador
 * @param input_samples Amostras de entrada
 * @param output_fft Saída da FFT
 * @param fft_size Tamanho da FFT (potência de 2)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_perform_fft(dsp_sensor_processor_t *processor,
                         const float *input_samples,
                         float *output_fft,
                         uint16_t fft_size);

//=============================================================================
// Funções de Configuração
//=============================================================================

/**
 * @brief Configura taxa de amostragem
 * @param processor Ponteiro para o processador
 * @param sample_rate Nova frequência de amostragem
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_set_sample_rate(dsp_sensor_processor_t *processor, uint16_t sample_rate);

/**
 * @brief Habilita/desabilita processamento DSP
 * @param processor Ponteiro para o processador
 * @param enabled Estado do processamento
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_set_processing_enabled(dsp_sensor_processor_t *processor, bool enabled);

/**
 * @brief Obtém estatísticas de processamento
 * @param processor Ponteiro para o processador
 * @param cpu_usage_percent Uso de CPU (%)
 * @param processing_time_us Tempo de processamento (μs)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t dsp_get_processing_stats(dsp_sensor_processor_t *processor,
                                  float *cpu_usage_percent,
                                  uint32_t *processing_time_us);

#ifdef __cplusplus
}
#endif

#endif // DSP_SENSOR_PROCESSING_H
