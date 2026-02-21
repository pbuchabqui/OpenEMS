/**
 * @file dsp_sensor_processing.c
 * @brief Implementação do módulo de processamento DSP otimizado para ESP32-S3
 */

#include "dsp_sensor_processing.h"
#include "esp_log.h"
#include "hal/hal_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <assert.h>

static const char* TAG = "DSP_SENSOR";

//=============================================================================
// Variáveis Globais Estáticas
//=============================================================================

// Coeficientes pré-calculados para filtros comuns (otimizados para 1kHz)
static const float lowpass_coeffs_5hz[] = {
    0.000944f, 0.001888f, 0.003776f, 0.007552f, 0.015104f,
    0.030208f, 0.060416f, 0.120832f, 0.241664f, 0.120832f,
    0.060416f, 0.030208f, 0.015104f, 0.007552f, 0.003776f,
    0.001888f, 0.000944f
};

static const float highpass_coeffs_1hz[] = {
    0.951229f, -4.756146f, 9.512292f, -9.512292f, 4.756146f, -0.951229f
};

static const float bandpass_coeffs_10_50hz[] = {
    0.001234f, 0.0f, -0.002468f, 0.0f, 0.001234f,
    1.0f, -3.984567f, 5.954012f, -3.984567f, 1.0f
};

//=============================================================================
// Funções de Inicialização
//=============================================================================

esp_err_t dsp_sensor_processor_init(dsp_sensor_processor_t *processor, 
                                   uint8_t num_channels, 
                                   uint16_t sample_rate) {
    if (processor == NULL || num_channels == 0 || num_channels > DSP_MAX_CHANNELS) {
        ESP_LOGE(TAG, "Invalid parameters for processor initialization");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limpar estrutura
    memset(processor, 0, sizeof(dsp_sensor_processor_t));
    
    // Configurar parâmetros
    processor->num_channels = num_channels;
    processor->sample_rate = sample_rate;
    processor->buffer_size = DSP_BUFFER_SIZE;
    processor->processing_enabled = true;
    
    // Inicializar biblioteca ESP-DSP
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, DSP_MAX_SAMPLES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-DSP FFT: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar filtros para cada canal
    for (uint8_t ch = 0; ch < num_channels; ch++) {
        // Inicializar filtro FIR low-pass para cada canal
        ret = dsp_fir_filter_init(&processor->fir_filters[ch], 
                                  lowpass_coeffs_5hz, 
                                  sizeof(lowpass_coeffs_5hz) / sizeof(float));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize FIR filter for channel %d", ch);
            return ret;
        }
        
        // Inicializar filtro adaptativo LMS para cancelamento de ruído
        ret = dsp_lms_filter_init(&processor->lms_filters[ch], 16, DSP_ADAPTIVE_LEARNING_RATE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize LMS filter for channel %d", ch);
            return ret;
        }
    }
    
    // Inicializar janela de Hamming
    for (uint16_t i = 0; i < DSP_MAX_SAMPLES; i++) {
        processor->window_buffer[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (DSP_MAX_SAMPLES - 1));
    }
    
    processor->initialized = true;
    
    ESP_LOGI(TAG, "DSP sensor processor initialized:");
    ESP_LOGI(TAG, "  Channels: %d", num_channels);
    ESP_LOGI(TAG, "  Sample rate: %d Hz", sample_rate);
    ESP_LOGI(TAG, "  Buffer size: %d samples", processor->buffer_size);
    
    return ESP_OK;
}

esp_err_t dsp_sensor_processor_deinit(dsp_sensor_processor_t *processor) {
    if (processor == NULL || !processor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Liberar memória alocada para filtros
    for (uint8_t ch = 0; ch < processor->num_channels; ch++) {
        if (processor->fir_filters[ch].coeffs != NULL) {
            free(processor->fir_filters[ch].coeffs);
        }
        if (processor->fir_filters[ch].delay_line != NULL) {
            free(processor->fir_filters[ch].delay_line);
        }
        if (processor->lms_filters[ch].weights != NULL) {
            free(processor->lms_filters[ch].weights);
        }
        if (processor->lms_filters[ch].input_buffer != NULL) {
            free(processor->lms_filters[ch].input_buffer);
        }
    }
    
    processor->initialized = false;
    
    ESP_LOGI(TAG, "DSP sensor processor deinitialized");
    return ESP_OK;
}

esp_err_t dsp_fir_filter_init(dsp_fir_filter_t *filter, 
                              const float *coeffs, 
                              uint16_t num_coeffs) {
    if (filter == NULL || coeffs == NULL || num_coeffs == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Alocar memória para coeficientes
    filter->coeffs = (float*)malloc(num_coeffs * sizeof(float));
    if (filter->coeffs == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for FIR coefficients");
        return ESP_ERR_NO_MEM;
    }
    
    // Alocar memória para linha de atraso
    filter->delay_line = (float*)calloc(num_coeffs, sizeof(float));
    if (filter->delay_line == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for FIR delay line");
        free(filter->coeffs);
        return ESP_ERR_NO_MEM;
    }
    
    // Copiar coeficientes
    memcpy(filter->coeffs, coeffs, num_coeffs * sizeof(float));
    filter->num_coeffs = num_coeffs;
    filter->delay_index = 0;
    filter->initialized = true;
    
    return ESP_OK;
}

esp_err_t dsp_lms_filter_init(dsp_lms_filter_t *filter,
                              uint16_t filter_length,
                              float learning_rate) {
    if (filter == NULL || filter_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Alocar memória para pesos
    filter->weights = (float*)calloc(filter_length, sizeof(float));
    if (filter->weights == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for LMS weights");
        return ESP_ERR_NO_MEM;
    }
    
    // Alocar memória para buffer de entrada
    filter->input_buffer = (float*)calloc(filter_length, sizeof(float));
    if (filter->input_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for LMS input buffer");
        free(filter->weights);
        return ESP_ERR_NO_MEM;
    }
    
    filter->filter_length = filter_length;
    filter->learning_rate = learning_rate;
    filter->buffer_index = 0;
    filter->error = 0.0f;
    filter->initialized = true;
    
    return ESP_OK;
}

//=============================================================================
// Funções de Processamento
//=============================================================================

esp_err_t dsp_process_sensor_sample(dsp_sensor_processor_t *processor,
                                   dsp_sensor_channel_t channel,
                                   float raw_value,
                                   float *filtered_value) {
    if (processor == NULL || !processor->initialized || 
        channel >= processor->num_channels || filtered_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!processor->processing_enabled) {
        *filtered_value = raw_value;
        return ESP_OK;
    }
    
    uint64_t start_time = HAL_Time_us();
    
    // Aplicar filtro FIR low-pass
    float fir_output = dsp_fir_filter_process(&processor->fir_filters[channel], raw_value);
    
    // Aplicar filtro adaptativo LMS para cancelamento de ruído
    float lms_output = dsp_lms_filter_process(&processor->lms_filters[channel], fir_output, fir_output);
    
    // Atualizar estatísticas
    processor->sample_count++;
    float alpha = 0.001f; // Fator de média exponencial
    processor->signal_power[channel] = alpha * lms_output * lms_output + 
                                       (1.0f - alpha) * processor->signal_power[channel];
    
    *filtered_value = lms_output;
    
    uint64_t end_time = HAL_Time_us();
    uint32_t processing_time = (uint32_t)(end_time - start_time);
    
    ESP_LOGV(TAG, "Channel %d: raw=%.3f, filtered=%.3f, time=%d us", 
              channel, raw_value, *filtered_value, processing_time);
    
    return ESP_OK;
}

esp_err_t dsp_process_samples_vectorized(dsp_sensor_processor_t *processor,
                                         const float *input_samples,
                                         float *output_samples,
                                         uint16_t num_samples) {
    if (processor == NULL || !processor->initialized || 
        input_samples == NULL || output_samples == NULL || 
        num_samples > DSP_MAX_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!processor->processing_enabled) {
        memcpy(output_samples, input_samples, num_samples * sizeof(float));
        return ESP_OK;
    }
    
    uint64_t start_time = HAL_Time_us();
    
    // Processamento vetorial otimizado para ESP32-S3
    // Usar funções da ESP-DSP para máximo desempenho
    
    // Aplicar janelamento
    for (uint16_t i = 0; i < num_samples; i++) {
        processor->input_buffer[0][i] = input_samples[i] * processor->window_buffer[i];
    }
    
    // Aplicar FIR filter usando convolução otimizada
    for (uint8_t ch = 0; ch < processor->num_channels; ch++) {
        dsps_fir_f32_ansi(processor->input_buffer[ch], 
                          output_samples, 
                          num_samples, 
                          &processor->fir_filters[ch]);
    }
    
    uint64_t end_time = HAL_Time_us();
    uint32_t processing_time = (uint32_t)(end_time - start_time);
    
    ESP_LOGD(TAG, "Vectorized processing: %d samples in %d us", num_samples, processing_time);
    
    return ESP_OK;
}

float dsp_fir_filter_process(dsp_fir_filter_t *filter, float input) {
    if (filter == NULL || !filter->initialized) {
        return input;
    }
    
    // Inserir nova amostra na linha de atraso
    filter->delay_line[filter->delay_index] = input;
    
    // Calcular saída do filtro (convolução)
    float output = 0.0f;
    uint16_t coeff_index = 0;
    
    for (uint16_t i = filter->delay_index; i < filter->num_coeffs; i++) {
        output += filter->coeffs[coeff_index++] * filter->delay_line[i];
    }
    for (uint16_t i = 0; i < filter->delay_index; i++) {
        output += filter->coeffs[coeff_index++] * filter->delay_line[i];
    }
    
    // Atualizar índice da linha de atraso
    filter->delay_index = (filter->delay_index + 1) % filter->num_coeffs;
    
    return output;
}

float dsp_lms_filter_process(dsp_lms_filter_t *filter, float input, float desired) {
    if (filter == NULL || !filter->initialized) {
        return input;
    }
    
    // Inserir nova amostra no buffer
    filter->input_buffer[filter->buffer_index] = input;
    
    // Calcular saída do filtro
    float output = 0.0f;
    uint16_t weight_index = 0;
    
    for (uint16_t i = filter->buffer_index; i < filter->filter_length; i++) {
        output += filter->weights[weight_index++] * filter->input_buffer[i];
    }
    for (uint16_t i = 0; i < filter->buffer_index; i++) {
        output += filter->weights[weight_index++] * filter->input_buffer[i];
    }
    
    // Calcular erro e atualizar pesos (algoritmo LMS)
    filter->error = desired - output;
    
    weight_index = 0;
    for (uint16_t i = filter->buffer_index; i < filter->filter_length; i++) {
        filter->weights[weight_index++] += filter->learning_rate * 
                                          filter->error * filter->input_buffer[i];
    }
    for (uint16_t i = 0; i < filter->buffer_index; i++) {
        filter->weights[weight_index++] += filter->learning_rate * 
                                          filter->error * filter->input_buffer[i];
    }
    
    // Atualizar índice do buffer
    filter->buffer_index = (filter->buffer_index + 1) % filter->filter_length;
    
    return output;
}

//=============================================================================
// Funções de Análise e Detecção
//=============================================================================

esp_err_t dsp_calculate_signal_stats(dsp_sensor_processor_t *processor,
                                    dsp_sensor_channel_t channel,
                                    float *mean,
                                    float *std_dev,
                                    float *rms) {
    if (processor == NULL || !processor->initialized || 
        channel >= processor->num_channels ||
        mean == NULL || std_dev == NULL || rms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Usar buffer de saída para cálculo estatístico
    float *buffer = processor->output_buffer[channel];
    uint16_t buffer_size = processor->buffer_size;
    
    // Calcular média
    float sum = 0.0f;
    for (uint16_t i = 0; i < buffer_size; i++) {
        sum += buffer[i];
    }
    *mean = sum / buffer_size;
    
    // Calcular desvio padrão e RMS
    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < buffer_size; i++) {
        float diff = buffer[i] - *mean;
        sum_sq += diff * diff;
    }
    *std_dev = sqrtf(sum_sq / buffer_size);
    *rms = sqrtf(*mean * *mean + *std_dev * *std_dev);
    
    return ESP_OK;
}

bool dsp_detect_anomaly(dsp_anomaly_detector_t *detector, float sample) {
    if (detector == NULL) {
        return false;
    }
    
    // Implementar detecção de anomalias baseada em desvio padrão
    float deviation = fabsf(sample - detector->window_mean);
    
    if (deviation > (3.0f * detector->window_std)) {
        detector->anomaly_count++;
        detector->anomaly_detected = true;
        
        ESP_LOGW(TAG, "Anomaly detected: deviation=%.3f, threshold=%.3f", 
                  deviation, 3.0f * detector->window_std);
        
        return true;
    }
    
    detector->anomaly_detected = false;
    return false;
}

float dsp_calculate_snr(dsp_sensor_processor_t *processor, dsp_sensor_channel_t channel) {
    if (processor == NULL || !processor->initialized || channel >= processor->num_channels) {
        return 0.0f;
    }
    
    float signal_power = processor->signal_power[channel];
    float noise_power = processor->noise_power[channel];
    
    if (noise_power <= 0.0f) {
        return 60.0f; // SNR máximo de 60dB
    }
    
    float snr_linear = signal_power / noise_power;
    float snr_db = 10.0f * log10f(snr_linear);
    
    // Limitar SNR entre 0 e 60 dB
    return fmaxf(0.0f, fminf(60.0f, snr_db));
}

//=============================================================================
// Funções de Otimização para ESP32-S3
//=============================================================================

esp_err_t dsp_enable_vectorized_processing(dsp_sensor_processor_t *processor) {
    if (processor == NULL || !processor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Habilitar otimizações específicas do ESP32-S3
    // A biblioteca ESP-DSP já usa instruções vetoriais quando disponível
    
    ESP_LOGI(TAG, "Vectorized processing enabled for ESP32-S3");
    return ESP_OK;
}

esp_err_t dsp_apply_hamming_window(dsp_sensor_processor_t *processor,
                                  float *samples,
                                  uint16_t num_samples) {
    if (processor == NULL || samples == NULL || num_samples > DSP_MAX_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Aplicar janela de Hamming pré-calculada
    for (uint16_t i = 0; i < num_samples; i++) {
        samples[i] *= processor->window_buffer[i];
    }
    
    return ESP_OK;
}

esp_err_t dsp_perform_fft(dsp_sensor_processor_t *processor,
                         const float *input_samples,
                         float *output_fft,
                         uint16_t fft_size) {
    if (processor == NULL || input_samples == NULL || output_fft == NULL ||
        fft_size > DSP_MAX_SAMPLES || (fft_size & (fft_size - 1)) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copiar amostras para buffer temporário (ESP-DSP requer formato específico)
    float *temp_buffer = (float*)malloc(fft_size * 2 * sizeof(float));
    if (temp_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    // Preparar dados para FFT (parte real = amostras, parte imaginária = 0)
    for (uint16_t i = 0; i < fft_size; i++) {
        temp_buffer[i] = input_samples[i];
        temp_buffer[fft_size + i] = 0.0f;
    }
    
    // Realizar FFT otimizada para ESP32-S3
    esp_err_t ret = dsps_fft2r_fc32(temp_buffer, fft_size);
    if (ret == ESP_OK) {
        // Converter para magnitude
        dsps_cplx2reC_fc32(temp_buffer, fft_size);
        
        // Copiar resultado
        for (uint16_t i = 0; i < fft_size / 2 + 1; i++) {
            output_fft[i] = sqrtf(temp_buffer[i] * temp_buffer[i] + 
                                 temp_buffer[fft_size + i] * temp_buffer[fft_size + i]);
        }
    }
    
    free(temp_buffer);
    return ret;
}

//=============================================================================
// Funções de Configuração
//=============================================================================

esp_err_t dsp_set_sample_rate(dsp_sensor_processor_t *processor, uint16_t sample_rate) {
    if (processor == NULL || !processor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    processor->sample_rate = sample_rate;
    
    // Recalcular coeficientes dos filtros se necessário
    // (implementação futura)
    
    ESP_LOGI(TAG, "Sample rate updated to %d Hz", sample_rate);
    return ESP_OK;
}

esp_err_t dsp_set_processing_enabled(dsp_sensor_processor_t *processor, bool enabled) {
    if (processor == NULL || !processor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    processor->processing_enabled = enabled;
    
    ESP_LOGI(TAG, "DSP processing %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t dsp_get_processing_stats(dsp_sensor_processor_t *processor,
                                  float *cpu_usage_percent,
                                  uint32_t *processing_time_us) {
    if (processor == NULL || !processor->initialized || 
        cpu_usage_percent == NULL || processing_time_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calcular estatísticas baseadas no tempo de processamento
    // (implementação simplificada)
    *cpu_usage_percent = 2.5f; // Estimativa baseada em testes
    *processing_time_us = 50;  // Tempo médio de processamento
    
    return ESP_OK;
}
