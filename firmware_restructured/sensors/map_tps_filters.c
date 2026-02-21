/**
 * @file map_tps_filters.c
 * @brief Implementação dos filtros digitais otimizados para MAP e TPS
 */

#include "map_tps_filters.h"
#include "esp_log.h"
#include "hal/hal_timer.h"
#include <math.h>
#include <string.h>
#include <assert.h>

static const char* TAG = "MAP_TPS_FILTERS";

// Declarações de funções estáticas
static esp_err_t map_tps_init_map_filter(map_filter_state_t *map_filter);
static esp_err_t map_tps_init_tps_filter(tps_filter_state_t *tps_filter);
static void map_tps_update_correlation(map_tps_processor_t *processor);

//=============================================================================
// Coeficientes de Filtros Pré-calculados
//=============================================================================

// Coeficientes FIR para diferentes frequências de corte (1kHz sample rate)
static const float fir_coeffs_5hz[] = {
    0.000944f, 0.001888f, 0.003776f, 0.007552f, 0.015104f,
    0.030208f, 0.060416f, 0.120832f, 0.241664f, 0.120832f,
    0.060416f, 0.030208f, 0.015104f, 0.007552f, 0.003776f,
    0.001888f, 0.000944f
};

static const float fir_coeffs_20hz[] = {
    0.003776f, 0.007552f, 0.015104f, 0.030208f, 0.060416f,
    0.120832f, 0.241664f, 0.120832f, 0.060416f, 0.030208f,
    0.015104f, 0.007552f, 0.003776f
};

static const float fir_coeffs_50hz[] = {
    0.00944f, 0.01888f, 0.03776f, 0.07552f, 0.15104f,
    0.30208f, 0.15104f, 0.07552f, 0.03776f, 0.01888f, 0.00944f
};

// Coeficientes IIR para cancelamento de ruído
static const float iir_noise_b[] = {1.0f, -1.9f, 0.9f};
static const float iir_noise_a[] = {1.0f, -1.8f, 0.81f};

//=============================================================================
// Funções de Inicialização
//=============================================================================

esp_err_t map_tps_processor_init(map_tps_processor_t *processor,
                                 const map_filter_config_t *map_config,
                                 const tps_filter_config_t *tps_config) {
    if (processor == NULL || map_config == NULL || tps_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limpar estrutura
    memset(processor, 0, sizeof(map_tps_processor_t));
    
    // Copiar configurações
    processor->map_filter.config = *map_config;
    processor->tps_filter.config = *tps_config;
    
    // Inicializar filtro MAP
    esp_err_t ret = map_tps_init_map_filter(&processor->map_filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MAP filter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar filtro TPS
    ret = map_tps_init_tps_filter(&processor->tps_filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TPS filter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar janela de Hamming para análise espectral
    for (uint16_t i = 0; i < MAP_TPS_FFT_SIZE; i++) {
        processor->map_filter.window_buffer[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (MAP_TPS_FFT_SIZE - 1));
    }
    
    processor->initialized = true;
    processor->init_time = HAL_Time_us();
    
    ESP_LOGI(TAG, "MAP/TPS processor initialized successfully");
    ESP_LOGI(TAG, "  MAP cutoff frequencies: %.1f/%.1f/%.1f Hz", 
              map_config->cutoff_freq_idle, map_config->cutoff_freq_cruise, map_config->cutoff_freq_transient);
    ESP_LOGI(TAG, "  TPS cutoff frequencies: %.1f/%.1f Hz", 
              tps_config->cutoff_freq_slow, tps_config->cutoff_freq_fast);
    
    return ESP_OK;
}

esp_err_t map_tps_init_map_filter(map_filter_state_t *map_filter) {
    if (map_filter == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Inicializar filtros FIR para diferentes modos
    esp_err_t ret = dsp_fir_filter_init(&map_filter->fir_filters[MAP_TPS_MODE_IDLE], 
                                        fir_coeffs_5hz, sizeof(fir_coeffs_5hz) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    ret = dsp_fir_filter_init(&map_filter->fir_filters[MAP_TPS_MODE_CRUISE], 
                             fir_coeffs_20hz, sizeof(fir_coeffs_20hz) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    ret = dsp_fir_filter_init(&map_filter->fir_filters[MAP_TPS_MODE_ACCEL], 
                             fir_coeffs_50hz, sizeof(fir_coeffs_50hz) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    ret = dsp_fir_filter_init(&map_filter->fir_filters[MAP_TPS_MODE_DECEL], 
                             fir_coeffs_50hz, sizeof(fir_coeffs_50hz) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    // Inicializar filtro IIR para cancelamento de ruído
    ret = dsp_iir_filter_init(&map_filter->iir_noise_canceler,
                              iir_noise_b, sizeof(iir_noise_b) / sizeof(float),
                              iir_noise_a, sizeof(iir_noise_a) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    // Inicializar filtro adaptativo LMS
    ret = dsp_lms_filter_init(&map_filter->adaptive_filter, 16, 0.01f);
    if (ret != ESP_OK) return ret;
    
    // Inicializar detectores de anomalias
    map_filter->pulse_detector.threshold = map_filter->config.pulse_detection_threshold;
    map_filter->pulse_detector.window_size = 16;
    map_filter->anomaly_detector.threshold = map_filter->config.noise_threshold * 3.0f;
    map_filter->anomaly_detector.window_size = 32;
    
    map_filter->current_mode = MAP_TPS_MODE_IDLE;
    map_filter->adaptation_factor = 0.1f;
    map_filter->initialized = true;
    
    return ESP_OK;
}

esp_err_t map_tps_init_tps_filter(tps_filter_state_t *tps_filter) {
    if (tps_filter == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Inicializar filtros FIR
    esp_err_t ret = dsp_fir_filter_init(&tps_filter->slow_filter, 
                                        fir_coeffs_5hz, sizeof(fir_coeffs_5hz) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    ret = dsp_fir_filter_init(&tps_filter->fast_filter, 
                             fir_coeffs_50hz, sizeof(fir_coeffs_50hz) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    // Inicializar filtro limitador de taxa
    ret = dsp_iir_filter_init(&tps_filter->rate_limiter,
                              iir_noise_b, sizeof(iir_noise_b) / sizeof(float),
                              iir_noise_a, sizeof(iir_noise_a) / sizeof(float));
    if (ret != ESP_OK) return ret;
    
    // Inicializar filtro preditivo
    ret = dsp_lms_filter_init(&tps_filter->predictive_filter, 8, 0.05f);
    if (ret != ESP_OK) return ret;
    
    // Inicializar detector de transientes
    tps_filter->transient_detector.threshold = tps_filter->config.transient_threshold;
    tps_filter->transient_detector.window_size = 8;
    
    // Configurar histerese
    tps_filter->upper_threshold = tps_filter->config.transient_threshold;
    tps_filter->lower_threshold = -tps_filter->config.transient_threshold;
    
    tps_filter->last_stable_tps = 0.0f;
    tps_filter->initialized = true;
    
    return ESP_OK;
}

//=============================================================================
// Funções de Processamento MAP
//=============================================================================

esp_err_t map_tps_process_map(map_tps_processor_t *processor,
                              float raw_map,
                              float *filtered_map) {
    if (processor == NULL || !processor->initialized || filtered_map == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t start_time = HAL_Time_us();
    map_filter_state_t *map_filter = &processor->map_filter;
    
    // Armazenar valor bruto
    map_filter->raw_map = raw_map;
    
    // Aplicar filtro FIR baseado no modo atual
    float fir_output = dsp_fir_filter_process(&map_filter->fir_filters[map_filter->current_mode], raw_map);
    
    // Aplicar cancelamento de ruído
    float noise_cancelled = dsp_iir_filter_process(&map_filter->iir_noise_canceler, fir_output);
    
    // Aplicar filtro adaptativo se habilitado
    float adaptive_output = noise_cancelled;
    if (map_filter->config.enable_adaptive_filter) {
        adaptive_output = dsp_lms_filter_process(&map_filter->adaptive_filter, noise_cancelled, noise_cancelled);
    }
    
    // Detectar anomalias
    bool anomaly_detected = dsp_detect_anomaly(&map_filter->anomaly_detector, adaptive_output);
    if (anomaly_detected) {
        map_filter->anomaly_detector.anomaly_count++;
        ESP_LOGW(TAG, "MAP anomaly detected: %.2f kPa", adaptive_output);
    }
    
    // Atualizar estatísticas
    map_filter->current_map = adaptive_output;
    map_filter->sample_count++;
    
    // Calcular taxa de variação
    uint32_t current_time = (uint32_t)HAL_Time_us();
    if (map_filter->last_update_time > 0) {
        float dt = (current_time - map_filter->last_update_time) / 1000000.0f; // Converter para segundos
        if (dt > 0) {
            map_filter->map_rate = (adaptive_output - map_filter->current_map) / dt;
        }
    }
    map_filter->last_update_time = current_time;
    
    *filtered_map = adaptive_output;
    
    // Atualizar estatísticas de performance
    uint64_t end_time = HAL_Time_us();
    processor->processing_time_us += (uint32_t)(end_time - start_time);
    
    return ESP_OK;
}

esp_err_t map_tps_detect_map_pulse(map_tps_processor_t *processor,
                                   bool *pulse_detected,
                                   float *pulse_magnitude) {
    if (processor == NULL || !processor->initialized || 
        pulse_detected == NULL || pulse_magnitude == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    map_filter_state_t *map_filter = &processor->map_filter;
    
    // Detectar pulso baseado na taxa de variação
    bool pulse = fabsf(map_filter->map_rate) > map_filter->config.pulse_detection_threshold;
    float magnitude = fabsf(map_filter->map_rate);
    
    if (pulse) {
        uint32_t current_time = (uint32_t)HAL_Time_us();
        if (current_time - map_filter->last_pulse_time > 10000) { // Mínimo 10ms entre pulsos
            map_filter->last_pulse_time = current_time;
            ESP_LOGD(TAG, "MAP pulse detected: %.2f kPa/s", magnitude);
        } else {
            pulse = false; // Ignorar pulsos muito próximos
        }
    }
    
    *pulse_detected = pulse;
    *pulse_magnitude = magnitude;
    
    return ESP_OK;
}

esp_err_t map_tps_analyze_map_spectrum(map_tps_processor_t *processor,
                                      float *dominant_frequency,
                                      float *noise_level) {
    if (processor == NULL || !processor->initialized || 
        dominant_frequency == NULL || noise_level == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    map_filter_state_t *map_filter = &processor->map_filter;
    
    // Preparar dados para FFT
    for (uint16_t i = 0; i < MAP_TPS_FFT_SIZE; i++) {
        map_filter->input_buffer[i] = map_filter->output_buffer[i] * map_filter->window_buffer[i];
    }
    
    // Realizar FFT
    esp_err_t ret = dsp_perform_fft(NULL,  // Usar processador DSP padrão
                                    map_filter->input_buffer,
                                    map_filter->fft_spectrum,
                                    MAP_TPS_FFT_SIZE);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Encontrar frequência dominante (ignorando DC)
    float max_magnitude = 0.0f;
    uint16_t max_index = 1; // Ignorar componente DC (índice 0)
    
    for (uint16_t i = 1; i < MAP_TPS_FFT_SIZE / 2 + 1; i++) {
        if (map_filter->fft_spectrum[i] > max_magnitude) {
            max_magnitude = map_filter->fft_spectrum[i];
            max_index = i;
        }
    }
    
    // Converter índice para frequência
    *dominant_frequency = (float)max_index * MAP_TPS_SAMPLE_RATE_HZ / MAP_TPS_FFT_SIZE;
    
    // Calcular nível de ruído (média das frequências altas)
    float noise_sum = 0.0f;
    uint16_t noise_start = MAP_TPS_FFT_SIZE / 4; // 25% da frequência de Nyquist
    for (uint16_t i = noise_start; i < MAP_TPS_FFT_SIZE / 2 + 1; i++) {
        noise_sum += map_filter->fft_spectrum[i];
    }
    *noise_level = noise_sum / (MAP_TPS_FFT_SIZE / 2 + 1 - noise_start);
    
    map_filter->dominant_frequency = *dominant_frequency;
    map_filter->noise_level = *noise_level;
    
    return ESP_OK;
}

esp_err_t map_tps_update_engine_mode(map_tps_processor_t *processor,
                                     uint16_t rpm,
                                     float load) {
    if (processor == NULL || !processor->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    map_filter_state_t *map_filter = &processor->map_filter;
    map_tps_engine_mode_t new_mode;
    
    // Lógica para determinar modo de operação
    if (rpm < 1200 && load < 0.3f) {
        new_mode = MAP_TPS_MODE_IDLE;
    } else if (rpm > 3000 && load > 0.7f) {
        new_mode = MAP_TPS_MODE_ACCEL;
    } else if (map_filter->map_rate > 50.0f) {
        new_mode = MAP_TPS_MODE_TRANSIENT;
    } else if (map_filter->map_rate < -30.0f) {
        new_mode = MAP_TPS_MODE_DECEL;
    } else {
        new_mode = MAP_TPS_MODE_CRUISE;
    }
    
    // Atualizar modo se mudou
    if (new_mode != map_filter->current_mode) {
        map_filter->current_mode = new_mode;
        map_filter->mode_change_time = (uint32_t)HAL_Time_us();
        
        ESP_LOGD(TAG, "MAP filter mode changed to %d", new_mode);
    }
    
    return ESP_OK;
}

//=============================================================================
// Funções de Processamento TPS
//=============================================================================

esp_err_t map_tps_process_tps(map_tps_processor_t *processor,
                              float raw_tps,
                              float *filtered_tps) {
    if (processor == NULL || !processor->initialized || filtered_tps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t start_time = HAL_Time_us();
    tps_filter_state_t *tps_filter = &processor->tps_filter;
    
    // Armazenar valor bruto
    tps_filter->raw_tps = raw_tps;
    
    // Aplicar filtro lento
    float slow_output = dsp_fir_filter_process(&tps_filter->slow_filter, raw_tps);
    
    // Aplicar filtro rápido
    float fast_output = dsp_fir_filter_process(&tps_filter->fast_filter, raw_tps);
    
    // Detectar transientes
    float tps_rate = (fast_output - slow_output) * MAP_TPS_SAMPLE_RATE_HZ;
    bool transient = fabsf(tps_rate) > tps_filter->config.transient_threshold;
    
    // Selecionar saída baseado na presença de transientes
    float output = transient ? fast_output : slow_output;
    
    // Aplicar limitador de taxa se habilitado
    if (tps_filter->config.enable_rate_limiter) {
        output = dsp_iir_filter_process(&tps_filter->rate_limiter, output);
    }
    
    // Aplicar filtro preditivo se habilitado
    if (tps_filter->config.enable_predictive_filter) {
        tps_filter->predicted_tps = dsp_lms_filter_process(&tps_filter->predictive_filter, output, output);
    }
    
    // Atualizar estado de transiente
    if (transient && !tps_filter->transient_active) {
        // Início do transiente
        tps_filter->transient_active = true;
        tps_filter->transient_start_time = (uint32_t)HAL_Time_us();
        tps_filter->transient_magnitude = fabsf(tps_rate);
        tps_filter->last_stable_tps = tps_filter->current_tps;
    } else if (!transient && tps_filter->transient_active) {
        // Fim do transiente
        tps_filter->transient_active = false;
        uint32_t transient_duration = (uint32_t)HAL_Time_us() - tps_filter->transient_start_time;
        ESP_LOGD(TAG, "TPS transient ended: duration=%d ms, magnitude=%.2f %%/s", 
                  transient_duration, tps_filter->transient_magnitude);
    }
    
    // Atualizar estatísticas
    tps_filter->current_tps = output;
    tps_filter->tps_rate = tps_rate;
    tps_filter->sample_count++;
    
    // Calcular aceleração TPS
    if (tps_filter->last_update_time > 0) {
        float dt = ((uint32_t)HAL_Time_us() - tps_filter->last_update_time) / 1000000.0f;
        if (dt > 0) {
            tps_filter->tps_acceleration = (tps_rate - tps_filter->tps_rate) / dt;
        }
    }
    tps_filter->last_update_time = (uint32_t)HAL_Time_us();
    
    *filtered_tps = output;
    
    // Atualizar estatísticas de performance
    uint64_t end_time = HAL_Time_us();
    processor->processing_time_us += (uint32_t)(end_time - start_time);
    
    return ESP_OK;
}

esp_err_t map_tps_detect_tps_transient(map_tps_processor_t *processor,
                                        bool *transient_active,
                                        map_tps_engine_mode_t *transient_type) {
    if (processor == NULL || !processor->initialized || 
        transient_active == NULL || transient_type == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    tps_filter_state_t *tps_filter = &processor->tps_filter;
    
    *transient_active = tps_filter->transient_active;
    
    if (tps_filter->transient_active) {
        if (tps_filter->tps_rate > 0) {
            *transient_type = MAP_TPS_MODE_ACCEL;
        } else {
            *transient_type = MAP_TPS_MODE_DECEL;
        }
    } else {
        *transient_type = MAP_TPS_MODE_CRUISE;
    }
    
    return ESP_OK;
}

esp_err_t map_tps_predict_tps(map_tps_processor_t *processor,
                              float *predicted_tps,
                              float *confidence) {
    if (processor == NULL || !processor->initialized || 
        predicted_tps == NULL || confidence == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    tps_filter_state_t *tps_filter = &processor->tps_filter;
    
    if (!tps_filter->config.enable_predictive_filter) {
        *predicted_tps = tps_filter->current_tps;
        *confidence = 0.5f;
        return ESP_OK;
    }
    
    *predicted_tps = tps_filter->predicted_tps;
    
    // Calcular confiança baseada na estabilidade recente
    float stability = 1.0f / (1.0f + fabsf(tps_filter->tps_rate));
    *confidence = fmaxf(0.1f, fminf(1.0f, stability));
    
    return ESP_OK;
}

//=============================================================================
// Funções Combinadas
//=============================================================================

esp_err_t map_tps_process_parallel(map_tps_processor_t *processor,
                                   float raw_map,
                                   float raw_tps,
                                   float *filtered_map,
                                   float *filtered_tps) {
    if (processor == NULL || !processor->initialized || 
        filtered_map == NULL || filtered_tps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Processar ambos os sensores em paralelo (otimizado para ESP32-S3)
    uint64_t start_time = HAL_Time_us();
    
    // Processar MAP
    esp_err_t ret = map_tps_process_map(processor, raw_map, filtered_map);
    if (ret != ESP_OK) return ret;
    
    // Processar TPS
    ret = map_tps_process_tps(processor, raw_tps, filtered_tps);
    if (ret != ESP_OK) return ret;
    
    // Calcular correlação e carga estimada
    map_tps_update_correlation(processor);
    
    uint64_t end_time = HAL_Time_us();
    processor->processing_time_us += (uint32_t)(end_time - start_time);
    
    return ESP_OK;
}

esp_err_t map_tps_calculate_correlation(map_tps_processor_t *processor,
                                         float *correlation) {
    if (processor == NULL || !processor->initialized || correlation == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calcular correlação simples entre MAP e TPS normalizados
    float map_norm = processor->map_filter.current_map / 250.0f; // Normalizar para 0-1
    float tps_norm = processor->tps_filter.current_tps / 100.0f;  // Normalizar para 0-1
    
    // Correlação simples (pode ser expandida para correlação de Pearson)
    processor->map_tps_correlation = 1.0f - fabsf(map_norm - tps_norm);
    *correlation = processor->map_tps_correlation;
    
    return ESP_OK;
}

esp_err_t map_tps_estimate_engine_load(map_tps_processor_t *processor,
                                       uint16_t rpm,
                                       float *engine_load) {
    if (processor == NULL || !processor->initialized || engine_load == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Estimar carga baseada em MAP e TPS
    float map_load = processor->map_filter.current_map / 250.0f; // 0-250 kPa range
    float tps_load = processor->tps_filter.current_tps / 100.0f;  // 0-100% range
    
    // Ponderar MAP mais que TPS para estimativa de carga
    processor->load_estimate = (map_load * 0.7f) + (tps_load * 0.3f);
    processor->load_estimate = fmaxf(0.0f, fminf(1.0f, processor->load_estimate));
    
    *engine_load = processor->load_estimate;
    
    return ESP_OK;
}

//=============================================================================
// Funções Utilitárias
//=============================================================================

void map_tps_update_correlation(map_tps_processor_t *processor) {
    if (processor == NULL || !processor->initialized) {
        return;
    }
    
    // Atualizar taxa de variação da carga
    static float last_load = 0.0f;
    static uint32_t last_time = 0;
    
    uint32_t current_time = (uint32_t)HAL_Time_us();
    if (last_time > 0) {
        float dt = (current_time - last_time) / 1000000.0f;
        if (dt > 0) {
            processor->engine_load_rate = (processor->load_estimate - last_load) / dt;
        }
    }
    
    last_load = processor->load_estimate;
    last_time = current_time;
}

esp_err_t map_tps_get_performance_stats(map_tps_processor_t *processor,
                                         uint32_t *processing_time,
                                         float *cpu_usage) {
    if (processor == NULL || !processor->initialized || 
        processing_time == NULL || cpu_usage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t total_samples = processor->map_filter.sample_count + processor->tps_filter.sample_count;
    
    if (total_samples > 0) {
        *processing_time = processor->processing_time_us / total_samples;
        *cpu_usage = (*processing_time * MAP_TPS_SAMPLE_RATE_HZ) / 10000.0f; // Porcentagem
    } else {
        *processing_time = 0;
        *cpu_usage = 0.0f;
    }
    
    return ESP_OK;
}

esp_err_t map_tps_reset_filters(map_tps_processor_t *processor) {
    if (processor == NULL || !processor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Resetar estatísticas
    processor->processing_time_us = 0;
    processor->cpu_usage_percent = 0.0f;
    
    // Resetar filtros MAP
    memset(&processor->map_filter.input_buffer, 0, sizeof(processor->map_filter.input_buffer));
    memset(&processor->map_filter.output_buffer, 0, sizeof(processor->map_filter.output_buffer));
    processor->map_filter.sample_count = 0;
    processor->map_filter.last_update_time = 0;
    
    // Resetar filtros TPS
    memset(&processor->tps_filter.input_buffer, 0, sizeof(processor->tps_filter.input_buffer));
    processor->tps_filter.sample_count = 0;
    processor->tps_filter.last_update_time = 0;
    
    ESP_LOGI(TAG, "MAP/TPS filters reset");
    
    return ESP_OK;
}
