/**
 * @file esp32s3_integration.c
 * @brief Implementação do módulo de integração ESP32-S3
 */

#include "esp32s3_integration.h"
#include "esp_log.h"
#include "hal/hal_timer.h"
#include "espnow_link.h"
#include <string.h>
#include <math.h>

static const char* TAG = "ESP32S3_INTEGRATION";

//=============================================================================
// Funções Estáticas Internas
//=============================================================================

static esp_err_t esp32s3_init_dsp_components(esp32s3_integration_t *integration);
static esp_err_t esp32s3_init_ulp_monitoring(esp32s3_integration_t *integration);
static esp_err_t esp32s3_init_compression(esp32s3_integration_t *integration);
static esp_err_t esp32s3_init_vector_math(esp32s3_integration_t *integration);
static void esp32s3_update_performance_metrics(esp32s3_integration_t *integration);

//=============================================================================
// Funções de Inicialização
//=============================================================================

esp_err_t esp32s3_integration_init(esp32s3_integration_t *integration,
                                   const esp32s3_integration_config_t *config) {
    if (integration == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing ESP32-S3 competitive improvements integration v%d", 
              ESP32S3_INTEGRATION_VERSION);
    
    // Limpar estrutura
    memset(integration, 0, sizeof(esp32s3_integration_t));
    
    // Copiar configuração
    integration->config = *config;
    
    // Validar configuração
    if (config->num_cylinders == 0 || config->num_cylinders > ESP32S3_MAX_CYLINDERS) {
        ESP_LOGE(TAG, "Invalid number of cylinders: %d", config->num_cylinders);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ESP_OK;
    
    // Inicializar componentes DSP
    if (config->enable_dsp_processing) {
        ret = esp32s3_init_dsp_components(integration);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize DSP components: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Inicializar monitoramento ULP
    if (config->enable_ulp_monitoring) {
        ret = esp32s3_init_ulp_monitoring(integration);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ULP monitoring: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Inicializar compressão
    if (config->enable_compression) {
        ret = esp32s3_init_compression(integration);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize compression: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Inicializar matemática vetorial
    if (config->enable_vector_timing) {
        ret = esp32s3_init_vector_math(integration);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize vector math: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    integration->initialized = true;
    integration->start_time = HAL_Time_us();
    
    ESP_LOGI(TAG, "ESP32-S3 integration initialized successfully");
    ESP_LOGI(TAG, "  DSP processing: %s", config->enable_dsp_processing ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  MAP/TPS optimization: %s", config->enable_map_tps_optimization ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  ULP monitoring: %s", config->enable_ulp_monitoring ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  Compression: %s", config->enable_compression ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  Vector timing: %s", config->enable_vector_timing ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  Cylinders: %d", config->num_cylinders);
    
    return ESP_OK;
}

static esp_err_t esp32s3_init_dsp_components(esp32s3_integration_t *integration) {
    esp_err_t ret;
    
    // Inicializar processador DSP
    ret = dsp_sensor_processor_init(&integration->dsp_processor, 
                                    OPTIMIZED_SENSOR_COUNT, 
                                    integration->config.dsp_sample_rate);
    if (ret != ESP_OK) return ret;
    
    // Inicializar processador MAP/TPS
    map_filter_config_t map_config = {
        .cutoff_freq_idle = 5.0f,
        .cutoff_freq_cruise = 20.0f,
        .cutoff_freq_transient = 50.0f,
        .noise_threshold = 2.0f,
        .pulse_detection_threshold = 10.0f,
        .enable_pulse_detection = true,
        .enable_adaptive_filter = integration->config.enable_adaptive_filters,
        .enable_spectral_analysis = true
    };
    
    tps_filter_config_t tps_config = {
        .cutoff_freq_slow = 5.0f,
        .cutoff_freq_fast = 50.0f,
        .transient_threshold = 5.0f,
        .hysteresis_percent = 2.0f,
        .deadband_percent = 1.0f,
        .enable_transient_detection = true,
        .enable_rate_limiter = true,
        .enable_predictive_filter = true
    };
    
    ret = map_tps_processor_init(&integration->map_tps_processor, &map_config, &tps_config);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "DSP components initialized");
    return ESP_OK;
}

static esp_err_t esp32s3_init_ulp_monitoring(esp32s3_integration_t *integration) {
    esp_err_t ret;
    
    ret = ulp_monitor_init(&integration->ulp_monitor);
    if (ret != ESP_OK) return ret;
    
    // Configurar limiares críticos
    ret = ulp_monitor_set_critical_thresholds(&integration->ulp_monitor,
                                               integration->config.map_critical_temp,
                                               120.0f, // Oil temp
                                               integration->config.oil_pressure_critical,
                                               10.0f);  // Battery voltage
    if (ret != ESP_OK) return ret;
    
    // Configurar intervalo de amostragem
    ret = ulp_monitor_set_sample_interval(&integration->ulp_monitor,
                                          integration->config.ulp_sample_interval);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "ULP monitoring initialized");
    return ESP_OK;
}

static esp_err_t esp32s3_init_compression(esp32s3_integration_t *integration) {
    espnow_compress_config_t comp_config = {
        .type = integration->config.compression_type,
        .level = integration->config.compression_level,
        .quantization_bits = 10,
        .enable_adaptive = true,
        .use_simd = true,
        .min_size = 64,
        .compression_ratio_target = 0.4f
    };
    
    esp_err_t ret = espnow_compress_init(&integration->compression_context, &comp_config);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Compression initialized");
    return ESP_OK;
}

static esp_err_t esp32s3_init_vector_math(esp32s3_integration_t *integration) {
    esp_err_t ret = vector_math_init(&integration->vector_context);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Vector math initialized");
    return ESP_OK;
}

esp_err_t esp32s3_integration_start(esp32s3_integration_t *integration) {
    if (integration == NULL || !integration->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_OK;
    
    // Iniciar monitoramento ULP
    if (integration->config.enable_ulp_monitoring) {
        ret = ulp_monitor_start(&integration->ulp_monitor, integration->config.enable_deep_sleep);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start ULP monitoring: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    integration->running = true;
    integration->last_update_time = (uint32_t)HAL_Time_us();
    
    ESP_LOGI(TAG, "ESP32-S3 integration started");
    return ESP_OK;
}

esp_err_t esp32s3_integration_deinit(esp32s3_integration_t *integration) {
    if (integration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Parar processamento
    esp32s3_integration_stop(integration);
    
    // Desinicializar componentes
    if (integration->config.enable_dsp_processing) {
        dsp_sensor_processor_deinit(&integration->dsp_processor);
    }
    
    if (integration->config.enable_ulp_monitoring) {
        ulp_monitor_deinit(&integration->ulp_monitor);
    }
    
    if (integration->config.enable_compression) {
        espnow_compress_deinit(&integration->compression_context);
    }
    
    if (integration->config.enable_vector_timing) {
        vector_math_deinit(&integration->vector_context);
    }
    
    integration->initialized = false;
    
    ESP_LOGI(TAG, "ESP32-S3 integration deinitialized");
    return ESP_OK;
}

//=============================================================================
// Funções de Processamento Principal
//=============================================================================

esp_err_t esp32s3_process_engine_cycle(esp32s3_integration_t *integration,
                                        const float *raw_sensor_data,
                                        esp32s3_process_result_t *result) {
    if (integration == NULL || !integration->initialized || 
        raw_sensor_data == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t start_time = HAL_Time_us();
    
    // Resetar resultado
    memset(result, 0, sizeof(esp32s3_process_result_t));
    
    esp_err_t ret = ESP_OK;
    
    // Processar sensores
    if (integration->config.enable_dsp_processing && integration->config.enable_map_tps_optimization) {
        ret = esp32s3_process_sensors(integration,
                                      raw_sensor_data[0], // MAP
                                      raw_sensor_data[1], // TPS
                                      raw_sensor_data[2], // CLT
                                      raw_sensor_data[3], // Oil temp
                                      raw_sensor_data[4], // Oil pressure
                                      raw_sensor_data[5]); // Battery
        if (ret != ESP_OK) return ret;
        
        result->sensors_processed = 6;
    }
    
    // Verificar status ULP
    if (integration->config.enable_ulp_monitoring) {
        ret = esp32s3_check_ulp_status(integration,
                                         &result->critical_condition,
                                         NULL, &integration->data.critical_value);
        if (ret != ESP_OK) return ret;
    }
    
    // Calcular timing
    if (integration->config.enable_vector_timing) {
        // Simular cálculos de timing
        for (uint8_t i = 0; i < integration->config.num_cylinders; i++) {
            integration->data.injection_pw[i] = 2000.0f + (i * 100.0f); // Simulação
            integration->data.ignition_advance[i] = 20.0f + (i * 2.0f); // Simulação
        }
        
        ret = esp32s3_calculate_timing(integration,
                                        integration->data.current_rpm,
                                        integration->data.engine_load,
                                        integration->data.injection_pw,
                                        integration->data.ignition_advance);
        if (ret != ESP_OK) return ret;
        
        result->calculations_performed = integration->config.num_cylinders * 2; // injection + ignition
    }
    
    // Atualizar métricas
    integration->data.processing_time_us = (uint32_t)(HAL_Time_us() - start_time);
    result->processing_time_us = integration->data.processing_time_us;
    
    esp32s3_update_performance_metrics(integration);
    result->cpu_usage = integration->data.cpu_usage_percent;
    result->success = true;
    
    integration->update_count++;
    integration->last_update_time = (uint32_t)HAL_Time_us();
    
    ESP_LOGV(TAG, "Engine cycle processed: %d sensors, %d calculations, %d us", 
              result->sensors_processed, result->calculations_performed, result->processing_time_us);
    
    return ESP_OK;
}

esp_err_t esp32s3_process_sensors(esp32s3_integration_t *integration,
                                   float raw_map,
                                   float raw_tps,
                                   float raw_clt,
                                   float raw_oil_temp,
                                   float raw_oil_press,
                                   float raw_vbat) {
    if (integration == NULL || !integration->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    // Processar MAP e TPS com filtros otimizados
    if (integration->config.enable_map_tps_optimization) {
        ret = map_tps_process_parallel(&integration->map_tps_processor,
                                       raw_map, raw_tps,
                                       &integration->data.map_filtered,
                                       &integration->data.tps_filtered);
        if (ret != ESP_OK) return ret;
        
        // Estimar carga do motor
        ret = map_tps_estimate_engine_load(&integration->map_tps_processor,
                                            integration->data.current_rpm,
                                            &integration->data.engine_load);
        if (ret != ESP_OK) return ret;
        
        // Calcular correlação
        ret = map_tps_calculate_correlation(&integration->map_tps_processor,
                                             &integration->data.map_tps_correlation);
        if (ret != ESP_OK) return ret;
        
        // Detectar padrões
        bool accel_pattern, decel_pattern;
        ret = map_tps_detect_patterns(&integration->map_tps_processor,
                                       &accel_pattern, &decel_pattern);
        if (ret == ESP_OK) {
            integration->data.acceleration_detected = accel_pattern;
            integration->data.deceleration_detected = decel_pattern;
        }
    } else {
        // Valores diretos se otimização desabilitada
        integration->data.map_filtered = raw_map;
        integration->data.tps_filtered = raw_tps;
        integration->data.engine_load = raw_map / 250.0f; // Simplificação
    }
    
    // Processar outros sensores com DSP
    if (integration->config.enable_dsp_processing) {
        // CLT
        ret = optimized_process_sensor_sample(&integration->dsp_processor,
                                             OPTIMIZED_SENSOR_CLT,
                                             raw_clt,
                                             &integration->data.clt_filtered);
        if (ret != ESP_OK) return ret;
        
        // Oil temp
        ret = optimized_process_sensor_sample(&integration->dsp_processor,
                                             OPTIMIZED_SENSOR_OIL_TEMP,
                                             raw_oil_temp,
                                             &integration->data.oil_temp_filtered);
        if (ret != ESP_OK) return ret;
        
        // Oil pressure
        ret = optimized_process_sensor_sample(&integration->dsp_processor,
                                             OPTIMIZED_SENSOR_OIL_PRESS,
                                             raw_oil_press,
                                             &integration->data.oil_pressure_filtered);
        if (ret != ESP_OK) return ret;
        
        // Battery voltage
        ret = optimized_process_sensor_sample(&integration->dsp_processor,
                                             OPTIMIZED_SENSOR_VBAT,
                                             raw_vbat,
                                             &integration->data.battery_voltage_filtered);
        if (ret != ESP_OK) return ret;
    } else {
        // Valores diretos se DSP desabilitado
        integration->data.clt_filtered = raw_clt;
        integration->data.oil_temp_filtered = raw_oil_temp;
        integration->data.oil_pressure_filtered = raw_oil_press;
        integration->data.battery_voltage_filtered = raw_vbat;
    }
    
    ESP_LOGV(TAG, "Sensors processed: MAP=%.1f, TPS=%.1f, Load=%.2f", 
              integration->data.map_filtered, integration->data.tps_filtered, 
              integration->data.engine_load);
    
    return ESP_OK;
}

esp_err_t esp32s3_calculate_timing(esp32s3_integration_t *integration,
                                    uint16_t rpm,
                                    float engine_load,
                                    const float *pulse_widths,
                                    const float *advance_angles) {
    if (integration == NULL || !integration->initialized || 
        pulse_widths == NULL || advance_angles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    integration->data.current_rpm = rpm;
    
    if (integration->config.enable_vector_timing) {
        // Usar processamento vetorial para cálculos de timing
        esp_err_t ret = vector_calculate_injection_times_vectorized(&integration->vector_context,
                                                                    rpm,
                                                                    pulse_widths,
                                                                    integration->data.injection_times);
        if (ret != ESP_OK) return ret;
        
        ret = vector_calculate_ignition_times_vectorized(&integration->vector_context,
                                                          rpm,
                                                          advance_angles,
                                                          integration->data.ignition_times);
        if (ret != ESP_OK) return ret;
        
        // Copiar valores para referência
        for (uint8_t i = 0; i < integration->config.num_cylinders; i++) {
            integration->data.injection_pw[i] = pulse_widths[i];
            integration->data.ignition_advance[i] = advance_angles[i];
        }
    } else {
        // Cálculo simplificado sem vetorialização
        float us_per_degree = 166666.67f / (float)rpm;
        
        for (uint8_t i = 0; i < integration->config.num_cylinders; i++) {
            integration->data.injection_pw[i] = pulse_widths[i];
            integration->data.ignition_advance[i] = advance_angles[i];
            
            // Timing simplificado
            float timing_offset = (float)i * (720.0f / integration->config.num_cylinders);
            integration->data.injection_times[i] = (uint32_t)((timing_offset - advance_angles[i]) * us_per_degree);
            integration->data.ignition_times[i] = (uint32_t)((timing_offset + advance_angles[i]) * us_per_degree);
        }
    }
    
    ESP_LOGV(TAG, "Timing calculated for %d cylinders at %d RPM", 
              integration->config.num_cylinders, rpm);
    
    return ESP_OK;
}

esp_err_t esp32s3_check_ulp_status(esp32s3_integration_t *integration,
                                      bool *critical_condition,
                                      ulp_adc_channel_t *critical_channel,
                                      float *critical_value) {
    if (integration == NULL || !integration->initialized || critical_condition == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!integration->config.enable_ulp_monitoring) {
        *critical_condition = false;
        return ESP_OK;
    }
    
    ulp_check_result_t ulp_result;
    esp_err_t ret = ulp_monitor_check_status(&integration->ulp_monitor, &ulp_result);
    if (ret != ESP_OK) return ret;
    
    *critical_condition = ulp_result.critical_condition;
    integration->data.ulp_critical_condition = ulp_result.critical_condition;
    
    if (ulp_result.critical_condition) {
        integration->data.critical_channel = ulp_result.critical_channel;
        integration->data.critical_value = ulp_result.critical_value;
        
        if (critical_channel) *critical_channel = ulp_result.critical_channel;
        if (critical_value) *critical_value = ulp_result.critical_value;
        
        ESP_LOGW(TAG, "ULP critical condition: channel=%d, value=%.2f", 
                  ulp_result.critical_channel, ulp_result.critical_value);
    }
    
    return ESP_OK;
}

//=============================================================================
// Funções de Comunicação e Telemetria
//=============================================================================

esp_err_t esp32s3_prepare_telemetry(esp32s3_integration_t *integration,
                                      uint8_t *output_buffer,
                                      uint16_t buffer_size,
                                      uint16_t *compressed_size) {
    if (integration == NULL || !integration->initialized || 
        output_buffer == NULL || compressed_size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Preparar mensagem de status do motor
    espnow_engine_status_t status = {
        .rpm = integration->data.current_rpm,
        .map_kpa10 = (uint16_t)(integration->data.map_filtered * 10),
        .clt_c10 = (int16_t)(integration->data.clt_filtered * 10),
        .iat_c10 = (int16_t)(integration->data.oil_temp_filtered * 10), // Usando oil temp como IAT
        .tps_pct10 = (uint16_t)(integration->data.tps_filtered * 10),
        .battery_mv = (uint16_t)(integration->data.battery_voltage_filtered * 1000),
        .sync_status = 1, // Simplificado
        .limp_mode = integration->data.ulp_critical_condition ? 1 : 0,
        .advance_deg10 = (uint16_t)(integration->data.ignition_advance[0] * 10),
        .pw_us = (uint16_t)integration->data.injection_pw[0],
        .lambda_target = 1450, // Stoichiometric
        .lambda_measured = 1450, // Simplificado
        .timestamp_ms = (uint32_t)(HAL_Time_us() / 1000)
    };
    
    if (integration->config.enable_compression) {
        // Comprimir mensagem
        espnow_compress_result_t comp_result;
        esp_err_t ret = espnow_compress_engine_status(&integration->compression_context,
                                                      &status,
                                                      output_buffer,
                                                      buffer_size,
                                                      &comp_result);
        if (ret != ESP_OK) return ret;
        
        *compressed_size = comp_result.compressed_size;
        integration->data.compression_ratio = comp_result.compression_ratio;
        
        ESP_LOGV(TAG, "Telemetry compressed: %d->%d bytes, ratio=%.2f", 
                  sizeof(status), *compressed_size, comp_result.compression_ratio);
    } else {
        // Sem compressão
        if (buffer_size < sizeof(status)) {
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(output_buffer, &status, sizeof(status));
        *compressed_size = sizeof(status);
        integration->data.compression_ratio = 1.0f;
    }
    
    return ESP_OK;
}

//=============================================================================
// Funções Utilitárias
//=============================================================================

static void esp32s3_update_performance_metrics(esp32s3_integration_t *integration) {
    if (integration == NULL || !integration->initialized) {
        return;
    }
    
    // Calcular uso de CPU baseado no tempo de processamento
    uint32_t cycle_time_us = 1000000 / ESP32S3_MAIN_PROCESSING_FREQ;
    integration->data.cpu_usage_percent = 
        (float)integration->data.processing_time_us / (float)cycle_time_us * 100.0f;
    
    // Limitar a 100%
    if (integration->data.cpu_usage_percent > 100.0f) {
        integration->data.cpu_usage_percent = 100.0f;
    }
}

esp_err_t esp32s3_get_performance_stats(esp32s3_integration_t *integration,
                                         float *cpu_usage,
                                         uint32_t *processing_time,
                                         float *compression_ratio,
                                         uint32_t *ulp_wakeups) {
    if (integration == NULL || !integration->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (cpu_usage) *cpu_usage = integration->data.cpu_usage_percent;
    if (processing_time) *processing_time = integration->data.processing_time_us;
    if (compression_ratio) *compression_ratio = integration->data.compression_ratio;
    
    if (ulp_wakeups && integration->config.enable_ulp_monitoring) {
        uint32_t total_wakeups, critical_wakeups;
        ulp_monitor_get_statistics(&integration->ulp_monitor, &total_wakeups, 
                                   &critical_wakeups, NULL, NULL);
        *ulp_wakeups = total_wakeups;
    }
    
    return ESP_OK;
}

esp_err_t esp32s3_check_system_health(esp32s3_integration_t *integration,
                                        bool *all_operational) {
    if (integration == NULL || !integration->initialized || all_operational == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bool operational = true;
    
    // Verificar componentes DSP
    if (integration->config.enable_dsp_processing) {
        // Verificar se processador DSP está operacional
        if (integration->dsp_processor.initialized == false) {
            operational = false;
            ESP_LOGW(TAG, "DSP processor not operational");
        }
    }
    
    // Verificar monitoramento ULP
    if (integration->config.enable_ulp_monitoring) {
        bool ulp_operational;
        esp_err_t ret = ulp_monitor_is_operational(&integration->ulp_monitor, &ulp_operational);
        if (ret != ESP_OK || !ulp_operational) {
            operational = false;
            ESP_LOGW(TAG, "ULP monitor not operational");
        }
    }
    
    // Verificar compressão
    if (integration->config.enable_compression) {
        if (integration->compression_context.initialized == false) {
            operational = false;
            ESP_LOGW(TAG, "Compression not operational");
        }
    }
    
    // Verificar matemática vetorial
    if (integration->config.enable_vector_timing) {
        if (integration->vector_context.initialized == false) {
            operational = false;
            ESP_LOGW(TAG, "Vector math not operational");
        }
    }
    
    *all_operational = operational;
    
    ESP_LOGI(TAG, "System health check: %s", operational ? "OK" : "FAILURES DETECTED");
    
    return ESP_OK;
}
