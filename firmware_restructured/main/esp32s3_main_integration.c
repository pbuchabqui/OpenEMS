/**
 * @file esp32s3_main_integration.c
 * @brief Exemplo de integração das melhorias ESP32-S3 no sistema principal
 * 
 * Este arquivo demonstra como integrar todas as melhorias competitivas
 * implementadas no sistema principal OpenEMS.
 */

#include "integration/esp32s3_integration.h"
#include "esp_log.h"
#include "hal/hal_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char* TAG = "ESP32S3_MAIN";

// Contexto global da integração ESP32-S3
static esp32s3_integration_t g_esp32s3_integration;

// Task handles
static TaskHandle_t g_esp32s3_processing_task = NULL;
static TaskHandle_t g_esp32s3_telemetry_task = NULL;

// Protótipos de funções estáticas
static void esp32s3_processing_task(void *arg);
static void esp32s3_telemetry_task(void *arg);
static void esp32s3_critical_callback(ulp_adc_channel_t channel, float value);
static void esp32s3_warning_callback(ulp_adc_channel_t channel, float value);

//=============================================================================
// Funções de Inicialização Principal
//=============================================================================

esp_err_t esp32s3_main_init(void) {
    ESP_LOGI(TAG, "Initializing ESP32-S3 competitive improvements");
    
    // Configurar integração ESP32-S3
    esp32s3_integration_config_t config = {
        // Configurações DSP
        .enable_dsp_processing = true,
        .dsp_sample_rate = 1000,
        .enable_adaptive_filters = true,
        
        // Configurações de sensores
        .enable_map_tps_optimization = true,
        .map_critical_temp = 105.0f,
        .oil_pressure_critical = 100.0f,
        
        // Configurações ULP
        .enable_ulp_monitoring = true,
        .enable_deep_sleep = false, // Desabilitado inicialmente
        .ulp_sample_interval = 5000, // 5 segundos
        
        // Configurações de comunicação
        .enable_compression = true,
        .compression_type = ESPNOW_COMPRESS_HYBRID,
        .compression_level = 6,
        
        // Configurações de timing
        .enable_vector_timing = true,
        .num_cylinders = 4
    };
    
    // Inicializar integração
    esp_err_t ret = esp32s3_integration_init(&g_esp32s3_integration, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP32-S3 integration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configurar callbacks ULP
    if (config.enable_ulp_monitoring) {
        ret = ulp_monitor_set_critical_callback(&g_esp32s3_integration.ulp_monitor,
                                               esp32s3_critical_callback);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set ULP critical callback: %s", esp_err_to_name(ret));
        }
        
        ret = ulp_monitor_set_warning_callback(&g_esp32s3_integration.ulp_monitor,
                                              esp32s3_warning_callback);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set ULP warning callback: %s", esp_err_to_name(ret));
        }
    }
    
    ESP_LOGI(TAG, "ESP32-S3 main integration completed successfully");
    return ESP_OK;
}

esp_err_t esp32s3_main_start(void) {
    ESP_LOGI(TAG, "Starting ESP32-S3 processing tasks");
    
    // Iniciar integração
    esp_err_t ret = esp32s3_integration_start(&g_esp32s3_integration);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ESP32-S3 integration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Criar task de processamento principal
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        esp32s3_processing_task,
        "esp32s3_proc",
        4096,
        NULL,
        10, // Prioridade alta
        &g_esp32s3_processing_task,
        1  // Core 1
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ESP32-S3 processing task");
        return ESP_ERR_NO_MEM;
    }
    
    // Criar task de telemetria
    task_ret = xTaskCreatePinnedToCore(
        esp32s3_telemetry_task,
        "esp32s3_telem",
        3072,
        NULL,
        8, // Prioridade média
        &g_esp32s3_telemetry_task,
        1  // Core 1
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ESP32-S3 telemetry task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "ESP32-S3 processing tasks started successfully");
    return ESP_OK;
}

//=============================================================================
// Tasks de Processamento
//=============================================================================

static void esp32s3_processing_task(void *arg) {
    ESP_LOGI(TAG, "ESP32-S3 processing task started");
    
    uint32_t last_process_time = 0;
    const uint32_t process_interval_ms = 1000 / ESP32S3_MAIN_PROCESSING_FREQ; // 1ms
    
    while (1) {
        uint32_t current_time = (uint32_t)(HAL_Time_us() / 1000);
        
        if (current_time - last_process_time >= process_interval_ms) {
            // Simular dados brutos dos sensores (em implementação real, viriam dos ADCs)
            float raw_sensors[6];
            esp32s3_simulate_sensor_data(raw_sensors);
            
            // Processar ciclo completo do motor
            esp32s3_process_result_t result;
            esp_err_t ret = esp32s3_process_engine_cycle(&g_esp32s3_integration,
                                                           raw_sensors,
                                                           &result);
            
            if (ret == ESP_OK && result.success) {
                ESP_LOGV(TAG, "Engine cycle processed: %d us, %.1f%% CPU", 
                          result.processing_time_us, result.cpu_usage);
                
                // Verificar condições críticas
                if (result.critical_condition) {
                    ESP_LOGW(TAG, "Critical condition detected!");
                    // Aqui poderia acionar proteções do motor
                }
            } else {
                ESP_LOGE(TAG, "Failed to process engine cycle: %s", esp_err_to_name(ret));
            }
            
            last_process_time = current_time;
        }
        
        // Yield para evitar starvation
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void esp32s3_telemetry_task(void *arg) {
    ESP_LOGI(TAG, "ESP32-S3 telemetry task started");
    
    uint32_t last_telemetry_time = 0;
    const uint32_t telemetry_interval_ms = 1000 / ESP32S3_TELEMETRY_FREQ; // 20ms
    
    while (1) {
        uint32_t current_time = (uint32_t)(HAL_Time_us() / 1000);
        
        if (current_time - last_telemetry_time >= telemetry_interval_ms) {
            // Preparar mensagem de telemetria
            uint8_t telemetry_buffer[256];
            uint16_t compressed_size;
            
            esp_err_t ret = esp32s3_prepare_telemetry(&g_esp32s3_integration,
                                                         telemetry_buffer,
                                                         sizeof(telemetry_buffer),
                                                         &compressed_size);
            
            if (ret == ESP_OK) {
                // Enviar via ESP-NOW
                ret = esp32s3_send_telemetry(&g_esp32s3_integration, NULL); // Broadcast
                
                if (ret == ESP_OK) {
                    ESP_LOGV(TAG, "Telemetry sent: %d bytes, ratio=%.2f", 
                              compressed_size, g_esp32s3_integration.data.compression_ratio);
                } else {
                    ESP_LOGW(TAG, "Failed to send telemetry: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGW(TAG, "Failed to prepare telemetry: %s", esp_err_to_name(ret));
            }
            
            last_telemetry_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

//=============================================================================
// Callbacks ULP
//=============================================================================

static void esp32s3_critical_callback(ulp_adc_channel_t channel, float value) {
    ESP_LOGE(TAG, "ULP CRITICAL: Channel %d = %.2f", channel, value);
    
    // Ações para condições críticas
    switch (channel) {
        case ULP_ADC_CHANNEL_CLT:
            ESP_LOGE(TAG, "CRITICAL: Engine overheating! %.1f°C", value);
            // Poderia reduzir potência, acionar ventilador, etc.
            break;
            
        case ULP_ADC_CHANNEL_OIL_TEMP:
            ESP_LOGE(TAG, "CRITICAL: Oil overheating! %.1f°C", value);
            break;
            
        case ULP_ADC_CHANNEL_OIL_PRESS:
            ESP_LOGE(TAG, "CRITICAL: Low oil pressure! %.1f kPa", value);
            // Poderia desligar motor para evitar danos
            break;
            
        case ULP_ADC_CHANNEL_VBAT:
            ESP_LOGE(TAG, "CRITICAL: Low battery voltage! %.1f V", value);
            break;
            
        default:
            ESP_LOGE(TAG, "CRITICAL: Unknown channel %d = %.2f", channel, value);
            break;
    }
}

static void esp32s3_warning_callback(ulp_adc_channel_t channel, float value) {
    ESP_LOGW(TAG, "ULP WARNING: Channel %d = %.2f", channel, value);
    
    // Ações para condições de aviso
    switch (channel) {
        case ULP_ADC_CHANNEL_CLT:
            ESP_LOGW(TAG, "WARNING: High engine temperature: %.1f°C", value);
            break;
            
        case ULP_ADC_CHANNEL_OIL_PRESS:
            ESP_LOGW(TAG, "WARNING: Low oil pressure: %.1f kPa", value);
            break;
            
        default:
            ESP_LOGW(TAG, "WARNING: Channel %d = %.2f", channel, value);
            break;
    }
}

//=============================================================================
// Funções Utilitárias
//=============================================================================

static void esp32s3_simulate_sensor_data(float *raw_sensors) {
    static uint32_t simulation_time = 0;
    simulation_time++;
    
    // Simular dados realistas de sensores
    raw_sensors[0] = 100.0f + sinf(simulation_time * 0.01f) * 50.0f; // MAP (kPa)
    raw_sensors[1] = 10.0f + sinf(simulation_time * 0.02f) * 5.0f;   // TPS (%)
    raw_sensors[2] = 85.0f + sinf(simulation_time * 0.005f) * 10.0f; // CLT (°C)
    raw_sensors[3] = 90.0f + sinf(simulation_time * 0.008f) * 15.0f; // Oil temp (°C)
    raw_sensors[4] = 250.0f + sinf(simulation_time * 0.03f) * 50.0f; // Oil pressure (kPa)
    raw_sensors[5] = 13.8f + sinf(simulation_time * 0.001f) * 0.5f;  // Battery (V)
}

esp_err_t esp32s3_send_telemetry(esp32s3_integration_t *integration,
                                   const uint8_t *peer_mac) {
    if (integration == NULL || !integration->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Preparar mensagem de telemetria
    uint8_t telemetry_buffer[256];
    uint16_t compressed_size;
    
    esp_err_t ret = esp32s3_prepare_telemetry(integration,
                                                 telemetry_buffer,
                                                 sizeof(telemetry_buffer),
                                                 &compressed_size);
    if (ret != ESP_OK) return ret;
    
    // Enviar via ESP-NOW (usando módulo existente)
    // Em uma implementação real, isso usaria espnow_link_send_engine_status()
    ESP_LOGD(TAG, "Sending %d bytes of telemetry data", compressed_size);
    
    // Simular envio bem-sucedido
    return ESP_OK;
}

//=============================================================================
// Funções de Diagnóstico e Status
//=============================================================================

esp_err_t esp32s3_main_get_status(esp32s3_integration_t **integration) {
    if (integration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *integration = &g_esp32s3_integration;
    return ESP_OK;
}

esp_err_t esp32s3_main_run_diagnostics(void) {
    ESP_LOGI(TAG, "Running ESP32-S3 system diagnostics");
    
    // Verificar saúde do sistema
    bool all_operational;
    esp_err_t ret = esp32s3_check_system_health(&g_esp32s3_integration, &all_operational);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to check system health: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (all_operational) {
        ESP_LOGI(TAG, "✅ All ESP32-S3 components operational");
    } else {
        ESP_LOGW(TAG, "⚠️  Some ESP32-S3 components have issues");
    }
    
    // Obter estatísticas de performance
    float cpu_usage;
    uint32_t processing_time;
    float compression_ratio;
    uint32_t ulp_wakeups;
    
    ret = esp32s3_get_performance_stats(&g_esp32s3_integration,
                                         &cpu_usage,
                                         &processing_time,
                                         &compression_ratio,
                                         &ulp_wakeups);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "📊 Performance Statistics:");
        ESP_LOGI(TAG, "   CPU Usage: %.1f%%", cpu_usage);
        ESP_LOGI(TAG, "   Processing Time: %d μs", processing_time);
        ESP_LOGI(TAG, "   Compression Ratio: %.2f", compression_ratio);
        ESP_LOGI(TAG, "   ULP Wakeups: %d", ulp_wakeups);
    }
    
    // Executar diagnóstico completo
    float health_score;
    char *issues[10];
    uint8_t num_issues;
    
    ret = esp32s3_run_system_diagnostics(&g_esp32s3_integration,
                                           &health_score,
                                           issues,
                                           &num_issues);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "🏥 System Health Score: %.1f/100", health_score);
        
        if (num_issues > 0) {
            ESP_LOGW(TAG, "Issues detected:");
            for (uint8_t i = 0; i < num_issues; i++) {
                ESP_LOGW(TAG, "  - %s", issues[i]);
            }
        }
    }
    
    return ESP_OK;
}

esp_err_t esp32s3_main_stop(void) {
    ESP_LOGI(TAG, "Stopping ESP32-S3 processing tasks");
    
    // Parar tasks
    if (g_esp32s3_processing_task != NULL) {
        vTaskDelete(g_esp32s3_processing_task);
        g_esp32s3_processing_task = NULL;
    }
    
    if (g_esp32s3_telemetry_task != NULL) {
        vTaskDelete(g_esp32s3_telemetry_task);
        g_esp32s3_telemetry_task = NULL;
    }
    
    // Parar integração
    esp_err_t ret = esp32s3_integration_stop(&g_esp32s3_integration);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop ESP32-S3 integration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ESP32-S3 processing tasks stopped");
    return ESP_OK;
}
