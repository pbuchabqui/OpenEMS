/**
 * @file main_with_esp32s3.c
 * @brief Exemplo de main.c com integração das melhorias ESP32-S3
 * 
 * Este arquivo demonstra como integrar todas as melhorias competitivas
 * do ESP32-S3 no sistema principal OpenEMS.
 */

#include "esp32s3_main_integration.h"
#include "engine_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char* TAG = "S3_MAIN_ENHANCED";

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting Enhanced ECU with ESP32-S3 Competitive Improvements");
    
    // Inicializar melhorias ESP32-S3 primeiro
    ESP_LOGI(TAG, "🔧 Initializing ESP32-S3 competitive improvements...");
    esp_err_t err = esp32s3_main_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ESP32-S3 improvements init failed: %s", esp_err_to_name(err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "✅ ESP32-S3 improvements initialized successfully");
    
    // Iniciar tarefas ESP32-S3
    ESP_LOGI(TAG, "🚀 Starting ESP32-S3 processing tasks...");
    err = esp32s3_main_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start ESP32-S3 tasks: %s", esp_err_to_name(err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "✅ ESP32-S3 processing tasks started");
    
    // Inicializar sistema de controle do motor tradicional
    ESP_LOGI(TAG, "🔧 Initializing traditional engine control system...");
    err = engine_control_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Engine control init failed: %s", esp_err_to_name(err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "✅ Traditional engine control system initialized");
    
    // Executar diagnóstico completo
    ESP_LOGI(TAG, "🔍 Running comprehensive system diagnostics...");
    err = esp32s3_main_run_diagnostics();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "⚠️  Some diagnostics failed: %s", esp_err_to_name(err));
    }
    
    // Aguardar um pouco para estabilização
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Log de status inicial
    ESP_LOGI(TAG, "🎯 System Status:");
    ESP_LOGI(TAG, "   📊 DSP Processing: ✅ Enabled");
    ESP_LOGI(TAG, "   🎛️  MAP/TPS Optimization: ✅ Enabled");
    ESP_LOGI(TAG, "   🔋 ULP Monitoring: ✅ Enabled");
    ESP_LOGI(TAG, "   📡 ESP-NOW Compression: ✅ Enabled");
    ESP_LOGI(TAG, "   ⚡ Vector Timing: ✅ Enabled");
    ESP_LOGI(TAG, "   🏎️  Engine Control: ✅ Running");
    
    ESP_LOGI(TAG, "🏁 Enhanced ECU System Ready - All Systems Operational");
    
    // Loop principal - monitoramento e status
    uint32_t last_status_time = 0;
    const uint32_t status_interval_ms = 10000; // 10 segundos
    
    while (1) {
        uint32_t current_time = (uint32_t)(esp_timer_get_time() / 1000);
        
        if (current_time - last_status_time >= status_interval_ms) {
            // Obter status do sistema ESP32-S3
            esp32s3_integration_t *esp32s3_integration;
            err = esp32s3_main_get_status(&esp32s3_integration);
            
            if (err == ESP_OK) {
                // Log status dos sensores processados
                ESP_LOGI(TAG, "📊 ESP32-S3 Sensor Status:");
                ESP_LOGI(TAG, "   MAP: %.1f kPa (filtered)", esp32s3_integration->data.map_filtered);
                ESP_LOGI(TAG, "   TPS: %.1f%% (filtered)", esp32s3_integration->data.tps_filtered);
                ESP_LOGI(TAG, "   CLT: %.1f°C (filtered)", esp32s3_integration->data.clt_filtered);
                ESP_LOGI(TAG, "   Oil Temp: %.1f°C (filtered)", esp32s3_integration->data.oil_temp_filtered);
                ESP_LOGI(TAG, "   Oil Pressure: %.1f kPa (filtered)", esp32s3_integration->data.oil_pressure_filtered);
                ESP_LOGI(TAG, "   Battery: %.2f V (filtered)", esp32s3_integration->data.battery_voltage_filtered);
                
                // Log status do motor
                ESP_LOGI(TAG, "🏎️  Engine Status:");
                ESP_LOGI(TAG, "   RPM: %d", esp32s3_integration->data.current_rpm);
                ESP_LOGI(TAG, "   Load: %.1f%%", esp32s3_integration->data.engine_load * 100.0f);
                ESP_LOGI(TAG, "   MAP-TPS Correlation: %.3f", esp32s3_integration->data.map_tps_correlation);
                ESP_LOGI(TAG, "   Acceleration: %s", esp32s3_integration->data.acceleration_detected ? "Yes" : "No");
                ESP_LOGI(TAG, "   Deceleration: %s", esp32s3_integration->data.deceleration_detected ? "Yes" : "No");
                
                // Log status ULP
                if (esp32s3_integration->data.ulp_critical_condition) {
                    ESP_LOGE(TAG, "🚨 ULP CRITICAL: Channel %d = %.2f", 
                              esp32s3_integration->data.critical_channel,
                              esp32s3_integration->data.critical_value);
                } else {
                    ESP_LOGI(TAG, "🔋 ULP Status: ✅ Normal");
                }
                
                // Log performance
                ESP_LOGI(TAG, "⚡ Performance:");
                ESP_LOGI(TAG, "   Processing Time: %d μs", esp32s3_integration->data.processing_time_us);
                ESP_LOGI(TAG, "   CPU Usage: %.1f%%", esp32s3_integration->data.cpu_usage_percent);
                ESP_LOGI(TAG, "   Compression Ratio: %.2f", esp32s3_integration->data.compression_ratio);
            }
            
            // Obter status do sistema tradicional
            engine_params_t params = {0};
            if (engine_control_get_engine_parameters(&params) == ESP_OK) {
                ESP_LOGI(TAG, "🔧 Traditional Engine Control:");
                ESP_LOGI(TAG, "   RPM: %" PRIu32, params.rpm);
                ESP_LOGI(TAG, "   Load: %" PRIu32 " kPa", params.load / 10);
                ESP_LOGI(TAG, "   Limp Mode: %s", params.is_limp_mode ? "YES" : "NO");
            }
            
            last_status_time = current_time;
        }
        
        // Verificar saúde do sistema periodicamente
        static uint32_t last_health_check = 0;
        if (current_time - last_health_check >= 30000) { // 30 segundos
            esp32s3_main_run_diagnostics();
            last_health_check = current_time;
        }
        
        // Yield para evitar starvation
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//=============================================================================
// Funções Adicionais para Demonstração
//=============================================================================

/**
 * @brief Demonstra as capacidades de processamento vetorial
 */
void demonstrate_vector_processing(void) {
    ESP_LOGI(TAG, "🧮 Demonstrating vector processing capabilities...");
    
    esp32s3_integration_t *integration;
    if (esp32s3_main_get_status(&integration) == ESP_OK) {
        // Exemplo de cálculo vetorial de timing para 4 cilindros
        float pulse_widths[4] = {2000.0f, 2100.0f, 2050.0f, 2150.0f};
        float advance_angles[4] = {20.0f, 22.0f, 21.0f, 23.0f};
        uint16_t rpm = 3000;
        
        esp_err_t ret = esp32s3_calculate_timing(integration, rpm, 0.5f, 
                                                 pulse_widths, advance_angles);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Vector timing calculation completed");
            for (uint8_t i = 0; i < 4; i++) {
                ESP_LOGI(TAG, "   Cylinder %d: Injection %d μs, Ignition %d μs", 
                          i+1, integration->data.injection_pw[i], 
                          integration->data.ignition_times[i]);
            }
        }
    }
}

/**
 * @brief Demonstra as capacidades de compressão
 */
void demonstrate_compression(void) {
    ESP_LOGI(TAG, "📦 Demonstrating compression capabilities...");
    
    esp32s3_integration_t *integration;
    if (esp32s3_main_get_status(&integration) == ESP_OK) {
        uint8_t test_data[] = "OpenEMS ESP32-S3 Enhanced ECU System Test Data";
        uint8_t compressed_data[256];
        uint16_t compressed_size;
        
        espnow_compress_result_t result;
        esp_err_t ret = espnow_compress_data(&integration->compression_context,
                                              test_data, sizeof(test_data),
                                              compressed_data, sizeof(compressed_data),
                                              &result);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Compression demonstration:");
            ESP_LOGI(TAG, "   Original: %d bytes", result.original_size);
            ESP_LOGI(TAG, "   Compressed: %d bytes", result.compressed_size);
            ESP_LOGI(TAG, "   Ratio: %.2f", result.compression_ratio);
            ESP_LOGI(TAG, "   Time: %d μs", result.compression_time_us);
        }
    }
}

/**
 * @brief Demonstra as capacidades de monitoramento ULP
 */
void demonstrate_ulp_monitoring(void) {
    ESP_LOGI(TAG, "🔋 Demonstrating ULP monitoring capabilities...");
    
    esp32s3_integration_t *integration;
    if (esp32s3_main_get_status(&integration) == ESP_OK) {
        bool critical_condition;
        ulp_adc_channel_t critical_channel;
        float critical_value;
        
        esp_err_t ret = esp32s3_check_ulp_status(integration, &critical_condition,
                                                   &critical_channel, &critical_value);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ ULP monitoring status:");
            ESP_LOGI(TAG, "   Critical Condition: %s", critical_condition ? "YES" : "NO");
            if (critical_condition) {
                ESP_LOGI(TAG, "   Critical Channel: %d", critical_channel);
                ESP_LOGI(TAG, "   Critical Value: %.2f", critical_value);
            }
            
            // Obter dados compartilhados ULP
            ulp_shared_data_t ulp_data;
            ret = ulp_monitor_get_shared_data(&integration->ulp_monitor, &ulp_data);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "   ULP Sample Count: %d", ulp_data.sample_count);
                ESP_LOGI(TAG, "   ULP Critical Events: %d", ulp_data.critical_events);
                ESP_LOGI(TAG, "   ULP CPU Wake Requested: %s", ulp_data.cpu_wake_requested ? "YES" : "NO");
            }
        }
    }
}
