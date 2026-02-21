/**
 * @file ulp_monitor.c
 * @brief Implementação do monitoramento ULP para sensores críticos
 */

#include "ulp_monitor.h"
#include "esp_log.h"
#include "hal/hal_timer.h"
#include "esp_sleep.h"
#include "ulp/ulp.h"
#include "ulp/ulp_riscv.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "hal/adc_ll.h"
#include <math.h>
#include <string.h>

static const char* TAG = "ULP_MONITOR";

//=============================================================================
// Variáveis Globais Estáticas
//=============================================================================

static ulp_monitor_context_t *g_ulp_ctx = NULL;
static bool g_ulp_initialized = false;

//=============================================================================
// Programa ULP Assembly (simplificado)
//=============================================================================

// Este seria o código ULP real em assembly
// Para este exemplo, vamos simular o comportamento

//=============================================================================
// Funções de Inicialização
//=============================================================================

esp_err_t ulp_monitor_init(ulp_monitor_context_t *ctx) {
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limpar contexto
    memset(ctx, 0, sizeof(ulp_monitor_context_t));
    
    // Alocar memória compartilhada na RTC slow memory
    ctx->shared_data = (ulp_shared_data_t*)malloc(sizeof(ulp_shared_data_t));
    if (ctx->shared_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate shared memory for ULP");
        return ESP_ERR_NO_MEM;
    }
    
    // Limpar dados compartilhados
    memset(ctx->shared_data, 0, sizeof(ulp_shared_data_t));
    
    // Configurações padrão para sensores
    for (int i = 0; i < ULP_ADC_CHANNEL_COUNT; i++) {
        ctx->sensors[i].adc_channel = (ulp_adc_channel_t)i;
        ctx->sensors[i].enable_monitoring = true;
        ctx->sensors[i].enable_wake_on_critical = true;
        ctx->sensors[i].sample_interval_ms = ULP_CHECK_INTERVAL_S * 1000;
    }
    
    // Configurar limiares padrão
    ctx->sensors[ULP_ADC_CHANNEL_CLT].critical_threshold = ULP_CRITICAL_TEMP_C;
    ctx->sensors[ULP_ADC_CHANNEL_OIL_TEMP].critical_threshold = 120.0f; // Oil temp mais alta
    ctx->sensors[ULP_ADC_CHANNEL_OIL_PRESS].critical_threshold = ULP_LOW_OIL_PRESSURE_KPA;
    ctx->sensors[ULP_ADC_CHANNEL_VBAT].critical_threshold = 10.0f; // 10V mínimo
    
    // Inicializar ADC para ULP
    esp_err_t ret = ulp_monitor_init_adc();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC for ULP: %s", esp_err_to_name(ret));
        free(ctx->shared_data);
        return ret;
    }
    
    // Inicializar programa ULP (simulado)
    ret = ulp_monitor_load_program();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load ULP program: %s", esp_err_to_name(ret));
        free(ctx->shared_data);
        return ret;
    }
    
    ctx->initialized = true;
    ctx->program_start_time = HAL_Time_us();
    g_ulp_ctx = ctx;
    g_ulp_initialized = true;
    
    ESP_LOGI(TAG, "ULP monitor initialized successfully");
    ESP_LOGI(TAG, "  Shared data address: %p", ctx->shared_data);
    ESP_LOGI(TAG, "  Sample interval: %d ms", ULP_CHECK_INTERVAL_S * 1000);
    ESP_LOGI(TAG, "  Critical thresholds:");
    ESP_LOGI(TAG, "    CLT: %.1f°C", ctx->sensors[ULP_ADC_CHANNEL_CLT].critical_threshold);
    ESP_LOGI(TAG, "    Oil Temp: %.1f°C", ctx->sensors[ULP_ADC_CHANNEL_OIL_TEMP].critical_threshold);
    ESP_LOGI(TAG, "    Oil Pressure: %.1f kPa", ctx->sensors[ULP_ADC_CHANNEL_OIL_PRESS].critical_threshold);
    ESP_LOGI(TAG, "    Battery: %.1f V", ctx->sensors[ULP_ADC_CHANNEL_VBAT].critical_threshold);
    
    return ESP_OK;
}

esp_err_t ulp_monitor_init_adc(void) {
    // Configurar ADC para modo ULP
    adc_digi_init_config_t adc_digi_config = {
        .max_store_buf_size = 1,
        .conv_num_each_intr = 1,
        .adc1_chan_mask = 0,
        .adc2_chan_mask = 0,
    };
    
    esp_err_t ret = adc_digi_initialize(&adc_digi_config);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    
    // Configurar canais ADC para ULP
    adc_digi_configuration_t dig_cfg = {
        .conv_limit_num = ULP_ADC_CHANNEL_COUNT,
        .sample_freq_hz = ULP_SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_FORMAT_12BIT,
    };
    
    // Configurar cada canal
    adc_digi_pattern_config_t adc_pattern[ULP_ADC_CHANNEL_COUNT] = {0};
    
    for (int i = 0; i < ULP_ADC_CHANNEL_COUNT; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = i + 3; // Canais 3-6
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    
    dig_cfg.pattern_num = ULP_ADC_CHANNEL_COUNT;
    dig_cfg.adc_pattern = adc_pattern;
    
    ret = adc_digi_controller_config(&dig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ADC initialized for ULP monitoring");
    return ESP_OK;
}

esp_err_t ulp_monitor_load_program(void) {
    // Em uma implementação real, aqui carregaríamos o programa ULP assembly
    // Para este exemplo, vamos simular que o programa foi carregado
    
    ESP_LOGI(TAG, "ULP program loaded (simulated)");
    
    // Configurar wake-up sources
    esp_sleep_enable_ulp_wakeup();
    
    return ESP_OK;
}

esp_err_t ulp_monitor_deinit(ulp_monitor_context_t *ctx) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Parar ULP
    ulp_monitor_stop(ctx);
    
    // Liberar memória
    if (ctx->shared_data != NULL) {
        free(ctx->shared_data);
        ctx->shared_data = NULL;
    }
    
    ctx->initialized = false;
    g_ulp_ctx = NULL;
    g_ulp_initialized = false;
    
    ESP_LOGI(TAG, "ULP monitor deinitialized");
    return ESP_OK;
}

//=============================================================================
// Funções de Controle
//=============================================================================

esp_err_t ulp_monitor_start(ulp_monitor_context_t *ctx, bool enable_deep_sleep) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Iniciar programa ULP
    esp_err_t ret = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ULP program: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ctx->ulp_running = true;
    ctx->deep_sleep_enabled = enable_deep_sleep;
    
    ESP_LOGI(TAG, "ULP monitoring started (deep sleep: %s)", 
              enable_deep_sleep ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t ulp_monitor_stop(ulp_monitor_context_t *ctx) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Parar programa ULP
    esp_err_t ret = ulp_stop();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to stop ULP program: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ctx->ulp_running = false;
    
    ESP_LOGI(TAG, "ULP monitoring stopped");
    return ESP_OK;
}

//=============================================================================
// Funções de Verificação e Leitura
//=============================================================================

esp_err_t ulp_monitor_check_status(ulp_monitor_context_t *ctx,
                                    ulp_check_result_t *result) {
    if (ctx == NULL || !ctx->initialized || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Simular leitura dos dados compartilhados
    ulp_shared_data_t *data = ctx->shared_data;
    
    // Em uma implementação real, estes dados viriam do ULP
    // Para simulação, vamos gerar dados realistas
    static uint32_t sim_counter = 0;
    sim_counter++;
    
    // Simular leituras de sensores
    data->clt_celsius = 85.0f + sinf(sim_counter * 0.1f) * 5.0f;
    data->oil_temp_celsius = 90.0f + sinf(sim_counter * 0.08f) * 8.0f;
    data->oil_pressure_kpa = 250.0f + sinf(sim_counter * 0.05f) * 50.0f;
    data->battery_voltage = 13.8f + sinf(sim_counter * 0.02f) * 0.5f;
    
    // Atualizar médias
    data->clt_avg = data->clt_avg * 0.9f + data->clt_celsius * 0.1f;
    data->oil_temp_avg = data->oil_temp_avg * 0.9f + data->oil_temp_celsius * 0.1f;
    data->oil_pressure_avg = data->oil_pressure_avg * 0.9f + data->oil_pressure_kpa * 0.1f;
    data->battery_voltage_avg = data->battery_voltage_avg * 0.9f + data->battery_voltage * 0.1f;
    
    // Verificar condições críticas
    data->clt_critical = data->clt_celsius > ctx->sensors[ULP_ADC_CHANNEL_CLT].critical_threshold;
    data->oil_temp_critical = data->oil_temp_celsius > ctx->sensors[ULP_ADC_CHANNEL_OIL_TEMP].critical_threshold;
    data->oil_pressure_critical = data->oil_pressure_kpa < ctx->sensors[ULP_ADC_CHANNEL_OIL_PRESS].critical_threshold;
    data->battery_critical = data->battery_voltage < ctx->sensors[ULP_ADC_CHANNEL_VBAT].critical_threshold;
    
    data->any_critical = data->clt_critical || data->oil_temp_critical || 
                         data->oil_pressure_critical || data->battery_critical;
    
    // Preencher resultado
    result->monitoring_active = ctx->ulp_running;
    result->critical_condition = data->any_critical;
    result->uptime_seconds = (HAL_Time_us() - ctx->program_start_time) / 1000000;
    result->samples_since_wakeup = data->sample_count;
    
    if (data->any_critical) {
        if (data->clt_critical) {
            result->critical_channel = ULP_ADC_CHANNEL_CLT;
            result->critical_value = data->clt_celsius;
        } else if (data->oil_temp_critical) {
            result->critical_channel = ULP_ADC_CHANNEL_OIL_TEMP;
            result->critical_value = data->oil_temp_celsius;
        } else if (data->oil_pressure_critical) {
            result->critical_channel = ULP_ADC_CHANNEL_OIL_PRESS;
            result->critical_value = data->oil_pressure_kpa;
        } else {
            result->critical_channel = ULP_ADC_CHANNEL_VBAT;
            result->critical_value = data->battery_voltage;
        }
        
        ESP_LOGW(TAG, "Critical condition detected: channel=%d, value=%.2f", 
                  result->critical_channel, result->critical_value);
    }
    
    // Atualizar contadores
    data->sample_count++;
    if (data->any_critical) {
        data->critical_events++;
        data->last_critical_time = (uint32_t)HAL_Time_us();
    }
    
    return ESP_OK;
}

esp_err_t ulp_monitor_get_shared_data(ulp_monitor_context_t *ctx,
                                      ulp_shared_data_t *data) {
    if (ctx == NULL || !ctx->initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copiar dados compartilhados
    memcpy(data, ctx->shared_data, sizeof(ulp_shared_data_t));
    
    return ESP_OK;
}

esp_err_t ulp_monitor_read_sensor(ulp_monitor_context_t *ctx,
                                  ulp_adc_channel_t channel,
                                  float *value,
                                  float *average) {
    if (ctx == NULL || !ctx->initialized || 
        channel >= ULP_ADC_CHANNEL_COUNT || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ulp_shared_data_t *data = ctx->shared_data;
    
    switch (channel) {
        case ULP_ADC_CHANNEL_CLT:
            *value = data->clt_celsius;
            if (average) *average = data->clt_avg;
            break;
        case ULP_ADC_CHANNEL_OIL_TEMP:
            *value = data->oil_temp_celsius;
            if (average) *average = data->oil_temp_avg;
            break;
        case ULP_ADC_CHANNEL_OIL_PRESS:
            *value = data->oil_pressure_kpa;
            if (average) *average = data->oil_pressure_avg;
            break;
        case ULP_ADC_CHANNEL_VBAT:
            *value = data->battery_voltage;
            if (average) *average = data->battery_voltage_avg;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

//=============================================================================
// Funções de Configuração
//=============================================================================

esp_err_t ulp_monitor_configure_sensor(ulp_monitor_context_t *ctx,
                                       ulp_adc_channel_t channel,
                                       const ulp_sensor_config_t *config) {
    if (ctx == NULL || !ctx->initialized || 
        channel >= ULP_ADC_CHANNEL_COUNT || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copiar configuração
    ctx->sensors[channel] = *config;
    
    // Atualizar limiares nos dados compartilhados
    ulp_shared_data_t *data = ctx->shared_data;
    
    // Converter limiares para valores ADC (simulação)
    switch (channel) {
        case ULP_ADC_CHANNEL_CLT:
            data->clt_critical_raw = (uint16_t)(config->critical_threshold * 10); // Simulação
            break;
        case ULP_ADC_CHANNEL_OIL_TEMP:
            data->oil_temp_critical_raw = (uint16_t)(config->critical_threshold * 10);
            break;
        case ULP_ADC_CHANNEL_OIL_PRESS:
            data->oil_press_critical_raw = (uint16_t)(config->critical_threshold * 4);
            break;
        case ULP_ADC_CHANNEL_VBAT:
            data->vbat_critical_raw = (uint16_t)(config->critical_threshold * 100);
            break;
    }
    
    ESP_LOGI(TAG, "Sensor %d configured: critical=%.2f, wake_on_critical=%s", 
              channel, config->critical_threshold, 
              config->enable_wake_on_critical ? "yes" : "no");
    
    return ESP_OK;
}

esp_err_t ulp_monitor_set_critical_thresholds(ulp_monitor_context_t *ctx,
                                              float clt_critical,
                                              float oil_temp_critical,
                                              float oil_pressure_critical,
                                              float vbat_critical) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Atualizar limiares
    ctx->sensors[ULP_ADC_CHANNEL_CLT].critical_threshold = clt_critical;
    ctx->sensors[ULP_ADC_CHANNEL_OIL_TEMP].critical_threshold = oil_temp_critical;
    ctx->sensors[ULP_ADC_CHANNEL_OIL_PRESS].critical_threshold = oil_pressure_critical;
    ctx->sensors[ULP_ADC_CHANNEL_VBAT].critical_threshold = vbat_critical;
    
    // Atualizar dados compartilhados
    ulp_shared_data_t *data = ctx->shared_data;
    data->clt_critical_raw = (uint16_t)(clt_critical * 10);
    data->oil_temp_critical_raw = (uint16_t)(oil_temp_critical * 10);
    data->oil_press_critical_raw = (uint16_t)(oil_pressure_critical * 4);
    data->vbat_critical_raw = (uint16_t)(vbat_critical * 100);
    
    ESP_LOGI(TAG, "Critical thresholds updated:");
    ESP_LOGI(TAG, "  CLT: %.1f°C", clt_critical);
    ESP_LOGI(TAG, "  Oil Temp: %.1f°C", oil_temp_critical);
    ESP_LOGI(TAG, "  Oil Pressure: %.1f kPa", oil_pressure_critical);
    ESP_LOGI(TAG, "  Battery: %.1f V", vbat_critical);
    
    return ESP_OK;
}

esp_err_t ulp_monitor_set_sample_interval(ulp_monitor_context_t *ctx,
                                          uint32_t interval_ms) {
    if (ctx == NULL || !ctx->initialized || interval_ms < 100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Atualizar intervalo em todos os sensores
    for (int i = 0; i < ULP_ADC_CHANNEL_COUNT; i++) {
        ctx->sensors[i].sample_interval_ms = interval_ms;
    }
    
    // Atualizar dados compartilhados
    ctx->shared_data->sample_interval = interval_ms;
    
    ESP_LOGI(TAG, "Sample interval set to %d ms", interval_ms);
    
    return ESP_OK;
}

//=============================================================================
// Funções de Callback
//=============================================================================

esp_err_t ulp_monitor_set_critical_callback(ulp_monitor_context_t *ctx,
                                             void (*callback)(ulp_adc_channel_t, float)) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ctx->critical_callback = callback;
    
    ESP_LOGI(TAG, "Critical callback configured");
    return ESP_OK;
}

esp_err_t ulp_monitor_set_warning_callback(ulp_monitor_context_t *ctx,
                                            void (*callback)(ulp_adc_channel_t, float)) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ctx->warning_callback = callback;
    
    ESP_LOGI(TAG, "Warning callback configured");
    return ESP_OK;
}

esp_err_t ulp_monitor_set_status_callback(ulp_monitor_context_t *ctx,
                                          void (*callback)(const ulp_shared_data_t *)) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ctx->status_callback = callback;
    
    ESP_LOGI(TAG, "Status callback configured");
    return ESP_OK;
}

//=============================================================================
// Funções de Estatísticas
//=============================================================================

esp_err_t ulp_monitor_get_statistics(ulp_monitor_context_t *ctx,
                                      uint32_t *total_wakeups,
                                      uint32_t *critical_wakeups,
                                      float *avg_sleep_duration,
                                      float *cpu_usage_percent) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (total_wakeups) *total_wakeups = ctx->total_wakeups;
    if (critical_wakeups) *critical_wakeups = ctx->critical_wakeups;
    if (avg_sleep_duration) *avg_sleep_duration = ctx->avg_sleep_duration;
    if (cpu_usage_percent) *cpu_usage_percent = 0.1f; // ULP usa muito pouco CPU
    
    return ESP_OK;
}

esp_err_t ulp_monitor_reset_statistics(ulp_monitor_context_t *ctx) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Resetar estatísticas do contexto
    ctx->total_wakeups = 0;
    ctx->critical_wakeups = 0;
    ctx->avg_sleep_duration = 0.0f;
    
    // Resetar estatísticas nos dados compartilhados
    ulp_shared_data_t *data = ctx->shared_data;
    data->sample_count = 0;
    data->critical_events = 0;
    data->warning_events = 0;
    data->last_critical_time = 0;
    data->last_warning_time = 0;
    
    ESP_LOGI(TAG, "ULP monitor statistics reset");
    
    return ESP_OK;
}

esp_err_t ulp_monitor_is_operational(ulp_monitor_context_t *ctx, bool *operational) {
    if (ctx == NULL || !ctx->initialized || operational == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Verificar se ULP está rodando e dados são atualizados
    static uint32_t last_sample_count = 0;
    static uint64_t last_check_time = 0;
    
    uint64_t current_time = HAL_Time_us();
    uint32_t current_sample_count = ctx->shared_data->sample_count;
    
    bool samples_increasing = (current_sample_count > last_sample_count);
    bool reasonable_timing = (current_time - last_check_time) < 10000000; // 10 segundos
    
    *operational = ctx->ulp_running && samples_increasing && reasonable_timing;
    
    last_sample_count = current_sample_count;
    last_check_time = current_time;
    
    return ESP_OK;
}
