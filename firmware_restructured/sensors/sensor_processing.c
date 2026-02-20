#include "sensor_processing.h"
#include "logger.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "config/engine_config.h"
#include <string.h>

// Static variables
static adc_continuous_handle_t adc_handle = NULL;
static sensor_data_t g_sensor_data = {0};
static sensor_config_t g_sensor_config = {0};
static SemaphoreHandle_t g_sensor_mutex = NULL;
static TaskHandle_t g_sensor_task_handle = NULL;
static volatile uint32_t g_sensor_seq = 0;

// Filter buffers
static uint32_t map_filter_buffer[16];
static uint8_t map_filter_index = 0;
static uint8_t low_rate_decimator = 0;
// H3 fix: filter state variables moved to file scope to avoid declaring
// 'static' inside switch cases, which is technically implementation-defined
// in C99/C11 and triggers warnings on some compilers.
static float tps_filtered = 0.0f;
static float clt_filtered = 0.0f;
static float iat_filtered = 0.0f;

static void process_sensors_task(void *pvParameters);
static float adc_to_range(uint32_t adc, float min_val, float max_val);

// Initialize sensor processing
esp_err_t sensor_init(void) {
    if (g_sensor_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    g_sensor_mutex = xSemaphoreCreateMutex();
    if (g_sensor_mutex == NULL) {
        ESP_LOGE("SENSOR", "Failed to create mutex");
        return ESP_FAIL;
    }

    // Load default configuration
    g_sensor_config.attenuation = ADC_ATTEN_DB_12;
    g_sensor_config.width = ADC_BITWIDTH_12;
    g_sensor_config.sample_rate_hz = 20000; // 20kHz
    g_sensor_config.map_filter_alpha = 0.2f;
    g_sensor_config.tps_filter_alpha = 0.05f;
    g_sensor_config.temp_filter_alpha = 0.05f;
    g_sensor_config.map_sync_enabled = true;
    g_sensor_config.map_sync_angle = 15; // Sincronizar no dente 15

    g_sensor_data.o2_mv = 450;
    g_sensor_data.vbat_dv = 120;

    ESP_LOGI("SENSOR", "Sensor processing initialized");
    return ESP_OK;
}

// Deinitialize sensor processing
esp_err_t sensor_deinit(void) {
    if (g_sensor_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Delete mutex
    vSemaphoreDelete(g_sensor_mutex);
    g_sensor_mutex = NULL;

    ESP_LOGI("SENSOR", "Sensor processing deinitialized");
    return ESP_OK;
}

// Start sensor reading
esp_err_t sensor_start(void) {
    if (g_sensor_mutex == NULL || adc_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create ADC configuration
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
    };
    esp_err_t err = adc_continuous_new_handle(&adc_config, &adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "Failed to create ADC handle: %s", esp_err_to_name(err));
        return err;
    }

    // Configure ADC channels
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = g_sensor_config.sample_rate_hz,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[7] = {
        {.atten = g_sensor_config.attenuation, .channel = ADC_CHANNEL_0, .unit = ADC_UNIT_1, .bit_width = g_sensor_config.width}, // MAP
        {.atten = g_sensor_config.attenuation, .channel = ADC_CHANNEL_1, .unit = ADC_UNIT_1, .bit_width = g_sensor_config.width}, // TPS
        {.atten = g_sensor_config.attenuation, .channel = ADC_CHANNEL_2, .unit = ADC_UNIT_1, .bit_width = g_sensor_config.width}, // CLT
        {.atten = g_sensor_config.attenuation, .channel = ADC_CHANNEL_3, .unit = ADC_UNIT_1, .bit_width = g_sensor_config.width}, // IAT
        {.atten = g_sensor_config.attenuation, .channel = ADC_CHANNEL_4, .unit = ADC_UNIT_1, .bit_width = g_sensor_config.width}, // O2
        {.atten = g_sensor_config.attenuation, .channel = ADC_CHANNEL_5, .unit = ADC_UNIT_1, .bit_width = g_sensor_config.width}, // VBAT
        {.atten = g_sensor_config.attenuation, .channel = ADC_CHANNEL_6, .unit = ADC_UNIT_1, .bit_width = g_sensor_config.width}, // SPARE
    };
    dig_cfg.adc_pattern = adc_pattern;
    dig_cfg.pattern_num = 7;

    err = adc_continuous_config(adc_handle, &dig_cfg);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "Failed to configure ADC: %s", esp_err_to_name(err));
        return err;
    }

    // Create sensor processing task
    BaseType_t result = xTaskCreatePinnedToCore(process_sensors_task, "sensor_task",
                                                SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIORITY, &g_sensor_task_handle,
                                                SENSOR_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE("SENSOR", "Failed to create sensor task");
        return ESP_FAIL;
    }

    // Start ADC
    err = adc_continuous_start(adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "Failed to start ADC: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI("SENSOR", "Sensor reading started");
    return ESP_OK;
}

// Stop sensor reading
esp_err_t sensor_stop(void) {
    if (g_sensor_mutex == NULL || adc_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop ADC
    esp_err_t err = adc_continuous_stop(adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "Failed to stop ADC: %s", esp_err_to_name(err));
        return err;
    }

    // Delete task
    if (g_sensor_task_handle != NULL) {
        vTaskDelete(g_sensor_task_handle);
        g_sensor_task_handle = NULL;
    }

    // Delete ADC handle
    err = adc_continuous_deinit(adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "Failed to delete ADC handle: %s", esp_err_to_name(err));
        return err;
    }
    adc_handle = NULL;

    ESP_LOGI("SENSOR", "Sensor reading stopped");
    return ESP_OK;
}

// Get sensor data
esp_err_t sensor_get_data(sensor_data_t *data) {
    if (g_sensor_mutex == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensor_get_data_fast(data);
}

esp_err_t sensor_get_data_fast(sensor_data_t *data) {
    if (g_sensor_mutex == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t attempt = 0; attempt < 8; attempt++) {
        uint32_t seq1 = __atomic_load_n(&g_sensor_seq, __ATOMIC_ACQUIRE);
        if (seq1 & 1U) {
            continue;
        }
        memcpy(data, &g_sensor_data, sizeof(sensor_data_t));
        uint32_t seq2 = __atomic_load_n(&g_sensor_seq, __ATOMIC_ACQUIRE);
        if (seq1 == seq2) {
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

// Set sensor configuration
esp_err_t sensor_set_config(const sensor_config_t *config) {
    if (g_sensor_mutex == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
        g_sensor_config = *config;
        xSemaphoreGive(g_sensor_mutex);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Get sensor configuration
esp_err_t sensor_get_config(sensor_config_t *config) {
    if (g_sensor_mutex == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
        *config = g_sensor_config;
        xSemaphoreGive(g_sensor_mutex);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Calibrate sensor
esp_err_t sensor_calibrate(sensor_channel_t channel, uint16_t raw_value, float engineering_value) {
    if (g_sensor_mutex == NULL || channel >= SENSOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Calibration would be implemented here
    // For now, just log the calibration request
    ESP_LOGI("SENSOR", "Calibration requested for channel %d: raw=%u, eng=%.2f",
             channel, raw_value, engineering_value);
    return ESP_OK;
}

// Sensor processing task
void process_sensors_task(void *pvParameters) {
    uint8_t result[256];
    uint32_t ret_num = 0;

    while (1) {
        // Read data from DMA
        if (adc_continuous_read(adc_handle, result, 256, &ret_num, 0) == ESP_OK) {
            if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
                // Process each sample
                __atomic_fetch_add(&g_sensor_seq, 1U, __ATOMIC_RELEASE); // odd: write in progress
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (void*)&result[i];
                    uint32_t chan = p->type2.channel;
                    uint32_t val = p->type2.data;

                    if (chan >= SENSOR_COUNT) {
                        continue;
                    }

                    // Store raw value
                    g_sensor_data.raw_adc[chan] = val;

                    // Apply filters based on channel
                    switch (chan) {
                        case SENSOR_MAP:
                            // Filtro média móvel de 16 amostras
                            map_filter_buffer[map_filter_index++] = val;
                            if (map_filter_index >= 16) map_filter_index = 0;
                            
                            uint32_t sum = 0;
                            for (int j = 0; j < 16; j++) {
                                sum += map_filter_buffer[j];
                            }
                            float map_kpa = adc_to_range(sum / 16, MAP_SENSOR_MIN, MAP_SENSOR_MAX);
                            g_sensor_data.map_kpa10 = (uint16_t)(map_kpa * 10.0f);
                            break;
                            
                        case SENSOR_TPS:
                            // Filtro passa-baixas de 1ª ordem
                            tps_filtered = (tps_filtered * (1.0f - g_sensor_config.tps_filter_alpha)) +
                                           (val * g_sensor_config.tps_filter_alpha);
                            g_sensor_data.tps_percent = (uint16_t)(adc_to_range((uint32_t)tps_filtered, TPS_SENSOR_MIN, TPS_SENSOR_MAX));
                            break;

                        case SENSOR_CLT:
                            if ((low_rate_decimator & 0x03) != 0) {
                                break;
                            }
                            // Filtro exponencial forte
                            clt_filtered = (clt_filtered * (1.0f - g_sensor_config.temp_filter_alpha)) +
                                           (val * g_sensor_config.temp_filter_alpha);
                            g_sensor_data.clt_c = (int16_t)(adc_to_range((uint32_t)clt_filtered, CLT_SENSOR_MIN, CLT_SENSOR_MAX));
                            break;

                        case SENSOR_IAT:
                            if ((low_rate_decimator & 0x03) != 0) {
                                break;
                            }
                            // Filtro exponencial forte
                            iat_filtered = (iat_filtered * (1.0f - g_sensor_config.temp_filter_alpha)) +
                                           (val * g_sensor_config.temp_filter_alpha);
                            g_sensor_data.iat_c = (int16_t)(adc_to_range((uint32_t)iat_filtered, IAT_SENSOR_MIN, IAT_SENSOR_MAX));
                            break;

                        case SENSOR_O2:
                            if ((low_rate_decimator & 0x03) != 0) {
                                break;
                            }
                            g_sensor_data.o2_mv = (uint16_t)(adc_to_range(val, O2_SENSOR_MIN, O2_SENSOR_MAX) * 1000.0f);
                            break;

                        case SENSOR_VBAT:
                            if ((low_rate_decimator & 0x03) != 0) {
                                break;
                            }
                            g_sensor_data.vbat_dv = (uint16_t)(adc_to_range(val, VBAT_SENSOR_MIN, VBAT_SENSOR_MAX) * 10.0f);
                            break;

                        case SENSOR_SPARE:
                            if ((low_rate_decimator & 0x03) != 0) {
                                break;
                            }
                            g_sensor_data.spare_mv = (uint16_t)(adc_to_range(val, 0.0f, 5.0f) * 1000.0f);
                            break;
                    }
                }

                // Update statistics
                g_sensor_data.sample_count++;
                low_rate_decimator++;
                __atomic_fetch_add(&g_sensor_seq, 1U, __ATOMIC_RELEASE); // even: stable snapshot
                xSemaphoreGive(g_sensor_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Processa a cada 1ms
    }
}

static float adc_to_range(uint32_t adc, float min_val, float max_val) {
    const float adc_max = 4095.0f;
    if (max_val <= min_val) {
        return min_val;
    }
    float ratio = (float)adc / adc_max;
    if (ratio < 0.0f) {
        ratio = 0.0f;
    } else if (ratio > 1.0f) {
        ratio = 1.0f;
    }
    return min_val + (max_val - min_val) * ratio;
}
