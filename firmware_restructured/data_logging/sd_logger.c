/**
 * @file data_logger.c
 * @brief Data Logger Module Implementation
 * 
 * This module provides recording capabilities for performance analysis,
 * tuning, and diagnostics.
 */

#include "logging/sd_logger.h"
#include "esp_log.h"
#include "hal/hal_timer.h"
#include "esp_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

// Include engine control headers
#include "engine_control.h"
#include "sensor_processing.h"
#include "decoder/trigger_60_2.h"
#include "diagnostics/fault_manager.h"

/*============================================================================
 * Constants
 *============================================================================*/

static const char *TAG = "data_logger";

/** @brief Logger task stack size */
#define LOGGER_TASK_STACK_SIZE   4096

/** @brief Logger task priority */
#define LOGGER_TASK_PRIORITY     2

/*============================================================================
 * Circular Buffer Structure
 *============================================================================*/

typedef struct {
    log_entry_t *buffer;          /**< Buffer memory */
    uint32_t    capacity;         /**< Number of entries */
    uint32_t    head;             /**< Write position */
    uint32_t    tail;             /**< Read position */
    uint32_t    count;            /**< Current entry count */
    bool        overwrite;        /**< Overwrite when full */
} log_circular_buffer_t;

/*============================================================================
 * Module State
 *============================================================================*/

typedef struct {
    // State
    bool                initialized;
    bool                logging;
    log_config_t        config;
    
    // Buffer
    log_circular_buffer_t buffer;
    log_entry_t         *buffer_memory;
    
    // Session
    log_session_header_t session;
    uint32_t            session_start_ms;
    
    // Statistics
    log_stats_t         stats;
    
    // Task
    TaskHandle_t        logger_task;
    
    // Mutex
    SemaphoreHandle_t   mutex;
    
    // Trigger state
    uint16_t            last_rpm;
    uint16_t            last_tps;
    uint16_t            last_map;
    bool                triggered;
    uint32_t            post_trigger_count;
} data_logger_t;

static data_logger_t g_logger = {
    .initialized = false,
    .logging = false,
    .buffer_memory = NULL,
    .logger_task = NULL,
    .mutex = NULL,
};

/*============================================================================
 * Circular Buffer Functions
 *============================================================================*/

static esp_err_t buffer_init(uint32_t capacity)
{
    if (g_logger.buffer_memory != NULL) {
        free(g_logger.buffer_memory);
    }
    
    g_logger.buffer_memory = calloc(capacity, sizeof(log_entry_t));
    if (g_logger.buffer_memory == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer memory");
        return ESP_ERR_NO_MEM;
    }
    
    g_logger.buffer.buffer = g_logger.buffer_memory;
    g_logger.buffer.capacity = capacity;
    g_logger.buffer.head = 0;
    g_logger.buffer.tail = 0;
    g_logger.buffer.count = 0;
    g_logger.buffer.overwrite = true;
    
    return ESP_OK;
}

static void buffer_deinit(void)
{
    if (g_logger.buffer_memory != NULL) {
        free(g_logger.buffer_memory);
        g_logger.buffer_memory = NULL;
    }
    g_logger.buffer.buffer = NULL;
    g_logger.buffer.capacity = 0;
    g_logger.buffer.count = 0;
}

static esp_err_t buffer_push(const log_entry_t *entry)
{
    if (g_logger.buffer.buffer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_logger.buffer.buffer[g_logger.buffer.head] = *entry;
    g_logger.buffer.head = (g_logger.buffer.head + 1) % g_logger.buffer.capacity;
    
    if (g_logger.buffer.count < g_logger.buffer.capacity) {
        g_logger.buffer.count++;
    } else {
        // Overwrite oldest
        g_logger.buffer.tail = (g_logger.buffer.tail + 1) % g_logger.buffer.capacity;
        g_logger.stats.buffer_overruns++;
    }
    
    return ESP_OK;
}

static esp_err_t buffer_get(uint32_t index, log_entry_t *entry)
{
    if (g_logger.buffer.buffer == NULL || entry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (index >= g_logger.buffer.count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t actual_index = (g_logger.buffer.tail + index) % g_logger.buffer.capacity;
    *entry = g_logger.buffer.buffer[actual_index];
    
    return ESP_OK;
}

static void buffer_clear(void)
{
    g_logger.buffer.head = 0;
    g_logger.buffer.tail = 0;
    g_logger.buffer.count = 0;
}

/*============================================================================
 * Trigger Detection
 *============================================================================*/

static bool check_triggers(const log_entry_t *entry)
{
    log_trigger_config_t *trigger = &g_logger.config.trigger;
    
    if (trigger->trigger_mask == 0) {
        return false;
    }
    
    // RPM above threshold
    if ((trigger->trigger_mask & LOG_TRIGGER_RPM_ABOVE) && 
        entry->rpm > trigger->rpm_high) {
        return true;
    }
    
    // RPM below threshold
    if ((trigger->trigger_mask & LOG_TRIGGER_RPM_BELOW) && 
        entry->rpm < trigger->rpm_low) {
        return true;
    }
    
    // TPS change
    if ((trigger->trigger_mask & LOG_TRIGGER_TPS_CHANGE) && 
        abs((int)entry->tps_pct10 - (int)g_logger.last_tps) > trigger->tps_delta) {
        return true;
    }
    
    // MAP change
    if ((trigger->trigger_mask & LOG_TRIGGER_MAP_CHANGE) && 
        abs((int)entry->map_kpa10 - (int)g_logger.last_map) > trigger->map_delta) {
        return true;
    }
    
    // Error condition
    if ((trigger->trigger_mask & LOG_TRIGGER_ERROR) && 
        entry->error_bitmap != 0) {
        return true;
    }
    
    // Sync loss
    if ((trigger->trigger_mask & LOG_TRIGGER_SYNC_LOSS) && 
        entry->sync_status == 0) {
        return true;
    }
    
    return false;
}

/*============================================================================
 * Logger Task
 *============================================================================*/

static void logger_task(void *arg)
{
    (void)arg;
    
    uint32_t last_capture_ms = 0;
    uint32_t interval_ms = 1000 / g_logger.config.sample_rate_hz;
    
    while (g_logger.logging) {
        uint32_t now_ms = (uint32_t)(HAL_Time_us() / 1000);
        
        if (now_ms - last_capture_ms >= interval_ms) {
            last_capture_ms = now_ms;
            
            // Capture entry
            log_entry_t entry = {0};
            entry.timestamp_ms = now_ms;
            
            // Get engine state
            engine_runtime_state_t state;
            uint32_t seq;
            engine_control_get_runtime_state(&state, &seq);
            
            entry.rpm = state.rpm;
            entry.map_kpa10 = state.load;
            entry.advance_deg10 = state.advance_deg10;
            entry.pw_us = state.pw_us;
            entry.lambda_target = (uint16_t)(state.lambda_target * 1000.0f);
            entry.lambda_measured = (uint16_t)(state.lambda_measured * 1000.0f);
            entry.sync_status = state.sync_status ? 1 : 0;
            entry.flags = state.limp_mode ? 0x01 : 0x00;
            
            // Get sensor data
            sensor_data_t sensors;
            if (sensor_get_data(&sensors) == ESP_OK) {
                entry.tps_pct10 = (uint16_t)(sensors.tps_pct * 10.0f);
                entry.clt_c10 = (int16_t)(sensors.clt_c * 10.0f);
                entry.iat_c10 = (int16_t)(sensors.iat_c * 10.0f);
                entry.o2_mv = (uint16_t)(sensors.o2_voltage * 1000.0f);
                entry.vbat_mv = (uint16_t)(sensors.vbat * 1000.0f);
            }
            
            // Get error bitmap
            limp_mode_t limp = safety_get_limp_mode_status();
            if (limp.active) {
                entry.error_bitmap |= (1 << 0);
            }
            
            // Check triggers
            if (!g_logger.triggered && check_triggers(&entry)) {
                g_logger.triggered = true;
                g_logger.post_trigger_count = 0;
                g_logger.session.trigger_type = g_logger.config.trigger.trigger_mask;
                g_logger.stats.trigger_count++;
                ESP_LOGI(TAG, "Trigger activated: 0x%04X", g_logger.config.trigger.trigger_mask);
            }
            
            // Store entry
            if (xSemaphoreTake(g_logger.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                buffer_push(&entry);
                g_logger.session.entry_count++;
                g_logger.stats.total_entries++;
                
                if (g_logger.triggered) {
                    g_logger.post_trigger_count++;
                }
                
                xSemaphoreGive(g_logger.mutex);
            }
            
            // Update last values for trigger detection
            g_logger.last_rpm = entry.rpm;
            g_logger.last_tps = entry.tps_pct10;
            g_logger.last_map = entry.map_kpa10;
            
            // Check if we should stop after post-trigger samples
            if (g_logger.triggered && 
                g_logger.post_trigger_count >= g_logger.config.trigger.post_trigger_samples &&
                g_logger.config.trigger.post_trigger_samples > 0) {
                ESP_LOGI(TAG, "Post-trigger samples captured, stopping");
                g_logger.logging = false;
            }
            
            // Check max session size
            if (g_logger.session.entry_count >= g_logger.config.max_session_size &&
                g_logger.config.max_session_size > 0) {
                ESP_LOGI(TAG, "Max session size reached, stopping");
                g_logger.logging = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    vTaskDelete(NULL);
}

/*============================================================================
 * Public API Implementation
 *============================================================================*/

esp_err_t data_logger_init(void)
{
    if (g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create mutex
    g_logger.mutex = xSemaphoreCreateMutex();
    if (g_logger.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Set default configuration
    memset(&g_logger.config, 0, sizeof(log_config_t));
    g_logger.config.enabled = true;
    g_logger.config.sample_rate_hz = LOG_DEFAULT_SAMPLE_RATE;
    g_logger.config.format = LOG_FORMAT_CSV;
    g_logger.config.storage_backend = LOG_STORAGE_STREAM;
    g_logger.config.buffer_size = LOG_DEFAULT_BUFFER_SIZE;
    g_logger.config.auto_export = false;
    g_logger.config.max_session_size = 0;  // Unlimited
    strcpy(g_logger.config.prefix, "log");
    g_logger.config.include_date = true;
    
    // Initialize buffer
    esp_err_t ret = buffer_init(g_logger.config.buffer_size);
    if (ret != ESP_OK) {
        vSemaphoreDelete(g_logger.mutex);
        g_logger.mutex = NULL;
        return ret;
    }
    
    // Reset statistics
    memset(&g_logger.stats, 0, sizeof(log_stats_t));
    
    g_logger.initialized = true;
    g_logger.logging = false;
    
    ESP_LOGI(TAG, "Data logger initialized");
    return ESP_OK;
}

esp_err_t data_logger_deinit(void)
{
    if (!g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop logging if active
    if (g_logger.logging) {
        data_logger_stop(false);
    }
    
    g_logger.initialized = false;
    
    // Free buffer
    buffer_deinit();
    
    // Delete mutex
    if (g_logger.mutex != NULL) {
        vSemaphoreDelete(g_logger.mutex);
        g_logger.mutex = NULL;
    }
    
    ESP_LOGI(TAG, "Data logger deinitialized");
    return ESP_OK;
}

esp_err_t data_logger_start(const char *name)
{
    if (!g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_logger.logging) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize session
    memset(&g_logger.session, 0, sizeof(log_session_header_t));
    g_logger.session.session_id = (uint32_t)(HAL_Time_us() / 1000);
    g_logger.session.start_time = g_logger.session.session_id;
    g_logger.session.sample_rate_hz = g_logger.config.sample_rate_hz;
    g_logger.session.format = g_logger.config.format;
    
    if (name != NULL) {
        strncpy(g_logger.session.name, name, LOG_SESSION_NAME_LEN - 1);
    } else {
        snprintf(g_logger.session.name, LOG_SESSION_NAME_LEN, "session_%lu", 
                 g_logger.session.session_id);
    }
    
    // Clear buffer
    buffer_clear();
    
    // Reset trigger state
    g_logger.triggered = (g_logger.config.trigger.trigger_mask == 0);
    g_logger.post_trigger_count = 0;
    g_logger.last_rpm = 0;
    g_logger.last_tps = 0;
    g_logger.last_map = 0;
    
    // Create logger task
    BaseType_t ret = xTaskCreate(
        logger_task,
        "logger",
        LOGGER_TASK_STACK_SIZE,
        NULL,
        LOGGER_TASK_PRIORITY,
        &g_logger.logger_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create logger task");
        return ESP_ERR_NO_MEM;
    }
    
    g_logger.logging = true;
    g_logger.session_start_ms = (uint32_t)(HAL_Time_us() / 1000);
    g_logger.stats.total_sessions++;
    
    ESP_LOGI(TAG, "Logging started: %s", g_logger.session.name);
    return ESP_OK;
}

esp_err_t data_logger_stop(bool export)
{
    if (!g_logger.initialized || !g_logger.logging) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_logger.logging = false;
    
    // Wait for task to finish
    vTaskDelay(pdMS_TO_TICKS(100));
    g_logger.logger_task = NULL;
    
    // Finalize session
    g_logger.session.end_time = (uint32_t)(HAL_Time_us() / 1000);
    
    // Calculate CRC
    g_logger.session.crc32 = esp_crc32_le(0, (const uint8_t *)g_logger.buffer_memory, 
                                          g_logger.buffer.count * sizeof(log_entry_t));
    
    ESP_LOGI(TAG, "Logging stopped: %lu entries", g_logger.session.entry_count);
    
    // Export if requested
    if (export) {
        data_logger_export(g_logger.config.format, NULL);
    }
    
    return ESP_OK;
}

bool data_logger_is_logging(void)
{
    return g_logger.logging;
}

esp_err_t data_logger_capture(void)
{
    if (!g_logger.initialized || !g_logger.logging) {
        return ESP_ERR_INVALID_STATE;
    }
    
    log_entry_t entry = {0};
    entry.timestamp_ms = (uint32_t)(HAL_Time_us() / 1000);
    
    // Get engine state
    engine_runtime_state_t state;
    uint32_t seq;
    engine_control_get_runtime_state(&state, &seq);
    
    entry.rpm = state.rpm;
    entry.map_kpa10 = state.load;
    entry.advance_deg10 = state.advance_deg10;
    entry.pw_us = state.pw_us;
    entry.lambda_target = (uint16_t)(state.lambda_target * 1000.0f);
    entry.lambda_measured = (uint16_t)(state.lambda_measured * 1000.0f);
    entry.sync_status = state.sync_status ? 1 : 0;
    
    // Get sensor data
    sensor_data_t sensors;
    if (sensor_get_data(&sensors) == ESP_OK) {
        entry.tps_pct10 = (uint16_t)(sensors.tps_pct * 10.0f);
        entry.clt_c10 = (int16_t)(sensors.clt_c * 10.0f);
        entry.iat_c10 = (int16_t)(sensors.iat_c * 10.0f);
        entry.o2_mv = (uint16_t)(sensors.o2_voltage * 1000.0f);
        entry.vbat_mv = (uint16_t)(sensors.vbat * 1000.0f);
    }
    
    if (xSemaphoreTake(g_logger.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        buffer_push(&entry);
        g_logger.session.entry_count++;
        g_logger.stats.total_entries++;
        xSemaphoreGive(g_logger.mutex);
    }
    
    return ESP_OK;
}

esp_err_t data_logger_trigger(void)
{
    if (!g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_logger.triggered = true;
    g_logger.post_trigger_count = 0;
    g_logger.session.trigger_type = LOG_TRIGGER_MANUAL;
    g_logger.stats.trigger_count++;
    
    ESP_LOGI(TAG, "Manual trigger activated");
    return ESP_OK;
}

void data_logger_get_config(log_config_t *config)
{
    if (config != NULL) {
        *config = g_logger.config;
    }
}

esp_err_t data_logger_set_config(const log_config_t *config)
{
    if (!g_logger.initialized || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_logger.logging) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate configuration
    if (config->sample_rate_hz < 1 || config->sample_rate_hz > LOG_MAX_SAMPLE_RATE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->buffer_size > LOG_MAX_BUFFER_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_logger.config = *config;
    
    // Reallocate buffer if size changed
    if (config->buffer_size != g_logger.buffer.capacity) {
        buffer_deinit();
        esp_err_t ret = buffer_init(config->buffer_size);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    return ESP_OK;
}

void data_logger_get_stats(log_stats_t *stats)
{
    if (stats != NULL) {
        *stats = g_logger.stats;
    }
}

esp_err_t data_logger_export(log_format_t format, const char *path)
{
    if (!g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_logger.buffer.count == 0) {
        ESP_LOGW(TAG, "No data to export");
        return ESP_ERR_INVALID_STATE;
    }
    
    (void)path;  // Not used for stream export
    
    ESP_LOGI(TAG, "Exporting %lu entries in format %d", g_logger.buffer.count, format);
    
    // For now, just log to console (stream export)
    if (format == LOG_FORMAT_CSV || format == LOG_FORMAT_BOTH) {
        ESP_LOGI(TAG, "timestamp_ms,rpm,map_kpa,tps_pct,clt_c,iat_c,o2_mv,vbat_mv,advance_deg,pw_us,lambda_target,lambda_measured,sync,flags,errors");
        
        for (uint32_t i = 0; i < g_logger.buffer.count && i < 100; i++) {
            log_entry_t entry;
            if (buffer_get(i, &entry) == ESP_OK) {
                ESP_LOGI(TAG, "%lu,%u,%.1f,%.1f,%.1f,%.1f,%u,%u,%.1f,%u,%.3f,%.3f,%u,%u,0x%04X",
                        entry.timestamp_ms,
                        entry.rpm,
                        entry.map_kpa10 / 10.0f,
                        entry.tps_pct10 / 10.0f,
                        entry.clt_c10 / 10.0f,
                        entry.iat_c10 / 10.0f,
                        entry.o2_mv,
                        entry.vbat_mv,
                        entry.advance_deg10 / 10.0f,
                        entry.pw_us,
                        entry.lambda_target / 1000.0f,
                        entry.lambda_measured / 1000.0f,
                        entry.sync_status,
                        entry.flags,
                        entry.error_bitmap);
            }
        }
        
        g_logger.stats.bytes_written += g_logger.buffer.count * sizeof(log_entry_t);
    }
    
    ESP_LOGI(TAG, "Export complete");
    return ESP_OK;
}

esp_err_t data_logger_clear(void)
{
    if (!g_logger.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(g_logger.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buffer_clear();
        g_logger.session.entry_count = 0;
        xSemaphoreGive(g_logger.mutex);
    }
    
    return ESP_OK;
}

uint32_t data_logger_get_entry_count(void)
{
    return g_logger.buffer.count;
}

esp_err_t data_logger_get_entry(uint32_t index, log_entry_t *entry)
{
    if (!g_logger.initialized || entry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return buffer_get(index, entry);
}

esp_err_t data_logger_set_trigger(const log_trigger_config_t *trigger)
{
    if (!g_logger.initialized || trigger == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_logger.config.trigger = *trigger;
    return ESP_OK;
}

esp_err_t data_logger_get_session(log_session_header_t *header)
{
    if (!g_logger.initialized || header == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *header = g_logger.session;
    return ESP_OK;
}
