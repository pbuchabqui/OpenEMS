/**
 * @file mock_esp_idf.h
 * @brief Mock implementation for ESP-IDF components
 * 
 * Provides controllable ESP-IDF functions for unit testing
 * without hardware dependencies.
 */

#ifndef MOCK_ESP_IDF_H
#define MOCK_ESP_IDF_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "unity.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mock ESP-IDF state
typedef struct {
    uint32_t esp_timer_value;
    bool esp_timer_initialized;
    uint32_t freertos_tick_count;
    bool freertos_running;
    uint32_t nvs_init_count;
    bool nvs_initialized;
    uint32_t can_init_count;
    bool can_initialized;
} mock_esp_idf_state_t;

extern mock_esp_idf_state_t g_mock_esp_idf;

// Mock control functions
void mock_esp_idf_reset(void);
void mock_esp_idf_set_timer_value(uint64_t time_us);
void mock_esp_idf_increment_timer(uint32_t increment_us);
void mock_esp_idf_set_freertos_tick(uint32_t tick);

// Mocked ESP-IDF functions
int64_t esp_timer_get_time(void);
esp_err_t esp_timer_init(void);
esp_err_t esp_timer_deinit(void);

// FreeRTOS mocks
uint32_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);
void vTaskDelay(uint32_t ticks);
BaseType_t xTaskCreate(TaskFunction_t pvTaskCode,
                      const char * const pcName,
                      uint32_t usStackDepth,
                      void *pvParameters,
                      UBaseType_t uxPriority,
                      TaskHandle_t *pvCreatedTask);

// NVS mocks
esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_deinit(void);
esp_err_t nvs_open(const char* name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle);
esp_err_t nvs_close(nvs_handle_t handle);
esp_err_t nvs_get_u32(nvs_handle_t handle, const char* key, uint32_t* out_value);
esp_err_t nvs_set_u32(nvs_handle_t handle, const char* key, uint32_t value);
esp_err_t nvs_commit(nvs_handle_t handle);

// CAN/TWAI mocks
esp_err_t twai_driver_install(const twai_general_config_t* g_config,
                              const twai_timing_config_t* t_config,
                              const twai_filter_config_t* f_config);
esp_err_t twai_driver_uninstall(void);
esp_err_t twai_start(void);
esp_err_t twai_stop(void);
esp_err_t twai_transmit(const twai_message_t* message, TickType_t ticks_to_wait);
esp_err_t twai_receive(twai_message_t* message, TickType_t ticks_to_wait);

// Logging mocks
void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...);

// Test helper macros
#define MOCK_ESP_IDF_ASSERT_TIMER_VALUE(expected) \
    TEST_ASSERT_EQUAL_UINT64_MESSAGE(expected, g_mock_esp_idf.esp_timer_value, \
        "ESP timer value mismatch")

#define MOCK_ESP_IDF_ASSERT_NVS_INITIALIZED(expected) \
    TEST_ASSERT_EQUAL_MESSAGE(expected, g_mock_esp_idf.nvs_initialized, \
        "NVS initialization state mismatch")

#ifdef __cplusplus
}
#endif

#endif // MOCK_ESP_IDF_H
