#include sync.h"
#include logger.h"
#include s3_control_config.h"
#include "driver/pulse_cnt.h"
#include "driver/gptimer.h"
#include "driver/gptimer_etm.h"
#include "driver/gpio.h"
#include "driver/gpio_etm.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_etm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

// Static variables
static SemaphoreHandle_t g_sync_mutex = NULL;
static sync_data_t g_sync_data = {0};
static sync_config_t g_sync_config = {
    .tooth_count = 58,           // 60-2 wheel (58 teeth + 2 gap)
    .gap_tooth = 58,             // Gap occurs after tooth 58
    .max_rpm = 8000,
    .min_rpm = 500,
    .enable_phase_detection = true
};
static pcnt_unit_handle_t g_sync_pcnt_unit = NULL;
static pcnt_channel_handle_t g_sync_pcnt_chan = NULL;
static gptimer_handle_t g_sync_gptimer = NULL;
static esp_etm_channel_handle_t g_sync_etm_chan = NULL;
static esp_etm_event_handle_t g_sync_gpio_event = NULL;
static esp_etm_task_handle_t g_sync_timer_task = NULL;
static gptimer_handle_t g_cmp_gptimer = NULL;
static esp_etm_channel_handle_t g_cmp_etm_chan = NULL;
static esp_etm_event_handle_t g_cmp_gpio_event = NULL;
static esp_etm_task_handle_t g_cmp_timer_task = NULL;
static portMUX_TYPE g_sync_spinlock = portMUX_INITIALIZER_UNLOCKED;
static uint64_t g_last_capture_us = 0;
static uint64_t g_last_cmp_capture_us = 0;
#if SOC_GPTIMER_SUPPORT_ETM
static uint32_t g_timer_resolution_hz = 1000000;
#endif
static bool g_hw_sync_enabled = false;
static bool g_sync_use_watch_step = false;
static sync_tooth_callback_t g_tooth_cb = NULL;
static void *g_tooth_cb_ctx = NULL;
static const uint32_t SYNC_VALID_TIMEOUT_US = 200000U;

static void sync_update_from_capture(uint64_t capture_us, bool from_isr, bool emit_log);
static void sync_update_cmp_capture(uint64_t capture_us, bool from_isr);
static void sync_cmp_gpio_isr(void *arg);
static void sync_ckp_gpio_isr(void *arg);
static bool sync_pcnt_on_reach(pcnt_unit_handle_t unit,
                                         const pcnt_watch_event_data_t *edata,
                                         void *user_ctx);
static esp_err_t sync_init_hardware_capture(void);
static void sync_deinit_hardware_capture(void);

// Initialize SYNC module
esp_err_t sync_init(void) {
    if (g_sync_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    g_sync_mutex = xSemaphoreCreateMutex();
    if (g_sync_mutex == NULL) {
        ESP_LOGE("SYNC", "Failed to create mutex");
        return ESP_FAIL;
    }

    // Initialize PCNT driver (pulse count)
    pcnt_unit_config_t unit_config = {
        .low_limit = -1000,
        .high_limit = 1000,
        .intr_priority = 0,
        .flags = {
            .accum_count = 0,
        },
    };

    esp_err_t err = pcnt_new_unit(&unit_config, &g_sync_pcnt_unit);
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to create PCNT unit: %s", esp_err_to_name(err));
        return err;
    }

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1250, // ~100 cycles @ 80MHz
    };
    err = pcnt_unit_set_glitch_filter(g_sync_pcnt_unit, &filter_config);
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to set PCNT filter: %s", esp_err_to_name(err));
        return err;
    }

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = CKP_GPIO,
        .level_gpio_num = -1,
        .flags = {
            .invert_edge_input = 0,
            .invert_level_input = 0,
            .virt_edge_io_level = 0,
            .virt_level_io_level = 0,
            .io_loop_back = 0,
        },
    };
    err = pcnt_new_channel(g_sync_pcnt_unit, &chan_config, &g_sync_pcnt_chan);
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to create PCNT channel: %s", esp_err_to_name(err));
        return err;
    }

    err = pcnt_channel_set_edge_action(g_sync_pcnt_chan,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to set PCNT edge action: %s", esp_err_to_name(err));
        return err;
    }

    err = pcnt_channel_set_level_action(g_sync_pcnt_chan,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to set PCNT level action: %s", esp_err_to_name(err));
        return err;
    }

    // Add a watch event to get an event on every tooth
#if SOC_PCNT_SUPPORT_STEP_NOTIFY
    err = pcnt_unit_add_watch_step(g_sync_pcnt_unit, 1);
    if (err == ESP_OK) {
        g_sync_use_watch_step = true;
    } else {
        ESP_LOGW("SYNC", "PCNT watch step unavailable, falling back to watch point: %s", esp_err_to_name(err));
    }
#endif
    if (!g_sync_use_watch_step) {
        err = pcnt_unit_add_watch_point(g_sync_pcnt_unit, 1);
        if (err != ESP_OK) {
            ESP_LOGE("SYNC", "Failed to add PCNT watch point: %s", esp_err_to_name(err));
            return err;
        }
    }

    pcnt_event_callbacks_t cbs = {
        .on_reach = sync_pcnt_on_reach,
    };
    err = pcnt_unit_register_event_callbacks(g_sync_pcnt_unit, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to register PCNT callbacks: %s", esp_err_to_name(err));
        return err;
    }

    err = pcnt_unit_enable(g_sync_pcnt_unit);
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to enable PCNT unit: %s", esp_err_to_name(err));
        return err;
    }

    err = pcnt_unit_stop(g_sync_pcnt_unit);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW("SYNC", "PCNT stop warning: %s", esp_err_to_name(err));
    }
    err = pcnt_unit_clear_count(g_sync_pcnt_unit);
    if (err != ESP_OK) {
        ESP_LOGW("SYNC", "PCNT clear warning: %s", esp_err_to_name(err));
    }

    // Initialize hardware capture (GPIO -> ETM -> GPTimer capture)
    err = sync_init_hardware_capture();
    if (err != ESP_OK) {
        ESP_LOGE("SYNC", "Failed to initialize hardware capture: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI("SYNC", "SYNC module initialized");
    return ESP_OK;
}

// Deinitialize SYNC module
esp_err_t sync_deinit(void) {
    if (g_sync_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop and delete PCNT resources
    if (g_sync_pcnt_unit != NULL) {
        pcnt_unit_stop(g_sync_pcnt_unit);
        pcnt_unit_clear_count(g_sync_pcnt_unit);
        pcnt_unit_disable(g_sync_pcnt_unit);
    }
    if (g_sync_pcnt_chan != NULL) {
        pcnt_del_channel(g_sync_pcnt_chan);
        g_sync_pcnt_chan = NULL;
    }
    if (g_sync_pcnt_unit != NULL) {
        pcnt_del_unit(g_sync_pcnt_unit);
        g_sync_pcnt_unit = NULL;
    }

    sync_deinit_hardware_capture();

    // Delete mutex
    vSemaphoreDelete(g_sync_mutex);
    g_sync_mutex = NULL;

    ESP_LOGI("SYNC", "SYNC module deinitialized");
    return ESP_OK;
}

// Start SYNC module
esp_err_t sync_start(void) {
    if (g_sync_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_sync_mutex, portMAX_DELAY) == pdTRUE) {
        // Reset sync data
        portENTER_CRITICAL(&g_sync_spinlock);
        memset(&g_sync_data, 0, sizeof(g_sync_data));
        g_sync_data.last_tooth_time = 0;
        g_sync_data.last_capture_time = 0;
        g_sync_data.last_update_time = 0;
        g_sync_data.last_cmp_time = 0;
        g_sync_data.sync_valid = false;
        g_sync_data.sync_acquired = false;
        g_sync_data.cmp_seen = false;
        g_sync_data.cmp_tooth_index = 0;
        g_sync_data.revolution_index = 0;
        g_last_capture_us = 0;
        g_last_cmp_capture_us = 0;
        portEXIT_CRITICAL(&g_sync_spinlock);

        // Enable PCNT counter
        pcnt_unit_stop(g_sync_pcnt_unit);
        pcnt_unit_clear_count(g_sync_pcnt_unit);
        pcnt_unit_start(g_sync_pcnt_unit);

        if (g_sync_gptimer) {
            gptimer_set_raw_count(g_sync_gptimer, 0);
            gptimer_start(g_sync_gptimer);
        }
        if (g_cmp_gptimer) {
            gptimer_set_raw_count(g_cmp_gptimer, 0);
            gptimer_start(g_cmp_gptimer);
        }
#if SOC_GPTIMER_SUPPORT_ETM
        if (g_sync_etm_chan) {
            esp_etm_channel_enable(g_sync_etm_chan);
        }
        if (g_cmp_etm_chan) {
            esp_etm_channel_enable(g_cmp_etm_chan);
        }
#endif
        g_hw_sync_enabled = true;

        xSemaphoreGive(g_sync_mutex);
        ESP_LOGI("SYNC", "SYNC module started");
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Stop SYNC module
esp_err_t sync_stop(void) {
    if (g_sync_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_sync_mutex, portMAX_DELAY) == pdTRUE) {
        // Disable PCNT counter
        pcnt_unit_stop(g_sync_pcnt_unit);
        g_hw_sync_enabled = false;
#if SOC_GPTIMER_SUPPORT_ETM
        if (g_sync_etm_chan) {
            esp_etm_channel_disable(g_sync_etm_chan);
        }
        if (g_cmp_etm_chan) {
            esp_etm_channel_disable(g_cmp_etm_chan);
        }
#endif
        if (g_sync_gptimer) {
            gptimer_stop(g_sync_gptimer);
        }
        if (g_cmp_gptimer) {
            gptimer_stop(g_cmp_gptimer);
        }

        xSemaphoreGive(g_sync_mutex);
        ESP_LOGI("SYNC", "SYNC module stopped");
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Reset SYNC module
esp_err_t sync_reset(void) {
    if (g_sync_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_sync_mutex, portMAX_DELAY) == pdTRUE) {
        // Reset sync data
        portENTER_CRITICAL(&g_sync_spinlock);
        memset(&g_sync_data, 0, sizeof(g_sync_data));
        g_sync_data.last_tooth_time = 0;
        g_sync_data.last_capture_time = 0;
        g_sync_data.last_update_time = 0;
        g_sync_data.last_cmp_time = 0;
        g_sync_data.sync_valid = false;
        g_sync_data.sync_acquired = false;
        g_sync_data.cmp_seen = false;
        g_sync_data.cmp_tooth_index = 0;
        g_sync_data.revolution_index = 0;
        g_last_capture_us = 0;
        g_last_cmp_capture_us = 0;
        portEXIT_CRITICAL(&g_sync_spinlock);

        // Clear PCNT counter
        pcnt_unit_clear_count(g_sync_pcnt_unit);

        xSemaphoreGive(g_sync_mutex);
        ESP_LOGI("SYNC", "SYNC module reset");
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Get sync data
esp_err_t sync_get_data(sync_data_t *data) {
    if (g_sync_mutex == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&g_sync_spinlock);
    memcpy(data, &g_sync_data, sizeof(sync_data_t));
    portEXIT_CRITICAL(&g_sync_spinlock);

    uint32_t now_us = (uint32_t)esp_timer_get_time();
    
    // Handle timer overflow correctly
    // 32-bit µs overflow occurs every ~72 minutes
    if (data->last_capture_time == 0) {
        data->latency_us = UINT32_MAX;
        data->sync_valid = false;
        data->sync_acquired = false;
        return ESP_OK;
    }
    
    // Calculate latency with overflow handling
    if (now_us >= data->last_capture_time) {
        data->latency_us = now_us - data->last_capture_time;
    } else {
        // Overflow occurred - calculate wrapped difference
        data->latency_us = (UINT32_MAX - data->last_capture_time) + now_us;
    }

    data->sync_valid = (data->rpm > 0) && (data->latency_us < SYNC_VALID_TIMEOUT_US);
    if (!data->sync_valid) {
        data->sync_acquired = false;
    }
    return ESP_OK;
}

// Set sync configuration
esp_err_t sync_set_config(const sync_config_t *config) {
    if (g_sync_mutex == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->tooth_count == 0 ||
        config->gap_tooth > config->tooth_count ||
        config->min_rpm == 0 ||
        config->max_rpm < config->min_rpm) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&g_sync_spinlock);
    g_sync_config = *config;
    portEXIT_CRITICAL(&g_sync_spinlock);
    ESP_LOGI("SYNC", "SYNC configuration updated");
    return ESP_OK;
}

// Get sync configuration
esp_err_t sync_get_config(sync_config_t *config) {
    if (g_sync_mutex == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&g_sync_spinlock);
    *config = g_sync_config;
    portEXIT_CRITICAL(&g_sync_spinlock);
    return ESP_OK;
}

esp_err_t sync_register_tooth_callback(sync_tooth_callback_t cb, void *ctx) {
    portENTER_CRITICAL(&g_sync_spinlock);
    g_tooth_cb = cb;
    g_tooth_cb_ctx = ctx;
    portEXIT_CRITICAL(&g_sync_spinlock);
    return ESP_OK;
}

void sync_unregister_tooth_callback(void) {
    portENTER_CRITICAL(&g_sync_spinlock);
    g_tooth_cb = NULL;
    g_tooth_cb_ctx = NULL;
    portEXIT_CRITICAL(&g_sync_spinlock);
}

IRAM_ATTR static void sync_update_from_capture(uint64_t capture_us, bool from_isr, bool emit_log) {
    if (from_isr) {
        portENTER_CRITICAL_ISR(&g_sync_spinlock);
    } else {
        portENTER_CRITICAL(&g_sync_spinlock);
    }

    if (g_last_capture_us == 0) {
        g_last_capture_us = capture_us;
        g_sync_data.last_tooth_time = (uint32_t)capture_us;
        g_sync_data.last_capture_time = (uint32_t)capture_us;
        g_sync_data.last_update_time = (uint32_t)esp_timer_get_time();
        if (from_isr) {
            portEXIT_CRITICAL_ISR(&g_sync_spinlock);
        } else {
            portEXIT_CRITICAL(&g_sync_spinlock);
        }
        return;
    }
    if (capture_us <= g_last_capture_us) {
        g_last_capture_us = capture_us;
        g_sync_data.last_tooth_time = (uint32_t)capture_us;
        g_sync_data.last_capture_time = (uint32_t)capture_us;
        g_sync_data.sync_valid = false;
        g_sync_data.sync_acquired = false;
        if (from_isr) {
            portEXIT_CRITICAL_ISR(&g_sync_spinlock);
        } else {
            portEXIT_CRITICAL(&g_sync_spinlock);
        }
        return;
    }

    uint32_t tooth_period = (uint32_t)(capture_us - g_last_capture_us);
    g_last_capture_us = capture_us;
    g_sync_data.last_tooth_time = (uint32_t)capture_us;
    g_sync_data.last_capture_time = (uint32_t)capture_us;
    g_sync_data.last_update_time = (uint32_t)esp_timer_get_time();

    bool gap = false;
    if (g_sync_data.tooth_period > 0) {
        gap = (tooth_period > (g_sync_data.tooth_period * 3) / 2);
    }

    if (gap) {
        g_sync_data.gap_detected = true;
        g_sync_data.tooth_index = 0;
        g_sync_data.gap_period = tooth_period;
        if (g_sync_data.cmp_seen) {
            g_sync_data.phase_detected = true;
            g_sync_data.phase_detected_time = (uint32_t)capture_us;
            g_sync_data.revolution_index = 0;
        } else {
            g_sync_data.phase_detected = false;
            g_sync_data.revolution_index ^= 1;
        }
        g_sync_data.cmp_seen = false;
    } else {
        g_sync_data.gap_detected = false;
        if (g_sync_config.tooth_count > 0) {
            g_sync_data.tooth_index = (g_sync_data.tooth_index + 1) % g_sync_config.tooth_count;
        } else {
            g_sync_data.tooth_index = 0;
        }
    }

    if (gap) {
        g_sync_data.tooth_period = tooth_period / 3;
    } else {
        g_sync_data.tooth_period = tooth_period;
    }

    // Calculate time per degree using total positions (teeth + missing)
    uint32_t total_positions = g_sync_config.tooth_count + 2;
    if (total_positions > 0) {
        g_sync_data.time_per_degree = ((g_sync_data.tooth_period * total_positions) + 180U) / 360U;
    }

    if (!g_sync_config.enable_phase_detection) {
        g_sync_data.phase_detected = true;
    }

    if (g_sync_data.gap_detected && g_sync_data.phase_detected) {
        g_sync_data.sync_acquired = true;
    }
    if (g_sync_data.tooth_period > 0) {
        uint32_t positions_per_revolution = g_sync_config.tooth_count + 2;
        uint32_t time_per_revolution = g_sync_data.tooth_period * positions_per_revolution;
        if (time_per_revolution > 0) {
            g_sync_data.rpm = (1000000 * 60) / time_per_revolution;
            if (g_sync_data.rpm < g_sync_config.min_rpm) {
                g_sync_data.rpm = 0;
            } else if (g_sync_data.rpm > g_sync_config.max_rpm) {
                g_sync_data.rpm = g_sync_config.max_rpm;
            }
        }
    }

    g_sync_data.sync_valid = (g_sync_data.rpm > 0);
    if (!g_sync_data.sync_valid) {
        g_sync_data.sync_acquired = false;
    }

    if (from_isr) {
        portEXIT_CRITICAL_ISR(&g_sync_spinlock);
    } else {
        portEXIT_CRITICAL(&g_sync_spinlock);
    }

    if (emit_log && gap) {
        ESP_LOGI("SYNC", "Gap detected at tooth %" PRIu32, g_sync_data.tooth_index);
    }
}

IRAM_ATTR static void sync_update_cmp_capture(uint64_t capture_us, bool from_isr) {
    if (from_isr) {
        portENTER_CRITICAL_ISR(&g_sync_spinlock);
    } else {
        portENTER_CRITICAL(&g_sync_spinlock);
    }

    g_last_cmp_capture_us = capture_us;
    g_sync_data.last_cmp_time = (uint32_t)capture_us;
    g_sync_data.cmp_detected = true;
    g_sync_data.cmp_seen = true;
    g_sync_data.cmp_tooth_index = g_sync_data.tooth_index;

    if (from_isr) {
        portEXIT_CRITICAL_ISR(&g_sync_spinlock);
    } else {
        portEXIT_CRITICAL(&g_sync_spinlock);
    }
}

static void IRAM_ATTR sync_cmp_gpio_isr(void *arg) {
    (void)arg;
    if (!g_hw_sync_enabled) {
        return;
    }

    if (g_cmp_gptimer) {
        uint64_t capture_us = 0;
        gptimer_get_captured_count(g_cmp_gptimer, &capture_us);
        sync_update_cmp_capture(capture_us, true);
        return;
    }

    uint64_t capture_us = esp_timer_get_time();
    sync_update_cmp_capture(capture_us, true);
}

static void IRAM_ATTR sync_ckp_gpio_isr(void *arg) {
    (void)arg;
    if (!g_hw_sync_enabled) {
        return;
    }
    uint64_t capture_us = esp_timer_get_time();
    sync_update_from_capture(capture_us, true, false);
}

static bool IRAM_ATTR sync_pcnt_on_reach(pcnt_unit_handle_t unit,
                                         const pcnt_watch_event_data_t *edata,
                                         void *user_ctx) {
    (void)unit;
    (void)edata;
    (void)user_ctx;

    if (!g_hw_sync_enabled) {
        return false;
    }

    uint64_t capture_us = 0;
    if (g_sync_gptimer) {
        gptimer_get_captured_count(g_sync_gptimer, &capture_us);
    } else {
        capture_us = esp_timer_get_time();
    }
    sync_update_from_capture(capture_us, true, false);
    if (!g_sync_use_watch_step) {
        pcnt_unit_clear_count(unit);
    }
    sync_tooth_callback_t cb = g_tooth_cb;
    void *cb_ctx = g_tooth_cb_ctx;
    if (cb != NULL) {
        cb(cb_ctx);
    }
    return false;
}

static esp_err_t sync_init_hardware_capture(void) {
#if SOC_GPTIMER_SUPPORT_ETM
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = g_timer_resolution_hz,
        .intr_priority = 0,
        .flags = {
            .intr_shared = 0,
            .allow_pd = 0,
        },
    };

    esp_err_t err = gptimer_new_timer(&timer_config, &g_sync_gptimer);
    if (err != ESP_OK) {
        return err;
    }

    err = gptimer_enable(g_sync_gptimer);
    if (err != ESP_OK) {
        return err;
    }

    gptimer_etm_task_config_t gptimer_task_conf = {
        .task_type = GPTIMER_ETM_TASK_CAPTURE,
    };
    err = gptimer_new_etm_task(g_sync_gptimer, &gptimer_task_conf, &g_sync_timer_task);
    if (err != ESP_OK) {
        return err;
    }

    gpio_etm_event_config_t gpio_event_config = {
        .edge = GPIO_ETM_EVENT_EDGE_POS,
    };
    err = gpio_new_etm_event(&gpio_event_config, &g_sync_gpio_event);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_etm_event_bind_gpio(g_sync_gpio_event, CKP_GPIO);
    if (err != ESP_OK) {
        return err;
    }

    esp_etm_channel_config_t etm_config = {};
    err = esp_etm_new_channel(&etm_config, &g_sync_etm_chan);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_etm_channel_connect(g_sync_etm_chan, g_sync_gpio_event, g_sync_timer_task);
    if (err != ESP_OK) {
        return err;
    }

    gptimer_config_t cmp_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = g_timer_resolution_hz,
        .intr_priority = 0,
        .flags = {
            .intr_shared = 0,
            .allow_pd = 0,
        },
    };

    err = gptimer_new_timer(&cmp_timer_config, &g_cmp_gptimer);
    if (err != ESP_OK) {
        return err;
    }

    err = gptimer_enable(g_cmp_gptimer);
    if (err != ESP_OK) {
        return err;
    }

    gptimer_etm_task_config_t cmp_task_conf = {
        .task_type = GPTIMER_ETM_TASK_CAPTURE,
    };
    err = gptimer_new_etm_task(g_cmp_gptimer, &cmp_task_conf, &g_cmp_timer_task);
    if (err != ESP_OK) {
        return err;
    }

    gpio_etm_event_config_t cmp_event_config = {
        .edge = GPIO_ETM_EVENT_EDGE_POS,
    };
    err = gpio_new_etm_event(&cmp_event_config, &g_cmp_gpio_event);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_etm_event_bind_gpio(g_cmp_gpio_event, CMP_GPIO);
    if (err != ESP_OK) {
        return err;
    }

    esp_etm_channel_config_t cmp_etm_config = {};
    err = esp_etm_new_channel(&cmp_etm_config, &g_cmp_etm_chan);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_etm_channel_connect(g_cmp_etm_chan, g_cmp_gpio_event, g_cmp_timer_task);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t cmp_gpio_cfg = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << CMP_GPIO,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    err = gpio_config(&cmp_gpio_cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = gpio_isr_handler_add(CMP_GPIO, sync_cmp_gpio_isr, NULL);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
#else
    gpio_config_t cmp_gpio_cfg = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << CMP_GPIO,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t err = gpio_config(&cmp_gpio_cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = gpio_isr_handler_add(CMP_GPIO, sync_cmp_gpio_isr, NULL);
    if (err != ESP_OK) {
        return err;
    }

    g_sync_gptimer = NULL;
    g_cmp_gptimer = NULL;
    g_sync_etm_chan = NULL;
    g_cmp_etm_chan = NULL;
    g_sync_gpio_event = NULL;
    g_cmp_gpio_event = NULL;
    g_sync_timer_task = NULL;
    g_cmp_timer_task = NULL;
    return ESP_OK;
#endif
}

static void sync_deinit_hardware_capture(void) {
#if SOC_GPTIMER_SUPPORT_ETM
    if (g_sync_etm_chan) {
        esp_etm_channel_disable(g_sync_etm_chan);
    }

    if (g_sync_gptimer) {
        gptimer_stop(g_sync_gptimer);
        gptimer_disable(g_sync_gptimer);
    }

    if (g_cmp_etm_chan) {
        esp_etm_channel_disable(g_cmp_etm_chan);
    }

    if (g_cmp_gptimer) {
        gptimer_stop(g_cmp_gptimer);
        gptimer_disable(g_cmp_gptimer);
    }

    if (g_sync_timer_task) {
        esp_etm_del_task(g_sync_timer_task);
        g_sync_timer_task = NULL;
    }

    if (g_sync_gpio_event) {
        esp_etm_del_event(g_sync_gpio_event);
        g_sync_gpio_event = NULL;
    }

    if (g_sync_etm_chan) {
        esp_etm_del_channel(g_sync_etm_chan);
        g_sync_etm_chan = NULL;
    }

    if (g_sync_gptimer) {
        gptimer_del_timer(g_sync_gptimer);
        g_sync_gptimer = NULL;
    }

    if (g_cmp_timer_task) {
        esp_etm_del_task(g_cmp_timer_task);
        g_cmp_timer_task = NULL;
    }

    if (g_cmp_gpio_event) {
        esp_etm_del_event(g_cmp_gpio_event);
        g_cmp_gpio_event = NULL;
    }

    if (g_cmp_etm_chan) {
        esp_etm_del_channel(g_cmp_etm_chan);
        g_cmp_etm_chan = NULL;
    }

    if (g_cmp_gptimer) {
        gptimer_del_timer(g_cmp_gptimer);
        g_cmp_gptimer = NULL;
    }

    gpio_isr_handler_remove(CMP_GPIO);
#else
    gpio_isr_handler_remove(CMP_GPIO);
#endif
}
