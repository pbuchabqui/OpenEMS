#include "logger.h"
#include "esp_log.h"
#include <stdarg.h>
#include <string.h>

static logger_config_t g_logger_cfg;

static const char *g_log_tags[LOG_CAT_MAX] = {
    "ENGINE",
    "SENSORS",
    "INJECTION",
    "IGNITION",
    "SAFETY",
    "CAN",
    "SYSTEM",
    "DEBUG"
};

static esp_log_level_t level_to_esp(log_level_t level) {
    switch (level) {
        case LOG_LEVEL_ERROR:
            return ESP_LOG_ERROR;
        case LOG_LEVEL_WARN:
            return ESP_LOG_WARN;
        case LOG_LEVEL_INFO:
            return ESP_LOG_INFO;
        case LOG_LEVEL_DEBUG:
            return ESP_LOG_DEBUG;
        case LOG_LEVEL_VERBOSE:
            return ESP_LOG_VERBOSE;
        default:
            return ESP_LOG_NONE;
    }
}

void logger_init(void) {
    logger_reset_config();
}

void logger_set_level(log_category_t category, log_level_t level) {
    if (category >= LOG_CAT_MAX) {
        return;
    }
    g_logger_cfg.level[category] = level;
}

log_level_t logger_get_level(log_category_t category) {
    if (category >= LOG_CAT_MAX) {
        return LOG_LEVEL_NONE;
    }
    return g_logger_cfg.level[category];
}

void logger_log(log_category_t category, log_level_t level,
                const char* file, uint32_t line, const char* function,
                const char* format, ...) {
    (void)file;
    (void)line;
    (void)function;

    if (category >= LOG_CAT_MAX) {
        return;
    }
    if (level == LOG_LEVEL_NONE || level > g_logger_cfg.level[category]) {
        return;
    }

    va_list args;
    va_start(args, format);
    esp_log_writev(level_to_esp(level), g_log_tags[category], format, args);
    va_end(args);
}

void logger_log_engine_status(void) {
    logger_log(LOG_CAT_ENGINE, LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__,
               "Engine status update");
}

void logger_log_sensor_status(void) {
    logger_log(LOG_CAT_SENSORS, LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__,
               "Sensor status update");
}

void logger_log_injection_status(void) {
    logger_log(LOG_CAT_INJECTION, LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__,
               "Injection status update");
}

void logger_log_ignition_status(void) {
    logger_log(LOG_CAT_IGNITION, LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__,
               "Ignition status update");
}

void logger_log_safety_event(const char* event_type, uint32_t value) {
    logger_log(LOG_CAT_SAFETY, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__,
               "Safety event %s: %u", event_type ? event_type : "unknown", value);
}

void logger_log_can_message(const char* message) {
    logger_log(LOG_CAT_CAN, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__,
               "CAN: %s", message ? message : "");
}

const logger_config_t* logger_get_config(void) {
    return &g_logger_cfg;
}

void logger_set_config(const logger_config_t *config) {
    if (!config) {
        return;
    }
    g_logger_cfg = *config;
}

void logger_reset_config(void) {
    memset(&g_logger_cfg, 0, sizeof(g_logger_cfg));
    for (int i = 0; i < LOG_CAT_MAX; i++) {
        g_logger_cfg.level[i] = LOG_LEVEL_INFO;
    }
    g_logger_cfg.timestamp_enabled = true;
    g_logger_cfg.category_enabled = true;
    g_logger_cfg.color_enabled = true;
    g_logger_cfg.buffer_size = 0;
}

void logger_flush(void) {
}

void logger_set_timestamp_enabled(bool enabled) {
    g_logger_cfg.timestamp_enabled = enabled;
}

void logger_set_category_enabled(bool enabled) {
    g_logger_cfg.category_enabled = enabled;
}

void logger_set_color_enabled(bool enabled) {
    g_logger_cfg.color_enabled = enabled;
}
