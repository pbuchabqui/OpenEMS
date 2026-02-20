#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Categorias de log
typedef enum {
    LOG_CAT_ENGINE = 0,
    LOG_CAT_SENSORS,
    LOG_CAT_INJECTION,
    LOG_CAT_IGNITION,
    LOG_CAT_SAFETY,
    LOG_CAT_CAN,
    LOG_CAT_SYSTEM,
    LOG_CAT_DEBUG,
    LOG_CAT_MAX
} log_category_t;

// Níveis de log
typedef enum {
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_VERBOSE
} log_level_t;

// Estrutura de configuração de logging
typedef struct {
    log_level_t level[LOG_CAT_MAX];
    bool timestamp_enabled;
    bool category_enabled;
    bool color_enabled;
    uint32_t buffer_size;
} logger_config_t;

// Estrutura de entrada de log
typedef struct {
    uint32_t timestamp_ms;
    log_category_t category;
    log_level_t level;
    uint32_t thread_id;
    const char* file;
    uint32_t line;
    const char* function;
    const char* message;
} log_entry_t;

/**
 * @brief Initialize logging system
 */
void logger_init(void);

/**
 * @brief Set log level for a category
 * @param category Log category
 * @param level Log level
 */
void logger_set_level(log_category_t category, log_level_t level);

/**
 * @brief Get current log level for a category
 * @param category Log category
 * @return Current log level
 */
log_level_t logger_get_level(log_category_t category);

/**
 * @brief Log a message
 * @param category Log category
 * @param level Log level
 * @param file Source file
 * @param line Line number
 * @param function Function name
 * @param format Format string
 * @param ... Arguments
 */
void logger_log(log_category_t category, log_level_t level, 
                const char* file, uint32_t line, const char* function,
                const char* format, ...);

/**
 * @brief Log engine status
 */
void logger_log_engine_status(void);

/**
 * @brief Log sensor status
 */
void logger_log_sensor_status(void);

/**
 * @brief Log injection status
 */
void logger_log_injection_status(void);

/**
 * @brief Log ignition status
 */
void logger_log_ignition_status(void);

/**
 * @brief Log safety events
 * @param event_type Type of safety event
 * @param value Event value
 */
void logger_log_safety_event(const char* event_type, uint32_t value);

/**
 * @brief Log CAN message
 * @param message CAN message details
 */
void logger_log_can_message(const char* message);

/**
 * @brief Get logger configuration
 * @return Pointer to logger configuration
 */
const logger_config_t* logger_get_config(void);

/**
 * @brief Set logger configuration
 * @param config Pointer to logger configuration
 */
void logger_set_config(const logger_config_t *config);

/**
 * @brief Reset logger configuration to defaults
 */
void logger_reset_config(void);

/**
 * @brief Flush log buffer
 */
void logger_flush(void);

/**
 * @brief Enable/disable timestamp in logs
 * @param enabled True to enable
 */
void logger_set_timestamp_enabled(bool enabled);

/**
 * @brief Enable/disable category in logs
 * @param enabled True to enable
 */
void logger_set_category_enabled(bool enabled);

/**
 * @brief Enable/disable color in logs
 * @param enabled True to enable
 */
void logger_set_color_enabled(bool enabled);

// Macros de conveniência para logging
#define LOG_ENGINE_E(fmt, ...) \
    logger_log(LOG_CAT_ENGINE, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_ENGINE_W(fmt, ...) \
    logger_log(LOG_CAT_ENGINE, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_ENGINE_I(fmt, ...) \
    logger_log(LOG_CAT_ENGINE, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_ENGINE_D(fmt, ...) \
    logger_log(LOG_CAT_ENGINE, LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SENSORS_E(fmt, ...) \
    logger_log(LOG_CAT_SENSORS, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SENSORS_W(fmt, ...) \
    logger_log(LOG_CAT_SENSORS, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SENSORS_I(fmt, ...) \
    logger_log(LOG_CAT_SENSORS, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_INJECTION_E(fmt, ...) \
    logger_log(LOG_CAT_INJECTION, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_INJECTION_W(fmt, ...) \
    logger_log(LOG_CAT_INJECTION, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_INJECTION_I(fmt, ...) \
    logger_log(LOG_CAT_INJECTION, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_IGNITION_E(fmt, ...) \
    logger_log(LOG_CAT_IGNITION, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_IGNITION_W(fmt, ...) \
    logger_log(LOG_CAT_IGNITION, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_IGNITION_I(fmt, ...) \
    logger_log(LOG_CAT_IGNITION, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_IGNITION_D(fmt, ...) \
    logger_log(LOG_CAT_IGNITION, LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SAFETY_E(fmt, ...) \
    logger_log(LOG_CAT_SAFETY, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SAFETY_W(fmt, ...) \
    logger_log(LOG_CAT_SAFETY, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SAFETY_I(fmt, ...) \
    logger_log(LOG_CAT_SAFETY, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_CAN_E(fmt, ...) \
    logger_log(LOG_CAT_CAN, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_CAN_W(fmt, ...) \
    logger_log(LOG_CAT_CAN, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_CAN_I(fmt, ...) \
    logger_log(LOG_CAT_CAN, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SYSTEM_E(fmt, ...) \
    logger_log(LOG_CAT_SYSTEM, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SYSTEM_W(fmt, ...) \
    logger_log(LOG_CAT_SYSTEM, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_SYSTEM_I(fmt, ...) \
    logger_log(LOG_CAT_SYSTEM, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_DEBUG_E(fmt, ...) \
    logger_log(LOG_CAT_DEBUG, LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_DEBUG_W(fmt, ...) \
    logger_log(LOG_CAT_DEBUG, LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_DEBUG_I(fmt, ...) \
    logger_log(LOG_CAT_DEBUG, LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#define LOG_DEBUG_D(fmt, ...) \
    logger_log(LOG_CAT_DEBUG, LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
