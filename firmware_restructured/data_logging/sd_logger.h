/**
 * @file data_logger.h
 * @brief Data Logger Module for ESP32-S3 EFI
 * 
 * This module provides recording capabilities for performance analysis,
 * tuning, and diagnostics.
 * 
 * Features:
 * - Circular buffer for continuous logging
 * - Trigger-based logging (RPM, error, manual)
 * - CSV and binary export formats
 * - SD card and flash storage support
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants and Configuration
 *============================================================================*/

/** @brief Log entry size in bytes */
#define LOG_ENTRY_SIZE              28

/** @brief Default buffer size (entries) */
#define LOG_DEFAULT_BUFFER_SIZE     1000

/** @brief Maximum buffer size (entries) */
#define LOG_MAX_BUFFER_SIZE         10000

/** @brief Maximum session name length */
#define LOG_SESSION_NAME_LEN        32

/** @brief Maximum file prefix length */
#define LOG_PREFIX_LEN              16

/** @brief Default sample rate (Hz) */
#define LOG_DEFAULT_SAMPLE_RATE     100

/** @brief Maximum sample rate (Hz) */
#define LOG_MAX_SAMPLE_RATE         1000

/*============================================================================
 * Types and Structures
 *============================================================================*/

/**
 * @brief Log entry structure
 */
typedef struct __attribute__((packed)) {
    uint32_t    timestamp_ms;     /**< System timestamp */
    uint16_t    rpm;              /**< Engine RPM */
    uint16_t    map_kpa10;        /**< MAP * 10 */
    uint16_t    tps_pct10;        /**< TPS * 10 */
    int16_t     clt_c10;          /**< CLT * 10 */
    int16_t     iat_c10;          /**< IAT * 10 */
    uint16_t    o2_mv;            /**< O2 voltage */
    uint16_t    vbat_mv;          /**< Battery voltage */
    uint16_t    advance_deg10;    /**< Ignition advance * 10 */
    uint16_t    pw_us;            /**< Injection pulse width */
    uint16_t    lambda_target;    /**< Lambda target * 1000 */
    uint16_t    lambda_measured;  /**< Lambda measured * 1000 */
    uint8_t     sync_status;      /**< Sync state */
    uint8_t     flags;            /**< Status flags */
    uint16_t    error_bitmap;     /**< Active errors */
} log_entry_t;

/**
 * @brief Trigger types
 */
typedef enum {
    LOG_TRIGGER_NONE       = 0,
    LOG_TRIGGER_RPM_ABOVE  = (1 << 0),  /**< RPM above threshold */
    LOG_TRIGGER_RPM_BELOW  = (1 << 1),  /**< RPM below threshold */
    LOG_TRIGGER_ERROR      = (1 << 2),  /**< On error condition */
    LOG_TRIGGER_WARNING    = (1 << 3),  /**< On warning condition */
    LOG_TRIGGER_TPS_CHANGE = (1 << 4),  /**< TPS change rate */
    LOG_TRIGGER_MAP_CHANGE = (1 << 5),  /**< MAP change rate */
    LOG_TRIGGER_MANUAL     = (1 << 6),  /**< Manual trigger */
    LOG_TRIGGER_SYNC_LOSS  = (1 << 7),  /**< Sync loss event */
} log_trigger_type_t;

/**
 * @brief Log format types
 */
typedef enum {
    LOG_FORMAT_CSV     = 0,        /**< CSV format */
    LOG_FORMAT_BINARY  = 1,        /**< Binary format */
    LOG_FORMAT_BOTH    = 2,        /**< Both formats */
} log_format_t;

/**
 * @brief Storage backend types
 */
typedef enum {
    LOG_STORAGE_SDCARD = 0,        /**< SD card */
    LOG_STORAGE_FLASH  = 1,        /**< Flash memory */
    LOG_STORAGE_STREAM = 2,        /**< USB stream */
} log_storage_t;

/**
 * @brief Trigger configuration
 */
typedef struct {
    uint16_t    trigger_mask;         /**< Active triggers */
    uint16_t    rpm_high;             /**< RPM high threshold */
    uint16_t    rpm_low;              /**< RPM low threshold */
    uint16_t    tps_delta;            /**< TPS change threshold */
    uint16_t    map_delta;            /**< MAP change threshold */
    uint16_t    pre_trigger_samples;  /**< Samples before trigger */
    uint16_t    post_trigger_samples; /**< Samples after trigger */
} log_trigger_config_t;

/**
 * @brief Logger configuration
 */
typedef struct {
    bool        enabled;             /**< Logger enabled */
    uint16_t    sample_rate_hz;      /**< Sample rate (1-1000 Hz) */
    uint8_t     format;              /**< Log format */
    uint8_t     storage_backend;     /**< Storage backend */
    uint32_t    buffer_size;         /**< Number of entries */
    log_trigger_config_t trigger;    /**< Trigger configuration */
    bool        auto_export;         /**< Auto export on session end */
    uint32_t    max_session_size;    /**< Max entries per session */
    char        prefix[LOG_PREFIX_LEN]; /**< File name prefix */
    bool        include_date;        /**< Include date in filename */
} log_config_t;

/**
 * @brief Log session header
 */
typedef struct {
    uint32_t    session_id;       /**< Unique session ID */
    uint32_t    start_time;       /**< Session start timestamp */
    uint32_t    end_time;         /**< Session end timestamp */
    uint32_t    entry_count;      /**< Number of entries */
    uint32_t    trigger_type;     /**< What triggered logging */
    char        name[LOG_SESSION_NAME_LEN]; /**< Session name */
    uint8_t     format;           /**< Log format */
    uint8_t     compression;      /**< Compression type */
    uint16_t    sample_rate_hz;   /**< Sample rate */
    uint32_t    crc32;            /**< CRC for integrity */
} log_session_header_t;

/**
 * @brief Logger statistics
 */
typedef struct {
    uint32_t    total_entries;    /**< Total entries logged */
    uint32_t    total_sessions;   /**< Total sessions */
    uint32_t    trigger_count;    /**< Trigger activations */
    uint32_t    buffer_overruns;  /**< Buffer overruns */
    uint32_t    write_errors;     /**< Write errors */
    uint32_t    bytes_written;    /**< Total bytes written */
} log_stats_t;

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize data logger
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t data_logger_init(void);

/**
 * @brief Deinitialize data logger
 * 
 * @return ESP_OK on success
 */
esp_err_t data_logger_deinit(void);

/**
 * @brief Start logging session
 * 
 * @param name Session name (can be NULL for auto-generated)
 * @return ESP_OK on success
 */
esp_err_t data_logger_start(const char *name);

/**
 * @brief Stop logging session
 * 
 * @param export Whether to export data
 * @return ESP_OK on success
 */
esp_err_t data_logger_stop(bool export);

/**
 * @brief Check if logging is active
 * 
 * @return true if logging
 */
bool data_logger_is_logging(void);

/**
 * @brief Capture a single log entry
 * 
 * Called automatically by logger task, or manually for custom logging.
 * 
 * @return ESP_OK on success
 */
esp_err_t data_logger_capture(void);

/**
 * @brief Trigger logging manually
 * 
 * @return ESP_OK on success
 */
esp_err_t data_logger_trigger(void);

/**
 * @brief Get logger configuration
 * 
 * @param config Configuration structure to fill
 */
void data_logger_get_config(log_config_t *config);

/**
 * @brief Set logger configuration
 * 
 * @param config New configuration
 * @return ESP_OK on success
 */
esp_err_t data_logger_set_config(const log_config_t *config);

/**
 * @brief Get logger statistics
 * 
 * @param stats Statistics structure to fill
 */
void data_logger_get_stats(log_stats_t *stats);

/**
 * @brief Export log data
 * 
 * @param format Export format
 * @param path Output path (can be NULL for auto-generated)
 * @return ESP_OK on success
 */
esp_err_t data_logger_export(log_format_t format, const char *path);

/**
 * @brief Clear log buffer
 * 
 * @return ESP_OK on success
 */
esp_err_t data_logger_clear(void);

/**
 * @brief Get number of entries in buffer
 * 
 * @return Number of entries
 */
uint32_t data_logger_get_entry_count(void);

/**
 * @brief Get entry from buffer
 * 
 * @param index Entry index (0 = oldest)
 * @param entry Entry structure to fill
 * @return ESP_OK on success
 */
esp_err_t data_logger_get_entry(uint32_t index, log_entry_t *entry);

/**
 * @brief Set trigger configuration
 * 
 * @param trigger Trigger configuration
 * @return ESP_OK on success
 */
esp_err_t data_logger_set_trigger(const log_trigger_config_t *trigger);

/**
 * @brief Get current session info
 * 
 * @param header Session header to fill
 * @return ESP_OK on success
 */
esp_err_t data_logger_get_session(log_session_header_t *header);

#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGER_H */
