/**
 * @file espnow_link.h
 * @brief ESP-NOW Communication Module for ESP32-S3 EFI
 * 
 * This module provides low-latency wireless communication for real-time
 * ECU supervision and tuning via ESP-NOW protocol.
 * 
 * Features:
 * - Engine status transmission at 10Hz
 * - Sensor data transmission at 10Hz
 * - Diagnostic message transmission at 1Hz
 * - Configuration update reception
 * - Peer management with encryption support
 * - Message acknowledgment and retry
 */

#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

#include "esp_err.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants and Configuration
 *============================================================================*/

/** @brief ESP-NOW message header size */
#define ESPNOW_MSG_HEADER_SIZE      8

/** @brief Maximum payload size (ESP-NOW max is 250 bytes) */
#define ESPNOW_MAX_PAYLOAD          232

/** @brief Maximum message size */
#define ESPNOW_MAX_MSG_SIZE         (ESPNOW_MSG_HEADER_SIZE + ESPNOW_MAX_PAYLOAD)

/** @brief Maximum number of peers */
#define ESPNOW_MAX_PEERS            4

/** @brief TX queue size */
#define ESPNOW_TX_QUEUE_SIZE        10

/** @brief Default engine status interval (ms) */
#define ESPNOW_ENGINE_STATUS_INTERVAL_MS    100

/** @brief Default sensor data interval (ms) */
#define ESPNOW_SENSOR_DATA_INTERVAL_MS      100

/** @brief Default diagnostic interval (ms) */
#define ESPNOW_DIAG_INTERVAL_MS             1000

/*============================================================================
 * Message Types
 *============================================================================*/

/**
 * @brief ESP-NOW message type identifiers
 */
typedef enum {
    ESPNOW_MSG_ENGINE_STATUS   = 0x01,  /**< ECU -> Peer: Engine status */
    ESPNOW_MSG_SENSOR_DATA     = 0x02,  /**< ECU -> Peer: Sensor data */
    ESPNOW_MSG_DIAGNOSTIC      = 0x03,  /**< ECU -> Peer: Diagnostic info */
    ESPNOW_MSG_CONFIG_REQUEST  = 0x10,  /**< Peer -> ECU: Request config */
    ESPNOW_MSG_CONFIG_RESPONSE = 0x11,  /**< ECU -> Peer: Config response */
    ESPNOW_MSG_TABLE_UPDATE    = 0x12,  /**< Peer -> ECU: Table update */
    ESPNOW_MSG_PARAM_SET       = 0x13,  /**< Peer -> ECU: Set parameter */
    ESPNOW_MSG_ACK             = 0xFF,  /**< Both: Acknowledgment */
} espnow_msg_type_t;

/*============================================================================
 * Message Flags
 *============================================================================*/

/** @brief Acknowledgment required flag */
#define ESPNOW_FLAG_ACK_REQUIRED    (1 << 0)

/** @brief High priority message flag */
#define ESPNOW_FLAG_HIGH_PRIORITY   (1 << 1)

/** @brief Encrypted message flag */
#define ESPNOW_FLAG_ENCRYPTED       (1 << 2)

/*============================================================================
 * Error Bitmap Flags
 *============================================================================*/

#define ESPNOW_ERR_OVER_REV       (1 << 0)   /**< Over-rev condition */
#define ESPNOW_ERR_OVERHEAT       (1 << 1)   /**< Engine overheat */
#define ESPNOW_ERR_UNDERVOLT      (1 << 2)   /**< Under-voltage */
#define ESPNOW_ERR_OVERVOLT       (1 << 3)   /**< Over-voltage */
#define ESPNOW_ERR_SENSOR_MAP     (1 << 4)   /**< MAP sensor fault */
#define ESPNOW_ERR_SENSOR_TPS     (1 << 5)   /**< TPS sensor fault */
#define ESPNOW_ERR_SENSOR_CLT     (1 << 6)   /**< CLT sensor fault */
#define ESPNOW_ERR_SENSOR_IAT     (1 << 7)   /**< IAT sensor fault */
#define ESPNOW_ERR_SENSOR_O2      (1 << 8)   /**< O2 sensor fault */
#define ESPNOW_ERR_SYNC_LOST      (1 << 9)   /**< Sync lost */
#define ESPNOW_ERR_LIMP_MODE      (1 << 10)  /**< Limp mode active */

/*============================================================================
 * Data Structures
 *============================================================================*/

/**
 * @brief ESP-NOW message header
 */
typedef struct __attribute__((packed)) {
    uint8_t     msg_type;       /**< Message type identifier */
    uint8_t     msg_version;    /**< Protocol version */
    uint16_t    msg_id;         /**< Sequence number */
    uint16_t    payload_len;    /**< Payload length */
    uint8_t     flags;          /**< Flags: ack_required, priority, etc. */
    uint8_t     checksum;       /**< XOR checksum */
} espnow_msg_header_t;

/**
 * @brief Engine status message payload
 * 
 * Contains real-time engine operating parameters for monitoring.
 */
typedef struct __attribute__((packed)) {
    uint16_t    rpm;              /**< RPM (0-8000) */
    uint16_t    map_kpa10;        /**< MAP * 10 (0-2500 = 0-250 kPa) */
    int16_t     clt_c10;          /**< CLT * 10 (-400 to 1200 = -40 to 120°C) */
    int16_t     iat_c10;          /**< IAT * 10 */
    uint16_t    tps_pct10;        /**< TPS * 10 (0-1000 = 0-100%) */
    uint16_t    battery_mv;       /**< Battery voltage in mV */
    uint8_t     sync_status;      /**< Sync state flags */
    uint8_t     limp_mode;        /**< Limp mode active */
    uint16_t    advance_deg10;    /**< Ignition advance * 10 */
    uint16_t    pw_us;            /**< Injection pulse width in us */
    uint16_t    lambda_target;    /**< Lambda target * 1000 */
    uint16_t    lambda_measured;  /**< Lambda measured * 1000 */
    uint32_t    timestamp_ms;     /**< Message timestamp */
} espnow_engine_status_t;

/**
 * @brief Sensor data message payload
 * 
 * Contains raw and filtered sensor readings for diagnostics.
 */
typedef struct __attribute__((packed)) {
    uint16_t    map_raw;          /**< MAP ADC raw */
    uint16_t    tps_raw;          /**< TPS ADC raw */
    uint16_t    clt_raw;          /**< CLT ADC raw */
    uint16_t    iat_raw;          /**< IAT ADC raw */
    uint16_t    o2_raw;           /**< O2 ADC raw */
    uint16_t    vbat_raw;         /**< Battery ADC raw */
    uint16_t    map_filtered;     /**< MAP filtered value */
    uint16_t    tps_filtered;     /**< TPS filtered value */
    uint8_t     sensor_faults;    /**< Sensor fault flags */
    uint8_t     reserved;         /**< Reserved for alignment */
    uint32_t    timestamp_ms;     /**< Message timestamp */
} espnow_sensor_data_t;

/**
 * @brief Diagnostic message payload
 * 
 * Contains system health and diagnostic information.
 */
typedef struct __attribute__((packed)) {
    uint8_t     error_count;      /**< Number of active errors */
    uint8_t     warning_count;    /**< Number of warnings */
    uint16_t    error_bitmap;     /**< Error flags */
    uint16_t    warning_bitmap;   /**< Warning flags */
    uint32_t    uptime_ms;        /**< System uptime */
    uint16_t    cpu_usage_pct;    /**< CPU usage percentage */
    uint16_t    free_heap;        /**< Free heap memory */
    uint32_t    sync_lost_count;  /**< Sync loss counter */
    uint32_t    tooth_count;      /**< Total tooth count */
} espnow_diagnostic_t;

/**
 * @brief Configuration request message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     config_type;      /**< What to configure */
    uint8_t     reserved;         /**< Reserved for alignment */
    uint16_t    config_id;        /**< Configuration parameter ID */
} espnow_config_request_t;

/**
 * @brief Configuration response message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     config_type;      /**< Configuration type */
    uint8_t     status;           /**< 0 = success */
    uint16_t    config_id;        /**< Configuration parameter ID */
    uint8_t     data[228];        /**< Configuration data */
} espnow_config_response_t;

/**
 * @brief Table update message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     table_id;         /**< Which table (VE, IGN, LAMBDA) */
    uint8_t     chunk_index;      /**< Chunk number for large tables */
    uint16_t    chunk_size;       /**< Size of this chunk */
    uint8_t     data[228];        /**< Table data chunk */
} espnow_table_update_t;

/**
 * @brief Parameter set message payload
 */
typedef struct __attribute__((packed)) {
    uint16_t    param_id;         /**< Parameter identifier */
    uint16_t    param_size;       /**< Parameter size */
    uint8_t     param_value[228]; /**< Parameter value */
} espnow_param_set_t;

/*============================================================================
 * Callback Types
 *============================================================================*/

/**
 * @brief Receive callback function type
 * 
 * @param msg_type Message type identifier
 * @param payload Payload data
 * @param payload_len Payload length
 * @param ctx User context pointer
 */
typedef void (*espnow_rx_callback_t)(uint8_t msg_type, const uint8_t *payload, 
                                      uint16_t payload_len, void *ctx);

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize ESP-NOW link module
 * 
 * Initializes WiFi in station mode, configures ESP-NOW, registers
 * callbacks, and creates the transmit queue.
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t espnow_link_init(void);

/**
 * @brief Deinitialize ESP-NOW link module
 * 
 * Stops communication, frees resources, and deinitializes ESP-NOW.
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t espnow_link_deinit(void);

/**
 * @brief Start ESP-NOW communication
 * 
 * Starts the periodic transmission task for engine status,
 * sensor data, and diagnostic messages.
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized or already started
 */
esp_err_t espnow_link_start(void);

/**
 * @brief Stop ESP-NOW communication
 * 
 * Stops the periodic transmission task.
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not started
 */
esp_err_t espnow_link_stop(void);

/**
 * @brief Add a peer device
 * 
 * Registers a peer device for unicast communication.
 * 
 * @param peer_mac Peer MAC address (6 bytes)
 * @param encrypt Enable encryption for this peer
 * @param lmk Local master key (16 bytes, required if encrypt=true)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_INVALID_ARG if peer_mac is NULL
 * @return ESP_ERR_NO_MEM if peer list is full
 */
esp_err_t espnow_link_add_peer(const uint8_t *peer_mac, bool encrypt, const uint8_t *lmk);

/**
 * @brief Remove a peer device
 * 
 * @param peer_mac Peer MAC address (6 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_NOT_FOUND if peer not found
 */
esp_err_t espnow_link_remove_peer(const uint8_t *peer_mac);

/**
 * @brief Register receive callback
 * 
 * Registers a callback function to be called when a message is received.
 * 
 * @param callback Callback function
 * @param ctx User context passed to callback
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_INVALID_ARG if callback is NULL
 */
esp_err_t espnow_link_register_rx_callback(espnow_rx_callback_t callback, void *ctx);

/**
 * @brief Send engine status message
 * 
 * Queues an engine status message for transmission to all peers.
 * 
 * @param status Engine status data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized or not started
 * @return ESP_ERR_INVALID_ARG if status is NULL
 * @return ESP_ERR_TIMEOUT if queue is full
 */
esp_err_t espnow_link_send_engine_status(const espnow_engine_status_t *status);

/**
 * @brief Send sensor data message
 * 
 * Queues a sensor data message for transmission to all peers.
 * 
 * @param data Sensor data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized or not started
 * @return ESP_ERR_INVALID_ARG if data is NULL
 * @return ESP_ERR_TIMEOUT if queue is full
 */
esp_err_t espnow_link_send_sensor_data(const espnow_sensor_data_t *data);

/**
 * @brief Send diagnostic message
 * 
 * Queues a diagnostic message for transmission to all peers.
 * 
 * @param diag Diagnostic data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized or not started
 * @return ESP_ERR_INVALID_ARG if diag is NULL
 * @return ESP_ERR_TIMEOUT if queue is full
 */
esp_err_t espnow_link_send_diagnostic(const espnow_diagnostic_t *diag);

/**
 * @brief Send configuration response
 * 
 * Sends a configuration response to a peer.
 * 
 * @param peer_mac Peer MAC address (NULL for broadcast)
 * @param response Configuration response data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_INVALID_ARG if response is NULL
 */
esp_err_t espnow_link_send_config_response(const uint8_t *peer_mac, 
                                            const espnow_config_response_t *response);

/**
 * @brief Get communication statistics
 * 
 * @param[out] tx_count Total messages transmitted
 * @param[out] rx_count Total messages received
 * @param[out] tx_errors Transmission errors
 * @param[out] rx_errors Reception errors
 */
void espnow_link_get_stats(uint32_t *tx_count, uint32_t *rx_count, 
                           uint32_t *tx_errors, uint32_t *rx_errors);

/**
 * @brief Check if ESP-NOW link is initialized
 * 
 * @return true if initialized
 * @return false if not initialized
 */
bool espnow_link_is_initialized(void);

/**
 * @brief Check if ESP-NOW link is started
 * 
 * @return true if started
 * @return false if not started
 */
bool espnow_link_is_started(void);

/**
 * @brief Get number of registered peers
 * 
 * @return Number of peers
 */
uint8_t espnow_link_get_peer_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ESPNOW_LINK_H */
