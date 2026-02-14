/**
 * @file tuning_protocol.h
 * @brief Remote Tuning Protocol Module for ESP32-S3 EFI
 * 
 * This protocol enables professional tuning software to communicate
 * with the ECU for real-time adjustments and data streaming.
 * 
 * Features:
 * - Real-time parameter adjustment
 * - Table upload/download with verification
 * - Live data streaming at configurable rates
 * - Session management with authentication
 */

#ifndef TUNING_PROTOCOL_H
#define TUNING_PROTOCOL_H

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

/** @brief Protocol version */
#define TUNING_PROTOCOL_VERSION    1

/** @brief Maximum payload size */
#define TUNING_MAX_PAYLOAD         240

/** @brief Maximum message size */
#define TUNING_MAX_MSG_SIZE        256

/** @brief Maximum client name length */
#define TUNING_CLIENT_NAME_LEN     32

/** @brief Maximum ECU name length */
#define TUNING_ECU_NAME_LEN        32

/** @brief Session ID length */
#define TUNING_SESSION_ID_LEN      8

/** @brief Auth response length */
#define TUNING_AUTH_RESPONSE_LEN   32

/** @brief Challenge length */
#define TUNING_CHALLENGE_LEN       16

/** @brief Maximum parameters */
#define TUNING_MAX_PARAMS          256

/** @brief Maximum tables */
#define TUNING_MAX_TABLES          16

/*============================================================================
 * Protocol Constants
 *============================================================================*/

/** @brief Message start byte */
#define TUNING_MSG_START           0xAA

/** @brief Message end byte */
#define TUNING_MSG_END             0x55

/*============================================================================
 * Message Types
 *============================================================================*/

typedef enum {
    // Handshake
    TUNING_MSG_HELLO          = 0x01,  /**< Client -> ECU */
    TUNING_MSG_HELLO_ACK      = 0x02,  /**< ECU -> Client */
    TUNING_MSG_AUTH           = 0x03,  /**< Client -> ECU */
    TUNING_MSG_AUTH_ACK       = 0x04,  /**< ECU -> Client */
    TUNING_MSG_BYE            = 0x05,  /**< Both */
    
    // Parameters
    TUNING_MSG_PARAM_GET      = 0x10,  /**< Client -> ECU */
    TUNING_MSG_PARAM_GET_ACK  = 0x11,  /**< ECU -> Client */
    TUNING_MSG_PARAM_SET      = 0x12,  /**< Client -> ECU */
    TUNING_MSG_PARAM_SET_ACK  = 0x13,  /**< ECU -> Client */
    TUNING_MSG_PARAM_LIST     = 0x14,  /**< Client -> ECU */
    TUNING_MSG_PARAM_LIST_ACK = 0x15,  /**< ECU -> Client */
    
    // Tables
    TUNING_MSG_TABLE_GET      = 0x20,  /**< Client -> ECU */
    TUNING_MSG_TABLE_GET_ACK  = 0x21,  /**< ECU -> Client */
    TUNING_MSG_TABLE_SET      = 0x22,  /**< Client -> ECU */
    TUNING_MSG_TABLE_SET_ACK  = 0x23,  /**< ECU -> Client */
    TUNING_MSG_TABLE_LIST     = 0x24,  /**< Client -> ECU */
    TUNING_MSG_TABLE_LIST_ACK = 0x25,  /**< ECU -> Client */
    
    // Streaming
    TUNING_MSG_STREAM_START   = 0x30,  /**< Client -> ECU */
    TUNING_MSG_STREAM_DATA    = 0x31,  /**< ECU -> Client */
    TUNING_MSG_STREAM_STOP    = 0x32,  /**< Client -> ECU */
    
    // Firmware
    TUNING_MSG_FW_INFO        = 0x40,  /**< Client -> ECU */
    TUNING_MSG_FW_INFO_ACK    = 0x41,  /**< ECU -> Client */
    TUNING_MSG_FW_DATA        = 0x42,  /**< Client -> ECU */
    TUNING_MSG_FW_DATA_ACK    = 0x43,  /**< ECU -> Client */
    TUNING_MSG_FW_APPLY       = 0x44,  /**< Client -> ECU */
    
    // Error
    TUNING_MSG_ERROR          = 0xFF,  /**< ECU -> Client */
} tuning_msg_type_t;

/*============================================================================
 * Message Flags
 *============================================================================*/

#define TUNING_FLAG_ACK_REQUIRED  (1 << 0)  /**< Requires acknowledgment */
#define TUNING_FLAG_COMPRESSED    (1 << 1)  /**< Payload is compressed */
#define TUNING_FLAG_ENCRYPTED     (1 << 2)  /**< Payload is encrypted */
#define TUNING_FLAG_FRAGMENT      (1 << 3)  /**< Fragmented message */
#define TUNING_FLAG_LAST_FRAGMENT (1 << 4)  /**< Last fragment */
#define TUNING_FLAG_PRIORITY      (1 << 5)  /**< High priority message */

/*============================================================================
 * Parameter Identifiers
 *============================================================================*/

typedef enum {
    PARAM_RPM_LIMIT          = 0x0001,
    PARAM_FUEL_CUTOFF        = 0x0002,
    PARAM_TEMP_LIMIT         = 0x0003,
    PARAM_BATTERY_MIN        = 0x0004,
    PARAM_BATTERY_MAX        = 0x0005,
    PARAM_EOI_BOUNDARY       = 0x0010,
    PARAM_EOI_NORMAL         = 0x0011,
    PARAM_STFT_LIMIT         = 0x0012,
    PARAM_LTFT_LIMIT         = 0x0013,
    PARAM_CLOSED_LOOP_EN     = 0x0014,
    PARAM_LAMBDA_PID_P       = 0x0020,
    PARAM_LAMBDA_PID_I       = 0x0021,
    PARAM_LAMBDA_PID_D       = 0x0022,
} tuning_param_id_t;

/*============================================================================
 * Error Codes
 *============================================================================*/

typedef enum {
    TUNING_ERR_NONE           = 0,
    TUNING_ERR_UNKNOWN_MSG    = 1,
    TUNING_ERR_INVALID_LEN    = 2,
    TUNING_ERR_CHECKSUM       = 3,
    TUNING_ERR_NOT_AUTH       = 4,
    TUNING_ERR_PARAM_NOT_FOUND = 5,
    TUNING_ERR_TABLE_NOT_FOUND = 6,
    TUNING_ERR_PERMISSION     = 7,
    TUNING_ERR_BUSY           = 8,
    TUNING_ERR_INTERNAL       = 9,
} tuning_error_t;

/*============================================================================
 * Data Structures
 *============================================================================*/

/**
 * @brief Message header structure
 */
typedef struct __attribute__((packed)) {
    uint8_t     start_byte;      /**< Start of message (0xAA) */
    uint8_t     msg_type;        /**< Message type */
    uint16_t    msg_id;          /**< Sequence number */
    uint16_t    payload_len;     /**< Payload length */
    uint8_t     flags;           /**< Flags */
    uint8_t     checksum;        /**< Header checksum */
} tuning_msg_header_t;

/**
 * @brief Complete message structure
 */
typedef struct {
    tuning_msg_header_t header;
    uint8_t     payload[TUNING_MAX_PAYLOAD];
    uint8_t     end_byte;
} tuning_msg_t;

/**
 * @brief HELLO message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     protocol_version;
    uint8_t     client_version;
    uint16_t    capabilities;
    char        client_name[TUNING_CLIENT_NAME_LEN];
} tuning_hello_t;

/**
 * @brief HELLO_ACK message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     protocol_version;
    uint8_t     ecu_version;
    uint16_t    capabilities;
    uint8_t     auth_required;
    char        ecu_name[TUNING_ECU_NAME_LEN];
    uint8_t     challenge[TUNING_CHALLENGE_LEN];
} tuning_hello_ack_t;

/**
 * @brief AUTH message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     auth_type;
    uint8_t     response[TUNING_AUTH_RESPONSE_LEN];
} tuning_auth_t;

/**
 * @brief AUTH_ACK message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     status;
    uint8_t     session_id[TUNING_SESSION_ID_LEN];
    uint16_t    permissions;
} tuning_auth_ack_t;

/**
 * @brief Parameter get/set message payload
 */
typedef struct __attribute__((packed)) {
    uint16_t    param_id;
    uint16_t    param_size;
    uint8_t     param_value[];
} tuning_param_msg_t;

/**
 * @brief Table identifier
 */
typedef enum {
    TABLE_VE          = 0x01,
    TABLE_IGNITION    = 0x02,
    TABLE_LAMBDA      = 0x03,
    TABLE_EOIT_NORMAL = 0x04,
} tuning_table_id_t;

/**
 * @brief Table get/set message payload
 */
typedef struct __attribute__((packed)) {
    uint8_t     table_id;
    uint8_t     chunk_index;
    uint16_t    chunk_size;
    uint8_t     data[];
} tuning_table_msg_t;

/**
 * @brief Stream configuration
 */
typedef struct {
    uint16_t    interval_ms;      /**< Stream interval in ms */
    uint16_t    data_mask;        /**< What data to include */
    uint8_t     format;           /**< Output format */
} tuning_stream_config_t;

/**
 * @brief Tuning session state
 */
typedef struct {
    bool        active;
    bool        authenticated;
    uint8_t     session_id[TUNING_SESSION_ID_LEN];
    uint16_t    permissions;
    uint32_t    last_activity_ms;
} tuning_session_t;

/**
 * @brief Tuning statistics
 */
typedef struct {
    uint32_t    msg_received;
    uint32_t    msg_sent;
    uint32_t    msg_errors;
    uint32_t    param_reads;
    uint32_t    param_writes;
    uint32_t    table_reads;
    uint32_t    table_writes;
} tuning_stats_t;

/*============================================================================
 * Callback Types
 *============================================================================*/

/**
 * @brief Message send callback
 * @param data Message data
 * @param len Data length
 * @return ESP_OK on success
 */
typedef esp_err_t (*tuning_send_cb_t)(const uint8_t *data, size_t len);

/**
 * @brief Parameter read callback
 * @param param_id Parameter ID
 * @param value Output value buffer
 * @param size Output size
 * @return ESP_OK on success
 */
typedef esp_err_t (*tuning_param_read_cb_t)(uint16_t param_id, void *value, size_t *size);

/**
 * @brief Parameter write callback
 * @param param_id Parameter ID
 * @param value Input value buffer
 * @param size Input size
 * @return ESP_OK on success
 */
typedef esp_err_t (*tuning_param_write_cb_t)(uint16_t param_id, const void *value, size_t size);

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize tuning protocol
 * 
 * @return ESP_OK on success
 */
esp_err_t tuning_protocol_init(void);

/**
 * @brief Deinitialize tuning protocol
 * 
 * @return ESP_OK on success
 */
esp_err_t tuning_protocol_deinit(void);

/**
 * @brief Start tuning protocol
 * 
 * @return ESP_OK on success
 */
esp_err_t tuning_protocol_start(void);

/**
 * @brief Stop tuning protocol
 * 
 * @return ESP_OK on success
 */
esp_err_t tuning_protocol_stop(void);

/**
 * @brief Register send callback
 * 
 * @param callback Send callback function
 * @return ESP_OK on success
 */
esp_err_t tuning_register_send_callback(tuning_send_cb_t callback);

/**
 * @brief Register parameter callbacks
 * 
 * @param read_cb Read callback
 * @param write_cb Write callback
 * @return ESP_OK on success
 */
esp_err_t tuning_register_param_callbacks(tuning_param_read_cb_t read_cb,
                                          tuning_param_write_cb_t write_cb);

/**
 * @brief Process received message
 * 
 * @param data Received data
 * @param len Data length
 * @return ESP_OK on success
 */
esp_err_t tuning_process_message(const uint8_t *data, size_t len);

/**
 * @brief Send message
 * 
 * @param msg_type Message type
 * @param payload Payload data (can be NULL)
 * @param payload_len Payload length
 * @param flags Message flags
 * @return ESP_OK on success
 */
esp_err_t tuning_send_message(uint8_t msg_type, const uint8_t *payload,
                              uint16_t payload_len, uint8_t flags);

/**
 * @brief Send error response
 * 
 * @param error Error code
 * @param msg_id Original message ID
 * @return ESP_OK on success
 */
esp_err_t tuning_send_error(tuning_error_t error, uint16_t msg_id);

/**
 * @brief Check if session is active
 * 
 * @return true if session is active
 */
bool tuning_is_session_active(void);

/**
 * @brief Check if authenticated
 * 
 * @return true if authenticated
 */
bool tuning_is_authenticated(void);

/**
 * @brief Get session info
 * 
 * @param session Session structure to fill
 * @return ESP_OK on success
 */
esp_err_t tuning_get_session(tuning_session_t *session);

/**
 * @brief Get statistics
 * 
 * @param stats Statistics structure to fill
 */
void tuning_get_stats(tuning_stats_t *stats);

/**
 * @brief Close current session
 * 
 * @return ESP_OK on success
 */
esp_err_t tuning_close_session(void);

#ifdef __cplusplus
}
#endif

#endif /* TUNING_PROTOCOL_H */
