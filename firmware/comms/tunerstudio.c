/**
 * @file tuning_protocol.c
 * @brief Remote Tuning Protocol Module Implementation
 * 
 * This protocol enables professional tuning software to communicate
 * with the ECU for real-time adjustments and data streaming.
 */

#include "comms/tunerstudio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

/*============================================================================
 * Constants
 *============================================================================*/

static const char *TAG = "tuning";

/** @brief Session timeout (ms) */
#define TUNING_SESSION_TIMEOUT_MS   60000

/** @brief ECU name */
#define TUNING_ECU_NAME             "ESP32-S3 EFI"

/** @brief ECU version */
#define TUNING_ECU_VERSION          0x01

/*============================================================================
 * Module State
 *============================================================================*/

typedef struct {
    // State
    bool                initialized;
    bool                started;
    
    // Session
    tuning_session_t    session;
    
    // Message tracking
    uint16_t            tx_msg_id;
    uint32_t            last_activity_ms;
    
    // Statistics
    tuning_stats_t      stats;
    
    // Callbacks
    tuning_send_cb_t    send_callback;
    tuning_param_read_cb_t  param_read_cb;
    tuning_param_write_cb_t param_write_cb;
    
    // Mutex
    SemaphoreHandle_t   mutex;
    
    // Streaming
    bool                streaming;
    uint16_t            stream_interval_ms;
    uint16_t            stream_data_mask;
    TaskHandle_t        stream_task;
} tuning_protocol_t;

static tuning_protocol_t g_tuning = {
    .initialized = false,
    .started = false,
    .send_callback = NULL,
    .param_read_cb = NULL,
    .param_write_cb = NULL,
    .mutex = NULL,
    .streaming = false,
    .stream_task = NULL,
};

/*============================================================================
 * Helper Functions
 *============================================================================*/

static uint8_t calc_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static void generate_session_id(uint8_t *session_id)
{
    for (int i = 0; i < TUNING_SESSION_ID_LEN; i++) {
        session_id[i] = (uint8_t)(esp_random() & 0xFF);
    }
}

static void generate_challenge(uint8_t *challenge)
{
    for (int i = 0; i < TUNING_CHALLENGE_LEN; i++) {
        challenge[i] = (uint8_t)(esp_random() & 0xFF);
    }
}

/*============================================================================
 * Message Building
 *============================================================================*/

static esp_err_t build_and_send(uint8_t msg_type, const uint8_t *payload,
                                 uint16_t payload_len, uint8_t flags)
{
    if (g_tuning.send_callback == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (payload_len > TUNING_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    uint8_t buffer[TUNING_MAX_MSG_SIZE];
    tuning_msg_header_t *header = (tuning_msg_header_t *)buffer;
    
    // Build header
    header->start_byte = TUNING_MSG_START;
    header->msg_type = msg_type;
    header->msg_id = g_tuning.tx_msg_id++;
    header->payload_len = payload_len;
    header->flags = flags;
    header->checksum = 0;
    
    // Copy payload
    if (payload_len > 0 && payload != NULL) {
        memcpy(buffer + sizeof(tuning_msg_header_t), payload, payload_len);
    }
    
    // Calculate header checksum
    header->checksum = calc_checksum(buffer, sizeof(tuning_msg_header_t));
    
    // Add end byte
    buffer[sizeof(tuning_msg_header_t) + payload_len] = TUNING_MSG_END;
    
    size_t total_len = sizeof(tuning_msg_header_t) + payload_len + 1;
    
    // Send
    esp_err_t ret = g_tuning.send_callback(buffer, total_len);
    if (ret == ESP_OK) {
        g_tuning.stats.msg_sent++;
    }
    
    return ret;
}

/*============================================================================
 * Message Handlers
 *============================================================================*/

static esp_err_t handle_hello(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(tuning_hello_t)) {
        return tuning_send_error(TUNING_ERR_INVALID_LEN, g_tuning.tx_msg_id);
    }
    
    tuning_hello_t *hello = (tuning_hello_t *)payload;
    
    // Check protocol version
    if (hello->protocol_version > TUNING_PROTOCOL_VERSION) {
        return tuning_send_error(TUNING_ERR_INTERNAL, g_tuning.tx_msg_id);
    }
    
    // Create session
    memset(&g_tuning.session, 0, sizeof(tuning_session_t));
    generate_session_id(g_tuning.session.session_id);
    generate_challenge(g_tuning.session.challenge);
    g_tuning.session.active = true;
    g_tuning.session.authenticated = false;
    g_tuning.session.last_activity_ms = (uint32_t)(esp_timer_get_time() / 1000);
    
    // Build response
    tuning_hello_ack_t ack = {0};
    ack.protocol_version = TUNING_PROTOCOL_VERSION;
    ack.ecu_version = TUNING_ECU_VERSION;
    ack.capabilities = 0x0001;  // Basic capabilities
    ack.auth_required = 0;      // No auth required for now
    strncpy(ack.ecu_name, TUNING_ECU_NAME, TUNING_ECU_NAME_LEN - 1);
    memcpy(ack.challenge, g_tuning.session.challenge, TUNING_CHALLENGE_LEN);
    
    ESP_LOGI(TAG, "HELLO from client: %s", hello->client_name);
    
    return build_and_send(TUNING_MSG_HELLO_ACK, (uint8_t *)&ack, sizeof(ack), 0);
}

static esp_err_t handle_auth(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(tuning_auth_t)) {
        return tuning_send_error(TUNING_ERR_INVALID_LEN, g_tuning.tx_msg_id);
    }
    
    if (!g_tuning.session.active) {
        return tuning_send_error(TUNING_ERR_NOT_AUTH, g_tuning.tx_msg_id);
    }
    
    // For now, accept any auth
    g_tuning.session.authenticated = true;
    g_tuning.session.permissions = 0xFFFF;  // Full permissions
    
    tuning_auth_ack_t ack = {0};
    ack.status = 0;  // Success
    memcpy(ack.session_id, g_tuning.session.session_id, TUNING_SESSION_ID_LEN);
    ack.permissions = g_tuning.session.permissions;
    
    ESP_LOGI(TAG, "Client authenticated");
    
    return build_and_send(TUNING_MSG_AUTH_ACK, (uint8_t *)&ack, sizeof(ack), 0);
}

static esp_err_t handle_param_get(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(uint16_t)) {
        return tuning_send_error(TUNING_ERR_INVALID_LEN, g_tuning.tx_msg_id);
    }
    
    if (!g_tuning.session.active || !g_tuning.session.authenticated) {
        return tuning_send_error(TUNING_ERR_NOT_AUTH, g_tuning.tx_msg_id);
    }
    
    uint16_t param_id = *(uint16_t *)payload;
    
    uint8_t response[64];
    tuning_param_msg_t *resp = (tuning_param_msg_t *)response;
    resp->param_id = param_id;
    resp->param_size = sizeof(response) - sizeof(tuning_param_msg_t);
    
    esp_err_t ret = ESP_OK;
    if (g_tuning.param_read_cb != NULL) {
        ret = g_tuning.param_read_cb(param_id, resp->param_value, (size_t *)&resp->param_size);
    } else {
        // Default handler - return error
        resp->param_size = 0;
        ret = ESP_ERR_NOT_FOUND;
    }
    
    if (ret != ESP_OK) {
        return tuning_send_error(TUNING_ERR_PARAM_NOT_FOUND, g_tuning.tx_msg_id);
    }
    
    g_tuning.stats.param_reads++;
    
    size_t resp_len = sizeof(tuning_param_msg_t) + resp->param_size;
    return build_and_send(TUNING_MSG_PARAM_GET_ACK, response, resp_len, 0);
}

static esp_err_t handle_param_set(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(tuning_param_msg_t)) {
        return tuning_send_error(TUNING_ERR_INVALID_LEN, g_tuning.tx_msg_id);
    }
    
    if (!g_tuning.session.active || !g_tuning.session.authenticated) {
        return tuning_send_error(TUNING_ERR_NOT_AUTH, g_tuning.tx_msg_id);
    }
    
    tuning_param_msg_t *msg = (tuning_param_msg_t *)payload;
    
    esp_err_t ret = ESP_OK;
    if (g_tuning.param_write_cb != NULL) {
        ret = g_tuning.param_write_cb(msg->param_id, msg->param_value, msg->param_size);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }
    
    // Build response
    uint8_t status = (ret == ESP_OK) ? 0 : 1;
    
    g_tuning.stats.param_writes++;
    
    ESP_LOGI(TAG, "PARAM_SET: id=0x%04X, size=%u, status=%d", 
             msg->param_id, msg->param_size, status);
    
    return build_and_send(TUNING_MSG_PARAM_SET_ACK, &status, 1, 0);
}

static esp_err_t handle_bye(void)
{
    ESP_LOGI(TAG, "Session closed by client");
    tuning_close_session();
    return build_and_send(TUNING_MSG_BYE, NULL, 0, 0);
}

/*============================================================================
 * Streaming Task
 *============================================================================*/

static void stream_task(void *arg)
{
    (void)arg;
    
    while (g_tuning.streaming) {
        // Build stream data message
        // For now, just send basic engine data
        uint8_t payload[64];
        uint16_t len = 0;
        
        // TODO: Get actual engine data and format
        
        build_and_send(TUNING_MSG_STREAM_DATA, payload, len, 0);
        
        vTaskDelay(pdMS_TO_TICKS(g_tuning.stream_interval_ms));
    }
    
    vTaskDelete(NULL);
}

/*============================================================================
 * Public API Implementation
 *============================================================================*/

esp_err_t tuning_protocol_init(void)
{
    if (g_tuning.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_tuning.mutex = xSemaphoreCreateMutex();
    if (g_tuning.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    memset(&g_tuning.session, 0, sizeof(tuning_session_t));
    memset(&g_tuning.stats, 0, sizeof(tuning_stats_t));
    g_tuning.tx_msg_id = 0;
    g_tuning.send_callback = NULL;
    g_tuning.param_read_cb = NULL;
    g_tuning.param_write_cb = NULL;
    g_tuning.streaming = false;
    g_tuning.started = false;
    
    g_tuning.initialized = true;
    
    ESP_LOGI(TAG, "Tuning protocol initialized");
    return ESP_OK;
}

esp_err_t tuning_protocol_deinit(void)
{
    if (!g_tuning.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_tuning.streaming) {
        g_tuning.streaming = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    g_tuning.initialized = false;
    
    if (g_tuning.mutex != NULL) {
        vSemaphoreDelete(g_tuning.mutex);
        g_tuning.mutex = NULL;
    }
    
    ESP_LOGI(TAG, "Tuning protocol deinitialized");
    return ESP_OK;
}

esp_err_t tuning_protocol_start(void)
{
    if (!g_tuning.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_tuning.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_tuning.started = true;
    
    ESP_LOGI(TAG, "Tuning protocol started");
    return ESP_OK;
}

esp_err_t tuning_protocol_stop(void)
{
    if (!g_tuning.initialized || !g_tuning.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_tuning.started = false;
    tuning_close_session();
    
    ESP_LOGI(TAG, "Tuning protocol stopped");
    return ESP_OK;
}

esp_err_t tuning_register_send_callback(tuning_send_cb_t callback)
{
    if (!g_tuning.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_tuning.send_callback = callback;
    return ESP_OK;
}

esp_err_t tuning_register_param_callbacks(tuning_param_read_cb_t read_cb,
                                          tuning_param_write_cb_t write_cb)
{
    if (!g_tuning.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_tuning.param_read_cb = read_cb;
    g_tuning.param_write_cb = write_cb;
    return ESP_OK;
}

esp_err_t tuning_process_message(const uint8_t *data, size_t len)
{
    if (!g_tuning.initialized || !g_tuning.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (data == NULL || len < sizeof(tuning_msg_header_t) + 1) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_tuning.stats.msg_received++;
    
    // Parse header
    const tuning_msg_header_t *header = (const tuning_msg_header_t *)data;
    
    // Verify start byte
    if (header->start_byte != TUNING_MSG_START) {
        g_tuning.stats.msg_errors++;
        return ESP_ERR_INVALID_ARG;
    }
    
    // Verify checksum
    uint8_t calc = calc_checksum(data, sizeof(tuning_msg_header_t));
    if (calc != header->checksum) {
        g_tuning.stats.msg_errors++;
        return tuning_send_error(TUNING_ERR_CHECKSUM, header->msg_id);
    }
    
    // Verify end byte
    if (data[sizeof(tuning_msg_header_t) + header->payload_len] != TUNING_MSG_END) {
        g_tuning.stats.msg_errors++;
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update activity
    g_tuning.last_activity_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (g_tuning.session.active) {
        g_tuning.session.last_activity_ms = g_tuning.last_activity_ms;
    }
    
    // Get payload
    const uint8_t *payload = data + sizeof(tuning_msg_header_t);
    
    // Handle message
    switch (header->msg_type) {
        case TUNING_MSG_HELLO:
            return handle_hello(payload, header->payload_len);
            
        case TUNING_MSG_AUTH:
            return handle_auth(payload, header->payload_len);
            
        case TUNING_MSG_PARAM_GET:
            return handle_param_get(payload, header->payload_len);
            
        case TUNING_MSG_PARAM_SET:
            return handle_param_set(payload, header->payload_len);
            
        case TUNING_MSG_BYE:
            return handle_bye();
            
        case TUNING_MSG_STREAM_START:
            if (!g_tuning.session.authenticated) {
                return tuning_send_error(TUNING_ERR_NOT_AUTH, header->msg_id);
            }
            g_tuning.streaming = true;
            if (header->payload_len >= 2) {
                g_tuning.stream_interval_ms = *(uint16_t *)payload;
            } else {
                g_tuning.stream_interval_ms = 100;
            }
            ESP_LOGI(TAG, "Streaming started: %u ms interval", g_tuning.stream_interval_ms);
            return ESP_OK;
            
        case TUNING_MSG_STREAM_STOP:
            g_tuning.streaming = false;
            ESP_LOGI(TAG, "Streaming stopped");
            return ESP_OK;
            
        default:
            ESP_LOGW(TAG, "Unknown message type: 0x%02X", header->msg_type);
            return tuning_send_error(TUNING_ERR_UNKNOWN_MSG, header->msg_id);
    }
}

esp_err_t tuning_send_message(uint8_t msg_type, const uint8_t *payload,
                              uint16_t payload_len, uint8_t flags)
{
    return build_and_send(msg_type, payload, payload_len, flags);
}

esp_err_t tuning_send_error(tuning_error_t error, uint16_t msg_id)
{
    uint8_t payload[2] = { (uint8_t)error, (uint8_t)(msg_id & 0xFF) };
    g_tuning.stats.msg_errors++;
    return build_and_send(TUNING_MSG_ERROR, payload, sizeof(payload), 0);
}

bool tuning_is_session_active(void)
{
    return g_tuning.session.active;
}

bool tuning_is_authenticated(void)
{
    return g_tuning.session.authenticated;
}

esp_err_t tuning_get_session(tuning_session_t *session)
{
    if (session == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *session = g_tuning.session;
    return ESP_OK;
}

void tuning_get_stats(tuning_stats_t *stats)
{
    if (stats != NULL) {
        *stats = g_tuning.stats;
    }
}

esp_err_t tuning_close_session(void)
{
    g_tuning.streaming = false;
    memset(&g_tuning.session, 0, sizeof(tuning_session_t));
    return ESP_OK;
}
