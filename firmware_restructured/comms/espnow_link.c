/**
 * @file espnow_link.c
 * @brief ESP-NOW Communication Module Implementation
 * 
 * This module provides low-latency wireless communication for real-time
 * ECU supervision and tuning via ESP-NOW protocol.
 */

#include "comms/espnow_link.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include <string.h>

/*============================================================================
 * Constants
 *============================================================================*/

static const char *TAG = "espnow_link";

/** @brief Protocol version */
#define ESPNOW_PROTOCOL_VERSION     1

/** @brief TX task stack size */
#define ESPNOW_TX_TASK_STACK_SIZE   4096

/** @brief TX task priority */
#define ESPNOW_TX_TASK_PRIORITY     5

/** @brief Maximum retry count for failed transmissions */
#define ESPNOW_MAX_RETRY            3

/** @brief Retry delay in milliseconds */
#define ESPNOW_RETRY_DELAY_MS       10

/*============================================================================
 * Internal Types
 *============================================================================*/

/**
 * @brief TX queue item
 */
typedef struct {
    uint8_t     data[ESPNOW_MAX_MSG_SIZE];
    size_t      len;
    uint8_t     dest_mac[6];
    uint8_t     retry_count;
} espnow_tx_item_t;

/**
 * @brief Module state
 */
typedef struct {
    bool                    initialized;
    bool                    started;
    uint8_t                 peer_count;
    
    // Message tracking
    uint16_t                tx_msg_id;
    uint32_t                tx_count;
    uint32_t                rx_count;
    uint32_t                tx_errors;
    uint32_t                rx_errors;
    
    // Transmit buffer
    QueueHandle_t           tx_queue;
    TaskHandle_t            tx_task;
    
    // Peer management
    esp_now_peer_info_t     peers[ESPNOW_MAX_PEERS];
    uint8_t                 broadcast_mac[6];
    
    // Callbacks
    espnow_rx_callback_t    rx_callback;
    void                    *rx_callback_ctx;
    
    // Timing
    uint32_t                last_engine_status_ms;
    uint32_t                last_sensor_data_ms;
    uint32_t                last_diagnostic_ms;
    
    // Mutex for thread safety
    SemaphoreHandle_t       mutex;
} espnow_link_t;

/*============================================================================
 * Module Instance
 *============================================================================*/

static espnow_link_t g_espnow = {
    .initialized = false,
    .started = false,
    .peer_count = 0,
    .tx_msg_id = 0,
    .tx_count = 0,
    .rx_count = 0,
    .tx_errors = 0,
    .rx_errors = 0,
    .tx_queue = NULL,
    .tx_task = NULL,
    .rx_callback = NULL,
    .rx_callback_ctx = NULL,
    .mutex = NULL,
};

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Calculate XOR checksum
 * 
 * @param data Data buffer
 * @param len Buffer length
 * @return Checksum byte
 */
static uint8_t espnow_calc_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Build a complete message with header
 * 
 * @param msg_type Message type
 * @param payload Payload data
 * @param payload_len Payload length
 * @param flags Message flags
 * @param[out] out_buf Output buffer (must be ESPNOW_MAX_MSG_SIZE)
 * @param[out] out_len Output length
 * @return ESP_OK on success
 */
static esp_err_t espnow_build_message(uint8_t msg_type, const uint8_t *payload,
                                       uint16_t payload_len, uint8_t flags,
                                       uint8_t *out_buf, size_t *out_len)
{
    if (payload_len > ESPNOW_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Build header
    espnow_msg_header_t *header = (espnow_msg_header_t *)out_buf;
    header->msg_type = msg_type;
    header->msg_version = ESPNOW_PROTOCOL_VERSION;
    header->msg_id = g_espnow.tx_msg_id++;
    header->payload_len = payload_len;
    header->flags = flags;
    
    // Copy payload
    if (payload_len > 0 && payload != NULL) {
        memcpy(out_buf + ESPNOW_MSG_HEADER_SIZE, payload, payload_len);
    }
    
    // Calculate checksum over header (except checksum field) and payload
    header->checksum = 0;
    header->checksum = espnow_calc_checksum(out_buf, ESPNOW_MSG_HEADER_SIZE + payload_len);
    
    *out_len = ESPNOW_MSG_HEADER_SIZE + payload_len;
    
    return ESP_OK;
}

/**
 * @brief Validate received message
 * 
 * @param data Received data
 * @param len Data length
 * @param[out] header Parsed header
 * @param[out] payload Payload pointer
 * @param[out] payload_len Payload length
 * @return ESP_OK if valid
 */
static esp_err_t espnow_validate_message(const uint8_t *data, size_t len,
                                          espnow_msg_header_t **header,
                                          const uint8_t **payload,
                                          uint16_t *payload_len)
{
    if (len < ESPNOW_MSG_HEADER_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    espnow_msg_header_t *hdr = (espnow_msg_header_t *)data;
    
    // Check version
    if (hdr->msg_version != ESPNOW_PROTOCOL_VERSION) {
        return ESP_ERR_INVALID_VERSION;
    }
    
    // Check payload length
    if (hdr->payload_len > ESPNOW_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Verify total length
    if (len < ESPNOW_MSG_HEADER_SIZE + hdr->payload_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Verify checksum
    uint8_t stored_checksum = hdr->checksum;
    hdr->checksum = 0;
    uint8_t calc_checksum = espnow_calc_checksum(data, ESPNOW_MSG_HEADER_SIZE + hdr->payload_len);
    hdr->checksum = stored_checksum;
    
    if (calc_checksum != stored_checksum) {
        return ESP_ERR_INVALID_CRC;
    }
    
    *header = hdr;
    *payload = data + ESPNOW_MSG_HEADER_SIZE;
    *payload_len = hdr->payload_len;
    
    return ESP_OK;
}

/**
 * @brief ESP-NOW send callback
 * 
 * @param mac_addr Destination MAC address
 * @param status Transmission status
 */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        g_espnow.tx_count++;
    } else {
        g_espnow.tx_errors++;
        ESP_LOGW(TAG, "Send failed to " MACSTR, MAC2STR(mac_addr));
    }
}

/**
 * @brief ESP-NOW receive callback
 * 
 * @param recv_info Receive info
 * @param data Received data
 * @param len Data length
 */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, 
                           const uint8_t *data, int len)
{
    espnow_msg_header_t *header;
    const uint8_t *payload;
    uint16_t payload_len;
    
    g_espnow.rx_count++;
    
    // Validate message
    esp_err_t ret = espnow_validate_message(data, len, &header, &payload, &payload_len);
    if (ret != ESP_OK) {
        g_espnow.rx_errors++;
        ESP_LOGW(TAG, "Invalid message: %s", esp_err_to_name(ret));
        return;
    }
    
    // Handle ACK messages internally
    if (header->msg_type == ESPNOW_MSG_ACK) {
        ESP_LOGD(TAG, "Received ACK for msg_id %d", header->msg_id);
        return;
    }
    
    // Call user callback if registered
    if (g_espnow.rx_callback != NULL) {
        g_espnow.rx_callback(header->msg_type, payload, payload_len, 
                             g_espnow.rx_callback_ctx);
    }
    
    // Send ACK if required
    if (header->flags & ESPNOW_FLAG_ACK_REQUIRED) {
        uint8_t ack_buf[ESPNOW_MSG_HEADER_SIZE];
        size_t ack_len;
        espnow_build_message(ESPNOW_MSG_ACK, NULL, 0, 0, ack_buf, &ack_len);
        esp_now_send(recv_info->src_addr, ack_buf, ack_len);
    }
}

/**
 * @brief TX task - processes transmit queue
 * 
 * @param arg Task argument (unused)
 */
static void espnow_tx_task(void *arg)
{
    espnow_tx_item_t item;
    
    while (g_espnow.started) {
        if (xQueueReceive(g_espnow.tx_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Try to send
            esp_err_t ret = esp_now_send(item.dest_mac, item.data, item.len);
            
            if (ret != ESP_OK) {
                // Retry logic
                if (item.retry_count < ESPNOW_MAX_RETRY) {
                    item.retry_count++;
                    ESP_LOGW(TAG, "Send failed, retry %d/%d", 
                             item.retry_count, ESPNOW_MAX_RETRY);
                    vTaskDelay(pdMS_TO_TICKS(ESPNOW_RETRY_DELAY_MS));
                    xQueueSendToFront(g_espnow.tx_queue, &item, 0);
                } else {
                    g_espnow.tx_errors++;
                    ESP_LOGE(TAG, "Send failed after %d retries", ESPNOW_MAX_RETRY);
                }
            }
        }
    }
    
    vTaskDelete(NULL);
}

/**
 * @brief Find peer index by MAC address
 * 
 * @param peer_mac MAC address to find
 * @return Peer index or -1 if not found
 */
static int espnow_find_peer(const uint8_t *peer_mac)
{
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
        if (memcmp(g_espnow.peers[i].peer_addr, peer_mac, 6) == 0) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find free peer slot
 * 
 * @return Free slot index or -1 if none available
 */
static int espnow_find_free_slot(void)
{
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
        bool is_empty = true;
        for (int j = 0; j < 6; j++) {
            if (g_espnow.peers[i].peer_addr[j] != 0) {
                is_empty = false;
                break;
            }
        }
        if (is_empty) {
            return i;
        }
    }
    return -1;
}

/*============================================================================
 * Public API Implementation
 *============================================================================*/

esp_err_t espnow_link_init(void)
{
    if (g_espnow.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    // Initialize NVS (required for WiFi)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize WiFi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    
    // Register callbacks
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    
    // Create mutex
    g_espnow.mutex = xSemaphoreCreateMutex();
    if (g_espnow.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        esp_now_deinit();
        return ESP_ERR_NO_MEM;
    }
    
    // Create TX queue
    g_espnow.tx_queue = xQueueCreate(ESPNOW_TX_QUEUE_SIZE, sizeof(espnow_tx_item_t));
    if (g_espnow.tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        vSemaphoreDelete(g_espnow.mutex);
        esp_now_deinit();
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize broadcast MAC
    memset(g_espnow.broadcast_mac, 0xFF, 6);
    
    // Add broadcast peer
    memset(&g_espnow.peers[0], 0, sizeof(esp_now_peer_info_t));
    memcpy(g_espnow.peers[0].peer_addr, g_espnow.broadcast_mac, 6);
    g_espnow.peers[0].channel = 0;  // Use current channel
    g_espnow.peers[0].ifidx = WIFI_IF_STA;
    g_espnow.peers[0].encrypt = false;
    
    ret = esp_now_add_peer(&g_espnow.peers[0]);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(ret));
    }
    
    // Reset state
    g_espnow.peer_count = 1;  // Broadcast peer
    g_espnow.tx_msg_id = 0;
    g_espnow.tx_count = 0;
    g_espnow.rx_count = 0;
    g_espnow.tx_errors = 0;
    g_espnow.rx_errors = 0;
    g_espnow.started = false;
    g_espnow.initialized = true;
    
    ESP_LOGI(TAG, "ESP-NOW link initialized");
    
    return ESP_OK;
}

esp_err_t espnow_link_deinit(void)
{
    if (!g_espnow.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop if running
    if (g_espnow.started) {
        espnow_link_stop();
    }
    
    // Delete mutex
    if (g_espnow.mutex != NULL) {
        vSemaphoreDelete(g_espnow.mutex);
        g_espnow.mutex = NULL;
    }
    
    // Delete queue
    if (g_espnow.tx_queue != NULL) {
        vQueueDelete(g_espnow.tx_queue);
        g_espnow.tx_queue = NULL;
    }
    
    // Remove all peers
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
        if (g_espnow.peers[i].peer_addr[0] != 0 || 
            g_espnow.peers[i].peer_addr[1] != 0) {
            esp_now_del_peer(g_espnow.peers[i].peer_addr);
        }
    }
    
    // Deinitialize ESP-NOW
    esp_now_deinit();
    
    g_espnow.initialized = false;
    g_espnow.peer_count = 0;
    
    ESP_LOGI(TAG, "ESP-NOW link deinitialized");
    
    return ESP_OK;
}

esp_err_t espnow_link_start(void)
{
    if (!g_espnow.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_espnow.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create TX task
    BaseType_t ret = xTaskCreate(
        espnow_tx_task,
        "espnow_tx",
        ESPNOW_TX_TASK_STACK_SIZE,
        NULL,
        ESPNOW_TX_TASK_PRIORITY,
        &g_espnow.tx_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task");
        return ESP_ERR_NO_MEM;
    }
    
    g_espnow.started = true;
    g_espnow.last_engine_status_ms = 0;
    g_espnow.last_sensor_data_ms = 0;
    g_espnow.last_diagnostic_ms = 0;
    
    ESP_LOGI(TAG, "ESP-NOW link started");
    
    return ESP_OK;
}

esp_err_t espnow_link_stop(void)
{
    if (!g_espnow.initialized || !g_espnow.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_espnow.started = false;
    
    // Wait for task to finish
    if (g_espnow.tx_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
        g_espnow.tx_task = NULL;
    }
    
    ESP_LOGI(TAG, "ESP-NOW link stopped");
    
    return ESP_OK;
}

esp_err_t espnow_link_add_peer(const uint8_t *peer_mac, bool encrypt, const uint8_t *lmk)
{
    if (!g_espnow.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (peer_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (encrypt && lmk == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if already exists
    if (espnow_find_peer(peer_mac) >= 0) {
        ESP_LOGW(TAG, "Peer already exists");
        return ESP_OK;
    }
    
    // Find free slot
    int slot = espnow_find_free_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "Peer list full");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure peer
    memset(&g_espnow.peers[slot], 0, sizeof(esp_now_peer_info_t));
    memcpy(g_espnow.peers[slot].peer_addr, peer_mac, 6);
    g_espnow.peers[slot].channel = 0;  // Use current channel
    g_espnow.peers[slot].ifidx = WIFI_IF_STA;
    g_espnow.peers[slot].encrypt = encrypt;
    
    if (encrypt) {
        memcpy(g_espnow.peers[slot].lmk, lmk, 16);
    }
    
    // Add peer to ESP-NOW
    esp_err_t ret = esp_now_add_peer(&g_espnow.peers[slot]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    g_espnow.peer_count++;
    ESP_LOGI(TAG, "Added peer " MACSTR, MAC2STR(peer_mac));
    
    return ESP_OK;
}

esp_err_t espnow_link_remove_peer(const uint8_t *peer_mac)
{
    if (!g_espnow.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (peer_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find peer
    int slot = espnow_find_peer(peer_mac);
    if (slot < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Remove from ESP-NOW
    esp_err_t ret = esp_now_del_peer(peer_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove peer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Clear slot
    memset(&g_espnow.peers[slot], 0, sizeof(esp_now_peer_info_t));
    g_espnow.peer_count--;
    
    ESP_LOGI(TAG, "Removed peer " MACSTR, MAC2STR(peer_mac));
    
    return ESP_OK;
}

esp_err_t espnow_link_register_rx_callback(espnow_rx_callback_t callback, void *ctx)
{
    if (!g_espnow.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_espnow.rx_callback = callback;
    g_espnow.rx_callback_ctx = ctx;
    
    return ESP_OK;
}

esp_err_t espnow_link_send_engine_status(const espnow_engine_status_t *status)
{
    if (!g_espnow.initialized || !g_espnow.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    espnow_tx_item_t item;
    esp_err_t ret = espnow_build_message(
        ESPNOW_MSG_ENGINE_STATUS,
        (const uint8_t *)status,
        sizeof(espnow_engine_status_t),
        0,
        item.data,
        &item.len
    );
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Broadcast to all peers
    memcpy(item.dest_mac, g_espnow.broadcast_mac, 6);
    item.retry_count = 0;
    
    if (xQueueSend(g_espnow.tx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

esp_err_t espnow_link_send_sensor_data(const espnow_sensor_data_t *data)
{
    if (!g_espnow.initialized || !g_espnow.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    espnow_tx_item_t item;
    esp_err_t ret = espnow_build_message(
        ESPNOW_MSG_SENSOR_DATA,
        (const uint8_t *)data,
        sizeof(espnow_sensor_data_t),
        0,
        item.data,
        &item.len
    );
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Broadcast to all peers
    memcpy(item.dest_mac, g_espnow.broadcast_mac, 6);
    item.retry_count = 0;
    
    if (xQueueSend(g_espnow.tx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

esp_err_t espnow_link_send_diagnostic(const espnow_diagnostic_t *diag)
{
    if (!g_espnow.initialized || !g_espnow.started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (diag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    espnow_tx_item_t item;
    esp_err_t ret = espnow_build_message(
        ESPNOW_MSG_DIAGNOSTIC,
        (const uint8_t *)diag,
        sizeof(espnow_diagnostic_t),
        0,
        item.data,
        &item.len
    );
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Broadcast to all peers
    memcpy(item.dest_mac, g_espnow.broadcast_mac, 6);
    item.retry_count = 0;
    
    if (xQueueSend(g_espnow.tx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

esp_err_t espnow_link_send_config_response(const uint8_t *peer_mac, 
                                            const espnow_config_response_t *response)
{
    if (!g_espnow.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (response == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    espnow_tx_item_t item;
    esp_err_t ret = espnow_build_message(
        ESPNOW_MSG_CONFIG_RESPONSE,
        (const uint8_t *)response,
        sizeof(espnow_config_response_t),
        ESPNOW_FLAG_ACK_REQUIRED,
        item.data,
        &item.len
    );
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Use provided MAC or broadcast
    if (peer_mac != NULL) {
        memcpy(item.dest_mac, peer_mac, 6);
    } else {
        memcpy(item.dest_mac, g_espnow.broadcast_mac, 6);
    }
    item.retry_count = 0;
    
    if (xQueueSend(g_espnow.tx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

void espnow_link_get_stats(uint32_t *tx_count, uint32_t *rx_count, 
                           uint32_t *tx_errors, uint32_t *rx_errors)
{
    if (tx_count != NULL) {
        *tx_count = g_espnow.tx_count;
    }
    if (rx_count != NULL) {
        *rx_count = g_espnow.rx_count;
    }
    if (tx_errors != NULL) {
        *tx_errors = g_espnow.tx_errors;
    }
    if (rx_errors != NULL) {
        *rx_errors = g_espnow.rx_errors;
    }
}

bool espnow_link_is_initialized(void)
{
    return g_espnow.initialized;
}

bool espnow_link_is_started(void)
{
    return g_espnow.started;
}

uint8_t espnow_link_get_peer_count(void)
{
    return g_espnow.peer_count;
}
