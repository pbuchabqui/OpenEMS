#include "twai_lambda.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "s3_control_config.h"
#include "engine_control.h"
#include <stdint.h>

typedef enum {
    PROTOCOL_UNKNOWN = 0,
    PROTOCOL_FUELTEC_NANO_V1,
    PROTOCOL_FUELTEC_NANO_V2,
    PROTOCOL_GENERIC_WIDEBAND,
    PROTOCOL_MAX
} protocol_type_t;

typedef struct {
    uint32_t can_id;
    uint8_t data_length;
    uint8_t afr_offset;
    uint8_t status_offset;
} lambda_protocol_t;

static const char *TAG = "TWAI_LAMBDA";

#define TWAI_EOIT_CMD_ID 0x6E0U
#define TWAI_EOIT_RSP_ID 0x6E1U
#define TWAI_EOIT_CMD_SET_CAL 0xA1U
#define TWAI_EOIT_CMD_SET_MAP_ENABLE 0xA2U
#define TWAI_EOIT_CMD_SET_MAP_CELL 0xA3U
#define TWAI_EOIT_CMD_GET_DIAG 0xA4U
#define TWAI_EOIT_CMD_GET_CAL 0xA5U

static const lambda_protocol_t g_protocols[PROTOCOL_MAX] = {
    [PROTOCOL_UNKNOWN] = {0},
    [PROTOCOL_FUELTEC_NANO_V1] = { .can_id = 0x7E8, .data_length = 3, .afr_offset = 0, .status_offset = 2 },
    [PROTOCOL_FUELTEC_NANO_V2] = { .can_id = 0x7E9, .data_length = 4, .afr_offset = 0, .status_offset = 2 },
    [PROTOCOL_GENERIC_WIDEBAND] = { .can_id = 0x7E0, .data_length = 3, .afr_offset = 0, .status_offset = 2 },
};

static TaskHandle_t g_can_task = NULL;
static volatile bool g_can_running = false;
static bool g_can_initialized = false;
static float g_latest_lambda = 1.0f;
static uint32_t g_latest_timestamp_ms = 0;
static twai_lambda_callback_t g_lambda_cb = NULL;
static void *g_lambda_cb_ctx = NULL;
static portMUX_TYPE g_twai_spinlock = portMUX_INITIALIZER_UNLOCKED;

static void can_rx_task(void *arg);
static protocol_type_t detect_protocol(const twai_message_t *msg);
static bool handle_eoit_command(const twai_message_t *msg);
static void send_eoit_ack(uint8_t cmd, esp_err_t status);
static int16_t decode_i16_be(const uint8_t *p);
static void encode_i16_be(uint8_t *p, int16_t v);

esp_err_t twai_lambda_init(void) {
    if (g_can_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return err;
    }

    g_can_running = true;
    BaseType_t ok = xTaskCreatePinnedToCore(can_rx_task, "twai_rx",
                                            COMM_TASK_STACK, NULL, COMM_TASK_PRIORITY, &g_can_task,
                                            COMM_TASK_CORE);
    if (ok != pdPASS) {
        g_can_running = false;
        twai_stop();
        twai_driver_uninstall();
        return ESP_FAIL;
    }

    g_can_initialized = true;
    ESP_LOGI(TAG, "TWAI lambda RX started");
    return ESP_OK;
}

void twai_lambda_deinit(void) {
    if (!g_can_initialized) {
        return;
    }
    g_can_running = false;
    if (g_can_task) {
        vTaskDelete(g_can_task);
        g_can_task = NULL;
    }
    twai_stop();
    twai_driver_uninstall();
    twai_lambda_unregister_callback();
    g_can_initialized = false;
}

bool twai_lambda_get_latest(float *out_lambda, uint32_t *out_age_ms) {
    if (!out_lambda) {
        return false;
    }
    float lambda = 1.0f;
    uint32_t ts_ms = 0;
    portENTER_CRITICAL(&g_twai_spinlock);
    lambda = g_latest_lambda;
    ts_ms = g_latest_timestamp_ms;
    portEXIT_CRITICAL(&g_twai_spinlock);

    if (ts_ms == 0) {
        return false;
    }
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (out_age_ms) {
        *out_age_ms = (now_ms >= ts_ms) ? (now_ms - ts_ms) : 0U;
    }
    *out_lambda = lambda;
    return true;
}

esp_err_t twai_lambda_register_callback(twai_lambda_callback_t cb, void *ctx) {
    portENTER_CRITICAL(&g_twai_spinlock);
    g_lambda_cb = cb;
    g_lambda_cb_ctx = ctx;
    portEXIT_CRITICAL(&g_twai_spinlock);
    return ESP_OK;
}

void twai_lambda_unregister_callback(void) {
    portENTER_CRITICAL(&g_twai_spinlock);
    g_lambda_cb = NULL;
    g_lambda_cb_ctx = NULL;
    portEXIT_CRITICAL(&g_twai_spinlock);
}

static void can_rx_task(void *arg) {
    (void)arg;
    while (g_can_running) {
        twai_message_t msg = {0};
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            continue;
        }

        if (handle_eoit_command(&msg)) {
            continue;
        }

        protocol_type_t proto = detect_protocol(&msg);
        if (proto == PROTOCOL_UNKNOWN) {
            continue;
        }

        const lambda_protocol_t *p = &g_protocols[proto];
        if (msg.data_length_code < p->data_length) {
            continue;
        }

        uint16_t afr_raw = ((uint16_t)msg.data[p->afr_offset] << 8) |
                           (uint16_t)msg.data[p->afr_offset + 1];
        uint8_t status = msg.data[p->status_offset];
        if ((status & 0x01) == 0) {
            continue;
        }

        float new_lambda = afr_raw / 14.7f;
        uint32_t ts_ms = (uint32_t)(esp_timer_get_time() / 1000);
        twai_lambda_callback_t cb = NULL;
        void *cb_ctx = NULL;

        portENTER_CRITICAL(&g_twai_spinlock);
        g_latest_lambda = new_lambda;
        g_latest_timestamp_ms = ts_ms;
        cb = g_lambda_cb;
        cb_ctx = g_lambda_cb_ctx;
        portEXIT_CRITICAL(&g_twai_spinlock);

        if (cb != NULL) {
            cb(new_lambda, ts_ms, cb_ctx);
        }
    }
    vTaskDelete(NULL);
}

static bool handle_eoit_command(const twai_message_t *msg) {
    if (!msg || msg->identifier != TWAI_EOIT_CMD_ID || msg->data_length_code < 1U) {
        return false;
    }

    uint8_t cmd = msg->data[0];
    esp_err_t err = ESP_ERR_INVALID_ARG;

    if (cmd == TWAI_EOIT_CMD_SET_CAL && msg->data_length_code >= 7U) {
        float boundary = decode_i16_be(&msg->data[1]) / 100.0f;
        float normal = decode_i16_be(&msg->data[3]) / 100.0f;
        float fallback = decode_i16_be(&msg->data[5]) / 100.0f;
        err = engine_control_set_eoit_calibration(boundary, normal, fallback);
        send_eoit_ack(cmd, err);
        return true;
    }

    if (cmd == TWAI_EOIT_CMD_SET_MAP_ENABLE && msg->data_length_code >= 2U) {
        err = engine_control_set_eoit_map_enabled(msg->data[1] != 0U);
        send_eoit_ack(cmd, err);
        return true;
    }

    if (cmd == TWAI_EOIT_CMD_SET_MAP_CELL && msg->data_length_code >= 5U) {
        uint8_t rpm_idx = msg->data[1];
        uint8_t load_idx = msg->data[2];
        float normal = decode_i16_be(&msg->data[3]) / 100.0f;
        err = engine_control_set_eoit_map_cell(rpm_idx, load_idx, normal);
        send_eoit_ack(cmd, err);
        return true;
    }

    if (cmd == TWAI_EOIT_CMD_GET_DIAG) {
        engine_injection_diag_t diag = {0};
        err = engine_control_get_injection_diag(&diag);
        twai_message_t rsp = {0};
        rsp.identifier = TWAI_EOIT_RSP_ID;
        rsp.data_length_code = 8U;
        rsp.data[0] = cmd;
        rsp.data[1] = (err == ESP_OK) ? 0U : 1U;
        encode_i16_be(&rsp.data[2], (int16_t)(diag.eoit_target_deg * 10.0f));
        encode_i16_be(&rsp.data[4], (int16_t)(diag.normal_used * 100.0f));
        uint16_t delay0 = (diag.delay_us[0] > 65535U) ? 65535U : (uint16_t)diag.delay_us[0];
        rsp.data[6] = (uint8_t)((delay0 >> 8) & 0xFFU);
        rsp.data[7] = (uint8_t)(delay0 & 0xFFU);
        (void)twai_transmit(&rsp, 0);
        return true;
    }

    if (cmd == TWAI_EOIT_CMD_GET_CAL) {
        float boundary = 0.0f;
        float normal = 0.0f;
        float fallback = 0.0f;
        err = engine_control_get_eoit_calibration(&boundary, &normal, &fallback);
        twai_message_t rsp = {0};
        rsp.identifier = TWAI_EOIT_RSP_ID;
        rsp.data_length_code = 8U;
        rsp.data[0] = cmd;
        rsp.data[1] = (err == ESP_OK) ? 0U : 1U;
        encode_i16_be(&rsp.data[2], (int16_t)(boundary * 100.0f));
        encode_i16_be(&rsp.data[4], (int16_t)(normal * 100.0f));
        encode_i16_be(&rsp.data[6], (int16_t)(fallback * 100.0f));
        (void)twai_transmit(&rsp, 0);
        return true;
    }

    send_eoit_ack(cmd, ESP_ERR_INVALID_ARG);
    return true;
}

static void send_eoit_ack(uint8_t cmd, esp_err_t status) {
    twai_message_t rsp = {0};
    rsp.identifier = TWAI_EOIT_RSP_ID;
    rsp.data_length_code = 4U;
    rsp.data[0] = cmd;
    rsp.data[1] = (status == ESP_OK) ? 0U : 1U;
    rsp.data[2] = (uint8_t)((status >> 8) & 0xFFU);
    rsp.data[3] = (uint8_t)(status & 0xFFU);
    (void)twai_transmit(&rsp, 0);
}

static int16_t decode_i16_be(const uint8_t *p) {
    if (!p) {
        return 0;
    }
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void encode_i16_be(uint8_t *p, int16_t v) {
    if (!p) {
        return;
    }
    p[0] = (uint8_t)(((uint16_t)v >> 8) & 0xFFU);
    p[1] = (uint8_t)((uint16_t)v & 0xFFU);
}

static protocol_type_t detect_protocol(const twai_message_t *msg) {
    if (!msg) {
        return PROTOCOL_UNKNOWN;
    }
    for (int i = 1; i < PROTOCOL_MAX; i++) {
        if (msg->identifier == g_protocols[i].can_id &&
            msg->data_length_code == g_protocols[i].data_length) {
            return (protocol_type_t)i;
        }
    }
    return PROTOCOL_UNKNOWN;
}
