/**
 * @file esp32s3_main_integration.h
 * @brief Interface principal para integração das melhorias ESP32-S3
 * 
 * Este header fornece uma interface simplificada para integrar
 * todas as melhorias competitivas do ESP32-S3 no sistema principal.
 */

#ifndef ESP32S3_MAIN_INTEGRATION_H
#define ESP32S3_MAIN_INTEGRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "integration/esp32s3_integration.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Funções Principais
//=============================================================================

/**
 * @brief Inicializa o sistema de melhorias ESP32-S3
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_main_init(void);

/**
 * @brief Inicia as tarefas de processamento ESP32-S3
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_main_start(void);

/**
 * @brief Para o sistema ESP32-S3
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_main_stop(void);

/**
 * @brief Obtém o contexto de integração atual
 * @param integration Ponteiro para o contexto (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_main_get_status(esp32s3_integration_t **integration);

/**
 * @brief Envia telemetria via ESP-NOW otimizado
 * @param integration Ponteiro para a integração
 * @param peer_mac MAC do peer (NULL para broadcast)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_send_telemetry(esp32s3_integration_t *integration,
                                   const uint8_t *peer_mac);

/**
 * @brief Executa diagnóstico completo do sistema ESP32-S3
 * @return ESP_OK em caso de sucesso
 */
esp_err_t esp32s3_main_run_diagnostics(void);

#ifdef __cplusplus
}
#endif

#endif // ESP32S3_MAIN_INTEGRATION_H
