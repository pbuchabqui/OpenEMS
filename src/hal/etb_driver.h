/**
 * @file etb_driver.h
 * @brief HAL Driver para Borboleta Eletrônica (ETB)
 * 
 * Controle de motor DC com ponte H, leitura dual de TPS e proteção de hardware.
 * STM32H562 - Timer PWM com dead-time inserido por hardware.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// CONFIGURAÇÃO DE HARDWARE
// ============================================================================

// Pinagem (ajustar conforme schematic)
#define ETB_PWM_PIN             PA8   // TIM1_CH1 - PWM controle motor
#define ETB_IN1_PIN             PA9   // Direção 1
#define ETB_IN2_PIN             PA10  // Direção 2
#define ETB_TPS1_PIN            PA0   // ADC1_INP0 - Sensor primário
#define ETB_TPS2_PIN            PA1   // ADC1_INP1 - Sensor secundário (redundante)

// Frequência PWM (20kHz para evitar ruído audível)
#define ETB_PWM_FREQ_HZ         20000u
#define ETB_PWM_RESOLUTION      10u   // 10-bit = 0-1023

// Dead-time em nanossegundos (hardware TIM1 insert)
#define ETB_DEADTIME_NS         200u  // 200ns previne shoot-through

// Faixas dos sensores TPS (0-3.3V → 0-4095 ADC 12-bit)
#define ETB_TPS_ADC_MIN         200u    // ~0.16V (ruído/falha aberto)
#define ETB_TPS_ADC_MAX         3900u   // ~3.15V (falha curto)
#define ETB_TPS_NORMAL_MIN      400u    // ~0.33V (borboleta fechada mecânica)
#define ETB_TPS_NORMAL_MAX      3700u   // ~3.0V (borboleta aberta máxima)

// Tolerância entre TPS1 e TPS2 para validação (mV)
#define ETB_TPS_MISMATCH_MV     400u    // 400mV de tolerância

// ============================================================================
// TIPOS E ESTRUTURAS
// ============================================================================

typedef enum {
    ETB_DRV_OK = 0,
    ETB_DRV_FAULT_TPS1_OPEN,
    ETB_DRV_FAULT_TPS1_SHORT,
    ETB_DRV_FAULT_TPS2_OPEN,
    ETB_DRV_FAULT_TPS2_SHORT,
    ETB_DRV_FAULT_TPS_MISMATCH,
    ETB_DRV_FAULT_OVERCURRENT,
    ETB_DRV_FAULT_NOT_INITIALIZED
} etb_driver_fault_t;

typedef enum {
    ETB_DRV_STATE_OFF = 0,
    ETB_DRV_STATE_INIT,
    ETB_DRV_STATE_READY,
    ETB_DRV_STATE_FAULT
} etb_driver_state_t;

typedef struct {
    uint16_t tps1_raw;          // Leitura bruta ADC1
    uint16_t tps2_raw;          // Leitura bruta ADC2
    float tps1_percent;         // 0.0 - 100.0%
    float tps2_percent;         // 0.0 - 100.0%
    float tps_validated;        // Valor validado (média se coerente)
    int16_t motor_pwm;          // -1023 a +1023 (negativo = fechar)
    etb_driver_state_t state;
    etb_driver_fault_t fault;
    uint32_t fault_count;
    uint32_t last_update_us;
} etb_driver_data_t;

// ============================================================================
// API PÚBLICA
// ============================================================================

/**
 * @brief Inicializa hardware ETB (TIM1 PWM + ADC + GPIOs)
 * @return true se sucesso, false se falha de hardware
 */
bool etb_driver_init(void);

/**
 * @brief Lê sensores TPS1/TPS2 e valida coerência
 * @param data Ponteiro para estrutura de dados
 * @return ETB_DRV_OK ou código de falha
 */
etb_driver_fault_t etb_driver_read_sensors(etb_driver_data_t* data);

/**
 * @brief Aplica comando PWM ao motor (com dead-time hardware)
 * @param pwm Duty cycle: -1023 (fechar) a +1023 (abrir), 0 = freio
 * @return true se aplicado, false se em falha
 */
bool etb_driver_set_motor_pwm(int16_t pwm);

/**
 * @brief Desliga motor e entra em estado seguro (mola fecha borboleta)
 */
void etb_driver_shutdown(void);

/**
 * @brief Reseta falhas após diagnóstico
 */
void etb_driver_clear_fault(void);

/**
 * @brief Obtém estado atual do driver
 */
etb_driver_state_t etb_driver_get_state(void);

/**
 * @brief Converte ADC raw para porcentagem (0-100%)
 */
float etb_driver_adc_to_percent(uint16_t adc_raw);

#ifdef __cplusplus
}
#endif
