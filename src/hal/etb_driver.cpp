/**
 * @file etb_driver.cpp
 * @brief Implementação HAL Driver Borboleta Eletrônica (ETB)
 * 
 * STM32H562 - TIM1 PWM com dead-time, ADC dual para TPS
 */

#include "etb_driver.h"
#include "stm32h5xx.h"
#include "adc.h"      // HAL ADC existente
#include "timer.h"    // HAL Timer existente
#include <string.h>

// ============================================================================
// VARIÁVEIS GLOBAIS
// ============================================================================

static etb_driver_data_t g_etb_data = {0};
static etb_driver_state_t g_state = ETB_DRV_STATE_OFF;
static etb_driver_fault_t g_fault = ETB_DRV_OK;
static uint32_t g_fault_count = 0;

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

float etb_driver_adc_to_percent(uint16_t adc_raw) {
    // Mapeia ADC (400-3700) → 0-100%
    if (adc_raw <= ETB_TPS_NORMAL_MIN) return 0.0f;
    if (adc_raw >= ETB_TPS_NORMAL_MAX) return 100.0f;
    
    float percent = ((float)(adc_raw - ETB_TPS_NORMAL_MIN) / 
                     (float)(ETB_TPS_NORMAL_MAX - ETB_TPS_NORMAL_MIN)) * 100.0f;
    
    // Clamp 0-100
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    return percent;
}

// ============================================================================
// INICIALIZAÇÃO
// ============================================================================

bool etb_driver_init(void) {
    // Reset estrutura
    memset(&g_etb_data, 0, sizeof(g_etb_data));
    g_state = ETB_DRV_STATE_INIT;
    g_fault = ETB_DRV_OK;
    
    // 1. Configurar GPIOs
    // PA8 (TIM1_CH1), PA9 (IN1), PA10 (IN2) - AF1 para TIM1
    // PA0, PA1 - ADC1_INP0, ADC1_INP1 (analógico)
    // NOTA: Implementação específica depende do HAL já existente
    
    // 2. Configurar TIM1 para PWM com dead-time
    // Frequência: 20kHz, Resolução: 10-bit
    // Dead-time: 200ns inserido por hardware no TIM1 BDTR register
    // NOTA: Chamar timer_etb_init() do módulo timer.c existente
    
    // 3. Configurar ADC para leitura dual TPS1/TPS2
    // Trigger por software ou timer
    // NOTA: ADC já inicializado em adc_init(), apenas configurar canais
    
    // 4. Desligar motor inicialmente (segurança)
    etb_driver_shutdown();
    
    // 5. Aguardar estabilização sensores
    for (volatile int i = 0; i < 10000; i++) __NOP();
    
    // 6. Ler sensores pela primeira vez
    etb_driver_fault_t fault = etb_driver_read_sensors(&g_etb_data);
    
    if (fault != ETB_DRV_OK) {
        g_state = ETB_DRV_STATE_FAULT;
        g_fault = fault;
        g_fault_count++;
        return false;
    }
    
    g_state = ETB_DRV_STATE_READY;
    g_etb_data.last_update_us = 0; // Será atualizado no loop
    
    return true;
}

// ============================================================================
// LEITURA DE SENSORES
// ============================================================================

etb_driver_fault_t etb_driver_read_sensors(etb_driver_data_t* data) {
    if (data == NULL) return ETB_DRV_FAULT_NOT_INITIALIZED;
    if (g_state == ETB_DRV_STATE_FAULT) return g_fault;
    
    // 1. Ler ADCs (implementação depende do HAL adc_read_channel)
    // Supondo adc_read_channel(uint8_t channel) já existe
    data->tps1_raw = adc_read_channel(0); // PA0 - ADC1_INP0
    data->tps2_raw = adc_read_channel(1); // PA1 - ADC1_INP1
    
    // 2. Validar faixas
    if (data->tps1_raw < ETB_TPS_ADC_MIN) {
        return ETB_DRV_FAULT_TPS1_OPEN;
    }
    if (data->tps1_raw > ETB_TPS_ADC_MAX) {
        return ETB_DRV_FAULT_TPS1_SHORT;
    }
    if (data->tps2_raw < ETB_TPS_ADC_MIN) {
        return ETB_DRV_FAULT_TPS2_OPEN;
    }
    if (data->tps2_raw > ETB_TPS_ADC_MAX) {
        return ETB_DRV_FAULT_TPS2_SHORT;
    }
    
    // 3. Converter para porcentagem
    data->tps1_percent = etb_driver_adc_to_percent(data->tps1_raw);
    data->tps2_percent = etb_driver_adc_to_percent(data->tps2_raw);
    
    // 4. Validar coerência entre TPS1 e TPS2
    // Diferença máxima permitida: 400mV ≈ 12% da faixa
    float diff = (data->tps1_percent > data->tps2_percent) ? 
                 (data->tps1_percent - data->tps2_percent) :
                 (data->tps2_percent - data->tps1_percent);
    
    if (diff > 12.0f) { // 400mV / 3.3V * 100 ≈ 12%
        return ETB_DRV_FAULT_TPS_MISMATCH;
    }
    
    // 5. Calcular valor validado (média ponderada)
    data->tps_validated = (data->tps1_percent + data->tps2_percent) / 2.0f;
    
    return ETB_DRV_OK;
}

// ============================================================================
// CONTROLE DO MOTOR
// ============================================================================

bool etb_driver_set_motor_pwm(int16_t pwm) {
    if (g_state == ETB_DRV_STATE_FAULT) return false;
    if (g_state != ETB_DRV_STATE_READY) return false;
    
    // Clamp -1023 a +1023
    if (pwm > 1023) pwm = 1023;
    if (pwm < -1023) pwm = -1023;
    
    g_etb_data.motor_pwm = pwm;
    
    // Controle de direção e PWM
    // pwm > 0: Abrir (IN1=1, IN2=0, TIM1_CH1 duty positivo)
    // pwm < 0: Fechar (IN1=0, IN2=1, TIM1_CH1 duty negativo)
    // pwm = 0: Freio/High-Z (IN1=0, IN2=0)
    
    if (pwm > 0) {
        // Abrir borboleta
        // Set IN1 high, IN2 low
        // TIM1 CCR1 = pwm
        // NOTA: Implementação específica GPIO/TIMER
    } else if (pwm < 0) {
        // Fechar borboleta
        // Set IN1 low, IN2 high
        // TIM1 CCR1 = -pwm
    } else {
        // Freio (coasting)
        // IN1=0, IN2=0, TIM1 CCR1=0
    }
    
    g_etb_data.last_update_us = 0; // Atualizar timestamp
    
    return true;
}

void etb_driver_shutdown(void) {
    // Desliga motor imediatamente
    // IN1=0, IN2=0 (freio ou coast, dependendo da topologia)
    // TIM1 CCR1=0 (0% duty)
    
    g_etb_data.motor_pwm = 0;
    
    // NOTA: Implementação específica GPIO
    // A mola mecânica fecha a borboleta quando motor desligado
}

void etb_driver_clear_fault(void) {
    if (g_state != ETB_DRV_STATE_FAULT) return;
    
    // Tentar recuperar
    g_fault = ETB_DRV_OK;
    g_state = ETB_DRV_STATE_INIT;
    
    // Re-inicializar
    if (etb_driver_init()) {
        g_state = ETB_DRV_STATE_READY;
    }
}

etb_driver_state_t etb_driver_get_state(void) {
    return g_state;
}
