/**
 * @file high_precision_timing.c
 * @brief Implementação do sistema de alta precisão de timing para ESP32-S3 EFI
 */

#include "scheduler/hp_timing.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char* TAG = "HP_TIMING";

//=============================================================================
// PREDITOR DE FASE ADAPTATIVO
//=============================================================================

void hp_init_phase_predictor(phase_predictor_t *predictor, float initial_period) {
    if (predictor == NULL || initial_period <= 0) {
        ESP_LOGE(TAG, "Invalid parameters for phase predictor initialization");
        return;
    }
    
    memset(predictor, 0, sizeof(phase_predictor_t));
    predictor->predicted_period = initial_period;
    predictor->last_period = initial_period;
    predictor->alpha = 0.1f;
    predictor->acceleration = 0.0f;
    predictor->last_dt = 0.0f;
    predictor->last_timestamp = 0;
    predictor->tooth_count = 0;
    
    ESP_LOGI(TAG, "Phase predictor initialized with period: %.2f us", initial_period);
}

IRAM_ATTR void hp_update_phase_predictor(phase_predictor_t *predictor, float current_period, uint32_t timestamp) {
    if (predictor == NULL || current_period <= 0) {
        return;
    }
    
    // Calcular delta de tempo desde última medição
    float dt = 0.0f;
    if (predictor->last_timestamp > 0) {
        dt = hp_cycles_to_us(timestamp - predictor->last_timestamp);
        // Limitar dt para evitar valores extremos
        if (dt > 100000.0f || dt < 0) {
            dt = 0.0f;
        }
    }
    predictor->last_dt = dt;
    
    // Calcular aceleração apenas se temos histórico
    if (predictor->predicted_period > 0 && dt > 0) {
        predictor->acceleration = (current_period - predictor->predicted_period) / dt;
    }
    
    // Calcular fator de adaptação baseado na magnitude da aceleração
    float abs_accel = fabsf(predictor->acceleration);
    // Fator adaptativo: maior aceleração = menor peso (mais reativo)
    predictor->alpha = 0.05f + (1.0f / (1.0f + abs_accel * 0.001f)) * 0.2f;
    // Limitar alpha entre 0.05 e 0.5
    predictor->alpha = fmaxf(0.05f, fminf(0.5f, predictor->alpha));
    
    // Atualizar predição com filtro adaptativo
    predictor->predicted_period = current_period + (predictor->acceleration * dt * predictor->alpha);
    predictor->last_period = current_period;
    predictor->last_timestamp = timestamp;
    predictor->tooth_count++;
}

//=============================================================================
// COMPENSAÇÃO DE LATÊNCIA FÍSICA
//=============================================================================

void hp_init_hardware_latency(hardware_latency_comp_t *comp) {
    if (comp == NULL) {
        return;
    }
    
    memset(comp, 0, sizeof(hardware_latency_comp_t));
    
    // Valores típicos baseados em componentes automotivos comuns
    comp->coil_delay_base = 100.0f;      // 100us latência típica da bobina
    comp->injector_delay_base = 50.0f;   // 50us latência típica do injetor
    comp->voltage_factor = 0.05f;         // 5% de variação por volt de diferença
    comp->temp_factor = 0.001f;           // 0.1% de variação por grau Celsius
    comp->temp_reference = 25.0f;         // Temperatura de referência (25°C)
    
    ESP_LOGI(TAG, "Hardware latency compensation initialized");
    ESP_LOGI(TAG, "  Coil base delay: %.1f us", comp->coil_delay_base);
    ESP_LOGI(TAG, "  Injector base delay: %.1f us", comp->injector_delay_base);
}

//=============================================================================
// MEDIÇÃO DE JITTER
//=============================================================================

void hp_init_jitter_measurer(jitter_measurer_t *measurer) {
    if (measurer == NULL) {
        return;
    }
    
    memset(measurer, 0, sizeof(jitter_measurer_t));
    measurer->min_jitter = UINT32_MAX;
    measurer->is_first_sample = true;
    
    ESP_LOGI(TAG, "Jitter measurer initialized");
}

IRAM_ATTR void hp_record_jitter(jitter_measurer_t *measurer, uint32_t target_cycles, uint32_t actual_cycles) {
    if (measurer == NULL) {
        return;
    }
    
    uint32_t jitter = (actual_cycles > target_cycles) ? 
                      (actual_cycles - target_cycles) : 
                      (target_cycles - actual_cycles);
    
    if (measurer->is_first_sample) {
        measurer->min_jitter = jitter;
        measurer->is_first_sample = false;
    } else {
        if (jitter > measurer->max_jitter) {
            measurer->max_jitter = jitter;
        }
        if (jitter < measurer->min_jitter) {
            measurer->min_jitter = jitter;
        }
    }
    
    measurer->jitter_sum += jitter;
    measurer->sample_count++;
    measurer->last_target = target_cycles;
    measurer->last_actual = actual_cycles;
}

void hp_get_jitter_stats(jitter_measurer_t *measurer, 
                         float *out_avg_jitter,
                         float *out_max_jitter,
                         float *out_min_jitter) {
    if (measurer == NULL || measurer->sample_count == 0) {
        if (out_avg_jitter) *out_avg_jitter = 0;
        if (out_max_jitter) *out_max_jitter = 0;
        if (out_min_jitter) *out_min_jitter = 0;
        return;
    }
    
    if (out_avg_jitter) {
        float avg_cycles = (float)measurer->jitter_sum / (float)measurer->sample_count;
        *out_avg_jitter = hp_cycles_to_us((uint32_t)avg_cycles);
    }
    
    if (out_max_jitter) {
        *out_max_jitter = hp_cycles_to_us(measurer->max_jitter);
    }
    
    if (out_min_jitter) {
        *out_min_jitter = hp_cycles_to_us(measurer->min_jitter);
    }
}

//=============================================================================
// CONFIGURAÇÃO DE CORE E PRIORIDADE
//=============================================================================

// Temporarily disabled - requires SMP FreeRTOS
#if 0
BaseType_t hp_set_task_core_affinity(TaskHandle_t task_handle, UBaseType_t core_id) {
    if (task_handle == NULL || core_id > 1) {
        ESP_LOGE(TAG, "Invalid parameters for core affinity");
        return pdFALSE;
    }
    
    vTaskCoreAffinitySet(task_handle, (1 << core_id));
    ESP_LOGI(TAG, "Task 0x%08X assigned to core %d", (unsigned)task_handle, core_id);
    
    return pdTRUE;
}
#endif

BaseType_t hp_set_task_max_priority(TaskHandle_t task_handle) {
    if (task_handle == NULL) {
        ESP_LOGE(TAG, "Invalid task handle for priority setting");
        return pdFALSE;
    }
    
    UBaseType_t current_priority = uxTaskPriorityGet(task_handle);
    UBaseType_t max_priority = configMAX_PRIORITIES - 1;
    
    if (current_priority == max_priority) {
        ESP_LOGI(TAG, "Task already at maximum priority: %lu", (unsigned long)current_priority);
        return pdPASS;
    }
    
    vTaskPrioritySet(task_handle, max_priority);
    
    ESP_LOGI(TAG, "Task priority set to maximum: %lu", (unsigned long)max_priority);
    
    return pdPASS;
}

BaseType_t hp_create_critical_task(TaskFunction_t pvTaskCode,
                                   const char * const pcName,
                                   const configSTACK_DEPTH_TYPE usStackDepth,
                                   void * const pvParameters,
                                   UBaseType_t uxPriority,
                                   TaskHandle_t * const pvCreatedTask,
                                   BaseType_t core_id) {
    if (pvTaskCode == NULL || pcName == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for critical task creation");
        return pdFALSE;
    }
    
    // Usar prioridade máxima se uxPriority for 0
    if (uxPriority == 0) {
        uxPriority = configMAX_PRIORITIES - 1;
    }
    
    BaseType_t result = xTaskCreatePinnedToCore(
        pvTaskCode,
        pcName,
        usStackDepth,
        pvParameters,
        uxPriority,
        pvCreatedTask,
        core_id
    );
    
    if (result == pdPASS) {
        ESP_LOGI(TAG, "Critical task '%s' created on core %d with priority %lu",
                 pcName, core_id, (unsigned long)uxPriority);
    } else {
        ESP_LOGE(TAG, "Failed to create critical task '%s'", pcName);
    }
    
    return result;
}
