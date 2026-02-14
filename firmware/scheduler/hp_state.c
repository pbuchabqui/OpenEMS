/**
 * @file hp_state.c
 * @brief Implementação do módulo centralizado de estado de alta precisão
 * 
 * Este módulo gerencia o estado compartilhado entre todos os componentes
 * de timing de alta precisão, eliminando duplicação de estado.
 */

#include "scheduler/hp_state.h"
#include "scheduler/hp_timing.h"
#include <string.h>

// Estado global de alta precisão
static phase_predictor_t g_phase_predictor;
static hardware_latency_comp_t g_hw_latency;
static jitter_measurer_t g_jitter_measurer;
static bool g_initialized = false;

bool hp_state_init(float initial_period_us) {
    if (g_initialized) {
        return true;
    }
    
    memset(&g_phase_predictor, 0, sizeof(g_phase_predictor));
    memset(&g_hw_latency, 0, sizeof(g_hw_latency));
    memset(&g_jitter_measurer, 0, sizeof(g_jitter_measurer));
    
    hp_init_phase_predictor(&g_phase_predictor, initial_period_us);
    hp_init_hardware_latency(&g_hw_latency);
    hp_init_jitter_measurer(&g_jitter_measurer);
    
    g_initialized = true;
    return true;
}

phase_predictor_t* hp_state_get_phase_predictor(void) {
    return &g_phase_predictor;
}

hardware_latency_comp_t* hp_state_get_hw_latency(void) {
    return &g_hw_latency;
}

jitter_measurer_t* hp_state_get_jitter_measurer(void) {
    return &g_jitter_measurer;
}

void hp_state_update_phase_predictor(float measured_period_us, uint32_t timestamp) {
    hp_update_phase_predictor(&g_phase_predictor, measured_period_us, timestamp);
}

float hp_state_predict_next_period(float fallback_period) {
    return hp_predict_next_period(&g_phase_predictor, fallback_period);
}

float hp_state_get_latency(float battery_voltage, float temperature) {
    return hp_get_coil_latency(&g_hw_latency, battery_voltage, temperature);
}

float hp_state_get_injector_latency(float battery_voltage, float temperature) {
    return hp_get_injector_latency(&g_hw_latency, battery_voltage, temperature);
}

void hp_state_record_jitter(uint32_t expected_us, uint32_t actual_us) {
    int32_t jitter = (int32_t)actual_us - (int32_t)expected_us;
    hp_record_jitter(&g_jitter_measurer, (float)jitter);
}

void hp_state_get_jitter_stats(float *avg_us, float *max_us, float *min_us) {
    hp_get_jitter_stats(&g_jitter_measurer, avg_us, max_us, min_us);
}
