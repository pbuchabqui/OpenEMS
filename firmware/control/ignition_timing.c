/**
 * @file ignition_timing.c
 * @brief Sistema de timing de ignição de alta precisão
 * 
 * Integração com drivers MCPWM HP:
 * - Timer contínuo com compare absoluto
 * - Preditor de fase adaptativo
 * - Compensação de latência física
 */

#include "ignition_timing.h"
#include "logger.h"
#include "engine_config.h"
#include "mcpwm_ignition.h"
#include "mcpwm_ignition_hp.h"
#include "mcpwm_injection.h"
#include "mcpwm_injection_hp.h"
#include "sensor_processing.h"
#include "sync.h"
#include "hp_state.h"
#include "math_utils.h"

static const float g_cyl_tdc_deg[4] = {0.0f, 180.0f, 360.0f, 540.0f};

static float apply_temp_dwell_bias(float battery_voltage, int16_t clt_c) {
    if (clt_c >= 105) {
        battery_voltage += 1.0f;
    } else if (clt_c >= 95) {
        battery_voltage += 0.5f;
    } else if (clt_c <= 0) {
        battery_voltage -= 0.7f;
    } else if (clt_c <= 20) {
        battery_voltage -= 0.4f;
    }
    return clamp_float(battery_voltage, 8.0f, 16.5f);
}

static float compute_current_angle_deg(const sync_data_t *sync, uint32_t tooth_count) {
    float degrees_per_tooth = 360.0f / (float)(tooth_count + 2);
    float current_angle = (sync->revolution_index * 360.0f) + (sync->tooth_index * degrees_per_tooth);
    return wrap_angle_720(current_angle);
}

static float sync_us_per_degree(const sync_data_t *sync, const sync_config_t *cfg) {
    if (!sync || !cfg || sync->tooth_period == 0U || cfg->tooth_count == 0U) {
        return 0.0f;
    }
    uint32_t total_positions = cfg->tooth_count + 2U;
    return ((float)sync->tooth_period * (float)total_positions) / 360.0f;
}

bool ignition_init(void) {
    // Inicializar módulo de estado HP centralizado
    if (!hp_state_init(10000.0f)) {  // 10ms inicial
        LOG_IGNITION_E("Failed to initialize HP state module");
        return false;
    }
    
    // Inicializar drivers HP
    bool ign_ok = mcpwm_ignition_hp_init();
    bool inj_ok = mcpwm_injection_hp_init();
    
    if (ign_ok && inj_ok) {
        LOG_IGNITION_I("HP Ignition timing system initialized");
        LOG_IGNITION_I("  Phase predictor: active (centralized)");
        LOG_IGNITION_I("  Hardware latency compensation: active (centralized)");
        LOG_IGNITION_I("  Jitter measurement: active (centralized)");
        return true;
    }

    LOG_IGNITION_E("HP Ignition timing init failed (ign=%d, inj=%d)", ign_ok, inj_ok);
    return false;
}

void ignition_apply_timing(uint16_t advance_deg10, uint16_t rpm, float vbat_v) {
    float advance_degrees = advance_deg10 / 10.0f;
    // H2 fix: use caller-supplied battery voltage (from plan snapshot) for
    // consistent dwell calculation. Still read CLT for temperature bias.
    float battery_voltage = (vbat_v > 0.0f) ? vbat_v : 13.5f;

    sensor_data_t sensors = {0};
    if (sensor_get_data_fast(&sensors) == ESP_OK) {
        battery_voltage = apply_temp_dwell_bias(battery_voltage, sensors.clt_c);
    }
    battery_voltage = clamp_float(battery_voltage, 8.0f, 16.5f);

    sync_data_t sync_data = {0};
    sync_config_t sync_cfg = {0};
    bool have_sync = (sync_get_data(&sync_data) == ESP_OK) &&
                     (sync_get_config(&sync_cfg) == ESP_OK) &&
                     sync_data.sync_valid &&
                     sync_data.sync_acquired &&
                     (sync_cfg.tooth_count > 0);
    
    float us_per_deg = 0.0f;
    uint32_t current_counter = 0;

    if (have_sync) {
        us_per_deg = sync_us_per_degree(&sync_data, &sync_cfg);
        if (us_per_deg <= 0.0f) {
            have_sync = false;
        } else {
            // Obter contador atual do timer MCPWM (valor real, não sintético)
            // Usa cilindro 0 como referência (todos os timers estão sincronizados)
            current_counter = mcpwm_ignition_hp_get_counter(0);
        }
    }

    if (have_sync) {
        float current_angle = compute_current_angle_deg(&sync_data, sync_cfg.tooth_count);
        
        for (uint8_t cylinder = 1; cylinder <= 4; cylinder++) {
            float spark_deg = wrap_angle_720(g_cyl_tdc_deg[cylinder - 1] - advance_degrees);
            float delta_deg = spark_deg - current_angle;
            if (delta_deg < 0.0f) {
                delta_deg += 720.0f;
            }
            
            // Calcular delay usando predição de fase centralizada
            float predicted_period = hp_state_predict_next_period(0);
            float base_delay = delta_deg * us_per_deg;
            
            // Aplicar compensação de latência física centralizada
            float compensated_delay = base_delay;
            float latency = hp_state_get_latency(battery_voltage, (float)sensors.clt_c);
            compensated_delay += latency;
            
            // Calcular target absoluto
            uint32_t delay_us = (uint32_t)(compensated_delay + 0.5f);
            uint32_t target_us = current_counter + delay_us;

            // S1-04: Dwell-timing conflict guard.
            // If the spark target is so close that even the minimum dwell (IGN_DWELL_MS_MIN)
            // cannot be completed, skip this cylinder rather than firing with a dangerously
            // short coil charge. This protects the coil from thermal damage.
            // DWELL_GUARD_US provides a small margin for scheduling overhead (~200 µs).
            static const uint32_t DWELL_GUARD_US = 200U;
            static const uint32_t MIN_DWELL_US = (uint32_t)(IGN_DWELL_MS_MIN * 1000.0f);
            if (delay_us < (MIN_DWELL_US + DWELL_GUARD_US)) {
                LOG_IGNITION_W("Cyl %u: spark in %lu µs < min dwell %lu µs — skipping",
                               (unsigned)cylinder, (unsigned long)delay_us,
                               (unsigned long)(MIN_DWELL_US + DWELL_GUARD_US));
                continue;
            }

            // Agendar com compare absoluto HP
            mcpwm_ignition_hp_schedule_one_shot_absolute(
                cylinder, target_us, rpm, battery_voltage, current_counter);
        }
        
        // Atualizar preditor de fase centralizado
        float measured_period = sync_data.tooth_period;
        hp_state_update_phase_predictor(measured_period, hp_get_cycle_count());
        
        LOG_IGNITION_D("HP Scheduled ignition (sync): %u deg10, %u RPM", advance_deg10, rpm);
        return;
    }
    
    // Fallback: usar predição de fase centralizada
    float predicted_period = hp_state_predict_next_period(0);
    uint32_t period_us = (uint32_t)(predicted_period + 0.5f);
    
    for (uint8_t cylinder = 1; cylinder <= 4; cylinder++) {
        float spark_deg = wrap_angle_720(g_cyl_tdc_deg[cylinder - 1] - advance_degrees);
        float delay_deg = (spark_deg >= 0.0f) ? spark_deg : spark_deg + 720.0f;
        
        float us_per_rev = period_us * 2;  // Uma revolução = 2 períodos de 360°
        float delay_us_f = (delay_deg / 720.0f) * us_per_rev;
        uint32_t delay_us = (uint32_t)(delay_us_f + 0.5f);
        
        mcpwm_ignition_hp_schedule_one_shot_absolute(
            cylinder, delay_us, rpm, battery_voltage, 0);
    }
    
    LOG_IGNITION_D("HP Applied ignition timing (fallback): %u deg10, %u RPM", advance_deg10, rpm);
}

void ignition_get_jitter_stats(float *avg_us, float *max_us, float *min_us) {
    hp_state_get_jitter_stats(avg_us, max_us, min_us);
}

void ignition_update_phase(float measured_period_us) {
    hp_state_update_phase_predictor(measured_period_us, hp_get_cycle_count());
}
