/**
 * @file fuel_injection.c
 * @brief Sistema de injeção de combustível de alta precisão
 * 
 * Integração com drivers MCPWM HP:
 * - Timer contínuo com compare absoluto
 * - Compensação de latência de injetor
 */

#include "fuel_injection.h"
#include "mcpwm_injection.h"
#include "mcpwm_injection_hp.h"
#include "sync.h"
#include "hp_state.h"
#include "math_utils.h"

static fuel_injection_config_t g_fuel_cfg = {
    .cyl_tdc_deg = {0.0f, 180.0f, 360.0f, 540.0f},
};

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

void fuel_injection_init(const fuel_injection_config_t *config) {
    if (config) {
        g_fuel_cfg = *config;
    }
    // Drivers HP já inicializados em ignition_init()
}

bool fuel_injection_schedule_eoi_ex(uint8_t cylinder_id,
                                      float target_eoi_deg,
                                      uint32_t pulsewidth_us,
                                      const sync_data_t *sync,
                                      fuel_injection_schedule_info_t *info) {
    if (!sync || cylinder_id < 1 || cylinder_id > 4) {
        return false;
    }

    sync_config_t sync_cfg = {0};
    if (sync_get_config(&sync_cfg) != ESP_OK || sync_cfg.tooth_count == 0) {
        return false;
    }
    float us_per_deg = sync_us_per_degree(sync, &sync_cfg);
    if (us_per_deg <= 0.0f) {
        return false;
    }

    float current_angle = compute_current_angle_deg(sync, sync_cfg.tooth_count);
    float eoi_deg = wrap_angle_720(target_eoi_deg + g_fuel_cfg.cyl_tdc_deg[cylinder_id - 1]);
    
    // Calcular pulso width compensado
    float compensated_pw = (float)pulsewidth_us;
    float battery_voltage = 13.5f;
    float temperature = 25.0f;
    
    // Aplicar compensação de latência do injetor usando estado centralizado
    mcpwm_injection_hp_apply_latency_compensation(&compensated_pw, battery_voltage, temperature);
    
    float pw_deg = compensated_pw / us_per_deg;
    float soi_deg = wrap_angle_720(eoi_deg - pw_deg);

    float delta_deg = soi_deg - current_angle;
    if (delta_deg < 0.0f) {
        delta_deg += 720.0f;
    }

    uint32_t delay_us = (uint32_t)((delta_deg * us_per_deg) + 0.5f);
    
    // Obter contador atual do timer MCPWM (valor real, não sintético)
    uint32_t current_counter = mcpwm_injection_hp_get_counter((uint8_t)(cylinder_id - 1));
    
    if (info) {
        info->eoi_deg = eoi_deg;
        info->soi_deg = soi_deg;
        info->delay_us = delay_us;
    }
    
    // Usar scheduling absoluto HP
    return mcpwm_injection_hp_schedule_one_shot_absolute(
        (uint8_t)(cylinder_id - 1), delay_us, (uint32_t)compensated_pw, current_counter);
}

bool fuel_injection_schedule_eoi(uint8_t cylinder_id,
                                   float target_eoi_deg,
                                   uint32_t pulsewidth_us,
                                   const sync_data_t *sync) {
    return fuel_injection_schedule_eoi_ex(cylinder_id, target_eoi_deg, pulsewidth_us, sync, NULL);
}

bool fuel_injection_schedule_sequential(uint32_t pulsewidth_us[4],
                                          float target_eoi_deg[4],
                                          const sync_data_t *sync) {
    if (!sync || !pulsewidth_us || !target_eoi_deg) {
        return false;
    }
    
    sync_config_t sync_cfg = {0};
    if (sync_get_config(&sync_cfg) != ESP_OK || sync_cfg.tooth_count == 0) {
        return false;
    }
    
    uint32_t offsets[4];
    // Obter contador atual do timer MCPWM (valor real, não sintético)
    uint32_t current_counter = mcpwm_injection_hp_get_counter(0);
    
    for (int i = 0; i < 4; i++) {
        fuel_injection_schedule_info_t info;
        if (!fuel_injection_schedule_eoi_ex(i + 1, target_eoi_deg[i], pulsewidth_us[i], sync, &info)) {
            return false;
        }
        offsets[i] = info.delay_us;
    }
    
    return mcpwm_injection_hp_schedule_sequential_absolute(0, pulsewidth_us[0], offsets, current_counter);
}
