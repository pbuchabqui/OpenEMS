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
#include "mcpwm_ignition_hp.h"
#include "mcpwm_injection_hp.h"
#include "sensor_processing.h"
#include "utils/sync.h"
#include "hp_state.h"
#include "math_utils.h"
#include "scheduler/event_scheduler.h"
#include "config/engine_config.h"

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
    float current_angle = (sync->revolution_index * 360.0f)
        + (sync->tooth_index * degrees_per_tooth)
        + TRIGGER_TDC_OFFSET_DEG;
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

bool ignition_schedule_events(uint16_t advance_deg10, uint16_t rpm, float vbat_v) {
    if (rpm == 0) {
        return false;
    }
    float advance_degrees = advance_deg10 / 10.0f;
    advance_degrees = clamp_float(advance_degrees, IGN_ADVANCE_MIN_DEG, IGN_ADVANCE_MAX_DEG);
    float battery_voltage = (vbat_v > 0.0f) ? vbat_v : 13.5f;

    sensor_data_t sensors = {0};
    if (sensor_get_data_fast(&sensors) == ESP_OK) {
        battery_voltage = apply_temp_dwell_bias(battery_voltage, sensors.clt_c);
    }
    battery_voltage = clamp_float(battery_voltage, 8.0f, 16.5f);

    bool ok = true;
    for (uint8_t cylinder = 1; cylinder <= 4; cylinder++) {
        float spark_deg = wrap_angle_720(g_cyl_tdc_deg[cylinder - 1] - advance_degrees);
        bool evt_ok = evt_schedule(EVT_IGNITION_DWELL,
                                   (uint8_t)(cylinder - 1),
                                   spark_deg,
                                   0,
                                   rpm,
                                   battery_voltage);
        ok = ok && evt_ok;
    }
    return ok;
}

void ignition_get_jitter_stats(float *avg_us, float *max_us, float *min_us) {
    hp_state_get_jitter_stats(avg_us, max_us, min_us);
}

void ignition_update_phase(float measured_period_us) {
    hp_state_update_phase_predictor(measured_period_us, hp_get_cycle_count());
}
