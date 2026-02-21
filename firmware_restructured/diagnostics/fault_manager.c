#include "safety_monitor.h"
#include "engine_config.h"
#include "logger.h"
#include "hal/hal_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/soc.h"

// Forward declaration of static functions
static bool should_apply_accel_enrichment(int current_map, int previous_map);

// Limp mode recovery configuration
#define LIMP_MIN_DURATION_MS 5000     // Minimum time in limp mode before recovery
#define LIMP_RECOVERY_HYSTERESIS_MS 2000  // Time conditions must be safe before recovery

// Thread safety spinlock
static portMUX_TYPE g_safety_spinlock = portMUX_INITIALIZER_UNLOCKED;

static limp_mode_t g_limp_mode = {
    .active = false,
    .rpm_limit = 3000,
    .ve_value = 800,
    .timing_value = 100,
    .lambda_target = 1000,
    .activation_time = 0,
};

static uint32_t g_limp_recovery_start_ms = 0;  // When conditions became safe
static bool g_limp_conditions_safe = false;

// Persistent knock protection state — shared across cycles
// timing_retard stored in units of 0.1° (multiply by 10 to convert from degrees)
static knock_protection_t g_knock_prot = {
    .knock_count    = 0,
    .timing_retard  = 0,
    .knock_detected = false,
};

static watchdog_config_t g_watchdog = {
    .enabled = false,
    .timeout_ms = 1000,
    .last_feed_time = 0,
};

static esp_task_wdt_user_handle_t g_wdt_user = NULL;

bool safety_monitor_init(void) {
    g_limp_mode.active = false;
    g_limp_mode.activation_time = 0;
    g_watchdog.enabled = false;
    g_watchdog.last_feed_time = 0;
    return true;
}

sensor_status_t safety_validate_sensor(int adc_value, int min_expected, int max_expected) {
    if (adc_value < min_expected) {
        return SENSOR_SHORT_GND;
    }
    if (adc_value > max_expected) {
        return SENSOR_SHORT_VCC;
    }
    return SENSOR_OK;
}

bool safety_check_over_rev(uint16_t rpm) {
    if (rpm >= FUEL_CUTOFF_RPM || rpm > MAX_RPM) {
        safety_log_event("OVER_REV", rpm);
        safety_activate_limp_mode();
        return true;
    }
    return false;
}

bool safety_check_overheat(int16_t temp) {
    // C7 fix: use CLT_OVERHEAT_C (105°C) as the overheat threshold, not
    // CLT_SENSOR_MAX (120°C) which is the ADC range limit, not an overheat point.
    if (temp > (int16_t)CLT_OVERHEAT_C) {
        safety_log_event("OVERHEAT", (uint32_t)temp);
        safety_activate_limp_mode();
        return true;
    }
    return false;
}

bool safety_check_battery_voltage(uint16_t voltage) {
    float v = voltage / 10.0f;
    if (v < VBAT_SENSOR_MIN || v > VBAT_SENSOR_MAX) {
        safety_log_event("VBAT", voltage);
        safety_activate_limp_mode();
        return true;
    }
    return false;
}

void safety_activate_limp_mode(void) {
    portENTER_CRITICAL(&g_safety_spinlock);
    if (!g_limp_mode.active) {
        g_limp_mode.active = true;
        g_limp_mode.activation_time = (uint32_t)(HAL_Time_us() / 1000);
        portEXIT_CRITICAL(&g_safety_spinlock);
        LOG_SAFETY_W("Limp mode activated");
    } else {
        portEXIT_CRITICAL(&g_safety_spinlock);
    }
}

void safety_deactivate_limp_mode(void) {
    portENTER_CRITICAL(&g_safety_spinlock);
    if (!g_limp_mode.active) {
        portEXIT_CRITICAL(&g_safety_spinlock);
        return;  // Already inactive
    }
    
    uint32_t now_ms = (uint32_t)(HAL_Time_us() / 1000);
    uint32_t time_in_limp = now_ms - g_limp_mode.activation_time;
    
    // Check minimum duration
    if (time_in_limp < LIMP_MIN_DURATION_MS) {
        portEXIT_CRITICAL(&g_safety_spinlock);
        return;  // Must stay in limp mode for minimum duration
    }
    
    // Check if conditions have been safe for hysteresis period
    if (!g_limp_conditions_safe) {
        g_limp_recovery_start_ms = now_ms;
        g_limp_conditions_safe = true;
        portEXIT_CRITICAL(&g_safety_spinlock);
        LOG_SAFETY_I("Limp mode recovery conditions met, monitoring...");
        return;
    }
    
    uint32_t safe_duration = now_ms - g_limp_recovery_start_ms;
    if (safe_duration < LIMP_RECOVERY_HYSTERESIS_MS) {
        portEXIT_CRITICAL(&g_safety_spinlock);
        return;  // Conditions must be safe for hysteresis period
    }
    
    // All checks passed - safe to recover
    g_limp_mode.active = false;
    g_limp_mode.activation_time = 0;
    g_limp_conditions_safe = false;
    g_limp_recovery_start_ms = 0;
    portEXIT_CRITICAL(&g_safety_spinlock);
    LOG_SAFETY_I("Limp mode deactivated - auto recovery");
}

void safety_mark_conditions_safe(bool safe) {
    portENTER_CRITICAL(&g_safety_spinlock);
    if (!safe) {
        g_limp_conditions_safe = false;
        g_limp_recovery_start_ms = 0;
    }
    portEXIT_CRITICAL(&g_safety_spinlock);
}

bool safety_is_limp_mode_active(void) {
    portENTER_CRITICAL(&g_safety_spinlock);
    bool active = g_limp_mode.active;
    portEXIT_CRITICAL(&g_safety_spinlock);
    return active;
}

limp_mode_t safety_get_limp_mode_status(void) {
    portENTER_CRITICAL(&g_safety_spinlock);
    limp_mode_t status = g_limp_mode;
    portEXIT_CRITICAL(&g_safety_spinlock);
    return status;
}

bool safety_watchdog_init(uint32_t timeout_ms) {
    esp_task_wdt_config_t cfg = {
        .timeout_ms = timeout_ms,
        .idle_core_mask = 0,
        .trigger_panic = false,
    };

    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return false;
    }

    err = esp_task_wdt_add_user("engine_control", &g_wdt_user);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return false;
    }

    g_watchdog.enabled = true;
    g_watchdog.timeout_ms = timeout_ms;
    g_watchdog.last_feed_time = (uint32_t)(HAL_Time_us() / 1000);
    return true;
}

bool safety_watchdog_feed(void) {
    if (!g_watchdog.enabled || g_wdt_user == NULL) {
        return false;
    }

    if (esp_task_wdt_reset_user(g_wdt_user) != ESP_OK) {
        return false;
    }

    g_watchdog.last_feed_time = (uint32_t)(HAL_Time_us() / 1000);
    return true;
}

bool safety_watchdog_check(void) {
    if (!g_watchdog.enabled) {
        return true;
    }

    uint32_t now = (uint32_t)(HAL_Time_us() / 1000);
    return (now - g_watchdog.last_feed_time) <= g_watchdog.timeout_ms;
}

void safety_handle_knock(knock_protection_t *knock_prot) {
    if (!knock_prot) {
        return;
    }

    // C8 fix: steps now match engine_config.h constants:
    //   KNOCK_RETARD_STEP_DEG   = 1.0° → 10 in 0.1° units   ✓ (unchanged)
    //   KNOCK_RECOVER_STEP_DEG  = 0.2° → 2 in 0.1° units    (was 5 — too fast)
    //   KNOCK_RETARD_MAX_DEG    = 10°  → 100 in 0.1° units   ✓ (unchanged)
    if (knock_prot->knock_detected) {
        knock_prot->knock_count++;
        // Retard up to KNOCK_RETARD_MAX_DEG (100 × 0.1°)
        if (knock_prot->timing_retard < 100) {
            knock_prot->timing_retard += 10;  // +1.0°
        }
    } else {
        // Recover at KNOCK_RECOVER_STEP_DEG = 0.2° per event
        if (knock_prot->timing_retard >= 2) {
            knock_prot->timing_retard -= 2;   // -0.2°  (was -5 = -0.5°, too fast)
        } else {
            knock_prot->timing_retard = 0;
        }
        if (knock_prot->knock_count > 0) {
            knock_prot->knock_count--;
        }
    }
}

void safety_knock_event(bool knock_detected) {
    portENTER_CRITICAL(&g_safety_spinlock);
    g_knock_prot.knock_detected = knock_detected;
    portEXIT_CRITICAL(&g_safety_spinlock);
    safety_handle_knock(&g_knock_prot);
}

uint16_t safety_get_knock_retard_deg10(void) {
    portENTER_CRITICAL(&g_safety_spinlock);
    uint16_t retard = g_knock_prot.timing_retard;
    portEXIT_CRITICAL(&g_safety_spinlock);
    return retard;
}

void safety_log_event(const char* event_type, uint32_t value) {
    logger_log_safety_event(event_type, value);
}

sensor_status_t safety_validate_map_sensor(int map_value) {
    return safety_validate_sensor(map_value, (int)MAP_SENSOR_MIN, (int)MAP_SENSOR_MAX);
}

bool safety_check_acceleration_enrichment(int current_map, int previous_map) {
    return should_apply_accel_enrichment(current_map, previous_map);
}

uint16_t safety_get_accel_enrichment_factor(void) {
    return TPS_DOT_ENRICH_MAX;
}

uint32_t safety_get_accel_enrichment_duration(void) {
    return 200;
}

static bool validate_configuration(void) {
    return true;
}

static void reset_failure_counters(void) {
}

static sensor_status_t validate_map_pressure(int map_value) {
    return safety_validate_map_sensor(map_value);
}

static bool should_apply_accel_enrichment(int current_map, int previous_map) {
    int delta = current_map - previous_map;
    return delta > TPS_DOT_THRESHOLD;
}
