#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

#include "esp_err.h"
#include "sensors/sensor_processing.h"

// Engine parameters
typedef struct {
    uint32_t rpm;
    uint32_t load;
    uint16_t advance_deg10;
    uint16_t fuel_enrichment;
    bool is_limp_mode;
} engine_params_t;

typedef struct {
    uint32_t planner_last_us;
    uint32_t planner_max_us;
    uint32_t planner_p95_us;
    uint32_t planner_p99_us;
    uint32_t executor_last_us;
    uint32_t executor_max_us;
    uint32_t executor_p95_us;
    uint32_t executor_p99_us;
    uint32_t planner_deadline_miss;
    uint32_t executor_deadline_miss;
    uint32_t queue_overruns;
    uint32_t queue_depth_peak;
    uint32_t sample_count;
} engine_perf_stats_t;

typedef struct {
    uint16_t rpm;
    uint16_t load;
    float boundary;
    float normal_used;
    float eoit_target_deg;
    float eoit_fallback_target_deg;
    uint32_t pulsewidth_us;
    float soi_deg[4];
    uint32_t delay_us[4];
    bool sync_acquired;
    bool map_mode_enabled;
    uint32_t updated_at_us;
} engine_injection_diag_t;

// Function prototypes
esp_err_t engine_control_init(void);
esp_err_t engine_control_start(void);
esp_err_t engine_control_stop(void);
esp_err_t engine_control_deinit(void);
esp_err_t engine_control_get_engine_parameters(engine_params_t *params);
esp_err_t engine_control_set_eoi_config(float eoi_deg, float eoi_fallback_deg);
esp_err_t engine_control_get_eoi_config(float *eoi_deg, float *eoi_fallback_deg);
esp_err_t engine_control_set_eoit_calibration(float boundary, float normal, float fallback_normal);
esp_err_t engine_control_get_eoit_calibration(float *boundary, float *normal, float *fallback_normal);
esp_err_t engine_control_set_eoit_map_enabled(bool enabled);
bool engine_control_get_eoit_map_enabled(void);
esp_err_t engine_control_set_eoit_map_cell(uint8_t rpm_idx, uint8_t load_idx, float normal);
esp_err_t engine_control_get_eoit_map_cell(uint8_t rpm_idx, uint8_t load_idx, float *normal);
esp_err_t engine_control_get_injection_diag(engine_injection_diag_t *diag);
bool engine_control_is_limp_mode(void);
void engine_control_set_closed_loop_enabled(bool enabled);
bool engine_control_get_closed_loop_enabled(void);
esp_err_t engine_control_get_perf_stats(engine_perf_stats_t *stats);

#endif // ENGINE_CONTROL_H
