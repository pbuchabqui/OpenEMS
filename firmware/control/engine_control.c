#include engine_control.h"
#include logger.h"
#include sensor_processing.h"
#include sync.h"
#include fuel_calc.h"
#include table_16x16.h"
#include lambda_pid.h"
#include s3_control_config.h"
#include fuel_injection.h"
#include ignition_timing.h"
#include config_manager.h"
#include map_storage.h"
#include safety_monitor.h"
#include twai_lambda.h"
#include math_utils.h"
#include espnow_link.h"
#include mcpwm_injection_hp.h"
#include mcpwm_ignition_hp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_rom_crc.h"
#include "esp_log.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

// Static variables
static fuel_calc_maps_t g_maps = {0};
static lambda_pid_t g_lambda_pid = {0};
static bool g_engine_math_ready = false;
static float g_target_eoi_deg = 360.0f;
static float g_target_eoi_deg_fallback = 360.0f;
static float g_eoit_boundary = 6.5f;
static float g_eoit_normal = 5.55f;
static float g_eoit_fallback_normal = 5.55f;
static table_16x16_t g_eoit_normal_map = {0};
static bool g_eoit_map_enabled = false;
static TaskHandle_t g_planner_task_handle = NULL;
static TaskHandle_t g_executor_task_handle = NULL;
static TaskHandle_t g_monitor_task_handle = NULL;
static SemaphoreHandle_t g_map_mutex = NULL;
static float g_stft = 0.0f;
static float g_ltft = 0.0f;
static uint16_t g_last_rpm = 0;
static uint16_t g_last_load = 0;
static uint32_t g_stable_start_ms = 0;
static uint32_t g_last_map_save_ms = 0;
static uint32_t g_map_version = 0;
static bool g_map_dirty = false;
static bool g_closed_loop_enabled = true;
static sensor_data_t g_last_sensor_snapshot = {0};
static bool g_last_sensor_valid = false;
static uint32_t g_last_sensor_timestamp_ms = 0;
static volatile uint32_t g_runtime_seq = 0;

#define CLOSED_LOOP_CONFIG_KEY "closed_loop_cfg"
#define CLOSED_LOOP_CONFIG_VERSION 1U

/**
 * @brief Short-Term Fuel Trim limits and configuration
 * 
 * STFT_LIMIT: Maximum adjustment factor (±25% from stoichiometric)
 * LTFT_LIMIT: Maximum long-term adjustment (±20% to prevent runaway)
 * LTFT_ALPHA: Exponential moving average factor for LTFT learning
 *             Lower = slower learning, more stable
 */
#define STFT_LIMIT 0.25f       // ±25% max STFT adjustment
#define LTFT_LIMIT 0.20f       // ±20% max LTFT adjustment
#define LTFT_ALPHA 0.01f       // EMA factor for LTFT (slow learning)

/**
 * @brief LTFT learning stability thresholds
 * 
 * LTFT_STABLE_MS: Time RPM/load must be stable before LTFT learning
 * LTFT_RPM_DELTA_MAX: Maximum RPM change to consider stable
 * LTFT_LOAD_DELTA_MAX: Maximum load change to consider stable
 * LTFT_APPLY_THRESHOLD: Minimum LTFT value before applying to fuel table
 */
#define LTFT_STABLE_MS 500U           // 500ms stability requirement
#define LTFT_RPM_DELTA_MAX 50U        // ±50 RPM stability window
#define LTFT_LOAD_DELTA_MAX 50U       // ±50 load units stability window
#define LTFT_APPLY_THRESHOLD 0.03f    // 3% minimum LTFT to apply

/**
 * @brief Timing and performance configuration
 * 
 * MAP_SAVE_INTERVAL_MS: How often to save fuel maps to NVS
 * PLANNER_DEADLINE_US: Maximum time for planner task execution
 * EXECUTOR_MAX_PLAN_AGE_US: Maximum age of execution plan before discard
 */
#define MAP_SAVE_INTERVAL_MS 5000U    // Save maps every 5 seconds
#define PLANNER_DEADLINE_US 700U      // 700µs planner deadline
#define EXECUTOR_MAX_PLAN_AGE_US 3000U // 3ms max plan age

/**
 * @brief Ring buffer and performance monitoring configuration
 */
#define PLAN_RING_SIZE 16U            // Command ring buffer size
#define PERF_WINDOW 128U              // Performance samples window
#define SENSOR_FALLBACK_TIMEOUT_MS 100U // Sensor data timeout before fallback

#define EOI_CONFIG_KEY "eoi_config"
#define EOI_CONFIG_VERSION 2U
#define EOIT_MAP_CONFIG_KEY "eoit_map_config"
#define EOIT_MAP_CONFIG_VERSION 1U
#define EOIT_DEFAULT_BOUNDARY 6.5f
#define EOIT_DEFAULT_NORMAL 5.55f
#define EOIT_SCALE_DEG 90.0f
#define EOIT_OFFSET_DEG 784.0f
#define EOIT_NORMAL_SCALE 100.0f
#define EOIT_BOUNDARY_MIN 0.0f
#define EOIT_BOUNDARY_MAX 20.0f
#define EOIT_NORMAL_MIN -8.0f
#define EOIT_NORMAL_MAX 16.0f

typedef struct {
    uint32_t version;
    float boundary;
    float normal;
    float fallback_normal;
    uint32_t crc32;
} eoi_config_blob_t;

typedef struct {
    uint32_t version;
    float eoi_deg;
    float eoi_fallback_deg;
    uint32_t crc32;
} eoi_config_blob_v1_t;

typedef struct {
    uint32_t version;
    uint8_t enabled;
    uint8_t reserved[3];
    table_16x16_t normal_map;
    uint32_t crc32;
} eoit_map_config_blob_t;

typedef struct {
    uint32_t version;
    uint8_t enabled;
    uint8_t reserved[3];
    uint32_t crc32;
} closed_loop_config_blob_t;

typedef struct {
    uint16_t rpm;
    uint16_t load;
    uint16_t advance_deg10;
    uint32_t pw_us;
    float eoit_normal_used;
    float eoi_target_deg;
    float eoi_fallback_deg;
    sync_data_t sync_data;
    uint32_t planned_at_us;
} engine_plan_cmd_t;

typedef struct {
    engine_plan_cmd_t items[PLAN_RING_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint32_t overruns;
    portMUX_TYPE mux;
} plan_ring_t;

typedef struct {
    uint32_t planner_samples[PERF_WINDOW];
    uint32_t executor_samples[PERF_WINDOW];
    uint16_t sample_idx;
    uint16_t sample_count;
    uint32_t planner_last_us;
    uint32_t planner_max_us;
    uint32_t executor_last_us;
    uint32_t executor_max_us;
    uint32_t planner_deadline_miss;
    uint32_t executor_deadline_miss;
    uint32_t queue_overruns;
    uint32_t queue_depth_peak;
} perf_stats_t;

typedef struct {
    uint16_t rpm;
    uint16_t load;
    uint16_t advance_deg10;
    uint32_t pulsewidth_us;
    bool valid;
} runtime_engine_state_t;

static plan_ring_t g_plan_ring = {
    .head = 0,
    .tail = 0,
    .overruns = 0,
    .mux = portMUX_INITIALIZER_UNLOCKED,
};
static perf_stats_t g_perf_stats = {0};
static runtime_engine_state_t g_runtime_state = {0};
static portMUX_TYPE g_perf_spinlock = portMUX_INITIALIZER_UNLOCKED;
static engine_injection_diag_t g_injection_diag = {0};
static volatile uint32_t g_injection_diag_seq = 0;
static bool g_engine_initialized = false;

static float compute_current_angle_360(const sync_data_t *sync, uint32_t tooth_count);
static float sync_us_per_degree(const sync_data_t *sync, const sync_config_t *cfg);

static uint32_t eoi_config_crc(const eoi_config_blob_t *cfg) {
    return esp_rom_crc32_le(0, (const uint8_t *)&cfg->boundary, (uint32_t)(sizeof(float) * 3));
}

static float eoit_target_from_calibration(float boundary, float normal) {
    float target = ((boundary + normal) * EOIT_SCALE_DEG) - EOIT_OFFSET_DEG;
    if (!isfinite(target)) {
        return 360.0f;
    }
    return wrap_angle_720(target);
}

static float eoit_normal_from_target(float boundary, float target_deg) {
    float normal = ((target_deg + EOIT_OFFSET_DEG) / EOIT_SCALE_DEG) - boundary;
    if (!isfinite(normal)) {
        return EOIT_DEFAULT_NORMAL;
    }
    return normal;
}

static float clamp_eoit_boundary(float boundary) {
    return clamp_float(boundary, EOIT_BOUNDARY_MIN, EOIT_BOUNDARY_MAX);
}

static float clamp_eoit_normal(float normal) {
    return clamp_float(normal, EOIT_NORMAL_MIN, EOIT_NORMAL_MAX);
}

static uint16_t eoit_normal_to_table(float normal) {
    float scaled = clamp_eoit_normal(normal) * EOIT_NORMAL_SCALE;
    if (scaled < 0.0f) {
        return 0U;
    }
    if (scaled > 65535.0f) {
        return 65535U;
    }
    return (uint16_t)(scaled + 0.5f);
}

static float eoit_normal_from_table(uint16_t raw) {
    return ((float)raw) / EOIT_NORMAL_SCALE;
}

static void eoi_config_apply(const eoi_config_blob_t *cfg) {
    if (!cfg) {
        return;
    }
    g_eoit_boundary = clamp_eoit_boundary(cfg->boundary);
    g_eoit_normal = clamp_eoit_normal(cfg->normal);
    g_eoit_fallback_normal = clamp_eoit_normal(cfg->fallback_normal);
    g_target_eoi_deg = eoit_target_from_calibration(g_eoit_boundary, g_eoit_normal);
    g_target_eoi_deg_fallback = eoit_target_from_calibration(g_eoit_boundary, g_eoit_fallback_normal);
}

static void eoi_config_defaults(eoi_config_blob_t *cfg) {
    if (!cfg) {
        return;
    }
    cfg->version = EOI_CONFIG_VERSION;
    cfg->boundary = EOIT_DEFAULT_BOUNDARY;
    cfg->normal = EOIT_DEFAULT_NORMAL;
    cfg->fallback_normal = EOIT_DEFAULT_NORMAL;
    cfg->crc32 = eoi_config_crc(cfg);
}

static uint32_t eoit_map_config_crc(const eoit_map_config_blob_t *cfg) {
    return esp_rom_crc32_le(0, (const uint8_t *)&cfg->enabled, (uint32_t)(sizeof(cfg->enabled) + sizeof(cfg->reserved) + sizeof(cfg->normal_map)));
}

static void eoit_map_config_defaults(eoit_map_config_blob_t *cfg) {
    if (!cfg) {
        return;
    }
    cfg->version = EOIT_MAP_CONFIG_VERSION;
    cfg->enabled = 0U;
    cfg->reserved[0] = 0U;
    cfg->reserved[1] = 0U;
    cfg->reserved[2] = 0U;
    table_16x16_init(&cfg->normal_map, DEFAULT_RPM_BINS, DEFAULT_LOAD_BINS, eoit_normal_to_table(EOIT_DEFAULT_NORMAL));
    cfg->crc32 = eoit_map_config_crc(cfg);
}

static void eoit_map_config_apply(const eoit_map_config_blob_t *cfg) {
    if (!cfg) {
        return;
    }
    g_eoit_map_enabled = (cfg->enabled != 0U);
    g_eoit_normal_map = cfg->normal_map;
}

static float compute_current_angle_360(const sync_data_t *sync, uint32_t tooth_count) {
    float degrees_per_tooth = 360.0f / (float)(tooth_count + 2);
    float current_angle = sync->tooth_index * degrees_per_tooth;
    return wrap_angle_360(current_angle);
}

static float sync_us_per_degree(const sync_data_t *sync, const sync_config_t *cfg) {
    if (!sync || !cfg || sync->tooth_period == 0U || cfg->tooth_count == 0U) {
        return 0.0f;
    }
    uint32_t total_positions = cfg->tooth_count + 2U;
    return ((float)sync->tooth_period * (float)total_positions) / 360.0f;
}

static uint32_t angle_delta_to_delay_us(float delta_deg, float cycle_deg, float us_per_deg) {
    if (cycle_deg <= 0.0f || us_per_deg <= 0.0f) {
        return 0U;
    }
    while (delta_deg < 0.0f) {
        delta_deg += cycle_deg;
    }
    while (delta_deg >= cycle_deg) {
        delta_deg -= cycle_deg;
    }
    float delay_f = delta_deg * us_per_deg;
    if (delay_f <= 0.0f) {
        return 0U;
    }
    if (delay_f >= 4294967040.0f) {
        return UINT32_MAX;
    }
    return (uint32_t)(delay_f + 0.5f);
}

static uint32_t closed_loop_config_crc(const closed_loop_config_blob_t *cfg) {
    return esp_rom_crc32_le(0, (const uint8_t *)&cfg->enabled, (uint32_t)sizeof(cfg->enabled) + sizeof(cfg->reserved));
}

static void closed_loop_config_defaults(closed_loop_config_blob_t *cfg) {
    if (!cfg) {
        return;
    }
    cfg->version = CLOSED_LOOP_CONFIG_VERSION;
    cfg->enabled = 1U;
    cfg->reserved[0] = 0;
    cfg->reserved[1] = 0;
    cfg->reserved[2] = 0;
    cfg->crc32 = closed_loop_config_crc(cfg);
}

static uint8_t find_bin_index(const uint16_t *bins, uint16_t value) {
    for (uint8_t i = 0; i < 15; i++) {
        if (value < bins[i + 1]) {
            return i;
        }
    }
    return 14;
}

static void apply_ltft_to_fuel_table(uint16_t rpm, uint16_t load) {
    if (g_map_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(g_map_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    table_16x16_t *table = &g_maps.fuel_table;
    uint8_t x = find_bin_index(table->rpm_bins, rpm);
    uint8_t y = find_bin_index(table->load_bins, load);

    float current = (float)table->values[y][x];
    float updated = current * (1.0f + g_ltft);
    if (updated < 0.0f) updated = 0.0f;
    if (updated > 65535.0f) updated = 65535.0f;
    table->values[y][x] = (uint16_t)(updated + 0.5f);
    table->checksum = table_16x16_checksum(table);
    fuel_calc_reset_interpolation_cache();
    g_map_dirty = true;
    g_map_version++;
    xSemaphoreGive(g_map_mutex);
}

static void maybe_persist_maps(uint32_t now_ms) {
    if (g_map_mutex == NULL) {
        return;
    }

    fuel_calc_maps_t maps_snapshot = {0};
    uint32_t version_snapshot = 0;
    bool should_save = false;

    if (xSemaphoreTake(g_map_mutex, portMAX_DELAY) == pdTRUE) {
        if (g_map_dirty && (now_ms - g_last_map_save_ms) >= MAP_SAVE_INTERVAL_MS) {
            maps_snapshot = g_maps;
            version_snapshot = g_map_version;
            should_save = true;
        }
        xSemaphoreGive(g_map_mutex);
    }

    if (!should_save) {
        return;
    }

    if (map_storage_save(&maps_snapshot) == ESP_OK) {
        if (xSemaphoreTake(g_map_mutex, portMAX_DELAY) == pdTRUE) {
            if (g_map_version == version_snapshot) {
                g_map_dirty = false;
            }
            g_last_map_save_ms = now_ms;
            xSemaphoreGive(g_map_mutex);
        }
    }
}

static bool ltft_can_update(uint16_t rpm, uint16_t load, uint32_t now_ms) {
    uint16_t drpm = (rpm > g_last_rpm) ? (rpm - g_last_rpm) : (g_last_rpm - rpm);
    uint16_t dload = (load > g_last_load) ? (load - g_last_load) : (g_last_load - load);

    g_last_rpm = rpm;
    g_last_load = load;

    if (drpm <= LTFT_RPM_DELTA_MAX && dload <= LTFT_LOAD_DELTA_MAX) {
        if (g_stable_start_ms == 0) {
            g_stable_start_ms = now_ms;
        }
        return (now_ms - g_stable_start_ms) >= LTFT_STABLE_MS;
    }

    g_stable_start_ms = 0;
    return false;
}

static uint32_t perf_percentile(const uint32_t *arr, uint16_t n, uint8_t pct) {
    if (!arr || n == 0) {
        return 0;
    }

    uint32_t copy[PERF_WINDOW] = {0};
    if (n > PERF_WINDOW) {
        n = PERF_WINDOW;
    }
    memcpy(copy, arr, sizeof(uint32_t) * n);

    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = i + 1; j < n; j++) {
            if (copy[j] < copy[i]) {
                uint32_t t = copy[i];
                copy[i] = copy[j];
                copy[j] = t;
            }
        }
    }

    uint16_t idx = (uint16_t)(((uint32_t)(n - 1) * pct) / 100U);
    return copy[idx];
}

static uint8_t plan_ring_depth(void) {
    uint8_t head = g_plan_ring.head;
    uint8_t tail = g_plan_ring.tail;
    if (head >= tail) {
        return (uint8_t)(head - tail);
    }
    return (uint8_t)(PLAN_RING_SIZE - tail + head);
}

static void plan_ring_push(const engine_plan_cmd_t *cmd) {
    if (!cmd) {
        return;
    }

    portENTER_CRITICAL(&g_plan_ring.mux);
    uint8_t head = g_plan_ring.head;
    uint8_t next = (uint8_t)((head + 1U) % PLAN_RING_SIZE);
    bool overrun = false;
    if (next == g_plan_ring.tail) {
        g_plan_ring.tail = (uint8_t)((g_plan_ring.tail + 1U) % PLAN_RING_SIZE);
        g_plan_ring.overruns++;
        overrun = true;
    }
    g_plan_ring.items[head] = *cmd;
    g_plan_ring.head = next;
    uint8_t depth = plan_ring_depth();
    portEXIT_CRITICAL(&g_plan_ring.mux);

    portENTER_CRITICAL(&g_perf_spinlock);
    if (overrun) {
        g_perf_stats.queue_overruns = g_plan_ring.overruns;
    }
    if (depth > g_perf_stats.queue_depth_peak) {
        g_perf_stats.queue_depth_peak = depth;
    }
    portEXIT_CRITICAL(&g_perf_spinlock);
}

static bool plan_ring_pop_latest(engine_plan_cmd_t *cmd) {
    if (!cmd) {
        return false;
    }

    bool ok = false;
    portENTER_CRITICAL(&g_plan_ring.mux);
    if (g_plan_ring.tail != g_plan_ring.head) {
        uint8_t latest_idx = (g_plan_ring.head == 0U) ? (PLAN_RING_SIZE - 1U) : (uint8_t)(g_plan_ring.head - 1U);
        *cmd = g_plan_ring.items[latest_idx];
        g_plan_ring.tail = g_plan_ring.head;
        ok = true;
    }
    portEXIT_CRITICAL(&g_plan_ring.mux);
    return ok;
}

static void perf_record_planner(uint32_t elapsed_us) {
    portENTER_CRITICAL(&g_perf_spinlock);
    g_perf_stats.planner_last_us = elapsed_us;
    if (elapsed_us > g_perf_stats.planner_max_us) {
        g_perf_stats.planner_max_us = elapsed_us;
    }
    if (elapsed_us > PLANNER_DEADLINE_US) {
        g_perf_stats.planner_deadline_miss++;
    }

    uint16_t idx = g_perf_stats.sample_idx % PERF_WINDOW;
    g_perf_stats.planner_samples[idx] = elapsed_us;
    portEXIT_CRITICAL(&g_perf_spinlock);
}

static void perf_record_executor(uint32_t elapsed_us, uint32_t queue_age_us) {
    portENTER_CRITICAL(&g_perf_spinlock);
    g_perf_stats.executor_last_us = elapsed_us;
    if (elapsed_us > g_perf_stats.executor_max_us) {
        g_perf_stats.executor_max_us = elapsed_us;
    }
    if (queue_age_us > PLANNER_DEADLINE_US) {
        g_perf_stats.executor_deadline_miss++;
    }

    uint16_t idx = g_perf_stats.sample_idx % PERF_WINDOW;
    g_perf_stats.executor_samples[idx] = elapsed_us;
    g_perf_stats.sample_idx = (uint16_t)((g_perf_stats.sample_idx + 1U) % PERF_WINDOW);
    if (g_perf_stats.sample_count < PERF_WINDOW) {
        g_perf_stats.sample_count++;
    }
    portEXIT_CRITICAL(&g_perf_spinlock);
}

static void runtime_state_publish(const engine_plan_cmd_t *cmd) {
    if (!cmd) {
        return;
    }
    __atomic_fetch_add(&g_runtime_seq, 1U, __ATOMIC_RELEASE);
    g_runtime_state.rpm = cmd->rpm;
    g_runtime_state.load = cmd->load;
    g_runtime_state.advance_deg10 = cmd->advance_deg10;
    g_runtime_state.pulsewidth_us = cmd->pw_us;
    g_runtime_state.valid = true;
    __atomic_fetch_add(&g_runtime_seq, 1U, __ATOMIC_RELEASE);
}

static bool runtime_state_read(runtime_engine_state_t *out) {
    if (!out) {
        return false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        uint32_t seq1 = __atomic_load_n(&g_runtime_seq, __ATOMIC_ACQUIRE);
        if (seq1 & 1U) {
            continue;
        }
        *out = g_runtime_state;
        uint32_t seq2 = __atomic_load_n(&g_runtime_seq, __ATOMIC_ACQUIRE);
        if (seq1 == seq2 && out->valid) {
            return true;
        }
    }
    return false;
}

static void injection_diag_publish(const engine_injection_diag_t *diag) {
    if (!diag) {
        return;
    }
    __atomic_fetch_add(&g_injection_diag_seq, 1U, __ATOMIC_RELEASE);
    g_injection_diag = *diag;
    __atomic_fetch_add(&g_injection_diag_seq, 1U, __ATOMIC_RELEASE);
}

static bool injection_diag_read(engine_injection_diag_t *out) {
    if (!out) {
        return false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        uint32_t seq1 = __atomic_load_n(&g_injection_diag_seq, __ATOMIC_ACQUIRE);
        if (seq1 & 1U) {
            continue;
        }
        *out = g_injection_diag;
        uint32_t seq2 = __atomic_load_n(&g_injection_diag_seq, __ATOMIC_ACQUIRE);
        if (seq1 == seq2 && out->updated_at_us != 0U) {
            return true;
        }
    }
    return false;
}

IRAM_ATTR static void engine_sync_tooth_callback(void *ctx) {
    (void)ctx;
    if (g_planner_task_handle == NULL) {
        return;
    }
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_planner_task_handle, &hp_woken);
    if (hp_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void schedule_semi_seq_injection(uint16_t rpm,
                                        uint16_t load,
                                        uint32_t pw_us,
                                        const sync_data_t *sync,
                                        float eoi_base_deg) {
    (void)rpm;
    (void)load;
    sync_config_t sync_cfg = {0};
    if (sync_get_config(&sync_cfg) != ESP_OK || sync_cfg.tooth_count == 0) {
        return;
    }

    float current_angle = compute_current_angle_360(sync, sync_cfg.tooth_count);
    float us_per_deg = sync_us_per_degree(sync, &sync_cfg);
    if (us_per_deg <= 0.0f) {
        return;
    }

    float pw_deg = pw_us / us_per_deg;
    // Pair 1: cylinders 1 & 4 at 0°
    float eoi0 = wrap_angle_360(eoi_base_deg);
    float soi0 = wrap_angle_360(eoi0 - pw_deg);
    float delta0 = soi0 - current_angle;
    uint32_t delay0 = angle_delta_to_delay_us(delta0, 360.0f, us_per_deg);
    uint32_t counter = mcpwm_injection_hp_get_counter(0);
    mcpwm_injection_hp_schedule_one_shot_absolute(0, delay0, pw_us, counter);
    mcpwm_injection_hp_schedule_one_shot_absolute(3, delay0, pw_us, counter);

    // Pair 2: cylinders 2 & 3 at 180°
    float eoi180 = wrap_angle_360(eoi_base_deg + 180.0f);
    float soi180 = wrap_angle_360(eoi180 - pw_deg);
    float delta180 = soi180 - current_angle;
    uint32_t delay180 = angle_delta_to_delay_us(delta180, 360.0f, us_per_deg);
    mcpwm_injection_hp_schedule_one_shot_absolute(1, delay180, pw_us, counter);
    mcpwm_injection_hp_schedule_one_shot_absolute(2, delay180, pw_us, counter);
}

static void schedule_wasted_spark(uint16_t advance_deg10, uint16_t rpm, const sync_data_t *sync) {
    sync_config_t sync_cfg = {0};
    if (sync_get_config(&sync_cfg) != ESP_OK || sync_cfg.tooth_count == 0) {
        return;
    }

    float current_angle = compute_current_angle_360(sync, sync_cfg.tooth_count);
    float us_per_deg = sync_us_per_degree(sync, &sync_cfg);
    if (us_per_deg <= 0.0f) {
        return;
    }

    float advance_deg = advance_deg10 / 10.0f;
    float spark0 = wrap_angle_360(0.0f - advance_deg);
    float delta0 = spark0 - current_angle;
    uint32_t delay0 = angle_delta_to_delay_us(delta0, 360.0f, us_per_deg);
    uint32_t counter = mcpwm_ignition_hp_get_counter(0);
    mcpwm_ignition_hp_schedule_one_shot_absolute(1, delay0, rpm, 13.5f, counter);
    mcpwm_ignition_hp_schedule_one_shot_absolute(4, delay0, rpm, 13.5f, counter);

    float spark180 = wrap_angle_360(180.0f - advance_deg);
    float delta180 = spark180 - current_angle;
    uint32_t delay180 = angle_delta_to_delay_us(delta180, 360.0f, us_per_deg);
    mcpwm_ignition_hp_schedule_one_shot_absolute(2, delay180, rpm, 13.5f, counter);
    mcpwm_ignition_hp_schedule_one_shot_absolute(3, delay180, rpm, 13.5f, counter);
}

static esp_err_t engine_control_build_plan(engine_plan_cmd_t *cmd) {
    if (!cmd) {
        return ESP_ERR_INVALID_ARG;
    }

    sync_data_t sync_data = {0};
    if (sync_get_data(&sync_data) != ESP_OK || !sync_data.sync_valid) {
        return ESP_FAIL;
    }

    sensor_data_t sensor_data = {0};
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (sensor_get_data_fast(&sensor_data) != ESP_OK) {
        if (!g_last_sensor_valid) {
            return ESP_FAIL;
        }
        // Check if fallback data is too old
        uint32_t fallback_age = now_ms - g_last_sensor_timestamp_ms;
        if (fallback_age > SENSOR_FALLBACK_TIMEOUT_MS) {
            g_last_sensor_valid = false;
            return ESP_FAIL;
        }
        sensor_data = g_last_sensor_snapshot;
    } else {
        g_last_sensor_snapshot = sensor_data;
        g_last_sensor_valid = true;
        g_last_sensor_timestamp_ms = now_ms;
    }

    uint16_t rpm = (uint16_t)sync_data.rpm;
    uint16_t load = sensor_data.map_kpa10;
    if (safety_check_over_rev(rpm) ||
        safety_check_overheat(sensor_data.clt_c) ||
        safety_check_battery_voltage(sensor_data.vbat_dv)) {
        return ESP_FAIL;
    }

    if (g_map_mutex == NULL || xSemaphoreTake(g_map_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    uint16_t ve_x10 = fuel_calc_lookup_ve(&g_maps, rpm, load);
    uint16_t advance_deg10 = fuel_calc_lookup_ignition(&g_maps, rpm, load);
    uint16_t lambda_target_raw = fuel_calc_lookup_lambda(&g_maps, rpm, load);
    float eoit_normal_used = g_eoit_normal;
    if (g_eoit_map_enabled) {
        uint16_t normal_raw = table_16x16_interpolate(&g_eoit_normal_map, rpm, load);
        eoit_normal_used = clamp_eoit_normal(eoit_normal_from_table(normal_raw));
    }
    xSemaphoreGive(g_map_mutex);

    float lambda_corr = 0.0f;
    if (g_engine_math_ready && g_closed_loop_enabled) {
        float lambda_target = lambda_target_raw / 1000.0f;
        float lambda_measured = 1.0f;
        bool lambda_valid = false;
        uint32_t lambda_age_ms = 0;
        if (twai_lambda_get_latest(&lambda_measured, &lambda_age_ms) && lambda_age_ms < 200) {
            lambda_valid = true;
        } else if (sensor_data.o2_mv > 0) {
            lambda_measured = (sensor_data.o2_mv / 1000.0f) / 0.45f;
            lambda_valid = true;
        }

        lambda_measured = clamp_float(lambda_measured, 0.7f, 1.3f);
        if (lambda_valid) {
            float stft = lambda_pid_update(&g_lambda_pid, lambda_target, lambda_measured, 0.01f);
            g_stft = clamp_float(stft, -STFT_LIMIT, STFT_LIMIT);
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (ltft_can_update(rpm, load, now_ms)) {
                g_ltft += LTFT_ALPHA * (g_stft - g_ltft);
                g_ltft = clamp_float(g_ltft, -LTFT_LIMIT, LTFT_LIMIT);
                if (fabsf(g_ltft) >= LTFT_APPLY_THRESHOLD) {
                    apply_ltft_to_fuel_table(rpm, load);
                    g_ltft = 0.0f;
                }
            }
            lambda_corr = clamp_float(g_stft + g_ltft, -STFT_LIMIT, STFT_LIMIT);
        }
    }

    cmd->rpm = rpm;
    cmd->load = load;
    cmd->advance_deg10 = advance_deg10;
    cmd->pw_us = fuel_calc_pulsewidth_us(&sensor_data, rpm, ve_x10, lambda_corr);
    cmd->eoit_normal_used = eoit_normal_used;
    cmd->eoi_target_deg = eoit_target_from_calibration(g_eoit_boundary, eoit_normal_used);
    cmd->eoi_fallback_deg = eoit_target_from_calibration(g_eoit_boundary, g_eoit_fallback_normal);
    cmd->sync_data = sync_data;
    cmd->planned_at_us = (uint32_t)esp_timer_get_time();
    return ESP_OK;
}

static void engine_control_execute_plan(const engine_plan_cmd_t *cmd) {
    if (!cmd) {
        return;
    }

    sync_data_t exec_sync = cmd->sync_data;
    sync_data_t live_sync = {0};
    if (sync_get_data(&live_sync) == ESP_OK && live_sync.sync_valid) {
        exec_sync = live_sync;
    }
    engine_injection_diag_t diag = {0};
    diag.rpm = cmd->rpm;
    diag.load = cmd->load;
    diag.boundary = g_eoit_boundary;
    diag.normal_used = cmd->eoit_normal_used;
    diag.eoit_target_deg = cmd->eoi_target_deg;
    diag.eoit_fallback_target_deg = cmd->eoi_fallback_deg;
    diag.pulsewidth_us = cmd->pw_us;
    diag.sync_acquired = exec_sync.sync_acquired;
    diag.map_mode_enabled = g_eoit_map_enabled;

    if (exec_sync.sync_acquired) {
        bool scheduling_ok = true;
        for (uint8_t cyl = 1; cyl <= 4; cyl++) {
            fuel_injection_schedule_info_t info = {0};
            bool inj_ok = fuel_injection_schedule_eoi_ex(cyl, cmd->eoi_target_deg, cmd->pw_us, &exec_sync, &info);
            scheduling_ok = scheduling_ok && inj_ok;
            diag.soi_deg[cyl - 1] = info.soi_deg;
            diag.delay_us[cyl - 1] = info.delay_us;
        }
        ignition_apply_timing(cmd->advance_deg10, cmd->rpm);
        if (!scheduling_ok) {
            LOG_SAFETY_E("Injection scheduling failure on synced path");
            safety_activate_limp_mode();
        }
    } else {
        LOG_SAFETY_W("Sync partial: fallback to semi-sequential + wasted spark");
        schedule_semi_seq_injection(cmd->rpm, cmd->load, cmd->pw_us, &exec_sync, cmd->eoi_fallback_deg);
        schedule_wasted_spark(cmd->advance_deg10, cmd->rpm, &exec_sync);

        sync_config_t sync_cfg = {0};
        if (sync_get_config(&sync_cfg) == ESP_OK && sync_cfg.tooth_count > 0) {
            float current_angle = compute_current_angle_360(&exec_sync, sync_cfg.tooth_count);
            float us_per_deg = sync_us_per_degree(&exec_sync, &sync_cfg);
            if (us_per_deg > 0.0f) {
                float pw_deg = cmd->pw_us / us_per_deg;

                float eoi0 = wrap_angle_360(cmd->eoi_fallback_deg);
                float soi0 = wrap_angle_360(eoi0 - pw_deg);
                float d0 = soi0 - current_angle;
                uint32_t delay0 = angle_delta_to_delay_us(d0, 360.0f, us_per_deg);
                diag.soi_deg[0] = soi0;
                diag.soi_deg[3] = soi0;
                diag.delay_us[0] = delay0;
                diag.delay_us[3] = delay0;

                float eoi180 = wrap_angle_360(cmd->eoi_fallback_deg + 180.0f);
                float soi180 = wrap_angle_360(eoi180 - pw_deg);
                float d180 = soi180 - current_angle;
                uint32_t delay180 = angle_delta_to_delay_us(d180, 360.0f, us_per_deg);
                diag.soi_deg[1] = soi180;
                diag.soi_deg[2] = soi180;
                diag.delay_us[1] = delay180;
                diag.delay_us[2] = delay180;
            }
        }
    }
    diag.updated_at_us = (uint32_t)esp_timer_get_time();
    injection_diag_publish(&diag);
    runtime_state_publish(cmd);
    safety_watchdog_feed();
}

static void engine_planner_task(void *arg) {
    (void)arg;
    while (1) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        if (notified == 0) {
            continue;
        }
        uint32_t t0 = (uint32_t)esp_timer_get_time();

        engine_plan_cmd_t cmd = {0};
        if (engine_control_build_plan(&cmd) == ESP_OK) {
            plan_ring_push(&cmd);
            if (g_executor_task_handle != NULL) {
                xTaskNotifyGive(g_executor_task_handle);
            }
        }

        uint32_t elapsed = (uint32_t)esp_timer_get_time() - t0;
        perf_record_planner(elapsed);
    }
}

static void engine_executor_task(void *arg) {
    (void)arg;
    while (1) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        if (notified == 0) {
            continue;
        }
        engine_plan_cmd_t cmd = {0};
        while (plan_ring_pop_latest(&cmd)) {
            uint32_t t0 = (uint32_t)esp_timer_get_time();
            uint32_t queue_age = t0 - cmd.planned_at_us;
            if (queue_age > EXECUTOR_MAX_PLAN_AGE_US) {
                perf_record_executor(0U, queue_age);
                continue;
            }
            engine_control_execute_plan(&cmd);
            uint32_t elapsed = (uint32_t)esp_timer_get_time() - t0;
            perf_record_executor(elapsed, queue_age);
        }
    }
}

static void engine_monitor_task(void *arg) {
    (void)arg;
    uint32_t last_espnow_status_ms = 0;
    uint32_t last_espnow_sensor_ms = 0;
    uint32_t last_espnow_diag_ms = 0;
    
    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        maybe_persist_maps(now_ms);
        
        // Publish ESP-NOW messages if initialized
        if (espnow_link_is_started()) {
            // Send engine status at 10Hz
            if (now_ms - last_espnow_status_ms >= ESPNOW_ENGINE_STATUS_INTERVAL_MS) {
                last_espnow_status_ms = now_ms;
                
                espnow_engine_status_t status = {0};
                engine_params_t params = {0};
                engine_control_get_engine_parameters(&params);
                
                status.rpm = params.rpm;
                status.map_kpa10 = params.load;
                status.advance_deg10 = params.advance_deg10;
                status.limp_mode = params.is_limp_mode ? 1 : 0;
                status.timestamp_ms = now_ms;
                
                // Get additional sensor data
                sensor_data_t sensors;
                if (sensor_get_data(&sensors) == ESP_OK) {
                    status.clt_c10 = (int16_t)(sensors.clt_c * 10.0f);
                    status.iat_c10 = (int16_t)(sensors.iat_c * 10.0f);
                    status.tps_pct10 = (uint16_t)(sensors.tps_percent * 10.0f);
                    status.battery_mv = (uint16_t)(sensors.vbat_dv * 100.0f);
                }
                
                espnow_link_send_engine_status(&status);
            }
            
            // Send sensor data at 10Hz
            if (now_ms - last_espnow_sensor_ms >= ESPNOW_SENSOR_DATA_INTERVAL_MS) {
                last_espnow_sensor_ms = now_ms;
                
                espnow_sensor_data_t sensor_data = {0};
                sensor_data_t sensors;
                
                if (sensor_get_data(&sensors) == ESP_OK) {
                    sensor_data.map_filtered = sensors.map_kpa10;
                    sensor_data.tps_filtered = (uint16_t)(sensors.tps_percent * 10.0f);
                    sensor_data.timestamp_ms = now_ms;
                }
                
                espnow_link_send_sensor_data(&sensor_data);
            }
            
            // Send diagnostic at 1Hz
            if (now_ms - last_espnow_diag_ms >= ESPNOW_DIAG_INTERVAL_MS) {
                last_espnow_diag_ms = now_ms;
                
                espnow_diagnostic_t diag = {0};
                diag.uptime_ms = now_ms;
                diag.free_heap = (uint16_t)(esp_get_free_heap_size() / 1024);  // KB
                
                // Get sync statistics
                sync_data_t sync_data;
                if (sync_get_data(&sync_data) == ESP_OK) {
                    diag.tooth_count = sync_data.tooth_index;
                }
                
                // Get safety status
                limp_mode_t limp = safety_get_limp_mode_status();
                if (limp.active) {
                    diag.error_count = 1;
                    diag.error_bitmap |= ESPNOW_ERR_LIMP_MODE;
                }
                
                espnow_link_send_diagnostic(&diag);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms tick for better timing resolution
    }
}

static void engine_control_init_rollback(bool callback_registered,
                                         bool monitor_task_created,
                                         bool planner_task_created,
                                         bool executor_task_created,
                                         bool twai_started_here,
                                         bool sync_started_here,
                                         bool sync_initialized_here,
                                         bool sensor_started_here,
                                         bool sensor_initialized_here,
                                         bool map_mutex_created_here,
                                         bool config_initialized_here) {
    if (callback_registered) {
        sync_unregister_tooth_callback();
    }
    if (monitor_task_created && g_monitor_task_handle != NULL) {
        vTaskDelete(g_monitor_task_handle);
        g_monitor_task_handle = NULL;
    }
    if (planner_task_created && g_planner_task_handle != NULL) {
        vTaskDelete(g_planner_task_handle);
        g_planner_task_handle = NULL;
    }
    if (executor_task_created && g_executor_task_handle != NULL) {
        vTaskDelete(g_executor_task_handle);
        g_executor_task_handle = NULL;
    }
    if (twai_started_here) {
        twai_lambda_deinit();
    }
    if (sync_started_here) {
        sync_stop();
    }
    if (sync_initialized_here) {
        sync_deinit();
    }
    if (sensor_started_here) {
        sensor_stop();
    }
    if (sensor_initialized_here) {
        sensor_deinit();
    }
    if (map_mutex_created_here && g_map_mutex != NULL) {
        vSemaphoreDelete(g_map_mutex);
        g_map_mutex = NULL;
    }
    if (config_initialized_here) {
        config_manager_deinit();
    }
    g_engine_initialized = false;
}

// Initialize engine control system
esp_err_t engine_control_init(void) {
    ESP_LOGI("ENGINE_CONTROL", "Initializing engine control system");
    if (g_engine_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    bool config_initialized_here = false;
    bool map_mutex_created_here = false;
    bool sensor_initialized_here = false;
    bool sensor_started_here = false;
    bool sync_initialized_here = false;
    bool sync_started_here = false;
    bool twai_started_here = false;
    bool executor_task_created = false;
    bool planner_task_created = false;
    bool monitor_task_created = false;
    bool callback_registered = false;

    esp_err_t err = config_manager_init();
    if (err == ESP_OK) {
        config_initialized_here = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ENGINE_CONTROL", "Failed to initialize config manager");
        return err;
    }

    if (g_map_mutex == NULL) {
        g_map_mutex = xSemaphoreCreateMutex();
        if (g_map_mutex == NULL) {
            ESP_LOGE("ENGINE_CONTROL", "Failed to create map mutex");
            if (config_initialized_here) {
                config_manager_deinit();
            }
            return ESP_FAIL;
        }
        map_mutex_created_here = true;
    }

    if (map_storage_load(&g_maps) != ESP_OK) {
        fuel_calc_init_defaults(&g_maps);
        map_storage_save(&g_maps);
    } else {
        fuel_calc_reset_interpolation_cache();
    }
    g_map_version = 0;
    g_map_dirty = false;
    g_last_map_save_ms = 0;
    g_last_sensor_valid = false;
    memset(&g_runtime_state, 0, sizeof(g_runtime_state));
    __atomic_store_n(&g_runtime_seq, 0U, __ATOMIC_RELEASE);
    lambda_pid_init(&g_lambda_pid, 0.6f, 0.08f, 0.01f, -0.25f, 0.25f);
    g_engine_math_ready = true;

    closed_loop_config_blob_t cl_cfg = {0};
    if (config_manager_load(CLOSED_LOOP_CONFIG_KEY, &cl_cfg, sizeof(cl_cfg)) != ESP_OK ||
        cl_cfg.version != CLOSED_LOOP_CONFIG_VERSION ||
        cl_cfg.crc32 != closed_loop_config_crc(&cl_cfg)) {
        closed_loop_config_defaults(&cl_cfg);
        config_manager_save(CLOSED_LOOP_CONFIG_KEY, &cl_cfg, sizeof(cl_cfg));
    }
    g_closed_loop_enabled = (cl_cfg.enabled != 0);

    // Initialize sensor, sync, TWAI lambda, injection, and ignition subsystems
    err = sensor_init();
    if (err == ESP_OK) {
        sensor_initialized_here = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ENGINE_CONTROL", "Failed to init sensors");
        engine_control_init_rollback(callback_registered,
                                     monitor_task_created,
                                     planner_task_created,
                                     executor_task_created,
                                     twai_started_here,
                                     sync_started_here,
                                     sync_initialized_here,
                                     sensor_started_here,
                                     sensor_initialized_here,
                                     map_mutex_created_here,
                                     config_initialized_here);
        return err;
    }
    err = sensor_start();
    if (err == ESP_OK) {
        sensor_started_here = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ENGINE_CONTROL", "Failed to start sensors");
        engine_control_init_rollback(callback_registered,
                                     monitor_task_created,
                                     planner_task_created,
                                     executor_task_created,
                                     twai_started_here,
                                     sync_started_here,
                                     sync_initialized_here,
                                     sensor_started_here,
                                     sensor_initialized_here,
                                     map_mutex_created_here,
                                     config_initialized_here);
        return err;
    }

    err = sync_init();
    if (err == ESP_OK) {
        sync_initialized_here = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ENGINE_CONTROL", "Failed to init sync");
        engine_control_init_rollback(callback_registered,
                                     monitor_task_created,
                                     planner_task_created,
                                     executor_task_created,
                                     twai_started_here,
                                     sync_started_here,
                                     sync_initialized_here,
                                     sensor_started_here,
                                     sensor_initialized_here,
                                     map_mutex_created_here,
                                     config_initialized_here);
        return err;
    }
    err = sync_start();
    if (err == ESP_OK) {
        sync_started_here = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ENGINE_CONTROL", "Failed to start sync");
        engine_control_init_rollback(callback_registered,
                                     monitor_task_created,
                                     planner_task_created,
                                     executor_task_created,
                                     twai_started_here,
                                     sync_started_here,
                                     sync_initialized_here,
                                     sensor_started_here,
                                     sensor_initialized_here,
                                     map_mutex_created_here,
                                     config_initialized_here);
        return err;
    }

    err = twai_lambda_init();
    if (err == ESP_OK) {
        twai_started_here = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("ENGINE_CONTROL", "Failed to init TWAI lambda");
        engine_control_init_rollback(callback_registered,
                                     monitor_task_created,
                                     planner_task_created,
                                     executor_task_created,
                                     twai_started_here,
                                     sync_started_here,
                                     sync_initialized_here,
                                     sensor_started_here,
                                     sensor_initialized_here,
                                     map_mutex_created_here,
                                     config_initialized_here);
        return err;
    }

    fuel_injection_init(NULL);
    if (!ignition_init()) {
        ESP_LOGE("ENGINE_CONTROL", "Failed to initialize MCPWM ignition/injection");
        engine_control_init_rollback(callback_registered,
                                     monitor_task_created,
                                     planner_task_created,
                                     executor_task_created,
                                     twai_started_here,
                                     sync_started_here,
                                     sync_initialized_here,
                                     sensor_started_here,
                                     sensor_initialized_here,
                                     map_mutex_created_here,
                                     config_initialized_here);
        return ESP_FAIL;
    }
    safety_monitor_init();
    safety_watchdog_init(1000);

    // Initialize ESP-NOW link (optional - continues on failure)
    err = espnow_link_init();
    if (err == ESP_OK) {
        ESP_LOGI("ENGINE_CONTROL", "ESP-NOW link initialized");
        err = espnow_link_start();
        if (err != ESP_OK) {
            ESP_LOGW("ENGINE_CONTROL", "ESP-NOW link start failed, wireless telemetry disabled");
        }
    } else {
        ESP_LOGW("ENGINE_CONTROL", "ESP-NOW init failed, wireless telemetry disabled");
    }

    eoi_config_blob_t eoi_cfg = {0};
    if (config_manager_load(EOI_CONFIG_KEY, &eoi_cfg, sizeof(eoi_cfg)) != ESP_OK ||
        eoi_cfg.version != EOI_CONFIG_VERSION ||
        eoi_cfg.crc32 != eoi_config_crc(&eoi_cfg)) {
        eoi_config_blob_v1_t old_cfg = {0};
        if (config_manager_load(EOI_CONFIG_KEY, &old_cfg, sizeof(old_cfg)) == ESP_OK &&
            old_cfg.version == 1U &&
            old_cfg.crc32 == esp_rom_crc32_le(0, (const uint8_t *)&old_cfg.eoi_deg, (uint32_t)(sizeof(float) * 2))) {
            eoi_cfg.version = EOI_CONFIG_VERSION;
            eoi_cfg.boundary = EOIT_DEFAULT_BOUNDARY;
            eoi_cfg.normal = eoit_normal_from_target(eoi_cfg.boundary, old_cfg.eoi_deg);
            eoi_cfg.fallback_normal = eoit_normal_from_target(eoi_cfg.boundary, old_cfg.eoi_fallback_deg);
            eoi_cfg.crc32 = eoi_config_crc(&eoi_cfg);
            config_manager_save(EOI_CONFIG_KEY, &eoi_cfg, sizeof(eoi_cfg));
        } else {
            eoi_config_defaults(&eoi_cfg);
            config_manager_save(EOI_CONFIG_KEY, &eoi_cfg, sizeof(eoi_cfg));
        }
    }
    eoi_config_apply(&eoi_cfg);

    eoit_map_config_blob_t eoit_map_cfg = {0};
    if (config_manager_load(EOIT_MAP_CONFIG_KEY, &eoit_map_cfg, sizeof(eoit_map_cfg)) != ESP_OK ||
        eoit_map_cfg.version != EOIT_MAP_CONFIG_VERSION ||
        eoit_map_cfg.crc32 != eoit_map_config_crc(&eoit_map_cfg) ||
        !table_16x16_validate(&eoit_map_cfg.normal_map)) {
        eoit_map_config_defaults(&eoit_map_cfg);
        config_manager_save(EOIT_MAP_CONFIG_KEY, &eoit_map_cfg, sizeof(eoit_map_cfg));
    }
    eoit_map_config_apply(&eoit_map_cfg);

    if (g_executor_task_handle == NULL) {
        BaseType_t task_ok = xTaskCreatePinnedToCore(engine_executor_task, "engine_exec",
                                                     CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY, &g_executor_task_handle,
                                                     CONTROL_TASK_CORE);
        if (task_ok != pdPASS) {
            ESP_LOGE("ENGINE_CONTROL", "Failed to create executor task");
            engine_control_init_rollback(callback_registered,
                                         monitor_task_created,
                                         planner_task_created,
                                         executor_task_created,
                                         twai_started_here,
                                         sync_started_here,
                                         sync_initialized_here,
                                         sensor_started_here,
                                         sensor_initialized_here,
                                         map_mutex_created_here,
                                         config_initialized_here);
            return ESP_FAIL;
        }
        executor_task_created = true;
    }

    if (g_planner_task_handle == NULL) {
        BaseType_t task_ok = xTaskCreatePinnedToCore(engine_planner_task, "engine_plan",
                                                     CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY, &g_planner_task_handle,
                                                     CONTROL_TASK_CORE);
        if (task_ok != pdPASS) {
            ESP_LOGE("ENGINE_CONTROL", "Failed to create planner task");
            engine_control_init_rollback(callback_registered,
                                         monitor_task_created,
                                         planner_task_created,
                                         executor_task_created,
                                         twai_started_here,
                                         sync_started_here,
                                         sync_initialized_here,
                                         sensor_started_here,
                                         sensor_initialized_here,
                                         map_mutex_created_here,
                                         config_initialized_here);
            return ESP_FAIL;
        }
        planner_task_created = true;
    }
    if (g_monitor_task_handle == NULL) {
        BaseType_t task_ok = xTaskCreatePinnedToCore(engine_monitor_task, "engine_mon",
                                                     MONITOR_TASK_STACK, NULL, MONITOR_TASK_PRIORITY, &g_monitor_task_handle,
                                                     MONITOR_TASK_CORE);
        if (task_ok != pdPASS) {
            ESP_LOGE("ENGINE_CONTROL", "Failed to create monitor task");
            engine_control_init_rollback(callback_registered,
                                         monitor_task_created,
                                         planner_task_created,
                                         executor_task_created,
                                         twai_started_here,
                                         sync_started_here,
                                         sync_initialized_here,
                                         sensor_started_here,
                                         sensor_initialized_here,
                                         map_mutex_created_here,
                                         config_initialized_here);
            return ESP_FAIL;
        }
        monitor_task_created = true;
    }

    sync_register_tooth_callback(engine_sync_tooth_callback, NULL);
    callback_registered = true;
    g_engine_initialized = true;

    ESP_LOGI("ENGINE_CONTROL", "Engine control system initialized");
    return ESP_OK;
}

// Start engine control
esp_err_t engine_control_start(void) {
    if (!g_engine_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI("ENGINE_CONTROL", "Engine control started");
    return ESP_OK;
}

// Stop engine control
esp_err_t engine_control_stop(void) {
    if (!g_engine_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI("ENGINE_CONTROL", "Engine control stopped");
    return ESP_OK;
}

// Deinitialize engine control
esp_err_t engine_control_deinit(void) {
    if (!g_engine_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI("ENGINE_CONTROL", "Deinitializing engine control system");

    sync_unregister_tooth_callback();
    if (g_planner_task_handle != NULL) {
        vTaskDelete(g_planner_task_handle);
        g_planner_task_handle = NULL;
    }
    if (g_executor_task_handle != NULL) {
        vTaskDelete(g_executor_task_handle);
        g_executor_task_handle = NULL;
    }
    if (g_monitor_task_handle != NULL) {
        vTaskDelete(g_monitor_task_handle);
        g_monitor_task_handle = NULL;
    }

    sensor_stop();
    sensor_deinit();
    sync_stop();
    sync_deinit();
    twai_lambda_deinit();
    mcpwm_injection_hp_deinit();
    mcpwm_ignition_hp_deinit();

    config_manager_deinit();
    if (g_map_mutex != NULL) {
        vSemaphoreDelete(g_map_mutex);
        g_map_mutex = NULL;
    }

    ESP_LOGI("ENGINE_CONTROL", "Engine control system deinitialized");
    g_engine_initialized = false;
    return ESP_OK;
}

// Get engine parameters
esp_err_t engine_control_get_engine_parameters(engine_params_t *params) {
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }

    runtime_engine_state_t runtime = {0};
    if (!runtime_state_read(&runtime)) {
        return ESP_FAIL;
    }

    params->rpm = runtime.rpm;
    params->load = runtime.load;
    params->advance_deg10 = runtime.advance_deg10;
    params->fuel_enrichment = (uint16_t)((runtime.pulsewidth_us * 100U) / (uint32_t)REQ_FUEL_US);
    params->is_limp_mode = engine_control_is_limp_mode();

    return ESP_OK;
}

esp_err_t engine_control_set_eoi_config(float eoi_deg, float eoi_fallback_deg) {
    if (!isfinite(eoi_deg) || !isfinite(eoi_fallback_deg)) {
        return ESP_ERR_INVALID_ARG;
    }

    eoi_config_blob_t cfg = {0};
    cfg.version = EOI_CONFIG_VERSION;
    cfg.boundary = g_eoit_boundary;
    cfg.normal = eoit_normal_from_target(cfg.boundary, wrap_angle_720(eoi_deg));
    cfg.fallback_normal = eoit_normal_from_target(cfg.boundary, wrap_angle_720(eoi_fallback_deg));
    cfg.crc32 = eoi_config_crc(&cfg);

    esp_err_t err = config_manager_save(EOI_CONFIG_KEY, &cfg, sizeof(cfg));
    if (err != ESP_OK) {
        return err;
    }
    eoi_config_apply(&cfg);
    return ESP_OK;
}

esp_err_t engine_control_get_eoi_config(float *eoi_deg, float *eoi_fallback_deg) {
    if (!eoi_deg || !eoi_fallback_deg) {
        return ESP_ERR_INVALID_ARG;
    }
    *eoi_deg = g_target_eoi_deg;
    *eoi_fallback_deg = g_target_eoi_deg_fallback;
    return ESP_OK;
}

esp_err_t engine_control_set_eoit_calibration(float boundary, float normal, float fallback_normal) {
    if (!isfinite(boundary) || !isfinite(normal) || !isfinite(fallback_normal)) {
        return ESP_ERR_INVALID_ARG;
    }
    boundary = clamp_eoit_boundary(boundary);
    normal = clamp_eoit_normal(normal);
    fallback_normal = clamp_eoit_normal(fallback_normal);

    eoi_config_blob_t cfg = {0};
    cfg.version = EOI_CONFIG_VERSION;
    cfg.boundary = boundary;
    cfg.normal = normal;
    cfg.fallback_normal = fallback_normal;
    cfg.crc32 = eoi_config_crc(&cfg);

    esp_err_t err = config_manager_save(EOI_CONFIG_KEY, &cfg, sizeof(cfg));
    if (err != ESP_OK) {
        return err;
    }
    eoi_config_apply(&cfg);
    return ESP_OK;
}

esp_err_t engine_control_get_eoit_calibration(float *boundary, float *normal, float *fallback_normal) {
    if (!boundary || !normal || !fallback_normal) {
        return ESP_ERR_INVALID_ARG;
    }
    *boundary = g_eoit_boundary;
    *normal = g_eoit_normal;
    *fallback_normal = g_eoit_fallback_normal;
    return ESP_OK;
}

esp_err_t engine_control_set_eoit_map_enabled(bool enabled) {
    eoit_map_config_blob_t cfg = {0};
    cfg.version = EOIT_MAP_CONFIG_VERSION;
    cfg.enabled = enabled ? 1U : 0U;
    cfg.reserved[0] = 0U;
    cfg.reserved[1] = 0U;
    cfg.reserved[2] = 0U;

    if (g_map_mutex == NULL || xSemaphoreTake(g_map_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    cfg.normal_map = g_eoit_normal_map;
    xSemaphoreGive(g_map_mutex);

    cfg.crc32 = eoit_map_config_crc(&cfg);
    esp_err_t err = config_manager_save(EOIT_MAP_CONFIG_KEY, &cfg, sizeof(cfg));
    if (err != ESP_OK) {
        return err;
    }
    eoit_map_config_apply(&cfg);
    return ESP_OK;
}

bool engine_control_get_eoit_map_enabled(void) {
    return g_eoit_map_enabled;
}

esp_err_t engine_control_set_eoit_map_cell(uint8_t rpm_idx, uint8_t load_idx, float normal) {
    if (rpm_idx >= 16U || load_idx >= 16U || !isfinite(normal)) {
        return ESP_ERR_INVALID_ARG;
    }

    eoit_map_config_blob_t cfg = {0};
    cfg.version = EOIT_MAP_CONFIG_VERSION;
    cfg.enabled = g_eoit_map_enabled ? 1U : 0U;
    cfg.reserved[0] = 0U;
    cfg.reserved[1] = 0U;
    cfg.reserved[2] = 0U;

    if (g_map_mutex == NULL || xSemaphoreTake(g_map_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    g_eoit_normal_map.values[load_idx][rpm_idx] = eoit_normal_to_table(normal);
    g_eoit_normal_map.checksum = table_16x16_checksum(&g_eoit_normal_map);
    cfg.normal_map = g_eoit_normal_map;
    xSemaphoreGive(g_map_mutex);

    cfg.crc32 = eoit_map_config_crc(&cfg);
    esp_err_t err = config_manager_save(EOIT_MAP_CONFIG_KEY, &cfg, sizeof(cfg));
    if (err != ESP_OK) {
        return err;
    }
    eoit_map_config_apply(&cfg);
    return ESP_OK;
}

esp_err_t engine_control_get_eoit_map_cell(uint8_t rpm_idx, uint8_t load_idx, float *normal) {
    if (rpm_idx >= 16U || load_idx >= 16U || !normal) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_map_mutex == NULL || xSemaphoreTake(g_map_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    uint16_t raw = g_eoit_normal_map.values[load_idx][rpm_idx];
    xSemaphoreGive(g_map_mutex);
    *normal = eoit_normal_from_table(raw);
    return ESP_OK;
}

esp_err_t engine_control_get_injection_diag(engine_injection_diag_t *diag) {
    if (!diag) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!injection_diag_read(diag)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Check if in limp mode
bool engine_control_is_limp_mode(void) {
    return safety_is_limp_mode_active();
}

void engine_control_set_closed_loop_enabled(bool enabled) {
    if (g_closed_loop_enabled == enabled) {
        return;
    }
    g_closed_loop_enabled = enabled;

    closed_loop_config_blob_t cfg = {0};
    cfg.version = CLOSED_LOOP_CONFIG_VERSION;
    cfg.enabled = enabled ? 1U : 0U;
    cfg.reserved[0] = 0;
    cfg.reserved[1] = 0;
    cfg.reserved[2] = 0;
    cfg.crc32 = closed_loop_config_crc(&cfg);
    config_manager_save(CLOSED_LOOP_CONFIG_KEY, &cfg, sizeof(cfg));
}

bool engine_control_get_closed_loop_enabled(void) {
    return g_closed_loop_enabled;
}

esp_err_t engine_control_get_perf_stats(engine_perf_stats_t *stats) {
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }

    perf_stats_t snap = {0};
    portENTER_CRITICAL(&g_perf_spinlock);
    snap = g_perf_stats;
    portEXIT_CRITICAL(&g_perf_spinlock);

    memset(stats, 0, sizeof(*stats));
    stats->planner_last_us = snap.planner_last_us;
    stats->planner_max_us = snap.planner_max_us;
    stats->executor_last_us = snap.executor_last_us;
    stats->executor_max_us = snap.executor_max_us;
    stats->planner_deadline_miss = snap.planner_deadline_miss;
    stats->executor_deadline_miss = snap.executor_deadline_miss;
    stats->queue_overruns = snap.queue_overruns;
    stats->queue_depth_peak = snap.queue_depth_peak;
    stats->sample_count = snap.sample_count;

    uint16_t n = snap.sample_count;
    if (n > 0) {
        stats->planner_p95_us = perf_percentile(snap.planner_samples, n, 95);
        stats->planner_p99_us = perf_percentile(snap.planner_samples, n, 99);
        stats->executor_p95_us = perf_percentile(snap.executor_samples, n, 95);
        stats->executor_p99_us = perf_percentile(snap.executor_samples, n, 99);
    }
    return ESP_OK;
}
