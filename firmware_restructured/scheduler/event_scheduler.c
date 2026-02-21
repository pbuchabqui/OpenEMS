/**
 * @file event_scheduler.c
 * @brief Angle-based event scheduler implementation
 *
 * Core design:
 *   - Events are stored in a fixed-size array (no malloc).
 *   - On every tooth, each armed event is checked: if its angle falls
 *     within [current_angle, current_angle + deg_per_tooth), it is converted
 *     to an absolute timer value and fired via the MCPWM injector/ignition
 *     drivers using their absolute-compare API.
 *   - Cross-core safety: Core 1 writes via spinlock, Core 0 reads in ISR
 *     via portENTER_CRITICAL_ISR / portEXIT_CRITICAL_ISR.
 *
 * Angle convention:
 *   0°   = first tooth after gap (TDC cylinder 1 + tdc_offset_deg)
 *   360° = one full crank revolution later
 *   720° = end of 4-stroke cycle (same as 0° on next cycle)
 *
 * TDC offset calibration:
 *   tdc_offset_deg is the angular distance between the gap and actual TDC.
 *   Set via evt_set_tdc_offset() from config. Default 114° (common for 60-2).
 */

#include "event_scheduler.h"
#include "injector_driver.h"
#include "ignition_driver.h"
#include "mcpwm_injection_hp.h"
#include "mcpwm_ignition_hp.h"
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "../utils/latency_benchmark.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SCHED";

// ── Internal state ─────────────────────────────────────────────────────────────

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static engine_event_t   s_queue[EVT_QUEUE_SIZE];
static scheduler_engine_state_t s_state;
static uint32_t s_last_mcpwm_us = 0;
static uint32_t s_mcpwm_mismatch_count = 0;

// TDC offset: degrees from gap to TDC cylinder 1 (calibrated per engine)
static float s_tdc_offset_deg = 114.0f;

// Degrees per tooth for 60-2 wheel: 360 / 60 = 6.0 degrees
static float s_deg_per_tooth   = 6.0f;
static float s_inv_deg_per_tooth = 1.0f / 6.0f;

// Q16.16 fixed-point angle constants
#define ANGLE_Q_SHIFT 16U
#define ANGLE_Q_ONE   (1U << ANGLE_Q_SHIFT)
#define ANGLE_Q_720   (720U * ANGLE_Q_ONE)
#define ANGLE_Q_360   (360U * ANGLE_Q_ONE)

static uint32_t s_deg_per_tooth_q = (uint32_t)(6.0f * (float)ANGLE_Q_ONE);
static uint32_t s_tdc_offset_q = (uint32_t)(114.0f * (float)ANGLE_Q_ONE);

// Precomputed tooth angles (rev 0, tooth index 0..N-1)
static uint32_t s_tooth_angle_lut[128];
static uint8_t s_tooth_lut_size = 0;

// Window tolerance: fire if event is within next tooth + margin
#define FIRE_WINDOW_DEG  (s_deg_per_tooth * 1.5f)
static uint32_t s_fire_window_q = (uint32_t)(6.0f * 1.5f * (float)ANGLE_Q_ONE);

// OPTIMIZATION: Precomputed lookup tables for angle->delay conversion
#define RPM_BINS 32
#define ANGLE_BINS 720
static uint32_t s_angle_delay_lut[RPM_BINS][ANGLE_BINS]; // Q16.16 delays
static uint16_t s_rpm_bin_edges[RPM_BINS];
static bool s_lut_initialized = false;

// OPTIMIZATION: Fast path state cache (aligned to cache line)
typedef struct __attribute__((aligned(64))) {
    uint32_t cached_tooth_period_us;
    uint32_t cached_deg_per_tooth_q;
    uint16_t cached_rpm;
    uint8_t  cache_valid;
    uint8_t  padding[63 - 7]; // Pad to 64 bytes
} fast_state_cache_t;

static fast_state_cache_t s_fast_cache = {0};

// ── Helper: normalize angle to [0, 720) ───────────────────────────────────────

__attribute__((always_inline)) static inline uint32_t angle_deg_to_q(float a) {
    if (!isfinite(a)) {
        return 0U;
    }
    if (a <= 0.0f) {
        return 0U;
    }
    float q = a * (float)ANGLE_Q_ONE;
    if (q >= (float)ANGLE_Q_720) {
        q -= (float)ANGLE_Q_720;
    }
    return (uint32_t)(q + 0.5f);
}

__attribute__((always_inline)) static inline uint32_t normalize_angle_q(uint32_t a_q) {
    if (a_q >= ANGLE_Q_720) {
        a_q -= ANGLE_Q_720;
    }
    return a_q;
}

// ── Helper: angular distance from 'from' to 'to' in forward direction ─────────
// Returns value in [0, 720)

__attribute__((always_inline)) static inline uint32_t angle_forward_dist_q(uint32_t from_q, uint32_t to_q) {
    if (to_q >= from_q) {
        return to_q - from_q;
    }
    return (ANGLE_Q_720 - from_q) + to_q;
}

// ── OPTIMIZATION: Initialize lookup tables for fast angle->delay conversion ─────

static void evt_init_lookup_tables(void) {
    if (s_lut_initialized) {
        return;
    }
    
    // Initialize RPM bin edges (250 RPM per bin, 0-8000 RPM)
    for (int i = 0; i < RPM_BINS; i++) {
        s_rpm_bin_edges[i] = (uint16_t)(i * 250);
    }
    
    // Precompute angle->delay for each RPM bin
    for (int rpm_idx = 0; rpm_idx < RPM_BINS; rpm_idx++) {
        uint16_t rpm = s_rpm_bin_edges[rpm_idx];
        if (rpm == 0) rpm = 1; // Avoid division by zero
        
        // Time per degree in microseconds: (60s / (RPM * 360deg)) * 1,000,000
        uint32_t us_per_deg_q = (uint32_t)(((60.0f * 1000000.0f) / (rpm * 360.0f)) * (float)ANGLE_Q_ONE);
        
        for (int angle = 0; angle < ANGLE_BINS; angle++) {
            // Delay in Q16.16 format
            s_angle_delay_lut[rpm_idx][angle] = (uint32_t)((uint32_t)angle * us_per_deg_q) >> ANGLE_Q_SHIFT;
        }
    }
    
    s_lut_initialized = true;
    ESP_LOGI(TAG, "Lookup tables initialized for fast angle conversion");
}

// ── OPTIMIZATION: Fast angle->delay using lookup tables ───────────────────────

__attribute__((always_inline)) static inline uint32_t angle_to_us_fast(uint32_t angle_q, uint32_t tooth_period_us, uint16_t rpm) {
    // Update cache if RPM changed significantly
    if (!s_fast_cache.cache_valid || 
        (abs((int)rpm - (int)s_fast_cache.cached_rpm) > 50) ||
        (abs((int)tooth_period_us - (int)s_fast_cache.cached_tooth_period_us) > 100)) {
        
        s_fast_cache.cached_rpm = rpm;
        s_fast_cache.cached_tooth_period_us = tooth_period_us;
        s_fast_cache.cached_deg_per_tooth_q = s_deg_per_tooth_q;
        s_fast_cache.cache_valid = 1;
    }
    
    // Find RPM bin
    uint8_t rpm_bin = (rpm < 8000) ? (rpm / 250) : (RPM_BINS - 1);
    if (rpm_bin >= RPM_BINS) rpm_bin = RPM_BINS - 1;
    
    // Convert angle to degrees (integer)
    uint16_t angle_deg = (uint16_t)(angle_q >> ANGLE_Q_SHIFT);
    if (angle_deg >= ANGLE_BINS) angle_deg = angle_deg % ANGLE_BINS;
    
    // Lookup precomputed delay
    return s_angle_delay_lut[rpm_bin][angle_deg];
}

// ── Helper: convert angle offset to microseconds at current RPM ───────────────

IRAM_ATTR static uint32_t angle_to_us_q(uint32_t angle_from_now_q, uint32_t tooth_period_us) {
    if (s_deg_per_tooth_q == 0U) {
        return 0U;
    }
    // delay_us = angle_q * tooth_period_us / deg_per_tooth_q
    uint64_t num = (uint64_t)angle_from_now_q * (uint64_t)tooth_period_us;
    return (uint32_t)(num / s_deg_per_tooth_q);
}

// ── Fire a single event ────────────────────────────────────────────────────────

IRAM_ATTR static void fire_event(engine_event_t *evt,
                                 uint32_t base_time_us,
                                 uint32_t angle_from_now_q,
                                 uint32_t tooth_period_us) {
    
    // OPTIMIZATION: Use fast lookup instead of division
    uint32_t delay_us = angle_to_us_fast(angle_from_now_q, tooth_period_us, s_state.rpm);
    uint32_t fire_abs  = base_time_us + delay_us;

    switch (evt->type) {
        case EVT_INJECTOR_OPEN:
            // EVT_INJECTOR_OPEN: schedule a one-shot injection pulse.
            // param_us carries the pulsewidth.  current_counter = base_time_us.
            mcpwm_injection_hp_schedule_one_shot_absolute(
                evt->cylinder,
                fire_abs,
                evt->param_us,
                base_time_us);
            break;

        default:
            // Unknown event type - ignore
            break;
    }

    evt->armed = false;
}

// ── Public: init ──────────────────────────────────────────────────────────────

void evt_scheduler_init(void) {
    portENTER_CRITICAL(&s_lock);
    memset(s_queue, 0, sizeof(s_queue));
    memset(&s_state, 0, sizeof(s_state));
    s_state.deg_per_tooth_q = s_deg_per_tooth_q;
    s_last_mcpwm_us = 0;
    s_mcpwm_mismatch_count = 0;
    portEXIT_CRITICAL(&s_lock);

    // Initialize tooth angle LUT
    uint8_t tooth_count = 58; // 60-2 wheel
    for (uint8_t i = 0; i < tooth_count && i < 128; i++) {
        s_tooth_angle_lut[i] = (uint32_t)(i * s_deg_per_tooth_q) + s_tdc_offset_q;
    }
    s_tooth_lut_size = tooth_count;
    
    // OPTIMIZATION: Initialize fast lookup tables
    evt_init_lookup_tables();
    
    // OPTIMIZATION: Reset fast cache
    memset(&s_fast_cache, 0, sizeof(s_fast_cache));
    
    ESP_LOGI(TAG, "Event scheduler initialized (%d slots)", EVT_QUEUE_SIZE);
}

// ── Public: on_tooth (Core 0 ISR) ─────────────────────────────────────────────

IRAM_ATTR void evt_scheduler_on_tooth(uint32_t tooth_time_us,
                                      uint32_t tooth_period_us,
                                      uint8_t  tooth_index,
                                      uint8_t  revolution_idx,
                                      uint16_t rpm,
                                      bool     sync_acquired) {
    uint32_t mcpwm_now_us = mcpwm_injection_hp_get_counter(0);
    evt_scheduler_on_tooth_mcpwm(tooth_time_us,
                                 tooth_period_us,
                                 tooth_index,
                                 revolution_idx,
                                 rpm,
                                 sync_acquired,
                                 mcpwm_now_us);
}

IRAM_ATTR void evt_scheduler_on_tooth_mcpwm(uint32_t tooth_time_us,
                                            uint32_t tooth_period_us,
                                            uint8_t  tooth_index,
                                            uint8_t  revolution_idx,
                                            uint16_t rpm,
                                            bool     sync_acquired,
                                            uint32_t mcpwm_now_us) {

    // OPTIMIZATION: Benchmark ISR entry
    BENCHMARK_ISR_START();
    
    // OPTIMIZATION: Fast path - prepare data outside critical section
    if (mcpwm_now_us == 0U) {
        mcpwm_now_us = tooth_time_us;
    }
    
    // Pre-compute angle outside critical section
    uint32_t rev_offset_q = (revolution_idx == 0) ? 0U : ANGLE_Q_360;
    uint32_t tooth_angle_q = 0U;
    if (tooth_index < s_tooth_lut_size) {
        tooth_angle_q = s_tooth_angle_lut[tooth_index] + rev_offset_q;
    } else {
        tooth_angle_q = rev_offset_q
                      + ((uint32_t)tooth_index * s_deg_per_tooth_q)
                      + s_tdc_offset_q;
    }
    tooth_angle_q = normalize_angle_q(tooth_angle_q);
    
    // OPTIMIZATION: Benchmark critical section timing
    BENCHMARK_SCHEDULER_START();
    
    portENTER_CRITICAL_ISR(&s_lock);

    // H1 fix: propagate decoder sync state so events fire only when fully synced.
    s_state.sync_valid = sync_acquired;

    // Update engine state (minimized critical section)
    s_state.tooth_time_us    = mcpwm_now_us;
    s_state.tooth_period_us  = tooth_period_us;
    s_state.deg_per_tooth_q  = s_deg_per_tooth_q;
    s_state.rpm              = rpm;
    s_state.revolution_index = revolution_idx;
    s_state.current_angle_q = tooth_angle_q;

    // OPTIMIZATION: Simplified MCPWM validation
    if (s_last_mcpwm_us != 0U && tooth_period_us > 0U) {
        uint32_t delta = mcpwm_now_us - s_last_mcpwm_us;
        if (delta > (tooth_period_us + (tooth_period_us >> 1))) {
            s_mcpwm_mismatch_count++;
        }
    }
    s_last_mcpwm_us = mcpwm_now_us;

    if (!s_state.sync_valid || tooth_period_us == 0) {
        portEXIT_CRITICAL_ISR(&s_lock);
        BENCHMARK_SCHEDULER_END();
        BENCHMARK_ISR_END();
        return;
    }

    // OPTIMIZATION: Batch event processing - copy events to local array
    typedef struct {
        engine_event_t* evt;
        uint32_t dist_q;
    } event_batch_t;
    
    event_batch_t events_to_fire[8]; // Max 8 events per tooth
    uint8_t fire_count = 0;
    uint32_t current_q = s_state.current_angle_q;
    
    // Collect events to fire (still in critical section but minimal work)
    for (int i = 0; i < EVT_QUEUE_SIZE && fire_count < 8; i++) {
        if (!s_queue[i].armed) continue;
        
        uint32_t dist_q = angle_forward_dist_q(current_q, s_queue[i].angle_q);
        if (dist_q <= s_fire_window_q) {
            events_to_fire[fire_count].evt = &s_queue[i];
            events_to_fire[fire_count].dist_q = dist_q;
            fire_count++;
        }
    }

    portEXIT_CRITICAL_ISR(&s_lock);
    
    BENCHMARK_SCHEDULER_END();
    
    // OPTIMIZATION: Benchmark MCPWM operations
    BENCHMARK_MCPWM_START();
    
    // OPTIMIZATION: Fire events outside critical section
    for (uint8_t i = 0; i < fire_count; i++) {
        fire_event(events_to_fire[i].evt, mcpwm_now_us, events_to_fire[i].dist_q, tooth_period_us);
    }
    
    BENCHMARK_MCPWM_END();
    BENCHMARK_ISR_END();
}

// ── Public: schedule event (Core 1) ───────────────────────────────────────────

bool evt_schedule(evt_type_t type, uint8_t cylinder, float angle_deg,
                  uint32_t param_us, uint16_t rpm_snap, float vbat_snap) {
    if (cylinder >= EVT_NUM_CYLINDERS) return false;
    if (type >= EVT_TYPE_COUNT)        return false;

    uint32_t angle_q = angle_deg_to_q(angle_deg);

    portENTER_CRITICAL(&s_lock);

    // Find empty slot
    int slot = -1;
    for (int i = 0; i < EVT_QUEUE_SIZE; i++) {
        if (!s_queue[i].armed) { slot = i; break; }
    }

    if (slot < 0) {
        portEXIT_CRITICAL(&s_lock);
        return false;  // Queue full
    }

    s_queue[slot].type      = type;
    s_queue[slot].cylinder  = cylinder;
    s_queue[slot].angle_q   = angle_q;
    s_queue[slot].param_us  = param_us;
    s_queue[slot].rpm_snap  = rpm_snap;
    s_queue[slot].vbat_snap = vbat_snap;
    s_queue[slot].armed     = true;

    portEXIT_CRITICAL(&s_lock);
    return true;
}

// ── Public: cancel helpers ────────────────────────────────────────────────────

void evt_cancel_cylinder(uint8_t cylinder) {
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < EVT_QUEUE_SIZE; i++) {
        if (s_queue[i].armed && s_queue[i].cylinder == cylinder) {
            s_queue[i].armed = false;
        }
    }
    portEXIT_CRITICAL(&s_lock);
}

void evt_cancel_type(evt_type_t type) {
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < EVT_QUEUE_SIZE; i++) {
        if (s_queue[i].armed && s_queue[i].type == type) {
            s_queue[i].armed = false;
        }
    }
    portEXIT_CRITICAL(&s_lock);
}

void evt_cancel_all(void) {
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < EVT_QUEUE_SIZE; i++) {
        s_queue[i].armed = false;
    }
    portEXIT_CRITICAL(&s_lock);
}

uint8_t evt_pending_count(void) {
    uint8_t count = 0;
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < EVT_QUEUE_SIZE; i++) {
        if (s_queue[i].armed) count++;
    }
    portEXIT_CRITICAL(&s_lock);
    return count;
}

void evt_get_engine_state(scheduler_engine_state_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}

// ── Public: sync_valid flag (set by decoder) ──────────────────────────────────

void evt_set_sync_valid(bool valid) {
    portENTER_CRITICAL(&s_lock);
    s_state.sync_valid = valid;
    portEXIT_CRITICAL(&s_lock);
}

// ── Public: configure TDC offset and trigger wheel ───────────────────────────

void evt_set_tdc_offset(float offset_deg) {
    portENTER_CRITICAL(&s_lock);
    s_tdc_offset_deg = offset_deg;
    s_tdc_offset_q = angle_deg_to_q(offset_deg);
    for (uint8_t i = 0; i < s_tooth_lut_size; i++) {
        s_tooth_angle_lut[i] = ((uint32_t)i * s_deg_per_tooth_q) + s_tdc_offset_q;
    }
    portEXIT_CRITICAL(&s_lock);
}

void evt_set_trigger_teeth(uint8_t total_teeth) {
    // total_teeth = 60 for 60-2 (including the 2 missing)
    if (total_teeth == 0) return;
    portENTER_CRITICAL(&s_lock);
    s_deg_per_tooth = 360.0f / (float)total_teeth;
    s_inv_deg_per_tooth = 1.0f / s_deg_per_tooth;
    s_deg_per_tooth_q = (uint32_t)(s_deg_per_tooth * (float)ANGLE_Q_ONE);
    s_fire_window_q = (uint32_t)((s_deg_per_tooth * 1.5f) * (float)ANGLE_Q_ONE);
    uint8_t max_lut = (uint8_t)(sizeof(s_tooth_angle_lut) / sizeof(s_tooth_angle_lut[0]));
    s_tooth_lut_size = (total_teeth < max_lut ? total_teeth : max_lut);
    for (uint8_t i = 0; i < s_tooth_lut_size; i++) {
        s_tooth_angle_lut[i] = ((uint32_t)i * s_deg_per_tooth_q) + s_tdc_offset_q;
    }
    portEXIT_CRITICAL(&s_lock);
}

uint32_t evt_get_mcpwm_mismatch_count(void) {
    uint32_t count = 0;
    portENTER_CRITICAL(&s_lock);
    count = s_mcpwm_mismatch_count;
    portEXIT_CRITICAL(&s_lock);
    return count;
}
