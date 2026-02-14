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
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SCHED";

// ── Internal state ─────────────────────────────────────────────────────────────

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static engine_event_t   s_queue[EVT_QUEUE_SIZE];
static scheduler_engine_state_t s_state;

// TDC offset: degrees from gap to TDC cylinder 1 (calibrated per engine)
static float s_tdc_offset_deg = 114.0f;

// Degrees per tooth for 60-2 wheel: 360 / 60 = 6.0 degrees
static float s_deg_per_tooth   = 6.0f;

// Window tolerance: fire if event is within next tooth + margin
#define FIRE_WINDOW_DEG  (s_deg_per_tooth * 1.5f)

// ── Helper: normalize angle to [0, 720) ───────────────────────────────────────

__attribute__((always_inline)) static inline float normalize_angle(float a) {
    while (a >= 720.0f) a -= 720.0f;
    while (a <    0.0f) a += 720.0f;
    return a;
}

// ── Helper: angular distance from 'from' to 'to' in forward direction ─────────
// Returns value in [0, 720)

__attribute__((always_inline)) static inline float angle_forward_dist(float from, float to) {
    float d = to - from;
    if (d < 0.0f) d += 720.0f;
    return d;
}

// ── Helper: convert angle offset to microseconds at current RPM ───────────────

IRAM_ATTR static uint32_t angle_to_us(float angle_from_now_deg, uint32_t tooth_period_us) {
    // tooth_period_us is for one tooth = s_deg_per_tooth degrees
    // us_per_deg = tooth_period_us / s_deg_per_tooth
    // result = angle_from_now_deg * tooth_period_us / s_deg_per_tooth
    return (uint32_t)((angle_from_now_deg * (float)tooth_period_us) / s_deg_per_tooth);
}

// ── Fire a single event ────────────────────────────────────────────────────────

IRAM_ATTR static void fire_event(engine_event_t *evt,
                                 uint32_t base_time_us,
                                 float    angle_from_now_deg,
                                 uint32_t tooth_period_us) {
    uint32_t delay_us = angle_to_us(angle_from_now_deg, tooth_period_us);
    uint32_t fire_abs = base_time_us + delay_us;

    switch (evt->type) {
        case EVT_INJECTOR_OPEN:
            mcpwm_injection_hp_schedule_open(evt->cylinder, fire_abs);
            break;
        case EVT_INJECTOR_CLOSE:
            mcpwm_injection_hp_schedule_close(evt->cylinder, fire_abs);
            break;
        case EVT_IGNITION_DWELL:
            mcpwm_ignition_hp_schedule_dwell(evt->cylinder, fire_abs);
            break;
        case EVT_IGNITION_SPARK:
            mcpwm_ignition_hp_schedule_spark(evt->cylinder, fire_abs);
            break;
        default:
            break;
    }

    evt->armed = false;
}

// ── Public: init ──────────────────────────────────────────────────────────────

void evt_scheduler_init(void) {
    portENTER_CRITICAL(&s_lock);
    memset(s_queue, 0, sizeof(s_queue));
    memset(&s_state, 0, sizeof(s_state));
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "Event scheduler initialized (%d slots)", EVT_QUEUE_SIZE);
}

// ── Public: on_tooth (Core 0 ISR) ─────────────────────────────────────────────

IRAM_ATTR void evt_scheduler_on_tooth(uint32_t tooth_time_us,
                                      uint32_t tooth_period_us,
                                      uint8_t  tooth_index,
                                      uint8_t  revolution_idx,
                                      uint16_t rpm) {

    portENTER_CRITICAL_ISR(&s_lock);

    // Update engine state
    s_state.tooth_time_us    = tooth_time_us;
    s_state.tooth_period_us  = tooth_period_us;
    s_state.deg_per_tooth    = s_deg_per_tooth;
    s_state.rpm              = rpm;
    s_state.revolution_index = revolution_idx;

    // Compute current absolute crank angle
    // tooth_index 0 = first tooth after gap = s_tdc_offset_deg before TDC
    float rev_offset = (revolution_idx == 0) ? 0.0f : 360.0f;
    float tooth_angle = rev_offset + (float)tooth_index * s_deg_per_tooth;
    s_state.current_angle_deg = normalize_angle(tooth_angle);

    if (!s_state.sync_valid || tooth_period_us == 0) {
        portEXIT_CRITICAL_ISR(&s_lock);
        return;
    }

    float current = s_state.current_angle_deg;

    // Check every queued event
    for (int i = 0; i < EVT_QUEUE_SIZE; i++) {
        if (!s_queue[i].armed) continue;

        float dist = angle_forward_dist(current, s_queue[i].angle_deg);

        // Fire if the event is within the upcoming tooth window (with margin)
        if (dist <= FIRE_WINDOW_DEG) {
            fire_event(&s_queue[i], tooth_time_us, dist, tooth_period_us);
        }
    }

    portEXIT_CRITICAL_ISR(&s_lock);
}

// ── Public: schedule event (Core 1) ───────────────────────────────────────────

bool evt_schedule(evt_type_t type, uint8_t cylinder, float angle_deg) {
    if (cylinder >= EVT_NUM_CYLINDERS) return false;
    if (type >= EVT_TYPE_COUNT)        return false;

    angle_deg = normalize_angle(angle_deg);

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

    s_queue[slot].type         = type;
    s_queue[slot].cylinder     = cylinder;
    s_queue[slot].angle_deg    = angle_deg;
    s_queue[slot].armed        = true;

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
    portEXIT_CRITICAL(&s_lock);
}

void evt_set_trigger_teeth(uint8_t total_teeth) {
    // total_teeth = 60 for 60-2 (including the 2 missing)
    if (total_teeth == 0) return;
    portENTER_CRITICAL(&s_lock);
    s_deg_per_tooth = 360.0f / (float)total_teeth;
    portEXIT_CRITICAL(&s_lock);
}
