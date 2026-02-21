/**
 * @file event_scheduler.h
 * @brief Angle-based event scheduler — Core 0, time-critical
 *
 * Schedules injector and ignition events by crankshaft angle (degrees)
 * rather than absolute time. On each tooth interrupt, pending events
 * whose angle falls within the next tooth window are converted from
 * degrees to microseconds and fired via MCPWM absolute compare.
 *
 * This eliminates cumulative timing error when RPM changes between
 * the scheduling decision and the actual event — exactly the rusefi
 * approach adapted for the ESP32 MCPWM hardware.
 *
 * Core 0 / ISR only. No FreeRTOS calls, no malloc.
 *
 * Usage:
 *   1. Call evt_scheduler_init() once at startup.
 *   2. From Core 1 control task: call evt_schedule() to queue events.
 *   3. From Core 0 tooth ISR: call evt_scheduler_on_tooth() on every tooth.
 *      The scheduler fires any events due within the next tooth window.
 */

#ifndef EVENT_SCHEDULER_H
#define EVENT_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Configuration ─────────────────────────────────────────────────────────────

/** Maximum number of pending events in the queue (must be power of 2) */
#define EVT_QUEUE_SIZE      16

/** Number of cylinders */
#define EVT_NUM_CYLINDERS   4

/** Degrees of crankshaft per revolution */
#define EVT_DEG_PER_REV     720.0f   // 4-stroke = 2 crank revolutions per cycle

// ── Event types ───────────────────────────────────────────────────────────────

typedef enum {
    EVT_INJECTOR_OPEN   = 0,   // Open injector at angle
    EVT_INJECTOR_CLOSE  = 1,   // Close injector at angle (= open + pulse_width)
    EVT_IGNITION_DWELL  = 2,   // Start coil charging at angle
    EVT_IGNITION_SPARK  = 3,   // Fire spark at angle
    EVT_TYPE_COUNT
} evt_type_t;

// ── Event structure ────────────────────────────────────────────────────────────

typedef struct {
    evt_type_t  type;               // What to do
    uint8_t     cylinder;           // Which cylinder (0-3)
    uint32_t    angle_q;            // Crank angle to fire (0–720, Q16.16)
    uint32_t    schedule_rev;       // Revolution counter when event was scheduled
    bool        armed;              // Set when queued, cleared after firing
    // Extra parameters forwarded to driver APIs
    uint32_t    param_us;           // Injection: pulsewidth_us; Ignition: unused
    uint16_t    rpm_snap;           // Ignition: RPM at schedule time (for dwell calc)
    float       vbat_snap;          // Ignition: battery voltage at schedule time
} engine_event_t;

// ── Engine state (updated by decoder on every tooth) ─────────────────────────

typedef struct {
    volatile uint32_t tooth_time_us;        // Timestamp of last tooth (µs)
    volatile uint32_t tooth_period_us;      // Time between last two teeth (µs)
    volatile uint32_t deg_per_tooth_q;      // Degrees per tooth (Q16.16)
    volatile uint32_t current_angle_q;      // Current crank angle (0–720, Q16.16)
    volatile uint16_t rpm;                  // Last computed RPM
    volatile uint32_t revolution_index;     // 0 or 1 (which half-cycle)
    volatile bool     sync_valid;           // True if fully synchronized
} scheduler_engine_state_t;

// ── Fixed-point angle helpers (Q16.16) ───────────────────────────────────────

#define EVT_ANGLE_Q_SHIFT 16U
#define EVT_ANGLE_Q_ONE   (1U << EVT_ANGLE_Q_SHIFT)
#define EVT_ANGLE_Q_720   (720U * EVT_ANGLE_Q_ONE)
#define EVT_ANGLE_Q_360   (360U * EVT_ANGLE_Q_ONE)

static inline uint32_t evt_angle_deg_to_q(float a) {
    if (!isfinite(a)) {
        return 0U;
    }
    if (a <= 0.0f) {
        return 0U;
    }
    float q = a * (float)EVT_ANGLE_Q_ONE;
    if (q >= (float)EVT_ANGLE_Q_720) {
        q -= (float)EVT_ANGLE_Q_720;
    }
    return (uint32_t)(q + 0.5f);
}

// ── API ───────────────────────────────────────────────────────────────────────

/**
 * @brief Initialize the event scheduler. Call once at startup (before starting sync).
 */
void evt_scheduler_init(void);

/**
 * @brief Update engine state from tooth interrupt.
 *
 * Called by Core 0 tooth ISR on every tooth.
 * Updates current_angle_deg, fires any events in window, advances internal state.
 *
 * @param tooth_time_us   Timestamp of this tooth in microseconds
 * @param tooth_period_us Time since previous tooth in microseconds
 * @param tooth_index     Position in wheel (0 = first tooth after gap)
 * @param revolution_idx  0 = first revolution, 1 = second revolution of cycle
 * @param rpm             Current RPM (computed by decoder)
 */
// H1 fix: added sync_acquired so the ISR can propagate sync state into the
// scheduler without a separate non-ISR-safe evt_set_sync_valid() call.
IRAM_ATTR void evt_scheduler_on_tooth(uint32_t tooth_time_us,
                                      uint32_t tooth_period_us,
                                      uint8_t  tooth_index,
                                      uint8_t  revolution_idx,
                                      uint16_t rpm,
                                      bool     sync_acquired);

/**
 * @brief Update engine state from tooth interrupt with explicit MCPWM timebase.
 *
 * @param mcpwm_now_us Current MCPWM counter in microseconds (absolute).
 */
IRAM_ATTR void evt_scheduler_on_tooth_mcpwm(uint32_t tooth_time_us,
                                            uint32_t tooth_period_us,
                                            uint8_t  tooth_index,
                                            uint8_t  revolution_idx,
                                            uint16_t rpm,
                                            bool     sync_acquired,
                                            uint32_t mcpwm_now_us);

/**
 * @brief Schedule an engine event (call from Core 1 control task).
 *
 * Thread-safe — uses spinlock internally.
 * The event will fire at the next occurrence of angle_deg in the engine cycle.
 *
 * @param type       Event type (injector open/close, ignition dwell/spark)
 * @param cylinder   Cylinder index 0-3
 * @param angle_deg  Absolute crank angle (0–720) at which to fire
 * @param param_us   For injection events: pulsewidth in µs. Ignored for ignition.
 * @param rpm_snap   For ignition events: current RPM (used for dwell calculation).
 * @param vbat_snap  For ignition events: current battery voltage in volts.
 * @return           true if event was queued, false if queue full
 */
bool evt_schedule(evt_type_t type, uint8_t cylinder, float angle_deg,
                  uint32_t param_us, uint16_t rpm_snap, float vbat_snap);

/**
 * @brief Schedule an engine event using precomputed Q16.16 angle.
 *
 * @param angle_q  Absolute crank angle (0–720, Q16.16)
 */
bool evt_schedule_q(evt_type_t type, uint8_t cylinder, uint32_t angle_q,
                    uint32_t param_us, uint16_t rpm_snap, float vbat_snap);

/**
 * @brief Cancel all pending events for a cylinder (e.g., limp mode).
 * Thread-safe.
 */
void evt_cancel_cylinder(uint8_t cylinder);

/**
 * @brief Cancel all pending events of a given type.
 * Thread-safe.
 */
void evt_cancel_type(evt_type_t type);

/**
 * @brief Cancel all queued events (e.g., engine stop).
 * Thread-safe.
 */
void evt_cancel_all(void);

/**
 * @brief Get number of pending events (diagnostic).
 */
uint8_t evt_pending_count(void);

/**
 * @brief Get current engine state snapshot (for Core 1 use).
 * Copies state atomically via spinlock.
 */
void evt_get_engine_state(scheduler_engine_state_t *out);

/**
 * @brief Set sync_valid flag (called by decoder after successful sync).
 * Thread-safe.
 */
void evt_set_sync_valid(bool valid);

/**
 * @brief Set the TDC offset — angular distance from the missing-tooth gap to TDC cyl.1.
 * Default 114° (60-2 wheel with common gap position).
 * Thread-safe.
 */
void evt_set_tdc_offset(float offset_deg);

/**
 * @brief Set total tooth count of the trigger wheel (including missing teeth).
 * E.g., 60 for a 60-2 wheel. Recomputes deg_per_tooth.
 * Thread-safe.
 */
void evt_set_trigger_teeth(uint8_t total_teeth);

/**
 * @brief Get count of MCPWM/timebase mismatches (diagnostic).
 */
uint32_t evt_get_mcpwm_mismatch_count(void);

#ifdef __cplusplus
}
#endif

#endif // EVENT_SCHEDULER_H
