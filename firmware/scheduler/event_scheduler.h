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
    float       angle_deg;          // Crank angle to fire (0–720, absolute per cycle)
    uint32_t    schedule_rev;       // Revolution counter when event was scheduled
    bool        armed;              // Set when queued, cleared after firing
} engine_event_t;

// ── Engine state (updated by decoder on every tooth) ─────────────────────────

typedef struct {
    volatile uint32_t tooth_time_us;        // Timestamp of last tooth (µs)
    volatile uint32_t tooth_period_us;      // Time between last two teeth (µs)
    volatile float    deg_per_tooth;        // Degrees per tooth (= 360 / total_teeth)
    volatile float    current_angle_deg;    // Current crank angle (0–720)
    volatile uint16_t rpm;                  // Last computed RPM
    volatile uint32_t revolution_index;     // 0 or 1 (which half-cycle)
    volatile bool     sync_valid;           // True if fully synchronized
} scheduler_engine_state_t;

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
IRAM_ATTR void evt_scheduler_on_tooth(uint32_t tooth_time_us,
                                      uint32_t tooth_period_us,
                                      uint8_t  tooth_index,
                                      uint8_t  revolution_idx,
                                      uint16_t rpm);

/**
 * @brief Schedule an engine event (call from Core 1 control task).
 *
 * Thread-safe — uses spinlock internally.
 * The event will fire at the next occurrence of angle_deg in the engine cycle.
 *
 * @param type       Event type (injector open/close, ignition dwell/spark)
 * @param cylinder   Cylinder index 0-3
 * @param angle_deg  Absolute crank angle (0–720) at which to fire
 * @return           true if event was queued, false if queue full
 */
bool evt_schedule(evt_type_t type, uint8_t cylinder, float angle_deg);

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

#ifdef __cplusplus
}
#endif

#endif // EVENT_SCHEDULER_H
