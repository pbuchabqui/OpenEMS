#ifndef IGNITION_TIMING_H
#define IGNITION_TIMING_H

#include <stdint.h>
#include <stdbool.h>

bool ignition_init(void);
// Schedules ignition events via the angle-based scheduler.
bool ignition_schedule_events(uint16_t advance_deg10, uint16_t rpm, float vbat_v);
bool ignition_schedule_events(uint16_t advance_deg10, uint16_t rpm, float vbat_v);

// Get jitter statistics from high-precision timing system
void ignition_get_jitter_stats(float *avg_us, float *max_us, float *min_us);

// Update phase predictor with measured period
void ignition_update_phase(float measured_period_us);

#endif // IGNITION_TIMING_H
