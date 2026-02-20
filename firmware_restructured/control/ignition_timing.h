#ifndef IGNITION_TIMING_H
#define IGNITION_TIMING_H

#include <stdint.h>
#include <stdbool.h>

bool ignition_init(void);
// H2 fix: added vbat_v so callers can pass the plan's battery voltage for
// consistent dwell calculation instead of always re-reading sensors.
void ignition_apply_timing(uint16_t advance_deg10, uint16_t rpm, float vbat_v);

// Get jitter statistics from high-precision timing system
void ignition_get_jitter_stats(float *avg_us, float *max_us, float *min_us);

// Update phase predictor with measured period
void ignition_update_phase(float measured_period_us);

#endif // IGNITION_TIMING_H
