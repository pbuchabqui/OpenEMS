#ifndef SYNC_H
#define SYNC_H

#include "esp_err.h"
#include "esp_timer.h"
#include <stdbool.h>
#include "config/engine_config.h"

// Sync configuration
typedef struct {
    uint32_t tooth_count;        // Number of teeth (excluding gap)
    uint32_t gap_tooth;          // Tooth number where gap occurs
    uint32_t max_rpm;            // Maximum RPM for calculation
    uint32_t min_rpm;            // Minimum RPM for calculation
    bool enable_phase_detection; // Enable phase detection
} sync_config_t;

// Sync data
typedef struct {
    uint32_t tooth_index;        // Current tooth index (0 to tooth_count-1)
    uint32_t time_per_degree;    // Time per degree in microseconds
    bool phase_detected;         // Phase detection status
    uint32_t rpm;                // Calculated RPM
    uint32_t last_tooth_time;    // Timestamp of last tooth in microseconds
    uint32_t tooth_period;       // Period between teeth in microseconds
    uint32_t gap_detected;       // Gap detection flag
    uint32_t phase_detected_time;// Timestamp of phase detection
    uint32_t gap_period;         // Period across the missing tooth gap (microseconds)
    uint32_t last_capture_time;  // Last captured CKP timestamp (microseconds)
    uint32_t last_update_time;   // Last time sync data updated (microseconds, esp_timer)
    uint32_t last_cmp_time;      // Last captured CMP timestamp (microseconds)
    bool cmp_detected;           // CMP edge detected
    bool cmp_seen;               // CMP seen since last gap
    uint32_t cmp_tooth_index;    // Tooth index when CMP was detected
    uint8_t revolution_index;    // 0 or 1 within 720-degree cycle
    bool sync_acquired;          // Full sync acquired (gap + phase)
    bool sync_valid;             // Sync validity based on freshness and RPM limits
    uint32_t latency_us;         // Estimated latency between capture and update
} sync_data_t;

typedef void (*sync_tooth_callback_t)(void *ctx);

// Function prototypes
esp_err_t sync_init(void);
esp_err_t sync_deinit(void);
esp_err_t sync_start(void);
esp_err_t sync_stop(void);
esp_err_t sync_reset(void);
esp_err_t sync_get_data(sync_data_t *data);
esp_err_t sync_set_config(const sync_config_t *config);
esp_err_t sync_get_config(sync_config_t *config);
esp_err_t sync_register_tooth_callback(sync_tooth_callback_t cb, void *ctx);
void sync_unregister_tooth_callback(void);

#endif // SYNC_H
