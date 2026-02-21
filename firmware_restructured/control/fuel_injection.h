#ifndef FUEL_INJECTION_H
#define FUEL_INJECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "utils/sync.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float cyl_tdc_deg[4];      // TDC angle for each cylinder (0-720)
} fuel_injection_config_t;

typedef struct {
    float eoi_deg;
    float soi_deg;
    uint32_t delay_us;
} fuel_injection_schedule_info_t;

// Initialize fuel injection scheduling
void fuel_injection_init(const fuel_injection_config_t *config);

// Schedule injection using EOI (End of Injection) logic
bool fuel_injection_schedule_eoi(uint8_t cylinder_id,
                                 float target_eoi_deg,
                                 uint32_t pulsewidth_us,
                                 const sync_data_t *sync);

bool fuel_injection_schedule_eoi_ex(uint8_t cylinder_id,
                                     float target_eoi_deg,
                                     uint32_t pulsewidth_us,
                                     const sync_data_t *sync,
                                     fuel_injection_schedule_info_t *info,
                                     float battery_voltage);

// Prepare an injection event for the angle-based scheduler (no hardware I/O)
bool fuel_injection_prepare_event(uint8_t cylinder_id,
                                  float target_eoi_deg,
                                  uint32_t pulsewidth_us,
                                  const sync_data_t *sync,
                                  fuel_injection_schedule_info_t *info,
                                  float battery_voltage,
                                  uint32_t *pulsewidth_us_out);

// Schedule sequential injection for all cylinders
bool fuel_injection_schedule_sequential(uint32_t pulsewidth_us[4],
                                         float target_eoi_deg[4],
                                         const sync_data_t *sync,
                                         float battery_voltage);

#ifdef __cplusplus
}
#endif

#endif // FUEL_INJECTION_H
