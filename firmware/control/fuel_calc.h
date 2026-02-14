#ifndef FUEL_CALC_H
#define FUEL_CALC_H

#include <stdint.h>
#include <stdbool.h>
#include "config/engine_config.h"
#include "sensor_processing.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    table_16x16_t fuel_table;
    table_16x16_t ignition_table;
    table_16x16_t lambda_table;
} fuel_calc_maps_t;

void fuel_calc_init_defaults(fuel_calc_maps_t *maps);
void fuel_calc_reset_interpolation_cache(void);

uint16_t fuel_calc_lookup_ve(const fuel_calc_maps_t *maps, uint16_t rpm, uint16_t load);
uint16_t fuel_calc_lookup_ignition(const fuel_calc_maps_t *maps, uint16_t rpm, uint16_t load);
uint16_t fuel_calc_lookup_lambda(const fuel_calc_maps_t *maps, uint16_t rpm, uint16_t load);

uint32_t fuel_calc_pulsewidth_us(const sensor_data_t *sensors,
                                 uint16_t rpm,
                                 uint16_t ve_x10,
                                 float lambda_correction);

uint16_t fuel_calc_warmup_enrichment(const sensor_data_t *sensors);

/**
 * @brief Calculate acceleration enrichment factor
 * 
 * Detects rapid MAP increase (throttle tip-in) and provides
 * temporary enrichment to prevent lean condition.
 * 
 * @param current_map_kpa10 Current MAP value in kPa * 10
 * @param now_ms Current time in milliseconds
 * @return Enrichment factor (100 = no enrichment, 150 = 50% enrichment)
 */
uint16_t fuel_calc_accel_enrichment(uint16_t current_map_kpa10, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif // FUEL_CALC_H
