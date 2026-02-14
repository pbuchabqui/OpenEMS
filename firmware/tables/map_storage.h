#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include "esp_err.h"
#include "fuel_calc.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t map_storage_load(fuel_calc_maps_t *maps);
esp_err_t map_storage_save(const fuel_calc_maps_t *maps);

#ifdef __cplusplus
}
#endif

#endif // MAP_STORAGE_H
