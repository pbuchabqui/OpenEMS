#include map_storage.h"
#include config_manager.h"
#include "esp_err.h"
#include "esp_rom_crc.h"

#define MAP_STORAGE_KEY "fuel_maps"
#define MAP_STORAGE_VERSION 1U

typedef struct {
    uint32_t version;
    fuel_calc_maps_t maps;
    uint32_t crc32;
} map_storage_blob_t;

static uint32_t map_storage_crc(const map_storage_blob_t *blob) {
    return esp_rom_crc32_le(0, (const uint8_t *)&blob->maps, (uint32_t)sizeof(blob->maps));
}

esp_err_t map_storage_load(fuel_calc_maps_t *maps) {
    if (!maps) {
        return ESP_ERR_INVALID_ARG;
    }

    map_storage_blob_t blob = {0};
    esp_err_t err = config_manager_load(MAP_STORAGE_KEY, &blob, sizeof(blob));
    if (err != ESP_OK) {
        return err;
    }

    if (blob.version != MAP_STORAGE_VERSION) {
        return ESP_ERR_INVALID_VERSION;
    }

    uint32_t crc = map_storage_crc(&blob);
    if (crc != blob.crc32) {
        return ESP_ERR_INVALID_CRC;
    }

    *maps = blob.maps;
    return ESP_OK;
}

esp_err_t map_storage_save(const fuel_calc_maps_t *maps) {
    if (!maps) {
        return ESP_ERR_INVALID_ARG;
    }

    map_storage_blob_t blob = {
        .version = MAP_STORAGE_VERSION,
        .maps = *maps,
        .crc32 = 0,
    };
    blob.crc32 = map_storage_crc(&blob);

    return config_manager_save(MAP_STORAGE_KEY, &blob, sizeof(blob));
}
