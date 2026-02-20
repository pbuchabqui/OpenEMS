#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "esp_err.h"
#include "esp_system.h"

// Configuration types
typedef enum {
    CONFIG_TYPE_ENGINE_PARAMS,
    CONFIG_TYPE_SENSOR_SETTINGS,
    CONFIG_TYPE_SYSTEM_SETTINGS
} config_type_t;

// Configuration metadata
typedef struct {
    config_type_t type;
    const char *name;
    size_t size;
    uint32_t version;
} config_metadata_t;

// Function prototypes
esp_err_t config_manager_init(void);
esp_err_t config_manager_deinit(void);
esp_err_t config_manager_load(const char *name, void *config, size_t size);
esp_err_t config_manager_save(const char *name, const void *config, size_t size);
esp_err_t config_manager_set_default(const char *name, const void *config, size_t size);
esp_err_t config_manager_get_metadata(const char *name, config_metadata_t *metadata);
esp_err_t config_manager_validate_config(const char *name, const void *config, size_t size);

#endif // CONFIG_MANAGER_H
