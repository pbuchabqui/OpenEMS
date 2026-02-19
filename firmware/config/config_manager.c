#include "config_manager.h"
#include "logger.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

// Static variables
static bool g_config_manager_initialized = false;

// Initialize configuration manager
esp_err_t config_manager_init(void) {
    if (g_config_manager_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE("CONFIG_MANAGER", "Failed to initialize NVS: %s", esp_err_to_name(err));
        return err;
    }

    g_config_manager_initialized = true;
    ESP_LOGI("CONFIG_MANAGER", "Configuration manager initialized");
    return ESP_OK;
}

// Deinitialize configuration manager
esp_err_t config_manager_deinit(void) {
    if (!g_config_manager_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // NVS deinitialization is not required
    g_config_manager_initialized = false;
    ESP_LOGI("CONFIG_MANAGER", "Configuration manager deinitialized");
    return ESP_OK;
}

// Load configuration from NVS
esp_err_t config_manager_load(const char *name, void *config, size_t size) {
    if (!g_config_manager_initialized || !name || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("CONFIG_MANAGER", "Failed to open NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_blob(nvs_handle, name, config, &size);
    nvs_close(nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("CONFIG_MANAGER", "Configuration '%s' not found, using defaults", name);
        return ESP_ERR_NOT_FOUND;
    } else if (err != ESP_OK) {
        ESP_LOGE("CONFIG_MANAGER", "Failed to load configuration '%s': %s", name, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI("CONFIG_MANAGER", "Loaded configuration '%s'", name);
    return ESP_OK;
}

// Save configuration to NVS
esp_err_t config_manager_save(const char *name, const void *config, size_t size) {
    if (!g_config_manager_initialized || !name || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("CONFIG_MANAGER", "Failed to open NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, name, config, size);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE("CONFIG_MANAGER", "Failed to save configuration '%s': %s", name, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI("CONFIG_MANAGER", "Saved configuration '%s'", name);
    return ESP_OK;
}

// Set default configuration
esp_err_t config_manager_set_default(const char *name, const void *config, size_t size) {
    if (!g_config_manager_initialized || !name || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Save default configuration
    return config_manager_save(name, config, size);
}

// Get configuration metadata
esp_err_t config_manager_get_metadata(const char *name, config_metadata_t *metadata) {
    if (!g_config_manager_initialized || !name || !metadata) {
        return ESP_ERR_INVALID_ARG;
    }

    metadata->type = CONFIG_TYPE_SYSTEM_SETTINGS;
    metadata->name = name;
    metadata->size = 0;
    metadata->version = 1;
    return ESP_OK;
}

// Validate configuration
esp_err_t config_manager_validate_config(const char *name, const void *config, size_t size) {
    if (!g_config_manager_initialized || !name || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Basic validation - would need to be extended
    if (size == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}
