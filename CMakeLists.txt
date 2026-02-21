cmake_minimum_required(VERSION 3.16)

# ESP-IDF project
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(openems VERSION 0.1.0 LANGUAGES C CXX)

# Collect all firmware source subdirectories as a single component
set(COMPONENT_SRCS
    # Decoder (Core 0 — time critical)
    firmware_restructured/sensors/trigger_60_2.c

    # Scheduler (Core 0 — time critical)
    firmware_restructured/scheduler/event_scheduler.c
    firmware_restructured/scheduler/hp_timing.c
    firmware_restructured/engine_core/hp_state.c

    # High-Precision Drivers (Core 0 — time critical)
    firmware_restructured/drivers/mcpwm_injection_hp.c
    firmware_restructured/drivers/mcpwm_ignition_hp.c

    # Sensors (Core 1)
    firmware_restructured/sensors/sensor_processing.c
    firmware_restructured/sensors/dsp_sensor_processing.c
    firmware_restructured/sensors/optimized_sensor_processing.c
    firmware_restructured/sensors/map_tps_filters.c
    firmware_restructured/sensors/ulp_monitor.c

    # Control loops (Core 1)
    firmware_restructured/engine_core/engine_control.c
    firmware_restructured/control/fuel_calc.c
    firmware_restructured/control/fuel_injection.c
    firmware_restructured/control/ignition_timing.c
    firmware_restructured/control/closed_loop_fuel.c

    # Tables
    firmware_restructured/config/table_16x16.c
    firmware_restructured/config/map_storage.c

    # Communications (Core 1)
    firmware_restructured/comms/can_wideband.c
    firmware_restructured/comms/espnow_link.c
    firmware_restructured/comms/espnow_compression.c
    firmware_restructured/comms/tunerstudio.c

    # Diagnostics (Core 1)
    firmware_restructured/diagnostics/fault_manager.c

    # Logging (Core 1)
    firmware_restructured/data_logging/sd_logger.c

    # Config
    firmware_restructured/config/config_manager.c

    # Utils
    firmware_restructured/utils/logger.c
    firmware_restructured/utils/cli_interface.c
    firmware_restructured/utils/latency_benchmark.c
    firmware_restructured/utils/vector_math.c

    # ESP32-S3 Integration
    firmware_restructured/integration/esp32s3_integration.c

    # Main entry point
    firmware_restructured/main/esp32s3_main_integration.c
)

set(COMPONENT_ADD_INCLUDEDIRS
    firmware_restructured
    firmware_restructured/hal
    firmware_restructured/sensors
    firmware_restructured/scheduler
    firmware_restructured/control
    firmware_restructured/config
    firmware_restructured/comms
    firmware_restructured/diagnostics
    firmware_restructured/data_logging
    firmware_restructured/utils
    firmware_restructured/integration
    firmware_restructured/main
    firmware_restructured/drivers
    firmware_restructured/engine_core
)

# ESP-IDF component registration
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
    REQUIRES
        driver
        esp_timer
        freertos
        nvs_flash
        esp_wifi
        esp_now
        esp_eth
        fatfs
        sdmmc
        log
        soc
        hal
        esp_rom
        esp_hw_support
        esp_dsp
        esp_nn
)

# Compiler flags
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2
    -Wall
    -Wextra
    -Wno-unused-parameter
    -ffunction-sections
    -fdata-sections
)
