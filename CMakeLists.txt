cmake_minimum_required(VERSION 3.16)

# ESP-IDF project
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(openems VERSION 0.1.0 LANGUAGES C CXX)

# Collect all firmware source subdirectories as a single component
set(COMPONENT_SRCS
    # Decoder (Core 0 — time critical)
    firmware/decoder/trigger_60_2.c

    # Scheduler (Core 0 — time critical)
    firmware/scheduler/event_scheduler.c
    firmware/scheduler/hp_timing.c
    firmware/scheduler/hp_state.c
    firmware/scheduler/injector_driver.c
    firmware/scheduler/ignition_driver.c

    # Sensors (Core 1)
    firmware/sensors/sensor_processing.c

    # Control loops (Core 1)
    firmware/control/engine_control.c
    firmware/control/fuel_calc.c
    firmware/control/fuel_injection.c
    firmware/control/ignition_timing.c
    firmware/control/closed_loop_fuel.c

    # Tables
    firmware/tables/table_16x16.c
    firmware/tables/map_storage.c

    # Communications (Core 1)
    firmware/comms/can_wideband.c
    firmware/comms/espnow_link.c
    firmware/comms/tunerstudio.c

    # Diagnostics (Core 1)
    firmware/diagnostics/fault_manager.c

    # Logging (Core 1)
    firmware/logging/sd_logger.c

    # Config
    firmware/config/config_manager.c

    # Utils
    firmware/utils/logger.c
    firmware/utils/cli_interface.c
    firmware/utils/test_framework.c

    # Main entry point
    firmware/main/main.c
)

set(COMPONENT_ADD_INCLUDEDIRS
    firmware
    firmware/hal
    firmware/decoder
    firmware/scheduler
    firmware/sensors
    firmware/control
    firmware/tables
    firmware/comms
    firmware/diagnostics
    firmware/logging
    firmware/config
    firmware/utils
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
