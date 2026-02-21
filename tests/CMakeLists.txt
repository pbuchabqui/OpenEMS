cmake_minimum_required(VERSION 3.16)

# OpenEMS Test Framework
# Uses Unity testing framework (already configured in ESP-IDF)

# Collect all test sources
file(GLOB_RECURSE TEST_SOURCES 
    "unit/*.c"
    "integration/*.c"
    "performance/*.c"
)

# Collect mock sources
file(GLOB MOCK_SOURCES
    "mocks/*.c"
)

# Create test component
idf_component_register(
    SRCS ${TEST_SOURCES} ${MOCK_SOURCES}
    INCLUDE_DIRS
        "."
        "mocks"
        "fixtures"
        "../firmware_restructured"
        "../firmware_restructured/hal"
        "../firmware_restructured/sensors"
        "../firmware_restructured/scheduler"
        "../firmware_restructured/control"
        "../firmware_restructured/config"
        "../firmware_restructured/comms"
        "../firmware_restructured/diagnostics"
        "../firmware_restructured/data_logging"
        "../firmware_restructured/utils"
        "../firmware_restructured/integration"
        "../firmware_restructured/main"
        "../firmware_restructured/drivers"
        "../firmware_restructured/engine_core"
    REQUIRES
        unity
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

# Compiler flags for tests
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2
    -Wall
    -Wextra
    -Wno-unused-parameter
    -ffunction-sections
    -fdata-sections
    -DUNIT_TESTING
)

# Link with main component for testing
target_link_libraries(${COMPONENT_LIB} PRIVATE openems)

# Coverage configuration (when enabled)
if(ENABLE_COVERAGE)
    target_compile_options(${COMPONENT_LIB} PRIVATE --coverage)
    target_link_libraries(${COMPONENT_LIB} PRIVATE --coverage)
endif()

# Test-specific definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    UNITY_INCLUDE_CONFIG_H
    UNITY_ENABLE_FLOAT
    UNITY_ENABLE_DOUBLE
)
