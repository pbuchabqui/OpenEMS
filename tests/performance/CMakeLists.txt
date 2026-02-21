# Performance Tests CMakeLists.txt for OpenEMS
cmake_minimum_required(VERSION 3.16)

# Performance test component
idf_component_register(
    SRCS
        timing_precision_tests.c
        system_load_tests.c
    INCLUDE_DIRS
        .
        ../fixtures
        ../../firmware_restructured
        ../../firmware_restructured/hal
        ../../firmware_restructured/scheduler
        ../../firmware_restructured/utils
    REQUIRES
        driver
        esp_timer
        freertos
        unity
    PRIV_REQUIRES
        main
)

# Compiler optimizations for performance tests
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2
    -ffast-math
    -funroll-loops
)

# Test-specific definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    TEST_PERFORMANCE_ENABLED=1
    CONFIG_ESP32_DEFAULT_CPU_FREQ_240=1
)
