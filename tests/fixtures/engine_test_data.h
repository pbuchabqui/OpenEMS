/**
 * @file engine_test_data.h
 * @brief Test data and fixtures for engine-related tests
 */

#ifndef ENGINE_TEST_DATA_H
#define ENGINE_TEST_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Engine configuration test data
typedef struct {
    uint32_t displacement_cc;     // Engine displacement in cc
    uint32_t cylinders;           // Number of cylinders
    uint32_t req_fuel_us;         // Required fuel pulse width
    float injector_flow_cc_min;    // Injector flow rate
    uint32_t trigger_tdc_offset;  // TDC offset in degrees
    uint32_t max_rpm;             // Maximum RPM
    uint32_t idle_rpm;            // Idle RPM
} engine_config_test_t;

// Sensor test data
typedef struct {
    float map_kpa;               // Manifold absolute pressure
    float tps_percent;           // Throttle position
    float iat_celsius;           // Intake air temperature
    float clt_celsius;           // Coolant temperature
    float battery_voltage;       // Battery voltage
    float lambda;                // Air-fuel ratio
    uint32_t rpm;                // Engine speed
    bool sync_acquired;          // Sync status
} sensor_data_test_t;

// Trigger wheel test data (60-2)
typedef struct {
    uint32_t tooth_count;        // Total teeth (60)
    uint32_t missing_teeth;      // Missing teeth (2)
    uint32_t gap_tooth;          // Gap position
    uint32_t teeth_per_rev;      // Teeth per revolution
    uint32_t tooth_times[58];    // Timestamps for each tooth
    uint32_t rpm;                // Test RPM
    float tooth_period_us;       // Period per tooth
} trigger_wheel_test_t;

// Fuel calculation test data
typedef struct {
    float ve_percent;            // Volumetric efficiency
    float map_kpa;               // MAP pressure
    float iat_celsius;           // Intake temperature
    uint32_t rpm;                // Engine speed
    float req_fuel_us;           // Required fuel
    float lambda_target;         // Target lambda
    float expected_fuel_us;      // Expected fuel pulse width
    float tolerance_percent;     // Acceptable tolerance
} fuel_calc_test_t;

// Ignition timing test data
typedef struct {
    uint32_t rpm;                // Engine speed
    float map_kpa;               // MAP pressure
    float iat_celsius;           // Intake temperature
    float base_timing_deg;       // Base timing
    float total_timing_deg;      // Total timing
    float tolerance_deg;         // Acceptable tolerance
} ignition_timing_test_t;

// Performance test data
typedef struct {
    uint32_t rpm;                // Test RPM
    uint32_t expected_tooth_period_us;  // Expected tooth period
    uint32_t max_jitter_us;      // Maximum acceptable jitter
    uint32_t min_events_per_rev; // Minimum events per revolution
    uint32_t max_latency_us;     // Maximum acceptable latency
} performance_test_t;

// Predefined test datasets
extern const engine_config_test_t ENGINE_CONFIG_SMALL;
extern const engine_config_test_t ENGINE_CONFIG_MEDIUM;
extern const engine_config_test_t ENGINE_CONFIG_LARGE;

extern const sensor_data_test_t SENSOR_DATA_IDLE;
extern const sensor_data_test_t SENSOR_DATA_CRUISE;
extern const sensor_data_test_t SENSOR_DATA_WOT;

extern const trigger_wheel_test_t TRIGGER_60_2_1000_RPM;
extern const trigger_wheel_test_t TRIGGER_60_2_3000_RPM;
extern const trigger_wheel_test_t TRIGGER_60_2_6000_RPM;

extern const fuel_calc_test_t FUEL_CALC_TESTS[];
extern const size_t FUEL_CALC_TESTS_COUNT;

extern const ignition_timing_test_t IGNITION_TIMING_TESTS[];
extern const size_t IGNITION_TIMING_TESTS_COUNT;

extern const performance_test_t PERFORMANCE_TESTS[];
extern const size_t PERFORMANCE_TESTS_COUNT;

// Test helper functions
void generate_trigger_wheel_times(trigger_wheel_test_t* data);
float calculate_expected_fuel_pulse(const fuel_calc_test_t* test);
float calculate_expected_timing(const ignition_timing_test_t* test);
bool validate_timing_performance(uint32_t actual_us, uint32_t expected_us, uint32_t tolerance_us);

// Test macros
#define ENGINE_ASSERT_EQ_CONFIG(expected, actual) \
    do { \
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected.displacement_cc, actual.displacement_cc, "Engine displacement mismatch"); \
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected.cylinders, actual.cylinders, "Cylinder count mismatch"); \
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected.req_fuel_us, actual.req_fuel_us, "Required fuel mismatch"); \
    } while(0)

#define SENSOR_ASSERT_EQ_DATA(expected, actual, tolerance) \
    do { \
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, expected.map_kpa, actual.map_kpa, "MAP pressure mismatch"); \
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, expected.tps_percent, actual.tps_percent, "TPS percent mismatch"); \
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, expected.iat_celsius, actual.iat_celsius, "IAT temperature mismatch"); \
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected.rpm, actual.rpm, "RPM mismatch"); \
    } while(0)

#ifdef __cplusplus
}
#endif

#endif // ENGINE_TEST_DATA_H
