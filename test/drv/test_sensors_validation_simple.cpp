/**
 * @file test/drv/test_sensors_validation_simple.cpp
 * @brief Simple host unit tests for sensor validation functions (CRITICAL FIX)
 * 
 * Tests only the validation functions without requiring HAL mocks.
 */

#include <cstdint>
#include <cstdio>
#include <cassert>

#define EMS_HOST_TEST 1
#include "drv/sensors.h"

namespace {
int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_FALSE(cond) do { \
    ++g_tests_run; \
    if (cond) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: expected false but got true\n", __FILE__, __LINE__); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

ems::drv::SensorData create_test_sensor_data() {
    ems::drv::SensorData data = {};
    data.map_kpa_x10 = 1000;        // 100.0 kPa
    data.clt_degc_x10 = 800;        // 80.0°C
    data.iat_degc_x10 = 300;        // 30.0°C
    data.tps_pct_x10 = 500;         // 50.0%
    data.vbatt_mv = 12000;           // 12.0V
    data.fuel_press_kpa_x10 = 3000; // 300.0 kPa
    data.oil_press_kpa_x10 = 2000;  // 200.0 kPa
    data.fault_bits = 0u;
    return data;
}

} // namespace

// =============================================================================
// Test: Sensor Values Validation
// =============================================================================

void test_validate_sensor_values_all_valid() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_values(data));
}

void test_validate_sensor_values_invalid_map() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    // Test MAP too low
    data.map_kpa_x10 = 50;  // 5.0 kPa - below minimum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    // Test MAP too high
    data.map_kpa_x10 = 3000;  // 300.0 kPa - above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
}

void test_validate_sensor_values_invalid_temperature() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    // Test CLT too low
    data.clt_degc_x10 = -500;  // -50.0°C - below minimum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    // Test CLT too high
    data.clt_degc_x10 = 2000;  // 200.0°C - above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    // Reset and test IAT
    data = create_test_sensor_data();
    data.iat_degc_x10 = -500;  // -50.0°C - below minimum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
}

void test_validate_sensor_values_invalid_voltage() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    // Test voltage too low
    data.vbatt_mv = 5000;  // 5.0V - below minimum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    // Test voltage too high
    data.vbatt_mv = 20000;  // 20.0V - above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
}

void test_validate_sensor_values_invalid_tps() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    // Test TPS too high
    data.tps_pct_x10 = 1100;  // 110.0% - above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
}

void test_validate_sensor_values_invalid_pressures() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    // Test fuel pressure too high
    data.fuel_press_kpa_x10 = 6000;  // 600.0 kPa - above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    // Test oil pressure too high
    data.oil_press_kpa_x10 = 12000;  // 1200.0 kPa - above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
}

// =============================================================================
// Test: Edge Cases and Boundary Conditions
// =============================================================================

void test_validate_sensor_values_boundary_conditions() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    // Test MAP at boundaries
    data.map_kpa_x10 = 100;   // 10.0 kPa - minimum valid
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_values(data));
    
    data.map_kpa_x10 = 2500;  // 250.0 kPa - maximum valid
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_values(data));
    
    // Test temperature at boundaries
    data.clt_degc_x10 = -400;  // -40.0°C - minimum valid
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_values(data));
    
    data.clt_degc_x10 = 1500;  // 150.0°C - maximum valid
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_values(data));
    
    // Test voltage at boundaries
    data.vbatt_mv = 6000;   // 6.0V - minimum valid
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_values(data));
    
    data.vbatt_mv = 18000;  // 18.0V - maximum valid
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_values(data));
}

void test_validate_sensor_values_boundary_invalid() {
    ems::drv::SensorData data = create_test_sensor_data();
    
    // Test MAP just outside boundaries
    data.map_kpa_x10 = 99;    // 9.9 kPa - just below minimum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    data.map_kpa_x10 = 2501;  // 250.1 kPa - just above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    // Test temperature just outside boundaries
    data.clt_degc_x10 = -401;  // -40.1°C - just below minimum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
    
    data.clt_degc_x10 = 1501;  // 150.1°C - just above maximum
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_values(data));
}

// =============================================================================
// Test: Sensor Health Status
// =============================================================================

void test_get_sensor_health_status_interface() {
    // Test that the function exists and returns a reasonable value
    uint8_t status = ems::drv::get_sensor_health_status();
    
    // Should return a value (0-255)
    TEST_ASSERT_TRUE(status <= 255);
}

// =============================================================================
// Main Test Runner
// =============================================================================

int main() {
    printf("Running EMS Sensor Validation Simple Tests...\n");
    
    // Sensor values validation tests
    test_validate_sensor_values_all_valid();
    test_validate_sensor_values_invalid_map();
    test_validate_sensor_values_invalid_temperature();
    test_validate_sensor_values_invalid_voltage();
    test_validate_sensor_values_invalid_tps();
    test_validate_sensor_values_invalid_pressures();
    
    // Edge case and boundary tests
    test_validate_sensor_values_boundary_conditions();
    test_validate_sensor_values_boundary_invalid();
    
    // Sensor health status test
    test_get_sensor_health_status_interface();
    
    printf("Sensor validation simple tests completed: %d run, %d failed\n", 
           g_tests_run, g_tests_failed);
    
    return (g_tests_failed == 0) ? 0 : 1;
}
