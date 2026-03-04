/**
 * @file test/drv/test_sensors_validation.cpp
 * @brief Host unit tests for sensor validation functions (CRITICAL FIX)
 * 
 * Tests the sensor validation implementation to ensure safety-critical
 * input validation works correctly for all sensor types.
 */

#include <cstdint>
#include <cstdio>
#include <cassert>

#define EMS_HOST_TEST 1
#include "drv/sensors.h"
#include "drv/ckp.h"
#include "hal/adc.h"

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
// Test: Sensor Range Validation
// =============================================================================

void test_validate_sensor_range_valid_values() {
    ems::drv::sensors_init();
    
    // Test valid MAP values
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_range(ems::drv::SensorId::MAP, 1000));
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_range(ems::drv::SensorId::MAP, 4095));
    
    // Test valid CLT values
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_range(ems::drv::SensorId::CLT, 1000));
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_range(ems::drv::SensorId::CLT, 3800));
    
    // Test valid TPS values
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_range(ems::drv::SensorId::TPS, 500));
    TEST_ASSERT_TRUE(ems::drv::validate_sensor_range(ems::drv::SensorId::TPS, 4095));
}

void test_validate_sensor_range_invalid_values() {
    ems::drv::sensors_init();
    
    // Test invalid MAP values (below minimum)
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_range(ems::drv::SensorId::MAP, 49));
    
    // Test invalid MAP values (above maximum)
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_range(ems::drv::SensorId::MAP, 4096));
    
    // Test invalid CLT values (below minimum)
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_range(ems::drv::SensorId::CLT, 99));
    
    // Test invalid CLT values (above maximum)
    TEST_ASSERT_FALSE(ems::drv::validate_sensor_range(ems::drv::SensorId::CLT, 3801));
}

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
// Test: Sensor Health Status
// =============================================================================

void test_get_sensor_health_status_all_good() {
    ems::drv::sensors_init();
    ems::drv::sensors_test_reset();
    
    // All sensors should be healthy initially
    uint8_t status = ems::drv::get_sensor_health_status();
    TEST_ASSERT_EQ_U32(0u, status);
}

void test_get_sensor_health_status_map_fault() {
    ems::drv::sensors_init();
    ems::drv::sensors_test_reset();
    
    // Simulate MAP sensor fault by setting a narrow range and feeding
    // three fast-sampling rounds with out-of-range MAP values.
    ems::drv::sensors_set_range(ems::drv::SensorId::MAP, {100u, 200u});

    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAP_SE10, 4000u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::MAF_V_SE11, 1000u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::TPS_SE12, 1000u);
    ems::hal::adc_test_set_raw_adc0(ems::hal::Adc0Channel::O2_SE4B, 1000u);

    ems::drv::CkpSnapshot snap = {100000u, 0u, 0u, 0u, ems::drv::SyncState::FULL_SYNC, false};
    for (int i = 0; i < 15; ++i) {
        ems::drv::sensors_on_tooth(snap);
    }

    uint8_t status = ems::drv::get_sensor_health_status();
    TEST_ASSERT_TRUE((status & (1u << 0u)) != 0u);
}

// =============================================================================
// Test: Edge Cases
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

// =============================================================================
// Main Test Runner
// =============================================================================

int main() {
    printf("Running EMS Sensor Validation Tests...\n");
    
    // Sensor range validation tests
    test_validate_sensor_range_valid_values();
    test_validate_sensor_range_invalid_values();
    
    // Sensor values validation tests
    test_validate_sensor_values_all_valid();
    test_validate_sensor_values_invalid_map();
    test_validate_sensor_values_invalid_temperature();
    test_validate_sensor_values_invalid_voltage();
    test_validate_sensor_values_invalid_tps();
    test_validate_sensor_values_invalid_pressures();
    
    // Sensor health status tests
    test_get_sensor_health_status_all_good();
    test_get_sensor_health_status_map_fault();
    
    // Edge case tests
    test_validate_sensor_values_boundary_conditions();
    
    printf("Sensor validation tests completed: %d run, %d failed\n", 
           g_tests_run, g_tests_failed);
    
    return (g_tests_failed == 0) ? 0 : 1;
}
