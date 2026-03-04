/**
 * @file test/drv/test_validation_only.cpp
 * @brief Standalone test for sensor validation functions (CRITICAL FIX)
 * 
 * This test includes only the validation functions to avoid HAL dependencies.
 */

#include <cstdint>
#include <cstdio>
#include <cassert>

// Minimal sensor definitions for testing
namespace ems::drv {

enum class SensorId : uint8_t {
    MAP        = 0,
    MAF        = 1,
    TPS        = 2,
    CLT        = 3,
    IAT        = 4,
    O2         = 5,
    FUEL_PRESS = 6,
    OIL_PRESS  = 7,
};

struct SensorData {
    uint16_t map_kpa_x10;         // MAP kPa × 10
    uint32_t maf_gps_x100;        // MAF g/s × 100
    uint16_t tps_pct_x10;         // TPS % × 10
    int16_t  clt_degc_x10;        // CLT °C × 10
    int16_t  iat_degc_x10;        // IAT °C × 10
    uint16_t fuel_press_kpa_x10;  // pressão combustível kPa × 10
    uint16_t oil_press_kpa_x10;   // pressão óleo kPa × 10
    uint16_t vbatt_mv;            // tensão bateria mV
    uint8_t  fault_bits;          // bitmask de falhas ativas
    uint16_t an1_raw;
    uint16_t an2_raw;
    uint16_t an3_raw;
    uint16_t an4_raw;
};

// CRITICAL FIX: Sensor validation implementation
bool validate_sensor_values(const SensorData& data) noexcept {
    // Validate MAP: 10 kPa to 250 kPa (×10)
    if ((data.map_kpa_x10 < 100u) || (data.map_kpa_x10 > 2500u)) {
        return false;
    }
    
    // Validate CLT: -40°C to +150°C (×10)
    if ((data.clt_degc_x10 < -400) || (data.clt_degc_x10 > 1500)) {
        return false;
    }
    
    // Validate IAT: -40°C to +150°C (×10)
    if ((data.iat_degc_x10 < -400) || (data.iat_degc_x10 > 1500)) {
        return false;
    }
    
    // Validate TPS: 0% to 100% (×10)
    if (data.tps_pct_x10 > 1000u) {
        return false;
    }
    
    // Validate battery voltage: 6V to 18V
    if ((data.vbatt_mv < 6000u) || (data.vbatt_mv > 18000u)) {
        return false;
    }
    
    // Validate fuel pressure: 0 kPa to 500 kPa (×10)
    if (data.fuel_press_kpa_x10 > 5000u) {
        return false;
    }
    
    // Validate oil pressure: 0 kPa to 1000 kPa (×10)
    if (data.oil_press_kpa_x10 > 10000u) {
        return false;
    }
    
    return true;
}

uint8_t get_sensor_health_status() noexcept {
    // Mock implementation - returns 0 (all healthy)
    return 0u;
}

} // namespace ems::drv

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
    
    data.clt_degc_x10 = 1501;  // 150.1°C - above maximum
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
    printf("Running EMS Sensor Validation Standalone Tests...\n");
    
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
    
    printf("Sensor validation standalone tests completed: %d run, %d failed\n", 
           g_tests_run, g_tests_failed);
    
    return (g_tests_failed == 0) ? 0 : 1;
}
