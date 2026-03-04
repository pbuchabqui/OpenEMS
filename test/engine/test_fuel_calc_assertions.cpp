/**
 * @file test/engine/test_fuel_calc_assertions.cpp
 * @brief Host unit tests for fuel calculation assertions (CRITICAL FIX)
 * 
 * Tests the input validation and assertions in fuel calculation functions
 * to ensure safety-critical parameter validation works correctly.
 */

#include <cstdint>
#include <cstdio>
#include <cassert>

#define EMS_HOST_TEST 1
#include "engine/fuel_calc.h"
#include "engine/table3d.h"

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

#define TEST_ASSERT_IN_RANGE(val, min, max) do { \
    ++g_tests_run; \
    if ((val) < (min) || (val) > (max)) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: value %u not in range [%u,%u]\n", __FILE__, __LINE__, \
               (unsigned)(val), (unsigned)(min), (unsigned)(max)); \
    } \
} while (0)

// Mock table3d functions for testing
uint8_t mock_table3d_lookup_u8(uint8_t table[][16], const uint16_t* rpm_axis, 
                              const uint16_t* load_axis, uint16_t rpm, uint16_t load) {
    // Simple mock implementation
    if (rpm == 1000 && load == 100) return 80;
    if (rpm == 2000 && load == 100) return 85;
    if (rpm == 1000 && load == 200) return 90;
    return 75; // default
}

} // namespace

// =============================================================================
// Test: VE Calculation Validation
// =============================================================================

void test_get_ve_valid_inputs() {
    // Test valid RPM and MAP combinations
    uint8_t ve = ems::engine::get_ve(1000, 100);  // 1000 RPM, 100 kPa
    TEST_ASSERT_IN_RANGE(ve, 50, 110);  // VE should be in valid range
    
    ve = ems::engine::get_ve(2000, 100);  // 2000 RPM, 100 kPa
    TEST_ASSERT_IN_RANGE(ve, 50, 110);
    
    ve = ems::engine::get_ve(1000, 200);  // 1000 RPM, 200 kPa
    TEST_ASSERT_IN_RANGE(ve, 50, 110);
}

void test_get_ve_boundary_values() {
    // Test boundary RPM values
    uint8_t ve = ems::engine::get_ve(0, 100);      // 0 RPM (minimum)
    TEST_ASSERT_IN_RANGE(ve, 50, 110);
    
    ve = ems::engine::get_ve(20000, 100);  // 2000 RPM (maximum ×10)
    TEST_ASSERT_IN_RANGE(ve, 50, 110);
    
    // Test boundary MAP values
    ve = ems::engine::get_ve(1000, 10);     // 10 kPa (minimum)
    TEST_ASSERT_IN_RANGE(ve, 50, 110);
    
    ve = ems::engine::get_ve(1000, 250);    // 250 kPa (maximum)
    TEST_ASSERT_IN_RANGE(ve, 50, 110);
}

// =============================================================================
// Test: Base Pulse Width Calculation Validation
// =============================================================================

void test_calc_base_pw_us_valid_inputs() {
    // Test with valid parameters
    uint32_t pw = ems::engine::calc_base_pw_us(8000, 80, 100, 100);
    TEST_ASSERT_IN_RANGE(pw, 1, 100000);  // Should be reasonable pulse width
    
    pw = ems::engine::calc_base_pw_us(12000, 90, 150, 100);
    TEST_ASSERT_IN_RANGE(pw, 1, 100000);
    
    pw = ems::engine::calc_base_pw_us(5000, 70, 50, 100);
    TEST_ASSERT_IN_RANGE(pw, 1, 100000);
}

void test_calc_base_pw_us_edge_cases() {
    // Test with minimum VE (should return 0)
    uint32_t pw = ems::engine::calc_base_pw_us(8000, 0, 100, 100);
    TEST_ASSERT_EQ_U32(0U, pw);
    
    // Test with zero MAP reference (should return 0)
    pw = ems::engine::calc_base_pw_us(8000, 80, 100, 0);
    TEST_ASSERT_EQ_U32(0U, pw);
    
    // Test with reasonable fuel request
    pw = ems::engine::calc_base_pw_us(50000, 110, 250, 100);  // Max reasonable values
    TEST_ASSERT_IN_RANGE(pw, 1, 100000);
}

void test_calc_base_pw_us_result_validation() {
    // Test that results are within expected bounds
    uint32_t pw = ems::engine::calc_base_pw_us(8000, 80, 100, 100);
    
    // At 100 kPa reference, 80% VE, 8ms req fuel:
    // Expected: 8ms * 0.80 * 100/100 = 6.4ms = 6400μs
    TEST_ASSERT_IN_RANGE(pw, 5000, 8000);  // Allow some tolerance
    
    // Test high MAP condition
    pw = ems::engine::calc_base_pw_us(8000, 80, 200, 100);
    TEST_ASSERT_IN_RANGE(pw, 10000, 15000);  // Should be roughly double
}

// =============================================================================
// Test: Temperature Correction Validation
// =============================================================================

void test_corr_clt_valid_temperatures() {
    // Test valid CLT temperatures
    uint16_t corr = ems::engine::corr_clt(-400);   // -40°C (minimum)
    TEST_ASSERT_IN_RANGE(corr, 200, 400);  // Should be reasonable correction
    
    corr = ems::engine::corr_clt(800);    // 80°C (normal operating)
    TEST_ASSERT_IN_RANGE(corr, 200, 400);
    
    corr = ems::engine::corr_clt(1500);   // 150°C (maximum)
    TEST_ASSERT_IN_RANGE(corr, 200, 400);
}

void test_corr_iat_valid_temperatures() {
    // Test valid IAT temperatures
    uint16_t corr = ems::engine::corr_iat(-200);   // -20°C
    TEST_ASSERT_IN_RANGE(corr, 200, 300);
    
    corr = ems::engine::corr_iat(300);    // 30°C (normal)
    TEST_ASSERT_IN_RANGE(corr, 200, 300);
    
    corr = ems::engine::corr_iat(1200);   // 120°C (high)
    TEST_ASSERT_IN_RANGE(corr, 200, 300);
}

void test_corr_warmup_valid_temperatures() {
    // Test warmup correction with valid temperatures
    uint16_t corr = ems::engine::corr_warmup(-400);  // -40°C (cold start)
    TEST_ASSERT_IN_RANGE(corr, 250, 450);  // Should be high correction
    
    corr = ems::engine::corr_warmup(800);   // 80°C (warm)
    TEST_ASSERT_IN_RANGE(corr, 250, 300);   // Should be normal correction
    
    corr = ems::engine::corr_warmup(1500);  // 150°C (very hot)
    TEST_ASSERT_IN_RANGE(corr, 250, 300);   // Should be minimal correction
}

// =============================================================================
// Test: Voltage Correction Validation
// =============================================================================

void test_corr_vbatt_valid_voltages() {
    // Test valid battery voltages
    uint16_t corr = ems::engine::corr_vbatt(6000);   // 6.0V (minimum)
    TEST_ASSERT_IN_RANGE(corr, 1000, 1500);  // Should be high dead time
    
    corr = ems::engine::corr_vbatt(12000);  // 12.0V (normal)
    TEST_ASSERT_IN_RANGE(corr, 800, 1000);   // Should be normal dead time
    
    corr = ems::engine::corr_vbatt(18000);  // 18.0V (maximum)
    TEST_ASSERT_IN_RANGE(corr, 600, 800);   // Should be low dead time
}

void test_corr_vbatt_clamping() {
    // Test that voltage correction handles out-of-range inputs gracefully
    uint16_t corr_low = ems::engine::corr_vbatt(5000);   // Below minimum
    uint16_t corr_normal = ems::engine::corr_vbatt(7000); // At minimum
    
    // Should be clamped to minimum behavior
    TEST_ASSERT_IN_RANGE(corr_low, 1000, 1500);
    TEST_ASSERT_IN_RANGE(corr_normal, 1000, 1500);
    
    uint16_t corr_high = ems::engine::corr_vbatt(20000);  // Above maximum
    uint16_t corr_max = ems::engine::corr_vbatt(17000);   // At maximum
    
    // Should be clamped to maximum behavior
    TEST_ASSERT_IN_RANGE(corr_high, 600, 800);
    TEST_ASSERT_IN_RANGE(corr_max, 600, 800);
}

// =============================================================================
// Test: Final Pulse Width Calculation
// =============================================================================

void test_calc_final_pw_us_valid_inputs() {
    // Test with valid correction factors
    uint32_t final_pw = ems::engine::calc_final_pw_us(10000, 256, 256, 1000);
    TEST_ASSERT_IN_RANGE(final_pw, 10000, 12000);  // Should be close to base + dead time
    
    final_pw = ems::engine::calc_final_pw_us(8000, 300, 280, 1200);
    TEST_ASSERT_IN_RANGE(final_pw, 8000, 12000);
    
    final_pw = ems::engine::calc_final_pw_us(15000, 200, 220, 800);
    TEST_ASSERT_IN_RANGE(final_pw, 10000, 12000);
}

void test_calc_final_pw_us_edge_cases() {
    // Test with minimum values
    uint32_t final_pw = ems::engine::calc_final_pw_us(1000, 200, 200, 600);
    TEST_ASSERT_IN_RANGE(final_pw, 1000, 2000);
    
    // Test with maximum reasonable values
    final_pw = ems::engine::calc_final_pw_us(50000, 400, 400, 1500);
    TEST_ASSERT_IN_RANGE(final_pw, 120000, 130000);
}

// =============================================================================
// Test: Integration Scenarios
// =============================================================================

void test_fuel_calculation_integration() {
    // Simulate a complete fuel calculation scenario
    uint16_t rpm_x10 = 1500;      // 150 RPM
    uint16_t map_kpa = 100;       // 100 kPa
    int16_t clt_x10 = 800;       // 80°C
    int16_t iat_x10 = 300;       // 30°C
    uint16_t vbatt_mv = 12000;    // 12.0V
    uint16_t req_fuel_us = 8000;  // 8ms
    
    // Calculate VE
    uint8_t ve = ems::engine::get_ve(rpm_x10, map_kpa);
    TEST_ASSERT_IN_RANGE(ve, 50, 110);
    
    // Calculate base pulse width
    uint32_t base_pw = ems::engine::calc_base_pw_us(req_fuel_us, ve, map_kpa, 100);
    TEST_ASSERT_IN_RANGE(base_pw, 1, 50000);
    
    // Get corrections
    uint16_t clt_corr = ems::engine::corr_clt(clt_x10);
    uint16_t iat_corr = ems::engine::corr_iat(iat_x10);
    uint16_t vbatt_corr = ems::engine::corr_vbatt(vbatt_mv);
    
    TEST_ASSERT_IN_RANGE(clt_corr, 200, 400);
    TEST_ASSERT_IN_RANGE(iat_corr, 200, 300);
    TEST_ASSERT_IN_RANGE(vbatt_corr, 600, 1500);
    
    // Calculate final pulse width
    uint32_t final_pw = ems::engine::calc_final_pw_us(base_pw, clt_corr, iat_corr, vbatt_corr);
    TEST_ASSERT_IN_RANGE(final_pw, base_pw, base_pw + vbatt_corr + 1000);  // Reasonable increase
}

void test_fuel_calculation_error_conditions() {
    // Test error handling in fuel calculations
    
    // Test with zero VE (should return 0)
    uint32_t pw = ems::engine::calc_base_pw_us(8000, 0, 100, 100);
    TEST_ASSERT_EQ_U32(0U, pw);
    
    // Test with zero MAP reference (should return 0)
    pw = ems::engine::calc_base_pw_us(8000, 80, 100, 0);
    TEST_ASSERT_EQ_U32(0U, pw);
    
    // Test with very high corrections (should still be reasonable)
    uint32_t final_pw = ems::engine::calc_final_pw_us(10000, 500, 500, 2000);
    TEST_ASSERT_IN_RANGE(final_pw, 10000, 50000);  // Should not overflow
}

// =============================================================================
// Main Test Runner
// =============================================================================

int main() {
    printf("Running EMS Fuel Calculation Assertions Tests...\n");
    
    // VE calculation tests
    test_get_ve_valid_inputs();
    test_get_ve_boundary_values();
    
    // Base pulse width tests
    test_calc_base_pw_us_valid_inputs();
    test_calc_base_pw_us_edge_cases();
    test_calc_base_pw_us_result_validation();
    
    // Temperature correction tests
    test_corr_clt_valid_temperatures();
    test_corr_iat_valid_temperatures();
    test_corr_warmup_valid_temperatures();
    
    // Voltage correction tests
    test_corr_vbatt_valid_voltages();
    test_corr_vbatt_clamping();
    
    // Final pulse width tests
    test_calc_final_pw_us_valid_inputs();
    test_calc_final_pw_us_edge_cases();
    
    // Integration tests
    test_fuel_calculation_integration();
    test_fuel_calculation_error_conditions();
    
    printf("Fuel calculation assertions tests completed: %d run, %d failed\n", 
           g_tests_run, g_tests_failed);
    
    return (g_tests_failed == 0) ? 0 : 1;
}
