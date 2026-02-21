/**
 * @file engine_test_data.c
 * @brief Test data and fixtures for engine-related tests
 */

#include "engine_test_data.h"
#include <math.h>

// Engine configuration test data
const engine_config_test_t ENGINE_CONFIG_SMALL = {
    .displacement_cc = 1000,
    .cylinders = 4,
    .req_fuel_us = 8000,
    .injector_flow_cc_min = 240,
    .trigger_tdc_offset = 60,
    .max_rpm = 8000,
    .idle_rpm = 800
};

const engine_config_test_t ENGINE_CONFIG_MEDIUM = {
    .displacement_cc = 2000,
    .cylinders = 4,
    .req_fuel_us = 12000,
    .injector_flow_cc_min = 360,
    .trigger_tdc_offset = 60,
    .max_rpm = 7000,
    .idle_rpm = 750
};

const engine_config_test_t ENGINE_CONFIG_LARGE = {
    .displacement_cc = 3000,
    .cylinders = 4,
    .req_fuel_us = 16000,
    .injector_flow_cc_min = 480,
    .trigger_tdc_offset = 60,
    .max_rpm = 6500,
    .idle_rpm = 700
};

// Sensor test data
const sensor_data_test_t SENSOR_DATA_IDLE = {
    .map_kpa = 40.0f,
    .tps_percent = 5.0f,
    .iat_celsius = 25.0f,
    .clt_celsius = 85.0f,
    .battery_voltage = 13.8f,
    .lambda = 0.95f,
    .rpm = 800,
    .sync_acquired = true
};

const sensor_data_test_t SENSOR_DATA_CRUISE = {
    .map_kpa = 70.0f,
    .tps_percent = 25.0f,
    .iat_celsius = 30.0f,
    .clt_celsius = 90.0f,
    .battery_voltage = 14.2f,
    .lambda = 1.0f,
    .rpm = 2500,
    .sync_acquired = true
};

const sensor_data_test_t SENSOR_DATA_WOT = {
    .map_kpa = 100.0f,
    .tps_percent = 95.0f,
    .iat_celsius = 45.0f,
    .clt_celsius = 95.0f,
    .battery_voltage = 13.5f,
    .lambda = 0.85f,
    .rpm = 6000,
    .sync_acquired = true
};

// Trigger wheel test data
trigger_wheel_test_t TRIGGER_60_2_1000_RPM = {
    .tooth_count = 58,
    .missing_teeth = 2,
    .gap_tooth = 57,
    .teeth_per_rev = 58,
    .rpm = 1000,
    .tooth_period_us = 1000000 / (1000 * 58 / 60),  // 1034 us
    .max_jitter_us = 50,
    .min_events_per_rev = 58,
    .max_latency_us = 100
};

trigger_wheel_test_t TRIGGER_60_2_3000_RPM = {
    .tooth_count = 58,
    .missing_teeth = 2,
    .gap_tooth = 57,
    .teeth_per_rev = 58,
    .rpm = 3000,
    .tooth_period_us = 1000000 / (3000 * 58 / 60),  // 345 us
    .max_jitter_us = 30,
    .min_events_per_rev = 58,
    .max_latency_us = 50
};

trigger_wheel_test_t TRIGGER_60_2_6000_RPM = {
    .tooth_count = 58,
    .missing_teeth = 2,
    .gap_tooth = 57,
    .teeth_per_rev = 58,
    .rpm = 6000,
    .tooth_period_us = 1000000 / (6000 * 58 / 60),  // 172 us
    .max_jitter_us = 20,
    .min_events_per_rev = 58,
    .max_latency_us = 25
};

// Fuel calculation test data com tolerâncias adaptativas por RPM
const fuel_calc_test_t FUEL_CALC_TESTS[] = {
    {
        .ve_percent = 80.0f,
        .map_kpa = 40.0f,
        .iat_celsius = 25.0f,
        .rpm = 800,
        .req_fuel_us = 8000,
        .lambda_target = 1.0f,
        .expected_fuel_us = 6400.0f,  // 8000 * 0.8
        .tolerance_percent = 0.2f  // ±0.2% para marcha lenta
    },
    {
        .ve_percent = 85.0f,
        .map_kpa = 70.0f,
        .iat_celsius = 30.0f,
        .rpm = 1500,
        .req_fuel_us = 10000,
        .lambda_target = 1.0f,
        .expected_fuel_us = 8500.0f,  // 10000 * 0.85
        .tolerance_percent = 0.3f  // ±0.3% para baixa rotação
    },
    {
        .ve_percent = 75.0f,
        .map_kpa = 70.0f,
        .iat_celsius = 30.0f,
        .rpm = 2500,
        .req_fuel_us = 10000,
        .lambda_target = 1.0f,
        .expected_fuel_us = 7500.0f,  // 10000 * 0.75
        .tolerance_percent = 0.5f  // ±0.5% para média rotação
    },
    {
        .ve_percent = 95.0f,
        .map_kpa = 100.0f,
        .iat_celsius = 45.0f,
        .rpm = 6000,
        .req_fuel_us = 12000,
        .lambda_target = 0.85f,
        .expected_fuel_us = 13411.0f,  // 12000 * 0.95 / 0.85
        .tolerance_percent = 0.8f  // ±0.8% para alta rotação
    }
};

const size_t FUEL_CALC_TESTS_COUNT = sizeof(FUEL_CALC_TESTS) / sizeof(FUEL_CALC_TESTS[0]);

// Ignition timing test data com tolerâncias adaptativas por RPM
const ignition_timing_test_t IGNITION_TIMING_TESTS[] = {
    {
        .rpm = 800,
        .map_kpa = 40.0f,
        .iat_celsius = 25.0f,
        .base_timing_deg = 10.0f,
        .total_timing_deg = 10.0f,
        .tolerance_deg = 0.2f  // ±0.2° para marcha lenta
    },
    {
        .rpm = 1500,
        .map_kpa = 70.0f,
        .iat_celsius = 30.0f,
        .base_timing_deg = 15.0f,
        .total_timing_deg = 18.0f,  // Com avanço
        .tolerance_deg = 0.3f  // ±0.3° para baixa rotação
    },
    {
        .rpm = 2500,
        .map_kpa = 70.0f,
        .iat_celsius = 30.0f,
        .base_timing_deg = 25.0f,
        .total_timing_deg = 28.0f,  // Com avanço
        .tolerance_deg = 0.5f  // ±0.5° para média rotação
    },
    {
        .rpm = 6000,
        .map_kpa = 100.0f,
        .iat_celsius = 45.0f,
        .base_timing_deg = 20.0f,
        .total_timing_deg = 32.0f,  // Com avanço máximo
        .tolerance_deg = 0.8f  // ±0.8° para alta rotação
    }
};

const size_t IGNITION_TIMING_TESTS_COUNT = sizeof(IGNITION_TIMING_TESTS) / sizeof(IGNITION_TIMING_TESTS[0]);

// Performance test data com jitter adaptativo por RPM
const performance_test_t PERFORMANCE_TESTS[] = {
    {
        .rpm = 1000,
        .expected_tooth_period_us = 1034,
        .max_jitter_us = 15,     // ±15µs para marcha lenta
        .min_events_per_rev = 58,
        .max_latency_us = 100
    },
    {
        .rpm = 1500,
        .expected_tooth_period_us = 689,
        .max_jitter_us = 20,     // ±20µs para baixa rotação
        .min_events_per_rev = 58,
        .max_latency_us = 80
    },
    {
        .rpm = 2500,
        .expected_tooth_period_us = 413,
        .max_jitter_us = 30,     // ±30µs para média rotação
        .min_events_per_rev = 58,
        .max_latency_us = 50
    },
    {
        .rpm = 4000,
        .expected_tooth_period_us = 258,
        .max_jitter_us = 40,     // ±40µs para alta rotação
        .min_events_per_rev = 58,
        .max_latency_us = 35
    },
    {
        .rpm = 6000,
        .expected_tooth_period_us = 172,
        .max_jitter_us = 50,     // ±50µs para máxima rotação
        .min_events_per_rev = 58,
        .max_latency_us = 25
    }
};

const size_t PERFORMANCE_TESTS_COUNT = sizeof(PERFORMANCE_TESTS) / sizeof(PERFORMANCE_TESTS[0]);

// Test helper functions
void generate_trigger_wheel_times(trigger_wheel_test_t* data) {
    if (!data) return;
    
    uint32_t base_time = 1000;  // Start at 1ms
    uint32_t normal_period = data->tooth_period_us;
    
    for (uint32_t i = 0; i < data->tooth_count; i++) {
        if (i == data->gap_tooth) {
            // Gap period (3x normal for 2 missing teeth)
            data->tooth_times[i] = base_time + (normal_period * 3);
            base_time += (normal_period * 3);
        } else {
            data->tooth_times[i] = base_time + normal_period;
            base_time += normal_period;
        }
    }
}

float calculate_expected_fuel_pulse(const fuel_calc_test_t* test) {
    if (!test) return 0.0f;
    
    // Basic fuel calculation: req_fuel * VE / lambda_target
    float fuel_pulse = test->req_fuel_us * (test->ve_percent / 100.0f);
    
    if (test->lambda_target > 0.0f) {
        fuel_pulse /= test->lambda_target;
    }
    
    return fuel_pulse;
}

float calculate_expected_timing(const ignition_timing_test_t* test) {
    if (!test) return 0.0f;
    
    // Simple timing calculation based on RPM and MAP
    // In real implementation, this would use complex timing tables
    float timing = test->base_timing_deg;
    
    // Add advance based on RPM (simplified)
    if (test->rpm > 2000) {
        timing += (test->rpm - 2000) * 0.002f;  // 0.002 deg per RPM above 2000
    }
    
    // Add MAP-based advance (simplified)
    if (test->map_kpa > 50.0f) {
        timing += (test->map_kpa - 50.0f) * 0.1f;  // 0.1 deg per kPa above 50
    }
    
    return timing;
}

bool validate_timing_performance(uint32_t actual_us, uint32_t expected_us, uint32_t tolerance_us) {
    if (expected_us == 0) return false;
    
    uint32_t diff = (actual_us > expected_us) ? (actual_us - expected_us) : (expected_us - actual_us);
    return diff <= tolerance_us;
}
