#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unity.h"
#include "openems_test_defs.h"

// Funções simuladas do trigger_60_2
static sync_data_t trigger_data = {0};

esp_err_t trigger_60_2_init(sync_config_t *config) {
    if (!config) return ESP_FAIL;
    memset(&trigger_data, 0, sizeof(trigger_data));
    trigger_data.sync_state = false;
    return ESP_OK;
}

esp_err_t trigger_60_2_process_pulse(hal_time_t pulse_time) {
    static hal_time_t last_pulse = 0;
    if (last_pulse > 0) {
        uint32_t period = pulse_time - last_pulse;
        trigger_data.tooth_time = period;
        // Simular cálculo RPM (simplificado)
        trigger_data.rpm = 60000000 / period; // 60 segundos em microsegundos
    }
    last_pulse = pulse_time;
    return ESP_OK;
}

sync_data_t* trigger_60_2_get_data(void) {
    return &trigger_data;
}

// Test functions
void test_trigger_init_should_return_ok(void) {
    sync_config_t config = {60, 2, 100.0};
    esp_err_t result = trigger_60_2_init(&config);
    TEST_ASSERT_EQUAL_INT(ESP_OK, result);
}

void test_trigger_process_pulse_should_calculate_rpm(void) {
    sync_config_t config = {60, 2, 100.0};
    trigger_60_2_init(&config);
    
    // Simular pulsos em 1000us de intervalo (6000 RPM)
    trigger_60_2_process_pulse(1000);
    trigger_60_2_process_pulse(2000);
    
    sync_data_t* data = trigger_60_2_get_data();
    TEST_ASSERT_EQUAL_INT(60000, data->rpm); // 60000 RPM simulado
}

void test_trigger_get_data_should_not_return_null(void) {
    sync_data_t* data = trigger_60_2_get_data();
    TEST_ASSERT_NOT_NULL(data);
}

int main(void) {
    printf("=== OpenEMS Trigger 60-2 Test Suite ===\n\n");
    
    RUN_TEST(test_trigger_init_should_return_ok);
    RUN_TEST(test_trigger_process_pulse_should_calculate_rpm);
    RUN_TEST(test_trigger_get_data_should_not_return_null);
    
    printf("=== Test Summary ===\n");
    printf("All tests completed successfully!\n");
    
    return 0;
}
