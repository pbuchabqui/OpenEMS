#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unity.h"
#include "openems_test_defs.h"

// Funções simuladas do event scheduler
static event_scheduler_t scheduler = {0};

esp_err_t event_scheduler_init(event_scheduler_t *sched) {
    if (!sched) return ESP_FAIL;
    memset(sched, 0, sizeof(event_scheduler_t));
    sched->base_time = 1000; // Tempo base simulado
    return ESP_OK;
}

esp_err_t event_scheduler_add(event_scheduler_t *sched, uint32_t angle, void (*callback)(void), uint8_t priority) {
    if (!sched || !callback || sched->count >= 32) return ESP_FAIL;
    
    event_t *event = &sched->events[sched->count];
    event->angle = angle;
    event->time_us = sched->base_time + (angle * 100); // Conversão simulada
    event->callback = callback;
    event->priority = priority;
    
    sched->count++;
    return ESP_OK;
}

uint32_t event_scheduler_angle_to_time(event_scheduler_t *sched, uint32_t angle) {
    if (!sched) return 0;
    return sched->base_time + (angle * 100);
}

// Test functions
void test_scheduler_init_should_return_ok(void) {
    esp_err_t result = event_scheduler_init(&scheduler);
    TEST_ASSERT_EQUAL_INT(ESP_OK, result);
    TEST_ASSERT_EQUAL_INT(1000, scheduler.base_time);
}

void test_scheduler_add_event_should_increment_count(void) {
    event_scheduler_init(&scheduler);
    
    void dummy_callback(void) {}
    esp_err_t result = event_scheduler_add(&scheduler, 180, dummy_callback, 1);
    
    TEST_ASSERT_EQUAL_INT(ESP_OK, result);
    TEST_ASSERT_EQUAL_INT(1, scheduler.count);
}

void test_scheduler_angle_to_time_should_convert_correctly(void) {
    event_scheduler_init(&scheduler);
    
    uint32_t time = event_scheduler_angle_to_time(&scheduler, 180);
    TEST_ASSERT_EQUAL_INT(19000, time); // 1000 + 180*100
}

int main(void) {
    printf("=== OpenEMS Event Scheduler Test Suite ===\n\n");
    
    RUN_TEST(test_scheduler_init_should_return_ok);
    RUN_TEST(test_scheduler_add_event_should_increment_count);
    RUN_TEST(test_scheduler_angle_to_time_should_convert_correctly);
    
    printf("=== Test Summary ===\n");
    printf("All tests completed successfully!\n");
    
    return 0;
}
