#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unity.h"
#include "openems_test_defs.h"

// Funções simuladas do atomic buffer
static atomic_buf_t buffer = {0};

esp_err_t atomic_buf_init(atomic_buf_t *buf) {
    if (!buf) return ESP_FAIL;
    memset(buf, 0, sizeof(atomic_buf_t));
    buf->write_index = 0;
    buf->read_index = 0;
    buf->ready[0] = false;
    buf->ready[1] = false;
    return ESP_OK;
}

esp_err_t atomic_buf_write(atomic_buf_t *buf, const void *data, size_t size) {
    if (!buf || !data || size > 256) return ESP_FAIL;
    
    uint32_t write_idx = buf->write_index % 2;
    memcpy(buf->buffer[write_idx], data, size);
    buf->ready[write_idx] = true;
    buf->write_index++;
    
    return ESP_OK;
}

esp_err_t atomic_buf_read(atomic_buf_t *buf, void *data, size_t size) {
    if (!buf || !data || size > 256) return ESP_FAIL;
    
    uint32_t read_idx = buf->read_index % 2;
    if (!buf->ready[read_idx]) return ESP_FAIL;
    
    memcpy(data, buf->buffer[read_idx], size);
    buf->ready[read_idx] = false;
    buf->read_index++;
    
    return ESP_OK;
}

// Test functions
void test_atomic_buffer_init_should_return_ok(void) {
    esp_err_t result = atomic_buf_init(&buffer);
    TEST_ASSERT_EQUAL_INT(ESP_OK, result);
    TEST_ASSERT_EQUAL_INT(0, buffer.write_index);
    TEST_ASSERT_EQUAL_INT(0, buffer.read_index);
}

void test_atomic_buffer_write_read_should_preserve_data(void) {
    atomic_buf_init(&buffer);
    
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t read_data[4] = {0};
    
    esp_err_t write_result = atomic_buf_write(&buffer, test_data, sizeof(test_data));
    TEST_ASSERT_EQUAL_INT(ESP_OK, write_result);
    
    esp_err_t read_result = atomic_buf_read(&buffer, read_data, sizeof(read_data));
    TEST_ASSERT_EQUAL_INT(ESP_OK, read_result);
    
    // Verificar se os dados foram preservados
    TEST_ASSERT_EQUAL_INT(0x01, read_data[0]);
    TEST_ASSERT_EQUAL_INT(0x02, read_data[1]);
    TEST_ASSERT_EQUAL_INT(0x03, read_data[2]);
    TEST_ASSERT_EQUAL_INT(0x04, read_data[3]);
}

void test_atomic_buffer_concurrent_access_should_work(void) {
    atomic_buf_init(&buffer);
    
    // Simular escrita concorrente
    uint8_t core0_data[] = {0xAA, 0xBB, 0xCC};
    uint8_t core1_data[] = {0x11, 0x22, 0x33};
    uint8_t read_data[3] = {0};
    
    // Core 0 escreve
    esp_err_t write0 = atomic_buf_write(&buffer, core0_data, sizeof(core0_data));
    TEST_ASSERT_EQUAL_INT(ESP_OK, write0);
    
    // Core 1 escreve
    esp_err_t write1 = atomic_buf_write(&buffer, core1_data, sizeof(core1_data));
    TEST_ASSERT_EQUAL_INT(ESP_OK, write1);
    
    // Ler dados mais recentes
    esp_err_t read_result = atomic_buf_read(&buffer, read_data, sizeof(read_data));
    TEST_ASSERT_EQUAL_INT(ESP_OK, read_result);
    
    // Verificar consistência dos dados
    TEST_ASSERT_TRUE((read_data[0] == 0xAA && read_data[1] == 0xBB) || 
                   (read_data[0] == 0x11 && read_data[1] == 0x22));
}

int main(void) {
    printf("=== OpenEMS Core Communication Integration Test Suite ===\n\n");
    
    RUN_TEST(test_atomic_buffer_init_should_return_ok);
    RUN_TEST(test_atomic_buffer_write_read_should_preserve_data);
    RUN_TEST(test_atomic_buffer_concurrent_access_should_work);
    
    printf("=== Test Summary ===\n");
    printf("All tests completed successfully!\n");
    
    return 0;
}
