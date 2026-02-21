/**
 * @file espnow_compression.c
 * @brief Implementação do módulo de compressão para ESP-NOW
 */

#include "espnow_compression.h"
#include "esp_log.h"
#include "hal/hal_timer.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char* TAG = "ESPNOW_COMPRESS";

//=============================================================================
// Funções Estáticas Internas
//=============================================================================

static uint16_t espnow_calculate_checksum(const uint8_t *data, uint16_t size);
static esp_err_t espnow_build_huffman_tree(espnow_compress_context_t *ctx,
                                           const uint8_t *data,
                                           uint16_t size);
static void espnow_generate_huffman_codes(espnow_compress_context_t *ctx);
static esp_err_t espnow_quantize_value(float value, uint8_t bits, uint16_t *quantized);

//=============================================================================
// Funções de Inicialização
//=============================================================================

esp_err_t espnow_compress_init(espnow_compress_context_t *ctx,
                               const espnow_compress_config_t *config) {
    if (ctx == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limpar contexto
    memset(ctx, 0, sizeof(espnow_compress_context_t));
    
    // Copiar configuração
    ctx->config = *config;
    
    // Validar configuração
    if (config->level == 0 || config->level > 9) {
        ESP_LOGE(TAG, "Invalid compression level: %d", config->level);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Inicializar tabelas padrão
    for (int i = 0; i < 256; i++) {
        ctx->delta_table[i] = i;
        ctx->quantization_table[i] = (float)i;
        ctx->huffman_table[i].symbol = i;
        ctx->huffman_table[i].frequency = 1;
        ctx->huffman_table[i].code = i;
        ctx->huffman_table[i].code_length = 8;
    }
    
    ctx->initialized = true;
    ctx->adaptive_mode = config->enable_adaptive;
    ctx->current_level = config->level;
    
    ESP_LOGI(TAG, "ESP-NOW compression initialized:");
    ESP_LOGI(TAG, "  Type: %d", config->type);
    ESP_LOGI(TAG, "  Level: %d", config->level);
    ESP_LOGI(TAG, "  Quantization bits: %d", config->quantization_bits);
    ESP_LOGI(TAG, "  Adaptive mode: %s", config->enable_adaptive ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  SIMD optimization: %s", config->use_simd ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t espnow_compress_deinit(espnow_compress_context_t *ctx) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Limpar contexto
    memset(ctx, 0, sizeof(espnow_compress_context_t));
    
    ESP_LOGI(TAG, "ESP-NOW compression deinitialized");
    return ESP_OK;
}

//=============================================================================
// Funções de Compressão Principal
//=============================================================================

esp_err_t espnow_compress_data(espnow_compress_context_t *ctx,
                               const uint8_t *input,
                               uint16_t input_size,
                               uint8_t *output,
                               uint16_t output_size,
                               espnow_compress_result_t *result) {
    if (ctx == NULL || !ctx->initialized || 
        input == NULL || output == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (input_size == 0) {
        result->success = false;
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t start_time = HAL_Time_us();
    
    // Verificar se vale a pena comprimir
    if (input_size < ctx->config.min_size) {
        result->type = ESPNOW_COMPRESS_NONE;
        result->original_size = input_size;
        result->compressed_size = input_size;
        result->compression_ratio = 1.0f;
        result->success = true;
        
        // Copiar dados sem compressão
        if (input_size <= output_size) {
            memcpy(output, input, input_size);
        } else {
            result->success = false;
            return ESP_ERR_INVALID_SIZE;
        }
        
        result->compression_time_us = (uint32_t)(HAL_Time_us() - start_time);
        return ESP_OK;
    }
    
    // Selecionar melhor algoritmo se modo adaptativo
    espnow_compress_type_t compress_type = ctx->config.type;
    if (ctx->adaptive_mode) {
        esp_err_t ret = espnow_select_best_compression(ctx, input, input_size, &compress_type);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to select best compression, using default");
            compress_type = ctx->config.type;
        }
    }
    
    // Executar compressão
    uint16_t compressed_size = 0;
    esp_err_t ret = ESP_OK;
    
    switch (compress_type) {
        case ESPNOW_COMPRESS_DELTA:
            ret = espnow_compress_delta(ctx, input, input_size, output, output_size, &compressed_size);
            break;
            
        case ESPNOW_COMPRESS_HUFFMAN:
            ret = espnow_compress_huffman(ctx, input, input_size, output, output_size, &compressed_size);
            break;
            
        case ESPNOW_COMPRESS_QUANTIZED:
            // Para dados quantizados, precisamos converter para float primeiro
            if (input_size % 4 != 0) {
                ret = ESP_ERR_INVALID_ARG;
            } else {
                uint16_t num_samples = input_size / 4;
                ret = espnow_compress_quantized(ctx, (const float*)input, num_samples, 
                                                 output, output_size, &compressed_size);
            }
            break;
            
        case ESPNOW_COMPRESS_HYBRID:
            ret = espnow_compress_hybrid(ctx, input, input_size, output, output_size, &compressed_size);
            break;
            
        default:
            // Sem compressão
            if (input_size <= output_size) {
                memcpy(output, input, input_size);
                compressed_size = input_size;
            } else {
                ret = ESP_ERR_INVALID_SIZE;
            }
            break;
    }
    
    // Preencher resultado
    result->type = compress_type;
    result->original_size = input_size;
    result->compressed_size = compressed_size;
    result->compression_ratio = (float)input_size / (float)compressed_size;
    result->success = (ret == ESP_OK && compressed_size > 0 && compressed_size <= output_size);
    result->compression_time_us = (uint32_t)(HAL_Time_us() - start_time);
    
    // Atualizar estatísticas
    if (result->success) {
        ctx->total_compressed += compressed_size;
        ctx->total_decompressed += input_size;
        ctx->compression_time_us += result->compression_time_us;
        ctx->compression_ratio = (float)ctx->total_decompressed / (float)ctx->total_compressed;
        ctx->avg_compression_time = (float)ctx->compression_time_us / ctx->sample_count;
        ctx->sample_count++;
    }
    
    ESP_LOGD(TAG, "Compression: %d->%d bytes, ratio=%.2f, time=%d us, type=%d", 
              input_size, compressed_size, result->compression_ratio, 
              result->compression_time_us, compress_type);
    
    return ret;
}

esp_err_t espnow_compress_engine_status(espnow_compress_context_t *ctx,
                                         const espnow_engine_status_t *status,
                                         uint8_t *output,
                                         uint16_t output_size,
                                         espnow_compress_result_t *result) {
    if (ctx == NULL || !ctx->initialized || status == NULL || output == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Converter estrutura para array de bytes
    uint8_t input_buffer[sizeof(espnow_engine_status_t)];
    memcpy(input_buffer, status, sizeof(espnow_engine_status_t));
    
    // Aplicar compressão delta (ótima para dados numéricos sequenciais)
    espnow_compress_config_t delta_config = ctx->config;
    delta_config.type = ESPNOW_COMPRESS_DELTA;
    delta_config.level = 4; // Nível moderado para dados em tempo real
    
    espnow_compress_context_t delta_ctx;
    esp_err_t ret = espnow_compress_init(&delta_ctx, &delta_config);
    if (ret != ESP_OK) return ret;
    
    ret = espnow_compress_data(&delta_ctx, input_buffer, sizeof(espnow_engine_status_t),
                               output, output_size, result);
    
    espnow_compress_deinit(&delta_ctx);
    
    return ret;
}

esp_err_t espnow_compress_sensor_data(espnow_compress_context_t *ctx,
                                      const espnow_sensor_data_t *sensor_data,
                                      uint8_t *output,
                                      uint16_t output_size,
                                      espnow_compress_result_t *result) {
    if (ctx == NULL || !ctx->initialized || sensor_data == NULL || output == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Converter estrutura para array de bytes
    uint8_t input_buffer[sizeof(espnow_sensor_data_t)];
    memcpy(input_buffer, sensor_data, sizeof(espnow_sensor_data_t));
    
    // Usar compressão quantizada para dados de sensores (valores ADC)
    espnow_compress_config_t quant_config = ctx->config;
    quant_config.type = ESPNOW_COMPRESS_QUANTIZED;
    quant_config.quantization_bits = 10; // 10 bits é suficiente para ADC de 12 bits
    
    espnow_compress_context_t quant_ctx;
    esp_err_t ret = espnow_compress_init(&quant_ctx, &quant_config);
    if (ret != ESP_OK) return ret;
    
    ret = espnow_compress_data(&quant_ctx, input_buffer, sizeof(espnow_sensor_data_t),
                               output, output_size, result);
    
    espnow_compress_deinit(&quant_ctx);
    
    return ret;
}

//=============================================================================
// Algoritmos de Compressão
//=============================================================================

esp_err_t espnow_compress_delta(espnow_compress_context_t *ctx,
                                const uint8_t *input,
                                uint16_t input_size,
                                uint8_t *output,
                                uint16_t output_size,
                                uint16_t *compressed_size) {
    if (ctx == NULL || input == NULL || output == NULL || compressed_size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (input_size == 0 || output_size < input_size + 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Formato: [primeiro_byte][delta1][delta2]...[checksum]
    uint16_t out_index = 0;
    
    // Primeiro byte original
    output[out_index++] = input[0];
    
    // Calcular deltas
    uint8_t last_value = input[0];
    for (uint16_t i = 1; i < input_size; i++) {
        uint8_t delta = input[i] - last_value;
        output[out_index++] = delta;
        last_value = input[i];
        
        if (out_index >= output_size - 2) {
            return ESP_ERR_INVALID_SIZE; // Buffer cheio
        }
    }
    
    // Adicionar checksum
    uint16_t checksum = espnow_calculate_checksum(output, out_index);
    output[out_index++] = (checksum >> 8) & 0xFF;
    output[out_index++] = checksum & 0xFF;
    
    *compressed_size = out_index;
    
    ESP_LOGV(TAG, "Delta compression: %d->%d bytes", input_size, *compressed_size);
    return ESP_OK;
}

esp_err_t espnow_compress_huffman(espnow_compress_context_t *ctx,
                                  const uint8_t *input,
                                  uint16_t input_size,
                                  uint8_t *output,
                                  uint16_t output_size,
                                  uint16_t *compressed_size) {
    if (ctx == NULL || input == NULL || output == NULL || compressed_size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (input_size == 0 || output_size < input_size + 4) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Construir árvore de frequências
    esp_err_t ret = espnow_build_huffman_tree(ctx, input, input_size);
    if (ret != ESP_OK) return ret;
    
    // Gerar códigos Huffman
    espnow_generate_huffman_codes(ctx);
    
    // Comprimir usando códigos Huffman
    uint16_t out_index = 0;
    uint32_t bit_buffer = 0;
    uint8_t bits_in_buffer = 0;
    
    // Header: tamanho original + checksum da tabela
    output[out_index++] = (input_size >> 8) & 0xFF;
    output[out_index++] = input_size & 0xFF;
    
    uint16_t table_checksum = espnow_calculate_checksum((uint8_t*)ctx->huffman_table, 
                                                      sizeof(ctx->huffman_table));
    output[out_index++] = (table_checksum >> 8) & 0xFF;
    output[out_index++] = table_checksum & 0xFF;
    
    // Comprimir dados
    for (uint16_t i = 0; i < input_size; i++) {
        uint8_t symbol = input[i];
        espnow_huffman_table_t *entry = &ctx->huffman_table[symbol];
        
        // Adicionar código ao buffer de bits
        bit_buffer = (bit_buffer << entry->code_length) | entry->code;
        bits_in_buffer += entry->code_length;
        
        // Escrever bytes completos
        while (bits_in_buffer >= 8) {
            if (out_index >= output_size - 1) {
                return ESP_ERR_INVALID_SIZE;
            }
            output[out_index++] = (bit_buffer >> (bits_in_buffer - 8)) & 0xFF;
            bits_in_buffer -= 8;
        }
    }
    
    // Escrever bits restantes
    if (bits_in_buffer > 0) {
        if (out_index >= output_size) {
            return ESP_ERR_INVALID_SIZE;
        }
        output[out_index++] = (bit_buffer << (8 - bits_in_buffer)) & 0xFF;
    }
    
    *compressed_size = out_index;
    
    ESP_LOGV(TAG, "Huffman compression: %d->%d bytes", input_size, *compressed_size);
    return ESP_OK;
}

esp_err_t espnow_compress_quantized(espnow_compress_context_t *ctx,
                                    const float *input,
                                    uint16_t input_size,
                                    uint8_t *output,
                                    uint16_t output_size,
                                    uint16_t *compressed_size) {
    if (ctx == NULL || input == NULL || output == NULL || compressed_size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (input_size == 0 || output_size < input_size * 2 + 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    uint16_t out_index = 0;
    uint8_t bits = ctx->config.quantization_bits;
    
    // Header: número de amostras + bits de quantização
    output[out_index++] = (input_size >> 8) & 0xFF;
    output[out_index++] = input_size & 0xFF;
    output[out_index++] = bits;
    
    // Quantizar cada valor
    for (uint16_t i = 0; i < input_size; i++) {
        uint16_t quantized;
        esp_err_t ret = espnow_quantize_value(input[i], bits, &quantized);
        if (ret != ESP_OK) return ret;
        
        // Armazenar como 2 bytes (big-endian)
        if (out_index + 1 >= output_size) {
            return ESP_ERR_INVALID_SIZE;
        }
        output[out_index++] = (quantized >> 8) & 0xFF;
        output[out_index++] = quantized & 0xFF;
    }
    
    *compressed_size = out_index;
    
    ESP_LOGV(TAG, "Quantized compression: %d floats->%d bytes", input_size, *compressed_size);
    return ESP_OK;
}

esp_err_t espnow_compress_hybrid(espnow_compress_context_t *ctx,
                                 const uint8_t *input,
                                 uint16_t input_size,
                                 uint8_t *output,
                                 uint16_t output_size,
                                 uint16_t *compressed_size) {
    if (ctx == NULL || input == NULL || output == NULL || compressed_size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Compressão híbrida: Delta + Huffman
    uint8_t delta_buffer[ESPNOW_DECOMPRESS_BUFFER_SIZE];
    uint16_t delta_size;
    
    // Primeiro aplicar compressão delta
    esp_err_t ret = espnow_compress_delta(ctx, input, input_size, 
                                          delta_buffer, sizeof(delta_buffer), &delta_size);
    if (ret != ESP_OK) return ret;
    
    // Depois aplicar Huffman no resultado delta
    ret = espnow_compress_huffman(ctx, delta_buffer, delta_size,
                                  output, output_size, compressed_size);
    
    return ret;
}

//=============================================================================
// Funções de Descompressão
//=============================================================================

esp_err_t espnow_decompress_data(espnow_compress_context_t *ctx,
                                 const uint8_t *input,
                                 uint16_t input_size,
                                 uint8_t *output,
                                 uint16_t output_size,
                                 espnow_decompress_result_t *result) {
    if (ctx == NULL || !ctx->initialized || 
        input == NULL || output == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t start_time = HAL_Time_us();
    
    // Para simplificar, implementar apenas descompressão delta
    // Em uma implementação completa, detectar o tipo de compressão do header
    
    if (input_size < 3) {
        result->success = false;
        return ESP_ERR_INVALID_ARG;
    }
    
    // Verificar checksum
    uint16_t received_checksum = (input[input_size - 2] << 8) | input[input_size - 1];
    uint16_t calculated_checksum = espnow_calculate_checksum(input, input_size - 2);
    
    if (received_checksum != calculated_checksum) {
        ESP_LOGE(TAG, "Checksum mismatch: expected 0x%04X, got 0x%04X", 
                  calculated_checksum, received_checksum);
        result->success = false;
        result->checksum_valid = false;
        return ESP_ERR_INVALID_CRC;
    }
    
    // Descompressão delta
    uint16_t decompressed_size = input_size - 2; // Excluir checksum
    
    if (decompressed_size > output_size) {
        result->success = false;
        return ESP_ERR_INVALID_SIZE;
    }
    
    output[0] = input[0]; // Primeiro byte original
    
    for (uint16_t i = 1; i < decompressed_size; i++) {
        output[i] = output[i-1] + input[i];
    }
    
    // Preencher resultado
    result->decompressed_size = decompressed_size;
    result->decompression_time_us = (uint32_t)(HAL_Time_us() - start_time);
    result->success = true;
    result->checksum_valid = true;
    
    ESP_LOGV(TAG, "Decompression: %d->%d bytes, time=%d us", 
              input_size, decompressed_size, result->decompression_time_us);
    
    return ESP_OK;
}

//=============================================================================
// Funções de Otimização
//=============================================================================

esp_err_t espnow_select_best_compression(espnow_compress_context_t *ctx,
                                         const uint8_t *input,
                                         uint16_t input_size,
                                         espnow_compress_type_t *best_type) {
    if (ctx == NULL || input == NULL || best_type == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Testar diferentes algoritmos em uma amostra pequena
    uint16_t sample_size = (input_size > 256) ? 256 : input_size;
    
    espnow_compress_result_t results[ESPNOW_COMPRESS_COUNT];
    memset(results, 0, sizeof(results));
    
    // Testar compressão delta
    uint8_t test_buffer[512];
    espnow_compress_delta(ctx, input, sample_size, test_buffer, sizeof(test_buffer), 
                          &results[ESPNOW_COMPRESS_DELTA].compressed_size);
    results[ESPNOW_COMPRESS_DELTA].compression_ratio = 
        (float)sample_size / (float)results[ESPNOW_COMPRESS_DELTA].compressed_size;
    
    // Testar compressão Huffman
    espnow_compress_huffman(ctx, input, sample_size, test_buffer, sizeof(test_buffer),
                           &results[ESPNOW_COMPRESS_HUFFMAN].compressed_size);
    results[ESPNOW_COMPRESS_HUFFMAN].compression_ratio = 
        (float)sample_size / (float)results[ESPNOW_COMPRESS_HUFFMAN].compressed_size;
    
    // Selecionar melhor baseado na taxa de compressão
    float best_ratio = 1.0f;
    espnow_compress_type_t best = ESPNOW_COMPRESS_NONE;
    
    for (int i = 1; i < ESPNOW_COMPRESS_COUNT; i++) {
        if (results[i].compression_ratio > best_ratio) {
            best_ratio = results[i].compression_ratio;
            best = (espnow_compress_type_t)i;
        }
    }
    
    // Usar o melhor apenas se a compressão for significativa (>20%)
    if (best_ratio > 1.2f) {
        *best_type = best;
    } else {
        *best_type = ESPNOW_COMPRESS_NONE;
    }
    
    ESP_LOGD(TAG, "Best compression type: %d (ratio: %.2f)", *best_type, best_ratio);
    
    return ESP_OK;
}

//=============================================================================
// Funções Utilitárias
//=============================================================================

static uint16_t espnow_calculate_checksum(const uint8_t *data, uint16_t size) {
    uint16_t checksum = 0xFFFF;
    
    for (uint16_t i = 0; i < size; i++) {
        checksum ^= data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (checksum & 0x8000) {
                checksum = (checksum << 1) ^ 0x1021;
            } else {
                checksum <<= 1;
            }
        }
    }
    
    return checksum;
}

static esp_err_t espnow_quantize_value(float value, uint8_t bits, uint16_t *quantized) {
    if (quantized == NULL || bits == 0 || bits > 16) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limitar valor ao range de 16 bits signed
    if (value > 32767.0f) value = 32767.0f;
    if (value < -32768.0f) value = -32768.0f;
    
    // Quantizar
    float scale = (float)(1 << (bits - 1));
    *quantized = (uint16_t)(value * scale / 32768.0f) & ((1 << bits) - 1);
    
    return ESP_OK;
}

static esp_err_t espnow_build_huffman_tree(espnow_compress_context_t *ctx,
                                           const uint8_t *data,
                                           uint16_t size) {
    if (ctx == NULL || data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Resetar frequências
    for (int i = 0; i < ESPNOW_MAX_HUFFMAN_SYMBOLS; i++) {
        ctx->huffman_table[i].frequency = 0;
    }
    
    // Contar frequências
    for (uint16_t i = 0; i < size; i++) {
        ctx->huffman_table[data[i]].frequency++;
    }
    
    return ESP_OK;
}

static void espnow_generate_huffman_codes(espnow_compress_context_t *ctx) {
    // Implementação simplificada - em uma versão completa,
    // construiríamos a árvore Huffman propriamente
    
    for (int i = 0; i < ESPNOW_MAX_HUFFMAN_SYMBOLS; i++) {
        if (ctx->huffman_table[i].frequency > 0) {
            // Código simplificado baseado na frequência
            uint8_t length = 8 - (uint8_t)(log2f(ctx->huffman_table[i].frequency) / 2.0f);
            length = (length < 4) ? 4 : length;
            length = (length > 12) ? 12 : length;
            
            ctx->huffman_table[i].code_length = length;
            ctx->huffman_table[i].code = i & ((1 << length) - 1);
        }
    }
}

esp_err_t espnow_get_compression_stats(espnow_compress_context_t *ctx,
                                       uint32_t *total_compressed,
                                       uint32_t *total_decompressed,
                                       float *avg_compression_ratio,
                                       uint32_t *avg_compression_time) {
    if (ctx == NULL || !ctx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (total_compressed) *total_compressed = ctx->total_compressed;
    if (total_decompressed) *total_decompressed = ctx->total_decompressed;
    if (avg_compression_ratio) *avg_compression_ratio = ctx->compression_ratio;
    if (avg_compression_time) *avg_compression_time = (uint32_t)ctx->avg_compression_time;
    
    return ESP_OK;
}
