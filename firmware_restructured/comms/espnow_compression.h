/**
 * @file espnow_compression.h
 * @brief Módulo de compressão otimizado para comunicação ESP-NOW
 * 
 * Implementa algoritmos de compressão específicos para dados de telemetria
 * de motores, otimizados para o limite de 250 bytes do ESP-NOW e
 * aproveitando as capacidades do ESP32-S3.
 * 
 * Recursos:
 * - Compressão delta para dados numéricos
 * - Compressão Huffman adaptativa para texto
 * - Quantização inteligente para sensores
 * - Compressão vetorial usando SIMD
 * - Taxa de compressão de 60-80% para dados típicos
 */

#ifndef ESPNOW_COMPRESSION_H
#define ESPNOW_COMPRESSION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "espnow_link.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Configurações e Constantes
//=============================================================================

/** @brief Tamanho máximo do buffer de compressão */
#define ESPNOW_COMPRESS_BUFFER_SIZE   512

/** @brief Tamanho máximo do buffer descomprimido */
#define ESPNOW_DECOMPRESS_BUFFER_SIZE 1024

/** @brief Nível de compressão (1-9) */
#define ESPNOW_COMPRESS_LEVEL         6

/** @brief Limiar para usar compressão (bytes) */
#define ESPNOW_COMPRESS_THRESHOLD     64

/** @brief Precisão de quantização (bits) */
#define ESPNOW_QUANTIZATION_BITS     10

/** @brief Número máximo de símbolos Huffman */
#define ESPNOW_MAX_HUFFMAN_SYMBOLS   256

//=============================================================================
// Tipos de Compressão
//=============================================================================

typedef enum {
    ESPNOW_COMPRESS_NONE = 0,        /**< Sem compressão */
    ESPNOW_COMPRESS_DELTA,           /**< Compressão delta */
    ESPNOW_COMPRESS_HUFFMAN,         /**< Compressão Huffman */
    ESPNOW_COMPRESS_LZ77,            /**< Compressão LZ77 */
    ESPNOW_COMPRESS_QUANTIZED,       /**< Compressão quantizada */
    ESPNOW_COMPRESS_HYBRID,          /**< Compressão híbrida */
    ESPNOW_COMPRESS_COUNT
} espnow_compress_type_t;

//=============================================================================
// Estruturas de Dados
//=============================================================================

/**
 * @brief Configuração da compressão
 */
typedef struct {
    espnow_compress_type_t type;     /**< Tipo de compressão */
    uint8_t level;                  /**< Nível de compressão (1-9) */
    uint8_t quantization_bits;       /**< Bits para quantização */
    bool enable_adaptive;            /**< Habilitar compressão adaptativa */
    bool use_simd;                   /**< Usar otimizações SIMD */
    uint16_t min_size;               /**< Tamanho mínimo para comprimir */
    float compression_ratio_target;   /**< Taxa de compressão alvo */
} espnow_compress_config_t;

/**
 * @brief Tabela de frequências para Huffman
 */
typedef struct {
    uint16_t symbol;                 /**< Símbolo */
    uint32_t frequency;              /**< Frequência */
    uint16_t code;                   /**< Código Huffman */
    uint8_t code_length;             /**< Comprimento do código */
} espnow_huffman_table_t;

/**
 * @brief Contexto de compressão
 */
typedef struct {
    // Buffers de trabalho
    uint8_t input_buffer[ESPNOW_DECOMPRESS_BUFFER_SIZE];
    uint8_t output_buffer[ESPNOW_COMPRESS_BUFFER_SIZE];
    uint8_t temp_buffer[ESPNOW_COMPRESS_BUFFER_SIZE];
    
    // Tabelas de compressão
    espnow_huffman_table_t huffman_table[ESPNOW_MAX_HUFFMAN_SYMBOLS];
    uint16_t delta_table[256];
    float quantization_table[256];
    
    // Estatísticas
    uint32_t total_compressed;
    uint32_t total_decompressed;
    uint32_t compression_time_us;
    uint32_t decompression_time_us;
    float compression_ratio;
    float avg_compression_time;
    
    // Configuração
    espnow_compress_config_t config;
    
    // Estado
    bool initialized;
    bool adaptive_mode;
    uint8_t current_level;
    uint32_t sample_count;
} espnow_compress_context_t;

/**
 * @brief Resultado da compressão
 */
typedef struct {
    espnow_compress_type_t type;     /**< Tipo usado */
    uint16_t original_size;          /**< Tamanho original */
    uint16_t compressed_size;        /**< Tamanho comprimido */
    float compression_ratio;         /**< Taxa de compressão */
    uint32_t compression_time_us;    /**< Tempo de compressão */
    bool success;                    /**< Sucesso da operação */
} espnow_compress_result_t;

/**
 * @brief Resultado da descompressão
 */
typedef struct {
    uint16_t decompressed_size;      /**< Tamanho descomprimido */
    uint32_t decompression_time_us;  /**< Tempo de descompressão */
    bool success;                    /**< Sucesso da operação */
    bool checksum_valid;             /**< Checksum válido */
} espnow_decompress_result_t;

//=============================================================================
// Funções de Inicialização
//=============================================================================

/**
 * @brief Inicializa o módulo de compressão
 * @param ctx Ponteiro para o contexto
 * @param config Configuração da compressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_init(espnow_compress_context_t *ctx,
                               const espnow_compress_config_t *config);

/**
 * @brief Desinicializa o módulo de compressão
 * @param ctx Ponteiro para o contexto
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_deinit(espnow_compress_context_t *ctx);

/**
 * @brief Configura parâmetros de compressão
 * @param ctx Ponteiro para o contexto
 * @param config Nova configuração
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_configure(espnow_compress_context_t *ctx,
                                    const espnow_compress_config_t *config);

//=============================================================================
// Funções de Compressão
//=============================================================================

/**
 * @brief Comprime dados usando método configurado
 * @param ctx Ponteiro para o contexto
 * @param input Dados de entrada
 * @param input_size Tamanho dos dados de entrada
 * @param output Buffer de saída
 * @param output_size Tamanho disponível no buffer de saída
 * @param result Resultado da compressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_data(espnow_compress_context_t *ctx,
                               const uint8_t *input,
                               uint16_t input_size,
                               uint8_t *output,
                               uint16_t output_size,
                               espnow_compress_result_t *result);

/**
 * @brief Comprime mensagem de status do motor
 * @param ctx Ponteiro para o contexto
 * @param status Dados de status do motor
 * @param output Buffer de saída
 * @param output_size Tamanho disponível
 * @param result Resultado da compressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_engine_status(espnow_compress_context_t *ctx,
                                         const espnow_engine_status_t *status,
                                         uint8_t *output,
                                         uint16_t output_size,
                                         espnow_compress_result_t *result);

/**
 * @brief Comprime dados de sensores
 * @param ctx Ponteiro para o contexto
 * @param sensor_data Dados dos sensores
 * @param output Buffer de saída
 * @param output_size Tamanho disponível
 * @param result Resultado da compressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_sensor_data(espnow_compress_context_t *ctx,
                                      const espnow_sensor_data_t *sensor_data,
                                      uint8_t *output,
                                      uint16_t output_size,
                                      espnow_compress_result_t *result);

/**
 * @brief Comprime dados diagnósticos
 * @param ctx Ponteiro para o contexto
 * @param diagnostic Dados diagnósticos
 * @param output Buffer de saída
 * @param output_size Tamanho disponível
 * @param result Resultado da compressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_diagnostic(espnow_compress_context_t *ctx,
                                     const espnow_diagnostic_t *diagnostic,
                                     uint8_t *output,
                                     uint16_t output_size,
                                     espnow_compress_result_t *result);

//=============================================================================
// Funções de Descompressão
//=============================================================================

/**
 * @brief Descomprime dados
 * @param ctx Ponteiro para o contexto
 * @param input Dados comprimidos
 * @param input_size Tamanho dos dados comprimidos
 * @param output Buffer de saída
 * @param output_size Tamanho disponível no buffer de saída
 * @param result Resultado da descompressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_decompress_data(espnow_compress_context_t *ctx,
                                 const uint8_t *input,
                                 uint16_t input_size,
                                 uint8_t *output,
                                 uint16_t output_size,
                                 espnow_decompress_result_t *result);

/**
 * @brief Descomprime mensagem de status do motor
 * @param ctx Ponteiro para o contexto
 * @param input Dados comprimidos
 * @param input_size Tamanho dos dados
 * @param status Estrutura de status (saída)
 * @param result Resultado da descompressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_decompress_engine_status(espnow_compress_context_t *ctx,
                                          const uint8_t *input,
                                          uint16_t input_size,
                                          espnow_engine_status_t *status,
                                          espnow_decompress_result_t *result);

/**
 * @brief Descomprime dados de sensores
 * @param ctx Ponteiro para o contexto
 * @param input Dados comprimidos
 * @param input_size Tamanho dos dados
 * @param sensor_data Estrutura de sensores (saída)
 * @param result Resultado da descompressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_decompress_sensor_data(espnow_compress_context_t *ctx,
                                        const uint8_t *input,
                                        uint16_t input_size,
                                        espnow_sensor_data_t *sensor_data,
                                        espnow_decompress_result_t *result);

//=============================================================================
// Algoritmos Específicos
//=============================================================================

/**
 * @brief Compressão delta para dados numéricos
 * @param ctx Ponteiro para o contexto
 * @param input Dados de entrada
 * @param input_size Tamanho dos dados
 * @param output Buffer de saída
 * @param output_size Tamanho disponível
 * @param compressed_size Tamanho comprimido (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_delta(espnow_compress_context_t *ctx,
                                const uint8_t *input,
                                uint16_t input_size,
                                uint8_t *output,
                                uint16_t output_size,
                                uint16_t *compressed_size);

/**
 * @brief Compressão Huffman adaptativa
 * @param ctx Ponteiro para o contexto
 * @param input Dados de entrada
 * @param input_size Tamanho dos dados
 * @param output Buffer de saída
 * @param output_size Tamanho disponível
 * @param compressed_size Tamanho comprimido (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_huffman(espnow_compress_context_t *ctx,
                                  const uint8_t *input,
                                  uint16_t input_size,
                                  uint8_t *output,
                                  uint16_t output_size,
                                  uint16_t *compressed_size);

/**
 * @brief Compressão quantizada para sensores
 * @param ctx Ponteiro para o contexto
 * @param input Dados de entrada (float)
 * @param input_size Número de amostras
 * @param output Buffer de saída
 * @param output_size Tamanho disponível
 * @param compressed_size Tamanho comprimido (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_quantized(espnow_compress_context_t *ctx,
                                    const float *input,
                                    uint16_t input_size,
                                    uint8_t *output,
                                    uint16_t output_size,
                                    uint16_t *compressed_size);

/**
 * @brief Compressão híbrida (delta + Huffman)
 * @param ctx Ponteiro para o contexto
 * @param input Dados de entrada
 * @param input_size Tamanho dos dados
 * @param output Buffer de saída
 * @param output_size Tamanho disponível
 * @param compressed_size Tamanho comprimido (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_compress_hybrid(espnow_compress_context_t *ctx,
                                 const uint8_t *input,
                                 uint16_t input_size,
                                 uint8_t *output,
                                 uint16_t output_size,
                                 uint16_t *compressed_size);

//=============================================================================
// Funções de Otimização
//=============================================================================

/**
 * @brief Seleciona melhor algoritmo de compressão
 * @param ctx Ponteiro para o contexto
 * @param input Dados de entrada
 * @param input_size Tamanho dos dados
 * @param best_type Melhor tipo de compressão (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_select_best_compression(espnow_compress_context_t *ctx,
                                         const uint8_t *input,
                                         uint16_t input_size,
                                         espnow_compress_type_t *best_type);

/**
 * @brief Otimiza tabelas de compressão baseado em dados históricos
 * @param ctx Ponteiro para o contexto
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_optimize_compression_tables(espnow_compress_context_t *ctx);

/**
 * @brief Habilita modo adaptativo
 * @param ctx Ponteiro para o contexto
 * @param enabled Habilitar modo adaptativo
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_enable_adaptive_compression(espnow_compress_context_t *ctx,
                                             bool enabled);

//=============================================================================
// Funções de Estatísticas e Diagnóstico
//=============================================================================

/**
 * @brief Obtém estatísticas de compressão
 * @param ctx Ponteiro para o contexto
 * @param total_compressed Total comprimido
 * @param total_decompressed Total descomprimido
 * @param avg_compression_ratio Taxa média de compressão
 * @param avg_compression_time Tempo médio de compressão
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_get_compression_stats(espnow_compress_context_t *ctx,
                                       uint32_t *total_compressed,
                                       uint32_t *total_decompressed,
                                       float *avg_compression_ratio,
                                       uint32_t *avg_compression_time);

/**
 * @brief Testa performance dos algoritmos
 * @param ctx Ponteiro para o contexto
 * @param test_data Dados para teste
 * @param test_size Tamanho dos dados de teste
 * @param results Array de resultados por algoritmo
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_benchmark_compression(espnow_compress_context_t *ctx,
                                        const uint8_t *test_data,
                                        uint16_t test_size,
                                        espnow_compress_result_t results[ESPNOW_COMPRESS_COUNT]);

/**
 * @brief Reseta estatísticas de compressão
 * @param ctx Ponteiro para o contexto
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_reset_compression_stats(espnow_compress_context_t *ctx);

/**
 * @brief Verifica integridade dos dados comprimidos
 * @param ctx Ponteiro para o contexto
 * @param compressed_data Dados comprimidos
 * @param compressed_size Tamanho dos dados
 * @param valid Dados válidos (saída)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t espnow_verify_compressed_data(espnow_compress_context_t *ctx,
                                         const uint8_t *compressed_data,
                                         uint16_t compressed_size,
                                         bool *valid);

#ifdef __cplusplus
}
#endif

#endif // ESPNOW_COMPRESSION_H
