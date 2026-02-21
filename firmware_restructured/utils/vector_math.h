/**
 * @file vector_math.h
 * @brief Módulo de matemática vetorial otimizado para ESP32-S3
 * 
 * Aproveita as instruções vetoriais do ESP32-S3 para acelerar
 * cálculos matemáticos comuns no sistema de injeção eletrônica.
 * 
 * Recursos:
 * - Operações vetoriais SIMD otimizadas
 * - Funções trigonométricas vetoriais
 * - Interpolação bilinear vetorial
 * - Cálculos de timing em lote
 */

#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "esp_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Configurações e Constantes
//=============================================================================

/** @brief Tamanho máximo dos vetores para processamento */
#define VECTOR_MAX_SIZE        64

/** @brief Número de elementos para processamento SIMD (ESP32-S3) */
#define VECTOR_SIMD_WIDTH      4

/** @brief Fator de conversão de graus para radianos */
#define VECTOR_DEG_TO_RAD      (M_PI / 180.0f)

/** @brief Fator de conversão de radianos para graus */
#define VECTOR_RAD_TO_DEG      (180.0f / M_PI)

/** @brief Precisão para comparações de ponto flutuante */
#define VECTOR_EPSILON         1e-6f

//=============================================================================
// Estruturas de Dados
//=============================================================================

/**
 * @brief Estrutura para vetor 2D
 */
typedef struct {
    float x;
    float y;
} vector2d_t;

/**
 * @brief Estrutura para vetor 3D
 */
typedef struct {
    float x;
    float y;
    float z;
} vector3d_t;

/**
 * @brief Estrutura para matriz 3x3
 */
typedef struct {
    float m[3][3];
} matrix3x3_t;

/**
 * @brief Estrutura para contexto de processamento vetorial
 */
typedef struct {
    float buffer_a[VECTOR_MAX_SIZE];
    float buffer_b[VECTOR_MAX_SIZE];
    float buffer_c[VECTOR_MAX_SIZE];
    float sin_table[360];      // Tabela de seno pré-calculada
    float cos_table[360];      // Tabela de cosseno pré-calculada
    bool initialized;
} vector_context_t;

//=============================================================================
// Funções de Inicialização
//=============================================================================

/**
 * @brief Inicializa o módulo de matemática vetorial
 * @param ctx Ponteiro para o contexto vetorial
 * @return ESP_OK em caso de sucesso
 */
esp_err_t vector_math_init(vector_context_t *ctx);

/**
 * @brief Desinicializa o módulo de matemática vetorial
 * @param ctx Ponteiro para o contexto vetorial
 * @return ESP_OK em caso de sucesso
 */
esp_err_t vector_math_deinit(vector_context_t *ctx);

//=============================================================================
// Operações Vetoriais Básicas (Otimizadas ESP32-S3)
//=============================================================================

/**
 * @brief Soma vetorial otimizada com SIMD
 * @param a Primeiro vetor
 * @param b Segundo vetor
 * @param result Vetor resultado
 * @param size Tamanho dos vetores
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_add(const float *a, const float *b, float *result, uint16_t size);

/**
 * @brief Subtração vetorial otimizada com SIMD
 * @param a Primeiro vetor
 * @param b Segundo vetor
 * @param result Vetor resultado
 * @param size Tamanho dos vetores
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_sub(const float *a, const float *b, float *result, uint16_t size);

/**
 * @brief Multiplicação escalar vetorial otimizada
 * @param vector Vetor de entrada
 * @param scalar Escalar
 * @param result Vetor resultado
 * @param size Tamanho do vetor
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_scale(const float *vector, float scalar, float *result, uint16_t size);

/**
 * @brief Produto escalar (dot product) otimizado
 * @param a Primeiro vetor
 * @param b Segundo vetor
 * @param size Tamanho dos vetores
 * @return Produto escalar
 */
IRAM_ATTR float vector_dot(const float *a, const float *b, uint16_t size);

/**
 * @brief Produto vetorial (cross product) para vetores 3D
 * @param a Primeiro vetor
 * @param b Segundo vetor
 * @param result Vetor resultado
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_cross_3d(const vector3d_t *a, const vector3d_t *b, vector3d_t *result);

/**
 * @brief Magnitude (norma) de vetor
 * @param vector Vetor de entrada
 * @param size Tamanho do vetor
 * @return Magnitude do vetor
 */
IRAM_ATTR float vector_magnitude(const float *vector, uint16_t size);

/**
 * @brief Normalização vetorial
 * @param vector Vetor de entrada/saída
 * @param size Tamanho do vetor
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_normalize(float *vector, uint16_t size);

//=============================================================================
// Funções Trigonométricas Vetoriais
//=============================================================================

/**
 * @brief Calcula seno para múltiplos ângulos (usando tabela lookup)
 * @param angles Array de ângulos em graus
 * @param results Array de resultados (senos)
 * @param size Número de ângulos
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_sin_deg(const float *angles, float *results, uint16_t size);

/**
 * @brief Calcula cosseno para múltiplos ângulos (usando tabela lookup)
 * @param angles Array de ângulos em graus
 * @param results Array de resultados (cossenos)
 * @param size Número de ângulos
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_cos_deg(const float *angles, float *results, uint16_t size);

/**
 * @brief Calcula tangente para múltiplos ângulos
 * @param angles Array de ângulos em graus
 * @param results Array de resultados (tangentes)
 * @param size Número de ângulos
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_tan_deg(const float *angles, float *results, uint16_t size);

/**
 * @brief Converte graus para radianos (vetorial)
 * @param degrees Array de graus
 * @param radians Array de radianos (saída)
 * @param size Número de elementos
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_deg_to_rad(const float *degrees, float *radians, uint16_t size);

/**
 * @brief Converte radianos para graus (vetorial)
 * @param radians Array de radianos
 * @param degrees Array de graus (saída)
 * @param size Número de elementos
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_rad_to_deg(const float *radians, float *degrees, uint16_t size);

//=============================================================================
// Interpolação e Mapas
//=============================================================================

/**
 * @brief Interpolação linear vetorial
 * @param x Array de valores x
 * @param x0 Ponto inicial x
 * @param x1 Ponto final x
 * @param y0 Valor inicial y
 * @param y1 Valor final y
 * @param result Array de resultados interpolados
 * @param size Número de pontos
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_lerp(const float *x, float x0, float x1, 
                               float y0, float y1, float *result, uint16_t size);

/**
 * @brief Interpolação bilinear otimizada para mapas 16x16
 * @param map Ponteiro para o mapa 16x16
 * @param x Coordenada x (0-15)
 * @param y Coordenada y (0-15)
 * @return Valor interpolado
 */
IRAM_ATTR float vector_bilinear_interp_16x16(const uint16_t map[16][16], float x, float y);

/**
 * @brief Interpolação bilinear vetorial para múltiplos pontos
 * @param map Ponteiro para o mapa 16x16
 * @param coords Array de coordenadas (x,y) normalizadas (0.0-1.0)
 * @param results Array de resultados
 * @param num_points Número de pontos
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_bilinear_interp_batch(const uint16_t map[16][16],
                                                 const vector2d_t *coords,
                                                 float *results,
                                                 uint16_t num_points);

//=============================================================================
// Cálculos de Timing Específicos para EFI
//=============================================================================

/**
 * @brief Calcula tempo por grau para múltiplos RPMs
 * @param rpms Array de valores RPM
 * @param us_per_degree Array de tempos por grau em microssegundos
 * @param size Número de valores RPM
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_rpm_to_us_per_degree(const uint16_t *rpms, 
                                                float *us_per_degree, 
                                                uint16_t size);

/**
 * @brief Calcula ângulos para múltiplos tempos (dados RPM)
 * @param times Array de tempos em microssegundos
 * @param rpms Array de RPMs correspondentes
 * @param angles Array de ângulos em graus (saída)
 * @param size Número de valores
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_us_to_degrees(const float *times, 
                                         const uint16_t *rpms,
                                         float *angles, 
                                         uint16_t size);

/**
 * @brief Calcula tempos de injeção para múltiplos cilindros
 * @param pulse_widths Array de larguras de pulso desejadas
 * @param rpm RPM atual
 * @param timing_offsets Array de offsets de timing por cilindro
 * @param injection_times Array de tempos absolutos de injeção
 * @param num_cylinders Número de cilindros
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_calculate_injection_times(const float *pulse_widths,
                                                    uint16_t rpm,
                                                    const float *timing_offsets,
                                                    uint32_t *injection_times,
                                                    uint8_t num_cylinders);

/**
 * @brief Calcula tempos de ignição para múltiplos cilindros
 * @param advance_angles Array de avanços de ignição em graus
 * @param rpm RPM atual
 * @param timing_offsets Array de offsets por cilindro
 * @param ignition_times Array de tempos absolutos de ignição
 * @param num_cylinders Número de cilindros
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_calculate_ignition_times(const float *advance_angles,
                                                   uint16_t rpm,
                                                   const float *timing_offsets,
                                                   uint32_t *ignition_times,
                                                   uint8_t num_cylinders);

//=============================================================================
// Funções de Estatística e Análise
//=============================================================================

/**
 * @brief Calcula média vetorial
 * @param vector Array de valores
 * @param size Tamanho do array
 * @return Média dos valores
 */
IRAM_ATTR float vector_mean(const float *vector, uint16_t size);

/**
 * @brief Calcula desvio padrão vetorial
 * @param vector Array de valores
 * @param size Tamanho do array
 * @return Desvio padrão
 */
IRAM_ATTR float vector_std_dev(const float *vector, uint16_t size);

/**
 * @brief Encontra valor mínimo e máximo em vetor
 * @param vector Array de valores
 * @param size Tamanho do array
 * @param min_value Valor mínimo (saída)
 * @param max_value Valor máximo (saída)
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_min_max(const float *vector, uint16_t size, 
                                   float *min_value, float *max_value);

/**
 * @brief Aplica filtro de média móvel vetorial
 * @param input Array de entrada
 * @param output Array de saída
 * @param window_size Tamanho da janela
 * @param size Tamanho do array
 * @return ESP_OK em caso de sucesso
 */
IRAM_ATTR esp_err_t vector_moving_average(const float *input, float *output,
                                          uint16_t window_size, uint16_t size);

//=============================================================================
// Funções de Otimização para ESP32-S3
//=============================================================================

/**
 * @brief Verifica se processamento vetorial está disponível
 * @return true se SIMD disponível
 */
bool vector_simd_available(void);

/**
 * @brief Alinha memória para operações SIMD
 * @param ptr Ponteiro para alinhar
 * @param alignment Alinhamento desejado (bytes)
 * @return Ponteiro alinhado
 */
void* vector_align_memory(void *ptr, size_t alignment);

/**
 * @brief Otimiza ordem das operações para cache do ESP32-S3
 * @param operations Array de operações
 * @param num_operations Número de operações
 * @return ESP_OK em caso de sucesso
 */
esp_err_t vector_optimize_cache_order(uint32_t *operations, uint16_t num_operations);

#ifdef __cplusplus
}
#endif

#endif // VECTOR_MATH_H
