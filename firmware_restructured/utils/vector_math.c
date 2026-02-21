/**
 * @file vector_math.c
 * @brief Implementação do módulo de matemática vetorial otimizado para ESP32-S3
 */

#include "vector_math.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include <assert.h>

static const char* TAG = "VECTOR_MATH";

//=============================================================================
// Variáveis Globais Estáticas
//=============================================================================

static vector_context_t g_vector_ctx = {0};
static bool g_vector_initialized = false;

//=============================================================================
// Funções de Inicialização
//=============================================================================

esp_err_t vector_math_init(vector_context_t *ctx) {
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limpar contexto
    memset(ctx, 0, sizeof(vector_context_t));
    
    // Pré-calcular tabelas trigonométricas para lookup rápido
    for (uint16_t i = 0; i < 360; i++) {
        float rad = (float)i * VECTOR_DEG_TO_RAD;
        ctx->sin_table[i] = sinf(rad);
        ctx->cos_table[i] = cosf(rad);
    }
    
    ctx->initialized = true;
    g_vector_initialized = true;
    
    ESP_LOGI(TAG, "Vector math module initialized with SIMD support");
    ESP_LOGI(TAG, "  SIMD width: %d elements", VECTOR_SIMD_WIDTH);
    ESP_LOGI(TAG, "  Max vector size: %d elements", VECTOR_MAX_SIZE);
    
    return ESP_OK;
}

esp_err_t vector_math_deinit(vector_context_t *ctx) {
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ctx->initialized = false;
    g_vector_initialized = false;
    
    ESP_LOGI(TAG, "Vector math module deinitialized");
    return ESP_OK;
}

//=============================================================================
// Operações Vetoriais Básicas (Otimizadas ESP32-S3)
//=============================================================================

IRAM_ATTR esp_err_t vector_add(const float *a, const float *b, float *result, uint16_t size) {
    if (a == NULL || b == NULL || result == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_vector_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Implementação otimizada para ESP32-S3 usando loop unrolling
    uint16_t i = 0;
    
    // Processar 4 elementos por vez (SIMD width do ESP32-S3)
    uint16_t simd_size = size & ~3; // Arredondar para múltiplo de 4
    
    for (i = 0; i < simd_size; i += 4) {
        result[i] = a[i] + b[i];
        result[i + 1] = a[i + 1] + b[i + 1];
        result[i + 2] = a[i + 2] + b[i + 2];
        result[i + 3] = a[i + 3] + b[i + 3];
    }
    
    // Processar elementos restantes
    for (; i < size; i++) {
        result[i] = a[i] + b[i];
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_sub(const float *a, const float *b, float *result, uint16_t size) {
    if (a == NULL || b == NULL || result == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_vector_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Implementação otimizada com loop unrolling
    uint16_t i = 0;
    uint16_t simd_size = size & ~3;
    
    for (i = 0; i < simd_size; i += 4) {
        result[i] = a[i] - b[i];
        result[i + 1] = a[i + 1] - b[i + 1];
        result[i + 2] = a[i + 2] - b[i + 2];
        result[i + 3] = a[i + 3] - b[i + 3];
    }
    
    for (; i < size; i++) {
        result[i] = a[i] - b[i];
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_scale(const float *vector, float scalar, float *result, uint16_t size) {
    if (vector == NULL || result == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_vector_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Implementação otimizada
    uint16_t i = 0;
    uint16_t simd_size = size & ~3;
    
    for (i = 0; i < simd_size; i += 4) {
        result[i] = vector[i] * scalar;
        result[i + 1] = vector[i + 1] * scalar;
        result[i + 2] = vector[i + 2] * scalar;
        result[i + 3] = vector[i + 3] * scalar;
    }
    
    for (; i < size; i++) {
        result[i] = vector[i] * scalar;
    }
    
    return ESP_OK;
}

IRAM_ATTR float vector_dot(const float *a, const float *b, uint16_t size) {
    if (a == NULL || b == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return 0.0f;
    }
    
    if (!g_vector_initialized) {
        return 0.0f;
    }
    
    float result = 0.0f;
    
    // Implementação otimizada com acumulador em registrador
    uint16_t i = 0;
    uint16_t simd_size = size & ~3;
    
    for (i = 0; i < simd_size; i += 4) {
        result += a[i] * b[i];
        result += a[i + 1] * b[i + 1];
        result += a[i + 2] * b[i + 2];
        result += a[i + 3] * b[i + 3];
    }
    
    for (; i < size; i++) {
        result += a[i] * b[i];
    }
    
    return result;
}

IRAM_ATTR esp_err_t vector_cross_3d(const vector3d_t *a, const vector3d_t *b, vector3d_t *result) {
    if (a == NULL || b == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Produto vetorial: a × b = (a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)
    result->x = a->y * b->z - a->z * b->y;
    result->y = a->z * b->x - a->x * b->z;
    result->z = a->x * b->y - a->y * b->x;
    
    return ESP_OK;
}

IRAM_ATTR float vector_magnitude(const float *vector, uint16_t size) {
    if (vector == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return 0.0f;
    }
    
    // Magnitude = sqrt(soma dos quadrados)
    float sum_squares = 0.0f;
    
    for (uint16_t i = 0; i < size; i++) {
        sum_squares += vector[i] * vector[i];
    }
    
    return sqrtf(sum_squares);
}

IRAM_ATTR esp_err_t vector_normalize(float *vector, uint16_t size) {
    if (vector == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float magnitude = vector_magnitude(vector, size);
    
    if (magnitude < VECTOR_EPSILON) {
        return ESP_ERR_INVALID_RESPONSE; // Vetor muito pequeno para normalizar
    }
    
    // Dividir cada elemento pela magnitude
    return vector_scale(vector, 1.0f / magnitude, vector, size);
}

//=============================================================================
// Funções Trigonométricas Vetoriais
//=============================================================================

IRAM_ATTR esp_err_t vector_sin_deg(const float *angles, float *results, uint16_t size) {
    if (angles == NULL || results == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_vector_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (uint16_t i = 0; i < size; i++) {
        // Normalizar ângulo para 0-360 graus
        float angle = fmodf(angles[i], 360.0f);
        if (angle < 0) angle += 360.0f;
        
        // Lookup na tabela pré-calculada com interpolação linear
        uint16_t index = (uint16_t)angle;
        float fraction = angle - (float)index;
        
        if (index >= 359) {
            // Caso especial para 359-360 graus
            results[i] = g_vector_ctx.sin_table[359] * (1.0f - fraction) + 
                         g_vector_ctx.sin_table[0] * fraction;
        } else {
            results[i] = g_vector_ctx.sin_table[index] * (1.0f - fraction) + 
                         g_vector_ctx.sin_table[index + 1] * fraction;
        }
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_cos_deg(const float *angles, float *results, uint16_t size) {
    if (angles == NULL || results == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_vector_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (uint16_t i = 0; i < size; i++) {
        // Normalizar ângulo para 0-360 graus
        float angle = fmodf(angles[i], 360.0f);
        if (angle < 0) angle += 360.0f;
        
        // Lookup na tabela pré-calculada com interpolação linear
        uint16_t index = (uint16_t)angle;
        float fraction = angle - (float)index;
        
        if (index >= 359) {
            results[i] = g_vector_ctx.cos_table[359] * (1.0f - fraction) + 
                         g_vector_ctx.cos_table[0] * fraction;
        } else {
            results[i] = g_vector_ctx.cos_table[index] * (1.0f - fraction) + 
                         g_vector_ctx.cos_table[index + 1] * fraction;
        }
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_tan_deg(const float *angles, float *results, uint16_t size) {
    if (angles == NULL || results == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float sin_vals[VECTOR_MAX_SIZE];
    float cos_vals[VECTOR_MAX_SIZE];
    
    // Calcular seno e cosseno
    esp_err_t ret = vector_sin_deg(angles, sin_vals, size);
    if (ret != ESP_OK) return ret;
    
    ret = vector_cos_deg(angles, cos_vals, size);
    if (ret != ESP_OK) return ret;
    
    // Calcular tangente = sin/cos (com proteção contra divisão por zero)
    for (uint16_t i = 0; i < size; i++) {
        if (fabsf(cos_vals[i]) < VECTOR_EPSILON) {
            results[i] = (sin_vals[i] > 0) ? 1000000.0f : -1000000.0f; // Aproximação de infinito
        } else {
            results[i] = sin_vals[i] / cos_vals[i];
        }
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_deg_to_rad(const float *degrees, float *radians, uint16_t size) {
    if (degrees == NULL || radians == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Multiplicação vetorial otimizada
    return vector_scale(degrees, VECTOR_DEG_TO_RAD, radians, size);
}

IRAM_ATTR esp_err_t vector_rad_to_deg(const float *radians, float *degrees, uint16_t size) {
    if (radians == NULL || degrees == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Multiplicação vetorial otimizada
    return vector_scale(radians, VECTOR_RAD_TO_DEG, degrees, size);
}

//=============================================================================
// Interpolação e Mapas
//=============================================================================

IRAM_ATTR esp_err_t vector_lerp(const float *x, float x0, float x1, 
                               float y0, float y1, float *result, uint16_t size) {
    if (x == NULL || result == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (fabsf(x1 - x0) < VECTOR_EPSILON) {
        // Evitar divisão por zero
        for (uint16_t i = 0; i < size; i++) {
            result[i] = y0;
        }
        return ESP_OK;
    }
    
    // Interpolação linear: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    float scale = (y1 - y0) / (x1 - x0);
    
    for (uint16_t i = 0; i < size; i++) {
        result[i] = y0 + (x[i] - x0) * scale;
    }
    
    return ESP_OK;
}

IRAM_ATTR float vector_bilinear_interp_16x16(const uint16_t map[16][16], float x, float y) {
    if (map == NULL) {
        return 0.0f;
    }
    
    // Limitar coordenadas ao intervalo [0, 15]
    x = fmaxf(0.0f, fminf(15.0f, x));
    y = fmaxf(0.0f, fminf(15.0f, y));
    
    // Obter índices inteiros e frações
    uint16_t x0 = (uint16_t)x;
    uint16_t y0 = (uint16_t)y;
    uint16_t x1 = (x0 < 15) ? x0 + 1 : x0;
    uint16_t y1 = (y0 < 15) ? y0 + 1 : y0;
    
    float fx = x - (float)x0;
    float fy = y - (float)y0;
    
    // Obter valores dos 4 pontos vizinhos
    float q00 = (float)map[y0][x0];
    float q10 = (float)map[y0][x1];
    float q01 = (float)map[y1][x0];
    float q11 = (float)map[y1][x1];
    
    // Interpolação bilinear
    float a = q00 * (1.0f - fx) + q10 * fx;
    float b = q01 * (1.0f - fx) + q11 * fx;
    
    return a * (1.0f - fy) + b * fy;
}

IRAM_ATTR esp_err_t vector_bilinear_interp_batch(const uint16_t map[16][16],
                                                 const vector2d_t *coords,
                                                 float *results,
                                                 uint16_t num_points) {
    if (map == NULL || coords == NULL || results == NULL || 
        num_points == 0 || num_points > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Converter coordenadas normalizadas (0.0-1.0) para índices de mapa (0-15)
    float x_coords[VECTOR_MAX_SIZE];
    float y_coords[VECTOR_MAX_SIZE];
    
    for (uint16_t i = 0; i < num_points; i++) {
        x_coords[i] = coords[i].x * 15.0f;
        y_coords[i] = coords[i].y * 15.0f;
    }
    
    // Calcular interpolação para cada ponto
    for (uint16_t i = 0; i < num_points; i++) {
        results[i] = vector_bilinear_interp_16x16(map, x_coords[i], y_coords[i]);
    }
    
    return ESP_OK;
}

//=============================================================================
// Cálculos de Timing Específicos para EFI
//=============================================================================

IRAM_ATTR esp_err_t vector_rpm_to_us_per_degree(const uint16_t *rpms, 
                                                float *us_per_degree, 
                                                uint16_t size) {
    if (rpms == NULL || us_per_degree == NULL || size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Tempo por grau = 60.000.000 / (rpm * 360) = 166.666,67 / rpm
    for (uint16_t i = 0; i < size; i++) {
        if (rpms[i] == 0) {
            us_per_degree[i] = 0.0f;
        } else {
            us_per_degree[i] = 166666.67f / (float)rpms[i];
        }
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_us_to_degrees(const float *times, 
                                         const uint16_t *rpms,
                                         float *angles, 
                                         uint16_t size) {
    if (times == NULL || rpms == NULL || angles == NULL || 
        size == 0 || size > VECTOR_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Graus = tempo * rpm * 360 / 60.000.000 = tempo * rpm * 0.000006
    for (uint16_t i = 0; i < size; i++) {
        angles[i] = times[i] * (float)rpms[i] * 0.000006f;
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_calculate_injection_times(const float *pulse_widths,
                                                    uint16_t rpm,
                                                    const float *timing_offsets,
                                                    uint32_t *injection_times,
                                                    uint8_t num_cylinders) {
    if (pulse_widths == NULL || timing_offsets == NULL || 
        injection_times == NULL || num_cylinders == 0 || 
        num_cylinders > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calcular tempo por grau para o RPM atual
    float us_per_degree = 166666.67f / (float)rpm;
    
    // Calcular tempos absolutos de injeção para cada cilindro
    for (uint8_t i = 0; i < num_cylinders; i++) {
        // Tempo = offset + largura do pulso
        float offset_time = timing_offsets[i] * us_per_degree;
        injection_times[i] = (uint32_t)(offset_time + pulse_widths[i]);
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_calculate_ignition_times(const float *advance_angles,
                                                   uint16_t rpm,
                                                   const float *timing_offsets,
                                                   uint32_t *ignition_times,
                                                   uint8_t num_cylinders) {
    if (advance_angles == NULL || timing_offsets == NULL || 
        ignition_times == NULL || num_cylinders == 0 || 
        num_cylinders > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calcular tempo por grau para o RPM atual
    float us_per_degree = 166666.67f / (float)rpm;
    
    // Calcular tempos absolutos de ignição para cada cilindro
    for (uint8_t i = 0; i < num_cylinders; i++) {
        // Tempo = offset - avanço (avanço é antes do TDC)
        float offset_time = (timing_offsets[i] - advance_angles[i]) * us_per_degree;
        ignition_times[i] = (uint32_t)offset_time;
    }
    
    return ESP_OK;
}

//=============================================================================
// Funções de Estatística e Análise
//=============================================================================

IRAM_ATTR float vector_mean(const float *vector, uint16_t size) {
    if (vector == NULL || size == 0) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        sum += vector[i];
    }
    
    return sum / (float)size;
}

IRAM_ATTR float vector_std_dev(const float *vector, uint16_t size) {
    if (vector == NULL || size == 0) {
        return 0.0f;
    }
    
    float mean = vector_mean(vector, size);
    float sum_squares = 0.0f;
    
    for (uint16_t i = 0; i < size; i++) {
        float diff = vector[i] - mean;
        sum_squares += diff * diff;
    }
    
    return sqrtf(sum_squares / (float)size);
}

IRAM_ATTR esp_err_t vector_min_max(const float *vector, uint16_t size, 
                                   float *min_value, float *max_value) {
    if (vector == NULL || size == 0 || min_value == NULL || max_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *min_value = vector[0];
    *max_value = vector[0];
    
    for (uint16_t i = 1; i < size; i++) {
        if (vector[i] < *min_value) *min_value = vector[i];
        if (vector[i] > *max_value) *max_value = vector[i];
    }
    
    return ESP_OK;
}

IRAM_ATTR esp_err_t vector_moving_average(const float *input, float *output,
                                          uint16_t window_size, uint16_t size) {
    if (input == NULL || output == NULL || window_size == 0 || 
        size == 0 || window_size > size) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Implementar média móvel simples
    for (uint16_t i = 0; i < size; i++) {
        uint16_t start = (i < window_size) ? 0 : i - window_size + 1;
        uint16_t count = i - start + 1;
        
        float sum = 0.0f;
        for (uint16_t j = start; j <= i; j++) {
            sum += input[j];
        }
        
        output[i] = sum / (float)count;
    }
    
    return ESP_OK;
}

//=============================================================================
// Funções de Otimização para ESP32-S3
//=============================================================================

bool vector_simd_available(void) {
    // ESP32-S3 sempre tem suporte SIMD
    return true;
}

void* vector_align_memory(void *ptr, size_t alignment) {
    // Alinhar ponteiro para boundary específico
    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t aligned = (addr + alignment - 1) & ~(alignment - 1);
    return (void*)aligned;
}

esp_err_t vector_optimize_cache_order(uint32_t *operations, uint16_t num_operations) {
    if (operations == NULL || num_operations == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Implementação simplificada - ordenar operações por padrão de acesso
    // Em uma implementação completa, analisaria padrões de acesso à memória
    // e reordenaria para maximizar hits de cache
    
    // Por enquanto, apenas validar que as operações são válidas
    for (uint16_t i = 0; i < num_operations; i++) {
        if (operations[i] == 0) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    return ESP_OK;
}
