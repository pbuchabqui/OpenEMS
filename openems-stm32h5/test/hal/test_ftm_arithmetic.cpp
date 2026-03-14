/**
 * @file test/hal/test_ftm_arithmetic.cpp
 * @brief Teste host-side: aritmética circular uint16_t para timestamps FTM
 *
 * NÃO requer hardware. Testa apenas a propriedade matemática de subtração
 * circular de uint16_t usada para calcular períodos de dentes do CKP.
 *
 * Contexto:
 *   FTM0/FTM3 são contadores de 16 bits free-running @ 60 MHz efetivo.
 *   Overflow a cada 65536 ticks ≈ 1.09 ms.
 *   Período de dente a 1000 RPM (60t): ~1 ms → pode cruzar o overflow.
 *
 * Regra inegociável (Seção 0.2 do contrato global):
 *   uint16_t delta = (uint16_t)(current - previous);  // CORRETO
 *   if (current > previous) ...                        // ERRADO
 *
 * Framework: Unity (compatível com PlatformIO test runner)
 * Build: g++ -std=c++17 -I../../src test_ftm_arithmetic.cpp -o test_ftm_arith
 *        ./test_ftm_arith
 */

#include <cstdint>
#include <cassert>
#include <cstdio>

// ──────────────────────────────────────────────────────────────────────────────
// Implementação mínima de runner de testes (sem dependência de Unity no host)
// ──────────────────────────────────────────────────────────────────────────────
static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT_EQUAL_UINT16(expected, actual) do { \
    ++g_tests_run; \
    if ((uint16_t)(expected) != (uint16_t)(actual)) { \
        ++g_tests_failed; \
        printf("  FAIL [%s:%d]: expected 0x%04X, got 0x%04X\n", \
               __FILE__, __LINE__, (unsigned)(uint16_t)(expected), \
               (unsigned)(uint16_t)(actual)); \
    } \
} while(0)

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        printf("  FAIL [%s:%d]: condition false: " #cond "\n", __FILE__, __LINE__); \
    } \
} while(0)

#define RUN_TEST(fn) do { \
    printf("[ RUN ] " #fn "\n"); \
    fn(); \
    printf("[  OK ] " #fn "\n"); \
} while(0)

// ──────────────────────────────────────────────────────────────────────────────
// Função sob teste: delta circular de timestamps FTM (uint16_t)
// Esta é a única operação permitida para calcular períodos de dentes.
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Calcula o período em ticks entre dois timestamps FTM de 16 bits.
 *
 * Correto para qualquer par de valores, incluindo quando current < previous
 * (overflow do contador). O resultado é sempre o menor arco positivo de
 * previous → current no espaço circular de 0..65535.
 *
 * @param current  Timestamp do evento mais recente (FTM_CnV no momento da captura)
 * @param previous Timestamp do evento anterior
 * @return Diferença circular em ticks (sempre no intervalo [0, 65535])
 */
static inline uint16_t ftm_delta_ticks(uint16_t current, uint16_t previous) noexcept {
    // Subtração em uint16_t — wrap-around automático pelo padrão C++17 §6.9.1
    // Equivale a: (current - previous + 65536) % 65536
    return static_cast<uint16_t>(current - previous);
}

/**
 * @brief Converte delta de ticks FTM para período em nanosegundos.
 *
 * Com prescaler 2 e clock de 120 MHz: 1 tick = 1/60MHz ≈ 16.667 ns
 * Usando aritmética inteira: ns = ticks * 50 / 3  (evita float)
 * Precisão: erro máximo = 0.01 ns por tick, desprezível.
 *
 * @param delta_ticks  Período em ticks (resultado de ftm_delta_ticks)
 * @return Período em nanosegundos (uint32_t — máximo ~1.09 ms * 65536 = saturação)
 */
static inline uint32_t ftm_ticks_to_ns(uint16_t delta_ticks) noexcept {
    // 120 MHz / prescaler 2 = 60 MHz → período de tick = 1/60e6 s = 16.667 ns
    // 16.667 ns = 50/3 ns → multiplica por 50 e divide por 3
    return (static_cast<uint32_t>(delta_ticks) * 50u) / 3u;
}

// ──────────────────────────────────────────────────────────────────────────────
// Casos de teste
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Caso trivial: sem overflow, current > previous.
 * Cenário: dente capturado em 0x1000, anterior em 0x0800.
 * Delta esperado: 0x0800 ticks.
 */
static void test_delta_no_overflow(void) {
    uint16_t prev    = 0x0800u;
    uint16_t current = 0x1000u;
    uint16_t delta   = ftm_delta_ticks(current, prev);
    TEST_ASSERT_EQUAL_UINT16(0x0800u, delta);
}

/**
 * @brief Overflow simples: current < previous (contador passou por 0xFFFF → 0x0000).
 * Cenário real: dente anterior em 0xFE00, atual em 0x0200.
 * Delta correto (circular): 0xFFFF - 0xFE00 + 1 + 0x0200 = 0x0400
 * Com subtração uint16_t: (0x0200 - 0xFE00) = 0x0400 ✓
 *
 * Demonstra o bug do RusEFI #1488: comparação direta daria resultado errado
 * pois 0x0200 < 0xFE00 levaria a branch incorreta.
 */
static void test_delta_single_overflow(void) {
    uint16_t prev    = 0xFE00u;
    uint16_t current = 0x0200u;
    uint16_t expected = 0x0400u;
    uint16_t delta   = ftm_delta_ticks(current, prev);
    TEST_ASSERT_EQUAL_UINT16(expected, delta);

    // Demonstração explícita do bug: comparação direta é ERRADA
    // (comentado pois causaria assert failure intencional se descomentado)
    // uint16_t wrong_delta = (current > prev) ? (current - prev) : 0; // BUG!
    // TEST_ASSERT_EQUAL_UINT16(expected, wrong_delta); // falharia: 0 != 0x0400
}

/**
 * @brief Overflow exatamente no zero: previous = 0xFFFF, current = 0x0000.
 * Delta esperado: 1 tick.
 */
static void test_delta_overflow_at_zero(void) {
    uint16_t prev    = 0xFFFFu;
    uint16_t current = 0x0000u;
    uint16_t delta   = ftm_delta_ticks(current, prev);
    TEST_ASSERT_EQUAL_UINT16(1u, delta);
}

/**
 * @brief Overflow com delta máximo possível: exatamente meio período (0x8000).
 * Subtração circular é ambígua aqui (poderia ser +32768 ou -32768),
 * mas uint16_t resolve como +32768 — comportamento documentado e aceito.
 */
static void test_delta_half_period(void) {
    uint16_t prev    = 0x8000u;
    uint16_t current = 0x0000u;
    uint16_t delta   = ftm_delta_ticks(current, prev);
    TEST_ASSERT_EQUAL_UINT16(0x8000u, delta);
}

/**
 * @brief Múltiplos overflows: current = previous + N onde N > 65535.
 * O FTM pode sofrer múltiplos overflows entre dentes a RPM muito baixo
 * (< ~55 RPM com prescaler 2).
 * NOTA: este caso NÃO é recuperável com uint16_t simples — requer
 * contador de overflows externo. Este teste documenta a limitação.
 * Para RPM > 55, delta < 65535 ticks → sem problema.
 */
static void test_delta_limitation_at_very_low_rpm(void) {
    // A 30 RPM (motor de manutenção), período de dente ≈ 2.22 ms
    // = 2,222,222 ns / 16.667 ns/tick ≈ 133,333 ticks > 65535
    // Resultado com uint16_t: 133333 % 65536 = 2261 — ERRADO
    // Este teste documenta que o módulo CKP deve lidar com isso via
    // contagem de overflows do FTM_SC[TOF].
    //
    // Para os casos normais de operação (RPM > 55), o delta sempre cabe
    // em 16 bits. Apenas documentamos a limitação aqui.
    uint16_t prev    = 0x0000u;
    uint16_t current = static_cast<uint16_t>(133333u % 65536u);  // 2261
    uint16_t naive_delta = ftm_delta_ticks(current, prev);
    // Valor retornado é 2261 — correto apenas se o scheduler rastrear overflows
    TEST_ASSERT_EQUAL_UINT16(2261u, naive_delta);
    // Documentação: este resultado é válido SOMENTE se overflow_count == 2
    // (133333 = 2*65536 + 2261)
}

/**
 * @brief Teste de conversão de ticks para nanosegundos.
 * 60 MHz efetivo → 1 tick = 16.667 ns
 * 3 ticks → 50 ns (3 * 50 / 3 = 50)
 * 6000 ticks → 100,000 ns = 100 µs (6000 * 50 / 3 = 100,000)
 */
static void test_ticks_to_ns_conversion(void) {
    // 3 ticks = 50 ns
    TEST_ASSERT_EQUAL_UINT16(50u, (uint16_t)ftm_ticks_to_ns(3u));

    // 6000 ticks = 100,000 ns = 100 µs
    TEST_ASSERT_TRUE(ftm_ticks_to_ns(6000u) == 100000u);

    // 1 dente a 1000 RPM, 60 dentes: período de dente = 1/1000 s / 60 = 16.667 µs
    // 16,667 ns / 16.667 ns/tick = 1000.0 ticks → 999 ou 1000 (arredondamento inteiro)
    // 1000 ticks * 50 / 3 = 16666 ns (erro de 1 ns — aceitável)
    TEST_ASSERT_TRUE(ftm_ticks_to_ns(1000u) == 16666u);

    // Verifica ausência de overflow em delta máximo de 16 bits
    // 65535 ticks * 50 = 3,276,750; / 3 = 1,092,250 ns ≈ 1.09 ms
    // Deve caber em uint32_t (max ~4.29 s)
    uint32_t max_ns = ftm_ticks_to_ns(0xFFFFu);
    TEST_ASSERT_TRUE(max_ns == 1092250u);
}

/**
 * @brief Teste de consistência: delta(a→b) + delta(b→a) = 65536 (espaço circular).
 * Propriedade fundamental do grupo abeliano Z/65536Z.
 */
static void test_delta_circular_consistency(void) {
    uint16_t a = 0x3000u;
    uint16_t b = 0xC000u;

    uint16_t d_ab = ftm_delta_ticks(b, a);  // 0xC000 - 0x3000 = 0x9000
    uint16_t d_ba = ftm_delta_ticks(a, b);  // 0x3000 - 0xC000 = 0x7000 (circular)

    // d_ab + d_ba deve ser exatamente 65536 (0 em uint16_t, mas como uint32_t = 65536)
    uint32_t sum = static_cast<uint32_t>(d_ab) + static_cast<uint32_t>(d_ba);
    TEST_ASSERT_TRUE(sum == 65536u);

    TEST_ASSERT_EQUAL_UINT16(0x9000u, d_ab);
    TEST_ASSERT_EQUAL_UINT16(0x7000u, d_ba);
}

/**
 * @brief Teste do cenário real RusEFI #1488:
 * ISR chega 65 µs atrasada. Sem aritmética circular, o sistema interpreta
 * um gap normal como missing tooth (falso trigger de sincronização).
 *
 * A 3000 RPM, 60-2 dentes: período de dente normal ≈ 333 µs
 * = 333,000 ns / 16.667 ns/tick ≈ 19,980 ticks
 *
 * ISR atrasada em 65 µs = 65,000 ns / 16.667 ns/tick ≈ 3,900 ticks
 * O timestamp capturado é correto (hardware capturou no momento certo),
 * mas se a ISR usasse ftm0_count() em vez de FTM_CnV, o delta seria correto
 * apenas se processado a tempo.
 *
 * Este teste verifica que mesmo com processamento atrasado, o TIMESTAMP
 * capturado pelo hardware é válido para aritmética circular.
 */
static void test_reusefi_1488_scenario(void) {
    // Simulação: 3000 RPM, 60-2 wheel
    // Tick de dente normal: 19,980 ticks
    // Dente 56 capturado em t=0x4000, dente 57 em t=0x4000+19980=0x4EEC
    // ISR do dente 57 atrasa 3900 ticks, mas FTM_CnV já tem o timestamp correto

    uint16_t tooth_56 = 0x4000u;
    uint16_t tooth_57_hw = static_cast<uint16_t>(tooth_56 + 19980u);  // 0x4EEC

    // Delta correto via subtração circular
    uint16_t delta = ftm_delta_ticks(tooth_57_hw, tooth_56);
    TEST_ASSERT_EQUAL_UINT16(19980u, delta);

    // O timestamp 0x4EEC está ANTES do valor atual do contador
    // (ISR atrasada: contador atual = 0x4EEC + 3900 = 0x5E44)
    // Se a ISR lesse o contador atual em vez de FTM_CnV, teria delta = 23880 → ERRO
    // Mas usando FTM_CnV (correto), delta = 19980 → OK
    uint16_t ftm_cnv_value = tooth_57_hw;              // hardware capturou correto
    uint16_t isr_late_count = static_cast<uint16_t>(tooth_57_hw + 3900u);  // 0x5E44

    uint16_t delta_correct = ftm_delta_ticks(ftm_cnv_value,    tooth_56);  // 19980
    uint16_t delta_wrong   = ftm_delta_ticks(isr_late_count,   tooth_56);  // 23880

    TEST_ASSERT_EQUAL_UINT16(19980u, delta_correct);
    TEST_ASSERT_TRUE(delta_wrong != delta_correct);
    TEST_ASSERT_EQUAL_UINT16(23880u, delta_wrong);
}

/**
 * @brief Casos literais exigidos pelo Prompt 1, Entregável #3.
 * Valores em decimal conforme especificado no contrato do módulo HAL.
 *
 *   now=10,    prev=65530  → delta deve ser 16    (overflow: 65536 - 65530 + 10 = 16)
 *   now=1000,  prev=500    → delta deve ser 500   (normal, sem overflow)
 *   now=0,     prev=65535  → delta deve ser 1     (overflow exato na borda)
 */
static void test_delta_prompt_required_cases(void) {
    TEST_ASSERT_EQUAL_UINT16(16u,  ftm_delta_ticks(10u,   65530u));
    TEST_ASSERT_EQUAL_UINT16(500u, ftm_delta_ticks(1000u, 500u));
    TEST_ASSERT_EQUAL_UINT16(1u,   ftm_delta_ticks(0u,    65535u));
}

// ──────────────────────────────────────────────────────────────────────────────
// Runner principal
// ──────────────────────────────────────────────────────────────────────────────
int main(void) {
    printf("═══════════════════════════════════════════════════════════════\n");
    printf(" OpenEMS HAL — FTM uint16_t Circular Arithmetic Tests\n");
    printf(" Target: host x86_64 | FTM prescaler 2 @ 120 MHz sys clock\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    RUN_TEST(test_delta_prompt_required_cases);
    RUN_TEST(test_delta_no_overflow);
    RUN_TEST(test_delta_single_overflow);
    RUN_TEST(test_delta_overflow_at_zero);
    RUN_TEST(test_delta_half_period);
    RUN_TEST(test_delta_limitation_at_very_low_rpm);
    RUN_TEST(test_ticks_to_ns_conversion);
    RUN_TEST(test_delta_circular_consistency);
    RUN_TEST(test_reusefi_1488_scenario);

    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf(" Results: %d tests, %d failed\n", g_tests_run, g_tests_failed);
    printf("═══════════════════════════════════════════════════════════════\n");

    return g_tests_failed == 0 ? 0 : 1;
}
