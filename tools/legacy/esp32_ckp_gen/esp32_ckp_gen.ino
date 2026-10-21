/*
 * esp32_ckp_gen.ino — Gerador CKP (60-2) + CMP para bancada OpenEMS
 * ─────────────────────────────────────────────────────────────────────
 * Plataforma : ESP32 (Arduino core ≥ 2.0, ESP-IDF ≥ 4.4)
 * Timer      : esp_timer em ISR context → jitter < 5 µs
 *
 * Ligações ao STM32H562:
 *   GPIO CKP_GPIO → PA0  (TIM5_CH1, input capture CKP)
 *   GPIO CMP_GPIO → PA1  (TIM5_CH2, input capture CMP)
 *   GND           → GND  (referência comum OBRIGATÓRIA)
 *   Nota: nível lógico 3.3 V ↔ 3.3 V, sem divisor necessário.
 *         Se o ESP32 operar a 5 V usar divisor resistivo 5 V→3.3 V.
 *
 * Protocolo série (115200 baud):
 *   '+' / '-'  → RPM ± 100
 *   '0'–'9'    → preset: 100/200/300/500/700/1000/1500/2000/3000/5000 RPM
 *   's'        → imprimir estado actual
 *
 * Sinal CKP (60-2):
 *   58 dentes reais, 2 posições em gap.
 *   Duty cycle 50 % por dente.
 *   Período do gap = 3 × T_dente (a partir da RE do último dente real).
 *   O firmware OpenEMS detecta gap quando período_actual > 2 × período_anterior.
 *
 * Sinal CMP:
 *   1 pulso por ciclo completo (720°  = 2 rotações do virabrequim).
 *   Ocorre no dente CMP_TOOTH da revolução 0.
 *   Duração = T/2 (metade do período de dente).
 *
 * ─────────────────────────────────────────────────────────────────────
 * Tabela de tempos por RPM:
 *
 *   RPM    T(µs)  HIGH(µs)  LOW_normal(µs)  LOW_gap(µs)  Rev(ms)
 *   ────   ─────  ────────  ──────────────  ──────────── ───────
 *    200    5000     2500           2500         12500    300.0
 *    500    2000     1000           1000          5000    120.0
 *   1000    1000      500            500          2500     60.0
 *   3000     333      166            166           832     20.0
 *
 * ─────────────────────────────────────────────────────────────────────
 */

#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"

// ── Configuração ─────────────────────────────────────────────────────────────

#define CKP_GPIO   ((gpio_num_t)2)   // Sinal CKP → PA0 do STM32
#define CMP_GPIO   ((gpio_num_t)4)   // Sinal CMP → PA1 do STM32
#define LED_GPIO   ((gpio_num_t)LED_BUILTIN)  // LED de estado (pisca por rev.)

static constexpr uint32_t kRpmInit     = 500;  // RPM ao arrancar
static constexpr int      kCmpTooth    = 5;    // Dente onde CMP sobe (0-indexed)
static constexpr int      kRealTeeth   = 58;   // Dentes reais na roda 60-2
static constexpr uint32_t kRpmMin      = 50;
static constexpr uint32_t kRpmMax      = 6000;

// ── Estado da FSM ─────────────────────────────────────────────────────────────

static volatile uint32_t g_rpm        = kRpmInit;
static volatile int      g_tooth      = 0;
static volatile bool     g_high       = false;
// g_revolution: alterna 0/1 a cada volta do virabrequim.
// CMP só é gerado quando g_revolution == 0.
static volatile int      g_revolution = 0;

static esp_timer_handle_t g_ckp_timer = nullptr;
static esp_timer_handle_t g_cmp_off_timer = nullptr;

// Contadores para diagnóstico (sem critical section — leitura aproximada OK)
static volatile uint32_t g_rev_count  = 0;
static volatile uint32_t g_cmp_count  = 0;

// ── ISRs ─────────────────────────────────────────────────────────────────────

// Desliga o pino CMP após duração T/2
static void IRAM_ATTR cmp_off_isr(void*) {
    gpio_set_level(CMP_GPIO, 0);
}

/*
 * ckp_isr — máquina de estados do sinal CKP.
 *
 * Cada chamada alterna o nível do GPIO e agenda a próxima chamada:
 *   Rising  → HIGH durante T/2
 *   Falling → LOW durante T/2 (normal) ou T/2 + 2T (último dente = gap)
 *
 * O intervalo de LOW do gap (5T/2) faz com que o tempo entre a RE do
 * dente 57 e a RE do dente 0 seguinte seja exatamente 3T, condição
 * necessária para o detector 60-2 do firmware OpenEMS.
 */
static void IRAM_ATTR ckp_isr(void*) {
    const uint32_t rpm = g_rpm;
    // T em µs: número de microsegundos entre posições de dente consecutivas
    // (inclui a posição do dente fisicamente ausente no gap).
    const uint32_t T_us = 60000000UL / (rpm * 60UL);

    if (!g_high) {
        // ── Rising edge ──────────────────────────────────────────────
        gpio_set_level(CKP_GPIO, 1);
        g_high = true;

        // Gerar pulso CMP no dente kCmpTooth da revolução 0
        if (g_revolution == 0 && g_tooth == kCmpTooth) {
            gpio_set_level(CMP_GPIO, 1);
            esp_timer_start_once(g_cmp_off_timer, (uint64_t)T_us / 2);
            g_cmp_count++;
        }

        esp_timer_start_once(g_ckp_timer, (uint64_t)T_us / 2);

    } else {
        // ── Falling edge ─────────────────────────────────────────────
        gpio_set_level(CKP_GPIO, 0);
        g_high = false;

        uint64_t low_us;
        g_tooth++;

        if (g_tooth >= kRealTeeth) {
            // Fim da revolução: introduzir gap de 2 períodos extra.
            // LOW total = T/2 + 2T = 5T/2 → gap de 3T desde a última RE.
            low_us = (uint64_t)T_us / 2 + (uint64_t)T_us * 2;
            g_tooth = 0;
            g_revolution ^= 1;
            g_rev_count++;
            // Piscar LED a cada revolução par (visível a ~4 Hz a 500 RPM)
            if ((g_rev_count & 0x03u) == 0) {
                gpio_set_level(LED_GPIO, (g_rev_count >> 2) & 1u);
            }
        } else {
            low_us = (uint64_t)T_us / 2;
        }

        esp_timer_start_once(g_ckp_timer, low_us);
    }
}

// ── Inicialização ─────────────────────────────────────────────────────────────

static void ckp_print_params(uint32_t rpm) {
    const uint32_t T_us = 60000000UL / (rpm * 60UL);
    Serial.printf(
        "[CKP] RPM=%lu  T=%lu µs  HIGH=%lu µs  LOW_gap=%lu µs  rev=%.1f ms\n",
        rpm, T_us, T_us / 2, T_us / 2 + T_us * 2, T_us * 60.0f / 1000.0f
    );
}

void setup() {
    Serial.begin(115200);
    delay(200);  // aguardar USB enumerar

    // Configurar GPIOs
    gpio_reset_pin(CKP_GPIO);
    gpio_reset_pin(CMP_GPIO);
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(CKP_GPIO,  GPIO_MODE_OUTPUT);
    gpio_set_direction(CMP_GPIO,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO,  GPIO_MODE_OUTPUT);
    gpio_set_level(CKP_GPIO, 0);
    gpio_set_level(CMP_GPIO, 0);
    gpio_set_level(LED_GPIO, 0);

    // Criar timer CKP (ISR context — menor jitter, sem overhead de task)
    {
        esp_timer_create_args_t a = {};
        a.callback         = ckp_isr;
        a.dispatch_method  = ESP_TIMER_ISR;  // requer ESP-IDF >= 4.4
        a.name             = "ckp";
        a.skip_unhandled_events = false;
        ESP_ERROR_CHECK(esp_timer_create(&a, &g_ckp_timer));
    }

    // Criar timer para desligar CMP após T/2
    {
        esp_timer_create_args_t a = {};
        a.callback        = cmp_off_isr;
        a.dispatch_method = ESP_TIMER_ISR;
        a.name            = "cmp_off";
        a.skip_unhandled_events = false;
        ESP_ERROR_CHECK(esp_timer_create(&a, &g_cmp_off_timer));
    }

    // Primeira chamada: arrancar com rising edge
    const uint32_t T0 = 60000000UL / (g_rpm * 60UL);
    ESP_ERROR_CHECK(esp_timer_start_once(g_ckp_timer, T0 / 2));

    Serial.println("=== OpenEMS CKP Generator (ESP32) ===");
    Serial.println("Comandos: '+'/'-' RPM±100 | '0'-'9' preset | 's' estado");
    ckp_print_params(g_rpm);
}

// ── Loop principal (só interface série) ──────────────────────────────────────

static const uint32_t kRpmPresets[] = {
    100, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000
};

void loop() {
    if (Serial.available()) {
        const char c = (char)Serial.read();

        uint32_t new_rpm = g_rpm;
        if      (c == '+') new_rpm = min(g_rpm + 100u, kRpmMax);
        else if (c == '-') new_rpm = max(g_rpm - 100u, kRpmMin);
        else if (c >= '0' && c <= '9') new_rpm = kRpmPresets[c - '0'];
        else if (c == 's') {
            Serial.printf("[CKP] RPM=%lu  revolutions=%lu  cmp_pulses=%lu\n",
                          g_rpm, g_rev_count, g_cmp_count);
        }

        if (new_rpm != g_rpm) {
            g_rpm = new_rpm;
            ckp_print_params(g_rpm);
        }
    }
    delay(100);
}
