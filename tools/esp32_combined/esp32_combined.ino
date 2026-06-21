/*
 * esp32_combined.ino — CKP Generator + Logic Scope para bancada OpenEMS
 * ═══════════════════════════════════════════════════════════════════════
 * Um único ESP32 substitui os dois sketches separados (esp32_ckp_gen +
 * esp32_scope). O timer ISR que gera o sinal CKP também escreve os eventos
 * directamente no ring buffer do scope — não é necessário fio de loopback.
 *
 * Plataforma : ESP32 (Arduino core ≥ 2.0, ESP-IDF ≥ 4.4)
 * Resolução  : 1 µs  (esp_timer_get_time)
 *
 * ── Ligações ao STM32H562 ───────────────────────────────────────────────────
 *
 *   ESP32 GPIO  STM32      Função
 *   ──────────  ─────────  ────────────────────
 *   GPIO 2  →  PA0        CKP output (60-2)
 *   GPIO 4  →  PA1        CMP output (1 pulso/ciclo)
 *   GPIO 32 ←  PA8        IGN0 (TIM1_CH1)
 *   GPIO 33 ←  PE11       IGN1 (TIM1_CH2)
 *   GPIO 25 ←  PE13       IGN2 (TIM1_CH3)
 *   GPIO 26 ←  PE14       IGN3 (TIM1_CH4)
 *   GPIO 27 ←  PC6        INJ0 (TIM2_CH1)
 *   GPIO 14 ←  PC7        INJ1 (TIM2_CH2)
 *   GPIO 12 ←  PB10       INJ2 (TIM2_CH3)
 *   GPIO 13 ←  PC4        INJ3 (TIM2_CH4)
 *   GND     —  GND        OBRIGATÓRIO
 *
 *   Nota: NÃO ligar GPIO36 (era o loopback CKP — aqui desnecessário).
 *
 * ── Comandos série (115200 baud) ────────────────────────────────────────────
 *
 *   CKP generator:
 *     +  / -   RPM ± 100
 *     0  – 9   presets: 100/200/300/500/700/1000/1500/2000/3000/5000 RPM
 *     S        estado do gerador (RPM, contadores)
 *
 *   Scope:
 *     l        Live table (actualiza a cada 1 s)  [default]
 *     e        Edge log (cada bordo)
 *     p        Pulse log (cada pulso completo)
 *     w        Waveform bar (últimos 300 ms)
 *     t        Timing analysis: sequência IGN + ângulo de avanço
 *     s        Estatísticas por canal
 *     r        Reset estatísticas
 *     ?        Esta ajuda
 *
 *   Conflito: 's' scope e 'S' gerador usam letras diferentes intencionalmente.
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <cstring>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 1 — CKP/CMP Generator
// ═══════════════════════════════════════════════════════════════════════════

#define CKP_GPIO    ((gpio_num_t)2)
#define CMP_GPIO    ((gpio_num_t)4)
#ifndef LED_BUILTIN
#define LED_GPIO    ((gpio_num_t)GPIO_NUM_MAX)
#else
#define LED_GPIO    ((gpio_num_t)LED_BUILTIN)
#endif

static constexpr uint32_t kRpmInit   = 500;
static constexpr int      kCmpTooth  = 5;
static constexpr int      kRealTeeth = 58;
static constexpr uint32_t kRpmMin    = 50;
static constexpr uint32_t kRpmMax    = 6000;

static const uint32_t kRpmPresets[] = {
    100, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000
};

static volatile uint32_t g_rpm        = kRpmInit;
static volatile int      g_tooth      = 0;
static volatile bool     g_ckp_high   = false;
static volatile int      g_revolution = 0;
static volatile uint32_t g_rev_count  = 0;
static volatile uint32_t g_cmp_count  = 0;

static esp_timer_handle_t g_ckp_timer    = nullptr;
static esp_timer_handle_t g_cmp_off_tmr  = nullptr;

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 2 — Ring Buffer e Canais (Scope)
// ═══════════════════════════════════════════════════════════════════════════

struct EdgeEvent {
    int64_t ts_us;
    uint8_t ch;
    uint8_t level;
};

static constexpr int kBufSize = 2048;
static constexpr int kBufMask = kBufSize - 1;

static volatile EdgeEvent g_buf[kBufSize];
static volatile uint16_t  g_head = 0;  // escrito por ISRs
static volatile uint16_t  g_tail = 0;  // lido por loop()

// Empurrar evento para o buffer (chamado de ISR ou timer ISR)
static inline void IRAM_ATTR buf_push(int64_t ts, uint8_t ch, uint8_t level) {
    const uint16_t next = (g_head + 1u) & kBufMask;
    if (next != g_tail) {
        g_buf[g_head].ts_us = ts;
        g_buf[g_head].ch    = ch;
        g_buf[g_head].level = level;
        g_head = next;
    }
    // Se buffer cheio: evento descartado (incremento de overflow tratado no process_events)
}

// ── Definição de canais scope ─────────────────────────────────────────────
// CH8 (CKP) é virtual: alimentado pelo timer ISR, não tem GPIO físico.
// gpio = GPIO_NUM_MAX indica canal virtual — não instalar ISR GPIO.

struct ChanDef {
    gpio_num_t  gpio;
    const char  name[6];
    const char  pin[6];
    bool        enabled;
};

static constexpr gpio_num_t kVirtual = GPIO_NUM_MAX;

static ChanDef kChan[] = {
    { GPIO_NUM_32,  "IGN0", "PA8",  true  },   // CH0
    { GPIO_NUM_33,  "IGN1", "PE11", true  },   // CH1
    { GPIO_NUM_25,  "IGN2", "PE13", true  },   // CH2
    { GPIO_NUM_26,  "IGN3", "PE14", true  },   // CH3
    { GPIO_NUM_27,  "INJ0", "PC6",  true  },   // CH4
    { GPIO_NUM_14,  "INJ1", "PC7",  true  },   // CH5
    { GPIO_NUM_12,  "INJ2", "PB10", true  },   // CH6
    { GPIO_NUM_13,  "INJ3", "PC4",  true  },   // CH7
    { kVirtual,     "CKP",  "PA0",  true  },   // CH8 — virtual, via timer ISR
};
static constexpr int kNChan = (int)(sizeof(kChan) / sizeof(kChan[0]));
static constexpr int kCkpChan = 8;  // índice do canal CKP

// ── Métricas por canal ────────────────────────────────────────────────────

struct ChanMetrics {
    int64_t  last_rise_us;
    int64_t  last_fall_us;
    int64_t  last_event_us;
    uint32_t pw_us;
    uint32_t period_us;
    uint32_t pw_min_us;
    uint32_t pw_max_us;
    uint64_t pw_sum_us;
    uint32_t pulse_count;
    uint32_t overflow_count;
};

static ChanMetrics g_m[kNChan];

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 3 — Timer ISRs (CKP + CMP)
// ═══════════════════════════════════════════════════════════════════════════

static void IRAM_ATTR cmp_off_isr(void*) {
    gpio_set_level(CMP_GPIO, 0);
}

/*
 * ckp_isr — Gera sinal CKP (60-2) E alimenta o scope (CH8).
 *
 * Cada chamada: alterna nível GPIO2, agenda próxima chamada, e empurra
 * um EdgeEvent para o ring buffer com timestamp esp_timer_get_time().
 * O gap é detectado pelo timing_feed() através do intervalo fall→rise.
 */
static void IRAM_ATTR ckp_isr(void*) {
    const uint32_t rpm   = g_rpm;
    const uint32_t T_us  = 60000000UL / (rpm * 60UL);
    const int64_t  ts    = esp_timer_get_time();

    if (!g_ckp_high) {
        // ── Rising edge ────────────────────────────────────────────
        gpio_set_level(CKP_GPIO, 1);
        g_ckp_high = true;

        // Registar no scope (CH8, nível 1)
        buf_push(ts, (uint8_t)kCkpChan, 1u);

        // CMP no dente kCmpTooth da revolução 0
        if (g_revolution == 0 && g_tooth == kCmpTooth) {
            gpio_set_level(CMP_GPIO, 1);
            esp_timer_start_once(g_cmp_off_tmr, (uint64_t)T_us / 2);
            g_cmp_count++;
        }

        esp_timer_start_once(g_ckp_timer, (uint64_t)T_us / 2);

    } else {
        // ── Falling edge ───────────────────────────────────────────
        gpio_set_level(CKP_GPIO, 0);
        g_ckp_high = false;

        // Registar no scope (CH8, nível 0)
        buf_push(ts, (uint8_t)kCkpChan, 0u);

        uint64_t low_us;
        g_tooth++;

        if (g_tooth >= kRealTeeth) {
            // Fim da revolução: gap = T/2 + 2T (total LOW = 5T/2)
            low_us = (uint64_t)T_us / 2 + (uint64_t)T_us * 2;
            g_tooth = 0;
            g_revolution ^= 1;
            g_rev_count++;
            if ((g_rev_count & 0x03u) == 0) {
                if (LED_GPIO != GPIO_NUM_MAX)
                    gpio_set_level(LED_GPIO, (g_rev_count >> 2) & 1u);
            }
        } else {
            low_us = (uint64_t)T_us / 2;
        }

        esp_timer_start_once(g_ckp_timer, low_us);
    }
}

// ── ISR GPIO scope ────────────────────────────────────────────────────────

static void IRAM_ATTR edge_isr(void* arg) {
    const uint8_t ch  = (uint8_t)(uint32_t)arg;
    const int64_t ts  = esp_timer_get_time();
    const int     lvl = gpio_get_level(kChan[ch].gpio);
    buf_push(ts, ch, (uint8_t)lvl);
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 4 — Timing Analysis (IGN sequência + avanço)
// ═══════════════════════════════════════════════════════════════════════════

static constexpr int kIgnFirst = 0;
static constexpr int kIgnCount = 4;
static constexpr uint8_t kExpectedFiringOrder[kIgnCount] = {0, 2, 3, 1};

enum class TmState : uint8_t { IDLE, WAIT_GAP, CAPTURE, DONE };

struct TimingCapture {
    int64_t  gap_ts_us;
    int64_t  last_ckp_fall_us;
    int64_t  spark_ts_us[kIgnCount];
    bool     spark_done[kIgnCount];
    uint32_t ckp_period_us;
    int      sparks_captured;
    uint32_t timeout_us;
};

static TmState       g_tm_state = TmState::IDLE;
static TimingCapture g_tm_cap;

static void timing_report();

static void timing_start() {
    g_tm_cap              = {};
    g_tm_cap.ckp_period_us = (g_m[kCkpChan].period_us > 0)
                             ? g_m[kCkpChan].period_us : 2000u;
    g_tm_cap.timeout_us   = g_tm_cap.ckp_period_us * 120u;
    g_tm_state = TmState::WAIT_GAP;
    Serial.println("  [TIMING] A aguardar gap CKP...");
    Serial.printf("  [TIMING] T_dente=%.3f ms  timeout=%.0f ms\n",
                  g_tm_cap.ckp_period_us / 1000.0f,
                  g_tm_cap.timeout_us    / 1000.0f);
}

static void timing_feed(const EdgeEvent& ev) {
    TimingCapture& c = g_tm_cap;

    if (g_tm_state == TmState::WAIT_GAP) {
        if (ev.ch != (uint8_t)kCkpChan) { return; }
        if (ev.level == 0u) {
            c.last_ckp_fall_us = ev.ts_us;
        } else if (c.last_ckp_fall_us > 0) {
            const int64_t gap = ev.ts_us - c.last_ckp_fall_us;
            // Threshold: 1.8 × T_dente (gap real = 5T/2 = 2.5T)
            if (gap > (int64_t)(c.ckp_period_us * 18u / 10u)) {
                c.gap_ts_us = ev.ts_us;
                g_tm_state  = TmState::CAPTURE;
                Serial.printf("  [TIMING] Gap OK (%.2f ms). A capturar 4 sparks...\n",
                              gap / 1000.0f);
            }
        }
        return;
    }

    if (g_tm_state == TmState::CAPTURE) {
        // Timeout
        if ((ev.ts_us - c.gap_ts_us) > (int64_t)c.timeout_us) {
            Serial.println("  [TIMING] Timeout — verificar ligações IGN (PC6-9).");
            g_tm_state = TmState::IDLE;
            return;
        }
        // FALL dos canais IGN = instante do spark
        if (ev.level == 0u &&
            ev.ch >= (uint8_t)kIgnFirst &&
            ev.ch <  (uint8_t)(kIgnFirst + kIgnCount)) {
            const int idx = (int)(ev.ch - kIgnFirst);
            if (!c.spark_done[idx]) {
                c.spark_ts_us[idx] = ev.ts_us;
                c.spark_done[idx]  = true;
                c.sparks_captured++;
                Serial.printf("  [TIMING] IGN%d spark @ +%.3f ms\n",
                              idx, (ev.ts_us - c.gap_ts_us) / 1000.0f);
                if (c.sparks_captured == kIgnCount) {
                    g_tm_state = TmState::DONE;
                    timing_report();
                }
            }
        }
    }
}

static void timing_report() {
    const TimingCapture& c  = g_tm_cap;
    const float          T  = c.ckp_period_us / 1000.0f;
    const float expected_ic = T * 60.0f * 2.0f / 4.0f;  // T × 30 ms

    // Ordenar canais por timestamp
    uint8_t order[kIgnCount] = {0, 1, 2, 3};
    for (int i = 0; i < kIgnCount - 1; ++i)
        for (int j = 0; j < kIgnCount - 1 - i; ++j)
            if (c.spark_ts_us[order[j]] > c.spark_ts_us[order[j + 1]]) {
                uint8_t t = order[j]; order[j] = order[j+1]; order[j+1] = t;
            }

    Serial.println();
    Serial.println("  ╔══════════════════════════════════════════════════════╗");
    Serial.println("  ║  Ignition Timing Analysis                            ║");
    Serial.printf( "  ║  T_dente=%.3f ms   Inter-cil esperado=%.3f ms       ║\n",
                   T, expected_ic);
    Serial.println("  ╚══════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("  Canal   Desde gap     Inter-cil    Esperado   Desvio   OK?");
    Serial.println("  ──────  ────────────  ──────────   ────────   ──────   ───");

    float prev_ms = 0.0f;
    bool  timing_ok = true;
    for (int i = 0; i < kIgnCount; ++i) {
        const int   ch  = order[i];
        const float dt  = (c.spark_ts_us[ch] - c.gap_ts_us) / 1000.0f;
        const float ic  = (i == 0) ? 0.0f : dt - prev_ms;
        const float dev = (i == 0) ? 0.0f : ic - expected_ic;
        const bool  ok  = (i == 0) || (fabsf(dev) < 1.5f);
        if (!ok) timing_ok = false;
        if (i == 0)
            Serial.printf("  IGN%d   %8.3f ms      —            —          —       —\n",
                          ch, dt);
        else
            Serial.printf("  IGN%d   %8.3f ms  %8.3f ms  %7.3f ms  %+6.3f ms  %s\n",
                          ch, dt, ic, expected_ic, dev, ok ? "✓" : "✗");
        prev_ms = dt;
    }

    // Ordem de disparo
    Serial.println();
    Serial.print("  Detectada : ");
    for (int i = 0; i < kIgnCount; ++i) { if (i) Serial.print("→"); Serial.printf("IGN%d", order[i]); }
    Serial.println();
    Serial.print("  Esperada  : ");
    for (int i = 0; i < kIgnCount; ++i) { if (i) Serial.print("→"); Serial.printf("IGN%d", kExpectedFiringOrder[i]); }
    bool order_ok = true;
    for (int i = 0; i < kIgnCount; ++i) if (order[i] != kExpectedFiringOrder[i]) order_ok = false;
    Serial.printf("  %s\n", order_ok ? "  ✓ CORRECTA" : "  ✗ ERRADA");

    // Ângulo desde gap
    const int   first = order[0];
    const float dt0   = (c.spark_ts_us[first] - c.gap_ts_us) / 1000.0f;
    const float teeth = (T > 0.0f) ? dt0 / T : 0.0f;
    Serial.println();
    Serial.printf("  IGN%d (1º spark): +%.3f ms = %.2f dentes = %.1f° desde dente 0\n",
                  first, dt0, teeth, teeth * 6.0f);
    Serial.printf("\n  Resultado final: %s\n",
                  (timing_ok && order_ok) ? "✓ IGNIÇÃO OK" : "✗ VER FALHAS ACIMA");
    Serial.println();

    g_tm_state = TmState::IDLE;
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 5 — Modos de visualização
// ═══════════════════════════════════════════════════════════════════════════

enum class Mode : uint8_t { LIVE, EDGE, PULSE, WAVE, TIMING };
static volatile Mode g_mode = Mode::LIVE;

static void process_events() {
    while (g_tail != g_head) {
        const EdgeEvent ev = {
            .ts_us = g_buf[g_tail].ts_us,
            .ch    = g_buf[g_tail].ch,
            .level = g_buf[g_tail].level,
        };
        g_tail = (g_tail + 1u) & kBufMask;
        if (ev.ch >= kNChan) continue;

        if (g_tm_state == TmState::WAIT_GAP || g_tm_state == TmState::CAPTURE)
            timing_feed(ev);

        ChanMetrics& m = g_m[ev.ch];
        m.last_event_us = ev.ts_us;

        if (ev.level == 1u) {
            if (m.last_rise_us > 0) {
                const int64_t p = ev.ts_us - m.last_rise_us;
                if (p > 0 && p < 10000000LL) m.period_us = (uint32_t)p;
            }
            m.last_rise_us = ev.ts_us;
            if (g_mode == Mode::EDGE)
                Serial.printf("  %10lld µs  CH%d %-5s RISE\n",
                              ev.ts_us, ev.ch, kChan[ev.ch].name);
        } else {
            if (m.last_rise_us > 0) {
                const int64_t pw = ev.ts_us - m.last_rise_us;
                if (pw > 0 && pw < 5000000LL) {
                    m.pw_us = (uint32_t)pw;
                    m.pulse_count++;
                    m.pw_sum_us += (uint64_t)pw;
                    if (m.pw_min_us == 0 || pw < m.pw_min_us) m.pw_min_us = (uint32_t)pw;
                    if (pw > m.pw_max_us) m.pw_max_us = (uint32_t)pw;
                }
            }
            m.last_fall_us = ev.ts_us;
            if (g_mode == Mode::EDGE)
                Serial.printf("  %10lld µs  CH%d %-5s FALL  PW=%.3f ms\n",
                              ev.ts_us, ev.ch, kChan[ev.ch].name, m.pw_us / 1000.0f);
            if (g_mode == Mode::PULSE && m.pw_us > 0)
                Serial.printf("  CH%d %-5s  PW=%7.3f ms  Period=%7.3f ms  Count=%lu\n",
                              ev.ch, kChan[ev.ch].name,
                              m.pw_us / 1000.0f, m.period_us / 1000.0f, m.pulse_count);
        }
    }
}

static void print_live_table() {
    const int64_t now = esp_timer_get_time();

    // RPM a partir do período CKP (CH8): 60 dentes × 2 revoluções / período
    const uint32_t ckp_per = g_m[kCkpChan].period_us;
    const float    rpm_ckp = (ckp_per > 0)
                             ? (60000000.0f / (float)ckp_per)   // 1 dente = 1/60 rev
                             : 0.0f;

    Serial.println();
    Serial.println("+──+──────+───────+────────+────────+────────+───────+─────────+");
    Serial.printf( "│  OpenEMS Bench Scope  RPM_CKP=%.0f                          │\n",
                   rpm_ckp);
    Serial.println("+──+──────+───────+────────+────────+────────+───────+─────────+");
    Serial.println("│CH│ Name │STM32  │PW (ms) │Per(ms) │Freq(Hz)│ Count │ Status  │");
    Serial.println("+──+──────+───────+────────+────────+────────+───────+─────────+");

    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) continue;
        const ChanMetrics& m = g_m[ch];
        const bool  idle = (now - m.last_event_us) > 2000000LL;
        const float pw   = m.pw_us    / 1000.0f;
        const float per  = m.period_us / 1000.0f;
        const float freq = (m.period_us > 0) ? 1e6f / m.period_us : 0.0f;
        const char* src  = (ch == kCkpChan) ? "PA0*" : kChan[ch].pin;
        Serial.printf("│%2d│%-6s│%-7s│%8.3f│%8.3f│%8.2f│%7lu│%s│\n",
                      ch, kChan[ch].name, src,
                      pw, per, freq, m.pulse_count,
                      idle ? "  IDLE   " : "  OK     ");
    }
    Serial.println("+──+──────+───────+────────+────────+────────+───────+─────────+");
    Serial.println("  * CKP: virtual — gerado internamente, sem pino GPIO.");
    for (int ch = 0; ch < kNChan; ++ch)
        if (g_m[ch].overflow_count > 0)
            Serial.printf("  AVISO CH%d: %lu overflow(s)\n", ch, g_m[ch].overflow_count);
}

static void print_stats() {
    Serial.println();
    Serial.println("  CH  Name   STM32  Count     PW_last   PW_min    PW_max    PW_avg");
    Serial.println("  ──  ─────  ─────  ─────     ───────   ──────    ──────    ──────");
    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) continue;
        const ChanMetrics& m = g_m[ch];
        const float avg = (m.pulse_count > 0)
                          ? (float)(m.pw_sum_us / m.pulse_count) / 1000.0f : 0.0f;
        Serial.printf("  %2d  %-5s  %-5s  %7lu   %7.3fms  %7.3fms  %7.3fms  %7.3fms\n",
                      ch, kChan[ch].name, kChan[ch].pin,
                      m.pulse_count,
                      m.pw_us     / 1000.0f,
                      m.pw_min_us / 1000.0f,
                      m.pw_max_us / 1000.0f,
                      avg);
    }
}

static void print_help() {
    Serial.println();
    Serial.println("  ╔══════════════════════════════════════════════════╗");
    Serial.println("  ║  OpenEMS ESP32 Combined — CKP Gen + Logic Scope  ║");
    Serial.println("  ╚══════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("  CKP Generator:");
    Serial.println("    +/-     RPM ± 100");
    Serial.println("    0–9     Presets: 100/200/300/500/700/1000/1500/2000/3000/5000");
    Serial.println("    S       Estado do gerador");
    Serial.println();
    Serial.println("  Scope:");
    Serial.println("    l       Live table (1 s)     p  Pulse log");
    Serial.println("    e       Edge log              w  Waveform bar");
    Serial.println("    t       Timing analysis IGN   s  Estatísticas");
    Serial.println("    r       Reset stats           ?  Esta ajuda");
    Serial.println();
    Serial.println("  Canais:");
    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) continue;
        if (kChan[ch].gpio == kVirtual)
            Serial.printf("    CH%d  %-5s  (virtual — gerado internamente)\n",
                          ch, kChan[ch].name);
        else
            Serial.printf("    CH%d  %-5s  STM32 %-5s → GPIO%d\n",
                          ch, kChan[ch].name, kChan[ch].pin, (int)kChan[ch].gpio);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── setup() e loop()
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(300);
    memset(g_m, 0, sizeof(g_m));

    // ── GPIOs de saída (CKP/CMP/LED) ─────────────────────────────────────
    {
        gpio_num_t outs[] = {CKP_GPIO, CMP_GPIO, LED_GPIO};
        for (size_t i = 0; i < (sizeof(outs) / sizeof(outs[0])); ++i) {
            gpio_num_t g = outs[i];
            if (g == GPIO_NUM_MAX) continue;
            gpio_reset_pin(g);
            gpio_set_direction(g, GPIO_MODE_OUTPUT);
            gpio_set_level(g, 0);
        }
    }

    // ── GPIOs de entrada (scope, apenas canais físicos) ───────────────────
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled || kChan[ch].gpio == kVirtual) continue;
        const gpio_num_t g = kChan[ch].gpio;
        gpio_reset_pin(g);
        gpio_set_direction(g, GPIO_MODE_INPUT);
        gpio_set_pull_mode(g, GPIO_PULLDOWN_ONLY);
        gpio_set_intr_type(g, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(g, edge_isr, (void*)(uint32_t)ch);
        gpio_intr_enable(g);
    }

    // ── Timer CKP (ISR context) ───────────────────────────────────────────
    {
        esp_timer_create_args_t a = {};
        a.callback = ckp_isr;
        a.arg = nullptr;
        a.name = "ckp";
        ESP_ERROR_CHECK(esp_timer_create(&a, &g_ckp_timer));
    }

    // ── Timer CMP off (ISR context) ───────────────────────────────────────
    {
        esp_timer_create_args_t a = {};
        a.callback = cmp_off_isr;
        a.arg = nullptr;
        a.name = "cmp_off";
        ESP_ERROR_CHECK(esp_timer_create(&a, &g_cmp_off_tmr));
    }

    // ── Arrancar CKP ─────────────────────────────────────────────────────
    const uint32_t T0 = 60000000UL / (g_rpm * 60UL);
    ESP_ERROR_CHECK(esp_timer_start_once(g_ckp_timer, T0 / 2));

    print_help();
    Serial.printf("\n  CKP: %lu RPM  Modo scope: LIVE\n\n", g_rpm);
}

static uint32_t g_last_live_ms = 0;

void loop() {
    process_events();

    if (Serial.available()) {
        const char c = (char)Serial.read();

        // CKP generator commands
        uint32_t new_rpm = g_rpm;
        if      (c == '+') new_rpm = min(g_rpm + 100u, kRpmMax);
        else if (c == '-') new_rpm = max(g_rpm - 100u, kRpmMin);
        else if (c >= '0' && c <= '9') new_rpm = kRpmPresets[c - '0'];
        else if (c == 'S') {
            const uint32_t T_us = 60000000UL / (g_rpm * 60UL);
            Serial.printf("  [GEN] RPM=%lu  T=%lu µs  revs=%lu  cmp=%lu\n",
                          g_rpm, T_us, g_rev_count, g_cmp_count);
        }

        if (new_rpm != g_rpm) {
            g_rpm = new_rpm;
            const uint32_t T_us = 60000000UL / (g_rpm * 60UL);
            Serial.printf("  [GEN] RPM=%lu  T=%lu µs\n", g_rpm, T_us);
        }

        // Scope commands
        switch (c) {
            case 'l': g_mode = Mode::LIVE;  Serial.println("  Modo: LIVE");   break;
            case 'e': g_mode = Mode::EDGE;  Serial.println("  Modo: EDGE");   break;
            case 'p': g_mode = Mode::PULSE; Serial.println("  Modo: PULSE");  break;
            case 'w': g_mode = Mode::WAVE;  Serial.println("  Modo: WAVE");   break;
            case 't': g_mode = Mode::TIMING; timing_start(); break;
            case 's': print_stats(); break;
            case 'r':
                for (int ch = 0; ch < kNChan; ++ch) {
                    g_m[ch].pw_min_us = 0; g_m[ch].pw_max_us = 0;
                    g_m[ch].pw_sum_us = 0; g_m[ch].pulse_count = 0;
                    g_m[ch].overflow_count = 0;
                }
                Serial.println("  Stats reset.");
                break;
            case '?': print_help(); break;
            default: break;
        }
        g_last_live_ms = 0;  // forçar refresh no modo LIVE
    }

    if (g_mode == Mode::LIVE) {
        const uint32_t now = millis();
        if (now - g_last_live_ms >= 1000u) {
            g_last_live_ms = now;
            print_live_table();
        }
    }

    if (g_mode == Mode::WAVE) {
        static uint32_t last_wave = 0;
        const uint32_t now = millis();
        if (now - last_wave >= 500u) {
            last_wave = now;
            // Waveform simplificado: PW e período de cada canal
            const int64_t ts = esp_timer_get_time();
            for (int ch = 0; ch < kNChan; ++ch) {
                if (!kChan[ch].enabled || g_m[ch].period_us == 0) continue;
                Serial.printf("  CH%d %-5s  PW=%.3f ms  T=%.3f ms\n",
                              ch, kChan[ch].name,
                              g_m[ch].pw_us    / 1000.0f,
                              g_m[ch].period_us / 1000.0f);
            }
        }
    }

    delay(5);
}
