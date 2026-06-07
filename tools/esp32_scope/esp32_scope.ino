/*
 * esp32_scope.ino — Osciloscópio lógico para bancada OpenEMS
 * ════════════════════════════════════════════════════════════
 * Plataforma : ESP32 (Arduino core ≥ 2.0)
 * Resolução  : 1 µs  (esp_timer_get_time)
 * Latência   : 2–5 µs por bordo (ISR context)
 *
 * Sinais monitorizados:
 *   CH0 IGN0  ← PC6  TIM8_CH1  (bobina cil.0)
 *   CH1 IGN1  ← PC7  TIM8_CH2  (bobina cil.1)
 *   CH2 IGN2  ← PC8  TIM8_CH3  (bobina cil.2)
 *   CH3 IGN3  ← PC9  TIM8_CH4  (bobina cil.3)
 *   CH4 INJ0  ← PA15 TIM2_CH1  (injector cil.0)
 *   CH5 INJ1  ← PB3  TIM2_CH2  (injector cil.1)
 *   CH6 INJ2  ← PB10 TIM2_CH3  (injector cil.2)
 *   CH7 INJ3  ← PB11 TIM2_CH4  (injector cil.3)
 *   CH8 CKP   ← PA0  (loopback — ligar ao gerador CKP)
 *
 * Ligações STM32H562 → ESP32:
 *   PC6  → GPIO 32    PC7  → GPIO 33
 *   PC8  → GPIO 25    PC9  → GPIO 26
 *   PA15 → GPIO 27    PB3  → GPIO 14
 *   PB10 → GPIO 12    PB11 → GPIO 13
 *   PA0  → GPIO 36    GND  → GND  ← OBRIGATÓRIO
 *
 * Comandos série (115200 baud):
 *   l  — Live table (actualiza a cada 1 s) [default]
 *   e  — Edge log (imprime cada bordo)
 *   p  — Pulse log (imprime cada pulso completo)
 *   w  — Waveform bar da última janela de 300 ms
 *   t  — Timing analysis: sequência IGN + ângulo de avanço
 *   r  — Reset estatísticas
 *   ?  — Ajuda
 *
 * Nota: Se o mesmo ESP32 gerar o sinal CKP (esp32_ckp_gen.ino),
 * ligar GPIO 2 → GPIO 36 com um fio para monitorizar o CKP gerado.
 * ════════════════════════════════════════════════════════════
 */

#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"

// ── Definição de canais ───────────────────────────────────────────────────────

struct ChanDef {
    gpio_num_t  gpio;
    const char  name[6];     // nome curto (ex: "IGN0")
    const char  pin[6];      // pino STM32 (ex: "PC6")
    bool        enabled;
};

// Editar GPIOs conforme o seu DevKit.
// GPIO 34-39: input-only (VP, VN, etc.) — bons para monitorização.
// GPIO 6-11: flash SPI — NÃO usar.
static ChanDef kChan[] = {
    { GPIO_NUM_32, "IGN0",  "PC6",  true },
    { GPIO_NUM_33, "IGN1",  "PC7",  true },
    { GPIO_NUM_25, "IGN2",  "PC8",  true },
    { GPIO_NUM_26, "IGN3",  "PC9",  true },
    { GPIO_NUM_27, "INJ0",  "PA15", true },
    { GPIO_NUM_14, "INJ1",  "PB3",  true },
    { GPIO_NUM_12, "INJ2",  "PB10", true },
    { GPIO_NUM_13, "INJ3",  "PB11", true },
    { GPIO_NUM_36, "CKP",   "PA0",  true },  // input-only; loopback ou directo
};
static constexpr int kNChan = (int)(sizeof(kChan) / sizeof(kChan[0]));

// ── Buffer de eventos (lock-free SPSC) ────────────────────────────────────────

struct EdgeEvent {
    int64_t ts_us;   // tempo absoluto em µs
    uint8_t ch;      // índice do canal
    uint8_t level;   // 1=subida, 0=descida
};

static constexpr int kBufSize = 2048;
static constexpr int kBufMask = kBufSize - 1;

static volatile EdgeEvent g_buf[kBufSize];
static volatile uint16_t  g_head = 0;   // escrito pela ISR
static volatile uint16_t  g_tail = 0;   // lido pelo loop

// ── Métricas por canal ────────────────────────────────────────────────────────

struct ChanMetrics {
    int64_t  last_rise_us;      // timestamp da última subida
    int64_t  last_fall_us;      // timestamp da última descida
    int64_t  last_event_us;     // timestamp do último bordo (qualquer)
    uint32_t pw_us;             // largura do último pulso completo
    uint32_t period_us;         // período rise→rise do último ciclo
    uint32_t pw_min_us;
    uint32_t pw_max_us;
    uint64_t pw_sum_us;
    uint32_t pulse_count;
    uint32_t overflow_count;    // eventos descartados por buffer cheio
};

static ChanMetrics g_m[kNChan];

// ── Modos de visualização ─────────────────────────────────────────────────────

enum class Mode : uint8_t { LIVE, EDGE, PULSE, WAVE, TIMING };
static volatile Mode g_mode = Mode::LIVE;

// ── Máquina de estados para Timing Analysis ───────────────────────────
// Captura uma sequência completa: gap CKP → 4 sparks IGN0-3.
// CH8 (CKP loopback) obrigatório para detecção do gap.

static constexpr int kCkpChan  = 8;   // índice do canal CKP no kChan[]
static constexpr int kIgnFirst = 0;   // primeiro canal IGN
static constexpr int kIgnCount = 4;   // número de cilindros

// Ordem de disparo esperada: IGN0→IGN2→IGN3→IGN1  (1-3-4-2)
static constexpr uint8_t kExpectedFiringOrder[kIgnCount] = {0, 2, 3, 1};

enum class TmState : uint8_t {
    IDLE,
    WAIT_GAP,    // aguardar gap CKP (fall→rise com intervalo > 2T)
    CAPTURE,     // capturar os 4 sparks (FALL dos IGN)
    DONE,
};

struct TimingCapture {
    int64_t  gap_ts_us;                    // timestamp da RE do dente 0 (após gap)
    int64_t  last_ckp_fall_us;             // última descida do CKP
    int64_t  spark_ts_us[kIgnCount];       // FALL de cada canal IGN (= instante do spark)
    bool     spark_done[kIgnCount];        // canal capturado
    uint32_t ckp_period_us;               // T medido pelo scope
    int      sparks_captured;             // quantos já foram capturados
    uint32_t timeout_us;                  // tempo limite de captura
};

static TmState        g_tm_state  = TmState::IDLE;
static TimingCapture  g_tm_cap;

// Limpar captura e iniciar
static void timing_start() {
    g_tm_cap = {};
    g_tm_cap.ckp_period_us = (g_m[kCkpChan].period_us > 0)
                             ? g_m[kCkpChan].period_us : 2000u;
    g_tm_cap.timeout_us    = g_tm_cap.ckp_period_us * 120u; // 1 ciclo completo de margem
    g_tm_state = TmState::WAIT_GAP;
    Serial.println("  [TIMING] A aguardar gap CKP no CH8 (loopback PA0)...");
    if (g_m[kCkpChan].period_us == 0) {
        Serial.println("  AVISO: CH8 sem sinal CKP. Ligar GPIO36 → PA0 do STM32.");
    }
}

// ── ISR ───────────────────────────────────────────────────────────────────────

static void IRAM_ATTR edge_isr(void* arg) {
    const uint8_t ch  = (uint8_t)(uint32_t)arg;
    const int64_t t   = esp_timer_get_time();
    const int     lvl = gpio_get_level(kChan[ch].gpio);

    const uint16_t next = (g_head + 1u) & kBufMask;
    if (next != g_tail) {
        // Escrita sem memcpy — campos individuais para garantir atomicidade
        g_buf[g_head].ts_us = t;
        g_buf[g_head].ch    = ch;
        g_buf[g_head].level = (uint8_t)lvl;
        g_head = next;
    } else {
        // Buffer cheio — incrementar contador (lido depois no loop)
        // Nota: acesso sem lock a g_m é seguro aqui porque overflow_count
        // é o único campo escrito pela ISR (uint32 alinhado = escrita atómica).
        g_m[ch].overflow_count++;
    }
}

// ── Timing Analysis ────────────────────────────────────────────────────────────────────────────────────

static void timing_report();

static void timing_start() {
    g_tm_cap                  = {};
    g_tm_cap.ckp_period_us    = (g_m[kCkpChan].period_us > 0)
                                ? g_m[kCkpChan].period_us : 2000u;
    g_tm_cap.timeout_us       = g_tm_cap.ckp_period_us * 120u; // 1 ciclo + margem
    g_tm_state = TmState::WAIT_GAP;
    Serial.println("  [TIMING] A aguardar gap CKP (CH8 loopback PA0)...");
    if (g_m[kCkpChan].period_us == 0) {
        Serial.println("  AVISO: CH8 sem sinal. Ligar GPIO36 ao PA0 do STM32.");
    }
}

static void timing_feed(const EdgeEvent& ev) {
    TimingCapture& c = g_tm_cap;

    if (g_tm_state == TmState::WAIT_GAP) {
        if (ev.ch != (uint8_t)kCkpChan) { return; }
        if (ev.level == 0u) {
            c.last_ckp_fall_us = ev.ts_us;
        } else if (c.last_ckp_fall_us > 0) {
            const int64_t gap = ev.ts_us - c.last_ckp_fall_us;
            // Threshold: 1.8 × T (gap real é 2.5T)
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
        if ((ev.ts_us - c.gap_ts_us) > (int64_t)c.timeout_us) {
            Serial.println("  [TIMING] Timeout. Verificar ligações IGN (PC6-9).");
            g_tm_state = TmState::IDLE;
            g_mode     = Mode::LIVE;
            return;
        }
        // Capturar FALL de cada canal IGN (falling edge = instante do spark)
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
                    g_mode = Mode::LIVE;
                }
            }
        }
    }
}

static void timing_report() {
    const TimingCapture& c  = g_tm_cap;
    const float          T  = c.ckp_period_us / 1000.0f;   // ms
    // T_cycle = 60 dentes/rev × 2 rev × T_dente
    const float expected_inter_ms = T * 60.0f * 2.0f / 4.0f;  // = T × 30

    // Ordenar canais por timestamp (bubble sort, 4 elementos)
    uint8_t order[kIgnCount] = {0, 1, 2, 3};
    for (int i = 0; i < kIgnCount - 1; ++i)
        for (int j = 0; j < kIgnCount - 1 - i; ++j)
            if (c.spark_ts_us[order[j]] > c.spark_ts_us[order[j + 1]]) {
                uint8_t t = order[j]; order[j] = order[j+1]; order[j+1] = t;
            }

    Serial.println();
    Serial.println("  ╔══════════════════════════════════════════════════════╗");
    Serial.println("  ║  Ignition Timing Analysis                      ║");
    Serial.printf( "  ║  T_dente=%.3f ms  Inter-cil esperado=%.3f ms %s║\n",
                   T, expected_inter_ms, "");
    Serial.println("  ╚══════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("  Canal  Desde gap      Inter-cil     Esperado  Desvio  OK?");
    Serial.println("  ─────  ─────────────  ───────────  ────────  ──────  ───");

    float prev_ms = 0.0f;
    bool  timing_ok = true;
    for (int i = 0; i < kIgnCount; ++i) {
        const int   ch  = order[i];
        const float dt  = (c.spark_ts_us[ch] - c.gap_ts_us) / 1000.0f;
        const float ic  = (i == 0) ? 0.0f : dt - prev_ms;
        const float dev = (i == 0) ? 0.0f : ic - expected_inter_ms;
        const bool  ok  = (i == 0) || (fabsf(dev) < 1.0f);
        if (!ok) { timing_ok = false; }
        if (i == 0)
            Serial.printf("  IGN%d   %8.3f ms        —             —       —      —\n",
                          ch, dt);
        else
            Serial.printf("  IGN%d   %8.3f ms  %8.3f ms  %7.3f ms %+6.3f ms %s\n",
                          ch, dt, ic, expected_inter_ms, dev, ok ? "✓" : "✗ FALHA");
        prev_ms = dt;
    }

    // Ordem de disparo
    Serial.println();
    Serial.print("  Detectada : ");
    for (int i = 0; i < kIgnCount; ++i) {
        if (i) Serial.print("→");
        Serial.printf("IGN%d", order[i]);
    }
    Serial.println();
    Serial.print("  Esperada  : ");
    for (int i = 0; i < kIgnCount; ++i) {
        if (i) Serial.print("→");
        Serial.printf("IGN%d", kExpectedFiringOrder[i]);
    }
    bool order_ok = true;
    for (int i = 0; i < kIgnCount; ++i)
        if (order[i] != kExpectedFiringOrder[i]) { order_ok = false; }
    Serial.printf("  %s\n", order_ok ? "  ✓ CORRECTA" : "  ✗ ERRADA");

    // Ângulo desde gap do primeiro spark
    const int   first = order[0];
    const float dt0   = (c.spark_ts_us[first] - c.gap_ts_us) / 1000.0f;
    const float teeth = dt0 / T;
    const float deg   = teeth * 6.0f;
    Serial.println();
    Serial.printf("  IGN%d (1º spark): +%.3f ms = %.2f dentes = %.1f° desde dente 0\n",
                  first, dt0, teeth, deg);
    Serial.println("  Usar snapshot UART 'A' → byte 8 (advance_p40) para confirmar.");

    Serial.println();
    Serial.printf("  Resultado final: %s\n",
                  (timing_ok && order_ok) ? "✓ IGNIÇÃO OK" : "✗ VER FALHAS ACIMA");
    Serial.println();
}

// ── Processamento de eventos (loop principal) ─────────────────────────────────

static void process_events() {
    while (g_tail != g_head) {
        // Leitura do buffer (sem barreira explícita — ESP32 Xtensa é TSO)
        const EdgeEvent ev = {
            .ts_us = g_buf[g_tail].ts_us,
            .ch    = g_buf[g_tail].ch,
            .level = g_buf[g_tail].level,
        };
        g_tail = (g_tail + 1u) & kBufMask;

        if (ev.ch >= kNChan) { continue; }

        // Alimentar FSM de timing (activa apenas durante modo TIMING)
        if (g_tm_state != TmState::IDLE && g_tm_state != TmState::DONE) {
            timing_feed(ev);
        }

        ChanMetrics& m = g_m[ev.ch];
        m.last_event_us = ev.ts_us;

        if (ev.level == 1u) {
            // Bordo de subida
            if (m.last_rise_us > 0) {
                const int64_t period = ev.ts_us - m.last_rise_us;
                if (period > 0 && period < 10000000LL) {   // < 10 s — sanity
                    m.period_us = (uint32_t)period;
                }
            }
            m.last_rise_us = ev.ts_us;

            if (g_mode == Mode::EDGE) {
                Serial.printf("  %10lld µs  CH%d %-5s RISE\n",
                              ev.ts_us, ev.ch, kChan[ev.ch].name);
            }
        } else {
            // Bordo de descida
            if (m.last_rise_us > 0) {
                const int64_t pw = ev.ts_us - m.last_rise_us;
                if (pw > 0 && pw < 5000000LL) {   // < 5 s — sanity
                    m.pw_us = (uint32_t)pw;
                    m.pulse_count++;
                    m.pw_sum_us += (uint64_t)pw;
                    if (m.pw_min_us == 0 || pw < m.pw_min_us) { m.pw_min_us = (uint32_t)pw; }
                    if (pw > m.pw_max_us) { m.pw_max_us = (uint32_t)pw; }
                }
            }
            m.last_fall_us = ev.ts_us;

            if (g_mode == Mode::EDGE) {
                Serial.printf("  %10lld µs  CH%d %-5s FALL  PW=%.3f ms\n",
                              ev.ts_us, ev.ch, kChan[ev.ch].name,
                              m.pw_us / 1000.0f);
            }
            if (g_mode == Mode::PULSE && m.pw_us > 0) {
                Serial.printf("  CH%d %-5s  PW=%7.3f ms  Period=%7.3f ms  "
                              "Count=%lu\n",
                              ev.ch, kChan[ev.ch].name,
                              m.pw_us / 1000.0f,
                              m.period_us / 1000.0f,
                              m.pulse_count);
            }
        }
    }
}

// ── Tabela LIVE ───────────────────────────────────────────────────────────────

static void print_live_table() {
    const int64_t now = esp_timer_get_time();

    Serial.println();
    Serial.println("+─────────────────────────────────────────────────────────────+");
    Serial.printf( "│  OpenEMS Scope @ %.3f s%36s│\n",
                   now / 1e6, "");
    Serial.println("+──+──────+───────+────────+────────+────────+───────+─────────+");
    Serial.println("│CH│ Name │STM32  │PW (ms) │Per(ms) │Freq(Hz)│ Count │ Status  │");
    Serial.println("+──+──────+───────+────────+────────+────────+───────+─────────+");

    float ign0_rpm = 0.0f;

    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) { continue; }
        const ChanMetrics& m = g_m[ch];

        const bool idle  = (now - m.last_event_us) > 2000000LL;  // > 2 s
        const float pw   = m.pw_us / 1000.0f;
        const float per  = m.period_us / 1000.0f;
        const float freq = (m.period_us > 0) ? 1e6f / m.period_us : 0.0f;

        if (ch == 0 && m.period_us > 0) {
            // RPM a partir do período do IGN0: 1 pulso / ciclo (720°)
            // frequência ciclo = freq Hz → RPM = freq × 60
            ign0_rpm = freq * 60.0f;
        }

        const char* status = idle ? "  IDLE   " : "  OK     ";

        Serial.printf("│%2d│%-6s│%-7s│%8.3f│%8.3f│%8.2f│%7lu│%s│\n",
                      ch, kChan[ch].name, kChan[ch].pin,
                      pw, per, freq,
                      m.pulse_count,
                      status);
    }

    Serial.println("+──+──────+───────+────────+────────+────────+───────+─────────+");

    if (ign0_rpm > 0) {
        Serial.printf("  RPM estimado (IGN0 period): %.1f\n", ign0_rpm);
    }

    // Avisos
    for (int ch = 0; ch < kNChan; ++ch) {
        if (g_m[ch].overflow_count > 0) {
            Serial.printf("  AVISO CH%d: %lu eventos descartados (buffer overflow)\n",
                          ch, g_m[ch].overflow_count);
        }
    }
}

// ── Waveform bar ──────────────────────────────────────────────────────────────
// Mostra os últimos kWaveWidthMs ms de cada canal como barra de texto.
// Resolução: kWaveWidthMs / kWaveWidth caracteres.

static constexpr int   kWaveWidth   = 60;    // colunas do gráfico
static constexpr int   kWaveWinMs   = 300;   // janela de tempo (ms)
static constexpr float kUsPerCol    = (float)(kWaveWinMs * 1000) / kWaveWidth;

// Amostras fixas guardadas nos eventos do buffer (não disponíveis após drenagem).
// Para o waveform, reconstituímos a partir das métricas actuais:
// - conhecemos pw_us e period_us do último ciclo
// - desenhamos N períodos dentro da janela

static void print_waveform() {
    const int64_t now = esp_timer_get_time();
    const int64_t win_start = now - (int64_t)(kWaveWinMs * 1000);

    Serial.println();
    Serial.printf("  Janela: %d ms  (%.1f µs/coluna)\n",
                  kWaveWinMs, kUsPerCol);

    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) { continue; }
        const ChanMetrics& m = g_m[ch];
        if (m.period_us == 0 || m.pw_us == 0) {
            Serial.printf("  CH%d %-5s: sem dados\n", ch, kChan[ch].name);
            continue;
        }

        char bar[kWaveWidth + 1];
        // Reconstruir sinal: última descida conhecida = last_fall_us
        // Trabalhar para trás a partir de last_fall_us por períodos completos.
        for (int col = 0; col < kWaveWidth; ++col) {
            const int64_t t = win_start + (int64_t)(col * kUsPerCol);
            // Posição dentro do período (0 = início do HIGH)
            // Último rising = last_fall_us - pw_us (aproximado)
            const int64_t last_rise = m.last_fall_us - m.pw_us;
            int64_t phase = (t - last_rise) % (int64_t)m.period_us;
            if (phase < 0) { phase += m.period_us; }
            bar[col] = (phase < m.pw_us) ? '\xDB' : '_';  // █ ou _
        }
        bar[kWaveWidth] = '\0';

        Serial.printf("  CH%d %-5s [%s]\n", ch, kChan[ch].name, bar);
        Serial.printf("       %-5s  PW=%.3f ms  T=%.3f ms\n",
                      "", m.pw_us / 1000.0f, m.period_us / 1000.0f);
    }
}

// ── Estatísticas ──────────────────────────────────────────────────────────────

static void print_stats() {
    Serial.println();
    Serial.println("  CH  Name   STM32   Count     PW_last   PW_min    PW_max    PW_avg");
    Serial.println("  ──  ─────  ─────   ─────     ───────   ──────    ──────    ──────");
    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) { continue; }
        const ChanMetrics& m = g_m[ch];
        const float avg = (m.pulse_count > 0)
                          ? (float)(m.pw_sum_us / m.pulse_count) / 1000.0f
                          : 0.0f;
        Serial.printf("  %2d  %-5s  %-5s  %7lu   %7.3fms  %7.3fms  %7.3fms  %7.3fms\n",
                      ch, kChan[ch].name, kChan[ch].pin,
                      m.pulse_count,
                      m.pw_us    / 1000.0f,
                      m.pw_min_us / 1000.0f,
                      m.pw_max_us / 1000.0f,
                      avg);
    }
}

// ── Reset ──────────────────────────────────────────────────────────────────────

static void reset_stats() {
    for (int ch = 0; ch < kNChan; ++ch) {
        ChanMetrics& m = g_m[ch];
        m.pw_min_us = 0;
        m.pw_max_us = 0;
        m.pw_sum_us = 0;
        m.pulse_count = 0;
        m.overflow_count = 0;
    }
    Serial.println("  Estatísticas resetadas.");
}

// ── Help ──────────────────────────────────────────────────────────────────────

static void print_help() {
    Serial.println();
    Serial.println("  OpenEMS ESP32 Scope — Comandos:");
    Serial.println("  ────────────────────────────────");
    Serial.println("  l  Live table (1 s)       p  Pulse log");
    Serial.println("  e  Edge log               w  Waveform bar");
    Serial.println("  t  Timing analysis IGN    s  Estatísticas");
    Serial.println("  r  Reset stats            ?  Esta ajuda");
    Serial.println();
    Serial.println("  Timing (t): detecta gap CKP, captura 4 sparks,");
    Serial.println("    verifica ordem de disparo e inter-cil timing.");
    Serial.println("    Requer CH8 (GPIO36) ligado ao PA0 do STM32.");
    Serial.println();
    Serial.println("  Canais activos:");
    for (int ch = 0; ch < kNChan; ++ch) {
        if (kChan[ch].enabled) {
            Serial.printf("    CH%d  %-5s  STM32 %-5s → ESP32 GPIO%d\n",
                          ch, kChan[ch].name, kChan[ch].pin,
                          (int)kChan[ch].gpio);
        }
    }
}

// ── setup() ──────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(300);

    // Inicializar métricas
    memset(g_m, 0, sizeof(g_m));

    // Instalar serviço de ISR GPIO (partilhado por todos os canais)
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) { continue; }
        const gpio_num_t g = kChan[ch].gpio;

        gpio_reset_pin(g);
        gpio_set_direction(g, GPIO_MODE_INPUT);

        // Pull-down interno: quando o STM32 não está ligado, o pino fica LOW
        // e não gera falsas interrupções.
        gpio_set_pull_mode(g, GPIO_PULLDOWN_ONLY);

        gpio_set_intr_type(g, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(g, edge_isr, (void*)(uint32_t)ch);
    }

    // Habilitar interrupções GPIO
    for (int ch = 0; ch < kNChan; ++ch) {
        if (kChan[ch].enabled) {
            gpio_intr_enable(kChan[ch].gpio);
        }
    }

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   OpenEMS ESP32 Logic Scope              ║");
    Serial.println("║   Resolução: 1 µs  Latência: ~5 µs      ║");
    Serial.println("╚══════════════════════════════════════════╝");
    print_help();
    Serial.println("  Modo: LIVE (enviar '?' para ajuda)");
}

// ── loop() ───────────────────────────────────────────────────────────────────

static uint32_t g_last_live_ms = 0;

void loop() {
    // Processar eventos do buffer (sempre, independentemente do modo)
    process_events();

    // Comandos série
    if (Serial.available()) {
        const char c = (char)Serial.read();
        switch (c) {
            case 'l': g_mode = Mode::LIVE;  Serial.println("  Modo: LIVE");  break;
            case 'e': g_mode = Mode::EDGE;  Serial.println("  Modo: EDGE");  break;
            case 'p': g_mode = Mode::PULSE; Serial.println("  Modo: PULSE"); break;
            case 'w': g_mode = Mode::WAVE;  Serial.println("  Modo: WAVE");  break;
            case 't': g_mode = Mode::TIMING; timing_start(); break;
            case 's': print_stats(); break;
            case 'r': reset_stats(); break;
            case '?': print_help();  break;
            default: break;
        }
        g_last_live_ms = 0;   // forçar refresh imediato no modo LIVE
    }

    // Impressão periódica no modo LIVE
    if (g_mode == Mode::LIVE) {
        const uint32_t now = millis();
        if (now - g_last_live_ms >= 1000u) {
            g_last_live_ms = now;
            print_live_table();
        }
    }

    // No modo WAVE, redesenhar a cada 500 ms
    if (g_mode == Mode::WAVE) {
        static uint32_t last_wave_ms = 0;
        const uint32_t now = millis();
        if (now - last_wave_ms >= 500u) {
            last_wave_ms = now;
            print_waveform();
        }
    }

    // Pequena pausa para não monopolizar CPU em idle
    delay(5);
}
