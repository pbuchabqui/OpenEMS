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

enum class Mode : uint8_t { LIVE, EDGE, PULSE, WAVE };
static volatile Mode g_mode = Mode::LIVE;

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
    Serial.println("  s  Estatísticas           r  Reset stats");
    Serial.println("  ?  Esta ajuda");
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
