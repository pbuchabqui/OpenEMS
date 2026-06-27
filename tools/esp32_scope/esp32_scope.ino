/*
 * esp32_scope.ino — Osciloscópio lógico para bancada OpenEMS
 * ════════════════════════════════════════════════════════════
 * Plataforma : ESP32 / ESP32-C6 (Arduino core ≥ 2.0)
 * Resolução  : 1 µs  (esp_timer_get_time)
 * Latência   : 2–5 µs por bordo (ISR context)
 *
 * Sinais monitorizados:
 *   CH0 IGN0  ← PE9  TIM1_CH1  (bobina cil.1)
 *   CH1 IGN1  ← PE11 TIM1_CH2  (bobina cil.2)
 *   CH2 IGN2  ← PE13 TIM1_CH3  (bobina cil.3)
 *   CH3 IGN3  ← PE14 TIM1_CH4  (bobina cil.4)
 *   CH4 INJ0  ← PC6  TIM3_CH1  (injector cil.1)
 *   CH5 INJ1  ← PC7  TIM3_CH2  (injector cil.2)
 *   CH6 INJ2  ← PC8  TIM3_CH3  (injector cil.3)
 *   CH7 INJ3  ← PC9  TIM3_CH4  (injector cil.4)
 *   CH8 CKP   ← PA0  (loopback — ligar ao gerador CKP do stimulator)
 *   CH9 CMP   ← PA1  (loopback — ligar ao gerador CMP do stimulator)
 *
 * Ligações STM32H562 → ESP32-C6 (GPIO 0-7 + 10-11):
 *   PE9  → GPIO 0     PE11 → GPIO 1
 *   PE13 → GPIO 2     PE14 → GPIO 3
 *   PC6  → GPIO 4     PC7  → GPIO 5
 *   PC8  → GPIO 6     PC9  → GPIO 7
 *   PA0  → GPIO 10    PA1  → GPIO 11   GND → GND  ← OBRIGATÓRIO
 *
 *   (CKP/CMP: ligar GPIO2/4 do stimulator a GPIO10/11 + PA0/PA1 do STM32)
 *
 *   NOTA C6: GPIO16/17 são UART0 (bloqueados pelo Periman), GPIO24-30 são
 *   flash SPI externo, GPIO12/13 são USB D-/D+. Usar só GPIO 0-7 + 10-11.
 *
 *   NOTA ESP32 original: GPIO 0-11 existem (evitar 6-11 que são flash SPI).
 *   Se usar ESP32 original com este sketch, editar kChan[] para GPIO32-39
 *   que são input-only e mais convenientes.
 *
 * Comandos série (115200 baud):
 *   l  — Live table (actualiza a cada 1 s) [default]
 *   e  — Edge log (imprime cada bordo)
 *   p  — Pulse log (imprime cada pulso completo)
 *   w  — Waveform bar da última janela de 300 ms
 *   t  — Timing analysis 720°: 1×/ciclo, ordem 1-3-4-2, ângulo IGN+INJ
 *   r  — Reset estatísticas
 *   ?  — Ajuda
 *
 * ════════════════════════════════════════════════════════════
 */

#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <atomic>     // barreiras de memória p/ o SPSC ISR→loop (C6 é RISC-V, weak ordering)

// ── Definição de canais ───────────────────────────────────────────────────────

struct ChanDef {
    gpio_num_t  gpio;
    const char  name[6];     // nome curto (ex: "IGN0")
    const char  pin[6];      // pino STM32 (ex: "PC6")
    bool        enabled;
};

// Editar GPIOs conforme o seu DevKit.
// ESP32-C6: usar GPIO0-7 + GPIO10-11 (GPIO16/17=UART, 24-30=flash, 12/13=USB).
// ESP32 original: GPIO 0-11 também existem, mas 6-11 são flash SPI — preferir
// GPIO32-39 (input-only) editando kChan[] abaixo.
static ChanDef kChan[] = {
    { GPIO_NUM_0,  "IGN0", "PE9",  true },   // TIM1_CH1
    { GPIO_NUM_1,  "IGN1", "PE11", true },   // TIM1_CH2
    { GPIO_NUM_2,  "IGN2", "PE13", true },   // TIM1_CH3
    { GPIO_NUM_3,  "IGN3", "PE14", true },   // TIM1_CH4
    { GPIO_NUM_4,  "INJ0", "PC6",  true },   // TIM3_CH1
    { GPIO_NUM_5,  "INJ1", "PC7",  true },   // TIM3_CH2
    { GPIO_NUM_6,  "INJ2", "PC8",  true },   // TIM3_CH3
    { GPIO_NUM_7,  "INJ3", "PC9",  true },   // TIM3_CH4
    { GPIO_NUM_10, "CKP",  "PA0",  true },   // loopback CKP do stimulator
    { GPIO_NUM_11, "CMP",  "PA1",  true },   // loopback CMP do stimulator
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

static const char* mode_name(Mode m) {
    switch (m) {
        case Mode::LIVE:   return "LIVE";
        case Mode::EDGE:   return "EDGE";
        case Mode::PULSE:  return "PULSE";
        case Mode::WAVE:   return "WAVE";
        case Mode::TIMING: return "TIMING";
    }
    return "?";
}

// ── Máquina de estados para Timing Analysis 720° ─────────────────────────────
// Captura um ciclo completo de 720° (2 gaps CKP) e verifica:
//   1. Cada IGN/INJ dispara exatamente 1× por ciclo de 720°
//   2. Ordem de disparo 1-3-4-2 (IGN0→IGN2→IGN3→IGN1)
//   3. Inter-cilindro 180°±3°; ângulo absoluto desde gap1
//   4. CMP (CH9) presente 1×/720° com offset esperado ~30° (kCmpTooth=5 × 6°/dente)
// CH8 (CKP loopback PA0→GPIO10) e CH9 (CMP loopback GPIO4→GPIO11) obrigatórios.

static constexpr int kCkpChan  = 8;
static constexpr int kCmpChan  = 9;
static constexpr int kIgnFirst = 0;
static constexpr int kIgnCount = 4;
static constexpr int kInjFirst = 4;
static constexpr int kInjCount = 4;
static constexpr int kCylCount = 4;

// TDC de cada cilindro em dentes desde dente 0 do gap (60-2, firing order 1-3-4-2)
// Cil.1=IGN0: TDC @ dente 0; Cil.3=IGN2: TDC @ 15; Cil.4=IGN3: TDC @ 30; Cil.2=IGN1: TDC @ 45
// (baseado em firing order 1-3-4-2 com TDC equidistantes a 180°/15 dentes)
static constexpr float kTdcDente[kCylCount] = { 0.0f, 15.0f, 30.0f, 45.0f };
// kTdcDente[i] = TDC do cilindro cujo IGN é kExpectedFiringOrder[i]
static constexpr uint8_t kExpectedFiringOrder[kIgnCount] = {0, 2, 3, 1};

enum class TmState : uint8_t {
    IDLE,
    WAIT_GAP1,   // aguardar 1º gap
    CAPTURE,     // capturar IGN/INJ entre gap1 e gap2 (360°)
    WAIT_GAP2,   // aguardar 2º gap (confirma 720°)
    VERIFY,      // verificar que nenhum canal disparou 2× no mesmo ciclo
    DONE,
};

struct TmEvent {
    int64_t ts_us;
    uint8_t ch;
    uint8_t count;  // nº de vezes que este canal disparou no ciclo
};

struct TimingCapture {
    int64_t  gap1_ts_us;
    int64_t  gap2_ts_us;
    int64_t  last_ckp_fall_us;
    int64_t  ign_ts_us[kIgnCount];
    uint8_t  ign_count[kIgnCount];
    int64_t  inj_ts_us[kInjCount];
    uint8_t  inj_count[kInjCount];
    int64_t  cmp_ts_us;   // 1ª RISE do CMP no ciclo
    uint8_t  cmp_count;   // nº de bordas CMP no ciclo (esperado: 1)
    uint32_t ckp_period_us;
    uint32_t timeout_us;
    int      gap_count;
    bool     in_cycle;
};

static TmState       g_tm_state = TmState::IDLE;
static TimingCapture g_tm_cap;

static void timing_report();

static void timing_start() {
    g_tm_cap              = {};
    g_tm_cap.ckp_period_us = (g_m[kCkpChan].period_us > 0)
                              ? g_m[kCkpChan].period_us : 2000u;
    // timeout = 2 ciclos completos (2 × 60 dentes × 2 revs) + margem
    g_tm_cap.timeout_us   = g_tm_cap.ckp_period_us * 240u;

    if (g_m[kCkpChan].period_us == 0) {
        Serial.println("  AVISO: CH8 sem sinal. Ligar GPIO10 ao gerador CKP do stimulator.");
        Serial.println("  [TIMING] Aguardar sinal CKP antes de usar 't'.");
        g_tm_state = TmState::IDLE;
        return;
    }
    // Sinal com period < 200µs (>5kHz dente = >~6000 RPM equiv) é ruído — rejeitar.
    if (g_m[kCkpChan].period_us < 200u) {
        Serial.printf("  ERRO: CKP period=%.3f ms parece ruído (esperado ~0.5-5 ms).\n",
                      g_m[kCkpChan].period_us / 1000.0f);
        Serial.println("  Verificar fio: GPIO2 stimulator → GPIO10 + PA0 STM32.");
        g_tm_state = TmState::IDLE;
        return;
    }

    g_tm_state = TmState::WAIT_GAP1;
    Serial.printf("  [TIMING] CKP OK (period=%.3f ms). A aguardar 1º gap...\n",
                  g_m[kCkpChan].period_us / 1000.0f);
}

// ── ISR ───────────────────────────────────────────────────────────────────────

static void IRAM_ATTR edge_isr(void* arg) {
    const uint8_t ch  = (uint8_t)(uint32_t)arg;
    const int64_t t   = esp_timer_get_time();
    const int     lvl = gpio_get_level(kChan[ch].gpio);

    const uint16_t next = (g_head + 1u) & kBufMask;
    if (next != g_tail) {
        g_buf[g_head].ts_us = t;
        g_buf[g_head].ch    = ch;
        g_buf[g_head].level = (uint8_t)lvl;
        // C6 é RISC-V (weak ordering): fence release garante que os campos do
        // evento fiquem visíveis ANTES de g_head avançar (visto pelo consumidor).
        std::atomic_thread_fence(std::memory_order_release);
        g_head = next;
    } else {
        g_m[ch].overflow_count++;
    }
}

// ── Timing Analysis 720° ──────────────────────────────────────────────────────

static void timing_feed(const EdgeEvent& ev) {
    TimingCapture& c = g_tm_cap;

    // Timeout global
    if (c.gap1_ts_us > 0 &&
        (ev.ts_us - c.gap1_ts_us) > (int64_t)c.timeout_us) {
        Serial.println("  [TIMING] Timeout — verificar ligações e sync CKP.");
        g_tm_state = TmState::IDLE;
        g_mode     = Mode::LIVE;
        return;
    }

    // ── Deteção de gap CKP ──────────────────────────────────────────────────
    if (ev.ch == (uint8_t)kCkpChan) {
        if (ev.level == 0u) {
            c.last_ckp_fall_us = ev.ts_us;
        } else if (c.last_ckp_fall_us > 0) {
            const int64_t gap = ev.ts_us - c.last_ckp_fall_us;
            const bool is_gap = (gap > (int64_t)(c.ckp_period_us * 18u / 10u));
            if (is_gap) {
                c.gap_count++;
                if (c.gap_count == 1) {
                    c.gap1_ts_us       = ev.ts_us;
                    c.last_ckp_fall_us = 0;  // reset para não reutilizar fall do gap1
                    c.in_cycle         = true;
                    g_tm_state         = TmState::CAPTURE;
                    Serial.printf("  [TIMING] Gap1 OK (%.2f ms) — a capturar 720°...\n",
                                  gap / 1000.0f);
                } else if (c.gap_count == 2) {
                    c.gap2_ts_us = ev.ts_us;
                    c.in_cycle   = false;
                    g_tm_state   = TmState::DONE;
                    Serial.printf("  [TIMING] Gap2 OK — ciclo de %.2f ms capturado.\n",
                                  (ev.ts_us - c.gap1_ts_us) / 1000.0f);
                    timing_report();
                    g_mode = Mode::LIVE;
                }
            }
        }
        return;
    }

    // ── Captura IGN/INJ/CMP entre gap1 e gap2 ───────────────────────────────
    if (!c.in_cycle) { return; }

    // CMP: capturar borda de RISE (pulso activo-alto do stimulator)
    if (ev.ch == (uint8_t)kCmpChan && ev.level == 1u) {
        c.cmp_count++;
        if (c.cmp_count == 1) {
            c.cmp_ts_us = ev.ts_us;
            Serial.printf("  [TIMING] CMP  @ +%.3f ms (%.1f°)\n",
                          (ev.ts_us - c.gap1_ts_us) / 1000.0f,
                          (float)(ev.ts_us - c.gap1_ts_us) / c.ckp_period_us * 6.0f);
        } else {
            Serial.printf("  [TIMING] !! CMP disparou %dx (esperado 1×/720°)\n", c.cmp_count);
        }
        return;
    }

    if (ev.level != 0u) { return; }  // IGN/INJ: só FALL

    if (ev.ch >= (uint8_t)kIgnFirst && ev.ch < (uint8_t)(kIgnFirst + kIgnCount)) {
        const int idx = (int)(ev.ch - kIgnFirst);
        c.ign_count[idx]++;
        if (c.ign_count[idx] == 1) {
            c.ign_ts_us[idx] = ev.ts_us;
            Serial.printf("  [TIMING] IGN%d @ +%.3f ms (%.1f°)\n", idx,
                          (ev.ts_us - c.gap1_ts_us) / 1000.0f,
                          (float)(ev.ts_us - c.gap1_ts_us) / c.ckp_period_us * 6.0f);
        } else if (c.ign_count[idx] == 2) {
            Serial.printf("  [TIMING] !! IGN%d disparou 2x (wasted-spark ou fase errada)\n", idx);
        }
    } else if (ev.ch >= (uint8_t)kInjFirst && ev.ch < (uint8_t)(kInjFirst + kInjCount)) {
        const int idx = (int)(ev.ch - kInjFirst);
        c.inj_count[idx]++;
        if (c.inj_count[idx] == 1) {
            c.inj_ts_us[idx] = ev.ts_us;
        }
    }
}

static void timing_report() {
    const TimingCapture& c = g_tm_cap;
    const float T720 = (c.gap2_ts_us - c.gap1_ts_us) / 1000.0f;  // ms/ciclo medido
    const float T  = (T720 > 0.0f) ? T720 / 120.0f : c.ckp_period_us / 1000.0f;  // ms/dente
    // Inter-cilindro esperado: 720° / 4 cilindros = 180° = 30 dentes
    const float inter_exp_ms  = T * 30.0f;
    const float inter_exp_deg = 180.0f;

    Serial.println();
    Serial.println("  ╔═══════════════════════════════════════════════════════════╗");
    Serial.println("  ║  Sequential Ignition Analysis — 720° Cycle              ║");
    Serial.printf( "  ║  T_dente=%.3f ms   T_ciclo=%.2f ms (esp. %.2f ms) %s║\n",
                   T, T720, T * 120.0f, "");
    Serial.println("  ╚═══════════════════════════════════════════════════════════╝");

    // ── 1. Verificação 1×/720° ──────────────────────────────────────────────
    Serial.println();
    Serial.println("  [1] Contagem de disparos por ciclo de 720° (esperado: 1×)");
    Serial.println("  Canal  Contagem  OK?");
    bool count_ok = true;
    for (int i = 0; i < kIgnCount; ++i) {
        const bool ok = (c.ign_count[i] == 1);
        if (!ok) count_ok = false;
        Serial.printf("  IGN%d   %dx        %s\n", i, c.ign_count[i], ok ? "✓" : "✗ FALHA");
    }
    for (int i = 0; i < kInjCount; ++i) {
        const bool ok = (c.inj_count[i] == 1);
        if (!ok) count_ok = false;
        Serial.printf("  INJ%d   %dx        %s\n", i, c.inj_count[i], ok ? "✓" : "✗ FALHA");
    }

    // ── 2. Ordem de disparo ─────────────────────────────────────────────────
    // Ordenar canais IGN por timestamp (só os que dispararam 1×)
    uint8_t order[kIgnCount] = {0, 1, 2, 3};
    for (int i = 0; i < kIgnCount - 1; ++i)
        for (int j = 0; j < kIgnCount - 1 - i; ++j)
            if (c.ign_ts_us[order[j]] > c.ign_ts_us[order[j+1]]) {
                uint8_t tmp = order[j]; order[j] = order[j+1]; order[j+1] = tmp;
            }

    bool order_ok = true;
    for (int i = 0; i < kIgnCount; ++i)
        if (order[i] != kExpectedFiringOrder[i]) { order_ok = false; }

    Serial.println();
    Serial.println("  [2] Ordem de disparo IGN");
    Serial.print("  Detectada : ");
    for (int i = 0; i < kIgnCount; ++i) { if (i) Serial.print("→"); Serial.printf("IGN%d", order[i]); }
    Serial.println();
    Serial.print("  Esperada  : IGN0→IGN2→IGN3→IGN1");
    Serial.printf("  %s\n", order_ok ? "  ✓" : "  ✗ ERRADA");

    // ── 3. Ângulo por cilindro ──────────────────────────────────────────────
    Serial.println();
    Serial.println("  [3] Ângulo de avanço por cilindro (FALL IGN desde gap1)");
    Serial.println("  Canal  ms desde gap1  Dentes   Graus    Inter-cil  Dev inter  OK?");
    Serial.println("  ─────  ────────────  ──────  ──────   ─────────  ─────────  ───");
    float prev_deg = 0.0f;
    bool  angle_ok = true;
    for (int i = 0; i < kIgnCount; ++i) {
        const int   ch  = order[i];
        const bool  cap = (c.ign_count[ch] == 1);
        const float dt  = cap ? (c.ign_ts_us[ch] - c.gap1_ts_us) / 1000.0f : -1.0f;
        const float deg = cap ? dt / T * 6.0f : -1.0f;
        const float ic_deg = (i == 0) ? 0.0f : deg - prev_deg;
        const float ic_dev = (i == 0) ? 0.0f : ic_deg - inter_exp_deg;
        const bool  ic_ok  = (i == 0) || (fabsf(ic_dev) < 3.0f);  // ±3° = ~±0.5 dentes
        if (!ic_ok) { angle_ok = false; }
        if (!cap) {
            Serial.printf("  IGN%d  (não capturado)                                    ✗\n", ch);
            angle_ok = false;
        } else if (i == 0) {
            Serial.printf("  IGN%d  %8.3f ms  %6.2f  %6.1f°     —          —        —\n",
                          ch, dt, dt / T, deg);
        } else {
            Serial.printf("  IGN%d  %8.3f ms  %6.2f  %6.1f°  %6.1f°  %+6.1f°    %s\n",
                          ch, dt, dt / T, deg, ic_deg, ic_dev, ic_ok ? "✓" : "✗");
        }
        if (cap) { prev_deg = deg; }
    }

    // ── 4. INJ ──────────────────────────────────────────────────────────────
    Serial.println();
    Serial.println("  [4] Injetores (FALL INJ desde gap1)");
    Serial.println("  Canal  ms desde gap1  Graus");
    for (int i = 0; i < kInjCount; ++i) {
        if (c.inj_count[i] == 1) {
            const float dt  = (c.inj_ts_us[i] - c.gap1_ts_us) / 1000.0f;
            const float deg = dt / T * 6.0f;
            Serial.printf("  INJ%d   %8.3f ms  %6.1f°\n", i, dt, deg);
        } else {
            Serial.printf("  INJ%d   (não capturado ou %dx)\n", i, c.inj_count[i]);
        }
    }

    // ── 5. CMP ──────────────────────────────────────────────────────────────
    // kCmpTooth=5 → CMP sobe ~5 dentes após dente 0 do gap = ~30°
    static constexpr float kCmpExpectedDeg = 30.0f;
    static constexpr float kCmpToleranceDeg = 18.0f;  // ±3 dentes
    Serial.println();
    Serial.println("  [5] CMP (cam sensor) — esperado 1×/720°");
    bool cmp_ok = false;
    if (c.cmp_count == 0) {
        Serial.println("  CMP   0×  ✗ AUSENTE — verificar ligação GPIO11←GPIO4 e CMP_GPIO do stimulator");
    } else if (c.cmp_count > 1) {
        Serial.printf("  CMP   %dx ✗ DUPLICADO\n", c.cmp_count);
    } else {
        const float dt  = (c.cmp_ts_us - c.gap1_ts_us) / 1000.0f;
        const float deg = dt / T * 6.0f;
        const float dev = deg - kCmpExpectedDeg;
        cmp_ok = (fabsf(dev) < kCmpToleranceDeg);
        Serial.printf("  CMP   1×  @ +%.3f ms = %.1f°  (esp. ~%.0f°, dev %+.1f°)  %s\n",
                      dt, deg, kCmpExpectedDeg, dev, cmp_ok ? "✓" : "✗ OFFSET ERRADO");
    }

    // ── Resultado ───────────────────────────────────────────────────────────
    const bool all_ok = count_ok && order_ok && angle_ok && cmp_ok;
    Serial.println();
    Serial.printf("  Resultado: %s\n", all_ok ? "✓ SEQUENCIAL OK (720°)" : "✗ VER FALHAS ACIMA");
    Serial.println("  (para confirmar ângulo: snapshot CDC 'A' byte 8 = advance_p40 × 0.25°)");
    Serial.println();
}

// ── Processamento de eventos (loop principal) ─────────────────────────────────

static void process_events() {
    while (g_tail != g_head) {
        // C6 é RISC-V (weak ordering): fence acquire garante que os campos do
        // evento já estão visíveis depois de g_head ter avançado na ISR
        // (emparelha com o fence release em edge_isr).
        std::atomic_thread_fence(std::memory_order_acquire);
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
    Serial.println("  Timing (t): captura 1 ciclo de 720° (2 gaps CKP).");
    Serial.println("    Verifica: 1) IGN/INJ/CMP dispara 1x/720°; 2) ordem 1-3-4-2;");
    Serial.println("    3) inter-cilindro 180°±3°; 4) CMP ~30° desde gap.");
    Serial.println("    CH8=GPIO10←PA0 (CKP); CH9=GPIO11←GPIO4 (CMP loopback).");
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
#if ARDUINO_USB_CDC_ON_BOOT
    // C6/C-series com CDC On Boot: Serial = HWCDC (USB nativo). Sem isto o
    // HWCDC BLOQUEIA/DESCARTA os prints quando o host não está a drenar →
    // saída fica muda. Timeout 0 = escrita não-bloqueante (essencial no C6).
    Serial.setTxTimeoutMs(0);
#endif
    delay(300);

    // Inicializar métricas
    memset(g_m, 0, sizeof(g_m));

    // Configurar GPIOs via Arduino API (portável ESP32/C6)
    // NOTA: O core do ESP32 já instala o ISR service internamente no primeiro
    // attachInterruptArg() — NÃO chamar gpio_install_isr_service() novamente.
    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled) { continue; }
        const gpio_num_t g = kChan[ch].gpio;

        pinMode((uint8_t)g, INPUT_PULLDOWN);

        attachInterruptArg(digitalPinToInterrupt((uint8_t)g), edge_isr,
                           (void*)(uint32_t)ch, CHANGE);
    }

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   OpenEMS ESP32 Logic Scope              ║");
    Serial.println("║   Resolução: 1 µs  Latência: ~5 µs      ║");
    Serial.println("╚══════════════════════════════════════════╝");
    print_help();
    Serial.println("  Modo: LIVE (enviar '?' para ajuda)");

    // LED onboard como heart-beat (GPIO2 no ESP32, RGB no C6)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
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

    // Heartbeat LED: pulso curto a cada 2 s (indica que o scope está vivo)
    // ESP32-C6: RGB_BUILTIN (GPIO8, WS2812 via RMT); ESP32: LED_BUILTIN (GPIO13/2)
    {
        static uint32_t last_hb = 0;
        static bool     hb_on  = false;
        const uint32_t now = millis();
        if (!hb_on && now - last_hb >= 2000u) {
            digitalWrite(LED_BUILTIN, HIGH);
            hb_on    = true;
            last_hb = now;
            // Heartbeat no Serial: confirma que o loop() corre e que a UART0 está
            // a sair, independentemente do banner de boot (que pode perder-se).
            Serial.printf("[HB] scope vivo modo=%s uptime=%lus\n",
                          mode_name(g_mode), (unsigned long)(now / 1000u));
        } else if (hb_on && now - last_hb >= 80u) {
            digitalWrite(LED_BUILTIN, LOW);
            hb_on = false;
        }
    }

    // Pequena pausa para não monopolizar CPU em idle
    delay(5);
}
