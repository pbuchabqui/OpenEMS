/*
 * ⚠️ DEPRECATED — use esp32_combined.ino instead.
 * This standalone scope is kept for reference. The combined tool integrates
 * CKP/CMP generation + logic scope in a single ESP32 (no external loopback).
 * Dashboard 720° mode has been ported to the combined tool.
 *
 * esp32_scope.ino — Osciloscópio lógico para bancada OpenEMS
 * ════════════════════════════════════════════════════════════
 * Plataforma : ESP32 / ESP32-C6 (Arduino core ≥ 2.0)
 * Resolução  : 1 µs  (esp_timer_get_time)
 * Latência   : 2–5 µs por bordo (ISR context)
 *
 * Sinais monitorizados:
 *   CH0 IGN1  ← PE9  TIM1_CH1  (bobina cil.1)
 *   CH1 IGN2  ← PE11 TIM1_CH2  (bobina cil.2)
 *   CH2 IGN3  ← PE13 TIM1_CH3  (bobina cil.3)
 *   CH3 IGN4  ← PE14 TIM1_CH4  (bobina cil.4)
 *   CH4 INJ1  ← PC6  TIM3_CH1  (injector cil.1)
 *   CH5 INJ2  ← PC7  TIM3_CH2  (injector cil.2)
 *   CH6 INJ3  ← PC8  TIM3_CH3  (injector cil.3)
 *   CH7 INJ4  ← PC9  TIM3_CH4  (injector cil.4)
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
    { GPIO_NUM_0,  "IGN4", "PE14", true },   // TIM1_CH4 — cil.4 (GPIO0→PE14 na bancada)
    { GPIO_NUM_1,  "IGN3", "PE13", true },   // TIM1_CH3 — cil.3 (GPIO1→PE13)
    { GPIO_NUM_2,  "IGN2", "PE11", true },   // TIM1_CH2 — cil.2 (GPIO2→PE11)
    { GPIO_NUM_3,  "IGN1", "PE9",  true },   // TIM1_CH1 — cil.1 (GPIO3→PE9)
    { GPIO_NUM_4,  "INJ4", "PC9",  false },  // TIM3_CH4 — cil.4
    { GPIO_NUM_5,  "INJ3", "PC8",  false },  // TIM3_CH3 — cil.3
    { GPIO_NUM_6,  "INJ2", "PC7",  false },  // TIM3_CH2 — cil.2 (GPIO6→PC7)
    { GPIO_NUM_7,  "INJ1", "PC6",  false },  // TIM3_CH1 — cil.1 (GPIO7→PC6)
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

static constexpr int kBufSize = 8192;  // 4× original — reduz overflow com 10 canais a 700+ RPM
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

enum class Mode : uint8_t { LIVE, EDGE, PULSE, WAVE, TIMING, DASH };
static volatile Mode g_mode = Mode::LIVE;

static const char* mode_name(Mode m) {
    switch (m) {
        case Mode::LIVE:   return "LIVE";
        case Mode::EDGE:   return "EDGE";
        case Mode::PULSE:  return "PULSE";
        case Mode::WAVE:   return "WAVE";
        case Mode::TIMING: return "TIMING";
        case Mode::DASH:   return "DASH";
    }
    return "?";
}

// ── Máquina de estados para Timing Analysis 720° ─────────────────────────────
// Captura um ciclo completo de 720° (3 gaps CKP: gap1→gap3 = 2 revoluções) e verifica:
//   1. Cada IGN/INJ dispara exatamente 1× por ciclo de 720°
//   2. Ordem de disparo 1-3-4-2 (IGN1→IGN3→IGN4→IGN2)
//   3. Inter-cilindro 180°±3°; ângulo absoluto desde gap1
//   4. CMP (CH9) presente 2×/720° (janela A dente 5 + janela B dente 34)
// CH8 (CKP loopback PA0→GPIO10) e CH9 (CMP loopback GPIO4→GPIO11) obrigatórios.

static constexpr int kCkpChan  = 8;
static constexpr int kCmpChan  = 9;
static constexpr int kIgnFirst = 0;
static constexpr int kIgnCount = 4;
static constexpr int kInjFirst = 4;
static constexpr int kInjCount = 4;
static constexpr int kCylCount = 4;

// TDC de cada cilindro em dentes desde dente 0 do gap1 (60-2, firing order 1-3-4-2)
// Ciclo completo = 720° = 120 dentes virtuais (2 revoluções × 60 dentes).
// Cil.1=IGN1: TDC @ dente 0 (0°); Cil.3=IGN3: TDC @ dente 30 (180°);
// Cil.4=IGN4: TDC @ dente 60 (360°); Cil.2=IGN2: TDC @ dente 90 (540°)
static constexpr float kTdcDente[kCylCount] = { 0.0f, 30.0f, 60.0f, 90.0f };
// kTdcDente[i] = TDC do cilindro cujo IGN é kExpectedFiringOrder[i]
// CH0=IGN4(PE14), CH1=IGN3(PE13), CH2=IGN2(PE11), CH3=IGN1(PE9)
// Firing order 1-3-4-2: IGN1→IGN3→IGN4→IGN2 → CH3→CH1→CH0→CH2
static constexpr uint8_t kExpectedFiringOrder[kIgnCount] = {3, 1, 0, 2};

enum class TmState : uint8_t {
    IDLE,
    WAIT_GAP1,   // aguardar 1º gap
    CAPTURE,     // capturar IGN/INJ entre gap1 e gap3 (720° = 2 revoluções)
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
    int64_t  gap2_ts_us;   // 1ª volta (360°) — referência de fase
    int64_t  gap3_ts_us;   // 2ª volta (720°) — fim do ciclo completo
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

// ── Dashboard 720° oscilloscope ──────────────────────────────────────────────
// Captura TODAS as bordas num ciclo 720° e renderiza diagrama ASCII.
static constexpr int kDashMaxEdges = 600;   // ~284 esperados — folga 2×
static constexpr int kDashWidth    = 120;   // colunas (6°/col em 720° = 1 dente/col)
static constexpr float kDashDegPerCol = 720.0f / (float)kDashWidth;

struct DashEvent {
    int64_t ts_us;
    uint8_t ch;
    uint8_t level;  // 0=FALL, 1=RISE
};
static DashEvent g_dash_events[kDashMaxEdges];
static int       g_dash_event_count = 0;
static int64_t   g_dash_gap1_us = 0;
static int64_t   g_dash_gap2_us = 0;  // timestamp do 2º gap (referência de metade do ciclo)
static int64_t   g_dash_gap3_us = 0;
static int       g_dash_gap_count = 0;
static bool      g_dash_capturing = false;
static int64_t   g_dash_last_ckp_fall_us = 0;  // reset in dashboard_start() — evita gap falso na 2ª invocação
static int        g_dash_post_gap3_ckp_edges = 0;  // contador de dentes CKP após gap3 (captura de sparks tardios)

static void dashboard_start();
static void dashboard_feed(const EdgeEvent& ev);
static void render_dashboard();

static TmState       g_tm_state = TmState::IDLE;
static TimingCapture g_tm_cap;

static void timing_report();

static void timing_start() {
    g_tm_cap              = {};
    // Flush ring buffer — descarta eventos antigos que encheriam o buffer
    // e causariam perda de gaps durante a captura de 720°.
    g_tail = g_head;
    g_tm_cap.ckp_period_us = (g_m[kCkpChan].period_us > 0)
                              ? g_m[kCkpChan].period_us : 2000u;
    // timeout = 2 ciclos completos de 720° (2 × 120 dentes virtuais) + margem
    g_tm_cap.timeout_us   = g_tm_cap.ckp_period_us * 480u;

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

    // CKP: em modos não-TIMING/DASH, gravar só bordas RISE (½ eventos, 700/s vs 1400/s)
    // Nos modos TIMING e DASH precisamos de ambas as bordas para deteção de gap.
    if (ch == (uint8_t)kCkpChan && lvl == 0 && g_mode != Mode::TIMING && g_mode != Mode::DASH) {
        return;
    }

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
                    Serial.printf("  [TIMING] Gap2 OK (1ª volta, +%.0f ms) — a continuar...\n",
                                  (ev.ts_us - c.gap1_ts_us) / 1000.0f);
                } else if (c.gap_count == 3) {
                    c.gap3_ts_us = ev.ts_us;
                    c.in_cycle   = false;
                    g_tm_state   = TmState::DONE;
                    Serial.printf("  [TIMING] Gap3 OK — ciclo de 720° capturado (%.2f ms).\n",
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
            Serial.printf("  [TIMING] CMP  janela A @ +%.3f ms (%.1f°)\n",
                          (ev.ts_us - c.gap1_ts_us) / 1000.0f,
                          (float)(ev.ts_us - c.gap1_ts_us) / c.ckp_period_us * 6.0f);
        } else if (c.cmp_count == 2) {
            Serial.printf("  [TIMING] CMP  janela B @ +%.3f ms (%.1f°)\n",
                          (ev.ts_us - c.gap1_ts_us) / 1000.0f,
                          (float)(ev.ts_us - c.gap1_ts_us) / c.ckp_period_us * 6.0f);
        } else {
            Serial.printf("  [TIMING] !! CMP disparou %dx (esperado 2×/720°)\n", c.cmp_count);
        }
        return;
    }

    if (ev.level != 0u) { return; }  // IGN/INJ: só FALL

    if (ev.ch >= (uint8_t)kIgnFirst && ev.ch < (uint8_t)(kIgnFirst + kIgnCount)) {
        const int idx = (int)(ev.ch - kIgnFirst);
        c.ign_count[idx]++;
        if (c.ign_count[idx] == 1) {
            c.ign_ts_us[idx] = ev.ts_us;
            Serial.printf("  [TIMING] IGN%d @ +%.3f ms (%.1f°)\n", idx + 1,
                          (ev.ts_us - c.gap1_ts_us) / 1000.0f,
                          (float)(ev.ts_us - c.gap1_ts_us) / c.ckp_period_us * 6.0f);
        } else if (c.ign_count[idx] == 2) {
            Serial.printf("  [TIMING] !! IGN%d disparou 2x (wasted-spark ou fase errada)\n", idx + 1);
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
    const float T720 = (c.gap3_ts_us - c.gap1_ts_us) / 1000.0f;  // ms/ciclo 720° medido
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
        Serial.printf("  IGN%d   %dx        %s\n", i + 1, c.ign_count[i], ok ? "✓" : "✗ FALHA");
    }
    for (int i = 0; i < kInjCount; ++i) {
        const bool ok = (c.inj_count[i] == 1);
        if (!ok) count_ok = false;
        Serial.printf("  INJ%d   %dx        %s\n", i + 1, c.inj_count[i], ok ? "✓" : "✗ FALHA");
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
    for (int i = 0; i < kIgnCount; ++i) { if (i) Serial.print("→"); Serial.printf("IGN%d", order[i] + 1); }
    Serial.println();
    Serial.print("  Esperada  : IGN3→IGN4→IGN2→IGN1  (temporal CH1→CH0→CH2→CH3)");
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
            Serial.printf("  IGN%d  (não capturado)                                    ✗\n", ch + 1);
            angle_ok = false;
        } else if (i == 0) {
            Serial.printf("  IGN%d  %8.3f ms  %6.2f  %6.1f°     —          —        —\n",
                          ch + 1, dt, dt / T, deg);
        } else {
            Serial.printf("  IGN%d  %8.3f ms  %6.2f  %6.1f°  %6.1f°  %+6.1f°    %s\n",
                          ch + 1, dt, dt / T, deg, ic_deg, ic_dev, ic_ok ? "✓" : "✗");
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
            Serial.printf("  INJ%d   %8.3f ms  %6.1f°\n", i + 1, dt, deg);
        } else {
            Serial.printf("  INJ%d   (não capturado ou %dx)\n", i + 1, c.inj_count[i]);
        }
    }

    // ── 5. CMP ──────────────────────────────────────────────────────────────
    // Encoding A/B: 1ª borda CMP no dente 5 (janela A, ~30°), 2ª no dente 34 (janela B, ~564°)
    static constexpr float kCmpExpectedDeg = 30.0f;
    static constexpr float kCmpToleranceDeg = 18.0f;  // ±3 dentes
    Serial.println();
    Serial.println("  [5] CMP (cam sensor) — esperado 2×/720° (janela A + B)");
    bool cmp_ok = false;
    if (c.cmp_count == 0) {
        Serial.println("  CMP   0×  ✗ AUSENTE — verificar ligação GPIO11←GPIO4 e CMP_GPIO do stimulator");
    } else if (c.cmp_count > 2) {
        Serial.printf("  CMP   %dx ✗ EXCESSIVO (esperado 2)\n", c.cmp_count);
    } else {
        const float dt  = (c.cmp_ts_us - c.gap1_ts_us) / 1000.0f;
        const float deg = dt / T * 6.0f;
        const float dev = deg - kCmpExpectedDeg;
        cmp_ok = (fabsf(dev) < kCmpToleranceDeg);
        Serial.printf("  CMP   %dx  @ +%.3f ms = %.1f°  (esp. ~%.0f°, dev %+.1f°)  %s\n",
                      c.cmp_count, dt, deg, kCmpExpectedDeg, dev, cmp_ok ? "✓" : "✗ OFFSET ERRADO");
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
    int batch = 0;
    while (g_tail != g_head && batch < 512) {
        ++batch;
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

        // Alimentar dashboard (activo apenas durante modo DASH)
        if (g_mode == Mode::DASH) {
            dashboard_feed(ev);
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
        Serial.printf("  RPM estimado (IGN1 period): %.1f\n", ign0_rpm);
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

// ── Dashboard 720° Oscilloscope ────────────────────────────────────────────────

static void dashboard_start() {
    g_dash_event_count = 0;
    g_dash_gap_count   = 0;
    g_dash_gap1_us     = 0;
    g_dash_gap2_us     = 0;
    g_dash_gap3_us     = 0;
    g_dash_capturing        = false;
    g_dash_last_ckp_fall_us = 0;
    g_dash_post_gap3_ckp_edges = 0;
    // Auto- Ligar canais INJ (necessários p/ dashboard)
    for (int ch = (int)kInjFirst; ch < (int)(kInjFirst + kInjCount); ++ch) {
        if (!kChan[ch].enabled) {
            kChan[ch].enabled = true;
            attachInterruptArg(kChan[ch].gpio, edge_isr, (void*)(uint32_t)ch, CHANGE);
        }
    }
    // Flush ring buffer
    g_tail = g_head;
    uint32_t ckp_period = (g_m[kCkpChan].period_us > 0) ? g_m[kCkpChan].period_us : 2000u;
    Serial.printf("  [DASH] À espera do 1º gap CKP (period=%.3f ms)...\n", ckp_period / 1000.0f);
}

// Chamado por process_events() para cada evento quando g_mode == Mode::DASH
static void dashboard_feed(const EdgeEvent& ev) {
    // ── Deteção de gap CKP ──────────────────────────────────────────────
    if (ev.ch == (uint8_t)kCkpChan) {
        if (ev.level == 0u) {
            g_dash_last_ckp_fall_us = ev.ts_us;
        } else if (g_dash_last_ckp_fall_us > 0) {
            const uint32_t ckp_period = (g_m[kCkpChan].period_us > 0) ? g_m[kCkpChan].period_us : 2000u;
            const int64_t gap = ev.ts_us - g_dash_last_ckp_fall_us;
            if (gap > (int64_t)(ckp_period * 18u / 10u)) {
                g_dash_gap_count++;
                if (g_dash_gap_count == 1) {
                    g_dash_gap1_us   = ev.ts_us;
                    g_dash_capturing = true;
                    g_dash_event_count = 0;
                    Serial.printf("  [DASH] Gap1 OK — a capturar ciclo 720°...\n");
                } else if (g_dash_gap_count == 2) {
                    g_dash_gap2_us = ev.ts_us;
                    Serial.printf("  [DASH] Gap2 OK (1ª volta, +%.0f ms) — a continuar...\n",
                                  (ev.ts_us - g_dash_gap1_us) / 1000.0f);
                } else if (g_dash_gap_count == 3) {
                    g_dash_gap3_us   = ev.ts_us;
                    g_dash_capturing = false;
                    g_dash_post_gap3_ckp_edges = 0;  // iniciar buffer pós-gap3
                    Serial.printf("  [DASH] Gap3 OK — ciclo 720° capturado (%.2f ms). A capturar sparks tardios...\n",
                                  (ev.ts_us - g_dash_gap1_us) / 1000.0f);
                    // NÃO chamar render_dashboard() aqui — esperar pelos eventos pós-gap3
                }
            }
        }
        // Sempre gravar CKP no buffer durante captura ou pós-gap3
        const bool recording = g_dash_capturing || (g_dash_gap_count >= 3 && g_dash_post_gap3_ckp_edges < 8);
        if (recording && g_dash_event_count < kDashMaxEdges) {
            DashEvent de;
            de.ts_us = ev.ts_us; de.ch = ev.ch; de.level = ev.level;
            g_dash_events[g_dash_event_count++] = de;
        }
        // Após gap3, contar dentes CKP para decidir quando renderizar
        if (!g_dash_capturing && g_dash_gap_count >= 3 && ev.level == 1u) {
            g_dash_post_gap3_ckp_edges++;
            if (g_dash_post_gap3_ckp_edges >= 8) {
                render_dashboard();
                g_mode = Mode::LIVE;
            }
        }
        return;
    }

    // ── Gravar todos os canais durante captura ou pós-gap3 ──────────────
    const bool recording = g_dash_capturing || (g_dash_gap_count >= 3 && g_dash_post_gap3_ckp_edges < 8);
    if (recording && g_dash_event_count < kDashMaxEdges) {
        DashEvent de;
        de.ts_us = ev.ts_us; de.ch = ev.ch; de.level = ev.level;
        g_dash_events[g_dash_event_count++] = de;
    }
}

static void render_dashboard() {
    if (g_dash_event_count == 0 || g_dash_gap3_us <= g_dash_gap1_us) {
        Serial.println("  [DASH] ERRO: sem dados ou ciclo inválido.");
        return;
    }

    const float T720_ms = (g_dash_gap3_us - g_dash_gap1_us) / 1000.0f;
    const float T_tooth_ms = T720_ms / 120.0f;  // 120 dentes virtuais em 720°
    const float rpm_f = (T720_ms > 0) ? 120000.0f / T720_ms : 0.0f;
    const float us_per_col = T720_ms * 1000.0f / (float)kDashWidth;

    // ── Cabeçalho ──────────────────────────────────────────────────────
    Serial.println();
    Serial.println("  ╔════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗");
    Serial.printf("  ║ 720° Engine Dashboard — RPM=%.0f  T_ciclo=%.2f ms  events=%d                     ║\n",
                  rpm_f, T720_ms, g_dash_event_count);
    Serial.println("  ╠════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣");

    // ── Régua de ângulo (marcas a cada 90°, labels a cada 180°) ──────────
    // Pré-construir a linha da régua como string
    char ruler[kDashWidth + 1];
    for (int col = 0; col < kDashWidth; ++col) {
        float deg = col * kDashDegPerCol;
        int deg_i = (int)(deg + 0.5f);
        if (deg_i % 180 == 0 && fabsf(deg - (float)deg_i) < kDashDegPerCol) {
            ruler[col] = '0' + (deg_i / 180);  // 0,1,2,3,4 para 0,180,360,540,720
        } else if (deg_i % 90 == 0 && fabsf(deg - (float)deg_i) < kDashDegPerCol / 2.0f) {
            ruler[col] = '|';
        } else if (deg_i % 30 == 0 && fabsf(deg - (float)deg_i) < kDashDegPerCol / 2.0f) {
            ruler[col] = '\'';
        } else {
            ruler[col] = ' ';
        }
    }
    ruler[kDashWidth] = '\0';
    Serial.print("  ║"); Serial.print(ruler);
    Serial.printf(" ║ 0=0° 1=180° 2=360° 3=540° 4=720°\n");

    // ── Marcadores TDC ──────────────────────────────────────────────────
    // T1=cyl1@0°/720°, T3=cyl3@180°, T4=cyl4@360°, T2=cyl2@540°
    char tdc[kDashWidth + 1];
    for (int col = 0; col < kDashWidth; ++col) {
        float deg = col * kDashDegPerCol;
        float near_0 = fabsf(fmodf(deg + kDashDegPerCol, 720.0f));
        if (deg < kDashDegPerCol * 2.0f || fabsf(deg - 360.0f) < kDashDegPerCol * 1.5f) {
            tdc[col] = '1';  // TDC cyl 1
        } else if (fabsf(deg - 180.0f) < kDashDegPerCol * 1.5f) {
            tdc[col] = '3';  // TDC cyl 3
        } else if (fabsf(deg - 540.0f) < kDashDegPerCol * 1.5f) {
            tdc[col] = '2';  // TDC cyl 2
        } else {
            tdc[col] = ' ';
        }
    }
    // Sobrescrever 360° com '4' (TDC cyl 4)
    for (int col = 0; col < kDashWidth; ++col) {
        float deg = col * kDashDegPerCol;
        if (fabsf(deg - 360.0f) < kDashDegPerCol * 1.5f) { tdc[col] = '4'; }
    }
    tdc[kDashWidth] = '\0';
    Serial.print("  ║TDC"); Serial.print(tdc);
    Serial.println("║ 1=cyl1 3=cyl3 4=cyl4 2=cyl2");

    // ── Phase bar ───────────────────────────────────────────────────────
    Serial.print("  ║PHASE");
    for (int col = 0; col < kDashWidth - 5; ++col) {
        float deg = col * kDashDegPerCol;
        Serial.print(deg < 360.0f ? 'A' : 'B');
    }
    Serial.println("║");

    // ── CKP 60-2 wheel (reconstruído analiticamente) ────────────────────
    Serial.print("  ║CKP");
    for (int col = 0; col < kDashWidth - 3; ++col) {
        float deg = col * kDashDegPerCol;
        float tooth_pos = fmodf(deg, 360.0f);
        int tooth = (int)(tooth_pos / 6.0f);
        float in_tooth = fmodf(tooth_pos, 6.0f);
        bool is_gap = (tooth >= 58);
        Serial.print(is_gap ? '_' : (in_tooth < 3.0f ? '\xDB' : '_'));
    }
    Serial.println("║ 60-2 wheel (gap = __)");

    // ── CMP + IGN + INJ traces (sticky rendering) ───────────────────────
    static const uint8_t kDashChannels[] = {
        (uint8_t)kCmpChan,  // 9
        0, 1, 2, 3,         // IGN1-4
        4, 5, 6, 7          // INJ1-4
    };
    static const int kDashNumChannels = sizeof(kDashChannels) / sizeof(kDashChannels[0]);

    for (int ci = 0; ci < kDashNumChannels; ++ci) {
        const uint8_t ch = kDashChannels[ci];
        if (!kChan[ch].enabled) { continue; }

        // Sticky: percorrer todos os eventos UMA vez, construir níveis por coluna
        // O nível inicial (antes do 1º evento) é LOW para todos os canais.
        bool level = false;
        int ev_idx = 0;
        char trace[kDashWidth + 1];

        for (int col = 0; col < kDashWidth; ++col) {
            int64_t t_col = g_dash_gap1_us + (int64_t)(col * us_per_col);

            // Avançar eventos até ao tempo desta coluna, actualizando o nível
            while (ev_idx < g_dash_event_count && g_dash_events[ev_idx].ts_us <= t_col) {
                if (g_dash_events[ev_idx].ch == ch) {
                    level = (g_dash_events[ev_idx].level == 1u);
                }
                ev_idx++;
            }
            trace[col] = level ? '\xDB' : ' ';
        }
        trace[kDashWidth] = '\0';

        Serial.print("  ║"); Serial.print(kChan[ch].name); Serial.print(trace); Serial.println("║");
    }

    // ── Rodapé com verificação ──────────────────────────────────────────
    Serial.println("  ╠════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣");

    // Extrair pares RISE→FALL de IGN/INJ.
    // Noise filter: em vez de tomar o 1º par (que pode ser ruído em GPIO0/GPIO2),
    // recolhemos TODOS os pares e escolhemos o de MAIOR PW > limiar mínimo (100µs).
    // O spark real tem PW ≥ 1ms, ruído tem PW de µs.
    // Phase alignment: CMP marca cil 1 (~30° na fase A). Se CMP RISE está na
    // 2ª metade (gap2→gap3), gap1 começou 360° depois → subtrair 360°.
    int64_t ign_best_rise[4] = {0, 0, 0, 0};
    int64_t ign_best_fall[4] = {0, 0, 0, 0};
    int64_t ign_best_pw[4] = {0, 0, 0, 0};
    uint8_t ign_pair_count[4] = {0, 0, 0, 0};
    int64_t ign_pending_rise[4] = {0, 0, 0, 0};
    int64_t inj_best_rise[4] = {0, 0, 0, 0};
    int64_t inj_best_fall[4] = {0, 0, 0, 0};
    int64_t inj_best_pw[4] = {0, 0, 0, 0};
    uint8_t inj_pair_count[4] = {0, 0, 0, 0};
    int64_t inj_pending_rise[4] = {0, 0, 0, 0};
    int ign_order[4] = {0, 1, 2, 3};
    int cmp_rise_count = 0;
    int64_t cmp_rise_ts = 0;  // timestamp do 1º CMP RISE p/ phase alignment

    static constexpr int64_t kMinPwUs = 100;  // rejeitar pares com PW < 100µs (ruído)

    for (int i = 0; i < g_dash_event_count; ++i) {
        const DashEvent& e = g_dash_events[i];
        if (e.ch >= 0 && e.ch < 4) {
            // Canal IGN (0-3)
            if (e.level == 1u) {
                ign_pending_rise[e.ch] = e.ts_us;
            } else if (e.level == 0u && ign_pending_rise[e.ch] > 0) {
                const int64_t pw = e.ts_us - ign_pending_rise[e.ch];
                if (pw > kMinPwUs && pw > ign_best_pw[e.ch]) {
                    ign_best_rise[e.ch] = ign_pending_rise[e.ch];
                    ign_best_fall[e.ch] = e.ts_us;
                    ign_best_pw[e.ch] = pw;
                }
                ign_pair_count[e.ch]++;
                ign_pending_rise[e.ch] = 0;
            }
        }
        if (e.ch >= 4 && e.ch < 8) {
            // Canal INJ (4-7 → INJ1-4)
            const uint8_t ich = e.ch - 4u;
            if (e.level == 1u) {
                inj_pending_rise[ich] = e.ts_us;
            } else if (e.level == 0u && inj_pending_rise[ich] > 0) {
                const int64_t pw = e.ts_us - inj_pending_rise[ich];
                if (pw > kMinPwUs && pw > inj_best_pw[ich]) {
                    inj_best_rise[ich] = inj_pending_rise[ich];
                    inj_best_fall[ich] = e.ts_us;
                    inj_best_pw[ich] = pw;
                }
                inj_pair_count[ich]++;
                inj_pending_rise[ich] = 0;
            }
        }
        if (e.ch == (uint8_t)kCmpChan && e.level == 1u) {
            if (cmp_rise_count == 0) { cmp_rise_ts = e.ts_us; }
            cmp_rise_count++;
        }
    }

    // ── Phase alignment: CMP marca cil 1. Se CMP na 2ª metade → offset -360° ──
    float phase_offset_deg = 0.0f;
    if (cmp_rise_count >= 1 && g_dash_gap2_us > 0 && cmp_rise_ts > g_dash_gap2_us) {
        phase_offset_deg = -360.0f;
    }

    // Helper para aplicar offset e normalizar para [0, 720)
    auto norm_deg = [&](float deg) -> float {
        deg += phase_offset_deg;
        while (deg < 0.0f) deg += 720.0f;
        while (deg >= 720.0f) deg -= 720.0f;
        return deg;
    };

    // ── Tabela detalhada de ângulos ────────────────────────────────────
    Serial.println("  ╠════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣");
    Serial.println("  ║ Canal  BestFALL(°) BestRISE(°)  PW(°)  Advance(°)  MsPk  INJ RISE(°)  INJ FALL(°)  INJ→IGN  ║");
    Serial.println("  ║ ─────  ─────────── ───────────  ─────  ──────────  ────  ───────────  ───────────  ───────  ║");
    for (int ch = 0; ch < 4; ++ch) {
        float ign_fall_raw = (ign_best_fall[ch] > 0)
            ? (ign_best_fall[ch] - g_dash_gap1_us) / 1000.0f / T_tooth_ms * 6.0f : -1.0f;
        float ign_rise_raw = (ign_best_rise[ch] > 0)
            ? (ign_best_rise[ch] - g_dash_gap1_us) / 1000.0f / T_tooth_ms * 6.0f : -1.0f;
        float inj_rise_raw = (inj_best_rise[ch] > 0)
            ? (inj_best_rise[ch] - g_dash_gap1_us) / 1000.0f / T_tooth_ms * 6.0f : -1.0f;
        float inj_fall_raw = (inj_best_fall[ch] > 0)
            ? (inj_best_fall[ch] - g_dash_gap1_us) / 1000.0f / T_tooth_ms * 6.0f : -1.0f;

        // Aplicar phase alignment
        float ign_fall_deg = norm_deg(ign_fall_raw);
        float ign_rise_deg = norm_deg(ign_rise_raw);
        float inj_rise_deg = norm_deg(inj_rise_raw);
        float inj_fall_deg = norm_deg(inj_fall_raw);

        float inj_to_ign = (ign_fall_deg > 0 && inj_rise_deg > 0) ? ign_fall_deg - inj_rise_deg : 0.0f;

        // Calcular TDC e avanço para este canal
        // Mapeamento: CH0=IGN4(cyl3,pos2,TDC=360°), CH1=IGN3(cyl2,pos1,TDC=180°),
        //             CH2=IGN2(cyl1,pos3,TDC=540°), CH3=IGN1(cyl0,pos0,TDC=0°/720°)
        float tdc_deg = -1.0f;
        if (ch == 3)      { tdc_deg = 0.0f; }      // IGN1 → cyl 0, pos 0
        else if (ch == 1) { tdc_deg = 180.0f; }     // IGN3 → cyl 2, pos 1
        else if (ch == 0) { tdc_deg = 360.0f; }     // IGN4 → cyl 3, pos 2
        else if (ch == 2) { tdc_deg = 540.0f; }     // IGN2 → cyl 1, pos 3
        float adv = (ign_fall_deg > 0 && tdc_deg >= 0)
                   ? fmodf(tdc_deg + 720.0f - ign_fall_deg, 720.0f) : 0.0f;
        if (adv > 360.0f) { adv -= 360.0f; }  // advance em graus (0-180 típico)

        // Indicador de multi-spark
        const char* mspk_str = (ign_pair_count[ch] > 1) ? " ✗" : "  ";

        Serial.printf("  ║ %-5s  %9.1f°  %9.1f°  %4.1f°  %8.1f°  %3s  %9.1f°  %9.1f°  %+6.1f°  ║\n",
                      kChan[ch].name,
                      ign_fall_deg, ign_rise_deg,
                      (ign_best_fall[ch] > 0 && ign_best_rise[ch] > 0)
                          ? (float)(ign_best_fall[ch] - ign_best_rise[ch]) / 1000.0f / T_tooth_ms * 6.0f : 0.0f,
                      adv,
                      mspk_str,
                      inj_rise_deg, inj_fall_deg, inj_to_ign);
    }
    Serial.println("  ╠════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣");

    // Ordenar IGN por timestamp (ordem temporal no ciclo 720°)
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3 - i; ++j)
            if (ign_best_fall[ign_order[j]] > ign_best_fall[ign_order[j+1]]) {
                int tmp = ign_order[j]; ign_order[j] = ign_order[j+1]; ign_order[j+1] = tmp;
            }

    // Ordem temporal esperada p/ firing order 1-3-4-2 com gap1 = TDC cyl1:
    // IGN3@168°(CH1) → IGN4@348°(CH0) → IGN2@528°(CH2) → IGN1@708°(CH3)
    static const uint8_t kExpectedTemporalOrder[4] = {1, 0, 2, 3};  // IGN3, IGN4, IGN2, IGN1
    bool ign_order_ok = true;
    for (int i = 0; i < 4; ++i) {
        const int ch = ign_order[i];
        if (ign_best_fall[ch] <= 0) { ign_order_ok = false; break; }
        if (ign_order[i] != kExpectedTemporalOrder[i]) { ign_order_ok = false; }
    }

    // INJ sync: cada INJ deve começar (RISE) antes do IGN (FALL) do mesmo canal
    bool inj_sync_ok = true;
    for (int i = 0; i < 4; ++i) {
        if (ign_best_fall[i] == 0 || inj_best_rise[i] == 0) { inj_sync_ok = false; break; }
        if (inj_best_rise[i] > ign_best_fall[i]) { inj_sync_ok = false; }
    }

    // Inter-cilindro 180° ± 12° (tolerância maior p/ resolução 6°/col)
    bool spacing_ok = true;
    for (int i = 1; i < 4; ++i) {
        int ch_a = ign_order[i-1], ch_b = ign_order[i];
        if (ign_best_fall[ch_a] > 0 && ign_best_fall[ch_b] > 0) {
            float deg_a = (ign_best_fall[ch_a] - g_dash_gap1_us) / 1000.0f / T_tooth_ms * 6.0f;
            float deg_b = (ign_best_fall[ch_b] - g_dash_gap1_us) / 1000.0f / T_tooth_ms * 6.0f;
            float inter = norm_deg(deg_b) - norm_deg(deg_a);
            if (inter < 0.0f) inter += 720.0f;  // unwrap
            if (fabsf(inter - 180.0f) > 12.0f) { spacing_ok = false; }
        } else { spacing_ok = false; }
    }

    // CMP: 1 RISE por ciclo 720° (cam sensor 2:1 reduction, 1 pulso/720°)
    bool cmp_ok = (cmp_rise_count >= 1);

    // Mostrar ordem detectada
    Serial.print("  ║ Ordem IGN temporal: ");
    for (int i = 0; i < 4; ++i) { if (i) Serial.print("→"); Serial.print(kChan[ign_order[i]].name); }
    Serial.println();

    Serial.printf("  ║ %s ordem  %s INJ sync  %s spacing 180°  %s CMP (%d RISE)          ║\n",
                  ign_order_ok ? "✓" : "✗",
                  inj_sync_ok ? "✓" : "✗",
                  spacing_ok ? "✓" : "✗",
                  cmp_ok ? "✓" : "✗", cmp_rise_count);
    // Nota sobre multi-spark
    bool any_mspk = false;
    for (int i = 0; i < 4; ++i) if (ign_pair_count[i] > 1) { any_mspk = true; break; }
    if (any_mspk) {
        Serial.print("  ║ ℹ Multi-spark detetado em: ");
        bool first = true;
        for (int i = 0; i < 4; ++i) {
            if (ign_pair_count[i] > 1) {
                if (!first) Serial.print(", ");
                Serial.printf("%s(%d sparks)", kChan[i].name, ign_pair_count[i]);
                first = false;
            }
        }
        Serial.println();
    }
    Serial.println("  ╚════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝");
    Serial.println();
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

// ── Toggle INJ channels ────────────────────────────────────────────────────────

static void toggle_inj_channels() {
    bool any_on = false;
    for (int ch = (int)kInjFirst; ch < (int)(kInjFirst + kInjCount); ++ch) {
        if (kChan[ch].enabled) { any_on = true; break; }
    }
    const bool enable = !any_on;
    for (int ch = (int)kInjFirst; ch < (int)(kInjFirst + kInjCount); ++ch) {
        kChan[ch].enabled = enable;
    }
    Serial.printf("  INJ1-4: %s\n", enable ? "LIGADOS" : "DESLIGADOS");
    if (enable) {
        for (int ch = (int)kInjFirst; ch < (int)(kInjFirst + kInjCount); ++ch) {
            attachInterruptArg(kChan[ch].gpio, edge_isr, (void*)(uint32_t)ch, CHANGE);
        }
    } else {
        for (int ch = (int)kInjFirst; ch < (int)(kInjFirst + kInjCount); ++ch) {
            detachInterrupt(kChan[ch].gpio);
        }
    }
}

// ── Help ──────────────────────────────────────────────────────────────────────

static void print_help() {
    Serial.println();
    Serial.println("  OpenEMS ESP32 Scope — Comandos:");
    Serial.println("  ────────────────────────────────");
    Serial.println("  l  Live table (1 s)       p  Pulse log");
    Serial.println("  e  Edge log               w  Waveform bar");
    Serial.println("  t  Timing 720° analysis   d  Dashboard 720°");
    Serial.println("  s  Estatísticas           i  Toggle INJ1-4");
    Serial.println("  r  Reset stats            ?  Esta ajuda");
    Serial.println();
    Serial.println("  Timing (t): captura 720° (3 gaps CKP, gap1→gap3).");
    Serial.println("    Verifica: 1) IGN/INJ dispara 1x/720°; 2) ordem 1-3-4-2;");
    Serial.println("    3) inter-cilindro 180°±3°; 4) CMP 1×/720° (cam sensor phase anchor).");
    Serial.println("    CH8=GPIO10←PA0 (CKP); CH9=GPIO11←GPIO4 (CMP loopback).");
    Serial.println("    INJ1-4 desligados por défice — 'i' para ligar/desligar.");
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
            case 'd': g_mode = Mode::DASH;   dashboard_start(); break;
            case 's': print_stats(); break;
            case 'r': reset_stats(); break;
            case 'i': toggle_inj_channels(); break;
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
