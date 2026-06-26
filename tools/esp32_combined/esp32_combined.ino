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
 *   GPIO 32 ←  PE9        IGN0 (TIM1_CH1)
 *   GPIO 33 ←  PE11       IGN1 (TIM1_CH2)
 *   GPIO 25 ←  PE13       IGN2 (TIM1_CH3)
 *   GPIO 26 ←  PE14       IGN3 (TIM1_CH4)
 *   GPIO 27 ←  PC6        INJ0 (TIM3_CH1)
 *   GPIO 14 ←  PC7        INJ1 (TIM3_CH2)
 *   GPIO 12 ←  PC8        INJ2 (TIM3_CH3)
 *   GPIO 13 ←  PC9        INJ3 (TIM3_CH4)
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
 *     t        Timing analysis 720°: 1×/ciclo, ordem 1-3-4-2, ângulo IGN+INJ
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
    { GPIO_NUM_32,  "IGN0", "PE9",  true  },   // CH0 TIM1_CH1
    { GPIO_NUM_33,  "IGN1", "PE11", true  },   // CH1 TIM1_CH2
    { GPIO_NUM_25,  "IGN2", "PE13", true  },   // CH2 TIM1_CH3
    { GPIO_NUM_26,  "IGN3", "PE14", true  },   // CH3 TIM1_CH4
    { GPIO_NUM_27,  "INJ0", "PC6",  true  },   // CH4 TIM3_CH1
    { GPIO_NUM_14,  "INJ1", "PC7",  true  },   // CH5 TIM3_CH2
    { GPIO_NUM_12,  "INJ2", "PC8",  true  },   // CH6 TIM3_CH3
    { GPIO_NUM_13,  "INJ3", "PC9",  true  },   // CH7 TIM3_CH4
    { kVirtual,     "CKP",  "PA0",  true  },   // CH8 — virtual, via timer ISR
    { kVirtual,     "CMP",  "PA1",  true  },   // CH9 — virtual, via timer ISR
};
static constexpr int kNChan = (int)(sizeof(kChan) / sizeof(kChan[0]));
static constexpr int kCkpChan = 8;  // índice do canal CKP
static constexpr int kCmpChan = 9;  // índice do canal CMP

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
    const int64_t ts = esp_timer_get_time();
    gpio_set_level(CMP_GPIO, 0);
    buf_push(ts, (uint8_t)kCmpChan, 0u);  // scope CH9 FALL
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
            buf_push(ts, (uint8_t)kCmpChan, 1u);  // scope CH9 RISE
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
// ── SECÇÃO 4 — Timing Analysis 720° (sequencial + ângulo)
// ═══════════════════════════════════════════════════════════════════════════
// Captura um ciclo completo de 720° (2 gaps CKP, 60-2) e verifica:
//   1. Cada IGN/INJ dispara exatamente 1× por ciclo
//   2. Ordem de disparo 1-3-4-2 (IGN0→IGN2→IGN3→IGN1)
//   3. Inter-cilindro 180°±3°; ângulo absoluto desde gap1
//   4. CMP (CH9) presente 1×/720° com offset esperado ~30°
//
// CH8 (CKP, virtual) e CH9 (CMP, virtual) são alimentados pelo timer ISR
// do gerador CKP/CMP — não precisam de fio de loopback externo.

static constexpr int kIgnFirst = 0;
static constexpr int kIgnCount = 4;
static constexpr int kInjFirst = 4;
static constexpr int kInjCount = 4;
static constexpr int kCylCount = 4;

// TDC de cada cilindro em dentes desde dente 0 do gap (60-2, firing order 1-3-4-2)
// Cil.1=IGN0: TDC @ dente 0; Cil.3=IGN2: TDC @ 15; Cil.4=IGN3: TDC @ 30; Cil.2=IGN1: TDC @ 45
static constexpr float kTdcDente[kCylCount] = { 0.0f, 15.0f, 30.0f, 45.0f };
static constexpr uint8_t kExpectedFiringOrder[kIgnCount] = {0, 2, 3, 1};

enum class TmState : uint8_t {
    IDLE,
    WAIT_GAP1,   // aguardar 1º gap
    CAPTURE,     // capturar IGN/INJ/CMP entre gap1 e gap2 (360°)
    WAIT_GAP2,   // aguardar 2º gap (confirma 720°)
    VERIFY,      // verificar que nenhum canal disparou 2× no mesmo ciclo
    DONE,
};

struct TimingCapture {
    int64_t  gap1_ts_us;
    int64_t  gap2_ts_us;
    int64_t  last_ckp_fall_us;
    int64_t  ign_ts_us[kIgnCount];
    uint8_t  ign_count[kIgnCount];
    int64_t  inj_ts_us[kInjCount];
    uint8_t  inj_count[kInjCount];
    int64_t  cmp_ts_us;       // 1ª RISE do CMP no ciclo
    uint8_t  cmp_count;       // nº de bordas CMP no ciclo (esperado: 1)
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
        Serial.println("  AVISO: CH8 sem sinal CKP. O gerador está activo?");
        Serial.println("  [TIMING] Usar '+'/'−' para definir RPM primeiro.");
        g_tm_state = TmState::IDLE;
        return;
    }
    // Sinal com period < 200µs (>5kHz dente = >~6000 RPM equiv) é ruído — rejeitar.
    if (g_m[kCkpChan].period_us < 200u) {
        Serial.printf("  ERRO: CKP period=%.3f ms parece ruído (esperado ~0.5-5 ms).\n",
                      g_m[kCkpChan].period_us / 1000.0f);
        Serial.println("  Verificar RPM (comando '+'/'−') e gerador CKP.");
        g_tm_state = TmState::IDLE;
        return;
    }

    g_tm_state = TmState::WAIT_GAP1;
    Serial.printf("  [TIMING] CKP OK (period=%.3f ms). A aguardar 1º gap...\n",
                  g_m[kCkpChan].period_us / 1000.0f);
}

static void timing_feed(const EdgeEvent& ev) {
    TimingCapture& c = g_tm_cap;

    // Timeout global
    if (c.gap1_ts_us > 0 &&
        (ev.ts_us - c.gap1_ts_us) > (int64_t)c.timeout_us) {
        Serial.println("  [TIMING] Timeout — verificar CKP e ligações IGN/INJ.");
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

    // CMP: capturar borda de RISE (pulso activo-alto do gerador)
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

    // ── IGN ─────────────────────────────────────────────────────────────────
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
    // ── INJ ─────────────────────────────────────────────────────────────────
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

    // ── 5. CMP (cam sensor) ─────────────────────────────────────────────────
    static constexpr float kCmpExpectedDeg = 30.0f;
    static constexpr float kCmpToleranceDeg = 18.0f;  // ±3 dentes
    Serial.println();
    Serial.println("  [5] CMP (cam sensor) — esperado 1×/720°");
    bool cmp_ok = false;
    if (c.cmp_count == 0) {
        Serial.println("  CMP   0×  ✗ AUSENTE — verificar CMP_GPIO no gerador");
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

        if (g_tm_state != TmState::IDLE && g_tm_state != TmState::DONE)
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
    Serial.println("    t       Timing 720° IGN+INJ   s  Estatísticas");
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
