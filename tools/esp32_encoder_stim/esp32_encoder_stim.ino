/*
 * esp32_encoder_stim.ino — Estimulador ABZ + CMP para OpenEMS encoder mode
 * ═══════════════════════════════════════════════════════════════════════
 * Substitui o CKP 60-2 do esp32_stimulator/combined quando o firmware STM32
 * está compilado com EMS_MT6835_ENCODER=1 (sem chip MT6835 — TIM2 conta AB).
 *
 * Plataforma: ESP32 clássico (testado) e variantes compatíveis.
 *   Quadratura A/B via LEDC (driver/ledc.h, dois canais no mesmo timer,
 *   fase fixada por hpoint) — não depende de rmt_new_sync_manager(), que
 *   falha sempre no ESP32 clássico (ESP_ERR_NOT_SUPPORTED, ver nota junto a
 *   kRmtResHz). Sem requisito de Arduino core ≥ 3.0 / IDF ≥ 5.1 por causa
 *   disto (o resto do ficheiro — DAC, LEDC do PWM analógico, esp_timer —
 *   já não tinha essa dependência).
 *
 * ── Ligações mínimas (WeAct STM32H562VGT6) ───────────────────────────────
 *
 *  ESP32 GPIO  →  STM32   Sinal
 *  ─────────────────────────────────────────────────────────────────────
 *  GPIO2       →  PA0     Quadratura A  (TIM2_CH1)
 *  GPIO4       →  PA1     Quadratura B  (TIM2_CH2)
 *  GPIO5       →  PC6     CMP came      (TIM3_CH1, captura na descida)
 *  GND         —  GND     referência (obrigatório)
 *
 *  Sensores analógicos — mesma tabela do esp32_stimulator (MAP/TPS DAC,
 *  CLT/IAT/APP/FUEL/OIL/ETB via LEDC + RC 10kΩ/100nF).
 *
 *  Scope opcional (VGT6 IGN/INJ — pinos livres de LEDC):
 *  GPIO32 ← PE9  IGN1    GPIO33 ← PE11 IGN2
 *  GPIO34 ← PE0  INJ1    GPIO35 ← PE2  INJ2
 *
 * ── Protocolo (serial 115200 + TCP :3333) — compatível StimLink ──────────
 *  RPM / MAP / TPS / CLT / IAT / APP / FUEL / OIL / ETB
 *  CMP_TOOTH <0-16383>  — deslocamento do pulso CMP no ciclo 720°
 *  IDLE CRANK CRUISE WOT COAST / STATUS / SCOPE / ?
 *
 * Docs: tools/esp32_encoder_stim/README.md
 *       docs/dev/mt6835_encoder_fork.md (secção estimulador)
 */

#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/ledc.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#include <WiFi.h>
#endif

// ═══════════════════════════════════════════════════════════════════════════
// Pinos
// ═══════════════════════════════════════════════════════════════════════════

#define ENC_A_GPIO   ((gpio_num_t)2)   // → PA0 TIM2_CH1
#define ENC_B_GPIO   ((gpio_num_t)4)   // → PA1 TIM2_CH2
#define CMP_GPIO     ((gpio_num_t)5)   // → PC6 TIM3_CH1

// ═══════════════════════════════════════════════════════════════════════════
// Sensores (espelho esp32_stimulator)
// ═══════════════════════════════════════════════════════════════════════════

struct PwmChan {
    gpio_num_t     gpio;
    ledc_channel_t ch;
    ledc_timer_t   timer;
    const char     name[10];
    const char     stm32[5];
};

static const PwmChan kPwm[] = {
    { GPIO_NUM_13, LEDC_CHANNEL_0, LEDC_TIMER_0, "CLT",      "PB0"  },
    { GPIO_NUM_12, LEDC_CHANNEL_1, LEDC_TIMER_0, "IAT",      "PB1"  },
    { GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_TIMER_0, "APP1",     "PB0"  },
    { GPIO_NUM_27, LEDC_CHANNEL_3, LEDC_TIMER_0, "APP2",     "PB1"  },
    { GPIO_NUM_16, LEDC_CHANNEL_4, LEDC_TIMER_1, "ETB_TPS1", "PA2"  },
    { GPIO_NUM_17, LEDC_CHANNEL_5, LEDC_TIMER_1, "OIL",      "PC5"  },
    { GPIO_NUM_18, LEDC_CHANNEL_6, LEDC_TIMER_1, "FUEL",     "PC0"  },
    { GPIO_NUM_19, LEDC_CHANNEL_7, LEDC_TIMER_1, "ETB_TPS2", "PC1"  },
};
static constexpr int kNPwm = (int)(sizeof(kPwm) / sizeof(kPwm[0]));
enum PwmIdx : int { kCLT = 0, kIAT, kAPP1, kAPP2, kETB1, kOIL, kFUEL, kETB2 };

static constexpr ledc_timer_bit_t kPwmBits = LEDC_TIMER_12_BIT;
static constexpr uint32_t         kPwmFreq = 19000u;

struct SimState {
    uint32_t rpm;
    uint16_t map_kpa;
    uint8_t  tps_pct;
    int16_t  clt_degc;
    int16_t  iat_degc;
    uint8_t  app_pct;
    uint16_t fuel_bar_x10;
    uint16_t oil_bar_x10;
    uint8_t  etb_pct;
    uint16_t cmp_tooth;  // 0..16383 — offset do flanco CMP no ciclo 720°
};

static SimState g_sim;

// STM32 ADC is 0–3.3 V = raw 0–4095. Same scale here (ESP32 DAC/PWM 3.3 V).
static uint16_t map_kpa_to_raw(uint16_t kpa) {
    if (kpa > 300u) kpa = 300u;
    return (uint16_t)((uint32_t)kpa * 4095u / 300u);
}
static uint16_t tps_pct_to_raw(uint8_t pct) {
    if (pct > 100u) pct = 100u;
    return (uint16_t)((uint32_t)pct * 4095u / 100u);
}
static uint16_t temp_to_raw(int16_t degc) {
    int32_t tx10 = (int32_t)degc * 10;
    if (tx10 < -400) tx10 = -400;
    if (tx10 > 1500) tx10 = 1500;
    int32_t idx = ((tx10 + 400) * 127) / 1900;
    if (idx < 0) idx = 0;
    if (idx > 127) idx = 127;
    return (uint16_t)((uint32_t)idx * 32u + 16u);
}
static uint16_t press_bar_x10_to_raw(uint16_t bar_x10) {
    uint32_t raw = (uint32_t)bar_x10 * 100u * 4095u / 2500u;
    if (raw > 4095u) raw = 4095u;
    return (uint16_t)raw;
}
static uint8_t raw_to_dac8(uint16_t raw) { return (uint8_t)(raw >> 4u); }

// ═══════════════════════════════════════════════════════════════════════════
// Quadratura AB via LEDC — dois canais no MESMO timer, hpoint fixo (16384
// counts/volta X4)
// ═══════════════════════════════════════════════════════════════════════════
// TIM2 SMS=encoder mode 3 conta ambas as bordas de A e B → 4 counts por
// ciclo elétrico. 4096 ciclos/volta × 4 = 16384 (paridade MT6835 4096 PPR).
//
// Histórico: a 1ª versão gerava A/B via RMT (2 canais TX independentes) com
// rmt_new_sync_manager() para arrancar os dois no mesmo ciclo de clock. Em
// bancada (ESP32 clássico) rmt_new_sync_manager() falha sempre
// (ESP_ERR_NOT_SUPPORTED — esse sync manager só existe em chips novos:
// S3/C3/H2). Sem ele, a fase inicial entre os dois rmt_transmit() ficava à
// mercê da latência de software (µs), e cada boot/troca de RPM tinha ~50% de
// sair com a quadratura invertida (TIM2 do STM32 conta ao contrário → RPM
// lido = 0, sem nada de errado na fiação — confirmado em bancada, RPM ficou
// preso a 0 mesmo depois de trocar os fios A/B à mão).
//
// Troca para LEDC: A e B são dois canais do MESMO timer LEDC, cada um com
// duty=50% e um `hpoint` (fase de início dentro do período) fixo — B a 1/4
// de período à frente de A. Os dois canais só existem em relação ao MESMO
// contador de hardware do timer, portanto a relação de fase é uma
// propriedade do registrador, não uma corrida entre duas chamadas de API —
// não há "não suportado" possível aqui, LEDC com hpoint é standard em todo
// o silício ESP32 (clássico incluído).
//
// kRmtResHz deixa de ser uma resolução de RMT real (não há RMT nenhum aqui
// para A/B) — fica só como o "relógio virtual" comum que converte a mesma
// unidade `ticks` (usada por phase_ticks_for_rpm) em Hz para o LEDC (A/B) e
// em µs para o esp_timer do CMP (ticks_to_us). Garante que A/B e CMP
// continuam derivados da mesma base, como já era antes desta troca.
// Cap documentado: se jitter falhar na bancada, baixar kRpmMax para 4000.

static constexpr uint32_t kCountsPerRev = 16384u;
static constexpr uint32_t kRpmMin       = 50u;
static constexpr uint32_t kRpmMax       = 6000u;
static constexpr uint32_t kRmtResHz     = 40000000u;  // relógio virtual (25 ns/tick) — ver nota acima
static constexpr uint16_t kRmtMaxDur    = 32767u;

static constexpr ledc_mode_t      kEncSpeedMode = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t     kEncTimer     = LEDC_TIMER_2;   // grupo low-speed: livre (PWM analógico usa high-speed 0/1)
static constexpr ledc_channel_t   kEncChA       = LEDC_CHANNEL_0; // idem — canais 0-7 do grupo low-speed, sem conflito
static constexpr ledc_channel_t   kEncChB       = LEDC_CHANNEL_1;
static constexpr ledc_timer_bit_t kEncBits      = LEDC_TIMER_4_BIT;  // 16 níveis — hpoint exato em 1/4 (=4)
static constexpr uint32_t         kEncDutyMax   = (1u << 4) - 1u;    // 15, resolução de kEncBits

static volatile uint32_t    g_enc_rpm_active = 0u;
static volatile uint32_t    g_cmp_pulse_count = 0u;
static bool                 g_enc_ledc_ready = false;

static uint16_t phase_ticks_for_rpm(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    // ticks = res_hz / f_step ; f_step = rpm/60 * 16384
    //       = res_hz * 60 / (rpm * 16384)
    // ARREDONDA (antes truncava): a 5307 RPM o truncamento dava 6 ticks em vez
    // de 6.9 → crank real a 6103 RPM, erro de 15%.
    const uint64_t den = (uint64_t)rpm * kCountsPerRev;
    uint64_t ticks = ((uint64_t)kRmtResHz * 60ull + den / 2ull) / den;
    if (ticks < 2ull) ticks = 2ull;
    if (ticks > (uint64_t)kRmtMaxDur) ticks = kRmtMaxDur;
    return (uint16_t)ticks;
}

// RPM realmente sintetizado depois da quantização em ticks inteiros — a
// mesma unidade `ticks` que agora vira a frequência LEDC (ver
// elec_hz_from_ticks), então esta continua a prever o erro real.
static uint32_t rpm_from_ticks(uint16_t ticks) {
    const uint64_t den = (uint64_t)ticks * kCountsPerRev;
    return (uint32_t)(((uint64_t)kRmtResHz * 60ull + den / 2ull) / den);
}

// f_elec = kRmtResHz / (4 × ticks): 1 ciclo elétrico = 4 counts TIM2 = 4×ticks
// do relógio virtual. É a frequência de A (e de B, mesmo timer).
static uint32_t elec_hz_from_ticks(uint16_t ticks) {
    const uint64_t den = 4ull * (uint64_t)ticks;
    uint32_t hz = (uint32_t)((kRmtResHz + den / 2ull) / den);
    return hz < 1u ? 1u : hz;
}

static void enc_apply_freq(uint32_t rpm) {
    const uint16_t ticks = phase_ticks_for_rpm(rpm);
    const uint32_t hz = elec_hz_from_ticks(ticks);
    const esp_err_t e = ledc_set_freq(kEncSpeedMode, kEncTimer, hz);
    if (e != ESP_OK) {
        // Falha aqui já mordeu uma vez em silêncio (ver nota em enc_init() —
        // LEDC_AUTO_CLK escolhia um clock lento demais para RPM alto).
        // Reportar sempre, não só na 1ª vez: outra causa de falha no futuro
        // não deve voltar a ficar invisível.
        Serial.printf("  [ENC-STIM] ledc_set_freq falhou (%lu Hz, %s)\n",
                      (unsigned long)hz, esp_err_to_name(e));
    }
}

// ── CMP via esp_timer (1 pulso / 720° = 2 voltas) ─────────────────────────
// TIM3 captura na DESCIDA com pull-up idle HIGH → idle HIGH, pulso = LOW curto.
static esp_timer_handle_t g_cmp_period_tmr = nullptr;
static esp_timer_handle_t g_cmp_release_tmr = nullptr;
static constexpr uint32_t kCmpPulseUs = 200u;  // largura do LOW

static void IRAM_ATTR cmp_release_cb(void* /*arg*/) {
    gpio_set_level(CMP_GPIO, 1);
}

static void IRAM_ATTR cmp_period_cb(void* /*arg*/) {
    gpio_set_level(CMP_GPIO, 0);
    ++g_cmp_pulse_count;
    // Agenda VOLTA a HIGH após kCmpPulseUs (one-shot).
    esp_timer_stop(g_cmp_release_tmr);
    esp_timer_start_once(g_cmp_release_tmr, kCmpPulseUs);
}

// O ciclo 720° tem de ser medido na MESMA base de tempo do virabrequim, ou
// seja em ticks RMT quantizados — não no RPM pedido. Com o RPM pedido, a 6000
// RPM o crank corre a 19.66 ms/ciclo e o CMP a 20.00 ms: o flanco do came
// caminhava ~12° de cambota por ciclo e dava a volta aos 720° em ~1.1 s.
//   720° = 2 voltas = 32768 counts, cada um com `ticks` do relógio RMT
static uint64_t ticks_to_us(uint64_t t) {   // independente de kRmtResHz
    return (t * 1000000ull + kRmtResHz / 2ull) / (uint64_t)kRmtResHz;
}

static uint64_t cmp_period_us_from_ticks(uint16_t ticks) {
    return ticks_to_us((uint64_t)ticks * 2ull * (uint64_t)kCountsPerRev);
}

// Offset CMP_TOOTH: 0..16383 mapeado sobre o CICLO COMPLETO de 720°
// (antes era sobre 1 volta — o came nunca conseguia cair na 2ª volta).
//   tooth =     0 →   0°      tooth =  8192 → 360° (início da 2ª volta)
//   tooth = 12288 → 540° (meio da 2ª volta)   tooth = 16383 → ~720°
// Resolução: 720/16384 = 0.0439° de cambota por unidade.
static uint64_t cmp_first_delay_us(uint16_t ticks, uint16_t tooth) {
    if (tooth > 16383u) tooth = 16383u;
    // (32768 × ticks) × tooth / 16384 = 2 × ticks × tooth  [ticks RMT]
    return ticks_to_us((uint64_t)ticks * (uint64_t)tooth * 2ull);
}

static esp_timer_handle_t g_cmp_first_tmr = nullptr;
static volatile uint64_t  g_cmp_period_us = 0ull;

static void cmp_first_cb(void* /*arg*/) {
    cmp_period_cb(nullptr);
    esp_timer_start_periodic(g_cmp_period_tmr, g_cmp_period_us);
}

static void cmp_timer_restart(uint32_t rpm) {
    if (g_cmp_period_tmr == nullptr) return;
    esp_timer_stop(g_cmp_period_tmr);
    esp_timer_stop(g_cmp_release_tmr);
    if (g_cmp_first_tmr != nullptr) esp_timer_stop(g_cmp_first_tmr);
    gpio_set_level(CMP_GPIO, 1);  // idle HIGH

    const uint16_t ticks  = phase_ticks_for_rpm(rpm);
    const uint64_t period = cmp_period_us_from_ticks(ticks);
    const uint64_t delay0 = cmp_first_delay_us(ticks, g_sim.cmp_tooth);
    g_cmp_period_us = period;
    if (delay0 > 0ull) {
        if (g_cmp_first_tmr == nullptr) {
            esp_timer_create_args_t args = {};
            args.callback = cmp_first_cb;
            args.name = "cmp_first";
            esp_timer_create(&args, &g_cmp_first_tmr);
        }
        esp_timer_start_once(g_cmp_first_tmr, delay0);
    } else {
        esp_timer_start_periodic(g_cmp_period_tmr, period);
    }
}

static void enc_set_rpm(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    if (rpm > kRpmMax) rpm = kRpmMax;
    if (g_enc_ledc_ready) {
        // Só muda a frequência do timer partilhado — hpoint de cada canal
        // (fixo desde enc_init()) não precisa de re-arme, a fase entre A e B
        // é uma propriedade do par duty/hpoint sobre o MESMO contador, não
        // algo que se perca ao trocar de RPM.
        enc_apply_freq(rpm);
    }
    g_enc_rpm_active = rpm;
    cmp_timer_restart(rpm);
}

// ── Rampa suave de RPM ───────────────────────────────────────────────────
// g_sim.rpm é o ALVO (o que RPM/presets pedem); g_enc_rpm_active é o que
// está de facto a ser sintetizado. Sem isto, IDLE→WOT era um salto
// instantâneo de frequência — nenhum motor real acelera em 0 s, e o STM32
// via um "teleporte" de RPM em vez de uma aceleração real (pedido do
// utilizador, 2026-08-13, depois de observar os saltos nos presets).
// enc_ramp_tick() aproxima g_enc_rpm_active de g_sim.rpm a um ritmo
// limitado, chamado de loop() a cada kRampStepMs.
static constexpr uint32_t kRampStepMs    = 50u;
static constexpr uint32_t kRampRpmPerSec = 3000u;  // 0→6000 RPM em ~2 s
static uint32_t g_ramp_last_ms = 0u;
static bool     g_enc_signal_lost = false;  // ver enc_signal_lost_set()

static void enc_ramp_tick() {
    if (!g_enc_ledc_ready || g_enc_signal_lost) return;
    const uint32_t now_ms = (uint32_t)millis();
    if (now_ms - g_ramp_last_ms < kRampStepMs) return;
    g_ramp_last_ms = now_ms;

    const uint32_t target = g_sim.rpm;
    if (g_enc_rpm_active == target) return;

    const uint32_t max_step = (kRampRpmPerSec * kRampStepMs) / 1000u;
    uint32_t next;
    if (g_enc_rpm_active < target) {
        const uint32_t remaining = target - g_enc_rpm_active;
        next = g_enc_rpm_active + (remaining < max_step ? remaining : max_step);
    } else {
        const uint32_t remaining = g_enc_rpm_active - target;
        next = g_enc_rpm_active - (remaining < max_step ? remaining : max_step);
    }
    enc_set_rpm(next);
}

// ── Simulação de perda de sinal (comando STOP / RESUME) ─────────────────
// Testa o watchdog novo do STM32 (ckp_stall_poll_encoder(), ckp.cpp): mata
// A/B (ledc_stop — pino fica preso no idle_level, zero transições, TIM2
// congela de verdade) e o CMP (pára os esp_timer), em vez de só levar o RPM
// a kRpmMin — isso ainda geraria bordas reais, não testava o caso "sensor
// morto" que o watchdog existe para cobrir.
static void enc_signal_lost_set(bool lost) {
    if (lost == g_enc_signal_lost) return;
    g_enc_signal_lost = lost;
    if (lost) {
        if (g_enc_ledc_ready) {
            ledc_stop(kEncSpeedMode, kEncChA, 0);
            ledc_stop(kEncSpeedMode, kEncChB, 0);
        }
        esp_timer_stop(g_cmp_period_tmr);
        esp_timer_stop(g_cmp_release_tmr);
        if (g_cmp_first_tmr != nullptr) esp_timer_stop(g_cmp_first_tmr);
        gpio_set_level(CMP_GPIO, 1);  // idle HIGH, sem mais pulsos
        Serial.println("  [ENC-STIM] SINAL PERDIDO (A/B/CMP mortos) — "
                        "RPM/CMP no STM32 devem cair a 0/LOSS_OF_SYNC");
    } else {
        if (g_enc_ledc_ready) {
            // ledc_stop() desliga o CANAL (pino preso no idle_level) — só
            // ledc_set_freq() (dentro de enc_set_rpm(), via enc_apply_freq())
            // mexe no TIMER partilhado, nunca reactiva um canal parado.
            // Confirmado em bancada: sem isto, RESUME fazia CMP voltar (usa
            // esp_timer/gpio_set_level directo, não LEDC) mas A/B ficavam
            // mudos — TIM2 do STM32 nunca recomeçava a contar, sync preso em
            // LOSS_OF_SYNC apesar do ESP32 já reportar RPM activo correcto.
            // ledc_set_duty()+ledc_update_duty() reaplica o duty/hpoint fixo
            // (ver enc_init()) e volta a gerar o sinal.
            ledc_set_duty_with_hpoint(kEncSpeedMode, kEncChA,
                                       kEncDutyMax / 2u, (kEncDutyMax + 1u) / 4u);
            ledc_update_duty(kEncSpeedMode, kEncChA);
            ledc_set_duty_with_hpoint(kEncSpeedMode, kEncChB, kEncDutyMax / 2u, 0);
            ledc_update_duty(kEncSpeedMode, kEncChB);
        }
        g_ramp_last_ms = (uint32_t)millis();
        enc_set_rpm(g_enc_rpm_active);  // religa CMP + garante a frequência certa
        Serial.println("  [ENC-STIM] Sinal restaurado");
    }
}

static void enc_init() {
    ledc_timer_config_t t = {};
    t.speed_mode      = kEncSpeedMode;
    t.duty_resolution = kEncBits;
    t.timer_num       = kEncTimer;
    t.freq_hz         = elec_hz_from_ticks(phase_ticks_for_rpm(g_sim.rpm));
    // LEDC_AUTO_CLK escolhia REF_TICK (1 MHz) no grupo low-speed — máximo
    // 1MHz/2^kEncBits = 62.5 kHz, suficiente só até ~900 RPM. Acima disso
    // ledc_set_freq() falhava silenciosamente (não verificávamos o retorno)
    // e a frequência ficava presa no último valor válido — confirmado em
    // bancada: RPM saltava correctamente 700→200 (CRANK, desce) mas ficava
    // preso a 698 em CRUISE/WOT/COAST (RPM pedido > ~900, precisa de
    // 80MHz/16=5 MHz de tecto). Fixar APB_CLK explicitamente resolve.
    t.clk_cfg         = LEDC_USE_APB_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    // A e B no MESMO timer (kEncTimer): duty=50% nos dois, hpoint de A a 1/4
    // de período à frente de B. A relação de fase fica gravada nos
    // registradores duty/hpoint de cada canal, referenciados ao mesmo
    // contador de hardware — não há sequência de chamadas de API cujo
    // atraso relativo possa inverter a direção (ver nota no topo do
    // ficheiro: era exactamente esse o problema com RMT+sync manager).
    // Sentido (A adianta B, não o contrário) verificado em bancada: com
    // B a adiantar A o TIM2 do STM32 contava para trás (RPM=0 sempre,
    // determinístico — ver histórico git). Trocado aqui, não é arbitrário.
    ledc_channel_config_t ca = {};
    ca.gpio_num   = (int)ENC_A_GPIO;
    ca.speed_mode = kEncSpeedMode;
    ca.channel    = kEncChA;
    ca.intr_type  = LEDC_INTR_DISABLE;
    ca.timer_sel  = kEncTimer;
    ca.duty       = kEncDutyMax / 2u;   // 50%
    ca.hpoint     = (kEncDutyMax + 1u) / 4u;  // 1/4 de período = 90° eléctricos
    ESP_ERROR_CHECK(ledc_channel_config(&ca));

    ledc_channel_config_t cb = ca;
    cb.gpio_num = (int)ENC_B_GPIO;
    cb.channel  = kEncChB;
    cb.hpoint   = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&cb));

    g_enc_ledc_ready = true;

    {
        esp_timer_create_args_t args = {};
        args.callback = cmp_period_cb;
        args.name = "cmp_period";
        ESP_ERROR_CHECK(esp_timer_create(&args, &g_cmp_period_tmr));
    }
    {
        esp_timer_create_args_t args = {};
        args.callback = cmp_release_cb;
        args.name = "cmp_release";
        ESP_ERROR_CHECK(esp_timer_create(&args, &g_cmp_release_tmr));
    }

    gpio_reset_pin(CMP_GPIO);
    gpio_set_direction(CMP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CMP_GPIO, 1);

    g_enc_rpm_active = g_sim.rpm;
    cmp_timer_restart(g_sim.rpm);
}

// ═══════════════════════════════════════════════════════════════════════════
// Analógicos + presets
// ═══════════════════════════════════════════════════════════════════════════

static void set_ledc(PwmIdx idx, uint16_t raw) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, kPwm[idx].ch, (uint32_t)raw);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, kPwm[idx].ch);
}

static void update_analog(const SimState& s) {
    dac_output_voltage(DAC_CHANNEL_1, raw_to_dac8(map_kpa_to_raw(s.map_kpa)));
    dac_output_voltage(DAC_CHANNEL_2, raw_to_dac8(tps_pct_to_raw(s.tps_pct)));
    set_ledc(kCLT, temp_to_raw(s.clt_degc));
    set_ledc(kIAT, temp_to_raw(s.iat_degc));
    const uint16_t app_raw = tps_pct_to_raw(s.app_pct);
    set_ledc(kAPP1, app_raw);
    set_ledc(kAPP2, app_raw);
    set_ledc(kFUEL, press_bar_x10_to_raw(s.fuel_bar_x10));
    set_ledc(kOIL, press_bar_x10_to_raw(s.oil_bar_x10));
    set_ledc(kETB1, tps_pct_to_raw(s.etb_pct));
    set_ledc(kETB2, tps_pct_to_raw((uint8_t)(100u - s.etb_pct)));
}

static void apply_preset(const SimState& p, const char* label) {
    const uint16_t tooth = g_sim.cmp_tooth;  // preserva CMP_TOOTH
    g_sim = p;
    g_sim.cmp_tooth = tooth;
    update_analog(g_sim);
    // Só actualiza o ALVO — enc_ramp_tick() (loop()) aproxima g_enc_rpm_active
    // gradualmente, em vez de saltar direto para o RPM do preset.
    enc_signal_lost_set(false);
    Serial.printf("  [ENC-STIM] Preset: %s\n", label);
}

static void preset_idle() {
    apply_preset({ 1500, 35, 3, 90, 25, 0, 35, 20, 3, 0 }, "IDLE");
}
static void preset_crank() {
    apply_preset({ 200, 101, 0, 20, 15, 0, 20, 5, 0, 0 }, "CRANK");
}
static void preset_cruise() {
    apply_preset({ 2000, 55, 20, 90, 35, 20, 35, 30, 20, 0 }, "CRUISE");
}
static void preset_wot() {
    apply_preset({ 4000, 100, 100, 90, 40, 100, 38, 40, 100, 0 }, "WOT");
}
static void preset_coast() {
    apply_preset({ 2000, 25, 0, 90, 35, 0, 35, 25, 0, 0 }, "COAST");
}

// ═══════════════════════════════════════════════════════════════════════════
// Scope IGN/INJ (opcional — GPIO livres de LEDC)
// ═══════════════════════════════════════════════════════════════════════════

struct ScopeCh {
    gpio_num_t gpio;
    const char* name;
    volatile uint32_t count;
    volatile uint32_t rise_us;
    volatile uint32_t pw_us;
    volatile uint32_t period_us;
    volatile uint32_t last_rise_us;
};

static ScopeCh g_scope[] = {
    { GPIO_NUM_32, "IGN1", 0, 0, 0, 0, 0 },  // ← PE9
    { GPIO_NUM_33, "IGN2", 0, 0, 0, 0, 0 },  // ← PE11
    { GPIO_NUM_34, "INJ1", 0, 0, 0, 0, 0 },  // ← PE0 (input-only)
    { GPIO_NUM_35, "INJ2", 0, 0, 0, 0, 0 },  // ← PE2 (input-only)
};
static constexpr int kScopeCh = 4;

static void IRAM_ATTR scope_isr(void* arg) {
    ScopeCh* ch = (ScopeCh*)arg;
    const uint32_t now = (uint32_t)esp_timer_get_time();
    if (gpio_get_level(ch->gpio)) {
        ch->rise_us = now;
        if (ch->last_rise_us != 0u) ch->period_us = now - ch->last_rise_us;
        ch->last_rise_us = now;
        ch->count++;
    } else if (ch->rise_us != 0u) {
        ch->pw_us = now - ch->rise_us;
    }
}

static void scope_init() {
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    for (int i = 0; i < kScopeCh; ++i) {
        gpio_reset_pin(g_scope[i].gpio);
        gpio_set_direction(g_scope[i].gpio, GPIO_MODE_INPUT);
        gpio_set_pull_mode(g_scope[i].gpio, GPIO_PULLDOWN_ONLY);
        gpio_set_intr_type(g_scope[i].gpio, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(g_scope[i].gpio, scope_isr, &g_scope[i]);
    }
}

static void scope_print() {
    Serial.println("  ── Output Scope (IGN1/2 PE9/11, INJ1/2 PE0/2) ──");
    Serial.println("  CH   Name   PW(ms)   Per(ms)  Freq(Hz)  Count");
    for (int i = 0; i < kScopeCh; ++i) {
        const ScopeCh& c = g_scope[i];
        const float pw_ms = c.pw_us / 1000.0f;
        const float per_ms = c.period_us / 1000.0f;
        const float freq = (c.period_us > 0) ? 1000000.0f / c.period_us : 0.0f;
        Serial.printf("  %d    %-4s   %6.2f   %7.2f  %8.1f  %lu\n",
                      i, c.name, pw_ms, per_ms, freq, (unsigned long)c.count);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Comandos
// ═══════════════════════════════════════════════════════════════════════════

static void print_status() {
    const SimState& s = g_sim;
    // g_enc_rpm_active é o que está DE FACTO a ser sintetizado agora — com a
    // rampa (enc_ramp_tick()), pode diferir do alvo (s.rpm) durante uma
    // transição. Todos os cálculos de frequência/CMP abaixo têm de usar o
    // activo, senão o STATUS mostrava a frequência do alvo em vez da real
    // enquanto o motor "acelera".
    const uint32_t active  = g_enc_rpm_active;
    const uint16_t ticks   = phase_ticks_for_rpm(active);
    const uint32_t rpm_act = rpm_from_ticks(ticks);   // depois da quantização
    const float f_a_hz = (rpm_act / 60.0f) * 4096.0f;  // freq eléctrica de A
    const float err_pct = 100.0f * ((float)rpm_act - (float)active) / (float)active;
    const float cmp_deg = (s.cmp_tooth * 720.0f) / 16384.0f;
    Serial.println();
    Serial.println("  ╔═══════════════════════════════════════════════════════════╗");
    Serial.println("  ║  OpenEMS Encoder Stim — Estado                            ║");
    Serial.println("  ╚═══════════════════════════════════════════════════════════╝");
    if (g_enc_signal_lost) {
        Serial.println("  SINAL PERDIDO (STOP activo) — A/B/CMP mortos, sem transições");
    }
    Serial.printf("  RPM alvo:   %lu\n", (unsigned long)s.rpm);
    Serial.printf("  RPM activo: %lu%s\n", (unsigned long)active,
                  (active == s.rpm) ? "" : "  (a rampar)");
    Serial.printf("  RPM real:   %lu  (erro %+.2f%% — quantização de ticks)\n",
                  (unsigned long)rpm_act, err_pct);
    Serial.printf("  A freq:     %.1f Hz  (4096×RPM_real/60)\n", f_a_hz);
    Serial.printf("  phase_tick: %u ticks @ %.1f MHz (relógio virtual)\n", ticks,
                  kRmtResHz / 1000000.0);
    Serial.printf("  A/B sync:   LEDC hpoint (garantido por hardware)\n");
    Serial.printf("  CMP_TOOTH:  %u / 16383  (%.1f° cambota no ciclo 720°)\n",
                  s.cmp_tooth, cmp_deg);
    Serial.printf("  CMP per:    %.3f ms  (= 32768 counts × %u ticks)\n",
                  (double)g_cmp_period_us / 1000.0, ticks);
    Serial.printf("  CMP pulses: %lu\n", (unsigned long)g_cmp_pulse_count);
    Serial.printf("  MAP: %u kPa  TPS: %u%%  CLT: %d°C  IAT: %d°C\n",
                  s.map_kpa, s.tps_pct, s.clt_degc, s.iat_degc);
    Serial.println();
    scope_print();
    Serial.println();
}

static void print_help() {
    Serial.println();
    Serial.println("  OpenEMS ESP32 Encoder Stim (ABZ + CMP → TIM2/TIM3)");
    Serial.println("  ─────────────────────────────────────────────────");
    Serial.println("  RPM MAP TPS CLT IAT APP FUEL OIL ETB <n>");
    Serial.println("  CMP_TOOTH <0-16383>   offset do CMP no ciclo 720°");
    Serial.println("                        0=0°  8192=360°  12288=540°");
    Serial.println("  IDLE CRANK CRUISE WOT COAST   (RPM ramps a 3000 RPM/s, sem saltos)");
    Serial.println("  STOP    mata A/B/CMP — simula sensor morto (testa watchdog do STM32)");
    Serial.println("  RESUME  religa A/B/CMP na cadência actual");
    Serial.println("  STATUS  SCOPE  ?");
    Serial.println();
    Serial.printf("  GPIO%-2d → PA0  A (quad)\n", (int)ENC_A_GPIO);
    Serial.printf("  GPIO%-2d → PA1  B (quad)\n", (int)ENC_B_GPIO);
    Serial.printf("  GPIO%-2d → PC6  CMP (1/720°)\n", (int)CMP_GPIO);
    Serial.println("  GPIO25 → PA3  MAP (DAC)   GPIO26 → PA4  TPS (DAC)");
    Serial.println();
}

static void parse_cmd(const char* raw) {
    char buf[64];
    int len = 0;
    while (raw[len] && len < 63) {
        buf[len] = raw[len];
        len++;
    }
    buf[len] = '\0';
    for (int i = 0; i < len; ++i)
        if (buf[i] >= 'a' && buf[i] <= 'z') buf[i] -= 32;

    char cmd[16] = {};
    int val = 0;
    const bool has_val = (sscanf(buf, "%15s %d", cmd, &val) >= 2);
    bool changed = true;

    if (strcmp(cmd, "RPM") == 0 && has_val) {
        g_sim.rpm = (uint32_t)constrain(val, (int)kRpmMin, (int)kRpmMax);
        // Só actualiza o ALVO — enc_ramp_tick() (loop()) faz a transição
        // gradual, não um salto instantâneo de frequência.
        enc_signal_lost_set(false);
        Serial.printf("  [ENC-STIM] RPM alvo=%lu (a rampar)\n", (unsigned long)g_sim.rpm);
    } else if (strcmp(cmd, "STOP") == 0) {
        enc_signal_lost_set(true);
        changed = false;
    } else if (strcmp(cmd, "RESUME") == 0) {
        enc_signal_lost_set(false);
        Serial.println("  [ENC-STIM] RESUME");
        changed = false;
    } else if (strcmp(cmd, "CMP_TOOTH") == 0 && has_val) {
        g_sim.cmp_tooth = (uint16_t)constrain(val, 0, 16383);
        cmp_timer_restart(g_sim.rpm);
        Serial.printf("  [ENC-STIM] CMP_TOOTH=%u\n", g_sim.cmp_tooth);
        changed = false;
    } else if (strcmp(cmd, "MAP") == 0 && has_val) {
        g_sim.map_kpa = (uint16_t)constrain(val, 0, 300);
        Serial.printf("  [ENC-STIM] MAP=%u kPa\n", g_sim.map_kpa);
    } else if (strcmp(cmd, "TPS") == 0 && has_val) {
        g_sim.tps_pct = (uint8_t)constrain(val, 0, 100);
        Serial.printf("  [ENC-STIM] TPS=%u%%\n", g_sim.tps_pct);
    } else if (strcmp(cmd, "CLT") == 0 && has_val) {
        g_sim.clt_degc = (int16_t)constrain(val, -40, 150);
        Serial.printf("  [ENC-STIM] CLT=%d°C\n", g_sim.clt_degc);
    } else if (strcmp(cmd, "IAT") == 0 && has_val) {
        g_sim.iat_degc = (int16_t)constrain(val, -40, 150);
        Serial.printf("  [ENC-STIM] IAT=%d°C\n", g_sim.iat_degc);
    } else if (strcmp(cmd, "APP") == 0 && has_val) {
        g_sim.app_pct = (uint8_t)constrain(val, 0, 100);
        Serial.printf("  [ENC-STIM] APP=%u%%\n", g_sim.app_pct);
    } else if (strcmp(cmd, "FUEL") == 0 && has_val) {
        g_sim.fuel_bar_x10 = (uint16_t)constrain(val, 0, 50);
        Serial.printf("  [ENC-STIM] FUEL=%.1f bar\n", g_sim.fuel_bar_x10 / 10.0f);
    } else if (strcmp(cmd, "OIL") == 0 && has_val) {
        g_sim.oil_bar_x10 = (uint16_t)constrain(val, 0, 50);
        Serial.printf("  [ENC-STIM] OIL=%.1f bar\n", g_sim.oil_bar_x10 / 10.0f);
    } else if (strcmp(cmd, "ETB") == 0 && has_val) {
        g_sim.etb_pct = (uint8_t)constrain(val, 0, 100);
        Serial.printf("  [ENC-STIM] ETB=%u%%\n", g_sim.etb_pct);
    } else if (strcmp(cmd, "IDLE") == 0) {
        preset_idle();
    } else if (strcmp(cmd, "CRANK") == 0) {
        preset_crank();
    } else if (strcmp(cmd, "CRUISE") == 0) {
        preset_cruise();
    } else if (strcmp(cmd, "WOT") == 0) {
        preset_wot();
    } else if (strcmp(cmd, "COAST") == 0) {
        preset_coast();
    } else if (strcmp(cmd, "STATUS") == 0) {
        print_status();
        changed = false;
    } else if (strcmp(cmd, "SCOPE") == 0) {
        scope_print();
        changed = false;
    } else if (strcmp(cmd, "?") == 0) {
        print_help();
        changed = false;
    } else {
        Serial.printf("  [ENC-STIM] Comando desconhecido: %s\n", buf);
        changed = false;
    }
    if (changed) update_analog(g_sim);
}

static char g_line[64];
static int  g_line_pos = 0;

static void feed_char(char c) {
    if (c == '\r') return;
    if (c == '\n') {
        g_line[g_line_pos] = '\0';
        if (g_line_pos > 0) parse_cmd(g_line);
        g_line_pos = 0;
        return;
    }
    if (g_line_pos < 63) g_line[g_line_pos++] = c;
}

// ── WiFi TCP :3333 (StimLink) ─────────────────────────────────────────────
#if defined(WIFI_SSID)
static WiFiServer g_tcp(3333);
static WiFiClient g_tcp_client;
static char       g_tcp_line[96];
static int        g_tcp_pos = 0;

static void wifi_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WIFI] ligando a " WIFI_SSID " ");
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] OK  IP=%s  TCP 3333\n",
                      WiFi.localIP().toString().c_str());
        g_tcp.begin();
        g_tcp.setNoDelay(true);
    } else {
        Serial.println("\n[WIFI] FALHOU — só serial");
    }
}

static void wifi_poll() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!g_tcp_client || !g_tcp_client.connected()) {
        g_tcp_client = g_tcp.available();
        g_tcp_pos = 0;
    }
    while (g_tcp_client && g_tcp_client.available()) {
        const char c = (char)g_tcp_client.read();
        if (c == '\r') continue;
        if (c == '\n') {
            g_tcp_line[g_tcp_pos] = '\0';
            if (g_tcp_pos > 0) parse_cmd(g_tcp_line);
            g_tcp_pos = 0;
        } else if (g_tcp_pos < 95) {
            g_tcp_line[g_tcp_pos++] = c;
        }
    }
}
#else
static void wifi_setup() {
    Serial.println("[WIFI] sem wifi_credentials.h — só serial");
}
static void wifi_poll() {}
#endif

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(300);

    for (gpio_num_t g : { ENC_A_GPIO, ENC_B_GPIO, CMP_GPIO }) {
        gpio_reset_pin(g);
        gpio_set_direction(g, GPIO_MODE_OUTPUT);
        gpio_set_level(g, (g == CMP_GPIO) ? 1 : 0);
    }

    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);

    {
        ledc_timer_config_t t;
        memset(&t, 0, sizeof(t));
        t.speed_mode = LEDC_HIGH_SPEED_MODE;
        t.duty_resolution = kPwmBits;
        t.timer_num = LEDC_TIMER_0;
        t.freq_hz = kPwmFreq;
        t.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));
        t.timer_num = LEDC_TIMER_1;
        ESP_ERROR_CHECK(ledc_timer_config(&t));
    }
    for (int i = 0; i < kNPwm; ++i) {
        ledc_channel_config_t cc;
        memset(&cc, 0, sizeof(cc));
        cc.gpio_num = (int)kPwm[i].gpio;
        cc.speed_mode = LEDC_HIGH_SPEED_MODE;
        cc.channel = kPwm[i].ch;
        cc.intr_type = LEDC_INTR_DISABLE;
        cc.timer_sel = kPwm[i].timer;
        cc.duty = 0u;
        cc.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&cc));
    }

    // 12288/16384 × 720° = 540° → meio da 2ª volta (com 8192 dava 360°)
    g_sim = { 1500, 35, 3, 90, 25, 0, 35, 20, 3, 12288 };
    update_analog(g_sim);
    enc_init();
    scope_init();
    wifi_setup();
    print_help();
    print_status();
}

void loop() {
    while (Serial.available()) feed_char((char)Serial.read());
    wifi_poll();
    enc_ramp_tick();
    delay(1);
}
