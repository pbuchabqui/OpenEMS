/*
 * esp32_encoder_stim.ino — Estimulador ABZ + CMP para OpenEMS encoder mode
 * ═══════════════════════════════════════════════════════════════════════
 * Substitui o CKP 60-2 do esp32_stimulator/combined quando o firmware STM32
 * está compilado com EMS_MT6835_ENCODER=1 (sem chip MT6835 — TIM2 conta AB).
 *
 * Plataforma: ESP32 (Arduino core ≥ 2.0 / ESP-IDF ≥ 4.4)
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
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
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

static uint16_t map_kpa_to_raw(uint16_t kpa) {
    if (kpa > 300u) kpa = 300u;
    return (uint16_t)((uint32_t)kpa * 4095u / 300u);
}
static uint16_t tps_pct_to_raw(uint8_t pct) {
    if (pct > 100u) pct = 100u;
    return (uint16_t)(200u + (uint32_t)pct * 3695u / 100u);
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
// Quadratura AB via RMT — 4 fases Gray em loop (16384 counts/volta X4)
// ═══════════════════════════════════════════════════════════════════════════
// TIM2 SMS=encoder mode 3 conta ambas as bordas de A e B → 4 counts por
// ciclo elétrico. 4096 ciclos/volta × 4 = 16384 (paridade MT6835 4096 PPR).
//
// Resolução RMT 10 MHz (0.1 µs/tick) → 6000 RPM ≈ 6 ticks/fase (ok).
// Cap documentado: se jitter falhar na bancada, baixar kRpmMax para 4000.

static constexpr uint32_t kCountsPerRev = 16384u;
static constexpr uint32_t kRpmMin       = 50u;
static constexpr uint32_t kRpmMax       = 6000u;
static constexpr uint32_t kRmtResHz     = 10000000u;  // 0.1 µs
static constexpr uint16_t kRmtMaxDur    = 32767u;
static constexpr int      kGrayPhases  = 4;

static rmt_channel_handle_t g_a_chan = nullptr;
static rmt_channel_handle_t g_b_chan = nullptr;
static rmt_encoder_handle_t g_a_enc  = nullptr;
static rmt_encoder_handle_t g_b_enc  = nullptr;
static rmt_symbol_word_t    g_a_sym[kGrayPhases];
static rmt_symbol_word_t    g_b_sym[kGrayPhases];
static volatile uint32_t    g_enc_rpm_active = 0u;
static volatile uint32_t    g_cmp_pulse_count = 0u;

// Gray: fase → (A,B): 0→(0,0) 1→(1,0) 2→(1,1) 3→(0,1)
static const uint8_t kGrayA[kGrayPhases] = { 0, 1, 1, 0 };
static const uint8_t kGrayB[kGrayPhases] = { 0, 0, 1, 1 };

static uint16_t phase_ticks_for_rpm(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    // ticks = res_hz / f_step ; f_step = rpm/60 * 16384
    //       = res_hz * 60 / (rpm * 16384)
    uint64_t ticks = (uint64_t)kRmtResHz * 60ull / ((uint64_t)rpm * kCountsPerRev);
    if (ticks < 1ull) ticks = 1ull;
    if (ticks > (uint64_t)kRmtMaxDur) ticks = kRmtMaxDur;
    return (uint16_t)ticks;
}

static void build_quad_pattern(uint32_t rpm) {
    // Cada fase Gray = 1 count TIM2. Total ticks = d0+d1 com mesmo nível.
    uint16_t d = phase_ticks_for_rpm(rpm);
    if (d < 2u) d = 2u;
    const uint16_t d0 = (uint16_t)(d - 1u);
    const uint16_t d1 = 1u;
    for (int i = 0; i < kGrayPhases; ++i) {
        g_a_sym[i].level0 = kGrayA[i];
        g_a_sym[i].duration0 = d0;
        g_a_sym[i].level1 = kGrayA[i];
        g_a_sym[i].duration1 = d1;
        g_b_sym[i].level0 = kGrayB[i];
        g_b_sym[i].duration0 = d0;
        g_b_sym[i].level1 = kGrayB[i];
        g_b_sym[i].duration1 = d1;
    }
}

static void transmit_ab() {
    rmt_transmit_config_t txcfg = {};
    txcfg.loop_count = -1;  // infinito
    rmt_transmit(g_a_chan, g_a_enc, g_a_sym, sizeof(g_a_sym), &txcfg);
    rmt_transmit(g_b_chan, g_b_enc, g_b_sym, sizeof(g_b_sym), &txcfg);
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

static uint64_t cmp_period_us_for_rpm(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    // 2 voltas: T = 2 * 60e6 / rpm µs
    return (120000000ull) / (uint64_t)rpm;
}

// Offset CMP_TOOTH: atrasa o 1º pulso em (tooth/16384) * 1_rev_us.
// tooth=0 → flanco no início do ciclo 720°; tooth=8192 → meio da 2ª volta.
static uint64_t cmp_first_delay_us(uint32_t rpm, uint16_t tooth) {
    if (tooth > 16383u) tooth = 16383u;
    const uint64_t rev_us = 60000000ull / (uint64_t)rpm;
    return (rev_us * (uint64_t)tooth) / kCountsPerRev;
}

static esp_timer_handle_t g_cmp_first_tmr = nullptr;

static void cmp_first_cb(void* /*arg*/) {
    cmp_period_cb(nullptr);
    esp_timer_start_periodic(g_cmp_period_tmr, cmp_period_us_for_rpm(g_sim.rpm));
}

static void cmp_timer_restart(uint32_t rpm) {
    if (g_cmp_period_tmr == nullptr) return;
    esp_timer_stop(g_cmp_period_tmr);
    esp_timer_stop(g_cmp_release_tmr);
    if (g_cmp_first_tmr != nullptr) esp_timer_stop(g_cmp_first_tmr);
    gpio_set_level(CMP_GPIO, 1);  // idle HIGH

    const uint64_t period = cmp_period_us_for_rpm(rpm);
    const uint64_t delay0 = cmp_first_delay_us(rpm, g_sim.cmp_tooth);
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
    if (rpm == g_enc_rpm_active && g_a_chan != nullptr) {
        // Ainda actualiza CMP (tooth pode ter mudado).
        cmp_timer_restart(rpm);
        return;
    }
    build_quad_pattern(rpm);
    if (g_a_chan != nullptr) {
        rmt_disable(g_a_chan);
        rmt_disable(g_b_chan);
        rmt_encoder_reset(g_a_enc);
        rmt_encoder_reset(g_b_enc);
        rmt_enable(g_a_chan);
        rmt_enable(g_b_chan);
        transmit_ab();
    }
    g_enc_rpm_active = rpm;
    cmp_timer_restart(rpm);
}

static void enc_init() {
    auto make_ch = [](gpio_num_t gpio, rmt_channel_handle_t* ch,
                      rmt_encoder_handle_t* enc) {
        rmt_tx_channel_config_t cfg = {};
        cfg.gpio_num          = gpio;
        cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
        cfg.resolution_hz     = kRmtResHz;
        cfg.mem_block_symbols = 64;
        cfg.trans_queue_depth = 4;
        ESP_ERROR_CHECK(rmt_new_tx_channel(&cfg, ch));
        rmt_copy_encoder_config_t ec = {};
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&ec, enc));
    };
    make_ch(ENC_A_GPIO, &g_a_chan, &g_a_enc);
    make_ch(ENC_B_GPIO, &g_b_chan, &g_b_enc);

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

    build_quad_pattern(g_sim.rpm);
    rmt_enable(g_a_chan);
    rmt_enable(g_b_chan);
    transmit_ab();
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
    if (g_a_chan != nullptr) enc_set_rpm(g_sim.rpm);
    Serial.printf("  [ENC-STIM] Preset: %s\n", label);
}

static void preset_idle() {
    apply_preset({ 700, 35, 3, 90, 25, 0, 35, 20, 3, 0 }, "IDLE");
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
    const uint16_t ticks = phase_ticks_for_rpm(s.rpm);
    const float f_a_hz = (s.rpm / 60.0f) * 4096.0f;  // freq eléctrica de A
    Serial.println();
    Serial.println("  ╔═══════════════════════════════════════════════════════════╗");
    Serial.println("  ║  OpenEMS Encoder Stim — Estado                            ║");
    Serial.println("  ╚═══════════════════════════════════════════════════════════╝");
    Serial.printf("  RPM:        %lu\n", (unsigned long)s.rpm);
    Serial.printf("  A freq:     %.1f Hz  (expect ~4096×RPM/60)\n", f_a_hz);
    Serial.printf("  phase_tick: %u (0.1 µs units @ 10 MHz RMT)\n", ticks);
    Serial.printf("  CMP_TOOTH:  %u / 16383\n", s.cmp_tooth);
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
    Serial.println("  CMP_TOOTH <0-16383>   offset do pulso CMP no ciclo 720°");
    Serial.println("  IDLE CRANK CRUISE WOT COAST");
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
        enc_set_rpm(g_sim.rpm);
        Serial.printf("  [ENC-STIM] RPM=%lu\n", (unsigned long)g_sim.rpm);
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

    g_sim = { 700, 35, 3, 90, 25, 0, 35, 20, 3, 8192 };  // CMP mid 2ª volta
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
    delay(1);
}
