/*
 * esp32_stimulator.ino — Engine Sensor Stimulator para bancada OpenEMS
 * ═══════════════════════════════════════════════════════════════════════
 * Gera todos os sinais de entrada do STM32H562: CKP/CMP digitais e 10
 * canais analógicos de sensor (MAP, TPS, CLT, IAT, APP1/2, FUEL, OIL,
 * ETB_TPS1/2).
 *
 * Plataforma: ESP32 (Arduino core ≥ 2.0, ESP-IDF ≥ 4.4)
 *
 * ── Ligações ao STM32H562 ────────────────────────────────────────────────
 *
 *  GPIO  →  STM32   Sinal       Interface
 *  ────────────────────────────────────────────────────────────────────
 *  GPIO2  → PA0     CKP 60-2    Directo (3.3V logic)
 *  GPIO4  → PA1     CMP         Directo (1 pulso/ciclo)
 *  GPIO25 → PA2     MAP         DAC1 analógico — sem filtro externo
 *  GPIO26 → PA4     TPS         DAC2 analógico — sem filtro externo
 *  GPIO13 → PC2     CLT         LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GPIO12 → PC3     IAT         LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GPIO14 → PB0     APP1        LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GPIO27 → PB1     APP2        LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GPIO16 → PC4     FUEL_PRESS  LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GPIO17 → PC5     OIL_PRESS   LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GPIO18 → PC0     ETB_TPS1    LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GPIO19 → PC1     ETB_TPS2    LEDC PWM → R=10 kΩ, C=100 nF → ADC
 *  GND    — GND     referência  OBRIGATÓRIO
 *
 *  Nota sobre CLT/IAT: se a PCB tiver pull-up para 3.3 V nestes pinos,
 *  remover o resistor (DNP) ou interpor um buffer rail-to-rail (e.g.
 *  LM358/TLV9001) para evitar conflito de tensão.
 *
 * ── Calibração (mirrors exactos do firmware sensors.cpp) ─────────────────
 *
 *  MAP        raw = kPa × 4095 / 300          (linear 0-300 kPa → 0-3.3 V)
 *  TPS / APP  raw = 200 + pct/100 × 3695      (range calibrado [200..3895])
 *  CLT / IAT  tabela linear por defeito -40..+150 °C, 128 entradas
 *  FUEL / OIL raw = bar×1000 × 4095 / 2500    (linear)
 *  ETB_TPS1   = TPS directo;  ETB_TPS2 = TPS invertido (100-pct)
 *
 * ── Comandos série (115200 baud) ─────────────────────────────────────────
 *
 *  Parâmetro único:
 *    RPM <n>    RPM (50-6000)
 *    MAP <n>    MAP em kPa (0-300)
 *    TPS <n>    TPS em % (0-100)
 *    CLT <n>    Temperatura refrigerante °C (-40 a +150)
 *    IAT <n>    Temperatura ar admissão °C (-40 a +150)
 *    APP <n>    Pedal acelerador % (0-100, aplica a APP1 e APP2)
 *    FUEL <n>   Pressão combustível bar×10 (ex: 35 = 3.5 bar)
 *    OIL <n>    Pressão óleo bar×10
 *    ETB <n>    Posição borboleta ETB % (0-100)
 *
 *  Presets:
 *    IDLE       Ralenti   (700 RPM, MAP=35 kPa, TPS=3%,  CLT=90°C)
 *    CRANK      Arranque  (200 RPM, MAP=101 kPa, TPS=0%, CLT=20°C)
 *    CRUISE     Cruzeiro  (2000 RPM, MAP=55 kPa, TPS=20%, CLT=90°C)
 *    WOT        Fundo     (4000 RPM, MAP=100 kPa, TPS=100%, CLT=90°C)
 *    COAST      Overrun   (2000 RPM, MAP=25 kPa, TPS=0%, CLT=90°C)
 *
 *  Outros:
 *    STATUS     Imprimir estado + tensões calculadas
 *    ?          Esta ajuda
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/ledc.h"
#include <cstring>
#include <cstdlib>

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 1 — Definição de pinos
// ═══════════════════════════════════════════════════════════════════════════

#define CKP_GPIO  ((gpio_num_t)2)
#define CMP_GPIO  ((gpio_num_t)4)

// DAC interno (8-bit, 0-3.3 V, sem filtro RC externo necessário):
// GPIO25 = DAC_CHANNEL_1 → MAP (PA2/ADC1_IN3)
// GPIO26 = DAC_CHANNEL_2 → TPS (PA4/ADC1_IN5)

// LEDC PWM 12-bit @ 39 kHz + filtro RC 10 kΩ / 100 nF (fc ≈ 159 Hz):
struct PwmChan {
    gpio_num_t      gpio;
    ledc_channel_t  ch;
    ledc_timer_t    timer;
    const char      name[10];
    const char      stm32[5];
};

static const PwmChan kPwm[] = {
    { GPIO_NUM_13, LEDC_CHANNEL_0, LEDC_TIMER_0, "CLT",      "PC2"  },
    { GPIO_NUM_12, LEDC_CHANNEL_1, LEDC_TIMER_0, "IAT",      "PC3"  },
    { GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_TIMER_0, "APP1",     "PB0"  },
    { GPIO_NUM_27, LEDC_CHANNEL_3, LEDC_TIMER_0, "APP2",     "PB1"  },
    { GPIO_NUM_16, LEDC_CHANNEL_4, LEDC_TIMER_1, "FUEL",     "PC4"  },
    { GPIO_NUM_17, LEDC_CHANNEL_5, LEDC_TIMER_1, "OIL",      "PC5"  },
    { GPIO_NUM_18, LEDC_CHANNEL_6, LEDC_TIMER_1, "ETB_TPS1", "PC0"  },
    { GPIO_NUM_19, LEDC_CHANNEL_7, LEDC_TIMER_1, "ETB_TPS2", "PC1"  },
};
static constexpr int kNPwm = (int)(sizeof(kPwm) / sizeof(kPwm[0]));

// índices semânticos no array kPwm:
enum PwmIdx : int { kCLT=0, kIAT, kAPP1, kAPP2, kFUEL, kOIL, kETB1, kETB2 };

// resolução LEDC: 12-bit (raw 0-4095 mapeado directamente como duty)
static constexpr ledc_timer_bit_t kPwmBits  = LEDC_TIMER_12_BIT;
// 12 bits @ 80 MHz APB: freq máx = 80e6/4096 ≈ 19.5 kHz (39 kHz aborta no IDF5).
// A 19 kHz o RC 10k+100n (fc≈159 Hz) ainda atenua ~41 dB — ripple < 3 mV.
static constexpr uint32_t         kPwmFreq  = 19000u;  // Hz

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 2 — Estado da simulação
// ═══════════════════════════════════════════════════════════════════════════

struct SimState {
    uint32_t rpm;           // rotações por minuto
    uint16_t map_kpa;       // pressão absoluta admissão [kPa]
    uint8_t  tps_pct;       // posição borboleta cabo legado [0-100%]
    int16_t  clt_degc;      // temperatura refrigerante [°C]
    int16_t  iat_degc;      // temperatura ar admissão [°C]
    uint8_t  app_pct;       // pedal acelerador [0-100%]
    uint16_t fuel_bar_x10;  // pressão combustível [bar × 10]
    uint16_t oil_bar_x10;   // pressão óleo [bar × 10]
    uint8_t  etb_pct;       // posição borboleta ETB [0-100%]
};

static SimState g_sim;

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 3 — Conversões (mirrors exactos de sensors.cpp)
// ═══════════════════════════════════════════════════════════════════════════

// MAP: raw = kPa * 4095 / 300
static uint16_t map_kpa_to_raw(uint16_t kpa) {
    if (kpa > 300u) kpa = 300u;
    return (uint16_t)((uint32_t)kpa * 4095u / 300u);
}

// TPS / APP: raw = 200 + pct/100 * 3695   (range [200..3895])
static uint16_t tps_pct_to_raw(uint8_t pct) {
    if (pct > 100u) pct = 100u;
    return (uint16_t)(200u + (uint32_t)pct * 3695u / 100u);
}

// CLT / IAT — tabela linear por defeito do firmware (init_tables em sensors.cpp):
//   index = (T×10 + 400) * 127 / 1900
//   raw   = index * 32 + 16   (midpoint do bloco de 32 contagens)
static uint16_t temp_to_raw(int16_t degc) {
    int32_t tx10 = (int32_t)degc * 10;
    if (tx10 < -400) tx10 = -400;
    if (tx10 >  1500) tx10 = 1500;
    int32_t idx = ((tx10 + 400) * 127) / 1900;
    if (idx < 0)   idx = 0;
    if (idx > 127) idx = 127;
    return (uint16_t)((uint32_t)idx * 32u + 16u);
}

// FUEL / OIL: raw = bar×1000 * 4095 / 2500
static uint16_t press_bar_x10_to_raw(uint16_t bar_x10) {
    uint32_t raw = (uint32_t)bar_x10 * 100u * 4095u / 2500u;
    if (raw > 4095u) raw = 4095u;
    return (uint16_t)raw;
}

// raw [0..4095] → DAC 8-bit [0..255]  (divide por 16 = >> 4)
static uint8_t raw_to_dac8(uint16_t raw) {
    return (uint8_t)(raw >> 4u);
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 4 — CKP / CMP Generator (timer ISR)
// ═══════════════════════════════════════════════════════════════════════════

static constexpr int      kRealTeeth = 58;
static constexpr int      kCmpTooth  = 5;     // dente onde sobe o CMP (rev 0)
static constexpr uint32_t kRpmMin    = 50u;
static constexpr uint32_t kRpmMax    = 6000u;

static volatile int      g_tooth      = 0;
static volatile bool     g_ckp_high   = false;
static volatile int      g_revolution = 0;
static volatile uint32_t g_rev_count  = 0;

// Timers de hardware (GPTimer @ 1 MHz, one-shot rearmado na ISR).
// Core 3.x (IDF 5) removeu ESP_TIMER_ISR do esp_timer; o GPTimer dá dispatch
// em ISR real (IRAM) com jitter menor — necessário a 6000 RPM (½ dente = 83 µs).
static hw_timer_t* g_ckp_timer   = nullptr;
static hw_timer_t* g_cmp_off_tmr = nullptr;

static inline void IRAM_ATTR timer_arm_once(hw_timer_t* t, uint64_t us) {
    timerWrite(t, 0);
    timerAlarm(t, us, false, 0);
}

static void ARDUINO_ISR_ATTR cmp_off_isr() {
    gpio_set_level(CMP_GPIO, 0);
}

static void ARDUINO_ISR_ATTR ckp_isr() {
    const uint32_t rpm  = g_sim.rpm;
    const uint32_t T_us = 60000000UL / (rpm * 60UL);   // período de 1 dente [µs]

    if (!g_ckp_high) {
        // ── Rising edge ───────────────────────────────────────────────────
        gpio_set_level(CKP_GPIO, 1);
        g_ckp_high = true;

        if (g_revolution == 0 && g_tooth == kCmpTooth) {
            gpio_set_level(CMP_GPIO, 1);
            timer_arm_once(g_cmp_off_tmr, (uint64_t)T_us / 2u);
        }
        timer_arm_once(g_ckp_timer, (uint64_t)T_us / 2u);

    } else {
        // ── Falling edge ──────────────────────────────────────────────────
        gpio_set_level(CKP_GPIO, 0);
        g_ckp_high = false;
        g_tooth++;

        uint64_t low_us;
        if (g_tooth >= kRealTeeth) {
            // gap de 2 dentes: LOW extra de 2 × T
            low_us  = (uint64_t)T_us / 2u + (uint64_t)T_us * 2u;
            g_tooth = 0;
            g_revolution ^= 1;
            g_rev_count++;
        } else {
            low_us = (uint64_t)T_us / 2u;
        }
        timer_arm_once(g_ckp_timer, low_us);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 5 — Actualização dos sensores analógicos
// ═══════════════════════════════════════════════════════════════════════════

static void set_ledc(PwmIdx idx, uint16_t raw) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, kPwm[idx].ch, (uint32_t)raw);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, kPwm[idx].ch);
}

static void update_analog(const SimState& s) {
    // MAP → DAC1 (GPIO25)
    dac_output_voltage(DAC_CHANNEL_1, raw_to_dac8(map_kpa_to_raw(s.map_kpa)));

    // TPS → DAC2 (GPIO26)
    dac_output_voltage(DAC_CHANNEL_2, raw_to_dac8(tps_pct_to_raw(s.tps_pct)));

    // CLT / IAT
    set_ledc(kCLT,  temp_to_raw(s.clt_degc));
    set_ledc(kIAT,  temp_to_raw(s.iat_degc));

    // APP1 / APP2 (espelho)
    const uint16_t app_raw = tps_pct_to_raw(s.app_pct);
    set_ledc(kAPP1, app_raw);
    set_ledc(kAPP2, app_raw);

    // FUEL / OIL
    set_ledc(kFUEL, press_bar_x10_to_raw(s.fuel_bar_x10));
    set_ledc(kOIL,  press_bar_x10_to_raw(s.oil_bar_x10));

    // ETB_TPS1 (directo) + ETB_TPS2 (invertido — redundância anti-phase)
    set_ledc(kETB1, tps_pct_to_raw(s.etb_pct));
    set_ledc(kETB2, tps_pct_to_raw((uint8_t)(100u - s.etb_pct)));
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 6 — Presets de cenário
// ═══════════════════════════════════════════════════════════════════════════

static void apply_preset(const SimState& p, const char* label) {
    g_sim = p;
    update_analog(g_sim);
    Serial.printf("  [STIM] Preset: %s\n", label);
}

static void preset_idle()   { apply_preset({  700,  35,   3,  90,  25,   0, 35, 20,   3 }, "IDLE");   }
static void preset_crank()  { apply_preset({  200, 101,   0,  20,  15,   0, 20,  5,   0 }, "CRANK");  }
static void preset_cruise() { apply_preset({ 2000,  55,  20,  90,  35,  20, 35, 30,  20 }, "CRUISE"); }
static void preset_wot()    { apply_preset({ 4000, 100, 100,  90,  40, 100, 38, 40, 100 }, "WOT");    }
static void preset_coast()  { apply_preset({ 2000,  25,   0,  90,  35,   0, 35, 25,   0 }, "COAST");  }

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 7 — Interface série
// ═══════════════════════════════════════════════════════════════════════════

static void print_status() {
    const SimState& s = g_sim;
    Serial.println();
    Serial.println("  ╔═══════════════════════════════════════════════════════════╗");
    Serial.println("  ║  OpenEMS Stimulator — Estado Actual                       ║");
    Serial.println("  ╚═══════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.printf("  %-10s %6lu\n",          "RPM:",       (unsigned long)s.rpm);
    Serial.println();

    auto pline = [](const char* lbl, const char* stm32, uint16_t raw, float v) {
        Serial.printf("  %-10s %-4s  raw=%4u  V=%.3f V\n", lbl, stm32, raw, v);
    };
    auto raw_v  = [](uint16_t r) -> float { return r * 3.3f / 4095.0f; };

    uint16_t r;
    r = map_kpa_to_raw(s.map_kpa);
    Serial.printf("  %-10s %-4s  %3u kPa   raw=%4u  V=%.3f V\n",
                  "MAP:", "PA2", s.map_kpa, r, raw_v(r));

    r = tps_pct_to_raw(s.tps_pct);
    Serial.printf("  %-10s %-4s  %3u %%     raw=%4u  V=%.3f V\n",
                  "TPS:", "PA4", s.tps_pct, r, raw_v(r));

    r = temp_to_raw(s.clt_degc);
    Serial.printf("  %-10s %-4s  %3d °C    raw=%4u  V=%.3f V\n",
                  "CLT:", "PC2", s.clt_degc, r, raw_v(r));

    r = temp_to_raw(s.iat_degc);
    Serial.printf("  %-10s %-4s  %3d °C    raw=%4u  V=%.3f V\n",
                  "IAT:", "PC3", s.iat_degc, r, raw_v(r));

    r = tps_pct_to_raw(s.app_pct);
    Serial.printf("  %-10s %-4s  %3u %%     raw=%4u  V=%.3f V\n",
                  "APP1/2:", "PB0/1", s.app_pct, r, raw_v(r));

    r = press_bar_x10_to_raw(s.fuel_bar_x10);
    Serial.printf("  %-10s %-4s  %.1f bar   raw=%4u  V=%.3f V\n",
                  "FUEL:", "PC4", s.fuel_bar_x10 / 10.0f, r, raw_v(r));

    r = press_bar_x10_to_raw(s.oil_bar_x10);
    Serial.printf("  %-10s %-4s  %.1f bar   raw=%4u  V=%.3f V\n",
                  "OIL:", "PC5", s.oil_bar_x10 / 10.0f, r, raw_v(r));

    r = tps_pct_to_raw(s.etb_pct);
    Serial.printf("  %-10s %-4s  %3u %%     raw=%4u  (TPS2 invertido)\n",
                  "ETB:", "PC0/1", s.etb_pct, r);

    Serial.println();
}

static void print_help() {
    Serial.println();
    Serial.println("  ╔══════════════════════════════════════════════════════════╗");
    Serial.println("  ║  OpenEMS ESP32 Sensor Stimulator                         ║");
    Serial.println("  ╚══════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("  Parâmetros (ex: RPM 1500):");
    Serial.println("    RPM <n>    50-6000");
    Serial.println("    MAP <n>    kPa (0-300)");
    Serial.println("    TPS <n>    % (0-100)");
    Serial.println("    CLT <n>    °C (-40 a +150)");
    Serial.println("    IAT <n>    °C (-40 a +150)");
    Serial.println("    APP <n>    % (0-100) — aplica a APP1 e APP2");
    Serial.println("    FUEL <n>   bar×10  (ex: 35 = 3.5 bar)");
    Serial.println("    OIL <n>    bar×10");
    Serial.println("    ETB <n>    % (0-100) — TPS2 = 100-n (anti-phase)");
    Serial.println();
    Serial.println("  Presets:");
    Serial.println("    IDLE    700 RPM  MAP=35   TPS=3   CLT=90");
    Serial.println("    CRANK   200 RPM  MAP=101  TPS=0   CLT=20");
    Serial.println("    CRUISE 2000 RPM  MAP=55   TPS=20  CLT=90");
    Serial.println("    WOT    4000 RPM  MAP=100  TPS=100 CLT=90");
    Serial.println("    COAST  2000 RPM  MAP=25   TPS=0   CLT=90");
    Serial.println();
    Serial.println("  STATUS / ?");
    Serial.println();
    Serial.println("  Pinos:");
    Serial.printf("    GPIO%-2d → PA0  CKP 60-2\n",  (int)CKP_GPIO);
    Serial.printf("    GPIO%-2d → PA1  CMP\n",        (int)CMP_GPIO);
    Serial.printf("    GPIO25 → PA2  MAP  (DAC1)\n");
    Serial.printf("    GPIO26 → PA4  TPS  (DAC2)\n");
    for (int i = 0; i < kNPwm; ++i) {
        Serial.printf("    GPIO%-2d → %s  %-9s (LEDC PWM + RC 10kΩ/100nF)\n",
                      (int)kPwm[i].gpio, kPwm[i].stm32, kPwm[i].name);
    }
    Serial.println();
}

// acumulador de linha série
static char g_line[64];
static int  g_line_pos = 0;

static void parse_cmd(const char* raw) {
    // cópia local para toupper sem alterar original
    char buf[64];
    int  len = 0;
    while (raw[len] && len < 63) { buf[len] = raw[len]; len++; }
    buf[len] = '\0';
    for (int i = 0; i < len; ++i)
        if (buf[i] >= 'a' && buf[i] <= 'z') buf[i] -= 32;

    char cmd[16] = {};
    int  val = 0;
    const bool has_val = (sscanf(buf, "%15s %d", cmd, &val) >= 2);

    bool changed = true;

    if      (strcmp(cmd, "RPM")    == 0 && has_val) {
        g_sim.rpm = (uint32_t)constrain(val, (int)kRpmMin, (int)kRpmMax);
        Serial.printf("  [STIM] RPM=%lu\n", (unsigned long)g_sim.rpm);
    }
    else if (strcmp(cmd, "MAP")    == 0 && has_val) {
        g_sim.map_kpa = (uint16_t)constrain(val, 0, 300);
        Serial.printf("  [STIM] MAP=%u kPa\n", g_sim.map_kpa);
    }
    else if (strcmp(cmd, "TPS")    == 0 && has_val) {
        g_sim.tps_pct = (uint8_t)constrain(val, 0, 100);
        Serial.printf("  [STIM] TPS=%u%%\n", g_sim.tps_pct);
    }
    else if (strcmp(cmd, "CLT")    == 0 && has_val) {
        g_sim.clt_degc = (int16_t)constrain(val, -40, 150);
        Serial.printf("  [STIM] CLT=%d°C\n", g_sim.clt_degc);
    }
    else if (strcmp(cmd, "IAT")    == 0 && has_val) {
        g_sim.iat_degc = (int16_t)constrain(val, -40, 150);
        Serial.printf("  [STIM] IAT=%d°C\n", g_sim.iat_degc);
    }
    else if (strcmp(cmd, "APP")    == 0 && has_val) {
        g_sim.app_pct = (uint8_t)constrain(val, 0, 100);
        Serial.printf("  [STIM] APP=%u%%\n", g_sim.app_pct);
    }
    else if (strcmp(cmd, "FUEL")   == 0 && has_val) {
        g_sim.fuel_bar_x10 = (uint16_t)constrain(val, 0, 50);
        Serial.printf("  [STIM] FUEL=%.1f bar\n", g_sim.fuel_bar_x10 / 10.0f);
    }
    else if (strcmp(cmd, "OIL")    == 0 && has_val) {
        g_sim.oil_bar_x10  = (uint16_t)constrain(val, 0, 50);
        Serial.printf("  [STIM] OIL=%.1f bar\n",  g_sim.oil_bar_x10  / 10.0f);
    }
    else if (strcmp(cmd, "ETB")    == 0 && has_val) {
        g_sim.etb_pct = (uint8_t)constrain(val, 0, 100);
        Serial.printf("  [STIM] ETB=%u%%\n", g_sim.etb_pct);
    }
    else if (strcmp(cmd, "IDLE")   == 0) { preset_idle();   }
    else if (strcmp(cmd, "CRANK")  == 0) { preset_crank();  }
    else if (strcmp(cmd, "CRUISE") == 0) { preset_cruise(); }
    else if (strcmp(cmd, "WOT")    == 0) { preset_wot();    }
    else if (strcmp(cmd, "COAST")  == 0) { preset_coast();  }
    else if (strcmp(cmd, "STATUS") == 0) { print_status(); changed = false; }
    else if (strcmp(cmd, "?")      == 0) { print_help();   changed = false; }
    else { Serial.printf("  [STIM] Comando desconhecido: %s\n", buf); changed = false; }

    if (changed) update_analog(g_sim);
}

// ═══════════════════════════════════════════════════════════════════════════
// ── setup() e loop()
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(300);

    // ── GPIOs digitais de saída ───────────────────────────────────────────
    for (gpio_num_t g : { CKP_GPIO, CMP_GPIO }) {
        gpio_reset_pin(g);
        gpio_set_direction(g, GPIO_MODE_OUTPUT);
        gpio_set_level(g, 0);
    }

    // ── DAC (MAP + TPS) ───────────────────────────────────────────────────
    dac_output_enable(DAC_CHANNEL_1);   // GPIO25 → MAP (PA2)
    dac_output_enable(DAC_CHANNEL_2);   // GPIO26 → TPS (PA4)

    // ── LEDC timer 0: CLT, IAT, APP1, APP2 ───────────────────────────────
    {
        ledc_timer_config_t t;
        memset(&t, 0, sizeof(t));
        t.speed_mode      = LEDC_HIGH_SPEED_MODE;
        t.duty_resolution = kPwmBits;
        t.timer_num       = LEDC_TIMER_0;
        t.freq_hz         = kPwmFreq;
        t.clk_cfg         = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));
    }

    // ── LEDC timer 1: FUEL, OIL, ETB_TPS1, ETB_TPS2 ─────────────────────
    {
        ledc_timer_config_t t;
        memset(&t, 0, sizeof(t));
        t.speed_mode      = LEDC_HIGH_SPEED_MODE;
        t.duty_resolution = kPwmBits;
        t.timer_num       = LEDC_TIMER_1;
        t.freq_hz         = kPwmFreq;
        t.clk_cfg         = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));
    }

    // ── LEDC canais ───────────────────────────────────────────────────────
    for (int i = 0; i < kNPwm; ++i) {
        ledc_channel_config_t cc;
        memset(&cc, 0, sizeof(cc));
        cc.gpio_num   = (int)kPwm[i].gpio;
        cc.speed_mode = LEDC_HIGH_SPEED_MODE;
        cc.channel    = kPwm[i].ch;
        cc.intr_type  = LEDC_INTR_DISABLE;
        cc.timer_sel  = kPwm[i].timer;
        cc.duty       = 0u;
        cc.hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&cc));
    }

    // ── Timers CKP / CMP — GPTimer @ 1 MHz (1 tick = 1 µs) ──────────────
    g_ckp_timer = timerBegin(1000000u);
    timerAttachInterrupt(g_ckp_timer, &ckp_isr);
    g_cmp_off_tmr = timerBegin(1000000u);
    timerAttachInterrupt(g_cmp_off_tmr, &cmp_off_isr);

    // ── Preset inicial + arrancar CKP ─────────────────────────────────────
    preset_idle();
    update_analog(g_sim);

    const uint32_t T0 = 60000000UL / (g_sim.rpm * 60UL);
    timer_arm_once(g_ckp_timer, T0 / 2u);

    print_help();
    print_status();
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n' || g_line_pos >= (int)(sizeof(g_line) - 1)) {
            g_line[g_line_pos] = '\0';
            if (g_line_pos > 0) parse_cmd(g_line);
            g_line_pos = 0;
        } else {
            g_line[g_line_pos++] = c;
        }
    }
    delay(5);
}
