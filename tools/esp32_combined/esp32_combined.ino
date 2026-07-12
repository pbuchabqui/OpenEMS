/*
 * esp32_combined.ino — CKP Generator + Logic Scope para bancada OpenEMS
 * ═══════════════════════════════════════════════════════════════════════
 * Um único ESP32: gera CKP/CMP via RMT (hardware, jitter ~zero) + scope lógico
 * de 10 canais para IGN/INJ. CKP loopback para scope via GPIO34 (wire interno).
 *
 * Plataforma : ESP32 (Arduino core ≥ 2.0, ESP-IDF ≥ 4.4)
 * Resolução  : 1 µs  (esp_timer_get_time)
 *
 * ── Ligações ao STM32H562 ───────────────────────────────────────────────────
 *
 *   ESP32 GPIO  STM32      Função
 *   ──────────  ─────────  ────────────────────
 *   GPIO 2  →  PA0        CKP output (60-2, RMT hardware)
 *   GPIO 4  →  PA1        CMP output (1 pulso/720°, RMT hardware)
 *   GPIO 2  →  GPIO34     Wire loopback p/ scope CKP (mesma placa)
 *   GPIO 32 ←  PE9        IGN1 (TIM1_CH1)
 *   GPIO 33 ←  PE11       IGN2 (TIM1_CH2)
 *   GPIO 26 →  PA3        MAP  (DAC2 real, 8-bit, ligação DIRETA sem RC)
 *   GPIO 25 →  PA4        TPS  (DAC1 real — reteste; cmd DAC25 <0-255>)
 *   GPIO 27 ←  PC6        INJ1 (TIM3_CH1)
 *   GPIO 14 ←  PC7        INJ2 (TIM3_CH2)
 *   GPIO 12 ←  PC8        INJ3 (TIM3_CH3)
 *   GPIO 13 ←  PC9        INJ4 (TIM3_CH4)
 *   GND     —  GND        OBRIGATÓRIO
 *
 * ── Comandos série (115200 baud) ────────────────────────────────────────────
 *
 *   CKP generator:
 *     +  / -   RPM ± 50
 *     RPM <n>  RPM exacto (50-9000)
 *     MAP <kpa> / CLT <c> / TPS <pct> / IAT <c>  (sensores)
 *     IDLE / CRANK / CRUISE / WOT / COAST  (presets)
 *     STATUS   estado completo
 *
 *   Scope:
 *     l        Live table (actualiza a cada 1 s)  [default]
 *     e        Edge log (cada bordo)
 *     p        Pulse log (cada pulso completo)
 *     w        Waveform bar (últimos 300 ms)
 *     t        Timing analysis: sequência IGN + ângulo de avanço
 *     d        Dashboard 720° (diagrama ASCII, régua de ângulo)
 *     s        Estatísticas por canal
 *     r        Reset estatísticas
 *     ?        Esta ajuda
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include <cstring>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 1 — CKP/CMP Generator (RMT hardware)
// ═══════════════════════════════════════════════════════════════════════════

#define CKP_GPIO    ((gpio_num_t)2)
#define CMP_GPIO    ((gpio_num_t)4)
#define CKP_LB_GPIO ((gpio_num_t)34)  // loopback p/ scope CKP
#ifndef LED_BUILTIN
#define LED_GPIO    ((gpio_num_t)GPIO_NUM_MAX)
#else
#define LED_GPIO    ((gpio_num_t)LED_BUILTIN)
#endif

static constexpr uint32_t kRpmInit   = 500;
static constexpr int      kCmpTooth  = 5;
static constexpr int      kRealTeeth = 58;
static constexpr uint32_t kRpmMin    = 50;
static constexpr uint32_t kRpmMax    = 9000;
static constexpr uint32_t kRmtResHz  = 1000000u;  // 1 tick RMT = 1 µs
static constexpr uint16_t kRmtMaxDur = 32767u;
static constexpr int kCmpSymCount = kRealTeeth * 2;  // 116 symbols = 720°

static const uint32_t kRpmPresets[] = {
    100, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000
};

static volatile uint32_t g_rpm       = kRpmInit;
static volatile uint32_t g_rev_count = 0;
static volatile uint32_t g_cmp_count = 0;

// RMT handles (ESP-IDF 4.x API)
static rmt_channel_t g_ckp_chan = RMT_CHANNEL_0;
static rmt_item32_t  g_ckp_sym[kRealTeeth];
static rmt_channel_t g_cmp_chan = RMT_CHANNEL_1;
static rmt_item32_t  g_cmp_sym[kCmpSymCount];
static volatile uint32_t    g_ckp_rpm_active = 0u;

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
// CH8 (CKP) em GPIO34: wire loopback interno GPIO2→GPIO34 para capturar o
// sinal real gerado pelo RMT (não virtual, timestamp real da borda).
// CH9 (CMP) é virtual: alimentado por software no momento do pulso CMP.

struct ChanDef {
    gpio_num_t  gpio;
    const char  name[6];
    const char  pin[6];
    bool        enabled;
};

static constexpr gpio_num_t kVirtual = GPIO_NUM_MAX;

static ChanDef kChan[] = {
    // IGN: GPIO assignments matching scope.ino corrected pin mapping
    { GPIO_NUM_32,  "IGN1", "PE9",  true  },   // CH0 — IGN1 (cyl 1)
    { GPIO_NUM_33,  "IGN2", "PE11", true  },   // CH1 — IGN2 (cyl 2)
    { GPIO_NUM_25,  "IGN3", "PE13", false },   // CH2 — disabled (GPIO25 = TPS DAC1)
    { GPIO_NUM_26,  "IGN4", "PE14", false },   // CH3 — disabled (GPIO26 = MAP DAC2)
    // INJ: GPIO assignments matching scope.ino corrected pin mapping
    { GPIO_NUM_27,  "INJ1", "PC6",  true  },   // CH4 — INJ1 (cyl 1)
    { GPIO_NUM_14,  "INJ2", "PC7",  true  },   // CH5 — INJ2 (cyl 2)
    { GPIO_NUM_12,  "INJ3", "PC8",  true  },   // CH6 — INJ3 (cyl 3)
    { GPIO_NUM_13,  "INJ4", "PC9",  true  },   // CH7 — INJ4 (cyl 4)
    // CKP/CMP: CKP via GPIO34 loopback, CMP virtual
    { CKP_LB_GPIO,  "CKP",  "PA0",  true  },   // CH8 — loopback real do RMT
    { kVirtual,     "CMP",  "PA1",  true  },   // CH9 — virtual (software)
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
// ── SECÇÃO 3 — RMT CKP/CMP Generator (hardware, jitter ~zero)
// ═══════════════════════════════════════════════════════════════════════════

static void build_ckp_pattern(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    const uint32_t T = 60000000UL / (rpm * 60UL);
    const uint16_t h = (uint16_t)(T / 2u);
    const uint16_t l = (uint16_t)(T - (T / 2u));
    uint32_t gap_low = (uint32_t)l + 2u * T;
    if (gap_low > kRmtMaxDur) gap_low = kRmtMaxDur;
    for (int i = 0; i < kRealTeeth - 1; i++) {
        g_ckp_sym[i].level0 = 1; g_ckp_sym[i].duration0 = h;
        g_ckp_sym[i].level1 = 0; g_ckp_sym[i].duration1 = l;
    }
    g_ckp_sym[kRealTeeth - 1].level0 = 1; g_ckp_sym[kRealTeeth - 1].duration0 = h;
    g_ckp_sym[kRealTeeth - 1].level1 = 0; g_ckp_sym[kRealTeeth - 1].duration1 = (uint16_t)gap_low;
}

static void build_cmp_pattern(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    const uint32_t T = 60000000UL / (rpm * 60UL);
    const uint16_t h = (uint16_t)(T / 2u);
    const uint16_t l = (uint16_t)(T - (T / 2u));
    uint32_t gap_low = (uint32_t)l + 2u * T;
    if (gap_low > kRmtMaxDur) gap_low = kRmtMaxDur;
    for (int rev = 0; rev < 2; rev++) {
        for (int i = 0; i < kRealTeeth; i++) {
            const int idx = rev * kRealTeeth + i;
            const bool is_gap_tooth = (i == kRealTeeth - 1);
            const bool is_cmp_pulse = (rev == 0 && i == kCmpTooth);
            g_cmp_sym[idx].level0 = is_cmp_pulse ? 1 : 0;
            g_cmp_sym[idx].duration0 = h;
            g_cmp_sym[idx].level1 = 0;
            g_cmp_sym[idx].duration1 = is_gap_tooth ? (uint16_t)gap_low : l;
        }
    }
    // Dessincronizar CMP de CKP em 1 tick RMT (1µs) para evitar race no STM32:
    // CKP e CMP partilham o mesmo clock RMT — as bordas podem coincidir dentro
    // da janela LDR-STR do TIM5_SR, aniquilando CC2IF. Um offset de 1µs garante
    // que as bordas CMP chegam sempre depois das CKP.
    g_cmp_sym[0].duration0 = h + 1u;
}

// Arranque SIMULTÂNEO dos loops CKP/CMP. Sem isto o offset CKP↔CMP era o
// tempo de software entre os dois rmt_write_items, re-sorteado a cada mudança
// de RPM: um atraso fixo em tempo vira deslocamento angular ∝ RPM
// (δ×RPM×0.006°/ms) e com offset >360° o pulso CMP cai na revolução errada
// (fase invertida no decoder do STM32).
// O ESP32 original NÃO tem tx_sim (SOC_RMT_SUPPORT_TX_SYNCHRO só em S2/S3/C3),
// então: preenche a RAM do RMT sem iniciar (rmt_fill_tx_items) e dispara os
// dois tx_start por registrador, back-to-back com IRQs desligadas — offset
// residual ~100ns, constante (0.01° @ 10000 RPM). Como o CMP loopa em
// exatamente 2× o período do CKP, a sincronia do arranque persiste para
// sempre; o único offset restante é o 1µs intencional do build_cmp_pattern
// (anti-race no TIM5 do STM32).
static void rmt_apply_tx_pair() {
    // Terminador (duração 0): em loop mode o RMT reinicia ao encontrá-lo.
    // rmt_write_items escrevia-o automaticamente; rmt_fill_tx_items NÃO —
    // sem ele o transmissor varre RAM suja além do padrão (timing corrompido).
    static const rmt_item32_t kEnd = {};
    rmt_tx_stop(g_ckp_chan);
    rmt_tx_stop(g_cmp_chan);
    rmt_fill_tx_items(g_ckp_chan, g_ckp_sym, kRealTeeth, 0);
    rmt_fill_tx_items(g_ckp_chan, &kEnd, 1, kRealTeeth);
    rmt_fill_tx_items(g_cmp_chan, g_cmp_sym, kCmpSymCount, 0);
    rmt_fill_tx_items(g_cmp_chan, &kEnd, 1, kCmpSymCount);
    rmt_set_tx_loop_mode(g_ckp_chan, true);
    rmt_set_tx_loop_mode(g_cmp_chan, true);
    portDISABLE_INTERRUPTS();
    RMT.conf_ch[g_ckp_chan].conf1.mem_rd_rst = 1;
    RMT.conf_ch[g_ckp_chan].conf1.mem_rd_rst = 0;
    RMT.conf_ch[g_cmp_chan].conf1.mem_rd_rst = 1;
    RMT.conf_ch[g_cmp_chan].conf1.mem_rd_rst = 0;
    RMT.conf_ch[g_ckp_chan].conf1.tx_start = 1;
    RMT.conf_ch[g_cmp_chan].conf1.tx_start = 1;
    portENABLE_INTERRUPTS();
}

static void ckp_set_rpm(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    if (rpm > kRpmMax) rpm = kRpmMax;
    if (rpm == g_ckp_rpm_active) return;
    build_ckp_pattern(rpm);
    build_cmp_pattern(rpm);
    rmt_apply_tx_pair();
    g_ckp_rpm_active = rpm;
    g_rpm = rpm;
    g_rev_count++;
}

// Init RMT channel (ESP-IDF 4.x API — clk_div 80 = 1µs/tick)
static void rmt_chan_init(rmt_channel_t ch, gpio_num_t gpio, int mem_blocks) {
    rmt_config_t cfg = {};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = ch;
    cfg.gpio_num = gpio;
    cfg.mem_block_num = mem_blocks;
    cfg.clk_div = 80;  // 80MHz / 80 = 1MHz = 1µs resolution
    cfg.tx_config.loop_en = 0;
    cfg.tx_config.carrier_en = 0;
    cfg.tx_config.idle_output_en = 1;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_config(&cfg);
    rmt_driver_install(cfg.channel, 0, 0);
}

static void rmt_ckp_init() {
    rmt_chan_init(g_ckp_chan, CKP_GPIO, 1);
    rmt_chan_init(g_cmp_chan, CMP_GPIO, 2);
    build_ckp_pattern(kRpmInit);
    build_cmp_pattern(kRpmInit);
    rmt_apply_tx_pair();
    g_ckp_rpm_active = kRpmInit;
    Serial.println("RMT OK (tx_sim)");
}

// ── ISR GPIO scope (IGN, INJ, CKP loopback) ──────────────────────────────

static void IRAM_ATTR edge_isr(void* arg) {
    const uint8_t ch  = (uint8_t)(uint32_t)arg;
    const int64_t ts  = esp_timer_get_time();
    const int     lvl = gpio_get_level(kChan[ch].gpio);
    buf_push(ts, ch, (uint8_t)lvl);
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 4 — Sensores analógicos (PWM + RC filter)
// ═══════════════════════════════════════════════════════════════════════════

#include "driver/ledc.h"

// Sensores lentos (CLT/IAT/APP/OIL/FUEL/ETB): LEDC PWM 12-bit @ 19kHz + RC.
// MAP/TPS: DAC REAL de 8 bits — sem RC, sem ripple (o ADC do STM32 lê nível
// DC direto). MAP no DAC2/GPIO26 (funcional); TPS no DAC1/GPIO25 em RETESTE
// (tido como danificado na era do stimulator antigo — comando DAC25 <0-255>
// permite medir o pino cru com multímetro; se confirmar morto, TPS volta a
// PWM no GPIO21, liberado pelo MAP).
// Cablagem: GPIO26→PA3(MAP, direto), GPIO25→PA4(TPS, direto).

struct SimState {
    uint32_t rpm;
    uint16_t map_kpa;       // kPa
    uint8_t  tps_pct;       // 0-100%
    int16_t  clt_degc;      // °C
    int16_t  iat_degc;      // °C
    uint8_t  app_pct;       // 0-100%
    uint16_t fuel_bar_x10;
    uint16_t oil_bar_x10;
    uint8_t  etb_pct;
};

static SimState g_sim;

// MAP: 0-300 kPa → DAC 8-bit (0-255). Resolução ~1.18 kPa/step.
// Calibração medida na bancada (2026-07-12, varredura 35-250 kPa lida na
// ECU): o DAC do ESP32 não alcança os rails (satura ~0.08-3.1V), dando
// lido = 0.931×alvo + 8.2 kPa. Compensação inversa: alvo_corr = (kpa-8.2)/0.931
// → em contas inteiras: dac = (kpa*1000 - 8200) * 255 / (931 * 300 / ... )
// simplificado: dac = ((kpa - 8.2) / 0.931) * 255/300 = (kpa*10 - 82) * 2550 / (9310*3)
static uint8_t map_to_dac(uint16_t kpa) {
    if (kpa > 300) kpa = 300;
    int32_t corr_x10 = ((int32_t)kpa * 10 - 82) * 1000 / 931;  // kPa×10 corrigido
    if (corr_x10 < 0) corr_x10 = 0;
    uint32_t dac = (uint32_t)corr_x10 * 255u / 3000u;
    return (uint8_t)(dac > 255u ? 255u : dac);
}
// TPS: 0-100% → DAC 8-bit com piso ~0.2V (15/255 ≈ 0.19V @ 3.3V).
// Mesma calibração do MAP (DAC1 mediu 1.65V p/ 1.75V nominal @ 50% —
// ganho ~0.931 + offset ~0.09V dos rails): dac_corr = (dac_nom - 7)/0.931.
static uint8_t tps_to_dac(uint8_t pct) {
    if (pct > 100) pct = 100;
    const uint32_t nom = 15u + (uint32_t)pct * 240u / 100u;
    int32_t corr = ((int32_t)nom * 1000 - 6950) / 931;
    if (corr < 0) corr = 0;
    return (uint8_t)(corr > 255 ? 255 : corr);
}

#include "driver/dac.h"
static constexpr dac_channel_t kMapDac = DAC_CHANNEL_2;  // GPIO26 → PA3
static constexpr dac_channel_t kTpsDac = DAC_CHANNEL_1;  // GPIO25 → PA4 (reteste)
static void dac_init_map_tps() {
    dac_output_enable(kMapDac);
    dac_output_enable(kTpsDac);
}

// LEDC PWM: 12-bit, 19 kHz. Timer 0 (ch0-6): CLT,IAT,APP,OIL,FUEL,ETB.
//             Timer 1 (ch0-1): MAP(GPIO25), TPS(GPIO26) — ex-DAC avariado.
static constexpr int      kPwmBits = 12;
static constexpr uint32_t kPwmFreq = 19000;

struct PwmChan { gpio_num_t gpio; ledc_channel_t ch; ledc_timer_t timer; const char* name; };
static PwmChan kPwm[] = {
    { GPIO_NUM_13, LEDC_CHANNEL_0, LEDC_TIMER_0, "CLT" },
    { GPIO_NUM_12, LEDC_CHANNEL_1, LEDC_TIMER_0, "IAT" },
    { GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_TIMER_0, "APP" },
    { GPIO_NUM_17, LEDC_CHANNEL_3, LEDC_TIMER_0, "OIL" },
    { GPIO_NUM_18, LEDC_CHANNEL_4, LEDC_TIMER_0, "FUEL" },
    { GPIO_NUM_16, LEDC_CHANNEL_5, LEDC_TIMER_0, "ETB1" },
    { GPIO_NUM_19, LEDC_CHANNEL_6, LEDC_TIMER_0, "ETB2" },
};
static constexpr int kNPwm = (int)(sizeof(kPwm) / sizeof(kPwm[0]));

static void pwm_init() {
    // Timer 0: CLT, IAT, APP, OIL, FUEL, ETB
    ledc_timer_config_t tcfg0 = {};
    tcfg0.speed_mode = LEDC_LOW_SPEED_MODE;
    tcfg0.timer_num  = LEDC_TIMER_0;
    tcfg0.duty_resolution = (ledc_timer_bit_t)kPwmBits;
    tcfg0.freq_hz    = kPwmFreq;
    tcfg0.clk_cfg    = LEDC_AUTO_CLK;
    ledc_timer_config(&tcfg0);
    // Timer 1: MAP (GPIO25), TPS (GPIO26)
    ledc_timer_config_t tcfg1 = {};
    tcfg1.speed_mode = LEDC_LOW_SPEED_MODE;
    tcfg1.timer_num  = LEDC_TIMER_1;
    tcfg1.duty_resolution = (ledc_timer_bit_t)kPwmBits;
    tcfg1.freq_hz    = kPwmFreq;
    tcfg1.clk_cfg    = LEDC_AUTO_CLK;
    ledc_timer_config(&tcfg1);
    for (int i = 0; i < kNPwm; ++i) {
        ledc_channel_config_t ccfg = {};
        ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
        ccfg.channel    = kPwm[i].ch;
        ccfg.timer_sel  = kPwm[i].timer;
        ccfg.intr_type  = LEDC_INTR_DISABLE;
        ccfg.gpio_num   = (int)kPwm[i].gpio;
        ccfg.duty       = 0;
        ccfg.hpoint     = 0;
        ledc_channel_config(&ccfg);
        gpio_set_pull_mode(kPwm[i].gpio, GPIO_FLOATING);
    }
}

static void pwm_write(int idx, uint32_t duty_12bit) {
    if (duty_12bit > 4095) duty_12bit = 4095;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, kPwm[idx].ch, duty_12bit);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, kPwm[idx].ch);
}

// CLT/IAT: tabela linear simplificada — raw = (degc + 40) * 4095 / 190
static uint32_t temp_to_pwm(int16_t degc) {
    if (degc < -40) degc = -40;
    if (degc > 150) degc = 150;
    return (uint32_t)(degc + 40) * 4095u / 190u;
}

static void update_all_sensors() {
    SimState& s = g_sim;
    dac_output_voltage(kMapDac, map_to_dac(s.map_kpa));
    dac_output_voltage(kTpsDac, tps_to_dac(s.tps_pct));
    pwm_write(0, temp_to_pwm(s.clt_degc));   // CLT
    pwm_write(1, temp_to_pwm(s.iat_degc));   // IAT
    pwm_write(2, (uint32_t)s.app_pct * 4095u / 100u);  // APP
    pwm_write(3, (uint32_t)s.oil_bar_x10 * 4095u / 500u);  // OIL
    pwm_write(4, (uint32_t)s.fuel_bar_x10 * 4095u / 500u); // FUEL
    pwm_write(5, (uint32_t)s.etb_pct * 4095u / 100u);  // ETB1
    pwm_write(6, (uint32_t)(100u - s.etb_pct) * 4095u / 100u); // ETB2 inverted
}

// Presets
static void preset(const char* name, uint32_t rpm, uint16_t map_kpa, uint8_t tps,
                   int16_t clt, int16_t iat, uint8_t app, uint16_t fuel, uint16_t oil, uint8_t etb) {
    ckp_set_rpm(rpm);
    g_sim.map_kpa = map_kpa;  g_sim.tps_pct = tps;
    g_sim.clt_degc = clt;     g_sim.iat_degc = iat;
    g_sim.app_pct  = app;     g_sim.fuel_bar_x10 = fuel;
    g_sim.oil_bar_x10 = oil;  g_sim.etb_pct = etb;
    update_all_sensors();
    Serial.printf("  [SIM] %s: %lu RPM MAP=%u TPS=%u CLT=%d IAT=%d\n",
                  name, (unsigned long)rpm, map_kpa, tps, clt, iat);
}

static void preset_idle()   { preset("IDLE",   700,  35,  3,  90, 25,  0, 350, 20, 3); }
static void preset_crank()  { preset("CRANK",  200, 101,  0,  20, 15,  0, 350, 10, 0); }
static void preset_cruise() { preset("CRUISE",2000,  55, 20,  90, 35, 20, 350, 30, 20); }
static void preset_wot()    { preset("WOT",   4000, 100,100,  90, 40,100, 350, 40,100); }
static void preset_coast()  { preset("COAST", 2000,  25,  0,  90, 35,  0, 350, 20, 0); }

static void print_status() {
    SimState& s = g_sim;
    Serial.printf("  RPM=%lu MAP=%u TPS=%u CLT=%d IAT=%d APP=%u FUEL=%.1f OIL=%.1f ETB=%u\n",
                  (unsigned long)s.rpm, s.map_kpa, s.tps_pct,
                  s.clt_degc, s.iat_degc, s.app_pct,
                  s.fuel_bar_x10 / 10.0f, s.oil_bar_x10 / 10.0f, s.etb_pct);
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 5 — Timing Analysis (IGN sequência + avanço)
// ═══════════════════════════════════════════════════════════════════════════

static constexpr int kIgnFirst = 0;
static constexpr int kIgnCount = 4;
static constexpr uint8_t kExpectedFiringOrder[kIgnCount] = {0, 2, 3, 1};
static constexpr int kInjFirst = 4;

static constexpr float kTdcDeg[kIgnCount] = {0.0f, 540.0f, 180.0f, 360.0f};
static constexpr float kDefaultAdvanceDeg = 15.0f;

enum class TmState : uint8_t { IDLE, WAIT_GAP, CAPTURE, WAIT_GAP_2, DONE };

struct TimingCapture {
    int64_t  gap_ts_us[2];
    int64_t  last_ckp_fall_us;
    int64_t  spark_ts_us[2][kIgnCount];
    bool     spark_done[2][kIgnCount];
    int64_t  inj_ts_us[2][kIgnCount];
    bool     inj_done[2][kIgnCount];
    uint32_t ckp_period_us;
    int      sparks_captured[2];
    int      inj_captured[2];
    uint8_t  half;
    uint32_t timeout_us;
};

static TmState       g_tm_state = TmState::IDLE;
static TimingCapture g_tm_cap;

static void timing_report();

static void timing_start() {
    g_tm_cap               = {};
    g_tm_cap.ckp_period_us = (g_m[kCkpChan].period_us > 0)
                             ? g_m[kCkpChan].period_us : 2000u;
    g_tm_cap.timeout_us    = g_tm_cap.ckp_period_us * 120u;
    g_tm_cap.half          = 0;
    g_tm_state = TmState::WAIT_GAP;
    Serial.println("  [TIMING 720] A aguardar gap CKP...");
    Serial.printf("  [TIMING] T_dente=%.3f ms  timeout=%.0f ms\n",
                  g_tm_cap.ckp_period_us / 1000.0f,
                  g_tm_cap.timeout_us    / 1000.0f);
}

static void timing_feed(const EdgeEvent& ev) {
    TimingCapture& c = g_tm_cap;

    if (g_tm_state == TmState::WAIT_GAP || g_tm_state == TmState::WAIT_GAP_2) {
        if (ev.ch != (uint8_t)kCkpChan) { return; }
        if (ev.level == 0u) {
            c.last_ckp_fall_us = ev.ts_us;
        } else if (c.last_ckp_fall_us > 0) {
            const int64_t gap = ev.ts_us - c.last_ckp_fall_us;
            if (gap > (int64_t)(c.ckp_period_us * 18u / 10u)) {
                c.gap_ts_us[c.half] = ev.ts_us;
                g_tm_state = TmState::CAPTURE;
                Serial.printf("  [TIMING] Gap %d OK (%.2f ms). A capturar half %d...\n",
                              c.half, gap / 1000.0f, c.half);
            }
        }
        return;
    }

    if (g_tm_state == TmState::CAPTURE) {
        if ((ev.ts_us - c.gap_ts_us[c.half]) > (int64_t)c.timeout_us) {
            Serial.println("  [TIMING] Timeout — verificar ligações IGN/INJ.");
            g_tm_state = TmState::IDLE;
            return;
        }
        if (ev.level != 0u) { return; }
        const int h = c.half;

        if (ev.ch >= (uint8_t)kIgnFirst &&
            ev.ch <  (uint8_t)(kIgnFirst + kIgnCount)) {
            const int idx = (int)(ev.ch - kIgnFirst);
            if (!c.spark_done[h][idx]) {
                c.spark_ts_us[h][idx] = ev.ts_us;
                c.spark_done[h][idx]  = true;
                c.sparks_captured[h]++;
                Serial.printf("  [TIMING] IGN%d spark @ +%.3f ms (half %d)\n",
                              idx, (ev.ts_us - c.gap_ts_us[h]) / 1000.0f, h);
            }
        }

        if (ev.ch >= (uint8_t)kInjFirst &&
            ev.ch <  (uint8_t)(kInjFirst + kIgnCount)) {
            const int idx = (int)(ev.ch - kInjFirst);
            if (!c.inj_done[h][idx]) {
                c.inj_ts_us[h][idx] = ev.ts_us;
                c.inj_done[h][idx]  = true;
                c.inj_captured[h]++;
                Serial.printf("  [TIMING] INJ%d @ +%.3f ms (half %d)\n",
                              idx, (ev.ts_us - c.gap_ts_us[h]) / 1000.0f, h);
            }
        }

        if (c.sparks_captured[h] == kIgnCount) {
            if (h == 0) {
                c.half = 1;
                g_tm_state = TmState::WAIT_GAP_2;
                Serial.println("  [TIMING] Half 0 completa. A aguardar gap 2...");
            } else {
                g_tm_state = TmState::DONE;
                timing_report();
            }
        }
    }
}

static void timing_report() {
    const TimingCapture& c  = g_tm_cap;
    const float          T  = c.ckp_period_us / 1000.0f;
    const float expected_inter_ms = T * 60.0f * 2.0f / 4.0f;

    uint8_t order[kIgnCount] = {0, 1, 2, 3};
    for (int i = 0; i < kIgnCount - 1; ++i)
        for (int j = 0; j < kIgnCount - 1 - i; ++j)
            if (c.spark_ts_us[0][order[j]] > c.spark_ts_us[0][order[j + 1]]) {
                uint8_t t = order[j]; order[j] = order[j+1]; order[j+1] = t;
            }

    Serial.println();
    Serial.println("  ╔══════════════════════════════════════════════════════╗");
    Serial.println("  ║  Ignition Timing Analysis — 720°                    ║");
    Serial.printf( "  ║  T_dente=%.3f ms  Inter-cil esperado=%.3f ms       ║\n",
                   T, expected_inter_ms);
    Serial.println("  ╚══════════════════════════════════════════════════════╝");

    // ── Inter-cylinder spacing (half 0) ──
    Serial.println();
    Serial.println("  Canal  Desde gap      Inter-cil     Esperado  Desvio  OK?");
    Serial.println("  ─────  ─────────────  ───────────  ────────  ──────  ───");

    float prev_ms = 0.0f;
    bool  timing_ok = true;
    for (int i = 0; i < kIgnCount; ++i) {
        const int   ch  = order[i];
        const float dt  = (c.spark_ts_us[0][ch] - c.gap_ts_us[0]) / 1000.0f;
        const float ic  = (i == 0) ? 0.0f : dt - prev_ms;
        const float dev = (i == 0) ? 0.0f : ic - expected_inter_ms;
        const bool  ok  = (i == 0) || (fabsf(dev) < 1.5f);
        if (!ok) timing_ok = false;
        if (i == 0)
            Serial.printf("  IGN%d   %8.3f ms        —             —       —      —\n",
                          ch, dt);
        else
            Serial.printf("  IGN%d   %8.3f ms  %8.3f ms  %7.3f ms %+6.3f ms %s\n",
                          ch, dt, ic, expected_inter_ms, dev, ok ? "✓" : "✗ FALHA");
        prev_ms = dt;
    }

    // ── Firing order ──
    Serial.println();
    Serial.print("  Detectada : ");
    for (int i = 0; i < kIgnCount; ++i) { if (i) Serial.print("→"); Serial.printf("IGN%d", order[i]); }
    Serial.println();
    Serial.print("  Esperada  : ");
    for (int i = 0; i < kIgnCount; ++i) { if (i) Serial.print("→"); Serial.printf("IGN%d", kExpectedFiringOrder[i]); }
    bool order_ok = true;
    for (int i = 0; i < kIgnCount; ++i) if (order[i] != kExpectedFiringOrder[i]) order_ok = false;
    Serial.printf("  %s\n", order_ok ? "  ✓ CORRECTA" : "  ✗ ERRADA");

    // ── 720° period per cylinder (sequential proof) ──
    Serial.println();
    Serial.println("  ── Período 720° por cilindro ──");
    const float gap_period_us = (float)(c.gap_ts_us[1] - c.gap_ts_us[0]);
    const float expected_720_ms = gap_period_us * 2.0f / 1000.0f;
    Serial.printf("  Gap period (360°) = %.3f ms → esperado 720° = %.3f ms\n",
                  gap_period_us / 1000.0f, expected_720_ms);
    Serial.println("  Cil    Período 720°   Esperado   Desvio   OK?");
    Serial.println("  ───    ────────────    ────────   ──────   ───");

    bool period_ok = true;
    for (int cyl = 0; cyl < kIgnCount; ++cyl) {
        const float p_ms = (c.spark_ts_us[1][cyl] - c.spark_ts_us[0][cyl]) / 1000.0f;
        const float dev  = p_ms - expected_720_ms;
        const float tol  = expected_720_ms * 0.05f;
        const bool  ok   = fabsf(dev) < tol;
        if (!ok) { period_ok = false; }
        Serial.printf("  IGN%d  %8.3f ms  %8.3f ms  %+6.3f ms  %s\n",
                      cyl, p_ms, expected_720_ms, dev, ok ? "✓" : "✗ FALHA");
    }

    // ── Spark angle vs TDC (informational) ──
    Serial.println();
    Serial.println("  ── Ângulo spark vs TDC (informativo) ──");
    Serial.println("  Nota: ângulo absoluto requer calibração trigger_tooth0_engine_deg.");
    Serial.printf("  Advance configurado: %.1f°\n", kDefaultAdvanceDeg);
    Serial.println("  Cil    Medido°    Esperado°   Desvio°");
    Serial.println("  ───    ───────    ─────────   ───────");
    for (int cyl = 0; cyl < kIgnCount; ++cyl) {
        const float dt_us = (float)(c.spark_ts_us[0][cyl] - c.gap_ts_us[0]);
        const float measured_deg = (dt_us / (float)c.ckp_period_us) * 6.0f;
        float expected_deg = kTdcDeg[cyl] - kDefaultAdvanceDeg;
        if (expected_deg < 0.0f) expected_deg += 720.0f;
        float dev = measured_deg - expected_deg;
        if (dev > 360.0f) dev -= 720.0f;
        if (dev < -360.0f) dev += 720.0f;
        Serial.printf("  IGN%d  %7.1f°   %7.1f°    %+6.1f°\n",
                      cyl, measured_deg, expected_deg, dev);
    }

    // ── INJ offset (informational) ──
    int inj_total = c.inj_captured[0];
    if (inj_total > 0) {
        Serial.println();
        Serial.println("  ── Offset INJ → IGN (informativo) ──");
        Serial.println("  Cil    INJ→IGN (ms)");
        Serial.println("  ───    ────────────");
        for (int cyl = 0; cyl < kIgnCount; ++cyl) {
            if (c.inj_done[0][cyl]) {
                const float off = (c.inj_ts_us[0][cyl] - c.spark_ts_us[0][cyl]) / 1000.0f;
                Serial.printf("  INJ%d  %+8.3f ms\n", cyl, off);
            } else {
                Serial.printf("  INJ%d  sem dados\n", cyl);
            }
        }
    }

    // ── Overall result ──
    Serial.println();
    const bool all_ok = timing_ok && order_ok && period_ok;
    Serial.printf("  Resultado final: %s\n",
                  all_ok ? "✓ IGNIÇÃO SEQUENCIAL OK (720°)"
                         : "✗ VER FALHAS ACIMA");
    if (!period_ok) {
        Serial.println("  NOTA: Período 720° falhou — possível wasted-spark ou fase errada.");
    }
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
    Serial.println("    t       Timing 720° (seq)     s  Estatísticas");
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

    // ── LED output (se disponível) ────────────────────────────────────────
    if (LED_GPIO != GPIO_NUM_MAX) {
        gpio_reset_pin(LED_GPIO);
        gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(LED_GPIO, 0);
    }

    // ── GPIOs de entrada (scope: IGN, INJ, CKP loopback) ──────────────────
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

    // ── CKP/CMP via RMT (hardware, jitter ~zero) ──────────────────────────
    rmt_ckp_init();

    // ── Sensores analógicos (PWM + RC filter) ──────────────────────────
    pwm_init();
    dac_init_map_tps();
    g_sim = {};  // zero-init
    g_sim.rpm = kRpmInit;
    update_all_sensors();

    print_help();
    Serial.printf("\n  CKP: %lu RPM (RMT)  Modo scope: LIVE\n", (unsigned long)g_rpm);
    Serial.printf("  Wire loopback: GPIO2 -> GPIO34 (CKP scope)\n\n");
}

static uint32_t g_last_live_ms = 0;

// Line buffer for text commands (e.g. "RPM 800\n" from hil_test.py)
static char g_line[64];
static int  g_line_pos = 0;

static void parse_text_cmd(const char* raw) {
    char buf[64];
    int  len = 0;
    while (raw[len] && len < 63) { buf[len] = raw[len]; len++; }
    buf[len] = '\0';
    for (int i = 0; i < len; ++i)
        if (buf[i] >= 'a' && buf[i] <= 'z') buf[i] -= 32;

    char cmd[16] = {};
    int  val = 0;
    const bool has_val = (sscanf(buf, "%15s %d", cmd, &val) >= 2);

    if (strcmp(cmd, "RPM") == 0 && has_val) {
        ckp_set_rpm((uint32_t)constrain(val, (int)kRpmMin, (int)kRpmMax));
        g_sim.rpm = g_rpm;
        Serial.printf("  [GEN] RPM=%lu\n", (unsigned long)g_rpm);
    }
    else if (strcmp(cmd, "MAP") == 0 && has_val) { g_sim.map_kpa = constrain(val, 0, 300); update_all_sensors(); }
    else if (strcmp(cmd, "TPS") == 0 && has_val) { g_sim.tps_pct = constrain(val, 0, 100); update_all_sensors(); }
    else if (strcmp(cmd, "CLT") == 0 && has_val) { g_sim.clt_degc = constrain(val, -40, 150); update_all_sensors(); }
    else if (strcmp(cmd, "IAT") == 0 && has_val) { g_sim.iat_degc = constrain(val, -40, 150); update_all_sensors(); }
    else if (strcmp(cmd, "APP") == 0 && has_val) { g_sim.app_pct = constrain(val, 0, 100); update_all_sensors(); }
    else if (strcmp(cmd, "FUEL")== 0 && has_val) { g_sim.fuel_bar_x10 = constrain(val, 0, 500); update_all_sensors(); }
    else if (strcmp(cmd, "OIL") == 0 && has_val)  { g_sim.oil_bar_x10  = constrain(val, 0, 500); update_all_sensors(); }
    else if (strcmp(cmd, "ETB") == 0 && has_val)  { g_sim.etb_pct = constrain(val, 0, 100); update_all_sensors(); }
    else if (strcmp(cmd, "IDLE")   == 0)   preset_idle();
    else if (strcmp(cmd, "CRANK")  == 0)   preset_crank();
    else if (strcmp(cmd, "CRUISE") == 0)   preset_cruise();
    else if (strcmp(cmd, "WOT")    == 0)   preset_wot();
    else if (strcmp(cmd, "COAST")  == 0)   preset_coast();
    else if (strcmp(cmd, "DAC25") == 0 && has_val) {
        // Reteste do DAC1/GPIO25: escreve valor cru p/ medição com multímetro
        dac_output_voltage(DAC_CHANNEL_1, (uint8_t)constrain(val, 0, 255));
        Serial.printf("  [DAC] GPIO25=%d (%.2fV esperado)\n", val, val * 3.3f / 255.0f);
    }
    else if (strcmp(cmd, "DAC26") == 0 && has_val) {
        dac_output_voltage(DAC_CHANNEL_2, (uint8_t)constrain(val, 0, 255));
        Serial.printf("  [DAC] GPIO26=%d (%.2fV esperado)\n", val, val * 3.3f / 255.0f);
    }
    else if (strcmp(cmd, "STATUS") == 0)   print_status();
}

void loop() {
    process_events();

    while (Serial.available()) {
        const char c = (char)Serial.read();

        // Accumulate text commands terminated by '\n' or '\r'
        if (c == '\n' || c == '\r') {
            if (g_line_pos > 0) {
                g_line[g_line_pos] = '\0';
                parse_text_cmd(g_line);
                g_line_pos = 0;
            }
            // Also handle as single-char scope command if buffer was empty
        } else {
            if (g_line_pos < (int)(sizeof(g_line) - 1))
                g_line[g_line_pos++] = c;
        }

        // Single-char commands (only when not mid-line)
        if (g_line_pos == 1) {
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
        } // end if (g_line_pos == 1)
    } // end while Serial.available()

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
