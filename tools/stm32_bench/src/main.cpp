/*
 * stm32_bench — gerador CKP/CMP + sensores + scope lógico para bancada OpenEMS
 * ═══════════════════════════════════════════════════════════════════════════
 * Ferramenta de teste da OpenEMS (não é firmware da ECU). Um STM32H562RGT6
 * (WeAct 64 pinos) faz o papel do esp32_combined: gera CKP/CMP 60-2 por timer
 * de hardware, simula os sensores lentos e mede INJ/IGN da ECU sob teste.
 * Mesmo protocolo serial do esp32_combined (hil_test, scope_link, stim_link).
 *
 * Plataforma : STM32H562RGT6, core STM32duino (PlatformIO, framework arduino)
 * Resolução  : 1 µs — TIM5 de 32 bits é a base de tempo de tudo (gerador e scope)
 *
 * ── Ligações (STM32 gerador ↔ ECU sob teste, mapa BOARD=vgt6 de out_pins.h) ──
 *
 *   Gerador   ECU       Função
 *   ────────  ────────  ────────────────────────────────────────────
 *   PA0   →   PA0       CKP (60-2)   TIM5_CH1 AF2, toggle por compare
 *   PA1   →   PA1       CMP (1/720°) TIM5_CH2 AF2, toggle por compare
 *   PA4   →   PA3       MAP  (DAC1_OUT1, direto)
 *   PA5   →   PA4       TPS  (DAC1_OUT2, direto)
 *   PA6   →   PB0       CLT  (TIM3_CH1 PWM + RC)
 *   PA7   →   PB1       IAT  (TIM3_CH2 PWM + RC)
 *   PB0   →   PC0       APP1 (TIM3_CH3 PWM + RC)
 *   PB1   →   PC1       OIL  (TIM3_CH4 PWM + RC)
 *   PB6   →   PC4       FUEL (TIM4_CH1 PWM + RC)
 *   PB7   →   PA2       ETB TPS1 (TIM4_CH2 PWM + RC)
 *   PB8   →   PC5       ETB TPS2, invertido (TIM4_CH3 PWM + RC)
 *   PC0   ←   PE9       IGN1        PC4   ←   PE0        INJ1
 *   PC1   ←   PE11      IGN2        PC5   ←   PE2        INJ2
 *   PC2   ←   PE13      IGN3        PB12  ←   PE4        INJ3
 *   PC3   ←   PE15      IGN4        PB13  ←   PE6        INJ4
 *   GND   —   GND       OBRIGATÓRIO
 *
 *   Se a ECU sob teste for BOARD=rgt6: INJ PA15/PB3/PC10/PC11, IGN PC6-PC9.
 *   Entradas do scope usam EXTI (um número de pino por linha: 0-5, 12, 13).
 *   Pinos, AF e DAC: docs do datasheet em docs/alternatefunctions2.pdf.
 *
 * ── Comandos série (115200, USB CDC) ───────────────────────────────────────
 *
 *   +  / -   RPM ± 100          0-9  presets de RPM
 *   RPM <n>  RPM exato (50-9000), com rampa     SLEW <n>  passo da rampa (%/50 ms)
 *   MAP <kPa> / TPS <%> / CLT <C> / IAT <C> / APP <%> / FUEL <bar*10> / OIL <bar*10> / ETB <%>
 *   IDLE / CRANK / CRUISE / WOT / COAST          STATUS
 *   PROFILE TAPER | PROFILE STOP
 *   DAC1 <0-4095> / DAC2 <0-4095>   código cru nos canais do DAC (calibração)
 *   Scope: l e p w t s r ?
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <cstring>
#include <cmath>

static void parse_text_cmd(const char* raw);

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 1 — Base de tempo (TIM5, 32 bits, 1 MHz) e gerador CKP/CMP
// ═══════════════════════════════════════════════════════════════════════════

// PA0/PA1 sem ALT1 resolvem para TIM2 no pinmap do core; TIM5_CH1/CH2 (AF2) exigem ALT1.
static constexpr uint32_t kCkpPin    = PA0_ALT1;   // TIM5_CH1 AF2
static constexpr uint32_t kCmpPin    = PA1_ALT1;   // TIM5_CH2 AF2
static constexpr uint32_t kTickHz    = 1000000u;

static constexpr uint32_t kRpmInit   = 500;
static constexpr int      kCmpTooth  = 5;
static constexpr int      kRealTeeth = 58;
static constexpr int      kEdgesRev  = kRealTeeth * 2;   // bordas por volta do virabrequim
static constexpr uint32_t kRpmMin    = 50;
static constexpr uint32_t kRpmMax    = 9000;

static const uint32_t kRpmPresets[] = {
    100, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000
};

static volatile uint32_t g_rpm       = kRpmInit;
static volatile uint32_t g_rev_count = 0;
static volatile uint32_t g_cmp_count = 0;

static HardwareTimer* g_tim5 = nullptr;
static volatile uint32_t g_hi = 0;   // palavra alta do tempo: overflow de TIM5 a cada ~71 min

static volatile uint32_t g_ckp_rpm_active = 0u;
// Período do dente (µs) do RPM ativo. A ISR amostra na borda de subida de cada dente,
// então cada dente é internamente consistente e a rampa de RPM avança dente a dente
// (mais fino que a volta inteira, sem mistura old/new dentro de um dente).
static volatile uint32_t g_tooth_us = 1000000u / kRpmInit;
static volatile uint32_t g_rpm_target = kRpmInit;
static uint32_t g_slew_last_ms = 0;
static uint32_t g_slew_pct = 2u;   // passo máximo por tick de 50 ms (comando SLEW)

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

// Tempo de TIM5 estendido a 64 bits. hi é lido antes e depois de CNT: se o overflow
// ocorreu no meio, o valor baixo de CNT pertence à volta nova.
static inline int64_t ts_now() {
    const uint32_t hi  = g_hi;
    const uint32_t c   = TIM5->CNT;
    const uint32_t hi2 = g_hi;
    uint32_t h = hi;
    if (hi2 != hi) { if (c < 0x80000000u) h = hi2; }
    else if ((TIM5->SR & TIM_SR_UIF) && c < 0x80000000u) h = hi + 1u;   // overflow ainda não tratado
    return ((int64_t)h << 32) | c;
}

// Estende um valor de 32 bits de TIM5 (p.ex. CCR1) para 64 bits, perto de agora.
static inline int64_t ts_ext(uint32_t t) {
    const int64_t now = ts_now();
    return now + (int32_t)(t - (uint32_t)now);
}

// Empurrar evento para o buffer (chamado de ISR; ISRs de prioridades diferentes
// escrevem aqui, então a atualização do head é atômica).
static inline void buf_push(int64_t ts, uint8_t ch, uint8_t level) {
    const uint32_t pm = __get_PRIMASK();
    __disable_irq();
    const uint16_t next = (g_head + 1u) & kBufMask;
    if (next != g_tail) {
        g_buf[g_head].ts_us = ts;
        g_buf[g_head].ch    = ch;
        g_buf[g_head].level = level;
        g_head = next;
    }
    // Se buffer cheio: evento descartado
    __set_PRIMASK(pm);
}

// ── Definição de canais scope ─────────────────────────────────────────────
// CH8 (CKP) e CH9 (CMP) são virtuais: a ISR do gerador os empurra com o instante
// exato da borda (valor de CCR), sem fio de loopback.

struct ChanDef {
    uint32_t    gpio;      // pino Arduino (PC0...) ou kVirtual
    const char  name[6];
    const char  pin[6];    // pino da ECU sob teste (VGT6)
    const char  gpio_name[5];
    bool        enabled;
};

static constexpr uint32_t kVirtual = 0xFFFFFFFFu;

static ChanDef kChan[] = {
    { PC0,  "IGN1", "PE9",  "PC0",  true  },   // CH0 — IGN1 (cyl 1)
    { PC1,  "IGN2", "PE11", "PC1",  true  },   // CH1 — IGN2 (cyl 2)
    { PC2,  "IGN3", "PE13", "PC2",  true  },   // CH2 — IGN3 (cyl 3)
    { PC3,  "IGN4", "PE15", "PC3",  true  },   // CH3 — IGN4 (cyl 4)
    { PC4,  "INJ1", "PE0",  "PC4",  true  },   // CH4 — INJ1 (cyl 1)
    { PC5,  "INJ2", "PE2",  "PC5",  true  },   // CH5 — INJ2 (cyl 2)
    { PB12, "INJ3", "PE4",  "PB12", true  },   // CH6 — INJ3 (cyl 3)
    { PB13, "INJ4", "PE6",  "PB13", true  },   // CH7 — INJ4 (cyl 4)
    { kVirtual, "CKP", "PA0", "-",  true  },   // CH8 — virtual, da ISR do gerador
    { kVirtual, "CMP", "PA1", "-",  true  },   // CH9 — virtual, da ISR do gerador
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
// ── SECÇÃO 3 — Gerador CKP/CMP (TIM5 CH1/CH2, OC toggle, borda em hardware)
// ═══════════════════════════════════════════════════════════════════════════
//
// CH1 alterna PA0 no match de CCR1; a ISR só carrega o próximo CCR1 (≥55 µs à frente
// a 9000 rpm). A borda é do hardware: o jitter de ISR não aparece no pino, desde que a
// latência fique abaixo de meio dente (TIM5 tem prioridade 0).
// CH2 gera o CMP: 1 µs depois da borda do CKP do dente kCmpTooth da volta 0 (mesmo
// offset anti-race do TIM5 da ECU que o esp32_combined usava). Os dois canais partilham
// o contador, então a fase CKP↔CMP é fixa e não depende de RPM.

static volatile uint16_t g_edge = 0;      // índice (0..115) da borda que está armada em CCR1
static volatile uint8_t  g_rev  = 0;      // 0/1: volta do ciclo de 720° da borda armada
static volatile uint32_t g_tooth_cur = 1000000u / kRpmInit;   // período amostrado no dente atual
static volatile bool     g_cmp_high = false;
static volatile uint32_t g_cmp_fall_t = 0;

static void tim5_ovf_isr() { g_hi = g_hi + 1u; }

// ISR de CCR1: a borda `g_edge` acabou de ocorrer em t = CCR1.
static void ckp_isr() {
    const uint32_t t   = TIM5->CCR1;
    const uint16_t e   = g_edge;
    const uint8_t  rev = g_rev;
    const uint8_t  lvl = (e & 1u) ? 0u : 1u;            // pares: subida
    const int64_t  ts  = ts_ext(t);
    const int      tooth = e >> 1;

    buf_push(ts, kCkpChan, lvl);
    if (rev == 0u && tooth == kCmpTooth) buf_push(ts + 1, kCmpChan, lvl);
    if (lvl) { g_tooth_cur = g_tooth_us; if (e == 0u && rev == 0u) g_rev_count++; }

    const uint32_t T = g_tooth_cur;
    const uint32_t h = T / 2u;
    const uint32_t l = T - h;
    uint32_t dt;
    if (lvl)                              dt = h;
    else if (tooth == kRealTeeth - 1)     dt = l + 2u * T;   // gap: 2 dentes faltantes
    else                                  dt = l;

    uint16_t ne = (uint16_t)(e + 1u);
    uint8_t  nrev = rev;
    if (ne == kEdgesRev) { ne = 0; nrev = (uint8_t)(rev ^ 1u); }

    const uint32_t nt = t + dt;
    TIM5->CCR1 = nt;
    // CMP: arma a subida junto da borda do CKP que a antecede; a descida é armada
    // pela ISR de CCR2 depois que a subida aconteceu (sem corrida com o match).
    if (nrev == 0u && ne == 2 * kCmpTooth) TIM5->CCR2 = nt + 1u;
    if (rev == 0u && e == 2 * kCmpTooth)   g_cmp_fall_t = nt + 1u;

    g_edge = ne;
    g_rev  = nrev;
}

// ISR de CCR2: o CMP acabou de alternar. A ordem CC1→CC2 dentro do mesmo handler do
// TIM5 garante que g_cmp_fall_t já foi escrito pela ckp_isr da mesma borda.
static void cmp_isr() {
    if (!g_cmp_high) {
        g_cmp_high = true;
        g_cmp_count++;
        TIM5->CCR2 = g_cmp_fall_t;
    } else {
        g_cmp_high = false;
        TIM5->CCR2 = TIM5->CCR2 + 0x7FFFFFFFu;   // estacionado ~35 min à frente; reescrito antes da próxima subida
    }
}

static void ckp_set_rpm(uint32_t rpm) {
    if (rpm < kRpmMin) rpm = kRpmMin;
    if (rpm > kRpmMax) rpm = kRpmMax;
    g_rpm_target = rpm;   // o slew (ckp_slew_tick) leva o RPM ativo até lá
    g_rpm = rpm;
}

// Rampa o RPM ativo em direção ao alvo (≤g_slew_pct %/tick de 50 ms, padrão 2).
// A ISR lê o período a cada dente, sem parar o gerador.
static void ckp_slew_tick() {
    const uint32_t now = millis();
    if (now - g_slew_last_ms < 50u) return;
    g_slew_last_ms = now;
    const uint32_t tgt = g_rpm_target;
    uint32_t cur = g_ckp_rpm_active;
    if (cur == tgt) return;
    const uint32_t step = (cur > 50u) ? (cur * g_slew_pct) / 100u : 1u;  // g_slew_pct %
    if (tgt > cur) { cur = (cur + step >= tgt) ? tgt : cur + step; }
    else           { cur = (cur <= tgt + step) ? tgt : cur - step; }
    g_tooth_us = 1000000u / cur;   // T = 60e6 / (rpm * 60)
    g_ckp_rpm_active = cur;
}

static void ckp_gen_init() {
    g_tim5 = new HardwareTimer(TIM5);
    g_tim5->setPrescaleFactor(g_tim5->getTimerClkFreq() / kTickHz);
    g_tim5->setOverflow(0xFFFFFFFFu, TICK_FORMAT);
    g_tim5->setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE, kCkpPin);
    g_tim5->setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE, kCmpPin);
    g_tim5->setCaptureCompare(1, 1000u, TICK_COMPARE_FORMAT);          // primeira borda 1 ms após o start
    g_tim5->setCaptureCompare(2, 0x7FFFFFFFu, TICK_COMPARE_FORMAT);    // CMP estacionado
    g_tim5->attachInterrupt(1, ckp_isr);
    g_tim5->attachInterrupt(2, cmp_isr);
    g_tim5->attachInterrupt(tim5_ovf_isr);
    g_tim5->setInterruptPriority(0, 0);
    g_ckp_rpm_active = kRpmInit;
    g_rpm_target     = kRpmInit;
    g_tooth_us       = 1000000u / kRpmInit;
    g_tim5->resume();
    Serial.println("TIM5 OK (CKP/CMP por compare, borda em hardware)");
}

// ── Entradas do scope (IGN, INJ) por EXTI ─────────────────────────────────

static GPIO_TypeDef* g_in_port[kNChan];
static uint32_t      g_in_mask[kNChan];

static inline void edge_isr(uint8_t ch) {
    const int64_t ts = ts_ext(TIM5->CNT);
    buf_push(ts, ch, (g_in_port[ch]->IDR & g_in_mask[ch]) ? 1u : 0u);
}

template <uint8_t CH> static void edge_isr_t() { edge_isr(CH); }

static void (*const kEdgeIsr[8])() = {
    edge_isr_t<0>, edge_isr_t<1>, edge_isr_t<2>, edge_isr_t<3>,
    edge_isr_t<4>, edge_isr_t<5>, edge_isr_t<6>, edge_isr_t<7>,
};

static void scope_inputs_init() {
    for (int ch = 0; ch < kNChan; ++ch) {
        if (!kChan[ch].enabled || kChan[ch].gpio == kVirtual) continue;
        const PinName pn = digitalPinToPinName(kChan[ch].gpio);
        g_in_port[ch] = get_GPIO_Port(STM_PORT(pn));
        g_in_mask[ch] = 1u << STM_PIN(pn);
        pinMode(kChan[ch].gpio, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(kChan[ch].gpio), kEdgeIsr[ch], CHANGE);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── SECÇÃO 4 — Sensores analógicos (DAC1 + PWM com filtro RC)
// ═══════════════════════════════════════════════════════════════════════════

// MAP/TPS: DAC1 de 12 bits (OUT1=PA4, OUT2=PA5), ligação direta ao ADC da ECU.
// Os trims abaixo estão neutros: calibre na bancada com multímetro (DAC1/DAC2 <0-4095>)
// e a leitura na ECU. O buffer de saída do DAC pode não chegar aos trilhos; confirmar
// nas características elétricas do DAC (DS14138) e ajustar os trims.
// Sensores lentos: PWM de 19 kHz em TIM3/TIM4 + RC (10 kΩ + 100 nF por canal).

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

static constexpr int32_t kMapGainPermille = 1000, kMapOffsetCode = 0;
static constexpr int32_t kTpsGainPermille = 1000, kTpsOffsetCode = 0;

static uint16_t dac_trim(int32_t code, int32_t gain_permille, int32_t offset) {
    code = code * gain_permille / 1000 + offset;
    if (code < 0) code = 0;
    if (code > 4095) code = 4095;
    return (uint16_t)code;
}

// MAP: 0-300 kPa → 0-VDDA (linear nominal).
static uint16_t map_to_dac(uint16_t kpa) {
    if (kpa > 300) kpa = 300;
    return dac_trim((int32_t)kpa * 4095 / 300, kMapGainPermille, kMapOffsetCode);
}
// TPS: 0-100% → piso de 15/255 do fundo de escala (≈0,19 V), como no esp32_combined.
static uint16_t tps_to_dac(uint8_t pct) {
    if (pct > 100) pct = 100;
    const int32_t nom = 15 + (int32_t)pct * 240 / 100;   // 15..255
    return dac_trim(nom * 4095 / 255, kTpsGainPermille, kTpsOffsetCode);
}

static void dac_init_map_tps() {
    __HAL_RCC_DAC1_CLK_ENABLE();
    pinMode(PA4, INPUT_ANALOG);
    pinMode(PA5, INPUT_ANALOG);
    DAC1->CR |= DAC_CR_EN1 | DAC_CR_EN2;
    delay(1);   // tWAKEUP
}
static void dac_write_map(uint16_t code) { DAC1->DHR12R1 = code; }
static void dac_write_tps(uint16_t code) { DAC1->DHR12R2 = code; }

// PWM 19 kHz: CLT, IAT, APP, OIL (TIM3) e FUEL, ETB1, ETB2 (TIM4); duty de 12 bits.
// Nenhum destes pinos está em kChan (scope).
static constexpr uint32_t kPwmFreq = 19000;

struct PwmChan { uint32_t pin; TIM_TypeDef* tim; uint8_t ch; const char* name; HardwareTimer* ht; };
static PwmChan kPwm[] = {
    { PA6, TIM3, 1, "CLT",  nullptr },
    { PA7_ALT1, TIM3, 2, "IAT",  nullptr },   // sem ALT1 seria TIM1_CH1N
    { PB0_ALT1, TIM3, 3, "APP",  nullptr },
    { PB1_ALT1, TIM3, 4, "OIL",  nullptr },
    { PB6, TIM4, 1, "FUEL", nullptr },
    { PB7, TIM4, 2, "ETB1", nullptr },
    { PB8, TIM4, 3, "ETB2", nullptr },
};
static constexpr int kNPwm = (int)(sizeof(kPwm) / sizeof(kPwm[0]));

static void pwm_init() {
    HardwareTimer* t3 = new HardwareTimer(TIM3);
    HardwareTimer* t4 = new HardwareTimer(TIM4);
    t3->setOverflow(kPwmFreq, HERTZ_FORMAT);
    t4->setOverflow(kPwmFreq, HERTZ_FORMAT);
    for (int i = 0; i < kNPwm; ++i) {
        HardwareTimer* ht = (kPwm[i].tim == TIM3) ? t3 : t4;
        kPwm[i].ht = ht;
        ht->setMode(kPwm[i].ch, TIMER_OUTPUT_COMPARE_PWM1, kPwm[i].pin);
        ht->setCaptureCompare(kPwm[i].ch, 0, RESOLUTION_12B_COMPARE_FORMAT);
    }
    t3->resume();
    t4->resume();
}

static void pwm_write(int idx, uint32_t duty_12bit) {
    if (duty_12bit > 4095) duty_12bit = 4095;
    kPwm[idx].ht->setCaptureCompare(kPwm[idx].ch, duty_12bit, RESOLUTION_12B_COMPARE_FORMAT);
}

// CLT/IAT: tabela linear simplificada — raw = (degc + 40) * 4095 / 190
static uint32_t temp_to_pwm(int16_t degc) {
    if (degc < -40) degc = -40;
    if (degc > 150) degc = 150;
    return (uint32_t)(degc + 40) * 4095u / 190u;
}

static void update_all_sensors() {
    SimState& s = g_sim;
    dac_write_map(map_to_dac(s.map_kpa));
    dac_write_tps(tps_to_dac(s.tps_pct));
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
    Serial.printf("  SLEW=%lu%%/50ms\n", (unsigned long)g_slew_pct);
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

// Inteiro de 64 bits em decimal: o printf do newlib-nano não garante %lld.
static const char* i64_str(int64_t v, char* out) {
    char t[24];
    int n = 0, i = 0;
    uint64_t u = (v < 0) ? (uint64_t)(-v) : (uint64_t)v;
    do { t[n++] = (char)('0' + (u % 10u)); u /= 10u; } while (u != 0u);
    if (v < 0) out[i++] = '-';
    while (n > 0) out[i++] = t[--n];
    out[i] = '\0';
    return out;
}

static void process_events() {
    char tsbuf[24];
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
                Serial.printf("  %10s µs  CH%d %-5s RISE\n",
                              i64_str(ev.ts_us, tsbuf), ev.ch, kChan[ev.ch].name);
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
                Serial.printf("  %10s µs  CH%d %-5s FALL  PW=%.3f ms\n",
                              i64_str(ev.ts_us, tsbuf), ev.ch, kChan[ev.ch].name, m.pw_us / 1000.0f);
            if (g_mode == Mode::PULSE && m.pw_us > 0)
                Serial.printf("  CH%d %-5s  PW=%7.3f ms  Period=%7.3f ms  Count=%lu\n",
                              ev.ch, kChan[ev.ch].name,
                              m.pw_us / 1000.0f, m.period_us / 1000.0f, m.pulse_count);
        }
    }
}

static void print_live_table() {
    const int64_t now = ts_now();

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
    Serial.println("  ║  OpenEMS STM32 Bench — CKP Gen + Logic Scope  ║");
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
            Serial.printf("    CH%d  %-5s  ECU %-5s → %s\n",
                          ch, kChan[ch].name, kChan[ch].pin, kChan[ch].gpio_name);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ── setup() e loop()
// ═══════════════════════════════════════════════════════════════════════════

static uint32_t g_last_live_ms = 0;

// Line buffer for text commands (e.g. "RPM 800\n" from hil_test.py)
static char g_line[64];
static int  g_line_pos = 0;

// ── Perfil de partida para o teste do taper do ETB ─────────────────────────
// CRANK (3 s) → 1700 rpm (1 s) → 850 rpm (fica). O slew limita a subida; use
// SLEW <n> (1-30 %/50 ms) para aproximar a partida real e confira o sync.
static constexpr uint8_t kProfOff = 0, kProfCrank = 1, kProfRise = 2, kProfSettle = 3;
static uint8_t g_profile = kProfOff;
static uint32_t g_profile_t0 = 0u;

static void profile_enter(uint8_t step, const char* name) {
    g_profile = step;
    g_profile_t0 = millis();
    Serial.printf("  [PROFILE] %s t=%lu\n", name, (unsigned long)g_profile_t0);
}

static void profile_start() {
    preset_crank();
    profile_enter(kProfCrank, "CRANK 200 rpm");
}

static void profile_tick() {
    if (g_profile == kProfOff) return;
    const uint32_t el = millis() - g_profile_t0;
    if (g_profile == kProfCrank && el >= 3000u) {
        ckp_set_rpm(1700u); g_sim.rpm = g_rpm;
        profile_enter(kProfRise, "POS-PARTIDA 1700 rpm");
    } else if (g_profile == kProfRise && el >= 1000u) {
        ckp_set_rpm(850u); g_sim.rpm = g_rpm;
        profile_enter(kProfSettle, "ALVO 850 rpm");
    } else if (g_profile == kProfSettle && el >= 5000u) {
        g_profile = kProfOff;
        Serial.printf("  [PROFILE] fim t=%lu\n", (unsigned long)millis());
    }
}

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

    if (strncmp(buf, "PROFILE", 7) == 0) {
        if (strstr(buf, "STOP") != nullptr) { g_profile = kProfOff; Serial.println("  [PROFILE] parado"); }
        else if (strstr(buf, "TAPER") != nullptr) { profile_start(); }
        else { Serial.println("  PROFILE TAPER | PROFILE STOP"); }
        return;
    }

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
    else if (strcmp(cmd, "SLEW") == 0 && has_val) {
        g_slew_pct = (uint32_t)constrain(val, 1, 30);
        Serial.printf("  [GEN] SLEW=%lu %%/50ms\n", (unsigned long)g_slew_pct);
    }
    else if (strcmp(cmd, "IDLE")   == 0)   preset_idle();
    else if (strcmp(cmd, "CRANK")  == 0)   preset_crank();
    else if (strcmp(cmd, "CRUISE") == 0)   preset_cruise();
    else if (strcmp(cmd, "WOT")    == 0)   preset_wot();
    else if (strcmp(cmd, "COAST")  == 0)   preset_coast();
    else if (strcmp(cmd, "DAC1") == 0 && has_val) {
        // Código cru no DAC1_OUT1 (PA4, MAP) para medir com multímetro
        const int code = constrain(val, 0, 4095);
        dac_write_map((uint16_t)code);
        Serial.printf("  [DAC] PA4=%d (%.3fV esperado)\n", code, code * 3.3f / 4095.0f);
    }
    else if (strcmp(cmd, "DAC2") == 0 && has_val) {
        const int code = constrain(val, 0, 4095);
        dac_write_tps((uint16_t)code);
        Serial.printf("  [DAC] PA5=%d (%.3fV esperado)\n", code, code * 3.3f / 4095.0f);
    }
    else if (strcmp(cmd, "STATUS") == 0)   print_status();
}

void setup() {
    Serial.begin(115200);
    {   // não bloqueia sem host: segue após 1,5 s
        const uint32_t t0 = millis();
        while (!Serial && (millis() - t0) < 1500u) { delay(10); }
    }
    memset(g_m, 0, sizeof(g_m));

#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
#endif

    // ── Sensores analógicos (DAC + PWM com RC) ────────────────────────────
    pwm_init();
    dac_init_map_tps();
    g_sim = {};  // zero-init
    g_sim.rpm = kRpmInit;
    update_all_sensors();

    // ── Gerador CKP/CMP (TIM5) e entradas do scope (EXTI) ─────────────────
    ckp_gen_init();
    scope_inputs_init();

    print_help();
    Serial.printf("\n  CKP: %lu RPM (TIM5)  Modo scope: LIVE\n\n", (unsigned long)g_rpm);
}

void loop() {
    process_events();
    ckp_slew_tick();
    profile_tick();

    while (Serial.available()) {
        const char c = (char)Serial.read();

        // Accumulate text commands terminated by '\n' or '\r'
        if (c == '\n' || c == '\r') {
            if (g_line_pos > 0) {
                g_line[g_line_pos] = '\0';
                parse_text_cmd(g_line);
                g_line_pos = 0;
            }
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
                          (unsigned long)g_rpm, (unsigned long)T_us,
                          (unsigned long)g_rev_count, (unsigned long)g_cmp_count);
        }

        if (new_rpm != g_rpm) {
            ckp_set_rpm(new_rpm);   // o esp32_combined só atualizava g_rpm aqui e o gerador ignorava
            g_sim.rpm = g_rpm;
            const uint32_t T_us = 60000000UL / (g_rpm * 60UL);
            Serial.printf("  [GEN] RPM=%lu  T=%lu µs\n", (unsigned long)g_rpm, (unsigned long)T_us);
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
