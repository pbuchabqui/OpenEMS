/**
 * @file hal/stm32h562/adc.cpp
 * @brief ADC1 + ADC2 com trigger via TIM6 TRGO — STM32H562RGT6
 *        Substitui hal/adc.cpp da versão Kinetis.
 *
 * Mapeamento de canais:
 *
 *   ADC1 (equivale ao ADC0 do Kinetis):
 *     MAP_SE10   → ADC1_IN3  (PA2)
 *     MAF_V_SE11 → ADC1_IN4  (PA3)
 *     TPS_SE12   → ADC1_IN5  (PA4)
 *     O2_SE4B    → ADC1_IN6  (PA5)
 *     AN1_SE6B   → ADC1_IN7  (PB0)
 *     AN2_SE7B   → ADC1_IN8  (PB1)
 *     AN3_SE8B   → ADC1_IN9  (PC0)
 *     AN4_SE9B   → ADC1_IN10 (PC1)
 *
 *   ADC2 (equivale ao ADC1 do Kinetis):
 *     CLT_SE14       → ADC2_IN1 (PC2)
 *     IAT_SE15       → ADC2_IN2 (PC3)
 *     FUEL_PRESS_SE5B → ADC2_IN3 (PA6 — cuidado: compartilhado com TIM3_CH1)
 *                       NOTA: usar PC4 (ADC2_IN13) para evitar conflito com PWM
 *     OIL_PRESS_SE6B  → ADC2_IN4 (PA7 — conflito com TIM3_CH2)
 *                       NOTA: usar PC5 (ADC2_IN14) para evitar conflito
 *
 * Trigger: TIM6 TRGO (Update Event) disparado pela ckp adc_pdb_on_tooth().
 *   TIM6 → prescaler configura período → TRGO → ADC1 + ADC2 disparo simultâneo
 *
 * Resolução: 12 bits (RES=00)
 * Amostragem: 47.5 ciclos ADC (melhor SNR para sensores de temperatura)
 * Clock ADC: HCLK/4 = 62.5 MHz (PRESC[3:0] = 0010)
 */

#ifndef EMS_HOST_TEST

#include "hal/adc.h"
#include "hal/stm32h562/regs.h"

// ── Cache das últimas leituras ADC ───────────────────────────────────────────
static volatile uint16_t g_adc1_raw[8] = {};  // canais ADC1 IN3-IN10
static volatile uint16_t g_adc2_raw[4] = {};  // canais ADC2 IN1-IN4

// ── Índices para acumulação EOC-driven (um por canal por ISR) ─────────────────
static volatile uint8_t g_adc1_idx = 0u;
static volatile uint8_t g_adc2_idx = 0u;

// ── Mapeamento Adc0Channel → índice do array g_adc1_raw ─────────────────────
static constexpr uint8_t kAdc1ChMap[8] = {
    // Adc0Channel::MAP_SE10   → ADC1_IN3  → índice 0
    // Adc0Channel::MAF_V_SE11 → ADC1_IN4  → índice 1
    // Adc0Channel::TPS_SE12   → ADC1_IN5  → índice 2
    // Adc0Channel::O2_SE4B    → ADC1_IN6  → índice 3
    // Adc0Channel::AN1_SE6B   → ADC1_IN7  → índice 4
    // Adc0Channel::AN2_SE7B   → ADC1_IN8  → índice 5
    // Adc0Channel::AN3_SE8B   → ADC1_IN9  → índice 6
    // Adc0Channel::AN4_SE9B   → ADC1_IN10 → índice 7
    3, 4, 5, 6, 7, 8, 9, 10
};

static constexpr uint8_t kAdc2ChMap[4] = {
    // Adc1Channel::CLT_SE14        → ADC2_IN1 → índice 0
    // Adc1Channel::IAT_SE15        → ADC2_IN2 → índice 1
    // Adc1Channel::FUEL_PRESS_SE5B → ADC2_IN13 (PC4) → índice 2
    // Adc1Channel::OIL_PRESS_SE6B  → ADC2_IN14 (PC5) → índice 3
    1, 2, 13, 14
};

// ── Tempo de amostragem para todos os canais: 47.5 ciclos = 011b ─────────────
static constexpr uint32_t kSmpr = 0x03u;  // SMP[2:0] = 011 → 47.5 ciclos

// ── Sequência de conversão ADC1 (8 canais em sequência) ─────────────────────
// SQR1: L[3:0] = 7 (8 conversões - 1), SQ1-SQ4 nos bits [10:6],[16:12],[22:18],[28:24]
static constexpr uint32_t kAdc1Sqr1 = (7u << 0)    // L = 7 (8 conv)
                                     | (3u << 6)    // SQ1 = IN3
                                     | (4u << 12)   // SQ2 = IN4
                                     | (5u << 18)   // SQ3 = IN5
                                     | (6u << 24);  // SQ4 = IN6
static constexpr uint32_t kAdc1Sqr2 = (7u << 0)    // SQ5 = IN7
                                     | (8u << 6)    // SQ6 = IN8
                                     | (9u << 12)   // SQ7 = IN9
                                     | (10u << 18); // SQ8 = IN10

// ── Sequência de conversão ADC2 (4 canais) ───────────────────────────────────
static constexpr uint32_t kAdc2Sqr1 = (3u << 0)    // L = 3 (4 conv)
                                     | (1u << 6)    // SQ1 = IN1
                                     | (2u << 12)   // SQ2 = IN2
                                     | (13u << 18)  // SQ3 = IN13
                                     | (14u << 24); // SQ4 = IN14

namespace ems::hal {

// ── Funções auxiliares ───────────────────────────────────────────────────────

static void adc_wait_ready(volatile uint32_t& isr) noexcept {
    constexpr uint32_t kTimeout = 1000000u;
    for (uint32_t i = 0u; i < kTimeout; ++i) {
        if (isr & ADC_ISR_ADRDY) { break; }
    }
}

static void adc_calibrate_and_enable(volatile uint32_t& cr,
                                      volatile uint32_t& isr) noexcept {
    // 1. Sair de deep power-down e habilitar regulador interno
    cr &= ~ADC_CR_DEEPPWD;
    cr |= ADC_CR_ADVREGEN;
    // Aguardar estabilização do regulador (~20 µs)
    for (volatile uint32_t i = 0u; i < 5000u; ++i) { (void)i; }

    // 2. Calibração single-ended
    cr &= ~ADC_CR_ADCALDIF;  // single-ended (não diferencial)
    cr |= ADC_CR_ADCAL;
    while (cr & ADC_CR_ADCAL) { /* aguarda */ }

    // 3. Habilitar ADC
    isr = ADC_ISR_ADRDY;     // limpa ADRDY antes de habilitar
    cr |= ADC_CR_ADEN;
    adc_wait_ready(isr);
}

// ── API pública ───────────────────────────────────────────────────────────────

void adc_init() noexcept {
    // ── 1. Habilitar clocks ADC e GPIOs ─────────────────────────────────
    RCC_AHB1ENR |= RCC_AHB1ENR_ADC12EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOCEN;  // GPIOC para PC0-PC5

    // ── 2. Configurar pinos analógicos (MODER = 11b = ANALOG) ────────────
    // ADC1: PA2(IN3), PA3(IN4), PA4(IN5), PA5(IN6), PB0(IN7), PB1(IN8)
    gpio_set_analog(&GPIOA_MODER, 2u);
    gpio_set_analog(&GPIOA_MODER, 3u);
    gpio_set_analog(&GPIOA_MODER, 4u);
    gpio_set_analog(&GPIOA_MODER, 5u);
    gpio_set_analog(&GPIOB_MODER, 0u);
    gpio_set_analog(&GPIOB_MODER, 1u);
    // PC0(IN9), PC1(IN10), PC2(IN1), PC3(IN2), PC4(IN13), PC5(IN14)
    volatile uint32_t* gpioc_moder = reinterpret_cast<volatile uint32_t*>(
        GPIOC_BASE + GPIO_MODER_OFF);
    for (uint8_t p = 0u; p <= 5u; ++p) {
        gpio_set_analog(gpioc_moder, p);
    }

    // ── 3. Clock ADC: HCLK/4 = 62.5 MHz ─────────────────────────────────
    ADC12_CCR = ADC12_CCR_PRESC_DIV4;

    // ── 4. Calibrar e habilitar ADC1 ─────────────────────────────────────
    adc_calibrate_and_enable(ADC1_CR, ADC1_ISR);

    // Tempo de amostragem: 47.5 ciclos para todos os canais (SMPR1 bits [29:0])
    ADC1_SMPR1 = (kSmpr)       // IN1
               | (kSmpr << 3)  // IN2
               | (kSmpr << 6)  // IN3 (MAP)
               | (kSmpr << 9)  // IN4 (MAF)
               | (kSmpr << 12) // IN5 (TPS)
               | (kSmpr << 15) // IN6 (O2)
               | (kSmpr << 18) // IN7 (AN1)
               | (kSmpr << 21) // IN8 (AN2)
               | (kSmpr << 24) // IN9 (AN3)
               | (kSmpr << 27);// IN10 (AN4)

    // Sequência de conversão ADC1
    ADC1_SQR1 = kAdc1Sqr1;
    ADC1_SQR2 = kAdc1Sqr2;

    // CFGR1: 12-bit, trigger externo TIM6_TRGO, rising edge
    ADC1_CFGR1 = ADC_CFGR1_RES_12BIT
               | ADC_CFGR1_EXTSEL_TIM6_TRGO
               | ADC_CFGR1_EXTEN_RISING;

    // ── 5. Calibrar e habilitar ADC2 ─────────────────────────────────────
    adc_calibrate_and_enable(ADC2_CR, ADC2_ISR);

    ADC2_SMPR1 = (kSmpr)
               | (kSmpr << 3)  // IN1 (CLT)
               | (kSmpr << 6)  // IN2 (IAT)
               | (kSmpr << 9)  // IN3
               | (kSmpr << 12);// IN4

    ADC2_SMPR2 = (kSmpr << 9)  // IN13 (FUEL_PRESS) — bits [(13-10)*3+offset]
               | (kSmpr << 12);// IN14 (OIL_PRESS)

    ADC2_SQR1 = kAdc2Sqr1;

    // ADC2: trigger TIM6_TRGO simultâneo
    ADC2_CFGR1 = ADC_CFGR1_RES_12BIT
               | ADC_CFGR1_EXTSEL_TIM6_TRGO
               | ADC_CFGR1_EXTEN_RISING;

    // ── 6. Configurar TIM6 como gerador de TRGO ───────────────────────────
    RCC_APB1LENR |= RCC_APB1LENR_TIM6EN;
    TIM6_CR1 = 0u;
    TIM6_CR2 = TIM_CR2_MMS_UPDATE;  // MMS = 010 → TRGO on Update Event
    // Período padrão: 1 ms (atualizado por adc_pdb_on_tooth)
    TIM6_PSC = 0u;
    TIM6_ARR = static_cast<uint32_t>(kTimClockHz / 1000u) - 1u;  // ~1 ms
    TIM6_EGR = 1u;
    TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_URS;  // URS: apenas overflow gera TRGO

    // ── 7. Habilitar interrupções EOC+EOS e NVIC ─────────────────────────
    // EOC dispara 1× por canal convertido; EOS dispara ao fim da sequência.
    ADC1_IER = ADC_IER_EOCIE | ADC_IER_EOSIE;
    ADC2_IER = ADC_IER_EOCIE | ADC_IER_EOSIE;
    // IRQ_ADC1=37 cobre ambos ADC1 e ADC2 (prioridade 5 = mesmo que Kinetis ADC0)
    nvic_set_priority(IRQ_ADC1, 5u);
    nvic_enable_irq(IRQ_ADC1);

    // ── 8. Disparar primeira conversão para popular o cache ──────────────
    ADC1_CR |= ADC_CR_ADSTART;
    ADC2_CR |= ADC_CR_ADSTART;
}

void adc_pdb_on_tooth(uint16_t tooth_period_ticks) noexcept {
    // Equivalente ao adc_pdb_on_tooth() do Kinetis:
    // Ajusta período do TIM6 para que o ADC seja amostrado a cada dente CKP.
    // tooth_period_ticks está em ticks de 62.5 MHz (TIM5).
    // TIM6 opera a kTimClockHz; converte diretamente.
    if (tooth_period_ticks == 0u) { return; }
    // Amostrar na metade do período do dente (equivalente ao delay do PDB)
    const uint32_t arr = (tooth_period_ticks / 2u);
    if (arr > 0u) {
        TIM6_ARR = arr - 1u;
        TIM6_EGR = 1u;  // UG: força reload imediato
    }
}

uint16_t adc0_read(Adc0Channel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx >= 8u) { return 0u; }
    return g_adc1_raw[idx];
}

uint16_t adc1_read(Adc1Channel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx >= 4u) { return 0u; }
    return g_adc2_raw[idx];
}

} // namespace ems::hal

// ── ISR ADC — acumula resultados canal a canal via EOC ───────────────────────
// EOC dispara 1× por canal em scan mode; leitura de ADC_DR limpa EOC automaticamente.
// EOS dispara ao fim da sequência completa → reinicia índice e re-arma.
extern "C" void ADC1_2_IRQHandler(void) {
    const uint32_t sr1 = ems::hal::ADC1_ISR;
    const uint32_t sr2 = ems::hal::ADC2_ISR;

    // ADC1: acumula 1 canal por EOC (8 canais: IN3-IN10)
    if (sr1 & ems::hal::ADC_ISR_EOC) {
        if (g_adc1_idx < 8u) {
            g_adc1_raw[g_adc1_idx++] =
                static_cast<uint16_t>(ems::hal::ADC1_DR & 0xFFFu);  // leitura limpa EOC
        }
    }
    if (sr1 & ems::hal::ADC_ISR_EOS) {
        g_adc1_idx = 0u;
        ems::hal::ADC1_ISR = ems::hal::ADC_ISR_EOS;   // limpa EOS (W0C)
        ems::hal::ADC1_CR |= ems::hal::ADC_CR_ADSTART; // re-arm sequência
    }

    // ADC2: acumula 1 canal por EOC (4 canais: IN1, IN2, IN13, IN14)
    if (sr2 & ems::hal::ADC_ISR_EOC) {
        if (g_adc2_idx < 4u) {
            g_adc2_raw[g_adc2_idx++] =
                static_cast<uint16_t>(ems::hal::ADC2_DR & 0xFFFu);
        }
    }
    if (sr2 & ems::hal::ADC_ISR_EOS) {
        g_adc2_idx = 0u;
        ems::hal::ADC2_ISR = ems::hal::ADC_ISR_EOS;
        ems::hal::ADC2_CR |= ems::hal::ADC_CR_ADSTART;
    }
}

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/adc.h"
namespace ems::hal {
static uint16_t g_adc0[8] = {};
static uint16_t g_adc1[4] = {};
static uint16_t g_last_pdb_mod = 0u;
void     adc_init() noexcept {}
void     adc_pdb_on_tooth(uint16_t t) noexcept { g_last_pdb_mod = t; }
uint16_t adc0_read(Adc0Channel ch) noexcept { return g_adc0[static_cast<uint8_t>(ch)]; }
uint16_t adc1_read(Adc1Channel ch) noexcept { return g_adc1[static_cast<uint8_t>(ch)]; }
void adc_test_set_raw_adc0(Adc0Channel ch, uint16_t v) noexcept { g_adc0[static_cast<uint8_t>(ch)] = v; }
void adc_test_set_raw_adc1(Adc1Channel ch, uint16_t v) noexcept { g_adc1[static_cast<uint8_t>(ch)] = v; }
uint16_t adc_test_last_pdb_mod() noexcept { return g_last_pdb_mod; }
} // namespace ems::hal

#endif  // EMS_HOST_TEST
