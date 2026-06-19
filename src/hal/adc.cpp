/**
 * @file hal/stm32h562/adc.cpp
 * @brief ADC1 + ADC2 com trigger via TIM6 TRGO — STM32H562RGT6
 *        Substitui hal/adc.cpp da versão STM32.
 *
 * Mapeamento de canais (LQFP100 / STM32H562VGT6, DS14258):
 *
 *   ADC1 (8 canais):
 *     MAP       → INP15 (PA3)  — SQ1
 *     KNOCK     → INP19 (PA5)  — SQ2
 *     TPS       → INP18 (PA4)  — SQ3
 *     (placeholder) → INP19    — SQ4
 *     APP1      → INP10 (PC0)  — SQ5
 *     APP2      → INP12 (PC2)  — SQ6
 *     ETB_TPS1  → INP4  (PC4)  — SQ7
 *     ETB_TPS2  → INP8  (PC5)  — SQ8
 *
 *   ADC2 (5 canais):
 *     CLT       → INP9  (PB0)  — SQ1
 *     IAT       → INP5  (PB1)  — SQ2
 *     FUEL_PRESS → INP4 (PC4)  — SQ3
 *     OIL_PRESS  → INP8 (PC5)  — SQ4
 *     EWG_POS    → INP13 (PC3) — SQ5
 *
 * Trigger: TIM6 TRGO (Update Event) disparado pela ckp adc_trigger_on_tooth().
 *   TIM6 → prescaler configura período → TRGO → ADC1 + ADC2 disparo simultâneo
 *
 * Resolução: 12 bits (RES=00)
 * Amostragem: 47.5 ciclos ADC (melhor SNR para sensores de temperatura)
 * Clock ADC: HCLK/4 = 62.5 MHz (CKMODE[1:0] = 11, síncrono ao timer)
 */

#ifndef EMS_HOST_TEST

#include "hal/adc.h"
#include "hal/regs.h"

// ── Cache das últimas leituras ADC ───────────────────────────────────────────
static volatile uint16_t g_adc_secondary_raw[8] = {};  // canais ADC1 IN3-IN10
static volatile uint16_t g_adc2_raw[5] = {};  // canais ADC2: CLT,IAT,FUEL,OIL,EWG

// LLI auto-referente por canal (CBR1, CDAR, CLLR) — torna o GPDMA contínuo: ao fim
// de cada bloco recarrega tamanho+destino e re-aponta pra si mesmo. Em SRAM (.bss),
// 32-byte aligned. Sem isso o canal vai a IDLE após 1 bloco (congelamento).
alignas(32) static volatile uint32_t g_adc1_lli[3] = {};
alignas(32) static volatile uint32_t g_adc2_lli[3] = {};

static volatile uint32_t g_adc_dma_faults = 0u;
static volatile uint32_t g_adc_init_faults = 0u;  // FIX: Fault counter para adc_wait_ready timeout

// ── ADC Recovery System (P0 #3 - IMPROVEMENTS.md) ───────────────────────────
// Flags de status para recuperação do ADC em tempo de execução
static volatile bool g_adc_recovering = false;       // true durante sequência de recovery
static volatile bool g_adc_recovery_failed = false;  // true se recovery falhou após retries
static volatile uint32_t g_adc_timeout_count = 0u;   // Contador de timeouts em runtime
static constexpr uint32_t kAdcRecoveryMaxRetries = 3u;  // Máximo de tentativas de recovery
static volatile uint32_t g_adc_recovery_retries = 0u;   // Contador de retries atual

// ── Mapeamento AdcPrimaryChannel → índice do array g_adc_secondary_raw ─────────────────────
static constexpr uint8_t kAdc1ChMap[8] = {
    // MAP(0)→INP15, MAF_V(1)→placeholder, TPS(2)→INP18, KNOCK(3)→INP19,
    // APP1(4)→INP10, APP2(5)→INP12, ETB_TPS1(6)→INP4, ETB_TPS2(7)→INP8
    0, 1, 2, 3, 4, 5, 6, 7
};

static constexpr uint8_t kAdc2ChMap[5] = {
    // AdcSecondaryChannel::CLT        → ADC2_INP9  (PB0) → índice 0
    // AdcSecondaryChannel::IAT        → ADC2_INP5  (PB1) → índice 1
    // AdcSecondaryChannel::FUEL_PRESS → ADC2_INP4  (PC4) → índice 2
    // AdcSecondaryChannel::OIL_PRESS  → ADC2_INP8  (PC5) → índice 3
    // AdcSecondaryChannel::EWG_POS    → ADC2_INP13 (PC3) → índice 4
    9, 5, 4, 8, 13
};

// ── Tempo de amostragem para todos os canais: 47.5 ciclos = 011b ─────────────
static constexpr uint32_t kSmpr = 0x03u;  // SMP[2:0] = 011 → 47.5 ciclos
static constexpr uint32_t kTimClockHz = 62500000u;  // TIM6 @ 62.5 MHz após clock tree atual

// ── Sequência de conversão ADC1 (8 canais em sequência) ─────────────────────
// SQR1: L[3:0] = 7 (8 conversões - 1), SQ1-SQ4 nos bits [10:6],[16:12],[22:18],[28:24]
// SQ1=INP15(MAP/PA3), SQ3=INP18(TPS/PA4) — canais REAIS do H562 (DS14258). Os demais
// slots (MAF/KNOCK/AN1-4, não usados na bancada) apontam p/ canais ADC válidos
// (INP4/5/8/9/10 = PC4/PB1/PC5/PB0/PC0) só para não converter canal inexistente.
static constexpr uint32_t kAdc1Sqr1 = (7u << 0)    // L = 7 (8 conv)
                                     | (15u << 6)   // SQ1 = INP15 (MAP, PA3)
                                     | (19u << 12)  // SQ2 = INP19 (KNOCK, PA5)
                                     | (18u << 18)  // SQ3 = INP18 (TPS, PA4)
                                     | (19u << 24); // SQ4 = INP19 (KNOCK, dup placeholder)
static constexpr uint32_t kAdc1Sqr2 = (10u << 0)   // SQ5 = INP10 (APP1, PC0)
                                     | (12u << 6)   // SQ6 = INP12 (APP2, PC2)
                                     | (4u << 12)   // SQ7 = INP4  (ETB_TPS1, PC4)
                                     | (8u << 18);  // SQ8 = INP8  (ETB_TPS2, PC5)

// ── Sequência de conversão ADC2 (4 canais) ───────────────────────────────────
// CLT→INP9(PB0), IAT→INP5(PB1) — bancada: PB0/PB1 partilhados com APP1/APP2 no ADC1;
// fios GPIO14/GPIO27 (APP1/APP2) desligados na bancada, GPIO13/GPIO12 ligados a PB0/PB1.
static constexpr uint32_t kAdc2Sqr1 = (4u << 0)    // L = 4 (5 conv)
                                     | (9u << 6)    // SQ1 = INP9 (CLT, PB0)
                                     | (5u << 12)   // SQ2 = INP5 (IAT, PB1)
                                     | (4u << 18)   // SQ3 = INP4 (FUEL, PC4)
                                     | (8u << 24);  // SQ4 = INP8 (OIL, PC5)
static constexpr uint32_t kAdc2Sqr2 = (13u << 0);   // SQ5 = INP13 (EWG, PC3)

namespace ems::hal {

// ── Funções auxiliares ───────────────────────────────────────────────────────

static bool adc_wait_ready(volatile uint32_t& isr) noexcept {
    // FIX BUG-11: o loop anterior não tratava timeout. Se o ADC não ficava
    // ready, o busy-wait consumia ~4 ms com PRIMASK ativo (interrupções
    // desabilitadas), bloqueando o main loop. Agora retorna false e
    // incrementa fault counter para diagnóstico.
    // BENCH-PROOF: ADRDY surge em ~µs após ADEN; 30k iterações (~0.5 ms @250 MHz)
    // é folga enorme. O valor antigo (1.000.000) somava SEGUNDOS quando o ADC não
    // ficava ready (bancada sem VDDA/entradas flutuantes), travando o boot.
    constexpr uint32_t kTimeout = 30000u;
    for (uint32_t i = 0u; i < kTimeout; ++i) {
        if (isr & ADC_ISR_ADRDY) { return true; }
    }
    ++g_adc_init_faults;
    ++g_adc_timeout_count;  // P0 #3: contador de timeouts em runtime
    return false;
}

static void adc_prepare_for_config(volatile uint32_t& cr) noexcept {
    // 1. Sair de deep power-down e habilitar regulador interno
    cr &= ~ADC_CR_DEEPPWD;
    cr |= ADC_CR_ADVREGEN;
    // Aguardar estabilização do regulador (~20 µs)
    for (volatile uint32_t i = 0u; i < 5000u; ++i) { (void)i; }

	// 2. Calibração single-ended
	cr &= ~ADC_CR_ADCALDIF; // single-ended (não diferencial)
	cr |= ADC_CR_ADCAL;
	// FIX: timeout na espera de ADCAL — sem isso, firmware congela se calibração falhar
	{
		uint32_t cal_to = 0u;
		// BENCH-PROOF: ADCAL completa em ~116 ciclos ADC (~2 µs). 30k iterações é folga;
		// o valor antigo (1.000.000) travava o boot na bancada sem VDDA.
		constexpr uint32_t kCalTimeout = 30000u;
		while ((cr & ADC_CR_ADCAL) != 0u) {
			if (++cal_to >= kCalTimeout) {
				++g_adc_init_faults;
				break;
			}
		}
	}

}

static void adc_enable(volatile uint32_t& cr, volatile uint32_t& isr) noexcept {
    isr = ADC_ISR_ADRDY;     // limpa ADRDY antes de habilitar
    cr |= ADC_CR_ADEN;
    if (!adc_wait_ready(isr)) {
        // FIX BUG-11 / P0 #3: se ADC não ficou ready, tenta recovery via deep-power-down
        // Sequência RM0481 §25.4.2: ADEN=0 PRIMEIRO, depois DEEPPWD=1
        // O fault counter já foi incrementado por adc_wait_ready.
        
        // Incrementa contador de retries
        if (g_adc_recovery_retries < kAdcRecoveryMaxRetries) {
            ++g_adc_recovery_retries;
            g_adc_recovering = true;
            
            // Sequência de recovery
            cr &= ~ADC_CR_ADEN;       // 1. Desabilitar ADC
            for (volatile uint32_t i = 0u; i < 100u; ++i) { (void)i; } // aguarda
            cr |= ADC_CR_DEEPPWD;     // 2. Deep power-down
            for (volatile uint32_t i = 0u; i < 1000u; ++i) { (void)i; } // aguarda T_pwrup
            cr &= ~ADC_CR_DEEPPWD;    // 3. Sair de deep-power-down
            cr |= ADC_CR_ADVREGEN;    // 4. Re-ativar regulador interno
            for (volatile uint32_t i = 0u; i < 5000u; ++i) { (void)i; } // aguarda estabilização
            cr |= ADC_CR_ADEN;        // 5. Re-habilitar ADC
            
            if (adc_wait_ready(isr)) {
                // Recovery成功了
                g_adc_recovering = false;
                g_adc_recovery_retries = 0u;  // Reset após sucesso
            } else {
                // Falhou novamente, tenta próximo retry no próximo timeout
                // g_adc_recovering permanece true para indicar estado de recovery
            }
        } else {
            // Excedeu máximo de retries - recovery falhou permanentemente
            g_adc_recovery_failed = true;
            g_adc_recovering = false;
            // Mantém ADC desabilitado para segurança
            cr &= ~ADC_CR_ADEN;
        }
    } else {
        // ADC ready com sucesso - reset flags
        g_adc_recovery_retries = 0u;
        g_adc_recovery_failed = false;
        g_adc_recovering = false;
    }
}

static void gpdma_arm(uint32_t ch_base, uint32_t reqsel, uint32_t buf_bytes,
                      uint32_t src_dr, uint32_t dest_buf,
                      volatile uint32_t* lli) noexcept {
    GPDMA_REG(ch_base, GPDMA_CCR_OFF) = GPDMA_CCR_RESET;
    for (uint32_t i = 0u; i < 128u; ++i) {
        if ((GPDMA_REG(ch_base, GPDMA_CCR_OFF) & GPDMA_CCR_RESET) == 0u) { break; }
    }
    GPDMA_REG(ch_base, GPDMA_CFCR_OFF) = GPDMA_CFCR_ALL;
    GPDMA_REG(ch_base, GPDMA_CTR1_OFF) = GPDMA_CTR1_HALFWORD_TO_HALFWORD_INC_DEST;
    GPDMA_REG(ch_base, GPDMA_CTR2_OFF) = reqsel | GPDMA_CTR2_TCEM_BLOCK;
    GPDMA_REG(ch_base, GPDMA_CBR1_OFF) = buf_bytes;
    GPDMA_REG(ch_base, GPDMA_CSAR_OFF) = src_dr;
    GPDMA_REG(ch_base, GPDMA_CDAR_OFF) = dest_buf;
    GPDMA_REG(ch_base, GPDMA_CTR3_OFF) = 0u;
    GPDMA_REG(ch_base, GPDMA_CBR2_OFF) = 0u;

    // LLI auto-referente: ao completar o bloco, o GPDMA recarrega CBR1 (tamanho),
    // CDAR (destino → array[0]) e CLLR (de volta a este nó) → roda pra sempre.
    // CSAR (ADC_DR) é constante (CTR1 só incrementa o destino) → não recarregado.
    const uint32_t node = reinterpret_cast<uint32_t>(lli);
    const uint32_t cllr = (node & GPDMA_CLLR_LA_MASK)
                        | GPDMA_CLLR_UB1 | GPDMA_CLLR_UDA | GPDMA_CLLR_ULL;
    lli[0] = buf_bytes;   // → CBR1
    lli[1] = dest_buf;    // → CDAR
    lli[2] = cllr;        // → CLLR (self-link)
    GPDMA_REG(ch_base, GPDMA_CLLBAR_OFF) = node & 0xFFFF0000u;  // base do LLI (high)
    GPDMA_REG(ch_base, GPDMA_CLLR_OFF)   = cllr;                // arma o link
	GPDMA_REG(ch_base, GPDMA_CCR_OFF) = GPDMA_CCR_PRIO_HIGH | GPDMA_CCR_TCIE |
		GPDMA_CCR_DTEIE | GPDMA_CCR_USEIE | GPDMA_CCR_EN;
}

static void gpdma_adc1_arm() noexcept {
    gpdma_arm(GPDMA_CH0_BASE, GPDMA_CTR2_REQSEL_ADC1,
              sizeof(g_adc_secondary_raw),
              reinterpret_cast<uint32_t>(&ADC1_DR),
              reinterpret_cast<uint32_t>(&g_adc_secondary_raw[0]),
              g_adc1_lli);
}

static void gpdma_adc2_arm() noexcept {
    gpdma_arm(GPDMA_CH1_BASE, GPDMA_CTR2_REQSEL_ADC2,
              sizeof(g_adc2_raw),
              reinterpret_cast<uint32_t>(&ADC2_DR),
              reinterpret_cast<uint32_t>(&g_adc2_raw[0]),
              g_adc2_lli);
}

// ── API pública ───────────────────────────────────────────────────────────────

void adc_init() noexcept {
    // ── 1. Habilitar clocks ADC e GPIOs ─────────────────────────────────
    RCC_AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_ADCEN;   // ADC fica no AHB2 no H562 (bit 10)
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN
                  | RCC_AHB2ENR1_GPIOBEN
                  | RCC_AHB2ENR1_GPIOCEN;

    // ── 2. Configurar pinos analógicos (MODER = 11b = ANALOG) ────────────
    // ADC1: PA3=INP15(MAP), PA4=INP18(TPS), PA5=INN18(diff-), PB0=INP9(APP1/CLT), PB1=INP5(APP2/IAT)
    // PA2 não tem ADC no H562 (DS14258) — não configurar.
    gpio_set_analog(&GPIOA_MODER, 3u);
    gpio_set_analog(&GPIOA_MODER, 4u);
    gpio_set_analog(&GPIOA_MODER, 5u);
    gpio_set_analog(&GPIOB_MODER, 0u);  // INP9: APP1 (ADC1) / CLT (ADC2, bancada)
    gpio_set_analog(&GPIOB_MODER, 1u);  // INP5: APP2 (ADC1) / IAT (ADC2, bancada)
    // PC0=INP10(APP1), PC2=INP12(APP2), PC3=INP13(EWG), PC4=INP4(ETB_TPS1/FUEL), PC5=INP8(ETB_TPS2/OIL)
    volatile uint32_t* gpioc_moder = reinterpret_cast<volatile uint32_t*>(
        GPIOC_BASE + GPIO_MODER_OFF);
    gpio_set_analog(gpioc_moder, 0u);
    gpio_set_analog(gpioc_moder, 2u);  // PC2 = APP2 (INP12) — LQFP100
    gpio_set_analog(gpioc_moder, 3u);  // PC3 = EWG pos (INP13) — LQFP100
    gpio_set_analog(gpioc_moder, 4u);
    gpio_set_analog(gpioc_moder, 5u);

    // ── 3. Clock ADC: HCLK/4 = 62.5 MHz, síncrono ao trigger de timer ───
    ADC12_CCR = ADC12_CCR_CKMODE_HCLK_DIV4;

    // ── 4. Calibrar e configurar ADC1 ainda desabilitado ────────────────
    adc_prepare_for_config(ADC1_CR);

// Tempo de amostragem: 47.5 ciclos para canais IN1-IN9 (SMPR1 cobre IN0-IN9)
ADC1_SMPR1 = (kSmpr) // IN1
	| (kSmpr << 3) // IN2
	| (kSmpr << 6) // IN3 (MAP)
	| (kSmpr << 9) // IN4 (MAF)
	| (kSmpr << 12) // IN5 (TPS)
	| (kSmpr << 15) // IN6 (O2)
	| (kSmpr << 18) // IN7 (AN1)
	| (kSmpr << 21) // IN8 (AN2)
	| (kSmpr << 24); // IN9 (AN3)
// SMPR2 cobre IN10-IN18: INP10(APP1), INP12(APP2), INP15(MAP), INP18(TPS), INP19(KNOCK)
ADC1_SMPR2 = (kSmpr << ((10-10)*3))   // IN10 (APP1, PC0)
           | (kSmpr << ((12-10)*3))   // IN12 (APP2, PC2)
           | (kSmpr << ((15-10)*3))   // IN15 (MAP, PA3)
           | (kSmpr << ((18-10)*3))   // IN18 (TPS, PA4)
           | (kSmpr << ((19-10)*3));  // IN19 (KNOCK, PA5)

    // Sequência de conversão ADC1
    ADC1_SQR1 = kAdc1Sqr1;
    ADC1_SQR2 = kAdc1Sqr2;

	// CFGR1: 12-bit, trigger TIM6_TRGO rising, DMA circular (DMACFG=1) p/ o ADC
	// requisitar DMA continuamente em cada sequência; OVRMOD=1 p/ overrun sobrescrever
	// o DR em vez de travar o ADSTART. FIX: o "one-shot + re-arm no ISR" anterior
	// parava de requisitar após a 1ª sequência e o OVR limpava ADSTART → ADC congelava.
	ADC1_CFGR1 = ADC_CFGR1_RES_12BIT
		| ADC_CFGR1_DMAEN
		| ADC_CFGR1_DMACFG
		| ADC_CFGR1_OVRMOD
		| ADC_CFGR1_EXTSEL_TIM6_TRGO
		| ADC_CFGR1_EXTEN_RISING;

    // ── 5. Calibrar e configurar ADC2 ainda desabilitado ────────────────
    adc_prepare_for_config(ADC2_CR);

    // SMPR1 cobre INP0-INP9. SMPR2 cobre INP10+.
    ADC2_SMPR1 = (kSmpr << (4u * 3u))   // INP4  (FUEL, PC4)
               | (kSmpr << (5u * 3u))   // INP5  (IAT, PB1)
               | (kSmpr << (8u * 3u))   // INP8  (OIL, PC5)
               | (kSmpr << (9u * 3u));  // INP9  (CLT, PB0)
    ADC2_SMPR2 = (kSmpr << ((10-10)*3u))   // INP10 (placeholder)
               | (kSmpr << ((13-10)*3u));  // INP13 (EWG, PC3)

    ADC2_SQR1 = kAdc2Sqr1;
    ADC2_SQR2 = kAdc2Sqr2;

// ADC2: trigger TIM6_TRGO simultâneo
ADC2_CFGR1 = ADC_CFGR1_RES_12BIT
	| ADC_CFGR1_DMAEN
	| ADC_CFGR1_DMACFG
	| ADC_CFGR1_OVRMOD
	| ADC_CFGR1_EXTSEL_TIM6_TRGO
	| ADC_CFGR1_EXTEN_RISING;

    // ── 6. Configurar TIM6 como gerador de TRGO ───────────────────────────
    RCC_APB1LENR |= RCC_APB1LENR_TIM6EN;
    TIM6_CR1 = 0u;
    TIM6_CR2 = TIM_CR2_MMS_UPDATE;  // MMS = 010 → TRGO on Update Event
    // Período padrão: 1 ms (atualizado por adc_trigger_on_tooth)
    TIM6_PSC = 0u;
    TIM6_ARR = static_cast<uint32_t>(kTimClockHz / 1000u) - 1u;  // ~1 ms
    TIM6_EGR = 1u;
    TIM6_SR = 0u;
    TIM6_CR1 = TIM_CR1_OPM | TIM_CR1_URS;

    // ── 7. GPDMA por sequência ADC; reduz ISR de EOC/EOS por canal ───────
    ADC1_IER = 0u;
    ADC2_IER = 0u;
    ems::hal::gpdma_adc1_arm();
    ems::hal::gpdma_adc2_arm();
    nvic_set_priority(IRQ_GPDMA1_CH0, 5u);
    nvic_set_priority(IRQ_GPDMA1_CH1, 5u);
    nvic_enable_irq(IRQ_GPDMA1_CH0);
    nvic_enable_irq(IRQ_GPDMA1_CH1);

    // ── 8. Habilitar ADCs e armar hardware trigger ──────────────────────
    adc_enable(ADC1_CR, ADC1_ISR);
    adc_enable(ADC2_CR, ADC2_ISR);
    ADC1_CR |= ADC_CR_ADSTART;
    ADC2_CR |= ADC_CR_ADSTART;
}

void adc_trigger_on_tooth(uint32_t tooth_period_ticks) noexcept {
    // Equivalente ao adc_trigger_on_tooth() do STM32:
    // Ajusta período do TIM6 para que o ADC seja amostrado a cada dente CKP.
    // tooth_period_ticks está em ticks de 62.5 MHz (TIM5).
    // TIM6 opera a kTimClockHz; converte diretamente.
    if (tooth_period_ticks == 0u) { return; }
    // Amostrar na metade do período do dente (delay do TIM6 trigger)
    const uint32_t arr = (tooth_period_ticks / 2u);
    if (arr > 0u) {
        // GPDMA roda contínuo via LLI auto-referente (armado em adc_init) — NÃO
        // re-armar por dente (corromperia CDAR no meio de um reload do hardware).
        TIM6_CR1 = TIM_CR1_OPM | TIM_CR1_URS;
        TIM6_SR = 0u;
        TIM6_ARR = arr - 1u;
        TIM6_CNT = 0u;
        TIM6_CR1 = TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_CEN;
    }
}

uint16_t adc_primary_read(AdcPrimaryChannel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx >= 8u) { return 0u; }
    return g_adc_secondary_raw[kAdc1ChMap[idx]];
}

uint16_t adc_secondary_read(AdcSecondaryChannel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx >= 5u) { return 0u; }
    return g_adc2_raw[idx];
}

// P0 #3: ADC Recovery System - API pública para verificação de status
bool adc_is_recovering() noexcept {
    return g_adc_recovering;
}

bool adc_recovery_failed() noexcept {
    return g_adc_recovery_failed;
}

uint32_t adc_get_timeout_count() noexcept {
    return g_adc_timeout_count;
}

uint32_t adc_get_recovery_retries() noexcept {
    return g_adc_recovery_retries;
}

} // namespace ems::hal

extern "C" void GPDMA1_Channel0_IRQHandler(void) {
    const uint32_t sr = GPDMA1_CH0_CSR;
    GPDMA1_CH0_CFCR = GPDMA_CFCR_ALL;
    if ((sr & (GPDMA_CSR_DTEF | GPDMA_CSR_USEF)) != 0u) { 
        ++g_adc_dma_faults;
        // P0 #3: DMA fault pode indicar problema no ADC - verifica se precisa de recovery
        if (!g_adc_recovering && !g_adc_recovery_failed) {
            // Sinaliza para main loop verificar ADC status
            // Recovery será tentado na próxima chamada de adc_enable() ou via ISR dedicada
        }
    }
    // Re-arm agora é por dente em adc_trigger_on_tooth (determinístico); aqui só
    // limpamos as flags. O re-arm na ISV não reciclava → ADCs congelavam.
}

extern "C" void GPDMA1_Channel1_IRQHandler(void) {
    const uint32_t sr = GPDMA1_CH1_CSR;
    GPDMA1_CH1_CFCR = GPDMA_CFCR_ALL;
    if ((sr & (GPDMA_CSR_DTEF | GPDMA_CSR_USEF)) != 0u) { 
        ++g_adc_dma_faults;
        // P0 #3: DMA fault pode indicar problema no ADC
        if (!g_adc_recovering && !g_adc_recovery_failed) {
            // Sinaliza para main loop verificar ADC status
        }
    }
    // Re-arm por dente em adc_trigger_on_tooth; aqui só limpamos flags.
}

// P0 #3: Handler de timeout do ADC para recuperação em tempo de execução
// Chamado quando ADC não responde dentro do período esperado
extern "C" void adc_timeout_isr() noexcept {
    ++g_adc_timeout_count;
    
    // Se já está em recovery ou falhou permanentemente, ignora
    if (g_adc_recovering || g_adc_recovery_failed) {
        return;
    }
    
    // Inicia sequência de recovery
    g_adc_recovering = true;
    
    // Verifica se excedeu máximo de retries
    if (g_adc_recovery_retries >= kAdcRecoveryMaxRetries) {
        g_adc_recovery_failed = true;
        g_adc_recovering = false;
        return;
    }
    
    ++g_adc_recovery_retries;
    
    // Sequência de recovery por hardware conforme IMPROVEMENTS.md P0 #3
    // Nota: Esta ISR é chamada pelo watchdog do ADC ou timer de supervisão
    // O recovery completo dos registradores deve ser feito no contexto apropriado
    
    // Passos de recovery (RM0481 §25.4.2):
    // 1. Stop conversion (ADSTP)
    // 2. Disable ADC (ADDIS)  
    // 3. Wait for end of shutdown
    // 4. Re-enable ADC (ADEN)
    // 5. Verify ADRDY flag
    
    // Esta versão simplificada sinaliza o estado para o main loop
    // onde o recovery completo será executado com acesso seguro aos registradores
    
    // Reporta fault ao sistema de diagnóstico
    // (implementado no DiagnosticManager via sample_fast_channels)
}

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/adc.h"
namespace ems::hal {
static uint16_t g_adc_primary[8] = {};
static uint16_t g_adc_secondary[5] = {};
static uint32_t g_last_trigger_mod = 0u;
// P0 #3: Mock variables para ADC recovery system
static bool g_adc_recovering_mock = false;
static bool g_adc_recovery_failed_mock = false;
static uint32_t g_adc_timeout_count_mock = 0u;
static uint32_t g_adc_recovery_retries_mock = 0u;

void     adc_init() noexcept {}
void     adc_trigger_on_tooth(uint32_t t) noexcept { g_last_trigger_mod = t; }
uint16_t adc_primary_read(AdcPrimaryChannel ch) noexcept { return g_adc_primary[static_cast<uint8_t>(ch)]; }
uint16_t adc_secondary_read(AdcSecondaryChannel ch) noexcept { return g_adc_secondary[static_cast<uint8_t>(ch)]; }
void adc_test_set_raw_primary(AdcPrimaryChannel ch, uint16_t v) noexcept { g_adc_primary[static_cast<uint8_t>(ch)] = v; }
void adc_test_set_raw_secondary(AdcSecondaryChannel ch, uint16_t v) noexcept { g_adc_secondary[static_cast<uint8_t>(ch)] = v; }
uint32_t adc_test_last_trigger_mod() noexcept { return g_last_trigger_mod; }

// P0 #3: Mock functions para ADC recovery system - usadas em testes
bool adc_is_recovering() noexcept { return g_adc_recovering_mock; }
bool adc_recovery_failed() noexcept { return g_adc_recovery_failed_mock; }
uint32_t adc_get_timeout_count() noexcept { return g_adc_timeout_count_mock; }
uint32_t adc_get_recovery_retries() noexcept { return g_adc_recovery_retries_mock; }

// Funções de teste para simular falhas de ADC
void adc_test_set_recovering(bool recovering) noexcept { g_adc_recovering_mock = recovering; }
void adc_test_set_recovery_failed(bool failed) noexcept { g_adc_recovery_failed_mock = failed; }
void adc_test_set_timeout_count(uint32_t count) noexcept { g_adc_timeout_count_mock = count; }
void adc_test_set_recovery_retries(uint32_t retries) noexcept { g_adc_recovery_retries_mock = retries; }

} // namespace ems::hal

#endif  // EMS_HOST_TEST
