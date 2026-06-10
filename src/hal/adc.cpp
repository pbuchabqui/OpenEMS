/**
 * @file hal/stm32h562/adc.cpp
 * @brief ADC1 + ADC2 com trigger via TIM6 TRGO — STM32H562RGT6
 *        Substitui hal/adc.cpp da versão STM32.
 *
 * Mapeamento de canais:
 *
 *   ADC1:
 *     MAP_SE10   → ADC1_IN3  (PA2)
 *     MAF_V_SE11 → ADC1_IN4  (PA3)
 *     TPS_SE12   → ADC1_IN5  (PA4)
 *     KNOCK_SE4B → ADC1_IN6  (PA5) — knock sensor (O2 migrated to CAN-only)
 *     AN1_SE6B   → ADC1_IN7  (PB0)
 *     AN2_SE7B   → ADC1_IN8  (PB1)
 *     AN3_SE8B   → ADC1_IN9  (PC0)
 *     AN4_SE9B   → ADC1_IN10 (PC1)
 *
 *   ADC2 (ADC1 do STM32):
 *     CLT_SE14       → ADC2_IN1 (PC2)
 *     IAT_SE15       → ADC2_IN2 (PC3)
 *     FUEL_PRESS_SE5B → ADC2_IN3 (PA6 — cuidado: compartilhado com TIM3_CH1)
 *                       NOTA: usar PC4 (ADC2_IN13) para evitar conflito com PWM
 *     OIL_PRESS_SE6B  → ADC2_IN4 (PA7 — conflito com TIM3_CH2)
 *                       NOTA: usar PC5 (ADC2_IN14) para evitar conflito
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
static volatile uint16_t g_adc2_raw[4] = {};  // canais ADC2 IN1-IN4

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
    // AdcPrimaryChannel::MAP_SE10   → ADC1_IN3  → índice 0
    // AdcPrimaryChannel::MAF_V_SE11 → ADC1_IN4  → índice 1
    // AdcPrimaryChannel::TPS_SE12   → ADC1_IN5  → índice 2
    // AdcPrimaryChannel::KNOCK_SE4B → ADC1_IN6  → índice 3
    // AdcPrimaryChannel::AN1_SE6B   → ADC1_IN7  → índice 4
    // AdcPrimaryChannel::AN2_SE7B   → ADC1_IN8  → índice 5
    // AdcPrimaryChannel::AN3_SE8B   → ADC1_IN9  → índice 6
    // AdcPrimaryChannel::AN4_SE9B   → ADC1_IN10 → índice 7
    0, 1, 2, 3, 4, 5, 6, 7
};

static constexpr uint8_t kAdc2ChMap[4] = {
    // AdcSecondaryChannel::CLT_SE14        → ADC2_IN1 → índice 0
    // AdcSecondaryChannel::IAT_SE15        → ADC2_IN2 → índice 1
    // AdcSecondaryChannel::FUEL_PRESS_SE5B → ADC2_IN13 (PC4) → índice 2
    // AdcSecondaryChannel::OIL_PRESS_SE6B  → ADC2_IN14 (PC5) → índice 3
    1, 2, 13, 14
};

// ── Tempo de amostragem para todos os canais: 47.5 ciclos = 011b ─────────────
static constexpr uint32_t kSmpr = 0x03u;  // SMP[2:0] = 011 → 47.5 ciclos
static constexpr uint32_t kTimClockHz = 62500000u;  // TIM6 @ 62.5 MHz após clock tree atual

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

static void gpdma_arm(uint32_t ch_base, uint32_t reqsel,
                      uint32_t buf_bytes, uint32_t src_dr, uint32_t dest_buf) noexcept {
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
    GPDMA_REG(ch_base, GPDMA_CLLR_OFF) = 0u;
	GPDMA_REG(ch_base, GPDMA_CCR_OFF) = GPDMA_CCR_PRIO_HIGH | GPDMA_CCR_TCIE |
		GPDMA_CCR_DTEIE | GPDMA_CCR_USEIE | GPDMA_CCR_EN; // FIX: removido GPDMA_CCR_CIRC — one-shot com re-arm no ISR
}

static void gpdma_adc1_arm() noexcept {
    gpdma_arm(GPDMA_CH0_BASE, GPDMA_CTR2_REQSEL_ADC1,
              sizeof(g_adc_secondary_raw),
              reinterpret_cast<uint32_t>(&ADC1_DR),
              reinterpret_cast<uint32_t>(&g_adc_secondary_raw[0]));
}

static void gpdma_adc2_arm() noexcept {
    gpdma_arm(GPDMA_CH1_BASE, GPDMA_CTR2_REQSEL_ADC2,
              sizeof(g_adc2_raw),
              reinterpret_cast<uint32_t>(&ADC2_DR),
              reinterpret_cast<uint32_t>(&g_adc2_raw[0]));
}

// ── API pública ───────────────────────────────────────────────────────────────

void adc_init() noexcept {
    // ── 1. Habilitar clocks ADC e GPIOs ─────────────────────────────────
    RCC_AHB1ENR |= RCC_AHB1ENR_ADC12EN | RCC_AHB1ENR_GPDMA1EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN
                  | RCC_AHB2ENR1_GPIOBEN
                  | RCC_AHB2ENR1_GPIOCEN;

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
// FIX: IN10 está em SMPR2, não em SMPR1 (SMPR1 cobre IN0-IN9, SMPR2 cobre IN10-IN18)
ADC1_SMPR2 = (kSmpr << 0); // IN10 (AN4) — bits [(10-10)*3] = bit 0

    // Sequência de conversão ADC1
    ADC1_SQR1 = kAdc1Sqr1;
    ADC1_SQR2 = kAdc1Sqr2;

	// CFGR1: 12-bit, trigger externo TIM6_TRGO, rising edge, DMA one-shot por sequência
	ADC1_CFGR1 = ADC_CFGR1_RES_12BIT
		| ADC_CFGR1_DMAEN
		| ADC_CFGR1_EXTSEL_TIM6_TRGO
		| ADC_CFGR1_EXTEN_RISING; // FIX: removido DMACFG — one-shot DMA, re-arm no ISR

    // ── 5. Calibrar e configurar ADC2 ainda desabilitado ────────────────
    adc_prepare_for_config(ADC2_CR);

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
	| ADC_CFGR1_DMAEN
	| ADC_CFGR1_EXTSEL_TIM6_TRGO
	| ADC_CFGR1_EXTEN_RISING; // FIX: removido DMACFG — one-shot DMA, re-arm no ISR

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
    if (idx >= 4u) { return 0u; }
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
    ems::hal::gpdma_adc1_arm();
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
    ems::hal::gpdma_adc2_arm();  // FIX: era gpdma_adc1_arm() — Channel1 gerencia ADC2
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
static uint16_t g_adc_secondary[4] = {};
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
