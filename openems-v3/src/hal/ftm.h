/**
 * @file hal/ftm.h
 * @brief FlexTimer Module HAL — OpenEMS Engine Management System
 *
 * Referência: K64P144M120SF5 Reference Manual, Rev. 2
 *   Cap. 43 — FlexTimer Module (FTM)
 *   §43.3.1  FTM_SC   — Status and Control
 *   §43.3.3  FTM_MOD  — Modulo Register
 *   §43.3.5  FTM_CnSC — Channel Status and Control
 *   §43.3.6  FTM_CnV  — Channel Value
 *   §43.4.1  FTM_MODE — Features Mode Selection (write-protect)
 *
 * Mapeamento de periféricos:
 *   FTM0 (120 MHz / prescaler 2 = 16.67 ns/tick, overflow ~1.09 ms)
 *     CH0-CH3: reservados (INJ via FTM0_CH2/CH3 ou PIT — ver pinout)
 *     CH4-CH7: IGN4-IGN1 output compare (PTD4-PTD7)
 *
 *   FTM3 (120 MHz / prescaler 2 = 16.67 ns/tick, overflow ~1.09 ms)
 *     CH0: CKP input capture (PTD0) — NVIC prio 1
 *     CH1: CMP/fase input capture (PTD1) — NVIC prio 2
 *
 *   FTM1 (bus clock 60 MHz / prescaler configurável)
 *     CH0: IACV PWM (PTA8)
 *     CH1: Wastegate PWM (PTA9)
 *
 *   FTM2 (bus clock 60 MHz / prescaler configurável)
 *     CH0: VVT Escape PWM (PTA10)
 *     CH1: VVT Admissão PWM (PTA11)
 *
 * ATENÇÃO: Todo acesso a registradores FTM deve passar por este módulo.
 * Camadas superiores (drv/, app/) NUNCA tocam FTMx_* diretamente.
 *
 * ARMADILHA: FTM0/FTM3 são de 16 bits. Aritmética de delta DEVE ser
 * subtração circular uint16_t:
 *   uint16_t delta = (uint16_t)(current - previous);  // CORRETO
 *   if (current > previous) ...                        // ERRADO
 */

#pragma once
#include <cstdint>

namespace ems::hal {

// ──────────────────────────────────────────────────────────────────────────────
// FTM0 — Output Compare (Ignição + Injeção)
// Clock: 120 MHz / prescaler 2 → 16.67 ns/tick, overflow 65536 ticks ≈ 1.09 ms
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Inicializa FTM0 completo para output compare.
 *
 * Configura:
 *   - Write-protect desabilitado (FTM_MODE[WPDIS]=1) ANTES de qualquer config
 *   - Free-running counter, system clock, prescaler 2
 *   - CH4-CH7: output compare "set on match" (IGN4-IGN1)
 *   - CH2-CH3: output compare "set on match" (INJ1-INJ2)
 *   - FTM_COMBINE = 0 (canais independentes)
 *   - NVIC FTM0_IRQn, prioridade 2
 *
 * Deve ser chamada uma única vez durante init, antes de ftm3_init().
 */
void ftm0_init(void);

/**
 * @brief Programa o comparador de um canal FTM0.
 * @param ch   Canal 0-7
 * @param ticks Valor absoluto do contador FTM0 onde o evento deve ocorrer.
 *
 * Usa aritmética de 16 bits — o evento ocorre quando FTM0_CNT atinge
 * o valor `ticks` (com wrap-around natural de uint16_t).
 */
void ftm0_set_compare(uint8_t ch, uint16_t ticks) noexcept;

/**
 * @brief Limpa o flag CHF (Channel Flag) de um canal FTM0.
 * @param ch Canal 0-7
 * Deve ser chamado no início da ISR de cada canal para re-armar a interrupção.
 */
void ftm0_clear_chf(uint8_t ch) noexcept;

/**
 * @brief Retorna o valor atual do contador FTM0 (16 bits, free-running).
 * Seguro para chamar de qualquer contexto, incluindo ISR.
 */
uint16_t ftm0_count() noexcept;


// ──────────────────────────────────────────────────────────────────────────────
// FTM3 — Input Capture (CKP + CMP)
// Clock: 120 MHz / prescaler 2 → 16.67 ns/tick, overflow ~1.09 ms
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Inicializa FTM3 completo para input capture.
 *
 * Configura:
 *   - WPDIS antes de qualquer config
 *   - Free-running counter, system clock, prescaler 2
 *   - CH0 (PTD0/CKP): rising edge capture, NVIC prio 1 (máxima do sistema)
 *   - CH1 (PTD1/CMP): rising edge capture, NVIC prio 2
 *   - Mux de pino: PTD_PCR0 e PTD_PCR1 → ALT4 (FTM3)
 *
 * ARMADILHA (RusEFI #1488): mesmo configurado para rising only, ruído pode
 * gerar capturas espúrias. A ISR DEVE verificar o estado atual do pino via
 * PTD_PDIR antes de processar.
 */
void ftm3_init(void);

/**
 * @brief Retorna o valor atual do contador FTM3 (16 bits, free-running).
 */
uint16_t ftm3_count() noexcept;


// ──────────────────────────────────────────────────────────────────────────────
// FTM1 — PWM (IACV + Wastegate)
// Clock: bus clock 60 MHz
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Inicializa FTM1 em modo PWM edge-aligned.
 * @param freq_hz  Frequência desejada em Hz (ex: 50 para IACV, 100 para WG)
 *
 * Calcula automaticamente MOD e prescaler para a frequência solicitada.
 * Pinos: PTA8 (CH0/IACV), PTA9 (CH1/Wastegate) — ALT3.
 * Duty inicial = 0% em ambos os canais.
 */
void ftm1_pwm_init(uint32_t freq_hz);

/**
 * @brief Define o duty cycle de um canal FTM1.
 * @param ch           Canal 0 (IACV) ou 1 (Wastegate)
 * @param duty_pct_x10 Duty em décimos de percentual (0=0%, 1000=100%)
 */
void ftm1_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;


// ──────────────────────────────────────────────────────────────────────────────
// FTM2 — PWM (VVT Admissão + Escape)
// Clock: bus clock 60 MHz
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Inicializa FTM2 em modo PWM edge-aligned.
 * @param freq_hz  Frequência desejada em Hz (tipicamente 100-300 Hz para VVT)
 *
 * Pinos: PTA10 (CH0/VVT Esc), PTA11 (CH1/VVT Adm) — ALT3.
 * Duty inicial = 0% em ambos os canais.
 */
void ftm2_pwm_init(uint32_t freq_hz);

/**
 * @brief Define o duty cycle de um canal FTM2.
 * @param ch           Canal 0 (VVT Escape) ou 1 (VVT Admissão)
 * @param duty_pct_x10 Duty em décimos de percentual (0=0%, 1000=100%)
 */
void ftm2_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;


// ──────────────────────────────────────────────────────────────────────────────
// FTM0 — Output Compare para ignição com acionamento PURO por hardware
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Arma um canal FTM0 para disparar o pino de ignição por hardware.
 *
 * Configura FTM0_CnSC para Output Compare, Clear on match: o pino do canal
 * vai LOW automaticamente quando FTM0_CNT == CnV, sem intervenção da CPU.
 * Habilita CHIE para que a ISR FTM0 execute cleanup pós-disparo (re-arm
 * de dwell, log de evento, etc.).
 *
 * SETUP dos bits FTM0_CnSC (K64 RM §43.3.5):
 *   Bit 7 (CHF)  = 0  → Clear flag anterior (W0C)
 *   Bit 6 (CHIE) = 1  → Channel Interrupt Enable (cleanup pós-match)
 *   Bit 5 (MSnB) = 1  → Output Compare mode (MSnB:MSnA = 10)
 *   Bit 4 (MSnA) = 0
 *   Bit 3 (ELSnB)= 0  → Clear output on match (ELSnB:ELSnA = 01)
 *   Bit 2 (ELSnA)= 1
 *   CnSC = 0x64 (CHIE | MSnB | ELSnA)
 *
 * ATENÇÃO: o pino deve estar configurado como FTM output (mux ALT4) para
 *   que a transição de hardware seja visível externamente. Se configurado
 *   como GPIO, apenas a interrupção é gerada (sem ação de pino).
 *
 * JITTER ELIMINADO:
 *   A transição do pino ocorre no ciclo exato em que FTM0_CNT atinge CnV,
 *   independente do estado da CPU (ISR em execução, pipeline stall, etc.).
 *   Resolução: 1 tick = 16,67 ns ≈ 0,006° a 6000 RPM.
 *
 * @param ch    Canal FTM0 (0–7). Para ignição: IGN1=CH7, IGN2=CH6, IGN3=CH5, IGN4=CH4.
 * @param ticks Valor absoluto do contador FTM0 onde o disparo deve ocorrer.
 */
void ftm0_arm_ignition(uint8_t ch, uint16_t ticks) noexcept;


// ──────────────────────────────────────────────────────────────────────────────
// ISR Handlers — declarados aqui para linkagem, NÃO chamar diretamente
// ──────────────────────────────────────────────────────────────────────────────

extern "C" void FTM0_IRQHandler(void);
extern "C" void FTM3_IRQHandler(void);

} // namespace ems::hal
