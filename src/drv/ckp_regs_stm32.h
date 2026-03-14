#pragma once
/**
 * @file drv/ckp_regs_stm32.h
 * @brief Remapeia macros de registradores CKP do Kinetis para STM32H562RGT6.
 *
 * Incluído por drv/ckp.cpp quando TARGET_STM32H562 está definido.
 * Mantém os nomes simbólicos FTM3_C0V, FTM3_C1V e GPIOD_PDIR inalterados
 * no código do decodificador — apenas os endereços mudam.
 *
 * Mapeamento:
 *   FTM3_C0V  (FTM3 CH0 capture register, Kinetis 0x400B9010)
 *     → TIM5_CCR1 (TIM5 CH1 capture register, STM32H562 0x40000C34)
 *       TIM5 base = 0x40000C00, TIM_CCR1_OFF = 0x34
 *
 *   FTM3_C1V  (FTM3 CH1 capture register, Kinetis 0x400B9018)
 *     → TIM5_CCR2 (TIM5 CH2 capture register, STM32H562 0x40000C38)
 *       TIM5 base = 0x40000C00, TIM_CCR2_OFF = 0x38
 *
 *   GPIOD_PDIR (Kinetis GPIO input data register PTD, 0x400FF0C0)
 *     → GPIOA_IDR (STM32H562 GPIO A input data register, 0x42020010)
 *       GPIOA base = 0x42020000, GPIO_IDR_OFF = 0x10
 *       PA0 = CKP (bit 0), PA1 = CMP (bit 1)
 *
 * Conversão ticks→ns para TIM5 @ 62.5 MHz (prescaler 3 → /4):
 *   Kinetis (FTM3 @ 60.0 MHz):    ticks × 16667 / 1000 = ns
 *   STM32H562 (TIM5 @ 62.5 MHz):  ticks × 16000 / 1000 = ns
 *   Diferença: < 4% — sem impacto na precisão angular.
 *
 * Anti-glitch (equivalente ao PTD_PDIR check do Kinetis):
 *   Kinetis: (GPIOD_PDIR & (1u << 0)) != 0 → pino PTD0 HIGH = rising edge real
 *   STM32:   (GPIOA_IDR  & (1u << 0)) != 0 → pino PA0  HIGH = rising edge real
 */

// ── Substituição dos registradores de captura ────────────────────────────────
#undef  FTM3_C0V
#define FTM3_C0V   (*reinterpret_cast<volatile uint32_t*>(0x40000C34u))  // TIM5_CCR1

#undef  FTM3_C1V
#define FTM3_C1V   (*reinterpret_cast<volatile uint32_t*>(0x40000C38u))  // TIM5_CCR2

#undef  GPIOD_PDIR
#define GPIOD_PDIR (*reinterpret_cast<volatile uint32_t*>(0x42020010u))  // GPIOA_IDR

// ── Conversão ticks→ns para 62.5 MHz ─────────────────────────────────────────
// Kinetis usa: (ticks × 16667) / 1000
// STM32 usa:   (ticks × 16000) / 1000
// O macro TICKS_TO_NS_FACTOR é lido por ckp.cpp se definido.
#define TICKS_TO_NS_FACTOR   16000u   // ns/tick × 1000 para TIM5 @ 62.5 MHz
#define TICKS_TO_NS_DIVISOR  1000u
