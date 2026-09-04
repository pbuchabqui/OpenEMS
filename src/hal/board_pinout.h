#pragma once

/**
 * @file board_pinout.h
 * @brief Pinout at compile time: EMS_BOARD_VGT6 vs EMS_BOARD_RGT6 (default).
 *
 * VGT6 = STM32H562VGT6 LQFP100 (GPIOE: INJ/IGN/ETB)
 * RGT6 = STM32H562RGT6 LQFP64  (GPIOA/B/C: WeAct headers)
 *
 *   make firmware BOARD=rgt6   # default
 *   make firmware BOARD=vgt6
 */

#if defined(EMS_BOARD_VGT6)
#  define EMS_BOARD_IS_VGT6 1
#  define EMS_BOARD_NAME "VGT6"
#else
#  define EMS_BOARD_IS_VGT6 0
#  ifndef EMS_BOARD_RGT6
#    define EMS_BOARD_RGT6 1
#  endif
#  define EMS_BOARD_NAME "RGT6"
#endif

// O hardware analógico de knock (bandpass->rectificador->peak-hold) está DNP na
// v1 (docs/hw/schematic/10_knock_dnp.md) — o footprint TPIC8101 nunca foi
// populado. `knock_window_open()` foi apagado por acidente do scheduler
// (commit f42c450, varredura de "dead code" que levou consigo o wiring de
// knock introduzido em 39e3b65). Restaurar esse wiring sem gate ligaria
// knock_adc_update() a um pino PA5 flutuante: ruído cruza o threshold de
// forma imprevisível e knock_cycle_complete() tem um ratchet positivo
// (adc_threshold -= 64 a cada hit) que o tornaria cada vez mais sensível —
// produziria retard falso real num motor a correr. Default 0: o wiring fica
// restaurado no código (testável em host) mas nunca abre uma janela de
// verdade. Só passar a 1 quando o front-end analógico existir fisicamente.
#ifndef EMS_KNOCK_HW_PRESENT
#  define EMS_KNOCK_HW_PRESENT 0
#endif
