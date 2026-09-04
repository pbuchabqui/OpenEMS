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

// Analog knock front-end is DNP on v1. Default 0: a floating PA5 would
// open a window on noise, and knock_cycle_complete() ratchets the
// threshold down — false retard on a running engine. Set to 1 only with
// the analog front-end populated.
#ifndef EMS_KNOCK_HW_PRESENT
#  define EMS_KNOCK_HW_PRESENT 0
#endif
