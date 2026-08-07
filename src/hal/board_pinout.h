#pragma once

/**
 * @file board_pinout.h
 * @brief Pinout at compile time: VGT6 | MRE | RGT6 (default).
 *
 * VGT6 = STM32H562VGT6 LQFP100 — OpenEMS ideal (GPIOE INJ/IGN PE0/2/4/6 + PE9/…)
 * MRE  = H562 on microRusEFI copper — PD12–15 IGN, PE14–11 INJ, SPI PD5+PB3/4/5
 *        (see docs/hw/pinout_mre_bringup.md + github.com/rusefi/hw_microRusEfi)
 * RGT6 = STM32H562RGT6 LQFP64  — WeAct headers A/B/C
 *
 *   make firmware BOARD=rgt6   # default
 *   make firmware BOARD=vgt6
 *   make firmware BOARD=mre
 */

#if defined(EMS_BOARD_MRE)
#  define EMS_BOARD_IS_MRE  1
#  define EMS_BOARD_IS_VGT6 0
#  define EMS_BOARD_NAME "MRE"
#elif defined(EMS_BOARD_VGT6)
#  define EMS_BOARD_IS_MRE  0
#  define EMS_BOARD_IS_VGT6 1
#  define EMS_BOARD_NAME "VGT6"
#else
#  define EMS_BOARD_IS_MRE  0
#  define EMS_BOARD_IS_VGT6 0
#  ifndef EMS_BOARD_RGT6
#    define EMS_BOARD_RGT6 1
#  endif
#  define EMS_BOARD_NAME "RGT6"
#endif
