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

// Fork MT6835/TIM2-encoder (docs/dev/mt6835_encoder_fork.md), VGT6 apenas.
// Default 0: boot idêntico à produção (tim5_ic_init(), CKP/CMP via Hall).
// Com 1: troca para tim5_freerun_init()+tim2_encoder_init()+
// tim3_cmp_ic_init()+mt6835_init() em main_stm32.cpp — mutuamente exclusivo
// com o caminho de produção, nunca os dois no mesmo boot.
#ifndef EMS_MT6835_ENCODER
#  define EMS_MT6835_ENCODER 0
#endif

// Que fase (ECU_PHASE_A/B) um flanco do CMP representa é uma constante de
// calibração de hardware (onde o Hall está montado face ao ciclo de 720°) —
// ainda NÃO medida em bancada. Default 0: ecu_sched_encoder_phase_set_anchor()
// nunca é chamada a partir de hardware real, phase_valid() fica sempre 0, e
// o recompute do dispatcher em ângulo cai sempre em presync — nunca dispara
// sequencial com uma constante adivinhada. Só passar a 1 depois da medição
// em bancada (ver docs/dev/mt6835_encoder_fork.md).
#ifndef EMS_MT6835_CMP_PHASE_CALIBRATED
#  define EMS_MT6835_CMP_PHASE_CALIBRATED 0
#endif
