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

// Fork MT6835/TIM2-encoder — este repo é só encoder. TIM2 AB + TIM5 freerun
// + TIM3 CMP. Não há build Hall/60-2 aqui (docs/dev/mt6835_encoder_fork.md).
#ifndef EMS_MT6835_ENCODER
#  define EMS_MT6835_ENCODER 1
#endif

// Que fase (ECU_PHASE_A/B) um flanco do CMP representa é uma constante de
// calibração de hardware (onde o Hall está montado face ao ciclo de 720°).
// 2026-08-15: deixou de ser flag de compilação — passou a
// cfg::g_eng_cfg.cmp_phase_state (NVM, engine/engine_config.h), calibrável
// ao vivo via comando 'M' (src/app/ui_protocol.cpp) + burn ('b'), mesmo
// padrão de encoder_tdc1_origin_deg/comando 'X'. Default
// kCmpPhaseUncalibrated=0: ecu_sched_encoder_phase_set_anchor() nunca é
// chamada a partir de hardware real, phase_valid() fica sempre 0, e o
// recompute do dispatcher em ângulo cai sempre em presync — nunca dispara
// sequencial com uma fase adivinhada (mesma garantia que a antiga
// EMS_MT6835_CMP_PHASE_CALIBRATED=0 dava, sem precisar de recompilar depois
// de medida em bancada — ver docs/dev/mt6835_encoder_fork.md).

// O hardware analógico de knock (bandpass→rectificador→peak-hold) está DNP na
// v1 (docs/hw/schematic/10_knock_dnp.md) — o footprint TPIC8101 nunca foi
// populado. `knock_window_open()` foi apagado por acidente do scheduler
// (commit f42c450, varredura de "dead code" que levou consigo o wiring de
// knock introduzido em 39e3b65) — confirmado em falta em ambos os branches,
// não é específico deste fork. Restaurar esse wiring sem gate ligaria
// knock_adc_update() a um pino PA5 flutuante: ruído cruza o threshold de
// forma imprevisível e knock_cycle_complete() tem um ratchet positivo
// (adc_threshold -= 64 a cada hit) que o tornaria cada vez mais sensível —
// produziria retard falso real num motor a correr. Default 0: o wiring fica
// restaurado no código (testável em host) mas nunca abre uma janela de
// verdade. Só passar a 1 quando o front-end analógico existir fisicamente.
#ifndef EMS_KNOCK_HW_PRESENT
#  define EMS_KNOCK_HW_PRESENT 0
#endif

// TLE8888 — descartado neste fork (docs/dev/mt6835_encoder_fork.md,
// architecture_v2). INJ/IGN são GPIO directo; INJEN/IGNEN sobem no boot
// sem esperar SPI. Default 0: driver não toca em SPI2 (conflito com
// MT6835) e tle8888_ok() é true. Só passar a 1 se o CI voltar à placa.
#ifndef EMS_TLE8888_PRESENT
#  define EMS_TLE8888_PRESENT 0
#endif

// Detector de misfire em modo encoder (engine/misfire_encoder.h/.cpp) —
// matemática nova (queda de velocidade angular por janela de cilindro),
// paralela a engine/misfire_detect.cpp (caminho CKP, intocado). O módulo
// corre e acumula internamente sempre, independentemente desta flag
// (testável em host sem build especial) — só a publicação nos contadores
// DTC-facing (main_stm32.cpp, consumidos por misfire_get_event_count()
// equivalente) fica atrás dela. Default 0: mesmo padrão de
// EMS_MT6835_CMP_PHASE_CALIBRATED — prova-se em isolamento antes de
// confiar. Só passar a 1 depois de validar o módulo em bancada.
#ifndef EMS_MISFIRE_ENCODER_ENABLE
#  define EMS_MISFIRE_ENCODER_ENABLE 0
#endif

// Convenience for call sites that only need a boolean (same as EMS_MT6835_ENCODER).
#define EMS_ENCODER_MODE (EMS_MT6835_ENCODER != 0)
