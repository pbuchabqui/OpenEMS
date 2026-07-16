#pragma once
/**
 * @file hal/out_pins.h
 * INJ/IGN GPIO BSRR maps + inline write (hygiene PR-12).
 *
 * Channel index MUST match ecu_sched ECU_CH_* / event.channel order:
 *   [0]=INJ3 [1]=INJ4 [2]=INJ1 [3]=INJ2 [4]=IGN4 [5]=IGN3 [6]=IGN2 [7]=IGN1
 *
 * Hot-path rule: out_pin_write is inline in this header so ecu_sched can
 * inline without LTO (see ecu_sched_internal.h).
 */
#include <cstdint>

#include "hal/board_pinout.h"

#if defined(EMS_HOST_TEST)
#include "hal/out_pins_host.h"
#else
#include "hal/regs.h"
#endif

namespace ems::hal {

enum : uint8_t { kOutPortA = 0U, kOutPortB = 1U, kOutPortC = 2U, kOutPortE = 3U };

// Pin metric index 0..3 INJ, 4..7 IGN — same as former k_ch_to_pin_idx.
inline constexpr uint8_t kOutChToPinIdx[8] = {2U, 3U, 0U, 1U, 7U, 6U, 5U, 4U};

#if EMS_BOARD_IS_VGT6
// VGT6: INJ PE0/2/4/6 · IGN PE9/11/13/15 — ordered by ECU_CH_*
inline constexpr uint8_t kOutBsrrPort[8] = {
    kOutPortE, kOutPortE, kOutPortE, kOutPortE,
    kOutPortE, kOutPortE, kOutPortE, kOutPortE
};
inline constexpr uint8_t kOutBsrrPin[8] = {
    4U, 6U, 0U, 2U,
    15U, 13U, 11U, 9U
};
#else
// RGT6: INJ PA15/PB3/PC10/PC11 · IGN PC6–9 — ordered by ECU_CH_*
inline constexpr uint8_t kOutBsrrPort[8] = {
    kOutPortC, kOutPortC, kOutPortA, kOutPortB,
    kOutPortC, kOutPortC, kOutPortC, kOutPortC
};
inline constexpr uint8_t kOutBsrrPin[8] = {
    10U, 11U, 15U, 3U,
    9U, 8U, 7U, 6U
};
#endif

// Compile-time pin sanity (host always RGT6 tables; VGT6 checked on firmware-vgt6).
#if EMS_BOARD_IS_VGT6
static_assert(kOutBsrrPort[2] == kOutPortE && kOutBsrrPin[2] == 0U, "INJ1=PE0");
static_assert(kOutBsrrPort[7] == kOutPortE && kOutBsrrPin[7] == 9U, "IGN1=PE9");
#else
static_assert(kOutBsrrPort[2] == kOutPortA && kOutBsrrPin[2] == 15U, "INJ1=PA15");
static_assert(kOutBsrrPort[3] == kOutPortB && kOutBsrrPin[3] == 3U, "INJ2=PB3");
static_assert(kOutBsrrPort[7] == kOutPortC && kOutBsrrPin[7] == 6U, "IGN1=PC6");
#endif

/** Active-high actuators: high=1 drives pin high (ON/DWELL), high=0 drives low (OFF/SPARK). */
inline void out_pin_write(uint8_t channel, uint8_t high) noexcept {
    if (channel >= 8U) { return; }
    const uint8_t pin = kOutBsrrPin[channel];
    const uint32_t mask = high ? (1U << pin)
                               : (1U << (static_cast<uint32_t>(pin) + 16U));
    switch (kOutBsrrPort[channel]) {
    case kOutPortA: GPIOA_BSRR = mask; break;
    case kOutPortB: GPIOB_BSRR = mask; break;
    case kOutPortE: GPIOE_BSRR = mask; break;
    default:        GPIOC_BSRR = mask; break;
    }
}

/**
 * Clocks + MODER/OTYPER/PUPDR/AFR + multi-pin BSRR safe (all LOW).
 * Bit-identical to former ecu_sched_outputs_safe_early().
 */
void out_pins_hw_init() noexcept;

#if defined(EMS_HOST_TEST)
/** Port index: A=0 B=1 C=2 E=3 — last BSRR write value (host mock). */
uint32_t out_pins_test_bsrr_snapshot(uint8_t port) noexcept;
void out_pins_test_reset_stubs() noexcept;
#endif

}  // namespace ems::hal
