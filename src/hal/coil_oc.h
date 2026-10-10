#pragma once
/**
 * @file hal/coil_oc.h
 * Coil outputs driven by hardware output compare (MS42 CC6–CC11 style).
 *
 * The IGN pins are timer channels: VGT6 PE9/11/13/15 = TIM1_CH1–4 (AF1),
 * RGT6 PC6–9 = TIM8_CH1–4 (AF3). Coil index = 7 − ECU_CH_IGNx, so coil 0
 * (IGN1) is CH1 … coil 3 (IGN4) is CH4.
 *
 * TIM1/TIM8 run from the same 250 MHz kernel clock and prescaler as TIM5
 * (16 ns tick), so TIMx_CNT − TIM5_CNT is a constant measured once at init
 * (≤ 1 tick of fractional prescaler phase, no drift). The scheduler hands a
 * coil edge to its channel ~100 µs ahead ("set/clear on match"); the pin then
 * toggles on the exact tick, independent of TIM5 ISR latency. The queued
 * commit at the same timestamp forces the level (FORCE_ACTIVE/INACTIVE), so
 * a missed match still lands — at software latency, like the GPIO path.
 *
 * Force modes override a pending match immediately: safe-state cuts, purges
 * and the dwell watchdog keep working exactly as with plain GPIO.
 *
 * Fallback: build with EMS_IGN_HW_OC=0 (or anything re-running
 * out_pins_hw_init) leaves the pins in GPIO mode; the commit's BSRR write then
 * drives them at dispatch time, i.e. the previous behaviour.
 */
#include <cstdint>

#include "hal/board_pinout.h"
#include "hal/critical_section.h"

#if !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

#ifndef EMS_IGN_HW_OC
#define EMS_IGN_HW_OC 1
#endif

namespace ems::hal {

inline constexpr uint8_t kCoilNone = 0xFFU;

/** ECU_CH_IGN4..IGN1 (4..7) → coil 3..0; anything else → kCoilNone. */
inline constexpr uint8_t coil_oc_index(uint8_t channel) noexcept {
    return (channel >= 4U && channel < 8U) ? static_cast<uint8_t>(7U - channel) : kCoilNone;
}

/** 1 once the coil pins are in AF mode and the timer drives them. */
extern volatile uint8_t g_coil_oc_enabled;
/** (TIMx_CNT − TIM5_CNT) mod 2^16, measured at init. */
extern volatile uint16_t g_coil_oc_offset;

/** Configure TIM1/TIM8, measure the offset, hand the pins to the timer. */
bool coil_oc_hw_init() noexcept;

#if defined(EMS_HOST_TEST)
// Host model of the four compare channels. Unit tests leave `model` = 0 (pins
// behave as GPIO); the engine simulator sets it and calls advance() so coil
// edges are logged on the compare tick.
namespace coil_oc_host {
extern uint8_t model;
extern uint8_t armed[4];
extern uint8_t arm_high[4];
extern uint32_t arm_ts[4];
extern uint8_t ref[4];
extern void (*edge_hook)(uint8_t channel, uint8_t high, uint32_t ts);
/** Fire matches with from < ts ≤ to (TIM5 domain), in time order. */
void advance(uint32_t from, uint32_t to) noexcept;
void reset() noexcept;
}  // namespace coil_oc_host

inline void coil_oc_arm(uint8_t coil, uint8_t high, uint32_t tim5_ts) noexcept {
    if (g_coil_oc_enabled == 0U || coil >= 4U) { return; }
    coil_oc_host::armed[coil] = 1U;
    coil_oc_host::arm_high[coil] = high;
    coil_oc_host::arm_ts[coil] = tim5_ts;
}

inline void coil_oc_force(uint8_t coil, uint8_t high) noexcept {
    if (g_coil_oc_enabled == 0U || coil >= 4U) { return; }
    coil_oc_host::armed[coil] = 0U;
    coil_oc_host::ref[coil] = high;
}

inline uint8_t coil_oc_pin_level(uint8_t coil) noexcept {
    return (coil < 4U) ? coil_oc_host::ref[coil] : 0U;
}
#else

#if EMS_BOARD_IS_VGT6
#define COIL_TIM_CCMR1 TIM1_CCMR1
#define COIL_TIM_CCMR2 TIM1_CCMR2
#define COIL_TIM_CCR1  TIM1_CCR1
#define COIL_TIM_CCR2  TIM1_CCR2
#define COIL_TIM_CCR3  TIM1_CCR3
#define COIL_TIM_CCR4  TIM1_CCR4
#define COIL_GPIO_IDR  GPIOE_IDR
inline constexpr uint8_t kCoilPin[4] = {9U, 11U, 13U, 15U};
#else
#define COIL_TIM_CCMR1 TIM8_CCMR1
#define COIL_TIM_CCMR2 TIM8_CCMR2
#define COIL_TIM_CCR1  TIM8_CCR1
#define COIL_TIM_CCR2  TIM8_CCR2
#define COIL_TIM_CCR3  TIM8_CCR3
#define COIL_TIM_CCR4  TIM8_CCR4
#define COIL_GPIO_IDR  GPIOC_IDR
inline constexpr uint8_t kCoilPin[4] = {6U, 7U, 8U, 9U};
#endif

// OCxM[2:0] values (OCxM[3] stays 0): 1 = active on match, 2 = inactive on
// match, 4 = force inactive, 5 = force active.
inline void coil_oc_set_mode(uint8_t coil, uint32_t mode) noexcept {
    const uint32_t shift = ((coil & 1U) != 0U) ? 12U : 4U;
    CriticalSectionGuard guard;  // CCMR is shared by two channels: RMW must be atomic
    if (coil < 2U) {
        COIL_TIM_CCMR1 = (COIL_TIM_CCMR1 & ~(7U << shift)) | (mode << shift);
    } else {
        COIL_TIM_CCMR2 = (COIL_TIM_CCMR2 & ~(7U << shift)) | (mode << shift);
    }
}

inline void coil_oc_arm(uint8_t coil, uint8_t high, uint32_t tim5_ts) noexcept {
    if (g_coil_oc_enabled == 0U || coil >= 4U) { return; }
    const uint32_t ccr = (tim5_ts + g_coil_oc_offset) & 0xFFFFU;
    switch (coil) {
    case 0U: COIL_TIM_CCR1 = ccr; break;
    case 1U: COIL_TIM_CCR2 = ccr; break;
    case 2U: COIL_TIM_CCR3 = ccr; break;
    default: COIL_TIM_CCR4 = ccr; break;
    }
    coil_oc_set_mode(coil, (high != 0U) ? 1U : 2U);
}

inline void coil_oc_force(uint8_t coil, uint8_t high) noexcept {
    if (g_coil_oc_enabled == 0U || coil >= 4U) { return; }
    coil_oc_set_mode(coil, (high != 0U) ? 5U : 4U);
}

inline uint8_t coil_oc_pin_level(uint8_t coil) noexcept {
    return (coil < 4U) ? static_cast<uint8_t>((COIL_GPIO_IDR >> kCoilPin[coil]) & 1U) : 0U;
}
#endif

}  // namespace ems::hal
