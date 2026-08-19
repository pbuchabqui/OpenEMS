/**
 * @file ecu_sched_pins.cpp
 * @brief GPIO bookkeeping, watchdogs TIM5 de parede, prime e pulsos de teste.
 *
 * Não é o dispatcher de produção (isso é TIM2/CH3). Qualquer caminho que
 * mude um pino — fila TIM2, fila TIM5 ou force_output — passa por
 * pin_transition() e arma os relógios de dwell 1,4× / inj 36 ms.
 */
#include "engine/ecu_sched.h"
#include "engine/ecu_sched_internal.h"
#include "hal/out_pins.h"
#include "hal/critical_section.h"
#if !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

namespace si = ems::engine::sched_internal;

#include <stdint.h>

// Pin transition verification: count every actual pin state change
volatile uint32_t g_pin_high_count[8];  // [0-3]=INJ CH1-4, [4-7]=IGN CH1-4
volatile uint32_t g_pin_low_count[8];
volatile uint32_t g_pin_seq_error[8];   // consecutive same-direction transitions
static uint8_t    g_pin_last_state[8];  // 0=LOW, 1=HIGH, 0xFF=unknown

// ── Dwell watchdog (MS42 §2.2.2.1.3 — TD × 1.4) ──────────────────────────
// Escrito pela ISR (arm_channel / pin HIGH), lido pelo main loop.
volatile uint32_t g_dwell_arm_tick[4]  = {0U, 0U, 0U, 0U};
volatile uint32_t g_dwell_wdog_ticks[4] = {0U, 0U, 0U, 0U};
volatile uint32_t g_dwell_watchdog_count = 0U;

// ── Injector open watchdog (lost INJ_OFF / queue overflow backstop) ────────
// Timeout: 1.2 × PW quando arm_channel programa; senão tecto 36 ms.
volatile uint32_t g_inj_open_tick[4]   = {0U, 0U, 0U, 0U};
volatile uint32_t g_inj_wdog_ticks[4]  = {0U, 0U, 0U, 0U};
volatile uint32_t g_inj_watchdog_count = 0U;

void pin_transition(uint8_t idx, uint8_t high, uint8_t is_safe_state) {
    if (idx >= 8U) { return; }
    if (g_pin_last_state[idx] == high && high != 0xFFU) {
        if (is_safe_state == 0U) { ++g_pin_seq_error[idx]; }
        return;  // redundant transition — don't double-count
    }
    if (high) {
        ++g_pin_high_count[idx];
        if (idx < 4U) {
            g_inj_open_tick[idx] = TIM5_CNT | 1U;
            if (g_inj_wdog_ticks[idx] == 0U) {
                g_inj_wdog_ticks[idx] = si::kInjOpenWdogHardTicks;
            }
        } else if (idx >= kIgnChFirst && idx < (kIgnChFirst + 4U)) {
            const uint8_t ign_idx = (uint8_t)(idx - kIgnChFirst);
            g_dwell_arm_tick[ign_idx] = TIM5_CNT | 1U;
            g_dwell_wdog_ticks[ign_idx] = (si::g_dwell_ticks * 7U) / 5U;
        }
    } else {
        ++g_pin_low_count[idx];
        if (idx < 4U) {
            g_inj_open_tick[idx] = 0U;
            g_inj_wdog_ticks[idx] = 0U;
        } else if (idx >= kIgnChFirst && idx < (kIgnChFirst + 4U)) {
            g_dwell_arm_tick[idx - kIgnChFirst] = 0U;
            g_dwell_wdog_ticks[idx - kIgnChFirst] = 0U;
        }
    }
    g_pin_last_state[idx] = high;
}

void force_close_cyl_mask(uint8_t mask, uint8_t is_ign)
{
    for (uint8_t cyl = 0U; cyl < 4U; ++cyl) {
        if ((mask & (1U << cyl)) == 0U) { continue; }
        if (is_ign != 0U) {
            force_output(si::kIgnCh[cyl], ECU_ACT_SPARK, 1U);
            g_dwell_arm_tick[cyl] = 0U;
        } else {
            force_output(si::kInjCh[cyl], ECU_ACT_INJ_OFF, 1U);
        }
    }
}

void force_output(uint8_t ch, uint8_t action, uint8_t is_safe_state,
                  uint8_t bypass_inhibit)
{
    // Safe-state transitions (INJ_OFF / SPARK) always allowed — never block a cut.
    // Non-safe ON paths honor inhibit masks so prime cannot bypass fuel-protect.
    // bypass_inhibit: só ecu_sched_test_pulse_inj/_ign (comando manual).
    if (is_safe_state == 0U && bypass_inhibit == 0U) {
        const uint8_t is_inj = (ch < kIgnChFirst) ? 1U : 0U;
        if (is_inj != 0U && action == ECU_ACT_INJ_ON) {
            const uint8_t cyl_bit = (ch < 8U) ? si::k_inj_ch_to_bit[ch] : 0U;
            if (cyl_bit != 0U && (g_inj_inhibit_mask & cyl_bit) != 0U) { return; }
        }
        if (is_inj == 0U && action == ECU_ACT_DWELL_START) {
            const uint8_t cyl_bit = (ch < 8U) ? si::k_ign_ch_to_bit[ch] : 0U;
            if (cyl_bit != 0U && (g_ign_inhibit_mask & cyl_bit) != 0U) { return; }
        }
    }
    const uint8_t high = ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U;
    const uint8_t idx = channel_pin_idx(ch);
    if (idx != 0xFFU) {
        ems::hal::out_pin_write(ch, high);
        pin_transition(idx, high, is_safe_state);
    }
}

void ecu_sched_dwell_watchdog(void)
{
    if (g_inj_pw_override != 0U) { return; }  // test mode — disable watchdog
    const uint32_t now = TIM5_CNT;
    for (uint8_t i = 0U; i < 4U; ++i) {
        ems::hal::CriticalSectionGuard guard;
        const uint32_t arm  = g_dwell_arm_tick[i];
        const uint32_t tout = g_dwell_wdog_ticks[i];
        if (arm != 0U && tout != 0U && (now - arm) >= tout) {
            purge_events_for_cyl_mask(static_cast<uint8_t>(1U << i), 1U);
            g_dwell_arm_tick[i] = 0U;
            g_dwell_wdog_ticks[i] = 0U;
            ++g_dwell_watchdog_count;
        }
    }
}

uint32_t ecu_sched_dwell_watchdog_count(void) { return g_dwell_watchdog_count; }

void ecu_sched_inj_watchdog(void)
{
    if (g_inj_pw_override != 0U) { return; }
    const uint32_t now = TIM5_CNT;
    for (uint8_t i = 0U; i < 4U; ++i) {
        ems::hal::CriticalSectionGuard guard;
        const uint32_t open = g_inj_open_tick[i];
        const uint32_t tout = g_inj_wdog_ticks[i];
        if (open != 0U && tout != 0U && (now - open) >= tout) {
            purge_events_for_cyl_mask(static_cast<uint8_t>(1U << i), 0U);
            g_inj_open_tick[i] = 0U;
            g_inj_wdog_ticks[i] = 0U;
            ++g_inj_watchdog_count;
        }
    }
}

uint32_t ecu_sched_inj_watchdog_count(void) { return g_inj_watchdog_count; }

void ecu_sched_fire_prime_pulse(uint32_t pw_us)
{
    if (pw_us == 0U) { return; }
    if (pw_us > 30000U) { pw_us = 30000U; }
    const uint32_t off_cnv = TIM5_CNT + ECU_SCHED_US_TO_TICKS_INTERNAL(pw_us);
    for (uint8_t i = 0U; i < 4U; ++i) { force_output(si::kInjCh[i], ECU_ACT_INJ_ON); }
    for (uint8_t i = 0U; i < 4U; ++i) { arm_channel(si::kInjCh[i], off_cnv, ECU_ACT_INJ_OFF); }
    ++g_diag_prime_fired;
}

void ecu_sched_test_pulse_inj(uint8_t cyl, uint32_t pw_us)
{
    if (cyl > 3U || pw_us == 0U) { return; }
    if (pw_us > 30000U) { pw_us = 30000U; }
    const uint8_t ch = si::kInjCh[cyl];
    const uint32_t off_cnv = TIM5_CNT + ECU_SCHED_US_TO_TICKS_INTERNAL(pw_us);
    force_output(ch, ECU_ACT_INJ_ON, 0U, 1U);
    arm_channel(ch, off_cnv, ECU_ACT_INJ_OFF);
}

void ecu_sched_test_pulse_ign(uint8_t cyl, uint32_t dwell_us)
{
    if (cyl > 3U) { return; }
    if (dwell_us == 0U) { dwell_us = 3000U; }
    if (dwell_us > 10000U) { dwell_us = 10000U; }
    const uint8_t ch = si::kIgnCh[cyl];
    const uint32_t spark_cnv = TIM5_CNT + ECU_SCHED_US_TO_TICKS_INTERNAL(dwell_us);
    force_output(ch, ECU_ACT_DWELL_START, 0U, 1U);
    {
        ems::hal::CriticalSectionGuard guard;
        g_dwell_arm_tick[cyl]  = TIM5_CNT | 1U;
        g_dwell_wdog_ticks[cyl] = (ECU_SCHED_US_TO_TICKS_INTERNAL(dwell_us) * 7U) / 5U;
    }
    arm_channel(ch, spark_cnv, ECU_ACT_SPARK);
}

#if defined(EMS_HOST_TEST)
uint32_t ecu_sched_test_get_dwell_arm_tick(uint8_t cyl)
{
    return (cyl < 4U) ? g_dwell_arm_tick[cyl] : 0U;
}
#endif
