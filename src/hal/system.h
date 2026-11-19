#pragma once

#if defined(TARGET_STM32H562) || !defined(EMS_HOST_TEST)
#include "hal/stm32h562/system.h"
#else
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif
void system_stm32_init(void) noexcept;
bool system_clock_ok(void) noexcept;
bool boot_reset_was_power_on(void) noexcept;
uint32_t chip_idcode(void) noexcept;
void iwdg_kick(void) noexcept;
uint32_t millis(void) noexcept;
uint32_t micros(void) noexcept;
// ICACHE: no host é um mock fixo (hits 1000, misses 7, ligado), só para o teste do protocolo ('k').
uint32_t icache_hits(void) noexcept;
uint32_t icache_misses(void) noexcept;
bool icache_is_enabled(void) noexcept;
void icache_monitors_reset(void) noexcept;
// Host-test only: control mock wall clock for duration-gated logic (X-τ learn, etc.).
void host_set_millis(uint32_t ms) noexcept;
void host_advance_millis(uint32_t dt) noexcept;
void host_set_clock_ok(bool ok) noexcept;
uint32_t host_icache_monitor_resets(void) noexcept;   // quantas vezes icache_monitors_reset() rodou
#ifdef __cplusplus
}
#endif
#endif
