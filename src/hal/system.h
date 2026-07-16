#pragma once

#if defined(TARGET_STM32H562) || !defined(EMS_HOST_TEST)
#include "hal/stm32h562/system.h"
#else
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif
void system_stm32_init(void) noexcept;
void iwdg_kick(void) noexcept;
uint32_t millis(void) noexcept;
uint32_t micros(void) noexcept;
// Host-test only: control mock wall clock for duration-gated logic (X-τ learn, etc.).
void host_set_millis(uint32_t ms) noexcept;
void host_advance_millis(uint32_t dt) noexcept;
#ifdef __cplusplus
}
#endif
#endif
