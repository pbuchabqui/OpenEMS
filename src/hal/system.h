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
#ifdef __cplusplus
}
#endif
#endif
