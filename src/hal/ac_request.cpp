#include "hal/ac_request.h"

#ifdef TARGET_STM32H562
#include "hal/stm32h562/regs.h"

namespace {

constexpr uint32_t kPin = 11u;           // PB11

ems::hal::AcDebounce g_deb;

}  // namespace

namespace ems::hal {

void ac_request_init() noexcept {
    GPIOB_MODER = GPIOB_MODER & ~(3u << (kPin * 2u));  // entrada
    GPIOB_PUPDR = (GPIOB_PUPDR & ~(3u << (kPin * 2u))) | (2u << (kPin * 2u));  // pull-down
}

bool ac_request_poll(uint32_t now_ms) noexcept {
    const bool raw = (STM32_REG32(GPIOB_BASE + GPIO_IDR_OFF) & (1u << kPin)) != 0u;
    return g_deb.step(raw, now_ms);
}

}  // namespace ems::hal
#endif
