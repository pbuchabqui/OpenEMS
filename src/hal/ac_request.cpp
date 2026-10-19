#include "hal/ac_request.h"

#ifdef TARGET_STM32H562
#include "hal/stm32h562/regs.h"

namespace {

constexpr uint32_t kPin = 11u;           // PB11
constexpr uint32_t kDebounceMs = 50u;

bool     g_state = false;                // estado filtrado
bool     g_raw = false;                  // última amostra crua
uint32_t g_raw_since_ms = 0u;

}  // namespace

namespace ems::hal {

void ac_request_init() noexcept {
    GPIOB_MODER = GPIOB_MODER & ~(3u << (kPin * 2u));  // entrada
    GPIOB_PUPDR = (GPIOB_PUPDR & ~(3u << (kPin * 2u))) | (2u << (kPin * 2u));  // pull-down
}

bool ac_request_poll(uint32_t now_ms) noexcept {
    const bool raw = (STM32_REG32(GPIOB_BASE + GPIO_IDR_OFF) & (1u << kPin)) != 0u;
    if (raw != g_raw) {
        g_raw = raw;
        g_raw_since_ms = now_ms;
    } else if (raw != g_state && (now_ms - g_raw_since_ms) >= kDebounceMs) {
        g_state = raw;
    }
    return g_state;
}

}  // namespace ems::hal
#endif
