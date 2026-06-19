#include "hal/ewg_driver.h"

#ifdef TARGET_STM32H562
#include "hal/stm32h562/regs.h"
#include "hal/timer.h"
#include "hal/adc.h"

namespace {

// EWG H-bridge: PA7 = IN1 (open), PD3 = IN2 (close), PA6 = TIM3_CH1 PWM
constexpr uint8_t kIn1Pin = 7u;   // PA7
constexpr uint8_t kIn2Pin = 3u;   // PD3

}  // namespace

namespace ems::hal {

bool ewg_driver_init() noexcept {
    // PA7 = IN1 (GPIO output)
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (kIn1Pin * 2u))) | (1u << (kIn1Pin * 2u));
    // PD3 = IN2 (GPIO output)
    GPIOD_MODER = (GPIOD_MODER & ~(3u << (kIn2Pin * 2u))) | (1u << (kIn2Pin * 2u));

    // TIM3_CH1 (PA6) PWM @ 10 kHz for EWG motor
    tim3_pwm_init(10000u);

    ewg_driver_shutdown();
    return true;
}

void ewg_driver_set_motor_pwm(int16_t pwm) noexcept {
    if (pwm >  1000) { pwm =  1000; }
    if (pwm < -1000) { pwm = -1000; }

    const uint16_t duty = static_cast<uint16_t>((pwm >= 0) ? pwm : -pwm);

    if (pwm > 0) {
        GPIOA_BSRR = (1u << kIn1Pin);                  // IN1=1 (open)
        GPIOD_BSRR = (1u << (kIn2Pin + 16u));          // IN2=0
    } else if (pwm < 0) {
        GPIOA_BSRR = (1u << (kIn1Pin + 16u));          // IN1=0
        GPIOD_BSRR = (1u << kIn2Pin);                  // IN2=1 (close)
    } else {
        GPIOA_BSRR = (1u << (kIn1Pin + 16u));          // IN1=0
        GPIOD_BSRR = (1u << (kIn2Pin + 16u));          // IN2=0 (brake)
    }
    tim3_set_duty(0u, duty);
}

uint16_t ewg_driver_read_position_raw() noexcept {
    return adc_secondary_read(ems::hal::AdcSecondaryChannel::EWG_POS);
}

void ewg_driver_shutdown() noexcept {
    GPIOA_BSRR = (1u << (kIn1Pin + 16u));
    GPIOD_BSRR = (1u << (kIn2Pin + 16u));
    tim3_set_duty(0u, 0u);
}

}  // namespace ems::hal

#else  // host test stub

namespace ems::hal {
bool ewg_driver_init() noexcept { return true; }
void ewg_driver_set_motor_pwm(int16_t) noexcept {}
uint16_t ewg_driver_read_position_raw() noexcept { return 2048u; }
void ewg_driver_shutdown() noexcept {}
}

#endif
