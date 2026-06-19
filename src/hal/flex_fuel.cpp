#include "hal/flex_fuel.h"

#ifdef TARGET_STM32H562
#include "hal/stm32h562/regs.h"
#include "hal/system.h"

namespace {

// Flex fuel sensor (GM/Continental): PB5, EXTI5
// Frequency 50-150 Hz → ethanol 0-100%
// Duty cycle → fuel temperature (-40 to 125°C)
// Signal: 50Hz = 0% ethanol, 150Hz = 100% ethanol (linear)
constexpr uint32_t kFreqMin = 50u;    // 0% ethanol
constexpr uint32_t kFreqMax = 150u;   // 100% ethanol
constexpr uint32_t kTimeoutMs = 500u; // no signal = invalid

volatile uint32_t g_last_rising_us = 0u;
volatile uint32_t g_last_falling_us = 0u;
volatile uint32_t g_period_us = 0u;
volatile uint32_t g_high_us = 0u;
volatile uint32_t g_last_edge_ms = 0u;
volatile bool g_got_rising = false;

}  // namespace

namespace ems::hal {

void flex_fuel_init() noexcept {
    // PB5 = input with pull-down
    GPIOB_MODER = GPIOB_MODER & ~(3u << (5u * 2u));  // input mode
    // PUPDR: pull-down (10b)
    volatile uint32_t& pupdr = STM32_REG32(GPIOB_BASE + GPIO_PUPDR_OFF);
    pupdr = (pupdr & ~(3u << (5u * 2u))) | (2u << (5u * 2u));

    // EXTI5: PB5 → EXTI line 5, both edges
    // EXTI_EXTICR2: EXTI5 source = PB (0x01) in bits [7:0]
    constexpr uint32_t EXTI_BASE = 0x44022000UL;
    volatile uint32_t& exticr2 = STM32_REG32(EXTI_BASE + 0x064UL);  // EXTICR2
    exticr2 = (exticr2 & ~0xFFu) | 0x01u;  // PB for EXTI5

    // Rising + falling edge trigger
    volatile uint32_t& rtsr1 = STM32_REG32(EXTI_BASE + 0x000UL);
    volatile uint32_t& ftsr1 = STM32_REG32(EXTI_BASE + 0x004UL);
    rtsr1 |= (1u << 5u);
    ftsr1 |= (1u << 5u);

    // Unmask EXTI5 interrupt
    volatile uint32_t& imr1 = STM32_REG32(EXTI_BASE + 0x080UL);
    imr1 |= (1u << 5u);

    // NVIC: EXTI5 = IRQ 23 (EXTI[9:5] on STM32H5)
    nvic_set_priority(23u, 8u);
    nvic_enable_irq(23u);
}

void flex_fuel_edge_isr() noexcept {
    const uint32_t now_us = micros();
    const uint32_t now_ms = millis();

    // Read PB5 state to determine rising or falling
    const bool level = (STM32_REG32(GPIOB_BASE + GPIO_IDR_OFF) & (1u << 5u)) != 0u;

    if (level) {
        // Rising edge: measure period from last rising
        if (g_got_rising && g_last_rising_us != 0u) {
            g_period_us = now_us - g_last_rising_us;
        }
        g_last_rising_us = now_us;
        g_got_rising = true;
    } else {
        // Falling edge: measure high time for duty cycle
        if (g_got_rising) {
            g_high_us = now_us - g_last_rising_us;
        }
    }
    g_last_edge_ms = now_ms;

    // Clear EXTI5 pending
    constexpr uint32_t EXTI_BASE = 0x44022000UL;
    STM32_REG32(EXTI_BASE + 0x088UL) = (1u << 5u);  // RPR1 (rising pending)
    STM32_REG32(EXTI_BASE + 0x08CUL) = (1u << 5u);  // FPR1 (falling pending)
}

uint8_t flex_fuel_ethanol_pct() noexcept {
    if (g_period_us == 0u) { return 0u; }
    const uint32_t freq = 1000000u / g_period_us;
    if (freq <= kFreqMin) { return 0u; }
    if (freq >= kFreqMax) { return 100u; }
    return static_cast<uint8_t>(((freq - kFreqMin) * 100u) / (kFreqMax - kFreqMin));
}

int16_t flex_fuel_temp_x10() noexcept {
    if (g_period_us == 0u || g_high_us == 0u) { return -400; }
    // Duty cycle 10-90% maps to -40°C to 125°C
    const uint32_t duty_pct = (g_high_us * 100u) / g_period_us;
    if (duty_pct <= 10u) { return -400; }
    if (duty_pct >= 90u) { return 1250; }
    return static_cast<int16_t>(-400 + static_cast<int32_t>((duty_pct - 10u) * 16500u) / 800);
}

bool flex_fuel_valid() noexcept {
    return (millis() - g_last_edge_ms) < kTimeoutMs && g_period_us > 0u;
}

}  // namespace ems::hal

extern "C" void EXTI5_9_IRQHandler() noexcept {
    ems::hal::flex_fuel_edge_isr();
}

#else  // host test stub

namespace ems::hal {
void flex_fuel_init() noexcept {}
void flex_fuel_edge_isr() noexcept {}
uint8_t flex_fuel_ethanol_pct() noexcept { return 0u; }
int16_t flex_fuel_temp_x10() noexcept { return 200; }
bool flex_fuel_valid() noexcept { return false; }
}

#endif
