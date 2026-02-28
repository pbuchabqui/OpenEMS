#if defined(EMS_HOST_TEST)
int main() {
    return 0;
}
#else

#include <Arduino.h>

#include <cstdint>

#include "app/can_stack.h"
#include "app/tuner_studio.h"
#include "drv/ckp.h"
#include "drv/scheduler.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/flexnvm.h"
#include "hal/ftm.h"
#include "hal/uart.h"

namespace {

// NVIC register access (MK64F Cortex-M4)
#define NVIC_ISER(n) (*reinterpret_cast<volatile uint32_t*>(0xE000E100u + ((n) * 4u)))

constexpr uint32_t kNvicIprBase = 0xE000E400u;

constexpr uint32_t kIrqAdc0 = 39u;
constexpr uint32_t kIrqFtm0 = 42u;
constexpr uint32_t kIrqFtm3 = 71u;
constexpr uint32_t kIrqPit0 = 68u;
constexpr uint32_t kIrqPit1 = 69u;

// PIT register map
#define SIM_SCGC6_PIT_MASK (1u << 23u)

constexpr uint32_t kPitBusHz = 60000000u;
constexpr uint32_t kPit0Hz = 1000000u;
constexpr uint32_t kPit1TimeoutMs = 100u;

constexpr uint32_t kPit0Ldval = (kPitBusHz / kPit0Hz) - 1u;
constexpr uint32_t kPit1Ldval = ((kPitBusHz / 1000u) * kPit1TimeoutMs) - 1u;
constexpr uint16_t kTsRxBudgetBytes = 64u;
constexpr uint16_t kTsTxBudgetBytes = 96u;

alignas(4) static uint8_t g_calib_page0[512] = {};

volatile uint32_t g_datalog_ticks_1us = 0u;
volatile bool g_wdog_flag = false;

uint32_t g_last_5ms = 0u;
uint32_t g_last_10ms = 0u;
uint32_t g_last_20ms = 0u;
uint32_t g_last_50ms = 0u;
uint32_t g_last_100ms = 0u;
uint32_t g_last_500ms = 0u;

bool g_calib_dirty = false;

inline void nvic_set_priority_enable(uint32_t irq, uint8_t priority) noexcept {
    volatile uint32_t* const ipr = reinterpret_cast<volatile uint32_t*>(kNvicIprBase + (irq / 4u) * 4u);
    const uint32_t shift = (irq % 4u) * 8u;
    const uint32_t prio = static_cast<uint32_t>(priority) << 4u;

    *ipr = (*ipr & ~(0xFFu << shift)) | (prio << shift);
    NVIC_ISER(irq / 32u) = (1u << (irq % 32u));
}

inline void pdb_init() noexcept {
    // adc_init() já configura PDB0 no HAL atual.
}

inline void can_init() noexcept {
    ems::hal::can0_init();
}

inline void flexnvm_init() noexcept {
    static_cast<void>(ems::hal::nvm_load_calibration(0u, g_calib_page0, sizeof(g_calib_page0)));
}

inline void ckp_init() noexcept {
    // Decoder inicializa estado estático em zero; sem init dedicado no módulo atual.
}

inline void sched_init() noexcept {
    ems::drv::sched_cancel_all();
}

inline void fuel_init() noexcept {
    ems::engine::fuel_reset_adaptives();
}

inline void ign_init() noexcept {
    // ign_calc usa tabelas estáticas; sem init dedicado no módulo atual.
}

inline void aux_init() noexcept {
    ems::engine::auxiliaries_init();
}

inline void pit_init() noexcept {
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT_MCR = 0u;

    PIT_LDVAL0 = kPit0Ldval;
    PIT_TFLG0 = PIT_TFLG_TIF;
    PIT_TCTRL0 = PIT_TCTRL_TEN | PIT_TCTRL_TIE;

    PIT_LDVAL1 = kPit1Ldval;
    PIT_TFLG1 = PIT_TFLG_TIF;
    PIT_TCTRL1 = PIT_TCTRL_TEN | PIT_TCTRL_TIE;
}

inline void pit1_kick() noexcept {
    PIT_TCTRL1 &= ~PIT_TCTRL_TEN;
    PIT_TFLG1 = PIT_TFLG_TIF;
    PIT_LDVAL1 = kPit1Ldval;
    PIT_TCTRL1 |= PIT_TCTRL_TEN;
}

[[noreturn]] inline void system_reset() noexcept {
    volatile uint32_t& scb_aircr = *reinterpret_cast<volatile uint32_t*>(0xE000ED0Cu);
    constexpr uint32_t kVectkey = 0x5FAu << 16u;
    constexpr uint32_t kSysresetreq = 1u << 2u;
    scb_aircr = kVectkey | kSysresetreq;
    while (true) {
    }
}

inline bool elapsed_ms(uint32_t now, uint32_t last, uint32_t period) noexcept {
    return static_cast<uint32_t>(now - last) >= period;
}

inline void ts_service_budgeted() noexcept {
    ems::hal::uart0_poll_rx(kTsRxBudgetBytes);
    ems::app::ts_process();

    uint8_t txb = 0u;
    uint16_t tx_count = 0u;
    while (tx_count < kTsTxBudgetBytes) {
        if (!ems::hal::uart0_tx_ready()) {
            break;
        }
        if (!ems::app::ts_tx_pop(txb)) {
            break;
        }
        if (!ems::hal::uart0_tx_byte(txb)) {
            break;
        }
        ++tx_count;
    }
}

inline void task_5ms(const ems::drv::CkpSnapshot& ckp, const ems::drv::SensorData& sensors) noexcept {
    if (ckp.state != ems::drv::SyncState::SYNCED) {
        return;
    }

    const uint8_t ve = ems::engine::get_ve(static_cast<uint16_t>(ckp.rpm_x10), static_cast<uint16_t>(sensors.map_kpa_x10 / 10u));
    const uint32_t base_pw = ems::engine::calc_base_pw_us(3000u, ve, static_cast<uint16_t>(sensors.map_kpa_x10 / 10u), 100u);
    const uint32_t final_pw = ems::engine::calc_final_pw_us(base_pw,
                                                            ems::engine::corr_clt(sensors.clt_degc_x10),
                                                            ems::engine::corr_iat(sensors.iat_degc_x10),
                                                            ems::engine::corr_vbatt(sensors.vbatt_mv));
    static_cast<void>(ems::engine::inj_pw_us_to_ftm0_ticks(final_pw));

    const int16_t adv = ems::engine::get_advance(static_cast<uint16_t>(ckp.rpm_x10), static_cast<uint16_t>(sensors.map_kpa_x10 / 10u));
    static_cast<void>(ems::engine::build_ign_schedule(0u,
                                                      static_cast<int16_t>(adv * 10),
                                                      ems::engine::dwell_ms_x10_from_vbatt(sensors.vbatt_mv),
                                                      static_cast<uint16_t>(ckp.rpm_x10 / 10u)));
}

inline void task_50ms_can(uint32_t now,
                          const ems::drv::CkpSnapshot& ckp,
                          const ems::drv::SensorData& sensors) noexcept {
    const int16_t adv = ems::engine::get_advance(static_cast<uint16_t>(ckp.rpm_x10), static_cast<uint16_t>(sensors.map_kpa_x10 / 10u));
    const int8_t adv_i8 = static_cast<int8_t>((adv < -128) ? -128 : ((adv > 127) ? 127 : adv));

    const int16_t stft_x10 = ems::engine::fuel_get_stft_pct_x10();
    const int8_t stft_i8 = static_cast<int8_t>(stft_x10 / 10);

    uint8_t status = 0u;
    if (ckp.state == ems::drv::SyncState::SYNCED) {
        status = static_cast<uint8_t>(status | 0x01u);
    }
    if (ckp.phase_A) {
        status = static_cast<uint8_t>(status | 0x02u);
    }
    if (sensors.fault_bits != 0u) {
        status = static_cast<uint8_t>(status | 0x04u);
    }

    ems::app::can_stack_process(now, ckp, sensors, adv_i8, 0u, stft_i8, 0u, 0u, status);
}

}  // namespace

extern "C" void ADC0_IRQHandler(void) {
    // Reservado para pipeline ADC por interrupção.
}

extern "C" void PIT0_IRQHandler(void) {
    PIT_TFLG0 = PIT_TFLG_TIF;
    ++g_datalog_ticks_1us;
}

extern "C" void PIT1_IRQHandler(void) {
    PIT_TFLG1 = PIT_TFLG_TIF;
    g_wdog_flag = true;
    system_reset();
}

void setup() {
    // 1) Clocks PLL -> 120 MHz (Teensyduino runtime)

    // 2) FTMs
    ems::hal::ftm0_init();
    ems::hal::ftm3_init();
    ems::hal::ftm1_pwm_init(125u);
    ems::hal::ftm2_pwm_init(150u);

    // 3) ADC + PDB
    ems::hal::adc_init();
    pdb_init();

    // 4) CAN + UART0
    can_init();
    ems::hal::uart0_init();

    // 5) FlexNVM -> calibração RAM
    flexnvm_init();

    // 6) Drivers
    ckp_init();
    sched_init();
    ems::drv::sensors_init();

    // 7) Engine
    fuel_init();
    ign_init();
    aux_init();
    ems::engine::knock_init();

    // 8) App
    ems::app::ts_init();
    ems::app::can_stack_init();

    // 9) NVIC priorities
    nvic_set_priority_enable(kIrqFtm3, 1u);
    nvic_set_priority_enable(kIrqFtm0, 4u);
    nvic_set_priority_enable(kIrqAdc0, 5u);
    nvic_set_priority_enable(kIrqPit0, 12u);
    nvic_set_priority_enable(kIrqPit1, 11u);

    // PIT timestamp + watchdog (Prompt 11)
    pit_init();

    // 10) Aguardar SYNCED antes de agendar eventos.
    while (ems::drv::ckp_snapshot().state != ems::drv::SyncState::SYNCED) {
        ts_service_budgeted();
        pit1_kick();
    }

    const uint32_t now = millis();
    g_last_5ms = now;
    g_last_10ms = now;
    g_last_20ms = now;
    g_last_50ms = now;
    g_last_100ms = now;
    g_last_500ms = now;
}

void loop() {
    pit1_kick();

    const uint32_t now = millis();
    const ems::drv::CkpSnapshot ckp = ems::drv::ckp_snapshot();
    const ems::drv::SensorData& sensors = ems::drv::sensors_get();

    if (elapsed_ms(now, g_last_5ms, 5u)) {
        g_last_5ms = now;
        task_5ms(ckp, sensors);
    }

    if (elapsed_ms(now, g_last_10ms, 10u)) {
        g_last_10ms = now;
        ems::engine::auxiliaries_tick_10ms();
    }

    if (elapsed_ms(now, g_last_20ms, 20u)) {
        g_last_20ms = now;
        ts_service_budgeted();
    }

    if (elapsed_ms(now, g_last_50ms, 50u)) {
        g_last_50ms = now;
        ems::drv::sensors_tick_50ms();
        task_50ms_can(now, ckp, sensors);
    }

    if (elapsed_ms(now, g_last_100ms, 100u)) {
        g_last_100ms = now;
        ems::drv::sensors_tick_100ms();

        const bool o2_valid = ems::app::can_stack_wbo2_fresh(now);
        const int16_t lambda_meas = static_cast<int16_t>(ems::app::can_stack_lambda_milli());
        static_cast<void>(ems::engine::fuel_update_stft(static_cast<uint16_t>(ckp.rpm_x10),
                                                        static_cast<uint16_t>(sensors.map_kpa_x10 / 10u),
                                                        1000,
                                                        lambda_meas,
                                                        sensors.clt_degc_x10,
                                                        o2_valid,
                                                        false,
                                                        false));
    }

    if (elapsed_ms(now, g_last_500ms, 500u)) {
        g_last_500ms = now;
        if (g_calib_dirty) {
            static_cast<void>(ems::hal::nvm_save_calibration(0u, g_calib_page0, sizeof(g_calib_page0)));
            g_calib_dirty = false;
        }
    }
}

#endif
