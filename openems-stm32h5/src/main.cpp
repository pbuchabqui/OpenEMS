// =============================================================================
// OpenEMS — src/main_stm32.cpp
// STM32H562RGT6 (ARM Cortex-M33 @ 250 MHz)
//
// Entry point para a versão STM32H562 do firmware.
// Substitui src/main.cpp da versão Kinetis/Teensyduino.
//
// Diferenças em relação ao main.cpp original:
//   - Sem #include <Arduino.h> / Teensyduino
//   - PLL 250 MHz via system_stm32_init() (em vez de runtime Teensyduino)
//   - millis() provido por SysTick_Handler em system.cpp
//   - pit1_kick() → iwdg_kick() (IWDG em vez de PIT1)
//   - PIT0 (timestamp µs) → SysTick no system.cpp
//   - NVIC setup usa IRQs do STM32H562 (IRQ_TIM5=50, IRQ_TIM1_CC=27)
//   - Sem pit_init() — SysTick e IWDG já inicializados em system_stm32_init()
//   - nvm_flush_adaptive_maps() no slot 500ms (Flash Bank2)
// =============================================================================

#if defined(EMS_HOST_TEST)

int main() { return 0; }

#else  // ── target STM32H562 ────────────────────────────────────────────────

#include <cstdint>

#include "hal/system.h"
#include "hal/regs.h"

#include "app/can_stack.h"
#include "app/tuner_studio.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/cycle_sched.h"
#include "engine/ecu_sched.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/quick_crank.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/flexnvm.h"
#include "hal/runtime_seed.h"
#include "hal/ftm.h"
#include "hal/uart.h"

// =============================================================================
// Estado de background (idêntico ao main.cpp Kinetis)
// =============================================================================

static constexpr uint16_t kCalibPageBytes = 512u;
alignas(4) static uint8_t g_calib_page0[kCalibPageBytes];
static bool                g_calib_dirty  = false;

// g_datalog_us: no STM32 usa micros() de system.cpp em vez de ISR PIT0
// Mantemos a variável para compatibilidade com o frame CAN 0x400
volatile uint32_t g_datalog_us = 0u;

static uint32_t g_t2ms   = 0u;
static uint32_t g_t10ms  = 0u;
static uint32_t g_t20ms  = 0u;
static uint32_t g_t50ms  = 0u;
static uint32_t g_t100ms = 0u;
static uint32_t g_t500ms = 0u;

static int8_t  g_last_advance_deg = 0;
static uint8_t g_last_pw_ms_x10   = 0u;

static constexpr uint32_t kLimpRpmLimit_x10 = 30000u;
static constexpr uint8_t  kFaultBitMap = (1u << 0u);
static constexpr uint8_t  kFaultBitClt = (1u << 3u);
static bool g_limp_active = false;
static bool g_engine_was_running = false;
static bool g_runtime_seed_saved_for_stop = false;
static bool g_runtime_seed_arm_window_active = false;
static uint32_t g_prev_rpm_x10 = 0u;
static uint32_t g_zero_rpm_since_ms = 0u;
static uint32_t g_runtime_seed_arm_window_start_ms = 0u;
static bool g_have_last_full_sync = false;
static ems::drv::CkpSnapshot g_last_full_sync_snapshot = {
    0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false
};
static bool g_have_last_gap_sync = false;
static ems::drv::CkpSnapshot g_last_gap_sync_snapshot = {
    0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false
};

static constexpr uint32_t kRuntimeSeedSaveDelayMs = 100u;
static constexpr uint32_t kRuntimeSeedArmWindowMs = 2000u;
static constexpr uint16_t kDefaultReqFuelUs = 8000u;
static constexpr uint16_t kMapRefKpa        = 100u;
static constexpr uint32_t kDefaultSoiLeadDeg = 62u;

// =============================================================================
// Utilitários
// =============================================================================

[[noreturn]] static inline void system_reset() noexcept {
    // ARM Cortex-M33: AIRCR reset (idêntico ao M4)
    *reinterpret_cast<volatile uint32_t*>(0xE000ED0Cu) =
        (0x5FAu << 16u) | (1u << 2u);
    for (;;) {}
}

static inline bool elapsed(uint32_t now, uint32_t last, uint32_t period) noexcept {
    return static_cast<uint32_t>(now - last) >= period;
}

static inline void ts_service() noexcept {
    ems::hal::uart0_poll_rx(64u);
    ems::app::ts_process();
    uint8_t b = 0u;
    for (uint16_t n = 0u;
         n < 96u && ems::hal::uart0_tx_ready() && ems::app::ts_tx_pop(b);
         ++n) {
        if (!ems::hal::uart0_tx_byte(b)) { break; }
    }
}

// =============================================================================
// Inicialização — sequência idêntica ao main.cpp Kinetis
// =============================================================================

static void openems_init() noexcept {
    // 1) PLL → 250 MHz + SysTick 1ms + IWDG 100ms
    system_stm32_init();

    // 2) Timers (TIM5=CKP IC, TIM1=OC ignição, TIM3/TIM4=PWM)
    ems::hal::ftm0_init();   // → TIM1 output compare
    ems::hal::ftm3_init();   // → TIM5 input capture (CKP + CMP)
    ems::hal::ftm1_pwm_init(125u);  // → TIM3 PWM (IACV + Wastegate)
    ems::hal::ftm2_pwm_init(150u);  // → TIM4 PWM (VVT)

    // 2a) Scheduler unificado
    ::ECU_Hardware_Init();

    // 3) ADC (ADC1/ADC2 + TIM6 trigger)
    ems::hal::adc_init();

    // 4) CAN + UART
    ems::hal::can0_init();
    ems::hal::uart0_init();

    // 5) Flash Bank2 → carrega calibração page-0
    static_cast<void>(
        ems::hal::nvm_load_calibration(0u, g_calib_page0, kCalibPageBytes));
    {
        ems::hal::RuntimeSyncSeed seed = {};
        if (ems::hal::nvm_load_runtime_seed(&seed) &&
            ems::hal::runtime_seed_fast_reacquire_compatible_60_2(seed)) {
            const bool phase_a =
                ((seed.flags & ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A) != 0u);
            ems::drv::ckp_seed_arm(phase_a);
            g_runtime_seed_arm_window_active = true;
            g_runtime_seed_arm_window_start_ms = millis();
            static_cast<void>(ems::hal::nvm_clear_runtime_seed());
        }
    }

    // 6) Drivers
    ems::drv::sensors_init();

    // 7) Engine
    ems::engine::fuel_reset_adaptives();
    ems::engine::auxiliaries_init();
    ems::engine::knock_init();
    ems::engine::quick_crank_reset();
    ems::engine::cycle_sched_init();

    // 8) Aplicação
    ems::app::ts_init();
    ems::app::can_stack_init();

    // 9) NVIC — mesma hierarquia de prioridades do Kinetis
    //    TIM5 (CKP) = prio 1 (máxima), TIM1_CC (ignição) = prio 4
    //    SysTick configurado em system_stm32_init() com prio 11
    //    IWDG: hardware — sem NVIC necessário
    nvic_set_priority(IRQ_TIM5,    1u);   // CKP
    nvic_enable_irq(IRQ_TIM5);
    nvic_set_priority(IRQ_TIM1_CC, 4u);   // ignição/injeção
    nvic_enable_irq(IRQ_TIM1_CC);

    // 10) Aguardar CKP sync (timeout 5 s)
    const uint32_t sync_deadline = millis() + 5000u;
    while (millis() < sync_deadline) {
        iwdg_kick();
        const auto snap = ems::drv::ckp_snapshot();
        if (snap.state == ems::drv::SyncState::FULL_SYNC) { break; }
    }
}

// =============================================================================
// main() — substituição do setup()/loop() do Teensyduino
// =============================================================================

int main() {
    openems_init();

    uint32_t g_t2ms_   = millis();
    uint32_t g_t10ms_  = g_t2ms_;
    uint32_t g_t20ms_  = g_t2ms_;
    uint32_t g_t50ms_  = g_t2ms_;
    uint32_t g_t100ms_ = g_t2ms_;
    uint32_t g_t500ms_ = g_t2ms_;

    for (;;) {
        // ── Watchdog kick (primeiro statement) ───────────────────────────
        iwdg_kick();
        g_datalog_us = micros();

        const uint32_t now = millis();

        // ── 2ms: fuel + ign recalc + commit calibration ───────────────────
        if (elapsed(now, g_t2ms_, 2u)) {
            g_t2ms_ = now;

            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            const bool synced  = (snap.state == ems::drv::SyncState::FULL_SYNC);

            if (g_runtime_seed_arm_window_active) {
                if (elapsed(now, g_runtime_seed_arm_window_start_ms,
                            kRuntimeSeedArmWindowMs)) {
                    ems::drv::ckp_seed_disarm();
                    g_runtime_seed_arm_window_active = false;
                }
            }

            if (snap.state == ems::drv::SyncState::FULL_SYNC) {
                g_have_last_full_sync = true;
                g_last_full_sync_snapshot = snap;
            }
            if (snap.state == ems::drv::SyncState::HALF_SYNC ||
                snap.state == ems::drv::SyncState::FULL_SYNC) {
                g_have_last_gap_sync = true;
                g_last_gap_sync_snapshot = snap;
            }

            const bool rev_cut = g_limp_active &&
                (snap.rpm_x10 > kLimpRpmLimit_x10);

            if (synced && !rev_cut) {
                const uint16_t map_kpa = sensors.map_kpa;
                const uint8_t  ve = ems::engine::get_ve(snap.rpm_x10, map_kpa);
                const uint32_t base_pw_us =
                    ems::engine::calc_base_pw_us(kDefaultReqFuelUs, ve,
                                                  map_kpa, kMapRefKpa);
                const uint16_t corr_clt_x256 =
                    ems::engine::corr_clt(sensors.clt_x10);
                const uint16_t corr_iat_x256 =
                    ems::engine::corr_iat(sensors.iat_x10);
                const uint16_t dead_time_us =
                    ems::engine::corr_vbatt(sensors.vbatt_mv);
                const uint32_t final_pw_us =
                    ems::engine::calc_final_pw_us(base_pw_us,
                                                   corr_clt_x256,
                                                   corr_iat_x256,
                                                   dead_time_us);
                g_last_pw_ms_x10 = static_cast<uint8_t>(
                    (final_pw_us / 100u) > 255u ? 255u : (final_pw_us / 100u));

                const int16_t advance_deg10 =
                    ems::engine::calc_advance(snap.rpm_x10, map_kpa,
                                              sensors.clt_x10, sensors.iat_x10);
                g_last_advance_deg = static_cast<int8_t>(advance_deg10 / 10);

                ems::engine::ecu_sched_commit_calibration(
                    final_pw_us, static_cast<uint16_t>(advance_deg10),
                    kDefaultSoiLeadDeg, snap);
            }

            // Limp mode: MAP ou CLT em fault
            const bool map_fault = (sensors.fault_bits & kFaultBitMap) != 0u;
            const bool clt_fault = (sensors.fault_bits & kFaultBitClt) != 0u;
            g_limp_active = map_fault || clt_fault;
        }

        // ── 10ms: IACV, VVT, wastegate PID ───────────────────────────────
        if (elapsed(now, g_t10ms_, 10u)) {
            g_t10ms_ = now;
            const auto sensors = ems::drv::sensors_get();
            const auto snap    = ems::drv::ckp_snapshot();
            ems::engine::auxiliaries_tick_10ms(sensors, snap.rpm_x10);
        }

        // ── 20ms: TunerStudio + aux tasks ────────────────────────────────
        if (elapsed(now, g_t20ms_, 20u)) {
            g_t20ms_ = now;
            ts_service();
            const auto sensors = ems::drv::sensors_get();
            const auto snap    = ems::drv::ckp_snapshot();
            ems::engine::auxiliaries_tick_20ms(sensors, snap.rpm_x10);
        }

        // ── 50ms: sensores lentos + CAN 0x400 ────────────────────────────
        if (elapsed(now, g_t50ms_, 50u)) {
            g_t50ms_ = now;
            g_datalog_us = micros();
            ems::drv::sensors_tick_50ms();
            const auto snap = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            ems::app::can_stack_tick_50ms(snap, sensors,
                                           g_last_advance_deg,
                                           g_last_pw_ms_x10,
                                           g_datalog_us);
        }

        // ── 100ms: sensores + CAN 0x401 + STFT ───────────────────────────
        if (elapsed(now, g_t100ms_, 100u)) {
            g_t100ms_ = now;
            ems::drv::sensors_tick_100ms();
            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            ems::app::can_stack_tick_100ms(snap, sensors);

            if (snap.state == ems::drv::SyncState::FULL_SYNC) {
                ems::engine::fuel_update_stft(
                    snap.rpm_x10, sensors.map_kpa,
                    1000, sensors.o2_mv,
                    sensors.clt_x10, sensors.o2_valid,
                    false, false);
            }

            // Runtime seed — salva posição para re-sincronização rápida
            const uint32_t rpm = snap.rpm_x10;
            if (rpm > 0u) {
                g_engine_was_running = true;
                g_zero_rpm_since_ms  = 0u;
                g_runtime_seed_saved_for_stop = false;
            } else {
                if (g_engine_was_running && g_zero_rpm_since_ms == 0u) {
                    g_zero_rpm_since_ms = now;
                }
                if (g_engine_was_running && !g_runtime_seed_saved_for_stop &&
                    g_zero_rpm_since_ms != 0u &&
                    elapsed(now, g_zero_rpm_since_ms, kRuntimeSeedSaveDelayMs) &&
                    g_have_last_full_sync) {
                    ems::hal::RuntimeSyncSeed seed = {};
                    seed.flags = static_cast<uint8_t>(
                        ems::hal::RUNTIME_SYNC_SEED_FLAG_VALID |
                        ems::hal::RUNTIME_SYNC_SEED_FLAG_FULL_SYNC |
                        (g_last_full_sync_snapshot.phase_A
                             ? ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A : 0u));
                    seed.tooth_index = g_last_full_sync_snapshot.tooth_index;
                    seed.decoder_tag =
                        ems::hal::RUNTIME_SYNC_SEED_DECODER_TAG_60_2;
                    static_cast<void>(ems::hal::nvm_save_runtime_seed(&seed));
                    g_runtime_seed_saved_for_stop = true;
                }
            }
        }

        // ── 500ms: flush Flash (LTFT + knock + calibração) ────────────────
        if (elapsed(now, g_t500ms_, 500u)) {
            g_t500ms_ = now;
            if (g_calib_dirty) {
                static_cast<void>(
                    ems::hal::nvm_save_calibration(0u, g_calib_page0,
                                                    kCalibPageBytes));
                g_calib_dirty = false;
            }
            // Flush LTFT + knock maps para Flash Bank2 (STM32-specific)
            static_cast<void>(ems::hal::nvm_flush_adaptive_maps());
        }

        // ── CAN RX (polling contínuo) ─────────────────────────────────────
        ems::hal::CanFrame rx_frame{};
        while (ems::hal::can0_rx_pop(rx_frame)) {
            ems::app::can_stack_rx(rx_frame);
        }
    }
}

#endif  // EMS_HOST_TEST
