// =============================================================================
// OpenEMS — src/main_stm32.cpp
// STM32H562RGT6 (ARM Cortex-M33 @ 250 MHz)
//
// Entry point STM32H562 do firmware.
//
// Configuracao principal:
//   - PLL 250 MHz via system_stm32_init()
//   - millis() provido por SysTick_Handler em system.cpp
//   - iwdg_kick() alimenta o watchdog IWDG
//   - SysTick fornece millis()/micros()
//   - NVIC setup usa IRQs do STM32H562 (TIM5 para CKP; TIM2/TIM8 são OC por hardware)
//   - SysTick e IWDG já inicializados em system_stm32_init()
//   - nvm_flush_adaptive_maps() no slot 500ms (Flash Bank2)
// =============================================================================

#if defined(EMS_HOST_TEST)

int main() { return 0; }

#elif defined(TARGET_STM32H562)  // ── target STM32H562 ─────────────────────

#include <cstdint>
#include <cstring>

#include "engine/math_utils.h"

#include "hal/stm32h562/system.h"
#include "hal/stm32h562/regs.h"
#include "hal/stm32h562/usb_cdc.h"
#include "hal/uart.h"

#include "app/can_stack.h"
#include "app/ui_protocol.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/calibration.h"
#include "engine/constants.h"
#include "engine/ecu_sched.h"
#include "engine/engine_config.h"
#include "engine/etb_control.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/map_estimator.h"
#include "engine/quick_crank.h"
#include "engine/torque_manager.h"
#include "engine/transient_fuel.h"
#include "engine/xtau_autocalib.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/etb_driver.h"
#include "hal/flash.h"
#include "hal/runtime_seed.h"
#include "hal/timer.h"
#include "hal/etb_driver.h"

// =============================================================================
// Estado de background (do firmware)
// =============================================================================

static constexpr uint16_t kCalibPageBytes = 512u;
alignas(4) static uint8_t g_calib_page0[kCalibPageBytes];
static bool                g_calib_dirty  = false;

// g_datalog_us: no STM32 usa micros() de system.cpp em vez de SysTick
// Mantemos a variável para quadro CAN 0x400
volatile uint32_t g_datalog_us = 0u;
volatile uint32_t g_flash_write_faults = 0u; // FIX: fault counter para falhas de escrita NVM


static int8_t  g_last_advance_deg = 0;
static uint8_t g_last_pw_ms_x10   = 0u;
static int8_t  g_last_stft_pct    = 0;

static constexpr uint32_t kLimpRpmLimit_x10 = 30000u;
static constexpr uint8_t  kFaultBitMap = (1u << 0u);
static constexpr uint8_t  kFaultBitClt = (1u << 3u);
static bool g_limp_active = false;
static bool g_engine_was_running = false;
static bool g_runtime_seed_saved_for_stop = false;
static bool g_runtime_seed_arm_window_active = false;
static bool g_ae_active = false;
static uint32_t g_zero_rpm_since_ms = 0u;
static uint32_t g_runtime_seed_arm_window_start_ms = 0u;
static uint16_t g_prev_tps_pct_x10 = 0u;
static bool g_have_last_full_sync = false;
static ems::drv::CkpSnapshot g_last_full_sync_snapshot = {
    0u, 0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false
};
static bool g_have_last_gap_sync = false;
static ems::drv::CkpSnapshot g_last_gap_sync_snapshot = {
    0u, 0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false
};
static uint32_t g_loop2ms_last_us = 0u;
static uint32_t g_loop2ms_max_us = 0u;

static constexpr uint32_t kRuntimeSeedSaveDelayMs = 100u;
static constexpr uint32_t kRuntimeSeedArmWindowMs = 300000u;  // 5 minutos para start-stop
static constexpr uint32_t kSchedulerTicksPerMs = 10000u;
static constexpr uint32_t kCalibSaveMinIntervalMs = 300000u;
static constexpr uint16_t kMapMinKpa = 10u;
static constexpr uint16_t kMapMaxKpa = 300u;
static constexpr uint16_t kLambdaMinMilli = 700u;
static constexpr uint16_t kLambdaMaxMilli = 1400u;
static constexpr uint16_t kAePeriodMs = 2u;

// =============================================================================
// Estado ETB e Torque Manager
// =============================================================================

static torque_manager_inputs_t g_torque_inputs = {0};
static torque_manager_outputs_t g_torque_outputs = {0};
static bool g_etb_initialized = false;
static uint32_t g_etb_loop_timer_ms = 0;

// =============================================================================
// Utilitários
// =============================================================================

static inline bool elapsed(uint32_t now, uint32_t last, uint32_t period) noexcept {
    return static_cast<uint32_t>(now - last) >= period;
}

static inline uint16_t build_status_bits(const ems::drv::CkpSnapshot& snap,
                                           const ems::drv::SensorData& sensors) noexcept {
    uint16_t status = 0u;
    if (snap.state == ems::drv::SyncState::FULL_SYNC) {
        status |= ems::app::STATUS_SYNC_FULL;
    }
    if (snap.phase_A) {
        status |= ems::app::STATUS_PHASE_A;
    }
    if (sensors.fault_bits != 0u) {
        status |= ems::app::STATUS_SENSOR_FAULT;
    }
    return status;
}

using ems::engine::clamp_u16;
using ems::engine::clamp_i16;

static inline int8_t clamp_i8(int16_t v, int8_t lo, int8_t hi) noexcept {
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return static_cast<int8_t>(v);
}

static bool page_range_is_zero(const uint8_t* data, uint16_t off, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (data[off + i] != 0u) {
            return false;
        }
    }
    return true;
}

static bool page_range_is_erased(const uint8_t* data, uint16_t off, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (data[off + i] != 0xFFu) { return false; }
    }
    return true;
}

static bool page_is_erased(const uint8_t* data, uint16_t len) noexcept {
    return page_range_is_erased(data, 0u, len);
}

static void load_corr_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[256] = {};
    if (!ems::hal::nvm_load_calibration(4u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;
    }

    const uint8_t* p = page;
    std::memcpy(ems::engine::clt_corr_axis_x10,          p +   0, 16u);
    std::memcpy(ems::engine::clt_corr_x256,              p +  16, 16u);
    std::memcpy(ems::engine::iat_corr_axis_x10,          p +  32, 16u);
    std::memcpy(ems::engine::iat_corr_x256,              p +  48, 16u);
    std::memcpy(ems::engine::warmup_corr_axis_x10,       p +  64, 16u);
    std::memcpy(ems::engine::warmup_corr_x256,           p +  80, 16u);
    std::memcpy(ems::engine::vbatt_corr_axis_mv,         p +  96, 16u);
    std::memcpy(ems::engine::injector_dead_time_us,      p + 112, 16u);
    std::memcpy(ems::engine::ae_clt_corr_axis_x10,       p + 128, 16u);
    std::memcpy(ems::engine::ae_clt_sens,                p + 144, 16u);
    std::memcpy(ems::engine::dwell_vbatt_axis_mv,        p + 160, 16u);
    std::memcpy(ems::engine::dwell_ms_x10_table,         p + 176, 16u);
    std::memcpy(ems::engine::lambda_delay_rpm_axis_x10,  p + 192, 12u);
    std::memcpy(ems::engine::lambda_delay_load_axis_kpa, p + 204, 12u);
    std::memcpy(ems::engine::lambda_delay_ms_table,      p + 216, 18u);
    if (!page_range_is_zero(page, 234u, 6u)) {
        std::memcpy(&ems::engine::ae_tpsdot_threshold_x10, p + 234, 2u);
        std::memcpy(&ems::engine::ae_taper_cycles,         p + 236, 2u);
        std::memcpy(&ems::engine::ae_max_pw_us,            p + 238, 2u);
    }
    if (page_range_is_zero(page, 240u, 16u)) {
        return;
    }
    std::memcpy(&ems::engine::idle_spark_tps_max_x10,             p + 240, 2u);
    std::memcpy(&ems::engine::idle_spark_map_max_kpa,             p + 242, 2u);
    std::memcpy(&ems::engine::idle_spark_rpm_min_x10,             p + 244, 2u);
    std::memcpy(&ems::engine::idle_spark_window_above_target_x10, p + 246, 2u);
    std::memcpy(&ems::engine::idle_spark_deadband_rpm_x10,        p + 248, 2u);
    std::memcpy(&ems::engine::idle_spark_rpm_per_deg_x10,         p + 250, 2u);
    std::memcpy(&ems::engine::idle_spark_retard_limit_deg,        p + 252, 2u);
    std::memcpy(&ems::engine::idle_spark_advance_limit_deg,       p + 254, 2u);
}

static void load_xtau_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[80] = {};
    if (!ems::hal::nvm_load_calibration(5u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page)) ||
        page_range_is_zero(page, 0u, 48u)) {
        return;
    }

    const uint8_t* p = page;
    std::memcpy(ems::engine::xtau_clt_axis_x10,  p +  0, 16u);
    std::memcpy(ems::engine::xtau_x_fraction_q8, p + 16, 16u);
    std::memcpy(ems::engine::xtau_tau_cycles,    p + 32, 16u);
    if (!page_range_is_zero(page, 48u, 16u)) {
        std::memcpy(ems::engine::ae_tpsdot_axis_x10, p + 48, 8u);
        std::memcpy(ems::engine::ae_pw_adder_us,     p + 56, 8u);
    }
    if (!page_range_is_zero(page, 64u, 12u) &&
        !page_range_is_erased(page, 64u, 12u)) {
        std::memcpy(&ems::engine::crank_enter_rpm_x10,   p + 64, 2u);
        std::memcpy(&ems::engine::crank_exit_rpm_x10,    p + 66, 2u);
        std::memcpy(&ems::engine::crank_spark_deg,       p + 68, 2u);
        std::memcpy(&ems::engine::crank_min_pw_us,       p + 70, 2u);
        std::memcpy(&ems::engine::crank_prime_tooth,     p + 72, 2u);
        std::memcpy(&ems::engine::crank_prime_max_pw_us, p + 74, 2u);
    }
}

struct CachedFuelCorrections {
    bool valid;
    int16_t clt_x10;
    int16_t iat_x10;
    uint16_t vbatt_mv;
    uint16_t corr_clt_x256;
    uint16_t corr_iat_x256;
    uint16_t dead_time_us;
    uint16_t dwell_ms_x10;
};

static CachedFuelCorrections g_fuel_corr_cache = {};

static inline const CachedFuelCorrections& fuel_corrections_for(
    const ems::drv::SensorData& sensors) noexcept {
    if (!g_fuel_corr_cache.valid ||
        g_fuel_corr_cache.clt_x10 != sensors.clt_degc_x10 ||
        g_fuel_corr_cache.iat_x10 != sensors.iat_degc_x10 ||
        g_fuel_corr_cache.vbatt_mv != sensors.vbatt_mv) {
        g_fuel_corr_cache.valid = true;
        g_fuel_corr_cache.clt_x10 = sensors.clt_degc_x10;
        g_fuel_corr_cache.iat_x10 = sensors.iat_degc_x10;
        g_fuel_corr_cache.vbatt_mv = sensors.vbatt_mv;
        g_fuel_corr_cache.corr_clt_x256 = ems::engine::corr_clt(sensors.clt_degc_x10);
        g_fuel_corr_cache.corr_iat_x256 = ems::engine::corr_iat(sensors.iat_degc_x10);
        g_fuel_corr_cache.dead_time_us = ems::engine::corr_vbatt(sensors.vbatt_mv);
        g_fuel_corr_cache.dwell_ms_x10 = ems::engine::dwell_ms_x10_from_vbatt(sensors.vbatt_mv);
    }
    return g_fuel_corr_cache;
}

static inline void ui_service() noexcept {
    // ── UART path (transporte primário: adaptador USB-UART em PA9/PA10) ──
    ems::hal::uart0_poll_rx(64u);
    {
        uint8_t b = 0u;
        while (ems::hal::uart0_rx_pop(b)) {
            ems::app::ui_rx_byte(b);
        }
    }

    // ── USB CDC path (stub; no-op até driver real) ─────────────────────────
    ems::hal::usb_cdc_poll();
    if (ems::hal::usb_cdc_dtr()) {
        uint8_t rx_buf[64] = {};
        const uint16_t rx_n = ems::hal::usb_cdc_read_bytes(rx_buf, 64u);
        for (uint16_t i = 0u; i < rx_n; ++i) {
            ems::app::ui_rx_byte(rx_buf[i]);
        }
    }

    // ── Processar protocolo ────────────────────────────────────────────────
    ems::app::ui_process();

    // ── Drenar TX para ambos os transportes ───────────────────────────────
    uint8_t tx_buf[96] = {};
    uint16_t tx_n = 0u;
    while (tx_n < 96u && ems::app::ui_tx_pop(tx_buf[tx_n])) { ++tx_n; }
    if (tx_n != 0u) {
        for (uint16_t i = 0u; i < tx_n; ++i) {
            ems::hal::uart0_tx_push(tx_buf[i]);
        }
        ems::hal::usb_cdc_send_bytes(tx_buf, tx_n);
    }
    ems::hal::uart0_tx_poll(32u);  // máx 32 bytes/ciclo de 20 ms (~230 B budget)
}

// =============================================================================
// Inicialização — sequência idêntica ao main.cpp
// =============================================================================

static void openems_init() noexcept {
    // 1) PLL → 250 MHz + SysTick 1ms + IWDG 100ms
    system_stm32_init();

    // 2) Timers (TIM5=CKP IC, TIM2/TIM8=OC injeção/ignição)
    // TIM3/TIM4 PWM auxiliares são inicializados em auxiliaries_init().
    // ECU_Hardware_Init() owns TIM2/TIM8 for injection/ignition scheduling.
    ems::hal::tim5_ic_init();   // → TIM5 input capture (CKP + CMP)

    // 2a) Scheduler unificado
    ::ECU_Hardware_Init();

    // 3) ADC (ADC1/ADC2 + TIM6 trigger)
    ems::hal::adc_init();

    // 4) CAN + bench communication. MVP transport: USART1 PA9/PA10.
    ems::hal::can0_init();
    ems::hal::uart0_init(115200u);
    ems::hal::usb_cdc_init();

	// 5) Flash Bank2 → carrega calibrações persistidas
	if (!ems::hal::nvm_load_calibration(0u, g_calib_page0, kCalibPageBytes)) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
	ems::engine::cfg::engine_config_load(g_calib_page0, kCalibPageBytes);
	load_corr_calibration_from_nvm();
	load_xtau_calibration_from_nvm();
	if (!ems::hal::nvm_load_adaptive_maps()) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
    {
        ems::hal::RuntimeSyncSeed seed = {};
        if (ems::hal::nvm_load_runtime_seed(&seed) &&
            ems::hal::runtime_seed_fast_reacquire_compatible_60_2(seed)) {
            const bool phase_a =
                ((seed.flags & ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A) != 0u);
            ems::drv::ckp_seed_arm(phase_a);
            g_runtime_seed_arm_window_active = true;
            g_runtime_seed_arm_window_start_ms = millis();
	if (!ems::hal::nvm_clear_runtime_seed()) {
		++g_flash_write_faults; // FIX: rastrear falha de limpeza NVM
	}
        }
    }

    // 6) Drivers
    ems::drv::sensors_init();

    // 6a) Inicializa sistemas "invisíveis" ao motorista
    ems::engine::map_estimator_init();
    ems::engine::xtau_autocalib_init();

    // 6b) Inicializa ETB e Torque Manager (borboleta eletrônica)
    g_etb_initialized = etb_control_init();
    torque_manager_init();

    // 7) Engine
    ems::engine::fuel_reset_adaptives();
    ems::engine::auxiliaries_init();
    ems::engine::knock_init();
    ems::engine::quick_crank_reset();

    // 8) Aplicação
    ems::app::ui_init();
    ems::app::can_stack_init();

    // 9) NVIC — CKP fica com prioridade máxima. Injeção/ignição em TIM2/TIM8
    //    usam output compare direto por hardware, sem ISR no caminho crítico.
    //    SysTick configurado em system_stm32_init() com prio 11.
    nvic_set_priority(IRQ_TIM5, 1u);
    nvic_enable_irq(IRQ_TIM5);

    // 10) Aguardar CKP sync (timeout 5 s)
    const uint32_t sync_deadline = millis() + 5000u;
    while (millis() < sync_deadline) {
        iwdg_kick();
        const auto snap = ems::drv::ckp_snapshot();
        if (snap.state == ems::drv::SyncState::FULL_SYNC) { break; }
    }
}

// =============================================================================
// main() — substituição do setup()/loop() do STM32 runtime
// =============================================================================

int main() {
    openems_init();

    uint32_t g_t2ms_   = millis();
    uint32_t g_t10ms_  = g_t2ms_;
    uint32_t g_t20ms_  = g_t2ms_;
    uint32_t g_t50ms_  = g_t2ms_;
    uint32_t g_t100ms_ = g_t2ms_;
    uint32_t g_t500ms_ = g_t2ms_;
    uint32_t g_t_etb_ms = g_t2ms_;

    for (;;) {
        // ── Watchdog kick (primeiro statement) ───────────────────────────
        iwdg_kick();
        g_datalog_us = micros();

        const uint32_t now = millis();

        // ── 2ms: fuel + ign recalc + commit calibration ───────────────────
        if (elapsed(now, g_t2ms_, 2u)) {
            g_t2ms_ = now;
            const uint32_t loop2ms_start_us = micros();

            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            const bool full_sync = (snap.state == ems::drv::SyncState::FULL_SYNC);
            const bool sched_sync = (snap.state == ems::drv::SyncState::HALF_SYNC || full_sync);

            if (g_runtime_seed_arm_window_active) {
                if (elapsed(now, g_runtime_seed_arm_window_start_ms,
                            kRuntimeSeedArmWindowMs)) {
                    ems::drv::ckp_seed_disarm();
                    g_runtime_seed_arm_window_active = false;
                }
            }

            if (full_sync) {
                g_have_last_full_sync = true;
                g_last_full_sync_snapshot = snap;
            }
            if (sched_sync && snap.tooth_index == 0u) {
                g_have_last_gap_sync = true;
                g_last_gap_sync_snapshot = snap;
            }

            const uint16_t map_kpa_raw = static_cast<uint16_t>(sensors.map_kpa_x10 / 10u);
            const uint16_t map_kpa_sensor = clamp_u16(map_kpa_raw, kMapMinKpa, kMapMaxKpa);
            
            // Atualiza estimador de MAP com sensor fusion para resposta transiente rápida
            const uint16_t map_kpa = ems::engine::map_estimator_update(
                map_kpa_sensor,
                sensors.tps_pct_x10,
                kAePeriodMs,
                snap.rpm_x10);
            
            // Loop ETB (1kHz = 1ms) - controle de borboleta eletrônica
            if (g_etb_initialized && elapsed(now, g_etb_loop_timer_ms, 1u)) {
                g_etb_loop_timer_ms = now;
                
                // Preparar entradas do torque manager
                // APP (pedal do acelerador) na entrada analógica de expansão AN1
                // (raw 12-bit, 0–4095). Distinto de tps_pct_x10, que é a posição
                // da borboleta (feedback do ETB), não a demanda do motorista.
                g_torque_inputs.pedal_percent = (static_cast<float>(sensors.an1_raw) * 100.0f) / 4095.0f;
                g_torque_inputs.engine_rpm = snap.rpm_x10 / 10.0f;
                g_torque_inputs.coolant_temp = sensors.clt_degc_x10 / 10.0f;
                g_torque_inputs.intake_air_temp = sensors.iat_degc_x10 / 10.0f;
                g_torque_inputs.idle_mode = (snap.rpm_x10 < 1500u) && 
                                            (sensors.tps_pct_x10 < 50u);
                g_torque_inputs.limp_mode = g_limp_active;
                
                // Executar torque manager
                torque_manager_loop(&g_torque_inputs, &g_torque_outputs);
                
                // Executar controle ETB com target do torque manager
                etb_control_loop(g_torque_outputs.throttle_target, 
                                snap.rpm_x10 / 10.0f, 
                                1.0f);
            }
            
            const bool map_fault = (sensors.fault_bits & kFaultBitMap) != 0u;
            const bool clt_fault = (sensors.fault_bits & kFaultBitClt) != 0u;
            g_limp_active = map_fault || clt_fault;
            const bool rev_cut = g_limp_active &&
                (snap.rpm_x10 > kLimpRpmLimit_x10);
            const CachedFuelCorrections& fuel_corr = fuel_corrections_for(sensors);
            const bool crank_rpm_window =
                sched_sync && snap.rpm_x10 > 0u &&
                snap.rpm_x10 < ems::engine::crank_exit_rpm_x10;
            ems::engine::quick_crank_set_prime_context(sensors.clt_degc_x10,
                                                       fuel_corr.dead_time_us);

            if (sched_sync && !rev_cut) {
                const ems::engine::Table2dLookup fuel_lookup =
                    ems::engine::table3d_prepare_lookup(ems::engine::kRpmAxisX10,
                                                        ems::engine::kLoadAxisKpa,
                                                        snap.rpm_x10,
                                                        map_kpa);
                const uint8_t  ve = ems::engine::get_ve_prepared(fuel_lookup);
                const uint16_t lambda_target_x1000 =
                    ems::engine::get_lambda_target_x1000_prepared(fuel_lookup);
                const int16_t fuel_trim_pct_x10 = crank_rpm_window ? 0 : clamp_i16(
                    static_cast<int16_t>(ems::engine::fuel_get_stft_pct_x10() +
                                         ems::engine::fuel_get_ltft_pct_x10(fuel_lookup.yi, fuel_lookup.xi)),
                    -500, 500);
                const int32_t ae_pw_us = crank_rpm_window ? 0 : ems::engine::calc_ae_pw_us(
                    sensors.tps_pct_x10,
                    g_prev_tps_pct_x10,
                    kAePeriodMs,
                    sensors.clt_degc_x10);
                uint32_t final_pw_us_base =
                    ems::engine::calc_fuel_pw_us_default_fast(ve,
                                                               map_kpa,
                                                               lambda_target_x1000,
                                                               fuel_trim_pct_x10,
                                                               fuel_corr.corr_clt_x256,
                                                               fuel_corr.corr_iat_x256,
                                                               fuel_corr.dead_time_us);
                if (final_pw_us_base > fuel_corr.dead_time_us) {
                    const uint32_t fuel_pw_us =
                        final_pw_us_base - static_cast<uint32_t>(fuel_corr.dead_time_us);
                    const bool xtau_enabled = !crank_rpm_window && (snap.rpm_x10 >= 7000u);
                    
                    // Detecta transiente para auto-calibração X-τ
                    const bool is_transient = ems::engine::map_is_transient();
                    
                    // Auto-calibração X-τ baseada em erro de lambda.
                    // Lambda vem exclusivamente do WBO2 via CAN (mesma fonte do
                    // STFT em 100ms); SensorData não carrega mais o2/lambda.
                    const bool lambda_valid = ems::app::can_stack_wbo2_fresh(now);
                    if (full_sync && !crank_rpm_window && lambda_valid) {
                        const uint16_t lambda_measured = clamp_u16(
                            ems::app::can_stack_lambda_milli_safe(now),
                            kLambdaMinMilli, kLambdaMaxMilli);
                        ems::engine::xtau_autocalib_update(
                            snap.rpm_x10,
                            map_kpa,
                            lambda_target_x1000,
                            static_cast<int16_t>(lambda_measured),
                            sensors.clt_degc_x10,
                            is_transient);
                    }
                    
                    // Usa modelo X-τ com parâmetros aprendidos
                    const uint32_t xtau_fuel_pw_us =
                        ems::engine::transient_fuel_xtau_with_autocalib(fuel_pw_us,
                                                                        sensors.clt_degc_x10,
                                                                        xtau_enabled);
                    final_pw_us_base = xtau_fuel_pw_us + static_cast<uint32_t>(fuel_corr.dead_time_us);
                } else {
                    ems::engine::transient_fuel_reset();
                }
                if (ae_pw_us > 0) {
                    const uint32_t ae_add = static_cast<uint32_t>(ae_pw_us);
                    final_pw_us_base = (ae_add >= 100000u || final_pw_us_base > (100000u - ae_add))
                        ? 100000u
                        : (final_pw_us_base + ae_add);
                    g_ae_active = true;
                }
                const int16_t base_advance_deg = ems::engine::get_advance_prepared(fuel_lookup);
                const uint16_t knock_retard_x10 = ems::engine::knock_get_retard_x10(0u);
                const uint16_t idle_target_rpm_x10 =
                    ems::engine::auxiliaries_idle_target_rpm_x10(sensors.clt_degc_x10);
                const int16_t idle_spark_corr_deg = crank_rpm_window ? 0 :
                    ems::engine::calc_idle_spark_correction_deg(snap.rpm_x10,
                                                                idle_target_rpm_x10,
                                                                sensors.tps_pct_x10,
                                                                map_kpa);
                const int16_t advance_deg = ems::engine::calc_total_advance(
                    base_advance_deg,
                    {0, idle_spark_corr_deg, static_cast<int16_t>(knock_retard_x10 / 10u)});

                const auto qc = ems::engine::quick_crank_update(
                    now, snap.rpm_x10, sched_sync, sensors.clt_degc_x10, advance_deg);
                const uint32_t final_pw_us =
                    ems::engine::quick_crank_apply_pw_us(final_pw_us_base,
                                                         qc.fuel_mult_x256,
                                                         qc.min_pw_us);
                const uint32_t pw_100 = final_pw_us / 100u;
                g_last_pw_ms_x10 = static_cast<uint8_t>(pw_100 > 255u ? 255u : pw_100);
                g_last_advance_deg = clamp_i8(qc.spark_deg, -10, 40);

                const uint32_t inj_pw_ticks = ems::engine::inj_pw_us_to_scheduler_ticks(final_pw_us);
                const uint32_t dwell_ticks =
                    (static_cast<uint32_t>(fuel_corr.dwell_ms_x10) * kSchedulerTicksPerMs) / 10u;

                ::ecu_sched_commit_calibration(
                    static_cast<uint32_t>(qc.spark_deg < 0 ? 0 : qc.spark_deg),
                    dwell_ticks,
                    inj_pw_ticks,
                    ems::engine::cfg::kDefaultSoiLeadDeg);
            } else if (sched_sync && rev_cut) {
                const int16_t base_advance_deg = ems::engine::get_advance(snap.rpm_x10, map_kpa);
                const uint32_t dwell_ticks =
                    (static_cast<uint32_t>(fuel_corr.dwell_ms_x10) * kSchedulerTicksPerMs) / 10u;
                ::ecu_sched_commit_calibration(
                    static_cast<uint32_t>(base_advance_deg < 0 ? 0 : base_advance_deg),
                    dwell_ticks,
                    0u,
                    ems::engine::cfg::kDefaultSoiLeadDeg);
                g_last_pw_ms_x10 = 0u;
                g_ae_active = false;
            } else if (snap.rpm_x10 == 0u) {
                static_cast<void>(ems::engine::quick_crank_update(
                    now, snap.rpm_x10, false, sensors.clt_degc_x10, 0));
            }
            g_prev_tps_pct_x10 = sensors.tps_pct_x10;

            const uint32_t prime_pw = ems::engine::quick_crank_consume_prime();
            if (prime_pw != 0u) {
                ::ecu_sched_fire_prime_pulse(prime_pw);
            }

            g_loop2ms_last_us = micros() - loop2ms_start_us;
            if (g_loop2ms_last_us > g_loop2ms_max_us) {
                g_loop2ms_max_us = g_loop2ms_last_us;
            }
            ems::app::ui_update_loop_diag(g_loop2ms_last_us, g_loop2ms_max_us);
        }

        // ── 10ms: IACV, VVT, wastegate PID ───────────────────────────────
        if (elapsed(now, g_t10ms_, 10u)) {
            g_t10ms_ = now;
            ems::engine::auxiliaries_tick_10ms();
        }

        // ── 20ms: UI proprietaria + aux tasks ───────────────────────────
        if (elapsed(now, g_t20ms_, 20u)) {
            g_t20ms_ = now;
            const auto snap = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            ems::app::ui_update_rt_metrics(g_last_pw_ms_x10, g_last_advance_deg, g_last_stft_pct);
            ems::app::ui_update_rt_sched_diag(
                g_late_event_count,
                g_cycle_schedule_drop_count,
                g_calibration_clamp_count,
                ems::drv::ckp_seed_loaded_count(),
                ems::drv::ckp_seed_confirmed_count(),
                ems::drv::ckp_seed_rejected_count(),
                static_cast<uint8_t>(snap.state));
            ems::app::ui_update_ivc_diag(::ecu_sched_ivc_clamp_count());
            ui_service();
            ems::engine::auxiliaries_tick_20ms();
            ems::app::can_stack_process(now, snap, sensors,
                                        g_last_advance_deg,
                                        g_last_pw_ms_x10,
                                        g_last_stft_pct,
                                        0u, 0u,
                                        build_status_bits(snap, sensors));
        }

        // ── 50ms: sensores lentos ──────────────────────────────────────────
        if (elapsed(now, g_t50ms_, 50u)) {
            g_t50ms_ = now;
            ems::drv::sensors_tick_50ms();
        }

        // ── 100ms: sensores + STFT ───────────────────────────────────────
        if (elapsed(now, g_t100ms_, 100u)) {
            g_t100ms_ = now;
            ems::drv::sensors_tick_100ms();
            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();

            if (snap.state == ems::drv::SyncState::FULL_SYNC) {
                const uint16_t map_kpa = clamp_u16(
                    static_cast<uint16_t>(sensors.map_kpa_x10 / 10u), kMapMinKpa, kMapMaxKpa);
                const uint16_t lambda_measured = clamp_u16(
                    ems::app::can_stack_lambda_milli_safe(now), kLambdaMinMilli, kLambdaMaxMilli);
                const bool lambda_valid = ems::app::can_stack_wbo2_fresh(now);
                const uint16_t lambda_target_x1000 =
                    ems::engine::get_lambda_target_x1000(snap.rpm_x10, map_kpa);
                const bool rev_cut = g_limp_active &&
                    (snap.rpm_x10 > kLimpRpmLimit_x10);
                const bool ae_active = g_ae_active;
                const int16_t stft = ems::engine::fuel_update_stft_delayed(
                    now, snap.rpm_x10, map_kpa,
                    static_cast<int16_t>(lambda_target_x1000),
                    static_cast<int16_t>(lambda_measured),
                    sensors.clt_degc_x10, lambda_valid,
                    ae_active, rev_cut);
                g_ae_active = false;
                g_last_stft_pct = clamp_i8(static_cast<int16_t>(stft / 10), -25, 25);

                // X-tau autocalibration (100ms lambda-gated)
                const bool xtau_learning_ok = (stft >= -500 && stft <= 500 &&
                    lambda_valid && snap.rpm_x10 >= 2000u);
                ems::engine::xtau_autocal_tick(
                    now, stft, sensors.clt_degc_x10, xtau_learning_ok);
                if (ems::engine::xtau_autocal_is_dirty()) {
                    ems::engine::xtau_autocal_apply_learned_tables();
                    ems::engine::xtau_autocal_clear_dirty();
                }
            } else {
                g_last_stft_pct = 0;
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
                    g_have_last_gap_sync) {
                    const auto seed_snap = g_last_gap_sync_snapshot;
                    ems::hal::RuntimeSyncSeed seed = {};
                    seed.flags = static_cast<uint8_t>(
                        ems::hal::RUNTIME_SYNC_SEED_FLAG_VALID |
                        ems::hal::RUNTIME_SYNC_SEED_FLAG_FULL_SYNC |
                        (seed_snap.phase_A
                             ? ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A : 0u));
                    seed.tooth_index = seed_snap.tooth_index;
                    seed.decoder_tag =
                        ems::hal::RUNTIME_SYNC_SEED_DECODER_TAG_60_2;
	if (!ems::hal::nvm_save_runtime_seed(&seed)) {
		// FIX: não descartar retorno — falha de flash deve ser rastreada
		++g_flash_write_faults; // fault counter para diagnóstico
	}
	g_runtime_seed_saved_for_stop = true;
                }
            }
        }

        // ── 500ms: agenda flush Flash; execução avança em passos curtos ───
        // FIX P0: Only allow flash writes when engine is stopped or below safe RPM
        static bool adaptive_flush_pending = false;
        static uint32_t last_calib_save_ms = 0u;
        if (elapsed(now, g_t500ms_, 500u)) {
            g_t500ms_ = now;
            const auto snap = ems::drv::ckp_snapshot();
            if (g_calib_dirty &&
                (last_calib_save_ms == 0u ||
                 elapsed(now, last_calib_save_ms, kCalibSaveMinIntervalMs))) {
                ems::engine::cfg::engine_config_serialize(g_calib_page0, kCalibPageBytes);
                if (ems::hal::nvm_save_calibration(0u, g_calib_page0,
                                                   kCalibPageBytes)) {
                    g_calib_dirty = false;
                    last_calib_save_ms = now;
                }
            }
            
            // FIX P0: Only schedule adaptive map flush when engine is below safe RPM
            // Flash writes during engine operation can cause timing jitter and misfires
            const bool engine_running_fast = (snap.rpm_x10 > ems::engine::kFlashWriteSafeRpmX10);
            if (!engine_running_fast) {
                adaptive_flush_pending = true;
            }
            // If engine is running fast, keep adaptive_flush_pending false to defer flash write
        }
        if (adaptive_flush_pending) {
            // Double-check RPM before actually writing (engine may have started)
            const auto snap = ems::drv::ckp_snapshot();
            const bool engine_running_fast = (snap.rpm_x10 > ems::engine::kFlashWriteSafeRpmX10);
            if (!engine_running_fast) {
                adaptive_flush_pending = !ems::hal::nvm_flush_adaptive_maps();
            }
        }

        // ── ETB control (2ms cadence) ─────────────────────────────────────
        if (elapsed(now, g_t_etb_ms, 2u)) {
            g_t_etb_ms = now;
            const auto sensors_etb = ems::drv::sensors_get();
            const auto snap_etb = ems::drv::ckp_snapshot();
            const bool etb_rev_cut = g_limp_active && (snap_etb.rpm_x10 > kLimpRpmLimit_x10);
            const auto torque_out = ems::engine::torque_manager_update(
                snap_etb, sensors_etb, true, g_limp_active, etb_rev_cut,
                ems::engine::auxiliaries_idle_target_rpm_x10(sensors_etb.clt_degc_x10), 2u);
            (void)ems::engine::etb_control_update(
                torque_out.etb_target_pct_x10, sensors_etb.etb_tps_pct_x10,
                torque_out.etb_enable_request, 2u);
        }
    }
}

#endif  // TARGET_STM32H562
