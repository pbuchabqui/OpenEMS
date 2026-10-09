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
//   - NVIC setup usa IRQs do STM32H562 (TIM5 para CKP; TIM2/TIM1 são OC por hardware)
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
#include "app/can_rx_map.h"
#include "app/nvm_boot.h"
#include "app/ui_protocol.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/calibration.h"
#include "engine/constants.h"
#include "engine/cut_reason.h"
#include "engine/map_window.h"
#include "engine/vehicle_inputs.h"
#include "engine/ecu_sched.h"
#include "engine/engine_calc.h"
#include "engine/engine_config.h"
#include "engine/etb_control.h"
#include "engine/etb_autocal.h"
#include "hal/etb_driver.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/limp_gating.h"
#include "engine/map_estimator.h"
#include "engine/output_test.h"
#include "engine/diagnostic_manager.h"
#include "engine/misfire_detect.h"
#include "engine/quick_crank.h"
#include "engine/torque_manager.h"
#include "engine/transient_fuel.h"
#include "engine/spark_skip.h"
#include "engine/xtau_autocalib.h"
#include "engine/ewg_control.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/flash.h"
#include "hal/out_pins.h"
#include "hal/flex_fuel.h"
#include "hal/timer.h"

// =============================================================================
// Estado de background (do firmware)
// =============================================================================

static constexpr uint16_t kCalibPageBytes = 512u;
alignas(4) static uint8_t g_calib_page0[kCalibPageBytes];
static bool                g_calib_dirty  = false;

volatile uint32_t g_flash_write_faults = 0u; // FIX: fault counter para falhas de escrita NVM


static int8_t  g_last_advance_deg = 0;
static int16_t g_last_advance_x10 = 0;
// Spark retard from torque manager (TC/launch), updated in 2 ms ETB slot.
static int16_t g_torque_spark_retard_deg = 0;
static uint8_t g_last_pw_ms_x10   = 0u;
static int8_t  g_last_stft_pct    = 0;
static uint8_t g_last_lambda_target_d4 = 0u;
// MAP fundido do último tick de 2ms — fonte única p/ os consumidores de 100ms
// (STFT/alvo λ): usar o sensor cru ali dava alvo≠combustível na fronteira de célula.
static uint16_t g_last_map_fused_x100 = 0u;
static int8_t  g_last_ltft_pct    = 0;

static constexpr uint32_t kLimpRpmLimit_x10 = 30000u;
static bool g_limp_active = false;
// DIAG rev-limit: conta bordas de subida (false→true) do rev-limiter e latcha o
// rpm_x10 que o disparou. Suspeita: glitch fantasma de rpm_x10 (corrida ISR↔main)
// dispara o limiter ao ralenti → spikes de PW=0 de 1 loop. Expostos no dump 'D'.
uint32_t g_dbg_rev_limit_trips = 0u;
uint32_t g_dbg_rev_limit_rpm_x10 = 0u;   // último rpm_x10 que armou o trip
uint32_t g_dbg_rev_limit_rpm_max = 0u;   // maior rpm_x10 alguma vez visto (glitch?)

static uint32_t g_last_net_pw_us   = 0u;
// Barometric correction: amostrar MAP quando motor parado por >300ms após key-on
static uint32_t g_baro_stopped_since_ms = 0u;
static bool     g_baro_sampled          = false;
static uint32_t g_loop2ms_last_us = 0u;
static uint32_t g_loop2ms_max_us = 0u;

static constexpr uint32_t kCalibSaveMinIntervalMs = 300000u;
static constexpr uint16_t kMapMinBarX100 = 10u;
static constexpr uint16_t kMapMaxBarX100 = 300u;
static constexpr uint16_t kLambdaMinMilli = 700u;
static constexpr uint16_t kLambdaMaxMilli = 1400u;

// =============================================================================
// Estado ETB e Torque Manager
// =============================================================================

static bool g_etb_initialized = false;

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
    if (g_rev_limit_active) {
        status |= ems::app::STATUS_REV_LIMIT;
    }
    if (ems::engine::torque_manager_get_launch_active() != 0u) {
        status |= ems::app::STATUS_LAUNCH_ACTIVE;
    }
    if (ems::engine::torque_manager_get_tc_reduction() > 0u) {
        status |= ems::app::STATUS_TC_ACTIVE;
    }
    return status;
}

using ems::engine::clamp_u16;

static inline int8_t clamp_i8(int16_t v, int8_t lo, int8_t hi) noexcept {
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return static_cast<int8_t>(v);
}

// NVM map/corr loaders: app/nvm_boot.cpp (relocate-only hygiene PR-09).

static inline void comms_pump() noexcept {
    // Shuttle UART+USB↔protocolo a 2 ms. Ambos os transportes são servidos no
    // MESMO ritmo curto: o handshake de conexão do TunerStudio envia comandos
    // PLAIN (F, Q, ...) e, se a resposta demora, ele estoura o timeout curto e
    // envia o próximo comando antes de ler a resposta anterior — as duas
    // respostas saem então coladas ("001OpenEMS_v1.2") e o TS rejeita com
    // "Unsupported Controller Firmware". Servir o USB a 20 ms (como antes)
    // garantia esse gluing; a 2 ms cada comando é respondido a tempo, como nas
    // implementações de referência (Speeduino/rusEFI: request→resposta→flush
    // sub-ms). Nada aqui bloqueia (drena TX só com espaço no FIFO/rings).
    ems::hal::uart0_poll_rx(32u);
    {
        uint8_t b = 0u;
        while (ems::hal::uart0_rx_pop(b)) {
            ems::app::ui_rx_byte(b);
        }
    }
    ems::hal::usb_cdc_poll();
    if (ems::hal::usb_cdc_dtr()) {
        uint8_t rx_buf[64] = {};
        const uint16_t rx_n = ems::hal::usb_cdc_read_bytes(rx_buf, 64u);
        for (uint16_t i = 0u; i < rx_n; ++i) {
            ems::app::ui_rx_byte(rx_buf[i]);
        }
    }
    ems::app::ui_process();

    // Drena ui_tx → UART + USB (espelhado), limitado pelo MENOR espaço livre
    // entre os dois rings de destino — usb_cdc_send_bytes() descarta bytes
    // em silêncio se o seu próprio ring encher, então o orçamento tem de
    // respeitar ambos, não só o do UART, para nunca perder bytes de um frame.
    uint16_t budget = ems::hal::uart0_tx_free();
    const uint16_t usb_free = ems::hal::usb_cdc_tx_free();
    if (usb_free < budget) { budget = usb_free; }
    if (budget > 32u) { budget = 32u; }
    uint8_t tx_buf[32] = {};
    uint16_t tx_n = 0u;
    while (tx_n < budget && ems::app::ui_tx_pop(tx_buf[tx_n])) { ++tx_n; }
    if (tx_n != 0u) {
        for (uint16_t i = 0u; i < tx_n; ++i) {
            ems::hal::uart0_tx_push(tx_buf[i]);
        }
        ems::hal::usb_cdc_send_bytes(tx_buf, tx_n);
    }
    ems::hal::uart0_tx_poll_nb(16u);
}

// =============================================================================
// Inicialização — sequência idêntica ao main.cpp
// =============================================================================

static void openems_init() noexcept {
    // 1) PLL → 250 MHz + SysTick 1ms + IWDG 100ms
    system_stm32_init();

    // 1a) CRITICAL: INJ/IGN LOW before any delay.
    // PA15 resets as JTDI with pull-up → HIGH → injectors on (active-high).
    // PC10/11 float; external pull-ups can also pull INJ high.
    // Must run before the USB 300 ms wait (was: ECU_Hardware_Init after that).
    ::ecu_sched_outputs_safe_early();
    ::ecu_sched_set_inj_inhibit_mask(0x0Fu);
    ::ecu_sched_set_inj_pw_ticks(0u);

    // 1b) Reabilitar IRQs globais EXPLICITAMENTE. O Reset_Handler faz cpsid i e nunca
    // reabilita; antes isto só acontecia por efeito colateral do 1º cpsie de uma seção
    // crítica adiante, o que deixava a ISR do USB (e outras) mascaradas se a ordem mudasse.
    // Com SysTick já configurado em system_stm32_init(), é seguro habilitar aqui.
    __asm__ volatile("cpsie i" ::: "memory");

    // 1c) PB2 (LED WeAct) como saída — heartbeat visível desde o boot.
    GPIOB_MODER = (GPIOB_MODER & ~(3u << 4u)) | (1u << 4u);

    // 1d) USB CDC CEDO: só depende de clock (HSI48/CRS já prontos) + IRQs. Subir aqui,
    // antes dos inits da ECU (ADC/CAN/CKP), garante enumeração mesmo que algum init
    // adiante demore/bloqueie numa placa de bancada sem motor — a ISR cuida do resto.
    ems::hal::usb_cdc_init();

    // 1e) Janela p/ a enumeração USB (ISR-driven) completar antes dos inits da ECU, que
    // podem entrar em seção crítica (cpsid i) e mascarar a ISR do USB por um tempo.
    // ~300 ms kicando o IWDG (timeout 100 ms) a cada ~1 ms @ 250 MHz.
    // Re-assert INJ/IGN LOW each 100 ms in case USB/other init touches GPIOA.
    for (uint32_t ms = 0u; ms < 300u; ++ms) {
        for (volatile uint32_t d = 0u; d < 60000u; ++d) { /* ~1ms */ }
        iwdg_kick();
        ems::hal::usb_cdc_poll();
        if ((ms % 100u) == 0u) {
            GPIOB_ODR ^= (1u << 2u);
            ::ecu_sched_outputs_safe_early();
        }
    }

    // 2) Timers (TIM5=CKP IC)
    // misfire_init() DEVE preceder tim5_ic_init(): a tabela g_tooth_to_cyl parte de
    // BSS (zero), mas 0 é um índice de cilindro válido — o ISR do CKP leria cyl=0
    // para todos os dentes antes da tabela ser preenchida, gerando DTCs falsos.
    ems::engine::misfire_init();
    ems::hal::tim5_ic_init();   // → TIM5 input capture (CKP + CMP)
    iwdg_kick();

    // 2a) Scheduler unificado (re-asserts pin safe + clears event queue)
    ::ECU_Hardware_Init();
    ::ecu_sched_set_presync_inj_auto(1u);  // auto-select SIMULTANEOUS/SEMI_SEQUENTIAL by cranking
    ::ecu_sched_set_inj_inhibit_mask(0x0Fu);
    ::ecu_sched_set_inj_pw_ticks(0u);
    iwdg_kick();

    // 3) ADC (ADC1/ADC2 + TIM6 trigger)
    ems::hal::adc_init();
    iwdg_kick();

    // 4) CAN + bench communication. MVP transport: USART1 PA9/PA10.
    // (usb_cdc_init() já foi chamado cedo em 1b, antes dos inits da ECU.)
    ems::hal::can0_init();
    ems::hal::uart0_init(115200u);
    ems::hal::uart0_enable_rx();  // RX fica desligado por padrão (uart.cpp:61)
    ems::engine::ewg_control_init();
    ems::hal::flex_fuel_init();
    iwdg_kick();

	// 5) Flash Bank2 → carrega calibrações persistidas
	if (!ems::hal::nvm_load_calibration(0u, g_calib_page0, kCalibPageBytes)) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
	// Page 0 → globals through the protocol's own apply (one path for boot
	// and for a TunerStudio/dashboard write; timing light forced off).
	ems::app::ui_boot_apply_page0(g_calib_page0, kCalibPageBytes);
	// Gate de layout: páginas de tabela só carregam se a versão gravada no
	// page0 (byte 175) bater com o firmware — um blob de dimensão antiga
	// lido com o tamanho novo ganharia cauda 0xFF (VE=255!). Sem versão →
	// defaults de compilação; um "burn all" no dashboard re-persiste tudo.
	const bool cal_layout_ok =
		g_calib_page0[ems::engine::kCalLayoutVersionOffset] ==
		ems::engine::kCalLayoutVersion;
	ems::app::nvm_boot_load_tables(cal_layout_ok);
	if (!ems::hal::nvm_load_adaptive_maps()) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
    // 6) Drivers
    ems::drv::sensors_init();
    iwdg_kick();

    // 6a) Inicializa sistemas "invisíveis" ao motorista
    ems::engine::DiagnosticManager::init();
    ems::engine::map_estimator_init();
    ems::engine::xtau_autocalib_init();

    // 6b) Inicializa ETB e Torque Manager (borboleta eletrônica)
    g_etb_initialized = etb_control_init();
    // Auto-cal dos limites TPS a cada power-on (varre batentes no tick 2ms;
    // pulada se harness ausente; falha mantém a calibração de flash).
    if (g_etb_initialized) {
        ems::engine::etb_autocal_start();
    }
    torque_manager_init();
    iwdg_kick();

    // 7) Engine
    ems::engine::fuel_reset_adaptives();
    ems::engine::auxiliaries_init();
    ems::engine::knock_init();
    ems::engine::quick_crank_reset();
    iwdg_kick();

    // 8) Aplicação
    ems::app::ui_init();
    ems::app::can_stack_init(ems::engine::wbo2_can_id);

    // 9) NVIC — CKP fica com prioridade máxima. Injeção/ignição em TIM2/TIM1
    //    usam output compare direto por hardware, sem ISR no caminho crítico.
    //    SysTick configurado em system_stm32_init() com prio 11.
    nvic_set_priority(IRQ_TIM5, 1u);
    nvic_enable_irq(IRQ_TIM5);

    // 10) Aguardar CKP sync (timeout 5 s), heartbeat 5 Hz
    {
        const uint32_t sync_deadline = millis() + 5000u;
        uint32_t next_toggle = millis() + 100u;
        while (millis() < sync_deadline) {
            iwdg_kick();
            if (millis() >= next_toggle) {
                GPIOB_ODR ^= (1u << 2u);
                next_toggle = millis() + 100u;
            }
            const auto snap = ems::drv::ckp_snapshot();
            if (snap.state == ems::drv::SyncState::FULL_SYNC) { break; }
        }
    }

    // Kick final antes de entrar no main loop — garante que openems_init()
    // não ultrapassa o timeout de 100 ms do IWDG.
    iwdg_kick();
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
    uint32_t g_t_comms_ms = g_t2ms_;

    // Estreitar IWDG de 10s (boot) para 100ms (runtime): o main loop kica a cada
    // ciclo; 100ms detecta travamento de runtime sem tolerar os inits longos do boot.
    // Nota: IWDG_PR segue /256 (boot) → RLR=99 dá ~0.8s efetivo; suficiente p/ runtime
    // e evita esperar PVU/RVU de novo no caminho crítico.
    IWDG_KR  = IWDG_KR_ACCESS;
    IWDG_RLR = IWDG_RLR_100MS;
    IWDG_KR  = IWDG_KR_REFRESH;

    for (;;) {
        // ── Watchdog kick (primeiro statement) ───────────────────────────
        iwdg_kick();

        const uint32_t now = millis();

        // ── 2ms: fuel + ign recalc + commit calibration ───────────────────
        if (elapsed(now, g_t2ms_, 2u)) {
            g_t2ms_ = now;
            const uint32_t loop2ms_start_us = micros();

            // Stall watchdog: detecta virabrequim parado entre dentes.
            // Deve preceder ckp_snapshot() para que o snapshot deste ciclo
            // já reflicta LOSS_OF_SYNC se o motor parou.
            // Reactivado: os falsos stalls vinham do wrap 16-bit do TIM3;
            // desde a migração para TIM5 (32-bit) o elapsed é correcto.
            // Também decai rpm_x10 fantasma de ruído em CKP sem sync.
            ems::drv::ckp_stall_poll(ems::hal::tim5_count());

            // Dwell / injector open watchdogs (lost SPARK / lost INJ_OFF).
            ecu_sched_dwell_watchdog();
            ecu_sched_inj_watchdog();

            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();

            // Teste de saídas em bancada: aborto imediato se RPM > 0 e
            // timeout de keepalive (o dwell watchdog acima continua activo).
            ems::engine::output_test_poll(now, snap.rpm_x10);
            ems::engine::EngineCalcIn calc_in{};
            calc_in.now_ms = now;
            calc_in.snap = snap;
            calc_in.sensors = sensors;
            calc_in.lambda_x1000 = ems::app::can_stack_lambda_milli();
            calc_in.lambda_valid = ems::app::can_stack_wbo2_fresh(now);
            calc_in.torque_spark_retard_deg = g_torque_spark_retard_deg;
            const ems::engine::EngineCalcOut& calc = ems::engine::engine_calc_step(calc_in);
            ems::app::ui_set_rev_limit_active(g_rev_limit_active);
            ems::app::ui_update_rt_map_fuel(calc.map_fused_x100, calc.net_pw_us);
            g_last_map_fused_x100 = calc.map_fused_x100;
            g_last_net_pw_us = calc.net_pw_us;
            g_last_pw_ms_x10 = calc.pw_ms_x10;
            g_limp_active = calc.limp_active;
            g_dbg_rev_limit_rpm_max = calc.rpm_max_x10;
            if (calc.committed) {
                // Telemetry keeps whole degrees (rounded); the scheduler got 0.1°.
                const int16_t x10 = calc.spark_x10;
                g_last_advance_x10 = x10;
                g_last_advance_deg = clamp_i8(
                    static_cast<int16_t>((x10 + (x10 >= 0 ? 5 : -5)) / 10), -20, 60);
            }

            // EWG position inner loop (2ms cadence)
            if (!ems::engine::output_test_active()) {
                const uint16_t demand = ems::engine::auxiliaries_ewg_position_demand_x10();
                const uint16_t pos = ems::engine::ewg_read_position_pct_x10();
                ems::engine::ewg_control_update(demand, pos);
            }

            g_loop2ms_last_us = micros() - loop2ms_start_us;
            if (g_loop2ms_last_us > g_loop2ms_max_us) {
                g_loop2ms_max_us = g_loop2ms_last_us;
            }
            ems::app::ui_update_loop_diag(g_loop2ms_last_us, g_loop2ms_max_us);
        }

        // ── 10ms: VVT, wastegate PID ─────────────────────────────────────
        if (elapsed(now, g_t10ms_, 10u)) {
            g_t10ms_ = now;
            ems::engine::auxiliaries_tick_10ms();
        }

        // ── 20ms: UI proprietaria + aux tasks ───────────────────────────
        if (elapsed(now, g_t20ms_, 20u)) {
            g_t20ms_ = now;
            const auto snap = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            ems::app::ui_update_rt_metrics(g_last_pw_ms_x10, g_last_advance_deg, g_last_stft_pct,
                                           g_last_lambda_target_d4, g_last_ltft_pct);
            ems::app::ui_update_rt_sched_diag(
                g_late_event_count,
                g_cycle_schedule_drop_count,
                g_calibration_clamp_count,
                g_last_advance_x10,
                static_cast<uint8_t>(snap.state));
            // Transporte (UART+USB RX/TX/parse) vive em comms_pump() a 2 ms.
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

        // ── 100ms: sensores + STFT + misfire report + baro ──────────────
        if (elapsed(now, g_t100ms_, 100u)) {
            g_t100ms_ = now;
            ems::drv::sensors_tick_100ms();

            // Knock sensor morto (FOME #578): report único na transição.
            {
                static bool s_knock_dead_reported = false;
                const bool dead = ems::engine::knock_sensor_dead();
                if (dead && !s_knock_dead_reported) {
                    ems::engine::DiagnosticManager::report_fault(
                        ems::engine::DiagnosticCode::KNOCK_SENSOR_FAULT,
                        ems::engine::FaultSeverity::WARNING);
                    s_knock_dead_reported = true;
                } else if (!dead) {
                    s_knock_dead_reported = false;
                }
            }

            // Flex fuel: stoich AFR from ethanol % (E0 = 14.7, E100 = 9.0,
            // linear) as a runtime override — the configured stoich (page 0)
            // is never overwritten, so it cannot be burned to flash.
            ems::engine::fuel_set_stoich_override_x100(
                ems::hal::flex_fuel_valid()
                    ? static_cast<uint16_t>(1470u - (ems::hal::flex_fuel_ethanol_pct() * 570u) / 100u)
                    : 0u);
            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();

            // Compensação barométrica: amostrar MAP enquanto motor parado.
            // Aguarda 300ms estabilizado antes de aceitar a leitura (ADC settle).
            if (snap.rpm_x10 == 0u) {
                if (g_baro_stopped_since_ms == 0u) {
                    g_baro_stopped_since_ms = now;
                    g_baro_sampled = false;
                } else if (!g_baro_sampled &&
                           (now - g_baro_stopped_since_ms) >= 300u) {
                    const uint16_t map_baro = clamp_u16(
                        static_cast<uint16_t>(sensors.map_bar_x1000 / 10u),
                        70u, 110u);
                    ems::engine::fuel_set_baro_bar_x100(map_baro);
                    g_baro_sampled = true;
                }
            } else {
                g_baro_stopped_since_ms = 0u;
            }

            // Misfire: reporte de DTCs acumulados no período de 100ms
            if (snap.state == ems::drv::SyncState::FULL_SYNC) {
                constexpr ems::engine::DiagnosticCode kMisfireCodes[4] = {
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_1,
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_2,
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_3,
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_4,
                };
                for (uint8_t c = 0u; c < 4u; ++c) {
                    if (ems::engine::misfire_get_event_count(c) >=
                        ems::engine::kMisfireFaultThreshold) {
                        ems::engine::DiagnosticManager::report_fault(
                            kMisfireCodes[c],
                            ems::engine::FaultSeverity::WARNING);
                        ems::engine::misfire_clear_events(c);
                    }
                }
            }

            if (snap.state == ems::drv::SyncState::FULL_SYNC) {
                // MAP fundido (mesma fonte do cálculo de combustível de 2ms):
                // o cru daqui divergia na fronteira de célula → alvo λ do
                // gauge oscilava sem o ponto de operação mudar.
                const uint16_t map_bar_x100 = clamp_u16(
                    g_last_map_fused_x100, kMapMinBarX100, kMapMaxBarX100);
                const uint16_t lambda_measured = clamp_u16(
                    ems::app::can_stack_lambda_milli_safe(now), kLambdaMinMilli, kLambdaMaxMilli);
                const bool lambda_valid = ems::app::can_stack_wbo2_fresh(now);
                const uint16_t lambda_target_x1000 =
                    ems::engine::get_lambda_target_x1000(snap.rpm_x10, map_bar_x100);
                const bool rev_cut = g_limp_active &&
                    (snap.rpm_x10 > kLimpRpmLimit_x10);
                const bool ae_active = ems::engine::fuel_ae_stft_freeze_active();
                // STFT congelado em qualquer condição de corte intencional de combustível:
                // - rev_cut: limp mode
                // - decel_cut: borboleta fechada em desaceleração
                // - inj_inhibit_mask != 0: rev limiter cortou injeção em ≥1 cilindro
                //   (lambda leria lean sem combustível → STFT aprenderia errado)
                const bool stft_inhibit = rev_cut ||
                    ems::engine::fuel_decel_cut_active() ||
                    (::ecu_sched_get_inj_inhibit_mask() != 0u);
                // APP (pedido do condutor) para estabilidade do acumulador LTFT:
                // ETB mexe sozinho em idle e rejeitaria hits sem o condutor mexer.
                // Célula continua a ser (MAP, RPM); APP só filtra regime.
                const int16_t stft = ems::engine::fuel_update_stft_delayed(
                    now, snap.rpm_x10, map_bar_x100,
                    static_cast<int16_t>(lambda_target_x1000),
                    static_cast<int16_t>(lambda_measured),
                    sensors.clt_degc_x10, lambda_valid,
                    ae_active, stft_inhibit, g_last_net_pw_us,
                    sensors.app_pct_x10);
                ems::engine::fuel_ae_stft_freeze_clear();
                g_last_stft_pct = clamp_i8(static_cast<int16_t>(stft / 10), -25, 25);
                // ÷5 (não ÷4): ÷4 saturava o u8 em 1020 — alvos 1.02-1.27 exibiam 1.02
                g_last_lambda_target_d4 = ems::engine::clamp_u8(lambda_target_x1000 / 5u);
                g_last_ltft_pct = clamp_i8(
                    ems::engine::fuel_get_ltft_at(snap.rpm_x10, map_bar_x100) / 10,
                    -25, 25);

                // X-τ learn (100ms): exige transiente real (tpsdot) E gates de
                // qualidade λ/STFT/RPM. Nunca tratar "learning_ok" sozinho como
                // is_transient — isso corrompia a tabela 2D em regime estável.
                const bool xtau_learning_ok = (stft >= -500 && stft <= 500 &&
                    lambda_valid && snap.rpm_x10 >= 20000u);
                const bool xtau_transient = ems::engine::map_is_transient() &&
                    xtau_learning_ok && !ae_active;
                ems::engine::xtau_autocalib_update(
                    snap.rpm_x10,
                    map_bar_x100,  // MAP fundido (mesma fonte do PW 2ms)
                    static_cast<int16_t>(lambda_target_x1000),
                    static_cast<int16_t>(ems::app::can_stack_lambda_milli_safe(now)),
                    sensors.clt_degc_x10,
                    xtau_transient);
            } else {
                g_last_stft_pct = 0;
            }

            const uint32_t rpm = snap.rpm_x10;
            // Borda rodando→parado: força flush do NVM adaptativo (LTFT/knock/
            // etbcal) fora do rate-limit de 60s do flush periódico — evita
            // perder até 60s de aprendizado se a energia cair logo após o
            // key-off numa corrida curta.
            static bool s_prev_rpm_nonzero = false;
            if (rpm > 0u) {
                s_prev_rpm_nonzero = true;
            } else if (s_prev_rpm_nonzero) {
                s_prev_rpm_nonzero = false;
                ems::hal::nvm_request_adaptive_flush_now();
            }
        }

        // ── 500ms: agenda flush Flash + LED heartbeat (PB2 WeAct blue LED) ─
        // PB2 = LED blue on-board da WeAct STM32H562 LQFP100.
        // FIX P0: Only allow flash writes when engine is stopped or below safe RPM
        static bool adaptive_flush_pending = false;
        static uint32_t last_calib_save_ms = 0u;
        ems::hal::nvm_set_now_ms(now);
        if (elapsed(now, g_t500ms_, 500u)) {
            g_t500ms_ = now;
            // LED heartbeat: toggle PB2 a cada 500ms (1 Hz)
            GPIOB_MODER = (GPIOB_MODER & ~(3u << 4u)) | (1u << 4u);
            GPIOB_ODR ^= (1u << 2u);  // toggle PB2
            const auto snap = ems::drv::ckp_snapshot();
            // FIX: gate ALL flash writes behind the same RPM threshold — calibration
            // writes had no RPM check, creating a latent bug (see Blocker #3).
            const bool engine_running_fast = (snap.rpm_x10 > ems::engine::kFlashWriteSafeRpmX10);
            if (!engine_running_fast) {
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
                // Adaptive LTFT/knock: só agenda se dirty (rate-limit dentro do flush).
                if (ems::hal::nvm_adaptive_maps_dirty()) {
                    adaptive_flush_pending = true;
                }
            }
        }
        if (adaptive_flush_pending) {
            // Double-check RPM before actually writing (engine may have started)
            const auto snap = ems::drv::ckp_snapshot();
            const bool engine_running_fast = (snap.rpm_x10 > ems::engine::kFlashWriteSafeRpmX10);
            if (!engine_running_fast) {
                adaptive_flush_pending = !ems::hal::nvm_flush_adaptive_maps();
            }
        }

        // ── Comms pump UART (2ms cadence, fora da medição de loop2) ──────
        if (elapsed(now, g_t_comms_ms, 2u)) {
            g_t_comms_ms = now;
            comms_pump();
        }

        // ── ETB control (2ms cadence) ─────────────────────────────────────
        if (elapsed(now, g_t_etb_ms, 2u) && !ems::engine::output_test_active()) {
            g_t_etb_ms = now;
            const auto sensors_etb = ems::drv::sensors_get();
            const auto snap_etb = ems::drv::ckp_snapshot();
            // Auto-cal power-on em curso: varre batentes e pula torque/PID.
            if (ems::engine::etb_autocal_active()) {
                ems::engine::etb_autocal_tick(2u, snap_etb.rpm_x10);
            } else {
                const bool etb_rev_cut = g_limp_active && (snap_etb.rpm_x10 > kLimpRpmLimit_x10);
                const auto torque_out = ems::engine::torque_manager_update(
                    snap_etb, sensors_etb, true, g_limp_active, etb_rev_cut,
                    ems::engine::auxiliaries_idle_target_rpm_x10(sensors_etb.clt_degc_x10), 2u);
                g_torque_spark_retard_deg = torque_out.spark_retard_deg;
                const auto etb = ems::engine::etb_control_update(
                    torque_out.etb_target_pct_x10, sensors_etb.etb_tps_pct_x10,
                    torque_out.etb_enable_request, 2u);
                // Apply PID → H-bridge. On disable/fault/no-cal, spring-return safe.
                if (!torque_out.etb_enable_request || !etb.active || !g_etb_initialized) {
                    ::etb_driver_shutdown();
                } else {
                    // output_pct_x10 ∈ [-1000, 1000] → driver PWM ∈ [-1023, 1023]
                    int32_t pwm = (static_cast<int32_t>(etb.output_pct_x10) * 1023) / 1000;
                    if (pwm > 1023) { pwm = 1023; }
                    if (pwm < -1023) { pwm = -1023; }
                    if (!::etb_driver_set_motor_pwm(static_cast<int16_t>(pwm))) {
                        ::etb_driver_shutdown();
                    }
                }
            }
        }
    }
}

#endif  // TARGET_STM32H562
