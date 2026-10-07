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
#include "hal/tle8888.h"
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
static constexpr uint8_t  kFaultBitMap  = (1u << 0u);  // SensorId::MAP
static constexpr uint8_t  kFaultBitClt  = (1u << 3u);  // SensorId::CLT
static constexpr uint8_t  kFaultBitFuel = (1u << 6u);  // SensorId::FUEL_PRESS
static constexpr uint8_t  kFaultBitOil  = (1u << 7u);  // SensorId::OIL_PRESS
// Oil must be present above this RPM; fuel-rail fault cuts after cranking.
static constexpr uint32_t kOilProtectRpmX10  = 15000u;  // 1500 RPM
// Coolant protection (value-based, not just open/short fault_bits).
static constexpr int16_t  kOvertempWarnX10   = 1050;    // 105 °C
static constexpr int16_t  kOvertempCritX10   = 1150;    // 115 °C
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

static constexpr uint32_t kSchedulerTicksPerMs = 62500u;  // TIM5_CNT @ 62.5 MHz
static constexpr uint32_t kCalibSaveMinIntervalMs = 300000u;
static constexpr uint16_t kMapMinBarX100 = 10u;
static constexpr uint16_t kMapMaxBarX100 = 300u;
static constexpr uint16_t kLambdaMinMilli = 700u;
static constexpr uint16_t kLambdaMaxMilli = 1400u;
static constexpr uint16_t kAePeriodMs = 2u;

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
using ems::engine::clamp_i16;

static inline int8_t clamp_i8(int16_t v, int8_t lo, int8_t hi) noexcept {
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return static_cast<int8_t>(v);
}

// NVM map/corr loaders: app/nvm_boot.cpp (relocate-only hygiene PR-09).

struct CachedFuelCorrections {
    bool valid;
    int16_t clt_x10;
    int16_t iat_x10;
    uint16_t vbatt_mv;
    uint16_t corr_clt_x256;
    uint16_t corr_iat_x256;
    uint16_t iat_density_q8;
    uint16_t dead_time_us;
    // dwell_ms_x10 não é cacheado: depende de RPM que varia a cada dente.
    // Calculado inline via dwell_ms_x10_from_vbatt_rpm() em cada slot de 2ms.
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
        g_fuel_corr_cache.iat_density_q8 = ems::engine::corr_iat_density_q8(sensors.iat_degc_x10);
        g_fuel_corr_cache.dead_time_us = ems::engine::corr_vbatt(sensors.vbatt_mv);
    }
    return g_fuel_corr_cache;
}

// Final injector PW: ΔP + S-curve act on flow only; dead-time is added after
// (flow + dead×squirts) and never scaled. Telemetry shows 0 under fuel cut
// (mask inhibits the injectors; PW keeps being committed for smooth resume).
// Returns the per-opening pulse in scheduler ticks.
static uint32_t inj_finish_pw(uint32_t flow_us, uint8_t squirts, uint16_t dead_us,
                              const ems::drv::SensorData& sensors,
                              uint16_t map_bar_x100, bool fuel_cut_active,
                              uint32_t& cycle_on_us) noexcept {
    const uint32_t scurve_pw_us = ems::engine::apply_injector_scurve(
        ems::engine::apply_delta_p_compensation(
            flow_us,
            ((sensors.fault_bits & kFaultBitFuel) != 0u) ? 0u : sensors.fuel_press_bar_x1000,
            map_bar_x100));
    cycle_on_us = ems::engine::inj_cycle_pw_us(scurve_pw_us, dead_us, squirts);
    const uint32_t pw_100 = cycle_on_us / 100u;
    g_last_pw_ms_x10 = fuel_cut_active ? 0u
        : static_cast<uint8_t>(pw_100 > 255u ? 255u : pw_100);
    return ems::engine::inj_pw_us_to_scheduler_ticks(
        ems::engine::inj_pulse_pw_us(scurve_pw_us, dead_us, squirts));
}

static void commit_sched(int16_t spark_deg, uint32_t dwell_ticks, uint32_t inj_pw_ticks,
                         const ems::drv::CkpSnapshot& snap,
                         const ems::drv::SensorData& sensors) noexcept {
    g_last_advance_deg = clamp_i8(spark_deg, -10, 40);
    ::ecu_sched_commit_calibration(
        static_cast<uint32_t>(spark_deg < 0 ? 0 : spark_deg),
        dwell_ticks,
        inj_pw_ticks,
        static_cast<uint32_t>(ems::engine::calc_eoi_lead_deg(
            snap.rpm_x10, sensors.clt_degc_x10)));
}

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
    ems::hal::tle8888_init();
    // Enables de hardware do estágio de potência (INJEN=PE14 / IGNEN=PE3).
    // Nasceram LOW em out_pins_hw_init(); só sobem se o TLE8888 confirmou
    // comunicação E configuração (direct drive, VR, enables por canal). Se o CI
    // não respondeu, injecção e ignição ficam inibidas por hardware.
    // No RGT6 é no-op. Ver docs/hw/interface_board_v1.md.
    ems::hal::power_stage_enable(ems::hal::tle8888_ok());
    ems::engine::ewg_control_init();
    ems::hal::flex_fuel_init();
    iwdg_kick();

	// 5) Flash Bank2 → carrega calibrações persistidas
	if (!ems::hal::nvm_load_calibration(0u, g_calib_page0, kCalibPageBytes)) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
	ems::engine::cfg::engine_config_load(g_calib_page0, kCalibPageBytes);
	ems::engine::map_estimator_sync_engine_config();  // displacement → MAP model
	// Calibração de sensores persistida (página 0, bytes 16-55) → drivers
	ems::engine::apply_etb_calibration_from_page(g_calib_page0 + 16, 40u);
	ems::engine::push_sensor_calibration_to_drivers();
	// Trims / CMP window / anti-jerk / rev limiter / ckp skip (56-76).
	// Antes só a UI aplicava isto — reboot perdia a calibração.
	ems::engine::apply_page0_trims_driveability(g_calib_page0, kCalibPageBytes);
	// Polaridade CKP/CMP (page0[258]) — re-aplica TIM5 + pull (tim5_ic_init foi
	// antes da NVM, default subida/pull-down).
	ems::engine::apply_page0_capture_polarity(g_calib_page0, kCalibPageBytes);
	// Closed-loop / LEARN (page0[80-85])
	ems::engine::closed_loop_enable =
	    (g_calib_page0[80] != 0u) ? 1u : 0u;
	ems::engine::ltft_apply_burn_ve = (g_calib_page0[81] != 0u) ? 1u : 0u;
	std::memcpy(&ems::engine::closed_loop_post_start_s, g_calib_page0 + 82, 2u);
	std::memcpy(&ems::engine::ltft_adapt_min_rpm_x10,   g_calib_page0 + 84, 2u);
	// Authority LTFT (176-184) só se layout version actual — blob v2 tem lixo/zeros.
	if (g_calib_page0[ems::engine::kCalLayoutVersionOffset] ==
	    ems::engine::kCalLayoutVersion) {
		uint16_t mult_c = 0u, add_c = 0u, max_s = 0u;
		std::memcpy(&mult_c, g_calib_page0 + 176, 2u);
		std::memcpy(&add_c,  g_calib_page0 + 178, 2u);
		std::memcpy(&max_s,  g_calib_page0 + 182, 2u);
		if (mult_c != 0u) { ems::engine::ltft_mult_clamp_pct_x10 = mult_c; }
		if (add_c  != 0u) { ems::engine::ltft_add_clamp_us = add_c; }
		if (g_calib_page0[180] != 0u) { ems::engine::ltft_learn_div = g_calib_page0[180]; }
		if (g_calib_page0[181] != 0u) { ems::engine::ltft_commit_gain_pct = g_calib_page0[181]; }
		ems::engine::ltft_max_step_x10 = max_s;
		if (g_calib_page0[184] <= 1u) {
			ems::engine::ltft_adapt_enable = g_calib_page0[184];
		}
		{
			uint16_t hits = 0u;
			std::memcpy(&hits, g_calib_page0 + 185, 2u);
			if (hits != 0u) { ems::engine::ltft_learn_ready_hits = hits; }
			if (g_calib_page0[187] != 0u) {
				ems::engine::ltft_learn_max_err_x1000 = g_calib_page0[187];
			}
			if (g_calib_page0[188] != 0u) {
				ems::engine::ltft_learn_ready_max_mean_err = g_calib_page0[188];
			}
			if (g_calib_page0[189] != 0u) {
				ems::engine::ltft_learn_ready_min_stft_x10 = g_calib_page0[189];
			}
			if (g_calib_page0[190] != 0u) {
				ems::engine::ltft_learn_ready_max_stft_x10 = g_calib_page0[190];
			}
		}
		// Launch + TC knobs (page0 191-215, layout v5)
		ems::engine::launch_tc_apply_from_page0(g_calib_page0, kCalibPageBytes);
		// CAN RX map: gear / vehicle / driven wheel (216-245)
		ems::app::can_rx_map_apply_from_page0(g_calib_page0, kCalibPageBytes);
		// MAP janela angular (246-251); len=0 não substitui o default
		ems::engine::map_window_enable = (g_calib_page0[246] != 0u) ? 1u : 0u;
		// 247: use_for_fuel — gate separado, exige calibração prévia de
		// open_deg/len_deg no motor real (ver AVISO em map_window.h).
		ems::engine::map_window_use_for_fuel = (g_calib_page0[247] != 0u) ? 1u : 0u;
		{
			uint16_t od = 0u, wl = 0u;
			std::memcpy(&od, g_calib_page0 + 248, 2u);
			std::memcpy(&wl, g_calib_page0 + 250, 2u);
			ems::engine::map_window_open_deg =
			    (od >= 720u) ? static_cast<uint16_t>(od % 720u) : od;
			if (wl != 0u) {
				ems::engine::map_window_len_deg =
				    (wl < 10u) ? 10u : (wl > 180u) ? 180u : wl;
			}
		}
		// Protecção de duty INJ + gates DFCO + knock morto (252-257);
		// blob antigo = zeros = tudo off (tol=0 mantém default 300 ms).
		ems::engine::inj_duty_max_pct = g_calib_page0[252];
		if (g_calib_page0[253] != 0u) {
			ems::engine::inj_duty_tol_ms10 = g_calib_page0[253];
		}
		std::memcpy(&ems::engine::decel_cut_map_max_bar_x100,
		            g_calib_page0 + 254, 2u);
		ems::engine::decel_cut_gear_inhibit_ms10 = g_calib_page0[256];
		ems::engine::knock_dead_min_p2p = g_calib_page0[257];
		if (kCalibPageBytes > (ems::engine::kDecelCutRampMsPage0Off + 1u)) {
			std::memcpy(&ems::engine::decel_cut_ramp_ms,
			            g_calib_page0 + ems::engine::kDecelCutRampMsPage0Off, 2u);
		}
	}
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
            const bool full_sync = (snap.state == ems::drv::SyncState::FULL_SYNC);
            const bool sched_sync = (snap.state == ems::drv::SyncState::HALF_SYNC || full_sync);

            const bool map_fault = (sensors.fault_bits & kFaultBitMap) != 0u;
            // MAP "sensor": por padrão o IIR ao vivo de sensors.cpp (suaviza
            // no tempo, não sincronizado ao ângulo de admissão). Quando
            // map_window_use_for_fuel=1 (opt-in separado, exige calibração
            // prévia de open_deg/len_deg — ver AVISO em map_window.h) E sync
            // pleno/cam confirmada NO INSTANTE ATUAL (não só quando a janela
            // fechou — evita servir média congelada após perda de sync) E
            // já há pelo menos 1 ciclo medido, usa a média das 4 janelas
            // angulares por cilindro em vez do IIR — livre de ripple de
            // pulso síncrono ao motor, ao custo de até ~1 ciclo de lag
            // (aceitável: map_estimator já reduz o peso do "sensor" durante
            // transientes detectados via TPSdot, a favor do modelo).
            uint16_t map_bar_x100_raw = static_cast<uint16_t>(sensors.map_bar_x1000 / 10u);
            if (ems::engine::map_window_use_for_fuel != 0u &&
                ems::engine::map_window_enable != 0u &&
                snap.state == ems::drv::SyncState::FULL_SYNC &&
                snap.cmp_confirms >= 2u &&
                ems::engine::map_window_cycles() > 0u) {
                map_bar_x100_raw = static_cast<uint16_t>(
                    ems::engine::map_window_mean_bar_x1000() / 10u);
            }
            const uint16_t map_bar_x100_sensor = clamp_u16(map_bar_x100_raw, kMapMinBarX100, kMapMaxBarX100);
            // Throttle signal for manifold model: ETB blade if harness present, else APP.
            const uint16_t tps_for_map = (ems::engine::etb_harness_present != 0u)
                ? sensors.etb_tps_pct_x10
                : sensors.app_pct_x10;
            // Fusion: sensor_valid=false on MAP fault so fallback 1 bar is not trusted.
            const uint16_t map_bar_x100 = ems::engine::map_estimator_update(
                map_bar_x100_sensor,
                tps_for_map,
                kAePeriodMs,
                snap.rpm_x10,
                sensors.iat_degc_x10,
                !map_fault);
            const bool clt_fault = (sensors.fault_bits & kFaultBitClt) != 0u;
            const bool oil_fault = (sensors.fault_bits & kFaultBitOil) != 0u;
            const bool fuel_press_fault = (sensors.fault_bits & kFaultBitFuel) != 0u;
            // Overtemp from CLT value (open/short already covered by clt_fault).
            const bool overtemp_warn = sensors.clt_degc_x10 >= kOvertempWarnX10;
            const bool overtemp_crit = sensors.clt_degc_x10 >= kOvertempCritX10;
            if (overtemp_crit) {
                ems::engine::DiagnosticManager::report_fault(
                    ems::engine::DiagnosticCode::OVERTEMP_CRITICAL,
                    ems::engine::FaultSeverity::CRITICAL,
                    static_cast<uint16_t>(sensors.clt_degc_x10), 0u);
            } else if (overtemp_warn) {
                ems::engine::DiagnosticManager::report_fault(
                    ems::engine::DiagnosticCode::OVERTEMP_WARNING,
                    ems::engine::FaultSeverity::WARNING,
                    static_cast<uint16_t>(sensors.clt_degc_x10), 0u);
            } else {
                ems::engine::DiagnosticManager::clear_fault(
                    ems::engine::DiagnosticCode::OVERTEMP_CRITICAL);
                ems::engine::DiagnosticManager::clear_fault(
                    ems::engine::DiagnosticCode::OVERTEMP_WARNING);
            }
            const bool diag_critical =
                !ems::engine::DiagnosticManager::is_system_ready();
            g_limp_active = map_fault || clt_fault || oil_fault || overtemp_warn;
            // Fuel angular policy (cuts live in limp_gating):
            //   (1) FULL_SYNC → running fuel
            //   (2) HALF_SYNC + cranking → batch; Hall does NOT allow HALF running fuel
            //   (3) flood / protect / LOSS_OF_SYNC → inj cut
            const bool limp_rpm_cut = g_limp_active &&
                (snap.rpm_x10 > kLimpRpmLimit_x10);
            const CachedFuelCorrections& fuel_corr = fuel_corrections_for(sensors);
            // Dwell 2D: tensão × RPM (MS42 §2.2.2.2.1).
            // Calculado fora do cache porque depende de RPM que varia a cada dente.
            const uint16_t dwell_ms_x10 = ems::engine::dwell_ms_x10_from_vbatt_rpm(
                sensors.vbatt_mv, snap.rpm_x10);
            const uint32_t dwell_ticks =
                (static_cast<uint32_t>(dwell_ms_x10) * kSchedulerTicksPerMs) / 10u;

            // Multi-spark (MS42 §2.2.3): habilita/desabilita conforme RPM gate.
            // Hard ceiling 1500 RPM (kMsparkRpmCeilingX10) — window too short above.
            // O dwell inter-spark é mais curto (tabela dedicada mspark_inter_dwell_ms_x10).
            // Limite 18°ATDC garante que o último spark contribui para a combustão.
            {
                uint16_t ms_gate = ems::engine::mspark_max_rpm_x10;
                if (ms_gate == 0u || ms_gate > ems::engine::kMsparkRpmCeilingX10) {
                    ms_gate = ems::engine::kMsparkRpmCeilingX10;
                }
                const bool ms_on = snap.rpm_x10 < ms_gate && ems::engine::mspark_count > 0u;
                ::ecu_sched_set_mspark(
                    ms_on ? ems::engine::mspark_count : 0u,
                    ms_on ? (static_cast<uint32_t>(ems::engine::mspark_inter_dwell_ms_x10)
                             * kSchedulerTicksPerMs) / 10u : 0u,
                    18u);
            }
            // Quick-crank state once per 2 ms tick (HALF + FULL + stopped).
            // Must not be gated on FULL_SYNC fuel — is_cranking() drives presync
            // SIMULTANEOUS, ETB crank open-loop, and HALF batch fuel.
            ems::engine::quick_crank_set_prime_context(sensors.clt_degc_x10,
                                                       fuel_corr.dead_time_us);
            const auto qc = ems::engine::quick_crank_update(
                now, snap.rpm_x10, sched_sync, sensors.clt_degc_x10, 0);
            // Gate closed-loop enrichments during crank + afterstart (not raw RPM).
            const bool crank_or_ase = qc.cranking || qc.afterstart_active;
            const bool flood_clear =
                ems::engine::crank_flood_clear_active(sensors.app_pct_x10);
            const bool half_sync = sched_sync && !full_sync;

            if (snap.rpm_x10 > g_dbg_rev_limit_rpm_max) {
                g_dbg_rev_limit_rpm_max = snap.rpm_x10;
            }
            {
                const uint32_t hard = ems::engine::rev_limit_rpm_x10;
                const uint16_t win = ems::engine::spark_skip_window_rpm_x10;
                const uint8_t  mx  = ems::engine::spark_skip_max_q8;
                uint8_t ratio = 0u;
                if (win != 0u && mx != 0u && hard > win &&
                    snap.rpm_x10 >= (hard - win) && snap.rpm_x10 < hard) {
                    const uint32_t into = snap.rpm_x10 - (hard - win);
                    ratio = static_cast<uint8_t>(
                        (static_cast<uint32_t>(mx) * into) / win);
                }
                ems::engine::spark_skip_set_ratio_q8(ratio);
                static uint16_t s_prev_tooth = 0u;
                if (snap.tooth_index < s_prev_tooth) {
                    ems::engine::spark_skip_on_rev();
                }
                s_prev_tooth = snap.tooth_index;
            }

            const bool etb_fault =
                (ems::engine::etb_harness_present != 0u) &&
                ((sensors.throttle_fault_bits &
                  (ems::drv::THROTTLE_FAULT_ETB_TPS1 |
                   ems::drv::THROTTLE_FAULT_ETB_TPS2 |
                   ems::drv::THROTTLE_FAULT_ETB_PLAUS)) != 0u);
            const uint16_t lambda_x1000 = ems::app::can_stack_lambda_milli();
            const bool lambda_valid = ems::app::can_stack_wbo2_fresh(now);
            const uint16_t lambda_target_x1000_gate =
                ems::engine::get_lambda_target_x1000(snap.rpm_x10, map_bar_x100);

            ems::engine::LimpGatingInputs gate_in{};
            gate_in.rpm_x10 = snap.rpm_x10;
            gate_in.map_bar_x100 = map_bar_x100;
            gate_in.clt_degc_x10 = sensors.clt_degc_x10;
            gate_in.oil_press_bar_x1000 = sensors.oil_press_bar_x1000;
            gate_in.oil_fault = oil_fault;
            gate_in.fuel_press_fault = fuel_press_fault;
            gate_in.lambda_x1000 = lambda_x1000;
            gate_in.lambda_valid = lambda_valid;
            gate_in.lambda_target_x1000 = lambda_target_x1000_gate;
            gate_in.tps_pct_x10 = sensors.app_pct_x10;
            gate_in.cranking = qc.cranking;
            gate_in.full_sync = full_sync;
            gate_in.half_sync = half_sync;
            gate_in.phase_valid = true;
            gate_in.sequential = ::ecu_sched_is_sequential() != 0u;
            gate_in.etb_fault = etb_fault;
            gate_in.inj_duty_pct = static_cast<uint8_t>(
                ems::engine::fuel_inj_duty_pct_x10() / 10u);
            gate_in.inj_duty_cut = ems::engine::fuel_inj_duty_cut_active();
            gate_in.now_ms = now;
            gate_in.map_fault = map_fault;
            gate_in.overtemp_crit = overtemp_crit;
            gate_in.diag_critical = diag_critical;
            gate_in.flood_clear = flood_clear;
            gate_in.limp_rpm_cut = limp_rpm_cut;
            gate_in.half_sync_allows_fuel = false;
            const ems::engine::LimpGatingResult gate =
                ems::engine::limp_gating_update(gate_in);
            ems::app::ui_set_rev_limit_active(g_rev_limit_active);

            const bool fuel_protect_cut = gate.fuel_protect_cut;
            const bool half_fuel_lockout = gate.half_fuel_lockout;
            const bool allow_half_crank_batch =
                half_sync && qc.cranking && !flood_clear && !fuel_protect_cut;

            // Telemetry PW must match actuators: only when injectors are actually cut.
            const bool fuel_cut_active =
                g_rev_limit_active || fuel_protect_cut || half_fuel_lockout ||
                ems::engine::fuel_inj_duty_cut_active();

            // (1) FULL_SYNC: running fuel path (VE / trims / AE / X-τ when not crank-ASE).
            if (full_sync && !fuel_protect_cut) {
                const ems::engine::Table2dLookup fuel_lookup =
                    ems::engine::table3d_prepare_lookup(ems::engine::kRpmAxisX10,
                                                        ems::engine::kLoadAxisBarX100,
                                                        snap.rpm_x10,
                                                        map_bar_x100);
                const uint8_t  ve = ems::engine::get_ve_prepared(fuel_lookup);
                const uint16_t lambda_target_x1000 =
                    ems::engine::get_lambda_target_x1000_prepared(fuel_lookup);
                // LTFT apply = nearest cell (mesma política que crédito/store LEARN).
                // fuel_lookup.yi/xi são floor da bilineal VE — mid-bin errava a célula.
                const int16_t fuel_trim_pct_x10 = crank_or_ase ? 0 : clamp_i16(
                    static_cast<int16_t>(ems::engine::fuel_get_stft_pct_x10() +
                                         ems::engine::fuel_get_ltft_at(snap.rpm_x10, map_bar_x100)),
                    -500, 500);
                // AE/DE from map-fusion TPSdot (signed: tip-in >0, tip-out <0).
                const int16_t ae_tpsdot = ems::engine::map_get_tpsdot_x10();
                int32_t ae_pw_us = crank_or_ase ? 0
                    : ems::engine::calc_ae_pw_from_tpsdot(ae_tpsdot, sensors.clt_degc_x10);
                uint32_t final_pw_us_base =
                    ems::engine::calc_fuel_pw_us_default_fast(ve,
                                                               map_bar_x100,
                                                               fuel_corr.iat_density_q8,
                                                               lambda_target_x1000,
                                                               fuel_trim_pct_x10,
                                                               fuel_corr.corr_clt_x256,
                                                               fuel_corr.corr_iat_x256,
                                                               fuel_corr.dead_time_us);
                // Corte de combustível na desaceleração (MS42 TI_PUR).
                // Avaliado ANTES do X-Tau: evita alimentar o modelo de parede com PW
                // real e depois descartar o resultado, contaminando a auto-calibração.
                // Contexto DFCO: MAP p/ o gate de vácuo e marcha p/ inibição
                // pós-troca (ambos inertes com as respectivas cals a 0).
                ems::engine::fuel_decel_cut_notify_map(map_bar_x100);
                {
                    uint8_t gr = 0u;
                    if (ems::engine::vehicle_gear(gr, now)) {
                        ems::engine::fuel_decel_cut_notify_gear(gr, now);
                    }
                }
                // Pedal (APP) = driver intent, valid with or without ETB (the
                // ETB blade opens by itself for idle air; without ETB the
                // ETB TPS input reads 0 and would allow a cut under load).
                const bool decel_cut_active = !crank_or_ase &&
                    ems::engine::fuel_decel_cut_update(
                        snap.rpm_x10, sensors.app_pct_x10, sensors.clt_degc_x10);
                ems::engine::misfire_set_all_inhibit(
                    decel_cut_active || crank_or_ase || flood_clear);
                // X-τ desde !cranking (inclui afterstart frio — pior wall-wetting).
                // AE residual a 50% quando X-τ activo (evita empilhar enrich).
                const bool xtau_enabled = !qc.cranking;
                if (xtau_enabled && ae_pw_us > 0) {
                    ae_pw_us /= 2;
                }
                if (decel_cut_active) {
                    ems::engine::limp_gating_or_fuel_reason(ems::engine::kFuelCutDfco);
                    g_last_net_pw_us = 0u;
                    ems::engine::fuel_ae_notify_pulse(0);
                    // Filme: reset só na entrada (não a cada tick do cut).
                    if (ems::engine::fuel_decel_cut_just_entered()) {
                        ems::engine::transient_fuel_reset();
                    }
                } else if (final_pw_us_base > fuel_corr.dead_time_us) {
                    uint32_t fuel_pw_us =
                        final_pw_us_base - static_cast<uint32_t>(fuel_corr.dead_time_us);

                    // LTFT aditivo (MS42 TI_AD_ADD_MMV): offset no PW líquido, célula nearest
                    if (!crank_or_ase) {
                        const int16_t ltft_add =
                            ems::engine::fuel_get_ltft_add_at(snap.rpm_x10, map_bar_x100);
                        const int32_t pw_adj = static_cast<int32_t>(fuel_pw_us) + ltft_add;
                        fuel_pw_us = (pw_adj <= 0) ? 0u
                                   : (pw_adj > 100000) ? 100000u
                                   : static_cast<uint32_t>(pw_adj);
                    }
                    g_last_net_pw_us = fuel_pw_us;

                    // Learn X-τ: apenas no slot 100ms (λ + STFT + tpsdot gates).
                    // Modelo de parede com τ escalado a wall-clock (period_ms).
                    const uint32_t xtau_fuel_pw_us =
                        ems::engine::transient_fuel_xtau_with_autocalib(fuel_pw_us,
                                                                        snap.rpm_x10,
                                                                        map_bar_x100,
                                                                        sensors.clt_degc_x10,
                                                                        xtau_enabled,
                                                                        kAePeriodMs);
                    // Só a parcela de FLUXO segue no pipeline; o dead-time
                    // eléctrico é somado no fim, depois de ΔP/S-curve
                    // (convenção de calc_final_pw_us — dead-time nunca escala).
                    final_pw_us_base = xtau_fuel_pw_us;
                } else {
                    ems::engine::transient_fuel_reset();
                    final_pw_us_base = 0u;  // fluxo ≈ 0 (base ≤ dead-time)
                }
                // AE tip-in (add) ou DE tip-out (subtract), clamp a [0, 100ms].
                if (!decel_cut_active && ae_pw_us != 0) {
                    const int64_t adj = static_cast<int64_t>(final_pw_us_base) + ae_pw_us;
                    if (adj <= 0) {
                        final_pw_us_base = 0u;
                    } else if (adj > 100000) {
                        final_pw_us_base = 100000u;
                    } else {
                        final_pw_us_base = static_cast<uint32_t>(adj);
                    }
                }
                // Soft ramp-in pós-DFCO (antes de quick_crank / ΔP).
                if (!decel_cut_active) {
                    final_pw_us_base =
                        ems::engine::fuel_decel_cut_ramp_pw(final_pw_us_base, 2u);
                }
                // Sempre: tip-in (µs>0) freezes STFT; pulse==0 limpa no próprio 2 ms.
                if (!decel_cut_active) {
                    ems::engine::fuel_ae_notify_pulse(ae_pw_us);
                }
                const int16_t base_advance_deg = ems::engine::get_advance_prepared(fuel_lookup);
                // Máximo entre os 4 cilindros, não só o cilindro 0 (FIX:
                // knock_retard_x10[] é genuinamente por cilindro, mas este
                // valor é aplicado como escalar único e partilhado a todos
                // os cilindros abaixo — máximo é a escolha conservadora,
                // nunca sub-retarda o cilindro que mais precisa. Retard
                // verdadeiramente por cilindro precisa de infra-estrutura
                // nova que não existe hoje — ver AdvanceCorrections/
                // calc_total_advance, escalar único, fora de escopo aqui).
                uint16_t knock_retard_x10 = 0u;
                for (uint8_t kc = 0u; kc < 4u; ++kc) {
                    const uint16_t r = ems::engine::knock_get_retard_x10(kc);
                    if (r > knock_retard_x10) { knock_retard_x10 = r; }
                }
                const uint16_t idle_target_rpm_x10 =
                    ems::engine::auxiliaries_idle_target_rpm_x10(sensors.clt_degc_x10);
                // Idle spark OK during afterstart (helps settle); suppressed only while cranking.
                const int16_t idle_spark_corr_deg = qc.cranking ? 0 :
                    ems::engine::calc_idle_spark_correction_deg(snap.rpm_x10,
                                                                idle_target_rpm_x10,
                                                                sensors.app_pct_x10,
                                                                map_bar_x100);
                const int16_t iat_spark_deg = qc.cranking ? 0 :
                    ems::engine::calc_ign_iat_correction_deg(sensors.iat_degc_x10);
                const int16_t clt_spark_deg = qc.cranking ? 0 :
                    ems::engine::calc_ign_clt_correction_deg(sensors.clt_degc_x10);
                const int16_t antijerk_retard = crank_or_ase ? 0 :
                    ems::engine::calc_antijerk_retard_deg(ae_tpsdot);
                const int16_t advance_deg = ems::engine::calc_total_advance(
                    base_advance_deg,
                    {iat_spark_deg, clt_spark_deg,
                     static_cast<int16_t>(knock_retard_x10 / 10u),
                     idle_spark_corr_deg, antijerk_retard,
                     g_torque_spark_retard_deg});
                // Cranking spark from qc (base was 0 at update); else table+corr.
                const int16_t sched_spark_deg = qc.cranking
                    ? ems::engine::crank_spark_deg
                    : advance_deg;
                // Decel / flood: force PW=0 (do not apply min_pw floor).
                const uint32_t quick_crank_pw_us =
                    (decel_cut_active || flood_clear) ? 0u :
                    ems::engine::quick_crank_flow_us(qc, final_pw_us_base);
                // Formula is flow + dead×openings (seq: +1 dead, semi: +2).
                // The pin splits that total across the openings; the gauge
                // shows the formula, not one opening.
                const uint8_t squirts = ::ecu_sched_is_sequential() ? 1u : 2u;
                uint32_t cycle_on_us = 0u;
                const uint32_t inj_pw_ticks = inj_finish_pw(
                    quick_crank_pw_us, squirts, fuel_corr.dead_time_us, sensors,
                    map_bar_x100, fuel_cut_active, cycle_on_us);

                // Protecção de duty (FOME #215): alimenta com o PW final
                // comandado; o corte em si entra na mask do próximo tick.
                ems::engine::fuel_inj_duty_update(cycle_on_us, snap.rpm_x10, 2u);

                commit_sched(sched_spark_deg, dwell_ticks, inj_pw_ticks, snap, sensors);
            } else if (allow_half_crank_batch) {
                // (2) HALF_SYNC + cranking: simultaneous batch, crank PW only (no VE/STFT/AE).
                // Presync auto already selects SIMULTANEOUS while is_cranking().
                // Force mode in case auto was off or race with tooth ISR.
                ::ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
                ems::engine::misfire_set_all_inhibit(true);
                ems::engine::fuel_ae_notify_pulse(0);
                ems::engine::transient_fuel_reset();

                const uint32_t crank_flow_us = ems::engine::quick_crank_flow_us(qc, 0u);
                g_last_net_pw_us = crank_flow_us;
                // Cranking batch is simultaneous: formula flow+2×dead.
                uint32_t cycle_on_us = 0u;
                const uint32_t inj_pw_ticks = inj_finish_pw(
                    crank_flow_us, 2u, fuel_corr.dead_time_us, sensors,
                    map_bar_x100, fuel_cut_active, cycle_on_us);
                commit_sched(ems::engine::crank_spark_deg, dwell_ticks, inj_pw_ticks,
                             snap, sensors);
            } else if (sched_sync &&
                       (fuel_protect_cut || half_fuel_lockout || g_rev_limit_active)) {
                // (3) Spark-only: exit-crank HALF, flood, protect, rev-limit, anomaly path.
                // qc already updated — use crank spark only while still latched cranking.
                const int16_t base_advance_deg = ems::engine::get_advance(snap.rpm_x10, map_bar_x100);
                const int16_t sched_spark_deg = qc.cranking
                    ? ems::engine::crank_spark_deg
                    : base_advance_deg;
                commit_sched(sched_spark_deg, dwell_ticks, 0u, snap, sensors);
                g_last_pw_ms_x10 = 0u;
                g_last_net_pw_us = 0u;
                ems::engine::fuel_ae_notify_pulse(0);
            }
            ems::app::ui_update_rt_map_fuel(map_bar_x100, g_last_net_pw_us);
            g_last_map_fused_x100 = map_bar_x100;

            // Prime one-shot: suppressed on flood-clear, fuel-protect (MAP/oil/rail/
            // overtemp/diag/rev limp), and whenever inj mask already locks all cyls.
            // force_output also honors the mask (defense in depth vs bypass).
            const bool prime_blocked =
                flood_clear || fuel_protect_cut || half_fuel_lockout ||
                g_rev_limit_active;
            const uint32_t prime_pw = prime_blocked
                ? 0u
                : ems::engine::quick_crank_consume_prime();
            if (prime_pw != 0u && !ems::engine::output_test_active()) {
                ::ecu_sched_fire_prime_pulse(prime_pw);
            } else if (prime_blocked) {
                // Drop pending prime so protect/flood cannot fire after condition clears mid-tooth.
                static_cast<void>(ems::engine::quick_crank_consume_prime());
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
                0u, 0u, 0u,  // former sync-seed counters (feature removed)
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
            ems::hal::tle8888_poll_diag();

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
