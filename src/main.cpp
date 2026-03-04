// =============================================================================
// OpenEMS — src/main.cpp
// Teensy 3.5 (MK64FX512VMD12, 120 MHz)
//
// Responsabilidade: sequência de init + main loop de background.
// Nenhuma lógica de cálculo ou controle aqui — toda a lógica vive
// nos módulos drv/, engine/ e app/.
// =============================================================================

#if defined(EMS_HOST_TEST)

int main() { return 0; }

#else  // ── target real ──────────────────────────────────────────────────────

#include <Arduino.h>
#include <cstdint>

#include "app/can_stack.h"
#include "app/tuner_studio.h"
#include "drv/ckp.h"
#include "drv/scheduler.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/cycle_sched.h"
#include "engine/ecu_sched.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/flexnvm.h"
#include "hal/ftm.h"
#include "hal/uart.h"

// =============================================================================
// NVIC helpers — Cortex-M4 (MK64F)
// =============================================================================
#define NVIC_ISER(n)   (*reinterpret_cast<volatile uint32_t*>(0xE000E100u + (n)*4u))
#define NVIC_IPR_BASE   0xE000E400u

// IRQ numbers — MK64F Sub-Family Reference Manual, Tabela 3-7
static constexpr uint32_t kIrqFtm3 = 71u;   // CKP input-capture   — prio  1
static constexpr uint32_t kIrqFtm0 = 42u;   // scheduler output-cmp — prio  4
static constexpr uint32_t kIrqAdc0 = 39u;   // ADC0 EOC            — prio  5
static constexpr uint32_t kIrqPit0 = 68u;   // timestamp 1 µs      — prio 11
static constexpr uint32_t kIrqPit1 = 69u;   // watchdog 100 ms     — prio 12

static inline void nvic_set_priority_enable(uint32_t irq, uint8_t prio) noexcept {
    volatile uint32_t* ipr =
        reinterpret_cast<volatile uint32_t*>(NVIC_IPR_BASE + (irq / 4u) * 4u);
    const uint32_t sh = (irq % 4u) * 8u;
    *ipr = (*ipr & ~(0xFFu << sh)) | (static_cast<uint32_t>(prio) << (sh + 4u));
    NVIC_ISER(irq / 32u) = 1u << (irq % 32u);
}

// =============================================================================
// PIT — Programmable Interrupt Timer
//   PIT0: timestamp de 1 µs (bus 60 MHz / 60 − 1 = kPit0Ldval)
//   PIT1: watchdog de main loop — ISR dispara SystemReset() após 100 ms
// =============================================================================
#define SIM_SCGC6      (*reinterpret_cast<volatile uint32_t*>(0x4004803Cu))
#define SIM_SCGC6_PIT  (1u << 23u)
#define PIT_MCR        (*reinterpret_cast<volatile uint32_t*>(0x40037000u))
#define PIT_LDVAL(n)   (*reinterpret_cast<volatile uint32_t*>(0x40037100u + (n)*0x10u))
#define PIT_TCTRL(n)   (*reinterpret_cast<volatile uint32_t*>(0x40037108u + (n)*0x10u))
#define PIT_TFLG(n)    (*reinterpret_cast<volatile uint32_t*>(0x4003710Cu + (n)*0x10u))

static constexpr uint32_t kPitBusHz    = 60000000u;
static constexpr uint32_t kPit0Ldval   = (kPitBusHz / 1000000u) - 1u;  // 1 µs/tick
static constexpr uint32_t kPit1Ldval   = (kPitBusHz / 10u) - 1u;       // 100 ms
static constexpr uint32_t kPitTen      = 1u << 0u;
static constexpr uint32_t kPitTie      = 1u << 1u;
static constexpr uint32_t kPitTif      = 1u << 0u;

// =============================================================================
// Estado de background
// =============================================================================

// Calibração page-0 carregada em RAM no boot
static constexpr uint16_t kCalibPageBytes = 512u;
alignas(4) static uint8_t g_calib_page0[kCalibPageBytes];
static bool                g_calib_dirty  = false;

// Timestamp global: incrementado pela ISR PIT0 a cada 1 µs
volatile uint32_t g_datalog_us = 0u;

// Marca de tempo de cada tarefa de background
static uint32_t g_t2ms   = 0u;     // 2ms strategy + scheduling loop
static uint32_t g_t10ms  = 0u;
static uint32_t g_t20ms  = 0u;
static uint32_t g_t50ms  = 0u;
static uint32_t g_t100ms = 0u;
static uint32_t g_t500ms = 0u;

// Últimos valores calculados de avanço e PW — publicados no frame CAN 0x400
static int8_t  g_last_advance_deg = 0;
static uint8_t g_last_pw_ms_x10   = 0u;

// Modo limp: ativo quando sensor crítico (MAP ou CLT) está em fault
// Rev-cut acima de kLimpRpmLimit_x10 para proteger o motor
static constexpr uint32_t kLimpRpmLimit_x10 = 30000u;  // 3000 RPM
static constexpr uint8_t  kFaultBitMap = (1u << 0u);   // SensorId::MAP
static constexpr uint8_t  kFaultBitClt = (1u << 3u);   // SensorId::CLT
static bool g_limp_active = false;

// Parâmetros de referência para cálculo de PW (a ser configurável via NVM)
static constexpr uint16_t kDefaultReqFuelUs = 8000u;  // 8 ms a VE=100%, MAP=MAP_ref
static constexpr uint16_t kMapRefKpa        = 100u;   // MAP de referência (100 kPa ≈ atmosfera)

// =============================================================================
// Utilitários de infraestrutura — sem lógica de domínio
// =============================================================================

static inline void pit_init() noexcept {
    SIM_SCGC6   |= SIM_SCGC6_PIT;
    PIT_MCR      = 0u;                  // habilita módulo, não congela em debug

    PIT_LDVAL(0) = kPit0Ldval;
    PIT_TFLG(0)  = kPitTif;
    PIT_TCTRL(0) = kPitTen | kPitTie;

    PIT_LDVAL(1) = kPit1Ldval;
    PIT_TFLG(1)  = kPitTif;
    PIT_TCTRL(1) = kPitTen | kPitTie;
}

// Reload do watchdog — deve ser o primeiro statement de loop()
static inline void pit1_kick() noexcept {
    PIT_TCTRL(1) &= ~kPitTen;
    PIT_TFLG(1)   = kPitTif;
    PIT_LDVAL(1)  = kPit1Ldval;
    PIT_TCTRL(1) |= kPitTen;
}

[[noreturn]] static inline void system_reset() noexcept {
    *reinterpret_cast<volatile uint32_t*>(0xE000ED0Cu) =
        (0x5FAu << 16u) | (1u << 2u);
    for (;;) {}
}

// Verifica se `period` ms passou desde `last`
static inline bool elapsed(uint32_t now, uint32_t last, uint32_t period) noexcept {
    return static_cast<uint32_t>(now - last) >= period;
}

// Drena UART RX e TX dentro de orçamento para não bloquear o loop
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
// ISR Handlers
// =============================================================================

extern "C" void ADC0_IRQHandler(void) {
    // Pipeline ADC por interrupção — reservado para implementação futura
}

extern "C" void PIT0_IRQHandler(void) {
    PIT_TFLG(0) = kPitTif;
    ++g_datalog_us;
}

extern "C" void PIT1_IRQHandler(void) {
    PIT_TFLG(1) = kPitTif;
    system_reset();     // loop travado > 100 ms
}

// =============================================================================
// setup() — sequência obrigatória (Prompt 11, passos 1–10)
// =============================================================================
void setup() {
    // 1) PLL → 120 MHz: gerenciado pelo runtime Teensyduino antes de setup()

    // 2) FlexTimer Modules
    ems::hal::ftm0_init();
    ems::hal::ftm3_init();
    ems::hal::ftm1_pwm_init(125u);
    ems::hal::ftm2_pwm_init(150u);

    // 2a) Initialize unified scheduling system (FTM0, PDB, ADC)
    ::ECU_Hardware_Init();

    // 3) ADC + PDB  (PDB embutido em adc_init)
    ems::hal::adc_init();

    // 4) CAN + UART0
    ems::hal::can0_init();
    ems::hal::uart0_init();

    // 5) FlexNVM → carrega calibração page-0 em RAM
    static_cast<void>(
        ems::hal::nvm_load_calibration(0u, g_calib_page0, kCalibPageBytes));

    // 6) Drivers de sistema
    ems::drv::sched_cancel_all();   // limpa fila do scheduler (sched_init)
    ems::drv::sensors_init();
    // ckp_init: estado zero-inicializado; pino e NVIC configurados em ftm3_init()

    // 7) Camada engine
    ems::engine::fuel_reset_adaptives();   // fuel_init
    ems::engine::auxiliaries_init();       // aux_init
    ems::engine::knock_init();
    // ign_init: tabelas estáticas, sem init dedicado
    ems::engine::cycle_sched_init();       // pré-computa dentes-gatilho por cilindro

    // 8) Camada de aplicação
    ems::app::ts_init();
    ems::app::can_stack_init();

    // 9) NVIC — prioridades conforme Prompt 11
    nvic_set_priority_enable(kIrqFtm3, 1u);   // CKP — maior prioridade
    nvic_set_priority_enable(kIrqFtm0, 4u);   // scheduler
    nvic_set_priority_enable(kIrqAdc0, 5u);   // ADC0 EOC
    nvic_set_priority_enable(kIrqPit0, 11u);  // timestamp µs
    nvic_set_priority_enable(kIrqPit1, 12u);  // watchdog

    pit_init();     // PIT0 (timestamp) + PIT1 (watchdog 100 ms)

    // 10) Aguardar sincronismo CKP antes de habilitar sched_event()
    // Timeout de 5 s: se a roda fônica não girar (motor parado, sensor ausente),
    // continua em modo degradado sem injeção/ignição em vez de travar para sempre.
    {
        constexpr uint32_t kCkpSyncTimeoutMs = 5000u;
        const uint32_t t_sync_start = millis();
        while (ems::drv::ckp_snapshot().state != ems::drv::SyncState::FULL_SYNC) {
            ts_service();
            pit1_kick();
            if (static_cast<uint32_t>(millis() - t_sync_start) >= kCkpSyncTimeoutMs) {
                break;  // modo degradado: sem CKP sync, sem injeção/ignição
            }
        }
    }
    // Habilita agendamento por ângulo de virabrequim (ativo somente com CKP SYNCED)
    // Note: cycle_sched is disabled - using unified ecu_sched system instead
    // ems::engine::cycle_sched_enable(true);

    const uint32_t now = millis();
    g_t2ms = g_t10ms = g_t20ms = g_t50ms = g_t100ms = g_t500ms = now;
}

// =============================================================================
// loop() — tarefas de background com millis() (Prompt 11)
//
// pit1_kick() SEMPRE primeiro — garante detecção de travamento em qualquer tarefa.
// Snapshots de ckp/sensors capturados uma vez por iteração.
// =============================================================================
void loop() {
    pit1_kick();    // watchdog — obrigatório na primeira linha

    const uint32_t              now     = millis();
    const ems::drv::CkpSnapshot ckp     = ems::drv::ckp_snapshot();
    const ems::drv::SensorData& sensors = ems::drv::sensors_get();

    // ── 2 ms: Estratégia (Massa de Ar, VE, Tempo de Injeção/Ignição) ─────────
    if (elapsed(now, g_t2ms, 2u)) {
        g_t2ms += 2u;

        // Limp mode: sensores MAP ou CLT em fault → rev-cut acima de 3000 RPM
        const uint8_t critical_faults =
            static_cast<uint8_t>(sensors.fault_bits & (kFaultBitMap | kFaultBitClt));
        g_limp_active = (critical_faults != 0u) &&
                        (ckp.rpm_x10 > kLimpRpmLimit_x10);

        // Cálculo de Massa de Ar (VE) - estratégia principal
        // CRITICAL FIX: Validate sensor data before using in calculations
        if (!ems::drv::validate_sensor_values(sensors)) {
            // Invalid sensor data - force limp mode
            g_limp_active = true;
        }
        
        const uint8_t ve = ems::engine::get_ve(
            static_cast<uint16_t>(ckp.rpm_x10), sensors.map_kpa_x10 / 10u);

        // Cálculo de Tempo de Injeção (PW) - estratégia principal
        const uint16_t map_kpa = sensors.map_kpa_x10 / 10u;
        uint32_t base_pw_us;
        if (g_limp_active) {
            base_pw_us = 1000u;  // Safe limp mode fallback: 1ms
        } else {
            base_pw_us = ems::engine::calc_base_pw_us(
                kDefaultReqFuelUs, ve, map_kpa, kMapRefKpa);
        }
        const uint32_t pw_ms_x10_raw = base_pw_us / 100u;  // µs → ms×10
        g_last_pw_ms_x10 = static_cast<uint8_t>(
            pw_ms_x10_raw > 255u ? 255u : pw_ms_x10_raw);

        // Cálculo de Tempo de Ignição em Graus - estratégia principal
        const int16_t base_adv = ems::engine::get_advance(
            static_cast<uint16_t>(ckp.rpm_x10), sensors.map_kpa_x10 / 10u);
        g_last_advance_deg = static_cast<int8_t>(ems::engine::clamp_advance_deg(base_adv));

        // Atualiza parâmetros para o scheduler angular
        
        // Calculate ticks per revolution based on current RPM
        if (ckp.rpm_x10 > 0u) {
            // ticks_per_rev = (clock / prescaler) * 60s / rpm
            // = (ECU_SYSTEM_CLOCK_HZ / ECU_FTM0_PRESCALER) * 60 * 10 / rpm_x10
            // = (ECU_SYSTEM_CLOCK_HZ * 600) / (ECU_FTM0_PRESCALER * rpm_x10)
            ::ecu_sched_set_ticks_per_rev(static_cast<uint32_t>(
                (static_cast<uint64_t>(ECU_SYSTEM_CLOCK_HZ) * 60U * 10U) 
                / (static_cast<uint64_t>(ECU_FTM0_PRESCALER) * ckp.rpm_x10)
            ));
        }

        const uint32_t advance_deg = (g_last_advance_deg > 0)
            ? static_cast<uint32_t>(g_last_advance_deg)
            : 0u;
        ::ecu_sched_set_advance_deg(advance_deg);
        
        // Convert dwell time from ms to ticks
        const uint16_t dwell_ms_x10 = ems::engine::dwell_ms_x10_from_vbatt(sensors.vbatt_mv);
        const uint32_t dwell_ticks = (static_cast<uint32_t>(dwell_ms_x10) * ECU_FTM0_TICKS_PER_MS) / 100u;
        ::ecu_sched_set_dwell_ticks(dwell_ticks);

        // Atualiza métricas de tempo real
        ems::app::ts_update_rt_metrics(
            g_last_pw_ms_x10,
            g_last_advance_deg,
            static_cast<int8_t>(ems::engine::fuel_get_stft_pct_x10() / 10));

        // Use unified scheduling system with 32-bit timestamps
        // Calculate current timestamp for scheduling
        const uint32_t current_timestamp = (::g_overflow_count << 16) | ems::hal::ftm0_count();
        
        // Schedule the complete 720° cycle using unified system
        ::Calculate_Sequential_Cycle(current_timestamp);
    }

    // ── 10 ms: IACV PID, VVT PID, Boost PID ──────────────────────────────────
    if (elapsed(now, g_t10ms, 10u)) {
        g_t10ms += 10u;
        ems::engine::auxiliaries_tick_10ms();
    }

    // ── 20 ms: buffer TunerStudio RX/TX + auxiliares 20 ms ───────────────────
    if (elapsed(now, g_t20ms, 20u)) {
        g_t20ms += 20u;
        ts_service();
        ems::engine::auxiliaries_tick_20ms();
    }

    // ── 50 ms: sensores lentos + CAN 0x400 ───────────────────────────────────
    if (elapsed(now, g_t50ms, 50u)) {
        g_t50ms += 50u;
        ems::drv::sensors_tick_50ms();
        ems::app::can_stack_process(
            now, ckp, sensors,
            g_last_advance_deg,
            g_last_pw_ms_x10,
            /*stft_pct*/      static_cast<int8_t>(ems::engine::fuel_get_stft_pct_x10() / 10),
            /*vvt_intake_pct*/  0u,
            /*vvt_exhaust_pct*/ 0u,
            /*status*/  static_cast<uint8_t>(
                (ckp.state == ems::drv::SyncState::FULL_SYNC ? 0x01u : 0u) |
                (ckp.phase_A                               ? 0x02u : 0u) |
                (sensors.fault_bits != 0u                  ? 0x04u : 0u) |
                (g_limp_active                             ? 0x08u : 0u)));
    }

    // ── 100 ms: sensores muito lentos + CAN 0x401 + LTFT ─────────────────────
    if (elapsed(now, g_t100ms, 100u)) {
        g_t100ms += 100u;
        ems::drv::sensors_tick_100ms();
        static_cast<void>(ems::engine::fuel_update_stft(
            static_cast<uint16_t>(ckp.rpm_x10),
            static_cast<uint16_t>(sensors.map_kpa_x10 / 10u),
            1000,
            static_cast<int16_t>(ems::app::can_stack_lambda_milli()),
            sensors.clt_degc_x10,
            ems::app::can_stack_wbo2_fresh(now),
            /*ae_active*/ false,
            /*rev_cut*/   g_limp_active));
    }

    // ── 500 ms: flush calibração → FlexNVM se dirty + knock NVM ─────────────
    if (elapsed(now, g_t500ms, 500u)) {
        g_t500ms += 500u;
        if (g_calib_dirty) {
            static_cast<void>(
                ems::hal::nvm_save_calibration(0u, g_calib_page0, kCalibPageBytes));
            g_calib_dirty = false;
        }
        ems::engine::knock_save_to_nvm();
    }
}

#endif  // EMS_HOST_TEST
