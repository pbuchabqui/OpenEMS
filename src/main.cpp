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
static uint32_t g_t5ms   = 0u;
static uint32_t g_t10ms  = 0u;
static uint32_t g_t20ms  = 0u;
static uint32_t g_t50ms  = 0u;
static uint32_t g_t100ms = 0u;
static uint32_t g_t500ms = 0u;

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
    while (ems::drv::ckp_snapshot().state != ems::drv::SyncState::SYNCED) {
        ts_service();
        pit1_kick();
    }

    const uint32_t now = millis();
    g_t5ms = g_t10ms = g_t20ms = g_t50ms = g_t100ms = g_t500ms = now;
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

    // ── 5 ms: fuel_calc + ign_calc → sched_event para próximos ciclos ────────
    if (elapsed(now, g_t5ms, 5u)) {
        g_t5ms = now;
        ems::drv::sched_recalc(ckp);
    }

    // ── 10 ms: IACV PID, VVT PID, Boost PID ──────────────────────────────────
    if (elapsed(now, g_t10ms, 10u)) {
        g_t10ms = now;
        ems::engine::auxiliaries_tick_10ms();
    }

    // ── 20 ms: buffer TunerStudio RX/TX + auxiliares 20 ms ───────────────────
    if (elapsed(now, g_t20ms, 20u)) {
        g_t20ms = now;
        ts_service();
        ems::engine::auxiliaries_tick_20ms();
    }

    // ── 50 ms: sensores lentos + CAN 0x400 ───────────────────────────────────
    if (elapsed(now, g_t50ms, 50u)) {
        g_t50ms = now;
        ems::drv::sensors_tick_50ms();
        ems::app::can_stack_process(
            now, ckp, sensors,
            /*adv_i8*/  0,
            /*egt*/     0u,
            /*stft_i8*/ static_cast<int8_t>(ems::engine::fuel_get_stft_pct_x10() / 10),
            /*ltft_i8*/ 0,
            /*boost*/   0u,
            /*status*/  static_cast<uint8_t>(
                (ckp.state == ems::drv::SyncState::SYNCED ? 0x01u : 0u) |
                (ckp.phase_A                               ? 0x02u : 0u) |
                (sensors.fault_bits != 0u                  ? 0x04u : 0u)));
    }

    // ── 100 ms: sensores muito lentos + CAN 0x401 + LTFT ─────────────────────
    if (elapsed(now, g_t100ms, 100u)) {
        g_t100ms = now;
        ems::drv::sensors_tick_100ms();
        static_cast<void>(ems::engine::fuel_update_stft(
            static_cast<uint16_t>(ckp.rpm_x10),
            static_cast<uint16_t>(sensors.map_kpa_x10 / 10u),
            1000,
            static_cast<int16_t>(ems::app::can_stack_lambda_milli()),
            sensors.clt_degc_x10,
            ems::app::can_stack_wbo2_fresh(now),
            /*ae_active*/ false,
            /*rev_cut*/   false));
    }

    // ── 500 ms: flush calibração → FlexNVM se dirty ──────────────────────────
    if (elapsed(now, g_t500ms, 500u)) {
        g_t500ms = now;
        if (g_calib_dirty) {
            static_cast<void>(
                ems::hal::nvm_save_calibration(0u, g_calib_page0, kCalibPageBytes));
            g_calib_dirty = false;
        }
    }
}

#endif  // EMS_HOST_TEST
