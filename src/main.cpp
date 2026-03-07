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

// Parâmetros de referência para cálculo de PW (a ser configurável via NVM)
// REQ_FUEL: tempo de injeção base a VE=100% e MAP=MAP_ref.
// Derivação: REQ_FUEL [µs] = (displacement_cc / cylinders) * BSFC / (injector_cc_per_min / 60e6)
// Exemplo: 1600cc / 4 cyl = 400cc; BSFC ~250g/kWh; injetor 240cc/min → ~8000 µs
// Ajustar via calibração em bancada após medição de lambda em regime estacionário.
static constexpr uint16_t kDefaultReqFuelUs  = 8000u;

// MAP de referência usado na fórmula PW = REQ_FUEL × VE × (MAP / MAP_REF).
// 100 kPa = pressão atmosférica ao nível do mar (ponto de calibração do REQ_FUEL).
static constexpr uint16_t kMapRefKpa         = 100u;

// SOI lead: ângulo antes do TDC compressão em que o injetor abre.
// 62° BTDC = ~60° antes do TDC, típico para injeção port-fuel com tempo de
// voo de ~3 ms a 3000 RPM (3 ms × 3000/60 × 360° = 54° — arredondado com margem).
// Ajustar conforme diâmetro de válvula e pressão de injeção do motor específico.
static constexpr uint32_t kDefaultSoiLeadDeg = 62u;

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
    // Pipeline ADC por interrupção — não deve ser chamada enquanto NVIC
    // estiver desabilitado para ADC0. Implementação futura: ler ADC0->RA,
    // publicar resultado via fila lock-free para sensors_tick_50ms().
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
    {
        ems::hal::RuntimeSyncSeed seed = {};
        if (ems::hal::nvm_load_runtime_seed(&seed) &&
            ems::hal::runtime_seed_fast_reacquire_compatible_60_2(seed)) {
            const bool phase_a =
                ((seed.flags & ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A) != 0u);
            ems::drv::ckp_seed_arm(phase_a);
            g_runtime_seed_arm_window_active = true;
            g_runtime_seed_arm_window_start_ms = millis();
            // One-shot policy: avoid reusing stale seed across repeated power cycles.
            static_cast<void>(ems::hal::nvm_clear_runtime_seed());
        }
    }

    // 6) Drivers de sistema
    ems::drv::sensors_init();
    // ckp_init: estado zero-inicializado; pino e NVIC configurados em ftm3_init()

    // 7) Camada engine
    ems::engine::fuel_reset_adaptives();   // fuel_init
    ems::engine::auxiliaries_init();       // aux_init
    ems::engine::knock_init();
    ems::engine::quick_crank_reset();
    // ign_init: tabelas estáticas, sem init dedicado
    ems::engine::cycle_sched_init();       // pré-computa dentes-gatilho por cilindro

    // 8) Camada de aplicação
    ems::app::ts_init();
    ems::app::can_stack_init();

    // 9) NVIC — prioridades conforme Prompt 11
    nvic_set_priority_enable(kIrqFtm3, 1u);   // CKP — maior prioridade
    nvic_set_priority_enable(kIrqFtm0, 4u);   // scheduler
    // ADC0 IRQ: prioridade configurada mas NVIC não habilitado até a ISR
    // ser implementada. Habilitar com nvic_set_priority_enable(kIrqAdc0, 5u)
    // quando o pipeline de ADC por interrupção estiver pronto.
    // nvic_set_priority_enable(kIrqAdc0, 5u);
    nvic_set_priority_enable(kIrqPit0, 11u);  // timestamp µs
    nvic_set_priority_enable(kIrqPit1, 12u);  // watchdog

    pit_init();     // PIT0 (timestamp) + PIT1 (watchdog 100 ms)

    // 10) Aguardar sincronismo CKP antes de habilitar o agendamento angular
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

    if (ckp.state == ems::drv::SyncState::FULL_SYNC) {
        g_last_full_sync_snapshot = ckp;
        g_have_last_full_sync = true;
        if (ckp.tooth_index == 0u) {
            g_last_gap_sync_snapshot = ckp;
            g_have_last_gap_sync = true;
        }
        g_runtime_seed_arm_window_active = false;
    } else if (g_runtime_seed_arm_window_active &&
               elapsed(now, g_runtime_seed_arm_window_start_ms, kRuntimeSeedArmWindowMs)) {
        ems::drv::ckp_seed_disarm();
        g_runtime_seed_arm_window_active = false;
    }
    if (ckp.rpm_x10 > 800u) {
        g_engine_was_running = true;
        g_runtime_seed_saved_for_stop = false;
    }
    // Marca o instante em que o RPM passou de >0 para 0 (borda de descida).
    // Atualiza APENAS nessa transição — não enquanto o motor está girando,
    // pois isso apagaria a referência temporal usada pelo runtime seed save.
    if ((g_prev_rpm_x10 > 0u) && (ckp.rpm_x10 == 0u)) {
        g_zero_rpm_since_ms = now;
    }
    g_prev_rpm_x10 = ckp.rpm_x10;

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

            // Aplica correções de temperatura (CLT, IAT) e dead-time de bateria.
            // Essencial para partida a frio e compensação de tensão do injetor.
            const uint16_t corr_clt_x256  = ems::engine::corr_clt(sensors.clt_degc_x10);
            const uint16_t corr_iat_x256  = ems::engine::corr_iat(sensors.iat_degc_x10);
            const uint16_t dead_time_us   = ems::engine::corr_vbatt(sensors.vbatt_mv);
            base_pw_us = ems::engine::calc_final_pw_us(
                base_pw_us, corr_clt_x256, corr_iat_x256, dead_time_us);
        }

        // Cálculo de Tempo de Ignição em Graus - estratégia principal
        const int16_t base_adv = ems::engine::get_advance(
            static_cast<uint16_t>(ckp.rpm_x10), sensors.map_kpa_x10 / 10u);
        const bool full_sync = (ckp.state == ems::drv::SyncState::FULL_SYNC);
        const ems::engine::QuickCrankOutput qc = ems::engine::quick_crank_update(
            now, ckp.rpm_x10, full_sync, sensors.clt_degc_x10, base_adv);

        if (!g_limp_active) {
            base_pw_us = ems::engine::quick_crank_apply_pw_us(
                base_pw_us, qc.fuel_mult_x256, qc.min_pw_us);
        }

        g_last_advance_deg = static_cast<int8_t>(
            ems::engine::clamp_advance_deg(qc.spark_deg));
        const uint32_t pw_ms_x10_raw = base_pw_us / 100u;  // µs → ms×10
        g_last_pw_ms_x10 = static_cast<uint8_t>(
            pw_ms_x10_raw > 255u ? 255u : pw_ms_x10_raw);

        // Atualiza parâmetros para o scheduler angular.
        // advance_deg é BTDC em graus. Valores negativos indicam retardo
        // (after TDC) — válidos em knock retard ou cold-start. O scheduler
        // recebe uint32_t, então representamos retardo como offset em cima
        // de ECU_CYCLE_DEG para que o wrap aritmético produza o ângulo correto.
        // Faixa válida de clamp_advance_deg: [-10, +35] graus.
        const uint32_t advance_deg =
            (g_last_advance_deg >= 0)
                ? static_cast<uint32_t>(g_last_advance_deg)
                : static_cast<uint32_t>(static_cast<int32_t>(ECU_CYCLE_DEG)
                                        + g_last_advance_deg);
        
        // Convert dwell time from ms to ticks
        const uint16_t dwell_ms_x10 = ems::engine::dwell_ms_x10_from_vbatt(sensors.vbatt_mv);
        const uint32_t dwell_ticks = (static_cast<uint32_t>(dwell_ms_x10) * ECU_FTM0_TICKS_PER_MS) / 100u;

        // Convert injector pulse width to FTM0 ticks at PS=8:
        // ticks = us * (ticks/ms) / 1000
        const uint32_t inj_pw_ticks = (base_pw_us * ECU_FTM0_TICKS_PER_MS) / 1000u;

        // Atomically commit a coherent scheduler calibration snapshot once per cycle.
        if (ckp.rpm_x10 > 0u) {
            const uint32_t ticks_per_rev = static_cast<uint32_t>(
                (static_cast<uint64_t>(ECU_SYSTEM_CLOCK_HZ) * 60U * 10U)
                / (static_cast<uint64_t>(ECU_FTM0_PRESCALER) * ckp.rpm_x10)
            );
            ::ecu_sched_commit_calibration(
                ticks_per_rev, advance_deg, dwell_ticks, inj_pw_ticks, kDefaultSoiLeadDeg);
        }

        // Atualiza métricas de tempo real
        ems::app::ts_update_rt_metrics(
            g_last_pw_ms_x10,
            g_last_advance_deg,
            static_cast<int8_t>(ems::engine::fuel_get_stft_pct_x10() / 10));
        ems::app::ts_update_rt_sched_diag(
            g_late_event_count,
            g_late_delay_max_ticks,
            g_queue_depth_peak,
            g_queue_depth_last_cycle_peak,
            g_cycle_schedule_drop_count,
            g_calibration_clamp_count,
            ems::drv::ckp_seed_loaded_count(),
            ems::drv::ckp_seed_confirmed_count(),
            ems::drv::ckp_seed_rejected_count(),
            static_cast<uint8_t>(ckp.state));

        // The ecu_sched queue is filled on CKP tooth hook (schedule_on_tooth),
        // aligned to sync boundaries to avoid duplicate cycle insertion.
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
                (ckp.state == ems::drv::SyncState::FULL_SYNC ? ems::app::STATUS_SYNC_FULL : 0u) |
                (ckp.phase_A                                  ? ems::app::STATUS_PHASE_A : 0u) |
                (sensors.fault_bits != 0u                     ? ems::app::STATUS_SENSOR_FAULT : 0u) |
                (g_limp_active                                ? ems::app::STATUS_LIMP_MODE : 0u) |
                (g_late_event_count != 0u                     ? ems::app::STATUS_SCHED_LATE : 0u) |
                (g_cycle_schedule_drop_count != 0u            ? ems::app::STATUS_SCHED_DROP : 0u) |
                (g_calibration_clamp_count != 0u              ? ems::app::STATUS_SCHED_CLAMP : 0u)));
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

        if (!g_runtime_seed_saved_for_stop &&
            g_engine_was_running &&
            g_have_last_full_sync &&
            (ckp.rpm_x10 == 0u) &&
            elapsed(now, g_zero_rpm_since_ms, kRuntimeSeedSaveDelayMs)) {
            ems::hal::RuntimeSyncSeed seed = {};
            const ems::drv::CkpSnapshot seed_snap =
                g_have_last_gap_sync ? g_last_gap_sync_snapshot : g_last_full_sync_snapshot;
            seed.flags = static_cast<uint8_t>(
                ems::hal::RUNTIME_SYNC_SEED_FLAG_FULL_SYNC |
                (seed_snap.phase_A
                    ? ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A
                    : 0u));
            seed.tooth_index = seed_snap.tooth_index;
            seed.decoder_tag = ems::hal::RUNTIME_SYNC_SEED_DECODER_TAG_60_2;
            if (ems::hal::nvm_save_runtime_seed(&seed)) {
                g_runtime_seed_saved_for_stop = true;
                g_engine_was_running = false;
            }
        }
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
