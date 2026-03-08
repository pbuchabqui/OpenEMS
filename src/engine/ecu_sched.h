/**
 * @file engine/ecu_sched.h
 * @brief ECU Scheduling Core v2 — 32-bit timer + hardware Output Compare
 *
 * Modulo C/C++ (estilo MISRA) para agendamento angular de eventos de injecao
 * e ignicao via FTM0 com extensao de 32 bits por contagem de overflow.
 *
 * Arquitetura:
 *   FTM0 @ 120 MHz / PS_8 = 15 MHz ~ 66,7 ns/tick, free-running 16-bit.
 *   g_overflow_count estende o contador para 32 bits logicos:
 *     timestamp32 = (g_overflow_count << 16) | FTM0->CNT
 *
 *   CH0-CH3 (INJ1-4): Output Compare, Set on match (HIGH = injetor energizado).
 *   CH4-CH7 (IGN4-1): Output Compare, Clear on match (LOW  = disparo de bobina).
 *   CH5 (IGN3):       Output Compare para ignição do cilindro 3.
 *                     NOTA: O PDB0 é disparado pelo trigger de saída do FTM0
 *                     (TRGSEL=0x8), que é independente de qualquer canal CnV.
 *                     NÃO há conflito de hardware entre IGN3 e o trigger PDB.
 *                     O comentário anterior 'reservado para PDB' estava incorreto.
 *
 *   PDB0: disparado por FTM0 output trigger (TRGSEL=0x8).
 *   ADC0: hardware averaging 4 amostras (SC3: AVGE=1, AVGS=00).
 *
 * Coexistencia com C++17:
 *   Este header envolve todas as declaracoes em extern "C" quando compilado
 *   como C++. O FTM0_IRQHandler e definido em ecu_sched.cpp e substitui o
 *   handler genérico em hal/ftm.cpp.
 *
 * Referencia: K64P144M120SF5 Reference Manual, Rev. 2
 *   Cap. 43 — FlexTimer Module (FTM)
 *   Cap. 36 — Programmable Delay Block (PDB)
 *   Cap. 31 — Analog-to-Digital Converter (ADC)
 */

#ifndef ENGINE_ECU_SCHED_H
#define ENGINE_ECU_SCHED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CMSIS-style peripheral structs
 * ========================================================================= */

/**
 * FTM channel register pair (CnSC + CnV), stride 8 bytes.
 * K64 RM Table 43-2: offset 0x0C + n*8 for CnSC, 0x10 + n*8 for CnV.
 */
typedef struct {
    volatile uint32_t CnSC; /*!< Channel Status and Control */
    volatile uint32_t CnV;  /*!< Channel Value              */
} FTM_Channel_t;

/**
 * FlexTimer Module (FTM) register map.
 * Base addresses: FTM0=0x40038000, FTM3=0x400B9000 (K64 RM Table 3-1).
 */
typedef struct {
    volatile uint32_t SC;           /*!< 0x00: Status and Control      */
    volatile uint32_t CNT;          /*!< 0x04: Counter                 */
    volatile uint32_t MOD;          /*!< 0x08: Modulo                  */
    FTM_Channel_t     CONTROLS[8];  /*!< 0x0C-0x48: Channels 0-7      */
    volatile uint32_t CNTIN;        /*!< 0x4C: Counter Initial Value   */
    volatile uint32_t STATUS;       /*!< 0x50: Capture and Compare Status */
    volatile uint32_t MODE;         /*!< 0x54: Features Mode Selection */
} FTM_Type;

/**
 * Programmable Delay Block (PDB) register map.
 * Base address: PDB0=0x40036000 (K64 RM Table 3-1).
 * Only the fields used by this module are mapped; reserved gaps are
 * represented as padding arrays to maintain correct offsets.
 */
typedef struct {
    volatile uint32_t SC;           /*!< 0x00: Status and Control      */
    volatile uint32_t MOD;          /*!< 0x04: Modulo                  */
    volatile uint32_t CNT;          /*!< 0x08: Counter                 */
    volatile uint32_t IDLY;         /*!< 0x0C: Interrupt Delay         */
    volatile uint32_t CH0C1;        /*!< 0x10: Channel 0 Control 1     */
    volatile uint32_t CH0S;         /*!< 0x14: Channel 0 Status        */
    volatile uint32_t CH0DLY0;      /*!< 0x18: Channel 0 Delay 0       */
    volatile uint32_t CH0DLY1;      /*!< 0x1C: Channel 0 Delay 1       */
    volatile uint32_t _pad0[8];     /*!< 0x20-0x3F: reserved           */
    volatile uint32_t CH1C1;        /*!< 0x40: Channel 1 Control 1     */
} PDB_Type;

/**
 * Analog-to-Digital Converter (ADC) register map.
 * Base address: ADC0=0x4003B000 (K64 RM Table 3-1).
 * Only the fields used by this module are mapped.
 */
typedef struct {
    volatile uint32_t SC1A;  /*!< 0x00: Status and Control 1 (channel A) */
    volatile uint32_t SC1B;  /*!< 0x04: Status and Control 1 (channel B) */
    volatile uint32_t CFG1;  /*!< 0x08: Configuration 1                  */
    volatile uint32_t CFG2;  /*!< 0x0C: Configuration 2                  */
    volatile uint32_t RA;    /*!< 0x10: Data Result A                    */
    volatile uint32_t RB;    /*!< 0x14: Data Result B                    */
    volatile uint32_t CV1;   /*!< 0x18: Compare Value 1                  */
    volatile uint32_t CV2;   /*!< 0x1C: Compare Value 2                  */
    volatile uint32_t SC2;   /*!< 0x20: Status and Control 2             */
    volatile uint32_t SC3;   /*!< 0x24: Status and Control 3             */
} ADC_Type;

/* ============================================================================
 * Peripheral base address macros (target hardware only)
 * ========================================================================= */

#if !defined(EMS_HOST_TEST)
#define FTM0  ((FTM_Type *)0x40038000U)  /*!< FlexTimer 0 base address */
#define PDB0  ((PDB_Type *)0x40036000U)  /*!< PDB 0 base address       */
#define ADC0  ((ADC_Type *)0x4003B000U)  /*!< ADC 0 base address       */
#else
/* In host test builds the structs are backed by mocks defined in the test
 * file. The macros are defined there to point at the mock instances. */
#endif

/* ============================================================================
 * FTM register bit field constants (K64 RM §43.3)
 * ========================================================================= */

/* FTM_SC bits */
#define FTM_SC_TOF_MASK     (1UL << 7U)  /*!< Timer Overflow Flag (W0C)      */
#define FTM_SC_TOIE_MASK    (1UL << 6U)  /*!< Timer Overflow Interrupt Enable */
#define FTM_SC_CLKS_SYSTEM  (1UL << 3U)  /*!< Clock Source: system clock      */
#define FTM_SC_PS_8         (3UL)        /*!< Prescaler 8                     */
#define FTM_SC_PS_128       (7UL)        /*!< Prescaler 128                   */

/* FTM_MODE bits */
#define FTM_MODE_WPDIS      (1UL << 2U)  /*!< Write-Protect Disable  */
#define FTM_MODE_FTMEN      (1UL << 0U)  /*!< FTM Enable             */

/* FTM CnSC bits (K64 RM §43.3.5) */
#define FTM_CnSC_CHF_MASK   (1UL << 7U)  /*!< Channel Flag (W0C)            */
#define FTM_CnSC_CHIE_MASK  (1UL << 6U)  /*!< Channel Interrupt Enable       */
#define FTM_CnSC_MSB_MASK   (1UL << 5U)  /*!< Mode Select B                  */
#define FTM_CnSC_ELSB_MASK  (1UL << 3U)  /*!< Edge/Level Select B            */
#define FTM_CnSC_ELSA_MASK  (1UL << 2U)  /*!< Edge/Level Select A            */

/* Output Compare, Set on match (HIGH): MSB=1, ELSB=1, ELSA=0 */
#define FTM_CnSC_OC_SET    (FTM_CnSC_CHIE_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK)
/* Output Compare, Clear on match (LOW): MSB=1, ELSB=0, ELSA=1 */
#define FTM_CnSC_OC_CLEAR  (FTM_CnSC_CHIE_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK)

/* ============================================================================
 * PDB register bit field constants (K64 RM §36.3)
 * ========================================================================= */

#define PDB_SC_PDBEN_MASK   (1UL << 0U)   /*!< PDB Enable             */
#define PDB_SC_LDOK_MASK    (1UL << 6U)   /*!< Load OK                */
/* TRGSEL field [15:12]: 0x8 = FTM0 output trigger */
#define PDB_SC_TRGSEL_FTM0  (0x8UL << 12U)
#define PDB_CHnC1_EN0_MASK  (1UL << 0U)   /*!< Channel pre-trigger 0 enable */

/* ============================================================================
 * ADC register bit field constants (K64 RM §31.3.5)
 * ========================================================================= */

/* ADC_CFG1: ADIV=1 (/2), MODE=01 (12-bit), ADICLK=00 (bus) */
#define ADC_CFG1_12B_DIV2   ((1UL << 5U) | (1UL << 2U))
/* ADC_SC3: AVGE=1 (bit2), AVGS=00 (bits[1:0]) -> hardware averaging 4 samples */
#define ADC_SC3_AVG4        (1UL << 2U)

/* ============================================================================
 * SIM clock gating (K64 RM §12.2)
 * ========================================================================= */

#define SIM_SCGC6_ADDR      (0x4004803CUL)
#define SIM_SCGC6_FTM0_MASK (1UL << 24U)
#define SIM_SCGC6_ADC0_MASK (1UL << 27U)
#define SIM_SCGC6_PDB_MASK  (1UL << 23U)

/* ============================================================================
 * Event action codes
 * ========================================================================= */

#define ECU_ACT_INJ_ON       0U  /*!< Injector open   (CH0-CH3 set HIGH)  */
#define ECU_ACT_INJ_OFF      1U  /*!< Injector close  (CH0-CH3 set LOW)   */
#define ECU_ACT_DWELL_START  2U  /*!< Coil dwell start (CH4-CH7 set HIGH) */
#define ECU_ACT_SPARK        3U  /*!< Spark / coil cut (CH4-CH7 set LOW)  */

/* Pre-sync fallback modes (used before FULL_SYNC) */
#define ECU_PRESYNC_INJ_SIMULTANEOUS   0U
#define ECU_PRESYNC_INJ_SEMI_SEQUENTIAL 1U
#define ECU_PRESYNC_IGN_WASTED_SPARK   0U

/* ============================================================================
 * Channel assignments (matches FTM0 channel wiring)
 * ========================================================================= */

#define ECU_CH_INJ1   2U  /*!< FTM0 CH2 — Injector 1 */
#define ECU_CH_INJ2   3U  /*!< FTM0 CH3 — Injector 2 */
#define ECU_CH_INJ3   0U  /*!< FTM0 CH0 — Injector 3 */
#define ECU_CH_INJ4   1U  /*!< FTM0 CH1 — Injector 4 */
#define ECU_CH_IGN1   7U  /*!< FTM0 CH7 — Ignition 1 */
#define ECU_CH_IGN2   6U  /*!< FTM0 CH6 — Ignition 2 */
#define ECU_CH_IGN3   5U  /*!< FTM0 CH5 — Ignition 3.
                             * O PDB0 usa o trigger de saída global do FTM0
                             * (TRGSEL=0x8), independente deste canal CnV.
                             * Não há conflito de hardware com o MAP windowing. */
#define ECU_CH_IGN4   4U  /*!< FTM0 CH4 — Ignition 4 */

/* ============================================================================
 * Queue and sizing constants
 * ========================================================================= */

/*!< Maximum scheduled events in flight. */
#define ECU_QUEUE_SIZE  32U

/* ============================================================================
 * Timing constants (derived from clock and prescaler)
 * ========================================================================= */

/*!< System clock frequency in Hz */
#define ECU_SYSTEM_CLOCK_HZ    120000000U

/*!< FTM0 prescaler (8) */
#define ECU_FTM0_PRESCALER     8U

/*!< FTM3 prescaler (2) */
#define ECU_FTM3_PRESCALER     2U

/*!< FTM0 ticks per millisecond: 120MHz / 8 / 1000 = 15000 */
#define ECU_FTM0_TICKS_PER_MS  15000U

/*!< FTM0 ticks per microsecond: 120MHz / 8 / 1e6 = 15 */
#define ECU_FTM0_TICKS_PER_US  15U

/*!< FTM3 tick period in nanoseconds: 2 / 120MHz * 1e9 = 16.667 */
#define ECU_FTM3_TICK_NS       17U

/* ============================================================================
 * Module globals (accessible for diagnostics / inter-module coordination)
 * ========================================================================= */

/*!< 32-bit overflow extension for FTM0 counter (high 16 bits of timestamp). */
extern volatile uint32_t g_overflow_count;

/*!< Cumulative count of events that arrived too late for scheduling. */
extern volatile uint32_t g_late_event_count;

/*!< Number of late events with measured delay (ticks). */
extern volatile uint32_t g_late_delay_samples;

/*!< Accumulated late-event delay in FTM0 ticks (for average computation). */
extern volatile uint32_t g_late_delay_sum_ticks;

/*!< Maximum single late-event delay observed in FTM0 ticks. */
extern volatile uint32_t g_late_delay_max_ticks;

/*!< Maximum queue depth observed since boot/reset. */
extern volatile uint8_t g_queue_depth_peak;

/*!< Peak queue depth observed during the last Calculate_Sequential_Cycle call. */
extern volatile uint8_t g_queue_depth_last_cycle_peak;

/*!< Number of cycle scheduling attempts dropped due to insufficient queue room. */
extern volatile uint32_t g_cycle_schedule_drop_count;

/*!< Number of calibration setter calls that required clamping/sanitization. */
extern volatile uint32_t g_calibration_clamp_count;

/* ============================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise FTM0, PDB0, and ADC0 hardware for ECU scheduling.
 *
 * Performs in order:
 *   1. Clock gating: FTM0, PDB0, ADC0 via SIM_SCGC6.
 *   2. FTM0: write-protect disable, free-running, MOD=0xFFFF, PS=8,
 *      TOIE=1 (overflow IRQ for 32-bit extension).
 *      CH0-CH3: Output Compare Set-on-match (injectors).
 *      CH4-CH7: Output Compare Clear-on-match (ignition coils).
 *   3. PDB0: trigger source = FTM0 (TRGSEL=0x8), CH0 enabled for ADC0.
 *   4. ADC0: 12-bit, bus-clock/2; SC3 = hardware averaging 4 samples.
 *
 * Must be called once during system initialisation, before any interrupts
 * are enabled and before Add_Event() or Calculate_Sequential_Cycle().
 */
void ECU_Hardware_Init(void);

/**
 * @brief Schedule a single output-compare event.
 *
 * Inserts the event into the sorted queue (ascending by timestamp32).
 * If timestamp32 is already in the past (ts >> 16 < g_overflow_count),
 * the event is executed immediately by software force and
 * g_late_event_count is incremented.
 *
 * @param timestamp32  Target 32-bit timestamp:
 *                     bits[31:16] = required value of g_overflow_count,
 *                     bits[15:0]  = value to load into FTM0->CONTROLS[ch].CnV.
 * @param channel      FTM0 channel index 0-7 (use ECU_CH_* constants).
 * @param action       Event action code (ECU_ACT_* constants).
 */
void Add_Event(uint32_t timestamp32, uint8_t channel, uint8_t action);

/**
 * @brief Schedule one complete 720-degree cycle (ignition + injection).
 *
 * Uses firing order 1-3-4-2. For each cylinder, adds:
 *   - ECU_ACT_DWELL_START at (TDC - advance - dwell_ticks)
 *   - ECU_ACT_SPARK        at (TDC - advance)
 *   - ECU_ACT_INJ_ON       at (TDC - soi_lead_deg)
 *   - ECU_ACT_INJ_OFF      at (INJ_ON + inj_pw_ticks)
 *
 * TDC compression angles (720-degree cycle):
 *   Cyl 1 = 0 deg, Cyl 3 = 180 deg, Cyl 4 = 360 deg, Cyl 2 = 540 deg.
 *
 * Timestamps are relative to current_timestamp plus the angular offset
 * converted to ticks using the module-level g_ticks_per_rev value.
 *
 * @param current_timestamp  Current 32-bit FTM0 timestamp (from
 *                           (g_overflow_count << 16) | FTM0->CNT).
 */
void Calculate_Sequential_Cycle(uint32_t current_timestamp);

/**
 * @brief Atomically commit scheduler calibration for one control cycle.
 *
 * Applies all timing parameters in one critical section so ISR-side cycle fill
 * observes a consistent snapshot (no mixed old/new fields).
 */
void ecu_sched_commit_calibration(uint32_t tpr,
                                  uint32_t advance_deg,
                                  uint32_t dwell_ticks,
                                  uint32_t inj_pw_ticks,
                                  uint32_t soi_lead_deg);

/**
 * @brief Set ticks-per-revolution used by Calculate_Sequential_Cycle().
 */
void ecu_sched_set_ticks_per_rev(uint32_t tpr);

/**
 * @brief Set spark advance angle in degrees BTDC.
 */
void ecu_sched_set_advance_deg(uint32_t adv);

/**
 * @brief Set dwell duration in FTM0 ticks.
 */
void ecu_sched_set_dwell_ticks(uint32_t dwell);

/**
 * @brief Set injector pulse width duration in FTM0 ticks.
 */
void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks);

/**
 * @brief Set SOI lead angle in degrees before TDC.
 */
void ecu_sched_set_soi_lead_deg(uint32_t soi_lead_deg);

/** Enable/disable fallback scheduling while in HALF_SYNC. */
void ecu_sched_set_presync_enable(uint8_t enable);

/** Select pre-sync injection mode (ECU_PRESYNC_INJ_*). */
void ecu_sched_set_presync_inj_mode(uint8_t mode);

/** Select pre-sync ignition mode (ECU_PRESYNC_IGN_*). */
void ecu_sched_set_presync_ign_mode(uint8_t mode);

/**
 * @brief Zera contadores de diagnóstico por ciclo de motor.
 *
 * Zera: g_calibration_clamp_count, g_late_event_count, g_late_delay_samples,
 * g_late_delay_sum_ticks, g_late_delay_max_ticks, g_cycle_schedule_drop_count,
 * g_queue_depth_peak.
 *
 * Deve ser chamada uma vez por stop de motor (rpm_x10=0 por >= 100 ms).
 * Os contadores passam a refletir apenas o período de marcha mais recente,
 * e não se acumulam como odômetro lifetime.
 *
 * Thread-safe: usa seção crítica. Segura para chamar do loop background.
 */
void ecu_sched_reset_diagnostic_counters(void);

/* ============================================================================
 * FTM0 interrupt handler (defined in ecu_sched.cpp, unified scheduler handler)
 * ========================================================================= */

void FTM0_IRQHandler(void);

/* ============================================================================
 * Test-only API (compiled only when EMS_HOST_TEST is defined)
 * ========================================================================= */

#if defined(EMS_HOST_TEST)
/** Reset all module state (queue, counters, globals) for test isolation. */
void ecu_sched_test_reset(void);

/** Return the number of events currently in the queue. */
uint8_t ecu_sched_test_queue_size(void);

/** Return a copy of one queue entry (0-indexed). Returns 0 if out of range. */
uint8_t ecu_sched_test_get_event(uint8_t index, uint32_t *ts,
                                  uint8_t *ch, uint8_t *act);

/** Set the ticks-per-revolution value used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_ticks_per_rev(uint32_t tpr);

/** Set the advance angle (degrees) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_advance_deg(uint32_t adv);

/** Set the dwell duration (ticks) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_dwell_ticks(uint32_t dwell);

/** Set injector pulse width (ticks) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_inj_pw_ticks(uint32_t pw_ticks);

/** Set SOI lead (degrees) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_soi_lead_deg(uint32_t soi_lead_deg);

/** Return late delay samples count. */
uint32_t ecu_sched_test_get_late_delay_samples(void);

/** Return accumulated late delay (ticks). */
uint32_t ecu_sched_test_get_late_delay_sum_ticks(void);

/** Return maximum late delay (ticks). */
uint32_t ecu_sched_test_get_late_delay_max_ticks(void);

/** Return peak queue depth since reset. */
uint8_t ecu_sched_test_get_queue_depth_peak(void);

/** Return peak queue depth during last cycle fill. */
uint8_t ecu_sched_test_get_queue_depth_last_cycle_peak(void);

/** Return number of dropped cycle scheduling attempts. */
uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void);

/** Return current sanitized ticks-per-rev calibration. */
uint32_t ecu_sched_test_get_ticks_per_rev(void);

/** Return current sanitized dwell ticks calibration. */
uint32_t ecu_sched_test_get_dwell_ticks(void);

/** Return current sanitized injection pulse-width ticks calibration. */
uint32_t ecu_sched_test_get_inj_pw_ticks(void);

/** Return current sanitized SOI lead calibration (degrees). */
uint32_t ecu_sched_test_get_soi_lead_deg(void);

/** Return number of calibration clamp events. */
uint32_t ecu_sched_test_get_calibration_clamp_count(void);
#endif /* EMS_HOST_TEST */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ENGINE_ECU_SCHED_H */
