/**
 * @file engine/ecu_sched.h
 * @brief ECU Scheduling Core v3 — Domínio Angular (Angle-Domain Scheduling)
 *
 * Módulo C/C++ para agendamento de eventos de injeção e ignição no domínio
 * angular. Cada evento é armazenado como (tooth_index, sub_frac_x256, phase_A)
 * e disparado na ISR CKP do dente correspondente, calculando o offset FTM0
 * com o período do dente ATUAL — correto para o RPM instântaneo.
 *
 * Vantagens vs scheduling no domínio do tempo (v2):
 *   - Elimina race condition g_overflow_count (FTM3 prio 1 vs FTM0 prio 4)
 *   - Elimina retime_far_events_from_tooth_rpm() O(n) por dente na ISR CKP
 *   - Timing exato em variação de RPM (usa período atual, não aproximação)
 *   - FTM0_IRQHandler simplificado: sem TOF, sem gestão de fila
 *
 * Arquitetura:
 *   FTM0 @ 120 MHz / PS_64 = 1,875 MHz ~ 533,3 ns/tick, free-running 16-bit.
 *   CH0-CH3 (INJ1-4): Output Compare, Set on match (HIGH = injetor energizado).
 *   CH4-CH7 (IGN4-1): Output Compare, Clear on match (LOW = disparo de bobina).
 *
 *   PDB0: disparado por FTM0 output trigger (TRGSEL=0x8).
 *   ADC0: hardware averaging 4 amostras (SC3: AVGE=1, AVGS=00).
 *
 * Coexistência com C++17:
 *   Este header envolve todas as declarações em extern "C" quando compilado
 *   como C++. O FTM0_IRQHandler é definido em ecu_sched.cpp.
 *
 * Referência: K64P144M120SF5 Reference Manual, Rev. 2
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
    FTM_Channel_t     CH[8];        /*!< 0x0C-0x48: Channels 0-7      */
    volatile uint32_t CNTIN;        /*!< 0x4C: Counter Initial Value   */
    volatile uint32_t STATUS;       /*!< 0x50: Capture and Compare Status */
    volatile uint32_t MODE;         /*!< 0x54: Features Mode Selection */
} FTM_Type;

/**
 * Programmable Delay Block (PDB) register map.
 * Base address: PDB0=0x40036000 (K64 RM Table 3-1).
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
#define FTM_SC_PS_8         (3UL)        /*!< Prescaler 8   (legacy, não usado) */
#define FTM_SC_PS_16        (4UL)        /*!< Prescaler 16  (PS[2:0]=100b)    */
#define FTM_SC_PS_64        (6UL)        /*!< Prescaler 64  (PS[2:0]=110b)    */
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
#define ECU_PRESYNC_INJ_SIMULTANEOUS    0U
#define ECU_PRESYNC_INJ_SEMI_SEQUENTIAL 1U
#define ECU_PRESYNC_IGN_WASTED_SPARK    0U

/* ============================================================================
 * Phase constants (domínio angular)
 * ========================================================================= */

#define ECU_PHASE_A    1U  /*!< Primeira revolução do ciclo de 720° (phase_A=true)  */
#define ECU_PHASE_B    0U  /*!< Segunda revolução do ciclo de 720° (phase_A=false)  */
#define ECU_PHASE_ANY  2U  /*!< Dispara em toda revolução — usado em HALF_SYNC      */

/* ============================================================================
 * Channel assignments (matches FTM0 channel wiring)
 * ========================================================================= */

#define ECU_CH_INJ1   2U  /*!< FTM0 CH2 — Injector 1 */
#define ECU_CH_INJ2   3U  /*!< FTM0 CH3 — Injector 2 */
#define ECU_CH_INJ3   0U  /*!< FTM0 CH0 — Injector 3 */
#define ECU_CH_INJ4   1U  /*!< FTM0 CH1 — Injector 4 */
#define ECU_CH_IGN1   7U  /*!< FTM0 CH7 — Ignition 1 */
#define ECU_CH_IGN2   6U  /*!< FTM0 CH6 — Ignition 2 */
#define ECU_CH_IGN3   5U  /*!< FTM0 CH5 — Ignition 3 */
#define ECU_CH_IGN4   4U  /*!< FTM0 CH4 — Ignition 4 */

/* ============================================================================
 * Angle-domain event table
 * ========================================================================= */

/*!< Maximum events in the angle table: 4 cyl × 4 eventos + margem presync */
#define ECU_ANGLE_TABLE_SIZE  20U

/**
 * @brief Evento de scheduling no domínio angular.
 *
 * Armazenado como posição angular (tooth_index + sub_frac_x256) em vez de
 * timestamp absoluto. O offset FTM0 é calculado na ISR CKP usando o período
 * do dente ATUAL — imune a variações de RPM entre scheduling e disparo.
 */
typedef struct {
    uint8_t tooth_index;     /*!< Dente que dispara este evento (0–57)             */
    uint8_t sub_frac_x256;   /*!< Fração do período do dente × 256 (0=borda, 255≈próximo) */
    uint8_t channel;         /*!< Canal FTM0 0–7 (use ECU_CH_*)                   */
    uint8_t action;          /*!< Código da ação (ECU_ACT_*)                       */
    uint8_t phase_A;         /*!< ECU_PHASE_A, ECU_PHASE_B ou ECU_PHASE_ANY        */
    uint8_t valid;           /*!< Não-zero se o slot está ocupado                  */
} AngleEvent_t;

/* ============================================================================
 * Timing constants (derived from clock and prescaler)
 * ========================================================================= */

/*!< System clock frequency in Hz */
#define ECU_SYSTEM_CLOCK_HZ    120000000U

/*!< FTM0 prescaler = 64 → 1,875 MHz → 533,3 ns/tick */
#define ECU_FTM0_PRESCALER     64U

/*!< FTM3 prescaler (unchanged: 2 → 60 MHz → 16,67 ns/tick) */
#define ECU_FTM3_PRESCALER     2U

/*!< FTM0 effective clock: 120 MHz / 64 = 1 875 000 Hz */
#define ECU_FTM0_CLOCK_HZ      1875000U

/*!< FTM0 ticks per millisecond: 1 875 000 / 1000 = 1875 */
#define ECU_FTM0_TICKS_PER_MS  1875U

/*!< FTM0 ticks per microsecond: 1 875 000 / 1 000 000 ≈ 2 (arredondado; use TICKS_PER_MS/1000 para precisão) */
#define ECU_FTM0_TICKS_PER_US  2U

/*!< FTM0 nanoseconds per tick: 1e9 / 1 875 000 ≈ 533 ns */
#define ECU_FTM0_NS_PER_TICK   533U

/*!< FTM3 tick period in nanoseconds: 2 / 120MHz * 1e9 ≈ 17 ns */
#define ECU_FTM3_TICK_NS       17U

/* ============================================================================
 * Module globals (accessible for diagnostics)
 * ========================================================================= */

/*!< Cumulative count of events that could not be armed (tooth already past). */
extern volatile uint32_t g_late_event_count;

/*!< Number of calibration setter calls that required clamping/sanitization. */
extern volatile uint32_t g_calibration_clamp_count;

/*!< Number of cycle scheduling attempts dropped (angle table full). */
extern volatile uint32_t g_cycle_schedule_drop_count;

/* ============================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise FTM0, PDB0, and ADC0 hardware for ECU scheduling.
 *
 * Performs in order:
 *   1. Clock gating: FTM0, PDB0, ADC0 via SIM_SCGC6.
 *   2. FTM0: write-protect disable, free-running, MOD=0xFFFF, PS=64.
 *      Sem TOIE — overflow não precisa ser tratado no domínio angular.
 *      CH0-CH3: Output Compare Set-on-match (injectors).
 *      CH4-CH7: Output Compare Clear-on-match (ignition coils).
 *   3. PDB0: trigger source = FTM0 (TRGSEL=0x8), CH0 enabled for ADC0.
 *   4. ADC0: 12-bit, bus-clock/2; SC3 = hardware averaging 4 samples.
 *
 * Must be called once during system initialisation, before any interrupts
 * are enabled and before ecu_sched_commit_calibration().
 */
void ECU_Hardware_Init(void);

/**
 * @brief Atomically commit scheduler calibration for one control cycle.
 *
 * Applies all timing parameters in one critical section so ISR-side cycle fill
 * observes a consistent snapshot (no mixed old/new fields).
 *
 * Nota: tpr (ticks_per_rev) foi removido desta assinatura. O período por
 * revolução é calculado internamente a partir de snap.tooth_period_ns no
 * momento do scheduling (ecu_sched_on_tooth_hook), garantindo uso do RPM
 * instântaneo e eliminando o parâmetro obsoleto do domínio do tempo.
 *
 * @param advance_deg  Avanço de ignição em graus BTDC.
 * @param dwell_ticks  Duração do dwell em ticks FTM0 (PS=64).
 * @param inj_pw_ticks Largura de pulso do injetor em ticks FTM0 (PS=64).
 * @param soi_lead_deg Ângulo SOI em graus antes do TDC.
 */
void ecu_sched_commit_calibration(uint32_t advance_deg,
                                  uint32_t dwell_ticks,
                                  uint32_t inj_pw_ticks,
                                  uint32_t soi_lead_deg);

/**
 * @brief Set spark advance angle in degrees BTDC.
 */
void ecu_sched_set_advance_deg(uint32_t adv);

/**
 * @brief Set dwell duration in FTM0 ticks (PS=64).
 */
void ecu_sched_set_dwell_ticks(uint32_t dwell);

/**
 * @brief Set injector pulse width duration in FTM0 ticks (PS=64).
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
 * Thread-safe: usa seção crítica. Segura para chamar do loop background.
 */
void ecu_sched_reset_diagnostic_counters(void);

/**
 * @brief Define o ângulo de fechamento da válvula de admissão em graus ABDC.
 *
 * O IVC clamp é aplicado apenas em modo open-valve injection (SOI antes do IVC).
 * Com a estratégia padrão closed-valve (soi_lead_deg=62), o clamp nunca atua.
 *
 * @param ivc_abdc_deg  Ângulo IVC em graus após BDC (padrão: 50, faixa: 0–180).
 */
void ecu_sched_set_ivc(uint8_t ivc_abdc_deg);

/**
 * @brief Retorna o número acumulado de eventos onde o IVC clampeou o EOI.
 *
 * Incrementado em Calculate_Sequential_Cycle() e calculate_presync_revolution()
 * sempre que inj_pw_deg for reduzido para respeitar o IVC.
 * Resetado por ecu_sched_test_reset() em testes de host.
 */
uint32_t ecu_sched_ivc_clamp_count(void);

/**
 * @brief Dispara um prime pulse simultâneo em todos os 4 injetores.
 *
 * Projetado para ser chamado UMA VEZ por partida, no 5º dente CKP, pelo loop
 * de fundo após quick_crank_consume_prime() retornar valor > 0.
 *
 * Comportamento:
 *   - Arma CH0-CH3 (INJ3-4, INJ1-2) em SET mode com CnV = CNT+20 (ON imediato).
 *   - Registra CnV de OFF em g_prime_off_cnv[]; FTM0_IRQHandler re-arma CLEAR
 *     quando CHF dispara para cada canal.
 *   - pw_us é saturado em 30 000 µs para prevenir afogamento.
 *   - Segura para chamar do loop background (usa seção crítica internamente).
 *
 * @param pw_us  Largura de pulso em microssegundos (0 = nenhuma ação).
 */
void ecu_sched_fire_prime_pulse(uint32_t pw_us);

/* ============================================================================
 * FTM0 interrupt handler (defined in ecu_sched.cpp)
 * ========================================================================= */

void FTM0_IRQHandler(void);

/* ============================================================================
 * Test-only API (compiled only when EMS_HOST_TEST is defined)
 * ========================================================================= */

#if defined(EMS_HOST_TEST)
/** Reset all module state for test isolation. */
void ecu_sched_test_reset(void);

/** Return the number of valid events in the angle table. */
uint8_t ecu_sched_test_angle_table_size(void);

/**
 * Return a copy of one angle table entry (0-indexed).
 * Returns 0 if out of range or slot not valid.
 */
uint8_t ecu_sched_test_get_angle_event(uint8_t index,
                                        uint8_t *tooth,
                                        uint8_t *sub_frac,
                                        uint8_t *ch,
                                        uint8_t *action,
                                        uint8_t *phase);

/** Set the advance angle (degrees) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_advance_deg(uint32_t adv);

/** Set the dwell duration (ticks) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_dwell_ticks(uint32_t dwell);

/** Set injector pulse width (ticks) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_inj_pw_ticks(uint32_t pw_ticks);

/** Set SOI lead (degrees) used by Calculate_Sequential_Cycle. */
void ecu_sched_test_set_soi_lead_deg(uint32_t soi_lead_deg);

/** Return current sanitized advance angle calibration. */
uint32_t ecu_sched_test_get_advance_deg(void);

/** Return current sanitized dwell ticks calibration. */
uint32_t ecu_sched_test_get_dwell_ticks(void);

/** Return current sanitized injection pulse-width ticks calibration. */
uint32_t ecu_sched_test_get_inj_pw_ticks(void);

/** Return current sanitized SOI lead calibration (degrees). */
uint32_t ecu_sched_test_get_soi_lead_deg(void);

/** Return number of calibration clamp events. */
uint32_t ecu_sched_test_get_calibration_clamp_count(void);

/** Return number of dropped cycle scheduling attempts. */
uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void);

/** Return cumulative late event count. */
uint32_t ecu_sched_test_get_late_event_count(void);

/** Set IVC angle in degrees ABDC for test purposes. */
void ecu_sched_test_set_ivc(uint8_t ivc_abdc_deg);

/** Return cumulative IVC clamp event count. */
uint32_t ecu_sched_test_get_ivc_clamp_count(void);
#endif /* EMS_HOST_TEST */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ENGINE_ECU_SCHED_H */
