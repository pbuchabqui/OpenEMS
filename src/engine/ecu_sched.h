#ifndef ENGINE_ECU_SCHED_H
#define ENGINE_ECU_SCHED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECU_ACT_INJ_ON       0U
#define ECU_ACT_INJ_OFF      1U
#define ECU_ACT_DWELL_START  2U
#define ECU_ACT_SPARK        3U

#define ECU_PRESYNC_INJ_SIMULTANEOUS    0U
#define ECU_PRESYNC_INJ_SEMI_SEQUENTIAL 1U
#define ECU_PRESYNC_IGN_WASTED_SPARK    0U

#define ECU_PHASE_A    1U
#define ECU_PHASE_B    0U
#define ECU_PHASE_ANY  2U

#define ECU_CH_INJ1   2U
#define ECU_CH_INJ2   3U
#define ECU_CH_INJ3   0U
#define ECU_CH_INJ4   1U
#define ECU_CH_IGN1   7U
#define ECU_CH_IGN2   6U
#define ECU_CH_IGN3   5U
#define ECU_CH_IGN4   4U

#define ECU_ANGLE_TABLE_SIZE  48U  // base 16 + até 3 sparks adicionais × 4 cil × 2 eventos = 40; margem 8

typedef struct {
    uint8_t tooth_index;
    uint8_t sub_frac_x256;
    uint8_t channel;
    uint8_t action;
    uint8_t phase_A;
    uint8_t valid;
} AngleEvent_t;

#define ECU_SYSTEM_CLOCK_HZ       250000000U
#define ECU_SCHED_CLOCK_HZ        62500000U   // TIM5_CNT: 62.5 MHz, 16 ns/tick
#define ECU_SCHED_TICKS_PER_MS    62500U
// TICKS_PER_US = 62.5 (não inteiro) — usar macro ECU_SCHED_US_TO_TICKS() em vez deste define
#define ECU_SCHED_NS_PER_TICK     16U

extern volatile uint32_t g_late_event_count;
extern volatile uint32_t g_calibration_clamp_count;
extern volatile uint32_t g_cycle_schedule_drop_count;

void ECU_Hardware_Init(void);

// eoi_lead_deg: EOI targeting — ângulo (° BTDC de combustão) em que a
// injecção TERMINA. O início é calculado para trás (SOI = EOI − PW°),
// recuando automaticamente com PW grande. Clampado a [0, 359] em runtime.
void ecu_sched_commit_calibration(uint32_t advance_deg,
                                  uint32_t dwell_ticks,
                                  uint32_t inj_pw_ticks,
                                  uint32_t eoi_lead_deg);
void ecu_sched_set_advance_deg(uint32_t adv);
void ecu_sched_set_dwell_ticks(uint32_t dwell);
void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks);
void ecu_sched_set_eoi_lead_deg(uint32_t eoi_lead_deg);
void ecu_sched_set_presync_enable(uint8_t enable);
void ecu_sched_set_presync_inj_mode(uint8_t mode);
void ecu_sched_set_presync_inj_auto(uint8_t on);
uint8_t ecu_sched_presync_inj_auto(void);
void ecu_sched_set_presync_ign_mode(uint8_t mode);
void ecu_sched_reset_diagnostic_counters(void);

// Dwell watchdog — chamar do main loop (slot 2ms); compara TIM5_CNT.
// Se uma bobina ficou activa por > 1.4 × dwell_ticks sem evento SPARK,
// força a saída LOW imediatamente para proteger o módulo de ignição.
void ecu_sched_dwell_watchdog(void);
uint32_t ecu_sched_dwell_watchdog_count(void);

// Multi-spark (MS42 §2.2.3): sparks adicionais por ciclo a baixo RPM.
// count: número de sparks adicionais (0=desabilitado, máx 3).
// inter_dwell_ticks: tempo de dwell entre sparks consecutivos (ticks TIM5).
// atdc_limit_deg: o último spark adicional não pode ultrapassar este ângulo ATDC (default 18°).
void ecu_sched_set_mspark(uint8_t  count,
                          uint32_t inter_dwell_ticks,
                          uint32_t atdc_limit_deg);

// Per-cylinder injection inhibit (MS42 §2.2.5 — corte de cilindros).
// mask: bit 0 = cyl 0, bit 1 = cyl 1, ..., bit 3 = cyl 3.
// Quando o bit está activo, ECU_ACT_INJ_ON é suprimido no canal correspondente.
// ECU_ACT_INJ_OFF passa normalmente (fechar injetor já fechado é inócuo).
// Aplica-se tanto ao modo sequencial como presync.
void ecu_sched_set_inj_inhibit_mask(uint8_t mask);
uint8_t ecu_sched_get_inj_inhibit_mask(void);

// Per-cylinder ignition inhibit (rev limiter por faísca).
// mask: bit 0 = cyl 0 … bit 3 = cyl 3.
// Suprime ECU_ACT_DWELL_START → bobina não carrega → sem faísca nesse cilindro.
void ecu_sched_set_ign_inhibit_mask(uint8_t mask);
uint8_t ecu_sched_get_ign_inhibit_mask(void);
// IVC: page0/protocol compatibility only. EOI targeting removed IVC PW clamp;
// ecu_sched_ivc_clamp_count() remains 0. Stored angle is not used by builders.
void ecu_sched_set_ivc(uint8_t ivc_abdc_deg);
uint32_t ecu_sched_ivc_clamp_count(void);
// Contador do duty clamp: incrementado quando PW_deg excede 90% do ciclo
// (648° sequencial / 324° presync) e é clampado. >0 = fuel shortfall.
uint32_t ecu_sched_pw_duty_clamp_count(void);
void ecu_sched_fire_prime_pulse(uint32_t pw_us);

// Bench protocol: next commit_calibration applies PW then locks (override=1).
// Prefer this over poking g_inj_pw_override via raw symbol linkage.
void ecu_sched_bench_pw_lock_next_commit(void);
uint8_t ecu_sched_bench_pw_override_state(void);

// Teste de saídas em bancada: pulso único num canal individual (motor parado).
// cyl = 0-3 na ordem INJ1..INJ4 / IGN1..IGN4. pw_us clamp ≤30000; dwell_us
// clamp ≤10000 (o dwell-watchdog fica armado como backstop do SPARK).
void ecu_sched_test_pulse_inj(uint8_t cyl, uint32_t pw_us);
void ecu_sched_test_pulse_ign(uint8_t cyl, uint32_t dwell_us);
// Descarta eventos TIM5 pendentes e leva todos os INJ/IGN ao estado seguro.
void ecu_sched_test_all_outputs_safe(void);

void ecu_sched_evt_dispatch(void);  // called from TIM5 ISR on CC3IF

// Modo de ignição actual: 1 = sequencial (full sync + CMP confirmado),
// 0 = wasted-spark (presync). Reflecte g_knock_sequential. Usado pela
// observabilidade (status bit IGN_SEQUENTIAL) e pelos host tests.
uint8_t ecu_sched_is_sequential(void);
uint8_t ecu_sched_presync_inj_mode(void);

#if defined(EMS_HOST_TEST)
void ecu_sched_test_reset(void);
uint8_t ecu_sched_test_angle_table_size(void);
uint8_t ecu_sched_test_get_angle_event(uint8_t index,
                                       uint8_t *tooth,
                                       uint8_t *sub_frac,
                                       uint8_t *ch,
                                       uint8_t *action,
                                       uint8_t *phase);
void ecu_sched_test_set_advance_deg(uint32_t adv);
void ecu_sched_test_set_dwell_ticks(uint32_t dwell);
void ecu_sched_test_set_inj_pw_ticks(uint32_t pw_ticks);
void ecu_sched_test_set_eoi_lead_deg(uint32_t eoi_lead_deg);
uint32_t ecu_sched_test_get_advance_deg(void);
uint32_t ecu_sched_test_get_dwell_ticks(void);
uint32_t ecu_sched_test_get_inj_pw_ticks(void);
uint32_t ecu_sched_test_get_eoi_lead_deg(void);
uint32_t ecu_sched_test_get_calibration_clamp_count(void);
uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void);
uint32_t ecu_sched_test_get_late_event_count(void);
void ecu_sched_test_set_ivc(uint8_t ivc_abdc_deg);
uint32_t ecu_sched_test_get_ivc_clamp_count(void);
uint32_t ecu_sched_test_get_pw_duty_clamp_count(void);
void     ecu_sched_test_set_tim1_cnt(uint32_t cnt) noexcept;
uint32_t ecu_sched_test_get_tim1_ccr(uint8_t channel) noexcept;
void     ecu_sched_test_set_tim2_cnt(uint32_t cnt) noexcept;
void     ecu_sched_test_reset_ccr(void) noexcept;   // zero all TIM1/TIM2 CCR mocks
void     ecu_sched_test_set_mspark(uint8_t count, uint32_t inter_dwell_ticks, uint32_t atdc_limit_deg);
uint8_t  ecu_sched_test_get_mspark_count(void);
// TIM5 event-queue accessors
uint8_t  ecu_sched_test_get_evt_count(void) noexcept;
uint32_t ecu_sched_test_get_tim5_ccr3(void)  noexcept;
void     ecu_sched_test_set_tim5_cnt(uint32_t v) noexcept;
// Contadores de revoluções por modo — validam a transição presync↔sequencial.
uint32_t ecu_sched_test_get_presync_revs(void);
uint32_t ecu_sched_test_get_seq_revs(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
