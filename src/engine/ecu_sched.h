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
#define ECU_SCHED_CLOCK_HZ        10000000U
#define ECU_SCHED_TICKS_PER_MS    10000U
#define ECU_SCHED_TICKS_PER_US    10U
#define ECU_SCHED_NS_PER_TICK     100U

extern volatile uint32_t g_late_event_count;
extern volatile uint32_t g_calibration_clamp_count;
extern volatile uint32_t g_cycle_schedule_drop_count;

void ECU_Hardware_Init(void);

void ecu_sched_commit_calibration(uint32_t advance_deg,
                                  uint32_t dwell_ticks,
                                  uint32_t inj_pw_ticks,
                                  uint32_t soi_lead_deg);
void ecu_sched_set_advance_deg(uint32_t adv);
void ecu_sched_set_dwell_ticks(uint32_t dwell);
void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks);
void ecu_sched_set_soi_lead_deg(uint32_t soi_lead_deg);
void ecu_sched_set_presync_enable(uint8_t enable);
void ecu_sched_set_presync_inj_mode(uint8_t mode);
void ecu_sched_set_presync_ign_mode(uint8_t mode);
void ecu_sched_reset_diagnostic_counters(void);

// Dwell watchdog — chamar do main loop (slot 2ms) com TIM2_CNT.
// Se uma bobina ficou activa por > 1.4 × dwell_ticks sem evento SPARK,
// força a saída LOW imediatamente para proteger o módulo de ignição.
void ecu_sched_dwell_watchdog(void);
uint32_t ecu_sched_dwell_watchdog_count(void);

// Multi-spark (MS42 §2.2.3): sparks adicionais por ciclo a baixo RPM.
// count: número de sparks adicionais (0=desabilitado, máx 3).
// inter_dwell_ticks: tempo de dwell entre sparks consecutivos (em ticks do TIM2/TIM8).
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
void ecu_sched_set_ivc(uint8_t ivc_abdc_deg);
uint32_t ecu_sched_ivc_clamp_count(void);
void ecu_sched_fire_prime_pulse(uint32_t pw_us);

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
void ecu_sched_test_set_soi_lead_deg(uint32_t soi_lead_deg);
uint32_t ecu_sched_test_get_advance_deg(void);
uint32_t ecu_sched_test_get_dwell_ticks(void);
uint32_t ecu_sched_test_get_inj_pw_ticks(void);
uint32_t ecu_sched_test_get_soi_lead_deg(void);
uint32_t ecu_sched_test_get_calibration_clamp_count(void);
uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void);
uint32_t ecu_sched_test_get_late_event_count(void);
void ecu_sched_test_set_ivc(uint8_t ivc_abdc_deg);
uint32_t ecu_sched_test_get_ivc_clamp_count(void);
void     ecu_sched_test_set_tim8_cnt(uint32_t cnt) noexcept;
uint32_t ecu_sched_test_get_tim8_ccr(uint8_t channel) noexcept;
void     ecu_sched_test_set_tim2_cnt(uint32_t cnt) noexcept;
void     ecu_sched_test_reset_ccr(void) noexcept;   // zero all TIM8/TIM2 CCR mocks
void     ecu_sched_test_set_mspark(uint8_t count, uint32_t inter_dwell_ticks, uint32_t atdc_limit_deg);
uint8_t  ecu_sched_test_get_mspark_count(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
