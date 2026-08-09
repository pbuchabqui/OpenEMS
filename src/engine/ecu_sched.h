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

// Sequential: 4 cyl × (DWELL+SPARK+INJ_ON+INJ_OFF) = 16
// + multi-spark: max 3 extra × 2 events × 4 cyl = 24 → 40. Presync wasted can
// reach ~40 with simultaneous inj + multi-spark. Keep margin.
#define ECU_ANGLE_TABLE_SIZE  48U

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

/**
 * @brief Força INJ/IGN a saída push-pull LOW o mais cedo possível no boot.
 *
 * PA15 reset = JTDI com pull-up interno → HIGH activa injectores activos-high.
 * PC10/PC11 flutuam e podem subir por pull-ups externos no TLE/placa.
 * Chamar logo após clocks GPIO (antes de delays USB / DFU).
 */
void ecu_sched_outputs_safe_early(void);

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

// Injector open watchdog — same 2 ms slot. If an injector pin stays HIGH
// beyond 1.2× current PW (hard cap 36 ms; covers prime ≤30 ms), force OFF
// and purge pending events for that cylinder (lost INJ_OFF backstop).
void ecu_sched_inj_watchdog(void);
uint32_t ecu_sched_inj_watchdog_count(void);

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

// Per-cylinder ignition inhibit (limp spark-cut; production rev-limit is fuel-only).
// mask: bit 0 = cyl 0 … bit 3 = cyl 3.
// Suprime ECU_ACT_DWELL_START, purga eventos IGN pendentes do canal e força pin LOW
// (não deixar bobina carregada a meio do dwell).
void ecu_sched_set_ign_inhibit_mask(uint8_t mask);
uint8_t ecu_sched_get_ign_inhibit_mask(void);
// Contador do duty clamp: incrementado quando PW_deg excede 90% do ciclo
// (648° sequencial / 324° presync) e é clampado. >0 = fuel shortfall.
uint32_t ecu_sched_pw_duty_clamp_count(void);
void ecu_sched_fire_prime_pulse(uint32_t pw_us);

// Bench protocol: next commit_calibration applies PW then locks (override=1).
// Prefer this over poking g_inj_pw_override via raw symbol linkage.
void ecu_sched_bench_pw_lock_next_commit(void);
uint8_t ecu_sched_bench_pw_override_state(void);

// ── Protocol / host observability (main-loop / UART only — not ISR hot path) ──

// Last 8 angle-trace samples from the dispatch ring (INJ1/IGN1 edges).
// Wire format for 'G' stays: gap_ts + ring_idx + 8×{ts, high, channel}.
typedef struct {
    uint32_t ts;
    uint8_t  high;
    uint8_t  channel;
} EcuSchedTsSample;

void ecu_sched_get_angle_trace(uint32_t *gap_ts,
                               uint8_t *ring_idx,
                               EcuSchedTsSample out_last8[8]);

// Pin transition counters for all 8 channels: high/low/seq_error interleaved
// as 24×u32 for protocol 'V' (same layout as before).
void ecu_sched_get_pin_counts_u32x24(uint32_t out[24]);

// Scheduler-owned fields used by protocol 'D' (order matches historical diag[]
// slots for these counters — callers still interleave CKP/fuel fields).
typedef struct {
    uint32_t late_event_count;
    uint32_t cycle_schedule_drop_count;
    uint32_t inj1_arm;
    uint32_t seq_calls;
    uint32_t evt_overflow;
    uint32_t clear_all_count;       // g_dbg_clear_all_count
    uint32_t presync_count;
    uint32_t dwell_watchdog_count;
    uint32_t phase_skip;
    uint32_t phase_fire;
    uint32_t evt_inserted;
    uint32_t evt_dispatched;
    uint32_t diag_presync_revs;
    uint32_t diag_seq_revs;
    uint32_t diag_clear_all_count;
} EcuSchedDiagSnapshot;

void ecu_sched_get_diag_snapshot(EcuSchedDiagSnapshot *out);

// Teste de saídas em bancada: pulso único num canal individual (motor parado).
// cyl = 0-3 na ordem INJ1..INJ4 / IGN1..IGN4. pw_us clamp ≤30000; dwell_us
// clamp ≤10000 (o dwell-watchdog fica armado como backstop do SPARK).
void ecu_sched_test_pulse_inj(uint8_t cyl, uint32_t pw_us);
void ecu_sched_test_pulse_ign(uint8_t cyl, uint32_t dwell_us);
// Descarta eventos TIM5 pendentes e leva todos os INJ/IGN ao estado seguro.
void ecu_sched_test_all_outputs_safe(void);

void ecu_sched_evt_dispatch(void);  // called from TIM5 ISR on CC3IF

// ── MT6835/TIM2 encoder — domínio de ângulo (EMS_MT6835_ENCODER apenas) ──────
// Ver docs/dev/mt6835_encoder_fork.md ("Dispatcher em domínio de ângulo") no
// fork feat/mt6835-encoder. Sem roda dentada, não há evento de dente para
// estimar RPM — ω vem de ΔTIM2_CNT/ΔTIM5_CNT amostrado pelo heartbeat
// TIM2_CH4 (hal/stm32h562/timer.cpp), não de outro contexto (mantém uma
// única cadeia de amostras consecutivas — chamar de dois sítios corrompe a
// estimativa).
void ecu_sched_encoder_omega_sample(uint32_t tim2_now, uint32_t tim5_now) noexcept;
// Estimativa corrente: contagens de TIM2 por tick de TIM5, fixed-point
// ×65536 (com sinal — negativo em rotação reversa/kick-back de cranking, o
// modo encoder de hardware decrementa TIM2_CNT nativamente). ×65536, não
// ×256: ω real (~0,0009 a idle/cranking, ~0,039 a redline, counts/tick)
// trunca para 0 em ×256 já a 200 rpm — ×65536 mantém resolução útil em
// toda a gama (~57 a 200 rpm, ~2577 a 9000 rpm). 0 se ainda não houver
// amostra válida — checar ecu_sched_encoder_omega_valid() antes de usar
// para conversões (dwell/PW em counts, piso de lead).
int32_t ecu_sched_encoder_omega_x65536(void) noexcept;
uint8_t ecu_sched_encoder_omega_valid(void) noexcept;

// Rastreador de fase: TIM2_CNT só dá posição mod 360° (1 volta de cambota);
// o motor tem ciclo de 720° (ECU_PHASE_A/B, ver acima). O CMP (sensor Hall
// inalterado, tim3_cmp_ic_init()) desambigua qual metade — mas ESTE módulo
// não decide sozinho a que fase corresponde um flanco do CMP (é uma
// constante de calibração de hardware — onde o sensor está montado — ainda
// não determinada; bloqueia bancada, não este dispatcher). Quem chama
// ecu_sched_encoder_phase_set_anchor() (o heartbeat CH4, quando existir)
// é responsável por saber essa fase; este módulo só mantém o anchor
// absoluto de 32 bits e conta voltas completas (16384 counts) desde o
// anchor para responder "que fase é agora" em qualquer instante — nunca por
// toggle, sempre recalculado a partir do anchor absoluto mais recente.
void ecu_sched_encoder_phase_set_anchor(uint32_t tim2_raw_at_cmp_edge,
                                        uint8_t phase) noexcept;
// ECU_PHASE_A ou ECU_PHASE_B — chamar só depois de ecu_sched_encoder_phase_valid().
uint8_t ecu_sched_encoder_phase_at(uint32_t tim2_raw_now) noexcept;
uint8_t ecu_sched_encoder_phase_valid(void) noexcept;

// Fila TIM2/CH3 — SEPARADA da fila TIM5/CH3 acima (ecu_sched_evt_dispatch),
// nunca partilha array nem registo. A fila TIM5 continua a servir só
// fire_prime_pulse()/test_pulse_inj()/test_pulse_ign() (motor parado,
// sempre por tempo) em qualquer um dos dois builds — ver
// docs/dev/mt6835_encoder_fork.md, secção 6, para o porquê de duas filas
// em vez de uma parametrizada.
//
// target_counts: alvo em counts CRUS de 32 bits do TIM2 (não mascarado a
// 14 bits) — mesmo domínio que TIM2_CNT já vive, wrap tratado por subtração
// com sinal, igual ao TIM5 hoje. Ainda sem piso de lead mínimo (pendente,
// ver plano — não bloqueia correção: um alvo já passado é processado
// inline como "late", nunca perdido).
void ecu_sched_encoder_arm_channel(uint8_t ch, uint32_t target_counts,
                                   uint8_t action) noexcept;
void ecu_sched_encoder_evt_dispatch(void) noexcept;  // called from TIM2 ISR on CC3IF

// Heartbeat TIM2_CH4 — chamado 1×/volta de cambota (16384 counts) pelo
// TIM2_IRQHandler em CC4IF. tim2_now/tim5_now: já lidos pelo HAL no
// instante do heartbeat, alimentam o estimador de ω
// (ecu_sched_encoder_omega_sample(), chamado internamente). cmp_angle/
// cmp_edge_count: leitura mais recente de cmp_angle_snapshot()/
// cmp_edge_count() (hal/timer.h) — usada para detectar um novo flanco do
// CMP desde o último tick (delta de cmp_edge_count).
//
// ⚠️ Ainda não faz o recompute barato de dwell/PW nem o bank-toggle do
// presync nem a verificação de deriva do CMP (próxima tarefa do plano,
// docs/dev/mt6835_encoder_fork.md — "Conversão graus→counts + recompute
// partilhado"): a fase que um flanco do CMP representa é uma constante de
// calibração de hardware ainda não medida em bancada
// (ecu_sched_encoder_phase_set_anchor() precisa dela), não algo que este
// heartbeat possa inventar. Por agora só alimenta ω, que não depende dessa
// constante.
void ecu_sched_encoder_heartbeat_tick(uint32_t tim2_now, uint32_t tim5_now,
                                      uint32_t cmp_angle,
                                      uint32_t cmp_edge_count) noexcept;

// Modo de ignição actual: 1 = sequencial (full sync + CMP confirmado),
// 0 = wasted-spark (presync). Reflecte g_knock_sequential. Usado pela
// observabilidade (status bit IGN_SEQUENTIAL) e pelos host tests.
uint8_t ecu_sched_is_sequential(void);
uint8_t ecu_sched_presync_inj_mode(void);

#if defined(EMS_HOST_TEST)
void ecu_sched_test_reset(void);
// Zera o estimador de ω do encoder — chamado por ecu_sched_test_reset()
// (ecu_sched_angle_encoder.cpp), evita estado a vazar entre testes.
void ecu_sched_encoder_omega_test_reset(void) noexcept;
// Idem para o rastreador de fase.
void ecu_sched_encoder_phase_test_reset(void) noexcept;
// Idem para a fila TIM2/CH3 (zera a fila + o mock de TIM2_CNT/CCR3/DIER).
void ecu_sched_encoder_queue_test_reset(void) noexcept;
// Idem para o heartbeat (delta de cmp_edge_count entre ticks).
void ecu_sched_encoder_heartbeat_test_reset(void) noexcept;
// Mock de TIM2_CNT para os testes da fila TIM2/CH3 (nome sem colisão com os
// aliases legados ecu_sched_test_set_tim2_cnt/get_tim1_ccr — esses mexem em
// ems_test_tim5_cnt por baixo, ver "TIM1 placeholders" acima; não são o
// mesmo mock que este).
void ecu_sched_encoder_test_set_tim2_cnt(uint32_t v) noexcept;
uint8_t ecu_sched_encoder_test_get_evt_count(void) noexcept;
uint32_t ecu_sched_encoder_test_get_ccr3(void) noexcept;
uint8_t ecu_sched_encoder_test_get_evt(uint8_t index,
                                       uint32_t *ts,
                                       uint8_t *channel,
                                       uint8_t *high) noexcept;
uint32_t ecu_sched_encoder_test_get_evt_overflow(void) noexcept;
uint32_t ecu_sched_encoder_test_get_late_event_count(void) noexcept;
uint32_t ecu_sched_encoder_test_get_dier(void) noexcept;  // CC3IE bit dinâmico
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
// Queue peeks for golden timestamp / channel identity tests (index 0 = earliest).
uint8_t  ecu_sched_test_get_evt(uint8_t index,
                                uint32_t *ts,
                                uint8_t *channel,
                                uint8_t *high) noexcept;
// Contadores de revoluções por modo — validam a transição presync↔sequencial.
uint32_t ecu_sched_test_get_presync_revs(void);
uint32_t ecu_sched_test_get_seq_revs(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
