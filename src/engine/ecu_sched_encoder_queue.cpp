/**
 * @file ecu_sched_encoder_queue.cpp
 * @brief Fila TIM2/CH3 (domínio de ângulo) + purge/clear.
 */
#include "engine/ecu_sched_encoder_priv.h"
#include "engine/enc_cyl_setpoints.h"
#include "engine/xtau_autocalib.h"
#include "engine/engine_config.h"
#include "engine/calibration.h"
#include "hal/out_pins.h"
#include "hal/critical_section.h"
#include "hal/board_pinout.h"
#include "drv/ckp.h"
#include "drv/crank_angle.h"
#include "drv/encoder_sync.h"
#include "engine/misfire_encoder.h"
#include "engine/spark_skip.h"
#include "drv/sensors.h"
#if !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

#include <stdint.h>
#if defined(EMS_HOST_TEST)
#include <cassert>
#endif

namespace si = ems::engine::sched_internal;
#if defined(EMS_HOST_TEST)
// Mock de TIM2 para a fila TIM2/CH3 — mesmo padrão do mock TIM5 em
// ecu_sched.cpp, registo próprio (nunca partilha estado com o mock TIM5,
// mesma disciplina de "duas filas separadas" desta unidade).
// TIM_SR_CC3IF / TIM_DIER_CC3IE são partilhados com o mock TIM5 e vêm de
// ecu_sched_internal.h (via ecu_sched_encoder_priv.h) — definição única.
static uint32_t ems_test_tim2_ccr3 = 0u;
static uint32_t ems_test_tim2_sr   = 0u;
static uint32_t ems_test_tim2_dier = 0u;
static uint32_t ems_test_tim2_cnt  = 0u;
#define TIM2_CCR3   ems_test_tim2_ccr3
#define TIM2_SR     ems_test_tim2_sr
#define TIM2_DIER   ems_test_tim2_dier
#define TIM2_CNT    ems_test_tim2_cnt
#endif
// ── Fila TIM2/CH3 — dispatcher em domínio de ângulo ─────────────────────
// SEPARADA da fila TIM5/CH3 de ecu_sched.cpp — nunca partilha array nem
// registo (ver aviso no topo do ficheiro e docs/dev/mt6835_encoder_fork.md,
// secção 6). Mesma forma (array ordenado por inserção, insertion sort
// wrap-safe com subtração de sinal, overflow policy que nunca larga
// silenciosamente um de-assert, CC3IE dinâmico por episódio
// fila-vazia↔não-vazia) — só a unidade muda: counts TIM2, não ticks TIM5.
// No despacho, enc_evt_execute_head() chama pin_transition() — os
// watchdogs de dwell/inj continuam TIM5/tempo (ecu_sched_pins.cpp).
//
// Piso de lead mínimo aplicado em ecu_sched_encoder_arm_channel() (via
// si::encoder::min_lead_counts(), plano secção 7). Mesmo sem piso, um alvo
// já passado no momento do arm cairia no caminho "late" do dispatch
// (processado inline, contado em g_enc_late_event_count) — nunca perdido; o
// piso é refinamento de reação mínima, não requisito de correção.


EncSchedEvent g_enc_evt_queue[ENC_EVT_QUEUE_SIZE];
volatile uint8_t g_enc_evt_count = 0U;

static volatile uint32_t g_enc_dbg_evt_overflow = 0U;
static volatile uint32_t g_enc_late_event_count = 0U;
volatile uint32_t g_enc_dbg_insert_count  = 0U;
volatile uint32_t g_enc_dbg_execute_count = 0U;
// TIM2 é contador de POSIÇÃO, não de tempo: programar CCR3 num alvo já
// passado (ou igual a CNT) e limpar CC3IF deixa o compare-match mudo até
// o wrap de 32 bits. Alvos ainda à frente geram match quando CNT chega;
// os já devidos vão pelo caminho "late" de ecu_sched_encoder_evt_dispatch.
static void enc_evt_rearm_or_dispatch(void) noexcept
{
    if (g_enc_evt_count == 0U) {
        TIM2_DIER &= ~TIM_DIER_CC3IE;
        return;
    }
    const uint32_t now = TIM2_CNT;
    const uint32_t ts = g_enc_evt_queue[0].timestamp;
    if ((int32_t)(ts - now) > 0) {
        TIM2_CCR3 = ts;
        TIM2_SR   = ~TIM_SR_CC3IF;
        TIM2_DIER |= TIM_DIER_CC3IE;
        return;
    }
    ecu_sched_encoder_evt_dispatch();
}

static uint8_t enc_evt_drop_one_assert(uint8_t prefer_channel) noexcept
{
    int8_t drop = -1;
    for (uint8_t i = 0U; i < g_enc_evt_count; ++i) {
        if (g_enc_evt_queue[i].high == 0U) { continue; }
        if (g_enc_evt_queue[i].channel == prefer_channel) { drop = (int8_t)i; break; }
        if (drop < 0) { drop = (int8_t)i; }
    }
    if (drop < 0) { return 0U; }
    for (uint8_t i = (uint8_t)drop; (uint8_t)(i + 1U) < g_enc_evt_count; ++i) {
        g_enc_evt_queue[i] = g_enc_evt_queue[i + 1U];
    }
    --g_enc_evt_count;
    return 1U;
}

void enc_evt_insert(uint32_t ts, uint8_t channel, uint8_t high) noexcept
{
    if (g_enc_evt_count >= ENC_EVT_QUEUE_SIZE) {
        ++g_enc_dbg_evt_overflow;
        if (high == 0U) {
            if (enc_evt_drop_one_assert(channel) == 0U) { return; }
        } else {
            return;  // prefer keeping de-asserts already queued
        }
    }
    uint8_t pos = g_enc_evt_count;
    for (uint8_t i = 0U; i < g_enc_evt_count; ++i) {
        if ((int32_t)(ts - g_enc_evt_queue[i].timestamp) < 0) { pos = i; break; }
    }
    for (uint8_t i = g_enc_evt_count; i > pos; --i) {
        g_enc_evt_queue[i] = g_enc_evt_queue[i - 1U];
    }
    g_enc_evt_queue[pos].timestamp = ts;
    g_enc_evt_queue[pos].channel = channel;
    g_enc_evt_queue[pos].high = high;
    ++g_enc_evt_count;
    ++g_enc_dbg_insert_count;

    if (pos == 0U) {
        enc_evt_rearm_or_dispatch();
    }
}

static inline void enc_evt_execute_head(void) noexcept
{
    const EncSchedEvent& e = g_enc_evt_queue[0];
    ems::hal::out_pin_write(e.channel, e.high);
    const uint8_t idx = (e.channel < 8U) ? ems::hal::kOutChToPinIdx[e.channel] : 0xFFU;
    if (idx != 0xFFU) {
        pin_transition(idx, e.high);  // watchdogs continuam sempre TIM5/tempo
    }
    ++g_enc_dbg_execute_count;
    --g_enc_evt_count;
    for (uint8_t i = 0U; i < g_enc_evt_count; ++i) {
        g_enc_evt_queue[i] = g_enc_evt_queue[i + 1U];
    }
}

static constexpr uint32_t kDispatchCcrMarginUs = 3U;
static constexpr uint32_t kDispatchCcrMarginFallbackCounts = 16U;

static uint32_t dispatch_ccr_margin_counts(void) noexcept;
void ecu_sched_encoder_evt_dispatch(void) noexcept
{
    const uint32_t now = TIM2_CNT;
    const uint32_t ccr_margin = dispatch_ccr_margin_counts();
    while (g_enc_evt_count > 0U) {
        const EncSchedEvent& e = g_enc_evt_queue[0];
        if ((int32_t)(e.timestamp - now) > 0) { break; }
        enc_evt_execute_head();
    }
    while (g_enc_evt_count > 0U) {
        const uint32_t next_ts = g_enc_evt_queue[0].timestamp;
        if ((int32_t)(next_ts - TIM2_CNT) > (int32_t)ccr_margin) {
            TIM2_CCR3 = next_ts;
            TIM2_SR   = ~TIM_SR_CC3IF;
            return;
        }
        ++g_enc_late_event_count;
        enc_evt_execute_head();
    }
    TIM2_DIER &= ~TIM_DIER_CC3IE;
}

void arm_channel_with_lead(uint8_t ch, uint32_t target_counts,
                                  uint8_t action, uint32_t min_lead) noexcept
{
    // Mesma razão do arm_channel() de ecu_sched.cpp: inserir na fila +
    // tocar CCR3/DIER não pode intercalar com o dispatch ISR (TIM2 CH3).
    ems::hal::CriticalSectionGuard guard;
    const uint8_t high =
        ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U;

    if (action == ECU_ACT_DWELL_START) {
        si::maybe_knock_on_dwell_start(ch);
    }

    // Piso mínimo de lead — caller passa min_lead já hoistado no rebuild
    // (evita recalcular ω→span em cada insert).
    const uint32_t now = TIM2_CNT;
    const int32_t lead = (int32_t)(target_counts - now);
    if (lead < (int32_t)min_lead) {
        enc_evt_insert(now + min_lead, ch, high);
    } else {
        enc_evt_insert(target_counts, ch, high);
    }
}

void ecu_sched_encoder_arm_channel(uint8_t ch, uint32_t target_counts,
                                   uint8_t action) noexcept
{
    arm_channel_with_lead(ch, target_counts, action,
                          si::encoder::min_lead_counts());
}

#if defined(EMS_HOST_TEST)
void ecu_sched_encoder_queue_test_reset(void) noexcept
{
    g_enc_evt_count = 0U;
    g_enc_dbg_evt_overflow = 0U;
    g_enc_late_event_count = 0U;
    g_enc_dbg_insert_count = 0U;
    g_enc_dbg_execute_count = 0U;
    ems_test_tim2_ccr3 = 0U; ems_test_tim2_sr = 0U; ems_test_tim2_dier = 0U; ems_test_tim2_cnt = 0U;
}
void ecu_sched_encoder_test_set_tim2_cnt(uint32_t v) noexcept { ems_test_tim2_cnt = v; }
uint8_t ecu_sched_encoder_test_get_evt_count(void) noexcept { return g_enc_evt_count; }
uint32_t ecu_sched_encoder_test_get_ccr3(void) noexcept { return ems_test_tim2_ccr3; }
uint8_t ecu_sched_encoder_test_get_evt(uint8_t index, uint32_t *ts,
                                       uint8_t *channel, uint8_t *high) noexcept
{
    if (index >= g_enc_evt_count) { return 0U; }
    if (ts != nullptr) { *ts = g_enc_evt_queue[index].timestamp; }
    if (channel != nullptr) { *channel = g_enc_evt_queue[index].channel; }
    if (high != nullptr) { *high = g_enc_evt_queue[index].high; }
    return 1U;
}
uint32_t ecu_sched_encoder_test_get_evt_overflow(void) noexcept
{
    return ecu_sched_encoder_evt_overflow();
}
uint32_t ecu_sched_encoder_test_get_late_event_count(void) noexcept
{
    return ecu_sched_encoder_late_event_count();
}
uint32_t ecu_sched_encoder_test_get_dier(void) noexcept { return ems_test_tim2_dier; }
#endif

uint32_t ecu_sched_encoder_late_event_count(void) noexcept
{
    return g_enc_late_event_count;
}
uint32_t ecu_sched_encoder_evt_overflow(void) noexcept
{
    return g_enc_dbg_evt_overflow;
}

namespace ems::engine::sched_internal {

// Varredura da fila TIM2/CH3 para purge_events_for_cyl_mask()/
// clear_all_events_and_drive_safe_outputs() (ecu_sched.cpp) — ver aviso em
// ecu_sched_internal.h. Mesma lógica de filtro que a varredura TIM5 já usa,
// aplicada ao array desta fila.
void encoder_purge_cyl_mask(uint8_t mask, uint8_t is_ign) noexcept
{
    if (mask == 0U) { return; }
    uint8_t w = 0U;
    for (uint8_t r = 0U; r < g_enc_evt_count; ++r) {
        const uint8_t ch = g_enc_evt_queue[r].channel;
        const uint8_t bit = (ch < 8U)
            ? (is_ign != 0U ? k_ign_ch_to_bit[ch] : k_inj_ch_to_bit[ch])
            : 0U;
        if (bit != 0U && (mask & bit) != 0U) { continue; }  // drop
        if (w != r) { g_enc_evt_queue[w] = g_enc_evt_queue[r]; }
        ++w;
    }
    g_enc_evt_count = w;
    enc_evt_rearm_or_dispatch();
}

void encoder_clear_all(void) noexcept
{
    g_enc_evt_count = 0U;
    TIM2_DIER &= ~TIM_DIER_CC3IE;
}

}  // namespace ems::engine::sched_internal

// Margem de latência do dispatcher TIM2/CH3: kDispatchCcrMarginUs convertidos
// em counts via si::encoder::duration_ticks_to_span_counts() — mesmo helper
// que min_lead_counts() já usa, em vez de reimplementar a fórmula ω→span
// (era o que este ficheiro fazia antes desta limpeza, uma segunda cópia da
// mesma aritmética). ω inválido: fallback kDispatchCcrMarginFallbackCounts
// counts (duration_ticks_to_span_counts() por si só devolveria 0 nesse caso,
// que não serve aqui — 0 counts de margem arriscaria reprogramar CCR3 em
// cima de um alvo já quase alcançado).
static uint32_t dispatch_ccr_margin_counts(void) noexcept
{
    if (ecu_sched_encoder_omega_valid() == 0U ||
        ecu_sched_encoder_omega_x65536() <= 0) {
        return kDispatchCcrMarginFallbackCounts;
    }
    const uint32_t margin = si::encoder::duration_ticks_to_span_counts(
        ECU_SCHED_US_TO_TICKS_INTERNAL(kDispatchCcrMarginUs));
    return (margin < 1U) ? 1U : margin;
}
