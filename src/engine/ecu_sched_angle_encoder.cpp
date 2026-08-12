/**
 * @file ecu_sched_angle_encoder.cpp
 * @brief Dispatcher em domínio de ângulo (MT6835/TIM2), paralelo a
 *        ecu_sched_angle.cpp — que fica intocado (caminho Hall/roda-dentada
 *        continua a compilar e a funcionar sem alteração nenhuma).
 *
 * Só existe (e só é chamado) quando EMS_MT6835_ENCODER=1
 * (hal/board_pinout.h). Com a flag em 0 (default de produção), este
 * ficheiro compila mas fica inerte — nada o chama.
 *
 * Desenho completo: docs/dev/mt6835_encoder_fork.md (secção "Dispatcher em
 * domínio de ângulo") e o plano que o originou. Resumo do porquê de um
 * ficheiro novo em vez de estender ecu_sched.cpp: a fila TIM5/CH3 existente
 * em ecu_sched.cpp serve ecu_sched_on_tooth_hook() (inerte por construção em
 * modo encoder — CC1IE nunca é ligado, ver tim5_freerun_init() vs.
 * tim5_ic_init()) E ecu_sched_fire_prime_pulse()/test_pulse_inj()/
 * test_pulse_ign() (motor parado, sempre por tempo — estes continuam a usar
 * TIM5 em qualquer um dos dois builds). Uma fila em contagens de TIM2 não
 * pode partilhar esse mecanismo: um alvo em ângulo é inalcançável com o
 * motor parado, um alvo em tempo é sempre alcançável — não são a mesma
 * unidade. Por isso esta fila é nova e separada, não uma variante
 * parametrizada da existente.
 *
 * Estado actual (ver docs/dev/mt6835_encoder_fork.md):
 *   - Fila TIM2/CH3 própria (evt_insert/dispatch, unidade counts).
 *   - Fase A/B: anchor absoluto + floor_div_16384; CMP via
 *     ecu_sched_encoder_phase_set_anchor() quando calibrado.
 *   - Estimador de ω: ΔTIM2_CNT/ΔTIM5_CNT (signed), fixed-point ×65536.
 *   - Heartbeat: sub-tick (misfire + try_arm sequencial) + tick pesado
 *     1×/volta (ω, CMP, recompute_presync ou try_arm, publish snapshot).
 *   - Construtor sequencial: armamento tardio por cilindro quando o alvo
 *     (dwell ou inj_on) entra numa janela ≤60° (kSeqArmWindowCounts); chamado
 *     a cada sub-tick TIM2_CH4. Presync mantém wasted 1×/volta.
 *   - Presync: wasted-spark em pares 0↔3 / 2↔1 a 180° + multi-spark por par.
 */

#include "engine/ecu_sched_internal.h"
#include "engine/enc_cyl_setpoints.h"
#include "engine/engine_config.h"
#include "engine/calibration.h"
#include "hal/out_pins.h"
#include "hal/critical_section.h"
#include "hal/board_pinout.h"
#include "drv/ckp.h"
#include "drv/encoder_sync.h"
#include "engine/misfire_encoder.h"
#include "drv/sensors.h"
#if !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

#include <stdint.h>
#if defined(EMS_HOST_TEST)
#include <cassert>
#endif

namespace si = ems::engine::sched_internal;

namespace ems::engine::sched_internal::encoder {
// Definida mais abaixo neste ficheiro ("Conversão graus→counts") — forward
// declare aqui porque ecu_sched_encoder_heartbeat_tick() (mais acima no
// ficheiro que a definição) precisa de a chamar.
void recompute_presync(uint32_t now_raw) noexcept;
void try_arm_sequential_due(uint32_t now_raw) noexcept;
void refresh_pending_omega_spans(uint32_t now_raw) noexcept;
void clear_cyl_arm_latches(void) noexcept;
// Definida mais abaixo ("Conversão graus→counts", junto de
// duration_ticks_to_span_counts()) — forward declare porque
// ecu_sched_encoder_arm_channel() (mais acima) precisa de a chamar.
uint32_t min_lead_counts(void) noexcept;
}  // namespace ems::engine::sched_internal::encoder

#if defined(EMS_HOST_TEST)
// Mock de TIM2 para a fila TIM2/CH3 — mesmo padrão do mock TIM5 em
// ecu_sched.cpp, registo próprio (nunca partilha estado com o mock TIM5,
// mesma disciplina de "duas filas separadas" desta unidade).
#ifndef TIM_SR_CC3IF
#define TIM_SR_CC3IF 0x8U
#endif
#ifndef TIM_DIER_CC3IE
#define TIM_DIER_CC3IE (1U << 3)
#endif
static uint32_t ems_test_tim2_ccr3 = 0u;
static uint32_t ems_test_tim2_sr   = 0u;
static uint32_t ems_test_tim2_dier = 0u;
static uint32_t ems_test_tim2_cnt  = 0u;
#define TIM2_CCR3   ems_test_tim2_ccr3
#define TIM2_SR     ems_test_tim2_sr
#define TIM2_DIER   ems_test_tim2_dier
#define TIM2_CNT    ems_test_tim2_cnt
#endif

// ── Estimador de ω (ΔTIM2_CNT/ΔTIM5_CNT) ─────────────────────────────────
// Sem roda dentada, não há evento de dente para estimar RPM — ω vem de duas
// amostras consecutivas de (TIM2_CNT, TIM5_CNT), tiradas pelo heavy tick
// do heartbeat TIM2_CH4 (1×/volta via subtick). Pura aritmética, sem
// acesso a registo —
// o chamador já leu os valores; por isso compila e testa-se identicamente
// em host-test e alvo real, sem #ifdef nenhum aqui.
//
// Delta com sinal em TIM2 é obrigatório: o modo encoder de hardware
// decrementa TIM2_CNT nativamente em rotação reversa (kick-back de
// compressão no cranking) — um ω negativo transitório é dado real, não
// erro. Delta de TIM5 (tempo) assume-se positivo dentro do intervalo entre
// dois ticks do heartbeat (muito menor que os ~68,7 s de wrap do TIM5 a
// qualquer RPM realista); um delta ≤0 (relógio não avançou, ou primeira
// amostra) descarta a atualização em vez de dividir por zero ou inverter o
// sinal.
//
// Fixed-point ×65536 (contagens de TIM2 por tick de TIM5) — NÃO ×256.
// ω real cabe entre ~0,00087 counts/tick (200 rpm) e ~0,039 counts/tick
// (9000 rpm): em ×256 isso trunca para 0 a 200 rpm (0,224 → 0 em inteiro,
// cranking leria "sem rotação") e dá só ~10 valores discretos distintos
// até ao redline — resolução insuficiente em toda a gama, não só no
// extremo. ×65536 dá ~57 (200 rpm) a ~2577 (9000 rpm), sem truncar a zero
// em nenhum ponto do range operacional. Nome inclui a escala (não
// "omega_q" genérico) de propósito: um nome estável sobre um valor
// re-escalado é exatamente a armadilha que os aliases legados de teste
// TIM1/TIM2 (ecu_sched.cpp) já demonstraram neste projeto.

static volatile uint32_t g_omega_prev_tim2  = 0U;
static volatile uint32_t g_omega_prev_tim5  = 0U;
static volatile uint8_t  g_omega_have_prev  = 0U;
static volatile int32_t  g_omega_x65536     = 0;
static volatile uint8_t  g_omega_valid      = 0U;

void ecu_sched_encoder_omega_sample(uint32_t tim2_now, uint32_t tim5_now) noexcept
{
    if (g_omega_have_prev != 0U) {
        const int32_t d_tim5 = (int32_t)(tim5_now - g_omega_prev_tim5);
        if (d_tim5 > 0) {
            const int32_t d_tim2 = (int32_t)(tim2_now - g_omega_prev_tim2);
            g_omega_x65536 = (int32_t)(((int64_t)d_tim2 * 65536) / (int64_t)d_tim5);
            g_omega_valid = 1U;
        }
        // d_tim5 <= 0: relógio não avançou (ou amostra fora de ordem) —
        // mantém a última estimativa válida, não atualiza.
    }
    g_omega_prev_tim2 = tim2_now;
    g_omega_prev_tim5 = tim5_now;
    g_omega_have_prev = 1U;
}

int32_t ecu_sched_encoder_omega_x65536(void) noexcept { return g_omega_x65536; }
uint8_t ecu_sched_encoder_omega_valid(void) noexcept { return g_omega_valid; }

#if defined(EMS_HOST_TEST)
// Chamado por ecu_sched_test_reset() (ecu_sched.cpp) — evita estado do
// estimador vazar entre casos de teste no mesmo binário.
void ecu_sched_encoder_omega_test_reset(void) noexcept
{
    g_omega_prev_tim2 = 0U;
    g_omega_prev_tim5 = 0U;
    g_omega_have_prev = 0U;
    g_omega_x65536 = 0;
    g_omega_valid = 0U;
}
#endif

// ── Rastreador de fase (anchor CMP → ECU_PHASE_A/B) ─────────────────────
// TIM2_CNT só dá posição mod 360° (1 volta de cambota); o motor tem ciclo
// de 720°. O CMP (sensor Hall inalterado, tim3_cmp_ic_init()) desambigua
// qual metade — mas ESTE módulo não decide a que fase corresponde um
// flanco do CMP (constante de calibração de hardware, ainda por medir em
// bancada); recebe a fase já resolvida via
// ecu_sched_encoder_phase_set_anchor() e só responde "que fase é agora"
// contando voltas completas (16384 counts) desde o anchor absoluto mais
// recente — nunca por toggle incremental, sempre recalculado do anchor.
//
// Pura aritmética, sem acesso a registo — mesmo raciocínio de
// host-testabilidade do estimador de ω acima.

static volatile uint32_t g_phase_anchor_raw   = 0U;
static volatile uint8_t  g_phase_anchor_value = ECU_PHASE_A;
static volatile uint8_t  g_phase_valid        = 0U;

// Floor division por 16384 (2^14) — só a paridade do quociente interessa
// (nº de voltas completas desde o anchor, par/ímpar decide a fase), mas a
// paridade só está correta se a divisão arredondar para -∞, não para zero.
// Ex.: delta=-1 (1 count antes do anchor) tem de cair na volta anterior
// (quociente -1, ímpar/fase invertida) — divisão truncada daria 0 (par),
// errado: fisicamente esse count já pertence à revolução anterior.
static inline int32_t floor_div_16384(int32_t delta) noexcept
{
    int32_t q = delta / 16384;
    if ((delta % 16384 != 0) && (delta < 0)) { --q; }
    return q;
}

void ecu_sched_encoder_phase_set_anchor(uint32_t tim2_raw_at_cmp_edge,
                                        uint8_t phase) noexcept
{
    g_phase_anchor_raw = tim2_raw_at_cmp_edge;
    g_phase_anchor_value = phase;
    g_phase_valid = 1U;
}

uint8_t ecu_sched_encoder_phase_at(uint32_t tim2_raw_now) noexcept
{
    const int32_t delta = (int32_t)(tim2_raw_now - g_phase_anchor_raw);
    const int32_t revs = floor_div_16384(delta);
    const uint8_t flipped = (uint8_t)((uint32_t)revs & 1U);
    if (flipped == 0U) { return g_phase_anchor_value; }
    return (g_phase_anchor_value == ECU_PHASE_A) ? ECU_PHASE_B : ECU_PHASE_A;
}

uint8_t ecu_sched_encoder_phase_valid(void) noexcept { return g_phase_valid; }

// Único caminho de produção que limpa g_phase_valid — até esta função existir
// só havia SET (phase_set_anchor) e um clear host-test-only. Sem isto, um
// fallback de staleness que publicasse HALF_SYNC no CkpSnapshot deixaria
// phase_valid() preso em 1 — o dispatcher branca em phase_valid(), não em
// snap.state. Chamador: ecu_sched_encoder_heartbeat_tick() após
// staleness_exceeded() (ver docs/dev/mt6835_encoder_fork.md).
void ecu_sched_encoder_phase_invalidate(void) noexcept { g_phase_valid = 0U; }

#if defined(EMS_HOST_TEST)
void ecu_sched_encoder_phase_test_reset(void) noexcept
{
    g_phase_anchor_raw = 0U;
    g_phase_anchor_value = ECU_PHASE_A;
    g_phase_valid = 0U;
}
#endif

// ── Fila TIM2/CH3 — dispatcher em domínio de ângulo ─────────────────────
// SEPARADA da fila TIM5/CH3 de ecu_sched.cpp — nunca partilha array nem
// registo (ver aviso no topo do ficheiro e docs/dev/mt6835_encoder_fork.md,
// secção 6). Mesma forma (array ordenado por inserção, insertion sort
// wrap-safe com subtração de sinal, overflow policy que nunca larga
// silenciosamente um de-assert, CC3IE dinâmico por episódio
// fila-vazia↔não-vazia) — só a unidade muda: counts TIM2, não ticks TIM5.
//
// Piso de lead mínimo aplicado em ecu_sched_encoder_arm_channel() (via
// si::encoder::min_lead_counts(), plano secção 7). Mesmo sem piso, um alvo
// já passado no momento do arm cairia no caminho "late" do dispatch
// (processado inline, contado em g_enc_late_event_count) — nunca perdido; o
// piso é refinamento de reação mínima, não requisito de correção.

#define ENC_EVT_QUEUE_SIZE 48U

struct EncSchedEvent {
    uint32_t timestamp;   // TIM2 raw 32-bit target (counts, não ticks)
    uint8_t  channel;     // ECU_CH_INJ1..IGN4
    uint8_t  high;        // 1=ON/DWELL, 0=OFF/SPARK
};

static EncSchedEvent g_enc_evt_queue[ENC_EVT_QUEUE_SIZE];
static volatile uint8_t g_enc_evt_count = 0U;

static volatile uint32_t g_enc_dbg_evt_overflow = 0U;
static volatile uint32_t g_enc_late_event_count = 0U;

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

static void enc_evt_insert(uint32_t ts, uint8_t channel, uint8_t high) noexcept
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

    if (pos == 0U) {
        TIM2_CCR3 = ts;
        TIM2_SR   = ~TIM_SR_CC3IF;  // rc_w0: só CC3IF é limpo
        TIM2_DIER |= TIM_DIER_CC3IE;
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
    while (g_enc_evt_count > 0U) {
        const EncSchedEvent& e = g_enc_evt_queue[0];
        if ((int32_t)(e.timestamp - now) > 0) { break; }
        enc_evt_execute_head();
    }
    while (g_enc_evt_count > 0U) {
        const uint32_t next_ts = g_enc_evt_queue[0].timestamp;
        if ((int32_t)(next_ts - TIM2_CNT) > (int32_t)dispatch_ccr_margin_counts()) {
            TIM2_CCR3 = next_ts;
            TIM2_SR   = ~TIM_SR_CC3IF;
            return;
        }
        ++g_enc_late_event_count;
        enc_evt_execute_head();
    }
    TIM2_DIER &= ~TIM_DIER_CC3IE;
}

static void arm_channel_with_lead(uint8_t ch, uint32_t target_counts,
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
uint32_t ecu_sched_encoder_test_get_evt_overflow(void) noexcept { return g_enc_dbg_evt_overflow; }
uint32_t ecu_sched_encoder_test_get_late_event_count(void) noexcept { return g_enc_late_event_count; }
uint32_t ecu_sched_encoder_test_get_dier(void) noexcept { return ems_test_tim2_dier; }
#endif

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
    if (g_enc_evt_count == 0U) {
        TIM2_DIER &= ~TIM_DIER_CC3IE;
    } else {
        TIM2_CCR3 = g_enc_evt_queue[0].timestamp;
        TIM2_SR   = ~TIM_SR_CC3IF;
        TIM2_DIER |= TIM_DIER_CC3IE;
    }
}

void encoder_clear_all(void) noexcept
{
    g_enc_evt_count = 0U;
    TIM2_DIER &= ~TIM_DIER_CC3IE;
}

}  // namespace ems::engine::sched_internal

// ── Heartbeat TIM2_CH4 — heavy tick (1×/volta) ───────────────────────────
// Chamado a cada 64º sub-tick por ecu_sched_encoder_heartbeat_subtick()
// (ISR CC4IF rearma CCR4 +256). Recebe valores já lidos pelo HAL.

static volatile uint32_t g_hb_last_cmp_edge_count = 0U;

// Estado do rastreador de flancos CMP — mantido aqui (não em encoder_sync.cpp,
// que só tem funções puras) porque é o heartbeat que sabe a cadência (1
// avaliação por flanco novo, não por tick) e é quem decide quando descartar a
// referência (streak_resync) ou reset (staleness). Ver
// docs/dev/mt6835_encoder_fork.md, "Sync-state em modo encoder".
static uint8_t  g_cmp_has_prev              = 0U;
static uint32_t g_cmp_prev_angle            = 0U;
static uint8_t  g_cmp_reject_streak         = 0U;
static uint32_t g_cmp_heartbeats_since_ok   = 0U;
static uint32_t g_cmp_reject_count          = 0U;  // diagnóstico
static uint32_t g_cmp_missed_edge_count     = 0U;  // diagnóstico (multiple>1)

// Split light/heavy do heartbeat (ver ecu_sched_encoder_heartbeat_subtick()
// abaixo) — conta sub-ticks (256 counts) desde o último tick pesado
// (16384 counts = 64 sub-ticks). Satura implicitamente a 64 pelo próprio
// reset a 0 dentro da função; nunca lido fora dela em produção, só em teste.
static uint8_t g_hb_subtick_count = 0U;

// Handoff presync→sequencial: a última chamada a recompute_presync() deixa
// 4+4 canais armados; ao entrar em phase_valid purgamos tudo uma vez antes
// do armamento tardio por janela (try_arm_sequential_due).
static uint8_t g_enc_last_builder_was_sequential = 0U;
// Pares dwell/spark ou inj_on/off saltados porque o alvo de spark/EOI
// estava dentro do piso min-lead (kickback / TIM2 a decrementar).
static volatile uint32_t g_enc_seq_min_lead_skip_count = 0U;
static volatile uint32_t g_enc_omega_refresh_count = 0U;

// ω (×65536, counts TIM2 por tick TIM5) → rpm_x10, para o CkpSnapshot
// partilhado. Mesma unidade/escala que ckp_instant_rpm_x10() já usa
// (rpm×10), derivação equivalente a rpm_x10_from_period_ticks() (ckp.cpp)
// mas para 1 revolução inteira (16384 counts), não 1/60 (1 dente): rpm_x10 =
// 600e9/(60×tooth_period_ns) no CKP vira 600e9/(1×rev_period_ns) aqui.
// rev_period_ticks = 16384×65536/ω ⇒ rpm_x10 = 600e9×ω/(16×16384×65536).
// ω negativo (rotação inversa, kick-back) ou inválido: 0 — mesmo princípio
// de "sem RPM sem posição fiável" já usado em rpm_if_synced() (ckp.cpp).
static uint32_t omega_x65536_to_rpm_x10(void) noexcept
{
    if (ecu_sched_encoder_omega_valid() == 0U) { return 0U; }
    const int32_t omega = ecu_sched_encoder_omega_x65536();
    if (omega <= 0) { return 0U; }
    constexpr uint64_t kNumerator = 600000000000ULL;
    constexpr uint64_t kDenominator = 16ULL * 16384ULL * 65536ULL;
    const uint64_t rpm_x10 = (kNumerator * static_cast<uint64_t>(omega)) / kDenominator;
    return static_cast<uint32_t>(rpm_x10);
}

void ecu_sched_encoder_heartbeat_tick(uint32_t tim2_now, uint32_t tim5_now,
                                      uint32_t cmp_angle,
                                      uint32_t cmp_edge_count) noexcept
{
    // Também amostra aqui: host tests / seed chamam o tick pesado sem
    // passar pelo sub-tick. Em produção o sub-tick já actualizou ω.
    ecu_sched_encoder_omega_sample(tim2_now, tim5_now);

    // Rastreio/validação de flancos CMP corre SEMPRE, mesmo sem calibração —
    // é diagnóstico seguro (conta rejeições/flancos perdidos) e testável em
    // host sem precisar recompilar com a flag. Só o passo final ("confiar
    // nisto para disparar sequencial", ecu_sched_encoder_phase_set_anchor())
    // fica atrás de EMS_MT6835_CMP_PHASE_CALIBRATED — expresso como `if`
    // sobre a macro (0/1 sempre definida em board_pinout.h), não `#if`: com a
    // flag em 0 o compilador elimina o ramo por constant-folding (custo zero
    // em produção), mas o texto continua um `if` normal — não esconde este
    // bloco inteiro de compilar/testar em host-test como um `#if` faria.
    if (cmp_edge_count != g_hb_last_cmp_edge_count) {
        g_hb_last_cmp_edge_count = cmp_edge_count;
        const ems::drv::encoder_sync::CmpEdgeResult r =
            ems::drv::encoder_sync::evaluate_cmp_edge(
                cmp_angle, g_cmp_has_prev != 0U, g_cmp_prev_angle, g_cmp_reject_streak);
        if (r.accepted) {
            g_cmp_has_prev            = 1U;
            g_cmp_prev_angle          = cmp_angle;
            g_cmp_reject_streak       = 0U;
            g_cmp_heartbeats_since_ok = 0U;
            if (r.multiple > 1U) { ++g_cmp_missed_edge_count; }
            if (EMS_MT6835_CMP_PHASE_CALIBRATED) {
                // Definição absoluta, nunca toggle — mesmo flanco pode
                // re-ancorar repetidamente sem se acumular.
                ecu_sched_encoder_phase_set_anchor(
                    cmp_angle, static_cast<uint8_t>(EMS_MT6835_CMP_PHASE_VALUE));
            }
        } else {
            ++g_cmp_reject_count;
            g_cmp_reject_streak = r.reject_streak;
            if (r.streak_resync) { g_cmp_has_prev = 0U; }  // descarta referência, re-arma no próximo flanco
        }
    }

    if (g_cmp_heartbeats_since_ok < 0xFFFFFFFFU) { ++g_cmp_heartbeats_since_ok; }
    if (ecu_sched_encoder_phase_valid() != 0U &&
        ems::drv::encoder_sync::staleness_exceeded(
            g_cmp_heartbeats_since_ok, ems::drv::sensors_is_bench_mode())) {
        ecu_sched_encoder_phase_invalidate();
    }

    // Sem calibração de fase, phase_valid() é sempre 0 — presync. Com fase
    // válida: armamento sequencial tardio (janela ≤60°) via try_arm; o
    // sub-tick também chama try_arm a cada CC4IF.
    if (ecu_sched_encoder_phase_valid() == 0U) {
        g_enc_last_builder_was_sequential = 0U;
        si::encoder::recompute_presync(tim2_now);
    } else {
        if (g_enc_last_builder_was_sequential == 0U) {
            si::encoder_purge_cyl_mask(0x0FU, 1U);
            si::encoder_purge_cyl_mask(0x0FU, 0U);
            // Fecha fisicamente qualquer pino deixado HIGH por um DWELL_START/
            // INJ_ON já dispatched pelo presync cuja contraparte (SPARK/INJ_OFF)
            // acabou de ser purgada acima — sem isto a bobina/injector ficava a
            // carregar até o watchdog (1.4×) cortar tarde. Mesmo mecanismo do
            // purge legado (ecu_sched.cpp:purge_events_for_cyl_mask).
            force_close_cyl_mask(0x0FU, 1U);
            force_close_cyl_mask(0x0FU, 0U);
            si::encoder::clear_cyl_arm_latches();
            g_enc_last_builder_was_sequential = 1U;
        }
        si::g_knock_sequential = 1U;
        si::encoder::refresh_pending_omega_spans(tim2_now);
        si::encoder::try_arm_sequential_due(tim2_now);
    }

    // Publica no CkpSnapshot partilhado — único ponto deste ficheiro que o
    // faz; o poll de saúde do MT6835 (main_stm32.cpp, ~100ms) só escreve
    // ems::drv::encoder_sync::set_health_ok(), lido aqui, para nunca haver
    // dois publicadores independentes de g_state.snap a pisarem-se.
    ems::drv::CkpSnapshot snap{};
    if (!ems::drv::encoder_sync::health_ok()) {
        snap.state = ems::drv::SyncState::LOSS_OF_SYNC;
    } else if (ecu_sched_encoder_phase_valid() != 0U) {
        snap.state = ems::drv::SyncState::FULL_SYNC;
        snap.cmp_confirms = 2U;
    } else {
        snap.state = ems::drv::SyncState::HALF_SYNC;
        snap.cmp_confirms = 0U;
    }
    snap.phase_A = (ecu_sched_encoder_phase_at(tim2_now) == ECU_PHASE_A);
    snap.rpm_x10 = omega_x65536_to_rpm_x10();
    snap.tooth_period_ns = 0U;  // sem equivalente encoder — misfire lê tim2/tim5 directo (ecu_sched_encoder_heartbeat_subtick), não este campo
    // Derivado de tim2_now (mesma fórmula que sensors_map_window_poll_encoder(),
    // sensors.cpp, usa localmente para o MAP window) — publicado aqui no
    // snapshot GLOBAL para que auxiliaries.cpp (run_vvt_control() /
    // calc_cam_pos_est_x10()) veja uma posição de came contínua em vez de
    // ficar preso em 0 (colapsava para um sinal quase-binário 0°/180° via
    // phase_A, corrompendo o PID de VVT em tempo real). misfire lê tim2/tim5
    // directo via caminho próprio (misfire_encoder_on_sample), nunca este
    // campo — ver misfire_detect.cpp:misfire_on_tooth(), dormant em modo
    // encoder (só corre a partir do ISR TIM5, que nunca dispara aqui).
    snap.tooth_index = static_cast<uint16_t>((tim2_now % 16384u) * 60u / 16384u);
    snap.last_tim5_capture = tim5_now;
    ems::drv::ckp_publish_encoder_snapshot(snap);
}

// Sub-tick do heartbeat — chamado a CADA CC4IF (256 counts, ~64×/volta),
// não só 1×/volta como ecu_sched_encoder_heartbeat_tick() acima. Caminho
// leve (misfire_encoder_on_sample(), sempre — precisa da cadência fina;
// uma janela de cilindro de 62° só tem ~11 sub-ticks de resolução
// angular) roda em CADA chamada. Caminho pesado (ω, CMP, staleness,
// recompute_presync / try_arm_sequential_due, publish) continua 1×/volta,
// chamado daqui a cada 64º sub-tick — cadência total idêntica à de antes
// desta tarefa (16384 counts), agora composta de 64 passos em vez de 1 (ver
// test_ecu_sched_encoder_heartbeat_subtick_cadence, que prova isto
// isoladamente, e os 3 testes pré-existentes test_ecu_sched_encoder_heartbeat*,
// que continuam a passar bit-a-bit chamando ecu_sched_encoder_heartbeat_tick()
// directamente).
void ecu_sched_encoder_heartbeat_subtick(uint32_t tim2_now, uint32_t tim5_now,
                                         uint32_t cmp_angle,
                                         uint32_t cmp_edge_count) noexcept
{
    ecu_sched_encoder_omega_sample(tim2_now, tim5_now);
    ems::engine::misfire_encoder_on_sample(tim2_now, tim5_now);

    // Armamento sequencial tardio + refresh de spans sob Δω.
    if (ecu_sched_encoder_phase_valid() != 0U) {
        si::g_knock_sequential = 1U;
        si::encoder::refresh_pending_omega_spans(tim2_now);
        si::encoder::try_arm_sequential_due(tim2_now);
    }

    ++g_hb_subtick_count;
    if (g_hb_subtick_count >= 64U) {
        g_hb_subtick_count = 0U;
        ecu_sched_encoder_heartbeat_tick(tim2_now, tim5_now, cmp_angle, cmp_edge_count);
    }
}

#if defined(EMS_HOST_TEST)
void ecu_sched_encoder_heartbeat_test_reset(void) noexcept
{
    g_hb_last_cmp_edge_count  = 0U;
    g_cmp_has_prev            = 0U;
    g_cmp_prev_angle          = 0U;
    g_cmp_reject_streak       = 0U;
    g_cmp_heartbeats_since_ok = 0U;
    g_cmp_reject_count        = 0U;
    g_cmp_missed_edge_count   = 0U;
    g_hb_subtick_count        = 0U;
    g_enc_last_builder_was_sequential = 0U;
    g_enc_seq_min_lead_skip_count = 0U;
    g_enc_omega_refresh_count = 0U;
    si::encoder::clear_cyl_arm_latches();
    ems::drv::encoder_sync::set_health_ok(true);
}
uint32_t ecu_sched_encoder_test_get_cmp_reject_count(void) noexcept { return g_cmp_reject_count; }
uint32_t ecu_sched_encoder_test_get_cmp_missed_edge_count(void) noexcept { return g_cmp_missed_edge_count; }
uint32_t ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(void) noexcept { return g_cmp_heartbeats_since_ok; }
uint8_t  ecu_sched_encoder_test_get_subtick_count(void) noexcept { return g_hb_subtick_count; }
uint32_t ecu_sched_encoder_test_get_seq_min_lead_skip_count(void) noexcept
{
    return g_enc_seq_min_lead_skip_count;
}
uint32_t ecu_sched_encoder_test_get_omega_refresh_count(void) noexcept
{
    return g_enc_omega_refresh_count;
}
#endif

namespace ems::engine::sched_internal::encoder {

// ── Conversão graus de motor → counts TIM2 ───────────────────────────────
//
// TIM2_CNT embrulha a cada 16384 contagens = 1 volta de cambota (360°), não
// 720° como o ciclo do motor. A origem (que ângulo de motor corresponde a
// TIM2_CNT==0) é uma constante de calibração de hardware — reaproveitada de
// cfg::g_eng_cfg.trigger_tooth0_engine_deg, o mesmo campo já usado pelo
// caminho roda-dentada (mesmo conceito físico: "que ângulo corresponde à
// posição bruta zero"), NVM-backed e já com procedimento de bancada — ver
// engine_config.h e docs/dev/mt6835_encoder_fork.md. Só a resídua MOD 360
// do campo é significativa aqui.
//
// Duplicado deliberadamente de engine_angle_to_trigger_angle()
// (ecu_sched_angle.cpp:47-53) em vez de partilhado: essa função usa
// cycle_deg=720 (ciclo do motor); aqui o domínio de embrulho É 360 (uma
// volta de TIM2). Partilhar reintroduziria exatamente a confusão 720/360
// que este ficheiro já teve de resolver.
static uint32_t engine_deg_to_counts_in_rev(uint32_t engine_angle_deg) noexcept
{
    const uint32_t origin_mod360 =
        static_cast<uint32_t>(cfg::g_eng_cfg.trigger_tooth0_engine_deg) % 360U;
    const uint32_t crank_deg =
        (engine_angle_deg % 360U + 360U - origin_mod360) % 360U;
    return (crank_deg * 16384U) / 360U;
}

// Posição-alvo dentro da volta (0..16383) → próxima ocorrência absoluta em
// counts de 32 bits. Sempre em (now_raw, now_raw+16384] — janela semi-aberta
// que casa com a cadência do heartbeat TIM2_CH4 (1×/volta): alvo==posição
// atual cai no FIM da janela (próxima volta), nunca no início, para não
// coincidir com o instante em que o próprio heartbeat acabou de disparar.
static uint32_t rev_target_to_absolute(uint32_t target_counts_in_rev,
                                       uint32_t now_raw) noexcept
{
    const uint32_t now_mod = now_raw & 0x3FFFU;
    const uint32_t fwd = (target_counts_in_rev > now_mod)
        ? (target_counts_in_rev - now_mod)
        : (16384U - now_mod + target_counts_in_rev);
    return now_raw + fwd;
}

static uint32_t engine_deg_to_absolute(uint32_t engine_angle_deg,
                                       uint32_t now_raw) noexcept
{
    return rev_target_to_absolute(engine_deg_to_counts_in_rev(engine_angle_deg), now_raw);
}

// Conversão fase-consciente grau→posição absoluta (ciclo 720°). Irmã de
// engine_deg_to_absolute() — essa fica intocada para recompute_presync()
// (cego à fase). phase_at() é o oráculo: g_phase_anchor_raw (CMP) não é
// congruente com 0 mod 16384, logo "início-do-bin + resíduo" misturaria
// dois referenciais. Somar exactamente um +16384 inverte a paridade de
// fase; +32768 preservá-la-ia e passaria 720° do alvo.
static uint32_t engine_deg720_to_absolute(uint32_t engine_angle_deg /* 0..719 */,
                                          uint32_t now_raw) noexcept
{
    const uint8_t target_phase =
        (engine_angle_deg < 360U) ? ECU_PHASE_A : ECU_PHASE_B;
    uint32_t candidate =
        rev_target_to_absolute(engine_deg_to_counts_in_rev(engine_angle_deg), now_raw);
    if (ecu_sched_encoder_phase_at(candidate) != target_phase) {
        candidate += 16384U;
    }
    return candidate;
}

// Duração (ticks TIM5) → extensão angular em counts, via ω mais recente —
// NÃO via graus. dwell/PW são tempo de bobina/injector convertido em
// comprimento angular; ir por graus só duplicaria arredondamento sem
// necessidade (ver estimador de ω acima e a nota "Iteração de desenho" do
// plano). ω inválido ou ≤0 (motor parado/estimativa não pronta): span 0 —
// esta chamada nunca deveria acontecer nesse estado, mas 0 é o valor seguro
// (dwell nulo é preferível a um span inventado).
static uint32_t duration_ticks_to_span_counts(uint32_t duration_ticks) noexcept
{
    if (ecu_sched_encoder_omega_valid() == 0U) { return 0U; }
    const int32_t omega = ecu_sched_encoder_omega_x65536();
    if (omega <= 0) { return 0U; }
    const int64_t span = (static_cast<int64_t>(duration_ticks)
                          * static_cast<int64_t>(omega)) / 65536;
    return (span < 0) ? 0U : static_cast<uint32_t>(span);
}

// Piso mínimo de lead do compare (plano, secção 7) — equivalente ao
// STM32_MIN_COMPARE_LEAD_TICKS de ecu_sched.cpp (125 ticks = 2 µs @
// 62,5 MHz), mas convertido para counts via ω em vez de hardcodado: os
// mesmos 2 µs valem ~0,001 count a idle e ~5 counts perto do redline (ver
// estimador de ω acima) — um piso fixo em counts seria irrelevante numa
// ponta e exagerado na outra. ω inválido (motor parado / estimativa ainda
// não pronta): piso 0, sem margem extra — correto por construção, um alvo
// já passado ainda cai no caminho "late" do dispatch (g_enc_late_event_count),
// nunca é perdido (ver comentário acima da fila TIM2/CH3).
uint32_t min_lead_counts(void) noexcept
{
    return duration_ticks_to_span_counts(ECU_SCHED_US_TO_TICKS_INTERNAL(2U));
}

static bool lead_ge_min(uint32_t target, uint32_t now_raw,
                        uint32_t min_lead) noexcept
{
    return static_cast<int32_t>(target - now_raw) >=
           static_cast<int32_t>(min_lead);
}

static uint32_t inj_pw_span_from_ticks(uint32_t pw_ticks) noexcept
{
    uint32_t inj_pw_span = duration_ticks_to_span_counts(pw_ticks);
    if (inj_pw_span > kMaxSeqInjPwCounts) {
        inj_pw_span = kMaxSeqInjPwCounts;
        ++g_pw_duty_clamp_count;
    }
    return inj_pw_span;
}

// Arm ON+OFF as a pair, or skip both (kickback / clamp-invert guard).
static void arm_pair_if_lead(uint8_t ch, uint32_t on_ts, uint32_t off_ts,
                             uint32_t now_raw, uint32_t min_lead,
                             uint8_t on_act, uint8_t off_act) noexcept
{
    if (!lead_ge_min(on_ts, now_raw, min_lead) ||
        !lead_ge_min(off_ts, now_raw, min_lead)) {
        ++g_enc_seq_min_lead_skip_count;
        return;
    }
    arm_channel_with_lead(ch, on_ts, on_act, min_lead);
    arm_channel_with_lead(ch, off_ts, off_act, min_lead);
}

static bool lead_in_arm_window(uint32_t target, uint32_t now_raw,
                               uint32_t min_lead, uint32_t window) noexcept
{
    const int32_t lead = static_cast<int32_t>(target - now_raw);
    return lead >= static_cast<int32_t>(min_lead) &&
           lead <= static_cast<int32_t>(window);
}

// Earlier absolute target from now (smaller positive lead). If one is already
// behind now, prefer the other.
static uint32_t earlier_abs_target(uint32_t a, uint32_t b, uint32_t now_raw) noexcept
{
    const int32_t la = static_cast<int32_t>(a - now_raw);
    const int32_t lb = static_cast<int32_t>(b - now_raw);
    if (la < 0) { return b; }
    if (lb < 0) { return a; }
    return (la <= lb) ? a : b;
}

static bool cyl_has_pending_events(uint8_t cyl) noexcept
{
    const uint8_t ign = kIgnCh[cyl];
    const uint8_t inj = kInjCh[cyl];
    for (uint8_t i = 0U; i < g_enc_evt_count; ++i) {
        const uint8_t ch = g_enc_evt_queue[i].channel;
        if (ch == ign || ch == inj) { return true; }
    }
    return false;
}

// Latch de ticks + alvos geométricos para refresh de spans sob Δω.
struct CylArmLatch {
    uint8_t  active;
    uint8_t  ign_locked;
    uint8_t  inj_locked;
    uint32_t dwell_ticks;
    uint32_t inj_pw_ticks;
    uint32_t spark_abs;
    uint32_t eoi_abs;
    int32_t  omega_x65536;
};
static CylArmLatch g_cyl_latch[4]{};

static void clear_cyl_latches(void) noexcept
{
    for (uint8_t i = 0U; i < 4U; ++i) {
        g_cyl_latch[i] = CylArmLatch{};
    }
}

void clear_cyl_arm_latches(void) noexcept
{
    clear_cyl_latches();
}

// Reposiciona o evento ON (high=1) do canal; OFF (spark/EOI) fica fixo.
static void enc_evt_retarget_high(uint8_t ch, uint32_t new_ts) noexcept
{
    ems::hal::CriticalSectionGuard guard;
    int8_t found = -1;
    for (uint8_t i = 0U; i < g_enc_evt_count; ++i) {
        if (g_enc_evt_queue[i].channel == ch && g_enc_evt_queue[i].high != 0U) {
            found = static_cast<int8_t>(i);
            break;
        }
    }
    if (found < 0) { return; }
    for (uint8_t i = static_cast<uint8_t>(found);
         static_cast<uint8_t>(i + 1U) < g_enc_evt_count; ++i) {
        g_enc_evt_queue[i] = g_enc_evt_queue[i + 1U];
    }
    --g_enc_evt_count;
    // Reinsert sorted (no knock re-open — dwell already armed): mesma cauda
    // "posição ordenada + rearm CCR3 se pos==0" de enc_evt_insert() — reusa-a
    // em vez de duplicar. Não passa pelo ramo de overflow (enc_evt_insert
    // ainda o verifica, mas é inatingível aqui: a entrada acabada de remover
    // acima garante espaço). enc_evt_insert() não toma guard própria, por
    // isso é seguro chamá-la dentro da CriticalSectionGuard já aberta.
    enc_evt_insert(new_ts, ch, 1U);
}

static bool omega_rel_change_ge(int32_t omega_now, int32_t omega_ref,
                                uint32_t rel_x1000) noexcept
{
    if (omega_ref == 0) { return omega_now != 0; }
    int32_t d = omega_now - omega_ref;
    if (d < 0) { d = -d; }
    int32_t base = omega_ref;
    if (base < 0) { base = -base; }
    return (static_cast<uint64_t>(d) * 1000ULL) >=
           (static_cast<uint64_t>(base) * static_cast<uint64_t>(rel_x1000));
}

void refresh_pending_omega_spans(uint32_t now_raw) noexcept
{
    if (ecu_sched_encoder_omega_valid() == 0U) { return; }
    const int32_t omega = ecu_sched_encoder_omega_x65536();
    if (omega <= 0) { return; }
    const uint32_t min_lead = min_lead_counts();

    for (uint8_t cyl = 0U; cyl < cfg::kCylinderCount; ++cyl) {
        CylArmLatch& L = g_cyl_latch[cyl];
        if (L.active == 0U) { continue; }
        if (!cyl_has_pending_events(cyl)) {
            L = CylArmLatch{};
            continue;
        }

        const int32_t lead_spark =
            static_cast<int32_t>(L.spark_abs - now_raw);
        const int32_t lead_eoi =
            static_cast<int32_t>(L.eoi_abs - now_raw);
        if (lead_spark <= static_cast<int32_t>(min_lead)) {
            L.ign_locked = 1U;
        }
        if (lead_eoi <= static_cast<int32_t>(min_lead)) {
            L.inj_locked = 1U;
        }
        if (L.ign_locked != 0U && L.inj_locked != 0U) { continue; }
        if (!omega_rel_change_ge(omega, L.omega_x65536, kOmegaRefreshRelX1000)) {
            continue;
        }

        const uint32_t dwell_span =
            duration_ticks_to_span_counts(L.dwell_ticks);
        const uint32_t inj_span = inj_pw_span_from_ticks(L.inj_pw_ticks);

        if (L.ign_locked == 0U) {
            const uint32_t new_dwell = L.spark_abs - dwell_span;
            if (lead_ge_min(new_dwell, now_raw, min_lead)) {
                enc_evt_retarget_high(kIgnCh[cyl], new_dwell);
#if defined(EMS_HOST_TEST)
                ++g_enc_omega_refresh_count;
#endif
            }
        }
        if (L.inj_locked == 0U) {
            const uint32_t new_inj_on = L.eoi_abs - inj_span;
            if (lead_ge_min(new_inj_on, now_raw, min_lead)) {
                enc_evt_retarget_high(kInjCh[cyl], new_inj_on);
#if defined(EMS_HOST_TEST)
                ++g_enc_omega_refresh_count;
#endif
            }
        }
        L.omega_x65536 = omega;
    }
}

// Arma IGN (+ multi-spark) e INJ a partir de setpoints já finalizados.
static void arm_sequential_cyl(uint8_t cyl, uint32_t now_raw,
                               const ems::engine::CylArmSetpoints& sp,
                               uint32_t min_lead,
                               uint32_t ms_inter_deg) noexcept
{
    const uint32_t tdc = cfg::cyl_tdc_deg(cyl);
    const uint32_t dwell_span =
        duration_ticks_to_span_counts(sp.dwell_ticks);
    const uint32_t inj_span = inj_pw_span_from_ticks(sp.inj_pw_ticks);

    const uint32_t spark_deg =
        (tdc + kCycleDeg - sp.advance_deg) % kCycleDeg;
    const uint32_t eoi_deg =
        (tdc + kCycleDeg - sp.eoi_lead_deg) % kCycleDeg;

    const uint32_t spark_target =
        engine_deg720_to_absolute(spark_deg, now_raw);
    const uint32_t dwell_target = spark_target - dwell_span;

    arm_pair_if_lead(kIgnCh[cyl], dwell_target, spark_target, now_raw, min_lead,
                     ECU_ACT_DWELL_START, ECU_ACT_SPARK);

    if (lead_ge_min(spark_target, now_raw, min_lead) &&
        lead_ge_min(dwell_target, now_raw, min_lead)) {
        emit_multispark_deg(spark_deg, kCycleDeg, ms_inter_deg,
            [&](uint32_t add_dwell_deg, uint32_t add_spark_deg) {
                const uint32_t add_dwell_t =
                    engine_deg720_to_absolute(add_dwell_deg, now_raw);
                const uint32_t add_spark_t =
                    engine_deg720_to_absolute(add_spark_deg, now_raw);
                arm_pair_if_lead(kIgnCh[cyl], add_dwell_t, add_spark_t, now_raw,
                                 min_lead, ECU_ACT_DWELL_START, ECU_ACT_SPARK);
            });
    }

    const uint32_t eoi_target = engine_deg720_to_absolute(eoi_deg, now_raw);
    const uint32_t inj_on_target = eoi_target - inj_span;
    arm_pair_if_lead(kInjCh[cyl], inj_on_target, eoi_target, now_raw, min_lead,
                     ECU_ACT_INJ_ON, ECU_ACT_INJ_OFF);

    if (cyl_has_pending_events(cyl) &&
        ecu_sched_encoder_omega_valid() != 0U) {
        CylArmLatch& L = g_cyl_latch[cyl];
        L.active = 1U;
        L.ign_locked = 0U;
        L.inj_locked = 0U;
        L.dwell_ticks = sp.dwell_ticks;
        L.inj_pw_ticks = sp.inj_pw_ticks;
        L.spark_abs = spark_target;
        L.eoi_abs = eoi_target;
        L.omega_x65536 = ecu_sched_encoder_omega_x65536();
    }
}

// Armamento sequencial tardio: só insere eventos quando dwell ou inj_on
// entram na janela [min_lead, 60°]. Chamado a cada sub-tick TIM2_CH4 e no
// heartbeat pesado (handoff). Não reconstrói meia-fase com 360° de antecipação.
void try_arm_sequential_due(uint32_t now_raw) noexcept
{
    static_assert(cfg::kCylinderCount == 4u, "ign/inj channel tables are 4-cyl");
    if (ecu_sched_encoder_omega_valid() == 0U) { return; }

    g_knock_sequential = 1U;

    const uint32_t min_lead = min_lead_counts();
    const uint32_t window = kSeqArmWindowCounts;
    uint32_t ms_inter_deg = 0U;
    if (g_mspark_count > 0U) {
        ms_inter_deg = (duration_ticks_to_span_counts(g_mspark_inter_dwell_ticks)
                        * kCycleDeg) / 32768U;
    }

    for (uint8_t cyl = 0U; cyl < cfg::kCylinderCount; ++cyl) {
        if (cyl_has_pending_events(cyl)) { continue; }

        // Peek: avalia o candidato (inclui X-τ) sem comitar o modelo de
        // parede — este loop corre ~65×/volta por cilindro pendente, e a
        // maioria das tentativas falha o teste de janela abaixo (continue).
        // Comitar aqui sobre-integrava o filme de parede dezenas de vezes
        // por spray real, neutralizando o enriquecimento de AE (fix bug 3).
        const ems::engine::CylArmSetpoints sp =
            ems::engine::finalize_cyl_setpoints(cyl, /*commit_fuel=*/false);
        const uint32_t dwell_span =
            duration_ticks_to_span_counts(sp.dwell_ticks);
        const uint32_t inj_span = inj_pw_span_from_ticks(sp.inj_pw_ticks);

        const uint32_t tdc = cfg::cyl_tdc_deg(cyl);
        const uint32_t spark_deg =
            (tdc + kCycleDeg - sp.advance_deg) % kCycleDeg;
        const uint32_t eoi_deg =
            (tdc + kCycleDeg - sp.eoi_lead_deg) % kCycleDeg;

        const uint32_t spark_target =
            engine_deg720_to_absolute(spark_deg, now_raw);
        const uint32_t dwell_target = spark_target - dwell_span;
        const uint32_t eoi_target =
            engine_deg720_to_absolute(eoi_deg, now_raw);
        const uint32_t inj_on_target = eoi_target - inj_span;

        const uint32_t arm_at =
            earlier_abs_target(dwell_target, inj_on_target, now_raw);
        if (!lead_in_arm_window(arm_at, now_raw, min_lead, window)) {
            continue;
        }

        // Janela confirmada — comita agora o passo do modelo X-τ. Reavalia
        // (não reutiliza o sp do peek acima) para garantir que o setpoint
        // armado é exatamente o que produziu a mutação persistida: contexto
        // single-threaded (heartbeat/sub-tick), sem outro escritor de
        // g_cyl_wall_us_q8 entre as duas chamadas nesta iteração.
        const ems::engine::CylArmSetpoints sp_committed =
            ems::engine::finalize_cyl_setpoints(cyl, /*commit_fuel=*/true);
        arm_sequential_cyl(cyl, now_raw, sp_committed, min_lead, ms_inter_deg);
    }
}

// ── Recompute presync — chamado pelo heartbeat TIM2_CH4 quando a fase A/B
// ainda não está confirmada (ecu_sched_encoder_phase_valid()==0, sempre
// verdade sem EMS_MT6835_CMP_PHASE_CALIBRATED — ver board_pinout.h).
// Equivalente a rebuild_presync_revolution() (ecu_sched_angle.cpp) em
// counts: mesma matemática de ângulo (spark/eoi/inj_on/inj_off, bank
// toggle), mas SPARK/EOI vão por engine_deg_to_absolute() (geometria pura)
// e dwell/PW vão por duration_ticks_to_span_counts() (ω), nunca por graus —
// ver nota acima.
//
// Multi-spark também aqui (wasted, cycle 360°) — paridade com
// rebuild_presync_revolution(); o gate de RPM em main_stm32 continua a ser
// quem liga/desliga g_mspark_count (cranking tipicamente abaixo do tecto).
void recompute_presync(uint32_t now_raw) noexcept
{
    static const uint8_t inj_a[2] = {ECU_CH_INJ1, ECU_CH_INJ4};
    static const uint8_t inj_b[2] = {ECU_CH_INJ2, ECU_CH_INJ3};

    // Paridade com rebuild_presync_revolution() (CKP): wasted-spark ⇒
    // g_knock_sequential=0. Também limpa o handoff flag — a próxima
    // transição para sequencial fará purga total 4+4.
    g_knock_sequential = 0U;
    g_enc_last_builder_was_sequential = 0U;
    clear_cyl_arm_latches();

    // Purga TUDO das 4 IGN + 4 INJ antes de reconstruir — garante que nunca
    // fica uma DWELL_START pendente sem o SPARK emparelhado (ou um INJ_ON
    // sem o INJ_OFF): o par é sempre inserido junto, na mesma passagem, logo
    // após limpar. Sem isto, uma janela sobreposta (RPM mudou entre
    // heartbeats, IRQ atrasada) podia deixar um par órfão na fila — o
    // cenário que danifica hardware (bobina a carregar sem descarga
    // agendada), não só correr mal.
    encoder_purge_cyl_mask(0x0FU, 1U);
    encoder_purge_cyl_mask(0x0FU, 0U);

    const uint32_t dwell_span = duration_ticks_to_span_counts(g_dwell_ticks);
    const uint32_t min_lead = min_lead_counts();
    const uint32_t raw_inj_pw_ticks =
        (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS)
            ? (g_inj_pw_ticks / 2U)
            : g_inj_pw_ticks;
    uint32_t inj_pw_span = duration_ticks_to_span_counts(raw_inj_pw_ticks);
    // Clamp de duty — paridade com kMaxPresyncInjPwDeg no caminho CKP.
    if (inj_pw_span > kMaxPresyncInjPwCounts) {
        inj_pw_span = kMaxPresyncInjPwCounts;
        ++g_pw_duty_clamp_count;
    }

    const uint32_t spark_a = (360U - (g_advance_deg % 360U)) % 360U;
    const uint32_t spark_b = (180U + 360U - (g_advance_deg % 360U)) % 360U;
    const uint32_t eoi_deg   = (360U - (g_eoi_lead_deg % 360U)) % 360U;

    const uint32_t eoi_target    = engine_deg_to_absolute(eoi_deg, now_raw);
    const uint32_t inj_on_target = eoi_target - inj_pw_span;

    // Wasted-spark: 2 bobinas por evento, pares a 180° — nunca as 4 no
    // mesmo alvo. g_knock_sequential fica 0 (limpo no topo).
    uint32_t ms_inter_deg = 0U;
    if (g_mspark_count > 0U) {
        ms_inter_deg = (duration_ticks_to_span_counts(g_mspark_inter_dwell_ticks)
                        * 360U) / 16384U;
    }
    const auto arm_wasted_pair = [&](uint32_t spark_deg, const uint8_t pair[2]) {
        const uint32_t spark_target = engine_deg_to_absolute(spark_deg, now_raw);
        const uint32_t dwell_target = spark_target - dwell_span;
        for (uint8_t i = 0U; i < 2U; ++i) {
            arm_channel_with_lead(pair[i], dwell_target, ECU_ACT_DWELL_START, min_lead);
            arm_channel_with_lead(pair[i], spark_target, ECU_ACT_SPARK, min_lead);
        }
        emit_multispark_deg(spark_deg, 360U, ms_inter_deg,
            [&](uint32_t add_dwell_deg, uint32_t add_spark_deg) {
                const uint32_t add_dwell_t =
                    engine_deg_to_absolute(add_dwell_deg, now_raw);
                const uint32_t add_spark_t =
                    engine_deg_to_absolute(add_spark_deg, now_raw);
                for (uint8_t i = 0U; i < 2U; ++i) {
                    arm_pair_if_lead(pair[i], add_dwell_t, add_spark_t, now_raw,
                                     min_lead, ECU_ACT_DWELL_START, ECU_ACT_SPARK);
                }
            });
    };
    arm_wasted_pair(spark_a, kWastedIgnPairA);
    arm_wasted_pair(spark_b, kWastedIgnPairB);

    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
        for (uint8_t i = 0U; i < 4U; ++i) {
            arm_channel_with_lead(kInjCh[i], inj_on_target, ECU_ACT_INJ_ON, min_lead);
            arm_channel_with_lead(kInjCh[i], eoi_target, ECU_ACT_INJ_OFF, min_lead);
        }
    } else {
        // bank_off == bank_on sempre (mesmo banco liga/desliga na mesma
        // volta) — simplificação provada equivalente à derivação em dois
        // passos de rebuild_presync_revolution() (toggle após o ON, reler
        // para o OFF): com toggle inicial T, bank_on=(T==0)?a:b e, após
        // T^=1, bank_off=(novo T==1)?a:b == bank_on sempre. Usar o mesmo
        // banco directamente evita reler o toggle duas vezes.
        const uint8_t *bank = (g_presync_bank_toggle == 0U) ? inj_a : inj_b;
        for (uint8_t i = 0U; i < 2U; ++i) {
            arm_channel_with_lead(bank[i], inj_on_target, ECU_ACT_INJ_ON, min_lead);
            arm_channel_with_lead(bank[i], eoi_target, ECU_ACT_INJ_OFF, min_lead);
        }
        g_presync_bank_toggle ^= 1U;
    }
}

}  // namespace ems::engine::sched_internal::encoder

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

#if defined(EMS_HOST_TEST)
// Hooks de teste — extern "C", free functions (mesma convenção do resto do
// ficheiro/ecu_sched.h), não namespaced: test_sched.cpp só vê o que estiver
// declarado em ecu_sched.h.
uint32_t ecu_sched_encoder_test_engine_deg_to_counts(uint32_t engine_angle_deg) noexcept
{
    return si::encoder::engine_deg_to_counts_in_rev(engine_angle_deg);
}
uint32_t ecu_sched_encoder_test_rev_target_to_absolute(uint32_t target_counts_in_rev,
                                                        uint32_t now_raw) noexcept
{
    return si::encoder::rev_target_to_absolute(target_counts_in_rev, now_raw);
}
uint32_t ecu_sched_encoder_test_deg720_to_absolute(uint32_t engine_angle_deg,
                                                     uint32_t now_raw) noexcept
{
    return si::encoder::engine_deg720_to_absolute(engine_angle_deg, now_raw);
}
uint32_t ecu_sched_encoder_test_arm_window_counts(void) noexcept
{
    return si::kSeqArmWindowCounts;
}
uint32_t ecu_sched_encoder_test_predict_arm_at(uint8_t cyl, uint32_t now_raw) noexcept
{
    namespace cfg = ems::engine::cfg;
    if (cyl >= cfg::kCylinderCount) { return now_raw; }
    const ems::engine::CylArmSetpoints sp =
        ems::engine::finalize_cyl_setpoints(cyl);
    const uint32_t tdc = cfg::cyl_tdc_deg(cyl);
    const uint32_t spark_deg =
        (tdc + si::kCycleDeg - sp.advance_deg) % si::kCycleDeg;
    const uint32_t eoi_deg =
        (tdc + si::kCycleDeg - sp.eoi_lead_deg) % si::kCycleDeg;
    const uint32_t spark_abs =
        ecu_sched_encoder_test_deg720_to_absolute(spark_deg, now_raw);
    const uint32_t eoi_abs =
        ecu_sched_encoder_test_deg720_to_absolute(eoi_deg, now_raw);
    // Mesma fórmula ω→span da produção (duration_ticks_to_span_counts), em
    // vez de reimplementá-la aqui: evita que este hook de teste "se prove a
    // si mesmo" caso a fórmula real mude sem o teste acompanhar.
    const uint32_t dwell_span =
        si::encoder::duration_ticks_to_span_counts(sp.dwell_ticks);
    const uint32_t inj_span =
        si::encoder::duration_ticks_to_span_counts(sp.inj_pw_ticks);
    const uint32_t dwell_abs = spark_abs - dwell_span;
    const uint32_t inj_on_abs = eoi_abs - inj_span;
    const int32_t ld = static_cast<int32_t>(dwell_abs - now_raw);
    const int32_t li = static_cast<int32_t>(inj_on_abs - now_raw);
    if (ld < 0) { return inj_on_abs; }
    if (li < 0) { return dwell_abs; }
    return (ld <= li) ? dwell_abs : inj_on_abs;
}
#endif
