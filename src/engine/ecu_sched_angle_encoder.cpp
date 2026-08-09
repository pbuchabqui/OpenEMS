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
 * Estrutura prevista (preenchida pelas tarefas seguintes, ainda vazias):
 *   - Fila TIM2/CH3 própria (evt_insert/dispatch equivalentes, unidade
 *     counts) — TODO.
 *   - Rastreador de fase: anchor absoluto de 32 bits, atualizado por
 *     definição absoluta a cada flanco do CMP (sensor Hall inalterado,
 *     cmp_angle_snapshot()) — TODO.
 *   - Estimador de ω: ΔTIM2_CNT/ΔTIM5_CNT, delta com sinal (sobrevive a
 *     bounce de cranking) — TODO.
 *   - Heartbeat TIM2_CH4: recompute barato de dwell/PW, bank-toggle do
 *     presync, verificação de deriva do CMP — TODO (HAL em
 *     hal/stm32h562/timer.cpp; este ficheiro só a lógica de resposta).
 *   - Conversão graus→counts + função de recompute partilhada com
 *     ecu_sched_commit_calibration() — TODO.
 */

#include "engine/ecu_sched_internal.h"
#include "hal/out_pins.h"
#include "hal/critical_section.h"
#if !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

#include <stdint.h>

namespace si = ems::engine::sched_internal;

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
// amostras consecutivas de (TIM2_CNT, TIM5_CNT), tiradas pelo heartbeat
// TIM2_CH4 (1×/volta de cambota, hal/stm32h562/timer.cpp — ainda não
// implementado, ver TODO abaixo). Pura aritmética, sem acesso a registo —
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
// Fixed-point ×256 (contagens de TIM2 por tick de TIM5) para não perder
// precisão numa razão tipicamente << 1 em ponto inteiro — mesma convenção
// _x256 já usada no projeto (sub_frac_x256).

static volatile uint32_t g_omega_prev_tim2  = 0U;
static volatile uint32_t g_omega_prev_tim5  = 0U;
static volatile uint8_t  g_omega_have_prev  = 0U;
static volatile int32_t  g_omega_x256       = 0;
static volatile uint8_t  g_omega_valid      = 0U;

void ecu_sched_encoder_omega_sample(uint32_t tim2_now, uint32_t tim5_now) noexcept
{
    if (g_omega_have_prev != 0U) {
        const int32_t d_tim5 = (int32_t)(tim5_now - g_omega_prev_tim5);
        if (d_tim5 > 0) {
            const int32_t d_tim2 = (int32_t)(tim2_now - g_omega_prev_tim2);
            g_omega_x256 = (int32_t)(((int64_t)d_tim2 * 256) / (int64_t)d_tim5);
            g_omega_valid = 1U;
        }
        // d_tim5 <= 0: relógio não avançou (ou amostra fora de ordem) —
        // mantém a última estimativa válida, não atualiza.
    }
    g_omega_prev_tim2 = tim2_now;
    g_omega_prev_tim5 = tim5_now;
    g_omega_have_prev = 1U;
}

int32_t ecu_sched_encoder_omega_x256(void) noexcept { return g_omega_x256; }
uint8_t ecu_sched_encoder_omega_valid(void) noexcept { return g_omega_valid; }

#if defined(EMS_HOST_TEST)
// Chamado por ecu_sched_test_reset() (ecu_sched.cpp) — evita estado do
// estimador vazar entre casos de teste no mesmo binário.
void ecu_sched_encoder_omega_test_reset(void) noexcept
{
    g_omega_prev_tim2 = 0U;
    g_omega_prev_tim5 = 0U;
    g_omega_have_prev = 0U;
    g_omega_x256 = 0;
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
// Ainda sem piso de lead mínimo (pendente — plano, secção 7): um alvo já
// passado no momento do arm cai no caminho "late" do dispatch (processado
// inline, contado em g_enc_late_event_count), nunca é perdido — piso é
// refinamento de reação mínima, não requisito de correção.

#define ENC_EVT_QUEUE_SIZE 48U

struct EncSchedEvent {
    uint32_t timestamp;   // TIM2 raw 32-bit target (counts, não ticks)
    uint8_t  channel;     // ECU_CH_INJ1..IGN4
    uint8_t  high;        // 1=ON/DWELL, 0=OFF/SPARK
    uint8_t  valid;
    uint8_t  _pad;
};

static EncSchedEvent g_enc_evt_queue[ENC_EVT_QUEUE_SIZE];
static volatile uint8_t g_enc_evt_count = 0U;
static volatile uint8_t g_enc_evt_armed = 0U;

static volatile uint32_t g_enc_dbg_evt_inserted   = 0U;
static volatile uint32_t g_enc_dbg_evt_dispatched = 0U;
static volatile uint32_t g_enc_dbg_evt_overflow   = 0U;
static volatile uint32_t g_enc_late_event_count   = 0U;

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
    ++g_enc_dbg_evt_inserted;
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
    g_enc_evt_queue[pos].valid = 1U;
    ++g_enc_evt_count;

    if (pos == 0U) {
        TIM2_CCR3 = ts;
        TIM2_SR   = ~TIM_SR_CC3IF;  // rc_w0: só CC3IF é limpo
        TIM2_DIER |= TIM_DIER_CC3IE;
        g_enc_evt_armed = 1U;
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
    ++g_enc_dbg_evt_dispatched;
    --g_enc_evt_count;
    for (uint8_t i = 0U; i < g_enc_evt_count; ++i) {
        g_enc_evt_queue[i] = g_enc_evt_queue[i + 1U];
    }
}

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
        if ((int32_t)(next_ts - TIM2_CNT) > 16) {
            TIM2_CCR3 = next_ts;
            TIM2_SR   = ~TIM_SR_CC3IF;
            g_enc_evt_armed = 1U;
            return;
        }
        ++g_enc_late_event_count;
        enc_evt_execute_head();
    }
    TIM2_DIER &= ~TIM_DIER_CC3IE;
    g_enc_evt_armed = 0U;
}

void ecu_sched_encoder_arm_channel(uint8_t ch, uint32_t target_counts,
                                   uint8_t action) noexcept
{
    // Mesma razão do arm_channel() de ecu_sched.cpp: inserir na fila +
    // tocar CCR3/DIER não pode intercalar com o dispatch ISR (TIM2 CH3).
    ems::hal::CriticalSectionGuard guard;
    const uint8_t high =
        ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U;
    enc_evt_insert(target_counts, ch, high);
}

#if defined(EMS_HOST_TEST)
void ecu_sched_encoder_queue_test_reset(void) noexcept
{
    g_enc_evt_count = 0U;
    g_enc_evt_armed = 0U;
    for (uint8_t i = 0U; i < ENC_EVT_QUEUE_SIZE; ++i) { g_enc_evt_queue[i].valid = 0U; }
    g_enc_dbg_evt_inserted = 0U;
    g_enc_dbg_evt_dispatched = 0U;
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
        g_enc_evt_armed = 0U;
    } else {
        TIM2_CCR3 = g_enc_evt_queue[0].timestamp;
        TIM2_SR   = ~TIM_SR_CC3IF;
        TIM2_DIER |= TIM_DIER_CC3IE;
        g_enc_evt_armed = 1U;
    }
}

void encoder_clear_all(void) noexcept
{
    g_enc_evt_count = 0U;
    g_enc_evt_armed = 0U;
    TIM2_DIER &= ~TIM_DIER_CC3IE;
}

}  // namespace ems::engine::sched_internal

// ── Heartbeat TIM2_CH4 — resposta ao tick ────────────────────────────────
// Chamado 1×/volta pelo TIM2_IRQHandler (CC4IF, hal/stm32h562/timer.cpp),
// que já cuida do rearme de CCR4 e das leituras de registo — esta função só
// recebe os valores já lidos (mesma disciplina de host-testabilidade do
// resto do ficheiro).

static volatile uint32_t g_hb_last_cmp_edge_count = 0U;

void ecu_sched_encoder_heartbeat_tick(uint32_t tim2_now, uint32_t tim5_now,
                                      uint32_t cmp_angle,
                                      uint32_t cmp_edge_count) noexcept
{
    ecu_sched_encoder_omega_sample(tim2_now, tim5_now);

    // Novo flanco do CMP desde o último tick? Só regista por agora — a fase
    // que esse flanco representa é uma constante de calibração de hardware
    // ainda não medida em bancada (ecu_sched_encoder_phase_set_anchor()
    // precisa dela), não algo que este heartbeat possa inventar. Resolver
    // isso + o recompute barato de dwell/PW + bank-toggle do presync é a
    // próxima tarefa do plano.
    if (cmp_edge_count != g_hb_last_cmp_edge_count) {
        g_hb_last_cmp_edge_count = cmp_edge_count;
        (void)cmp_angle;  // TODO: ecu_sched_encoder_phase_set_anchor(cmp_angle, <fase calibrada>)
    }
    // TODO: recompute barato de dwell_deg/inj_pw_deg + bank-toggle presync
    // (tarefa seguinte do plano — "Conversão graus→counts + recompute
    // partilhado").
}

#if defined(EMS_HOST_TEST)
void ecu_sched_encoder_heartbeat_test_reset(void) noexcept
{
    g_hb_last_cmp_edge_count = 0U;
}
#endif

namespace ems::engine::sched_internal::encoder {

// Placeholder — preenchido pela tarefa seguinte do plano (conversão
// graus→counts + recompute partilhado com ecu_sched_commit_calibration()).

}  // namespace ems::engine::sched_internal::encoder
