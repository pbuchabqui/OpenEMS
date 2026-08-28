/**
 * @file ecu_sched_encoder_heartbeat.cpp
 * @brief Heartbeat TIM2_CH4: subtick + heavy tick + publish snapshot.
 *
 * ISR: TIM2_IRQHandler CC4IF em hal/stm32h562/timer.cpp (CCR4 += 256,
 * depois este subtick). Caminho leve ~64×/volta; pesado 1×/volta.
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

// Confirmação extra antes de re-ancorar a fase depois de uma referência
// perdida (streak_resync ou phase_invalidate). "1 flanco após has_prev==
// false" trata de forma idêntica dois cenários diferentes: "nunca vi nada
// desde o boot" (tem de aceitar o primeiro, sem alternativa) e "acabei de
// descartar por 3 rejeições seguidas" (o próximo pode muito bem ser mais
// um flanco espúrio — ex.: TIM3/PC6 armado pelo watchdog sobre um pino
// agora flutuante). g_cmp_confirm_count (definido junto de g_phase_valid)
// exige 2 flancos aceites CONSECUTIVOS (o 2º já passa pela checagem de
// plausibilidade de evaluate_cmp_edge() contra o 1º) antes de aplicar o
// anchor de verdade. phase_invalidate() zera o contador para o mesmo
// gate valer depois de staleness/stall.

// Watchdog do TIM3 CMP IC: pedido de rearm de hardware quando
// g_cmp_heartbeats_since_ok cruza o mesmo limiar de staleness_exceeded(),
// mas DELIBERADAMENTE fora do `if (phase_valid())` que guarda o bloco
// acima — phase_valid() só fica 1 depois de cmp_phase_state calibrado
// (ecu_sched_encoder_phase_set_anchor()), então um watchdog pendurado
// nesse bloco nunca dispararia numa bancada fria/presync, exatamente o
// cenário onde mais falta faz. Ver ecu_sched_encoder_heartbeat_tick()
// abaixo e ecu_sched_encoder_cmp_watchdog_poll_and_clear() (consumido
// pelo loop de 2ms em main_stm32.cpp).
static volatile uint8_t  g_cmp_watchdog_rearm_pending  = 0U;
static volatile uint32_t g_cmp_watchdog_request_count  = 0U;

// Watchdog do builder sequencial: rede de segurança se phase_valid()==1
// mas nenhum cilindro consegue armar por várias voltas seguidas (âncora
// possivelmente corrompida por um re-anchor espúrio — Fix B reduz a
// probabilidade, não elimina; ruído é probabilístico). Independente do
// watchdog do TIM3 acima: aquele reage a "sem flanco CMP", este reage a
// "com flanco(s) CMP aceites, mas o sequencial nunca arma nada" — o
// sintoma real reportado (INJ/IGN mudos com CKP vivo).
static uint32_t g_seq_arm_watch_last_count    = 0U;
static uint32_t g_seq_heavy_ticks_without_arm = 0U;
static volatile uint32_t g_seq_arm_stall_count = 0U;  // diagnóstico
static volatile uint32_t g_presync_call_count  = 0U;  // heavy ticks in presync
static volatile uint32_t g_seq_call_count      = 0U;  // heavy ticks in sequential

uint32_t ecu_sched_encoder_presync_call_count(void) noexcept
{
    return g_presync_call_count;
}

uint32_t ecu_sched_encoder_seq_call_count(void) noexcept
{
    return g_seq_call_count;
}

uint32_t ecu_sched_encoder_enc_evt_insert_count(void) noexcept
{
    return g_enc_dbg_insert_count;
}

uint32_t ecu_sched_encoder_enc_evt_execute_count(void) noexcept
{
    return g_enc_dbg_execute_count;
}

// Split light/heavy do heartbeat (ver ecu_sched_encoder_heartbeat_subtick()
// abaixo) — conta sub-ticks (256 counts) desde o último tick pesado
// (16384 counts = 64 sub-ticks). Satura implicitamente a 64 pelo próprio
// reset a 0 dentro da função; nunca lido fora dela em produção, só em teste.
static uint8_t g_hb_subtick_count = 0U;

// Handoff presync→sequencial: a última chamada a recompute_presync() deixa
// 4+4 canais armados; ao entrar em phase_valid purgamos tudo uma vez antes
// do armamento tardio por janela (try_arm_sequential_due).
uint8_t g_enc_last_builder_was_sequential = 0U;

uint32_t ecu_sched_encoder_seq_min_lead_skip_count(void) noexcept
{
    return g_enc_seq_min_lead_skip_count;
}

// Consumido 1×/2ms pelo loop principal (main_stm32.cpp), fora de
// contexto de ISR — rearma ems::hal::tim3_cmp_ic_init() quando true.
// Poll+clear atômico: evita perder um pedido se outro heartbeat_tick
// correr entre a leitura e a limpeza da flag.
uint8_t ecu_sched_encoder_cmp_watchdog_poll_and_clear(void) noexcept
{
    ems::hal::CriticalSectionGuard guard;
    const uint8_t pending = g_cmp_watchdog_rearm_pending;
    g_cmp_watchdog_rearm_pending = 0U;
    return pending;
}

uint32_t ecu_sched_encoder_cmp_watchdog_request_count(void) noexcept
{
    return g_cmp_watchdog_request_count;
}

uint32_t ecu_sched_encoder_seq_arm_stall_count(void) noexcept
{
    return g_seq_arm_stall_count;
}

void ecu_sched_encoder_heartbeat_drop_cmp_ref(void) noexcept
{
    g_cmp_has_prev = 0U;
    g_cmp_reject_streak = 0U;
    g_enc_last_builder_was_sequential = 0U;
    si::encoder::clear_cyl_arm_latches();
}

static bool encoder_omega_forward(void) noexcept
{
    return (ecu_sched_encoder_omega_valid() != 0U) &&
           (ecu_sched_encoder_omega_x65536() > 0);
}

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
                                      uint32_t cmp_edge_count,
                                      uint8_t run_seq_arm) noexcept
{
    ecu_sched_encoder_omega_sample(tim2_now, tim5_now);

    // Rastreio/validação de flancos CMP corre SEMPRE, mesmo sem calibração —
    // é diagnóstico seguro (conta rejeições/flancos perdidos) e testável em
    // host sem precisar de nada calibrado. Só o passo final ("confiar nisto
    // para disparar sequencial", ecu_sched_encoder_phase_set_anchor()) fica
    // atrás de cfg::g_eng_cfg.cmp_phase_state (2026-08-15: campo NVM
    // calibrável ao vivo — comando 'M', ui_protocol.cpp — substitui os
    // antigos EMS_MT6835_CMP_PHASE_CALIBRATED/_VALUE de compilação,
    // removidos de board_pinout.h). Default kCmpPhaseUncalibrated=0 ⇒ este
    // `if` nunca corre até uma calibração real acontecer — mesma garantia
    // de segurança de antes ("nunca adivinha"), agora sem precisar de
    // recompilar/reflash para ativar depois de medido.
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
            if (g_cmp_confirm_count < 2U) { ++g_cmp_confirm_count; }
            if (r.multiple > 1U) { ++g_cmp_missed_edge_count; }
            const uint8_t was_phase_valid = ecu_sched_encoder_phase_valid();
            if (g_cmp_confirm_count >= 2U &&
                ems::engine::cfg::g_eng_cfg.cmp_phase_state !=
                ems::engine::cfg::kCmpPhaseUncalibrated) {
                // Definição absoluta, nunca toggle — mesmo flanco pode
                // re-ancorar repetidamente sem se acumular.
                //
                // Re-ancorar em TODO flanco aceite (não só uma vez) não é
                // checagem redundante: CKP e CMP têm ligação mecânica fixa
                // (corrente/correia de distribuição) só enquanto essa
                // ligação estiver saudável. `r.accepted` acima já passou
                // por evaluate_cmp_edge() (encoder_sync.h), cuja tolerância
                // de espaçamento é explicitamente um "orçamento de folga
                // MECÂNICA" — um salto de dente da corrente/correia
                // apareceria ali como flanco rejeitado, não como um anchor
                // silenciosamente errado. Sem este re-anchor contínuo,
                // ecu_sched_encoder_phase_at() (pura aritmética de
                // paridade) continuaria a confiar num anchor desatualizado
                // indefinidamente após um slip real.
                // Hardware: o flanco CMP marca sempre a fase A (nunca B).
                ecu_sched_encoder_phase_set_anchor(cmp_angle, ECU_PHASE_A);
            }
            // Scope: 1 lóbulo / 720°, na metade A (0–359).
            if (ecu_sched_encoder_phase_valid() != 0U) {
                if (was_phase_valid == 0U) {
                    ems::drv::ckp_scope_clear_cmp();
                }
                ems::drv::ckp_scope_push_cmp_deg720(
                    ems::drv::cycle_deg(cmp_angle, true));
            }
        } else {
            ++g_cmp_reject_count;
            g_cmp_reject_streak = r.reject_streak;
            if (r.streak_resync) {
                g_cmp_has_prev = 0U;
                g_cmp_confirm_count = 0U;
                // Same as staleness: keep sequential on a rejected cam
                // (jumped chain) is spark-on-exhaust. Drop the 720° half.
                if (ecu_sched_encoder_phase_valid() != 0U) {
                    ecu_sched_encoder_phase_invalidate();
                }
            }
        }
    }

    if (g_cmp_heartbeats_since_ok < 0xFFFFFFFFU) { ++g_cmp_heartbeats_since_ok; }
    {
        const bool cmp_stale = ems::drv::encoder_sync::staleness_exceeded(
            g_cmp_heartbeats_since_ok, ems::drv::sensors_is_bench_mode());
        if (cmp_stale) {
            // Descarta a referência angular — independente de phase_valid(),
            // mesmo princípio do watchdog do TIM3 abaixo. Sem isto, o 1º
            // flanco depois de um silêncio longo do CMP COM A CAMBOTA A RODAR
            // era comparado por evaluate_cmp_edge() contra um g_cmp_prev_angle
            // de antes do silêncio: o ângulo andou dezenas de spans entretanto,
            // delta ≫ 4×32768 (kCmpMaxAcceptedMultiple) ⇒ REJEITADO, e a
            // recuperação só acontecia pela via lenta das 3 rejeições até
            // streak_resync. Note-se o âmbito: isto só cobre o caso em que os
            // flancos VOLTAM a chegar (cmp_edge_count a mexer). Se o contador
            // cru ficar parado, este bloco nem corre — esse é o modo de falha
            // do watchdog do TIM3 abaixo.
            // Aqui a referência é velha POR CONSTRUÇÃO: heartbeats_since_ok
            // só é zerado por um flanco ACEITE (rejeitados deixam-no crescer),
            // logo staleness ⇒ nada foi aceite há ≥6 heartbeats.
            //
            // Gate de 2 flancos preservado: isto reproduz exatamente o estado
            // que streak_resync já produz acima (has_prev=0 + confirm_count=0),
            // um caminho existente e testado — flanco 1 só arma a referência
            // (!has_prev ⇒ accepted, sem anchor), flanco 2 é que re-ancora.
            //
            // Store repetido enquanto stale é deliberado e idempotente — NÃO
            // trocar por "==" como o watchdog do TIM3 abaixo (esse guarda um
            // contador; este não guarda estado nenhum).
            g_cmp_has_prev = 0U;   // g_cmp_prev_angle só é lido com has_prev!=0
            if (ecu_sched_encoder_phase_valid() != 0U) {
                ecu_sched_encoder_phase_invalidate();
            }
        }
    }

    // Watchdog do TIM3 CMP IC — independente do bloco acima (não exige
    // phase_valid()). "==" em vez de ">=": g_cmp_heartbeats_since_ok é
    // monótono não-decrescente até o próximo flanco aceite (que o zera),
    // então isto só é verdadeiro numa única volta por episódio de
    // silêncio — auto rate-limit sem precisar de cooldown/timer novo.
    {
        const uint32_t limit = ems::drv::sensors_is_bench_mode()
            ? ems::drv::encoder_sync::kMaxHeartbeatsWithoutCmpBench
            : ems::drv::encoder_sync::kMaxHeartbeatsWithoutCmp;
        if (g_cmp_heartbeats_since_ok == limit) {
            g_cmp_watchdog_rearm_pending = 1U;
            ++g_cmp_watchdog_request_count;
        }
    }

    const bool health_bad = !ems::drv::encoder_sync::health_ok();
    const bool reverse = (ecu_sched_encoder_omega_valid() != 0U) &&
                         (ecu_sched_encoder_omega_x65536() <= 0);

    // Refresh de spans só aqui (1×/volta), nunca no sub-tick: ω de 256
    // counts é ruidoso e puxava o dwell para trás até o watchdog cortar.
    if (!health_bad && encoder_omega_forward() &&
        ecu_sched_encoder_phase_valid() != 0U) {
        si::encoder::refresh_pending_omega_spans(tim2_now);
    }

    // Sem calibração de fase, phase_valid() é sempre 0 — presync. Com fase
    // válida: armamento sequencial tardio (janela ≤60°) via try_arm; o
    // sub-tick também chama try_arm a cada CC4IF.
    // health_ok=false / reverse: do not arm; close any HIGH pins (kick-back
    // / wrong encoder = hydrolock or spark-in-open-valve).
    if (health_bad) {
        ecu_sched_on_encoder_stall();
    } else if (reverse) {
        ecu_sched_drive_outputs_safe();
    } else if (ecu_sched_encoder_phase_valid() == 0U) {
        // Queda sequencial→presync (CMP desligado / stall / uncalibrated):
        // um INJ_ON/DWELL já despachado cuja contraparte ficou na fila
        // seria purgada por recompute_presync() e o pino ficava HIGH até
        // o watchdog — o "INJ descontrolado" do fallback. Fecha já, uma
        // vez, na borda da transição; rebuilds seguintes não fecham.
        if (g_enc_last_builder_was_sequential != 0U) {
            force_close_cyl_mask(0x0FU, 1U);
            force_close_cyl_mask(0x0FU, 0U);
        }
        g_enc_last_builder_was_sequential = 0U;
        ++g_presync_call_count;
        si::encoder::recompute_presync(tim2_now);
    } else {
        ++g_seq_call_count;
        uint8_t did_handoff = 0U;
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
            did_handoff = 1U;
            // Reentrada em sequencial após uma excursão por presync (CMP
            // stale/stall/uncalibrated): zera o crédito parcial do watchdog
            // de "nada armado" abaixo. Sem isto, ticks já acumulados antes
            // da excursão (recompute_presync() nunca chama try_arm, então
            // seq_arm_success_count() fica parado durante o presync — não
            // há "arm count change" para zerar isto sozinho) sobreviviam e
            // o episódio sequencial novo dispunha de menos que a margem
            // pretendida (~6 ticks/~3 revoluções) antes de um trip
            // desnecessário de volta a presync.
            g_seq_heavy_ticks_without_arm = 0U;
        }
        si::g_knock_sequential = 1U;
        // run_seq_arm=0: subtick já fez refresh+try_arm neste CC4IF — evita
        // duplicar. Após handoff (purge) ainda precisamos de try_arm já.
        if (run_seq_arm != 0U || did_handoff != 0U) {
            si::encoder::refresh_pending_omega_spans(tim2_now);
            si::encoder::try_arm_sequential_due(tim2_now);
        }

        // Watchdog de "nada armado" — roda incondicionalmente aqui (não só
        // quando run_seq_arm/did_handoff), cadência de 1×/volta: em regime
        // normal try_arm_sequential_due já é chamado ~64×/volta pelo
        // sub-tick, então este contador se move bem antes do timeout numa
        // sequência saudável (cada cilindro tem de conseguir armar pelo
        // menos 1×/720° = a cada 2 heavy-ticks).
        {
            const uint32_t arm_count_now = si::encoder::seq_arm_success_count();
            if (arm_count_now != g_seq_arm_watch_last_count) {
                g_seq_arm_watch_last_count = arm_count_now;
                g_seq_heavy_ticks_without_arm = 0U;
            } else {
                ++g_seq_heavy_ticks_without_arm;
                if (g_seq_heavy_ticks_without_arm >= ECU_SEQ_ARM_STALL_HEAVY_TICKS) {
                    ecu_sched_encoder_phase_invalidate();
                    g_seq_heavy_ticks_without_arm = 0U;
                    ++g_seq_arm_stall_count;
                }
            }
        }
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
    snap.tooth_period_ns = 0U;
    snap.tim2_cnt = tim2_now;
    snap.crank_deg = ems::drv::crank_deg(tim2_now);
    snap.last_tim5_capture = tim5_now;
    // Glitch count real do CMP (achado #4 da revisão 2fa1513..bc30ca6,
    // 2026-08-19): ckp_get_cmp_glitch_count() lê isto via CkpSnapshot em vez
    // de ficar preso em stub — mesmo padrão de push já usado por cmp_confirms.
    snap.cmp_reject_count = g_cmp_reject_count;
    ems::drv::ckp_publish_encoder_snapshot(snap);

    // One rev per heavy tick — spark-skip Bresenham. Not inferred from a
    // fake tooth_index wrap in the 2 ms loop (that misses wraps at high RPM).
    ems::engine::spark_skip_on_rev();
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
    // Amostra ω já no 2º sub-tick (~1 ms) para o sequencial poder armar
    // sem esperar 2 voltas. O refresh de dwell NÃO corre aqui (ver tick
    // pesado) — janela de 256 counts é ruidosa demais para reposicionar ON.
    ecu_sched_encoder_omega_sample(tim2_now, tim5_now);
    ems::engine::misfire_encoder_on_sample(tim2_now, tim5_now);
    {
        const bool pa = (ecu_sched_encoder_phase_at(tim2_now) == ECU_PHASE_A);
        ems::drv::ckp_scope_push_ckp_deg720(ems::drv::cycle_deg(tim2_now, pa));
    }

    if (!ems::drv::encoder_sync::health_ok()) {
        // Heavy tick does the full stall; skip arm here (64×/rev).
    } else if ((ecu_sched_encoder_omega_valid() != 0U) &&
               (ecu_sched_encoder_omega_x65536() <= 0)) {
        ecu_sched_drive_outputs_safe();
    } else if (ecu_sched_encoder_phase_valid() != 0U) {
        si::g_knock_sequential = 1U;
        si::encoder::try_arm_sequential_due(tim2_now);
    }

    ++g_hb_subtick_count;
    if (g_hb_subtick_count >= 64U) {
        g_hb_subtick_count = 0U;
        // Já fizemos refresh+try_arm acima — tick pesado só handoff/publish.
        ecu_sched_encoder_heartbeat_tick(tim2_now, tim5_now, cmp_angle,
                                         cmp_edge_count, /*run_seq_arm=*/0U);
    }
}

// ── Correção de drift via Z (índice, TIM3_CH2/PC7) ───────────────────────
// docs/dev/mt6835_encoder_fork.md, "Correção de drift via Z". Espelha o
// rastreador de CMP acima, mas mais simples: Z não ancora fase (isso
// continua exclusivo do CMP), só corrige o offset de software partilhado
// via ecu_sched_encoder_phase_correction_update_from_z(). Estado próprio,
// função irmã — não toca em nenhum dos globais/funções do CMP acima.

static uint32_t g_hb_last_z_edge_count   = 0U;
static uint8_t  g_z_has_prev             = 0U;
static uint32_t g_z_prev_raw             = 0U;
static uint8_t  g_z_reject_streak        = 0U;
static uint8_t  g_z_confirm_count        = 0U;  // mesmo Fix B do CMP: 2 flancos antes de confiar
static uint8_t  g_z_has_target           = 0U;
static uint32_t g_z_target_mod16384      = 0U;  // alvo fixo, estabelecido 1x ao confirmar
static uint32_t g_z_heartbeats_since_ok  = 0U;  // telemetria pura — nunca invalida fase/stall
static uint32_t g_z_reject_count         = 0U;  // diagnóstico
static uint32_t g_z_missed_edge_count    = 0U;  // diagnóstico (multiple>1)

void ecu_sched_encoder_heartbeat_z_tick(uint32_t z_angle_raw,
                                        uint32_t z_edge_count) noexcept
{
    if (z_edge_count != g_hb_last_z_edge_count) {
        g_hb_last_z_edge_count = z_edge_count;
        const ems::drv::encoder_sync::ZEdgeResult r =
            ems::drv::encoder_sync::evaluate_z_edge(
                z_angle_raw, g_z_has_prev != 0U, g_z_prev_raw, g_z_reject_streak);
        if (r.accepted) {
            g_z_has_prev            = 1U;
            g_z_prev_raw            = z_angle_raw;
            g_z_reject_streak       = 0U;
            g_z_heartbeats_since_ok = 0U;
            if (r.multiple > 1U) { ++g_z_missed_edge_count; }
            if (g_z_confirm_count < 2U) { ++g_z_confirm_count; }
            if (g_z_confirm_count >= 2U) {
                const uint32_t z_mod = z_angle_raw & 0x3FFFU;
                if (!g_z_has_target) {
                    // Estabelece o alvo fixo uma única vez — Z sempre
                    // dispara na mesma posição física do ímã, então não há
                    // razão para re-ancorar a cada flanco (ao contrário do
                    // CMP, cujo anchor é sobre FASE, não uma referência de
                    // drift). Re-ancorar aqui a cada flanco cancelaria
                    // exatamente o drift que queremos detectar.
                    g_z_target_mod16384 = z_mod;
                    g_z_has_target       = 1U;
                } else if (r.multiple == 1U) {
                    // Só corrige em volta única — span de 2×kZSpanCounts é
                    // ambíguo (1 flanco perdido vs. 2 voltas normais), ver
                    // encoder_sync.h.
                    si::encoder::encoder_phase_correction_update_from_z(
                        g_z_target_mod16384, z_mod);
                }
            }
        } else {
            ++g_z_reject_count;
            g_z_reject_streak = r.reject_streak;
            if (r.streak_resync) {
                g_z_has_prev     = 0U;
                g_z_confirm_count = 0U;
                // Descarta também o alvo fixo: depois de 3 rejeições
                // seguidas, não há razão para continuar a confiar num
                // anchor estabelecido antes do episódio de ruído — mais
                // seguro re-estabelecer do zero na próxima sequência boa.
                g_z_has_target    = 0U;
            }
        }
    }

    // Watchdog de Z morto — telemetria pura. Deliberadamente NÃO chama
    // phase_invalidate()/ecu_sched_on_encoder_stall(): Z não é
    // safety-critical (a Camada 3/SPI continua sendo o backstop de
    // segurança, inalterada); um cabo Z partido não deve, por si só, tirar
    // o motor de sincronismo.
    if (g_z_heartbeats_since_ok < 0xFFFFFFFFU) { ++g_z_heartbeats_since_ok; }
}

// Validação cruzada do alvo Z contra o SPI — achado do advisor
// (docs/dev/mt6835_encoder_fork.md, "Correção de drift via Z"): sem isto,
// um alvo Z estabelecido em cima de um drift de AB já existente (ex.: um
// glitch durante o cranking, antes das primeiras 2 voltas confirmadas)
// ficaria permanentemente errado — o rastreador de span nunca detecta isso,
// porque os spans entre flancos Z continuam ~kZSpanCounts independente de
// ONDE o alvo está ancorado. Chamado 1×/poll SPI de 100 ms
// (ems::drv::encoder_sync::poll_100ms(), que já tem spi_mod calculado) —
// não dá para validar no próprio instante do 2º flanco Z porque a leitura
// SPI é bloqueante (~150 µs+, ver kSpiWorstCaseUs) e nunca deve rodar
// dentro de uma ISR de timer.
static constexpr uint32_t kZAnchorSpiToleranceCounts = 32U;

void ecu_sched_encoder_z_target_check(uint32_t spi_mod16384) noexcept
{
    if (!g_z_has_target) { return; }
    const int32_t delta = ems::drv::encoder_sync::circular_diff16384(
        spi_mod16384, g_z_target_mod16384);
    const int32_t abs_delta = (delta < 0) ? -delta : delta;
    if (static_cast<uint32_t>(abs_delta) > kZAnchorSpiToleranceCounts) {
        // Alvo suspeito — descarta e deixa o rastreador reconstruir a
        // partir do próximo par de flancos Z confirmados. Mesmo espírito
        // do streak_resync: mais seguro reconstruir do zero do que
        // continuar a confiar num anchor que pode estar a corrigir na
        // direção errada. g_z_has_prev/prev_raw ficam intactos — a
        // validade do SPAN entre flancos não depende de onde o alvo está.
        g_z_has_target    = 0U;
        g_z_confirm_count = 0U;
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
    g_cmp_confirm_count       = 0U;
    g_hb_subtick_count        = 0U;
    g_enc_last_builder_was_sequential = 0U;
    g_enc_seq_min_lead_skip_count = 0U;
    g_enc_omega_refresh_count = 0U;
    g_cmp_watchdog_rearm_pending  = 0U;
    g_cmp_watchdog_request_count  = 0U;
    g_seq_arm_watch_last_count    = 0U;
    g_seq_heavy_ticks_without_arm = 0U;
    g_seq_arm_stall_count         = 0U;
    g_presync_call_count          = 0U;
    g_seq_call_count              = 0U;
    si::encoder::seq_arm_success_count_test_reset();
    si::encoder::clear_cyl_arm_latches();
    ems::drv::encoder_sync::set_health_ok(true);
    g_hb_last_z_edge_count   = 0U;
    g_z_has_prev             = 0U;
    g_z_prev_raw             = 0U;
    g_z_reject_streak        = 0U;
    g_z_confirm_count        = 0U;
    g_z_has_target           = 0U;
    g_z_target_mod16384      = 0U;
    g_z_heartbeats_since_ok  = 0U;
    g_z_reject_count         = 0U;
    g_z_missed_edge_count    = 0U;
}
uint32_t ecu_sched_encoder_test_get_cmp_reject_count(void) noexcept { return g_cmp_reject_count; }
uint32_t ecu_sched_encoder_test_get_cmp_missed_edge_count(void) noexcept { return g_cmp_missed_edge_count; }
uint32_t ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(void) noexcept { return g_cmp_heartbeats_since_ok; }
uint8_t  ecu_sched_encoder_test_get_cmp_confirm_count(void) noexcept { return g_cmp_confirm_count; }
uint32_t ecu_sched_encoder_test_get_z_reject_count(void) noexcept { return g_z_reject_count; }
uint32_t ecu_sched_encoder_test_get_z_missed_edge_count(void) noexcept { return g_z_missed_edge_count; }
uint32_t ecu_sched_encoder_test_get_z_heartbeats_since_ok(void) noexcept { return g_z_heartbeats_since_ok; }
uint8_t  ecu_sched_encoder_test_get_z_confirm_count(void) noexcept { return g_z_confirm_count; }
uint8_t  ecu_sched_encoder_test_has_z_target(void) noexcept { return g_z_has_target; }
uint8_t  ecu_sched_encoder_test_get_subtick_count(void) noexcept { return g_hb_subtick_count; }
uint32_t ecu_sched_encoder_test_get_seq_min_lead_skip_count(void) noexcept
{
    return ecu_sched_encoder_seq_min_lead_skip_count();
}
uint32_t ecu_sched_encoder_test_get_omega_refresh_count(void) noexcept
{
    return g_enc_omega_refresh_count;
}
#endif
