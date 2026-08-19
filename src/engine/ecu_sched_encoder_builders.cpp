/**
 * @file ecu_sched_encoder_builders.cpp
 * @brief Conversão °→counts, recompute_presync, try_arm_sequential_due.
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

volatile uint32_t g_enc_seq_min_lead_skip_count = 0U;
volatile uint32_t g_enc_omega_refresh_count = 0U;
namespace ems::engine::sched_internal::encoder {

// ── Conversão graus de motor → counts TIM2 ───────────────────────────────
//
// TIM2_CNT embrulha a cada 16384 contagens = 1 volta de cambota (360°), não
// 720° como o ciclo do motor. A origem (que ângulo de motor corresponde a
// TIM2_CNT==0) é uma constante de calibração de hardware —
// cfg::g_eng_cfg.encoder_tdc1_origin_deg, campo dedicado ao caminho encoder
// (NVM-backed, engine_config.h) — deliberadamente separado de
// trigger_tooth0_engine_deg (esse fica exclusivo do caminho roda-dentada;
// decisão do utilizador, 2026-08-13, para poder trocar entre os dois modos
// na mesma placa sem perder a calibração do outro). Só a resídua MOD 360 do
// campo é significativa aqui — ver ecu_sched_encoder_tdc1_calibrate_from_raw()
// abaixo para o preencher a partir de uma leitura real de TIM2.
//
// Duplicado deliberadamente de engine_angle_to_trigger_angle()
// (ecu_sched_angle.cpp:47-53) em vez de partilhado: essa função usa
// cycle_deg=720 (ciclo do motor); aqui o domínio de embrulho É 360 (uma
// volta de TIM2). Partilhar reintroduziria exatamente a confusão 720/360
// que este ficheiro já teve de resolver.
uint32_t engine_deg_to_counts_in_rev(uint32_t engine_angle_deg) noexcept
{
    const uint32_t origin_mod360 =
        static_cast<uint32_t>(cfg::g_eng_cfg.encoder_tdc1_origin_deg) % 360U;
    const uint32_t crank_deg =
        (engine_angle_deg % 360U + 360U - origin_mod360) % 360U;
    return (crank_deg * 16384U) / 360U;
}

// Posição-alvo dentro da volta (0..16383) → próxima ocorrência absoluta em
// counts de 32 bits. Sempre em (now_raw, now_raw+16384] — janela semi-aberta
// que casa com a cadência do heartbeat TIM2_CH4 (1×/volta): alvo==posição
// atual cai no FIM da janela (próxima volta), nunca no início, para não
// coincidir com o instante em que o próprio heartbeat acabou de disparar.
uint32_t rev_target_to_absolute(uint32_t target_counts_in_rev,
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
uint32_t engine_deg720_to_absolute(uint32_t engine_angle_deg /* 0..719 */,
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
uint32_t duration_ticks_to_span_counts(uint32_t duration_ticks) noexcept
{
    if (ecu_sched_encoder_omega_valid() == 0U) { return 0U; }
    const int32_t omega = ecu_sched_encoder_omega_x65536();
    if (omega <= 0) { return 0U; }
    const int64_t span = (static_cast<int64_t>(duration_ticks)
                          * static_cast<int64_t>(omega)) / 65536;
    if (span <= 0) { return 0U; }
    // Rede de segurança: uma bobina/injector nunca cobre mais de 360°.
    // O filtro de ω acima é o que evita o dwell-watchdog a idle.
    constexpr uint32_t kMaxDurationSpanCounts = 16384U;
    if (span > static_cast<int64_t>(kMaxDurationSpanCounts)) {
        return kMaxDurationSpanCounts;
    }
    return static_cast<uint32_t>(span);
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

// Presync: se o ON (dwell / inj_on) já passou ou está dentro do min-lead,
// o par inteiro (ON+OFF) vai para a próxima volta — preserva a duração.
// Sem isto arm_channel_with_lead() empurrava o ON para "agora" e o pulso
// ficava só com o resto até ao spark/EOI (PW/dwell truncados no scope).
static void shift_pair_one_rev_if_on_too_soon(uint32_t& on_ts, uint32_t& off_ts,
                                              uint32_t now_raw,
                                              uint32_t min_lead) noexcept
{
    if (static_cast<int32_t>(on_ts - now_raw) < static_cast<int32_t>(min_lead)) {
        on_ts  += 16384U;
        off_ts += 16384U;
    }
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

        uint32_t dwell_span =
            duration_ticks_to_span_counts(L.dwell_ticks);
        // Tecto 5/4 do span de armação: um ω ruidoso não pode alongar o
        // dwell em tempo até ao watchdog (1.4×). 5/4 deixa folga para RPM
        // a subir de verdade sem disparar o corte de 1.4×.
        if (L.omega_x65536 > 0) {
            const uint32_t armed_span = static_cast<uint32_t>(
                (static_cast<int64_t>(L.dwell_ticks) * L.omega_x65536) / 65536);
            const uint32_t max_span = (armed_span * 5U) / 4U;
            if (max_span != 0U && dwell_span > max_span) {
                dwell_span = max_span;
            }
        }
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

// Alvos absolutos TIM2 já calculados no peek de try_arm_sequential_due —
// evita re-derivar spans + engine_deg720_to_absolute no arm vencedor.
struct SeqArmAbsTargets {
    uint32_t spark_abs;
    uint32_t dwell_abs;
    uint32_t eoi_abs;
    uint32_t inj_on_abs;
    uint32_t spark_deg;
};

// Arma IGN (+ multi-spark) e INJ a partir de setpoints já finalizados.
static void arm_sequential_cyl(uint8_t cyl, uint32_t now_raw,
                               const ems::engine::CylArmSetpoints& sp,
                               const SeqArmAbsTargets& t,
                               uint32_t min_lead,
                               uint32_t ms_inter_deg) noexcept
{
    // Máscara de inibição de ignição (oil_protect_cut/overtemp_cut/
    // diag_critical, ign_mask_cut em main_stm32.cpp) — achado #3 da revisão
    // 2026-08-15: em modo encoder esta máscara nunca era consultada aqui,
    // só purgava a fila TIM2/CH3 UMA VEZ na borda de subida
    // (ecu_sched_set_ign_inhibit_mask, ecu_sched.cpp). Sem este gate,
    // try_arm_sequential_due (que corre ~64×/volta) voltava a armar
    // dwell/faísca no ciclo seguinte, tornando oil/overtemp/diag-critical
    // um blip momentâneo em vez de um corte sustentado. Espelha o gate já
    // existente em force_output() (ecu_sched.cpp) para o caminho legado.
    const uint8_t ign_bit =
        (kIgnCh[cyl] < 8U) ? si::k_ign_ch_to_bit[kIgnCh[cyl]] : 0U;
    const uint8_t ign_inhibited = (ign_bit != 0U &&
        (::ecu_sched_get_ign_inhibit_mask() & ign_bit) != 0U) ? 1U : 0U;

    if (ign_inhibited == 0U) {
        arm_pair_if_lead(kIgnCh[cyl], t.dwell_abs, t.spark_abs, now_raw, min_lead,
                         ECU_ACT_DWELL_START, ECU_ACT_SPARK);

        if (lead_ge_min(t.spark_abs, now_raw, min_lead) &&
            lead_ge_min(t.dwell_abs, now_raw, min_lead)) {
            emit_multispark_deg(t.spark_deg, kCycleDeg, ms_inter_deg,
                [&](uint32_t add_dwell_deg, uint32_t add_spark_deg) {
                    const uint32_t add_dwell_t =
                        engine_deg720_to_absolute(add_dwell_deg, now_raw);
                    const uint32_t add_spark_t =
                        engine_deg720_to_absolute(add_spark_deg, now_raw);
                    arm_pair_if_lead(kIgnCh[cyl], add_dwell_t, add_spark_t, now_raw,
                                     min_lead, ECU_ACT_DWELL_START, ECU_ACT_SPARK);
                });
        }
    }

    arm_pair_if_lead(kInjCh[cyl], t.inj_on_abs, t.eoi_abs, now_raw, min_lead,
                     ECU_ACT_INJ_ON, ECU_ACT_INJ_OFF);

    if (cyl_has_pending_events(cyl) &&
        ecu_sched_encoder_omega_valid() != 0U) {
        CylArmLatch& L = g_cyl_latch[cyl];
        L.active = 1U;
        L.ign_locked = 0U;
        L.inj_locked = 0U;
        L.dwell_ticks = sp.dwell_ticks;
        L.inj_pw_ticks = sp.inj_pw_ticks;
        L.spark_abs = t.spark_abs;
        L.eoi_abs = t.eoi_abs;
        L.omega_x65536 = ecu_sched_encoder_omega_x65536();
    }
}

// Armamento sequencial tardio: só insere eventos quando dwell ou inj_on
// entram na janela [min_lead, 60°]. Chamado a cada sub-tick TIM2_CH4 e no
// heartbeat pesado (handoff). Não reconstrói meia-fase com 360° de antecipação.
// Diagnóstico do watchdog de "nada armado" (ecu_sched_encoder_heartbeat_tick()):
// monotónico, nunca reseta — o chamador compara valores sucessivos para
// detetar heavy-ticks consecutivos sem nenhum cilindro armado enquanto
// phase_valid()==1 (âncora possivelmente corrompida por um re-anchor
// espúrio — ver Fix C, mesma sessão do watchdog do TIM3 CMP IC).
static uint32_t g_seq_arm_success_count = 0U;

uint32_t seq_arm_success_count(void) noexcept { return g_seq_arm_success_count; }
void seq_arm_success_count_test_reset(void) noexcept { g_seq_arm_success_count = 0U; }

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

        const uint32_t spark_abs = engine_deg720_to_absolute(spark_deg, now_raw);
        const uint32_t eoi_abs = engine_deg720_to_absolute(eoi_deg, now_raw);
        const SeqArmAbsTargets t = {
            spark_abs,
            spark_abs - dwell_span,
            eoi_abs,
            eoi_abs - inj_span,
            spark_deg,
        };

        const uint32_t arm_at =
            earlier_abs_target(t.dwell_abs, t.inj_on_abs, now_raw);
        if (!lead_in_arm_window(arm_at, now_raw, min_lead, window)) {
            continue;
        }

        // Janela confirmada — comita só o filme X-τ do último peek (sem
        // re-VE/λ/ΔP/S-curve). PW/avanço armados são os do peek.
        (void)ems::engine::transient_fuel_xtau_commit_last_peek(cyl);
        arm_sequential_cyl(cyl, now_raw, sp, t, min_lead, ms_inter_deg);
        ++g_seq_arm_success_count;
    }
}

// ── Recompute presync — chamado pelo heartbeat TIM2_CH4 quando a fase A/B
// ainda não está confirmada (ecu_sched_encoder_phase_valid()==0, sempre
// verdade sem EMS_MT6835_CMP_PHASE_CALIBRATED — ver board_pinout.h).
// Equivalente a rebuild_presync_revolution() (ecu_sched_angle.cpp) em
// counts: mesma matemática de ângulo (spark/eoi/inj_on/inj_off; semi
// arma os dois bancos a 180°), mas SPARK/EOI vão por engine_deg_to_absolute() (geometria pura)
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
    // g_inj_pw_ticks is one opening (cycle formula / squirts).
    // Do not halve here — dead was already applied per opening.
    const uint32_t raw_inj_pw_ticks = g_inj_pw_ticks;
    uint32_t inj_pw_span = duration_ticks_to_span_counts(raw_inj_pw_ticks);
    // Clamp de duty — paridade com kMaxPresyncInjPwDeg no caminho CKP.
    if (inj_pw_span > kMaxPresyncInjPwCounts) {
        inj_pw_span = kMaxPresyncInjPwCounts;
        ++g_pw_duty_clamp_count;
    }

    const PresyncWastedTargets pwt = presync_wasted_targets();

    uint32_t eoi_target    = engine_deg_to_absolute(pwt.eoi, now_raw);
    uint32_t inj_on_target = eoi_target - inj_pw_span;
    shift_pair_one_rev_if_on_too_soon(inj_on_target, eoi_target, now_raw, min_lead);

    // Wasted-spark: 2 bobinas por evento, pares a 180° — nunca as 4 no
    // mesmo alvo. g_knock_sequential fica 0 (limpo no topo).
    uint32_t ms_inter_deg = 0U;
    if (g_mspark_count > 0U) {
        ms_inter_deg = (duration_ticks_to_span_counts(g_mspark_inter_dwell_ticks)
                        * 360U) / 16384U;
    }
    // Mesmo gate de máscara de ignição do arm_sequential_cyl (achado #3,
    // revisão 2026-08-15) — aplicado aqui também porque recompute_presync()
    // é o caminho REALMENTE ativo hoje (EMS_MT6835_CMP_PHASE_CALIBRATED=0
    // ⇒ phase_valid() nunca fica válido ⇒ try_arm_sequential_due nunca
    // corre em produção); corrigir só o caminho sequencial deixaria o gate
    // sem efeito nenhum na config atual.
    const uint8_t ign_mask_presync = ::ecu_sched_get_ign_inhibit_mask();
    const auto arm_wasted_pair = [&](uint32_t spark_deg, const uint8_t pair[2]) {
        uint32_t spark_target = engine_deg_to_absolute(spark_deg, now_raw);
        uint32_t dwell_target = spark_target - dwell_span;
        shift_pair_one_rev_if_on_too_soon(dwell_target, spark_target, now_raw, min_lead);
        for (uint8_t i = 0U; i < 2U; ++i) {
            const uint8_t bit = (pair[i] < 8U) ? si::k_ign_ch_to_bit[pair[i]] : 0U;
            if (bit != 0U && (ign_mask_presync & bit) != 0U) { continue; }
            arm_channel_with_lead(pair[i], dwell_target, ECU_ACT_DWELL_START, min_lead);
            arm_channel_with_lead(pair[i], spark_target, ECU_ACT_SPARK, min_lead);
        }
        emit_multispark_deg(spark_deg, 360U, ms_inter_deg,
            [&](uint32_t add_dwell_deg, uint32_t add_spark_deg) {
                uint32_t add_dwell_t =
                    engine_deg_to_absolute(add_dwell_deg, now_raw);
                uint32_t add_spark_t =
                    engine_deg_to_absolute(add_spark_deg, now_raw);
                shift_pair_one_rev_if_on_too_soon(add_dwell_t, add_spark_t,
                                                  now_raw, min_lead);
                for (uint8_t i = 0U; i < 2U; ++i) {
                    const uint8_t bit = (pair[i] < 8U) ? si::k_ign_ch_to_bit[pair[i]] : 0U;
                    if (bit != 0U && (ign_mask_presync & bit) != 0U) { continue; }
                    arm_pair_if_lead(pair[i], add_dwell_t, add_spark_t, now_raw,
                                     min_lead, ECU_ACT_DWELL_START, ECU_ACT_SPARK);
                }
            });
    };
    arm_wasted_pair(pwt.spark_a, kWastedIgnPairA);
    arm_wasted_pair(pwt.spark_b, kWastedIgnPairB);

    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
        for (uint8_t i = 0U; i < 4U; ++i) {
            arm_channel_with_lead(kInjCh[i], inj_on_target, ECU_ACT_INJ_ON, min_lead);
            arm_channel_with_lead(kInjCh[i], eoi_target, ECU_ACT_INJ_OFF, min_lead);
        }
    } else {
        // Semi: bank A @ EOI, bank B @ EOI+180°. Two openings / 720°
        // whose widths sum to flow+2×dead.
        for (uint8_t i = 0U; i < 2U; ++i) {
            arm_channel_with_lead(inj_a[i], inj_on_target, ECU_ACT_INJ_ON, min_lead);
            arm_channel_with_lead(inj_a[i], eoi_target, ECU_ACT_INJ_OFF, min_lead);
        }
        uint32_t eoi_b = engine_deg_to_absolute((pwt.eoi + 180U) % 360U, now_raw);
        uint32_t inj_on_b = eoi_b - inj_pw_span;
        shift_pair_one_rev_if_on_too_soon(inj_on_b, eoi_b, now_raw, min_lead);
        for (uint8_t i = 0U; i < 2U; ++i) {
            arm_channel_with_lead(inj_b[i], inj_on_b, ECU_ACT_INJ_ON, min_lead);
            arm_channel_with_lead(inj_b[i], eoi_b, ECU_ACT_INJ_OFF, min_lead);
        }
    }
}

}  // namespace ems::engine::sched_internal::encoder

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
