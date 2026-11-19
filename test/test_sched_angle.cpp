#include "test/harness.h"

#include <cstdint>
#include <cstdio>

#include "drv/ckp.h"
#include "engine/calibration.h"
#include "engine/ecu_sched.h"
#include "engine/ecu_sched_internal.h"
#include "engine/engine_config.h"

using namespace ems::engine;
using namespace ems::engine::sched_internal;

namespace {

int32_t wrap_i(int32_t a, int32_t m) { return ((a % m) + m) % m; }

// Ângulo absoluto (×10) onde a entrada dispara: dente de armamento + offset, na fase certa.
int32_t event_angle_x10(const AngleEvent_t& e, bool sequential) {
    const int32_t base = (sequential && e.phase_A == ECU_PHASE_B) ? 3600 : 0;
    const int32_t cycle = sequential ? 7200 : 3600;
    return wrap_i(base + static_cast<int32_t>(e.tooth_index) * 60 +
                  (static_cast<int32_t>(e.offset_x256) * 60 + 128) / 256, cycle);
}

// 3000 rpm: 20 ms/volta, 60 posições → 333,3 µs por dente.
ems::drv::CkpSnapshot snap_3000(bool phase_a) {
    ems::drv::CkpSnapshot s{};
    s.tooth_period_ns = 333333u;
    s.phase_A = phase_a;
    s.rpm_x10 = 30000u;
    return s;
}

int count_action(uint8_t action, uint8_t ch) {
    int n = 0;
    for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
        if (g_angle_table[i].action == action && g_angle_table[i].channel == ch) { ++n; }
    }
    return n;
}

}  // namespace

void test_sched_angle_tables(void) {
    section("ecu_sched_angle: tabelas de ângulo (sequencial e presync)");

    // Estado de entrada do builder (salvo e restaurado no fim).
    const int32_t  adv0 = g_advance_x10;
    const uint32_t dw0 = g_dwell_ticks, pw0 = g_inj_pw_ticks, eoi0 = g_eoi_lead_deg;
    const uint8_t  pm0 = g_presync_inj_mode;
    const int16_t  fine0 = trigger_fine_x10;
    int16_t retard0[4];
    for (uint8_t i = 0u; i < 4u; ++i) { retard0[i] = g_cyl_retard_x10[i]; g_cyl_retard_x10[i] = 0; }

    g_advance_x10 = 250;                                   // 25,0° BTDC
    g_dwell_ticks = ECU_SCHED_US_TO_TICKS_INTERNAL(3000u); // 3 ms
    g_inj_pw_ticks = ECU_SCHED_US_TO_TICKS_INTERNAL(4000u);// 4 ms
    g_eoi_lead_deg = 60u;
    trigger_fine_x10 = 0;
    const int32_t trig_off = static_cast<int32_t>(cfg::g_eng_cfg.trigger_tooth0_engine_deg) * 10;

    // ── Sequencial ──────────────────────────────────────────────────────────
    rebuild_sequential_cycle(snap_3000(true));
    CHECK_EQ(g_knock_sequential, 1u, "sequencial: knock em modo sequencial");
    CHECK_TRUE(g_angle_table_count >= 12u && g_angle_table_count <= ECU_ANGLE_TABLE_SIZE,
               "12 eventos base (+ duplicatas de janela) cabem na tabela");

    bool teeth_ok = true, masks_ok = true;
    for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
        const AngleEvent_t& e = g_angle_table[i];
        if (e.tooth_index > 57u) { teeth_ok = false; }
        const bool in_mask = (e.tooth_index < 32u)
            ? ((g_angle_tooth_mask_lo >> e.tooth_index) & 1u)
            : ((g_angle_tooth_mask_hi >> (e.tooth_index - 32u)) & 1u);
        if (!in_mask) { masks_ok = false; }
        if (e.phase_A != ECU_PHASE_A && e.phase_A != ECU_PHASE_B) { teeth_ok = false; }
    }
    CHECK_TRUE(teeth_ok, "dentes ≤ 57 e fase A/B definida (nunca ANY em sequencial)");
    CHECK_TRUE(masks_ok, "máscara de dentes cobre todo dente de armamento");

    bool per_cyl = true, spark_exact = true, soi_ok = true;
    for (uint8_t cyl = 0u; cyl < cfg::kCylinderCount; ++cyl) {
        if (count_action(ECU_ACT_SPARK, kIgnCh[cyl]) != 1) { per_cyl = false; }
        if (count_action(ECU_ACT_DWELL_START, kIgnCh[cyl]) < 1) { per_cyl = false; }
        if (count_action(ECU_ACT_INJ_ON, kInjCh[cyl]) != 1) { per_cyl = false; }
        const int32_t tdc = static_cast<int32_t>(cfg::cyl_tdc_deg(cyl)) * 10;
        const int32_t spark_exp = wrap_i(tdc - 250 - trig_off, 7200);
        const int32_t eoi = tdc - 60 * 10 - trig_off;
        // 4 ms a 3000 rpm = 72° de pulso.
        const int32_t soi_exp = wrap_i(eoi - 720, 7200);
        for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
            const AngleEvent_t& e = g_angle_table[i];
            const int32_t a = event_angle_x10(e, true);
            if (e.action == ECU_ACT_SPARK && e.channel == kIgnCh[cyl]) {
                if (wrap_i(a - spark_exp, 7200) > 2 && wrap_i(spark_exp - a, 7200) > 2) { spark_exact = false; }
            }
            if (e.action == ECU_ACT_INJ_ON && e.channel == kInjCh[cyl]) {
                const int32_t d = wrap_i(a - soi_exp, 7200);
                if (d > 4 && d < 7200 - 4) { soi_ok = false; }
            }
        }
    }
    CHECK_TRUE(per_cyl, "cada cilindro: 1 SPARK, ≥1 DWELL_START, 1 INJ_ON");
    CHECK_TRUE(spark_exact, "ângulo reconstruído da centelha = TDC − avanço − offset do gatilho (±0,2°)");
    CHECK_TRUE(soi_ok, "SOI = EOI − pulso em graus (±0,4°)");

    // Dwell começa antes da centelha (arma cedo), nunca depois.
    bool dwell_before = true;
    for (uint8_t cyl = 0u; cyl < cfg::kCylinderCount; ++cyl) {
        int32_t spark_a = -1;
        for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
            if (g_angle_table[i].action == ECU_ACT_SPARK && g_angle_table[i].channel == kIgnCh[cyl]) {
                spark_a = event_angle_x10(g_angle_table[i], true);
            }
        }
        for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
            const AngleEvent_t& e = g_angle_table[i];
            if (e.action != ECU_ACT_DWELL_START || e.channel != kIgnCh[cyl]) { continue; }
            // O DWELL_START carrega o alvo (a centelha); o dente de armamento vem antes dela.
            const int32_t arm = wrap_i(static_cast<int32_t>(e.tooth_index) * 60 +
                                       ((e.phase_A == ECU_PHASE_B) ? 3600 : 0), 7200);
            const int32_t lead = wrap_i(spark_a - arm, 7200);
            if (lead == 0 || lead > 3000 + 60) { dwell_before = false; }
        }
    }
    CHECK_TRUE(dwell_before, "DWELL_START armado antes da centelha, dentro do teto de 300°");

    // Avanço extremo é limitado a −20°..+60° ao montar a tabela.
    g_advance_x10 = 900;                                    // pede 90° → 60°
    rebuild_sequential_cycle(snap_3000(true));
    int32_t a_spark = -1;
    for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
        if (g_angle_table[i].action == ECU_ACT_SPARK && g_angle_table[i].channel == kIgnCh[0]) {
            a_spark = event_angle_x10(g_angle_table[i], true);
        }
    }
    CHECK_TRUE(wrap_i(a_spark - wrap_i(0 - 600 - trig_off, 7200), 7200) <= 2 ||
               wrap_i(wrap_i(0 - 600 - trig_off, 7200) - a_spark, 7200) <= 2,
               "avanço 90° limitado a 60°");

    // Retardo de knock por cilindro atrasa só aquele cilindro.
    g_advance_x10 = 250;
    g_cyl_retard_x10[0] = 50;                               // 5,0° de retardo no cil 0
    rebuild_sequential_cycle(snap_3000(true));
    int32_t s0 = -1, s2 = -1;
    for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
        if (g_angle_table[i].action != ECU_ACT_SPARK) { continue; }
        if (g_angle_table[i].channel == kIgnCh[0]) { s0 = event_angle_x10(g_angle_table[i], true); }
        if (g_angle_table[i].channel == kIgnCh[2]) { s2 = event_angle_x10(g_angle_table[i], true); }
    }
    const int32_t exp0 = wrap_i(0 - 200 - trig_off, 7200);
    const int32_t exp2 = wrap_i(static_cast<int32_t>(cfg::cyl_tdc_deg(2)) * 10 - 250 - trig_off, 7200);
    CHECK_TRUE(wrap_i(s0 - exp0, 7200) <= 2 || wrap_i(exp0 - s0, 7200) <= 2, "knock: cil 0 com 5° de retardo");
    CHECK_TRUE(wrap_i(s2 - exp2, 7200) <= 2 || wrap_i(exp2 - s2, 7200) <= 2, "knock: cil 2 intacto");
    g_cyl_retard_x10[0] = 0;

    // Dente de período zero não divide por zero (dwell/pw em graus = 0).
    ems::drv::CkpSnapshot z = snap_3000(true);
    z.tooth_period_ns = 0u;
    rebuild_sequential_cycle(z);
    CHECK_TRUE(g_angle_table_count >= 12u, "período de dente 0: tabela ainda montada");

    // ── Presync (uma volta, centelha perdida, fase ANY) ─────────────────────
    g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
    rebuild_presync_revolution(snap_3000(true));
    CHECK_EQ(g_knock_sequential, 0u, "presync: knock não sequencial");
    bool any_ok = true, spark_pairs = true;
    for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
        if (g_angle_table[i].phase_A != ECU_PHASE_ANY) { any_ok = false; }
    }
    CHECK_TRUE(any_ok, "presync: todas as entradas valem em qualquer fase");
    for (uint8_t cyl = 0u; cyl < cfg::kCylinderCount; ++cyl) {
        if (count_action(ECU_ACT_SPARK, kIgnCh[cyl]) != 1) { spark_pairs = false; }
    }
    CHECK_TRUE(spark_pairs, "presync: 1 centelha por bobina (waste spark)");
    // Centelhas em cilindros companheiros caem a 360° uma da outra (TDC mod 360).
    int32_t sp[4];
    for (uint8_t cyl = 0u; cyl < 4u; ++cyl) {
        sp[cyl] = -1;
        for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
            if (g_angle_table[i].action == ECU_ACT_SPARK && g_angle_table[i].channel == kIgnCh[cyl]) {
                sp[cyl] = event_angle_x10(g_angle_table[i], false);
            }
        }
    }
    CHECK_TRUE(sp[0] >= 0 && sp[1] >= 0 && sp[2] >= 0 && sp[3] >= 0, "presync: ângulos presentes");
    // Modo simultâneo: todos os injetores no mesmo ângulo (referência cil 0).
    int32_t soi_a = -1;
    bool simult = true;
    for (uint8_t cyl = 0u; cyl < 4u; ++cyl) {
        for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
            if (g_angle_table[i].action == ECU_ACT_INJ_ON && g_angle_table[i].channel == kInjCh[cyl]) {
                const int32_t a = event_angle_x10(g_angle_table[i], false);
                if (soi_a < 0) { soi_a = a; } else if (a != soi_a) { simult = false; }
            }
        }
    }
    CHECK_TRUE(simult, "presync simultâneo: 4 injetores no mesmo ângulo");
    g_presync_inj_mode = ECU_PRESYNC_INJ_SEMI_SEQUENTIAL;
    rebuild_presync_revolution(snap_3000(true));
    int32_t distinct = 0;
    int32_t seen[4] = {-1, -1, -1, -1};
    for (uint8_t cyl = 0u; cyl < 4u; ++cyl) {
        for (uint8_t i = 0u; i < g_angle_table_count; ++i) {
            if (g_angle_table[i].action == ECU_ACT_INJ_ON && g_angle_table[i].channel == kInjCh[cyl]) {
                const int32_t a = event_angle_x10(g_angle_table[i], false);
                bool dup = false;
                for (int32_t k = 0; k < distinct; ++k) { if (seen[k] == a) { dup = true; } }
                if (!dup && distinct < 4) { seen[distinct++] = a; }
            }
        }
    }
    CHECK_TRUE(distinct >= 2, "presync semi-sequencial: injetores em ângulos distintos");

    // ── clear_angle_table ───────────────────────────────────────────────────
    clear_angle_table();
    CHECK_EQ(g_angle_table_count, 0u, "clear: contagem zerada");
    CHECK_TRUE(g_angle_tooth_mask_lo == 0u && g_angle_tooth_mask_hi == 0u, "clear: máscaras zeradas");

    // ── Estouro: sem espaço, descarta e conta ───────────────────────────────
    // 4 cil × 3 eventos + duplicatas ≤ 24. Força a duplicata com dwell enorme.
    const uint32_t drops0 = g_cycle_schedule_drop_count;
    g_dwell_ticks = ECU_SCHED_US_TO_TICKS_INTERNAL(20000u);
    rebuild_sequential_cycle(snap_3000(false));
    CHECK_TRUE(g_angle_table_count <= ECU_ANGLE_TABLE_SIZE, "nunca passa do tamanho da tabela");
    CHECK_TRUE(g_cycle_schedule_drop_count >= drops0, "contador de descartes monotônico");

    // Restaura.
    g_advance_x10 = adv0; g_dwell_ticks = dw0; g_inj_pw_ticks = pw0; g_eoi_lead_deg = eoi0;
    g_presync_inj_mode = pm0; trigger_fine_x10 = fine0;
    for (uint8_t i = 0u; i < 4u; ++i) { g_cyl_retard_x10[i] = retard0[i]; }
    clear_angle_table();
}

void test_sched_inj_pw_pair(void) {
    section("ecu_sched: pulso por abertura segue o modo em vigor (presync <-> sequencial)");

    const uint32_t pw0 = g_inj_pw_ticks, alt0 = g_inj_pw_alt_ticks;
    const uint8_t  ms0 = g_inj_pw_main_seq, ks0 = g_knock_sequential;

    // Sem par (boot, testes antigos): vale sempre o pulso principal.
    g_inj_pw_main_seq = 0xFFu;
    g_inj_pw_ticks = 1000u;
    g_inj_pw_alt_ticks = 600u;
    g_knock_sequential = 0u;
    CHECK_EQ(inj_pw_ticks_now(), 1000u, "sem par, presync: pulso principal");
    g_knock_sequential = 1u;
    CHECK_EQ(inj_pw_ticks_now(), 1000u, "sem par, sequencial: pulso principal");

    // Laço calculou para sequencial (principal) e mandou o de presync como alternativo.
    ecu_sched_set_inj_pw_pair(600u, 1u);
    g_knock_sequential = 1u;
    CHECK_EQ(inj_pw_ticks_now(), 1000u, "sequencial em vigor: pulso sequencial");
    g_knock_sequential = 0u;   // a ISR trocou para presync antes do tick seguinte
    CHECK_EQ(inj_pw_ticks_now(), 600u, "trocou para presync: pulso de presync, não o antigo");

    // Laço calculou para presync (principal) e mandou o sequencial como alternativo.
    g_inj_pw_ticks = 600u;
    ecu_sched_set_inj_pw_pair(1000u, 0u);
    g_knock_sequential = 0u;
    CHECK_EQ(inj_pw_ticks_now(), 600u, "presync em vigor: pulso de presync");
    g_knock_sequential = 1u;   // a ISR trocou para sequencial antes do tick seguinte
    CHECK_EQ(inj_pw_ticks_now(), 1000u, "trocou para sequencial: pulso sequencial");

    // A tabela de ângulos usa o mesmo pulso para recuar o SOI.
    g_knock_sequential = 0u;
    CHECK_EQ(inj_pw_ticks_now(), 600u, "builder de presync enxerga o pulso de presync");

    // Trava de bancada: só o pulso travado vale, o par é ignorado.
    ecu_sched_bench_pw_lock_next_commit();
    ecu_sched_set_inj_pw_pair(123u, 1u);
    g_knock_sequential = 0u;
    CHECK_EQ(inj_pw_ticks_now(), 600u, "trava de bancada: par ignorado");
    ecu_sched_bench_pw_lock_clear();

    // Commit + par numa só chamada (caminho do engine_calc): principal e alternativo chegam juntos,
    // para o modo em que foram calculados; nenhum estado intermediário (modo novo, pulso velho).
    const int32_t  adv1 = g_advance_x10;
    const uint32_t dw1 = g_dwell_ticks, eoi1 = g_eoi_lead_deg;
    g_inj_pw_main_seq = 0u; g_inj_pw_ticks = 600u; g_inj_pw_alt_ticks = 1000u;   // estado antigo: presync
    ecu_sched_commit_calibration_pair_x10(100, 22500u, 18000u, 30u, 9000u, 1u);   // novo: sequencial
    CHECK_EQ(g_inj_pw_ticks, 18000u, "commit+par: pulso principal gravado");
    CHECK_EQ(g_inj_pw_alt_ticks, 9000u, "commit+par: pulso alternativo gravado");
    CHECK_EQ(g_inj_pw_main_seq, 1u, "commit+par: principal calculado para sequencial");
    g_knock_sequential = 1u;
    CHECK_EQ(inj_pw_ticks_now(), 18000u, "commit+par: sequencial em vigor usa o pulso novo");
    g_knock_sequential = 0u;
    CHECK_EQ(inj_pw_ticks_now(), 9000u, "commit+par: presync em vigor usa o alternativo novo");

    // Trava de bancada: o commit+par respeita a trava como o par isolado (só o pulso travado vale).
    ecu_sched_bench_pw_lock_next_commit();
    ecu_sched_commit_calibration_pair_x10(100, 22500u, 7777u, 30u, 3333u, 1u);
    g_knock_sequential = 0u;
    CHECK_EQ(g_inj_pw_main_seq, 0xFFu, "commit+par com trava: par ignorado");
    ecu_sched_bench_pw_lock_clear();
    g_advance_x10 = adv1; g_dwell_ticks = dw1; g_eoi_lead_deg = eoi1;

    g_inj_pw_ticks = pw0; g_inj_pw_alt_ticks = alt0;
    g_inj_pw_main_seq = ms0; g_knock_sequential = ks0;
}

// ── Caracterização do builder da tabela angular ──────────────────────────────────────────────────────
// `ref_angle` é uma CÓPIA CONGELADA do builder de ecu_sched_angle.cpp como estava antes da otimização do
// custo do rebuild (12 µs dentro da ISR do gap). O builder real tem de gerar tabelas BYTE A BYTE iguais a ela
// numa grade de calibrações. NÃO "atualize" esta cópia para acompanhar o código: se o builder mudar de
// comportamento, este teste tem de falhar.
namespace {
namespace ref_angle {

constexpr int32_t kToothX10 = 60;
constexpr int32_t kRevX10 = 3600;
constexpr int32_t kMaxDwellSpanX10 = 3000;
constexpr int32_t kArmLeadX10 = 10;

AngleEvent_t table[ECU_ANGLE_TABLE_SIZE];
uint8_t count = 0U;
uint32_t mask_lo = 0U, mask_hi = 0U;
uint32_t drops = 0U;

int32_t wrap(int32_t a, int32_t cycle) { return ((a % cycle) + cycle) % cycle; }

int32_t ticks_to_x10(uint32_t ticks, uint32_t tooth_ticks)
{
    if (tooth_ticks == 0U) { return 0; }
    return static_cast<int32_t>((static_cast<uint64_t>(ticks) * kToothX10) / tooth_ticks);
}

void add_entry(int32_t arm, int32_t target, int32_t cycle, uint8_t ch, uint8_t action)
{
    if (count >= ECU_ANGLE_TABLE_SIZE) { ++drops; return; }
    arm = wrap(arm, cycle);
    const int32_t in_rev = arm % kRevX10;
    int32_t tooth = in_rev / kToothX10;
    if (tooth > 57) { tooth = 57; }
    const int32_t tooth_ang = arm - in_rev + tooth * kToothX10;
    const int32_t delta = wrap(target - tooth_ang, cycle);

    AngleEvent_t* e = &table[count++];
    e->tooth_index = static_cast<uint8_t>(tooth);
    e->offset_x256 = static_cast<uint16_t>((delta * 256 + kToothX10 / 2) / kToothX10);
    e->phase_A = (cycle == static_cast<int32_t>(kCycleDeg * 10U))
        ? ((arm < kRevX10) ? ECU_PHASE_A : ECU_PHASE_B)
        : ECU_PHASE_ANY;
    e->channel = ch;
    e->action = action;
    if (tooth < 32) { mask_lo |= (1UL << tooth); } else { mask_hi |= (1UL << (tooth - 32)); }
}

void table_add(int32_t arm, int32_t target, int32_t cycle, int32_t now, uint8_t ch, uint8_t action)
{
    add_entry(arm, target, cycle, ch, action);
    if (wrap(target - now, cycle) < wrap(target - arm, cycle)) {
        add_entry(now, target, cycle, ch, action);
    }
}

void add_cylinder(uint8_t cyl, int32_t tdc, int32_t cycle, int32_t now, int32_t inj_ref,
                  int32_t dwell_span, int32_t pw_x10, int32_t trig_off)
{
    int32_t adv = g_advance_x10 + static_cast<int32_t>(cyl_ign_trim_deg[cyl]) * 10 - g_cyl_retard_x10[cyl];
    if (adv > 600) { adv = 600; }
    if (adv < -200) { adv = -200; }
    const int32_t spark = wrap(tdc - adv - trig_off, cycle);
    table_add(spark - dwell_span, spark, cycle, now, kIgnCh[cyl], ECU_ACT_DWELL_START);
    table_add(spark - kArmLeadX10, spark, cycle, now, kIgnCh[cyl], ECU_ACT_SPARK);

    const int32_t eoi = inj_ref - static_cast<int32_t>(g_eoi_lead_deg) * 10 - trig_off;
    const int32_t soi = wrap(eoi - pw_x10, cycle);
    table_add(soi - kArmLeadX10, soi, cycle, now, kInjCh[cyl], ECU_ACT_INJ_ON);
}

void build(const ems::drv::CkpSnapshot& snap, int32_t cycle, bool presync)
{
    const int32_t now = (!presync && !snap.phase_A) ? kRevX10 : 0;
    count = 0U; mask_lo = 0U; mask_hi = 0U;
    const uint32_t tooth_ticks = TOOTH_NS_TO_SCHED_INTERNAL(snap.tooth_period_ns);
    const int32_t dwell_x10 = ticks_to_x10(g_dwell_ticks, tooth_ticks);
    int32_t dwell_span = dwell_x10 + kToothX10 + dwell_x10 / 4;
    if (dwell_span > kMaxDwellSpanX10) { dwell_span = kMaxDwellSpanX10; }
    int32_t pw_x10 = ticks_to_x10(inj_pw_ticks_now(), tooth_ticks);
    if (pw_x10 > cycle * 9 / 10) { pw_x10 = cycle * 9 / 10; }
    const int32_t trig_off =
        static_cast<int32_t>(cfg::g_eng_cfg.trigger_tooth0_engine_deg) * 10 + trigger_fine_x10;
    const bool simultaneous = (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS);

    for (uint8_t cyl = 0U; cyl < cfg::kCylinderCount; ++cyl) {
        int32_t tdc = static_cast<int32_t>(cfg::cyl_tdc_deg(cyl)) * 10;
        if (presync) { tdc %= kRevX10; }
        const int32_t inj_ref = (presync && simultaneous) ? 0 : tdc;
        add_cylinder(cyl, tdc, cycle, now, inj_ref, dwell_span, pw_x10, trig_off);
    }
}

}  // namespace ref_angle
}  // namespace

void test_sched_angle_equivalence(void) {
    section("ecu_sched_angle: o builder gera tabelas byte a byte iguais à referência congelada");

    // Entradas que o builder lê (salvas e restauradas).
    const int32_t  adv0 = g_advance_x10;
    const uint32_t dw0 = g_dwell_ticks, pw0 = g_inj_pw_ticks, alt0 = g_inj_pw_alt_ticks, eoi0 = g_eoi_lead_deg;
    const uint8_t  pm0 = g_presync_inj_mode, ms0 = g_inj_pw_main_seq, ks0 = g_knock_sequential;
    const int16_t  fine0 = trigger_fine_x10;
    const uint16_t trig0 = cfg::g_eng_cfg.trigger_tooth0_engine_deg;
    int16_t retard0[4]; int8_t trim0[4];
    for (uint8_t i = 0u; i < 4u; ++i) { retard0[i] = g_cyl_retard_x10[i]; trim0[i] = cyl_ign_trim_deg[i]; }
    g_inj_pw_main_seq = 0xFFu;   // sem par: vale sempre g_inj_pw_ticks

    const int32_t  advs[]    = {-200, 100, 253, 600};
    const int16_t  retards[2][4] = {{0, 0, 0, 0}, {0, 50, 100, 300}};
    const int8_t   trims[2][4]   = {{0, 0, 0, 0}, {-5, 3, 0, 8}};
    const uint32_t dwells[]  = {25000u, 140625u, 2000000u};
    const uint32_t pws[]     = {5000u, 140625u, 625000u, 6250000u};
    const uint32_t eois[]    = {0u, 355u, 719u};
    const uint16_t trigs[]   = {0u, 636u, 719u};
    const int16_t  fines[]   = {0, -50};
    const uint32_t periods_ns[] = {1250000u, 333333u, 20000u, 0u};
    // 0: sequencial fase A, 1: sequencial fase B, 2: presync simultâneo, 3: presync semi-sequencial
    // 4 x 4 x 2 x 3 x 4 x 3 x 3 x 2 x 4 = 27 648 combinações.
    uint32_t cases = 0u, mismatches = 0u;
    uint8_t max_count = 0u;
    char first[160] = {};

    for (uint8_t mode = 0u; mode < 4u; ++mode)
    for (const int32_t adv : advs)
    for (uint8_t rt = 0u; rt < 2u; ++rt)
    for (const uint32_t dwell : dwells)
    for (const uint32_t pw : pws)
    for (const uint32_t eoi : eois)
    for (const uint16_t trig : trigs)
    for (const int16_t fine : fines)
    for (const uint32_t period : periods_ns) {
        g_advance_x10 = adv; g_dwell_ticks = dwell; g_inj_pw_ticks = pw; g_eoi_lead_deg = eoi;
        trigger_fine_x10 = fine; cfg::g_eng_cfg.trigger_tooth0_engine_deg = trig;
        g_presync_inj_mode = (mode == 2u) ? ECU_PRESYNC_INJ_SIMULTANEOUS : ECU_PRESYNC_INJ_SEMI_SEQUENTIAL;
        for (uint8_t i = 0u; i < 4u; ++i) { g_cyl_retard_x10[i] = retards[rt][i]; cyl_ign_trim_deg[i] = trims[rt][i]; }
        ems::drv::CkpSnapshot snap{};
        snap.tooth_period_ns = period;
        snap.phase_A = (mode != 1u);

        const uint32_t drops0 = ecu_sched_test_get_cycle_schedule_drop_count();
        if (mode <= 1u) { rebuild_sequential_cycle(snap); } else { rebuild_presync_revolution(snap); }
        const uint32_t real_drops = ecu_sched_test_get_cycle_schedule_drop_count() - drops0;

        ref_angle::drops = 0u;
        ref_angle::build(snap, mode <= 1u ? 7200 : 3600, mode > 1u);   // g_knock_sequential já foi posto pelo rebuild

        bool same = (g_angle_table_count == ref_angle::count) &&
                    (g_angle_tooth_mask_lo == ref_angle::mask_lo) &&
                    (g_angle_tooth_mask_hi == ref_angle::mask_hi) &&
                    (real_drops == ref_angle::drops);
        for (uint8_t i = 0u; same && i < ref_angle::count; ++i) {
            const AngleEvent_t& a = g_angle_table[i];
            const AngleEvent_t& b = ref_angle::table[i];
            same = a.tooth_index == b.tooth_index && a.offset_x256 == b.offset_x256 && a.phase_A == b.phase_A &&
                   a.channel == b.channel && a.action == b.action;
        }
        ++cases;
        if (ref_angle::count > max_count) { max_count = ref_angle::count; }
        if (!same) {
            if (mismatches == 0u) {
                std::snprintf(first, sizeof(first), "mode=%u adv=%d dwell=%u pw=%u eoi=%u trig=%u fine=%d per=%u cnt %u/%u",
                              mode, adv, dwell, pw, eoi, trig, fine, period, g_angle_table_count, ref_angle::count);
            }
            ++mismatches;
        }
    }

    if (mismatches != 0u) { std::printf("  primeira divergência: %s\n", first); }
    CHECK_TRUE(cases > 3000u, "a grade exercita milhares de combinações");
    CHECK_EQ(mismatches, 0u, "tabelas idênticas à referência em todas as combinações");
    // Folga da tabela: com 24 posições o overflow não é alcançável (máximo = 12 entradas + até 12 cópias "now").
    CHECK_TRUE(max_count >= 12u && max_count <= ECU_ANGLE_TABLE_SIZE, "o máximo de entradas observado cabe na tabela");

    g_advance_x10 = adv0; g_dwell_ticks = dw0; g_inj_pw_ticks = pw0; g_inj_pw_alt_ticks = alt0; g_eoi_lead_deg = eoi0;
    g_presync_inj_mode = pm0; g_inj_pw_main_seq = ms0; g_knock_sequential = ks0;
    trigger_fine_x10 = fine0; cfg::g_eng_cfg.trigger_tooth0_engine_deg = trig0;
    for (uint8_t i = 0u; i < 4u; ++i) { g_cyl_retard_x10[i] = retard0[i]; cyl_ign_trim_deg[i] = trim0[i]; }
    clear_angle_table();
}
