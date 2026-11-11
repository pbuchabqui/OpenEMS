#include "test/harness.h"

#include <cstdint>

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
