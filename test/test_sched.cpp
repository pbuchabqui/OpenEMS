#include "test/harness.h"
#include "test/fixtures.h"
#include "test/ui_helpers.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "engine/etb_control.h"
#include "hal/etb_driver.h"
#include "engine/torque_manager.h"
#include "engine/calibration.h"
#include "app/can_rx_map.h"
#include "hal/adc.h"
#include "hal/system.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/auxiliaries.h"
#include "engine/knock.h"
#include "engine/table3d.h"
#include "engine/ecu_sched.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "engine/output_test.h"
#include "engine/engine_config.h"
#include "hal/timer.h"
#include "hal/flash.h"
#include "app/ui_protocol.h"
#include "app/status_bits.h"
#include "hal/crc32.h"

namespace ems::engine {
    int16_t etb_get_idle_spark_trim() noexcept;
}

extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

using namespace ems::drv;
using namespace ems::engine;
using namespace ems::app;
using namespace ems::hal;

void test_ecu_sched_setters(void) {
    section("ecu_sched: reset / setters / getters");
    ecu_sched_test_reset();

    // Defaults after reset: advance=10, dwell=140625, inj_pw=140625, eoi=355
    CHECK_EQ(ecu_sched_test_get_advance_deg(),  10u, "default advance=10°");
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(), 140625u, "default dwell=140625");
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 140625u, "default inj_pw=140625");
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 355u, "default eoi=355° (open-valve)");

    // Individual setters
    ecu_sched_set_advance_deg(20u);
    CHECK_EQ(ecu_sched_test_get_advance_deg(), 20u, "set_advance_deg(20)");

    ecu_sched_set_dwell_ticks(30000u);
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(), 30000u, "set_dwell_ticks(187500)");

    ecu_sched_set_inj_pw_ticks(15000u);
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 15000u, "set_inj_pw_ticks(15000)");

    ecu_sched_set_eoi_lead_deg(50u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 50u, "set_eoi_lead_deg(50)");

    // commit_calibration sets all four atomically
    ecu_sched_commit_calibration(25u, 25000u, 18000u, 55u);
    CHECK_EQ(ecu_sched_test_get_advance_deg(),   25u, "commit: advance=25");
    CHECK_EQ(ecu_sched_test_get_dwell_ticks(),  25000u, "commit: dwell=25000");
    CHECK_EQ(ecu_sched_test_get_inj_pw_ticks(), 18000u, "commit: inj_pw=18000");
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(),  55u, "commit: eoi=55");

    // Calibration clamp: advance > 719 → clamped
    ecu_sched_set_advance_deg(800u);
    CHECK_TRUE(ecu_sched_test_get_advance_deg() <= 719u, "advance > 720 → clamped");
    CHECK_EQ(ecu_sched_test_get_calibration_clamp_count(), 1u, "clamp count=1");

    // reset_diagnostic_counters
    ecu_sched_reset_diagnostic_counters();
    CHECK_EQ(ecu_sched_test_get_calibration_clamp_count(), 0u, "clamp_count=0 after reset");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), 0u, "late_count=0 after reset");
}

void test_ecu_sched_angle_table(void) {
    section("ecu_sched: schedule_on_tooth populates angle table in FULL_SYNC");
    ecu_sched_test_reset();
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);

    // Reach full sync — schedule_on_tooth fires each CKP tooth hook
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // After gap tooth, scheduler should have emitted events for 4 cylinders
    const uint8_t tbl_sz = ecu_sched_test_angle_table_size();
    CHECK_TRUE(tbl_sz > 0u, "angle table has events after FULL_SYNC");
    // Modo de presync default é SEMI_SEQUENTIAL (g_presync_inj_mode em
    // ecu_sched_test_reset()), não SIMULTANEOUS: injeta em só 2 dos 4
    // cilindros por revolução em presync. Ignição continua 1/cilindro
    // (DWELL+SPARK = 4×2 = 8), injeção só 2 cilindros (ON+OFF = 2×2 = 4) →
    // 12 eventos, não os 16 que a expectativa antiga assumia (modo
    // SIMULTANEOUS, que só é ativado automaticamente durante cranking).
    CHECK_TRUE(tbl_sz >= 12u, "angle table has ≥12 events (presync SEMI_SEQUENTIAL)");

    // Inspect first valid event: should be one of ECU_ACT_*
    uint8_t tooth, frac, ch, action, phase;
    const uint8_t ok = ecu_sched_test_get_angle_event(0u, &tooth, &frac, &ch, &action, &phase);
    CHECK_EQ(ok, 1u, "event[0] is valid");
    CHECK_TRUE(action <= ECU_ACT_SPARK, "action in [0,3] (DWELL/SPARK/INJ)");
    CHECK_TRUE(tooth < 58u, "tooth_index < kRealTeeth60_2");
}

// ── Transição wasted-spark ↔ sequencial via bordas CMP reais ────────────────
// Ao contrário dos outros testes sequenciais (que dão bypass ao gate com
// ckp_test_set_cmp_confirms(2u)), este conduz o caminho NUNCA testado: a
// validação temporal em ckp_tim5_ch2_isr via cam_fire(). Reproduz o sintoma
// de bancada ("ao ligar o CMP nada muda") com entradas ideais.
//
// Mecânica do timing: cada ckp_feed_n_then_gap(55) avança g_ckp_cap por
// 58×período (55 normais + gap 3×período). CMP dispara 1×/720° = 2 revs =
// 116×período, que é exactamente expected = 2×kRealTeethPerRev(58)×período em
// ckp_tim5_ch2_isr → a 2ª borda cai no centro da janela ±25%.
void test_ecu_sched_wasted_to_sequential(void) {
    section("ecu_sched: transição wasted→sequencial via CMP real (cam_fire)");
    ecu_sched_test_reset();
    // Janela de dente CMP desabilitada (default), defensivo contra herança de
    // estado de testes anteriores no mesmo processo.
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;

    // 1) FULL_SYNC sem CMP → wasted-spark (presync).
    ckp_reach_full_sync();                       // reseta g_ckp_cap=0, dirige o hook
    ckp_feed_n_then_gap(55u);                     // 1 rev extra em wasted
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "cmp_confirms=0 sem CMP");
    CHECK_TRUE(ecu_sched_test_get_presync_revs() > 0u, "presync_revs>0 (wasted a correr)");
    CHECK_EQ(ecu_sched_test_get_seq_revs(), 0u, "seq_revs=0 (ainda não sequencial)");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "is_sequential=0 sem CMP");

    // 2) 1ª borda CMP: só arma s_prev (confirms=0). 2ª: 1.º confirm (ainda <2).
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "1ª borda CMP só arma timestamp");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);                     // +2 revs → g_ckp_cap += 116×período
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 1u, "2ª borda → cmp_confirms=1");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "1 confirm insuficiente: continua wasted");

    // 3) 3ª borda CMP: delta = 116×período → cmp_confirms=2.
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "3ª borda coerente → cmp_confirms=2");
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "nenhuma borda CMP rejeitada");

    // 4) Próximas fronteiras de revolução → gate abre → Calculate_Sequential_Cycle.
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    CHECK_TRUE(ecu_sched_test_get_seq_revs() > 0u, "seq_revs>0 após CMP confirmado");
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "is_sequential=1: entrou em sequencial");

    // 5) Fallback CMP-ausente (Parte C-#2): mantendo o sincronismo mas SEM novas
    //    bordas de came, o contador de revoluções desde a última borda ultrapassa
    //    kMaxRevsWithoutCmp (6 em produção; 60 só em bench-mode para tolerar
    //    gaps do estimulador RMT) → cmp_confirms zera → o agendador reverte a wasted.
    for (uint32_t i = 0; i < 7u; ++i) { ckp_feed_n_then_gap(55u); }
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "sem came >6 revs → cmp_confirms zerado");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "fallback: reverteu a wasted-spark");
}

// ── Revalidação CMP após perda de sync do CKP ───────────────────────────────
// Perda de sync zera cmp_confirms (interno + export): exige 2 bordas CMP
// frescas antes de reabrir sequencial (evita 1 borda reabrir com fase velha).
void test_ecu_sched_cmp_revalidation_after_sync_loss(void) {
    section("ecu_sched: perda de sync fecha gate sequencial até 2 bordas CMP frescas");
    ecu_sched_test_reset();
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;

    // 1) FULL_SYNC + arm + 2 confirms → sequencial.
    ckp_reach_full_sync();
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                          // arma s_prev
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                          // confirms=1
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                          // confirms=2
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "3 bordas (arm+2) → cmp_confirms=2");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "sequencial activo antes da perda");

    // 2) Gap prematuro → LOSS; confirms zerados (ambos os contadores).
    ckp_feed_n_then_gap(25u);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "gap prematuro → LOSS_OF_SYNC");
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "perda de sync: cmp_confirms=0");

    // 3) Resync SEM came fresco → wasted.
    ckp_feed_n_then_gap(55u);                     // LOSS → HALF_SYNC
    ckp_feed_n_then_gap(55u);                     // HALF → FULL_SYNC
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "resync completo");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "pós-resync sem came fresco: fica em wasted");

    // 4) LOSS limpou s_prev → 1ª borda arma, 2ª/3ª confirmam.
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "1ª borda pós-LOSS só arma");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 1u, "2ª borda fresca → confirms=1");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "3ª borda fresca → confirms=2");
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "sequencial retomado após revalidação");
}

// Ruído no pino CMP (came desligado, PA1 a flutuar) gera bordas fantasma em
// posições de virabrequim ALEATÓRIAS. O gate de consistência de posição
// (ckp_tim5_ch2_isr) exige que bordas consecutivas ocorram no mesmo tooth_index:
// ruído nunca junta 2 coerentes → cmp_confirms não chega a 2 → fica em wasted.
void test_ecu_sched_noise_rejects_sequential(void) {
    section("ecu_sched: ruído CMP (posição inconsistente) NÃO entra em sequencial");
    ecu_sched_test_reset();
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;
    ckp_reach_full_sync();                                   // FULL_SYNC, tooth 0

    // Borda 1 no dente 5 → só arma s_prev (confirms=0).
    for (uint32_t i = 0; i < 5u; ++i) { ckp_fire(kNormalPeriod); }
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "1ª borda só arma timestamp");

    // Borda 2 ~2 revs no mesmo dente 5 → 1.º confirm (ancora posição).
    for (uint32_t i = 0; i < 50u; ++i) { ckp_fire(kNormalPeriod); } ckp_fire(kNormalPeriod * 3u);
    ckp_feed_n_then_gap(55u);
    for (uint32_t i = 0; i < 5u; ++i) { ckp_fire(kNormalPeriod); }
    cam_fire(g_ckp_cap);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 1u, "2ª borda coerente → confirms=1");

    // Borda 3 noutro dente (25≠5): passa temporal, falha posição → confirms→0.
    for (uint32_t i = 0; i < 50u; ++i) { ckp_fire(kNormalPeriod); } ckp_fire(kNormalPeriod * 3u);
    ckp_feed_n_then_gap(55u);
    for (uint32_t i = 0; i < 25u; ++i) { ckp_fire(kNormalPeriod); }   // tooth 25
    cam_fire(g_ckp_cap);
    CHECK_TRUE(ckp_snapshot().cmp_confirms < 2u, "borda em dente inconsistente não confirma");

    // Mais bordas em dentes sempre diferentes → nunca acumula 2 coerentes.
    const uint8_t teeth[3] = {40u, 12u, 33u};
    for (uint8_t k = 0; k < 3u; ++k) {
        for (uint32_t i = 0; i < 30u; ++i) { ckp_fire(kNormalPeriod); } ckp_fire(kNormalPeriod * 3u);
        ckp_feed_n_then_gap(55u);
        for (uint32_t i = 0; i < teeth[k]; ++i) { ckp_fire(kNormalPeriod); }
        cam_fire(g_ckp_cap);
    }
    CHECK_TRUE(ckp_snapshot().cmp_confirms < 2u, "ruído nunca atinge cmp_confirms=2");
    ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "permanece em wasted-spark sob ruído CMP");
}

// Após fallback a wasted (came ausente), o came RECONECTADO tem de recuperar o
// sequencial. Reproduz o deadlock: s_prev_cmp_capture fica obsoleto (borda real de
// há muitas revs) e cada borda reconectada é rejeitada por tempo contra ele. O
// resync por rejeições consecutivas (kCmpRejectResync) larga a referência e recupera.
void test_ecu_sched_recovers_after_fallback(void) {
    section("ecu_sched: recupera sequencial após fallback (came reconectado)");
    ecu_sched_test_reset();
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;
    ckp_reach_full_sync();

    // Entra em sequencial: arm + 2 confirms no tooth 0.
    cam_fire(g_ckp_cap);                                      // arm
    ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                                      // confirms=1
    ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);                                      // confirms=2
    ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "pré: sequencial (cmp_confirms=2)");
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "pré: is_sequential=1");

    // "Desconecta": >60 revs sem came (kMaxRevsWithoutCmp) → #2 fallback → wasted.
    // s_prev fica obsoleto.
    for (uint32_t i = 0; i < 61u; ++i) { ckp_feed_n_then_gap(55u); }
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "fallback: cmp_confirms=0");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "fallback: wasted");

    // "Reconecta": bordas coerentes no tooth 0. Sem o resync, todas seriam rejeitadas
    // por tempo contra o s_prev obsoleto (deadlock). Com o resync, recupera.
    for (uint8_t e = 0; e < 6u; ++e) {
        cam_fire(g_ckp_cap);
        ckp_feed_n_then_gap(55u); ckp_feed_n_then_gap(55u);
    }
    CHECK_EQ(ckp_snapshot().cmp_confirms, 2u, "recuperou: cmp_confirms=2 após reconexão");
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "recuperou: voltou a sequencial");
}

void test_ecu_sched_inhibit_masks(void) {
    section("ecu_sched: injection / ignition inhibit masks");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0u, "inj_inhibit=0 after reset");
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0u, "ign_inhibit=0 after reset");

    ecu_sched_set_inj_inhibit_mask(0x05u);  // cylinders 0 and 2
    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0x05u, "inj_inhibit=0x05");

    ecu_sched_set_ign_inhibit_mask(0x0Au);  // cylinders 1 and 3
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0x0Au, "ign_inhibit=0x0A");

    // Restore
    ecu_sched_set_inj_inhibit_mask(0u);
    ecu_sched_set_ign_inhibit_mask(0u);
    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0u, "inj_inhibit cleared");
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0u, "ign_inhibit cleared");

    section("ecu_sched: prime cannot bypass inj inhibit mask");
    {
        ecu_sched_test_reset();
        uint32_t pins[24] = {};
        ecu_sched_get_pin_counts_u32x24(pins);
        const uint32_t h0 = pins[0];   // INJ1 high count
        const uint32_t h1 = pins[3];   // INJ2 (idx 1 → offset 3)
        const uint32_t h2 = pins[6];
        const uint32_t h3 = pins[9];

        ecu_sched_set_inj_inhibit_mask(0x0Fu);
        ecu_sched_fire_prime_pulse(5000u);
        ecu_sched_get_pin_counts_u32x24(pins);
        CHECK_EQ(pins[0], h0, "prime+mask: INJ1 high count unchanged");
        CHECK_EQ(pins[3], h1, "prime+mask: INJ2 high count unchanged");
        CHECK_EQ(pins[6], h2, "prime+mask: INJ3 high count unchanged");
        CHECK_EQ(pins[9], h3, "prime+mask: INJ4 high count unchanged");

        ecu_sched_set_inj_inhibit_mask(0u);
        ecu_sched_fire_prime_pulse(5000u);
        ecu_sched_get_pin_counts_u32x24(pins);
        CHECK_TRUE(pins[0] > h0, "prime unmasked: INJ1 high count increases");
        CHECK_TRUE(pins[3] > h1, "prime unmasked: INJ2 high count increases");
        CHECK_TRUE(pins[6] > h2, "prime unmasked: INJ3 high count increases");
        CHECK_TRUE(pins[9] > h3, "prime unmasked: INJ4 high count increases");
    }
}

void test_ecu_sched_mspark(void) {
    section("ecu_sched: multi-spark");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_test_get_mspark_count(), 0u, "mspark=0 after reset");

    ecu_sched_set_mspark(2u, 5000u, 18u);
    CHECK_EQ(ecu_sched_test_get_mspark_count(), 2u, "mspark_count=2");

    // Overflow: count > 3 → clamped to 3
    ecu_sched_set_mspark(5u, 5000u, 18u);
    CHECK_TRUE(ecu_sched_test_get_mspark_count() <= 3u, "mspark_count clamped ≤3");

    // Disable
    ecu_sched_set_mspark(0u, 0u, 0u);
    CHECK_EQ(ecu_sched_test_get_mspark_count(), 0u, "mspark disabled");

    // Hard RPM ceiling for multi-spark gate (firmware policy)
    CHECK_EQ(ems::engine::kMsparkRpmCeilingX10, 15000u, "mspark ceiling = 1500 RPM");
    CHECK_TRUE(ems::engine::mspark_max_rpm_x10 <= ems::engine::kMsparkRpmCeilingX10,
               "default mspark gate ≤ 1500 RPM");
}

// ── EOI targeting ───────────────────────────────────────────────────────────
// Helper: procura na tabela angular o primeiro evento (channel, action).
uint8_t find_angle_event(uint8_t want_ch, uint8_t want_act,
                                uint8_t *out_tooth, uint8_t *out_frac, uint8_t *out_phase) {
    for (uint8_t i = 0u; i < ecu_sched_test_angle_table_size(); ++i) {
        uint8_t t, f, ch, act, ph;
        if (ecu_sched_test_get_angle_event(i, &t, &f, &ch, &act, &ph) != 0u &&
            ch == want_ch && act == want_act) {
            *out_tooth = t; *out_frac = f; *out_phase = ph;
            return 1u;
        }
    }
    return 0u;
}

// Constrói tabela sequencial com o PW dado e devolve eventos INJ1 ON/OFF.
// kNormalPeriod=10000 ticks → tooth_period=160000ns → tooth_ticks=10000
// → deg = ticks×6/10000 (720°); RPM ≈ 6250.
void build_seq_table_with_pw(uint32_t pw_ticks) {
    ecu_sched_test_reset();
    for (uint8_t i = 0u; i < 4u; ++i) { ems::engine::cyl_fuel_trim_pct[i] = 0; }
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(pw_ticks);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    ckp_test_set_cmp_confirms(2u);
    ckp_feed_n_then_gap(55u);  // gap → Calculate_Sequential_Cycle
}

void test_ecu_sched_eoi_targeting(void) {
    // Cyl0: tdc=0, eoi_lead=60 → EOI=660° (60° BTDC combustão).
    // Trigger frame (offset=0): ang=660%360=300 → 300×256/6=12800 → tooth=50,
    // frac=0, PHASE_B (660≥360). O INJ_OFF deve ficar AQUI para qualquer PW.
    uint8_t t_on, f_on, p_on, t_off, f_off, p_off;

    section("ecu_sched EOI: INJ_OFF fixo no alvo de EOI, independente do PW");
    // PW curto: 120° → ticks = 120×10000/6 = 200000. SOI = 660−120 = 540°
    // → ang=180 → 180×256/6=7680 → tooth=30, frac=0, PHASE_B.
    build_seq_table_with_pw(200000u);
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "PW=120°: INJ1 OFF presente");
    CHECK_EQ(t_off, 50u, "PW=120°: INJ_OFF em tooth 50 (EOI=660°)");
    CHECK_EQ(f_off, 0u,  "PW=120°: INJ_OFF frac=0");
    CHECK_EQ(p_off, ECU_PHASE_B, "PW=120°: INJ_OFF em PHASE_B");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "PW=120°: INJ1 ON presente");
    CHECK_EQ(t_on, 30u, "PW=120°: SOI recua para tooth 30 (540°)");
    CHECK_EQ(p_on, ECU_PHASE_B, "PW=120°: SOI em PHASE_B");
    CHECK_EQ(ecu_sched_test_get_pw_duty_clamp_count(), 0u,
             "PW=120°: duty clamp não dispara");

    section("ecu_sched EOI: SOI cruza a origem do ciclo (wrap) mantendo o EOI");
    // Com eoi_lead=60 e duty clamp de 648°, SOI = 660−PW ∈ [12,660] — nunca
    // cruza a origem. O wrap real ocorre com EOI mais cedo no ciclo:
    // eoi_lead=300 → EOI=420° → ang=60 → tooth=10, frac=0, PHASE_B.
    // PW: ticks=833333 → deg=499 (trunc). SOI=(420+720−499)%720=641°
    // → ang=281 → 281×256/6=11989 → tooth=46, frac=213, PHASE_B.
    // SOI (641°) fica DEPOIS do EOI (420°) em ângulo absoluto: o pulso
    // atravessa a fronteira 720→0 do ciclo — o caso que o SOI fixo não cobria.
    ecu_sched_test_reset();
    for (uint8_t i = 0u; i < 4u; ++i) { ems::engine::cyl_fuel_trim_pct[i] = 0; }
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(833333u);
    ecu_sched_set_eoi_lead_deg(300u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // ckp_reach_full_sync() já constrói uma tabela de presync (cmp_confirms
    // ainda a 0 nesse ponto) com este mesmo inj_pw_ticks — em modo presync o
    // clamp actua a 324° (ECU_MAX_PRESYNC_INJ_PW_DEG, janela de 360°) e o
    // PW de 499° dispara-o aí, incrementando g_pw_duty_clamp_count antes da
    // build sequencial que este teste quer isolar. Reset após confirmar CMP.
    ckp_test_set_cmp_confirms(2u);
    ecu_sched_reset_diagnostic_counters();
    ckp_feed_n_then_gap(55u);
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "PW=499°/EOI=420°: INJ1 OFF presente");
    CHECK_EQ(t_off, 10u, "PW=499°: INJ_OFF em tooth 10 (EOI=420° fixo)");
    CHECK_EQ(p_off, ECU_PHASE_B, "PW=499°: INJ_OFF em PHASE_B");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "PW=499°: INJ1 ON presente");
    CHECK_EQ(t_on, 46u, "PW=499°: SOI em tooth 46 (641° — wrap além da origem)");
    CHECK_EQ(p_on, ECU_PHASE_B, "PW=499°: SOI em PHASE_B");
    CHECK_EQ(ecu_sched_test_get_pw_duty_clamp_count(), 0u,
             "PW=499° ≤ 648°: duty clamp não dispara");

    section("ecu_sched EOI: duty clamp em PW ≥ 90% do ciclo");
    // PW máximo permitido: ticks=1250000 → deg=750 > 648 → clamp a 648°.
    // SOI=(660+720−648)%720=732%720=12° → ang=12 → 512 → tooth=2, frac=0,
    // PHASE_A. Contador: 4 incrementos (um por cilindro na build).
    build_seq_table_with_pw(1250000u);
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "PW=750°: INJ1 OFF presente");
    CHECK_EQ(t_off, 50u, "PW=750°: INJ_OFF permanece em tooth 50 (EOI fixo)");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "PW=750°: INJ1 ON presente");
    CHECK_EQ(t_on, 2u,  "PW=750°→648°: SOI em tooth 2 (12°)");
    CHECK_EQ(p_on, ECU_PHASE_A, "PW=750°→648°: SOI em PHASE_A");
    // Contador: ≥4 (um por cilindro na build sequencial). Nota: as builds
    // presync durante o padrão de sync (modo default SEMI_SEQUENTIAL, sem
    // halving de PW: 750° > 324°) também incrementam, pelo que a contagem
    // exacta depende do número de rev boundaries.
    CHECK_TRUE(ecu_sched_test_get_pw_duty_clamp_count() >= 4u,
             "PW=750°: duty clamp disparou ≥4× (1× por cilindro na build seq.)");

    section("ecu_sched EOI: presync usa EOI targeting na janela de 360°");
    // presync: eoi=(360−60)%360=300 → tooth 50, frac 0, PHASE_ANY.
    ecu_sched_test_reset();
    ecu_sched_test_set_tim1_cnt(0u);
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(55u);   // HALF_SYNC
    // Gap rev boundary (no phantom wrap): FULL_SYNC without CMP → presync table
    for (uint32_t i = 0u; i < 55u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kGapPeriod);
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "presync: INJ1 OFF presente");
    CHECK_EQ(t_off, 50u, "presync: INJ_OFF em tooth 50 (EOI=300° na janela 360°)");
    CHECK_EQ(p_off, ECU_PHASE_ANY, "presync: INJ_OFF em PHASE_ANY");

    section("ecu_sched EOI: default 355° — pulso presync cruza a fronteira de rev");
    // Com o default open-valve (eoi_lead=355), o EOI presync cai a
    // eoi=(360−355%360)%360=5° → ang=5 → 5×256/6=213 → tooth 0, frac 213.
    // Modo presync default é SEMI_SEQUENTIAL (ecu_sched_test_reset()) — SEM
    // halving do PW (isso só acontece em SIMULTANEOUS): PW=125000 ticks → 75°
    // → SOI=(5+360−75)%360=290° → 290×256/6=12373 → tooth 48, frac 85.
    // O pulso ON(tooth 48, rev N) → OFF(tooth 0, rev N+1) CRUZA a fronteira de
    // revolução onde a tabela é reconstruída — este check fixa que ambos os
    // eventos existem na tabela (o OFF da tabela nova fecha o injetor aberto
    // na rev anterior; toggle de bancos já validado acima).
    ecu_sched_test_reset();  // usa o default eoi_lead=355 — sem set explícito
    ecu_sched_test_set_tim1_cnt(0u);
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(55u);   // HALF_SYNC
    for (uint32_t i = 0u; i < 55u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kGapPeriod);  // gap rev boundary → presync table (no CMP)
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_OFF, &t_off, &f_off, &p_off), 1u,
             "presync 355°: INJ1 OFF presente");
    CHECK_EQ(t_off, 0u,  "presync 355°: INJ_OFF em tooth 0 (EOI=5°)");
    CHECK_EQ(f_off, 213u, "presync 355°: INJ_OFF frac=213 (5°×256/6)");
    CHECK_EQ(find_angle_event(ECU_CH_INJ1, ECU_ACT_INJ_ON, &t_on, &f_on, &p_on), 1u,
             "presync 355°: INJ1 ON presente");
    CHECK_EQ(t_on, 48u, "presync 355°: SOI em tooth 48 (290°) — antes da fronteira");
    CHECK_EQ(f_on, 85u, "presync 355°: SOI frac=85");
}

void test_eoi_blend(void) {
    section("fuel_calc: EOI blend de 2 pontos por RPM");
    // main = g_eng_cfg.default_eoi_lead_deg (355 por default de compilação)
    const uint16_t saved_main = ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg;
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = 355u;

    // Desligado (0/0 — page 0 antiga zerada): devolve sempre o main
    ems::engine::eoi_idle_deg = 60u;
    ems::engine::eoi_blend_rpm_lo = 0u;
    ems::engine::eoi_blend_rpm_hi = 0u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u), 355u, "blend off (0/0): main a 850 RPM");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(85000u), 355u, "blend off (0/0): main a 8500 RPM");

    // Desligado (hi < lo): gate contra janela invertida / divisão por zero
    ems::engine::eoi_blend_rpm_lo = 2500u;
    ems::engine::eoi_blend_rpm_hi = 1500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 355u, "blend off (hi<lo): main");
    ems::engine::eoi_blend_rpm_hi = 2500u;  // hi == lo também desliga
    ems::engine::eoi_blend_rpm_lo = 2500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 355u, "blend off (hi==lo): main");

    // Janela 1500→2500, idle=60, main=355 (ascendente)
    ems::engine::eoi_blend_rpm_lo = 1500u;
    ems::engine::eoi_blend_rpm_hi = 2500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u),  60u, "850 RPM (< lo): idle");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(15000u), 60u, "1500 RPM (== lo): idle");
    // 2000 RPM: 60 + 295×500/1000 = 60 + 147 = 207 (trunc)
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 207u, "2000 RPM (meio): 207");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(25000u), 355u, "2500 RPM (== hi): main");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(85000u), 355u, "8500 RPM (> hi): main");

    // Descendente (idle=365 pré-IVO > main=355): interpola para baixo
    ems::engine::eoi_idle_deg = 365u;
    // 2000 RPM: 365 + (−10)×500/1000 = 365 − 5 = 360
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(20000u), 360u, "descendente: 365→355 dá 360 no meio");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u),  365u, "descendente: idle=365 abaixo da janela");

    // Extremos int32: idle=0, main=719, janela de 1 RPM
    ems::engine::eoi_idle_deg = 0u;
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = 719u;
    ems::engine::eoi_blend_rpm_lo = 1000u;
    ems::engine::eoi_blend_rpm_hi = 1001u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(10000u), 0u,   "janela 1 RPM: == lo → idle");
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(10010u), 719u, "janela 1 RPM: == hi → main");

    // Entradas fora de gama são clampadas a 719 (defesa antes do sanitize)
    ems::engine::eoi_idle_deg = 60000u;
    ems::engine::eoi_blend_rpm_lo = 1500u;
    ems::engine::eoi_blend_rpm_hi = 2500u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(8500u), 719u, "idle fora de gama → clamp 719");
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = 60000u;
    CHECK_EQ(ems::engine::calc_eoi_lead_deg(85000u), 719u, "main fora de gama → clamp 719");

    // restaurar estado partilhado
    ems::engine::cfg::g_eng_cfg.default_eoi_lead_deg = saved_main;
    ems::engine::eoi_idle_deg = 60u;
    ems::engine::eoi_blend_rpm_lo = 0u;
    ems::engine::eoi_blend_rpm_hi = 0u;

    section("ecu_sched: sanitize aceita EOI até 719 (pré-IVO)");
    ecu_sched_test_reset();
    ecu_sched_set_eoi_lead_deg(719u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 719u, "eoi=719 aceite (clamp estendido)");
    ecu_sched_set_eoi_lead_deg(365u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 365u, "eoi=365 (pré-IVO) aceite");
    ecu_sched_set_eoi_lead_deg(720u);
    CHECK_EQ(ecu_sched_test_get_eoi_lead_deg(), 719u, "eoi=720 clampado a 719");
}

void test_ecu_sched_presync(void) {
    section("ecu_sched: presync enable/mode setters");
    ecu_sched_test_reset();

    // Just verify no crash
    ecu_sched_set_presync_enable(0u);
    ecu_sched_set_presync_enable(1u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SEMI_SEQUENTIAL);
    ecu_sched_set_presync_ign_mode(ECU_PRESYNC_IGN_WASTED_SPARK);
    ecu_sched_fire_prime_pulse(5000u);  // prime pulse: no crash with valid pw
    CHECK_TRUE(true, "presync setters and prime_pulse 5000: no crash");

    // ecu_sched_fire_prime_pulse edge cases:
    // pw=0 → guard: early return (no crash)
    ecu_sched_fire_prime_pulse(0u);
    CHECK_TRUE(true, "fire_prime_pulse(0): no crash (early return)");

    // pw > 30000 → clamped to 30000 (no crash, clamp happens internally)
    ecu_sched_fire_prime_pulse(100000u);
    CHECK_TRUE(true, "fire_prime_pulse(100000): no crash (clamped to 30ms)");
}

void test_ecu_sched_dwell_watchdog(void) {
    section("ecu_sched: dwell watchdog");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u, "watchdog_count=0 at start");
    // Calling watchdog with no armed dwell should be a no-op
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u, "watchdog_count=0 with no armed coil");
}

// ============================================================================
// QUICK CRANK
// ============================================================================

void test_ecu_sched_hardware_init(void) {
    section("ecu_sched: ECU_Hardware_Init runs without crash");
    // ECU_Hardware_Init writes to TIM2/TIM1/GPIO mock registers (file-scope statics
    // in ecu_sched.cpp, not externally observable). Only testable behavior:
    //   1. No crash.
    //   2. Angle table cleared — ecu_sched_test_angle_table_size()=0 after init.
    //   3. Diagnostic counters cleared.
    ECU_Hardware_Init();
    CHECK_EQ(ecu_sched_test_angle_table_size(), 0u,
             "angle table empty after ECU_Hardware_Init");
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "dwell_watchdog_count=0 after ECU_Hardware_Init");
    CHECK_TRUE(true, "ECU_Hardware_Init: no crash");
}

void test_ecu_sched_ccr_write(void) {
    section("ecu_sched: arm_channel inserts event into TIM5 queue on DWELL_START");

    // Scheduler now uses TIM5-based absolute-timestamp event queue + GPIO BSRR.
    // TIM1 CCRs are no longer written by arm_channel.
    // Verify: after firing 13 teeth, at least one event is in the TIM5 queue.
    ecu_sched_test_reset();
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();  // angle table built at FULL_SYNC gap

    // Events fire when specific teeth match angle table entries — not at tooth 0.
    // Fire 57 normals (tooth_index 1..57); 58 without gap would LOSS (no wrap).
    for (uint32_t i = 0u; i < 57u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u ||
               ecu_sched_test_get_tim5_ccr3() > 0u,
               "at least one event in TIM5 queue after full revolution");
}

void test_ecu_sched_late_events(void) {
    section("ecu_sched: small delta events are queued with minimum delay");

    // Scheduler now uses TIM5 absolute-timestamp queue. Events with very small
    // delta use a minimum delay (STM32_MIN_COMPARE_LEAD_TICKS) instead
    // of being rejected. The old g_late_event_count path is no longer reached.
    // Verify: with advance=0 (delta≈0 at tooth 0), events still reach the queue.
    ecu_sched_test_reset();
    ecu_sched_set_advance_deg(0u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // Events were inserted (with minimum delay) even for near-zero delta.
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u ||
               ecu_sched_test_get_tim5_ccr3() > 0u,
               "events queued with minimum delay when delta~=0");
}

// Golden identity checks (plan verification): min-lead timestamp formula, angle
// table shape, sorted queue order — no soft "count>0" only.
void test_ecu_sched_golden_min_lead_timestamp(void) {
    section("ecu_sched golden: min-lead insert timestamp (not STATUS late)");
    // STM32_MIN_COMPARE_LEAD_TICKS = 125 @ 62.5 MHz (2 µs).
    // ECU_SCHED_US_TO_TICKS(1) = 62 < 125 → OFF event must land at now+125.
    // Min-lead is a schedule safety policy — does NOT increment g_late_event_count
    // (that bit is reserved for dispatch path-2 true misses).
    constexpr uint32_t kNow = 100000u;
    constexpr uint32_t kMinLead = 125u;
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_pulse_inj(0u, 1u);  // 1 µs PW → short delta → min-lead
    CHECK_TRUE(ecu_sched_test_get_evt_count() >= 1u, "OFF event queued");
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "peek head event");
    CHECK_EQ(ts, kNow + kMinLead, "min-lead timestamp = TIM5_CNT + 125");
    CHECK_EQ(high, 0u, "OFF event is low");
    CHECK_EQ(ch, ECU_CH_INJ1, "INJ1 channel id unchanged");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), late0,
             "min-lead does not sticky-inflate late_event_count");
}

void test_ecu_sched_golden_dispatch_past_counts_late(void) {
    section("ecu_sched golden: path-2 tight re-arm increments late_event_count");
    // path-2: after due-loop, next event is already past or ≤16 ticks ahead.
    // Queue OFF far out, then set CNT to ts-5 so due-loop skips (still future)
    // and re-arm loop takes path-2 (5 ≤ 16).
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    ecu_sched_test_pulse_inj(0u, 1000u);  // OFF ~ now+62500
    uint32_t ts = 0u;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, nullptr, nullptr), 1u, "have event");
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_set_tim5_cnt(ts - 5u);  // 5 ticks before → path-2
    ecu_sched_evt_dispatch();
    CHECK_TRUE(ecu_sched_test_get_late_event_count() > late0,
               "path-2 tight re-arm increments late_event_count");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 0u, "event consumed");
}

void test_ecu_sched_golden_far_target_timestamp(void) {
    section("ecu_sched golden: far target uses exact delta (no min-lead)");
    constexpr uint32_t kNow = 50000u;
    constexpr uint32_t kPwUs = 1000u;  // 1 ms → 62500 ticks @ 62.5 MHz
    constexpr uint32_t kExpectedDelta = (kPwUs * 125u) / 2u;  // ECU_SCHED_US_TO_TICKS
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    const uint32_t late0 = ecu_sched_test_get_late_event_count();
    ecu_sched_test_pulse_inj(1u, kPwUs);
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "peek OFF event");
    CHECK_EQ(ts, kNow + kExpectedDelta, "timestamp = now + exact PW ticks");
    CHECK_EQ(ch, ECU_CH_INJ2, "INJ2 channel");
    CHECK_EQ(high, 0u, "OFF");
    CHECK_EQ(ecu_sched_test_get_late_event_count(), late0,
             "no late count when delta >= min-lead");
}

void test_ecu_sched_golden_queue_sorted(void) {
    section("ecu_sched golden: queue stays sorted by timestamp");
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    // Two pulses with different PW → two OFF times; queue must be ascending.
    ecu_sched_test_pulse_inj(0u, 2000u);  // later OFF
    ecu_sched_test_set_tim5_cnt(1000u);   // same now for second arm
    ecu_sched_test_pulse_inj(1u, 500u);   // earlier OFF
    const uint8_t n = ecu_sched_test_get_evt_count();
    CHECK_TRUE(n >= 2u, "at least two OFF events");
    uint32_t prev = 0u;
    for (uint8_t i = 0u; i < n; ++i) {
        uint32_t ts = 0u;
        CHECK_EQ(ecu_sched_test_get_evt(i, &ts, nullptr, nullptr), 1u, "peek evt");
        if (i > 0u) {
            CHECK_TRUE(ts >= prev, "queue non-decreasing timestamps");
        }
        prev = ts;
    }
}

void test_ecu_sched_golden_seq_angle_table_size(void) {
    section("ecu_sched golden: sequential base angle table is 16 events");
    // 4 cyl × (DWELL + SPARK + INJ_ON + INJ_OFF) = 16 without multi-spark.
    ecu_sched_test_reset();
    ecu_sched_set_mspark(0u, 0u, 18u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    ckp_test_set_cmp_confirms(2u);
    ckp_feed_n_then_gap(57u);  // rebuild sequential at gap
    // May need schedule_this_gap toggle: first sequential gap builds table.
    if (ecu_sched_test_angle_table_size() == 0u) {
        ckp_feed_n_then_gap(57u);
    }
    CHECK_EQ(ecu_sched_test_angle_table_size(), 16u,
             "sequential no-mspark table has 16 events");
    // Every event has valid channel + action.
    uint8_t n_dwell = 0u, n_spark = 0u, n_inj_on = 0u, n_inj_off = 0u;
    for (uint8_t i = 0u; i < 16u; ++i) {
        uint8_t tooth = 0, frac = 0, ch = 0, action = 0, phase = 0;
        CHECK_EQ(ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase),
                 1u, "event valid");
        if (action == ECU_ACT_DWELL_START) { ++n_dwell; }
        else if (action == ECU_ACT_SPARK) { ++n_spark; }
        else if (action == ECU_ACT_INJ_ON) { ++n_inj_on; }
        else if (action == ECU_ACT_INJ_OFF) { ++n_inj_off; }
    }
    CHECK_EQ(n_dwell, 4u, "4 dwell starts");
    CHECK_EQ(n_spark, 4u, "4 sparks");
    CHECK_EQ(n_inj_on, 4u, "4 inj on");
    CHECK_EQ(n_inj_off, 4u, "4 inj off");
}

// Multi-spark fills angle table: base 16 + 3×2×4 = 40 ≤ ECU_ANGLE_TABLE_SIZE 48
void test_ecu_sched_mspark_angle_table_margin(void) {
    section("ecu_sched: multi-spark sequential table fits with margin");
    ecu_sched_test_reset();
    // inter_dwell short so all 3 extras fit inside advance+18° window
    ecu_sched_set_mspark(3u, 1000u, 18u);  // 3 extras, tiny inter-dwell ticks
    ecu_sched_set_advance_deg(30u);        // window = 30+18 = 48°
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    ckp_test_set_cmp_confirms(2u);
    ckp_feed_n_then_gap(57u);
    if (ecu_sched_test_angle_table_size() == 0u) {
        ckp_feed_n_then_gap(57u);
    }
    const uint8_t n = ecu_sched_test_angle_table_size();
    CHECK_TRUE(n > 16u, "multi-spark adds events beyond base 16");
    CHECK_TRUE(n <= ECU_ANGLE_TABLE_SIZE, "table size ≤ ECU_ANGLE_TABLE_SIZE");
    CHECK_EQ(ecu_sched_test_get_cycle_schedule_drop_count(), 0u,
             "no angle-table drops with max multi-spark");
    // Count ign events: base 4+4 + up to 3×(4+4) extras
    uint8_t n_dwell = 0u, n_spark = 0u;
    for (uint8_t i = 0u; i < n; ++i) {
        uint8_t tooth = 0, frac = 0, ch = 0, action = 0, phase = 0;
        if (ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase) == 0u) {
            continue;
        }
        if (action == ECU_ACT_DWELL_START) { ++n_dwell; }
        else if (action == ECU_ACT_SPARK) { ++n_spark; }
    }
    CHECK_TRUE(n_dwell >= 4u && n_dwell <= 16u, "dwell count in [4,16] with mspark≤3");
    CHECK_TRUE(n_spark >= 4u && n_spark <= 16u, "spark count in [4,16] with mspark≤3");
    CHECK_EQ(n_dwell, n_spark, "dwell/spark pairs balanced");
    ecu_sched_set_mspark(0u, 0u, 18u);
}

void test_ecu_sched_golden_dispatch_identity(void) {
    section("ecu_sched golden: dispatch fires head GPIO order (channel, high)");
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(1000u);
    ecu_sched_test_pulse_inj(0u, 1000u);  // OFF at 1000+62500
    // Make event due and dispatch.
    uint32_t ts = 0u;
    uint8_t ch = 0u, high = 0xffu;
    CHECK_EQ(ecu_sched_test_get_evt(0u, &ts, &ch, &high), 1u, "have event");
    const uint8_t n0 = ecu_sched_test_get_evt_count();
    ecu_sched_test_set_tim5_cnt(ts);  // CNT == timestamp → due
    ecu_sched_evt_dispatch();
    CHECK_EQ(ecu_sched_test_get_evt_count(), static_cast<uint32_t>(n0 - 1u),
             "one event consumed by dispatch");
    // Pin counters: INJ1 pin index 0 — OFF is low transition after force ON.
    // At least one high and one low counted for pin 0 path (force ON + OFF).
    uint32_t pins[24];
    ecu_sched_get_pin_counts_u32x24(pins);
    CHECK_TRUE(pins[0] >= 1u, "INJ1 high_count >= 1 after force ON");
    CHECK_TRUE(pins[1] >= 1u, "INJ1 low_count >= 1 after OFF dispatch");
}

void test_ecu_sched_dwell_watchdog_fires(void) {
    section("ecu_sched: dwell watchdog fires after 1.4x dwell ticks");

    // Direct path: force dwell HIGH + queue SPARK without dispatching SPARK.
    // Watchdog must stay armed across SPARK *arm* (only pin LOW / trip release it)
    // so a lost SPARK cannot leave the coil charged indefinitely.
    const uint32_t kNow = 1000u;
    const uint32_t kDwellUs = 3000u;  // 3 ms → 187500 ticks @ 62.5 MHz
    const uint32_t kDwellTicks = (kDwellUs * 125u) / 2u;
    const uint32_t kWdogTicks = (kDwellTicks * 7u) / 5u;  // 1.4×
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    ecu_sched_test_pulse_ign(0u, kDwellUs);  // cyl0 HIGH + SPARK queued, wdog armed

    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "pre-cond: wdog_count=0 before threshold");
    // Still within window — no trip
    ecu_sched_test_set_tim5_cnt(kNow + kWdogTicks - 1u);
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 0u,
             "watchdog silent inside 1.4× dwell");
    // Past 1.4× dwell with SPARK still only queued → force pin LOW
    ecu_sched_test_set_tim5_cnt(kNow + kWdogTicks + 1u);
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 1u,
             "dwell watchdog fires: elapsed > 1.4× dwell (lost-SPARK backstop)");
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), 1u, "watchdog fires only once per arm");
}

void test_ecu_sched_inj_watchdog_fires(void) {
    section("ecu_sched: injector open watchdog fires after timeout (lost INJ_OFF)");

    // Force INJ HIGH without OFF (simulate queue drop of OFF). force_output path
    // uses hard 36 ms timeout.
    const uint32_t kNow = 5000u;
    const uint32_t kHardTicks = (36000u * 125u) / 2u;  // 36 ms @ 62.5 MHz
    ecu_sched_test_reset();
    ecu_sched_test_set_tim5_cnt(kNow);
    // test_pulse schedules OFF — fire raw force path via test pulse then clear OFF
    // by advancing past OFF without dispatch: use pulse then wipe queue.
    ecu_sched_test_pulse_inj(0u, 3000u);  // ON + OFF queued
    // Drop OFF from queue by resetting event queue only, keep pin state via
    // another open: re-force through pulse then zero events after arm.
    // Simpler: pulse with PW, discard OFF by resetting CCR/queue after ON.
    {
        // Re-open: force via second pulse; then clear queue so OFF never runs.
        ecu_sched_test_reset_ccr();
        // Pin may be LOW after reset_ccr — re-open with pulse and immediately
        // drop all events (lost OFF).
        ecu_sched_test_set_tim5_cnt(kNow);
        ecu_sched_test_pulse_inj(0u, 5000u);
        // Wipe queue: OFF is gone, pin still HIGH from force_output.
        ecu_sched_test_reset_ccr();
    }
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 0u, "pre: inj wdog count=0");
    ecu_sched_test_set_tim5_cnt(kNow + kHardTicks - 1u);
    ecu_sched_inj_watchdog();
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 0u, "silent inside hard timeout");
    ecu_sched_test_set_tim5_cnt(kNow + kHardTicks + 1u);
    ecu_sched_inj_watchdog();
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 1u,
             "inj watchdog fires: pin HIGH past hard timeout without OFF");
    ecu_sched_inj_watchdog();
    CHECK_EQ(ecu_sched_inj_watchdog_count(), 1u, "inj wdog fires once per open");
}

void test_ecu_sched_presync_table(void) {
    section("ecu_sched: presync table built at natural gap rev boundary (no CMP)");

    // Rev boundary is tooth_index reset by an accepted gap (not a phantom wrap
    // past 57). With no CMP, FULL_SYNC still uses presync / wasted builders.
    ecu_sched_test_reset();
    ecu_sched_test_set_tim1_cnt(0u);
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    ckp_test_reset(); g_ckp_cap = 0u;

    ckp_feed_n_then_gap(55u);   // → HALF_SYNC (no rev_boundary yet: prev_tooth=0)
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::HALF_SYNC), "pre-cond: HALF_SYNC");

    // 55+ normals then gap → FULL_SYNC, tooth_index=0, rev_boundary → presync table
    // (cmp_confirms < 2 → wasted/presync path).
    for (uint32_t i = 0u; i < 55u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kGapPeriod);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "post-gap: FULL_SYNC");
    const uint8_t tsz = ecu_sched_test_angle_table_size();
    CHECK_TRUE(tsz > 0u, "presync angle table populated after gap rev boundary");

    // Presync events use ECU_PHASE_ANY (= 2) so they fire on every revolution
    bool found_any = false;
    for (uint8_t i = 0u; i < tsz; ++i) {
        uint8_t tooth, frac, ch, action, phase;
        if (ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase) != 0u) {
            if (phase == ECU_PHASE_ANY) { found_any = true; }
        }
    }
    CHECK_TRUE(found_any, "at least one presync event uses ECU_PHASE_ANY");

    // Presync IGN events include DWELL_START and SPARK for all 4 coils simultaneously
    // → table should have ≥ 2 ignition actions (at minimum: DWELL_START + SPARK)
    uint8_t n_ign = 0u;
    for (uint8_t i = 0u; i < tsz; ++i) {
        uint8_t tooth, frac, ch, action, phase;
        if (ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase) != 0u) {
            if (action == ECU_ACT_DWELL_START || action == ECU_ACT_SPARK) { ++n_ign; }
        }
    }
    CHECK_TRUE(n_ign >= 2u, "presync table: ≥2 ignition events");
}

// ============================================================================
// CKP — FASE 3 (prime_on_tooth, snap fields, tooth_index, phase_A toggle)
// ============================================================================

static constexpr uint32_t kCrankPeriod = 100000u;  // rpm_x10 = 6250 < 7000 (cranking)

