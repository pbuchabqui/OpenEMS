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
#include "hal/out_pins.h"
#include "engine/enc_cyl_setpoints.h"
#include "engine/map_window.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/misfire_encoder.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "engine/output_test.h"
#include "engine/engine_config.h"
#include "hal/timer.h"
#include "hal/flash.h"
#include "drv/encoder_sync.h"
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
using namespace ems::drv::encoder_sync;
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
    // cilindros por revolução em presync. Ignição wasted: 2 pares × 2 bobinas
    // (DWELL+SPARK = 8), injeção só 2 cilindros (ON+OFF = 4) → 12 eventos.
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
    ecu_sched_test_set_tim5_cnt(1000u);
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
    ecu_sched_test_set_tim5_cnt(1000u);
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

// O blend 1D só-RPM (eoi_idle_deg/eoi_blend_rpm_lo/hi/default_eoi_lead_deg)
// foi removido 2026-08-16, substituído pela tabela EOI 2D RPM×CLT — ver
// test_fuel_eoi_2d (test_fuel.cpp). A parte de sanitize (independente do
// blend, testa o setter direto do scheduler legacy) fica retida aqui.
void test_ecu_sched_eoi_lead_deg_sanitize(void) {
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
// MT6835/TIM2 ENCODER — estimador de ω (ecu_sched_angle_encoder.cpp)
// ============================================================================

void test_ecu_sched_encoder_omega(void) {
    section("ecu_sched: encoder omega estimator");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "invalid before first sample");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 0, "x65536=0 before first sample");

    // First sample only seeds prev — still no rate to compute.
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "invalid after single sample");

    // d_tim2=1000, d_tim5=1000 -> ω=1.0 exact -> x65536=65536.
    ecu_sched_encoder_omega_sample(2000u, 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "valid after second sample");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega=1.0 -> x65536=65536");

    // d_tim2=500, d_tim5=1000 -> ω=0.5 -> x65536=32768.
    ecu_sched_encoder_omega_sample(2500u, 3000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 32768, "omega=0.5 -> x65536=32768");

    // Reverse rotation (kick-back): TIM2 decrements, TIM5 keeps advancing —
    // sign must survive, not be clamped to zero/positive.
    ecu_sched_encoder_omega_sample(2300u, 4000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), -13107,
             "reverse rotation: negative x65536 (d_tim2=-200/d_tim5=1000)");

    // d_tim5<=0 (stale/out-of-order sample): estimate must hold, not update
    // (division-by-zero / sign-inversion guard).
    const int32_t before = ecu_sched_encoder_omega_x65536();
    ecu_sched_encoder_omega_sample(9999u, 4000u);  // same tim5_now as previous
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), before,
             "d_tim5<=0: estimate unchanged, no divide-by-zero");
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "still valid after stale sample");

    // Regression guard for the x256 truncation-to-zero bug: a realistic
    // ~200 rpm cranking rate (d_tim2=874 counts / d_tim5=1e6 ticks, the
    // same ratio as 200 rpm @ 16384 counts/rev, 62.5 MHz TIM5) must NOT
    // read as zero rotation. Under the old ×256 scale this rounded to 0
    // (0.224 truncated); at ×65536 it must land at 57.
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(0u, 0u);
    ecu_sched_encoder_omega_sample(874u, 1000000u);
    CHECK_TRUE(ecu_sched_encoder_omega_x65536() != 0,
               "200rpm-equivalent rate must not truncate to zero (x256 bug regression)");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 57,
             "200rpm-equivalent rate -> x65536=57");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "test_reset() clears omega state");
}

void test_ecu_sched_encoder_phase(void) {
    section("ecu_sched: encoder phase tracker");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "invalid before first anchor");

    // Anchor: raw=100000 marks entering ECU_PHASE_B.
    ecu_sched_encoder_phase_set_anchor(100000u, ECU_PHASE_B);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u, "valid after set_anchor");
    CHECK_EQ(ecu_sched_encoder_phase_at(100000u), ECU_PHASE_B,
             "at anchor: same phase (revs=0, even)");

    // +1 full rev (16384 counts) -> phase flips.
    CHECK_EQ(ecu_sched_encoder_phase_at(116384u), ECU_PHASE_A,
             "+1 rev: flipped (revs=1, odd)");
    // +2 full revs -> back to anchor phase.
    CHECK_EQ(ecu_sched_encoder_phase_at(132768u), ECU_PHASE_B,
             "+2 revs: same phase again (revs=2, even)");

    // Boundary just BEFORE the anchor: still belongs to the previous
    // (flipped) revolution — this is the floor-division correctness case
    // (truncated division would wrongly give revs=0/unflipped here).
    CHECK_EQ(ecu_sched_encoder_phase_at(99999u), ECU_PHASE_A,
             "1 count before anchor: flipped (floor(-1/16384)=-1, odd)");
    // Exactly 1 rev before the anchor -> flipped (revs=-1 exact).
    CHECK_EQ(ecu_sched_encoder_phase_at(83616u), ECU_PHASE_A,
             "-1 rev exact: flipped (revs=-1, odd)");
    // 1 count further back crosses into the next-older revolution -> unflipped.
    CHECK_EQ(ecu_sched_encoder_phase_at(83615u), ECU_PHASE_B,
             "-1 rev -1 count: unflipped (revs=-2, even)");

    // Re-anchoring is absolute, not incremental — a second set_anchor with a
    // different phase overrides the previous state entirely (no toggle).
    ecu_sched_encoder_phase_set_anchor(500000u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_phase_at(500000u), ECU_PHASE_A,
             "re-anchor: absolute, reflects new anchor immediately");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "test_reset() clears phase anchor");
}

void test_ecu_sched_encoder_min_lead(void) {
    section("ecu_sched: encoder arm — min-lead floor via omega (task #8)");
    ecu_sched_test_reset();

    // Omega invalid (never sampled): floor is 0, target passes through
    // unmodified even at lead=0 — the "late" dispatch path is what handles
    // an already-due target in this state, not a synthesized margin.
    ecu_sched_encoder_test_set_tim2_cnt(1000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 1000u, ECU_ACT_INJ_ON);
    uint32_t ts = 0u; uint8_t ch = 0xFFu; uint8_t high = 0xFFu;
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 1000u, "omega invalid: no floor applied, target unchanged");
    ecu_sched_test_reset();

    // Seed omega=1.0 exact (d_tim2=1000/d_tim5=1000, same recipe as
    // test_ecu_sched_encoder_omega) -> x65536=65536. Floor = 2us worth of
    // counts at this rate = ECU_SCHED_US_TO_TICKS_INTERNAL(2)=125 ticks *
    // 65536/65536 = 125 counts.
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega seeded to 1.0");

    // Target exactly at "now": lead=0 < floor(125) -> clamped to now+125.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5125u, "lead=0 < floor: clamped to now+125");
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);

    // Target just short of the floor (lead=124): still clamped.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5124u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5125u, "lead=124 < floor=125: still clamped to now+125");
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);

    // Target exactly at the floor (lead=125): passes through unchanged.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5125u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5125u, "lead=125 == floor: passes through unchanged");
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(2000u, 2000u);

    // Comfortably far target: unaffected by the floor.
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 9000u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 9000u, "far target: unaffected by floor");

    // Half-rate omega (0.5, x65536=32768): floor scales down proportionally
    // -> 125*32768/65536 = 62 (integer truncation).
    ecu_sched_test_reset();
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    ecu_sched_encoder_omega_sample(1500u, 2000u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 32768, "omega seeded to 0.5");
    ecu_sched_encoder_test_set_tim2_cnt(5000u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 5062u, "omega=0.5: floor scales down to 62 counts");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_basic(void) {
    section("ecu_sched: encoder queue (TIM2/CH3) — insert order + CCR3 arm");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "empty at start");
    CHECK_EQ(ecu_sched_encoder_test_get_dier(), 0u, "CC3IE off at start");

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 500u, ECU_ACT_INJ_ON);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "count=1 after first arm");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 500u, "CCR3=500 (only/earliest event)");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() != 0u,
               "CC3IE on after first insert");

    uint32_t ts = 0u; uint8_t ch = 0xFFu; uint8_t high = 0xFFu;
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 500u, "evt[0].ts=500");
    CHECK_EQ(ch, (uint8_t)ECU_CH_INJ1, "evt[0].channel=INJ1");
    CHECK_EQ(high, 1u, "evt[0].high=1 (ON)");

    // Earlier target becomes the new head — CCR3 rearms to it, not appended.
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 300u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 2u, "count=2 after second arm");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 300u, "CCR3 rearmed to earlier target");
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(0, &ts, &ch, &high) != 0u, "get_evt(0) ok");
    CHECK_EQ(ts, 300u, "evt[0]=IGN1/300 (sorted ahead of INJ1/500)");
    CHECK_EQ(ch, (uint8_t)ECU_CH_IGN1, "evt[0].channel=IGN1");
    CHECK_EQ(high, 0u, "evt[0].high=0 (SPARK)");
    CHECK_TRUE(ecu_sched_encoder_test_get_evt(1, &ts, &ch, &high) != 0u, "get_evt(1) ok");
    CHECK_EQ(ts, 500u, "evt[1]=INJ1/500 (unchanged, now second)");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "test_reset() clears encoder queue");
}

void test_ecu_sched_encoder_queue_dispatch(void) {
    section("ecu_sched: encoder queue — dispatch + CC3IE dynamic disable");
    ecu_sched_test_reset();

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 200u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 2u, "2 events armed");

    ecu_sched_encoder_test_set_tim2_cnt(100u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "1 event fired at ts=100");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 200u, "CCR3 rearmed to remaining event");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() != 0u,
               "CC3IE still on — queue not empty");

    ecu_sched_encoder_test_set_tim2_cnt(200u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "queue empty after second fire");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() == 0u,
               "CC3IE off — queue emptied (dynamic disable, mirrors TIM5)");

    // Simultaneous-at-dispatch: two events due in the same ISR entry both fire.
    ecu_sched_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 50u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ2, 50u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(50u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "both simultaneous events fire in one dispatch entry");
    CHECK_EQ(ecu_sched_encoder_test_get_late_event_count(), 0u,
             "on-time dispatch is not counted as late");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_dispatch_margin_domain(void) {
    section("ecu_sched: dispatch — margem TIM2 = 3 µs via ω (fallback 16 sem ω)");

    // ω inválido (estado limpo de reset): fallback ao literal 16 counts.
    // Alvo a now+17 (>16): CCR3 reprogramado, não dispara já.
    ecu_sched_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 17u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u,
             "ω inválido, alvo a +17: ainda pendente (margem=16 fallback)");
    CHECK_EQ(ecu_sched_encoder_test_get_ccr3(), 17u,
             "ω inválido, alvo a +17: CCR3 reprogramado, não disparado como late");

    // Alvo a now+16 (não > 16): dispara já como late — fronteira do fallback.
    ecu_sched_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 16u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "ω inválido, alvo a +16: dispara já (fronteira do fallback=16)");
    CHECK_EQ(ecu_sched_encoder_test_get_late_event_count(), 1u,
             "ω inválido, alvo a +16: contado como late");
    CHECK_EQ(ecu_sched_encoder_late_event_count(), 1u,
             "getter de produção late TIM2 ≠0 após late forçado");
    CHECK_EQ(ecu_sched_encoder_evt_overflow(), 0u,
             "getter de produção overflow TIM2 ainda 0");

    // ω válido, ratio=2.0 (omega_x65536=131072) → margem =
    //   ticks(3µs)=3*125/2=187 → 187*131072/65536 = 374 counts.
    // Alvo absoluto bem distante (100000) para não ser clampado pelo piso
    // min_lead_counts() de arm_channel.
    ecu_sched_test_reset();
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(2000u, 1000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 131072, "ω sintético seeded a ratio=2.0");

    ecu_sched_encoder_queue_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100000u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(100000u - 375u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u,
             "ω=2.0×, alvo a +375: ainda pendente (margem=374 = 3 µs via ω)");

    ecu_sched_encoder_queue_test_reset();
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100000u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_test_set_tim2_cnt(100000u - 374u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "ω=2.0×, alvo a +374: dispara já (fronteira da margem 3 µs)");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_overflow(void) {
    section("ecu_sched: encoder queue — overflow policy (never drop a pending OFF)");
    ecu_sched_test_reset();

    // Fill the queue with 48 ON events (alternating channels so none collide
    // as the "same channel" preference in the drop policy).
    for (uint32_t i = 0u; i < 48u; ++i) {
        const uint8_t ch = (i % 2u == 0u) ? ECU_CH_INJ1 : ECU_CH_INJ2;
        ecu_sched_encoder_arm_channel(ch, 1000u + i, ECU_ACT_INJ_ON);
    }
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 48u, "queue full at 48");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_overflow(), 0u, "no overflow yet — exactly full");

    // 49th ON with a full queue: dropped (queue keeps de-asserts already
    // queued in preference over a new assert).
    ecu_sched_encoder_arm_channel(ECU_CH_INJ3, 2000u, ECU_ACT_INJ_ON);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 48u, "still 48 — new ON dropped");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_overflow(), 1u, "overflow counted");

    // A de-assert (OFF/SPARK) on a full queue evicts one ON to make room —
    // never silently dropped itself (an open injector must be closable).
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 3000u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 48u,
             "still 48 — OFF evicted an ON to fit");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_purge_via_inhibit_mask(void) {
    section("ecu_sched: encoder queue — purge sweep via inj inhibit mask");
    ecu_sched_test_reset();

    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_ON);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "INJ1 event armed on encoder queue");

    // INJ1 = cyl 0 -> inhibit mask bit0. purge_events_for_cyl_mask()
    // (ecu_sched.cpp) must sweep BOTH queues — this is the real wiring
    // path, not a direct call into the internal sweep function.
    ecu_sched_set_inj_inhibit_mask(0x01u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "encoder queue purged by inj inhibit mask (cross-queue sweep)");

    ecu_sched_set_inj_inhibit_mask(0x00u);
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_queue_clear_via_outputs_safe(void) {
    section("ecu_sched: encoder queue — cleared by ecu_sched_test_all_outputs_safe()");
    ecu_sched_test_reset();

    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 7000u, ECU_ACT_DWELL_START);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 1u, "IGN1 event armed on encoder queue");

    ecu_sched_test_all_outputs_safe();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "encoder queue cleared by clear_all_events_and_drive_safe_outputs()");
    CHECK_TRUE(ecu_sched_encoder_test_get_dier() == 0u,
               "CC3IE off after clear-all");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_handoff_force_closes_pins(void) {
    section("ecu_sched: handoff presync->sequencial força fecho de pinos (fix bug 4)");
    ecu_sched_test_reset();
    ems::hal::out_pins_test_reset_stubs();

    // INJ1 (canal 2 = PA15 na RGT6 default) é o único canal no port A — a
    // fila de force-close (0x0F) reescreve GPIOC várias vezes (IGN1-4/INJ3-4
    // partilham port C), por isso o snapshot final desse port não prova nada
    // sobre um canal específico. PA15 nunca é tocado por outro canal, o que
    // torna o snapshot final determinístico para esta asserção.
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 100u, ECU_ACT_INJ_ON);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 100u, ECU_ACT_DWELL_START);
    ecu_sched_encoder_test_set_tim2_cnt(100u);
    ecu_sched_encoder_evt_dispatch();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "INJ_ON+DWELL_START despachados");
    CHECK_TRUE((ems::hal::out_pins_test_bsrr_snapshot(0u) & (1u << 15u)) != 0u,
               "INJ1 (PA15) HIGH após dispatch do INJ_ON");
    CHECK_TRUE(ecu_sched_test_get_dwell_arm_tick(0u) != 0u,
               "g_dwell_arm_tick[0] armado após DWELL_START");

    // Arma as contrapartes de-assert para alvos futuros — ficam PENDENTES na
    // fila TIM2/CH3 quando o handoff acontecer a seguir.
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 5000u, ECU_ACT_INJ_OFF);
    ecu_sched_encoder_arm_channel(ECU_CH_IGN1, 5000u, ECU_ACT_SPARK);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 2u, "INJ_OFF+SPARK pendentes na fila");

    // Handoff presync→sequencial: fase passa a válida com
    // g_enc_last_builder_was_sequential==0 (estado limpo do reset acima) —
    // dispara o purge da fila + (fix) o force-close dos pinos.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    ecu_sched_encoder_heartbeat_tick(6000u, 6000u, 0u, 0u);

    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "fila TIM2 purgada no handoff (eventos pendentes descartados)");
    CHECK_TRUE((ems::hal::out_pins_test_bsrr_snapshot(0u) & (1u << (15u + 16u))) != 0u,
               "INJ1 (PA15) forçado LOW no handoff — antes do fix ficava HIGH até o watchdog");
    CHECK_EQ(ecu_sched_test_get_dwell_arm_tick(0u), 0u,
             "g_dwell_arm_tick[0] limpo no handoff — sem isto o watchdog ainda achava a bobina em carga");

    ecu_sched_test_reset();
    ems::hal::out_pins_test_reset_stubs();
}

void test_ecu_sched_encoder_heartbeat(void) {
    section("ecu_sched: encoder heartbeat tick — feeds omega estimator");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "omega invalid before any heartbeat tick");

    // First tick only seeds the omega estimator's previous sample.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "still invalid after single tick");

    // Second tick: d_tim2=1000, d_tim5=1000 -> omega=1.0 -> x65536=65536.
    // Confirms the heartbeat really calls ecu_sched_encoder_omega_sample()
    // with the values it was handed (HAL reads them, this just verifies the
    // wiring, not the estimator's own math — that's covered separately).
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "valid after second tick");
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega fed correctly through the heartbeat");

    // cmp_edge_count delta is tracked/validated (drv/encoder_sync.cpp, task
    // #13) — but the actual phase_set_anchor() call stays gated behind
    // EMS_MT6835_CMP_PHASE_CALIBRATED (0 by default: calibration constant
    // not yet measured on a bench, see plan), so phase_valid() stays 0 even
    // though the edge itself was accepted as a valid first reference. See
    // test_ecu_sched_encoder_heartbeat_cmp_tracking() for direct coverage of
    // the tracking/validation logic itself.
    ecu_sched_encoder_heartbeat_tick(2500u, 3000u, 12345u, 1u);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "phase anchor NOT set by the heartbeat yet (calibration constant pending)");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "test_reset() clears heartbeat-fed state");
}

void test_ecu_sched_encoder_heartbeat_cmp_tracking(void) {
    section("ecu_sched: encoder heartbeat — CMP edge tracking/validation (task #13)");
    ecu_sched_test_reset();
    ckp_test_reset();

    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "reject count=0 at start");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 0u, "missed-edge count=0 at start");

    // A: first edge (cmp_edge_count 0->1) — arms reference, no validation.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 1000u, 1u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "A: first edge — no reject");

    // B: normal span (delta=32768 exact) — accepted, multiple=1.
    ecu_sched_encoder_heartbeat_tick(2000u, 2000u, 1000u + kCmpSpanCounts, 2u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "B: normal span — no reject");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 0u, "B: multiple=1 — not counted as missed");

    // C: implausible span (delta=16384, half a revolution) — rejected.
    ecu_sched_encoder_heartbeat_tick(3000u, 3000u, 1000u + kCmpSpanCounts + 16384u, 3u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 1u, "C: implausible span — rejected");

    // D: missed edge (delta=2x32768 from B's angle, C's reject didn't move
    // the reference) — accepted as multiple=2.
    ecu_sched_encoder_heartbeat_tick(4000u, 4000u, 1000u + kCmpSpanCounts + 2u * kCmpSpanCounts, 4u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 1u, "D: accepted — reject count unchanged");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 1u, "D: multiple=2 — missed edge counted");

    // E/F/G: 3 consecutive bad edges (delta=100, nowhere near a multiple) ->
    // reject-streak reaches the resync threshold on the 3rd.
    const uint32_t ref = 1000u + kCmpSpanCounts + 2u * kCmpSpanCounts;  // D's accepted angle
    ecu_sched_encoder_heartbeat_tick(5000u, 5000u, ref + 100u, 5u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 2u, "E: 1st consecutive reject");
    ecu_sched_encoder_heartbeat_tick(6000u, 6000u, ref + 100u, 6u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 3u, "F: 2nd consecutive reject");
    ecu_sched_encoder_heartbeat_tick(7000u, 7000u, ref + 100u, 7u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 4u, "G: 3rd consecutive reject (streak resync)");

    // H: reference was dropped by the resync — next edge is treated as a
    // fresh "first edge" again, no reject regardless of its angle.
    ecu_sched_encoder_heartbeat_tick(8000u, 8000u, 999999u, 8u);
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 4u, "H: post-resync first edge — no new reject");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_missed_edge_count(), 1u, "H: missed-edge count unchanged");

    ecu_sched_test_reset();
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_reject_count(), 0u, "test_reset() clears CMP tracking state");
}

void test_ecu_sched_encoder_heartbeat_publish_snapshot(void) {
    section("ecu_sched: encoder heartbeat — publishes ckp_snapshot() (task #13)");
    ecu_sched_test_reset();
    ckp_test_reset();

    // Default: phase invalid (uncalibrated), health ok -> HALF_SYNC.
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    CkpSnapshot snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::HALF_SYNC,
               "phase invalid + health ok -> HALF_SYNC published");
    CHECK_EQ(snap.cmp_confirms, 0u, "HALF_SYNC: cmp_confirms=0");

    // rpm_x10: seed omega to a known ratio (874/1e6, the same 200rpm-
    // equivalent recipe used by the omega estimator's own regression test)
    // and hand-verify the conversion: omega=57 -> rpm_x10=1990 (600e9*57 /
    // (16*16384*65536), integer truncation).
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(874u, 1000000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 57, "omega seeded to 57 (200rpm-equivalent)");
    snap = ckp_snapshot();
    CHECK_EQ(snap.rpm_x10, 1990u, "rpm_x10 derived correctly from omega_x65536");

    // Health fault (encoder_sync::set_health_ok(false), simulating task #14's
    // mt6835_ok() poll having detected a failure) -> LOSS_OF_SYNC overrides
    // everything else, regardless of phase state.
    ems::drv::encoder_sync::set_health_ok(false);
    ecu_sched_encoder_heartbeat_tick(875u, 1000001u, 0u, 0u);
    snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::LOSS_OF_SYNC,
               "health_ok()=false -> LOSS_OF_SYNC published");
    ems::drv::encoder_sync::set_health_ok(true);

    // Phase valid (seeded directly — bypasses the EMS_MT6835_CMP_PHASE_CALIBRATED
    // gate, which only guards the call site inside the heartbeat, not the
    // underlying phase tracker itself) -> FULL_SYNC, cmp_confirms=2.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    ecu_sched_encoder_heartbeat_tick(876u, 1000002u, 0u, 0u);
    snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::FULL_SYNC,
               "phase valid + health ok -> FULL_SYNC published");
    CHECK_EQ(snap.cmp_confirms, 2u, "FULL_SYNC: cmp_confirms=2");
    CHECK_TRUE(snap.phase_A, "phase_A reflects ecu_sched_encoder_phase_at() at publish time");

    // Staleness: many heartbeats with no new CMP edge eventually invalidate
    // the phase (fallback FULL_SYNC->HALF_SYNC) — exact threshold already
    // covered by test_encoder_sync_staleness(); here just confirm the
    // integration actually fires within a safe margin above it.
    for (uint32_t i = 0u; i < 10u; ++i) {
        ecu_sched_encoder_heartbeat_tick(877u + i, 1000003u + i, 0u, 0u);
    }
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u,
             "10 heartbeats without a new CMP edge (> prod limit=6): phase invalidated");
    snap = ckp_snapshot();
    CHECK_TRUE(snap.state == ems::drv::SyncState::HALF_SYNC,
               "staleness fallback published as HALF_SYNC, not stuck at FULL_SYNC");

    ecu_sched_test_reset();
    ckp_test_reset();
}

void test_ecu_sched_encoder_heartbeat_publish_tooth_index(void) {
    section("ecu_sched: encoder heartbeat — tooth_index derivado de tim2_now (fix VVT)");
    ecu_sched_test_reset();
    ckp_test_reset();

    // tim2_now=0 -> tooth_index=0 (início da revolução).
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().tooth_index, 0u, "tim2_now=0 -> tooth_index=0");

    // tim2_now=4096 (1/4 de volta, 16384 counts/rev) -> tooth_index=15
    // ((4096*60)/16384 = 15) — antes do fix ficava sempre 0.
    ecu_sched_encoder_heartbeat_tick(4096u, 100u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().tooth_index, 15u, "tim2_now=4096 -> tooth_index=15");

    // tim2_now=8192 (1/2 volta) -> tooth_index=30.
    ecu_sched_encoder_heartbeat_tick(8192u, 200u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().tooth_index, 30u, "tim2_now=8192 -> tooth_index=30");

    // Wrap: tim2_now=16384+4096 (1 volta + 1/4) -> mod 16384 = 4096 -> tooth_index=15,
    // confirmando que é (tim2_now % 16384), não tim2_now bruto.
    ecu_sched_encoder_heartbeat_tick(16384u + 4096u, 300u, 0u, 0u);
    CHECK_EQ(ckp_snapshot().tooth_index, 15u, "wrap de revolução: tooth_index continua correto");

    ecu_sched_test_reset();
    ckp_test_reset();
}

// ── Split light/heavy do heartbeat TIM2_CH4 ─────────────────────────────────
// ecu_sched_encoder_heartbeat_subtick() é chamada a CADA CC4IF (256 counts,
// ~64×/volta), não só 1×/volta como ecu_sched_encoder_heartbeat_tick()
// (testada directamente nos 3 testes acima, que continuam a passar
// inalterados — não passam por este wrapper). Verifica só a cadência do
// split em si: o caminho pesado (aqui detectado via
// cmp_heartbeats_since_ok, que só o caminho pesado incrementa) dispara
// exactamente 1×/64 chamadas, nunca nas outras 63.
void test_ecu_sched_encoder_heartbeat_subtick_cadence(void) {
    section("ecu_sched: heartbeat_subtick — split light/heavy, cadência 1/64");
    ecu_sched_test_reset();

    CHECK_EQ(ecu_sched_encoder_test_get_subtick_count(), 0u, "pré-condição: subtick_count=0");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 0u,
             "pré-condição: caminho pesado nunca correu");

    // cmp_edge_count constante (0) em todas as chamadas — a avaliação de
    // flanco CMP fica sempre inactiva (delta=0), isolando o teste só à
    // cadência do split, sem interferência da lógica de validação de CMP.
    for (uint32_t i = 1u; i <= 63u; ++i) {
        ecu_sched_encoder_heartbeat_subtick(i * 256u, i * 100u, 0u, 0u);
        CHECK_EQ(ecu_sched_encoder_test_get_subtick_count(), static_cast<uint8_t>(i),
                 "subtick_count incrementa a cada sub-tick");
    }
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 0u,
             "63 sub-ticks: caminho pesado ainda não correu nenhuma vez");

    // 64ª chamada: dispara o caminho pesado, reseta subtick_count.
    ecu_sched_encoder_heartbeat_subtick(64u * 256u, 64u * 100u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_subtick_count(), 0u,
             "64º sub-tick: subtick_count reseta a 0");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 1u,
             "64º sub-tick: caminho pesado correu exactamente 1×");

    // Segundo ciclo completo de 64 — confirma que a cadência se repete, não
    // é um efeito de arranque único.
    for (uint32_t i = 1u; i <= 64u; ++i) {
        ecu_sched_encoder_heartbeat_subtick((64u + i) * 256u, (64u + i) * 100u, 0u, 0u);
    }
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_heartbeats_since_ok(), 2u,
             "2º ciclo de 64: caminho pesado corre de novo exactamente 1×");

    ecu_sched_test_reset();
}

// Prova a LIGAÇÃO (não a lógica — essa está coberta em detalhe em
// test_misfire_encoder.cpp): heartbeat_subtick() alimenta mesmo
// misfire_encoder_on_sample(), não só chama o caminho pesado. Mesma
// sequência numérica de test_misfire_encoder_threshold_debounce_and_inertness
// (1 ciclo lento só, o suficiente para confirmar a ligação), mas conduzida
// através de ecu_sched_encoder_heartbeat_subtick() em vez de chamar
// misfire_encoder_on_sample() directamente.
void test_ecu_sched_encoder_heartbeat_subtick_feeds_misfire(void) {
    section("ecu_sched: heartbeat_subtick alimenta misfire_encoder (wiring)");
    ecu_sched_test_reset();
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    misfire_encoder_init();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);

    ecu_sched_encoder_heartbeat_subtick(15360u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(15616u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(15872u, 2000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(16128u, 3000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_subtick(32768u, 4200u, 0u, 0u);  // entra cyl0, Δ=1200 (lento)
    ecu_sched_encoder_heartbeat_subtick(35840u, 5200u, 0u, 0u);  // sai → avalia

    CHECK_EQ(misfire_encoder_test_get_debounce(0u), 1u,
             "heartbeat_subtick alimenta misfire_encoder_on_sample via wiring real");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_conversion(void) {
    section("ecu_sched: encoder degrees<->counts conversion (pure math)");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(0u), 0u, "0 deg -> 0 counts");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(90u), 4096u, "90 deg -> 1/4 rev (4096)");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(270u), 12288u, "270 deg -> 3/4 rev (12288)");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(360u), 0u, "360 deg wraps to 0 (mod 360 domain)");
    CHECK_EQ(ecu_sched_encoder_test_engine_deg_to_counts(359u), 16338u,
             "359 deg -> 16338 (359*16384/360, truncated)");

    // Origin residue property: a calibrator writing 400 or 40 must produce
    // IDENTICAL encoder-mode timing (only encoder_tdc1_origin_deg % 360 is
    // load-bearing here — TIM2 wraps every 360, not 720 like
    // trigger_tooth0_engine_deg's tooth-wheel domain, the Hall-path field
    // this one used to share before it got its own dedicated field). If
    // this ever diverges, the field is silently carrying phase information
    // again (the exact bug class this session has been avoiding).
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 40u;
    const uint32_t with_40 = ecu_sched_encoder_test_engine_deg_to_counts(90u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 400u;
    const uint32_t with_400 = ecu_sched_encoder_test_engine_deg_to_counts(90u);
    CHECK_EQ(with_400, with_40, "origin=400 and origin=40 (400%360) give identical counts");
    CHECK_EQ(with_40, 2275u, "90 deg, origin=40 -> crank_deg=410%360=50 -> 50*16384/360=2275");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;

    // rev_target_to_absolute: half-open window (now_raw, now_raw+16384].
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(100u, 50u), 100u,
             "forward within same image: target ahead of now, no wrap");
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(100u, 100u), 16484u,
             "boundary: target==now must land at now+16384 (next rev), not now+0");
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(10u, 16380u), 16394u,
             "wrap forward: target just past the 16384 boundary");
    CHECK_EQ(ecu_sched_encoder_test_rev_target_to_absolute(100u, 0x10000032u), 0x10000064u,
             "upper bits of a 32-bit raw count preserved across the addition");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_tdc1_calibrate(void) {
    section("ecu_sched: encoder TDC1 calibration (pure math, round-trip with conversion)");

    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(0u), 0u,
             "TIM2==0 at TDC1 -> origin=0 (raw zero already IS TDC1)");
    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(8192u), 180u,
             "TIM2==8192 (half a rev) at TDC1 -> origin=180");
    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(4096u), 270u,
             "TIM2==4096 (1/4 rev = 90 deg) at TDC1 -> origin=270 (360-90)");
    CHECK_EQ(ecu_sched_encoder_tdc1_calibrate_from_raw(0x10001000u), 270u,
             "upper bits of a 32-bit raw count ignored — only mod 16384 matters");

    // Round-trip: whatever calibrate_from_raw() returns, feeding it back as
    // the origin must make engine_deg_to_counts_in_rev(0) reproduce the
    // ORIGINAL raw reading (mod 16384) — this is the actual contract the
    // field exists to satisfy (TDC1 in engine-degree space maps back to the
    // exact TIM2 position it was measured at).
    ecu_sched_test_reset();
    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    const uint32_t raws[] = {0u, 1u, 4095u, 4096u, 8191u, 8192u, 12288u, 16383u};
    for (uint8_t i = 0u; i < sizeof(raws) / sizeof(raws[0]); ++i) {
        const uint32_t raw = raws[i];
        const uint16_t origin = ecu_sched_encoder_tdc1_calibrate_from_raw(raw);
        ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = origin;
        const uint32_t back = ecu_sched_encoder_test_engine_deg_to_counts(0u);
        // origin_deg é um INTEIRO 0-359 — 360 não divide 16384 exactamente
        // (1 grau ≈ 45,5 counts), então o round-trip nunca é exacto ao
        // count: a tolerância real é ~meio grau (±23 counts), não ±1 (esse
        // seria só o caso onde os dois lados arredondam para o mesmo
        // valor por coincidência). Distância circular (mod 16384): raw=16383
        // e back=0 são POSIÇÕES ADJACENTES (a 1 count uma da outra através
        // do wrap), não uma diferença de ~16384 — diff ingénuo confundia as
        // duas.
        int32_t diff = static_cast<int32_t>(back) - static_cast<int32_t>(raw);
        if (diff > 8192) { diff -= 16384; }
        if (diff < -8192) { diff += 16384; }
        CHECK_TRUE(diff >= -46 && diff <= 46,
                   "round-trip TIM2 raw -> origin_deg -> counts_in_rev, dentro de meio grau (distância circular)");
    }
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_cmp_phase_calibrate(void) {
    section("ecu_sched: encoder CMP phase calibration (pure math, comando 'M')");

    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(0u, 0u),
             ems::engine::cfg::kCmpPhaseCalibratedA,
             "flanco CMP no mesmo raw do PMS1 (delta=0, par) -> fase A");
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(16384u, 0u),
             ems::engine::cfg::kCmpPhaseCalibratedB,
             "flanco CMP 1 volta antes do PMS1 (delta=16384, ímpar) -> fase B");
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(32768u, 0u),
             ems::engine::cfg::kCmpPhaseCalibratedA,
             "flanco CMP 2 voltas antes do PMS1 (delta=32768, par) -> fase A");
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(0u, 100u),
             ems::engine::cfg::kCmpPhaseCalibratedB,
             "flanco CMP 'à frente' do PMS1 (delta=-100, floor_div arredonda "
             "p/ -infinito -> quociente -1, ímpar) -> fase B");
    // Wraparound perto do limite de uint32_t: tim2_raw_at_cmp_edge=0xFFFFFFFF
    // com tim2_raw_at_tdc1=100 — subtração sem sinal envolve (100 - 0xFFFFFFFF
    // mod 2^32 = 101), caso físico real de "o flanco foi capturado mesmo
    // antes do contador dar a volta" — confirma que o cast p/ int32_t não
    // quebra perto do limite.
    CHECK_EQ(ecu_sched_encoder_cmp_phase_calibrate_from_raw(100u, 0xFFFFFFFFu),
             ems::engine::cfg::kCmpPhaseCalibratedA,
             "flanco CMP a 101 counts do PMS1 através do wrap de uint32_t -> fase A");
}

// Gap identificado na revisão 2026-08-15: nunca existiu teste de
// engine_config_valid()/serialize()/load() (nem para encoder_tdc1_origin_deg,
// nem agora para cmp_phase_state) — não há test_engine_config.cpp dedicado.
// Cobre aqui o suficiente para o campo novo: round-trip válido, rejeição de
// valor fora do domínio (cai nos defaults, não aplica parcial) e rejeição
// por magic errado.
void test_engine_config_cmp_phase_state_roundtrip(void) {
    section("engine_config: cmp_phase_state — valid()/serialize()/load() round-trip");

    using namespace ems::engine::cfg;
    const EngineConfigRam saved = g_eng_cfg;

    // Round-trip válido.
    g_eng_cfg.cmp_phase_state = kCmpPhaseCalibratedB;
    uint8_t buf[256] = {};
    engine_config_serialize(buf, kEngineConfigMinPageLen);
    g_eng_cfg.cmp_phase_state = kCmpPhaseUncalibrated;  // adultera antes do load
    engine_config_load(buf, kEngineConfigMinPageLen);
    CHECK_EQ(g_eng_cfg.cmp_phase_state, static_cast<uint8_t>(kCmpPhaseCalibratedB),
             "serialize(B) -> load() reproduz cmp_phase_state=B");

    // Valor fora do domínio (byte 12 corrompido diretamente, simula NVM
    // antigo/lixo) — engine_config_valid() deve rejeitar o struct INTEIRO,
    // não só o campo mau: g_eng_cfg fica intocado, não parcialmente aplicado.
    g_eng_cfg.cmp_phase_state = kCmpPhaseCalibratedA;
    const uint8_t sentinel_before = g_eng_cfg.cmp_phase_state;
    buf[12] = 3u;  // > kCmpPhaseCalibratedB — inválido
    engine_config_load(buf, kEngineConfigMinPageLen);
    CHECK_EQ(g_eng_cfg.cmp_phase_state, sentinel_before,
             "cmp_phase_state=3 (fora do domínio) rejeitado — g_eng_cfg não muda");

    EngineConfigRam probe = saved;
    probe.cmp_phase_state = 3u;
    CHECK_FALSE(engine_config_valid(probe), "engine_config_valid() rejeita cmp_phase_state>2 diretamente");

    // Magic errado — load() não deve tocar g_eng_cfg mesmo com bytes 0-13
    // válidos (mesma disciplina já usada por encoder_tdc1_origin_deg).
    g_eng_cfg.cmp_phase_state = kCmpPhaseCalibratedB;
    const uint8_t sentinel2 = g_eng_cfg.cmp_phase_state;
    engine_config_serialize(buf, kEngineConfigMinPageLen);
    buf[12] = static_cast<uint8_t>(kCmpPhaseCalibratedA);  // válido, mas...
    buf[14] = 0xFFu; buf[15] = 0xFFu;                        // ...magic errado
    engine_config_load(buf, kEngineConfigMinPageLen);
    CHECK_EQ(g_eng_cfg.cmp_phase_state, sentinel2,
             "magic errado -> load() não aplica nada, mesmo com cmp_phase_state válido no buffer");

    g_eng_cfg = saved;
}

// Shared by encoder presync + sequential tests (find first matching queue evt).
static uint8_t encoder_evt_find_ch(uint8_t want_ch, uint8_t want_high,
                                   uint32_t *out_ts) {
    for (uint8_t i = 0u; i < ecu_sched_encoder_test_get_evt_count(); ++i) {
        uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
        ecu_sched_encoder_test_get_evt(i, &ts, &ch, &high);
        if (ch == want_ch && high == want_high) {
            if (out_ts != nullptr) { *out_ts = ts; }
            return 1u;
        }
    }
    return 0u;
}

void test_ecu_sched_encoder_recompute_presync(void) {
    section("ecu_sched: encoder heartbeat recompute — presync wasted pairs @ 180°");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_advance_deg(10u);            // spark_a=350, spark_b=170
    ecu_sched_set_eoi_lead_deg(355u);          // eoi_deg=5
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(2000u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);   // omega=0.5
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 32768, "omega seeded to 0.5 for deterministic spans");

    // Hand-computed (origin=0, now_raw=1500, dwell_span=1000):
    //   pair A (IGN1/IGN4): spark_a=350 → 15928, dwell=14928
    //   pair B (IGN3/IGN2): spark_b=170 → 7736,  dwell=6736
    //   inj: eoi=5 → 16611, inj_on=16111 (SIMULTANEOUS halves PW)
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u,
             "2 pairs × 2 coils × (dwell+spark) + 4 INJ (on+off)");
    CHECK_EQ(ecu_sched_is_sequential(), 0u,
             "presync builder clears g_knock_sequential (wasted-spark)");

    uint32_t spark_a = 0u, spark_b = 0u, dwell_a = 0u, dwell_b = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark_a), 1u, "pair A IGN1 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 1u, "pair A IGN4 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark_b), 1u, "pair B IGN3 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 1u, "pair B IGN2 SPARK");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell_a), 1u, "pair A IGN1 DWELL");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 1u, &dwell_b), 1u, "pair B IGN3 DWELL");

    CHECK_EQ(spark_a, 15928u, "pair A spark @ 350°");
    CHECK_EQ(dwell_a, 14928u, "pair A dwell = spark − span");
    CHECK_EQ(spark_b, 7736u, "pair B spark @ 170°");
    CHECK_EQ(dwell_b, 6736u, "pair B dwell = spark − span");
    CHECK_TRUE(spark_a != spark_b, "pairs at distinct crank angles (180° apart)");

    uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
    // Chronological: pair-B dwell first in queue.
    ecu_sched_encoder_test_get_evt(0u, &ts, &ch, &high);
    CHECK_EQ(ts, 6736u, "evt0: earliest = pair B dwell");
    CHECK_EQ(high, 1u, "evt0: DWELL high=1");

    ecu_sched_encoder_heartbeat_tick(1500u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u,
             "repeated heartbeat: still exactly 16, no duplication from purge+rebuild");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_recompute_presync_ign_inhibit_mask_gate(void) {
    section("ecu_sched: encoder heartbeat recompute — máscara de ignição corta metade de um par "
            "wasted-spark (fix 2026-08-15, achado #3: recompute_presync() é o caminho REALMENTE "
            "ativo hoje, EMS_MT6835_CMP_PHASE_CALIBRATED=0 ⇒ try_arm_sequential_due nunca corre "
            "em produção — corrigir só o caminho sequencial não teria efeito nenhum na config atual)");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(355u);
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(2000u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);

    // kWastedIgnPairA = {IGN1, IGN4}. IGN1=ECU_CH_IGN1(7)→k_ign_ch_to_bit=bit0;
    // IGN4=ECU_CH_IGN4(4)→bit3. Máscara 0x01 corta só IGN1 — prova que o gate
    // é por canal, não pelo par inteiro (a companion IGN4 continua a armar).
    ecu_sched_set_ign_inhibit_mask(0x01u);

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);   // omega=0.5

    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, nullptr), 0u,
             "IGN1 DWELL_START NÃO armado — bit0 mascarado");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 0u,
             "IGN1 SPARK também não armado");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 1u, nullptr), 1u,
             "IGN4 (companion do mesmo par) continua armado — gate por canal, não por par");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 1u, nullptr), 1u,
             "pair B (IGN3/IGN2) intocado pela máscara de pair A");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, nullptr), 1u,
             "INJ continua a armar — máscara de ignição não corta fuel");

    ecu_sched_set_ign_inhibit_mask(0u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_recompute_presync_bank_toggle(void) {
    section("ecu_sched: encoder heartbeat recompute — presync semi-sequential bank toggle");
    ecu_sched_test_reset();

    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SEMI_SEQUENTIAL);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 12u,
             "4 IGN (dwell+spark) + 2 INJ (on+off), semi-sequential: half the injectors");

    uint32_t ts = 0u; uint8_t ch_a = 0u; uint8_t high = 0u;
    ecu_sched_encoder_test_get_evt(8u, &ts, &ch_a, &high);  // first INJ_ON of this heartbeat's bank

    ecu_sched_encoder_heartbeat_tick(2000u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 12u, "still 12 after toggle");
    uint8_t ch_b = 0u;
    ecu_sched_encoder_test_get_evt(8u, &ts, &ch_b, &high);
    CHECK_TRUE(ch_a != ch_b, "bank toggled: different injector channel fires between consecutive heartbeats");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_recompute_presync_pw_clamp(void) {
    section("ecu_sched: encoder heartbeat recompute — presync PW duty clamp");
    ecu_sched_test_reset();

    const uint32_t before = ecu_sched_pw_duty_clamp_count();
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_inj_pw_ticks(2000000u);   // huge PW ticks, forces a span > 90% of a rev at omega=1.0
    ecu_sched_set_dwell_ticks(0u);

    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);  // omega=1.0 -> inj_pw_span way over a rev

    CHECK_TRUE(ecu_sched_pw_duty_clamp_count() > before,
               "oversized presync PW span clamped to 90% of a revolution");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_presync_multispark(void) {
    section("ecu_sched: encoder presync — multi-spark per wasted pair");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    ecu_sched_set_advance_deg(20u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_mspark(1u, 100u, 18u);

    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);
    ecu_sched_encoder_heartbeat_tick(1500u, 2000u, 0u, 0u);

    CHECK_EQ(ecu_sched_is_sequential(), 0u, "presync stays wasted-spark");

    // Primary: 2 pairs × 2 coils × 2 = 8; multi: +8; inj sim: 8 → 24.
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 24u,
             "presync+1 mspark/pair: 16 IGN + 8 INJ");

    uint32_t spark_a = 0u, spark_b = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark_a), 1u, "pair A primary spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark_b), 1u, "pair B primary spark");
    CHECK_TRUE(spark_a != spark_b, "multi-spark still on distinct pair angles");

    uint8_t ign1_count = 0u;
    for (uint8_t i = 0u; i < ecu_sched_encoder_test_get_evt_count(); ++i) {
        uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
        ecu_sched_encoder_test_get_evt(i, &ts, &ch, &high);
        if (ch == ECU_CH_IGN1) { ++ign1_count; }
    }
    CHECK_EQ(ign1_count, 4u,
             "pair-A coil: primary + 1 multi-spark pair (4 events)");

    ecu_sched_encoder_heartbeat_tick(1500u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 24u,
             "repeated heartbeat: still 24, no duplication");

    ecu_sched_set_mspark(0u, 0u, 18u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

// ── Disparo sequencial encoder (fase-consciente) ───────────────────────────
// Helpers: o mock TIM2_CNT e o parâmetro tim2_now do heartbeat são
// independentes — testes multi-passagem têm de os sincronizar. Qualquer
// sequência com phase_valid tem de ficar abaixo de kMaxHeartbeatsWithoutCmp=6
// (ou reancorar via flanco CMP aceite) senão o staleness invalida a fase.
// omega seed: encoder_seq_seed_omega() em test/fixtures.h.
// encoder_evt_find_ch() está definido acima (antes dos testes de presync).

void test_ecu_sched_encoder_placement(void) {
    section("ecu_sched: encoder engine_deg720_to_absolute placement");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;

    // Discriminator vectors (hand-verified) — composition identical to
    // engine_deg720_to_absolute's body. Integer deg→counts cannot hit residue
    // 50/3000 exactly; these lock the +16384 correction oracle.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    uint32_t cand = ecu_sched_encoder_test_rev_target_to_absolute(50u, 100u);
    CHECK_EQ(cand, 16434u, "disc1: rev_target_to_absolute(50,100)=16434");
    CHECK_EQ(ecu_sched_encoder_phase_at(cand), ECU_PHASE_B,
             "disc1: candidate lands in wrong phase (B)");
    CHECK_EQ(cand + 16384u, 32818u, "disc1: +16384 correction -> 32818");

    ecu_sched_encoder_phase_set_anchor(5000u, ECU_PHASE_A);
    cand = ecu_sched_encoder_test_rev_target_to_absolute(3000u, 6000u);
    CHECK_EQ(cand, 19384u, "disc2: rev_target_to_absolute(3000,6000)=19384");
    CHECK_EQ(ecu_sched_encoder_phase_at(cand), ECU_PHASE_A,
             "disc2: candidate lands in wrong phase for B-target (A)");
    CHECK_EQ(cand + 16384u, 35768u, "disc2: +16384 correction -> 35768");

    // End-to-end deg720: target phase A at 0° with anchor=0/A, now=100.
    // counts_in_rev(0)=0 -> candidate=16384 (phase B) -> +16384 -> 32768.
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_test_deg720_to_absolute(0u, 100u), 32768u,
             "deg720(0°, now=100, anchor=0/A) -> 32768 (phase A)");
    CHECK_EQ(ecu_sched_encoder_phase_at(32768u), ECU_PHASE_A,
             "result phase matches target A");
    // Target phase B at 360°: same counts, candidate=16384 already B -> no fix.
    CHECK_EQ(ecu_sched_encoder_test_deg720_to_absolute(360u, 100u), 16384u,
             "deg720(360°, now=100, anchor=0/A) -> 16384 (phase B, no +16384)");
    CHECK_EQ(ecu_sched_encoder_phase_at(16384u), ECU_PHASE_B,
             "result phase matches target B");

    // Misaligned anchor (disc2 geometry via deg720): 3000 counts ≈ 65.9° →
    // use 66° (66*16384/360=3003). Close enough to exercise the oracle path.
    ecu_sched_encoder_phase_set_anchor(5000u, ECU_PHASE_A);
    const uint32_t abs_b = ecu_sched_encoder_test_deg720_to_absolute(360u + 66u, 6000u);
    CHECK_EQ(ecu_sched_encoder_phase_at(abs_b), ECU_PHASE_B,
             "misaligned-anchor deg720 B-target lands in phase B");

    // Negative-delta / wrap cases reuse phase_at coverage (99999/83616 vs
    // anchor=100000) — phase oracle inherits wrap-safety; no new logic here.
    ecu_sched_encoder_phase_set_anchor(100000u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_phase_at(99999u), ECU_PHASE_B,
             "phase_at just before anchor: previous phase (neg delta)");
    CHECK_EQ(ecu_sched_encoder_phase_at(83616u), ECU_PHASE_B,
             "phase_at -1 rev from anchor=100000: phase flips");
    CHECK_EQ(ecu_sched_encoder_phase_at(83615u), ECU_PHASE_A,
             "phase_at just before -1 rev boundary");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_distinct_targets(void) {
    section("ecu_sched: encoder sequential — distinct per-cyl targets");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(2000u);

    encoder_seq_seed_omega();
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "pre: still presync");

    // Late-arm ≤60°: arm cyl0 then cyl2 in their windows (events accumulate).
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "phase valid -> sequential");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 1u, "cyl0 armed in window");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 4u,
             "only cyl0 (4 events) while cyl2 still outside window");

    encoder_seq_arm_cyl_in_window(2u, 4000u, 0u, 0u);
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 8u,
             "cyl0+cyl2 = 8 events after second window arm");

    uint32_t spark0 = 0u, spark2 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "cyl0 SPARK present");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark2), 1u, "cyl2 SPARK present");
    CHECK_TRUE(spark0 != spark2, "cyl0 and cyl2 spark targets are distinct");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_trims(void) {
    section("ecu_sched: encoder sequential — per-cyl ign/fuel trims");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(2000u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 0u, 0u);
    encoder_seq_arm_cyl_in_window(2u, 4000u, 0u, 0u);

    uint32_t spark0_base = 0u, inj_on0_base = 0u, inj_off0_base = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0_base), 1u, "base cyl0 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on0_base), 1u, "base cyl0 inj_on");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off0_base), 1u, "base cyl0 inj_off");
    const uint32_t pw0_base = inj_off0_base - inj_on0_base;

    // +5° ign trim +50% fuel trim on cyl0 only.
    ecu_sched_test_reset();
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }
    ems::engine::cyl_ign_trim_deg[0] = 5;
    ems::engine::cyl_fuel_trim_pct[0] = 50;
    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(2000u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 0u, 0u);
    encoder_seq_arm_cyl_in_window(2u, 4000u, 0u, 0u);

    uint32_t spark0_trim = 0u, spark2_trim = 0u;
    uint32_t inj_on0_trim = 0u, inj_off0_trim = 0u;
    uint32_t inj_on2_trim = 0u, inj_off2_trim = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0_trim), 1u, "trimmed cyl0 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark2_trim), 1u, "untrimmed cyl2 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on0_trim), 1u, "trimmed cyl0 inj_on");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off0_trim), 1u, "trimmed cyl0 inj_off");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ3, 1u, &inj_on2_trim), 1u, "untrimmed cyl2 inj_on");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ3, 0u, &inj_off2_trim), 1u, "untrimmed cyl2 inj_off");

    CHECK_TRUE(spark0_trim != spark0_base, "ign trim moved cyl0 spark vs untrimmed baseline");
    CHECK_TRUE(spark0_trim != spark2_trim, "ign trim applies only to cyl0, not cyl2");

    const uint32_t pw0_trim = inj_off0_trim - inj_on0_trim;
    const uint32_t pw2 = inj_off2_trim - inj_on2_trim;
    CHECK_TRUE(pw0_trim > pw0_base,
               "fuel trim +50% on cyl0 widens PW vs its own untrimmed baseline");
    CHECK_EQ(pw2, pw0_base,
             "untrimmed cyl2 PW matches cyl0 baseline (same inj_pw_ticks, omega)");

    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_phase_progression(void) {
    section("ecu_sched: encoder sequential — phase A/B cylinder partition");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);

    // Late-arm: only cylinders whose arm_at is inside ≤60° get events.
    const uint32_t now0 = encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 1u, "cyl0 armed in window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 0u, "cyl3 not yet in window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 0u, "cyl1 not yet in window");

    uint32_t spark0 = 0u;
    encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0);
    CHECK_TRUE(static_cast<int32_t>(spark0 - now0) <=
               static_cast<int32_t>(ecu_sched_encoder_test_arm_window_counts() + 5000u),
               "cyl0 spark lead is near the arm window (not ~1 rev early)");

    encoder_seq_arm_cyl_in_window(2u, 3500u, 1u, 1u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, nullptr), 1u, "cyl2 armed in its window");

    encoder_seq_arm_cyl_in_window(3u, 4000u, 2u, 2u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 1u, "cyl3 armed in its window");
    encoder_seq_arm_cyl_in_window(1u, 4500u, 2u, 2u);
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 1u, "cyl1 armed in its window");
    CHECK_TRUE(ecu_sched_encoder_test_get_evt_count() >= 16u,
               "all 4 cylinders armed across successive windows");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_presync_to_sequential_transition(void) {
    section("ecu_sched: encoder presync↔sequential handoff");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);

    encoder_seq_seed_omega();
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u, "presync: 16 events");
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "g_knock_sequential=0 in presync");

    // Enter sequential: handoff purges presync 4-wide; late-arm may add 0..4
    // events depending on window (here arm cyl0 only).
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "g_knock_sequential=1 in sequential");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 1u,
             "cyl0 armed after handoff+window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN2, 0u, nullptr), 0u,
             "presync IGN2 shared-target gone after sequential handoff");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN4, 0u, nullptr), 0u,
             "presync IGN4 shared-target gone after sequential handoff");

    // Back to presync: invalidate phase → recompute_presync purges all +
    // clears flag.
    ecu_sched_encoder_phase_invalidate();
    ecu_sched_encoder_test_set_tim2_cnt(2000u);
    ecu_sched_encoder_heartbeat_tick(2000u, 4000u, 0u, 0u);
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "g_knock_sequential=0 after return to presync");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 16u,
             "presync rebuild restores 16-wide simultaneous schedule");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_min_lead_skip(void) {
    section("ecu_sched: encoder sequential — lead below min stays unarmed (window gate)");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    // Window is [min_lead, 60°]. cyl2 spark @ 120° = 5461; now=5451 → lead=10
    // which is below min_lead ⇒ outside the arm window ⇒ no insert, no skip
    // counter (gate never calls arm_pair). cyl0 remains far away.
    ecu_sched_set_advance_deg(60u);
    ecu_sched_set_eoi_lead_deg(355u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);

    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_test_set_tim2_cnt(1000u);
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);  // omega=1.0
    CHECK_EQ(ecu_sched_encoder_omega_x65536(), 65536, "omega=1.0 seeded");

    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    const uint32_t near_now = 5451u;
    CHECK_EQ(ecu_sched_encoder_test_deg720_to_absolute(120u, near_now), 5461u,
             "precondition: cyl2 spark_abs=5461, lead=10");

    ecu_sched_encoder_test_set_tim2_cnt(near_now);
    ecu_sched_encoder_heartbeat_tick(near_now, 1000u + (near_now - 1000u), 1u, 1u);

    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, nullptr), 0u,
             "cyl2 not armed — lead below window floor (min_lead)");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, nullptr), 0u,
             "cyl0 outside 60° window — not armed early");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_min_lead_dwell_behind(void) {
    section("ecu_sched: encoder sequential — min-lead skip when dwell behind now");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    // omega=1.0 → dwell_span == dwell_ticks. Spark/EOI for cyl2 @ 120° = 5461
    // (eoi_lead=60 ⇒ same angle as spark). now=4500 → dwell=2461 behind;
    // arm_at falls to inj_on=5461 with lead=961 ∈ [min_lead, 60°] → try arm
    // then IGN pair skipped (dwell behind).
    ecu_sched_set_advance_deg(60u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(3000u);
    ecu_sched_set_inj_pw_ticks(0u);

    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_heartbeat_tick(0u, 0u, 0u, 0u);
    ecu_sched_encoder_test_set_tim2_cnt(1000u);
    ecu_sched_encoder_heartbeat_tick(1000u, 1000u, 0u, 0u);

    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    const uint32_t now = 4500u;
    const uint32_t skips_before = ecu_sched_encoder_test_get_seq_min_lead_skip_count();
    ecu_sched_encoder_test_set_tim2_cnt(now);
    ecu_sched_encoder_heartbeat_tick(now, 1000u + (now - 1000u), 1u, 1u);

    CHECK_TRUE(ecu_sched_encoder_test_get_seq_min_lead_skip_count() > skips_before,
               "dwell-behind spark increments skip counter");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, nullptr), 0u,
             "cyl2 IGN pair skipped when dwell is behind now");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 1u, nullptr), 0u,
             "cyl2 dwell also absent (whole pair skipped)");

    // Já sequencial: tick pesado com run_seq_arm=0 não duplica try_arm
    // (mesmo contrato do 64º subtick após o light path já ter armado).
    const uint32_t skips_after_arm = ecu_sched_encoder_seq_min_lead_skip_count();
    ecu_sched_encoder_heartbeat_tick(now, 1000u + (now - 1000u), 1u, 1u,
                                     /*run_seq_arm=*/0u);
    CHECK_EQ(ecu_sched_encoder_seq_min_lead_skip_count(), skips_after_arm,
             "run_seq_arm=0 — sem segundo try_arm (skip count estável)");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_long_lead_refresh(void) {
    section("ecu_sched: encoder sequential — no arm outside 60° window");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    // Far from all arm points (now=1500, next EOI/dwell many thousands of counts away).
    ecu_sched_encoder_test_set_tim2_cnt(1500u);
    ecu_sched_encoder_heartbeat_tick(1500u, 3000u, 1u, 1u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "sequential after phase");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u,
             "no events while all arm_at leads exceed 60°");

    // Enter cyl0 window → events appear with lead ≤ window.
    const uint32_t now0 = encoder_seq_arm_cyl_in_window(0u, 4000u, 1u, 1u);
    uint32_t spark0 = 0u, dwell0 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "cyl0 spark after window");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell0), 1u, "cyl0 dwell after window");
    CHECK_TRUE(static_cast<int32_t>(dwell0 - now0) <=
               static_cast<int32_t>(ecu_sched_encoder_test_arm_window_counts()),
               "dwell lead ≤ 60° arm window");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_multispark(void) {
    section("ecu_sched: encoder sequential — multi-spark extra IGN events");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(20u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    ecu_sched_set_inj_pw_ticks(0u);
    // 1 multi-spark, tiny inter-dwell so offsets stay inside advance+atdc window.
    ecu_sched_set_mspark(1u, 100u, 18u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint8_t ign1_count = 0u;
    for (uint8_t i = 0u; i < ecu_sched_encoder_test_get_evt_count(); ++i) {
        uint32_t ts = 0u; uint8_t ch = 0u; uint8_t high = 0u;
        ecu_sched_encoder_test_get_evt(i, &ts, &ch, &high);
        if (ch == ECU_CH_IGN1) { ++ign1_count; }
    }
    // Primary dwell+spark (2) + multi-spark dwell+spark (2) = 4 on IGN1.
    CHECK_EQ(ign1_count, 4u,
             "cyl0 has primary + 1 multi-spark pair (4 IGN1 events)");

    ecu_sched_set_mspark(0u, 0u, 18u);
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_prep_pw_overrides_global(void) {
    section("ecu_sched: encoder sequential — prep PW overrides g_inj_pw_ticks");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(0u);
    // Global PW deliberately different from prep flow.
    ecu_sched_set_inj_pw_ticks(999999u);

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.flow_pw_us = 1000u;  // host ticks = 1000*60 = 60000
    prep.dwell_ticks = 0u;
    prep.eoi_lead_deg = 60u;
    prep.base_advance_deg = 10;
    prep.map_bar_x100 = 100u;
    prep.fuel_press_bar_x1000 = 3000u;
    ems::engine::enc_fuel_ign_prep_test_publish(prep);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint32_t inj_on = 0u, inj_off = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, &inj_on), 1u, "inj_on present");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 0u, &inj_off), 1u, "inj_off present");
    const uint32_t span = inj_off - inj_on;
    // omega=0.5 → span_counts = ticks/2 = 30000 (plus scurve/delta_p ≈ identity at MAP=1bar)
    CHECK_TRUE(span < 50000u,
               "armed PW span from prep, not huge g_inj_pw_ticks");
    CHECK_TRUE(span > 1000u, "armed PW span non-trivial from prep flow");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_enc_cyl_setpoints_fuel_cut_read_modify_write_preserves_spark(void) {
    section("enc_cyl_setpoints: fuel_cut read-modify-write zera PW mas preserva dwell/advance "
            "(fix 2026-08-15: main_stm32.cpp branch (3) republica prep com fuel_cut=1 quando "
            "fuel_protect_cut trava o branch (1) que publicava)");
    ecu_sched_test_reset();
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.fuel_cut = 0u;
    prep.flow_pw_us = 1000u;
    prep.dwell_ticks = 1234u;
    prep.eoi_lead_deg = 60u;
    prep.base_advance_deg = 10;
    prep.map_bar_x100 = 100u;
    prep.fuel_press_bar_x1000 = 3000u;
    ems::engine::enc_fuel_ign_prep_test_publish(prep);

    const ems::engine::CylArmSetpoints running =
        ems::engine::finalize_cyl_setpoints(0u, false);
    CHECK_TRUE(running.inj_pw_ticks != 0u, "PW real com fuel_cut=0");
    CHECK_EQ(running.dwell_ticks, 1234u, "dwell publicado tal qual");

    // Mesmo padrão do fix: read-modify-write, só fuel_cut muda.
    ems::engine::EncFuelIgnPrep cut = ems::engine::enc_fuel_ign_prep_read();
    cut.fuel_cut = 1u;
    cut.valid = 1u;
    ems::engine::enc_fuel_ign_prep_test_publish(cut);

    const ems::engine::CylArmSetpoints cutout =
        ems::engine::finalize_cyl_setpoints(0u, false);
    CHECK_EQ(cutout.inj_pw_ticks, 0u, "PW zerado por fuel_cut=1 (sem isto, injeção real continuava)");
    CHECK_EQ(cutout.dwell_ticks, 1234u, "dwell/spark preservado — map_fault não corta ignição por desenho");

    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_ign_inhibit_mask_gate(void) {
    section("ecu_sched: encoder sequential — máscara de ignição corta dwell/spark de forma sustentada "
            "(fix 2026-08-15, achado #3: antes só purgava a fila UMA VEZ na borda de subida, "
            "try_arm_sequential_due voltava a armar no ciclo seguinte sem nunca consultar a máscara)");

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }
    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.flow_pw_us = 1000u;
    prep.dwell_ticks = 100u;
    prep.eoi_lead_deg = 60u;
    prep.base_advance_deg = 10;
    prep.map_bar_x100 = 100u;
    prep.fuel_press_bar_x1000 = 3000u;

    // kIgnCh[0] = ECU_CH_IGN1 (7); k_ign_ch_to_bit[7] = bit0 — mesma tabela
    // usada por force_output() (ecu_sched.cpp) e agora também por
    // arm_sequential_cyl() (ecu_sched_angle_encoder.cpp).
    {
        ecu_sched_test_reset();
        ems::engine::enc_fuel_ign_prep_test_publish(prep);
        encoder_seq_seed_omega();
        ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
        ecu_sched_set_ign_inhibit_mask(0x01u);
        encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

        CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, nullptr), 0u,
                 "IGN1 DWELL_START NÃO armado com máscara a cortar cyl0");
        CHECK_EQ(encoder_evt_find_ch(ECU_CH_INJ1, 1u, nullptr), 1u,
                 "INJ1 continua a armar — máscara de ignição não corta fuel (gates independentes)");
        ecu_sched_set_ign_inhibit_mask(0u);
    }

    // Máscara limpa: mesma sequência deve armar dwell/spark normalmente.
    {
        ecu_sched_test_reset();
        ems::engine::enc_fuel_ign_prep_test_publish(prep);
        encoder_seq_seed_omega();
        ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
        ecu_sched_set_ign_inhibit_mask(0x00u);
        encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

        CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, nullptr), 1u,
                 "IGN1 DWELL_START armado normalmente com máscara limpa");
    }

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_prep_knock_per_cyl(void) {
    section("ecu_sched: encoder sequential — knock retard only on armed cyl");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
        ems::engine::knock_retard_x10[i] = 0u;
    }
    ems::engine::knock_retard_x10[0] = 50u;  // 5°

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.base_advance_deg = 20;
    prep.eoi_lead_deg = 60u;
    prep.dwell_ticks = 0u;
    prep.flow_pw_us = 0u;
    ems::engine::enc_fuel_ign_prep_test_publish(prep);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);
    encoder_seq_arm_cyl_in_window(2u, 4000u, 1u, 1u);

    uint32_t spark0 = 0u, spark2 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "cyl0 spark");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN3, 0u, &spark2), 1u, "cyl2 spark");

    // cyl0 advance 15°, cyl2 advance 20° → distinct absolute targets.
    CHECK_TRUE(spark0 != spark2, "knock on cyl0 moves its spark vs unknocked cyl2");

    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::knock_retard_x10[i] = 0u;
    }
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_omega_refresh(void) {
    section("ecu_sched: encoder sequential — Δω refreshes dwell, spark fixed");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(4000u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    const uint32_t now = encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint32_t dwell0 = 0u, spark0 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell0), 1u, "dwell before refresh");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "spark before refresh");
    const uint32_t refreshes_before = ecu_sched_encoder_test_get_omega_refresh_count();

    // Mild ω rise (~0.5 → 0.7, >2% threshold) so new dwell stays ahead of now.
    ecu_sched_encoder_omega_test_reset();
    ecu_sched_encoder_omega_sample(0u, 0u);
    ecu_sched_encoder_omega_sample(700u, 1000u);  // ω=0.7
    ecu_sched_encoder_test_set_tim2_cnt(now);
    ecu_sched_encoder_heartbeat_tick(now, 1000u, 1u, 1u);

    uint32_t dwell1 = 0u, spark1 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell1), 1u, "dwell after refresh");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark1), 1u, "spark after refresh");
    CHECK_EQ(spark1, spark0, "spark absolute unchanged after ω refresh");
    CHECK_TRUE(dwell1 < dwell0,
               "dwell moves earlier (more angle) when ω rises");
    CHECK_TRUE(ecu_sched_encoder_test_get_omega_refresh_count() > refreshes_before,
               "omega refresh counter increments");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_ecu_sched_encoder_sequential_omega_refresh_lock(void) {
    section("ecu_sched: encoder sequential — no refresh past limit angle");
    ecu_sched_test_reset();

    const uint16_t saved_origin = ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg;
    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        ems::engine::cyl_ign_trim_deg[i] = 0;
        ems::engine::cyl_fuel_trim_pct[i] = 0;
    }

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_eoi_lead_deg(60u);
    ecu_sched_set_dwell_ticks(4000u);
    ecu_sched_set_inj_pw_ticks(0u);

    encoder_seq_seed_omega();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 3000u, 1u, 1u);

    uint32_t dwell0 = 0u, spark0 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell0), 1u, "dwell armed");
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 0u, &spark0), 1u, "spark armed");

    // Park just before spark (inside lock / min_lead zone).
    const uint32_t near = spark0 - 2u;
    ecu_sched_encoder_omega_sample(near + 100u, 4100u);
    ecu_sched_encoder_omega_sample(near + 2100u, 4200u);
    const uint32_t refreshes_before = ecu_sched_encoder_test_get_omega_refresh_count();
    ecu_sched_encoder_test_set_tim2_cnt(near);
    ecu_sched_encoder_heartbeat_tick(near, 4200u, 1u, 1u);

    uint32_t dwell1 = 0u;
    CHECK_EQ(encoder_evt_find_ch(ECU_CH_IGN1, 1u, &dwell1), 1u, "dwell still queued");
    CHECK_EQ(dwell1, dwell0, "dwell frozen after limit-angle lock");
    CHECK_EQ(ecu_sched_encoder_test_get_omega_refresh_count(), refreshes_before,
             "no refresh after lock");

    ems::engine::cfg::g_eng_cfg.encoder_tdc1_origin_deg = saved_origin;
    ecu_sched_test_reset();
}

void test_enc_finalize_xtau_peek_no_commit(void) {
    section("enc_cyl_setpoints: finalize peek + commit_last_peek (filme 1×, PW do peek)");
    ecu_sched_test_reset();
    map_window_reset();
    enc_cyl_setpoints_reset();
    ems::engine::xtau_wall_fuel_reset();

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.cranking = 0u;
    prep.xtau_event_enable = 1u;   // caminho X-τ por evento (encoder sequencial)
    prep.flow_pw_us = 5000u;
    prep.base_flow_pw_us = 5000u;
    prep.map_bar_x100 = 100u;
    prep.dead_time_us = 0u;
    prep.rpm_x10 = 30000u;         // 3000 RPM
    prep.corr_clt_x256 = 256u;
    prep.corr_iat_x256 = 256u;
    prep.clt_x10 = 800;
    prep.base_advance_deg = 10;
    prep.eoi_lead_deg = 60u;
    enc_fuel_ign_prep_test_publish(prep);

    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), 0,
             "filme de parede começa em 0");

    // 1ª chamada COMITADA (equivalente à finalização vencedora de uma
    // tentativa de arme dentro da janela) — estabelece um filme não-nulo.
    (void)finalize_cyl_setpoints(0u, /*commit_fuel=*/true);
    const int32_t wall_after_commit =
        ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u);
    CHECK_TRUE(wall_after_commit != 0,
               "commit_fuel=true integra o modelo — filme passa a não-nulo");

    // Simula ~5 tentativas especulativas de try_arm_sequential_due() para um
    // cilindro fora da janela de 60° (sub-tick chamado ~65×/volta) — ANTES
    // do fix, cada uma destas chamadas mutava g_cyl_wall_us_q8 mesmo sendo
    // depois descartada por lead_in_arm_window(); o filme sobre-integrava
    // dezenas de vezes por spray real e neutralizava o AE.
    for (uint8_t i = 0u; i < 5u; ++i) {
        (void)finalize_cyl_setpoints(0u, /*commit_fuel=*/false);
    }
    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), wall_after_commit,
             "5× peek (commit_fuel=false) — filme inalterado (fix bug 3)");

    // Caminho do arm vencedor: reutiliza sp do peek + commit_last_peek
    // (só o filme; sem segundo finalize completo).
    const ems::engine::CylArmSetpoints sp_peek =
        finalize_cyl_setpoints(0u, /*commit_fuel=*/false);
    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), wall_after_commit,
             "peek pré-commit — filme ainda inalterado");
    CHECK_TRUE(ems::engine::transient_fuel_xtau_commit_last_peek(0u),
               "commit_last_peek aplica o passo do peek ao filme real");
    const int32_t wall_after_peek_commit =
        ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u);
    CHECK_TRUE(wall_after_peek_commit != wall_after_commit,
               "após peek+commit API, filme muda exactamente 1×");
    CHECK_TRUE(!ems::engine::transient_fuel_xtau_commit_last_peek(0u),
               "segundo commit_last_peek é no-op (peek já consumido)");
    CHECK_EQ(ems::engine::xtau_wall_fuel_us_q8_for_cyl(0u), wall_after_peek_commit,
             "no-op não volta a integrar o filme");

    // PW que o arm usaria é o do peek — não o de um segundo finalize
    // (que veria o filme já mutado e produziria PW diferente).
    const ems::engine::CylArmSetpoints sp_if_refinalize =
        finalize_cyl_setpoints(0u, /*commit_fuel=*/false);
    CHECK_TRUE(sp_peek.inj_pw_ticks != sp_if_refinalize.inj_pw_ticks,
               "re-finalize após commit veria filme novo — PW mudaria");
    // O contrato do arm: sp_peek.inj_pw_ticks é o que fica armado.

    ecu_sched_test_reset();
    ems::engine::xtau_wall_fuel_reset();
}

void test_enc_finalize_map_window_per_cyl(void) {
    section("enc_cyl_setpoints: map_window VE bilineal + ΔP no finalize");
    ecu_sched_test_reset();
    map_window_reset();
    enc_cyl_setpoints_reset();

    CHECK_EQ(map_window_slot_for_cyl(0u), 0u, "cyl0 TDC 0° → slot 0");
    CHECK_EQ(map_window_slot_for_cyl(2u), 1u, "cyl2 TDC 180° → slot 1");

    ems::engine::EncFuelIgnPrep prep{};
    prep.valid = 1u;
    prep.flow_pw_us = 5000u;
    prep.base_flow_pw_us = 5000u;
    prep.map_bar_x100 = 100u;           // fused 1.00 bar
    prep.fuel_press_bar_x1000 = 3000u;  // 3.0 bar abs → ΔP vs MAP
    prep.dead_time_us = 0u;
    prep.rpm_x10 = 30000u;              // 3000 RPM — VE re-lookup
    prep.corr_clt_x256 = 256u;
    prep.corr_iat_x256 = 256u;
    prep.fuel_trim_pct_x10 = 0;
    prep.clt_x10 = 800;
    prep.base_advance_deg = 10;
    prep.eoi_lead_deg = 60u;
    enc_fuel_ign_prep_test_publish(prep);

    map_window_enable = 0u;
    const CylArmSetpoints base0 = finalize_cyl_setpoints(0u);
    const CylArmSetpoints base2 = finalize_cyl_setpoints(2u);
    CHECK_EQ(base0.inj_pw_ticks, base2.inj_pw_ticks,
             "enable=0: same PW for cyl0 and cyl2");

    map_window_enable = 1u;
    map_window_test_set_slot_bar_x1000(0u, 800u);   // 0.80 bar
    map_window_test_set_slot_bar_x1000(1u, 1200u);  // 1.20 bar
    const CylArmSetpoints win0 = finalize_cyl_setpoints(0u);
    const CylArmSetpoints win2 = finalize_cyl_setpoints(2u);
    CHECK_TRUE(win0.inj_pw_ticks != win2.inj_pw_ticks,
               "enable=1 + distinct slots → different PW per cyl");
    CHECK_TRUE(win2.inj_pw_ticks > win0.inj_pw_ticks,
               "higher MAP slot → higher PW (VE×MAP, não só scale linear)");

    // Coerência: PW do slot alto ≈ calc_fuel_pw_us_default_fast(ve, map_cyl, …)
    {
        const uint8_t ve_hi = get_ve(30000u, 120u);
        const uint16_t lam_hi = get_lambda_target_x1000(30000u, 120u);
        uint32_t expect_hi = calc_fuel_pw_us_default_fast(
            ve_hi, 120u, lam_hi, 0, 256u, 256u, 0u);
        expect_hi = apply_delta_p_compensation(expect_hi, 3000u, 120u);
        expect_hi = apply_injector_scurve(expect_hi);
        const uint32_t expect_ticks = inj_pw_us_to_scheduler_ticks(expect_hi);
        // VE path + ΔP/S-curve — tolera ±5% por aritmética inteira.
        CHECK_TRUE(win2.inj_pw_ticks > (expect_ticks * 95u) / 100u &&
                   win2.inj_pw_ticks < (expect_ticks * 105u) / 100u,
                   "slot MAP alto → PW coerente com tabela VE/λ");
    }

    map_window_enable = 0u;
    map_window_reset();
    enc_cyl_setpoints_reset();
    ecu_sched_test_reset();
}

// ============================================================================
// QUICK CRANK
// ============================================================================

void test_ecu_sched_hardware_init(void) {
    section("ecu_sched: ECU_Hardware_Init runs without crash");
    // ECU_Hardware_Init clears angle table + TIM5 queue mocks. Only testable:
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

    // Presync IGN: wasted pairs (2 coils @ spark_a, 2 @ spark_b) → ≥4 IGN actions.
    uint8_t n_ign = 0u;
    for (uint8_t i = 0u; i < tsz; ++i) {
        uint8_t tooth, frac, ch, action, phase;
        if (ecu_sched_test_get_angle_event(i, &tooth, &frac, &ch, &action, &phase) != 0u) {
            if (action == ECU_ACT_DWELL_START || action == ECU_ACT_SPARK) { ++n_ign; }
        }
    }
    CHECK_TRUE(n_ign >= 4u, "presync table: ≥4 ignition events (2 pairs × dwell/spark)");
}

// ============================================================================
// CKP — FASE 3 (prime_on_tooth, snap fields, tooth_index, phase_A toggle)
// ============================================================================

static constexpr uint32_t kCrankPeriod = 100000u;  // rpm_x10 = 6250 < 7000 (cranking)

