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

// Alias used by seed tests (same as normal tooth period)
static constexpr uint32_t kCrankPeriod = 100000u;  // rpm_x10 = 6250 < 7000 (cranking)

void test_ckp_rpm_math(void) {
    section("ckp: rpm_x10 from period_ns");
    CHECK_EQ(ckp_test_rpm_x10_from_period_ns(160000u),  62500u, "160000 ns → 6250.0 RPM");
    CHECK_EQ(ckp_test_rpm_x10_from_period_ns(1250000u),  8000u, "1250000 ns → 800.0 RPM");
    CHECK_EQ(ckp_test_rpm_x10_from_period_ns(0u),            0u, "period=0 → rpm=0 (safe)");
}

void test_ckp_initial_state(void) {
    section("ckp: initial state after reset");
    ckp_test_reset(); g_ckp_cap = 0u;
    const CkpSnapshot s = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(s.state), static_cast<uint8_t>(SyncState::WAIT_GAP), "state=WAIT_GAP");
    CHECK_EQ(s.rpm_x10, 0u,    "rpm=0");
    CHECK_EQ(s.tooth_index, 0u, "tooth_index=0");
}

void test_ckp_half_sync(void) {
    section("ckp: WAIT_GAP → HALF_SYNC on first valid gap");
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_feed_n_then_gap(kWheelNormalTeeth);
    const CkpSnapshot s = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(s.state), static_cast<uint8_t>(SyncState::HALF_SYNC), "HALF_SYNC");
    CHECK_TRUE(s.rpm_x10 > 0u, "rpm > 0 in HALF_SYNC");
}

void test_ckp_full_sync(void) {
    section("ckp: HALF_SYNC → FULL_SYNC on second valid gap");
    ckp_reach_full_sync();
    const CkpSnapshot s = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(s.state), static_cast<uint8_t>(SyncState::FULL_SYNC), "FULL_SYNC");
    CHECK_EQ(s.tooth_index, 0u, "tooth_index=0 at gap");
    CHECK_NEAR(static_cast<float>(s.rpm_x10), 62500.0f, 500.0f, "rpm ≈ 62500");
}

void test_ckp_tooth_index_increments(void) {
    section("ckp: tooth_index increments in FULL_SYNC");
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 5u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(ckp_snapshot().tooth_index, 5u, "tooth_index=5 after 5 teeth");
}

void test_ckp_instant_rpm_360(void) {
    section("ckp: instant RPM volta-a-volta (mesmo dente, 360°)");
    ckp_test_reset();
    CHECK_EQ(ckp_instant_rpm_x10(), 0u, "sem bordas: instant rpm = 0");

    ckp_reach_full_sync();
    // Duas voltas completas de roda 60-2 (57 normais + gap 3×):
    // o mesmo slot de dente repete com espaçamento de exactamente 1 volta =
    // 57×10000 + 30000 = 600000 ticks = 60 posições × 10000.
    ckp_feed_n_then_gap(kWheelNormalTeeth, kNormalPeriod);
    ckp_feed_n_then_gap(kWheelNormalTeeth, kNormalPeriod);
    // 37.5e9 / 600000 = 62500 (rpm×10) = 60 s × 62.5 MHz / (60 × 10000)
    CHECK_EQ(ckp_instant_rpm_x10(), 62500u,
             "dt de 1 volta (600000 ticks) → 6250.0 rpm");

    // Reset limpa timestamps e medida.
    ckp_test_reset();
    CHECK_EQ(ckp_instant_rpm_x10(), 0u, "reset zera instant rpm");
}

void test_ckp_loss_of_sync_too_many_teeth(void) {
    section("ckp: LOSS_OF_SYNC when gap missed (>63 teeth)");
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 64u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "LOSS_OF_SYNC");
}

void test_ckp_loss_of_sync_early_gap(void) {
    section("ckp: LOSS_OF_SYNC on early gap (<55 teeth)");
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 10u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kGapPeriod);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "LOSS_OF_SYNC on early gap");
}

void test_ckp_skip_after_silence(void) {
    section("ckp: skip de dentes pós-silêncio (ckp_skip_pulses_after_gap)");
    // Desligado (default 0): sync normal, nada descartado.
    ckp_skip_pulses_after_gap = 0u;
    const uint32_t base0 = g_dbg_skip_after_silence;
    ckp_reach_full_sync();
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "skip=0: FULL_SYNC");
    CHECK_EQ(g_dbg_skip_after_silence, base0, "skip=0: nenhum dente descartado");

    // Silêncio ≥ timeout de stall com FULL_SYNC ainda de pé (race com o
    // stall poll do main loop): a própria borda derruba o sync e é descartada,
    // tal como as N-1 seguintes.
    ckp_skip_pulses_after_gap = 3u;
    const uint32_t base1 = g_dbg_skip_after_silence;
    ckp_fire(13000000u);  // > 12.5M ticks (200 ms) → silêncio
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC),
             "silêncio em FULL_SYNC → drop imediato");
    ckp_fire(kNormalPeriod);
    ckp_fire(kNormalPeriod);
    CHECK_EQ(g_dbg_skip_after_silence - base1, 3u, "3 dentes descartados");

    // Depois do skip o bootstrap recomeça limpo e o sync recupera normalmente.
    ckp_feed_n_then_gap(kWheelNormalTeeth);
    ckp_feed_n_then_gap(kWheelNormalTeeth);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "re-sync após skip");
    CHECK_EQ(g_dbg_skip_after_silence - base1, 3u,
             "dentes normais não são descartados");

    ckp_skip_pulses_after_gap = 0u;  // isolamento entre testes
}

void test_ckp_noise_rejection(void) {
    section("ckp: glitch < kMinToothTicks rejected");
    ckp_reach_full_sync();
    const CkpSnapshot before = ckp_snapshot();
    ckp_fire(10u);  // < 50 ticks
    const CkpSnapshot after = ckp_snapshot();
    CHECK_EQ(static_cast<uint8_t>(after.state), static_cast<uint8_t>(SyncState::FULL_SYNC),
             "FULL_SYNC maintained after glitch");
    CHECK_EQ(after.rpm_x10, before.rpm_x10, "rpm unchanged after glitch");
}

void test_ckp_stall_poll(void) {
    section("ckp: stall_poll detects stopped engine");
    ckp_reach_full_sync();
    // Produção: kMinStallTimeoutTicks = 12_500_000 (200 ms @ 62.5 MHz).
    const uint32_t stale = ckp_snapshot().last_tim5_capture + 13000000u;
    CHECK_TRUE(ckp_stall_poll(stale), "stall_poll=true after 208ms (kMinStallTimeoutTicks)");
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "LOSS_OF_SYNC after stall");

    // Bench-mode relaxa o timeout para 2 s (gaps de RMT restart do estimulador).
    section("ckp: stall_poll bench-mode usa timeout relaxado (2s)");
    ems::drv::sensors_set_bench_clt_iat(true, 900, 400);
    ckp_reach_full_sync();
    const uint32_t base = ckp_snapshot().last_tim5_capture;
    CHECK_FALSE(ckp_stall_poll(base + 13000000u), "bench: sem stall a 208ms");
    CHECK_TRUE(ckp_stall_poll(base + 130000000u), "bench: stall a 2.08s");
    ems::drv::sensors_set_bench_clt_iat(false, 0, 0);
}

void test_ckp_phantom_rpm_unsync(void) {
    section("ckp: RPM só reportado com sync (ruído em CKP desligado = 0 RPM)");
    ckp_test_reset(); g_ckp_cap = 0u;
    // Bordas periódicas de ruído (ex.: 60 Hz de rede) sem gap → sem sync.
    // RPM reportado deve ficar em 0 mesmo com capturas contínuas.
    for (uint32_t i = 0; i < 6u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::WAIT_GAP), "sem sync (WAIT_GAP)");
    CHECK_EQ(ckp_snapshot().rpm_x10, 0u, "ruído sem sync → RPM 0");
    // Ruído prolongado (simula 60 Hz contínuo por >58 dentes, sem gap)
    for (uint32_t i = 0; i < 200u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(ckp_snapshot().rpm_x10, 0u, "RPM continua 0 com ruído contínuo");
    // Com sync real (gap presente) o RPM aparece normalmente.
    ckp_reach_full_sync();
    for (uint32_t i = 0; i < 3u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_TRUE(ckp_snapshot().rpm_x10 != 0u, "com sync, RPM reportado");
    // E o stall_poll decai o RPM residual após perda de captura sem sync
    // (caso: sync perdido com rpm_x10 ainda populado).
    const uint32_t base = g_ckp_cap;
    ckp_stall_poll(base + 13000000u);  // stall → LOSS_OF_SYNC + rpm=0
    CHECK_EQ(ckp_snapshot().rpm_x10, 0u, "stall zera RPM");
}

void test_ckp_rpm_jump_recovery(void) {
    section("ckp: salto de RPM 3.75x recupera (deadlock do gap-como-normal)");
    // Reproduz a bancada: sync a "800 RPM" e salto p/ "3000" (periodo /3.75).
    // Com a media defasada, dente novo=SPIKE e gap novo cai na banda normal
    // (0.8x media) zerando o contador — sem o decaimento, re-bootstrap nunca
    // dispara e o CKP perfeito fica rejeitado para sempre.
    ckp_reach_full_sync(kNormalPeriod);               // "800 RPM"
    const uint32_t fast = kNormalPeriod * 100u / 375u; // periodo /3.75
    // 10 revolucoes no RPM novo: 57 dentes + gap 3x
    for (uint32_t rev = 0; rev < 10u; ++rev) {
        for (uint32_t i = 0; i < 57u; ++i) { ckp_fire(fast); }
        ckp_fire(fast * 3u);
    }
    const CkpSnapshot s = ckp_snapshot();
    CHECK_TRUE(s.state == SyncState::HALF_SYNC || s.state == SyncState::FULL_SYNC,
               "re-sincronizado apos salto 3.75x");
    CHECK_TRUE(s.rpm_x10 > 0u, "RPM reportado no novo regime");
}

void test_ckp_stall_poll_no_false_positive(void) {
    section("ckp: stall_poll false when teeth are recent");
    ckp_reach_full_sync();
    CHECK_FALSE(ckp_stall_poll(ckp_snapshot().last_tim5_capture + 1000u), "no stall");
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "FULL_SYNC maintained");

    // Corrida real de bancada (7 falsos stalls/180s a 700 RPM): a ISR captura
    // um dente ENTRE a leitura de TIM5_CNT no main loop e a comparação —
    // prev_capture fica à frente de tim5_cnt_now e a subtração unsigned dava
    // ~2^32 → falso stall. Elapsed negativo tem de ser tratado como "recente".
    section("ckp: stall_poll imune a captura à frente do CNT lido (corrida ISR)");
    ckp_reach_full_sync();
    const uint32_t cap = ckp_snapshot().last_tim5_capture;
    CHECK_FALSE(ckp_stall_poll(cap - 5000u), "captura 5000 ticks à frente → sem stall");
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC),
             "FULL_SYNC preservado na corrida");
    CHECK_TRUE(ckp_snapshot().rpm_x10 != 0u, "RPM preservado na corrida");
}


void test_ckp_cmp_glitch_count(void) {
    section("ckp: ckp_get_cmp_glitch_count on invalid cam timing");

    ckp_reach_full_sync();
    // ckp_test_reset() inside ckp_reach_full_sync() now also resets s_prev_cmp_capture.

    // First cam edge: s_prev_cmp_capture=0 → skip validation → always accepted.
    const uint32_t cap1 = g_ckp_cap;
    cam_fire(cap1);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "first cam edge always accepted");

    // Second cam edge too soon (delta=100, expected=58×10000=580000) → glitch
    cam_fire(cap1 + 100u);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 1u, "cam edge too soon → glitch counted");
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSORS — SEGUNDA FASE
// ═══════════════════════════════════════════════════════════════════════════

void test_ckp_prime_on_tooth(void) {
    section("ckp: prime_on_tooth generates prime pulse after bootstrap + target teeth");

    // crank_prime_tooth = 3. Bootstrap = first 3 teeth (prime not called).
    // Teeth 4, 5, 6 call prime_on_tooth with count=1,2,3. At count=3: fires.
    quick_crank_reset();
    ckp_test_reset(); g_ckp_cap = 0u;
    quick_crank_set_prime_context(-400, 500);  // CLT=-40°C, cold engine

    // rpm_x10 = 625000000 / 100000 = 6250 < crank_exit=7000 → cranking mode
    for (uint32_t i = 0u; i < 7u; ++i) { ckp_fire(kCrankPeriod); }

    const uint32_t prime_pw = quick_crank_consume_prime();
    CHECK_TRUE(prime_pw > 0u, "prime pulse generated at 7th cranking tooth (4 post-bootstrap)");
    CHECK_TRUE(prime_pw <= 30000u, "prime_pw ≤ max clamp (30 ms)");

    // One-shot: second consume returns 0
    CHECK_EQ(quick_crank_consume_prime(), 0u, "prime is one-shot");

    // After reset: no prime pending
    quick_crank_reset();
    CHECK_EQ(quick_crank_consume_prime(), 0u, "no prime after quick_crank_reset");
}

void test_ckp_snap_fields(void) {
    section("ckp: snap fields accurate in FULL_SYNC");

    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    const auto snap = ckp_snapshot();

    // tooth_period_ns = kNormalPeriod ticks * 16 ns/tick
    CHECK_EQ(snap.tooth_period_ns, kNormalPeriod * 16u,
             "tooth_period_ns = ticks × 16");

    // predicted period > 0 (set by predict_next_period_ticks on last normal tooth)
    CHECK_TRUE(snap.predicted_tooth_period_ns > 0u,
               "predicted_tooth_period_ns > 0");

    // With constant period, predicted ≈ actual (trend=0)
    CHECK_EQ(snap.predicted_tooth_period_ns, kNormalPeriod * 16u,
             "predicted_tooth_period_ns = actual at constant speed");

    // rpm_x10 = 625000000 / kNormalPeriod
    CHECK_EQ(snap.rpm_x10, 625000000u / kNormalPeriod, "rpm_x10 correct");

    // last_tim5_capture is updated with each tooth
    CHECK_TRUE(snap.last_tim5_capture > 0u, "last_tim5_capture > 0 after teeth");
}

void test_ckp_tooth_index_progression(void) {
    section("ckp: tooth_index increments; missing gap at 58 → LOSS (no wrap re-fire)");

    g_ckp_cap = 0u;
    ckp_reach_full_sync();  // tooth_index=0 at FULL_SYNC gap
    const uint16_t idx0 = ckp_snapshot().tooth_index;
    CHECK_EQ(idx0, 0u, "tooth_index=0 immediately after gap");

    ckp_fire(kNormalPeriod);
    CHECK_EQ(ckp_snapshot().tooth_index, 1u, "tooth_index=1 after 1 tooth");

    ckp_fire(kNormalPeriod);
    CHECK_EQ(ckp_snapshot().tooth_index, 2u, "tooth_index=2 after 2 teeth");

    // Advance to tooth 57 (fires 55 more teeth: 2+55=57)
    for (uint32_t i = 0u; i < 55u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_EQ(ckp_snapshot().tooth_index, 57u, "tooth_index=57 after 57 teeth");
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::FULL_SYNC), "FULL_SYNC at tooth 57");

    // 58th normal without accepted gap must NOT wrap to 0 (would re-schedule
    // teeth 0..N on a phantom second half-rev). Hardening: LOSS_OF_SYNC.
    ckp_fire(kNormalPeriod);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC),
             "missing gap at tooth 58 → LOSS_OF_SYNC (no index wrap)");
}

void test_ckp_phase_toggle(void) {
    section("ckp: CMP corrects phase_A to kCmpRefHalf; glitch is ignored");

    // CMP does NOT just toggle — it SETS phase_A to kCmpRefHalf (=0→true) at
    // the NEXT gap via advance_phase_half(). A glitch is rejected; the gap then
    // only performs the normal phase toggle.

    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_reach_full_sync();

    // First cam: arms s_prev only. Second: temporal OK + cmp_phase_pending.
    // Timestamps are absolute TIM5 capture values (not relative to g_ckp_cap alone).
    const uint32_t cam_arm = g_ckp_cap + kNormalPeriod * 58u;
    const uint32_t cam_ok1 = cam_arm + kNormalPeriod * 116u;
    cam_fire(cam_arm);
    cam_fire(cam_ok1);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "first pair of cam edges not glitches");
    ckp_feed_n_then_gap(kWheelNormalTeeth);  // gap applies pending CMP correction
    CHECK_EQ(ckp_snapshot().phase_A, true, "first CMP corrects phase_A to kCmpRefHalf (true)");

    // Next validated cam: delta = 116× from last accepted (cam_ok1).
    const uint32_t cam_ok2 = cam_ok1 + kNormalPeriod * 116u;
    cam_fire(cam_ok2);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 0u, "follow-up cam edge not a glitch");
    ckp_feed_n_then_gap(kWheelNormalTeeth);
    CHECK_EQ(ckp_snapshot().phase_A, true, "second CMP corrects phase_A to kCmpRefHalf (true)");

    // Glitch: delta from prev valid CMP is too small → rejected.
    cam_fire(cam_ok2 + 10u);
    CHECK_EQ(ckp_get_cmp_glitch_count(), 1u, "glitch cam edge counted");
    const bool phase_before = ckp_snapshot().phase_A;  // true
    ckp_feed_n_then_gap(kWheelNormalTeeth);
    CHECK_EQ(ckp_snapshot().phase_A, !phase_before, "after glitch: phase_A toggles normally (no CMP correction)");
}

// (all includes moved to top of file)

// ============================================================================
// TABLE3D
// ============================================================================


// A real tooth edge missed by the sensor must never shift the angle of the
// following teeth: the gap then arrives after 56 teeth → explicit sync loss.
void test_ckp_lost_tooth_loses_sync(void) {
    section("ckp: one lost tooth → LOSS_OF_SYNC at the next gap (no off-by-one angle)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_reach_full_sync();
    for (uint32_t i = 0u; i < 20u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(2u * kNormalPeriod);                 // tooth 21 missing (2 periods)
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC),
             "2-period interval at tooth 20 is a misplaced gap → LOSS_OF_SYNC at once");
    for (uint32_t i = 0u; i < 36u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kGapPeriod);                         // real gap: 36 teeth since loss
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::LOSS_OF_SYNC), "first gap after loss not trusted");
    ckp_feed_n_then_gap(kWheelNormalTeeth);
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::HALF_SYNC), "next full revolution → HALF_SYNC");
}

// Noise mid-tooth: before 0.5 period it is ignored; after, it stands in for
// the real edge, and the real edge then replaces it — never an extra tooth.
void test_ckp_noise_never_adds_or_loses_teeth(void) {
    section("ckp: noise pulse at f = 0.2 / 0.6 / 0.8 / 0.9 of a tooth keeps tooth_index exact");
    const uint32_t fr[4] = {2u, 6u, 8u, 9u};
    for (uint32_t k = 0u; k < 4u; ++k) {
        ckp_test_reset(); g_ckp_cap = 0u;
        ckp_reach_full_sync();
        for (uint32_t i = 0u; i < 10u; ++i) { ckp_fire(kNormalPeriod); }
        const uint32_t early = kNormalPeriod * fr[k] / 10u;
        ckp_fire(early);                          // noise
        ckp_fire(kNormalPeriod - early);          // real tooth 11
        CHECK_EQ(ckp_snapshot().tooth_index, 11u, "tooth_index exact after noise");
        for (uint32_t i = 0u; i < 46u; ++i) { ckp_fire(kNormalPeriod); }
        ckp_fire(kGapPeriod);
        CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
                 static_cast<uint8_t>(SyncState::FULL_SYNC), "gap still accepted (57 teeth)");
        CHECK_EQ(ckp_snapshot().tooth_index, 0u, "index 0 after gap");
    }
}

// Noise right AFTER a real tooth must not replace that tooth's timestamp.
void test_ckp_noise_after_tooth_keeps_real_edge(void) {
    section("ckp: noise 0.2 period after a real tooth leaves its capture untouched");
    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_reach_full_sync();
    for (uint32_t i = 0u; i < 10u; ++i) { ckp_fire(kNormalPeriod); }
    const uint32_t real_edge = g_ckp_cap;
    ckp_fire(kNormalPeriod / 5u);                 // noise at +0.2
    CHECK_EQ(ckp_snapshot().last_tim5_capture, real_edge, "real tooth capture kept");
    CHECK_EQ(ckp_snapshot().tooth_index, 10u, "index unchanged");
    ckp_fire(kNormalPeriod - kNormalPeriod / 5u); // next real tooth
    CHECK_EQ(ckp_snapshot().tooth_index, 11u, "next tooth counted once");
}

void test_ckp_acquisition_gap_window(void) {
    section("ckp: first gap accepted only with ratio 2..4 (MS42 window)");
    ckp_test_reset(); g_ckp_cap = 0u;
    for (uint32_t i = 0u; i < 10u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kNormalPeriod * 6u);                  // starter pause, not a gap
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::WAIT_GAP), "6x pause → still WAIT_GAP");
    ckp_fire(kNormalPeriod);
    for (uint32_t i = 0u; i < 5u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kNormalPeriod * 17u / 10u);           // 1.7x slow tooth
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::WAIT_GAP), "1.7x tooth → still WAIT_GAP");
    for (uint32_t i = 0u; i < 5u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(kNormalPeriod * 3u);                  // real gap
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::HALF_SYNC), "3x gap → HALF_SYNC");
}

void test_ckp_cmp_hygiene(void) {
    section("ckp: cam edges without sync never confirm; a loss drops a pending phase");
    ckp_test_reset(); g_ckp_cap = 0u;
    for (uint32_t i = 0u; i < 2u; ++i) { ckp_fire(kNormalPeriod); }
    cam_fire(g_ckp_cap);
    cam_fire(g_ckp_cap + kNormalPeriod * 120u);
    CHECK_EQ(ckp_snapshot().cmp_confirms, 0u, "WAIT_GAP: no cam confirmation");

    ckp_test_reset(); g_ckp_cap = 0u;
    ckp_reach_full_sync();
    const bool phase0 = ckp_snapshot().phase_A;
    const uint32_t cam_arm = g_ckp_cap + kNormalPeriod * 58u;
    cam_fire(cam_arm);
    cam_fire(cam_arm + kNormalPeriod * 116u);       // validated: phase pending
    for (uint32_t i = 0u; i < 10u; ++i) { ckp_fire(kNormalPeriod); }
    ckp_fire(2u * kNormalPeriod);                   // loss before the gap
    ckp_feed_n_then_gap(kWheelNormalTeeth);         // HALF_SYNC again
    CHECK_EQ(static_cast<uint8_t>(ckp_snapshot().state),
             static_cast<uint8_t>(SyncState::HALF_SYNC), "re-acquired");
    CHECK_EQ(ckp_snapshot().phase_A, !phase0, "stale cam phase not applied: plain toggle");
    CHECK_EQ(ckp_instant_rpm_x10(), 0u, "instant rpm cleared by the loss");
}
