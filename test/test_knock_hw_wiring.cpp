/**
 * @file test/test_knock_hw_wiring.cpp
 * @brief Teste de integração do wiring de knock através do caminho REAL do
 *        scheduler (arm_channel(), não knock_window_open() chamado
 *        directamente como test_aux_knock.cpp faz). É exactamente essa
 *        diferença que teria apanhado a regressão do commit f42c450 (bloco
 *        de knock_window_open()/knock_window_cycle_end() apagado por
 *        acidente de arm_channel() numa varredura de "dead code").
 *
 * Compilado duas vezes:
 *  - Suite principal (make host-test), EMS_KNOCK_HW_PRESENT=0 (default):
 *    afirma que a janela NUNCA abre — prova que o wiring restaurado fica
 *    inerte por omissão (sem hardware analógico de knock, DNP na v1).
 *  - `make host-test-knock-hw`, EMS_KNOCK_HW_PRESENT=1: afirma que a janela
 *    abre e segue cfg::kFiringOrder — prova que o wiring está correcto
 *    quando armado.
 */
#include "test/harness.h"
#include "test/fixtures.h"

#include <cstdint>

#include "engine/ecu_sched.h"
#include "engine/knock.h"
#include "engine/engine_config.h"
#include "engine/calibration.h"
#include "hal/board_pinout.h"
#include "drv/ckp.h"

using namespace ems::drv;
using namespace ems::engine;

void test_knock_window_scheduler_wiring(void) {
#if EMS_KNOCK_HW_PRESENT
    section("knock: arm_channel() abre janela real via scheduler sequencial (EMS_KNOCK_HW_PRESENT=1)");
#else
    section("knock: arm_channel() nunca abre janela por omissão (EMS_KNOCK_HW_PRESENT=0)");
#endif

    ecu_sched_test_reset();
    knock_init();
    ems::engine::cmp_window_open_tooth  = 0u;
    ems::engine::cmp_window_close_tooth = 0u;

    // Mesma sequência de test_ecu_sched_wasted_to_sequential (test_sched.cpp)
    // até entrar em sequencial — mesma mecânica de timing (2ª borda CMP cai
    // no centro da janela ±25% de tolerância).
    ckp_reach_full_sync();
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    cam_fire(g_ckp_cap);
    ckp_feed_n_then_gap(55u);
    ckp_feed_n_then_gap(55u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "pré-condição: entrou em sequencial");

    // Dente-a-dente daqui em diante (não ckp_feed_n_then_gap(), que é opaco
    // demais para observar transições intermédias) — reproduz exactamente a
    // sua forma interna (55 dentes normais + 1 dente de gap 3×período) para
    // manter tooth_index a envolver correctamente (o decoder só o reinicia
    // no gap detectado por razão de período) e cam_fire() a cada 2 "voltas"
    // (mesma cadência do setup acima) para não deixar cmp_confirms
    // envelhecer além de kMaxRevsWithoutCmp e reverter a wasted-spark.
#if EMS_KNOCK_HW_PRESENT
    // Regista transições de CILINDRO da janela — não de active/inactive: uma
    // vez aberta, a janela fica sempre "active" do ponto de vista de fora
    // (knock_window_cycle_end() fecha e knock_window_open() reabre no MESMO
    // par de chamadas, nunca há um instante observável com active=false
    // entre dois DWELL_START consecutivos). Sem prever à mão qual cilindro
    // dispara primeiro (depende da tabela de ângulos completa,
    // rebuild_sequential_cycle()); confirma que a SEQUÊNCIA observada bate
    // com cfg::kFiringOrder.
    int32_t first_cyl  = -1;
    int32_t second_cyl = -1;
    for (uint32_t rev = 0u; rev < 12u && second_cyl < 0; ++rev) {
        for (uint32_t t = 0u; t < 55u; ++t) {
            ckp_fire(kNormalPeriod);
            if (knock_test_window_active()) {
                const uint8_t cyl = knock_test_window_cyl();
                if (first_cyl < 0) {
                    first_cyl = static_cast<int32_t>(cyl);
                } else if (static_cast<uint8_t>(first_cyl) != cyl) {
                    second_cyl = static_cast<int32_t>(cyl);
                    break;
                }
            }
        }
        ckp_fire(kNormalPeriod * 3u);
        if ((rev % 2u) == 1u) { cam_fire(g_ckp_cap); }
    }

    CHECK_TRUE(first_cyl >= 0 && first_cyl <= 3,
               "janela abriu para um cilindro válido (0-3)");
    CHECK_TRUE(second_cyl >= 0 && second_cyl <= 3,
               "janela reabriu para o cilindro seguinte, também válido");
    CHECK_TRUE(first_cyl != second_cyl,
               "cilindros consecutivos são diferentes (não fica preso no mesmo)");

    // Próximo cilindro em cfg::kFiringOrder a seguir a first_cyl.
    uint8_t pos = 0u;
    for (; pos < 4u; ++pos) {
        if (cfg::kFiringOrder[pos] == static_cast<uint8_t>(first_cyl)) { break; }
    }
    const uint8_t expected_second = cfg::kFiringOrder[(pos + 1u) % 4u];
    CHECK_EQ(static_cast<uint8_t>(second_cyl), expected_second,
             "2º cilindro bate com kFiringOrder — wiring segue a ordem de disparo real");
#else
    // Mesmo drive, mas sem hardware: a janela nunca deve abrir.
    bool ever_active = knock_test_window_active();
    for (uint32_t rev = 0u; rev < 12u && !ever_active; ++rev) {
        for (uint32_t t = 0u; t < 55u && !ever_active; ++t) {
            ckp_fire(kNormalPeriod);
            ever_active = knock_test_window_active();
        }
        ckp_fire(kNormalPeriod * 3u);
        if ((rev % 2u) == 1u) { cam_fire(g_ckp_cap); }
    }
    CHECK_FALSE(ever_active,
                "EMS_KNOCK_HW_PRESENT=0: janela nunca abre mesmo em sequencial");
#endif
}

void test_knock_window_encoder_arm_wiring(void) {
#if EMS_KNOCK_HW_PRESENT
    section("knock: encoder arm_channel() abre janela em sequencial (EMS_KNOCK_HW_PRESENT=1)");
#else
    section("knock: encoder arm_channel() inerte em sequencial (EMS_KNOCK_HW_PRESENT=0)");
#endif
    ecu_sched_test_reset();
    knock_init();

    // omega seed (encoder_seq_seed_omega in fixtures)
    encoder_seq_seed_omega();

    ecu_sched_set_advance_deg(10u);
    ecu_sched_set_dwell_ticks(2000u);
    ecu_sched_set_inj_pw_ticks(0u);

    // Presync path: g_knock_sequential=0 → janela não deve abrir.
    ecu_sched_encoder_test_set_tim2_cnt(1500u);
    ecu_sched_encoder_heartbeat_tick(1500u, 3000u, 0u, 0u);
    CHECK_EQ(ecu_sched_is_sequential(), 0u, "pré: ainda presync");
    CHECK_FALSE(knock_test_window_active(),
                "presync encoder: knock window closed (g_knock_sequential=0)");

    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);
    encoder_seq_arm_cyl_in_window(0u, 4000u, 1u, 1u);
    CHECK_EQ(ecu_sched_is_sequential(), 1u, "phase valid → sequential");

#if EMS_KNOCK_HW_PRESENT
    CHECK_TRUE(knock_test_window_active(),
               "encoder sequential DWELL_START abre janela de knock");
    const uint8_t cyl = knock_test_window_cyl();
    CHECK_TRUE(cyl <= 3u, "knock cyl id válido");
#else
    CHECK_FALSE(knock_test_window_active(),
                "EMS_KNOCK_HW_PRESENT=0: encoder sequencial não abre janela");
#endif
    ecu_sched_test_reset();
}
