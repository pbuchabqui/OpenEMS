/**
 * @file test/test_misfire_encoder.cpp
 * @brief Testes de host para engine/misfire_encoder.h/.cpp (fork MT6835/TIM2,
 *        modo encoder). Cobre: atribuição de cilindro por posição/fase
 *        (incluindo a fronteira 0°/360° — a classe de bug que
 *        g_tooth_to_cyl[phase][tooth] existe para evitar no caminho CKP),
 *        paridade de threshold/debounce com as constantes do caminho CKP, e
 *        inércia com EMS_MISFIRE_ENCODER_ENABLE desligado (default).
 */
#include "test/harness.h"

#include <cstdint>

#include "engine/misfire_encoder.h"
#include "engine/engine_config.h"
#include "engine/ecu_sched.h"

using namespace ems::engine;

void test_misfire_encoder_cyl_window_boundaries(void) {
    section("misfire_encoder: mapeamento cilindro×posição, fronteira de fase 0°/360°");

    // Determinismo: origem em 0° (default de compilação), independente do
    // que um teste anterior no mesmo processo possa ter escrito.
    cfg::g_eng_cfg.trigger_tooth0_engine_deg = 0u;
    misfire_encoder_init();

    // kFiringOrder={0,2,3,1} (engine_config.h) → cyl_tdc_deg: cyl0=0°,
    // cyl2=180°, cyl3=360°, cyl1=540°. Janela ~62° → 12 buckets de 256
    // counts (span_buckets = ceil(2823/256) = 12).
    // phase_idx 0=ECU_PHASE_A (tdc<360°), 1=ECU_PHASE_B (tdc>=360°).

    // cyl0: phase A, bucket 0..11.
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 0u), 0,
             "phase A, bucket0 → cyl0 (TDC exacto)");
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 11u * 256u), 0,
             "phase A, bucket11 → cyl0 (última posição da janela)");
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 12u * 256u), -1,
             "phase A, bucket12 → sem cilindro (logo a seguir à janela)");

    // cyl2: phase A, bucket 32..43 (180°×16384/360/256 = 32).
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 32u * 256u), 2,
             "phase A, bucket32 → cyl2 (TDC a 180°)");
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 20u * 256u), -1,
             "phase A, bucket20 → sem cilindro (entre cyl0 e cyl2)");

    // Fronteira 0°/360°: MESMO bucket (0), fase DIFERENTE → cilindro
    // DIFERENTE (cyl3, TDC=360°, não cyl0). Se a tabela ignorasse a fase e
    // só olhasse para a posição residual, isto colidiria com cyl0.
    CHECK_EQ(misfire_encoder_test_cyl_at(1u, 0u), 3,
             "phase B, bucket0 → cyl3 (TDC a 360° — mesma posição, fase distinta de cyl0)");
    CHECK_EQ(misfire_encoder_test_cyl_at(1u, 32u * 256u), 1,
             "phase B, bucket32 → cyl1 (TDC a 540°)");

    // Consulta com phase_idx inválido — defensivo, não deve corresponder a
    // nenhum cilindro.
    CHECK_EQ(misfire_encoder_test_cyl_at(2u, 0u), -1,
             "phase_idx inválido (>1) → -1, sem crash");
}

void test_misfire_encoder_reinit_after_trigger_offset_change(void) {
    section("misfire_encoder: re-init reconstrói g_cyl_window com novo trigger_tooth0_engine_deg (fix boot-order/runtime-staleness)");

    // Baseline: offset=0, cyl0 (TDC=0°) cai no bucket0/phase A.
    cfg::g_eng_cfg.trigger_tooth0_engine_deg = 0u;
    misfire_encoder_init();
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 0u), 0,
             "offset=0: bucket0/phase A -> cyl0");

    // Simula um tuner a escrever um novo trigger_tooth0_engine_deg em runtime
    // (sync_table_from_page(0x00) em ui_protocol_pages.cpp) e a reconstrução
    // que o fix agora dispara logo a seguir. offset=90 desloca a origem:
    // crank_deg(cyl0) = (0+360-90)%360 = 270 -> start_counts=270*16384/360=12288
    // -> start_bucket=48.
    cfg::g_eng_cfg.trigger_tooth0_engine_deg = 90u;
    misfire_encoder_init();
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 0u), -1,
             "offset=90 após re-init: bucket0/phase A já NÃO é cyl0 (tabela realmente mudou)");
    CHECK_EQ(misfire_encoder_test_cyl_at(0u, 48u * 256u), 0,
             "offset=90 após re-init: cyl0 agora no bucket48 (janela deslocada com o novo offset)");

    // Sem o fix (init nunca re-chamado após o offset mudar), a tabela ficaria
    // presa no layout offset=0 acima — este teste falharia antes do fix.
    cfg::g_eng_cfg.trigger_tooth0_engine_deg = 0u;
    misfire_encoder_init();
}

void test_misfire_encoder_threshold_debounce_and_inertness(void) {
    section("misfire_encoder: threshold/debounce (paridade CKP) + inércia sem ENABLE");

    cfg::g_eng_cfg.trigger_tooth0_engine_deg = 0u;
    misfire_encoder_init();
    misfire_encoder_reset();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);

    // Sequência com números calculados à mão (ver comentário em cada
    // secção do desenho): 4 amostras de "priming" fora de qualquer janela
    // (bucket60-63) para estabelecer Δticks=1000 como tendência normal,
    // depois 3 ciclos "entra em cyl0 (bucket0, phase A) com Δticks=1200
    // (1.2× lento) → sai para bucket12 (fora de janela)" — cada saída
    // avalia a janela fechada.
    misfire_encoder_on_sample(15360u, 0u);       // prime 1: só arma prev
    misfire_encoder_on_sample(15616u, 1000u);    // prime 2: Δ=1000, gap
    misfire_encoder_on_sample(15872u, 2000u);    // prime 3: Δ=1000, gap
    misfire_encoder_on_sample(16128u, 3000u);    // prime 4: Δ=1000, gap

    // Ciclo 1: entra cyl0 (bucket0, tim2=32768 → 2 voltas, phase A par),
    // Δ=1200 (lento); sai para bucket12 → avalia: pred=1000,
    // threshold=(1000×287)>>8=1121, power=1200>1121 → misfire.
    misfire_encoder_on_sample(32768u, 4200u);    // Δ=1200, entra cyl0
    misfire_encoder_on_sample(35840u, 5200u);    // Δ=1000, sai (bucket12)
    CHECK_EQ(misfire_encoder_test_get_debounce(0u), 1u,
             "ciclo 1: debounce[0]=1 (1ª janela lenta confirmada)");

    // Ciclo 2: pred agora reflecte a tendência do ciclo 1 (875 — ver
    // desenho), threshold=(875×287)>>8=980, power=1200>980 → misfire.
    misfire_encoder_on_sample(65536u, 6400u);    // Δ=1200, entra cyl0
    misfire_encoder_on_sample(68608u, 7400u);    // Δ=1000, sai
    CHECK_EQ(misfire_encoder_test_get_debounce(0u), 2u,
             "ciclo 2: debounce[0]=2 (2ª janela lenta consecutiva)");

    // Ciclo 3: 3ª janela lenta consecutiva → atinge kMisfireDebounceCycles
    // (3) → debounce RESETA para 0 (evento confirmado internamente), mas
    // event_count fica em 0 porque EMS_MISFIRE_ENCODER_ENABLE=0 (default
    // deste build) — a lógica de threshold/debounce corre sempre, só a
    // publicação DTC-facing é que fica atrás da flag.
    misfire_encoder_on_sample(98304u, 8600u);    // Δ=1200, entra cyl0
    misfire_encoder_on_sample(101376u, 9600u);   // Δ=1000, sai
    CHECK_EQ(misfire_encoder_test_get_debounce(0u), 0u,
             "ciclo 3: debounce[0] reseta a 0 (3ª confirmação processada)");
    CHECK_EQ(misfire_encoder_get_event_count(0u), 0u,
             "EMS_MISFIRE_ENCODER_ENABLE=0 (default): event_count nunca publica");
    CHECK_EQ(misfire_encoder_get_event_count(1u), 0u, "cyl1 nunca tocado");
    CHECK_EQ(misfire_encoder_get_event_count(2u), 0u, "cyl2 nunca tocado");
    CHECK_EQ(misfire_encoder_get_event_count(3u), 0u, "cyl3 nunca tocado");

    // Janela normal (Δ=875, exactamente o valor previsto) não deve gerar
    // debounce — regressão de segurança: threshold só dispara acima de
    // 1.12×, não em qualquer desvio.
    misfire_encoder_on_sample(131072u, 10475u);  // Δ=875, entra cyl0
    misfire_encoder_on_sample(134144u, 11475u);  // Δ=1000, sai
    CHECK_EQ(misfire_encoder_test_get_debounce(0u), 0u,
             "janela normal (Δ=previsto) não confirma debounce");

    // Inibição: janela lenta enquanto inibido não deve alterar debounce —
    // nem a acumulação nem a avaliação-na-saída correm sob inibição.
    misfire_encoder_set_all_inhibit(true);
    misfire_encoder_on_sample(163840u, 12675u);  // Δ=1200, entra cyl0 (inibido)
    misfire_encoder_on_sample(166912u, 13675u);  // Δ=1000, sai (inibido)
    CHECK_EQ(misfire_encoder_test_get_debounce(0u), 0u,
             "inibido: janela lenta não altera debounce");
    misfire_encoder_set_all_inhibit(false);
}
