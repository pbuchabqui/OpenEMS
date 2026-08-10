#include "test/harness.h"
#include "test/fixtures.h"
#include "test/ui_helpers.h"

#include <cstdint>

#include "drv/encoder_sync.h"

using namespace ems::drv::encoder_sync;

void test_encoder_sync_cmp_edge(void) {
    section("encoder_sync: evaluate_cmp_edge() — span/multiple gate (task #12)");

    // Primeiro flanco (has_prev=false): só arma referência, não valida.
    {
        const CmpEdgeResult r = evaluate_cmp_edge(1000u, false, 0u, 0u);
        CHECK_TRUE(r.accepted, "primeiro flanco: sempre aceite (arma referência)");
        CHECK_EQ(r.multiple, 0u, "primeiro flanco: multiple=0 (não validado)");
        CHECK_EQ(r.reject_streak, 0u, "primeiro flanco: streak=0");
        CHECK_TRUE(!r.streak_resync, "primeiro flanco: sem resync");
    }

    // Span exacto (delta=32768): N=1, aceite.
    {
        const CmpEdgeResult r = evaluate_cmp_edge(32768u, true, 0u, 0u);
        CHECK_TRUE(r.accepted, "delta=32768 exacto: aceite");
        CHECK_EQ(r.multiple, 1u, "delta=32768 exacto: multiple=1");
        CHECK_EQ(r.reject_streak, 0u, "aceite: streak reset a 0");
    }

    // Dentro da tolerância (+199, tolerância=200): aceite.
    {
        const CmpEdgeResult r = evaluate_cmp_edge(32967u, true, 0u, 0u);
        CHECK_TRUE(r.accepted, "delta=32768+199 (dentro de ±200): aceite");
        CHECK_EQ(r.multiple, 1u, "multiple=1");
    }
    {
        const CmpEdgeResult r = evaluate_cmp_edge(32569u, true, 0u, 0u);
        CHECK_TRUE(r.accepted, "delta=32768-199 (dentro de ±200): aceite");
    }

    // Fora da tolerância (+201): rejeitado.
    {
        const CmpEdgeResult r = evaluate_cmp_edge(32969u, true, 0u, 0u);
        CHECK_TRUE(!r.accepted, "delta=32768+201 (fora de ±200): rejeitado");
        CHECK_EQ(r.multiple, 0u, "rejeitado: multiple=0");
        CHECK_EQ(r.reject_streak, 1u, "rejeitado: streak=1");
        CHECK_TRUE(!r.streak_resync, "1ª rejeição: ainda sem resync (limite=3)");
    }

    // Meia-volta (delta=16384, exactamente 1 revolução): claramente errado —
    // arredonda para N=1 mas o resto (-16384) fica muito fora da tolerância
    // (±200). Mesmo caso que motivou rejeitar a tolerância percentual do CKP
    // (±50% de 32768 = ±16384 teria aceite isto por pouco).
    {
        const CmpEdgeResult r = evaluate_cmp_edge(16384u, true, 0u, 0u);
        CHECK_TRUE(!r.accepted, "delta=16384 (meia-volta): rejeitado (resto fora de ±200)");
    }

    // Flanco perdido: delta≈2×32768 aceite como multiple=2.
    {
        const CmpEdgeResult r = evaluate_cmp_edge(65600u, true, 0u, 0u);
        CHECK_TRUE(r.accepted, "delta≈2×32768 (1 flanco perdido): aceite");
        CHECK_EQ(r.multiple, 2u, "multiple=2 (flanco perdido detectado)");
    }

    // 2 flancos perdidos: multiple=3.
    {
        const CmpEdgeResult r = evaluate_cmp_edge(98304u, true, 0u, 0u);
        CHECK_TRUE(r.accepted, "delta=3×32768: aceite");
        CHECK_EQ(r.multiple, 3u, "multiple=3");
    }

    // Além do múltiplo máximo aceite (kCmpMaxAcceptedMultiple=4): rejeitado,
    // mesmo caindo exactamente num múltiplo — a via de staleness é quem trata
    // ausência prolongada, não este gate por-flanco.
    {
        const CmpEdgeResult r = evaluate_cmp_edge(5u * kCmpSpanCounts, true, 0u, 0u);
        CHECK_TRUE(!r.accepted, "delta=5×32768 (além do máximo aceite): rejeitado");
    }

    // Reject-streak: 3 rejeições consecutivas -> streak_resync=true, streak
    // volta a 0 (chamador descarta a referência).
    {
        uint8_t streak = 0u;
        CmpEdgeResult r = evaluate_cmp_edge(0u, true, 0u, streak);  // delta=0: rejeitado
        CHECK_TRUE(!r.accepted, "1ª rejeição consecutiva");
        CHECK_EQ(r.reject_streak, 1u, "streak=1");
        streak = r.reject_streak;
        r = evaluate_cmp_edge(0u, true, 0u, streak);
        CHECK_EQ(r.reject_streak, 2u, "streak=2");
        streak = r.reject_streak;
        r = evaluate_cmp_edge(0u, true, 0u, streak);
        CHECK_TRUE(r.streak_resync, "3ª rejeição consecutiva: streak_resync=true");
        CHECK_EQ(r.reject_streak, 0u, "streak_resync: streak reposto a 0");
    }

    // Wrap-safe: cmp_angle_now < prev_cmp_angle por wrap de 32 bits — a
    // subtracção unsigned ainda dá o delta correcto.
    {
        const uint32_t prev = 0xFFFFFFFFu - 100u;
        const uint32_t now  = prev + kCmpSpanCounts;  // wrap around
        const CmpEdgeResult r = evaluate_cmp_edge(now, true, prev, 0u);
        CHECK_TRUE(r.accepted, "delta através do wrap de 32 bits: aceite");
        CHECK_EQ(r.multiple, 1u, "multiple=1 através do wrap");
    }
}

void test_encoder_sync_staleness(void) {
    section("encoder_sync: staleness_exceeded() — fallback FULL->HALF (task #12)");

    CHECK_TRUE(!staleness_exceeded(0u, false), "0 heartbeats: não excedeu (prod)");
    CHECK_TRUE(!staleness_exceeded(5u, false), "5 heartbeats: não excedeu (prod, limite=6)");
    CHECK_TRUE(staleness_exceeded(6u, false), "6 heartbeats: excedeu (prod, limite=6)");
    CHECK_TRUE(staleness_exceeded(100u, false), "100 heartbeats: excedeu (prod)");

    CHECK_TRUE(!staleness_exceeded(6u, true), "6 heartbeats: NÃO excedeu (bench, limite=60)");
    CHECK_TRUE(!staleness_exceeded(59u, true), "59 heartbeats: não excedeu (bench)");
    CHECK_TRUE(staleness_exceeded(60u, true), "60 heartbeats: excedeu (bench, limite=60)");
}
