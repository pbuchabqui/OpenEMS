#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "app/datalog.h"
#include "hal/sd.h"

using namespace ems::app;
using namespace ems::hal;

namespace {

constexpr uint32_t kStart = kDatalogStartLba;

DatalogSample sample(uint32_t n) {
    DatalogSample s{};
    s.t_ms = 1000u + n * 10u;
    s.rpm_x10 = static_cast<uint16_t>(8000u + n);
    s.map_bar_x100 = 45u;
    s.tps_x10 = 120u;
    s.clt_x10 = -50;
    s.iat_x10 = 321;
    s.lambda_x1000 = 1002u;
    s.lambda_target_x1000 = 1000u;
    s.pw_us = 3456u;
    s.advance_x10 = -25;
    s.stft_x10 = 17;
    s.ltft_x10 = -9;
    s.status_bits = 0x0241u;
    s.cut_reasons = 0x0004u;
    s.sync_state = 3u;
    s.flags = 1u;
    return s;
}

void fresh_boot(uint32_t region_blocks) {
    sd_sim_reset();
    datalog_init();
    datalog_test_set_region_blocks(region_blocks);
}

// Avança até o log ficar ativo (a busca do fim consome uma leitura por poll).
void run_until_active() {
    for (int i = 0; i < 200 && datalog_stats().state != DatalogState::ACTIVE; ++i) {
        sd_poll();
        datalog_poll();
    }
}

void pump(uint32_t polls) {
    for (uint32_t i = 0u; i < polls; ++i) {
        sd_poll();
        datalog_poll();
    }
}

void push_n(uint32_t n, uint32_t first = 0u) {
    for (uint32_t i = 0u; i < n; ++i) {
        datalog_push(sample(first + i));
    }
}

}  // namespace

void test_datalog_block_format(void) {
    section("datalog: formato do bloco (cabeçalho, CRC, registro de 32 B)");

    fresh_boot(64u);
    CHECK_TRUE(datalog_stats().state == DatalogState::OFF, "antes do poll: OFF");
    run_until_active();
    CHECK_TRUE(datalog_stats().state == DatalogState::ACTIVE, "cartão limpo: ACTIVE");
    CHECK_EQ(datalog_stats().session, 1u, "cartão limpo: sessão 1");

    push_n(kDatalogRecordsPerBlock);
    pump(4u);
    CHECK_EQ(sd_sim_write_count(), 1u, "15 amostras fecham exatamente 1 bloco");
    CHECK_EQ(datalog_stats().blocks_written, 1u, "contador de blocos gravados");

    const uint8_t* b = sd_sim_block(kStart);
    uint32_t session = 99u, seq = 99u;
    CHECK_TRUE(datalog_block_valid(b, session, seq), "bloco gravado é válido (magic + CRC-32)");
    CHECK_EQ(session, 1u, "cabeçalho: sessão");
    CHECK_EQ(seq, 0u, "cabeçalho: sequência");

    // Vetor dourado: o mesmo está em tools/datalog/sdlog.py (selftest).
    static const uint8_t kGolden[32] = {
        0xE8, 0x03, 0x00, 0x00,  // t_ms = 1000
        0x40, 0x1F,              // rpm_x10 = 8000
        0x2D, 0x00,              // map = 45
        0x78, 0x00,              // tps = 120
        0xCE, 0xFF,              // clt = -50
        0x41, 0x01,              // iat = 321
        0xEA, 0x03,              // lambda = 1002
        0xE8, 0x03,              // alvo = 1000
        0x80, 0x0D,              // pw = 3456
        0xE7, 0xFF,              // avanço = -25
        0x11, 0x00,              // stft = 17
        0xF7, 0xFF,              // ltft = -9
        0x41, 0x02,              // status = 0x0241
        0x04, 0x00,              // cortes = 4
        0x03, 0x01,              // sync = 3, flags = 1
    };
    CHECK_TRUE(std::memcmp(b + kDatalogHeaderBytes, kGolden, 32u) == 0, "registro 0 = vetor dourado");

    uint8_t bad[512];
    std::memcpy(bad, b, sizeof(bad));
    bad[100] ^= 0x01u;
    CHECK_FALSE(datalog_block_valid(bad, session, seq), "um bit trocado invalida o CRC");
    std::memset(bad, 0, sizeof(bad));
    CHECK_FALSE(datalog_block_valid(bad, session, seq), "bloco zerado é inválido");
}

void test_datalog_sequence_and_latency(void) {
    section("datalog: sequência, LBAs contíguos e cartão lento");

    fresh_boot(64u);
    run_until_active();
    sd_sim_set_op_latency_polls(5u);
    push_n(kDatalogRecordsPerBlock * 4u);
    pump(100u);
    CHECK_EQ(sd_sim_write_count(), 4u, "4 blocos gravados com 5 polls de latência cada");
    CHECK_EQ(datalog_stats().records_dropped, 0u, "sem descarte quando o cartão acompanha");

    bool contiguous = true;
    for (uint32_t i = 0u; i < 4u; ++i) {
        uint32_t session = 0u, seq = 99u;
        if (!datalog_block_valid(sd_sim_block(kStart + i), session, seq) || seq != i) {
            contiguous = false;
        }
    }
    CHECK_TRUE(contiguous, "sequência 0..3 em LBAs consecutivos");
    uint32_t s5 = 0u, q5 = 0u;
    CHECK_FALSE(datalog_block_valid(sd_sim_block(kStart + 4u), s5, q5), "nenhum bloco além do 4º");
}

void test_datalog_ring_overflow(void) {
    section("datalog: anel cheio descarta sem bloquear");

    fresh_boot(64u);
    run_until_active();
    sd_sim_set_op_latency_polls(1000000u);   // cartão travado em BUSY
    push_n(kDatalogRecordsPerBlock * (kDatalogRingBlocks + 3u));
    pump(10u);
    CHECK_EQ(sd_sim_write_count(), 0u, "cartão travado: nada gravado");
    CHECK_EQ(datalog_stats().records_dropped,
             static_cast<uint32_t>(3u * kDatalogRecordsPerBlock),
             "3 blocos além do anel descartados (45 amostras)");
}

void test_datalog_resume_after_reboot(void) {
    section("datalog: retoma depois do reboot (busca do fim + nova sessão)");

    fresh_boot(64u);
    run_until_active();
    push_n(kDatalogRecordsPerBlock * 3u);
    pump(20u);
    CHECK_EQ(sd_sim_write_count(), 3u, "primeira sessão: 3 blocos");

    // Reboot: o RAM do firmware zera, o cartão fica como está.
    datalog_init();
    datalog_test_set_region_blocks(64u);
    run_until_active();
    CHECK_EQ(datalog_stats().session, 2u, "segunda sessão = 2");
    push_n(kDatalogRecordsPerBlock);
    pump(10u);
    uint32_t session = 0u, seq = 99u;
    CHECK_TRUE(datalog_block_valid(sd_sim_block(kStart + 3u), session, seq), "bloco novo logo após o fim anterior");
    CHECK_EQ(session, 2u, "bloco novo na sessão 2");
    CHECK_EQ(seq, 0u, "sequência reinicia por sessão");

    // Bloco cortado no meio (CRC inválido) conta como fim do log.
    sd_sim_block(kStart + 3u)[200] ^= 0xFFu;
    datalog_init();
    datalog_test_set_region_blocks(64u);
    run_until_active();
    CHECK_EQ(datalog_stats().session, 2u, "bloco corrompido é o fim: o último válido é da sessão 1, nova sessão 2");
    push_n(kDatalogRecordsPerBlock);
    pump(10u);
    CHECK_TRUE(datalog_block_valid(sd_sim_block(kStart + 3u), session, seq) && session == 2u,
               "o bloco corrompido é sobrescrito");
}

void test_datalog_region_full(void) {
    section("datalog: região esgotada");

    fresh_boot(4u);
    run_until_active();
    push_n(kDatalogRecordsPerBlock * 6u);
    pump(40u);
    CHECK_EQ(sd_sim_write_count(), 4u, "só os 4 blocos da região são gravados");
    CHECK_TRUE(datalog_stats().state == DatalogState::FULL, "estado FULL");
    CHECK_TRUE(sd_sim_block(kStart + 4u)[0] == 0u, "nada gravado além da região");
    push_n(kDatalogRecordsPerBlock);
    CHECK_EQ(sd_sim_write_count(), 4u, "FULL ignora novas amostras");

    // Reboot com a região já cheia.
    datalog_init();
    datalog_test_set_region_blocks(4u);
    run_until_active();
    for (int i = 0; i < 200; ++i) { pump(1u); }
    CHECK_TRUE(datalog_stats().state == DatalogState::FULL, "reboot com região cheia: FULL");
}

void test_datalog_failures(void) {
    section("datalog: falha de gravação e cartão ausente");

    fresh_boot(64u);
    run_until_active();
    sd_sim_set_fail_ops(true);
    push_n(kDatalogRecordsPerBlock);
    pump(20u);
    CHECK_TRUE(datalog_stats().state == DatalogState::FAULT, "3 falhas seguidas: FAULT");
    CHECK_EQ(datalog_stats().write_errors, 3u, "3 tentativas contadas");
    const uint32_t writes = sd_sim_write_count();
    push_n(kDatalogRecordsPerBlock * 2u);
    pump(20u);
    CHECK_EQ(sd_sim_write_count(), writes, "FAULT não grava mais");

    sd_sim_reset();
    sd_sim_set_present(false);
    datalog_init();
    datalog_test_set_region_blocks(64u);
    pump(50u);
    CHECK_TRUE(datalog_stats().state == DatalogState::OFF, "sem cartão: OFF");
    push_n(kDatalogRecordsPerBlock);
    CHECK_EQ(sd_sim_write_count(), 0u, "sem cartão: push é ignorado");

    sd_sim_set_present(true);   // cartão inserido depois do boot
    sd_init();
    run_until_active();
    CHECK_TRUE(datalog_stats().state == DatalogState::ACTIVE, "cartão inserido depois: ACTIVE");
}
