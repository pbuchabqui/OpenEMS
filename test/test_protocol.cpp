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

void test_crc32_vectors(void) {
    section("crc32: vetores ISO-HDLC");
    const uint8_t check[9] = {'1','2','3','4','5','6','7','8','9'};
    CHECK_EQ(ems::hal::crc32_calc(check, 9u), 0xCBF43926u, "crc32(\"123456789\")=0xCBF43926");
    CHECK_EQ(ems::hal::crc32_calc(nullptr, 0u), 0x00000000u, "crc32(vazio)=0");
}

void test_legacy_protocol_regression(void) {
    section("ui_protocol legacy: intocado pelo envelope");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();

    uint8_t buf[80] = {};
    const uint8_t q = 'Q';
    ui_feed(&q, 1u);
    uint16_t n = ui_drain(buf, sizeof(buf));
    CHECK_EQ(n, 12u, "'Q' devolve 12 bytes crus (sem envelope)");
    CHECK_TRUE(memcmp(buf, "OpenEMS_v1.3", 12u) == 0, "assinatura OpenEMS_v1.3");

    const uint8_t c = 'C';
    ui_feed(&c, 1u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 2u && buf[0] == 0x00u && buf[1] == 0xAAu, "'C' → ACK+0xAA");

    // read legacy: 'r' page1 off0 len16
    const uint8_t rd[6] = {'r', 0x01u, 0x00u, 0x00u, 0x10u, 0x00u};
    ui_feed(rd, 6u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_EQ(n, 16u, "'r' page1 len16 → 16 bytes crus");
    CHECK_EQ(buf[0], ve_table[0][0], "primeiro byte = ve_table[0][0]");

    // write RAM-only legacy: 'x' page1 off0 len1 data=77
    const uint8_t wr[7] = {'x', 0x01u, 0x00u, 0x00u, 0x01u, 0x00u, 77u};
    ui_feed(wr, 7u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "'x' → ACK");
    CHECK_EQ(ve_table[0][0], 77u, "ve_table[0][0]=77 aplicado em RAM");

    const uint8_t d = 'd';
    ui_feed(&d, 1u);
    n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 1u && (buf[0] & 0x01u) != 0u, "'d' → 1 byte, página 1 dirty");
}

void test_ts_envelope_basic(void) {
    section("envelope TS: assinatura + comms test");
    ems::app::ui_test_reset();

    const uint8_t q = 'Q';
    EnvResp r = env_txn(&q, 1u);
    CHECK_TRUE(r.frame_ok, "resposta com framing válido");
    CHECK_TRUE(r.crc_ok, "CRC32 da resposta confere");
    CHECK_EQ(r.code, 0x00u, "code OK");
    CHECK_TRUE(r.len == 12u && memcmp(r.data, "OpenEMS_v1.3", 12u) == 0,
               "payload = assinatura");

    const uint8_t c = 'C';
    r = env_txn(&c, 1u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u && r.len == 1u &&
               r.data[0] == 0xAAu, "'C' → code OK + magic");

    const uint8_t z = 'Z';  // comando inexistente
    r = env_txn(&z, 1u);
    CHECK_TRUE(r.frame_ok && r.code == 0x83u, "comando desconhecido → 0x83");
}

void test_ts_envelope_crc_reject(void) {
    section("envelope TS: CRC corrompido rejeitado");
    ems::app::ui_test_reset();

    uint8_t frame[16] = {};
    const uint8_t q = 'Q';
    const uint16_t fl = env_frame(frame, &q, 1u);
    frame[fl - 1u] ^= 0xFFu;  // corrompe CRC
    ui_feed(frame, fl);

    uint8_t buf[32] = {};
    const uint16_t n = ui_drain(buf, sizeof(buf));
    CHECK_EQ(n, 7u, "resposta de erro tem 7 bytes (code sem dados)");
    CHECK_EQ(buf[2], 0x82u, "code 0x82 = CRC error");

    // parser recupera: próximo frame válido responde normalmente
    EnvResp r = env_txn(&q, 1u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "parser recuperado após CRC error");
}

void test_ts_envelope_read_write_burn(void) {
    section("envelope TS: r/w/b com página VE");
    ckp_test_reset(); g_ckp_cap = 0u;  // RPM=0 → burn permitido
    ems::app::ui_test_reset();

    // read: 'r' page1 off0 len16 (forma sem canId)
    const uint8_t rd[6] = {'r', 0x01u, 0x00u, 0x00u, 0x10u, 0x00u};
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u && r.len == 16u,
               "'r' 5-args → 16 bytes");
    CHECK_EQ(r.data[0], ve_table[0][0], "dados = ve_table");

    // read com canId à frente (forma TS canónica de 6 args)
    const uint8_t rd7[7] = {'r', 0x00u, 0x01u, 0x00u, 0x00u, 0x10u, 0x00u};
    r = env_txn(rd7, 7u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 16u,
               "'r' com canId=0 → 16 bytes");

    // chunk write: RAM-only, sem burn
    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t wr[10] = {'w', 0x01u, 0x00u, 0x00u, 0x04u, 0x00u, 11u, 22u, 33u, 44u};
    r = env_txn(wr, 10u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' chunk → OK");
    CHECK_EQ(ve_table[0][0], 11u, "VE[0][0]=11 aplicado em RAM");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before,
             "'w' envelope NÃO grava flash (RAM-only)");

    // dirty mask (16 bits LE no envelope)
    const uint8_t d = 'd';
    r = env_txn(&d, 1u);
    CHECK_TRUE(r.frame_ok && r.len == 2u && (r.data[0] & 0x01u) != 0u,
               "'d' → 2 bytes, página 1 dirty");

    // burn com motor parado
    const uint8_t burn[2] = {'b', 0x01u};
    r = env_txn(burn, 2u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'b' page1 @ 0 RPM → OK");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before + 1u,
             "burn gravou 1 página");

    r = env_txn(&d, 1u);
    CHECK_TRUE(r.frame_ok && (r.data[0] & 0x01u) == 0u, "dirty limpo após burn");
}

void test_eoi_blend_page0_roundtrip(void) {
    // Cobre o contrato de persistência do EOI blend (bytes 164-168 da page0)
    // que o boot-load em main_stm32.cpp restaura: serialize grava os globals
    // vivos; write-handler aplica-os de volta com clamp idle∈[0,719] — mesma
    // semântica do memcpy do boot-load.
    section("page0 EOI blend: serialize + write handler (persistência)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();

    // Serialize: globals vivos → bytes 164/166/168
    eoi_idle_deg = 60u; eoi_blend_rpm_lo = 2000u; eoi_blend_rpm_hi = 4000u;
    const uint8_t rd[6] = {'r', 0x00u, 0xA4u, 0x00u, 0x06u, 0x00u};  // page0 off=164 len=6
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 6u, "'r' page0 164..169 → 6B");
    CHECK_EQ(static_cast<uint16_t>(r.data[0] | (r.data[1] << 8u)),   60u, "serialize eoi_idle_deg");
    CHECK_EQ(static_cast<uint16_t>(r.data[2] | (r.data[3] << 8u)), 2000u, "serialize eoi_blend_rpm_lo");
    CHECK_EQ(static_cast<uint16_t>(r.data[4] | (r.data[5] << 8u)), 4000u, "serialize eoi_blend_rpm_hi");

    // Write-handler: aplica nos globals (idle=90, lo=1800, hi=3600)
    const uint8_t wr[12] = {'w', 0x00u, 0xA4u, 0x00u, 0x06u, 0x00u,
                            0x5Au, 0x00u, 0x08u, 0x07u, 0x10u, 0x0Eu};
    r = env_txn(wr, 12u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' page0 EOI → OK");
    CHECK_EQ(eoi_idle_deg,     90u,   "write aplicou eoi_idle_deg");
    CHECK_EQ(eoi_blend_rpm_lo, 1800u, "write aplicou eoi_blend_rpm_lo");
    CHECK_EQ(eoi_blend_rpm_hi, 3600u, "write aplicou eoi_blend_rpm_hi");

    // Clamp: idle=800 (0x0320) → 719, igual ao boot-load
    const uint8_t wc[8] = {'w', 0x00u, 0xA4u, 0x00u, 0x02u, 0x00u, 0x20u, 0x03u};
    r = env_txn(wc, 8u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' eoi_idle=800 → OK");
    CHECK_EQ(eoi_idle_deg, 719u, "write clampa eoi_idle_deg a 719");
}

void test_ts_envelope_burn_gate(void) {
    section("envelope TS: burn bloqueado com motor girando");
    ems::app::ui_test_reset();
    ckp_reach_full_sync();  // ~6250 RPM > kFlashWriteSafeRpmX10 (300 RPM)

    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t burn[2] = {'b', 0x01u};
    EnvResp r = env_txn(burn, 2u);
    CHECK_TRUE(r.frame_ok && r.code == 0x85u, "'b' com RPM alto → 0x85 busy");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before, "flash intocada");

    // legacy 'b' também bloqueado
    uint8_t buf[8] = {};
    const uint8_t lb[2] = {'b', 0x01u};
    ui_feed(lb, 2u);
    const uint16_t n = ui_drain(buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "legacy 'b' com RPM alto → NACK");

    ckp_test_reset(); g_ckp_cap = 0u;  // restaura RPM=0 p/ testes seguintes
}

void test_ts_axes_page(void) {
    section("página 11: eixos de tabela editáveis");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();

    // read: defaults serializados (rpm[0]=500, load[0]=20)
    const uint8_t rd[6] = {'r', 0x0Bu, 0x00u, 0x00u, 0x50u, 0x00u};
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 80u, "'r' page11 → 80 bytes");
    const uint16_t rpm0 = static_cast<uint16_t>(r.data[0] | (r.data[1] << 8u));
    const uint16_t load0 = static_cast<uint16_t>(r.data[40] | (r.data[41] << 8u));
    CHECK_EQ(rpm0, 500u, "rpm[0] default = 500");
    CHECK_EQ(load0, 20u, "load[0] default = 20 (0.20 bar)");

    // write monotónico: rpm 400..(passo 400), load 10..(passo 10)
    uint8_t wr[6u + 80u] = {'w', 0x0Bu, 0x00u, 0x00u, 0x50u, 0x00u};
    for (uint8_t i = 0u; i < 20u; ++i) {
        const uint16_t rv = static_cast<uint16_t>(400u + i * 400u);
        const uint16_t lv = static_cast<uint16_t>(10u + i * 10u);
        wr[6u + i * 2u]       = static_cast<uint8_t>(rv & 0xFFu);
        wr[7u + i * 2u]       = static_cast<uint8_t>(rv >> 8u);
        wr[6u + 40u + i * 2u] = static_cast<uint8_t>(lv & 0xFFu);
        wr[7u + 40u + i * 2u] = static_cast<uint8_t>(lv >> 8u);
    }
    r = env_txn(wr, sizeof(wr));
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' eixos monotónicos → OK");
    CHECK_EQ(kRpmAxisX10[0], 4000u, "kRpmAxisX10[0]=4000 (400 RPM ×10)");
    CHECK_EQ(kLoadAxisBarX100[19], 200u, "kLoadAxisBarX100[19]=200");

    // write não-monotónico rejeitado, eixos preservados
    wr[6u + 4u] = wr[6u + 0u];  // rpm[2] == rpm[0] → viola monotonicidade
    wr[7u + 4u] = wr[7u + 0u];
    r = env_txn(wr, sizeof(wr));
    CHECK_TRUE(r.frame_ok && r.code == 0x84u, "'w' não-monotónico → 0x84");
    CHECK_EQ(kRpmAxisX10[2], 12000u, "eixos preservados após rejeição");

    // buffer restaurado: 'r' devolve os eixos válidos, não o lixo rejeitado
    r = env_txn(rd, 6u);
    const uint16_t rpm2 = static_cast<uint16_t>(r.data[4] | (r.data[5] << 8u));
    CHECK_EQ(rpm2, 1200u, "'r' pós-rejeição devolve eixo válido (1200)");

    // burn página 11 → NVM slot 9
    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t burn[2] = {'b', 0x0Bu};
    r = env_txn(burn, 2u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'b' page11 → OK");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before + 1u, "NVM slot 9 gravado");

    // restaura defaults p/ não afetar outros testes
    const uint16_t rpm_def[20] = {500u, 750u, 1000u, 1250u, 1500u, 1750u, 2000u,
                                  2250u, 2500u, 2750u, 3000u, 3500u, 4000u, 4500u,
                                  5000u, 5500u, 6000u, 6500u, 7000u, 8000u};
    const uint16_t load_def[20] = {20u, 30u, 40u, 46u, 52u, 58u, 64u, 70u, 76u, 88u,
                                   94u, 100u, 110u, 130u, 160u, 190u, 220u, 250u,
                                   273u, 300u};
    CHECK_TRUE(table_axes_set(rpm_def, load_def), "defaults restaurados");
}

void test_ts_whole_page_800(void) {
    section("envelope TS: whole-page read de 800B (página 4, lambda 20×20)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();

    // 'r' page4 off0 count800 numa única transação — exatamente o caso que
    // motivou kEnvMaxChunk historicamente (Comm Manager pede página inteira).
    const uint8_t rd[6] = {'r', 0x04u, 0x00u, 0x00u, 0x20u, 0x03u};  // 0x0320=800
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u, "'r' 800B → OK");
    CHECK_EQ(r.len, 800u, "800 bytes devolvidos");
    // primeira célula = lambda_target[0][0] (default 1050)
    const int16_t l00 = static_cast<int16_t>(r.data[0] | (r.data[1] << 8u));
    CHECK_EQ(l00, (int16_t)1050, "lambda[0][0] = 1050");
    // última célula (célula 399) legível e plausível
    const int16_t l_last = static_cast<int16_t>(r.data[798] | (r.data[799] << 8u));
    CHECK_TRUE(l_last > 500 && l_last < 1200, "lambda[19][19] plausível");

    // whole-page 400B das páginas 1/2 também
    const uint8_t rd1[6] = {'r', 0x01u, 0x00u, 0x00u, 0x90u, 0x01u};  // 400
    r = env_txn(rd1, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 400u, "'r' VE 400B → OK");
    CHECK_EQ(r.data[0], ems::engine::ve_table[0][0], "VE[0][0] confere");
    CHECK_EQ(r.data[399], ems::engine::ve_table[19][19], "VE[19][19] confere");
}

void test_adaptives_reset_cmd_z(void) {
    section("protocolo: 'Z' learn session reset (STFT+accum+LTFT shadow)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    // Gera hits e STFT não-zero
    fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);
    CHECK_TRUE(fuel_ltft_accum_hits(mi, ri) > 0u, "hits antes do Z");
    CHECK_TRUE(fuel_get_stft_pct_x10() != 0 || g_stft_integrator_x1000 != 0,
               "STFT/integrador activo antes do Z");

    // Comando legacy 'Z' → ACK 0x00
    ems::app::ui_rx_byte(static_cast<uint8_t>('Z'));
    ems::app::ui_process();
    uint8_t ack = 0xFFu;
    bool got = false;
    for (int i = 0; i < 16; ++i) {
        if (ems::app::ui_tx_pop(ack)) { got = true; break; }
    }
    CHECK_TRUE(got, "Z produz byte TX");
    CHECK_EQ(ack, 0x00u, "Z → ACK OK");

    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 0u, "Z zera hits do acumulador");
    CHECK_EQ(fuel_get_stft_pct_x10(), 0, "Z zera STFT");
    CHECK_EQ(g_stft_integrator_x1000, 0, "Z zera integrador");
    CHECK_EQ(g_dbg_ltft_accum_accepted, 0u, "Z zera contadores accum");
}

void test_ltft_apply_cmd_y(void) {
    section("protocolo: 'Y' apply LEARN ready → VE (manual)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
    ltft_apply_burn_ve = 0u;

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // Aquece STFT e acumula até ready
    for (int n = 0; n < 150; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1020, 900, true, false, false, 5000u, 500u);
    }
    fuel_ltft_accum_reset();
    ve_table[mi][ri] = 100u;
    g_dbg_ltft_accum_commits = 0u;
    fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    for (uint16_t n = 0u; n < kLtftAccumReadyHits; ++n) {
        fuel_update_stft(30000u, 100u, 1000, 1000, 900, true, false, false, 5000u, 500u);
    }
    CHECK_TRUE(fuel_ltft_accum_cell_ready(mi, ri), "célula ready antes do Y");
    CHECK_EQ(ve_table[mi][ri], 100u, "VE intacta antes do Y");

    // Comando legacy 'Y' → ACK + n_commits
    while (true) {
        uint8_t dump = 0;
        if (!ems::app::ui_tx_pop(dump)) break;
    }
    ems::app::ui_rx_byte(static_cast<uint8_t>('Y'));
    ems::app::ui_process();
    uint8_t b0 = 0xFFu, b1 = 0xFFu;
    bool g0 = ems::app::ui_tx_pop(b0);
    bool g1 = ems::app::ui_tx_pop(b1);
    CHECK_TRUE(g0 && g1, "Y produz 2 bytes TX");
    CHECK_EQ(b0, 0x00u, "Y → ACK OK");
    CHECK_TRUE(b1 >= 1u, "Y → n_commits ≥ 1");
    CHECK_TRUE(g_dbg_ltft_accum_commits >= 1u, "commit registado");
    CHECK_TRUE(ve_table[mi][ri] > 100u, "Y alterou VE");
    CHECK_FALSE(fuel_ltft_accum_cell_ready(mi, ri), "stats limpos pós-Y");

    ve_table[mi][ri] = 88u;
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();
}

// Regressão: hit LEARN na célula dominante do trace VE (não no canto floor).
// Em 2000 rpm / 110 kPa exactos, floor = (1750,100) e nearest = (2000,110).
void test_ltft_hit_matches_ve_dominant_cell(void) {
    section("LEARN hit = célula dominante do VE (nearest, não floor)");
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    const uint32_t rpm_x10 = 20000u;   // 2000 rpm — nó exacto do eixo
    const uint16_t map_x100 = 110u;    // 110 kPa — nó exacto

    const uint8_t ri_near = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_near = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, map_x100);
    const uint8_t ri_floor = table_axis_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi_floor = table_axis_index(kLoadAxisBarX100, kTableAxisSize, map_x100);

    CHECK_EQ(ri_near, 6u, "nearest rpm 2000 → idx 6");
    CHECK_EQ(mi_near, 12u, "nearest map 110 → idx 12");
    CHECK_EQ(ri_floor, 5u, "floor rpm 2000 → idx 5 (pré-fix bug)");
    CHECK_EQ(mi_floor, 11u, "floor map 110 → idx 11 (pré-fix bug)");
    CHECK_TRUE(ri_near != ri_floor && mi_near != mi_floor,
               "caso de teste: floor ≠ nearest nos dois eixos");

    // 1 prev + N hits em regime estável
    fuel_update_stft(rpm_x10, map_x100, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 8; ++i) {
        fuel_update_stft(rpm_x10, map_x100, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }

    CHECK_EQ(fuel_ltft_accum_hits(mi_near, ri_near), 8u,
             "hits na célula dominante (2000×110) — igual ao trace VE");
    CHECK_EQ(fuel_ltft_accum_hits(mi_floor, ri_floor), 0u,
             "zero hits na célula floor (1750×100) — era o bug");
    // Também não deve espalhar para os outros cantos do rectângulo bilineal
    CHECK_EQ(fuel_ltft_accum_hits(mi_near, ri_floor), 0u, "sem hit no canto misto A");
    CHECK_EQ(fuel_ltft_accum_hits(mi_floor, ri_near), 0u, "sem hit no canto misto B");

    fuel_ltft_accum_reset();
    fuel_reset_adaptives();
}

void test_ltft_accum_page12(void) {
    section("página 12: LTFT accum export (hits u8 + mean_stft i8)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    fuel_reset_adaptives();
    fuel_ltft_accum_reset();

    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, 30000u);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, 100u);

    // 1 prev + 5 hits com err residual
    fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    for (int i = 0; i < 5; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1010, 900, true, false, false, 5000u, 500u);
    }
    CHECK_EQ(fuel_ltft_accum_hits(mi, ri), 5u, "5 hits na célula");

    uint8_t buf[kLtftAccumPageSize] = {};
    fuel_ltft_accum_export(buf, kLtftAccumPageSize);
    const uint16_t idx = static_cast<uint16_t>(mi) * kTableAxisSize + ri;
    CHECK_EQ(buf[idx] & 0x7Fu, 5u, "export hits[map][rpm] = 5");
    CHECK_EQ(buf[idx] & 0x80u, 0u, "export ready bit clear com 5 hits");
    // mean STFT i8 no 2º half
    const int8_t mean_wire = static_cast<int8_t>(buf[kTableCells + idx]);
    CHECK_EQ(static_cast<int>(mean_wire),
             static_cast<int>(fuel_ltft_accum_mean_stft_x10(mi, ri)),
             "export mean_stft confere");

    // Leitura via protocolo page 12 (800 B)
    const uint8_t rd[6] = {'r', 0x0Cu, 0x00u, 0x00u, 0x20u, 0x03u};  // 800
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u, "'r' page12 800B → OK");
    CHECK_EQ(r.len, 800u, "page12 len 800");
    CHECK_EQ(r.data[idx], 5u, "page12 wire hits = 5 (ready bit clear)");

    fuel_ltft_accum_reset();
    fuel_reset_adaptives();
}

void test_ltft_page_offsets_20(void) {
    section("página 10: offsets do layout 20×20 (mult 400 + add 10×10)");
    ckp_test_reset(); g_ckp_cap = 0u;
    ems::app::ui_test_reset();
    ems::hal::nvm_test_reset();

    // grava células de canto e lê a página inteira
    CHECK_TRUE(ems::hal::nvm_write_ltft(19u, 19u, 21), "ltft[19][19]=21");
    CHECK_TRUE(ems::hal::nvm_write_ltft_add(9u, 9u, -7), "add[9][9]=-7");
    const uint8_t rd[6] = {'r', 0x0Au, 0x00u, 0x00u, 0xF4u, 0x01u};  // 500
    EnvResp r = env_txn(rd, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 500u, "'r' page10 → 500B");
    // mult[m=19][r=19] no byte 19*20+19 = 399
    CHECK_EQ(static_cast<int8_t>(r.data[399]), (int8_t)21, "mult[19][19] no byte 399");
    // add[m=9][r=9] no byte 400 + 9*10+9 = 499
    CHECK_EQ(static_cast<int8_t>(r.data[499]), (int8_t)-7, "add[9][9] no byte 499");
}

// Formas de wire que o TunerStudio REAL emite quando pageIdentifier usa o
// prefixo \$tsCanId (obrigatório em msEnvelope_1.0, confirmado contra os
// projetos Speeduino 202501.6 e rusEFI): toda página fica com identificador
// de 2 bytes (canId+page), logo 'w'/'b' chegam sempre com 1 byte a mais do
// que a forma "sem canId" testada em test_ts_envelope_read_write_burn.
void test_ts_envelope_canid_forms(void) {
    section("envelope TS: formas com \\$tsCanId (write/burn/och)");
    ckp_test_reset(); g_ckp_cap = 0u;  // RPM=0 → burn permitido
    ems::app::ui_test_reset();

    // och lê página 3 → update_realtime_page() → get_ve(rpm, map_bar_x100), que
    // faz assert em map válido. map_bar_x1000 só é populado depois de pelo menos
    // uma sample_fast_channels() (5× sensors_on_tooth(), kFastSamplesPerRev=12 /
    // kRealTeethPerRev=58) seguida do commit staging→committed.
    sensor_setup(); sensors_init();
    {
        ems::drv::CkpSnapshot snap{};
        snap.tooth_period_ns = 160000u;
        snap.rpm_x10 = 62500u;
        for (int i = 0; i < 5; ++i) { sensors_on_tooth(snap); }
        sensors_test_tick_100ms();
    }

    // write canId-prefixed: 'w' canId(0) page(1) off(2) len(2) data(4)
    const uint32_t prog_before = ems::hal::nvm_test_program_count();
    const uint8_t wr[11] = {'w', 0x00u, 0x01u, 0x00u, 0x00u, 0x04u, 0x00u, 55u, 66u, 77u, 88u};
    EnvResp r = env_txn(wr, 11u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'w' canId+page+off+len → OK");
    CHECK_EQ(ve_table[0][0], 55u, "VE[0][0]=55 aplicado (forma canId)");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before,
             "'w' canId não grava flash (RAM-only)");

    // burn canId-prefixed: 'b' canId(0) page(1) — 3 bytes total
    const uint8_t burn[3] = {'b', 0x00u, 0x01u};
    r = env_txn(burn, 3u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u, "'b' canId+page → OK");
    CHECK_EQ(ems::hal::nvm_test_program_count(), prog_before + 1u,
             "burn (forma canId) gravou 1 página");

    // ochGetCommand real: 'r' canId(0) page(3) off(0) count(66) — 7 bytes
    const uint8_t och[7] = {'r', 0x00u, 0x03u, 0x00u, 0x00u, 0x42u, 0x00u};
    r = env_txn(och, 7u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u && r.len == 66u,
               "och via 'r' canId+page3+off0+count66 → 66 bytes");
}

// OCH page3: status bits 13/14 + tcReduction/spark @ reserved[31..]
void test_och_launch_tc_status(void) {
    section("OCH: launch/TC status bits + reduction scalars");
    ems::app::ui_test_reset();
    ems::engine::torque_manager_reset();
    etb_cal_valid = 1u;
    sensor_setup();
    sensors_init();
    {
        ems::drv::CkpSnapshot snap{};
        snap.tooth_period_ns = 160000u;
        snap.rpm_x10 = 62500u;
        for (int i = 0; i < 5; ++i) { sensors_on_tooth(snap); }
        sensors_test_tick_100ms();
    }

    // Idle latches → bits clear
    {
        const uint8_t och[7] = {'r', 0x00u, 0x03u, 0x00u, 0x00u, 0x56u, 0x00u};  // 86B
        EnvResp r = env_txn(och, 7u);
        CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u && r.len == 86u,
                   "och 86B OK");
        uint16_t st = 0u;
        std::memcpy(&st, r.data + 12, 2u);
        CHECK_TRUE((st & ems::app::STATUS_LAUNCH_ACTIVE) == 0u, "no launch bit idle");
        CHECK_TRUE((st & ems::app::STATUS_TC_ACTIVE) == 0u, "no TC bit idle");
        uint16_t tcr = 0u;
        std::memcpy(&tcr, r.data + 45, 2u);
        CHECK_EQ(tcr, 0u, "tcReduction=0 idle");
        CHECK_EQ(r.data[47], 0u, "spark retard=0 idle");
    }

    // Arm launch → STATUS_LAUNCH_ACTIVE
    ems::engine::torque_launch_force_enable(1u);
    ems::engine::launch_rpm_x10 = 45000u;
    ems::engine::launch_etb_pct_x10 = 600u;
    ems::engine::launch_app_arm_x10 = 200u;
    ems::engine::launch_app_disarm_x10 = 50u;
    {
        ems::drv::CkpSnapshot snap{};
        snap.rpm_x10 = 50000u;
        ems::drv::SensorData sens{};
        sens.app_pct_x10 = 800u;
        (void)ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        CHECK_EQ(ems::engine::torque_manager_test_get_launch_active(), 1u, "launch latched");
        const uint8_t och[7] = {'r', 0x00u, 0x03u, 0x00u, 0x00u, 0x56u, 0x00u};
        EnvResp r = env_txn(och, 7u);
        uint16_t st = 0u;
        std::memcpy(&st, r.data + 12, 2u);
        CHECK_TRUE((st & ems::app::STATUS_LAUNCH_ACTIVE) != 0u, "STATUS_LAUNCH_ACTIVE set");
    }
    ems::engine::torque_launch_force_enable(0u);
    ems::engine::torque_manager_reset();

    // External slip TC → STATUS_TC_ACTIVE + reduction scalar
    ems::engine::tc_enable = 1u;
    ems::engine::tc_max_reduction_pct_x10 = 800u;
    ems::engine::tc_spark_retard_max_deg = 12u;
    ems::engine::tc_reduction_rate_x10 = 10000u;
    {
        ems::drv::CkpSnapshot snap{};
        snap.rpm_x10 = 30000u;
        ems::drv::SensorData sens{};
        sens.app_pct_x10 = 1000u;
        (void)ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        ems::engine::torque_tc_set_external_slip_pct_x10(500u);
        for (int i = 0; i < 20; ++i) {
            (void)ems::engine::torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        }
        CHECK_TRUE(ems::engine::torque_manager_test_get_tc_reduction() > 0u, "TC latched >0");
        const uint8_t och[7] = {'r', 0x00u, 0x03u, 0x00u, 0x00u, 0x56u, 0x00u};
        EnvResp r = env_txn(och, 7u);
        uint16_t st = 0u;
        std::memcpy(&st, r.data + 12, 2u);
        CHECK_TRUE((st & ems::app::STATUS_TC_ACTIVE) != 0u, "STATUS_TC_ACTIVE set");
        uint16_t tcr = 0u;
        std::memcpy(&tcr, r.data + 45, 2u);
        CHECK_EQ(tcr, ems::engine::torque_manager_test_get_tc_reduction(),
                 "och tcReduction matches latch");
        CHECK_TRUE(r.data[47] > 0u, "och torqueSparkRetard > 0 under TC");
    }
    ems::engine::torque_tc_clear_external_slip();
    ems::engine::tc_enable = 0u;
    ems::engine::torque_manager_reset();
}

void test_ts_envelope_signature_via_r(void) {
    section("envelope TS: 'r' page 0x0F → assinatura (convenção Comm Manager)");
    ems::app::ui_test_reset();

    // 'r' canId(0) page(0x0F) off(0) count(0) — o Comm Manager real do
    // TunerStudio usa esta pseudo-página para validar o controlador após a
    // conexão, distinta do probe leve 'Q' cru da fase de deteção/wizard
    // (confirmado no comms.cpp real do Speeduino: "cmd == 0x0f → Request
    // for signature"). off/count no pedido são ignorados para esta página.
    const uint8_t req[7] = {'r', 0x00u, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u};
    EnvResp r = env_txn(req, 7u);
    CHECK_TRUE(r.frame_ok && r.crc_ok && r.code == 0x00u, "'r' page 0x0F → OK");
    CHECK_TRUE(r.len == 12u && memcmp(r.data, "OpenEMS_v1.3", 12u) == 0,
               "payload = assinatura OpenEMS_v1.3");

    // forma sem canId (6 bytes) também deve funcionar
    const uint8_t req6[6] = {'r', 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u};
    r = env_txn(req6, 6u);
    CHECK_TRUE(r.frame_ok && r.code == 0x00u && r.len == 12u,
               "'r' page 0x0F sem canId → também OK");
}
