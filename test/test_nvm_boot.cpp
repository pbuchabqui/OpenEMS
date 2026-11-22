#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "app/nvm_boot.h"
#include "engine/calibration.h"
#include "engine/etb_control.h"
#include "engine/table3d.h"
#include "hal/flash.h"

using namespace ems::app;
using namespace ems::engine;
using namespace ems::hal;

// O stub de host zera as páginas em nvm_test_reset(); flash apagada real lê 0xFF.
static void nvm_erase_all_pages() {
    nvm_test_reset();
    static uint8_t ff[1024];
    std::memset(ff, 0xFF, sizeof(ff));
    for (uint8_t pg = 0u; pg <= 9u; ++pg) { (void)nvm_save_calibration(pg, ff, sizeof(ff)); }
}

void test_nvm_boot_page_helpers(void) {
    section("nvm_boot: helpers de página apagada/zerada");

    uint8_t a[16];
    std::memset(a, 0xFF, sizeof(a));
    CHECK_TRUE(page_is_erased(a, sizeof(a)), "tudo 0xFF: apagada");
    CHECK_TRUE(page_range_is_erased(a, 4u, 8u), "faixa 0xFF: apagada");
    a[9] = 0x00u;
    CHECK_FALSE(page_is_erased(a, sizeof(a)), "um byte gravado: não apagada");
    CHECK_FALSE(page_range_is_erased(a, 4u, 8u), "byte dentro da faixa: não apagada");
    CHECK_TRUE(page_range_is_erased(a, 10u, 6u), "faixa que não cobre o byte: apagada");
    CHECK_TRUE(page_range_is_erased(a, 0u, 0u), "faixa vazia: apagada (vacuamente)");

    std::memset(a, 0x00, sizeof(a));
    CHECK_TRUE(page_range_is_zero(a, 0u, sizeof(a)), "tudo 0x00: zero");
    a[3] = 1u;
    CHECK_FALSE(page_range_is_zero(a, 0u, sizeof(a)), "byte != 0: não zero");
    CHECK_TRUE(page_range_is_zero(a, 4u, 12u), "faixa sem o byte: zero");
    CHECK_FALSE(page_is_erased(a, sizeof(a)), "página zerada não é apagada");

    // Em branco = apagada OU toda zero (o boot não aplica a página 0 em nenhum dos dois casos).
    std::memset(a, 0xFF, sizeof(a));
    CHECK_TRUE(page_is_blank(a, sizeof(a)), "tudo 0xFF: em branco");
    std::memset(a, 0x00, sizeof(a));
    CHECK_TRUE(page_is_blank(a, sizeof(a)), "tudo 0x00: em branco");
    a[7] = 0x45u;
    CHECK_FALSE(page_is_blank(a, sizeof(a)), "um byte gravado: não está em branco");
    std::memset(a, 0xFF, sizeof(a));
    a[7] = 0x00u;
    CHECK_FALSE(page_is_blank(a, sizeof(a)), "mistura de 0xFF e 0x00 não é em branco");
}

void test_nvm_boot_table_loaders(void) {
    section("nvm_boot: loaders de tabela (VE/spark/lambda/boost/pedal/eixos)");

    // Backups para restaurar no fim.
    static uint8_t  ve_bk[kTableAxisSize][kTableAxisSize];
    static int8_t   sp_bk[kTableAxisSize][kTableAxisSize];
    static int16_t  lm_bk[kTableAxisSize][kTableAxisSize];
    static uint16_t bo_bk[7][8];
    static uint16_t pe_bk[4][10];
    std::memcpy(ve_bk, ve_table, sizeof(ve_bk));
    std::memcpy(sp_bk, spark_table, sizeof(sp_bk));
    std::memcpy(lm_bk, lambda_target_table_x1000, sizeof(lm_bk));
    std::memcpy(bo_bk, boost_target_bar_x1000, sizeof(bo_bk));
    std::memcpy(pe_bk, etb_pedal_map, sizeof(pe_bk));
    uint16_t rpm_bk[kTableAxisSize], load_bk[kTableAxisSize];
    table_axes_get(rpm_bk, load_bk);

    // Flash apagada → mantém defaults de compilação.
    nvm_erase_all_pages();
    ve_table[3][4] = 77u;
    load_ve_table_from_nvm();
    CHECK_EQ(ve_table[3][4], 77u, "VE: flash apagada preserva a RAM");

    // VE: gravada → carrega.
    alignas(4) uint8_t ve_pg[sizeof(ve_table)];
    for (uint16_t i = 0u; i < sizeof(ve_pg); ++i) { ve_pg[i] = static_cast<uint8_t>(40u + (i % 50u)); }
    CHECK_TRUE(nvm_save_calibration(1u, ve_pg, sizeof(ve_pg)), "grava page VE");
    load_ve_table_from_nvm();
    CHECK_EQ(std::memcmp(ve_table, ve_pg, sizeof(ve_pg)), 0, "VE: carregada da NVM");

    // Spark.
    nvm_erase_all_pages();
    alignas(4) uint8_t sp_pg[sizeof(spark_table)];
    for (uint16_t i = 0u; i < sizeof(sp_pg); ++i) { sp_pg[i] = static_cast<uint8_t>(10 + (i % 20u)); }
    load_spark_table_from_nvm();
    CHECK_EQ(std::memcmp(spark_table, sp_bk, sizeof(sp_bk)), 0, "spark: sem página grava → intacta");
    CHECK_TRUE(nvm_save_calibration(2u, sp_pg, sizeof(sp_pg)), "grava page spark");
    load_spark_table_from_nvm();
    CHECK_EQ(std::memcmp(spark_table, sp_pg, sizeof(sp_pg)), 0, "spark: carregada da NVM");

    // Lambda alvo.
    alignas(4) uint8_t lm_pg[sizeof(lambda_target_table_x1000)];
    int16_t lv[kTableAxisSize * kTableAxisSize];
    for (uint16_t i = 0u; i < kTableAxisSize * kTableAxisSize; ++i) { lv[i] = static_cast<int16_t>(850 + (i % 100)); }
    std::memcpy(lm_pg, lv, sizeof(lm_pg));
    CHECK_TRUE(nvm_save_calibration(3u, lm_pg, sizeof(lm_pg)), "grava page lambda");
    load_lambda_target_table_from_nvm();
    CHECK_EQ(std::memcmp(lambda_target_table_x1000, lm_pg, sizeof(lm_pg)), 0, "lambda alvo: carregada");

    // Boost.
    alignas(4) uint8_t bo_pg[112];
    std::memset(bo_pg, 0, sizeof(bo_pg));
    for (uint16_t i = 0u; i < sizeof(boost_target_bar_x1000); ++i) { bo_pg[i] = static_cast<uint8_t>(i * 3u + 1u); }
    CHECK_TRUE(nvm_save_calibration(8u, bo_pg, sizeof(bo_pg)), "grava page boost");
    load_boost_map_from_nvm();
    CHECK_EQ(std::memcmp(boost_target_bar_x1000, bo_pg, sizeof(boost_target_bar_x1000)), 0, "boost: carregado");

    // Mapa de pedal.
    alignas(4) uint8_t pe_pg[80];
    for (uint16_t i = 0u; i < sizeof(pe_pg); ++i) { pe_pg[i] = static_cast<uint8_t>(i + 5u); }
    CHECK_TRUE(nvm_save_calibration(7u, pe_pg, sizeof(pe_pg)), "grava page pedal");
    load_pedal_map_from_nvm();
    CHECK_EQ(std::memcmp(etb_pedal_map, pe_pg, sizeof(etb_pedal_map)), 0, "pedal: carregado");

    // Eixos válidos (estritamente crescentes) são aplicados; corrompidos, rejeitados.
    alignas(4) uint8_t ax_pg[4u * kTableAxisSize];
    uint16_t rpm_new[kTableAxisSize], load_new[kTableAxisSize];
    for (uint8_t i = 0u; i < kTableAxisSize; ++i) {
        rpm_new[i]  = static_cast<uint16_t>(5u + 40u * i);
        load_new[i] = static_cast<uint16_t>(10u + 12u * i);
    }
    std::memcpy(ax_pg, rpm_new, 2u * kTableAxisSize);
    std::memcpy(ax_pg + 2u * kTableAxisSize, load_new, 2u * kTableAxisSize);
    CHECK_TRUE(nvm_save_calibration(9u, ax_pg, sizeof(ax_pg)), "grava page eixos válidos");
    load_table_axes_from_nvm();
    uint16_t rpm_now[kTableAxisSize], load_now[kTableAxisSize];
    table_axes_get(rpm_now, load_now);
    CHECK_EQ(std::memcmp(rpm_now, rpm_new, sizeof(rpm_now)), 0, "eixo RPM válido aplicado");
    CHECK_EQ(std::memcmp(load_now, load_new, sizeof(load_now)), 0, "eixo carga válido aplicado");

    rpm_new[7] = rpm_new[6];                 // não-monotônico
    std::memcpy(ax_pg, rpm_new, 2u * kTableAxisSize);
    CHECK_TRUE(nvm_save_calibration(9u, ax_pg, sizeof(ax_pg)), "grava page eixos corrompidos");
    load_table_axes_from_nvm();
    table_axes_get(rpm_now, load_now);
    CHECK_TRUE(rpm_now[7] > rpm_now[6], "eixo corrompido rejeitado: segue estritamente crescente");

    // Gate cal_layout_ok=false: nem VE, nem spark, nem lambda, nem eixos.
    nvm_erase_all_pages();
    ve_table[0][0] = 1u;
    CHECK_TRUE(nvm_save_calibration(1u, ve_pg, sizeof(ve_pg)), "regrava VE");
    nvm_boot_load_tables(false);
    CHECK_EQ(ve_table[0][0], 1u, "layout inválido: VE não é carregada");
    nvm_boot_load_tables(true);
    CHECK_EQ(ve_table[0][0], ve_pg[0], "layout ok: VE é carregada");

    // Restaura.
    nvm_test_reset();
    std::memcpy(ve_table, ve_bk, sizeof(ve_bk));
    std::memcpy(spark_table, sp_bk, sizeof(sp_bk));
    std::memcpy(lambda_target_table_x1000, lm_bk, sizeof(lm_bk));
    std::memcpy(boost_target_bar_x1000, bo_bk, sizeof(bo_bk));
    std::memcpy(etb_pedal_map, pe_bk, sizeof(pe_bk));
    (void)table_axes_set(rpm_bk, load_bk);
}

void test_nvm_boot_rejects_zeroed_corrections(void) {
    section("nvm_boot: página 5 e página 7 gravadas com zeros (CRC válido) mantêm os padrões");

    // Padrões de compilação vigentes antes do carregamento.
    int16_t  clt_axis[kCorrectionTableSize];   uint16_t clt_val[kCorrectionTableSize];
    uint16_t dead_ax[kCorrectionTableSize];    uint16_t dead_val[kCorrectionTableSize];
    uint16_t dw_ax[kCorrectionTableSize];      uint16_t dw_val[kCorrectionTableSize];
    uint16_t drpm_ax[kDwellRpmCorrSize];       uint16_t drpm_fac[kDwellRpmCorrSize];
    std::memcpy(clt_axis, clt_corr_axis_x10, sizeof(clt_axis));
    std::memcpy(clt_val, clt_corr_x256, sizeof(clt_val));
    std::memcpy(dead_ax, vbatt_corr_axis_mv, sizeof(dead_ax));
    std::memcpy(dead_val, injector_dead_time_us, sizeof(dead_val));
    std::memcpy(dw_ax, dwell_vbatt_axis_mv, sizeof(dw_ax));
    std::memcpy(dw_val, dwell_ms_x10_table, sizeof(dw_val));
    std::memcpy(drpm_ax, dwell_rpm_axis_rpm, sizeof(drpm_ax));
    std::memcpy(drpm_fac, dwell_rpm_factor_q8, sizeof(drpm_fac));
    CHECK_TRUE(clt_val[0] != 0u && dead_val[0] != 0u && dw_val[0] != 0u && drpm_fac[0] != 0u,
               "pré-condição: os padrões de compilação não são zero");

    nvm_erase_all_pages();
    uint8_t corr[256] = {};                 // tabelas zeradas, como o base_tune.json antigo gravava
    uint8_t dw2d[32] = {};
    CHECK_TRUE(nvm_save_calibration(4u, corr, sizeof(corr)), "grava a página 5 toda zero");
    CHECK_TRUE(nvm_save_calibration(6u, dw2d, sizeof(dw2d)), "grava a página 7 toda zero");
    load_corr_calibration_from_nvm();
    load_dwell2d_calibration_from_nvm();
    CHECK_TRUE(std::memcmp(clt_corr_axis_x10, clt_axis, sizeof(clt_axis)) == 0, "eixo de CLT mantido");
    CHECK_TRUE(std::memcmp(clt_corr_x256, clt_val, sizeof(clt_val)) == 0, "correção de CLT mantida (≠ 0)");
    CHECK_TRUE(std::memcmp(vbatt_corr_axis_mv, dead_ax, sizeof(dead_ax)) == 0, "eixo de VBATT mantido");
    CHECK_TRUE(std::memcmp(injector_dead_time_us, dead_val, sizeof(dead_val)) == 0, "dead time mantido (≠ 0)");
    CHECK_TRUE(std::memcmp(dwell_vbatt_axis_mv, dw_ax, sizeof(dw_ax)) == 0, "eixo do dwell mantido");
    CHECK_TRUE(std::memcmp(dwell_ms_x10_table, dw_val, sizeof(dw_val)) == 0, "dwell mantido (≠ 0)");
    CHECK_TRUE(std::memcmp(dwell_rpm_axis_rpm, drpm_ax, sizeof(drpm_ax)) == 0, "eixo RPM do dwell mantido");
    CHECK_TRUE(std::memcmp(dwell_rpm_factor_q8, drpm_fac, sizeof(drpm_fac)) == 0, "fator de dwell mantido (≠ 0)");

    // Tabela válida gravada pelo usuário continua carregando (a checagem não bloqueia calibração real).
    int16_t axis_ok[kCorrectionTableSize] = {-300, -50, 50, 250, 450, 750, 950, 1150};
    uint16_t val_ok[kCorrectionTableSize] = {400u, 360u, 330u, 300u, 280u, 260u, 256u, 256u};
    std::memcpy(corr + 0, axis_ok, 16u);
    std::memcpy(corr + 16, val_ok, 16u);
    CHECK_TRUE(nvm_save_calibration(4u, corr, sizeof(corr)), "grava CLT válida, o resto zero");
    load_corr_calibration_from_nvm();
    CHECK_EQ(clt_corr_x256[0], 400u, "CLT válida é carregada");
    CHECK_EQ(clt_corr_axis_x10[7], 1150, "eixo de CLT válido é carregado");
    CHECK_TRUE(std::memcmp(injector_dead_time_us, dead_val, sizeof(dead_val)) == 0,
               "bloco de dead time zerado segue ignorado");

    // Restaura.
    nvm_test_reset();
    std::memcpy(clt_corr_axis_x10, clt_axis, sizeof(clt_axis));
    std::memcpy(clt_corr_x256, clt_val, sizeof(clt_val));
}

void test_nvm_boot_rejects_zeroed_ve_spark_lambda(void) {
    section("nvm_boot: VE, avanço e lambda alvo gravados com zeros mantêm os padrões");
    static uint8_t ve_bk[sizeof(ve_table)];
    static uint8_t sp_bk[sizeof(spark_table)];
    static uint8_t lm_bk[sizeof(lambda_target_table_x1000)];
    std::memcpy(ve_bk, ve_table, sizeof(ve_bk));
    std::memcpy(sp_bk, spark_table, sizeof(sp_bk));
    std::memcpy(lm_bk, lambda_target_table_x1000, sizeof(lm_bk));

    nvm_erase_all_pages();
    static uint8_t zeros[sizeof(lambda_target_table_x1000)] = {};
    CHECK_TRUE(nvm_save_calibration(1u, zeros, sizeof(ve_table)), "grava VE toda zero");
    CHECK_TRUE(nvm_save_calibration(2u, zeros, sizeof(spark_table)), "grava avanço todo zero");
    CHECK_TRUE(nvm_save_calibration(3u, zeros, sizeof(lambda_target_table_x1000)), "grava lambda toda zero");
    load_ve_table_from_nvm();
    load_spark_table_from_nvm();
    load_lambda_target_table_from_nvm();
    CHECK_TRUE(std::memcmp(ve_table, ve_bk, sizeof(ve_bk)) == 0, "VE padrão mantida (zero daria PW 0)");
    CHECK_TRUE(std::memcmp(spark_table, sp_bk, sizeof(sp_bk)) == 0, "avanço padrão mantido");
    CHECK_TRUE(std::memcmp(lambda_target_table_x1000, lm_bk, sizeof(lm_bk)) == 0, "lambda padrão mantida");

    // Uma VE válida do usuário continua carregando.
    static uint8_t custom[sizeof(ve_table)];
    std::memset(custom, 77, sizeof(custom));
    CHECK_TRUE(nvm_save_calibration(1u, custom, sizeof(custom)), "grava VE válida");
    load_ve_table_from_nvm();
    CHECK_TRUE(std::memcmp(ve_table, custom, sizeof(custom)) == 0, "VE válida é carregada");

    nvm_test_reset();
    std::memcpy(ve_table, ve_bk, sizeof(ve_bk));
    std::memcpy(spark_table, sp_bk, sizeof(sp_bk));
    std::memcpy(lambda_target_table_x1000, lm_bk, sizeof(lm_bk));
}

void test_nvm_boot_rejects_zeroed_pedal_and_boost(void) {
    section("nvm_boot: mapa do pedal e alvo de boost gravados com zeros mantêm os padrões");
    uint16_t pedal_bk[4][10];
    uint16_t boost_bk[7][8];
    std::memcpy(pedal_bk, etb_pedal_map, sizeof(pedal_bk));
    std::memcpy(boost_bk, boost_target_bar_x1000, sizeof(boost_bk));
    CHECK_TRUE(pedal_bk[1][9] != 0u, "pré-condição: o mapa do pedal padrão não é zero");

    nvm_erase_all_pages();
    uint8_t zeros[112] = {};
    CHECK_TRUE(nvm_save_calibration(7u, zeros, 80u), "grava o pedal todo zero");
    CHECK_TRUE(nvm_save_calibration(8u, zeros, 112u), "grava o boost todo zero");
    load_pedal_map_from_nvm();
    load_boost_map_from_nvm();
    CHECK_TRUE(std::memcmp(etb_pedal_map, pedal_bk, sizeof(pedal_bk)) == 0, "pedal padrão mantido");
    CHECK_TRUE(std::memcmp(boost_target_bar_x1000, boost_bk, sizeof(boost_bk)) == 0, "boost padrão mantido");

    // Um mapa válido do usuário continua carregando.
    uint16_t custom[4][10] = {};
    for (uint8_t r = 0u; r < 4u; ++r) { for (uint8_t c = 0u; c < 10u; ++c) { custom[r][c] = static_cast<uint16_t>(c * 100u); } }
    CHECK_TRUE(nvm_save_calibration(7u, reinterpret_cast<const uint8_t*>(custom), sizeof(custom)), "grava pedal válido");
    load_pedal_map_from_nvm();
    CHECK_EQ(etb_pedal_map[2][9], 900u, "pedal válido é carregado");

    nvm_test_reset();
    std::memcpy(etb_pedal_map, pedal_bk, sizeof(pedal_bk));
    std::memcpy(boost_target_bar_x1000, boost_bk, sizeof(boost_bk));
}
