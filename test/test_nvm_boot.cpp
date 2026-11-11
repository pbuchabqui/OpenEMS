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
