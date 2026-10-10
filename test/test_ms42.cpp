// Estratégias portadas da Siemens MS42: bloco de calibração na page0 e o
// comportamento de cada estratégia (VVT com fase medida, ...).
#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "app/nvm_boot.h"
#include "drv/ckp.h"
#include "engine/calibration.h"
#include "engine/diagnostic_manager.h"
#include "engine/auxiliaries.h"
#include "engine/ms42_cal.h"
#include "hal/flash.h"
#include "test/fixtures.h"

using namespace ems::engine;

namespace {

// ~1560 rpm: 40 000 ticks de TIM5 (62,5 MHz) por dente.
constexpr uint32_t kVvtPeriod = 40000u;

// Uma volta completa a partir do gap; com cam=true a borda do came cai a meio
// do dente cam_tooth (ângulo = cam_tooth × 6° + 3°).
void feed_rev(bool cam, uint8_t cam_tooth) {
    for (uint32_t t = 1u; t <= kWheelNormalTeeth; ++t) {
        ckp_fire(kVvtPeriod);
        if (cam && t == cam_tooth) {
            cam_fire(g_ckp_cap + kVvtPeriod / 2u);
        }
    }
    ckp_fire(kVvtPeriod * 3u);  // gap
}

// n ticks de 10 ms; um ciclo de motor (2 voltas, uma borda de came) por tick.
void run_engine(uint32_t ticks, bool cam, uint8_t cam_tooth) {
    for (uint32_t i = 0u; i < ticks; ++i) {
        feed_rev(cam, cam_tooth);
        feed_rev(false, 0u);
        auxiliaries_tick_10ms();
    }
}

void vvt_flat_target(uint8_t deg) {
    std::memset(ms42.vvt_target_deg, deg, sizeof(ms42.vvt_target_deg));
}

}  // namespace

void test_ms42_page0_block(void) {
    section("ms42: bloco page0 (magic, roundtrip, defaults)");
    ms42_cal_defaults();
    CHECK_TRUE(static_cast<uint32_t>(kMs42Page0Off) + ms42_cal_page0_len() <= 512u, "bloco cabe na page0");

    uint8_t page[512] = {};
    ms42_apply_page0(page, sizeof(page));
    CHECK_EQ(ms42.fan_on_x10, 950, "sem magic -> defaults (ventoinha 95 C)");

    ms42.vvt_enable = 1u;
    ms42.vvt_target_deg[2][3] = 33u;
    ms42.dfco_entry_delay_ms = 250u;
    ms42.fan_on_x10 = 980;
    ms42_serialize_to_page0(page, sizeof(page));
    ms42_cal_defaults();
    ms42_apply_page0(page, sizeof(page));
    CHECK_EQ(ms42.vvt_enable, 1u, "roundtrip vvt_enable");
    CHECK_EQ(ms42.vvt_target_deg[2][3], 33u, "roundtrip alvo VVT");
    CHECK_EQ(ms42.dfco_entry_delay_ms, 250u, "roundtrip atraso DFCO");
    CHECK_EQ(ms42.fan_on_x10, 980, "roundtrip ventoinha");

    // Eixo não crescente é rejeitado (volta ao default), não fica corrompido.
    ms42.vvt_rpm_axis[3] = 0u;
    ms42.fan_off_x10 = 1200;  // off >= on
    ms42_serialize_to_page0(page, sizeof(page));
    ms42_apply_page0(page, sizeof(page));
    CHECK_EQ(ms42.vvt_rpm_axis[3], 40u, "eixo VVT invalido -> default");
    CHECK_EQ(ms42.fan_off_x10, 900, "histerese da ventoinha invertida -> default");

    // Versão do bloco diferente → defaults (layout pode ter mudado).
    page[kMs42Page0Off + 2u] = static_cast<uint8_t>(kMs42BlockVer + 1u);
    ms42_apply_page0(page, sizeof(page));
    CHECK_EQ(ms42.vvt_enable, 0u, "versao do bloco diferente -> defaults");
    ms42_cal_defaults();
}

void test_ms42_vvt_measured_phase(void) {
    section("ms42 A1: VVT de admissao com fase medida do came");
    ms42_cal_defaults();
    ms42.vvt_min_clt_x10 = -400;  // CLT do host fica fora da questão
    auxiliaries_test_reset();
    ckp_reach_full_sync(kVvtPeriod);

    // Came na posição de repouso (dente 10 + 3° = 63,0°). VVT desligado:
    // solenoide sem corrente e a referência é aprendida.
    run_engine(150u, true, 10u);
    CHECK_EQ(auxiliaries_test_get_vvt_adm_duty(), 0u, "VVT desligado -> duty 0");
    CHECK_NEAR(auxiliaries_vvt_ref_x10(), 630, 5, "referencia aprendida = borda em repouso");
    CHECK_NEAR(auxiliaries_vvt_advance_x10(), 0, 5, "avanco medido ~0 em repouso");

    // Ligado, alvo 20°: came ainda em repouso → erro positivo → duty acima
    // da retenção, a crescer com o integrador.
    ms42.vvt_enable = 1u;
    vvt_flat_target(20u);
    run_engine(30u, true, 10u);
    const uint16_t d1 = auxiliaries_test_get_vvt_adm_duty();
    CHECK_TRUE(d1 > 500u, "abaixo do alvo -> duty acima da retencao (50 %)");
    CHECK_EQ(auxiliaries_vvt_target_x10(), 200, "alvo 20,0 apos o slew");
    CHECK_EQ(auxiliaries_test_get_vvt_esc_duty(), 0u, "escape sempre desligado");

    // Came avança 30° (borda 5 dentes mais cedo): passa do alvo → duty desce.
    run_engine(30u, true, 5u);
    CHECK_NEAR(auxiliaries_vvt_advance_x10(), 300, 15, "avanco medido ~30 graus");
    CHECK_TRUE(auxiliaries_test_get_vvt_adm_duty() < d1, "acima do alvo -> duty desce");

    // A referência não é reaprendida enquanto o VVT atua.
    CHECK_NEAR(auxiliaries_vvt_ref_x10(), 630, 5, "referencia congelada com o VVT ativo");

    // Sem bordas de came: repouso em <= 200 ms.
    run_engine(25u, false, 0u);
    CHECK_EQ(auxiliaries_test_get_vvt_adm_duty(), 0u, "sem came -> duty 0 (repouso)");

    // Óleo frio: repouso.
    ms42.vvt_min_clt_x10 = 2000;
    run_engine(10u, true, 5u);
    CHECK_EQ(auxiliaries_test_get_vvt_adm_duty(), 0u, "CLT abaixo do minimo -> duty 0");

    // Referência calibrada tem prioridade sobre a aprendida.
    ms42.vvt_min_clt_x10 = -400;
    ms42.vvt_cam_ref_x10 = 900;
    run_engine(10u, true, 10u);
    CHECK_NEAR(auxiliaries_vvt_advance_x10(), 270, 15, "referencia calibrada 90,0 -> avanco 27");
    ms42_cal_defaults();
    auxiliaries_test_reset();
}

void test_ms42_cal_crc(void) {
    section("ms42 A2: CRC-32 por pagina de calibracao");
    using namespace ems::hal;
    namespace dm = ems::engine;
    uint8_t backup[sizeof(boost_target_bar_x1000)];
    std::memcpy(backup, boost_target_bar_x1000, sizeof(backup));

    uint8_t page[sizeof(boost_target_bar_x1000)];
    std::memcpy(page, backup, sizeof(page));
    page[0] = static_cast<uint8_t>(page[0] ^ 0x5Au);  // valor distinto do default

    nvm_test_reset();
    dm::DiagnosticManager::clear_all_faults();
    CHECK_TRUE(nvm_save_calibration(8u, page, sizeof(page)), "grava page 8 com trailer");
    ems::app::load_boost_map_from_nvm();
    CHECK_TRUE(nvm_calibration_status(8u) == NvmCalStatus::OK, "CRC confere -> OK");
    CHECK_EQ(reinterpret_cast<const uint8_t*>(boost_target_bar_x1000)[0], page[0],
             "pagina valida e aplicada");

    // Um bit trocado na flash: a pagina nao e aplicada (fica o default).
    std::memcpy(boost_target_bar_x1000, backup, sizeof(backup));
    uint32_t img_len = 0u;
    uint8_t* img = nvm_host_calibration_image(&img_len);
    const uint32_t slot = img_len / 10u;
    img[8u * slot + 5u] = static_cast<uint8_t>(img[8u * slot + 5u] ^ 0x01u);
    uint8_t probe[sizeof(page)] = {};
    CHECK_FALSE(nvm_load_calibration(8u, probe, sizeof(probe)), "CRC invalido -> load falha");
    CHECK_EQ(probe[0], 0xFFu, "CRC invalido -> buffer apagado (defaults)");
    CHECK_TRUE(nvm_calibration_status(8u) == NvmCalStatus::BAD_CRC, "status BAD_CRC");
    CHECK_EQ(nvm_calibration_bad_crc_mask(), 1u << 8, "mascara aponta a page 8");
    ems::app::nvm_boot_load_tables(false);
    CHECK_EQ(std::memcmp(boost_target_bar_x1000, backup, sizeof(backup)), 0,
             "pagina corrompida nao aplicada");
    CHECK_TRUE(dm::DiagnosticManager::is_fault_active(dm::DiagnosticCode::CAL_CRC_FAULT),
               "DTC CAL_CRC_FAULT registado");

    // Blob gravado por firmware anterior (sem trailer): aceito como está.
    nvm_test_reset();
    std::memcpy(&img[8u * slot], page, sizeof(page));
    ems::app::load_boost_map_from_nvm();
    CHECK_TRUE(nvm_calibration_status(8u) == NvmCalStatus::LEGACY_NO_CRC, "sem trailer -> legado");
    CHECK_EQ(reinterpret_cast<const uint8_t*>(boost_target_bar_x1000)[0], page[0],
             "blob legado continua a carregar");

    std::memcpy(boost_target_bar_x1000, backup, sizeof(backup));
    dm::DiagnosticManager::clear_all_faults();
    nvm_test_reset();
}
