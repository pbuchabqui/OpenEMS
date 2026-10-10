// Estratégias portadas da Siemens MS42: bloco de calibração na page0 e o
// comportamento de cada estratégia (VVT com fase medida, ...).
#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "app/nvm_boot.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "hal/adc.h"
#include "engine/calibration.h"
#include "engine/diagnostic_manager.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/limp_gating.h"
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

void test_ms42_vbatt_filter_dtc(void) {
    section("ms42 A3: VBATT filtrada + DTC de tensao");
    using namespace ems::hal;
    namespace dm = ems::engine;
    ms42_cal_defaults();
    dm::DiagnosticManager::clear_all_faults();
    sensor_setup();
    ems::drv::sensors_init();
    ems::drv::sensors_set_bench_clt_iat(false, 0, 0);

    // 12 V estável, depois um pico isolado de 16,5 V: o filtro só deixa
    // passar uma fração (α = 1/4).
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 2730u);
    ems::drv::sensors_test_tick_100ms();
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 3754u);
    ems::drv::sensors_test_tick_100ms();
    CHECK_NEAR(ems::drv::sensors_get().vbatt_mv, 13125, 40, "pico de 16,5 V atenuado a ~13,1 V");

    // Sem filtro (shift 0) o valor passa direto.
    ms42.vbatt_filter_shift = 0u;
    ems::drv::sensors_test_tick_100ms();
    CHECK_NEAR(ems::drv::sensors_get().vbatt_mv, 16500, 10, "shift 0 -> sem filtro");

    // 16,5 V por 2 s → VBATT_HIGH (mesmo com o motor parado).
    ems::drv::ckp_test_reset();
    auxiliaries_test_reset();
    for (int i = 0; i < 199; ++i) { auxiliaries_tick_10ms(); }
    CHECK_FALSE(dm::DiagnosticManager::is_fault_active(dm::DiagnosticCode::VBATT_HIGH),
                "VBATT_HIGH espera o debounce");
    auxiliaries_tick_10ms();
    CHECK_TRUE(dm::DiagnosticManager::is_fault_active(dm::DiagnosticCode::VBATT_HIGH),
               "VBATT_HIGH apos 2 s acima de 16 V");

    // 9,5 V com o motor parado (partida) não é falha; a trabalhar é.
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 2161u);
    ems::drv::sensors_test_tick_100ms();
    for (int i = 0; i < 300; ++i) { auxiliaries_tick_10ms(); }
    CHECK_FALSE(dm::DiagnosticManager::is_fault_active(dm::DiagnosticCode::VBATT_LOW),
                "9,5 V com o motor parado -> sem DTC");
    CHECK_FALSE(dm::DiagnosticManager::is_fault_active(dm::DiagnosticCode::VBATT_HIGH),
                "VBATT_HIGH limpa apos 2 s na faixa");
    ckp_reach_full_sync(kVvtPeriod);
    run_engine(210u, false, 0u);
    CHECK_TRUE(dm::DiagnosticManager::is_fault_active(dm::DiagnosticCode::VBATT_LOW),
               "9,5 V a ~1500 rpm por 2 s -> VBATT_LOW");

    ms42_cal_defaults();
    dm::DiagnosticManager::clear_all_faults();
    auxiliaries_test_reset();
    sensor_setup();
}

void test_ms42_dfco_curve_delay(void) {
    section("ms42 B4: DFCO por CLT + atraso de entrada");
    ms42_cal_defaults();
    const int16_t min_clt = decel_cut_min_clt_x10;
    decel_cut_min_clt_x10 = -400;
    // MS42: 1600 rpm a −30 °C … 1024 rpm a quente.
    const uint16_t entry[kDfcoCltPts] = {16000u, 14000u, 12000u, 10240u};
    std::memcpy(ms42.dfco_entry_rpm_x10, entry, sizeof(entry));
    ms42.dfco_hyst_rpm_x10 = 2000u;

    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_update(13000u, 0u, -300), "frio: 1300 < 1600 -> sem corte");
    CHECK_TRUE(fuel_decel_cut_update(13000u, 0u, 800), "quente: 1300 >= 1024 -> corta");
    CHECK_TRUE(fuel_decel_cut_update(8500u, 0u, 800), "saida = entrada - hist (824): 850 ainda corta");
    CHECK_FALSE(fuel_decel_cut_update(8000u, 0u, 800), "abaixo de 824 -> sai");
    fuel_decel_cut_reset();
    CHECK_TRUE(fuel_decel_cut_update(17000u, 0u, -300), "frio: 1700 >= 1600 -> corta");
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_update(12900u, 0u, 200), "20 C, meio de 0..40 C: entrada 1300");
    CHECK_TRUE(fuel_decel_cut_update(13000u, 0u, 200), "1300 >= 1300 -> corta");

    // Atraso de entrada: condições estáveis por 300 ms.
    ms42.dfco_entry_delay_ms = 300u;
    fuel_decel_cut_reset();
    fuel_decel_cut_notify_time(1000u);
    CHECK_FALSE(fuel_decel_cut_update(20000u, 0u, 800), "atraso: t=0 sem corte");
    fuel_decel_cut_notify_time(1200u);
    CHECK_FALSE(fuel_decel_cut_update(20000u, 0u, 800), "atraso: t=200 sem corte");
    fuel_decel_cut_notify_time(1250u);
    CHECK_FALSE(fuel_decel_cut_update(20000u, 50u, 800), "pedal mexeu -> reinicia");
    fuel_decel_cut_notify_time(1300u);
    CHECK_FALSE(fuel_decel_cut_update(20000u, 0u, 800), "recomeca a contar");
    fuel_decel_cut_notify_time(1600u);
    CHECK_TRUE(fuel_decel_cut_update(20000u, 0u, 800), "300 ms estaveis -> corta");

    // Curva toda a 0 = escalares antigos (1500/1200).
    ms42_cal_defaults();
    fuel_decel_cut_reset();
    CHECK_FALSE(fuel_decel_cut_update(14000u, 0u, 800), "sem curva: 1400 < 1500");
    CHECK_TRUE(fuel_decel_cut_update(15000u, 0u, 800), "sem curva: 1500 corta");
    decel_cut_min_clt_x10 = min_clt;
    fuel_decel_cut_reset();
}

void test_ms42_rev_roll_cut(void) {
    section("ms42 B5: corte rotativo de injecao abaixo do limite");
    ms42_cal_defaults();
    const uint32_t hard = rev_limit_rpm_x10;
    LimpGatingInputs in{};
    in.full_sync = true;
    in.phase_valid = true;
    in.sequential = true;
    in.lambda_valid = false;
    in.oil_press_bar_x1000 = 3000u;
    in.clt_degc_x10 = 850;
    in.now_ms = 10000u;
    in.rpm_x10 = 30000u;
    const uint16_t dis = 0x03FFu;  // só o limitador interessa aqui
    limp_gating_set_protect_disable(dis);
    LimpGatingResult r = limp_gating_update(in);
    const uint8_t base_mask = r.inj_inhibit_mask;

    // Desligado: nada na janela.
    in.rpm_x10 = hard - 500u;
    r = limp_gating_update(in);
    limp_gating_on_rev();
    r = limp_gating_update(in);
    CHECK_EQ(r.inj_inhibit_mask, base_mask, "rev_roll off -> sem corte na janela");

    // Ligado, 3/4 da janela (200 rpm): ~75 % de 2 injeções por volta.
    ms42.rev_roll_enable = 1u;
    in.rpm_x10 = hard - 500u;
    uint32_t cut = 0u;
    uint8_t seen = 0u;
    for (int rev = 0; rev < 40; ++rev) {
        limp_gating_update(in);
        limp_gating_on_rev();
        r = limp_gating_update(in);
        const uint8_t m = static_cast<uint8_t>(r.inj_inhibit_mask & ~base_mask);
        seen |= m;
        for (uint8_t b = 0u; b < 4u; ++b) { cut += (m >> b) & 1u; }
        CHECK_TRUE(m != 0x0Fu, "nunca corta os 4 de uma vez");
        if (m == 0x0Fu) { break; }
    }
    CHECK_NEAR(static_cast<int>(cut), 60, 2, "~75 % de 80 injecoes cortadas");
    CHECK_EQ(seen, 0x0Fu, "o corte roda pelos 4 cilindros");
    CHECK_TRUE(r.allow_injection, "corte parcial: injecao continua permitida");

    // Abaixo da janela: limpa na hora.
    in.rpm_x10 = hard - 3000u;
    r = limp_gating_update(in);
    CHECK_EQ(r.inj_inhibit_mask, base_mask, "abaixo da janela -> mascara limpa");

    // No limite duro: corte total, como antes.
    in.rpm_x10 = hard + 100u;
    r = limp_gating_update(in);
    CHECK_EQ(r.inj_inhibit_mask, 0x0Fu, "limite duro -> corte total");
    in.rpm_x10 = 10000u;
    limp_gating_update(in);
    limp_gating_set_protect_disable(0u);
    ms42_cal_defaults();
}

void test_ms42_spark_gradient(void) {
    section("ms42 B6: limitador de gradiente do avanco");
    ms42_cal_defaults();
    spark_gradient_reset();
    CHECK_EQ(spark_gradient_limit_x10(300, true, false), 300, "desligado: passa direto");
    CHECK_EQ(spark_gradient_limit_x10(100, true, false), 100, "desligado: degrau inteiro");

    ms42.spark_grad_inc_x10 = 10u;  // 1,0°/volta a subir
    ms42.spark_grad_dec_x10 = 30u;  // 3,0°/volta a descer
    spark_gradient_reset();
    CHECK_EQ(spark_gradient_limit_x10(100, false, false), 100, "1a chamada semeia");
    CHECK_EQ(spark_gradient_limit_x10(300, false, false), 100, "sem volta nova: segura");
    CHECK_EQ(spark_gradient_limit_x10(300, true, false), 110, "sobe 1,0 por volta");
    CHECK_EQ(spark_gradient_limit_x10(300, true, false), 120, "sobe 1,0 por volta (2)");
    CHECK_EQ(spark_gradient_limit_x10(50, true, false), 90, "desce 3,0 por volta");
    CHECK_EQ(spark_gradient_limit_x10(85, true, false), 85, "passo menor que o limite: chega");
    CHECK_EQ(spark_gradient_limit_x10(300, true, true), 300, "partida/luz de ponto: direto");
    CHECK_EQ(spark_gradient_limit_x10(100, true, false), 100, "apos bypass: semeia de novo");

    ms42_cal_defaults();
    spark_gradient_reset();
}
