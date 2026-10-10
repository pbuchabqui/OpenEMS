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
#include "engine/fuel_trim.h"
#include "engine/ign_calc.h"
#include "engine/limp_gating.h"
#include "engine/auxiliaries.h"
#include "engine/knock.h"
#include "engine/ms42_cal.h"
#include "engine/quick_crank.h"
#include "engine/torque_manager.h"
#include "hal/system.h"
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

    // Offsets usados em tools/ts/openems.ini (page 1 = fw page0).
    ms42.crank_mult_x256[0] = 0x1234u;
    ms42.idle_ff_x10[0] = 0x0456u;
    ms42.knock_gain_x10[0] = 0x2Au;
    ms42.stft_min_rpm_x10 = 0x0789u;
    ms42_serialize_to_page0(page, sizeof(page));
    CHECK_EQ(page[279], 1u, "INI: ms42VvtEnable @279");
    CHECK_EQ(page[300 + 2 * 6 + 3], 33u, "INI: ms42VvtTarget @300");
    CHECK_EQ(page[387] | (page[388] << 8), 0x1234, "INI: ms42CrankMult @387");
    CHECK_EQ(page[453], 0x2Au, "INI: ms42KnockGain @453");
    CHECK_EQ(page[459] | (page[460] << 8), 0x0456, "INI: ms42IdleFf @459");
    CHECK_EQ(page[503] | (page[504] << 8), 0x0789, "INI: ms42StftMinRpm @503");
    CHECK_EQ(page[505] | (page[506] << 8), 980, "INI: ms42FanOn @505");
    CHECK_EQ(kMs42Page0Off + ms42_cal_page0_len(), 509, "INI: bloco termina em 509");
    ms42_cal_defaults();
    ms42.vvt_enable = 1u;
    ms42.vvt_target_deg[2][3] = 33u;
    ms42.dfco_entry_delay_ms = 250u;
    ms42.fan_on_x10 = 980;

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

    // Sem bordas de came: repouso apos 6 voltas (3 ciclos de came).
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

namespace {

// Partida a 200 rpm, 100 ms por update; devolve o multiplicador do último.
uint16_t crank_run(uint32_t& now, uint32_t updates, int16_t clt_x10) {
    QuickCrankOutput qc{};
    for (uint32_t i = 0u; i < updates; ++i) {
        qc = quick_crank_update(now, 2000u, true, clt_x10, 10);
        now += 100u;
    }
    return qc.fuel_mult_x256;
}

}  // namespace

void test_ms42_crank_afterstart(void) {
    section("ms42 B7/B8: partida (taper, repartida quente, baro) e pos-partida por ciclos");
    ms42_cal_defaults();
    const uint16_t baro_saved = fuel_get_baro_bar_x100();
    uint32_t now = 1000u;

    quick_crank_reset();
    const uint16_t base = crank_run(now, 30u, 200);
    CHECK_EQ(base, 512u, "defaults: 2,0x a 20 C (tabela antiga)");

    // Taper: 4 ciclos até 70 % (200 rpm → 600 ms/ciclo → 2,4 s).
    ms42.crank_taper_cycles = 4u;
    quick_crank_reset();
    const uint16_t first = crank_run(now, 1u, 200);
    CHECK_EQ(first, 512u, "taper: 1o update ainda 100 %");
    const uint16_t mid = crank_run(now, 12u, 200);
    CHECK(mid < 512u && mid > 358u, "taper: a meio fica entre 100 % e 70 %");
    const uint16_t tapered = crank_run(now, 30u, 200);
    CHECK_EQ(tapered, static_cast<uint16_t>(512u * 70u / 100u), "taper: satura em 70 %");
    ms42.crank_taper_cycles = 0u;

    // Repartida a quente: CLT ≥ 90 °C → ×120 %.
    quick_crank_reset();
    const uint16_t hot0 = crank_run(now, 2u, 1000);
    ms42.hot_restart_pct = 120u;
    quick_crank_reset();
    const uint16_t hot1 = crank_run(now, 2u, 1000);
    CHECK_EQ(hot1, static_cast<uint16_t>(hot0 * 120u / 100u), "repartida quente x1,2");
    quick_crank_reset();
    const uint16_t warm1 = crank_run(now, 2u, 800);
    ms42.hot_restart_pct = 100u;
    quick_crank_reset();
    CHECK_EQ(crank_run(now, 2u, 800), warm1, "abaixo do limiar de CLT: sem fator");

    // Baro: 0,80 bar → ×80/101.
    fuel_set_baro_bar_x100(80u);
    quick_crank_reset();
    CHECK_EQ(crank_run(now, 2u, 200), 512u, "baro desligado: sem correção");
    ms42.crank_baro_enable = 1u;
    quick_crank_reset();
    CHECK_EQ(crank_run(now, 2u, 200), static_cast<uint16_t>(512u * 80u / 101u), "baro 0,80 bar");
    ms42.crank_baro_enable = 0u;
    fuel_set_baro_bar_x100(baro_saved);

    // Pós-partida por ciclos: 20 °C → 14 ciclos, início 1,25× (320).
    // A 2000 rpm (60 ms/ciclo), 7 updates de 60 ms = 7 ciclos → metade.
    ms42.afterstart_by_cycles = 1u;
    quick_crank_reset();
    crank_run(now, 3u, 200);
    QuickCrankOutput qc = quick_crank_update(now, 20000u, true, 200, 10);
    CHECK(qc.afterstart_active, "saiu da partida: pós-partida ativa");
    CHECK_EQ(qc.fuel_mult_x256, 320u, "pós-partida começa no início da curva");
    for (uint32_t i = 0u; i < 7u; ++i) {
        now += 60u;
        qc = quick_crank_update(now, 20000u, true, 200, 10);
    }
    CHECK_EQ(qc.fuel_mult_x256, 288u, "7 de 14 ciclos: metade do enriquecimento");
    for (uint32_t i = 0u; i < 8u; ++i) {
        now += 60u;
        qc = quick_crank_update(now, 20000u, true, 200, 10);
    }
    CHECK_EQ(qc.fuel_mult_x256, 256u, "14 ciclos: fim da pós-partida");

    ms42_cal_defaults();
    quick_crank_reset();
}

namespace {

// Janela de knock do cilindro 0 com amostras base e base+p2p.
void knock_window(uint16_t p2p) {
    knock_window_open(0u);
    knock_test_set_adc_raw(1000u);
    knock_test_set_adc_raw(static_cast<uint16_t>(1000u + p2p));
    knock_test_set_adc_raw(1000u);
    knock_window_cycle_end();
}

}  // namespace

void test_ms42_knock_relative(void) {
    section("ms42 C9: knock relativo ao ruido por cilindro + passo/max/recuperacao");
    ems::drv::ckp_test_reset();  // rpm 0 → ganho do 1o ponto (2,5×)
    ms42_cal_defaults();
    knock_init();
    knock_retard_x10[0] = 0u;
    knock_window(300u);
    CHECK_EQ(knock_get_retard_x10(0u), 0u, "absoluto: abaixo do limiar ADC, sem knock");

    ms42.knock_rel_enable = 1u;
    knock_init();
    knock_retard_x10[0] = 0u;
    knock_window(1000u);
    CHECK_EQ(knock_get_retard_x10(0u), 0u, "fase de aprendizagem: não deteta");
    knock_init();
    knock_retard_x10[0] = 0u;
    for (uint32_t i = 0u; i < 16u; ++i) { knock_window(100u); }
    CHECK_EQ(knock_test_get_cyl_noise_x16(0u), 1600u, "ruído aprendido = p2p 100 (×16)");
    knock_window(200u);
    CHECK_EQ(knock_get_retard_x10(0u), 0u, "2,0× o ruído < ganho 2,5: limpo");
    CHECK_EQ(knock_test_get_cyl_noise_x16(0u), 1800u, "janela limpa entra na média");
    knock_window(300u);
    CHECK_EQ(knock_get_retard_x10(0u), 20u, "acima de 2,5× o ruído: +2,0°");
    CHECK_EQ(knock_test_get_cyl_noise_x16(0u), 1800u, "janela com knock não entra na média");

    ms42.knock_step_x10 = 30u;
    ms42.knock_max_x10 = 40u;
    knock_window(400u);
    CHECK_EQ(knock_get_retard_x10(0u), 40u, "passo calibrável, saturado no máximo");

    ms42.knock_clean_cycles = 2u;
    ms42.knock_recovery_x10 = 5u;
    knock_window(100u);
    CHECK_EQ(knock_get_retard_x10(0u), 40u, "1 ciclo limpo: ainda não recupera");
    knock_window(100u);
    CHECK_EQ(knock_get_retard_x10(0u), 35u, "2 ciclos limpos: −0,5°");

    ms42_cal_defaults();
    knock_init();
    knock_retard_x10[0] = 0u;
}

void test_ms42_idle_p_ff_cat(void) {
    section("ms42 C10: marcha lenta FF por CLT + termo P + aprendido + aquecimento do cat");
    ms42_cal_defaults();
    const uint16_t s_min = etb_idle_min_opening_x10;
    const uint16_t s_max = etb_idle_max_opening_x10;
    const uint16_t s_open = etb_idle_open_pct_x10;
    const uint16_t s_rate = etb_max_rate_pct_per_s;
    const uint8_t  s_valid = etb_cal_valid;
    etb_cal_valid = 1u;
    etb_idle_min_opening_x10 = 30u;
    etb_idle_max_opening_x10 = 200u;
    etb_idle_open_pct_x10 = 80u;
    etb_max_rate_pct_per_s = 0u;
    for (uint8_t i = 0u; i < kIdleFfPts; ++i) { ms42.idle_ff_x10[i] = 120u; }

    ems::drv::CkpSnapshot snap{};
    ems::drv::SensorData sens{};
    sens.clt_degc_x10 = 800;
    sens.app_pct_x10 = 0u;
    auto start_engine = [&]() {
        quick_crank_reset();
        quick_crank_update(0u, 3000u, true, 800, 8);
        snap.rpm_x10 = 3000u;
        (void)torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        quick_crank_update(100u, 8000u, true, 800, 8);
        snap.rpm_x10 = 8500u;
        (void)torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
        host_advance_millis(5000u);  // passa o taper partida→marcha lenta
    };

    torque_manager_reset();
    host_set_millis(10000u);
    start_engine();
    auto out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
    CHECK_EQ(out.etb_target_pct_x10, 120u, "saída da partida: integrador = FF(CLT)");

    ms42.idle_kp_x10 = 50u;
    snap.rpm_x10 = 8000u;  // 50 rpm abaixo → P = 500×50/1000 = 25
    out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
    CHECK_EQ(out.etb_target_pct_x10, 146u, "P + I: 121 + 25");
    for (int i = 0; i < 29; ++i) {
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
    }
    CHECK_EQ(out.etb_target_pct_x10, 175u, "I continua a integrar (150 + 25)");
    ms42.idle_kp_x10 = 0u;

    // Sem persistência: nova partida volta ao FF.
    start_engine();
    out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
    CHECK_EQ(out.etb_target_pct_x10, 120u, "sem persist: recomeça no FF");
    snap.rpm_x10 = 8000u;
    for (int i = 0; i < 30; ++i) {
        out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
    }
    ms42.idle_persist = 1u;
    start_engine();
    snap.rpm_x10 = 8500u;
    out = torque_manager_update(snap, sens, true, false, false, 8500u, 2u);
    CHECK_EQ(out.etb_target_pct_x10, 150u, "persist: FF + aprendido (+30)");

    // Aquecimento do catalisador: +200 rpm a decair em 1 s.
    ms42_cal_defaults();
    auxiliaries_test_reset();
    ems::drv::ckp_test_reset();
    const uint16_t base = auxiliaries_idle_target_rpm_x10(800);
    ms42.cat_heat_rpm_x10 = 2000u;
    ms42.cat_heat_s = 1u;
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(800), base, "motor parado: sem acréscimo");
    run_engine(3u, false, 0u);
    const uint16_t t0 = auxiliaries_idle_target_rpm_x10(800);
    CHECK(t0 > base + 1800u && t0 <= base + 2000u, "logo após pegar: ~+200 rpm");
    run_engine(50u, false, 0u);
    const uint16_t t1 = auxiliaries_idle_target_rpm_x10(800);
    CHECK(t1 > base + 800u && t1 < base + 1200u, "a meio: ~+100 rpm");
    run_engine(60u, false, 0u);
    CHECK_EQ(auxiliaries_idle_target_rpm_x10(800), base, "após cat_heat_s: alvo normal");

    ms42_cal_defaults();
    torque_manager_reset();
    quick_crank_reset();
    auxiliaries_test_reset();
    etb_idle_min_opening_x10 = s_min;
    etb_idle_max_opening_x10 = s_max;
    etb_idle_open_pct_x10 = s_open;
    etb_max_rate_pct_per_s = s_rate;
    etb_cal_valid = s_valid;
}

void test_ms42_stft_gain_table(void) {
    section("ms42 C11: ganho do STFT por rpm x carga + rpm minima");
    ms42_cal_defaults();
    fuel_reset_adaptives();
    // 3000 rpm, 100 kPa, lambda 1,20 vs 1,00.
    const int16_t s1 = fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false, 5000u, 500u);
    CHECK(s1 > 0, "ganho 100 %: STFT positivo");

    std::memset(ms42.stft_gain_pct[3], 200, sizeof(ms42.stft_gain_pct[3]));  // 100 kPa
    fuel_reset_adaptives();
    const int16_t s2 = fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(s2, static_cast<int16_t>(2 * s1), "linha de 100 kPa a 200 %: dobra P e I");
    fuel_reset_adaptives();
    const int16_t s3 = fuel_update_stft(30000u, 30u, 1000, 1200, 900, true, false, false, 5000u, 500u);
    CHECK_EQ(s3, s1, "30 kPa: linha a 100 %, ganho inalterado");

    ms42.stft_min_rpm_x10 = 40000u;
    fuel_reset_adaptives();
    CHECK_EQ(fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false, 5000u, 500u), 0,
             "abaixo de stft_min_rpm: congelado");

    ms42_cal_defaults();
    fuel_reset_adaptives();
}
