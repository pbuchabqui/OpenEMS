#include "test/harness.h"
#include "test/fixtures.h"

#include <cstdint>

#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/diagnostic_manager.h"
#include "engine/sensors_diag.h"
#include "hal/adc.h"

using namespace ems::drv;
using namespace ems::engine;
using namespace ems::hal;

extern volatile uint32_t ems_test_tim5_cnt;

// Estado limpo: sensores com ADC válido, tabela de DTCs vazia, decoder parado.
static void diag_setup(void) {
    sensor_setup();
    sensors_init();
    DiagnosticManager::init();
    ckp_test_reset();
    g_ckp_cap = 0u;
    ems_test_tim5_cnt = 0u;
    adc_test_set_recovering(false);
    adc_test_set_recovery_failed(false);
}

// Gira a ISR do dente o bastante para o canal rápido amostrar e commitar (12 amostras/volta de 60 dentes).
static void feed_teeth(uint32_t rpm_x10, uint32_t n) {
    CkpSnapshot snap{};
    snap.tooth_period_ns = 160000u;
    snap.rpm_x10 = rpm_x10;
    for (uint32_t i = 0u; i < n; ++i) { sensors_on_tooth(snap); }
}

void test_diag_clear_if_not_critical(void) {
    section("DiagnosticManager: clear_fault_if_not_critical");
    DiagnosticManager::init();

    CHECK_FALSE(DiagnosticManager::clear_fault_if_not_critical(DiagnosticCode::ADC_RECOVERY_FAILED),
                "falha inexistente → false");

    DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED, FaultSeverity::WARNING);
    CHECK_TRUE(DiagnosticManager::clear_fault_if_not_critical(DiagnosticCode::ADC_RECOVERY_FAILED),
               "WARNING é removido");
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::ADC_RECOVERY_FAILED), "WARNING saiu da tabela");

    DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED, FaultSeverity::CRITICAL);
    CHECK_FALSE(DiagnosticManager::clear_fault_if_not_critical(DiagnosticCode::ADC_RECOVERY_FAILED),
                "CRITICAL NÃO é removido");
    CHECK_TRUE(DiagnosticManager::is_fault_active(DiagnosticCode::ADC_RECOVERY_FAILED), "CRITICAL segue preso");

    // WARNING escalado a CRITICAL (recuperando → falhou): vira CRITICAL e fica preso.
    DiagnosticManager::init();
    DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED, FaultSeverity::WARNING);
    DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED, FaultSeverity::CRITICAL);
    CHECK_FALSE(DiagnosticManager::clear_fault_if_not_critical(DiagnosticCode::ADC_RECOVERY_FAILED),
                "WARNING escalado a CRITICAL não é removido");
    DiagnosticManager::init();
}

void test_sensors_diag_plausibility_rule(void) {
    section("DiagnosticManager: plausibilidade MAP×TPS só com motor rodando (B2)");
    // Chave ligada e motor parado (rpm 0, inclusive sem sync): MAP atmosférico é normal.
    CHECK_TRUE(DiagnosticManager::check_sensor_plausibility(1000u, 20u, 0u), "KOEO, borboleta fechada, MAP 1 bar → plausível");
    CHECK_TRUE(DiagnosticManager::check_sensor_plausibility(1000u, 300u, 0u), "KOEO, TPS 30% → plausível");
    // Cranking (~250 RPM): idem.
    CHECK_TRUE(DiagnosticManager::check_sensor_plausibility(1000u, 20u, 2500u), "cranking 250 RPM → plausível");
    // Motor rodando em marcha lenta (800 RPM) com MAP alto: suspeito.
    CHECK_FALSE(DiagnosticManager::check_sensor_plausibility(900u, 200u, 8000u), "800 RPM, MAP 0,9 bar → implausível");
    CHECK_TRUE(DiagnosticManager::check_sensor_plausibility(400u, 200u, 8000u), "800 RPM, MAP 0,4 bar → plausível");
    // Borboleta fechada com MAP atmosférico, motor rodando: suspeito.
    CHECK_FALSE(DiagnosticManager::check_sensor_plausibility(960u, 10u, 15000u), "1500 RPM, TPS 1%, MAP 0,96 → implausível");
    // As regras que não dependem de o motor estar rodando seguem valendo.
    CHECK_FALSE(DiagnosticManager::check_sensor_plausibility(500u, 950u, 0u), "TPS 95% com MAP 0,5 bar → implausível mesmo parado");
    CHECK_FALSE(DiagnosticManager::check_sensor_plausibility(200u, 500u, 70000u), "7000 RPM com MAP 0,2 bar → implausível");
}

void test_sensors_diag_range_out_of_isr(void) {
    section("sensors_diag: DTC de faixa sai da ISR e vai para o poll");
    diag_setup();
    SensorRange r{100u, 4000u};
    sensors_set_range(SensorId::MAP, r);
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 0u);   // abaixo da faixa
    feed_teeth(8000u, 200u);

    CHECK_TRUE((sensors_get().fault_bits & (1u << static_cast<uint8_t>(SensorId::MAP))) != 0u,
               "ISR: bit de falha de MAP já está commitado");
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_SENSOR_RANGE),
                "ISR NÃO reporta o DTC (varredura sob CPSID fora do dente)");

    sensors_diag_poll();
    CHECK_TRUE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_SENSOR_RANGE), "poll reporta MAP_SENSOR_RANGE");

    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 2000u);   // volta à faixa
    feed_teeth(8000u, 200u);
    sensors_diag_poll();
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_SENSOR_RANGE), "poll limpa ao voltar à faixa");

    // TPS idem.
    sensors_set_range(SensorId::TPS, r);
    adc_test_set_raw_primary(AdcPrimaryChannel::TPS, 4095u);   // acima da faixa
    feed_teeth(8000u, 200u);
    sensors_diag_poll();
    CHECK_TRUE(DiagnosticManager::is_fault_active(DiagnosticCode::TPS_SENSOR_RANGE), "poll reporta TPS_SENSOR_RANGE");
    DiagnosticManager::init();
}

void test_sensors_diag_plausibility_poll(void) {
    section("sensors_diag: MAP_TPS_CORRELATION no poll (KOEO/cranking sem DTC, rodando com DTC)");
    // Motor parado (rpm 0): MAP ~1,46 bar (raw 2000) é normal.
    diag_setup();
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::TPS, 100u);
    feed_teeth(0u, 200u);
    sensors_diag_poll();
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_TPS_CORRELATION),
                "KOEO: sem MAP_TPS_CORRELATION espúrio");

    // Motor girando com sync (625 RPM), mesmo MAP alto com borboleta quase fechada: implausível.
    diag_setup();
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::TPS, 100u);
    ckp_reach_full_sync();
    ems_test_tim5_cnt = g_ckp_cap;
    CHECK_TRUE(ckp_snapshot().rpm_x10 >= 4000u, "pré-condição: decoder com RPM de motor rodando");
    feed_teeth(ckp_snapshot().rpm_x10, 200u);
    sensors_diag_poll();
    CHECK_TRUE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_TPS_CORRELATION),
               "motor rodando com MAP alto e marcha lenta: DTC no poll");

    ckp_test_reset(); g_ckp_cap = 0u; ems_test_tim5_cnt = 0u;
    DiagnosticManager::init();
}

void test_sensors_diag_adc_recovery(void) {
    section("sensors_diag: ADC em recuperação (WARNING limpa) × falhou (CRITICAL preso)");
    diag_setup();

    // Em recuperação: WARNING.
    adc_test_set_recovering(true);
    sensors_diag_poll();
    const DiagnosticEvent* ev = DiagnosticManager::get_event(DiagnosticCode::ADC_RECOVERY_FAILED);
    CHECK_TRUE(ev != nullptr && ev->severity == FaultSeverity::WARNING, "recuperando → WARNING");
    // Sem leitura confiável o poll não avalia faixa/plausibilidade.
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::MAP_TPS_CORRELATION),
                "recuperando: sem plausibilidade sobre dados de fallback");

    // Recuperou: o WARNING não fica preso (B3).
    adc_test_set_recovering(false);
    sensors_diag_poll();
    CHECK_FALSE(DiagnosticManager::is_fault_active(DiagnosticCode::ADC_RECOVERY_FAILED),
                "recuperou → WARNING limpo");

    // Esgotou as tentativas: CRITICAL.
    adc_test_set_recovery_failed(true);
    sensors_diag_poll();
    ev = DiagnosticManager::get_event(DiagnosticCode::ADC_RECOVERY_FAILED);
    CHECK_TRUE(ev != nullptr && ev->severity == FaultSeverity::CRITICAL, "falhou → CRITICAL");

    // Decisão do projeto (docs/protection_system.md): CRITICAL só sai com reset.
    adc_test_set_recovery_failed(false);
    sensors_diag_poll();
    CHECK_TRUE(DiagnosticManager::is_fault_active(DiagnosticCode::ADC_RECOVERY_FAILED),
               "CRITICAL continua preso mesmo se o flag do ADC voltar");

    adc_test_set_recovering(false);
    adc_test_set_recovery_failed(false);
    DiagnosticManager::init();
}
