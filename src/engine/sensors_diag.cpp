#include "engine/sensors_diag.h"

#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/diagnostic_manager.h"
#include "hal/adc.h"

namespace ems::engine {

void sensors_diag_poll() noexcept {
    const bool adc_failed = ems::hal::adc_recovery_failed();
    const bool adc_recovering = ems::hal::adc_is_recovering();

    // ADC: CRITICAL se esgotou as tentativas (fica preso até o reset, decisão de 2026-10-22 em
    // docs/protection_system.md); WARNING enquanto recupera. Recuperou: limpa só o WARNING
    // (clear_fault_if_not_critical nunca remove o CRITICAL).
    if (adc_failed) {
        DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED, FaultSeverity::CRITICAL,
                                        static_cast<uint16_t>(ems::hal::adc_get_timeout_count()),
                                        static_cast<uint16_t>(ems::hal::adc_get_recovery_retries()));
    } else if (adc_recovering) {
        // ADC_RECOVERY_IN_PROGRESS não existe: usa ADC_RECOVERY_FAILED como WARNING.
        DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED, FaultSeverity::WARNING,
                                        static_cast<uint16_t>(ems::hal::adc_get_timeout_count()),
                                        static_cast<uint16_t>(ems::hal::adc_get_recovery_retries()));
    } else {
        DiagnosticManager::clear_fault_if_not_critical(DiagnosticCode::ADC_RECOVERY_FAILED);
    }
    if (adc_failed || adc_recovering) {
        // Sem leitura confiável de MAP/TPS: a ISR já aplicou o fallback. Não avalia faixa nem plausibilidade
        // (os dados de fallback não são medição).
        return;
    }

    const ems::drv::SensorData d = ems::drv::sensors_get();
    const auto map_faulted = (d.fault_bits >> static_cast<uint8_t>(ems::drv::SensorId::MAP)) & 1u;
    const auto tps_faulted = (d.fault_bits >> static_cast<uint8_t>(ems::drv::SensorId::TPS)) & 1u;

    if (map_faulted != 0u) {
        DiagnosticManager::report_fault(DiagnosticCode::MAP_SENSOR_RANGE, FaultSeverity::WARNING,
                                        ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::MAP), 0u);
    } else {
        DiagnosticManager::clear_fault(DiagnosticCode::MAP_SENSOR_RANGE);
    }
    if (tps_faulted != 0u) {
        DiagnosticManager::report_fault(DiagnosticCode::TPS_SENSOR_RANGE, FaultSeverity::WARNING,
                                        ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::TPS), 0u);
    } else {
        DiagnosticManager::clear_fault(DiagnosticCode::TPS_SENSOR_RANGE);
    }

    // Plausibilidade MAP×TPS×RPM sobre o snapshot commitado e o RPM atual do decoder.
    const uint32_t rpm_x10 = ems::drv::ckp_snapshot().rpm_x10;
    if (!DiagnosticManager::check_sensor_plausibility(d.map_bar_x1000, d.tps_pct_x10, rpm_x10)) {
        DiagnosticManager::report_fault(DiagnosticCode::MAP_TPS_CORRELATION, FaultSeverity::WARNING,
                                        d.map_bar_x1000, d.tps_pct_x10);
    } else {
        DiagnosticManager::clear_fault(DiagnosticCode::MAP_TPS_CORRELATION);
    }
}

}  // namespace ems::engine
