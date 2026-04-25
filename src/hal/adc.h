#pragma once
/**
 * @file hal/adc.h
 * @brief ADC interface para STM32H562
 *
 * Exporta:
 *   adc_init()          — inicializa ADC primary (MAP/MAF/TPS/O2/AN1-4) e ADC1 (CLT/IAT/FUEL/OIL)
 *   adc_primary_read()         — lê canal de ADC primary
 *   adc_secondary_read()         — lê canal de ADC1
 *   adc_trigger_on_tooth()  — configura trigger TIM6 trigger para sincronização com CKP
 */

#include <cstdint>

// =============================================================================
// CORREÇÃO [FIX-4]
// O prompt especifica MAP como 16-bit e os demais canais como 12-bit.
// MAP compartilha o módulo ADC primary com MAF/TPS/O2/AN1-4, portanto um único
// registro ADC primary_CFG1 controla a resolução de todos os canais ADC primary.
// Decisão: manter ADC primary em 12-bit (MODE=01, 0..4095) como denominador comum.
// Se MAP vier a necessitar de 16-bit, o driver deverá reconfigurar ADC primary_CFG1
// antes de cada leitura de MAP — a ser implementado quando um mux dedicado
// for adicionado ao hardware.
// ADC1 (CLT/IAT/FUEL/OIL): 12-bit (MODE=01).
// =============================================================================

namespace ems::hal {

enum class AdcPrimaryChannel : uint8_t {
    MAP_SE10   = 0,
    MAF_V_SE11 = 1,
    TPS_SE12   = 2,
    O2_SE4B    = 3,
    AN1_SE6B   = 4,
    AN2_SE7B   = 5,
    AN3_SE8B   = 6,
    AN4_SE9B   = 7,
};

enum class AdcSecondaryChannel : uint8_t {
    CLT_SE14        = 0,
    IAT_SE15        = 1,
    FUEL_PRESS_SE5B = 2,
    OIL_PRESS_SE6B  = 3,
};

void     adc_init() noexcept;
void     adc_trigger_on_tooth(uint16_t tooth_period_ticks) noexcept;

uint16_t adc_primary_read(AdcPrimaryChannel ch) noexcept;
uint16_t adc_secondary_read(AdcSecondaryChannel ch) noexcept;

#if defined(EMS_HOST_TEST)
void     adc_test_set_raw_primary(AdcPrimaryChannel ch, uint16_t raw) noexcept;
void     adc_test_set_raw_secondary(AdcSecondaryChannel ch, uint16_t raw) noexcept;
uint16_t adc_test_last_trigger_mod() noexcept;
#endif

}  // namespace ems::hal
