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
    MAP       = 0,  // PA3 / INP15
    MAF_V     = 1,  // (reservado)
    TPS       = 2,  // PA4 / INP18
    KNOCK     = 3,  // PA5 / INP19
    APP1      = 4,  // PC0 / INP10
    APP2      = 5,  // PC2 / INP12
    ETB_TPS1  = 6,  // PC4 / INP4
    ETB_TPS2  = 7,  // PC5 / INP8
};

enum class AdcSecondaryChannel : uint8_t {
    CLT        = 0,  // PB0 / INP9
    IAT        = 1,  // PB1 / INP5
    FUEL_PRESS = 2,  // PC4 / INP4
    OIL_PRESS  = 3,  // PC5 / INP8
    EWG_POS    = 4,  // PC3 / INP13
};

void     adc_init() noexcept;
void     adc_trigger_on_tooth(uint32_t tooth_period_ticks) noexcept;

uint16_t adc_primary_read(AdcPrimaryChannel ch) noexcept;
uint16_t adc_secondary_read(AdcSecondaryChannel ch) noexcept;

// P0 #3: ADC Recovery System - status flags para verificação em tempo de execução
bool     adc_is_recovering() noexcept;
bool     adc_recovery_failed() noexcept;
uint32_t adc_get_timeout_count() noexcept;
uint32_t adc_get_recovery_retries() noexcept;

#if defined(EMS_HOST_TEST)
void     adc_test_set_raw_primary(AdcPrimaryChannel ch, uint16_t raw) noexcept;
void     adc_test_set_raw_secondary(AdcSecondaryChannel ch, uint16_t raw) noexcept;
uint32_t adc_test_last_trigger_mod() noexcept;
void     adc_test_set_recovering(bool recovering) noexcept;
void     adc_test_set_recovery_failed(bool failed) noexcept;
void     adc_test_set_timeout_count(uint32_t count) noexcept;
void     adc_test_set_recovery_retries(uint32_t retries) noexcept;
#endif

}  // namespace ems::hal
