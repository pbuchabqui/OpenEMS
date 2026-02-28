#pragma once

#include <cstdint>

namespace ems::hal {

enum class Adc0Channel : uint8_t {
    MAP_SE10 = 0,
    MAF_V_SE11 = 1,
    TPS_SE12 = 2,
    O2_SE4B = 3,
    AN1_SE6B = 4,
    AN2_SE7B = 5,
    AN3_SE8B = 6,
    AN4_SE9B = 7,
};

enum class Adc1Channel : uint8_t {
    CLT_SE14 = 0,
    IAT_SE15 = 1,
    FUEL_PRESS_SE5B = 2,
    OIL_PRESS_SE6B = 3,
};

void adc_init() noexcept;
void adc_pdb_on_tooth(uint16_t tooth_period_ftm3_ticks) noexcept;

uint16_t adc0_read(Adc0Channel ch) noexcept;
uint16_t adc1_read(Adc1Channel ch) noexcept;

#if defined(EMS_HOST_TEST)
void adc_test_set_raw_adc0(Adc0Channel ch, uint16_t raw) noexcept;
void adc_test_set_raw_adc1(Adc1Channel ch, uint16_t raw) noexcept;
uint16_t adc_test_last_pdb_mod() noexcept;
#endif

}  // namespace ems::hal
