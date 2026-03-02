#include "hal/adc.h"

#include <cstdint>

// =============================================================================
// CORREÇÃO [FIX-4]
// ADC_CFG1: ADIV=1 (/2), MODE=01 (12-bit), ADICLK=00 (bus clock 60 MHz)
// Anterior: MODE=11 (16-bit) para todos os canais — incorreto para 12-bit.
// Resultado raw agora em [0..4095] para ADC0 e ADC1.
// =============================================================================

namespace {

#if defined(EMS_HOST_TEST)
static uint16_t g_adc0_raw[8]   = {};
static uint16_t g_adc1_raw[4]   = {};
static uint16_t g_last_pdb_mod  = 0u;
#else
#define SIM_SCGC6              (*reinterpret_cast<volatile uint32_t*>(0x4004803Cu))
#define SIM_SCGC6_ADC0_MASK   (1u << 27)
#define SIM_SCGC6_PDB_MASK    (1u << 23)
#define SIM_SCGC3              (*reinterpret_cast<volatile uint32_t*>(0x40048030u))
#define SIM_SCGC3_ADC1_MASK   (1u << 27)

#define ADC0_SC1A  (*reinterpret_cast<volatile uint32_t*>(0x4003B000u))
#define ADC0_RA    (*reinterpret_cast<volatile uint32_t*>(0x4003B010u))
#define ADC0_CFG1  (*reinterpret_cast<volatile uint32_t*>(0x4003B008u))
#define ADC0_CFG2  (*reinterpret_cast<volatile uint32_t*>(0x4003B00Cu))

#define ADC1_SC1A  (*reinterpret_cast<volatile uint32_t*>(0x400BB000u))
#define ADC1_RA    (*reinterpret_cast<volatile uint32_t*>(0x400BB010u))
#define ADC1_CFG1  (*reinterpret_cast<volatile uint32_t*>(0x400BB008u))
#define ADC1_CFG2  (*reinterpret_cast<volatile uint32_t*>(0x400BB00Cu))

#define PDB0_SC      (*reinterpret_cast<volatile uint32_t*>(0x40036000u))
#define PDB0_MOD     (*reinterpret_cast<volatile uint32_t*>(0x40036004u))
#define PDB0_IDLY    (*reinterpret_cast<volatile uint32_t*>(0x40036008u))
#define PDB0_CH0C1   (*reinterpret_cast<volatile uint32_t*>(0x40036010u))
#define PDB0_CH0DLY0 (*reinterpret_cast<volatile uint32_t*>(0x40036018u))
#define PDB0_CH1C1   (*reinterpret_cast<volatile uint32_t*>(0x40036030u))
#define PDB0_CH1DLY0 (*reinterpret_cast<volatile uint32_t*>(0x40036038u))

#define ADC_SC1_COCO (1u << 7)

// [FIX-4] MODE=01 → 12-bit; ADIV=1 (/2) → clock efetivo = 30 MHz; ADICLK=00
// Anterior: MODE=11 (16-bit) — incorreto para todos os canais exceto MAP.
#define ADC_CFG1_12B_DIV2  ((0x1u << 5) | (0x1u << 2))

// MUXSEL=0 (side-A)
#define ADC_CFG2_MUXA      0u

// PDB_SC: PDBEN=1, TRGSEL=8 (FTM3 trigger), LDOK=1
#define PDB_SC_PDBEN       (1u << 0)
#define PDB_SC_LDOK        (1u << 6)
#define PDB_SC_TRGSEL_FTM3 (8u << 12)

#define PDB_CHnC1_EN0      (1u << 0)

inline uint16_t adc_read_blocking(volatile uint32_t& sc1a,
                                  volatile uint32_t& ra,
                                  uint8_t channel_code) noexcept {
    sc1a = static_cast<uint32_t>(channel_code & 0x1Fu);
    while ((sc1a & ADC_SC1_COCO) == 0u) {}
    return static_cast<uint16_t>(ra & 0xFFFFu);
}

inline uint8_t adc0_code(ems::hal::Adc0Channel ch) noexcept {
    switch (ch) {
        case ems::hal::Adc0Channel::MAP_SE10:   return 10u;
        case ems::hal::Adc0Channel::MAF_V_SE11: return 11u;
        case ems::hal::Adc0Channel::TPS_SE12:   return 12u;
        case ems::hal::Adc0Channel::O2_SE4B:    return 4u;
        case ems::hal::Adc0Channel::AN1_SE6B:   return 6u;
        case ems::hal::Adc0Channel::AN2_SE7B:   return 7u;
        case ems::hal::Adc0Channel::AN3_SE8B:   return 8u;
        case ems::hal::Adc0Channel::AN4_SE9B:   return 9u;
    }
    return 10u;
}

inline uint8_t adc1_code(ems::hal::Adc1Channel ch) noexcept {
    switch (ch) {
        case ems::hal::Adc1Channel::CLT_SE14:        return 14u;
        case ems::hal::Adc1Channel::IAT_SE15:        return 15u;
        case ems::hal::Adc1Channel::FUEL_PRESS_SE5B: return 5u;
        case ems::hal::Adc1Channel::OIL_PRESS_SE6B:  return 6u;
    }
    return 14u;
}
#endif

}  // namespace

namespace ems::hal {

void adc_init() noexcept {
#if defined(EMS_HOST_TEST)
    for (uint8_t i = 0u; i < 8u; ++i) { g_adc0_raw[i] = 0u; }
    for (uint8_t i = 0u; i < 4u; ++i) { g_adc1_raw[i] = 0u; }
    g_last_pdb_mod = 0u;
#else
    SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK | SIM_SCGC6_PDB_MASK);
    SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;

    // [FIX-4] ADC_CFG1_12B_DIV2 (MODE=01) em vez de 16-bit (MODE=11)
    ADC0_CFG1 = ADC_CFG1_12B_DIV2;
    ADC0_CFG2 = ADC_CFG2_MUXA;
    ADC1_CFG1 = ADC_CFG1_12B_DIV2;
    ADC1_CFG2 = ADC_CFG2_MUXA;

    // Hardware averaging: 4 amostras para ADC0 (K64 RM §31.3.5)
    // ADC_SC3: AVGE=bit2=1, AVGS=bits[1:0]=00 → 4 samples
    // Melhora relacao sinal/ruido do MAP sem custo de CPU.
    (*reinterpret_cast<volatile uint32_t*>(0x4003B024u)) = (1u << 2u);

    // PDB: acionado por FTM3, dispara ADC0 e ADC1 simultaneamente no mesmo
    // ângulo de virabrequim; DLY0=0 → amostragem imediata no trigger.
    PDB0_SC      = 0u;
    PDB0_IDLY    = 0u;
    PDB0_MOD     = 1000u;
    PDB0_CH0C1   = PDB_CHnC1_EN0;
    PDB0_CH1C1   = PDB_CHnC1_EN0;
    PDB0_CH0DLY0 = 0u;
    PDB0_CH1DLY0 = 0u;
    PDB0_SC      = PDB_SC_PDBEN | PDB_SC_TRGSEL_FTM3 | PDB_SC_LDOK;
#endif
}

// Chamado a cada dente de CKP via sensors_on_tooth().
// tooth_period_ticks está em ticks de FTM3.
//
// FTM3: 120 MHz / prescaler 2 = 60 MHz efetivo (16.667 ns/tick) — confirmado ftm.h
// PDB:  bus clock = 60 MHz
// Razão FTM3/PDB = 1:1 → PDB0_MOD = tooth_period_ticks (sem divisão).
//
// Bug anterior: dividia por 2 assumindo FTM3=120 MHz vs PDB=60 MHz — incorreto.
// Com a divisão, PDB disparava o ADC 2× mais rápido que o período de dente real.
void adc_pdb_on_tooth(uint16_t tooth_period_ticks) noexcept {
    const uint16_t mod = tooth_period_ticks;
#if defined(EMS_HOST_TEST)
    g_last_pdb_mod = mod;
#else
    PDB0_MOD = (mod == 0u) ? 1u : mod;
    PDB0_SC |= PDB_SC_LDOK;
#endif
}

uint16_t adc0_read(Adc0Channel ch) noexcept {
#if defined(EMS_HOST_TEST)
    const uint8_t idx = static_cast<uint8_t>(ch);
    return (idx < 8u) ? g_adc0_raw[idx] : 0u;
#else
    return adc_read_blocking(ADC0_SC1A, ADC0_RA, adc0_code(ch));
#endif
}

uint16_t adc1_read(Adc1Channel ch) noexcept {
#if defined(EMS_HOST_TEST)
    const uint8_t idx = static_cast<uint8_t>(ch);
    return (idx < 4u) ? g_adc1_raw[idx] : 0u;
#else
    return adc_read_blocking(ADC1_SC1A, ADC1_RA, adc1_code(ch));
#endif
}

#if defined(EMS_HOST_TEST)
void adc_test_set_raw_adc0(Adc0Channel ch, uint16_t raw) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx < 8u) { g_adc0_raw[idx] = raw; }
}

void adc_test_set_raw_adc1(Adc1Channel ch, uint16_t raw) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx < 4u) { g_adc1_raw[idx] = raw; }
}

uint16_t adc_test_last_pdb_mod() noexcept {
    return g_last_pdb_mod;
}
#endif

}  // namespace ems::hal
