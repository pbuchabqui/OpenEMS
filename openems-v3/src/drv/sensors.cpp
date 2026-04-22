#include "drv/sensors.h"

#include <cstdint>

#if __has_include("hal/adc.h")
#include "hal/adc.h"
#elif __has_include("adc.h")
#include "adc.h"
#endif

// =============================================================================
// CORRECOES APLICADAS
// =============================================================================
// [FIX-1] g_fault[] estatico sincronizado com reset_state() -- ranges identicos
//         em ambos os locais (MAP/TPS/MAF: max 4095; FUEL/OIL: max 4050).
//
// [FIX-2] sensors_on_tooth() converte snap.tooth_period_ns -> ticks FTM3 com
//         fator correto (*60/1000), pois FTM3 opera a 60 MHz efetivo
//         (120 MHz system / prescaler 2, confirmado em hal/ftm.h).
//         Codigo original usava fator 120 (erro 2x). CkpSnapshot nao expoe
//         tooth_period_ftm3_ticks -- tooth_period_ns e o campo canonico.
//
// [FIX-3] AN1-4 (ADC0_SE6b..9b) amostrados em sensors_tick_100ms() como
//         passthrough, publicados em SensorData.an1_raw..an4_raw.
//
// [FIX-4] ADC0 e ADC1 configurados em 12-bit (MODE=01). Codigo original
//         usava 16-bit (MODE=11) para todos os canais -- incorreto.
//
// [FIX-5] kMafFtm3ClockHz corrigido para 60_000_000 Hz.
//         FTM3 CH1 captura MAF no mesmo timer: 120 MHz / prescaler 2 = 60 MHz.
//         Valores anteriores (24 MHz) estavam incorretos.
// =============================================================================

namespace {

using ems::drv::SensorData;
using ems::drv::SensorId;
using ems::drv::SensorRange;

constexpr uint8_t  kFaultLimit         = 3u;
constexpr uint16_t kRealTeethPerRev    = 58u;
constexpr uint16_t kFastSamplesPerRev  = 12u;

constexpr uint16_t kFallbackMapKpaX10  = 1010u;
constexpr uint16_t kFallbackTpsPctX10  = 0u;
constexpr int16_t  kFallbackCltDegcX10 = 900;
constexpr int16_t  kFallbackIatDegcX10 = 250;

// FTM3: 120 MHz system clock / prescaler 2 = 60 MHz efetivo = 16.667 ns/tick
// PDB:  bus clock = 60 MHz
// Razão 1:1 — nenhuma conversão de escala necessária.
// Confirmado por hal/ftm.h: "FTM3: 120 MHz / prescaler 2 → 16.67 ns/tick"
constexpr uint32_t kMafFtm3ClockHz = 60000000u;

struct FaultTracker {
    SensorRange range;
    uint8_t     consecutive_bad;
    bool        active;
};

// [FIX-1] ranges canônicos definidos em um único lugar; usados tanto na
//         inicialização estática quanto em reset_state().
//         MAP (0): 50..4095  — full-scale válido para 16-bit e 12-bit
//         MAF (1): 10..4095  — sinal de tensão, 0 absoluto = chicote aberto
//         TPS (2): 50..4095  — igual ao MAP
//         CLT (3): 100..3800 — NTC: fio aberto → 4095, curto → 0
//         IAT (4): 100..3900 — NTC com range ligeiramente maior
//         O2  (5): 10..4095  — sinal banda estreita, 0 = sensor frio
//         FUEL(6): 50..4050  — transdutor pressão com dead-band nas bordas
//         OIL (7): 50..4050  — idem
constexpr FaultTracker kDefaultFault[8] = {
    {{  50u, 4095u}, 0u, false},  // MAP
    {{  10u, 4095u}, 0u, false},  // MAF
    {{  50u, 4095u}, 0u, false},  // TPS
    {{ 100u, 3800u}, 0u, false},  // CLT
    {{ 100u, 3900u}, 0u, false},  // IAT
    {{  10u, 4095u}, 0u, false},  // O2
    {{  50u, 4050u}, 0u, false},  // FUEL_PRESS
    {{  50u, 4050u}, 0u, false},  // OIL_PRESS
};

// FIX-6: volatile — escrita por sensors_on_tooth() (ISR FTM3, prio 1),
// lida pelo background via sensors_get(). volatile + CPSID em sensors_get()
// garantem snapshot consistente sem torn read.
static volatile SensorData g_data = {};

// [FIX-1] inicialização estática agora usa kDefaultFault — idêntico ao reset
static FaultTracker g_fault[8] = {
    kDefaultFault[0], kDefaultFault[1], kDefaultFault[2], kDefaultFault[3],
    kDefaultFault[4], kDefaultFault[5], kDefaultFault[6], kDefaultFault[7],
};

static uint16_t g_tps_raw_min = 200u;
static uint16_t g_tps_raw_max = 3895u;

static uint16_t g_map_filt  = 0u;
static uint16_t g_o2_filt   = 0u;

static uint16_t g_tps_buf[4]  = {};
static uint8_t  g_tps_pos     = 0u;

static uint16_t g_clt_buf[8]  = {};
static uint8_t  g_clt_pos     = 0u;
static uint16_t g_iat_buf[8]  = {};
static uint8_t  g_iat_pos     = 0u;

static uint16_t g_fuel_buf[4] = {};
static uint8_t  g_fuel_pos    = 0u;
static uint16_t g_oil_buf[4]  = {};
static uint8_t  g_oil_pos     = 0u;

static uint16_t g_maf_period_buf[4] = {};
static uint8_t  g_maf_period_pos    = 0u;

static int16_t g_clt_table[128] = {};
static int16_t g_iat_table[128] = {};

static uint16_t g_fast_sample_accum = 0u;

// -----------------------------------------------------------------------------
// reset_state — estado canônico usando kDefaultFault [FIX-1]
// -----------------------------------------------------------------------------
inline void reset_state() noexcept {
    const_cast<SensorData&>(g_data) = SensorData{};  // safe: reset antes de ISRs ativas

    g_map_filt  = 0u;
    g_o2_filt   = 0u;

    for (uint8_t i = 0u; i < 4u; ++i) {
        g_tps_buf[i]        = 0u;
        g_fuel_buf[i]       = 0u;
        g_oil_buf[i]        = 0u;
        g_maf_period_buf[i] = 0u;
    }
    for (uint8_t i = 0u; i < 8u; ++i) {
        g_clt_buf[i] = 0u;
        g_iat_buf[i] = 0u;
    }

    g_tps_pos        = 0u;
    g_clt_pos        = 0u;
    g_iat_pos        = 0u;
    g_fuel_pos       = 0u;
    g_oil_pos        = 0u;
    g_maf_period_pos = 0u;
    g_fast_sample_accum = 0u;

    g_tps_raw_min = 200u;
    g_tps_raw_max = 3895u;

    // [FIX-1] único ponto de verdade para os ranges padrão
    for (uint8_t i = 0u; i < 8u; ++i) {
        g_fault[i] = kDefaultFault[i];
    }
}

// -----------------------------------------------------------------------------
// Filtros IIR — inteiros, sem float
// α=0.3 → y = y + ((x - y) × 3) / 10
// α=0.1 → y = y + (x - y) / 10
// Usa int32_t intermediário para evitar overflow em operandos uint16_t.
// Nudge ±1 garante convergência monotônica mesmo quando |delta| < 10.
// -----------------------------------------------------------------------------
inline uint16_t iir_alpha_03(uint16_t y, uint16_t x) noexcept {
    const int32_t delta = static_cast<int32_t>(x) - static_cast<int32_t>(y);
    int32_t step = (delta * 3) / 10;
    if (step == 0 && delta != 0) {
        step = (delta > 0) ? 1 : -1;
    }
    const int32_t out = static_cast<int32_t>(y) + step;
    if (out <= 0)    { return 0u; }
    if (out >= 4095) { return 4095u; }
    return static_cast<uint16_t>(out);
}

inline uint16_t iir_alpha_01(uint16_t y, uint16_t x) noexcept {
    const int32_t delta = static_cast<int32_t>(x) - static_cast<int32_t>(y);
    int32_t step = delta / 10;
    if (step == 0 && delta != 0) {
        step = (delta > 0) ? 1 : -1;
    }
    const int32_t out = static_cast<int32_t>(y) + step;
    if (out <= 0)    { return 0u; }
    if (out >= 4095) { return 4095u; }
    return static_cast<uint16_t>(out);
}

inline uint8_t sensor_bit(SensorId id) noexcept {
    return static_cast<uint8_t>(id);
}

inline uint16_t avg4(const uint16_t* v) noexcept {
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(v[0]) + v[1] + v[2] + v[3]) / 4u);
}

inline uint16_t avg8(const uint16_t* v) noexcept {
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < 8u; ++i) { sum += v[i]; }
    return static_cast<uint16_t>(sum / 8u);
}

// -----------------------------------------------------------------------------
// Detecção de falha — 3 amostras consecutivas fora do range → fault ativo
// -----------------------------------------------------------------------------
inline void apply_fault(SensorId id, uint16_t raw) noexcept {
    FaultTracker& f = g_fault[static_cast<uint8_t>(id)];
    const bool bad = (raw < f.range.min_raw) || (raw > f.range.max_raw);

    if (bad) {
        if (f.consecutive_bad < 255u) { ++f.consecutive_bad; }
        if (f.consecutive_bad >= kFaultLimit) { f.active = true; }
    } else {
        f.consecutive_bad = 0u;
        f.active = false;
    }

    const uint8_t bit = sensor_bit(id);
    if (f.active) {
        g_data.fault_bits = static_cast<uint8_t>(
            g_data.fault_bits | static_cast<uint8_t>(1u << bit));
    } else {
        g_data.fault_bits = static_cast<uint8_t>(
            g_data.fault_bits & static_cast<uint8_t>(~(1u << bit)));
    }
}

// LUT 128 entradas; raw [0..4095] → índice [0..127] via shift de 5 bits
inline int16_t lut128(const int16_t* table, uint16_t raw) noexcept {
    const uint8_t idx = static_cast<uint8_t>(raw >> 5u);
    return table[idx];
}

// Tabela padrão linear: -40.0°C a +150.0°C (×10), 128 pontos
inline void init_tables() noexcept {
    for (uint16_t i = 0u; i < 128u; ++i) {
        const int32_t t = -400 + static_cast<int32_t>((1900u * i) / 127u);
        g_clt_table[i] = static_cast<int16_t>(t);
        g_iat_table[i] = static_cast<int16_t>(t);
    }
}

// MAP: 0-5V linear → 0..250.0 kPa (×10)
inline uint16_t map_raw_to_kpa_x10(uint16_t raw) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 2500u) / 4095u);
}

// O2: 0-5V linear → 0..1000 mV
inline uint16_t raw_to_mv(uint16_t raw) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 1000u) / 4095u);
}

// TPS: calibração dinâmica min/max → 0..100.0% (×10)
inline uint16_t tps_raw_to_pct_x10(uint16_t raw) noexcept {
    if (g_tps_raw_max <= g_tps_raw_min) { return 0u; }
    if (raw <= g_tps_raw_min)           { return 0u; }
    if (raw >= g_tps_raw_max)           { return 1000u; }
    const uint32_t num = static_cast<uint32_t>(raw - g_tps_raw_min) * 1000u;
    const uint32_t den = static_cast<uint32_t>(g_tps_raw_max - g_tps_raw_min);
    return static_cast<uint16_t>(num / den);
}

inline uint16_t maf_period_avg4() noexcept {
    return avg4(g_maf_period_buf);
}

// -----------------------------------------------------------------------------
// Canais rápidos — chamado ~12× por revolução via sensors_on_tooth()
// MAP, MAF-V, TPS, O2 — todos sincronizados ao mesmo ângulo de virabrequim
// -----------------------------------------------------------------------------
inline void sample_fast_channels() noexcept {
    const uint16_t map_raw  = ems::hal::adc0_read(ems::hal::Adc0Channel::MAP_SE10);
    const uint16_t mafv_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::MAF_V_SE11);
    const uint16_t tps_raw  = ems::hal::adc0_read(ems::hal::Adc0Channel::TPS_SE12);
    const uint16_t o2_raw   = ems::hal::adc0_read(ems::hal::Adc0Channel::O2_SE4B);

    g_map_filt = iir_alpha_03(g_map_filt, map_raw);
    g_o2_filt  = iir_alpha_01(g_o2_filt,  o2_raw);

    g_tps_buf[g_tps_pos] = tps_raw;
    g_tps_pos = static_cast<uint8_t>((g_tps_pos + 1u) & 0x3u);

    apply_fault(SensorId::MAP, map_raw);
    apply_fault(SensorId::MAF, mafv_raw);
    apply_fault(SensorId::TPS, tps_raw);
    apply_fault(SensorId::O2,  o2_raw);

    g_data.map_kpa_x10 = g_fault[static_cast<uint8_t>(SensorId::MAP)].active
                         ? kFallbackMapKpaX10
                         : map_raw_to_kpa_x10(g_map_filt);

    g_data.tps_pct_x10 = g_fault[static_cast<uint8_t>(SensorId::TPS)].active
                         ? kFallbackTpsPctX10
                         : tps_raw_to_pct_x10(avg4(g_tps_buf));

    // MAF: estimativa por frequência via FTM3 CH1 (120 MHz / prescaler 2 = 60 MHz)
    const uint16_t maf_avg_period = maf_period_avg4();
    g_data.maf_gps_x100 = (maf_avg_period > 0u)
                          ? static_cast<uint16_t>(kMafFtm3ClockHz / maf_avg_period)
                          : 0u;
}

}  // namespace

// =============================================================================
// API pública
// =============================================================================
namespace ems::drv {

// CRITICAL FIX: Sensor validation implementation
bool validate_sensor_range(SensorId id, uint16_t raw_value) noexcept {
    const FaultTracker& f = g_fault[static_cast<uint8_t>(id)];
    return (raw_value >= f.range.min_raw) && (raw_value <= f.range.max_raw);
}

bool validate_sensor_values(const SensorData& data) noexcept {
    // Validate MAP: 10 kPa to 250 kPa (×10)
    if ((data.map_kpa_x10 < 100u) || (data.map_kpa_x10 > 2500u)) {
        return false;
    }
    
    // Validate CLT: -40°C to +150°C (×10)
    if ((data.clt_degc_x10 < -400) || (data.clt_degc_x10 > 1500)) {
        return false;
    }
    
    // Validate IAT: -40°C to +150°C (×10)
    if ((data.iat_degc_x10 < -400) || (data.iat_degc_x10 > 1500)) {
        return false;
    }
    
    // Validate TPS: 0% to 100% (×10)
    if (data.tps_pct_x10 > 1000u) {
        return false;
    }
    
    // Validate battery voltage: 6V to 18V
    if ((data.vbatt_mv < 6000u) || (data.vbatt_mv > 18000u)) {
        return false;
    }
    
    // Validate fuel pressure: 0 kPa to 500 kPa (×10)
    if (data.fuel_press_kpa_x10 > 5000u) {
        return false;
    }
    
    // Validate oil pressure: 0 kPa to 1000 kPa (×10)
    if (data.oil_press_kpa_x10 > 10000u) {
        return false;
    }
    
    return true;
}

uint8_t get_sensor_health_status() noexcept {
    uint8_t status = 0u;
    
    // Check critical sensors for limp mode
    if (g_fault[static_cast<uint8_t>(SensorId::MAP)].active) {
        status |= (1u << 0u);
    }
    if (g_fault[static_cast<uint8_t>(SensorId::CLT)].active) {
        status |= (1u << 1u);
    }
    if (g_fault[static_cast<uint8_t>(SensorId::TPS)].active) {
        status |= (1u << 2u);
    }
    
    return status;
}

void sensors_init() noexcept {
    ems::hal::adc_init();
    init_tables();
    reset_state();
}

// FIX-2 (revisado): CkpSnapshot não expõe tooth_period_ftm3_ticks.
// Converte tooth_period_ns → ticks usando o clock efetivo do FTM3.
// FTM3: 120 MHz / prescaler 2 = 60 MHz → 1 tick = 16.667 ns
//   ticks = ns * 60 / 1000
// PDB opera no mesmo clock (bus clock = 60 MHz) → razão 1:1,
// adc_pdb_on_tooth usa o valor diretamente sem nova conversão.
void sensors_on_tooth(const CkpSnapshot& snap) noexcept {
    const uint16_t ticks = static_cast<uint16_t>((snap.tooth_period_ns * 60u) / 1000u);
    ems::hal::adc_pdb_on_tooth(ticks);

    g_fast_sample_accum = static_cast<uint16_t>(
        g_fast_sample_accum + kFastSamplesPerRev);
    if (g_fast_sample_accum >= kRealTeethPerRev) {
        g_fast_sample_accum = static_cast<uint16_t>(
            g_fast_sample_accum - kRealTeethPerRev);
        sample_fast_channels();
    }
}

void sensors_tick_50ms() noexcept {
    const uint16_t fuel_raw = ems::hal::adc1_read(ems::hal::Adc1Channel::FUEL_PRESS_SE5B);
    const uint16_t oil_raw  = ems::hal::adc1_read(ems::hal::Adc1Channel::OIL_PRESS_SE6B);

    g_fuel_buf[g_fuel_pos] = fuel_raw;
    g_fuel_pos = static_cast<uint8_t>((g_fuel_pos + 1u) & 0x3u);

    g_oil_buf[g_oil_pos] = oil_raw;
    g_oil_pos = static_cast<uint8_t>((g_oil_pos + 1u) & 0x3u);

    apply_fault(SensorId::FUEL_PRESS, fuel_raw);
    apply_fault(SensorId::OIL_PRESS,  oil_raw);

    g_data.fuel_press_kpa_x10 = static_cast<uint16_t>(
        (static_cast<uint32_t>(avg4(g_fuel_buf)) * 2500u) / 4095u);
    g_data.oil_press_kpa_x10 = static_cast<uint16_t>(
        (static_cast<uint32_t>(avg4(g_oil_buf)) * 2500u) / 4095u);
}

// [FIX-3] AN1-4 agora amostrados e publicados como passthrough em SensorData
void sensors_tick_100ms() noexcept {
    const uint16_t clt_raw = ems::hal::adc1_read(ems::hal::Adc1Channel::CLT_SE14);
    const uint16_t iat_raw = ems::hal::adc1_read(ems::hal::Adc1Channel::IAT_SE15);

    g_clt_buf[g_clt_pos] = clt_raw;
    g_clt_pos = static_cast<uint8_t>((g_clt_pos + 1u) & 0x7u);

    g_iat_buf[g_iat_pos] = iat_raw;
    g_iat_pos = static_cast<uint8_t>((g_iat_pos + 1u) & 0x7u);

    apply_fault(SensorId::CLT, clt_raw);
    apply_fault(SensorId::IAT, iat_raw);

    const uint16_t clt_avg = avg8(g_clt_buf);
    const uint16_t iat_avg = avg8(g_iat_buf);

    g_data.clt_degc_x10 = g_fault[static_cast<uint8_t>(SensorId::CLT)].active
                          ? kFallbackCltDegcX10
                          : lut128(g_clt_table, clt_avg);
    g_data.iat_degc_x10 = g_fault[static_cast<uint8_t>(SensorId::IAT)].active
                          ? kFallbackIatDegcX10
                          : lut128(g_iat_table, iat_avg);

    // Expansão AN1-4: passthrough direto — sem filtro, sem fault tracking
    // [FIX-3] canais antes ignorados; agora publicados em SensorData
    g_data.an1_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::AN1_SE6B);
    g_data.an2_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::AN2_SE7B);
    g_data.an3_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::AN3_SE8B);
    g_data.an4_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::AN4_SE9B);

    // TODO: VBATT via canal dedicado quando mapeamento elétrico for definido.
    g_data.vbatt_mv = 12000u;
}

void sensors_maf_freq_capture_isr(uint16_t period_ticks) noexcept {
    g_maf_period_buf[g_maf_period_pos] = period_ticks;
    g_maf_period_pos = static_cast<uint8_t>((g_maf_period_pos + 1u) & 0x3u);
}

void sensors_set_tps_cal(uint16_t raw_min, uint16_t raw_max) noexcept {
    g_tps_raw_min = raw_min;
    g_tps_raw_max = raw_max;
}

void sensors_set_range(SensorId id, SensorRange range) noexcept {
    g_fault[static_cast<uint8_t>(id)].range = range;
}

SensorData sensors_get() noexcept {
    // FIX-6: snapshot atômico — CPSID impede preempção pela ISR FTM3 durante
    // a cópia de 26 bytes, garantindo que todos os campos pertencem ao mesmo
    // instante de amostragem. Sem este critical section, um torn read pode
    // combinar map_kpa_x10 de antes da ISR com clt_degc_x10 de depois.
    SensorData out;
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsid i" ::: "memory");
#endif
    out.map_kpa_x10        = g_data.map_kpa_x10;
    out.maf_gps_x100       = g_data.maf_gps_x100;
    out.tps_pct_x10        = g_data.tps_pct_x10;
    out.clt_degc_x10       = g_data.clt_degc_x10;
    out.iat_degc_x10       = g_data.iat_degc_x10;
    out.fuel_press_kpa_x10 = g_data.fuel_press_kpa_x10;
    out.oil_press_kpa_x10  = g_data.oil_press_kpa_x10;
    out.vbatt_mv           = g_data.vbatt_mv;
    out.fault_bits         = g_data.fault_bits;
    out.an1_raw            = g_data.an1_raw;
    out.an2_raw            = g_data.an2_raw;
    out.an3_raw            = g_data.an3_raw;
    out.an4_raw            = g_data.an4_raw;
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsie i" ::: "memory");
#endif
    return out;
}

#if defined(EMS_HOST_TEST)
void sensors_test_reset() noexcept {
    reset_state();
}

void sensors_test_set_clt_table_entry(uint8_t idx, int16_t degc_x10) noexcept {
    if (idx < 128u) { g_clt_table[idx] = degc_x10; }
}

void sensors_test_set_iat_table_entry(uint8_t idx, int16_t degc_x10) noexcept {
    if (idx < 128u) { g_iat_table[idx] = degc_x10; }
}
#endif

}  // namespace ems::drv
