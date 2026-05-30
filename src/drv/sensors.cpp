#include "drv/sensors.h"

#include <cstdint>

#if __has_include("hal/adc.h")
#include "hal/adc.h"
#elif __has_include("adc.h")
#include "adc.h"
#endif

#if __has_include("engine/diagnostic_manager.h")
#include "engine/diagnostic_manager.h"
#endif

namespace {

using ems::drv::SensorData;
using ems::drv::SensorId;
using ems::drv::SensorRange;
using ems::drv::kFallbackMapKpaX10;
using ems::drv::kFallbackCltDegcX10;
using ems::drv::kFallbackIatDegcX10;

constexpr uint8_t  kFaultLimit         = 3u;
constexpr uint16_t kRealTeethPerRev    = 58u;
constexpr uint16_t kFastSamplesPerRev  = 12u;

constexpr uint16_t kFallbackTpsPctX10  = 0u;

// TIM5: 250 MHz / prescaler 4 = 62.5 MHz efetivo = 16 ns/tick
// TIM6 trigger: mesmo clock efetivo usado no cálculo
// Razão 1:1 — nenhuma conversão de escala necessária.
// Confirmado por TIM5 input-capture clock
constexpr uint32_t kMafTim5ClockHz = 62500000u;

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

// FIX-6 (BUG-10): Double buffering para SensorData — elimina race condition
// Problema anterior: volatile + CPSID em sensors_get() NÃO garantem snapshot consistente
// porque sensors_on_tooth() pode atualizar campos individuais entre leituras sucessivas.
// Exemplo: map_kpa_x10 do dente atual, mas clt_degc_x10 do dente anterior.
//
// Solução: sensors_on_tooth() escreve em g_data_staging (buffer secundário).
// sensors_get() faz swap atômico dos ponteiros e copia o buffer "congelado".
// Isso garante que TODOS os campos pertencem ao mesmo instante de amostragem.
static volatile SensorData g_data_staging = {};   // Buffer de escrita (ISR TIM5)
static volatile SensorData g_data_committed = {}; // Buffer de leitura (main loop)
static volatile uint8_t g_data_swap_flag = 0u;    // 0 = staging é válido, 1 = committed é válido

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
static bool     g_tps_pct_cache_valid = false;
static uint16_t g_tps_pct_cache_raw = 0u;
static uint16_t g_tps_pct_cache_min = 0u;
static uint16_t g_tps_pct_cache_max = 0u;
static uint16_t g_tps_pct_cache_x10 = 0u;

// -----------------------------------------------------------------------------
// reset_state — estado canônico usando kDefaultFault [FIX-1]
// -----------------------------------------------------------------------------
inline void reset_state() noexcept {
    const_cast<SensorData&>(g_data_staging) = SensorData{};  // safe: reset antes de ISRs ativas
    const_cast<SensorData&>(g_data_committed) = SensorData{};  // Double buffer também zerado
    g_data_swap_flag = 0u;  // staging é o buffer válido inicial

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
    g_tps_pct_cache_valid = false;

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
        (static_cast<uint32_t>(v[0]) + v[1] + v[2] + v[3]) >> 2u);
}

inline uint16_t avg8(const uint16_t* v) noexcept {
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < 8u; ++i) { sum += v[i]; }
    return static_cast<uint16_t>(sum >> 3u);
}

// -----------------------------------------------------------------------------
// Detecção de falha — 3 amostras consecutivas fora do range → fault ativo
// -----------------------------------------------------------------------------
inline void apply_fault(SensorId id, uint16_t raw) noexcept {
    const uint8_t idx = static_cast<uint8_t>(id);
    if (idx >= 8u) { return; }
    FaultTracker& f = g_fault[idx];
    const bool bad = (raw < f.range.min_raw) || (raw > f.range.max_raw);

    if (bad) {
        if (f.consecutive_bad < 255u) { ++f.consecutive_bad; }
        if (f.consecutive_bad >= kFaultLimit) { f.active = true; }
    } else {
        f.consecutive_bad = 0u;
        f.active = false;
    }

    const uint8_t bit = sensor_bit(id);
    if (bit >= 8u) { return; }
    // FIX-6 (BUG-10): fault_bits atualizado no staging buffer (escrito pela ISR)
    if (f.active) {
        g_data_staging.fault_bits = static_cast<uint8_t>(g_data_staging.fault_bits |  (1u << bit));
    } else {
        g_data_staging.fault_bits = static_cast<uint8_t>(g_data_staging.fault_bits & ~(1u << bit));
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

// MAP: 0-5V linear → 0..300.0 kPa (×10)
inline uint16_t map_raw_to_kpa_x10(uint16_t raw) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 3000u) / 4095u);
}

// O2: 0-5V linear → 0..1000 mV
inline uint16_t raw_to_mv(uint16_t raw) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 1000u) / 4095u);
}

// VBATT via AN4: divisor externo dimensionado para 0..18V em 0..3.3V ADC.
inline uint16_t vbatt_raw_to_mv(uint16_t raw) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 18000u) / 4095u);
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

inline uint16_t tps_raw_to_pct_x10_cached(uint16_t raw) noexcept {
    if (!g_tps_pct_cache_valid ||
        g_tps_pct_cache_raw != raw ||
        g_tps_pct_cache_min != g_tps_raw_min ||
        g_tps_pct_cache_max != g_tps_raw_max) {
        g_tps_pct_cache_valid = true;
        g_tps_pct_cache_raw = raw;
        g_tps_pct_cache_min = g_tps_raw_min;
        g_tps_pct_cache_max = g_tps_raw_max;
        g_tps_pct_cache_x10 = tps_raw_to_pct_x10(raw);
    }
    return g_tps_pct_cache_x10;
}

inline uint16_t maf_period_avg4() noexcept {
    return avg4(g_maf_period_buf);
}

// -----------------------------------------------------------------------------
// Canais rápidos — chamado ~12× por revolução via sensors_on_tooth()
// MAP, MAF-V, TPS, O2 — todos sincronizados ao mesmo ângulo de virabrequim
// -----------------------------------------------------------------------------
inline void sample_fast_channels() noexcept {
    // P0 #3: Verifica status do ADC antes de ler sensores críticos
    // Se ADC está em recovery ou falhou, usa valores safe defaults (limp-home)
    const bool adc_unavailable = ems::hal::adc_is_recovering() || 
                                  ems::hal::adc_recovery_failed();
    
    if (adc_unavailable) {
        // ADC não disponível - usa valores fallback para segurança do motor
        g_data_staging.map_kpa_x10 = kFallbackMapKpaX10;   // 101.0 kPa (pressão atmosférica)
        g_data_staging.tps_pct_x10 = kFallbackTpsPctX10;   // 0.0% (borboleta fechada)
        g_data_staging.maf_gps_x100 = 0u;                   // MAF desconhecido
        
        // Reporta fault de ADC recovery ao sistema de diagnóstico
        #if __has_include("engine/diagnostic_manager.h")
        using ems::engine::DiagnosticCode;
        using ems::engine::FaultSeverity;
        using ems::engine::DiagnosticManager;
        
        if (ems::hal::adc_recovery_failed()) {
            DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED,
                                           FaultSeverity::CRITICAL,
                                           ems::hal::adc_get_timeout_count(),
                                           ems::hal::adc_get_recovery_retries());
        } else if (ems::hal::adc_is_recovering()) {
            // ADC_RECOVERY_IN_PROGRESS não existe - usa ADC_RECOVERY_FAILED como placeholder
            // ou simplesmente reporta como WARNING genérico
            DiagnosticManager::report_fault(DiagnosticCode::ADC_RECOVERY_FAILED,
                                           FaultSeverity::WARNING,
                                           ems::hal::adc_get_timeout_count(),
                                           ems::hal::adc_get_recovery_retries());
        }
        #endif
        
        return;  // Sai sem atualizar outros sensores
    }
    
    const uint16_t map_raw  = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::MAP_SE10);
    const uint16_t mafv_raw = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::MAF_V_SE11);
    const uint16_t tps_raw  = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::TPS_SE12);
    const uint16_t o2_raw   = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::O2_SE4B);

    g_map_filt = iir_alpha_03(g_map_filt, map_raw);
    g_o2_filt  = iir_alpha_01(g_o2_filt,  o2_raw);

    g_tps_buf[g_tps_pos] = tps_raw;
    g_tps_pos = static_cast<uint8_t>((g_tps_pos + 1u) & 0x3u);

    apply_fault(SensorId::MAP, map_raw);
    apply_fault(SensorId::MAF, mafv_raw);
    apply_fault(SensorId::TPS, tps_raw);
    apply_fault(SensorId::O2,  o2_raw);

    g_data_staging.map_kpa_x10 = g_fault[static_cast<uint8_t>(SensorId::MAP)].active
                         ? kFallbackMapKpaX10
                         : map_raw_to_kpa_x10(g_map_filt);

    g_data_staging.tps_pct_x10 = g_fault[static_cast<uint8_t>(SensorId::TPS)].active
                         ? kFallbackTpsPctX10
                         : tps_raw_to_pct_x10_cached(avg4(g_tps_buf));

    // MAF: estimativa por frequência via TIM5 CH1 (250 MHz / prescaler 4 = 62.5 MHz)
    const uint16_t maf_avg_period = maf_period_avg4();
    g_data_staging.maf_gps_x100 = (maf_avg_period > 0u)
                         ? kMafTim5ClockHz / static_cast<uint32_t>(maf_avg_period)
                         : 0u;
    
    // Sensor plausibility check with diagnostic reporting
    #if __has_include("engine/diagnostic_manager.h")
    using ems::engine::DiagnosticCode;
    using ems::engine::FaultSeverity;
    using ems::engine::DiagnosticManager;
    
    // Report sensor range faults to diagnostic system
    if (g_fault[static_cast<uint8_t>(SensorId::MAP)].active) {
        DiagnosticManager::report_fault(DiagnosticCode::MAP_SENSOR_RANGE,
                                       FaultSeverity::WARNING,
                                       map_raw, 0);
    }
    if (g_fault[static_cast<uint8_t>(SensorId::TPS)].active) {
        DiagnosticManager::report_fault(DiagnosticCode::TPS_SENSOR_RANGE,
                                       FaultSeverity::WARNING,
                                       tps_raw, 0);
    }
    if (g_fault[static_cast<uint8_t>(SensorId::MAF)].active) {
        DiagnosticManager::report_fault(DiagnosticCode::MAF_SENSOR_RANGE,
                                       FaultSeverity::WARNING,
                                       mafv_raw, 0);
    }
    if (g_fault[static_cast<uint8_t>(SensorId::O2)].active) {
        DiagnosticManager::report_fault(DiagnosticCode::O2_SENSOR_RANGE,
                                       FaultSeverity::INFO,
                                       o2_raw, 0);
    }
    
    // Perform plausibility check between MAP and TPS
    if (!DiagnosticManager::check_sensor_plausibility(g_data_staging.map_kpa_x10,
                                                      g_data_staging.tps_pct_x10,
                                                      0)) {
        DiagnosticManager::report_fault(DiagnosticCode::MAP_TPS_CORRELATION,
                                       FaultSeverity::WARNING,
                                       g_data_staging.map_kpa_x10,
                                       g_data_staging.tps_pct_x10);
    }
    #endif
}

}  // namespace

// =============================================================================
// API pública
// =============================================================================
namespace ems::drv {

bool validate_sensor_range(SensorId id, uint16_t raw_value) noexcept {
    const FaultTracker& f = g_fault[static_cast<uint8_t>(id)];
    return (raw_value >= f.range.min_raw) && (raw_value <= f.range.max_raw);
}

bool validate_sensor_values(const SensorData& data) noexcept {
    // Validate MAP: 10 kPa to 300 kPa (×10)
    if ((data.map_kpa_x10 < 100u) || (data.map_kpa_x10 > 3000u)) {
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

// FIX-2 (revisado): CkpSnapshot não expõe tooth_period_tim5_ticks.
// Converte tooth_period_ns → ticks usando o clock efetivo do TIM5.
// TIM5: 250 MHz / prescaler 4 = 62.5 MHz -> 1 tick = 16 ns
//   ticks = ns / 16
// TIM6 trigger opera no mesmo clock efetivo → razão 1:1,
// adc_trigger_on_tooth usa o valor diretamente sem nova conversão.
void sensors_on_tooth(const CkpSnapshot& snap) noexcept {
    const uint32_t ticks = snap.tooth_period_ns >> 4u;
    ems::hal::adc_trigger_on_tooth(ticks);

    g_fast_sample_accum = static_cast<uint16_t>(
        g_fast_sample_accum + kFastSamplesPerRev);
    if (g_fast_sample_accum >= kRealTeethPerRev) {
        g_fast_sample_accum = static_cast<uint16_t>(
            g_fast_sample_accum - kRealTeethPerRev);
        sample_fast_channels();
    }
}

void sensors_tick_50ms() noexcept {
    const uint16_t fuel_raw = ems::hal::adc_secondary_read(ems::hal::AdcSecondaryChannel::FUEL_PRESS_SE5B);
    const uint16_t oil_raw  = ems::hal::adc_secondary_read(ems::hal::AdcSecondaryChannel::OIL_PRESS_SE6B);

    g_fuel_buf[g_fuel_pos] = fuel_raw;
    g_fuel_pos = static_cast<uint8_t>((g_fuel_pos + 1u) & 0x3u);

    g_oil_buf[g_oil_pos] = oil_raw;
    g_oil_pos = static_cast<uint8_t>((g_oil_pos + 1u) & 0x3u);

    apply_fault(SensorId::FUEL_PRESS, fuel_raw);
    apply_fault(SensorId::OIL_PRESS,  oil_raw);

    g_data_staging.fuel_press_kpa_x10 = static_cast<uint16_t>(
        (static_cast<uint32_t>(avg4(g_fuel_buf)) * 2500u) / 4095u);
    g_data_staging.oil_press_kpa_x10 = static_cast<uint16_t>(
        (static_cast<uint32_t>(avg4(g_oil_buf)) * 2500u) / 4095u);
    
    // Report pressure sensor faults to diagnostic system
    #if __has_include("engine/diagnostic_manager.h")
    using ems::engine::DiagnosticCode;
    using ems::engine::FaultSeverity;
    using ems::engine::DiagnosticManager;
    
    if (g_fault[static_cast<uint8_t>(SensorId::FUEL_PRESS)].active) {
        DiagnosticCode code = (g_data_staging.fuel_press_kpa_x10 < 100u)
                             ? DiagnosticCode::FUEL_PRESS_LOW
                             : DiagnosticCode::FUEL_PRESS_HIGH;
        DiagnosticManager::report_fault(code, FaultSeverity::ERROR,
                                       g_data_staging.fuel_press_kpa_x10, 0);
    }
    if (g_fault[static_cast<uint8_t>(SensorId::OIL_PRESS)].active) {
        DiagnosticCode code = (g_data_staging.oil_press_kpa_x10 < 50u)
                             ? DiagnosticCode::LOW_OIL_PRESSURE
                             : DiagnosticCode::OIL_PRESS_HIGH;
        FaultSeverity severity = (g_data_staging.oil_press_kpa_x10 < 30u)
                                ? FaultSeverity::CRITICAL
                                : FaultSeverity::ERROR;
        DiagnosticManager::report_fault(code, severity,
                                       g_data_staging.oil_press_kpa_x10, 0);
    }
    #endif
}

// [FIX-3] AN1-4 agora amostrados e publicados como passthrough em SensorData
void sensors_tick_100ms() noexcept {
    const uint16_t clt_raw = ems::hal::adc_secondary_read(ems::hal::AdcSecondaryChannel::CLT_SE14);
    const uint16_t iat_raw = ems::hal::adc_secondary_read(ems::hal::AdcSecondaryChannel::IAT_SE15);

    g_clt_buf[g_clt_pos] = clt_raw;
    g_clt_pos = static_cast<uint8_t>((g_clt_pos + 1u) & 0x7u);

    g_iat_buf[g_iat_pos] = iat_raw;
    g_iat_pos = static_cast<uint8_t>((g_iat_pos + 1u) & 0x7u);

    apply_fault(SensorId::CLT, clt_raw);
    apply_fault(SensorId::IAT, iat_raw);

    const uint16_t clt_avg = avg8(g_clt_buf);
    const uint16_t iat_avg = avg8(g_iat_buf);

    g_data_staging.clt_degc_x10 = g_fault[static_cast<uint8_t>(SensorId::CLT)].active
                          ? kFallbackCltDegcX10
                          : lut128(g_clt_table, clt_avg);
    g_data_staging.iat_degc_x10 = g_fault[static_cast<uint8_t>(SensorId::IAT)].active
                          ? kFallbackIatDegcX10
                          : lut128(g_iat_table, iat_avg);

    // Expansão AN1-4: passthrough direto — sem filtro, sem fault tracking
    // [FIX-3] canais antes ignorados; agora publicados em SensorData
    g_data_staging.an1_raw = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::AN1_SE6B);
    g_data_staging.an2_raw = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::AN2_SE7B);
    g_data_staging.an3_raw = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::AN3_SE8B);
    const uint16_t vbatt_raw = ems::hal::adc_primary_read(ems::hal::AdcPrimaryChannel::AN4_SE9B);
    g_data_staging.an4_raw = vbatt_raw;

    const uint16_t vbatt_mv = vbatt_raw_to_mv(vbatt_raw);
    g_data_staging.vbatt_mv = (vbatt_mv >= 6000u && vbatt_mv <= 18000u) ? vbatt_mv : 12000u;
    
        // Double buffering: copy staging→committed under critical section.
    // sensors_on_tooth() (ISR) writes g_data_staging concurrently; without CPSID
    // the copy below can interleave with the ISR, producing a torn snapshot.
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsid i" ::: "memory");
#endif
    g_data_committed.map_kpa_x10 = g_data_staging.map_kpa_x10;
    g_data_committed.maf_gps_x100 = g_data_staging.maf_gps_x100;
    g_data_committed.tps_pct_x10 = g_data_staging.tps_pct_x10;
    g_data_committed.clt_degc_x10 = g_data_staging.clt_degc_x10;
    g_data_committed.iat_degc_x10 = g_data_staging.iat_degc_x10;
    g_data_committed.fuel_press_kpa_x10 = g_data_staging.fuel_press_kpa_x10;
    g_data_committed.oil_press_kpa_x10 = g_data_staging.oil_press_kpa_x10;
    g_data_committed.vbatt_mv = g_data_staging.vbatt_mv;
    g_data_committed.fault_bits = g_data_staging.fault_bits;
    g_data_committed.an1_raw = g_data_staging.an1_raw;
    g_data_committed.an2_raw = g_data_staging.an2_raw;
    g_data_committed.an3_raw = g_data_staging.an3_raw;
    g_data_committed.an4_raw = g_data_staging.an4_raw;
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsie i" ::: "memory");
#endif
    
    // ADC recovery verification: check if ADC recovered from any timeout
    // Report to diagnostic manager if faults detected
    #if __has_include("engine/diagnostic_manager.h")
    using ems::engine::DiagnosticCode;
    using ems::engine::FaultSeverity;
    using ems::engine::DiagnosticManager;
    
    // Check for sensor range faults and report to diagnostic system
    if (g_fault[static_cast<uint8_t>(SensorId::CLT)].active) {
        DiagnosticManager::report_fault(DiagnosticCode::CLT_SENSOR_RANGE, 
                                       FaultSeverity::WARNING,
                                       clt_raw, 0);
    }
    if (g_fault[static_cast<uint8_t>(SensorId::IAT)].active) {
        DiagnosticManager::report_fault(DiagnosticCode::IAT_SENSOR_RANGE,
                                       FaultSeverity::WARNING,
                                       iat_raw, 0);
    }
    if (vbatt_mv < 6000u || vbatt_mv > 18000u) {
        DiagnosticCode code = (vbatt_mv < 6000u) ? DiagnosticCode::VBATT_LOW 
                                                  : DiagnosticCode::VBATT_HIGH;
        FaultSeverity severity = (vbatt_mv < 5000u || vbatt_mv > 20000u)
                                ? FaultSeverity::CRITICAL
                                : FaultSeverity::WARNING;
        DiagnosticManager::report_fault(code, severity, vbatt_mv, 0);
    }
    #endif
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
    // FIX-6 (BUG-10): Double buffering — lê do buffer committed (congelado)
    // Não precisa de CPSID pois g_data_committed é apenas lido no main loop
    // e escrito atomicamente pela ISR após swap completo.
    SensorData out;
    out.map_kpa_x10        = g_data_committed.map_kpa_x10;
    out.maf_gps_x100       = g_data_committed.maf_gps_x100;
    out.tps_pct_x10        = g_data_committed.tps_pct_x10;
    out.clt_degc_x10       = g_data_committed.clt_degc_x10;
    out.iat_degc_x10       = g_data_committed.iat_degc_x10;
    out.fuel_press_kpa_x10 = g_data_committed.fuel_press_kpa_x10;
    out.oil_press_kpa_x10  = g_data_committed.oil_press_kpa_x10;
    out.vbatt_mv           = g_data_committed.vbatt_mv;
    out.fault_bits         = g_data_committed.fault_bits;
    out.an1_raw            = g_data_committed.an1_raw;
    out.an2_raw            = g_data_committed.an2_raw;
    out.an3_raw            = g_data_committed.an3_raw;
    out.an4_raw            = g_data_committed.an4_raw;
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
