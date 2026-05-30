#include "engine/map_estimator.h"
#include "engine/math_utils.h"

#include <cstdint>

namespace {

using ems::engine::clamp_u16;
using ems::engine::clamp_i16;

// Estado global do estimador
ems::engine::MapEstimatorState g_map_state = {};

// Histórico de TPS para cálculo de derivada
constexpr uint8_t kTpsHistorySize = 4u;
uint16_t g_tps_history[kTpsHistorySize] = {};
uint32_t g_tps_time_history[kTpsHistorySize] = {};
uint8_t g_tps_history_pos = 0u;

// Ganhos do filtro complementar (Q8)
uint8_t g_steady_gain_q8 = 200u;   // 0.78 (78% sensor, 22% modelo em regime)
uint8_t g_transient_gain_q8 = 64u; // 0.25 (25% sensor, 75% modelo em transiente)

// Constantes do modelo físico do coletor
constexpr uint16_t kManifoldVolumeCc = 500u;      // Volume típico do coletor (ajustável por calibração)
constexpr uint16_t kEngineDisplacementCc = 2000u; // Cilindrada do motor
constexpr uint8_t kCylinderCount = 4u;

// Limites de detecção de transiente
constexpr int16_t kLightTransientTpsdotX10 = 50u;    // 5 %/s
constexpr int16_t kMediumTransientTpsdotX10 = 150u;  // 15 %/s
constexpr int16_t kHeavyTransientTpsdotX10 = 300u;   // 30 %/s

// Fator de correção baseado em RPM e carga para modelo
uint16_t calculate_model_map_correction(uint32_t rpm_x10,
                                        uint16_t tps_pct_x10) noexcept {
    // Modelo simplificado: MAP estimado = MAP anterior + (TPSdot * ganho volumétrico)
    // Ganho depende de RPM (maior RPM = menor tempo para encher coletor)
    
    if (rpm_x10 < 5000u || rpm_x10 > 100000u) {
        return 100u;  // Ganho unitário em RPM inválida
    }
    
    // Correção volumétrica baseada em RPM
    // Em baixa RPM: coletor enche mais devagar → ganho menor
    // Em alta RPM: coletor enche mais rápido → ganho maior
    const uint32_t rpm_factor = (rpm_x10 * 100u) / 60000u;  // Normaliza em 6000 RPM
    const uint16_t gain = static_cast<uint16_t>(clamp_u16(
        static_cast<uint32_t>(rpm_factor), 50u, 150u));
    
    return gain;
}

int16_t calculate_tpsdot() noexcept {
    // Calcula derivada usando diferença entre amostra atual e antiga
    const uint8_t oldest_pos = (g_tps_history_pos + 1u) % kTpsHistorySize;
    
    if (!g_tps_time_history[oldest_pos] || !g_tps_time_history[g_tps_history_pos]) {
        return 0;
    }
    
    const uint32_t dt_ms = g_tps_time_history[g_tps_history_pos] - 
                           g_tps_time_history[oldest_pos];
    
    if (dt_ms == 0u || dt_ms > 1000u) {
        return 0;
    }
    
    const int32_t delta_tps_x10 = static_cast<int32_t>(g_tps_history[g_tps_history_pos]) -
                                  static_cast<int32_t>(g_tps_history[oldest_pos]);
    
    // TPSdot em %/s × 10 = (delta_tps_x10 * 1000) / dt_ms
    const int32_t tpsdot_x10 = (delta_tps_x10 * 1000) / static_cast<int32_t>(dt_ms);
    
    return clamp_i16(static_cast<int16_t>(tpsdot_x10), -1000, 1000);
}

void update_tps_history(uint16_t tps_pct_x10, uint32_t time_ms) noexcept {
    g_tps_history[g_tps_history_pos] = tps_pct_x10;
    g_tps_time_history[g_tps_history_pos] = time_ms;
    g_tps_history_pos = (g_tps_history_pos + 1u) % kTpsHistorySize;
}

uint8_t detect_transient_strength(int16_t tpsdot_x10) noexcept {
    const int16_t abs_tpsdot = (tpsdot_x10 >= 0) ? tpsdot_x10 : -tpsdot_x10;
    
    if (abs_tpsdot >= kHeavyTransientTpsdotX10) {
        return 3u;  // heavy
    } else if (abs_tpsdot >= kMediumTransientTpsdotX10) {
        return 2u;  // medium
    } else if (abs_tpsdot >= kLightTransientTpsdotX10) {
        return 1u;  // light
    } else {
        return 0u;  // steady
    }
}

}  // namespace

namespace ems::engine {

void map_estimator_init() noexcept {
    g_map_state = {};
    g_map_state.map_estimated_kpa = 50u;  // Valor inicial seguro
    
    for (uint8_t i = 0u; i < kTpsHistorySize; ++i) {
        g_tps_history[i] = 0u;
        g_tps_time_history[i] = 0u;
    }
    g_tps_history_pos = 0u;
    
    g_steady_gain_q8 = 200u;
    g_transient_gain_q8 = 64u;
}

uint16_t map_estimator_update(uint16_t map_sensor_kpa,
                              uint16_t tps_pct_x10,
                              uint16_t dt_ms,
                              uint32_t rpm_x10) noexcept {
    // Validações básicas
    const bool sensor_ok = (map_sensor_kpa >= 10u && map_sensor_kpa <= 300u);
    if (sensor_ok) {
        g_map_state.map_sensor_kpa = map_sensor_kpa;
    } else {
        // Sensor fora de range: usa apenas modelo. NÃO alimentar o valor cru
        // (railed/0) no filtro complementar — abaixo zeramos o ganho do sensor.
        g_map_state.estimator_mode = 2u;
    }
    
    // Atualiza histórico de TPS e calcula derivada
    static uint32_t s_last_time_ms = 0u;
    if (dt_ms > 0u && dt_ms < 1000u) {
        update_tps_history(tps_pct_x10, s_last_time_ms + dt_ms);
        s_last_time_ms += dt_ms;
    }
    
    g_map_state.tpsdot_x10 = calculate_tpsdot();
    g_map_state.transient_strength = detect_transient_strength(g_map_state.tpsdot_x10);
    
    // Seleciona ganho baseado em força do transiente
    uint8_t gain_q8 = g_steady_gain_q8;
    if (!sensor_ok) {
        gain_q8 = 0u;  // Sensor inválido: saída = modelo puro (0% sensor)
    } else if (g_map_state.transient_strength >= 3u) {
        gain_q8 = g_transient_gain_q8;  // Heavy transient: confia mais no modelo
    } else if (g_map_state.transient_strength >= 2u) {
        // Medium transient: blend entre gains
        gain_q8 = static_cast<uint8_t>(
            (static_cast<uint16_t>(g_steady_gain_q8) + 
             static_cast<uint16_t>(g_transient_gain_q8)) >> 1u);
    }
    
    // Calcula predição baseada em TPSdot
    // MAP_predito = MAP_anterior + (TPSdot * ganho_volumétrico * dt)
    const uint16_t model_gain = calculate_model_map_correction(rpm_x10, tps_pct_x10);
    const int32_t map_delta = (static_cast<int32_t>(g_map_state.tpsdot_x10) * 
                               static_cast<int32_t>(model_gain) * 
                               static_cast<int32_t>(dt_ms)) / 10000;
    
    const int32_t map_predicted = static_cast<int32_t>(g_map_state.map_estimated_kpa) + 
                                   (map_delta / 10);  // Escala adequada
    const uint16_t map_predicted_clamped = clamp_u16(
        static_cast<uint16_t>(map_predicted > 0 ? map_predicted : 0),
        10u, 300u);
    
    // Aplica filtro complementar: MAP_final = (gain × sensor) + ((256-gain) × modelo)
    const int32_t map_filtered = (static_cast<int32_t>(map_sensor_kpa) * 
                                  static_cast<int32_t>(gain_q8)) / 256 +
                                 (static_cast<int32_t>(map_predicted_clamped) * 
                                  static_cast<int32_t>(256u - gain_q8)) / 256;
    
    g_map_state.map_estimated_kpa = clamp_u16(
        static_cast<uint16_t>(map_filtered), 10u, 300u);
    
    // Determina modo do estimador
    if (g_map_state.transient_strength == 0u) {
        g_map_state.estimator_mode = 0u;  // sensor_only (regime)
    } else {
        g_map_state.estimator_mode = 1u;  // fusion (transiente)
    }
    
    return g_map_state.map_estimated_kpa;
}

uint16_t map_get_estimated_kpa() noexcept {
    return g_map_state.map_estimated_kpa;
}

int16_t map_get_tpsdot_x10() noexcept {
    return g_map_state.tpsdot_x10;
}

bool map_is_transient() noexcept {
    return (g_map_state.transient_strength >= 1u);
}

MapEstimatorState map_estimator_get_state() noexcept {
    return g_map_state;
}

void map_estimator_set_gains(uint8_t steady_gain_q8,
                             uint8_t transient_gain_q8) noexcept {
    g_steady_gain_q8 = clamp_u16(steady_gain_q8, 64u, 240u);
    g_transient_gain_q8 = clamp_u16(transient_gain_q8, 16u, 128u);
}

}  // namespace ems::engine
