#include "engine/map_estimator.h"
#include "engine/math_utils.h"

#include <cstdint>

namespace {

using ems::engine::clamp_u16;
using ems::engine::clamp_i16;
using ems::engine::clamp_u32;

// Estado global do estimador
ems::engine::MapEstimatorState g_map_state = {};

// Parâmetros calibráveis do modelo termodinâmico (continuidade)
ems::engine::ManifoldModelParams g_model_params = {5000u, 256u, 256u};  // 500.0 cc, coeffs neutros (Q8=1.0)

// Histórico de TPS para cálculo de derivada
constexpr uint8_t kTpsHistorySize = 4u;
uint16_t g_tps_history[kTpsHistorySize] = {};
uint32_t g_tps_time_history[kTpsHistorySize] = {};
uint8_t g_tps_history_pos = 0u;

// Ganhos do filtro complementar (Q8)
uint8_t g_steady_gain_q8 = 200u;   // 0.78 (78% sensor, 22% modelo em regime)
uint8_t g_transient_gain_q8 = 64u; // 0.25 (25% sensor, 75% modelo em transiente)

// Acumulador fracionário (Q8) do incremento de MAP previsto pelo modelo. A um
// período de loop de poucos ms, o incremento físico é frequentemente < 1
// unidade de bar_x100 — sem carry, o truncamento inteiro perde-o todos os
// ciclos e o modelo nunca contribui. Mesma técnica de DDA/Bresenham.
int32_t g_map_delta_remainder_q8 = 0;

// Constantes do modelo físico do coletor (equação da continuidade / conservação de massa)
constexpr uint16_t kEngineDisplacementCc = 2000u;   // Cilindrada do motor (referência de escala)
constexpr uint32_t kAtmosphericPressureBarX100 = 100u;  // 1.00 bar

// Limites de detecção de transiente
constexpr int16_t kLightTransientTpsdotX10 = 50u;    // 5 %/s
constexpr int16_t kMediumTransientTpsdotX10 = 150u;  // 15 %/s
constexpr int16_t kHeavyTransientTpsdotX10 = 300u;   // 30 %/s

int32_t clamp_iat_kelvin_x10(int16_t iat_x10) noexcept {
    int32_t iat_k_x10 = static_cast<int32_t>(iat_x10) + 2730;
    if (iat_k_x10 < 2000) iat_k_x10 = 2000;
    if (iat_k_x10 > 4000) iat_k_x10 = 4000;
    return iat_k_x10;
}

// Fluxo de ar admitido pela borboleta (mg/ciclo aprox.), função de abertura,
// ΔP atmosfera-coletor e temperatura do ar admitido.
uint16_t calc_throttle_flow_impl(uint16_t tps_pct_x10, uint16_t map_bar_x100,
                                 int16_t iat_x10) noexcept {
    constexpr uint32_t kMaxFlowMg = 800u;

    const uint32_t throttle_frac_q8 = (static_cast<uint32_t>(tps_pct_x10) * 256u) / 1000u;

    int32_t delta_p_bar_x100 = static_cast<int32_t>(kAtmosphericPressureBarX100) -
                               static_cast<int32_t>(map_bar_x100);
    if (delta_p_bar_x100 < 0) {
        delta_p_bar_x100 = 0;
    }
    const uint32_t delta_p_frac_q8 = (static_cast<uint32_t>(delta_p_bar_x100) * 256u) /
                                     kAtmosphericPressureBarX100;

    const int32_t iat_k_x10 = clamp_iat_kelvin_x10(iat_x10);
    const uint32_t temp_comp_q8 = (2930u * 256u) / static_cast<uint32_t>(iat_k_x10);

    // Produto de três fatores Q8 (abertura × ΔP × temperatura) requer deslocar 24 bits
    // (8 bits por fator) — não 16, que era o erro da proposta original.
    const uint64_t flow_raw = static_cast<uint64_t>(kMaxFlowMg) * throttle_frac_q8 *
                              delta_p_frac_q8 * temp_comp_q8;
    const uint32_t flow_mg = static_cast<uint32_t>(flow_raw >> 24u);
    const uint32_t flow_trimmed = (flow_mg * g_model_params.throttle_flow_coeff_q8) >> 8u;

    return static_cast<uint16_t>(clamp_u32(flow_trimmed, 0u, 2000u));
}

// Fluxo de ar consumido pelo motor (bombeamento speed-density), função de RPM,
// MAP e temperatura, escalado pela cilindrada calibrada.
uint16_t calc_engine_pumping_impl(uint32_t rpm_x10, uint16_t map_bar_x100,
                                  int16_t iat_x10) noexcept {
    if (rpm_x10 < 500u) {
        return 0u;
    }

    const int32_t iat_k_x10 = clamp_iat_kelvin_x10(iat_x10);

    // m_dot ∝ coeff_q8 × MAP × RPM × cilindrada / T (bombeamento speed-density).
    // kPumpingScaleDiv calibra a magnitude para a mesma ordem de grandeza (0-2000 mg)
    // do fluxo de admissão, à cilindrada/RPM/MAP nominais (2000cc, 6000 RPM, 1 bar).
    constexpr uint64_t kPumpingScaleDiv = 1'000'000ull;
    const uint64_t flow_num = static_cast<uint64_t>(g_model_params.engine_pumping_coeff_q8) *
                              map_bar_x100 * rpm_x10 * kEngineDisplacementCc;
    const uint32_t flow_mg = static_cast<uint32_t>(
        flow_num / (static_cast<uint64_t>(iat_k_x10) * kPumpingScaleDiv));

    return static_cast<uint16_t>(clamp_u32(flow_mg, 0u, 2000u));
}

int16_t calculate_tpsdot() noexcept {
    // Calcula derivada usando diferença entre amostra mais recente e a mais
    // antiga ainda residente no buffer circular. g_tps_history_pos aponta
    // sempre para o próximo slot a escrever — ou seja, o slot mais antigo
    // ainda válido (prestes a ser sobrescrito). O mais recente escrito é o
    // slot imediatamente anterior (pos - 1, módulo o tamanho do buffer).
    const uint8_t oldest_pos = g_tps_history_pos;
    const uint8_t newest_pos = static_cast<uint8_t>(
        (g_tps_history_pos + kTpsHistorySize - 1u) % kTpsHistorySize);

    if (!g_tps_time_history[oldest_pos] || !g_tps_time_history[newest_pos]) {
        return 0;
    }

    const uint32_t dt_ms = g_tps_time_history[newest_pos] -
                           g_tps_time_history[oldest_pos];

    if (dt_ms == 0u || dt_ms > 1000u) {
        return 0;
    }

    const int32_t delta_tps_x10 = static_cast<int32_t>(g_tps_history[newest_pos]) -
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
    g_map_state.map_estimated_bar_x100 = 50u;  // Valor inicial seguro
    
    for (uint8_t i = 0u; i < kTpsHistorySize; ++i) {
        g_tps_history[i] = 0u;
        g_tps_time_history[i] = 0u;
    }
    g_tps_history_pos = 0u;
    g_map_delta_remainder_q8 = 0;

    g_steady_gain_q8 = 200u;
    g_transient_gain_q8 = 64u;
}

uint16_t map_estimator_update(uint16_t map_sensor_bar_x100,
                              uint16_t tps_pct_x10,
                              uint16_t dt_ms,
                              uint32_t rpm_x10,
                              int16_t iat_x10) noexcept {
    // Validações básicas
    const bool sensor_ok = (map_sensor_bar_x100 >= 10u && map_sensor_bar_x100 <= 300u);
    if (sensor_ok) {
        g_map_state.map_sensor_bar_x100 = map_sensor_bar_x100;
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
    
    // Modelo termodinâmico de continuidade: dP/dt ∝ (fluxo_admissão - fluxo_motor) / VolumeColetor
    // (temperatura já compensada dentro de calc_throttle_flow_impl/calc_engine_pumping_impl).
    const uint16_t flow_in_mg = calc_throttle_flow_impl(
        tps_pct_x10, g_map_state.map_estimated_bar_x100, iat_x10);
    const uint16_t flow_out_mg = calc_engine_pumping_impl(
        rpm_x10, g_map_state.map_estimated_bar_x100, iat_x10);
    const int32_t delta_flow_mg = static_cast<int32_t>(flow_in_mg) -
                                  static_cast<int32_t>(flow_out_mg);

    // dP/dt (bar_x100 por segundo). kDpDtScaleX100PerSec calibra a magnitude para
    // ~20 bar/s no diferencial de fluxo máximo com o coletor de referência (500cc).
    constexpr int64_t kDpDtScaleX100PerSec = 5000;
    const int64_t dpdt_x100_per_s = (static_cast<int64_t>(delta_flow_mg) * kDpDtScaleX100PerSec) /
                                    static_cast<int64_t>(g_model_params.volume_cc_x10);

    // Acumula em Q8 e extrai só a parte inteira, preservando a fração entre
    // chamadas (carry), para não perder incrementos sub-unitários em loops rápidos.
    g_map_delta_remainder_q8 += static_cast<int32_t>(
        (dpdt_x100_per_s * static_cast<int32_t>(dt_ms) * 256) / 1000);
    const int32_t map_delta = g_map_delta_remainder_q8 >> 8;
    g_map_delta_remainder_q8 -= (map_delta << 8);

    const int32_t map_predicted = static_cast<int32_t>(g_map_state.map_estimated_bar_x100) +
                                   map_delta;
    const uint16_t map_predicted_clamped = clamp_u16(
        static_cast<uint16_t>(map_predicted > 0 ? map_predicted : 0),
        10u, 300u);
    
    // Aplica filtro complementar: MAP_final = (gain × sensor) + ((256-gain) × modelo)
    const int32_t map_filtered = (static_cast<int32_t>(map_sensor_bar_x100) * 
                                  static_cast<int32_t>(gain_q8)) / 256 +
                                 (static_cast<int32_t>(map_predicted_clamped) * 
                                  static_cast<int32_t>(256u - gain_q8)) / 256;
    
    g_map_state.map_estimated_bar_x100 = clamp_u16(
        static_cast<uint16_t>(map_filtered), 10u, 300u);
    
    // Determina modo do estimador
    if (g_map_state.transient_strength == 0u) {
        g_map_state.estimator_mode = 0u;  // sensor_only (regime)
    } else {
        g_map_state.estimator_mode = 1u;  // fusion (transiente)
    }
    
    return g_map_state.map_estimated_bar_x100;
}

uint16_t map_get_estimated_bar_x100() noexcept {
    return g_map_state.map_estimated_bar_x100;
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

void map_estimator_set_model_params(const ManifoldModelParams& params) noexcept {
    g_model_params.volume_cc_x10 = clamp_u16(params.volume_cc_x10, 500u, 30000u);
    g_model_params.throttle_flow_coeff_q8 = clamp_u16(params.throttle_flow_coeff_q8, 32u, 512u);
    g_model_params.engine_pumping_coeff_q8 = clamp_u16(params.engine_pumping_coeff_q8, 32u, 512u);
}

}  // namespace ems::engine
