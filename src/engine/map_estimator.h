#pragma once

#include <cstdint>

namespace ems::engine {

// Estrutura de estado do estimador de MAP
struct MapEstimatorState {
    uint16_t map_estimated_kpa;      // MAP estimado/filtrado
    uint16_t map_sensor_kpa;         // MAP lido do sensor
    int16_t tpsdot_x10;              // Derivada do TPS (%/s)
    uint8_t estimator_mode;          // 0=sensor_only, 1=fusion, 2=model_only
    uint8_t transient_strength;      // 0=steady, 1=light, 2=medium, 3=heavy
};

// Inicializa o estimador de MAP
void map_estimator_init() noexcept;

// Atualiza estimativa de MAP com sensor fusion
// Combina leitura física do sensor + modelo baseado em TPSdot
uint16_t map_estimator_update(uint16_t map_sensor_kpa,
                              uint16_t tps_pct_x10,
                              uint16_t dt_ms,
                              uint32_t rpm_x10) noexcept;

// Obtém MAP estimado para cálculo de combustível
uint16_t map_get_estimated_kpa() noexcept;

// Obtém derivada do TPS (taxa de variação)
int16_t map_get_tpsdot_x10() noexcept;

// Detecta se está em transiente de carga
bool map_is_transient() noexcept;

// Obtém estado atual para diagnóstico
MapEstimatorState map_estimator_get_state() noexcept;

// Configura ganhos do filtro complementar (valores Q8)
void map_estimator_set_gains(uint8_t steady_gain_q8,
                             uint8_t transient_gain_q8) noexcept;

}  // namespace ems::engine
