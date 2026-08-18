#pragma once

#include <cstdint>

#include "drv/ckp.h"

namespace ems::drv {

// fault_bits: bitmask de falhas ativas em SensorData.
// Cada bit corresponde ao valor numérico do SensorId (SensorId::MAP=0, CLT=3, etc.)
// conforme o padrão em sensors.cpp:  fault_bits |= (1u << static_cast<uint8_t>(id))
//
// ATENÇÃO: FAULT_WBO2_TIMEOUT foi removido — WBO2 é monitorado via STATUS_WBO2_FAULT
// no can_stack (app/can_stack.h), não via fault_bits de sensores analógicos.
// Usar (1u << 0u) para MAP fault e (1u << 3u) para CLT fault (ver main.cpp).

// throttle_fault_bits (independente de fault_bits analógicos)
static constexpr uint8_t THROTTLE_FAULT_APP1       = (1u << 0u);
static constexpr uint8_t THROTTLE_FAULT_APP2       = (1u << 1u);
static constexpr uint8_t THROTTLE_FAULT_APP_PLAUS  = (1u << 2u);
static constexpr uint8_t THROTTLE_FAULT_ETB_TPS1   = (1u << 3u);
static constexpr uint8_t THROTTLE_FAULT_ETB_TPS2  = (1u << 4u);
static constexpr uint8_t THROTTLE_FAULT_ETB_PLAUS = (1u << 5u);

struct SensorData {
    uint16_t map_bar_x1000;         // MAP bar × 10
    uint32_t maf_gps_x100;        // MAF g/s × 100
    uint16_t tps_pct_x10;         // TPS % × 10 (cabo legado PA4)
    int16_t  clt_degc_x10;        // CLT °C × 10
    int16_t  iat_degc_x10;        // IAT °C × 10
    // o2_mv REMOVIDO — sistema usa exclusivamente WBO2 via CAN (ID 0x180)
    uint16_t fuel_press_bar_x1000;  // pressão combustível bar × 10
    uint16_t oil_press_bar_x1000;   // pressão óleo bar × 10
    uint16_t vbatt_mv;            // tensão bateria mV
    uint8_t  fault_bits;          // bitmask de falhas ativas
    uint16_t app1_pct_x10;        // APP1 (AN1/PB0) % × 10
    uint16_t app2_pct_x10;        // APP2 (AN2/PB1) % × 10
    uint16_t app_pct_x10;         // pedal demanda validada % × 10
    uint16_t etb_tps1_pct_x10;    // ETB blade TPS1 (AN3/PC0) % × 10
    uint16_t etb_tps2_pct_x10;    // ETB blade TPS2 (AN4/PC1) % × 10
    uint16_t etb_tps_pct_x10;     // posição borboleta validada % × 10
    uint8_t  throttle_fault_bits;
    // Expansão AN1-4 — passthrough bruto, atualizados em sensors_tick_100ms()
    uint16_t an1_raw;
    uint16_t an2_raw;
    uint16_t an3_raw;
    uint16_t an4_raw;
};

enum class SensorId : uint8_t {
    MAP        = 0,
    MAF        = 1,
    TPS        = 2,
    CLT        = 3,
    IAT        = 4,
    O2         = 5,
    FUEL_PRESS = 6,
    OIL_PRESS  = 7,
};

struct SensorRange {
    uint16_t min_raw;
    uint16_t max_raw;
};

void sensors_init() noexcept;
void sensors_on_tooth(const CkpSnapshot& snap) noexcept;

/**
 * @brief Fork MT6835/TIM2 (modo encoder): equivalente à metade "canais
 *        rápidos" de sensors_on_tooth() (MAP/MAF/TPS/knock — filtro IIR,
 *        fault-check, plausibilidade, commit no snapshot público), exposto
 *        para ser chamado do slot de 2 ms do loop principal em vez de 12×/rev
 *        via hook de dente (que não existe em modo encoder). NÃO dispara o
 *        ADC (isso é hardware puro — ver TIM6 free-running em hal/adc.h) nem
 *        toca map_window (chamada separada, mesmo slot). Actualiza o RPM
 *        cacheado usado pelo check de plausibilidade MAP×TPS — sem isto
 *        ficaria congelado em 0 (sensors_on_tooth nunca corre neste modo).
 */
void sensors_sample_fast_channels_encoder(uint32_t rpm_x10) noexcept;

/**
 * @brief Fork MT6835/TIM2 (modo encoder): equivalente ao bloco map_window de
 *        sensors_on_tooth() (leitura MAP crua + map_window_on_tooth()),
 *        exposto para o mesmo slot de 2 ms. Recebe só `tim2_now` (posição
 *        viva) e deriva cycle_deg + phase_A internamente AMBOS da mesma
 *        leitura — nunca combinar um ângulo vivo com o phase_A congelado
 *        do heartbeat (1×/volta): produziria um erro de 360° exactamente
 *        na fronteira de cada volta. state/cmp_confirms vêm de
 *        ckp_snapshot() (heartbeat) sem alteração — granularidade grosseira
 *        é aceitável para esses dois campos.
 */
void sensors_map_window_poll_encoder(uint32_t tim2_now) noexcept;
void sensors_tick_50ms() noexcept;
void sensors_tick_100ms() noexcept;
void sensors_maf_freq_capture_isr(uint16_t period_ticks) noexcept;
void sensors_set_tps_cal(uint16_t raw_min, uint16_t raw_max) noexcept;
void sensors_set_app_cal(uint16_t app1_min, uint16_t app1_max,
                         uint16_t app2_min, uint16_t app2_max) noexcept;
void sensors_set_etb_tps_cal(uint16_t tps1_min, uint16_t tps1_max,
                             uint16_t tps2_min, uint16_t tps2_max) noexcept;
void sensors_set_plausibility(uint16_t app_max_delta_pct_x10,
                              uint16_t etb_max_delta_pct_x10) noexcept;
void sensors_set_etb_harness_present(bool present) noexcept;
// Bench-mode HIL: força CLT/IAT a temperaturas fixas e limpa o fault desses
// canais (pinos físicos ausentes não acionam SENSOR_FAULT). enable=false (default)
// restaura o caminho normal de ADC. Não é resetado por sensors_init.
void sensors_set_bench_clt_iat(bool enable,
                               int16_t clt_degc_x10,
                               int16_t iat_degc_x10) noexcept;
// Query bench mode state (for protection gating in main loop).
bool sensors_is_bench_mode() noexcept;
void sensors_set_range(SensorId id, SensorRange range) noexcept;
// FIX-6: retorna por valor (snapshot atômico com CPSID/CPSIE) em vez de
// referência. Previne torn read: ISR TIM5 (prio 1) pode atualizar g_data entre
// acessos sucessivos a campos diferentes via referência.
SensorData sensors_get() noexcept;

// CRITICAL FIX: Sensor validation functions
bool validate_sensor_range(SensorId id, uint16_t raw_value) noexcept;
bool validate_sensor_values(const SensorData& data) noexcept;
uint8_t get_sensor_health_status() noexcept;

inline constexpr uint16_t kFallbackMapBarX1000  = 1010u;
inline constexpr int16_t  kFallbackCltDegcX10 = 900;
inline constexpr int16_t  kFallbackIatDegcX10 = 250;

#if defined(EMS_HOST_TEST)
void sensors_test_reset() noexcept;
void sensors_test_set_clt_table_entry(uint8_t idx, int16_t degc_x10) noexcept;
void sensors_test_set_iat_table_entry(uint8_t idx, int16_t degc_x10) noexcept;
void sensors_test_tick_100ms() noexcept;
#endif

}  // namespace ems::drv
