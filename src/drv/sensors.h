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

struct SensorData {
    uint16_t map_kpa_x10;         // MAP kPa × 10
    uint32_t maf_gps_x100;        // MAF g/s × 100
    uint16_t tps_pct_x10;         // TPS % × 10
    int16_t  clt_degc_x10;        // CLT °C × 10
    int16_t  iat_degc_x10;        // IAT °C × 10
    // o2_mv REMOVIDO — sistema usa exclusivamente WBO2 via CAN (ID 0x180)
    uint16_t fuel_press_kpa_x10;  // pressão combustível kPa × 10
    uint16_t oil_press_kpa_x10;   // pressão óleo kPa × 10
    uint16_t vbatt_mv;            // tensão bateria mV
    uint8_t  fault_bits;          // bitmask de falhas ativas
    // Expansão AN1-4 — passthrough, atualizados em sensors_tick_100ms()
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
void sensors_tick_50ms() noexcept;
void sensors_tick_100ms() noexcept;
void sensors_maf_freq_capture_isr(uint16_t period_ticks) noexcept;
void sensors_set_tps_cal(uint16_t raw_min, uint16_t raw_max) noexcept;
void sensors_set_range(SensorId id, SensorRange range) noexcept;
// FIX-6: retorna por valor (snapshot atômico com CPSID/CPSIE) em vez de
// referência. Previne torn read: ISR FTM3 (prio 1) pode atualizar g_data entre
// acessos sucessivos a campos diferentes via referência.
SensorData sensors_get() noexcept;

// CRITICAL FIX: Sensor validation functions
bool validate_sensor_range(SensorId id, uint16_t raw_value) noexcept;
bool validate_sensor_values(const SensorData& data) noexcept;
uint8_t get_sensor_health_status() noexcept;

#if defined(EMS_HOST_TEST)
void sensors_test_reset() noexcept;
void sensors_test_set_clt_table_entry(uint8_t idx, int16_t degc_x10) noexcept;
void sensors_test_set_iat_table_entry(uint8_t idx, int16_t degc_x10) noexcept;
#endif

}  // namespace ems::drv
