#pragma once

#include <cstdint>

namespace ems::drv {

// Bit mask para fault_bits
static constexpr uint8_t FAULT_WBO2_TIMEOUT = (1u << 0);

struct SensorData {
    uint16_t map_kpa_x10;       // MAP kPa × 10
    uint32_t maf_gps_x100;      // MAF g/s × 100
    uint16_t tps_pct_x10;       // TPS % × 10
    int16_t  clt_degc_x10;      // CLT °C × 10
    int16_t  iat_degc_x10;      // IAT °C × 10
    // o2_mv REMOVIDO — sistema usa exclusivamente WBO2 via CAN (ID 0x180)
    uint16_t fuel_press_kpa_x10;// pressão combustível kPa × 10
    uint16_t oil_press_kpa_x10; // pressão óleo kPa × 10
    uint16_t vbatt_mv;          // tensão bateria mV
    uint8_t  fault_bits;        // bitmask de falhas ativas
};

} // namespace ems::drv
