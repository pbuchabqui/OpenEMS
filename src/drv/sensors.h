#pragma once

#include <cstdint>

#include "drv/ckp.h"

namespace ems::drv {

struct SensorData {
    uint16_t map_kpa_x10;
    uint16_t maf_gps_x100;
    uint16_t tps_pct_x10;
    int16_t clt_degc_x10;
    int16_t iat_degc_x10;
    uint16_t o2_mv;
    uint16_t fuel_press_kpa_x10;
    uint16_t oil_press_kpa_x10;
    uint16_t vbatt_mv;
    uint8_t fault_bits;
};

enum class SensorId : uint8_t {
    MAP = 0,
    MAF = 1,
    TPS = 2,
    CLT = 3,
    IAT = 4,
    O2 = 5,
    FUEL_PRESS = 6,
    OIL_PRESS = 7,
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

const SensorData& sensors_get() noexcept;

#if defined(EMS_HOST_TEST)
void sensors_test_reset() noexcept;
void sensors_test_set_clt_table_entry(uint8_t idx, int16_t degc_x10) noexcept;
void sensors_test_set_iat_table_entry(uint8_t idx, int16_t degc_x10) noexcept;
#endif

}  // namespace ems::drv
