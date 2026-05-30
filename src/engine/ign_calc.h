#pragma once

#include <cstdint>

#include "engine/engine_config.h"
#include "engine/table3d.h"

namespace ems::engine {

struct IgnScheduleParams {
    uint16_t dwell_start_x10;
    uint16_t spark_x10;
    uint8_t cyl;
};

struct InjScheduleParams {
    uint16_t soi_x10;
    uint32_t pw_ticks;
    uint8_t cyl;
};

struct AdvanceCorrections {
    int16_t iat_deg;
    int16_t clt_deg;
    int16_t knock_retard_deg;
};

int16_t get_advance(uint32_t rpm_x10, uint16_t load_bar_x100) noexcept;
int16_t get_advance_prepared(const Table2dLookup& lookup) noexcept;
int16_t clamp_advance_deg(int16_t advance_deg) noexcept;

int16_t calc_total_advance(int16_t base_advance_deg,
                           AdvanceCorrections corr) noexcept;
int16_t calc_idle_spark_correction_deg(uint32_t rpm_x10,
                                       uint16_t idle_target_rpm_x10,
                                       uint16_t tps_pct_x10,
                                       uint16_t map_bar_x100) noexcept;

uint16_t dwell_ms_x10_from_vbatt(uint16_t vbatt_mv) noexcept;

// Dwell com correcção 2D tensão × RPM (MS42 §2.2.2.2.1 IP_TD__VB__N_32).
// Usa dwell_ms_x10_from_vbatt() como base e aplica o factor RPM Q8.
uint16_t dwell_ms_x10_from_vbatt_rpm(uint16_t vbatt_mv, uint32_t rpm_x10) noexcept;
uint16_t calc_dwell_angle_x10(uint16_t dwell_ms_x10, uint16_t rpm) noexcept;
int32_t calc_dwell_start_deg_x10(int16_t spark_deg_x10,
                                 uint16_t dwell_ms_x10,
                                 uint16_t rpm) noexcept;

IgnScheduleParams build_ign_schedule(uint8_t cyl,
                                     int16_t spark_deg_x10,
                                     uint16_t dwell_ms_x10,
                                     uint16_t rpm) noexcept;

uint32_t inj_pw_us_to_scheduler_ticks(uint32_t pw_us) noexcept;

}  // namespace ems::engine
