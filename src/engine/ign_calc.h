#pragma once

#include <cstdint>

#include "engine/table3d.h"

namespace ems::engine {

extern int8_t spark_table[kTableAxisSize][kTableAxisSize];

// CYC-02: ordem de disparo e ângulos TDC são constantes de calibração do motor —
// nunca devem ser modificados em runtime. Definidos inline aqui para garantir
// que qualquer TU que inclua este header tenha a definição completa.
// (extern constexpr não é válido em C++17; inline constexpr é a forma correcta.)
inline constexpr uint8_t  firing_order[4]        = {1u, 3u, 4u, 2u};
inline constexpr uint16_t cylinder_offset_deg[4] = {0u, 180u, 360u, 540u};

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

int16_t get_advance(uint16_t rpm_x10, uint16_t load_kpa) noexcept;
int16_t clamp_advance_deg(int16_t advance_deg) noexcept;

int16_t calc_total_advance(int16_t base_advance_deg,
                           int16_t corr_iat_deg,
                           int16_t corr_clt_deg,
                           int16_t knock_retard_deg) noexcept;

uint16_t dwell_ms_x10_from_vbatt(uint16_t vbatt_mv) noexcept;
uint16_t calc_dwell_angle_x10(uint16_t dwell_ms_x10, uint16_t rpm) noexcept;
int32_t calc_dwell_start_deg_x10(int16_t spark_deg_x10,
                                 uint16_t dwell_ms_x10,
                                 uint16_t rpm) noexcept;

IgnScheduleParams build_ign_schedule(uint8_t cyl,
                                     int16_t spark_deg_x10,
                                     uint16_t dwell_ms_x10,
                                     uint16_t rpm) noexcept;

uint32_t inj_pw_us_to_ftm0_ticks(uint32_t pw_us) noexcept;

}  // namespace ems::engine
