#pragma once

#include <cstdint>

namespace ems::engine {

constexpr uint8_t kTableAxisSize = 16u;

extern const uint16_t kRpmAxisX10[kTableAxisSize];
extern const uint16_t kLoadAxisKpa[kTableAxisSize];

uint8_t table_axis_index(const uint16_t* axis, uint8_t size, uint16_t value) noexcept;
uint8_t table_axis_frac_q8(const uint16_t* axis, uint8_t idx, uint16_t value) noexcept;

uint8_t table3d_lookup_u8(const uint8_t table[kTableAxisSize][kTableAxisSize],
                          const uint16_t* x_axis,
                          const uint16_t* y_axis,
                          uint16_t x,
                          uint16_t y) noexcept;

int16_t table3d_lookup_s16(const int16_t table[kTableAxisSize][kTableAxisSize],
                           const uint16_t* x_axis,
                           const uint16_t* y_axis,
                           uint16_t x,
                           uint16_t y) noexcept;

// Funções otimizadas para VE e Advance lookup com fixed-point arithmetic
uint16_t table3d_lookup_ve_q8(const uint8_t ve_table[kTableAxisSize][kTableAxisSize],
                             const uint16_t* x_axis,
                             const uint16_t* y_axis,
                             uint16_t x,
                             uint16_t y) noexcept;

int32_t table3d_lookup_advance_q10(const int16_t advance_table[kTableAxisSize][kTableAxisSize],
                                 const uint16_t* x_axis,
                                 const uint16_t* y_axis,
                                 uint16_t x,
                                 uint16_t y) noexcept;

}  // namespace ems::engine
