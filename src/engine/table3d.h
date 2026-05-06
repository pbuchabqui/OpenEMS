#pragma once

#include <cstdint>

namespace ems::engine {

constexpr uint8_t kTableAxisSize = 16u;

extern const uint32_t kRpmAxisX10[kTableAxisSize];
extern const uint32_t kLoadAxisKpa[kTableAxisSize];

struct Table2dLookup {
    uint8_t xi;
    uint8_t yi;
    uint8_t fx_q8;
    uint8_t fy_q8;
};

uint8_t table_axis_index(const uint32_t* axis, uint8_t size, uint32_t value) noexcept;
uint8_t table_axis_frac_q8(const uint32_t* axis, uint8_t idx, uint32_t value) noexcept;

Table2dLookup table3d_prepare_lookup(const uint32_t* x_axis,
                                     const uint32_t* y_axis,
                                     uint32_t x,
                                     uint32_t y) noexcept;

uint8_t table3d_lookup_u8_prepared(const uint8_t table[kTableAxisSize][kTableAxisSize],
                                   const Table2dLookup& lookup) noexcept;

int16_t table3d_lookup_i8_prepared(const int8_t table[kTableAxisSize][kTableAxisSize],
                                   const Table2dLookup& lookup) noexcept;

int16_t table3d_lookup_s16_prepared(const int16_t table[kTableAxisSize][kTableAxisSize],
                                    const Table2dLookup& lookup) noexcept;

uint8_t table3d_lookup_u8(const uint8_t table[kTableAxisSize][kTableAxisSize],
                          const uint32_t* x_axis,
                          const uint32_t* y_axis,
                          uint32_t x,
                          uint32_t y) noexcept;

int16_t table3d_lookup_s16(const int16_t table[kTableAxisSize][kTableAxisSize],
                           const uint32_t* x_axis,
                           const uint32_t* y_axis,
                           uint32_t x,
                           uint32_t y) noexcept;

// Funções otimizadas para VE e Advance lookup com fixed-point arithmetic
uint16_t table3d_lookup_ve_q8(const uint8_t ve_table[kTableAxisSize][kTableAxisSize],
                             const uint32_t* x_axis,
                             const uint32_t* y_axis,
                             uint32_t x,
                             uint32_t y) noexcept;

int32_t table3d_lookup_advance_q10(const int16_t advance_table[kTableAxisSize][kTableAxisSize],
                                 const uint32_t* x_axis,
                                 const uint32_t* y_axis,
                                 uint32_t x,
                                 uint32_t y) noexcept;

}  // namespace ems::engine
