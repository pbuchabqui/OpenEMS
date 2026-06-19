#pragma once

#include <cstdint>

namespace ems::app {

struct DatalogEntry {
    uint32_t timestamp_ms;
    uint16_t rpm_x10;
    uint16_t map_x100;
    uint16_t tps_x10;
    int16_t  clt_x10;
    int16_t  iat_x10;
    uint16_t pw_us;
    uint16_t advance_x10;
    uint16_t lambda_x1000;
    uint16_t afr_target_x100;
    uint16_t ewg_pos;
    uint16_t flags;
};

static_assert(sizeof(DatalogEntry) == 28u, "DatalogEntry must be 28 bytes");

void datalog_init() noexcept;
void datalog_append(const DatalogEntry& entry) noexcept;
void datalog_flush() noexcept;
bool datalog_is_active() noexcept;
uint32_t datalog_dropped_count() noexcept;

}  // namespace ems::app
