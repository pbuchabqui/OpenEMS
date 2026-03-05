#pragma once

#include <cstdint>

namespace ems::engine {

struct QuickCrankOutput {
    bool cranking;
    bool afterstart_active;
    uint16_t fuel_mult_x256;
    int16_t spark_deg;
    uint32_t min_pw_us;
};

void quick_crank_reset() noexcept;

QuickCrankOutput quick_crank_update(uint32_t now_ms,
                                    uint32_t rpm_x10,
                                    bool full_sync,
                                    int16_t clt_x10,
                                    int16_t base_spark_deg) noexcept;

uint32_t quick_crank_apply_pw_us(uint32_t base_pw_us,
                                 uint16_t fuel_mult_x256,
                                 uint32_t min_pw_us) noexcept;

}  // namespace ems::engine
