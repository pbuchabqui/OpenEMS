#include "engine/transient_fuel.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"
#include "engine/xtau_autocalib.h"

#include <cstdint>

namespace {

using ems::engine::interp_u16_8pt;

constexpr uint32_t kMaxPwUs = 100000u;
int32_t g_wall_fuel_us_q8 = 0;

}  // namespace

namespace ems::engine {

void transient_fuel_reset() noexcept {
    g_wall_fuel_us_q8 = 0;
    // Produção usa g_wall_state em xtau_autocalib — limpar ambos evita filme fantasma no DFCO.
    xtau_wall_fuel_reset();
}

uint32_t transient_fuel_xtau_update(uint32_t fuel_pw_us,
                                    int16_t clt_x10,
                                    bool enabled) noexcept {
    if (!enabled || fuel_pw_us == 0u) {
        transient_fuel_reset();
        return fuel_pw_us;
    }

    uint16_t x_q8 = interp_u16_8pt(xtau_clt_axis_x10, xtau_x_fraction_q8, kCorrectionTableSize, clt_x10);
    if (x_q8 > 192u) {
        x_q8 = 192u;
    }

    uint16_t tau = interp_u16_8pt(xtau_clt_axis_x10, xtau_tau_cycles, kCorrectionTableSize, clt_x10);
    if (tau == 0u) {
        tau = 1u;
    }
    if (tau > 255u) {
        tau = 255u;
    }

    const uint32_t clamped_pw = fuel_pw_us > kMaxPwUs ? kMaxPwUs : fuel_pw_us;
    const int32_t desired_q8 = static_cast<int32_t>(clamped_pw << 8u);
    const int32_t evap_q8 = g_wall_fuel_us_q8 / static_cast<int32_t>(tau);
    int32_t numerator_q8 = desired_q8 - evap_q8;
    if (numerator_q8 < 0) {
        numerator_q8 = 0;
    }

    const int32_t dry_fraction_q8 = 256 - static_cast<int32_t>(x_q8);
    // numerator_q8 chega a ~25.6M (kMaxPwUs<<8); ×256 estoura int32_t.
    // Intermediário de 64 bits evita overflow; o resultado pós-clamp cabe em int32_t.
    int32_t injected_q8 = static_cast<int32_t>(
        (static_cast<int64_t>(numerator_q8) * 256) / dry_fraction_q8);
    const int32_t max_q8 = static_cast<int32_t>(kMaxPwUs << 8u);
    if (injected_q8 > max_q8) {
        injected_q8 = max_q8;
    }

    g_wall_fuel_us_q8 += ((injected_q8 * static_cast<int32_t>(x_q8)) >> 8) - evap_q8;
    if (g_wall_fuel_us_q8 < 0) {
        g_wall_fuel_us_q8 = 0;
    }
    if (g_wall_fuel_us_q8 > max_q8) {
        g_wall_fuel_us_q8 = max_q8;
    }

    return static_cast<uint32_t>(injected_q8 >> 8);
}

}  // namespace ems::engine
