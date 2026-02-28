#include "engine/knock.h"

#include <cstdint>

namespace {

constexpr uint8_t kDefaultEventThreshold = 3u;
constexpr uint16_t kRetardStepX10 = 20u;      // +2.0 deg por evento acima do limiar
constexpr uint16_t kRetardMaxX10 = 100u;      // 10.0 deg
constexpr uint16_t kRecoveryStepX10 = 1u;     // -0.1 deg por ciclo limpo
constexpr uint8_t kRecoveryDelayCycles = 10u; // inicia recovery apos 10 ciclos limpos
constexpr uint8_t kVoselMax = 63u;            // CMP0_DACCR[VOSEL] 6-bit
constexpr uint8_t kVoselDefault = 32u;

constexpr uint8_t kCmpCr1En = (1u << 0u);
constexpr uint8_t kCmpDacVoselMask = 0x3Fu;
constexpr uint8_t kCmpDacDacen = (1u << 7u);

#if defined(EMS_HOST_TEST)
volatile uint8_t ems_test_cmp0_cr1 = 0u;
volatile uint8_t ems_test_cmp0_daccr = 0u;
#define CMP0_CR1 ems_test_cmp0_cr1
#define CMP0_DACCR ems_test_cmp0_daccr
#else
#define CMP0_CR1 (*reinterpret_cast<volatile uint8_t*>(0x40073001u))
#define CMP0_DACCR (*reinterpret_cast<volatile uint8_t*>(0x40073008u))
#endif

struct KnockState {
    uint8_t knock_count[ems::engine::kKnockCylinders];
    uint8_t clean_cycles[ems::engine::kKnockCylinders];

    uint8_t event_threshold;
    uint8_t vosel;
    uint8_t global_clean_cycles;

    uint8_t window_cyl;
    bool window_active;
};

static KnockState g = {};

uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void cmp0_set_enabled(bool enable) noexcept {
    if (enable) {
        CMP0_CR1 = static_cast<uint8_t>(CMP0_CR1 | kCmpCr1En);
    } else {
        CMP0_CR1 = static_cast<uint8_t>(CMP0_CR1 & static_cast<uint8_t>(~kCmpCr1En));
    }
}

void cmp0_set_vosel(uint8_t vosel) noexcept {
    const uint8_t v = static_cast<uint8_t>(vosel & kCmpDacVoselMask);
    g.vosel = v;
    CMP0_DACCR = static_cast<uint8_t>(kCmpDacDacen | v);
}

}  // namespace

namespace ems::engine {

volatile uint16_t knock_retard_x10[kKnockCylinders] = {};

void knock_init() noexcept {
    g = {};
    g.event_threshold = kDefaultEventThreshold;
    cmp0_set_vosel(kVoselDefault);
    cmp0_set_enabled(false);

    for (uint8_t i = 0u; i < kKnockCylinders; ++i) {
        knock_retard_x10[i] = 0u;
    }
}

void knock_set_event_threshold(uint8_t threshold) noexcept {
    g.event_threshold = threshold;
}

void knock_window_open(uint8_t cyl) noexcept {
    g.window_cyl = static_cast<uint8_t>(cyl & 0x3u);
    g.window_active = true;
    cmp0_set_enabled(true);
}

void knock_window_close(uint8_t cyl) noexcept {
    if ((g.window_cyl == static_cast<uint8_t>(cyl & 0x3u)) && g.window_active) {
        g.window_active = false;
        cmp0_set_enabled(false);
    }
}

void knock_cmp0_isr() noexcept {
    if (!g.window_active) {
        return;
    }

    const uint8_t cyl = g.window_cyl;
    if (g.knock_count[cyl] < 255u) {
        ++g.knock_count[cyl];
    }
}

void knock_cycle_complete(uint8_t cyl) noexcept {
    const uint8_t c = static_cast<uint8_t>(cyl & 0x3u);
    const uint8_t count = g.knock_count[c];

    if (count > g.event_threshold) {
        const uint16_t next = static_cast<uint16_t>(knock_retard_x10[c] + kRetardStepX10);
        knock_retard_x10[c] = clamp_u16(next, 0u, kRetardMaxX10);
        g.clean_cycles[c] = 0u;
        g.global_clean_cycles = 0u;

        if (g.vosel >= 2u) {
            cmp0_set_vosel(static_cast<uint8_t>(g.vosel - 2u));
        } else {
            cmp0_set_vosel(0u);
        }
    } else {
        if (g.clean_cycles[c] < 255u) {
            ++g.clean_cycles[c];
        }
        if ((g.clean_cycles[c] >= kRecoveryDelayCycles) && (knock_retard_x10[c] > 0u)) {
            knock_retard_x10[c] = static_cast<uint16_t>(knock_retard_x10[c] - kRecoveryStepX10);
        }

        if (g.global_clean_cycles < 255u) {
            ++g.global_clean_cycles;
        }
        if (g.global_clean_cycles >= 100u) {
            if (g.vosel < kVoselMax) {
                cmp0_set_vosel(static_cast<uint8_t>(g.vosel + 1u));
            }
            g.global_clean_cycles = 0u;
        }
    }

    g.knock_count[c] = 0u;
}

uint16_t knock_get_retard_x10(uint8_t cyl) noexcept {
    return knock_retard_x10[static_cast<uint8_t>(cyl & 0x3u)];
}

uint8_t knock_get_vosel() noexcept {
    return g.vosel;
}

#if defined(EMS_HOST_TEST)
uint8_t knock_test_get_knock_count(uint8_t cyl) noexcept {
    return g.knock_count[static_cast<uint8_t>(cyl & 0x3u)];
}

bool knock_test_window_active() noexcept {
    return g.window_active;
}

uint8_t knock_test_window_cyl() noexcept {
    return g.window_cyl;
}
#endif

}  // namespace ems::engine
