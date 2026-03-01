#include "drv/ckp.h"

#include <cstdint>

#if __has_include("hal/ftm.h")
#include "hal/ftm.h"
#elif __has_include("ftm.h")
#include "ftm.h"
#endif

#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_ftm3_c0v = 0u;
volatile uint32_t ems_test_ftm3_c1v = 0u;
volatile uint32_t ems_test_gpiod_pdir = 0u;
#endif

namespace {

constexpr uint16_t kToothAngleX1000 = 6207u;
constexpr uint16_t kRealTeethPerRev = 58u;
constexpr uint16_t kGapThresholdToothCount = 55u;
constexpr uint16_t kMaxToothCountBeforeResync = 60u;

#if defined(EMS_HOST_TEST)
#define FTM3_C0V ems_test_ftm3_c0v
#define FTM3_C1V ems_test_ftm3_c1v
#define GPIOD_PDIR ems_test_gpiod_pdir
#else
#define FTM3_C0V (*reinterpret_cast<volatile uint32_t*>(0x400B9010u))
#define FTM3_C1V (*reinterpret_cast<volatile uint32_t*>(0x400B9018u))
#define GPIOD_PDIR (*reinterpret_cast<volatile uint32_t*>(0x400FF0C0u))
#endif

struct DecoderState {
    ems::drv::CkpSnapshot snap;
    uint16_t prev_capture;
    uint32_t tooth_hist[4];
    uint16_t tooth_count;
    uint8_t hist_ready;
    uint8_t gap_count;
    uint8_t cmp_confirms;
};

static DecoderState g_state = {
    ems::drv::CkpSnapshot{0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT, false},
    0u,
    {0u, 0u, 0u, 0u},
    0u,
    0u,
    0u,
    0u,
};

inline uint32_t period_ticks_to_ns(uint16_t ticks) noexcept {
    return (static_cast<uint32_t>(ticks) * 16667u) / 1000u;
}

inline uint32_t rpm_x10_from_period_ns(uint32_t period_ns) noexcept {
    if (period_ns == 0u) {
        return 0u;
    }
    return static_cast<uint32_t>(600000000000ULL /
                                 (static_cast<uint64_t>(kRealTeethPerRev) * period_ns));
}

inline void shift_hist(uint32_t period) noexcept {
    g_state.tooth_hist[3] = g_state.tooth_hist[2];
    g_state.tooth_hist[2] = g_state.tooth_hist[1];
    g_state.tooth_hist[1] = g_state.tooth_hist[0];
    g_state.tooth_hist[0] = period;
    if (g_state.hist_ready < 3u) {
        ++g_state.hist_ready;
    }
}

inline void transition_to_syncing() noexcept {
    g_state.snap.state = ems::drv::SyncState::SYNCING;
    g_state.gap_count = 0u;
    g_state.tooth_count = 0u;
    g_state.snap.tooth_index = 0u;
}

inline void handle_gap() noexcept {
    if (g_state.snap.state == ems::drv::SyncState::WAIT) {
        g_state.snap.state = ems::drv::SyncState::SYNCING;
        g_state.gap_count = 1u;
    } else if (g_state.snap.state == ems::drv::SyncState::SYNCING) {
        if (g_state.gap_count < 0xFFu) {
            ++g_state.gap_count;
        }
        if (g_state.gap_count >= 2u) {
            g_state.snap.state = ems::drv::SyncState::SYNCED;
        }
    }

    g_state.tooth_count = 0u;
    g_state.snap.tooth_index = 0u;
}

inline void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

inline void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

}  // namespace

namespace ems::drv {
#if defined(__GNUC__)
__attribute__((weak))
#endif
void sensors_on_tooth(const CkpSnapshot& snap) noexcept {
    static_cast<void>(snap);
}

#if defined(__GNUC__)
__attribute__((weak))
#endif
void schedule_on_tooth(const CkpSnapshot& snap) noexcept {
    static_cast<void>(snap);
}
}  // namespace ems::drv

namespace ems::drv {

CkpSnapshot ckp_snapshot() noexcept {
    CkpSnapshot out;
    enter_critical();
    out = g_state.snap;
    exit_critical();
    return out;
}

uint16_t ckp_angle_to_ticks(uint16_t angle_x10, uint16_t ref_capture) noexcept {
    // FTM3: 120 MHz system clock / prescaler 2 = 60 MHz efetivo = 16.667 ns/tick
    // ticks = ns / 16.667 = ns * 60 / 1000
    // Bug original: usava fator 120 (implicava 120 MHz sem prescaler) — erro 2×
    const uint32_t tooth_period_ticks = (g_state.snap.tooth_period_ns * 60u) / 1000u;
    const uint32_t delta = (static_cast<uint32_t>(angle_x10) * tooth_period_ticks) /
                           kToothAngleX1000;
    return static_cast<uint16_t>(ref_capture + static_cast<uint16_t>(delta));
}

void ckp_ftm3_ch0_isr() noexcept {
    if ((GPIOD_PDIR & (1u << 0u)) == 0u) {
        return;
    }

    const uint16_t capture_now = static_cast<uint16_t>(FTM3_C0V & 0xFFFFu);
    const uint16_t period = static_cast<uint16_t>(capture_now - g_state.prev_capture);
    g_state.prev_capture = capture_now;

    const uint32_t period_ns = period_ticks_to_ns(period);
    g_state.snap.last_ftm3_capture = capture_now;

    if (g_state.hist_ready < 3u) {
        shift_hist(period);
        ++g_state.tooth_count;
        g_state.snap.tooth_period_ns = period_ns;
        g_state.snap.rpm_x10 = rpm_x10_from_period_ns(period_ns);
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    const uint32_t avg = (g_state.tooth_hist[0] + g_state.tooth_hist[1] + g_state.tooth_hist[2]) / 3u;
    const bool gap_ok = (period > ((avg * 3u) / 2u));

    if (gap_ok && g_state.tooth_count >= kGapThresholdToothCount) {
        handle_gap();
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    if (!gap_ok) {
        shift_hist(period);
        ++g_state.tooth_count;

        g_state.snap.tooth_period_ns = period_ns;
        g_state.snap.rpm_x10 = rpm_x10_from_period_ns(period_ns);

        if (g_state.snap.state != SyncState::WAIT) {
            g_state.snap.tooth_index =
                (g_state.snap.tooth_index < (kRealTeethPerRev - 1u)) ?
                static_cast<uint16_t>(g_state.snap.tooth_index + 1u) : 0u;
        }

        if (g_state.tooth_count > kMaxToothCountBeforeResync) {
            transition_to_syncing();
        }
    }
    sensors_on_tooth(g_state.snap);
    schedule_on_tooth(g_state.snap);
}

void ckp_ftm3_ch1_isr() noexcept {
    if ((GPIOD_PDIR & (1u << 1u)) == 0u) {
        return;
    }

    static_cast<void>(FTM3_C1V);
    g_state.snap.phase_A = !g_state.snap.phase_A;
    if (g_state.cmp_confirms < 2u) {
        ++g_state.cmp_confirms;
    }
}

#if defined(EMS_HOST_TEST)
void ckp_test_reset() noexcept {
    g_state = DecoderState{
        CkpSnapshot{0u, 0u, 0u, 0u, SyncState::WAIT, false},
        0u,
        {0u, 0u, 0u, 0u},
        0u,
        0u,
        0u,
        0u,
    };
    ems_test_ftm3_c0v = 0u;
    ems_test_ftm3_c1v = 0u;
    ems_test_gpiod_pdir = 0u;
}

uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept {
    return rpm_x10_from_period_ns(period_ns);
}
#endif

}  // namespace ems::drv
