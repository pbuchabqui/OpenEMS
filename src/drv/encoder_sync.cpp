#include "drv/encoder_sync.h"

namespace ems::drv::encoder_sync {

CmpEdgeResult evaluate_cmp_edge(uint32_t cmp_angle_now, bool has_prev,
                                uint32_t prev_cmp_angle,
                                uint8_t reject_streak_in) noexcept
{
    CmpEdgeResult r{};

    if (!has_prev) {
        // Primeiro flanco após boot/perda: só arma a referência, mesma
        // semântica do CKP (ckp.cpp: "arm timestamp only — no phase/confirm
        // until a second edge passes the temporal window").
        r.accepted      = true;
        r.multiple      = 0u;
        r.reject_streak = 0u;
        r.streak_resync = false;
        return r;
    }

    // Wrap-safe: aritmética unsigned de 32 bits, mesmo padrão já usado em
    // todo o resto do fork para deltas de TIM2/TIM5.
    const uint32_t delta = cmp_angle_now - prev_cmp_angle;
    const uint32_t n = (delta + kCmpSpanCounts / 2u) / kCmpSpanCounts;

    bool within_tolerance = false;
    if (n >= 1u && n <= kCmpMaxAcceptedMultiple) {
        const int32_t expected  = static_cast<int32_t>(n * kCmpSpanCounts);
        const int32_t remainder = static_cast<int32_t>(delta) - expected;
        within_tolerance = (remainder >= -static_cast<int32_t>(kCmpSpanToleranceCounts)) &&
                           (remainder <=  static_cast<int32_t>(kCmpSpanToleranceCounts));
    }

    if (within_tolerance) {
        r.accepted      = true;
        r.multiple      = static_cast<uint8_t>(n);
        r.reject_streak = 0u;
        r.streak_resync = false;
        return r;
    }

    r.accepted = false;
    r.multiple = 0u;
    const uint8_t streak = static_cast<uint8_t>(reject_streak_in + 1u);
    if (streak >= kCmpRejectResyncThreshold) {
        r.reject_streak = 0u;
        r.streak_resync = true;
    } else {
        r.reject_streak = streak;
        r.streak_resync = false;
    }
    return r;
}

bool staleness_exceeded(uint32_t heartbeats_since_accepted, bool bench_mode) noexcept
{
    const uint32_t limit = bench_mode ? kMaxHeartbeatsWithoutCmpBench
                                       : kMaxHeartbeatsWithoutCmp;
    return heartbeats_since_accepted >= limit;
}

}  // namespace ems::drv::encoder_sync
