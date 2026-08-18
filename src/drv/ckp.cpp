/**
 * @file drv/ckp.cpp
 * @brief Encoder crank snapshot + stall poll. No 60-2 decoder.
 *
 * Sync state is published by ecu_sched_encoder_heartbeat_tick() via
 * ckp_publish_encoder_snapshot(). TIM5 is only a freerun timebase.
 */

#include "drv/ckp.h"

#include "drv/sensors.h"
#include "hal/critical_section.h"

#include <cstring>

#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_tim5_ccr1 = 0u;
volatile uint32_t ems_test_tim5_ccr2 = 0u;
volatile uint32_t ems_test_cam_gpio_idr = 0u;
#endif

namespace ems::drv {

namespace {

struct DecoderState {
    CkpSnapshot snap;
};

DecoderState g_state{};

static constexpr uint32_t kMinStallTimeoutTicksBench = 130000000u;  // 2.08 s
static constexpr uint32_t kEncoderStallTimeoutTicksProd = 50000000u;  // 800 ms

inline uint32_t min_stall_timeout_ticks_encoder() noexcept
{
    return sensors_is_bench_mode() ? kMinStallTimeoutTicksBench
                                   : kEncoderStallTimeoutTicksProd;
}

}  // namespace

volatile uint32_t g_diag_tn1 = 0u;
volatile uint32_t g_diag_tn2 = 0u;
volatile uint32_t g_diag_delta = 0u;
volatile uint32_t g_diag_isr_count = 0u;
volatile uint32_t g_diag_hist_ready = 0u;
volatile uint32_t g_diag_tooth_count = 0u;
volatile uint32_t g_diag_consec_anom = 0u;
volatile uint32_t g_dbg_tc_gap = 0u;
volatile uint32_t g_dbg_tc_spike = 0u;
volatile uint32_t g_dbg_tc_normal = 0u;
volatile uint32_t g_diag_cmp_isr_count = 0u;
volatile uint32_t g_diag_last_ckp_edge_tick = 0u;
volatile uint32_t g_diag_last_cmp_edge_tick = 0u;
volatile uint32_t g_dbg_gap_accepted = 0u;
volatile uint32_t g_dbg_gap_premature = 0u;
volatile uint32_t g_dbg_gap_last_tc = 0u;
volatile uint32_t g_dbg_loss_missing_gap = 0u;
volatile uint32_t g_dbg_loss_stall = 0u;
volatile uint32_t g_dbg_loss_avg = 0u;
volatile uint32_t g_dbg_loss_delta = 0u;
volatile uint32_t g_dbg_loss_histogram = 0u;
volatile uint32_t g_dbg_loss_wrap = 0u;
volatile uint32_t g_dbg_loss_hist_mn = 0u;
volatile uint32_t g_dbg_loss_hist_mx = 0u;
volatile uint32_t g_dbg_skip_after_silence = 0u;
volatile uint32_t g_scope_ckp_ts[64] = {};
volatile uint8_t  g_scope_ckp_idx = 0u;
volatile uint32_t g_scope_cmp_ts[8] = {};
volatile uint8_t  g_scope_cmp_idx = 0u;

CkpSnapshot ckp_snapshot() noexcept
{
    CkpSnapshot out;
    ems::hal::CriticalSectionGuard guard;
    std::memcpy(&out, &g_state.snap, sizeof(out));
    return out;
}

void ckp_publish_encoder_snapshot(const CkpSnapshot& snap) noexcept
{
    ems::hal::CriticalSectionGuard guard;
    std::memcpy(&g_state.snap, &snap, sizeof(g_state.snap));
}

bool ckp_stall_poll_encoder(uint32_t tim5_cnt_now) noexcept
{
    const int32_t elapsed_signed =
        static_cast<int32_t>(tim5_cnt_now - g_state.snap.last_tim5_capture);
    const uint32_t elapsed_ticks =
        (elapsed_signed < 0) ? 0u : static_cast<uint32_t>(elapsed_signed);
    if (elapsed_ticks < min_stall_timeout_ticks_encoder()) {
        return false;
    }
    ems::hal::CriticalSectionGuard guard;
    const int32_t elapsed_now =
        static_cast<int32_t>(tim5_cnt_now - g_state.snap.last_tim5_capture);
    const bool still_stalled = elapsed_now >= 0 &&
        static_cast<uint32_t>(elapsed_now) >= min_stall_timeout_ticks_encoder();
    bool transitioned = false;
    if (still_stalled && g_state.snap.rpm_x10 != 0u) {
        ++g_dbg_loss_stall;
        g_state.snap.state = SyncState::LOSS_OF_SYNC;
        g_state.snap.rpm_x10 = 0u;
        transitioned = true;
    }
    return transitioned;
}

uint32_t ckp_get_cmp_glitch_count() noexcept { return 0u; }
uint8_t  ckp_get_cmp_ref_tooth() noexcept { return 0xFFu; }
uint32_t ckp_instant_rpm_x10() noexcept { return g_state.snap.rpm_x10; }

void ckp_tim5_ch1_isr() noexcept {}
void ckp_tim5_ch2_isr() noexcept {}
bool ckp_stall_poll(uint32_t tim5_cnt_now) noexcept
{
    return ckp_stall_poll_encoder(tim5_cnt_now);
}

#if defined(EMS_HOST_TEST)
void ckp_test_reset() noexcept
{
    g_state = DecoderState{};
    ems_test_tim5_ccr1 = 0u;
    ems_test_tim5_ccr2 = 0u;
    ems_test_cam_gpio_idr = 0u;
}

uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept
{
    if (period_ns == 0u) { return 0u; }
    // Encoder: one rev, not one 60-2 tooth. Same 600e9 / period_ns.
    return static_cast<uint32_t>(600000000000ULL / period_ns);
}

void ckp_test_set_cmp_confirms(uint8_t n) noexcept
{
    g_state.snap.cmp_confirms = n;
}
#endif

}  // namespace ems::drv
