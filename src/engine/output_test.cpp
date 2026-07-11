#include "engine/output_test.h"

#include "drv/ckp.h"
#include "engine/auxiliaries.h"
#include "engine/ecu_sched.h"
#include "hal/etb_driver.h"
#include "hal/ewg_driver.h"
#include "hal/timer.h"

namespace {

constexpr uint32_t kKeepaliveTimeoutMs = 5000u;
constexpr uint32_t kPulseGapMs = 100u;   // um pulso de cada vez
constexpr int16_t  kEtbPwmClamp = 307;   // ~30% de 1023
constexpr int16_t  kEwgPwmClamp = 300;   // 30% de 1000

bool     g_active = false;
uint8_t  g_abort_reason = ems::engine::kOutputTestAbortNone;
uint32_t g_deadline_ms = 0u;
uint32_t g_busy_until_ms = 0u;
uint32_t g_now_ms = 0u;

void restore_safe() noexcept
{
    ::ecu_sched_test_all_outputs_safe();
    ems::engine::auxiliaries_force_pump(false);
    ems::engine::auxiliaries_force_fan(false);
    ems::hal::tim4_set_duty(0u, 0u);
    ems::hal::tim4_set_duty(1u, 0u);
    ::etb_driver_shutdown();
    ems::hal::ewg_driver_shutdown();
    g_active = false;
}

inline void refresh_keepalive() noexcept
{
    g_deadline_ms = g_now_ms + kKeepaliveTimeoutMs;
}

// Guard comum: activo e (para pulsos) fora da janela busy.
inline bool cmd_allowed(bool is_pulse) noexcept
{
    if (!g_active) { return false; }
    if (is_pulse && static_cast<int32_t>(g_now_ms - g_busy_until_ms) < 0) {
        return false;
    }
    refresh_keepalive();
    return true;
}

}  // namespace

namespace ems::engine {

bool output_test_enter() noexcept
{
    if (ems::drv::ckp_snapshot().rpm_x10 != 0u) { return false; }
    g_active = true;
    g_abort_reason = kOutputTestAbortNone;
    g_busy_until_ms = g_now_ms;
    refresh_keepalive();
    return true;
}

void output_test_exit() noexcept
{
    if (!g_active) { return; }
    restore_safe();
}

bool output_test_active() noexcept { return g_active; }

void output_test_keepalive() noexcept
{
    if (!g_active) { return; }
    refresh_keepalive();
}

void output_test_poll(uint32_t now_ms, uint32_t rpm_x10) noexcept
{
    g_now_ms = now_ms;
    if (!g_active) { return; }
    if (rpm_x10 > 0u) {
        g_abort_reason = kOutputTestAbortRpm;
        restore_safe();
        return;
    }
    if (static_cast<int32_t>(now_ms - g_deadline_ms) > 0) {
        g_abort_reason = kOutputTestAbortTimeout;
        restore_safe();
    }
}

bool output_test_fire_injector(uint8_t cyl, uint16_t pw_us) noexcept
{
    if (cyl > 3u || pw_us == 0u) { return false; }
    if (!cmd_allowed(true)) { return false; }
    if (pw_us > 30000u) { pw_us = 30000u; }
    ::ecu_sched_test_pulse_inj(cyl, pw_us);
    g_busy_until_ms = g_now_ms + (pw_us / 1000u) + kPulseGapMs;
    return true;
}

bool output_test_fire_coil(uint8_t cyl, uint16_t dwell_us) noexcept
{
    if (cyl > 3u || dwell_us == 0u) { return false; }
    if (!cmd_allowed(true)) { return false; }
    if (dwell_us > 10000u) { dwell_us = 10000u; }
    ::ecu_sched_test_pulse_ign(cyl, dwell_us);
    g_busy_until_ms = g_now_ms + (dwell_us / 1000u) + kPulseGapMs;
    return true;
}

bool output_test_set_pump(bool on) noexcept
{
    if (!cmd_allowed(false)) { return false; }
    auxiliaries_force_pump(on);
    return true;
}

bool output_test_set_fan(bool on) noexcept
{
    if (!cmd_allowed(false)) { return false; }
    auxiliaries_force_fan(on);
    return true;
}

bool output_test_set_vvt(uint8_t ch, uint16_t duty_pct_x10) noexcept
{
    if (ch > 1u) { return false; }
    if (!cmd_allowed(false)) { return false; }
    if (duty_pct_x10 > 1000u) { duty_pct_x10 = 1000u; }
    ems::hal::tim4_set_duty(ch, duty_pct_x10);
    return true;
}

bool output_test_set_etb(int16_t pwm) noexcept
{
    if (!cmd_allowed(false)) { return false; }
    if (pwm > kEtbPwmClamp)  { pwm = kEtbPwmClamp; }
    if (pwm < -kEtbPwmClamp) { pwm = -kEtbPwmClamp; }
    return ::etb_driver_set_motor_pwm(pwm);  // false se driver não-READY
}

bool output_test_set_ewg(int16_t pwm) noexcept
{
    if (!cmd_allowed(false)) { return false; }
    if (pwm > kEwgPwmClamp)  { pwm = kEwgPwmClamp; }
    if (pwm < -kEwgPwmClamp) { pwm = -kEwgPwmClamp; }
    ems::hal::ewg_driver_set_motor_pwm(pwm);
    return true;
}

void output_test_status(uint8_t out[4]) noexcept
{
    out[0] = g_active ? 1u : 0u;
    out[1] = g_abort_reason;
    if (g_active) {
        const int32_t remain_ms = static_cast<int32_t>(g_deadline_ms - g_now_ms);
        out[2] = (remain_ms <= 0) ? 0u
               : static_cast<uint8_t>((remain_ms > 255000) ? 255 : remain_ms / 1000);
    } else {
        out[2] = 0u;
    }
    out[3] = (g_active && static_cast<int32_t>(g_now_ms - g_busy_until_ms) < 0) ? 1u : 0u;
}

#if defined(EMS_HOST_TEST)
void output_test_test_reset() noexcept
{
    g_active = false;
    g_abort_reason = kOutputTestAbortNone;
    g_deadline_ms = 0u;
    g_busy_until_ms = 0u;
    g_now_ms = 0u;
}
#endif

}  // namespace ems::engine
