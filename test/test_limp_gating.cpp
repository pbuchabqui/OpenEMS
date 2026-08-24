/**
 * @file test_limp_gating.cpp
 * @brief FOME test_limp.cpp equivalents against limp_gating_update.
 */
#include "test/harness.h"

#include "drv/ckp.h"
#include "engine/calibration.h"
#include "engine/constants.h"
#include "engine/cut_reason.h"
#include "engine/ecu_sched.h"
#include "engine/limp_gating.h"
#include "hal/out_pins.h"

using ems::engine::LimpGatingInputs;
using ems::engine::limp_gating_update;
using ems::engine::limp_gating_reset;
using ems::engine::limp_gating_fatal;
using ems::engine::limp_gating_report_etb_problem;

namespace {

LimpGatingInputs base_in() noexcept
{
    LimpGatingInputs in{};
    in.rpm_x10 = 20000u;
    in.map_bar_x100 = 80u;
    in.clt_degc_x10 = 800;
    in.oil_press_bar_x1000 = 3000u;
    in.lambda_x1000 = 1000u;
    in.lambda_valid = true;
    in.lambda_target_x1000 = 1000u;
    in.tps_pct_x10 = 100u;
    in.full_sync = true;
    in.phase_valid = true;
    in.sequential = true;
    in.now_ms = 1000u;
    return in;
}

}  // namespace

void test_limp_gating_fatal(void)
{
    section("limp_gating: fatal cuts inj+ign+etb");
    ecu_sched_test_reset();
    limp_gating_reset();
    CHECK_TRUE(ems::engine::limp_gating_allow_injection(), "inj allowed");
    CHECK_TRUE(ems::engine::limp_gating_allow_ignition(), "ign allowed");
    CHECK_TRUE(ems::engine::limp_gating_allow_etb(), "etb allowed");
    limp_gating_fatal();
    CHECK_FALSE(ems::engine::limp_gating_allow_injection(), "fatal inj cut");
    CHECK_FALSE(ems::engine::limp_gating_allow_ignition(), "fatal ign cut");
    CHECK_FALSE(ems::engine::limp_gating_allow_etb(), "fatal etb cut");
    CHECK_EQ(ecu_sched_get_inj_inhibit_mask(), 0x0Fu, "fatal inj mask 0x0F");
    CHECK_EQ(ecu_sched_get_ign_inhibit_mask(), 0x0Fu, "fatal ign mask 0x0F");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutFatal) != 0u,
               "kFuelCutFatal");
}

void test_limp_gating_rev_limit(void)
{
    section("limp_gating: hard rev limit + hysteresis (fuel only)");
    ecu_sched_test_reset();
    limp_gating_reset();
    ems::engine::rev_limit_rpm_x10 = 25000u;
    ems::engine::rev_limit_soft_window_x10 = 5000u;

    LimpGatingInputs in = base_in();
    in.rpm_x10 = 20000u;
    auto r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "under limit: inj");
    CHECK_TRUE(r.allow_ignition, "under limit: ign");

    in.rpm_x10 = 26000u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "over hard: fuel cut");
    CHECK_TRUE(r.allow_ignition, "over hard: spark stays (FOME default)");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutRevLimit) != 0u,
               "kFuelCutRevLimit");

    in.rpm_x10 = 22000u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "hysteresis: still cut at 2200");

    in.rpm_x10 = 20000u;
    r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "resume at hard-hyst");
}

void test_limp_gating_boost_cut(void)
{
    section("limp_gating: boost cut fuel-only + hysteresis");
    ecu_sched_test_reset();
    limp_gating_reset();
    ems::engine::boost_cut_map_bar_x100 = 100u;

    LimpGatingInputs in = base_in();
    in.map_bar_x100 = 80u;
    auto r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "MAP 80 kPa: inj");

    in.map_bar_x100 = 105u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "MAP 105: fuel cut");
    CHECK_TRUE(r.allow_ignition, "boost cut does not cut spark (bends valves)");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutBoost) != 0u,
               "kFuelCutBoost");

    in.map_bar_x100 = 95u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "95 kPa still cut (20 kPa hyst)");

    in.map_bar_x100 = 70u;
    r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "70 kPa resume");
}

void test_limp_gating_oil_after_start(void)
{
    section("limp_gating: oil after-start 5 s");
    ecu_sched_test_reset();
    limp_gating_reset();
    ems::engine::oil_min_after_start_bar_x1000 = 1500u;

    LimpGatingInputs in = base_in();
    in.cranking = false;
    in.rpm_x10 = 15000u;
    in.oil_press_bar_x1000 = 200u;
    in.now_ms = 1000u;
    auto r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "inside 5 s window: inj");

    in.now_ms = 7000u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "5 s without min oil: fuel cut");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutOilPress) != 0u,
               "kFuelCutOilPress after-start");

    in.rpm_x10 = 0u;
    r = limp_gating_update(in);
    in.rpm_x10 = 15000u;
    in.oil_press_bar_x1000 = 2000u;
    in.now_ms = 8000u;
    r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "stall reset + oil present: inj");
}

void test_limp_gating_lambda(void)
{
    section("limp_gating: lambda protection timeout + recover");
    ecu_sched_test_reset();
    limp_gating_reset();
    ems::engine::lambda_protect_timeout_ms = 200u;
    ems::engine::lambda_protect_dev_x1000 = 200u;
    ems::engine::lambda_protect_min_rpm_x10 = 10000u;
    ems::engine::lambda_protect_min_load_bar_x100 = 50u;

    LimpGatingInputs in = base_in();
    in.rpm_x10 = 20000u;
    in.map_bar_x100 = 80u;
    in.tps_pct_x10 = 500u;
    in.lambda_target_x1000 = 1000u;
    in.lambda_x1000 = 1400u;
    in.lambda_valid = true;
    in.now_ms = 0u;
    auto r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "lean just started: no cut yet");

    in.now_ms = 250u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "lean past timeout: fuel cut");
    CHECK_TRUE(r.allow_ignition, "lambda is fuel-only");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutLambda) != 0u,
               "kFuelCutLambda");

    in.lambda_x1000 = 1000u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "good AFR at WOT does not lift (FOME restore band)");
    in.rpm_x10 = 5000u;
    in.tps_pct_x10 = 50u;
    r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "lambda restored after lift");
}

void test_limp_gating_etb_problem(void)
{
    section("limp_gating: ETB problem → 1500 rpm fuel cut");
    ecu_sched_test_reset();
    limp_gating_reset();
    limp_gating_report_etb_problem();
    CHECK_FALSE(ems::engine::limp_gating_allow_etb(), "ETB disabled");

    LimpGatingInputs in = base_in();
    in.rpm_x10 = 10000u;
    auto r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "below 1500: inj");

    in.rpm_x10 = 20000u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "above 1500: fuel cut");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutEtbFault) != 0u,
               "kFuelCutEtbFault");
}

void test_limp_gating_flood_and_phase(void)
{
    section("limp_gating: flood clear + engine phase");
    ecu_sched_test_reset();
    limp_gating_reset();

    LimpGatingInputs in = base_in();
    in.cranking = true;
    in.flood_clear = true;
    in.full_sync = false;
    in.half_sync = true;
    auto r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "flood: fuel cut");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutFlood) != 0u,
               "kFuelCutFlood");

    in = base_in();
    in.sequential = true;
    in.phase_valid = false;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "seq without phase: fuel cut");
    CHECK_FALSE(r.allow_ignition, "seq without phase: spark cut");
    CHECK_TRUE((ems::engine::g_spark_cut_reasons & ems::engine::kSparkCutNoSync) != 0u,
               "kSparkCutNoSync");
}

void test_limp_gating_oil_fault_cuts_at_idle(void)
{
    section("limp_gating: oil_fault is fuel-only after 5 s running, never spark");
    ecu_sched_test_reset();
    limp_gating_reset();

    LimpGatingInputs in = base_in();
    in.oil_fault = true;
    in.cranking = false;
    in.rpm_x10 = 8000u;
    in.now_ms = 1000u;
    auto r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "oil_fault inside 5 s: inj still on");
    CHECK_TRUE(r.allow_ignition, "oil_fault never cuts spark");

    in.now_ms = 7000u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "oil_fault after 5 s running: fuel cut");
    CHECK_TRUE(r.allow_ignition, "spark stays (wasted-spark bench)");
    CHECK_TRUE((ems::engine::g_fuel_cut_reasons & ems::engine::kFuelCutOilPress) != 0u,
               "kFuelCutOilPress after 5 s dead sensor");

    in.rpm_x10 = 0u;
    r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "stopped: oil_fault does not cut");

    in.rpm_x10 = 8000u;
    in.cranking = true;
    in.now_ms = 20000u;
    r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "cranking: oil_fault does not block start");
}

void test_limp_gating_oil_running_timeout(void)
{
    section("limp_gating: running oil below min for 500 ms");
    ecu_sched_test_reset();
    limp_gating_reset();
    ems::engine::oil_min_after_start_bar_x1000 = 1500u;

    LimpGatingInputs in = base_in();
    in.rpm_x10 = 20000u;
    in.oil_press_bar_x1000 = 2000u;
    in.now_ms = 0u;
    auto r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "oil OK: inj");
    in.oil_press_bar_x1000 = 200u;
    in.now_ms = 100u;
    r = limp_gating_update(in);
    CHECK_TRUE(r.allow_injection, "low oil 100 ms: not yet");
    in.now_ms = 700u;
    r = limp_gating_update(in);
    CHECK_FALSE(r.allow_injection, "low oil 500 ms: fuel cut");
}

void test_ecu_sched_on_encoder_stall_safe_state(void)
{
    section("stall: phase_invalidate + pins LOW + queues empty");
    ecu_sched_test_reset();
    ems::drv::ckp_test_reset();
    ems::hal::out_pins_test_reset_stubs();

    ecu_sched_encoder_phase_set_anchor(100000u, ECU_PHASE_A);
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 1u, "anchored before stall");

    ecu_sched_encoder_omega_sample(0u, 0u);
    ecu_sched_encoder_omega_sample(1000u, 1000u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "omega valid");

    ecu_sched_encoder_test_set_tim2_cnt(0u);
    ecu_sched_encoder_arm_channel(ECU_CH_INJ1, 500u, ECU_ACT_INJ_ON);
    CHECK_TRUE(ecu_sched_encoder_test_get_evt_count() >= 1u, "INJ_ON queued");

    ecu_sched_on_encoder_stall();
    CHECK_EQ(ecu_sched_encoder_phase_valid(), 0u, "stall invalidates phase");
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 0u, "stall resets omega");
    CHECK_EQ(ecu_sched_encoder_test_get_evt_count(), 0u, "encoder queue empty");
    CHECK_EQ(ecu_sched_encoder_test_get_cmp_confirm_count(), 0u,
             "confirm_count 0 — one CMP edge will not re-enter sequential");
}
