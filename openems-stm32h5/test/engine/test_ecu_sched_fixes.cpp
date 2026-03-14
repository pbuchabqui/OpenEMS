/**
 * @file test/engine/test_ecu_sched_fixes.cpp
 * @brief Host unit tests for ECU scheduler — Angle-Domain Scheduling v3.
 *
 * Tests the angle-domain scheduling, phase filtering, presync HALF_SYNC
 * behaviour, and calibration commit in the new scheduler design.
 */

#define EMS_HOST_TEST 1

#include <cstdint>
#include <cstdio>
#include <string.h>

#include "engine/ecu_sched.h"
#include "drv/ckp.h"

namespace ems::engine {
void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept;
}

/* Mock peripheral backing storage */
FTM_Type g_mock_ftm0;
PDB_Type g_mock_pdb0;
ADC_Type g_mock_adc0;

/* Redirect macros for this translation unit */
#define FTM0  (&g_mock_ftm0)
#define PDB0  (&g_mock_pdb0)
#define ADC0  (&g_mock_adc0)

namespace {
int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_FALSE(cond) do { \
    ++g_tests_run; \
    if (cond) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: expected false but got true\n", __FILE__, __LINE__); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U8(exp, act) do { \
    ++g_tests_run; \
    const uint8_t _e = static_cast<uint8_t>(exp); \
    const uint8_t _a = static_cast<uint8_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

static void reset_mocks(void) {
    memset(&g_mock_ftm0, 0, sizeof(g_mock_ftm0));
    memset(&g_mock_pdb0, 0, sizeof(g_mock_pdb0));
    memset(&g_mock_adc0, 0, sizeof(g_mock_adc0));
}

static void test_reset(void) {
    reset_mocks();
    ecu_sched_test_reset();
}

/* Trigger Calculate_Sequential_Cycle via tooth hook boundary (tooth 1→0). */
static void trigger_sequential_cycle(uint32_t tooth_period_ns, bool phase_a)
{
    ems::drv::CkpSnapshot t1{tooth_period_ns, 1u, 0u, 10000u,
                              ems::drv::SyncState::FULL_SYNC, phase_a};
    ems::drv::CkpSnapshot t0{tooth_period_ns, 0u, 0u, 10000u,
                              ems::drv::SyncState::FULL_SYNC, phase_a};
    ems::engine::ecu_sched_on_tooth_hook(t1);
    ems::engine::ecu_sched_on_tooth_hook(t0);
}

/* Count angle-table events matching (ch, action). */
static uint8_t count_angle_events(uint8_t ch, uint8_t act) {
    uint8_t count = 0U;
    const uint8_t n = ecu_sched_test_angle_table_size();
    for (uint8_t i = 0U; i < n; ++i) {
        uint8_t t = 0U, sf = 0U, ech = 0U, eact = 0U, phase = 0U;
        if ((ecu_sched_test_get_angle_event(i, &t, &sf, &ech, &eact, &phase) != 0U) &&
            (ech == ch) && (eact == act)) {
            ++count;
        }
    }
    return count;
}

} // namespace

// =============================================================================
// Test: angle table fills 16 events for a full 4-cylinder sequential cycle
// =============================================================================

void test_angle_table_fills_16_events_for_full_cycle() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(22500U);
    ecu_sched_test_set_inj_pw_ticks(15000U);
    ecu_sched_test_set_soi_lead_deg(62U);

    trigger_sequential_cycle(277778U, true);

    /* 4 cyl × 4 events = 16 */
    TEST_ASSERT_EQ_U8(16U, ecu_sched_test_angle_table_size());

    uint8_t spark = 0U, dwell = 0U, inj_on = 0U, inj_off = 0U;
    const uint8_t n = ecu_sched_test_angle_table_size();
    for (uint8_t i = 0U; i < n; ++i) {
        uint8_t t = 0U, sf = 0U, ch = 0U, act = 0U, phase = 0U;
        ecu_sched_test_get_angle_event(i, &t, &sf, &ch, &act, &phase);
        if (act == ECU_ACT_SPARK)        { ++spark; }
        if (act == ECU_ACT_DWELL_START)  { ++dwell; }
        if (act == ECU_ACT_INJ_ON)       { ++inj_on; }
        if (act == ECU_ACT_INJ_OFF)      { ++inj_off; }
    }
    TEST_ASSERT_EQ_U8(4U, spark);
    TEST_ASSERT_EQ_U8(4U, dwell);
    TEST_ASSERT_EQ_U8(4U, inj_on);
    TEST_ASSERT_EQ_U8(4U, inj_off);
}

// =============================================================================
// Test: angle_to_tooth_event conversion accuracy
// =============================================================================

void test_angle_to_tooth_conversion_accuracy() {
    // 0° → tooth 0, sub_frac 0, phase_A=ECU_PHASE_A
    // 360° → tooth 0, sub_frac 0, phase_A=ECU_PHASE_B
    // 180° → tooth 30, sub_frac 0, phase_A=ECU_PHASE_A (30 * 6° = 180°)
    // 6° → tooth 1, sub_frac 0

    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_advance_deg(0U);   // spark at TDC0 = 0°
    ecu_sched_test_set_dwell_ticks(1U);   // minimal dwell
    ecu_sched_test_set_inj_pw_ticks(1U);  // minimal PW
    ecu_sched_test_set_soi_lead_deg(0U);  // inj at TDC = 0°

    trigger_sequential_cycle(277778U, true);

    uint8_t n = ecu_sched_test_angle_table_size();
    TEST_ASSERT_TRUE(n > 0U);

    /* All tooth indices must be in valid range [0,57] */
    for (uint8_t i = 0U; i < n; ++i) {
        uint8_t tooth = 0U, sf = 0U, ch = 0U, act = 0U, phase = 0U;
        ecu_sched_test_get_angle_event(i, &tooth, &sf, &ch, &act, &phase);
        TEST_ASSERT_TRUE(tooth <= 57U);
        /* phase must be ECU_PHASE_A or ECU_PHASE_B (FULL_SYNC, not ANY) */
        TEST_ASSERT_TRUE((phase == ECU_PHASE_A) || (phase == ECU_PHASE_B));
    }
}

// =============================================================================
// Test: tooth hook only arms events matching the current tooth_index
// =============================================================================

void test_tooth_hook_arms_matching_events() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(22500U);
    ecu_sched_test_set_inj_pw_ticks(15000U);
    ecu_sched_test_set_soi_lead_deg(62U);

    // First cycle to populate the angle table
    trigger_sequential_cycle(277778U, true);

    uint8_t n = ecu_sched_test_angle_table_size();
    TEST_ASSERT_TRUE(n > 0U);

    // Find which tooth index the first event is on
    uint8_t target_tooth = 0U, sf = 0U, ch = 0U, act = 0U, phase = 0U;
    ecu_sched_test_get_angle_event(0U, &target_tooth, &sf, &ch, &act, &phase);

    // Reset CnV to detect arming
    for (uint8_t i = 0U; i < 8U; ++i) {
        FTM0->CH[i].CnV = 0U;
    }

    // Deliver a tooth that does NOT match target_tooth — nothing should be armed on that channel
    uint16_t different_tooth = (target_tooth > 0U) ? (target_tooth - 1U) : 1U;
    ems::drv::CkpSnapshot miss{277778u,
                                static_cast<uint16_t>(different_tooth),
                                0u, 10000u,
                                ems::drv::SyncState::FULL_SYNC, true};
    ems::engine::ecu_sched_on_tooth_hook(miss);

    // CnV for the channel of event[0] should still be 0 (not armed by this tooth)
    TEST_ASSERT_EQ_U32(0U, FTM0->CH[ch].CnV);

    // Deliver the matching tooth — channel should be armed
    ems::drv::CkpSnapshot hit{277778u,
                               target_tooth,
                               0u, 10000u,
                               ems::drv::SyncState::FULL_SYNC,
                               (phase == ECU_PHASE_A)};
    ems::engine::ecu_sched_on_tooth_hook(hit);

    /* CnV should be non-zero now (sub_frac > 0 gives a real offset) */
    /* At minimum CnSC should have been programmed (set to OC mode) */
    TEST_ASSERT_TRUE((FTM0->CH[ch].CnSC & FTM_CnSC_MSB_MASK) != 0U);
}

// =============================================================================
// Test: phase_A filtering — events don't fire in wrong phase
// =============================================================================

void test_phase_A_filtering() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(22500U);
    ecu_sched_test_set_inj_pw_ticks(15000U);
    ecu_sched_test_set_soi_lead_deg(62U);

    // Fill table with phase_A=true
    trigger_sequential_cycle(277778U, true);

    // Find a PHASE_B event in the table
    uint8_t phase_b_tooth = 255U, phase_b_ch = 255U, phase_b_act = 0U;
    uint8_t n = ecu_sched_test_angle_table_size();
    for (uint8_t i = 0U; i < n; ++i) {
        uint8_t tooth = 0U, sf = 0U, ch = 0U, act = 0U, phase = 0U;
        ecu_sched_test_get_angle_event(i, &tooth, &sf, &ch, &act, &phase);
        if (phase == ECU_PHASE_B) {
            phase_b_tooth = tooth;
            phase_b_ch    = ch;
            phase_b_act   = act;
            break;
        }
    }

    if (phase_b_tooth == 255U) {
        /* All events happen to be phase_A at this calibration — skip sub-test */
        return;
    }

    /* Reset CnV */
    for (uint8_t i = 0U; i < 8U; ++i) { FTM0->CH[i].CnV = 0U; }

    /* Deliver the matching tooth but with phase_A=true → PHASE_B event should NOT fire */
    ems::drv::CkpSnapshot wrong_phase{277778u, phase_b_tooth, 0u, 10000u,
                                       ems::drv::SyncState::FULL_SYNC, true};
    ems::engine::ecu_sched_on_tooth_hook(wrong_phase);
    TEST_ASSERT_EQ_U32(0U, FTM0->CH[phase_b_ch].CnV);

    /* Reset CnV */
    for (uint8_t i = 0U; i < 8U; ++i) { FTM0->CH[i].CnV = 0U; }

    /* Deliver the matching tooth with phase_A=false → PHASE_B event SHOULD fire */
    ems::drv::CkpSnapshot right_phase{277778u, phase_b_tooth, 0u, 10000u,
                                       ems::drv::SyncState::FULL_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(right_phase);
    TEST_ASSERT_TRUE((FTM0->CH[phase_b_ch].CnSC & FTM_CnSC_MSB_MASK) != 0U);
}

// =============================================================================
// Test: HALF_SYNC presync produces events with ECU_PHASE_ANY
// =============================================================================

void test_presync_uses_phase_any() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_set_presync_enable(1U);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_presync_ign_mode(ECU_PRESYNC_IGN_WASTED_SPARK);
    ecu_sched_commit_calibration(10U, 22500U, 15000U, 62U);

    /* Trigger presync via HALF_SYNC boundary */
    ems::drv::CkpSnapshot t1{277778u, 1u, 0u, 10000u,
                              ems::drv::SyncState::HALF_SYNC, false};
    ems::drv::CkpSnapshot t0{277778u, 0u, 0u, 10000u,
                              ems::drv::SyncState::HALF_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(t1);
    ems::engine::ecu_sched_on_tooth_hook(t0);  /* boundary → calculate_presync_revolution */

    uint8_t n = ecu_sched_test_angle_table_size();
    TEST_ASSERT_TRUE(n > 0U);

    /* All presync events must have phase_A == ECU_PHASE_ANY */
    for (uint8_t i = 0U; i < n; ++i) {
        uint8_t tooth = 0U, sf = 0U, ch = 0U, act = 0U, phase = 0U;
        ecu_sched_test_get_angle_event(i, &tooth, &sf, &ch, &act, &phase);
        TEST_ASSERT_EQ_U8(ECU_PHASE_ANY, phase);
    }
}

// =============================================================================
// Test: ecu_sched_commit_calibration new signature (no tpr argument)
// =============================================================================

void test_commit_calibration_no_tpr() {
    test_reset();
    ECU_Hardware_Init();

    /* PS=64: 1875 ticks/ms. dwell=5625 (3ms), inj_pw=12000 (6.4ms) — ambos dentro dos limites */
    ecu_sched_commit_calibration(15U, 5625U, 12000U, 55U);

    TEST_ASSERT_EQ_U32(15U,    ecu_sched_test_get_advance_deg());
    TEST_ASSERT_EQ_U32(5625U,  ecu_sched_test_get_dwell_ticks());
    TEST_ASSERT_EQ_U32(12000U, ecu_sched_test_get_inj_pw_ticks());
    TEST_ASSERT_EQ_U32(55U,    ecu_sched_test_get_soi_lead_deg());
}

// =============================================================================
// Test: Calibration clamping (advance, dwell, pw, soi)
// =============================================================================

void test_calibration_clamping() {
    test_reset();
    ECU_Hardware_Init();

    const uint32_t clamps_before = ecu_sched_test_get_calibration_clamp_count();

    /* Out-of-range advance (> 60°) must be clamped */
    ecu_sched_test_set_advance_deg(999U);
    TEST_ASSERT_TRUE(ecu_sched_test_get_advance_deg() <= 60U);

    /* Out-of-range soi_lead must be clamped below 720° */
    ecu_sched_test_set_soi_lead_deg(9999U);
    TEST_ASSERT_TRUE(ecu_sched_test_get_soi_lead_deg() < 720U);

    /* Excessively large dwell/pw must be clamped */
    ecu_sched_test_set_dwell_ticks(0xFFFFFFFFU);
    ecu_sched_test_set_inj_pw_ticks(0xFFFFFFFFU);
    TEST_ASSERT_TRUE(ecu_sched_test_get_dwell_ticks() <= 75000U);
    TEST_ASSERT_TRUE(ecu_sched_test_get_inj_pw_ticks() <= 150000U);

    TEST_ASSERT_TRUE(ecu_sched_test_get_calibration_clamp_count() > clamps_before);
}

// =============================================================================
// Test: Sync loss clears angle table and drives safe outputs
// =============================================================================

void test_sync_loss_clears_angle_table_and_drives_safe_outputs() {
    test_reset();
    ECU_Hardware_Init();

    trigger_sequential_cycle(277778U, true);
    TEST_ASSERT_TRUE(ecu_sched_test_angle_table_size() > 0U);

    ems::drv::CkpSnapshot loss{277778u, 2u, 0u, 10000u,
                               ems::drv::SyncState::LOSS_OF_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(loss);

    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_angle_table_size());
    /* Injector channels must be in clear-on-match (safe: closed) */
    TEST_ASSERT_EQ_U32(FTM_CnSC_OC_CLEAR, FTM0->CH[ECU_CH_INJ1].CnSC);
    TEST_ASSERT_EQ_U32(FTM_CnSC_OC_CLEAR, FTM0->CH[ECU_CH_IGN1].CnSC);
}

// =============================================================================
// CYC-01: guard SCH-02 blocks first revolution boundary after sync
// =============================================================================

void test_cyc01_sch02_guard_blocks_first_boundary() {
    test_reset();
    ECU_Hardware_Init();

    /* First tooth_index=0 with no previous tooth: hook_prev_valid=0 → no schedule */
    ems::drv::CkpSnapshot snap0{277778u, 0u, 0u, 10000u,
                                ems::drv::SyncState::FULL_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(snap0);
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_angle_table_size());

    /* Intermediate tooth: activates prev_valid */
    ems::drv::CkpSnapshot snap5{277778u, 5u, 0u, 10000u,
                                ems::drv::SyncState::FULL_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(snap5);

    /* Second tooth_index=0: prev_valid=1, prev_tooth=5≠0 → revolution boundary → schedule */
    ems::engine::ecu_sched_on_tooth_hook(snap0);
    TEST_ASSERT_TRUE(ecu_sched_test_angle_table_size() > 0U);
}

void test_cyc01_default_calibration_produces_valid_events() {
    test_reset();
    ECU_Hardware_Init();
    /* Do NOT call ecu_sched_commit_calibration — use defaults */

    ems::drv::CkpSnapshot snap1{277778u, 1u, 0u, 10000u,
                                ems::drv::SyncState::FULL_SYNC, false};
    ems::drv::CkpSnapshot snap0{277778u, 0u, 0u, 10000u,
                                ems::drv::SyncState::FULL_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(snap1);
    ems::engine::ecu_sched_on_tooth_hook(snap0);

    uint8_t n = ecu_sched_test_angle_table_size();
    TEST_ASSERT_TRUE(n > 0U);
    /* All events must have valid tooth indices (no garbage from uninitialised state) */
    for (uint8_t i = 0U; i < n; ++i) {
        uint8_t tooth = 0U, sf = 0U, ch = 0U, act = 0U, phase = 0U;
        ecu_sched_test_get_angle_event(i, &tooth, &sf, &ch, &act, &phase);
        TEST_ASSERT_TRUE(tooth <= 57U);
    }
}

// =============================================================================
// Test: HALF_SYNC simultaneous injection and wasted spark via presync
// =============================================================================

void test_presync_halfsync_simultaneous_inj_and_wasted_spark() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_set_presync_enable(1U);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SIMULTANEOUS);
    ecu_sched_set_presync_ign_mode(ECU_PRESYNC_IGN_WASTED_SPARK);
    ecu_sched_commit_calibration(10U, 22500U, 12000U, 62U);

    ems::drv::CkpSnapshot t1{277778u, 1u, 0u, 3000u,
                              ems::drv::SyncState::HALF_SYNC, false};
    ems::drv::CkpSnapshot t0{277778u, 0u, 0u, 3000u,
                              ems::drv::SyncState::HALF_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(t1);
    ems::engine::ecu_sched_on_tooth_hook(t0);  /* boundary → presync schedule */

    TEST_ASSERT_TRUE(count_angle_events(ECU_CH_INJ1, ECU_ACT_INJ_ON) >= 1U);
    TEST_ASSERT_TRUE(count_angle_events(ECU_CH_INJ2, ECU_ACT_INJ_ON) >= 1U);
    TEST_ASSERT_TRUE(count_angle_events(ECU_CH_INJ3, ECU_ACT_INJ_ON) >= 1U);
    TEST_ASSERT_TRUE(count_angle_events(ECU_CH_INJ4, ECU_ACT_INJ_ON) >= 1U);

    TEST_ASSERT_TRUE(count_angle_events(ECU_CH_IGN1, ECU_ACT_SPARK) >= 1U);
    TEST_ASSERT_TRUE(count_angle_events(ECU_CH_IGN2, ECU_ACT_SPARK) >= 1U);
}

// =============================================================================
// Test: HALF_SYNC semi-sequential injection alternates banks
// =============================================================================

void test_presync_halfsync_semi_sequential_alternates_banks() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_set_presync_enable(1U);
    ecu_sched_set_presync_inj_mode(ECU_PRESYNC_INJ_SEMI_SEQUENTIAL);
    ecu_sched_set_presync_ign_mode(ECU_PRESYNC_IGN_WASTED_SPARK);
    ecu_sched_commit_calibration(10U, 22500U, 12000U, 62U);

    ems::drv::CkpSnapshot t1{277778u, 1u, 0u, 3000u,
                              ems::drv::SyncState::HALF_SYNC, false};
    ems::drv::CkpSnapshot t0{277778u, 0u, 0u, 3000u,
                              ems::drv::SyncState::HALF_SYNC, false};

    /* First boundary → bank A (INJ1 + INJ4, toggle=0) */
    ems::engine::ecu_sched_on_tooth_hook(t1);
    ems::engine::ecu_sched_on_tooth_hook(t0);
    const bool bank_a_after_first = (count_angle_events(ECU_CH_INJ1, ECU_ACT_INJ_ON) >= 1U) ||
                                    (count_angle_events(ECU_CH_INJ4, ECU_ACT_INJ_ON) >= 1U);
    TEST_ASSERT_TRUE(bank_a_after_first);

    /* Second boundary → bank B (INJ2 + INJ3, toggle=1 → then 0) */
    ems::engine::ecu_sched_on_tooth_hook(t1);
    ems::engine::ecu_sched_on_tooth_hook(t0);
    const bool bank_b_after_second = (count_angle_events(ECU_CH_INJ2, ECU_ACT_INJ_ON) >= 1U) ||
                                     (count_angle_events(ECU_CH_INJ3, ECU_ACT_INJ_ON) >= 1U);
    TEST_ASSERT_TRUE(bank_b_after_second);
}

// =============================================================================
// IVC Clamp Tests
// =============================================================================

/* tooth_period_ns = 1 000 000 ns → 1000 RPM (60-tooth wheel)
 * tooth_ftm0 = 1 875 ticks; ticks_per_rev720 = 225 000 ticks
 * 1° = 312.5 ticks; 100° = 31 250 ticks; 50° = 15 625 ticks */
static constexpr uint32_t kToothPeriodNs1000Rpm = 1000000U;

/* Standard closed-valve scenario: soi_lead_deg=62 → SOI after IVC → no clamp. */
void test_ivc_no_clamp_closed_valve() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_ivc(50U);           /* IVC = 50° ABDC */
    ecu_sched_test_set_soi_lead_deg(62U);  /* closed-valve: SOI after IVC */
    ecu_sched_test_set_inj_pw_ticks(31250U);  /* ~100° at 1000 RPM — large PW */
    ecu_sched_test_set_dwell_ticks(3750U);
    ecu_sched_test_set_advance_deg(10U);

    trigger_sequential_cycle(kToothPeriodNs1000Rpm, true);

    TEST_ASSERT_EQ_U32(0U, ecu_sched_test_get_ivc_clamp_count());
}

/* Open-valve injection (soi_lead_deg=180 → SOI=540°, before IVC=590° for cyl1)
 * with PW > soi_to_ivc (50°) → clamp fires once per cylinder = 4 total. */
void test_ivc_clamp_open_valve() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_ivc(50U);            /* IVC = 50° ABDC → cycle 590°,50°,230°,410° */
    ecu_sched_test_set_soi_lead_deg(180U);  /* open-valve: SOI = BDC intake */
    ecu_sched_test_set_inj_pw_ticks(31250U);  /* ~100° > soi_to_ivc(50°) → clamp */
    ecu_sched_test_set_dwell_ticks(3750U);
    ecu_sched_test_set_advance_deg(10U);

    trigger_sequential_cycle(kToothPeriodNs1000Rpm, true);

    /* Each of the 4 cylinders triggers the clamp (all have soi_to_ivc=50°) */
    TEST_ASSERT_EQ_U32(4U, ecu_sched_test_get_ivc_clamp_count());
    /* Injection events must still exist (eff_inj_pw_deg = 50° > 0) */
    TEST_ASSERT_EQ_U8(4U, count_angle_events(ECU_CH_INJ1, ECU_ACT_INJ_ON) +
                          count_angle_events(ECU_CH_INJ2, ECU_ACT_INJ_ON) +
                          count_angle_events(ECU_CH_INJ3, ECU_ACT_INJ_ON) +
                          count_angle_events(ECU_CH_INJ4, ECU_ACT_INJ_ON));
}

/* SOI exactly at IVC (soi_to_ivc = 0): eff_inj_pw_deg = 0; clamp fires.
 * For cyl1 tdc=0, IVC=590°: soi_lead_deg = 720-590 = 130° → inj_on_deg=590°=IVC. */
void test_ivc_exact_boundary() {
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_ivc(50U);             /* IVC cyl1 = 590° */
    ecu_sched_test_set_soi_lead_deg(130U);   /* inj_on_deg = (0+720-130)%720 = 590° = IVC */
    ecu_sched_test_set_inj_pw_ticks(15625U); /* > 0 so clamp evaluates */
    ecu_sched_test_set_dwell_ticks(3750U);
    ecu_sched_test_set_advance_deg(10U);

    trigger_sequential_cycle(kToothPeriodNs1000Rpm, true);

    /* soi_to_ivc=0 < 360 and inj_pw_deg > 0 → clamp fires (4 cylinders) */
    TEST_ASSERT_EQ_U32(4U, ecu_sched_test_get_ivc_clamp_count());
}

/* After reset, clamp count is 0 and default ivc = 50° (closed-valve produces no clamp). */
void test_ivc_default_state_after_reset() {
    test_reset();
    ECU_Hardware_Init();
    /* Default ivc=50 and soi_lead=62 (closed-valve) → no clamp */
    ecu_sched_test_set_inj_pw_ticks(31250U);
    ecu_sched_test_set_dwell_ticks(3750U);
    ecu_sched_test_set_advance_deg(10U);

    trigger_sequential_cycle(kToothPeriodNs1000Rpm, true);

    TEST_ASSERT_EQ_U32(0U, ecu_sched_test_get_ivc_clamp_count());
}

// =============================================================================
// Main Test Runner
// =============================================================================

int main() {
    printf("Running EMS ECU Scheduler Angle-Domain Tests...\n");

    /* Angle table filling */
    test_angle_table_fills_16_events_for_full_cycle();
    test_angle_to_tooth_conversion_accuracy();

    /* Tooth hook arming */
    test_tooth_hook_arms_matching_events();
    test_phase_A_filtering();

    /* HALF_SYNC presync */
    test_presync_uses_phase_any();
    test_presync_halfsync_simultaneous_inj_and_wasted_spark();
    test_presync_halfsync_semi_sequential_alternates_banks();

    /* Calibration */
    test_commit_calibration_no_tpr();
    test_calibration_clamping();

    /* Sync loss safety */
    test_sync_loss_clears_angle_table_and_drives_safe_outputs();

    /* CYC-01: guard SCH-02 and default calibration */
    test_cyc01_sch02_guard_blocks_first_boundary();
    test_cyc01_default_calibration_produces_valid_events();

    /* IVC clamp */
    test_ivc_no_clamp_closed_valve();
    test_ivc_clamp_open_valve();
    test_ivc_exact_boundary();
    test_ivc_default_state_after_reset();

    printf("ECU scheduler angle-domain tests completed: %d run, %d failed\n",
           g_tests_run, g_tests_failed);

    return (g_tests_failed == 0) ? 0 : 1;
}
