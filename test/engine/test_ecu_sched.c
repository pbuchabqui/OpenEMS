/**
 * @file test/engine/test_ecu_sched.c
 * @brief Host unit tests for engine/ecu_sched — Angle-Domain Scheduling v3.
 *
 * Compiled with (note: g++ compiles .c files as C++17):
 *   g++ -std=c++17 -DEMS_HOST_TEST -Isrc \
 *       test/engine/test_ecu_sched.c src/engine/ecu_sched.cpp -o test_ecu_sched
 *
 * Mock strategy:
 *   FTM0, PDB0, ADC0 are backed by global structs defined here.
 */

#define EMS_HOST_TEST 1

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "engine/ecu_sched.h"
#include "drv/ckp.h"

/* Mock peripheral backing storage */
FTM_Type g_mock_ftm0;
PDB_Type g_mock_pdb0;
ADC_Type g_mock_adc0;

/* Redirect macros for this translation unit */
#define FTM0  (&g_mock_ftm0)
#define PDB0  (&g_mock_pdb0)
#define ADC0  (&g_mock_adc0)

/* Forward-declare C++ hook function */
namespace ems::engine {
void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept;
}

/* ============================================================================
 * Test framework
 * ========================================================================= */

static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    uint32_t _e = (uint32_t)(exp); \
    uint32_t _a = (uint32_t)(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, \
               (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U8(exp, act) do { \
    ++g_tests_run; \
    uint8_t _e = (uint8_t)(exp); \
    uint8_t _a = (uint8_t)(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, \
               (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

/* ============================================================================
 * Test helpers
 * ========================================================================= */

static void reset_mocks(void)
{
    memset(&g_mock_ftm0, 0, sizeof(g_mock_ftm0));
    memset(&g_mock_pdb0, 0, sizeof(g_mock_pdb0));
    memset(&g_mock_adc0, 0, sizeof(g_mock_adc0));
}

static void test_reset(void)
{
    reset_mocks();
    ecu_sched_test_reset();
}

/* Trigger Calculate_Sequential_Cycle via tooth hook (revolution boundary). */
static void trigger_sequential_cycle(uint32_t tooth_period_ns, bool phase)
{
    ems::drv::CkpSnapshot t1{tooth_period_ns, 1u, 0u, 10000u,
                              ems::drv::SyncState::FULL_SYNC, phase};
    ems::drv::CkpSnapshot t0{tooth_period_ns, 0u, 0u, 10000u,
                              ems::drv::SyncState::FULL_SYNC, phase};
    /* First tooth activates hook_prev_valid */
    ems::engine::ecu_sched_on_tooth_hook(t1);
    /* tooth_index=0 with prev_valid=1 and prev_tooth!=0 → revolution boundary */
    ems::engine::ecu_sched_on_tooth_hook(t0);
}

/* ============================================================================
 * Test: ECU_Hardware_Init configures FTM0 correctly
 * ========================================================================= */

static void test_init_ftm0_sc(void)
{
    test_reset();
    ECU_Hardware_Init();

    /* SC must have CLKS=system, PS=64, NO TOIE (angle domain: overflow not managed) */
    uint32_t sc = FTM0->SC;
    TEST_ASSERT_TRUE((sc & FTM_SC_CLKS_SYSTEM) != 0U);
    TEST_ASSERT_TRUE((sc & FTM_SC_TOIE_MASK)   == 0U);   /* no overflow interrupt */
    TEST_ASSERT_TRUE((sc & 0x7U) == FTM_SC_PS_64);
}

static void test_init_ftm0_mod(void)
{
    test_reset();
    ECU_Hardware_Init();
    TEST_ASSERT_EQ_U32(0xFFFFU, FTM0->MOD);
}

static void test_init_inj_channels_set_on_match(void)
{
    uint8_t ch;
    test_reset();
    ECU_Hardware_Init();
    /* CH0-CH3: Set on match — must have MSB and ELSB set */
    for (ch = 0U; ch < 4U; ++ch) {
        uint32_t cnsc = FTM0->CH[ch].CnSC;
        TEST_ASSERT_TRUE((cnsc & FTM_CnSC_MSB_MASK)  != 0U);
        TEST_ASSERT_TRUE((cnsc & FTM_CnSC_ELSB_MASK) != 0U);
    }
}

static void test_init_ign_channels_clear_on_match(void)
{
    uint8_t ch;
    test_reset();
    ECU_Hardware_Init();
    /* CH4-CH7: Clear on match — must have MSB and ELSA set, ELSB clear */
    for (ch = 4U; ch < 8U; ++ch) {
        uint32_t cnsc = FTM0->CH[ch].CnSC;
        TEST_ASSERT_TRUE((cnsc & FTM_CnSC_MSB_MASK)  != 0U);
        TEST_ASSERT_TRUE((cnsc & FTM_CnSC_ELSA_MASK) != 0U);
        TEST_ASSERT_TRUE((cnsc & FTM_CnSC_ELSB_MASK) == 0U);
    }
}

/* ============================================================================
 * Test: ECU_Hardware_Init configures ADC0 hardware averaging
 * ========================================================================= */

static void test_init_adc0_averaging(void)
{
    test_reset();
    ECU_Hardware_Init();
    /* SC3 bit2 = AVGE must be set; bits[1:0] = AVGS = 00 for 4 samples */
    uint32_t sc3 = ADC0->SC3;
    TEST_ASSERT_TRUE((sc3 & ADC_SC3_AVG4) != 0U); /* AVGE=1 */
    TEST_ASSERT_TRUE((sc3 & 0x3U) == 0U);          /* AVGS=00 (4 samples) */
}

/* ============================================================================
 * Test: ECU_Hardware_Init configures PDB0
 * ========================================================================= */

static void test_init_pdb0_enabled(void)
{
    test_reset();
    ECU_Hardware_Init();
    TEST_ASSERT_TRUE((PDB0->SC & PDB_SC_PDBEN_MASK) != 0U);
    TEST_ASSERT_TRUE((PDB0->SC & PDB_SC_TRGSEL_FTM0) != 0U);
}

/* ============================================================================
 * Test: FTM0_IRQHandler clears CHF only — no TOF handling
 * ========================================================================= */

static void test_isr_no_tof_handling(void)
{
    test_reset();
    ECU_Hardware_Init();

    /* Set TOF and CHF flags in mock */
    FTM0->SC |= FTM_SC_TOF_MASK;
    FTM0->CH[ECU_CH_IGN1].CnSC |= FTM_CnSC_CHF_MASK;

    FTM0_IRQHandler();

    /* TOF should NOT be cleared — no overflow handling in angle domain */
    TEST_ASSERT_TRUE((FTM0->SC & FTM_SC_TOF_MASK) != 0U);
    /* CHF should be cleared */
    TEST_ASSERT_TRUE((FTM0->CH[ECU_CH_IGN1].CnSC & FTM_CnSC_CHF_MASK) == 0U);
}

static void test_isr_clears_all_chf(void)
{
    uint8_t ch;
    test_reset();
    ECU_Hardware_Init();

    for (ch = 0U; ch < 8U; ++ch) {
        FTM0->CH[ch].CnSC |= FTM_CnSC_CHF_MASK;
    }

    FTM0_IRQHandler();

    for (ch = 0U; ch < 8U; ++ch) {
        TEST_ASSERT_TRUE((FTM0->CH[ch].CnSC & FTM_CnSC_CHF_MASK) == 0U);
    }
}

/* ============================================================================
 * Test: Calculate_Sequential_Cycle fills angle table with 16 events
 * ========================================================================= */

static void test_angle_table_fills_16_events(void)
{
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(22500U);
    ecu_sched_test_set_inj_pw_ticks(15000U);
    ecu_sched_test_set_soi_lead_deg(62U);

    trigger_sequential_cycle(277778U, true);

    /* 4 cylinders × 4 events (DWELL_START + SPARK + INJ_ON + INJ_OFF) = 16 */
    TEST_ASSERT_EQ_U8(16U, ecu_sched_test_angle_table_size());
}

static void test_angle_table_action_counts(void)
{
    uint8_t spark_count = 0U;
    uint8_t dwell_count = 0U;
    uint8_t inj_on_count = 0U;
    uint8_t inj_off_count = 0U;
    uint8_t n, i;

    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(22500U);
    ecu_sched_test_set_inj_pw_ticks(15000U);
    ecu_sched_test_set_soi_lead_deg(62U);

    trigger_sequential_cycle(277778U, true);

    n = ecu_sched_test_angle_table_size();
    for (i = 0U; i < n; ++i) {
        uint8_t tooth = 0U, sf = 0U, ch = 0U, act = 0U, phase = 0U;
        ecu_sched_test_get_angle_event(i, &tooth, &sf, &ch, &act, &phase);
        if (act == ECU_ACT_SPARK)        { ++spark_count; }
        if (act == ECU_ACT_DWELL_START)  { ++dwell_count; }
        if (act == ECU_ACT_INJ_ON)       { ++inj_on_count; }
        if (act == ECU_ACT_INJ_OFF)      { ++inj_off_count; }
    }
    TEST_ASSERT_EQ_U8(4U, spark_count);
    TEST_ASSERT_EQ_U8(4U, dwell_count);
    TEST_ASSERT_EQ_U8(4U, inj_on_count);
    TEST_ASSERT_EQ_U8(4U, inj_off_count);
}

/* ============================================================================
 * Test: ecu_sched_commit_calibration new signature (no ticks_per_rev)
 * ========================================================================= */

static void test_commit_calibration_no_tpr(void)
{
    test_reset();
    ECU_Hardware_Init();

    /* PS=64: 1875 ticks/ms. dwell=5625 (3ms), inj_pw=15000 (8ms) — ambos dentro dos limites */
    ecu_sched_commit_calibration(20U, 5625U, 15000U, 62U);

    TEST_ASSERT_EQ_U32(20U,    ecu_sched_test_get_advance_deg());
    TEST_ASSERT_EQ_U32(5625U,  ecu_sched_test_get_dwell_ticks());
    TEST_ASSERT_EQ_U32(15000U, ecu_sched_test_get_inj_pw_ticks());
    TEST_ASSERT_EQ_U32(62U,    ecu_sched_test_get_soi_lead_deg());
}

/* ============================================================================
 * Test: angle table cleared on LOSS_OF_SYNC
 * ========================================================================= */

static void test_sync_loss_clears_angle_table(void)
{
    test_reset();
    ECU_Hardware_Init();

    trigger_sequential_cycle(277778U, true);
    TEST_ASSERT_TRUE(ecu_sched_test_angle_table_size() > 0U);

    ems::drv::CkpSnapshot loss{277778u, 2u, 0u, 10000u,
                               ems::drv::SyncState::LOSS_OF_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(loss);

    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_angle_table_size());
}

/* ============================================================================
 * main
 * ========================================================================= */

int main(void)
{
    /* ECU_Hardware_Init tests */
    test_init_ftm0_sc();
    test_init_ftm0_mod();
    test_init_inj_channels_set_on_match();
    test_init_ign_channels_clear_on_match();
    test_init_adc0_averaging();
    test_init_pdb0_enabled();

    /* FTM0_IRQHandler tests */
    test_isr_no_tof_handling();
    test_isr_clears_all_chf();

    /* Angle table tests */
    test_angle_table_fills_16_events();
    test_angle_table_action_counts();

    /* Calibration commit new signature */
    test_commit_calibration_no_tpr();

    /* Sync loss test */
    test_sync_loss_clears_angle_table();

    printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
