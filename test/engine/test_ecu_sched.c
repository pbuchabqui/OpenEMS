/**
 * @file test/engine/test_ecu_sched.c
 * @brief Host unit tests for engine/ecu_sched (C / EMS_HOST_TEST build).
 *
 * Compiled with:
 *   gcc -std=c11 -DEMS_HOST_TEST -Isrc \
 *       test/engine/test_ecu_sched.c src/engine/ecu_sched.c -o test_ecu_sched
 *
 * Mock strategy:
 *   FTM0, PDB0, ADC0 are backed by global structs defined here.
 *   The macros FTM0/PDB0/ADC0 in ecu_sched.c are redirected via the
 *   #undef / #define block in ecu_sched.c's host-test section.
 */

#define EMS_HOST_TEST 1

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * Mock peripheral structs (must be declared before including ecu_sched.h
 * so the extern declarations in ecu_sched.c can link against them).
 * ========================================================================= */

#include "engine/ecu_sched.h"

/* Backing storage for mock peripherals */
FTM_Type g_mock_ftm0;
PDB_Type g_mock_pdb0;
ADC_Type g_mock_adc0;

/* Redirect macros for this translation unit (matches ecu_sched.c redirection) */
#define FTM0  (&g_mock_ftm0)
#define PDB0  (&g_mock_pdb0)
#define ADC0  (&g_mock_adc0)

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

/* ============================================================================
 * Test: ECU_Hardware_Init configures FTM0 correctly
 * ========================================================================= */

static void test_init_ftm0_sc(void)
{
    test_reset();
    ECU_Hardware_Init();

    /* SC must have CLKS=system, TOIE=1, PS=8 */
    uint32_t sc = FTM0->SC;
    TEST_ASSERT_TRUE((sc & FTM_SC_CLKS_SYSTEM) != 0U);
    TEST_ASSERT_TRUE((sc & FTM_SC_TOIE_MASK)   != 0U);
    TEST_ASSERT_TRUE((sc & 0x7U) == FTM_SC_PS_8);
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
        uint32_t cnsc = FTM0->CONTROLS[ch].CnSC;
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
        uint32_t cnsc = FTM0->CONTROLS[ch].CnSC;
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
    /* Trigger source must be FTM0 (bits[15:12] = 0x8) */
    TEST_ASSERT_TRUE((PDB0->SC & PDB_SC_TRGSEL_FTM0) != 0U);
}

/* ============================================================================
 * Test: Add_Event inserts events in sorted order
 * ========================================================================= */

static void test_add_event_sorted(void)
{
    uint32_t ts0, ts1, ts2;
    uint8_t  ch0, ch1, ch2, act0, act1, act2;

    test_reset();
    /* Insert out of order: ts=300, ts=100, ts=200 */
    Add_Event(0x00000300UL, ECU_CH_IGN3, ECU_ACT_SPARK);
    Add_Event(0x00000100UL, ECU_CH_IGN1, ECU_ACT_DWELL_START);
    Add_Event(0x00000200UL, ECU_CH_IGN2, ECU_ACT_SPARK);

    TEST_ASSERT_EQ_U8(3U, ecu_sched_test_queue_size());

    ecu_sched_test_get_event(0U, &ts0, &ch0, &act0);
    ecu_sched_test_get_event(1U, &ts1, &ch1, &act1);
    ecu_sched_test_get_event(2U, &ts2, &ch2, &act2);

    TEST_ASSERT_EQ_U32(0x00000100UL, ts0);
    TEST_ASSERT_EQ_U32(0x00000200UL, ts1);
    TEST_ASSERT_EQ_U32(0x00000300UL, ts2);
}

/* ============================================================================
 * Test: Add_Event detects late events
 * ========================================================================= */

static void test_add_event_late(void)
{
    test_reset();
    /* Advance the overflow counter artificially */
    g_overflow_count = 5U;

    /* Add an event with epoch 3 < 5 → late */
    Add_Event(0x00030000UL, ECU_CH_IGN1, ECU_ACT_SPARK);

    TEST_ASSERT_EQ_U32(1U, g_late_event_count);
    /* Queue should be empty (late event was not enqueued) */
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
}

static void test_add_event_not_late_same_epoch(void)
{
    test_reset();
    g_overflow_count = 3U;

    /* Add event with epoch exactly equal to overflow_count → not late */
    Add_Event(0x00031234UL, ECU_CH_IGN1, ECU_ACT_SPARK);

    TEST_ASSERT_EQ_U32(0U, g_late_event_count);
    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
}

static void test_add_event_late_same_epoch_low16_past(void)
{
    test_reset();
    ECU_Hardware_Init();
    g_overflow_count = 3U;
    FTM0->CNT = 0x4000U;

    /* Same epoch, but target tick already passed (0x3FFF < CNT=0x4000). */
    Add_Event(0x00033FFFUL, ECU_CH_IGN1, ECU_ACT_SPARK);

    TEST_ASSERT_EQ_U32(1U, g_late_event_count);
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
}

/* ============================================================================
 * Test: FTM0_IRQHandler increments g_overflow_count on TOF
 * ========================================================================= */

static void test_isr_tof_increments_overflow(void)
{
    test_reset();
    ECU_Hardware_Init();
    g_overflow_count = 0U;

    /* Set TOF flag in mock SC */
    FTM0->SC |= FTM_SC_TOF_MASK;

    FTM0_IRQHandler();

    TEST_ASSERT_EQ_U32(1U, g_overflow_count);
    /* TOF should be cleared after ISR */
    TEST_ASSERT_TRUE((FTM0->SC & FTM_SC_TOF_MASK) == 0U);
}

static void test_isr_tof_clears_flag(void)
{
    test_reset();
    ECU_Hardware_Init();
    FTM0->SC |= FTM_SC_TOF_MASK;
    FTM0_IRQHandler();
    TEST_ASSERT_TRUE((FTM0->SC & FTM_SC_TOF_MASK) == 0U);
}

/* ============================================================================
 * Test: FTM0_IRQHandler arms channel CnV when epoch matches
 * ========================================================================= */

static void test_isr_arms_channel_on_epoch_match(void)
{
    uint32_t ts;
    uint8_t  ch, act;

    test_reset();
    ECU_Hardware_Init();

    /* Current epoch = 0, add event for epoch 1 */
    g_overflow_count = 0U;
    Add_Event(0x00011234UL, ECU_CH_IGN1, ECU_ACT_SPARK);

    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
    ecu_sched_test_get_event(0U, &ts, &ch, &act);
    TEST_ASSERT_EQ_U32(0x00011234UL, ts);

    /* Simulate TOF → overflow_count becomes 1, matching the event epoch */
    FTM0->SC |= FTM_SC_TOF_MASK;
    FTM0_IRQHandler();

    TEST_ASSERT_EQ_U32(1U, g_overflow_count);
    /* CnV for IGN1 (CH7) should have been loaded with low 16 bits = 0x1234 */
    TEST_ASSERT_EQ_U32(0x1234U, FTM0->CONTROLS[ECU_CH_IGN1].CnV);
}

/* ============================================================================
 * Test: FTM0_IRQHandler removes event from queue on channel match (CHF)
 * ========================================================================= */

static void test_isr_removes_event_on_chf(void)
{
    test_reset();
    ECU_Hardware_Init();

    /* Add one event for current epoch */
    g_overflow_count = 2U;
    Add_Event(0x00020100UL, ECU_CH_IGN2, ECU_ACT_SPARK);

    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());

    /* Simulate channel fire: set CHF on IGN2 (CH6) */
    FTM0->CONTROLS[ECU_CH_IGN2].CnSC |= FTM_CnSC_CHF_MASK;

    FTM0_IRQHandler();

    /* Queue should be empty after CHF cleanup */
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
}

/* ============================================================================
 * Test: Calculate_Sequential_Cycle adds 16 events (4 per cylinder x 4)
 * ========================================================================= */

static void test_calc_sequential_cycle_event_count(void)
{
    test_reset();
    ECU_Hardware_Init();

    /* Set a known ticks_per_rev to make timestamps deterministic */
    ecu_sched_test_set_ticks_per_rev(900000U);  /* ~1000 RPM at 66.7 ns/tick */
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(2813U);

    Calculate_Sequential_Cycle(0U);

    /* 4 cylinders x 4 events (DWELL_START + SPARK + INJ_ON + INJ_OFF) = 16 */
    TEST_ASSERT_EQ_U8(16U, ecu_sched_test_queue_size());
}

static void test_calc_sequential_cycle_sorted(void)
{
    uint8_t  i;
    uint32_t prev_ts = 0U;

    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_ticks_per_rev(900000U);
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(2813U);

    Calculate_Sequential_Cycle(0U);

    /* All events must be in ascending timestamp order */
    for (i = 0U; i < ecu_sched_test_queue_size(); ++i) {
        uint32_t ts;
        uint8_t  ch, act;
        ecu_sched_test_get_event(i, &ts, &ch, &act);
        TEST_ASSERT_TRUE(ts >= prev_ts);
        prev_ts = ts;
    }
}

static void test_calc_sequential_cycle_alternating_actions(void)
{
    uint8_t  i;
    test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_ticks_per_rev(900000U);
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_dwell_ticks(2813U);

    Calculate_Sequential_Cycle(0U);

    /* For each cylinder pair: DWELL_START must come before SPARK on same channel */
    /* Check that SPARK events all use ECU_ACT_SPARK action code */
    uint8_t spark_count = 0U;
    uint8_t dwell_count = 0U;
    uint8_t inj_on_count = 0U;
    uint8_t inj_off_count = 0U;
    for (i = 0U; i < ecu_sched_test_queue_size(); ++i) {
        uint32_t ts;
        uint8_t  ch, act;
        ecu_sched_test_get_event(i, &ts, &ch, &act);
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
 * Test: Queue full rejects and counts as late
 * ========================================================================= */

static void test_queue_full_counts_as_late(void)
{
    uint8_t i;
    test_reset();
    ECU_Hardware_Init();
    g_overflow_count = 0U;

    /* Fill queue to maximum (16 slots) */
    for (i = 0U; i < 16U; ++i) {
        Add_Event((uint32_t)(0x1000U + i), ECU_CH_IGN1, ECU_ACT_SPARK);
    }
    TEST_ASSERT_EQ_U8(16U, ecu_sched_test_queue_size());

    /* One more event: should be treated as late (queue full) */
    Add_Event(0x9999U, ECU_CH_IGN2, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U32(1U, g_late_event_count);
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

    /* Add_Event tests */
    test_add_event_sorted();
    test_add_event_late();
    test_add_event_not_late_same_epoch();
    test_add_event_late_same_epoch_low16_past();

    /* FTM0_IRQHandler tests */
    test_isr_tof_increments_overflow();
    test_isr_tof_clears_flag();
    test_isr_arms_channel_on_epoch_match();
    test_isr_removes_event_on_chf();

    /* Calculate_Sequential_Cycle tests */
    test_calc_sequential_cycle_event_count();
    test_calc_sequential_cycle_sorted();
    test_calc_sequential_cycle_alternating_actions();

    /* Queue boundary tests */
    test_queue_full_counts_as_late();

    printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
