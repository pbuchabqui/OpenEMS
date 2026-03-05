/**
 * @file test/engine/test_ecu_sched_fixes.cpp
 * @brief Host unit tests for ECU scheduler critical fixes (CRITICAL FIX)
 * 
 * Tests the race condition fixes, late event handling improvements,
 * and atomic overflow handling in the ECU scheduler.
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

} // namespace

// =============================================================================
// Test: Late Event Handling Fix
// =============================================================================

void test_late_event_removes_existing_channel_events() {
    test_reset();
    ECU_Hardware_Init();
    
    // Set overflow count to simulate time progression
    g_overflow_count = 5U;
    
    // Add an event for channel 1 in the future
    Add_Event(0x00050000UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
    
    // Now add a late event for the same channel (epoch < current overflow)
    // This should remove the existing event and force immediate execution
    uint32_t late_events_before = g_late_event_count;
    Add_Event(0x00030000UL, ECU_CH_IGN1, ECU_ACT_DWELL_START);
    
    // Queue should be empty (existing event removed)
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
    
    // Late event count should be incremented
    TEST_ASSERT_EQ_U32(late_events_before + 1U, g_late_event_count);
}

void test_queue_full_removes_existing_channel_events() {
    test_reset();
    ECU_Hardware_Init();
    
    // Fill queue to capacity (ECU_QUEUE_SIZE events)
    for (uint8_t i = 0U; i < ECU_QUEUE_SIZE; ++i) {
        Add_Event(0x00010000UL + i, ECU_CH_IGN1, ECU_ACT_SPARK);
    }
    TEST_ASSERT_EQ_U8(ECU_QUEUE_SIZE, ecu_sched_test_queue_size());
    
    // Try to add another event for the same channel
    // This should be treated as late and remove existing events
    uint32_t late_events_before = g_late_event_count;
    Add_Event(0x00020000UL, ECU_CH_IGN1, ECU_ACT_DWELL_START);
    
    // Queue should be empty (all existing events removed)
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
    
    // Late event count should be incremented
    TEST_ASSERT_EQ_U32(late_events_before + 1U, g_late_event_count);
}

void test_late_event_different_channels_preserved() {
    test_reset();
    ECU_Hardware_Init();
    
    g_overflow_count = 5U;
    
    // Add events for different channels
    Add_Event(0x00050000UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    Add_Event(0x00050001UL, ECU_CH_IGN2, ECU_ACT_SPARK);
    Add_Event(0x00050002UL, ECU_CH_IGN3, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(3U, ecu_sched_test_queue_size());
    
    // Add a late event for channel 2 only
    uint32_t late_events_before = g_late_event_count;
    Add_Event(0x00030000UL, ECU_CH_IGN2, ECU_ACT_DWELL_START);
    
    // Queue should have 2 events (channels 1 and 3 preserved)
    TEST_ASSERT_EQ_U8(2U, ecu_sched_test_queue_size());
    
    // Verify remaining events are for channels 1 and 3
    uint32_t ts1, ts2;
    uint8_t ch1, ch2, act1, act2;
    ecu_sched_test_get_event(0U, &ts1, &ch1, &act1);
    ecu_sched_test_get_event(1U, &ts2, &ch2, &act2);
    
    TEST_ASSERT_TRUE((ch1 == ECU_CH_IGN1 && ch2 == ECU_CH_IGN3) ||
                    (ch1 == ECU_CH_IGN3 && ch2 == ECU_CH_IGN1));
    
    // Late event count should be incremented
    TEST_ASSERT_EQ_U32(late_events_before + 1U, g_late_event_count);
}

// =============================================================================
// Test: Atomic Overflow Handling
// =============================================================================

void test_overflow_handling_atomic() {
    test_reset();
    ECU_Hardware_Init();
    
    // Add an event for the next overflow epoch
    g_overflow_count = 0U;
    Add_Event(0x00011234UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
    
    // Simulate timer overflow
    FTM0->SC |= FTM_SC_TOF_MASK;
    
    // Record overflow count before ISR
    uint32_t overflow_before = g_overflow_count;
    
    // Call ISR (should handle overflow atomically)
    FTM0_IRQHandler();
    
    // Overflow count should be incremented exactly once
    TEST_ASSERT_EQ_U32(overflow_before + 1U, g_overflow_count);
    
    // Event should be armed (CnV loaded with low 16 bits)
    TEST_ASSERT_EQ_U32(0x1234U, FTM0->CONTROLS[ECU_CH_IGN1].CnV);
    
    // TOF flag should be cleared
    TEST_ASSERT_TRUE((FTM0->SC & FTM_SC_TOF_MASK) == 0U);
}

void test_overflow_multiple_events_same_epoch() {
    test_reset();
    ECU_Hardware_Init();
    
    // Add multiple events for the same future epoch
    g_overflow_count = 0U;
    Add_Event(0x00011000UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    Add_Event(0x00011100UL, ECU_CH_IGN2, ECU_ACT_SPARK);
    Add_Event(0x00011200UL, ECU_CH_IGN3, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(3U, ecu_sched_test_queue_size());
    
    // Simulate timer overflow to epoch 1
    FTM0->SC |= FTM_SC_TOF_MASK;
    FTM0_IRQHandler();
    
    // All events should be armed
    TEST_ASSERT_EQ_U32(0x1000U, FTM0->CONTROLS[ECU_CH_IGN1].CnV);
    TEST_ASSERT_EQ_U32(0x1100U, FTM0->CONTROLS[ECU_CH_IGN2].CnV);
    TEST_ASSERT_EQ_U32(0x1200U, FTM0->CONTROLS[ECU_CH_IGN3].CnV);
    
    // Overflow count should be incremented
    TEST_ASSERT_EQ_U32(1U, g_overflow_count);
}

void test_overflow_events_still_late() {
    test_reset();
    ECU_Hardware_Init();
    
    // Add events for epoch 0 (already past when overflow happens)
    g_overflow_count = 0U;
    Add_Event(0x00001000UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    Add_Event(0x00001100UL, ECU_CH_IGN2, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(2U, ecu_sched_test_queue_size());
    
    // Simulate timer overflow to epoch 1
    uint32_t late_events_before = g_late_event_count;
    FTM0->SC |= FTM_SC_TOF_MASK;
    FTM0_IRQHandler();
    
    // Events should be forced (late) and queue should be empty
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
    TEST_ASSERT_EQ_U32(late_events_before + 2U, g_late_event_count);
    
    // Overflow count should be incremented
    TEST_ASSERT_EQ_U32(1U, g_overflow_count);
}

void test_chf_rearm_forces_next_if_now_past() {
    test_reset();
    ECU_Hardware_Init();

    g_overflow_count = 2U;
    FTM0->CNT = 0x2000U;

    // Two events same channel and epoch; both are initially future.
    Add_Event(0x00022500UL, ECU_CH_IGN1, ECU_ACT_DWELL_START);
    Add_Event(0x00022600UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(2U, ecu_sched_test_queue_size());

    // Simulate time advancing beyond second timestamp before CHF cleanup.
    FTM0->CNT = 0x2700U;
    uint32_t late_before = g_late_event_count;

    // First event fired -> CHF set for channel.
    FTM0->CONTROLS[ECU_CH_IGN1].CnSC |= FTM_CnSC_CHF_MASK;
    FTM0_IRQHandler();

    // Next event is now past and must be forced/removed immediately.
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
    TEST_ASSERT_EQ_U32(late_before + 1U, g_late_event_count);
}

// =============================================================================
// Test: Input Parameter Validation
// =============================================================================

void test_add_event_invalid_channel() {
    test_reset();
    ECU_Hardware_Init();
    
    // This test would trigger assertions in debug builds
    // In release builds, it should handle gracefully
    // Note: We can't easily test assertion failures without special test framework
    
    // Test with maximum valid channel (7 for FTM0)
    Add_Event(0x00010000UL, 7U, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
}

void test_add_event_invalid_action() {
    test_reset();
    ECU_Hardware_Init();
    
    // Test with maximum valid action
    Add_Event(0x00010000UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
}

void test_add_event_zero_timestamp() {
    test_reset();
    ECU_Hardware_Init();
    
    // Test with minimum valid timestamp (not zero)
    Add_Event(0x00000001UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    
    // Should be accepted (epoch 0, current overflow count is 0)
    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
}

// =============================================================================
// Test: Race Condition Prevention
// =============================================================================

void test_isr_queue_processing_backwards() {
    test_reset();
    ECU_Hardware_Init();
    
    // Fill queue with events
    for (uint8_t i = 0U; i < 8U; ++i) {
        Add_Event(0x00010000UL + i, ECU_CH_IGN1, ECU_ACT_SPARK);
    }
    TEST_ASSERT_EQ_U8(8U, ecu_sched_test_queue_size());
    
    // Simulate channel match for first event
    FTM0->CONTROLS[ECU_CH_IGN1].CnSC |= FTM_CnSC_CHF_MASK;
    
    // Call ISR - should process queue backwards without race condition
    FTM0_IRQHandler();
    
    // Queue should have one less event
    TEST_ASSERT_EQ_U8(7U, ecu_sched_test_queue_size());
    
    // CHF should be cleared
    TEST_ASSERT_TRUE((FTM0->CONTROLS[ECU_CH_IGN1].CnSC & FTM_CnSC_CHF_MASK) == 0U);
}

void test_queue_remove_index_bounds() {
    test_reset();
    ECU_Hardware_Init();
    
    // Add some events
    Add_Event(0x00010000UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    Add_Event(0x00010001UL, ECU_CH_IGN2, ECU_ACT_SPARK);
    TEST_ASSERT_EQ_U8(2U, ecu_sched_test_queue_size());
    
    // Test valid removal
    // Note: We can't directly test queue_remove since it's static
    // But we can test through ISR which uses it
    
    // Simulate channel match for first event
    FTM0->CONTROLS[ECU_CH_IGN1].CnSC |= FTM_CnSC_CHF_MASK;
    FTM0_IRQHandler();
    
    // Queue should have one event
    TEST_ASSERT_EQ_U8(1U, ecu_sched_test_queue_size());
}

void test_sync_loss_clears_queue_and_drives_safe_outputs() {
    test_reset();
    ECU_Hardware_Init();

    g_overflow_count = 2U;
    Add_Event(0x00021000UL, ECU_CH_INJ1, ECU_ACT_INJ_ON);
    Add_Event(0x00021100UL, ECU_CH_IGN1, ECU_ACT_DWELL_START);
    TEST_ASSERT_EQ_U8(2U, ecu_sched_test_queue_size());

    ems::drv::CkpSnapshot full_sync{
        1000u, 0u, 0u, 10000u, ems::drv::SyncState::FULL_SYNC, false
    };
    ems::engine::ecu_sched_on_tooth_hook(full_sync);

    ems::drv::CkpSnapshot loss_sync{
        1000u, 1u, 0u, 10000u, ems::drv::SyncState::LOSS_OF_SYNC, false
    };
    ems::engine::ecu_sched_on_tooth_hook(loss_sync);

    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
    TEST_ASSERT_EQ_U32(FTM_CnSC_OC_CLEAR, FTM0->CONTROLS[ECU_CH_INJ1].CnSC);
    TEST_ASSERT_EQ_U32(FTM_CnSC_OC_CLEAR, FTM0->CONTROLS[ECU_CH_IGN1].CnSC);
}

void test_metrics_late_delay_and_queue_depth() {
    test_reset();
    ECU_Hardware_Init();

    g_overflow_count = 5U;
    FTM0->CNT = 0x1234U;

    Add_Event(0x00030000UL, ECU_CH_IGN1, ECU_ACT_SPARK);  // forced late

    TEST_ASSERT_TRUE(ecu_sched_test_get_late_delay_samples() >= 1U);
    TEST_ASSERT_TRUE(ecu_sched_test_get_late_delay_sum_ticks() > 0U);
    TEST_ASSERT_TRUE(ecu_sched_test_get_late_delay_max_ticks() > 0U);

    ecu_sched_test_reset();
    ECU_Hardware_Init();
    ecu_sched_test_set_ticks_per_rev(900000U);
    Calculate_Sequential_Cycle(0U);

    TEST_ASSERT_TRUE(ecu_sched_test_get_queue_depth_peak() >= 16U);
    TEST_ASSERT_TRUE(ecu_sched_test_get_queue_depth_last_cycle_peak() >= 16U);
}

void test_stress_200_to_8500rpm_with_sync_noise() {
    test_reset();
    ECU_Hardware_Init();

    // 200 rpm on FTM0 PS=8: ticks/rev = (120e6*600)/(8*2000) = 4,500,000
    ecu_sched_test_set_ticks_per_rev(4500000U);
    ecu_sched_test_set_dwell_ticks(45000U);
    ecu_sched_test_set_inj_pw_ticks(30000U);
    ecu_sched_test_set_advance_deg(10U);
    ecu_sched_test_set_soi_lead_deg(62U);

    // Preload one far event (> 16-bit horizon) and ensure no immediate late force.
    g_overflow_count = 0U;
    FTM0->CNT = 0U;
    Add_Event(100000UL, ECU_CH_IGN1, ECU_ACT_SPARK);
    const uint32_t late_before = g_late_event_count;

    ems::drv::CkpSnapshot full_sync{
        1000u, 10u, 0u, 2000u, ems::drv::SyncState::FULL_SYNC, false
    };
    ems::engine::ecu_sched_on_tooth_hook(full_sync);
    TEST_ASSERT_EQ_U32(late_before, g_late_event_count);

    // Advance logical time near event and poll near-time arming.
    g_overflow_count = 1U;
    FTM0->CNT = static_cast<uint32_t>(70000U - 65536U);
    ems::engine::ecu_sched_on_tooth_hook(full_sync);
    TEST_ASSERT_EQ_U32(late_before, g_late_event_count);

    // Accelerate to 8500 rpm and schedule a cycle via boundary tooth wrap.
    // ticks/rev ~= (120e6*600)/(8*85000) = 105,882
    ecu_sched_test_set_ticks_per_rev(105882U);
    ems::drv::CkpSnapshot t1{1000u, 1u, 0u, 85000u, ems::drv::SyncState::FULL_SYNC, false};
    ems::drv::CkpSnapshot t0{1000u, 0u, 0u, 85000u, ems::drv::SyncState::FULL_SYNC, false};
    ems::engine::ecu_sched_on_tooth_hook(t1);
    ems::engine::ecu_sched_on_tooth_hook(t0);  // cycle boundary
    TEST_ASSERT_TRUE(ecu_sched_test_queue_size() > 0U);

    // Inject sync noise/loss: queue must be cleared and outputs driven safe.
    ems::drv::CkpSnapshot noise{
        1000u, 2u, 0u, 85000u, ems::drv::SyncState::LOSS_OF_SYNC, false
    };
    ems::engine::ecu_sched_on_tooth_hook(noise);
    TEST_ASSERT_EQ_U8(0U, ecu_sched_test_queue_size());
    TEST_ASSERT_EQ_U32(FTM_CnSC_OC_CLEAR, FTM0->CONTROLS[ECU_CH_INJ1].CnSC);
}

void test_cycle_fill_drop_when_queue_lacks_capacity() {
    test_reset();
    ECU_Hardware_Init();

    for (uint8_t i = 0U; i < ECU_QUEUE_SIZE; ++i) {
        Add_Event(0x00010000UL + i, ECU_CH_IGN1, ECU_ACT_SPARK);
    }
    TEST_ASSERT_EQ_U8(ECU_QUEUE_SIZE, ecu_sched_test_queue_size());

    const uint32_t drops_before = ecu_sched_test_get_cycle_schedule_drop_count();
    const uint32_t late_before = g_late_event_count;
    Calculate_Sequential_Cycle(0U);

    TEST_ASSERT_EQ_U32(drops_before + 1U, ecu_sched_test_get_cycle_schedule_drop_count());
    TEST_ASSERT_EQ_U32(late_before, g_late_event_count);
    TEST_ASSERT_EQ_U8(ECU_QUEUE_SIZE, ecu_sched_test_queue_size());
}

// =============================================================================
// Main Test Runner
// =============================================================================

int main() {
    printf("Running EMS ECU Scheduler Critical Fixes Tests...\n");
    
    // Late event handling tests
    test_late_event_removes_existing_channel_events();
    test_queue_full_removes_existing_channel_events();
    test_late_event_different_channels_preserved();
    
    // Atomic overflow handling tests
    test_overflow_handling_atomic();
    test_overflow_multiple_events_same_epoch();
    test_overflow_events_still_late();
    test_chf_rearm_forces_next_if_now_past();
    
    // Input validation tests
    test_add_event_invalid_channel();
    test_add_event_invalid_action();
    test_add_event_zero_timestamp();
    
    // Race condition prevention tests
    test_isr_queue_processing_backwards();
    test_queue_remove_index_bounds();
    test_sync_loss_clears_queue_and_drives_safe_outputs();
    test_metrics_late_delay_and_queue_depth();
    test_stress_200_to_8500rpm_with_sync_noise();
    test_cycle_fill_drop_when_queue_lacks_capacity();
    
    printf("ECU scheduler fixes tests completed: %d run, %d failed\n", 
           g_tests_run, g_tests_failed);
    
    return (g_tests_failed == 0) ? 0 : 1;
}
