/**
 * @file test/app/stub_ecu_sched_ivc.cpp
 * @brief Stub implementations of ecu_sched IVC functions for test_ts_protocol.
 *
 * test_ts_protocol links tuner_studio.cpp which calls ecu_sched_set_ivc() and
 * ecu_sched_ivc_clamp_count(). Providing stubs avoids pulling in the full
 * ecu_sched.cpp (which requires peripheral mocks).
 */

#define EMS_HOST_TEST 1

#include <stdint.h>

static uint8_t  g_stub_ivc_abdc_deg   = 50U;
static uint32_t g_stub_ivc_clamp_count = 0U;

extern "C" {

void ecu_sched_set_ivc(uint8_t ivc_abdc_deg)
{
    g_stub_ivc_abdc_deg = ivc_abdc_deg;
}

uint32_t ecu_sched_ivc_clamp_count(void)
{
    return g_stub_ivc_clamp_count;
}

}  /* extern "C" */
