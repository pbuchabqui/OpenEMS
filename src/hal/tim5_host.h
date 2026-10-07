#pragma once
/**
 * Host-test TIM5 register model — ONE simulated timer shared by the CKP/CMP
 * capture (CH1/CH2), the event dispatcher compare (CH3) and hal::tim5_count().
 * Real hardware has a single TIM5; the host build must too, otherwise tests
 * cannot see latency between a tooth capture and the scheduler's "now".
 */
#if defined(EMS_HOST_TEST)
#include <cstdint>

extern volatile uint32_t ems_test_tim5_cnt;
extern volatile uint32_t ems_test_tim5_ccr1;   // CKP capture
extern volatile uint32_t ems_test_tim5_ccr2;   // CMP capture
extern volatile uint32_t ems_test_tim5_ccr3;   // dispatcher compare
extern volatile uint32_t ems_test_tim5_sr;     // only CC3IF (bit 3) is modelled
extern volatile uint32_t ems_test_tim5_dier;
extern volatile uint32_t ems_test_cam_gpio_idr;
#endif
