#pragma once

#include <cstdint>

#include "drv/ckp.h"
#include "drv/sensors.h"

namespace ems::app {

void can_stack_init(uint16_t wbo2_rx_id = 0x180u) noexcept;
void can_stack_set_wbo2_rx_id(uint16_t id) noexcept;

void can_stack_process(uint32_t now_ms,
                       const ems::drv::CkpSnapshot& ckp,
                       const ems::drv::SensorData& sensors,
                       int8_t advance_deg,
                       uint8_t pw_ms_x10,
                       int8_t stft_pct,
                       uint8_t vvt_intake_pct,
                       uint8_t vvt_exhaust_pct,
                       uint8_t status_bits) noexcept;

uint16_t can_stack_lambda_milli() noexcept;
uint8_t can_stack_wbo2_status() noexcept;
bool can_stack_wbo2_fresh(uint32_t now_ms) noexcept;
uint16_t can_stack_o2_mv_effective(uint32_t now_ms, const ems::drv::SensorData& sensors) noexcept;

#if defined(EMS_HOST_TEST)
void can_stack_test_reset() noexcept;
#endif

}  // namespace ems::app

