#pragma once

#include <cstdint>

#include "drv/ckp.h"
#include "drv/sensors.h"
#include "app/status_bits.h"

namespace ems::app {

// Valor de lambda fixo retornado em caso de timeout do WBO2 (λ = 1.05)
static constexpr uint16_t WBO2_SAFE_LAMBDA_MILLI = 1050u;

void can_stack_init(uint16_t wbo2_rx_id = 0x180u) noexcept;
void can_stack_set_wbo2_rx_id(uint16_t id) noexcept;

void can_stack_process(uint32_t now_ms,
                       const ems::drv::CkpSnapshot& ckp,
                       const ems::drv::SensorData& sensors,
                       int8_t  advance_deg,
                       uint8_t pw_ms_x10,
                       int8_t  stft_pct,
                       uint8_t vvt_intake_pct,
                       uint8_t vvt_exhaust_pct,
                       uint8_t status_bits) noexcept;

// Lambda × 1000 do último frame WBO2 recebido (raw, sem fallback)
uint16_t can_stack_lambda_milli() noexcept;

// Lambda × 1000 seguro: retorna WBO2_SAFE_LAMBDA_MILLI se sensor offline
// Sinaliza automaticamente STATUS_WBO2_FAULT via can_stack_wbo2_fault()
uint16_t can_stack_lambda_milli_safe(uint32_t now_ms) noexcept;

// true se WBO2 transmitiu dentro dos últimos 500 ms
bool can_stack_wbo2_fresh(uint32_t now_ms) noexcept;

// true se o último status_bits transmitido continha STATUS_WBO2_FAULT
bool can_stack_wbo2_fault() noexcept;

uint8_t can_stack_wbo2_status() noexcept;

#if defined(EMS_HOST_TEST)
void can_stack_test_reset() noexcept;
#endif

} // namespace ems::app
