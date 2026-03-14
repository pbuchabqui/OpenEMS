#pragma once

#include <cstdint>

namespace ems::engine {

void auxiliaries_init() noexcept;
void auxiliaries_set_key_on(bool key_on) noexcept;
void auxiliaries_tick_10ms() noexcept;
void auxiliaries_tick_20ms() noexcept;

#if defined(EMS_HOST_TEST)
void auxiliaries_test_reset() noexcept;
uint16_t auxiliaries_test_get_iac_duty() noexcept;
uint16_t auxiliaries_test_get_wg_duty() noexcept;
uint16_t auxiliaries_test_get_vvt_esc_duty() noexcept;
uint16_t auxiliaries_test_get_vvt_adm_duty() noexcept;
bool auxiliaries_test_get_fan_state() noexcept;
bool auxiliaries_test_get_pump_state() noexcept;
bool auxiliaries_test_get_wg_failsafe() noexcept;
#endif

}  // namespace ems::engine
