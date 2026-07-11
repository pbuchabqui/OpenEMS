#pragma once

#include <cstdint>

namespace ems::engine {

void auxiliaries_init() noexcept;
void auxiliaries_set_key_on(bool key_on) noexcept;
void auxiliaries_tick_10ms() noexcept;
void auxiliaries_tick_20ms() noexcept;
uint16_t auxiliaries_idle_target_rpm_x10(int16_t clt_x10) noexcept;
uint16_t auxiliaries_ewg_position_demand_x10() noexcept;

// Teste de saídas em bancada: forçamento directo dos relés (só faz sentido
// com output_test activo — os ticks automáticos ficam suspensos nesse modo).
void auxiliaries_force_pump(bool on) noexcept;
void auxiliaries_force_fan(bool on) noexcept;

#if defined(EMS_HOST_TEST)
void auxiliaries_test_reset() noexcept;
uint16_t auxiliaries_test_get_wg_duty() noexcept;
uint16_t auxiliaries_test_get_vvt_esc_duty() noexcept;
uint16_t auxiliaries_test_get_vvt_adm_duty() noexcept;
bool auxiliaries_test_get_fan_state() noexcept;
bool auxiliaries_test_get_pump_state() noexcept;
bool auxiliaries_test_get_wg_failsafe() noexcept;
#endif

}  // namespace ems::engine
