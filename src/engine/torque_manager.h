/**
 * @file torque_manager.h
 * @brief Gerenciador Central de Torque
 *
 * Path de PRODUÇÃO:
 *   ems::engine::torque_manager_update — pedal map, limp, rev soft, launch,
 *   TC, idle air, dashpot, rate-limit ETB → TorqueOutput.
 *
 * (O modelo float de host, torque_manager_loop, foi removido: era um protótipo divergente da produção.)
 *
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool torque_manager_init(void);
void torque_manager_enter_limp(void);
bool torque_manager_is_ready(void);

#ifdef __cplusplus
}

#include "drv/ckp.h"
#include "drv/sensors.h"

namespace ems::engine {

constexpr uint8_t TORQUE_LIMP_MAP_CLT   = (1u << 0u);
constexpr uint8_t TORQUE_LIMP_APP_FAULT = (1u << 1u);
constexpr uint8_t TORQUE_LIMP_ETB_FAULT = (1u << 2u);
constexpr uint8_t TORQUE_LIMP_NO_CALIB  = (1u << 3u);
constexpr uint8_t TORQUE_LIMP_REV_CUT   = (1u << 4u);
constexpr uint8_t TORQUE_ACTIVE_LAUNCH  = (1u << 5u);
constexpr uint8_t TORQUE_ACTIVE_TC      = (1u << 6u);

struct TorqueOutput {
    uint16_t etb_target_pct_x10;
    uint16_t etb_max_rate_pct_per_s;
    uint8_t  limp_reason;           // limp bits + ACTIVE_LAUNCH / ACTIVE_TC
    bool     etb_enable_request;
    int16_t  spark_retard_deg;      // ≥0 degrees of retard for TC/launch
    uint16_t tc_reduction_pct_x10;  // 0–1000 applied throttle cut
    uint8_t  launch_active;         // 1 while launch state machine is ACTIVE
    uint8_t  tc_active;             // 1 while TC reduction > 0
};

void         torque_manager_reset() noexcept;
TorqueOutput torque_manager_update(
    const ems::drv::CkpSnapshot& snap,
    const ems::drv::SensorData&  sensors,
    bool key_on, bool map_clt_limp, bool rev_cut,
    uint16_t idle_target_rpm_x10, uint16_t period_ms) noexcept;

// Runtime hooks (tests / future CAN)
void torque_tc_set_external_slip_pct_x10(uint16_t slip_pct_x10) noexcept;
void torque_tc_clear_external_slip() noexcept;
// Carga de ar-condicionado pedida por fora (não há sensor do compressor).
void torque_idle_set_ac_load(uint8_t on) noexcept;
void torque_launch_force_enable(uint8_t on) noexcept;  // 0=use cal, 1=force on, 2=force off

// Latches de telemetria (OCH / status bits / host tests)
uint16_t torque_manager_get_target() noexcept;
uint8_t  torque_manager_get_limp_reason() noexcept;
uint8_t  torque_manager_get_launch_active() noexcept;
uint16_t torque_manager_get_tc_reduction() noexcept;
int16_t  torque_manager_get_spark_retard() noexcept;
uint16_t torque_manager_get_vehicle_kmh() noexcept;
uint16_t torque_manager_get_wheel_kmh() noexcept;

// Aliases legados (mvp_bench_tests)
inline uint16_t torque_manager_test_get_target() noexcept {
    return torque_manager_get_target();
}
inline uint8_t torque_manager_test_get_limp_reason() noexcept {
    return torque_manager_get_limp_reason();
}
inline uint8_t torque_manager_test_get_launch_active() noexcept {
    return torque_manager_get_launch_active();
}
inline uint16_t torque_manager_test_get_tc_reduction() noexcept {
    return torque_manager_get_tc_reduction();
}
inline uint16_t torque_manager_test_get_vehicle_kmh() noexcept {
    return torque_manager_get_vehicle_kmh();
}
inline uint16_t torque_manager_test_get_wheel_kmh() noexcept {
    return torque_manager_get_wheel_kmh();
}

}  // namespace ems::engine

#endif  // __cplusplus
