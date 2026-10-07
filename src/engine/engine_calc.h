#pragma once

#include <cstdint>

#include "drv/ckp.h"
#include "drv/sensors.h"

namespace ems::engine {

// One 2 ms engine-control step: MAP fusion, protections (limp gating),
// quick-crank, fuel (VE / trims / AE / X-tau / dP / dead time), advance and
// dwell, then the commit to the scheduler (ecu_sched) and the prime pulse.
// main_stm32.cpp only reads the sensors, calls this and publishes telemetry;
// the host tests and the virtual engine drive the same function.
struct EngineCalcIn {
    uint32_t now_ms;
    ems::drv::CkpSnapshot snap;
    ems::drv::SensorData sensors;
    uint16_t lambda_x1000;          // wideband (CAN), lambda x1000
    bool lambda_valid;              // wideband frame fresh
    int16_t torque_spark_retard_deg;  // TC / launch
};

struct EngineCalcOut {
    bool committed;          // a commit reached the scheduler this step
    int16_t spark_x10;       // last advance committed (0.1 deg BTDC)
    uint32_t dwell_ticks;    // last dwell committed
    uint32_t inj_pw_ticks;   // last per-opening pulse committed
    uint8_t pw_ms_x10;       // telemetry: cycle injector time (0 under cut)
    uint32_t net_pw_us;      // telemetry: net flow time before dP/dead time
    uint16_t map_fused_x100; // MAP used for fuel this step
    bool limp_active;        // sensor limp (MAP/CLT/oil/overtemp)
    uint32_t rpm_max_x10;    // highest rpm seen (diagnostics)
};

const EngineCalcOut& engine_calc_step(const EngineCalcIn& in) noexcept;
void engine_calc_reset() noexcept;

}  // namespace ems::engine
