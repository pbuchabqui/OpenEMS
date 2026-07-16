#include "test/harness.h"
#include "test/fixtures.h"
#include "test/ui_helpers.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "engine/etb_control.h"
#include "hal/etb_driver.h"
#include "engine/torque_manager.h"
#include "engine/calibration.h"
#include "app/can_rx_map.h"
#include "hal/adc.h"
#include "hal/system.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/auxiliaries.h"
#include "engine/knock.h"
#include "engine/table3d.h"
#include "engine/ecu_sched.h"
#include "engine/quick_crank.h"
#include "engine/transient_fuel.h"
#include "engine/map_estimator.h"
#include "engine/misfire_detect.h"
#include "engine/diagnostic_manager.h"
#include "engine/xtau_autocalib.h"
#include "engine/output_test.h"
#include "engine/engine_config.h"
#include "hal/timer.h"
#include "hal/flash.h"
#include "app/ui_protocol.h"
#include "app/status_bits.h"
#include "hal/crc32.h"

namespace ems::engine {
    int16_t etb_get_idle_spark_trim() noexcept;
}

extern volatile uint32_t ems_test_tim5_ccr1;
extern volatile uint32_t ems_test_tim5_ccr2;
extern volatile uint32_t ems_test_cam_gpio_idr;

using namespace ems::drv;
using namespace ems::engine;
using namespace ems::app;
using namespace ems::hal;

void test_timer_stubs(void) {
    section("timer HAL: all stubs execute without crash");
    using namespace ems::hal;
    tim5_ic_init();
    const uint32_t cnt = tim5_count();
    CHECK_EQ(cnt, 0u, "tim5_count() returns mock value (0)");
    tim3_pwm_init(15u);
    tim3_set_duty(0u, 500u);
    tim3_set_duty(1u, 250u);
    tim4_pwm_init(15u);
    tim4_set_duty(0u, 750u);
    tim4_set_duty(1u, 1000u);
    tim15_etb_pwm_init(20000u);
    tim15_etb_set_duty_x10(500u);
    CHECK_TRUE(true, "all timer stubs: no crash");
}

// ============================================================================
// ECU SCHED — FASE 2 (arm_channel, CCR, late events, dwell watchdog, presync)
// ============================================================================

