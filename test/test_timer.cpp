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
#include "hal/out_pins.h"
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
    etb_pwm_init(20000u);
    etb_pwm_set_duty_x10(500u);
    CHECK_TRUE(true, "all timer stubs: no crash");
}

void test_out_pins_bsrr_rgt6(void) {
    section("out_pins: RGT6 BSRR polarity (INJ1=PA15, IGN1=PC6, safe-early LOW)");
    using namespace ems::hal;
    out_pins_test_reset_stubs();

    // INJ1 = ECU_CH 2 → PA15 set bit
    out_pin_write(ECU_CH_INJ1, 1u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(0u) & (1u << 15u), (1u << 15u),
             "INJ1 high → GPIOA BSRR set pin 15");
    out_pin_write(ECU_CH_INJ1, 0u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(0u) & (1u << (15u + 16u)),
             (1u << (15u + 16u)), "INJ1 low → GPIOA BSRR reset pin 15");

    // INJ2 = ECU_CH 3 → PB3
    out_pin_write(ECU_CH_INJ2, 1u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(1u) & (1u << 3u), (1u << 3u),
             "INJ2 high → GPIOB BSRR set pin 3");

    // IGN1 = ECU_CH 7 → PC6
    out_pin_write(ECU_CH_IGN1, 1u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(2u) & (1u << 6u), (1u << 6u),
             "IGN1 high → GPIOC BSRR set pin 6");
    out_pin_write(ECU_CH_IGN1, 0u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(2u) & (1u << (6u + 16u)),
             (1u << (6u + 16u)), "IGN1 low → GPIOC BSRR reset pin 6");

    // Safe-early: all active-high outputs de-asserted (reset bits written)
    out_pins_test_reset_stubs();
    out_pins_hw_init();
    const uint32_t a = out_pins_test_bsrr_snapshot(0u);
    const uint32_t b = out_pins_test_bsrr_snapshot(1u);
    const uint32_t c = out_pins_test_bsrr_snapshot(2u);
    CHECK_TRUE((a & (1u << (15u + 16u))) != 0u, "safe-early: PA15 reset");
    CHECK_TRUE((b & (1u << (3u + 16u))) != 0u, "safe-early: PB3 reset");
    CHECK_TRUE((c & (1u << (6u + 16u))) != 0u, "safe-early: PC6 reset");
    CHECK_TRUE((c & (1u << (10u + 16u))) != 0u, "safe-early: PC10 reset (INJ3)");
}

// ============================================================================
// ECU SCHED — FASE 2 (arm_channel, CCR, late events, dwell watchdog, presync)
// ============================================================================

