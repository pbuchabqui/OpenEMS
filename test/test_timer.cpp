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
    tim4_pwm_init(15u);
    tim4_set_duty(0u, 750u);
    tim4_set_duty(1u, 1000u);
    etb_pwm_init(20000u);
    etb_pwm_set_duty_x10(500u);
    CHECK_TRUE(true, "all timer stubs: no crash");

    // MT6835/TIM2 encoder HAL stubs (VGT6-only real logic lives under
    // #ifndef EMS_HOST_TEST in stm32h562/timer.cpp — the ISR register
    // access itself isn't host-testable, same reasoning as
    // ecu_sched_encoder_* taking already-read values as plain params;
    // the pure catch-up decision inside the ISR IS testable, see
    // test_tim2_heartbeat_next_ccr4() below). This just confirms the
    // host-test mock layer (task #9) is wired: init/arm/heartbeat_start
    // don't crash, and the count getter/setter round-trips.
    tim5_freerun_init();
    tim2_encoder_init();
    CHECK_EQ(tim2_encoder_count(), 0u, "tim2_encoder_count() mock default 0");
    tim2_encoder_set_count(12345u);
    CHECK_EQ(tim2_encoder_count(), 12345u, "tim2_encoder_set_count/count round-trip");
    tim2_heartbeat_start();
    tim3_cmp_ic_init();
    CHECK_EQ(cmp_angle_snapshot(), 0u, "cmp_angle_snapshot() mock default 0");
    CHECK_EQ(cmp_edge_count(), 0u, "cmp_edge_count() mock default 0");
    CHECK_TRUE(true, "MT6835/TIM2 HAL stubs: no crash");
}

void test_tim2_heartbeat_next_ccr4(void) {
    section("timer HAL: tim2_heartbeat_next_ccr4() — TIM2_CH4 catch-up");
    using namespace ems::hal;

    // Caminho saudável: CCR4 (já incrementado +256 pelo chamador) continua
    // à frente de CNT — devolvido sem alteração, zero custo extra no
    // caminho nominal.
    CHECK_EQ(tim2_heartbeat_next_ccr4(1256u, 1000u), 1256u,
             "healthy: ccr4 ahead of cnt -> unchanged");
    CHECK_EQ(tim2_heartbeat_next_ccr4(256u, 0u), 256u,
             "healthy: ccr4 exactly at cnt+256 -> unchanged");

    // Fronteira: ccr4 == cnt não é "atrás" (delta assinado == 0, não < 0) —
    // fica tal como está, coerente com o resto do dispatcher (TIM2/TIM5 usam
    // sempre ">" ou "<" para o teste de "já passou", nunca ">=").
    CHECK_EQ(tim2_heartbeat_next_ccr4(1000u, 1000u), 1000u,
             "boundary: ccr4 == cnt -> unchanged (not treated as behind)");

    // Caminho degradado: IRQ atendida com atraso > 256 counts — o
    // incremento do chamador não foi suficiente para pôr CCR4 à frente de
    // CNT. Auto-recuperação: reancora em cnt+256, nunca deixa o comparador
    // preso atrás do contador (que só voltaria a disparar após um wrap
    // completo de 32 bits em modo encoder livre-corrente).
    CHECK_EQ(tim2_heartbeat_next_ccr4(1000u, 2000u), 2256u,
             "catch-up: ccr4 behind cnt -> re-armed at cnt+256");
    CHECK_TRUE(
        static_cast<int32_t>(tim2_heartbeat_next_ccr4(1000u, 2000u) - 2000u) > 0,
        "catch-up: next ccr4 always ahead of cnt");

    // Wrap-safe: subtração com sinal em aritmética modular de 32 bits — um
    // ccr4 "logicamente atrás" perto do wrap do uint32_t ainda é detectado
    // corretamente (mesma disciplina usada em todo o dispatcher TIM2/CH3).
    CHECK_EQ(tim2_heartbeat_next_ccr4(0xFFFFFFF0u, 0x00000010u), 0x00000110u,
             "catch-up: detected correctly across uint32_t wrap");
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

