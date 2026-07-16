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

uint16_t ot_txn(uint8_t sub, uint8_t a1, uint16_t a2, uint8_t* out, uint16_t max) {
    const uint8_t cmd[5] = {'T', sub, a1,
                            static_cast<uint8_t>(a2 & 0xFFu),
                            static_cast<uint8_t>(a2 >> 8u)};
    ui_feed(cmd, 5u);
    return ui_drain(out, max);
}

void ot_reset_all(void) {
    ckp_test_reset(); g_ckp_cap = 0u;
    ecu_sched_test_reset();
    ems::app::ui_test_reset();
    ems::engine::output_test_test_reset();
    ems::engine::auxiliaries_test_reset();
}

void test_output_test_enter_gate(void) {
    section("output_test: enter exige RPM=0 + magic");
    ot_reset_all();

    uint8_t buf[8] = {};
    // magic errado → NAK, continua inactivo
    uint16_t n = ot_txn(0x01u, 0u, 0x1234u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "ENTER magic errado → NAK");
    CHECK_TRUE(!ems::engine::output_test_active(), "continua inactivo");

    // magic certo com RPM=0 → OK
    n = ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "ENTER magic 0xA55A → ACK");
    CHECK_TRUE(ems::engine::output_test_active(), "modo activo");
    ems::engine::output_test_exit();

    // com motor girando → NAK
    ckp_reach_full_sync();
    n = ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "ENTER com RPM>0 → NAK");
    CHECK_TRUE(!ems::engine::output_test_active(), "inactivo com motor girando");
}

void test_output_test_fire_inj(void) {
    section("output_test: FIRE_INJ agenda pulso e clampa");
    ot_reset_all();

    uint8_t buf[8] = {};
    // fire sem enter → NAK, nada agendado
    uint16_t n = ot_txn(0x10u, 0u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "FIRE_INJ sem enter → NAK");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 0u, "fila vazia sem enter");

    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    ecu_sched_test_set_tim5_cnt(1000000u);
    n = ot_txn(0x10u, 0u, 5000u, buf, sizeof(buf));  // INJ1, 5000µs
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "FIRE_INJ cyl0 5000µs → ACK");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 1u, "1 evento OFF agendado");
    // OFF ≈ cnt + 5000µs × 62.5 = 1000000 + 312500
    const uint32_t ccr = ecu_sched_test_get_tim5_ccr3();
    CHECK_TRUE(ccr >= 1312500u && ccr <= 1313000u, "CCR3 = now + 312500 ticks");

    // cyl inválido → NAK
    n = ot_txn(0x10u, 4u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "cyl=4 → NAK");
    ems::engine::output_test_exit();
}

void test_output_test_busy_window(void) {
    section("output_test: janela busy serializa pulsos");
    ot_reset_all();

    uint8_t buf[8] = {};
    ems::engine::output_test_poll(10000u, 0u);  // g_now_ms = 10000
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    uint16_t n = ot_txn(0x10u, 0u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(buf[0] == 0x00u, "1º fire → ACK");
    n = ot_txn(0x10u, 1u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x01u, "2º fire imediato → NAK (busy)");
    // avança 200ms (>5ms pulso + 100ms gap) → liberto
    ems::engine::output_test_poll(10200u, 0u);
    n = ot_txn(0x10u, 1u, 5000u, buf, sizeof(buf));
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "fire após janela → ACK");
    ems::engine::output_test_exit();
}

void test_output_test_fire_ign_watchdog(void) {
    section("output_test: FIRE_IGN clampa dwell e arma watchdog");
    ot_reset_all();

    uint8_t buf[8] = {};
    ecu_sched_test_set_tim5_cnt(2000000u);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    uint16_t n = ot_txn(0x11u, 0u, 60000u, buf, sizeof(buf));  // pede 60ms
    CHECK_TRUE(n == 1u && buf[0] == 0x00u, "FIRE_IGN → ACK");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 1u, "evento SPARK agendado");
    // clamp p/ 10000µs = 625000 ticks
    const uint32_t ccr = ecu_sched_test_get_tim5_ccr3();
    CHECK_TRUE(ccr >= 2625000u && ccr <= 2625500u, "SPARK = now + 625000 (clamp 10ms)");
    // watchdog armado: avança TIM5 além de 1.4×dwell e verifica que dispara
    const uint32_t wd_before = ecu_sched_dwell_watchdog_count();
    ecu_sched_test_set_tim5_cnt(2000000u + 875000u + 100u);  // 1.4×625000 + margem
    ecu_sched_dwell_watchdog();
    CHECK_EQ(ecu_sched_dwell_watchdog_count(), wd_before + 1u,
             "dwell watchdog dispara como backstop");
    ems::engine::output_test_exit();
}

void test_output_test_rpm_abort(void) {
    section("output_test: aborto por RPM restaura estado seguro");
    ot_reset_all();

    uint8_t buf[8] = {};
    ems::engine::output_test_poll(20000u, 0u);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    ot_txn(0x20u, 1u, 0u, buf, sizeof(buf));  // bomba ON
    CHECK_TRUE(ems::engine::auxiliaries_test_get_pump_state(), "bomba ligada");
    ot_txn(0x10u, 0u, 20000u, buf, sizeof(buf));  // pulso em voo
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u, "evento em voo");

    ems::engine::output_test_poll(20002u, 7000u);  // RPM detectado
    CHECK_TRUE(!ems::engine::output_test_active(), "modo abortado");
    CHECK_TRUE(!ems::engine::auxiliaries_test_get_pump_state(), "bomba desligada");
    CHECK_EQ(ecu_sched_test_get_evt_count(), 0u, "fila de eventos limpa");

    uint8_t st[4] = {};
    uint16_t n = ot_txn(0x03u, 0u, 0u, st, sizeof(st));
    CHECK_TRUE(n == 4u && st[0] == 0u && st[1] == 1u,
               "STATUS: inactivo, abort_reason=RPM");
}

void test_output_test_keepalive_timeout(void) {
    section("output_test: timeout de keepalive");
    ot_reset_all();

    uint8_t buf[8] = {};
    ems::engine::output_test_poll(30000u, 0u);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    // keepalive em +4s mantém vivo
    ems::engine::output_test_poll(34000u, 0u);
    ot_txn(0x02u, 0u, 0u, buf, sizeof(buf));
    CHECK_TRUE(buf[0] == 0x00u, "KEEPALIVE → ACK");
    ems::engine::output_test_poll(38000u, 0u);
    CHECK_TRUE(ems::engine::output_test_active(), "vivo em +8s com keepalive em +4s");
    // sem keepalive: expira em +5s
    ems::engine::output_test_poll(43001u, 0u);
    CHECK_TRUE(!ems::engine::output_test_active(), "expirado após 5s sem keepalive");
    uint8_t st[4] = {};
    ot_txn(0x03u, 0u, 0u, st, sizeof(st));
    CHECK_TRUE(st[1] == 2u, "abort_reason=timeout");
}

void test_output_test_suspends_aux(void) {
    section("output_test: suspende controles automáticos");
    ot_reset_all();

    uint8_t buf[8] = {};
    // key-on dispara o prime da bomba (2s) via run_pump_control no tick —
    // excepto em modo teste, onde o tick retorna antes de tocar as saídas.
    ems::engine::auxiliaries_set_key_on(true);
    ot_txn(0x01u, 0u, 0xA55Au, buf, sizeof(buf));
    ems::engine::auxiliaries_tick_10ms();
    CHECK_TRUE(!ems::engine::auxiliaries_test_get_pump_state(),
               "tick_10ms não liga a bomba (prime) em modo teste");
    ems::engine::output_test_exit();
    ems::engine::auxiliaries_tick_10ms();
    CHECK_TRUE(ems::engine::auxiliaries_test_get_pump_state(),
               "após exit, prime de key-on religa a bomba");
    ems::engine::auxiliaries_set_key_on(false);
}

