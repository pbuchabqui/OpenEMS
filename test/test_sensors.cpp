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
#include "engine/map_window.h"
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

void test_sensors_validate_range(void) {
    section("sensors: validate_sensor_range");
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 100u),   "CLT=100 valid");
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 3800u),  "CLT=3800 valid");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 99u),   "CLT=99 invalid");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 3801u), "CLT=3801 invalid");
    CHECK_TRUE(validate_sensor_range(SensorId::MAP, 1000u),  "MAP=1000 valid");
    CHECK_FALSE(validate_sensor_range(SensorId::MAP, 10u),   "MAP=10 invalid");
}

void test_sensors_validate_values(void) {
    section("sensors: validate_sensor_values");
    SensorData d{};
    d.map_bar_x1000 = 1000u; d.clt_degc_x10 = 850; d.iat_degc_x10 = 250;
    d.tps_pct_x10 = 500u; d.vbatt_mv = 12000u;
    d.fuel_press_bar_x1000 = 3000u; d.oil_press_bar_x1000 = 5000u;
    CHECK_TRUE(validate_sensor_values(d), "all valid → true");
    SensorData bad = d; bad.map_bar_x1000 = 50u;
    CHECK_FALSE(validate_sensor_values(bad), "MAP < 0.10 bar → false");
    bad = d; bad.clt_degc_x10 = 1600;
    CHECK_FALSE(validate_sensor_values(bad), "CLT > 150°C → false");
    bad = d; bad.vbatt_mv = 5000u;
    CHECK_FALSE(validate_sensor_values(bad), "vbatt < 6V → false");
    bad = d; bad.tps_pct_x10 = 1001u;
    CHECK_FALSE(validate_sensor_values(bad), "TPS > 100% → false");
}

void test_sensors_health_status(void) {
    section("sensors: get_sensor_health_status");
    sensor_setup(); sensors_init();
    CHECK_EQ(get_sensor_health_status(), 0u, "health=0 after init with valid ADC");
}

void test_sensors_calibration(void) {
    section("sensors: set_tps_cal / set_app_cal / set_plausibility / set_etb_tps_cal");
    sensor_setup(); sensors_init();
    sensors_set_tps_cal(400u, 3800u);
    sensors_set_app_cal(400u, 3800u, 400u, 3800u);
    sensors_set_plausibility(100u, 100u);
    sensors_set_etb_tps_cal(400u, 3800u, 400u, 3800u);
    CHECK_TRUE(true, "calibration setters complete without crash");
}

void test_sensors_tick_100ms_clt_iat(void) {
    section("sensors: sensors_tick_100ms → CLT/IAT via lut128");
    sensor_setup(); sensors_init();
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT, 2000u);
    for (int i = 0; i < 8; ++i) { sensors_test_tick_100ms(); }
    const SensorData sd = sensors_get();
    const int16_t expected = static_cast<int16_t>(-400 + (1900 * 62) / 127);
    CHECK_EQ(sd.clt_degc_x10, expected, "CLT lut128(2000) matches formula");
    CHECK_EQ(sd.iat_degc_x10, expected, "IAT lut128(2000) matches formula");
}

// raw 2000, cal 0..4095 → 2000*1000/4095 = 488. A primeira amostra semeia o IIR.
void test_sensors_tick_pedal_publishes(void) {
    section("sensors: sensors_tick_pedal publica ETB sem o tick de 100 ms");
    using namespace ems::hal;
    sensor_setup(); sensors_init();
    CHECK_EQ(sensors_get().etb_tps_pct_x10, 0u, "antes do tick o ETB committed é 0");
    sensors_tick_pedal(2u);
    CHECK_EQ(sensors_get().etb_tps_pct_x10, 488u, "uma chamada com raw 2000 publica 48,8%");
    CHECK_EQ(sensors_get().app_pct_x10, 488u, "APP no mesmo tick");

    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 1000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 1000u);
    sensors_test_tick_100ms();
    CHECK_EQ(sensors_get().etb_tps_pct_x10, 488u, "tick de 100 ms não mexe no pedal");

    sensors_tick_pedal(2u);
    // iir(2000, 1000, 3/10) = 1700 nos dois canais → 1700*1000/4095 = 415
    CHECK_EQ(sensors_get().etb_tps_pct_x10, 415u, "segunda amostra entra pelo IIR");
}

void test_sensors_pedal_range_ceiling(void) {
    section("sensors: teto APP/ETB 4000; 4095 falha em 300 ms");
    using namespace ems::hal;
    sensor_setup(); sensors_init();

    adc_test_set_raw_primary(AdcPrimaryChannel::APP1, 4000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP2, 4000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 4000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 4000u);
    for (int i = 0; i < 8; ++i) { sensors_tick_pedal(50u); }
    CHECK_EQ(sensors_get().throttle_fault_bits, 0u, "raw 4000 não arma em 400 ms");
    CHECK_EQ(sensors_get().etb_tps_pct_x10, 976u, "4000 permanece posição válida");
    CHECK_EQ(sensors_get().app_pct_x10, 976u, "APP 4000 permanece válido");

    adc_test_set_raw_primary(AdcPrimaryChannel::APP1, 4095u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP2, 4095u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 4095u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 4095u);
    sensors_tick_pedal(10000u);
    CHECK_EQ(sensors_get().throttle_fault_bits, 0u, "dt enorme conta no máximo 50 ms");
    CHECK_TRUE(sensors_get().etb_tps_pct_x10 > 0u, "ainda não zerou aos 50 ms");
    for (int i = 0; i < 4; ++i) { sensors_tick_pedal(50u); }
    CHECK_EQ(sensors_get().throttle_fault_bits, 0u, "250 ms de 4095 ainda não é falha");
    CHECK_TRUE(sensors_get().etb_tps_pct_x10 > 0u, "percentual segue até fechar o debounce");
    sensors_tick_pedal(50u);
    const uint8_t bits = sensors_get().throttle_fault_bits;
    CHECK_EQ(sensors_get().etb_tps_pct_x10, 0u, "4095 aos 300 ms zera o ETB");
    CHECK_EQ(sensors_get().app_pct_x10, 0u, "4095 aos 300 ms zera o APP");
    CHECK_TRUE((bits & THROTTLE_FAULT_APP1) != 0u, "APP1 em falha");
    CHECK_TRUE((bits & THROTTLE_FAULT_APP2) != 0u, "APP2 em falha");
    CHECK_TRUE((bits & THROTTLE_FAULT_ETB_TPS1) != 0u, "TPS1 em falha");
    CHECK_TRUE((bits & THROTTLE_FAULT_ETB_TPS2) != 0u, "TPS2 em falha");

    adc_test_set_raw_primary(AdcPrimaryChannel::APP1, 2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP2, 2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 2000u);
    sensors_tick_pedal(2u);
    CHECK_EQ(sensors_get().throttle_fault_bits, 0u, "uma amostra dentro da faixa limpa a falha");
    CHECK_TRUE(sensors_get().etb_tps_pct_x10 > 0u, "ETB volta a publicar");
}

void test_sensors_pedal_plausibility_time(void) {
    section("sensors: plausibilidade APP continua ~300 ms");
    using namespace ems::hal;
    sensor_setup(); sensors_init();
    adc_test_set_raw_primary(AdcPrimaryChannel::APP1, 400u);
    adc_test_set_raw_primary(AdcPrimaryChannel::APP2, 3000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS1, 2000u);
    adc_test_set_raw_primary(AdcPrimaryChannel::ETB_TPS2, 2000u);
    for (int i = 0; i < 5; ++i) { sensors_tick_pedal(50u); }
    CHECK_TRUE((sensors_get().throttle_fault_bits & THROTTLE_FAULT_APP_PLAUS) == 0u,
               "divergência aos 250 ms ainda não trava");
    CHECK_EQ(sensors_get().app_pct_x10, 97u, "antes do debounce vale o menor APP");
    sensors_tick_pedal(50u);
    CHECK_TRUE((sensors_get().throttle_fault_bits & THROTTLE_FAULT_APP_PLAUS) != 0u,
               "divergência aos 300 ms trava plausibilidade");
    CHECK_EQ(sensors_get().app_pct_x10, 0u, "plausibilidade zera o pedal");
    CHECK_EQ(sensors_get().etb_tps_pct_x10, 488u, "TPS concordante não entra na falha");

    // A faixa olha o raw; a plausibilidade olha o percentual já filtrado.
    // Abrir o limite faz a amostra atual contar como boa e limpa no mesmo tick.
    sensors_set_plausibility(1000u, 1000u);
    sensors_tick_pedal(2u);
    CHECK_TRUE((sensors_get().throttle_fault_bits & THROTTLE_FAULT_APP_PLAUS) == 0u,
               "uma amostra dentro do limite limpa a plausibilidade");
    CHECK_TRUE(sensors_get().app_pct_x10 > 0u, "pedal volta a publicar");
}

void test_sensors_map_baro_refuses_vacuum(void) {
    section("sensors: baro não promove vácuo para 70 kPa");
    using namespace ems::hal;
    sensor_setup(); sensors_init();
    fuel_set_baro_bar_x100(101u);

    uint16_t out = 0u;
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 1365u);
    CHECK_TRUE(sensors_map_baro_bar_x100(&out), "raw ~1,00 bar é aceite");
    CHECK_EQ(out, 100u, "1365 → 100 kPa");

    out = 0u;
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 956u);
    CHECK_TRUE(sensors_map_baro_bar_x100(&out), "70 kPa no limite entra");
    CHECK_EQ(out, 70u, "956 → 70 kPa");

    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 955u);
    CHECK_FALSE(sensors_map_baro_bar_x100(&out), "69 kPa não é puxado para 70");
    CHECK_EQ(fuel_get_baro_bar_x100(), 101u, "rejeição não escreve o baro");

    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 477u);
    CHECK_FALSE(sensors_map_baro_bar_x100(&out), "vácuo ~35 kPa não entra");
    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 0u);
    CHECK_FALSE(sensors_map_baro_bar_x100(&out), "raw 0 está fora da faixa do MAP");
    CHECK_EQ(fuel_get_baro_bar_x100(), 101u, "vácuo deixa o baro anterior");

    adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 1365u);
    adc_test_set_recovering(true);
    CHECK_FALSE(sensors_map_baro_bar_x100(&out), "ADC a recuperar não publica baro");
    adc_test_set_recovering(false);
    adc_test_set_recovery_failed(true);
    CHECK_FALSE(sensors_map_baro_bar_x100(&out), "ADC em falha não publica baro");
    adc_test_set_recovery_failed(false);
    CHECK_EQ(fuel_get_baro_bar_x100(), 101u, "falha de ADC deixa o baro anterior");
}

void test_sensors_maf_freq_capture(void) {
    section("sensors: sensors_maf_freq_capture_isr");
    sensor_setup(); sensors_init();
    sensors_maf_freq_capture_isr(5000u);
    sensors_maf_freq_capture_isr(10000u);
    sensors_maf_freq_capture_isr(0u);
    CHECK_TRUE(true, "maf_freq_capture_isr handles various periods");
}

// ═══════════════════════════════════════════════════════════════════════════
// FUEL CALC
// ═══════════════════════════════════════════════════════════════════════════

void test_sensors_on_tooth(void) {
    section("sensors: sensors_on_tooth");
    sensor_setup(); sensors_init();
    ems::drv::CkpSnapshot snap{};
    snap.tooth_period_ns = 160000u;  // 10000 ticks × 16 ns = 160000 ns
    snap.rpm_x10 = 62500u;
    sensors_on_tooth(snap);  // triggers ADC sample accumulation (fast channels)
    CHECK_TRUE(true, "sensors_on_tooth completes without crash");
}

void test_map_window_angular(void) {
    section("map_window: janela angular por cilindro + balance");
    using ems::engine::map_window_on_tooth;
    using ems::engine::map_window_slot_bar_x1000;
    using ems::engine::map_window_balance_x1000;
    using ems::engine::map_window_cycles;
    using ems::engine::map_window_reset;
    using ems::engine::map_window_mean_bar_x1000;

    map_window_reset();
    ems::drv::CkpSnapshot s{};
    s.state = SyncState::FULL_SYNC;
    s.cmp_confirms = 2u;
    s.phase_A = true;
    s.tooth_index = 0u;

    // Desligado (default): no-op.
    ems::engine::map_window_enable = 0u;
    map_window_on_tooth(s, 500u);
    CHECK_EQ(map_window_cycles(), 0u, "enable=0: nenhum ciclo");
    CHECK_EQ(map_window_slot_bar_x1000(0u), 0u, "enable=0: slot vazio");
    CHECK_EQ(map_window_mean_bar_x1000(), 0u, "enable=0: média de slots vazios = 0");

    // 8 ciclos de 720° com MAP distinto por quadrante de 180°:
    // 500 / 520 / 480 / 500 → média 500, desvios 0 / +20 / -20 / 0.
    ems::engine::map_window_enable   = 1u;
    ems::engine::map_window_open_deg = 0u;
    ems::engine::map_window_len_deg  = 90u;
    for (uint8_t cycle = 0u; cycle < 8u; ++cycle) {
        for (uint8_t rev = 0u; rev < 2u; ++rev) {
            s.phase_A = (rev == 0u);
            for (uint16_t t = 0u; t < 58u; ++t) {
                s.tooth_index = t;
                const uint16_t deg  = static_cast<uint16_t>(t * 6u + (rev ? 360u : 0u));
                const uint8_t  quad = static_cast<uint8_t>(deg / 180u);
                const uint16_t map  = (quad == 1u) ? 520u : (quad == 2u) ? 480u : 500u;
                map_window_on_tooth(s, map);
            }
        }
    }
    CHECK_EQ(map_window_cycles(), 8u, "8 ciclos completos (4 janelas cada)");
    CHECK_EQ(map_window_slot_bar_x1000(0u), 500u, "slot 0 média = 500");
    CHECK_EQ(map_window_slot_bar_x1000(1u), 520u, "slot 1 média = 520");
    CHECK_EQ(map_window_slot_bar_x1000(2u), 480u, "slot 2 média = 480");
    CHECK_EQ(map_window_slot_bar_x1000(3u), 500u, "slot 3 média = 500");
    CHECK_EQ(map_window_mean_bar_x1000(), 500u,
             "média dos 4 slots (500+520+480+500)/4 = 500");
    // EMA α=1/8 a partir de 0: após 8 ciclos ≈ dev × 0,66.
    CHECK_TRUE(map_window_balance_x1000(1u) >= 10 && map_window_balance_x1000(1u) <= 20,
               "balance slot 1 → +20 (EMA parcial)");
    CHECK_TRUE(map_window_balance_x1000(2u) <= -10 && map_window_balance_x1000(2u) >= -20,
               "balance slot 2 → -20 (EMA parcial)");
    CHECK_TRUE(map_window_balance_x1000(0u) >= -1 && map_window_balance_x1000(0u) <= 1,
               "balance slot 0 ≈ 0");

    // Perda de fase de came: média deixa de ser viva (cycles=0) para o
    // fuel path não servir MAP de pré-dropout no re-lock.
    s.phase_A = true;
    s.tooth_index = 2u;          // dentro da janela do slot 0
    map_window_on_tooth(s, 900u);
    s.cmp_confirms = 1u;         // fase deixou de estar confirmada
    map_window_on_tooth(s, 900u);
    CHECK_EQ(map_window_cycles(), 0u, "perda de came: cycles invalidado");
    CHECK_EQ(map_window_mean_bar_x1000(), 0u, "perda de came: slots zerados");

    // Re-sync: fuel deve ficar no IIR até fechar um ciclo novo de 720°.
    s.cmp_confirms = 2u;
    s.phase_A = true;
    s.tooth_index = 0u;
    map_window_on_tooth(s, 500u);
    CHECK_EQ(map_window_cycles(), 0u, "re-sync: ainda sem ciclo completo");
    for (uint8_t rev = 0u; rev < 2u; ++rev) {
        s.phase_A = (rev == 0u);
        for (uint16_t t = 0u; t < 58u; ++t) {
            s.tooth_index = t;
            map_window_on_tooth(s, 500u);
        }
    }
    CHECK_EQ(map_window_cycles(), 1u, "após 720° fresco: um ciclo vivo");
    CHECK_EQ(map_window_mean_bar_x1000(), 500u, "média do ciclo novo");

    ems::engine::map_window_enable = 0u;  // isolamento entre testes
    map_window_reset();
}

void test_sensors_tick_50ms(void) {
    section("sensors: sensors_tick_50ms");
    sensor_setup(); sensors_init();
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::FUEL_PRESS, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::OIL_PRESS,  2000u);
    sensors_tick_50ms();
    CHECK_TRUE(true, "sensors_tick_50ms completes without crash");
    // After tick: fuel/oil pressure should be set (raw=2000 → some bar value)
    for (int i = 0; i < 4; ++i) { sensors_tick_50ms(); }  // fill staging buffers
    sensors_test_tick_100ms();  // commits staging → committed (double-buffer swap)
    const ems::drv::SensorData sd = sensors_get();
    CHECK_TRUE(sd.fuel_press_bar_x1000 > 0u, "fuel_press > 0 after tick_50ms with raw=2000");
}

void test_sensors_set_range(void) {
    section("sensors: sensors_set_range");
    sensor_setup(); sensors_init();
    // Widen CLT range so that raw=50 is accepted
    sensors_set_range(SensorId::CLT, {50u, 4000u});
    CHECK_TRUE(validate_sensor_range(SensorId::CLT, 50u), "CLT raw=50 valid after range change");
    CHECK_FALSE(validate_sensor_range(SensorId::CLT, 49u), "CLT raw=49 still invalid");
}

void test_sensors_etb_harness_present(void) {
    section("sensors: sensors_set_etb_harness_present");
    sensor_setup(); sensors_init();
    sensors_set_etb_harness_present(true);
    sensors_test_tick_100ms();
    CHECK_TRUE(true, "tick_100ms with harness_present=true: no crash");
    sensors_set_etb_harness_present(false);  // restore
}

// VBATT passou a ter canal próprio (PC3/INP13) em vez de partilhar o pino do
// ETB_TPS2. Estes casos fixam o contrato de que `corr_vbatt()` e o cálculo de dwell
// consomem: a tensão vem do ADC, já não do literal 12000 mV.
void test_sensors_vbatt_dedicated_channel(void) {
    section("sensors: VBATT em canal dedicado (PC3/INP13)");
    using namespace ems::hal;
    sensor_setup(); sensors_init();
    sensors_set_bench_clt_iat(false, 0, 0);

    // Divisor 0..18 V em 0..4095: 12 V → raw ≈ 12000*4095/18000 = 2730.
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 2730u);
    sensors_test_tick_100ms();
    CHECK_NEAR(static_cast<float>(sensors_get().vbatt_mv), 12000.0f, 30.0f,
               "raw 2730 → ~12000 mV");
    CHECK_FALSE(sensors_get().vbatt_fault, "12 V: sem vbatt_fault");

    // Cranking: a bateria cai a ~9,5 V. É o caso que o literal 12000 mascarava.
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 2161u);
    sensors_test_tick_100ms();
    CHECK_NEAR(static_cast<float>(sensors_get().vbatt_mv), 9500.0f, 30.0f,
               "raw 2161 → ~9500 mV (cranking)");
    CHECK_FALSE(sensors_get().vbatt_fault, "9,5 V (cranking) não é falha");

    // O chicote do ETB já não interfere: VBATT tem pino próprio.
    sensors_set_etb_harness_present(true);
    sensors_test_tick_100ms();
    CHECK_NEAR(static_cast<float>(sensors_get().vbatt_mv), 9500.0f, 30.0f,
               "harness do ETB presente não fixa mais 12000 mV");
    sensors_set_etb_harness_present(false);

    // Fora de 6..18 V (pino aberto/curto) → fallback seguro de 12 V.
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 0u);
    sensors_test_tick_100ms();
    CHECK_EQ(sensors_get().vbatt_mv, 12000u, "raw 0 (implausível) → fallback 12000 mV");
    CHECK_TRUE(sensors_get().vbatt_fault, "raw 0 → vbatt_fault");
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 800u);    // ~3,5 V: abaixo de 6 V
    sensors_test_tick_100ms();
    CHECK_FALSE(sensors_get().vbatt_fault, "raw 800 (<6 V): queda real não é falha");
    CHECK_TRUE(sensors_get().vbatt_mv < 6000u && sensors_get().vbatt_mv > 0u,
               "raw 800 (queda real) → telemetria crua, nem 12 V nem 6 V");
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 2730u);
    sensors_test_tick_100ms();
    CHECK_FALSE(sensors_get().vbatt_fault, "leitura válida limpa vbatt_fault");
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 0u);
    sensors_test_tick_100ms();

    // Bancada não tem divisor em PC3 — o modo de bancada tem de ignorar o ADC.
    sensors_set_bench_clt_iat(true, 800, 250);
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 4095u);
    sensors_test_tick_100ms();
    CHECK_EQ(sensors_get().vbatt_mv, 12000u, "bench mode fixa 12000 mV apesar do ADC");
    CHECK_FALSE(sensors_get().vbatt_fault, "bench mode não acusa vbatt_fault");
    sensors_set_bench_clt_iat(false, 0, 0);
}

void test_sensors_table_entry_setters(void) {
    section("sensors: sensors_test_set_clt_table_entry / set_iat_table_entry");
    sensor_setup(); sensors_init();
    // Manually set CLT table entry at index 62 (ADC=2000>>5=62) to 200 (20.0°C)
    sensors_test_set_clt_table_entry(62u, 200);
    sensors_test_set_iat_table_entry(62u, 150);
    using namespace ems::hal;
    adc_test_set_raw_secondary(AdcSecondaryChannel::CLT, 2000u);
    adc_test_set_raw_secondary(AdcSecondaryChannel::IAT, 2000u);
    for (int i = 0; i < 8; ++i) { sensors_test_tick_100ms(); }
    const ems::drv::SensorData sd = sensors_get();
    CHECK_EQ(sd.clt_degc_x10, 200, "CLT table entry 62 = 200 (20.0°C)");
    CHECK_EQ(sd.iat_degc_x10, 150, "IAT table entry 62 = 150 (15.0°C)");
}

// ═══════════════════════════════════════════════════════════════════════════
// KNOCK — SEGUNDA FASE
// ═══════════════════════════════════════════════════════════════════════════

