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

void test_sensors_sample_fast_channels_encoder(void) {
    section("sensors: sensors_sample_fast_channels_encoder (fork MT6835/TIM2)");
    using ems::hal::AdcPrimaryChannel;
    sensor_setup(); sensors_init();

    // raw=1365 → map_raw_to_bar_x1000 = 1365×3000/4095 = 1000 (1 bar exacto).
    ems::hal::adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 1365u);
    // g_map_filt parte de 0 e converge por IIR α=0.3/chamada — (0.7)^20≈0.0008,
    // 20 chamadas é suficiente p/ convergir dentro de poucas unidades.
    for (int i = 0; i < 20; ++i) {
        ems::drv::sensors_sample_fast_channels_encoder(3000u);
    }
    const SensorData sd = sensors_get();
    CHECK_TRUE(sd.map_bar_x1000 > 900u && sd.map_bar_x1000 < 1100u,
               "MAP convirgiu para ~1000 (1 bar) via caminho encoder, sem hook de dente");

    // Sem chamar a função nenhuma, sensors_get() nunca se moveria deste
    // valor — muda o raw e confirma que o wrapper continua vivo.
    ems::hal::adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 2730u);  // → 2000 (2 bar)
    for (int i = 0; i < 20; ++i) {
        ems::drv::sensors_sample_fast_channels_encoder(3000u);
    }
    const SensorData sd2 = sensors_get();
    CHECK_TRUE(sd2.map_bar_x1000 > 1900u && sd2.map_bar_x1000 < 2100u,
               "MAP acompanha um novo raw ADC (2 bar) — caminho continua vivo");
}

void test_sensors_map_window_poll_encoder(void) {
    section("sensors: sensors_map_window_poll_encoder — fronteira de volta TIM2 (fork MT6835/TIM2)");
    using namespace ems::engine;
    using ems::drv::CkpSnapshot;
    using ems::hal::AdcPrimaryChannel;

    sensor_setup(); sensors_init();
    map_window_reset();
    ecu_sched_encoder_phase_set_anchor(0u, ECU_PHASE_A);

    CkpSnapshot base{};
    base.state = SyncState::FULL_SYNC;
    base.cmp_confirms = 2u;
    ems::drv::ckp_publish_encoder_snapshot(base);

    // Desligado (default): no-op, mesmo com FULL_SYNC seeded acima.
    map_window_enable = 0u;
    ems::hal::adc_test_set_raw_primary(AdcPrimaryChannel::MAP, 1365u);
    ems::drv::sensors_map_window_poll_encoder(4096u);
    CHECK_EQ(map_window_cycles(), 0u, "enable=0: nenhum ciclo");

    map_window_enable   = 1u;
    map_window_open_deg = 0u;
    map_window_len_deg  = 90u;

    // Início de cada quadrante de 180° do ciclo de 720° (deg=0/180/360/540).
    // cycle_deg + phase_A vêm da mesma leitura tim2_now. Quad2/quad3
    // (deg 360-719) só existem depois da fronteira de volta do TIM2.
    constexpr uint32_t kTim2Quad[4] = {0u, 8192u, 16384u, 24576u};
    constexpr uint16_t kRawQuad[4]  = {1365u, 2730u, 1365u, 4095u};  // →1000/2000/1000/3000
    constexpr uint16_t kBarQuad[4]  = {1000u, 2000u, 1000u, 3000u};

    for (uint8_t cycle = 0u; cycle < 2u; ++cycle) {
        for (uint8_t q = 0u; q < 4u; ++q) {
            ems::hal::adc_test_set_raw_primary(AdcPrimaryChannel::MAP, kRawQuad[q]);
            for (uint8_t rep = 0u; rep < 2u; ++rep) {
                ems::drv::sensors_map_window_poll_encoder(kTim2Quad[q]);
            }
        }
    }
    // Fecha a última janela (quad3) ainda aberta, avançando p/ o quad0 seguinte.
    ems::hal::adc_test_set_raw_primary(AdcPrimaryChannel::MAP, kRawQuad[0]);
    ems::drv::sensors_map_window_poll_encoder(kTim2Quad[0]);

    CHECK_EQ(map_window_cycles(), 2u, "2 ciclos completos (4 janelas cada)");
    CHECK_EQ(map_window_slot_bar_x1000(0u), kBarQuad[0], "slot 0 (quad0, phase_A) = 1000");
    CHECK_EQ(map_window_slot_bar_x1000(1u), kBarQuad[1], "slot 1 (quad1, phase_A) = 2000");
    CHECK_EQ(map_window_slot_bar_x1000(2u), kBarQuad[2],
             "slot 2 (quad2, cruza a fronteira de volta A→B) = 1000 — cycle_deg/phase_A coerentes");
    CHECK_EQ(map_window_slot_bar_x1000(3u), kBarQuad[3], "slot 3 (quad3, phase_B) = 3000");

    map_window_enable = 0u;  // isolamento entre testes
    map_window_reset();
}

void test_map_window_angular(void) {
    section("map_window: janela angular por cilindro + balance");
    using ems::engine::map_window_on_sample;
    using ems::engine::map_window_slot_bar_x1000;
    using ems::engine::map_window_balance_x1000;
    using ems::engine::map_window_cycles;
    using ems::engine::map_window_reset;

    map_window_reset();

    // Desligado (default): no-op.
    ems::engine::map_window_enable = 0u;
    map_window_on_sample(0u, 500u, true, true);
    CHECK_EQ(map_window_cycles(), 0u, "enable=0: nenhum ciclo");
    CHECK_EQ(map_window_slot_bar_x1000(0u), 0u, "enable=0: slot vazio");

    // 8 ciclos de 720° com MAP distinto por quadrante de 180°:
    // 500 / 520 / 480 / 500 → média 500, desvios 0 / +20 / -20 / 0.
    ems::engine::map_window_enable   = 1u;
    ems::engine::map_window_open_deg = 0u;
    ems::engine::map_window_len_deg  = 90u;
    for (uint8_t cycle = 0u; cycle < 8u; ++cycle) {
        for (uint16_t deg = 0u; deg < 720u; deg = static_cast<uint16_t>(deg + 6u)) {
            const uint8_t  quad = static_cast<uint8_t>(deg / 180u);
            const uint16_t map  = (quad == 1u) ? 520u : (quad == 2u) ? 480u : 500u;
            map_window_on_sample(deg, map, true, true);
        }
    }
    CHECK_EQ(map_window_cycles(), 8u, "8 ciclos completos (4 janelas cada)");
    CHECK_EQ(map_window_slot_bar_x1000(0u), 500u, "slot 0 média = 500");
    CHECK_EQ(map_window_slot_bar_x1000(1u), 520u, "slot 1 média = 520");
    CHECK_EQ(map_window_slot_bar_x1000(2u), 480u, "slot 2 média = 480");
    CHECK_EQ(map_window_slot_bar_x1000(3u), 500u, "slot 3 média = 500");
    // EMA α=1/8 a partir de 0: após 8 ciclos ≈ dev × 0,66.
    CHECK_TRUE(map_window_balance_x1000(1u) >= 10 && map_window_balance_x1000(1u) <= 20,
               "balance slot 1 → +20 (EMA parcial)");
    CHECK_TRUE(map_window_balance_x1000(2u) <= -10 && map_window_balance_x1000(2u) >= -20,
               "balance slot 2 → -20 (EMA parcial)");
    CHECK_TRUE(map_window_balance_x1000(0u) >= -1 && map_window_balance_x1000(0u) <= 1,
               "balance slot 0 ≈ 0");

    // Perda de fase de came: zera cycles e slots para o fuel não reutilizar
    // MAP de pré-dropout no re-lock.
    map_window_on_sample(12u, 900u, true, true);   // dentro da janela do slot 0
    map_window_on_sample(12u, 900u, true, false);  // CMP deixou de estar confirmado
    CHECK_EQ(map_window_cycles(), 0u, "dropout zera cycles");
    CHECK_EQ(map_window_slot_bar_x1000(0u), 0u, "dropout zera slots");
    CHECK_EQ(map_window_slot_bar_x1000(1u), 0u, "dropout zera todos os slots");

    ems::engine::map_window_enable = 0u;  // isolamento entre testes
    map_window_reset();
}

// Regressão: poll de 2ms a RPM alto (ou map_window_len_deg calibrado curto)
// pode saltar uma janela inteira entre duas chamadas — sem detecção, o bit
// do slot saltado nunca entrava em fresh_mask, o ciclo nunca completava
// (nem para os outros 3 slots) e o slot saltado ficava congelado no último
// valor válido para sempre, sem fault nenhum a avisar (achado #2 da
// revisão dos commits 2fa1513..bc30ca6, 2026-08-19).
void test_map_window_skipped_slot(void) {
    section("map_window: janela inteira saltada (poll grosso a RPM alto) não trava o ciclo");
    using ems::engine::map_window_on_sample;
    using ems::engine::map_window_slot_bar_x1000;
    using ems::engine::map_window_balance_x1000;
    using ems::engine::map_window_cycles;
    using ems::engine::map_window_skip_count;
    using ems::engine::map_window_reset;

    map_window_reset();
    ems::engine::map_window_enable   = 1u;
    ems::engine::map_window_open_deg = 0u;
    ems::engine::map_window_len_deg  = 90u;
    // Janelas: slot0=[0,90) slot1=[180,270) slot2=[360,450) slot3=[540,630)

    CHECK_EQ(map_window_skip_count(), 0u, "skip_count=0 no início");

    // Amostra dentro do slot 0.
    map_window_on_sample(45u, 500u, true, true);
    CHECK_EQ(map_window_skip_count(), 0u, "1ª amostra nunca conta como salto");

    // Salta DIRETO para dentro do slot 2 — slot 1 nunca visitado.
    map_window_on_sample(405u, 480u, true, true);
    CHECK_EQ(map_window_skip_count(), 1u, "slot 1 saltado: skip_count=1");
    CHECK_EQ(map_window_slot_bar_x1000(1u), 0u,
             "slot 1 saltado mantém o último valor conhecido (0, nunca escrito)");

    // Continua slot 3, depois fecha o ciclo entrando de novo no slot 0.
    map_window_on_sample(585u, 520u, true, true);
    map_window_on_sample(45u, 500u, true, true);  // fecha slot3, fresh_mask completa
    CHECK_EQ(map_window_cycles(), 1u,
             "ciclo completa mesmo com 1 slot saltado (fresh_mask não trava)");
    CHECK_EQ(map_window_slot_bar_x1000(0u), 500u, "slot 0 correto");
    CHECK_EQ(map_window_slot_bar_x1000(2u), 480u, "slot 2 correto");
    CHECK_EQ(map_window_slot_bar_x1000(3u), 520u, "slot 3 correto");
    // balance entra no cálculo mesmo com slot1 stale (0) — só confirma que
    // o EMA avançou (não travou em 0 para sempre).
    CHECK_TRUE(map_window_balance_x1000(1u) != 0,
               "balance do slot 1 avança mesmo saltado (ciclo não travou)");

    // Saltar 2 slots seguidos (RPM ainda mais alto): slot0 → direto slot3.
    map_window_reset();
    ems::engine::map_window_enable = 1u;
    map_window_on_sample(45u, 500u, true, true);   // slot 0
    map_window_on_sample(585u, 500u, true, true);  // salta slot 1 E slot 2
    CHECK_EQ(map_window_skip_count(), 2u, "2 slots saltados de uma vez: skip_count=2");

    // Perda de sync zera estado (não conta o hiato como salto).
    map_window_reset();
    ems::engine::map_window_enable = 1u;
    map_window_on_sample(45u, 500u, true, true);   // slot 0
    map_window_on_sample(45u, 500u, false, true);  // perde FULL_SYNC
    CHECK_EQ(map_window_cycles(), 0u, "FULL_SYNC drop zera cycles");
    CHECK_EQ(map_window_slot_bar_x1000(0u), 0u, "FULL_SYNC drop zera slots");
    map_window_on_sample(405u, 480u, true, true);  // slot 2, sync recuperado
    CHECK_EQ(map_window_skip_count(), 0u,
             "hiato de sync não é contado como janela saltada");

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

    // Cranking: a bateria cai a ~9,5 V. É o caso que o literal 12000 mascarava.
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 2161u);
    sensors_test_tick_100ms();
    CHECK_NEAR(static_cast<float>(sensors_get().vbatt_mv), 9500.0f, 30.0f,
               "raw 2161 → ~9500 mV (cranking)");

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

    // Bancada não tem divisor em PC3 — o modo de bancada tem de ignorar o ADC.
    sensors_set_bench_clt_iat(true, 800, 250);
    adc_test_set_raw_secondary(AdcSecondaryChannel::VBATT, 4095u);
    sensors_test_tick_100ms();
    CHECK_EQ(sensors_get().vbatt_mv, 12000u, "bench mode fixa 12000 mV apesar do ADC");
    sensors_set_bench_clt_iat(false, 0, 0);
}

// Achado #4 da revisão 2026-08-15: sensors_tick_100ms() em bench mode
// limpava g_fault[]/fault_bits por INTEIRO (todos os 8 SensorId), não só
// CLT/IAT como o resto do ficheiro documenta — mascarando faults reais de
// MAP/MAF/TPS/O2 (incl. o caminho de falha de hardware ADC, que escreve
// fault_bits diretamente) por até 100ms de cada vez, alimentando
// fuel_protect_cut com dados errados.
void test_sensors_bench_mode_preserves_non_clt_iat_faults(void) {
    using namespace ems::hal;
    section("sensors: bench mode só limpa fault CLT/IAT, preserva os outros 6 (fix 2026-08-15)");
    sensor_setup(); sensors_init();
    sensors_set_bench_clt_iat(false, 0, 0);

    // Caminho de falha de hardware ADC (sensors.cpp:412-429) — OR direto em
    // fault_bits para MAP/MAF/TPS/O2, sem passar por g_fault[]/apply_fault.
    // sample_fast_channels() só publica em g_data_committed no ramo normal
    // (linha 564) — o ramo adc_unavailable faz `return` mais cedo (linha
    // 452) e não comita nada sozinho, por isso um tick_100ms (bench ainda
    // desligado, sem tocar CLT/IAT) força a publicação antes da leitura.
    adc_test_set_recovery_failed(true);
    ems::drv::sensors_sample_fast_channels_encoder(3000u);
    sensors_test_tick_100ms();
    const uint8_t map_bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(SensorId::MAP));
    CHECK_TRUE((sensors_get().fault_bits & map_bit) != 0u,
               "pré: MAP fault ativo (ADC recovery failed)");

    // Liga bench mode e corre um tick de 100ms — antes do fix isto zerava
    // fault_bits por inteiro, apagando o MAP fault que não tem nada a ver
    // com CLT/IAT.
    sensors_set_bench_clt_iat(true, 800, 250);
    sensors_test_tick_100ms();
    const SensorData sd = sensors_get();
    CHECK_TRUE((sd.fault_bits & map_bit) != 0u,
               "MAP fault sobrevive ao tick de 100ms com bench mode ligado");
    CHECK_EQ(sd.clt_degc_x10, 800, "CLT continua forçado para 80.0°C pelo bench mode");
    CHECK_EQ(sd.iat_degc_x10, 250, "IAT continua forçado para 25.0°C pelo bench mode");

    adc_test_set_recovery_failed(false);
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

