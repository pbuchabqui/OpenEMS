#include "test/harness.h"

#include <cstdint>

#include "engine/calibration.h"
#include "engine/ewg_control.h"
#include "hal/ac_request.h"
#include "hal/ewg_driver.h"
#include "hal/flex_fuel.h"

using namespace ems::engine;
using namespace ems::hal;

void test_ewg_control(void) {
    section("ewg_control: PID de posição do wastegate");

    const uint16_t kp = ewg_kp_x10, ki = ewg_ki_x10, kd = ewg_kd_x10;
    const uint16_t pmin = ewg_pos_min_raw, pmax = ewg_pos_max_raw;
    ewg_kp_x10 = 80u; ewg_ki_x10 = 5u; ewg_kd_x10 = 20u;
    ewg_pos_min_raw = 200u; ewg_pos_max_raw = 3800u;

    const uint32_t sd0 = ewg_driver_test_shutdown_count();
    ewg_control_init();
    CHECK_EQ(ewg_driver_test_shutdown_count(), sd0 + 1u, "init: sai com driver em shutdown");
    CHECK_EQ(ewg_driver_test_last_pwm(), 0, "init: PWM 0");

    // Erro zero → saída zero.
    CHECK_EQ(ewg_control_update(500u, 500u), 0, "erro 0: saída 0");

    // Erro positivo → saída positiva e repassada ao driver.
    ewg_control_reset();
    const int16_t out_pos = ewg_control_update(600u, 500u);
    CHECK_TRUE(out_pos > 0, "erro +: saída positiva");
    CHECK_EQ(ewg_driver_test_last_pwm(), out_pos, "saída vai ao driver");
    // P = 8.0*100/1 em x10: kp*err/10 = 80*100/10 = 800; I = 5*100/10 = 50; D = 20*100/10 = 200
    CHECK_EQ(out_pos, 1000, "1º passo: P+I+D = 1050 satura em +1000");

    // Erro negativo → saída negativa.
    ewg_control_reset();
    CHECK_TRUE(ewg_control_update(400u, 500u) < 0, "erro −: saída negativa");

    // Clamp em ±1000 mesmo com ganhos altos (sem inversão de sinal).
    ewg_kp_x10 = 60000u;
    ewg_control_reset();
    CHECK_EQ(ewg_control_update(1000u, 0u), 1000, "ganho alto: clamp +1000 (sem wrap)");
    CHECK_EQ(ewg_control_update(0u, 1000u), -1000, "ganho alto: clamp −1000 (sem wrap)");
    ewg_kp_x10 = 80u;

    // Anti-windup: com erro sustentado o integrador congela em ±1000; ao inverter o erro a
    // saída tem de cruzar zero em poucos passos (sem descarregar milhares de passos acumulados).
    ewg_kp_x10 = 0u; ewg_kd_x10 = 0u; ewg_ki_x10 = 50u;     // +500 por passo com erro 100
    ewg_control_reset();
    for (int i = 0; i < 500; ++i) { ewg_control_update(100u, 0u); }
    CHECK_EQ(ewg_control_update(100u, 0u), 1000, "windup: saturado em +1000");
    int16_t o = 1000;
    int steps = 0;
    while (o > 0 && steps < 200) { o = ewg_control_update(0u, 100u); ++steps; }
    CHECK_TRUE(o <= 0, "anti-windup: sai da saturação ao inverter o erro");
    CHECK_TRUE(steps <= 5, "anti-windup: saída cruza zero em ≤5 passos (integrador limitado)");

    // Reset zera estado e desliga o driver.
    const uint32_t sd1 = ewg_driver_test_shutdown_count();
    ewg_control_reset();
    CHECK_EQ(ewg_driver_test_shutdown_count(), sd1 + 1u, "reset: shutdown do driver");
    CHECK_EQ(ewg_control_update(500u, 500u), 0, "reset: integrador/derivada zerados");

    // Termo derivativo: só reage à variação do erro.
    ewg_kp_x10 = 0u; ewg_ki_x10 = 0u; ewg_kd_x10 = 20u;
    ewg_control_reset();
    ewg_control_update(500u, 500u);
    CHECK_EQ(ewg_control_update(550u, 500u), 100, "D: degrau de erro 50 → 20*50/10 = 100");
    CHECK_EQ(ewg_control_update(550u, 500u), 0, "D: erro constante → 0");

    ewg_kp_x10 = kp; ewg_ki_x10 = ki; ewg_kd_x10 = kd;
    ewg_pos_min_raw = pmin; ewg_pos_max_raw = pmax;
    ewg_control_reset();
}

void test_ewg_position(void) {
    section("ewg_read_position_pct_x10: raw ADC → % x10");

    const uint16_t pmin = ewg_pos_min_raw, pmax = ewg_pos_max_raw;
    ewg_pos_min_raw = 200u; ewg_pos_max_raw = 3800u;

    ewg_driver_test_set_raw(0u);
    CHECK_EQ(ewg_read_position_pct_x10(), 0u, "abaixo do min: 0 %");
    ewg_driver_test_set_raw(200u);
    CHECK_EQ(ewg_read_position_pct_x10(), 0u, "== min: 0 %");
    ewg_driver_test_set_raw(2000u);
    CHECK_EQ(ewg_read_position_pct_x10(), 500u, "ponto médio: 50.0 %");
    ewg_driver_test_set_raw(3800u);
    CHECK_EQ(ewg_read_position_pct_x10(), 1000u, "== max: 100 %");
    ewg_driver_test_set_raw(4095u);
    CHECK_EQ(ewg_read_position_pct_x10(), 1000u, "acima do max: 100 %");

    // Calibração inválida (max <= min) nunca divide por zero.
    ewg_pos_max_raw = ewg_pos_min_raw;
    ewg_driver_test_set_raw(2000u);
    CHECK_EQ(ewg_read_position_pct_x10(), 0u, "max<=min: 0 %");

    ewg_pos_min_raw = pmin; ewg_pos_max_raw = pmax;
    ewg_driver_test_set_raw(2048u);
}

void test_flex_fuel_conversions(void) {
    section("flex_fuel: frequência/duty → etanol e temperatura");

    CHECK_EQ(flex_ethanol_pct_from_hz(45u), 0u, "45 Hz: 0 %");
    CHECK_EQ(flex_ethanol_pct_from_hz(50u), 0u, "50 Hz: 0 %");
    CHECK_EQ(flex_ethanol_pct_from_hz(100u), 50u, "100 Hz: 50 %");
    CHECK_EQ(flex_ethanol_pct_from_hz(150u), 100u, "150 Hz: 100 %");
    CHECK_EQ(flex_ethanol_pct_from_hz(155u), 100u, "155 Hz: 100 %");

    CHECK_FALSE(flex_fuel_freq_plausible(44u), "44 Hz implausível");
    CHECK_TRUE(flex_fuel_freq_plausible(45u), "45 Hz plausível");
    CHECK_TRUE(flex_fuel_freq_plausible(155u), "155 Hz plausível");
    CHECK_FALSE(flex_fuel_freq_plausible(156u), "156 Hz implausível");

    CHECK_EQ(flex_ethanol_pct_from_period_us(0u), 0u, "período 0: 0 %");
    CHECK_EQ(flex_ethanol_pct_from_period_us(10000u), 50u, "10 ms (100 Hz): 50 %");
    CHECK_EQ(flex_ethanol_pct_from_period_us(20000u), 0u, "20 ms (50 Hz): 0 %");
    CHECK_EQ(flex_ethanol_pct_from_period_us(5000u), 100u, "5 ms (200 Hz): satura 100 %");

    CHECK_EQ(flex_temp_x10_from_us(0u, 0u), -400, "sem sinal: -40 °C");
    CHECK_EQ(flex_temp_x10_from_us(10000u, 0u), -400, "sem alto: -40 °C");
    CHECK_EQ(flex_temp_x10_from_us(10000u, 1000u), -400, "duty 10 %: -40 °C");
    CHECK_EQ(flex_temp_x10_from_us(10000u, 9000u), 1250, "duty 90 %: 125 °C");
    CHECK_EQ(flex_temp_x10_from_us(10000u, 9900u), 1250, "duty 99 %: 125 °C");
    CHECK_EQ(flex_temp_x10_from_us(10000u, 5000u), 425, "duty 50 %: 42.5 °C");
    const int16_t t30 = flex_temp_x10_from_us(10000u, 3000u);
    const int16_t t70 = flex_temp_x10_from_us(10000u, 7000u);
    CHECK_TRUE(t30 < 425 && t70 > 425, "temperatura monotônica no duty");

    CHECK_TRUE(flex_signal_valid(0u, 10000u, 500u), "borda recente + 100 Hz: válido");
    CHECK_FALSE(flex_signal_valid(500u, 10000u, 500u), "timeout 500 ms: inválido");
    CHECK_FALSE(flex_signal_valid(10u, 0u, 500u), "período 0: inválido");
    CHECK_FALSE(flex_signal_valid(10u, 50000u, 500u), "20 Hz: inválido");
    CHECK_FALSE(flex_signal_valid(10u, 5000u, 500u), "200 Hz: inválido");

    // Stub de host: nunca válido, nunca quebra.
    flex_fuel_init();
    flex_fuel_edge_isr();
    CHECK_FALSE(flex_fuel_valid(), "stub de host: inválido");
    CHECK_EQ(flex_fuel_ethanol_pct(), 0u, "stub de host: 0 %");
    CHECK_EQ(flex_fuel_temp_x10(), 200, "stub de host: 20 °C");
}

void test_ac_request_debounce(void) {
    section("ac_request: debounce de 50 ms");

    AcDebounce d;
    CHECK_FALSE(d.step(false, 0u), "inicial: desligado");
    CHECK_FALSE(d.step(true, 100u), "borda sobe: ainda filtrado");
    CHECK_FALSE(d.step(true, 149u), "49 ms: ainda desligado");
    CHECK_TRUE(d.step(true, 150u), "50 ms: liga");
    CHECK_TRUE(d.step(true, 1000u), "mantém ligado");

    // Glitch curto não desliga.
    CHECK_TRUE(d.step(false, 1100u), "borda desce: ainda ligado");
    CHECK_TRUE(d.step(true, 1120u), "volta antes de 50 ms: permanece ligado");
    CHECK_TRUE(d.step(true, 1400u), "estável ligado");

    // Desliga só após 50 ms estáveis em baixo.
    CHECK_TRUE(d.step(false, 2000u), "desce: ainda ligado");
    CHECK_TRUE(d.step(false, 2049u), "49 ms: ainda ligado");
    CHECK_FALSE(d.step(false, 2050u), "50 ms: desliga");

    // Oscilação rápida nunca liga.
    AcDebounce g;
    bool any = false;
    for (uint32_t t = 0u; t < 1000u; t += 10u) { any |= g.step((t / 10u) & 1u, t); }
    CHECK_FALSE(any, "chatter de 10 ms: nunca liga");

    // Wrap de millis (uint32): diferença continua correta.
    AcDebounce w;
    w.step(false, 0xFFFFFFF0u);
    w.step(true, 0xFFFFFFF8u);
    CHECK_FALSE(w.step(true, 0x00000010u), "wrap: 24 ms ainda filtrado");
    CHECK_TRUE(w.step(true, 0x0000003Eu), "wrap: >=50 ms liga");
}
