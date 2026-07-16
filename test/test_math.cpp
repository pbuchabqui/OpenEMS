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

void test_math_req_fuel(void) {
    // calc_req_fuel_us formula:
    //   num = displacement_cc × kAirDensityMgPerCcX1000(1184) × 100 × 60000000
    //   den = cylinders × stoich_afr_x100 × injector_flow_cc_min × kFuelDensityMgPerCc(755) × 1000
    //   result = num / den (integer)
    //
    // Derivado independentemente em Python:
    //   num = 1998×1184×100×60000000 = 14193792000000000
    //   den = 4×1470×440×755×1000   = 1953336000000
    //   result = 7266 µs
    section("MATH: calc_req_fuel_us formula exacta");
    CHECK_EQ(calc_req_fuel_us(1998u, 4u, 440u, 1470u), 7266u,
             "req_fuel(1998cc,4cyl,440cc/min,AFR14.7) = 7266µs");

    // Caso de proporcionalidade: deslocamento ÷ 2 → req_fuel ÷ 2
    // calc_req_fuel_us(999,4,440,1470) = 7266/2... mas integer, verificamos proporcionalidade
    const uint32_t req_half = calc_req_fuel_us(999u, 4u, 440u, 1470u);
    CHECK_TRUE(req_half >= 3630u && req_half <= 3634u,
               "half displacement → approx half req_fuel (3632±2)");

    // Injetor 2× mais caudal → req_fuel ÷ 2
    const uint32_t req_big_inj = calc_req_fuel_us(1998u, 4u, 880u, 1470u);
    CHECK_TRUE(req_big_inj >= 3630u && req_big_inj <= 3634u,
               "2× injector flow → approx half req_fuel (3633±2)");

    // Guards: zero inputs
    CHECK_EQ(calc_req_fuel_us(0u, 4u, 440u, 1470u), 0u, "displacement=0 → 0");
    CHECK_EQ(calc_req_fuel_us(1998u, 0u, 440u, 1470u), 0u, "cylinders=0 → 0");
}

void test_math_base_pw(void) {
    // calc_base_pw_us formula: pw = req_fuel × ve/100 × map/map_ref
    // With req=7266, ve=80, map=100, map_ref=100:
    //   num = 7266×80×100 = 58128000
    //   den = 100×100 = 10000
    //   result = 5812
    section("MATH: calc_base_pw_us formula exacta");
    CHECK_EQ(calc_base_pw_us(7266u, 80u, 100u, 100u), 5812u,
             "base_pw(req=7266,VE=80%,MAP=REF) = 5812µs");

    // VE=100%, MAP=REF → pw = req (identidade)
    CHECK_EQ(calc_base_pw_us(7266u, 100u, 100u, 100u), 7266u,
             "VE=100% MAP=REF → pw = req_fuel (identidade)");

    // MAP = 2×REF → pw = req×ve×2 = 7266×80×200/10000 = 11625
    //   num = 7266×80×200 = 116256000, den=10000, result=11625
    CHECK_EQ(calc_base_pw_us(7266u, 80u, 200u, 100u), 11625u,
             "MAP=2×REF → pw doubles proportionally (11625µs)");

    // MAP = REF/2 → pw halves: 7266×80×50/10000 = 2906
    CHECK_EQ(calc_base_pw_us(7266u, 80u, 50u, 100u), 2906u,
             "MAP=REF/2 → pw halves (2906µs)");

    // Proporcionalidade VE: VE=40 deve dar metade de VE=80
    const uint32_t pw80 = calc_base_pw_us(7266u, 80u, 100u, 100u);  // 5812
    const uint32_t pw40 = calc_base_pw_us(7266u, 40u, 100u, 100u);  // 2906
    CHECK_EQ(pw80, 2u * pw40, "VE=80 = 2×VE=40 (proporcionalidade linear)");
}

void test_math_lambda_pw(void) {
    // apply_lambda_target_pw_us: pw_out = base × 1000 / lambda_target
    //   base=5000, lambda=850 → 5000×1000/850 = 5000000/850 = 5882
    //   base=5000, lambda=1200 → 5000000/1200 = 4166
    section("MATH: apply_lambda_target_pw_us formula exacta");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 850u), 5882u,
             "lambda=0.850 → 5000×1000/850 = 5882µs");
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1200u), 4166u,
             "lambda=1.200 → 5000×1000/1200 = 4166µs");
    // lambda=1.000 → identidade
    CHECK_EQ(apply_lambda_target_pw_us(5000u, 1000u), 5000u,
             "lambda=1.000 → pw inalterado");
    // Proporcionalidade inversa: pw × lambda = constante (base × 1000)
    // lambda deve estar em [650,1200]; usamos 800 e 1000.
    // pw(800) = 6000×1000/800 = 7500;  pw(1000) = 6000×1000/1000 = 6000
    // pw_a×lambda_a = 6000×1000=6000000 = pw_b×lambda_b = 7500×800=6000000
    const uint32_t pw_a = apply_lambda_target_pw_us(6000u, 1000u);  // 6000
    const uint32_t pw_b = apply_lambda_target_pw_us(6000u,  800u);  // 7500
    CHECK_EQ(pw_a * 1000u, pw_b * 800u,
             "lambda proporcionalidade inversa: pw×lambda=constante (base×1000)");
}

void test_math_table3d_bilinear(void) {
    using namespace ems::engine;
    section("MATH: table3d bilinear interpolation 2D (fx>0, fy>0)");

    // Ponto: x=6250, y=25.
    // kRpmAxisX10:        [0]=5000, [1]=7500  → xi=0, fx=(6250-5000)×256/2500=128
    // kLoadAxisBarX100:   [0]=20,   [1]=30    → yi=0, fy=(25-20)×256/10=128
    //
    // Tabela (só as 4 células que importam, resto=0):
    //   [yi][xi]=[0][0]=10  [0][1]=20
    //   [1][0]=30           [1][1]=40
    //
    // Cálculo:
    //   v0 = lerp(10,20,128) = 10 + 10×128/256 = 15
    //   v1 = lerp(30,40,128) = 30 + 10×128/256 = 35
    //   v  = lerp(15,35,128) = 15 + 20×128/256 = 25
    static uint8_t tbl_u8[kTableAxisSize][kTableAxisSize] = {};
    tbl_u8[0][0] = 10u; tbl_u8[0][1] = 20u;
    tbl_u8[1][0] = 30u; tbl_u8[1][1] = 40u;

    const Table2dLookup lk_mid = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 6250u, 25u);
    CHECK_EQ(lk_mid.fx_q8, 128u, "fx=128 at x=6250 (midpoint axis[0]=5000..axis[1]=7500)");
    CHECK_EQ(lk_mid.fy_q8, 128u, "fy=128 at y=25 (midpoint axis[0]=20..axis[1]=30)");

    const uint8_t result = table3d_lookup_u8_prepared(tbl_u8, lk_mid);
    CHECK_EQ(result, 25u, "bilinear([10,20/30,40],fx=fy=128) = 25");

    // Verifica é realmente 2D: alterar apenas fy modifica o resultado
    const Table2dLookup lk_bottom = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 6250u, 20u);  // fy=0 (linha inferior)
    const uint8_t result_bottom = table3d_lookup_u8_prepared(tbl_u8, lk_bottom);
    CHECK_EQ(result_bottom, 15u,
             "bilinear com fy=0: lerp(10,20,128)=15 (só linha yi=0)");

    const Table2dLookup lk_top = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 6250u, 30u);  // fy=255 (linha superior)
    const uint8_t result_top = table3d_lookup_u8_prepared(tbl_u8, lk_top);
    CHECK_EQ(result_top, 35u,
             "bilinear com fy=255: lerp(30,40,128)=35 (linha yi=1 por frac=255)");

    // Caso i8 com valores negativos: [0][0]=-20,[0][1]=0,[1][0]=0,[1][1]=20
    // v0=lerp(-20,0,128)=-10, v1=lerp(0,20,128)=10, v=lerp(-10,10,128)=0
    static int8_t tbl_i8[kTableAxisSize][kTableAxisSize] = {};
    tbl_i8[0][0] = -20; tbl_i8[0][1] = 0;
    tbl_i8[1][0] =   0; tbl_i8[1][1] = 20;
    const int16_t result_i8 = table3d_lookup_i8_prepared(tbl_i8, lk_mid);
    CHECK_EQ(result_i8, 0, "bilinear i8([-20,0/0,20],fx=fy=128) = 0 (centro)");

    section("MATH: table3d frac=255 boundary (lerp retorna valor exato de b)");
    // Sem o caso especial frac=255, lerp(0,100,255)=99; com ele retorna 100.
    static uint8_t tbl_bound[kTableAxisSize][kTableAxisSize] = {};
    tbl_bound[0][0] =   0u;
    tbl_bound[0][1] = 100u;  // xi+1, yi=0
    // x=7485 → frac=(7485-5000)×256/2500=2485×256/2500=254 → lerp(0,100,254)=99
    const Table2dLookup lk_254 = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 7485u, 20u);
    CHECK_EQ(lk_254.fx_q8, 254u, "x=7485 → frac=254 (abaixo do caso especial)");
    const uint8_t r254 = table3d_lookup_u8_prepared(tbl_bound, lk_254);
    CHECK_EQ(r254, 99u, "lerp(0,100,254)=99 (truncamento normal)");

    // x=7500 → frac=255 → caso especial retorna b=100 (não 99)
    const Table2dLookup lk_255 = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 7500u, 20u);
    CHECK_EQ(lk_255.fx_q8, 255u, "x=7500=axis[1] → frac=255 (caso especial)");
    const uint8_t r255 = table3d_lookup_u8_prepared(tbl_bound, lk_255);
    CHECK_EQ(r255, 100u, "lerp(0,100,255)=100 (caso especial frac=255 retorna b exato)");
}

void test_math_corrections(void) {
    section("MATH: corr_clt valores exactos (interp linear em tabela)");
    // clt_corr_axis_x10 = {-400,-100,0,200,...}
    // clt_corr_x256     = {384, 352,320,288,...}
    // Ponto -250 (midpoint entre -400 e -100):
    //   frac = (-250-(-400))×256/(-100-(-400)) = 150×256/300 = 128
    //   lerp(384,352,128) = 384 + (352-384)×128/256 = 384-16 = 368
    CHECK_EQ(corr_clt(-250), 368u, "corr_clt(-250) = 368 (interp exacta)");

    // Ponto extremo inferior: usa clt_corr_x256[0]=384
    CHECK_EQ(corr_clt(-400), 384u, "corr_clt(-400) = 384 (valor eixo 0)");

    section("MATH: corr_iat valores exactos");
    // iat_corr_axis_x10 = {-200,0,200,400,600,800,1000,1200}
    // iat_corr_x256     = {272, 264,256,256,264,272, 280, 288}
    // Ponto 100 (midpoint 0..200):
    //   frac = 100×256/200 = 128
    //   lerp(264,256,128) = 264 + (256-264)×128/256 = 264-4 = 260
    CHECK_EQ(corr_iat(100), 260u, "corr_iat(100) = 260 (interp exacta)");
    CHECK_EQ(corr_iat(200), 256u, "corr_iat(200) = 256 (valor ref, frac=255→b)");

    section("MATH: corr_vbatt valores exactos (dead-time do injector)");
    // vbatt_corr_axis_mv  = {9000,...,12000,13000,...}
    // injector_dead_time  = {1400,..., 900,  800,...}
    // Ponto 12500 (midpoint 12000..13000):
    //   frac = 500×256/1000 = 128
    //   lerp(900,800,128) = 900 + (800-900)×128/256 = 900-50 = 850
    CHECK_EQ(corr_vbatt(12500u), 850u, "corr_vbatt(12500mV) = 850µs (interp exacta)");

    // Monotonícidade: vbatt mais baixo → dead-time maior
    CHECK_TRUE(corr_vbatt(9000u) > corr_vbatt(16000u),
               "dead-time monotonicamente decrescente com vbatt");
    CHECK_TRUE(corr_vbatt(11000u) > corr_vbatt(13000u),
               "dead-time(11V) > dead-time(13V)");
}

void test_math_stft_gains(void) {
    section("MATH: fuel_update_stft ganhos Kp e Ki exactos");
    // Kp = kStftKpNum/100 = 3/100. Por error=200 x1000: p = 200×3/100 = 6 x10
    // Ki = kStftKiNum/kStftKiDen = 1/200. Por error=200 x1000: i += 200/200 = 1 x10
    // Após N chamadas com erro constante de 200 x1000 (20% lean):
    //   integrator = N (sem clamp), stft = 6 + N
    // Verificar após 1 chamada: stft=7; após 10: stft=16; após 300: stft=250 (clampado)

    fuel_reset_adaptives();
    // 1 chamada: p=6, integrator=1, stft=7
    const int16_t s1 = fuel_update_stft(
        30000u, 100u, 1000, 1200, 900, true, false, false, 30000u, 500u);
    CHECK_EQ(s1, 7, "STFT após 1 call (lean 20%): p=6 + I=1 = 7");

    // Após 9 calls adicionais (total 10): integrator=10, stft=16
    for (int i = 0; i < 9; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false, 30000u, 500u);
    }
    CHECK_EQ(fuel_get_stft_pct_x10(), 16, "STFT após 10 calls: p=6 + I=10 = 16");

    section("MATH: fuel_update_stft clamp kStftClampX10=250");
    // Após 300 chamadas: integrator clamped=250, stft=clamp(6+250,-250,250)=250
    for (int i = 0; i < 290; ++i) {
        fuel_update_stft(30000u, 100u, 1000, 1200, 900, true, false, false, 30000u, 500u);
    }
    CHECK_EQ(fuel_get_stft_pct_x10(), 250, "STFT saturado no clamp kStftClampX10=250");

    section("MATH: fuel_update_stft congela (freeze) quando loop fechado desabilitado");
    // Com CLT fria (clt=600 < 700=70°C): closed_loop_allowed=false.
    // Anti-windup: stft congela no último valor (250, saturado no clamp acima),
    // não decai — evita degrau de combustível ao voltar a closed-loop.
    const int16_t s_cold1 = fuel_update_stft(
        30000u, 100u, 1000, 1200, 600, true, false, false, 30000u, 500u);
    CHECK_EQ(s_cold1, 250, "STFT congelado após 1 chamada fria: mantém 250");
    const int16_t s_cold2 = fuel_update_stft(
        30000u, 100u, 1000, 1200, 600, true, false, false, 30000u, 500u);
    CHECK_EQ(s_cold2, 250, "STFT congelado após 2 chamadas frias: mantém 250");
}

void test_math_inj_scheduler_ticks(void) {
    section("MATH: inj_pw_us_to_scheduler_ticks (host: ×60, prod: ×10)");
    using namespace ems::engine;
    // Em host (não TARGET_STM32H562): retorna pw_us × 60
    // Em producão (STM32H562): retorna pw_us × 10 (100 ns/tick @ 10 MHz)
    // Nota: o branch de host usa factor 60 (legado de outro target);
    // a formula de producao (factor 10) não é testada no host.
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(1000u), 60000u,
             "1000µs × 60 = 60000 ticks (host branch)");
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(100u), 6000u,
             "100µs × 60 = 6000 ticks (host branch)");
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(0u), 0u,
             "0µs → 0 ticks");
    // Proporcionalidade: dobrar PW dobra ticks
    CHECK_EQ(inj_pw_us_to_scheduler_ticks(2000u), 2u * inj_pw_us_to_scheduler_ticks(1000u),
             "proporcionalidade linear");
}

void test_math_xtau_convergence(void) {
    section("MATH: transient_fuel_xtau steady-state injected = desired");
    // Prova analítica: em regime permanente
    //   injected = (desired - wall/tau) × 256/(256-x)
    //   wall/tau = desired × x/256 (regime permanente)
    //   => injected = desired × (256-x)/256 × 256/(256-x) = desired
    // Após 200 ciclos com entrada constante de 5000µs @ clt=800:
    //   x_q8=32 (interp entre [5]=35 e [6]=28 com frac=128)
    //   tau=11  (interp entre [5]=12 e [6]=10 com frac=128)
    //   Convergencia em ~5×tau=55 ciclos; a 200 ciclos é estado permanente.
    transient_fuel_reset();
    uint32_t out = 0u;
    for (int i = 0u; i < 200; ++i) {
        out = transient_fuel_xtau_update(5000u, 800, true);
    }
    // Tolera ±20µs por truncamento Q8 (wall_q8 em aritmética inteira)
    CHECK_TRUE(out >= 4980u && out <= 5020u,
               "xtau steady-state: injected ≈ desired=5000 (±20µs)");

    section("MATH: transient_fuel_xtau transitorio — overshot no arranque (wall=0)");
    // Com wall=0 na chamada inicial: evap=0, numerator=desired,
    // injected = desired × 256/(256-x_q8) = 5000×256/224 ≈ 5714µs.
    // (x_q8=32 → dry_fraction=256-32=224)
    transient_fuel_reset();
    const uint32_t first_out = transient_fuel_xtau_update(5000u, 800, true);
    // first_out deve ser > desired (wall vazia → injetor compensa parede)
    CHECK_TRUE(first_out > 5000u,
               "xtau arranque: 1ª injecao > desired (parede vazia → overshot)");
    // E inferior ao limite de clamp (100ms)
    CHECK_TRUE(first_out <= 100000u, "xtau 1ª injeção ≤ 100ms clamp");
}

void test_math_production_tables(void) {
    using namespace ems::engine;
    section("MATH: get_ve / get_lambda / get_advance com tabelas reais");
    // Ponto (3000 RPM = rpm_x10=30000, MAP=100 kPa = map_bar_x100=100).
    // Eixos 20×20: kRpmAxisX10[10]=30000 → floor idx=9, frac=255
    //              kLoadAxisBarX100[11]=100 → floor idx=10, frac=255
    // Frac=255 → lerp devolve o canto alto exacto: ve_table[11][10]=88.
    CHECK_EQ(get_ve(30000u, 100u), 88u,
             "get_ve(3000RPM,100kPa) = 88 (tabela real, frac=255 ambos eixos)");

    // Prepared == direct para mesmo ponto
    const Table2dLookup lk = table3d_prepare_lookup(
        kRpmAxisX10, kLoadAxisBarX100, 30000u, 100u);
    CHECK_EQ(get_ve_prepared(lk), 88u, "get_ve_prepared == 88");

    // lambda_target_table[6]=all 1000; [7]=all 990
    //   v0=lerp(1000,1000,255)=1000; v1=lerp(990,990,255)=990; v=lerp(1000,990,255)=990
    CHECK_EQ(get_lambda_target_x1000(30000u, 100u), 990u,
             "get_lambda(3000RPM,100kPa) = 990 (tabela real)");
    CHECK_EQ(get_lambda_target_x1000_prepared(lk), 990u,
             "get_lambda_prepared == 990");

    // spark_table[6][6]=19,[6][7]=18,[7][6]=17,[7][7]=16
    //   v0=lerp(19,18,255)=18; v1=lerp(17,16,255)=16; v=lerp(18,16,255)=16
    CHECK_EQ(get_advance(30000u, 100u), 16,
             "get_advance(3000RPM,100kPa) = 16\u00b0 (tabela real)");
    CHECK_EQ(get_advance_prepared(lk), 16,
             "get_advance_prepared == 16");

    section("MATH: corr_warmup valores exactos");
    // warmup_corr_axis_x10={-400,-100,0,...}, warmup_corr_x256={420,380,350,...}
    // Eixo exacto -400 → 420; eixo exacto -100: idx=0,frac=255 → lerp(420,380,255)=380
    CHECK_EQ(corr_warmup(-400), 420u, "corr_warmup(-40\u00b0C) = 420");
    CHECK_EQ(corr_warmup(-100), 380u, "corr_warmup(-10\u00b0C) = 380 (eixo[1], frac=255)");
    // Midpoint entre -400 e -100 (-250): frac=150\u00d7256/300=128
    // lerp(420,380,128) = 420 + (380-420)\u00d7128/256 = 420-20 = 400
    CHECK_EQ(corr_warmup(-250), 400u, "corr_warmup(-25\u00b0C) = 400 (interp exacta)");

    section("MATH: dwell_ms_x10_from_vbatt valor exacto");
    // dwell_vbatt_axis_mv={9000,...,12000,...},dwell_ms_x10={42,...,30,...}
    // 12000 = axis[3]: idx=2, frac=255 → lerp(35,30,255)=30
    CHECK_EQ(dwell_ms_x10_from_vbatt(12000u), 30u,
             "dwell @ 12V = 30 x10 (3.0ms, valor exacto da tabela)");
    CHECK_EQ(dwell_ms_x10_from_vbatt(9000u),  42u,
             "dwell @ 9V = 42 x10 (4.2ms, limite inferior)");

    section("MATH: dwell_ms_x10_from_vbatt_rpm correc\u00e7\u00e3o por RPM");
    // dwell_rpm_axis_rpm={500,1200,4000,7000}, factor_q8={384,288,256,200}
    // rpm_x10=5000 → RPM=500 = axis[0]: frac=0 (value<=axis[0]) → factor=384
    // dwell_final = 30 \u00d7 384 / 256 = 45
    CHECK_EQ(dwell_ms_x10_from_vbatt_rpm(12000u, 5000u), 45u,
             "dwell @ 12V, 500 RPM = 45 x10 (1.5\u00d7 factor de arranque)");
    // rpm_x10=70000 → RPM=7000 = axis[3]: frac=255 → factor=200
    // dwell_final = 30 \u00d7 200 / 256 = 23 (integer: 6000/256=23)
    CHECK_EQ(dwell_ms_x10_from_vbatt_rpm(12000u, 70000u), 23u,
             "dwell @ 12V, 7000 RPM = 23 x10 (0.78\u00d7 factor alto RPM)");

    section("MATH: calc_ae_pw_from_tpsdot formula exacta");
    // Force legacy-style axis for closed-form check (defaults are 30/80/200/500).
    const uint16_t saved_axis[4] = {
        ae_tpsdot_axis_x10[0], ae_tpsdot_axis_x10[1],
        ae_tpsdot_axis_x10[2], ae_tpsdot_axis_x10[3]};
    const uint16_t saved_add[4] = {
        ae_pw_adder_us[0], ae_pw_adder_us[1],
        ae_pw_adder_us[2], ae_pw_adder_us[3]};
    ae_tpsdot_axis_x10[0] = 5u;  ae_tpsdot_axis_x10[1] = 20u;
    ae_tpsdot_axis_x10[2] = 50u; ae_tpsdot_axis_x10[3] = 100u;
    ae_pw_adder_us[0] = 300u; ae_pw_adder_us[1] = 800u;
    ae_pw_adder_us[2] = 1500u; ae_pw_adder_us[3] = 2500u;
    // tpsdot_x10=30 (%/s×10) on axis {5,20,50,100}: between 20 and 50
    //   frac = (30-20)×256/(50-20) = 85
    //   base_pw = lerp(800,1500,85) = 1032
    // clt=800 → bucket 5 → ae_clt_sens[5]=6 → 1032×6/8 = 774
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    fuel_ae_set_taper(4u);
    CHECK_EQ(calc_ae_pw_from_tpsdot(30, 800), 774,
             "calc_ae_pw_from_tpsdot(30,clt=800): 1032×6/8=774µs");
    // DE: same magnitude / 2, negative
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    CHECK_EQ(calc_ae_pw_from_tpsdot(static_cast<int16_t>(-30), 800), -387,
             "DE tip-out: −(774/2)=−387µs");
    // calc_ae_pw_us: delta 300 in 10ms → tpsdot = 300×1000/10 = 30000 → clamp 1000
    fuel_reset_adaptives();
    fuel_ae_set_threshold(10u);
    CHECK_TRUE(calc_ae_pw_us(800u, 500u, 10u, 800) > 0,
               "calc_ae_pw_us large step → ae > 0 (tpsdot in %/s×10)");
    for (int i = 0; i < 4; ++i) {
        ae_tpsdot_axis_x10[i] = saved_axis[i];
        ae_pw_adder_us[i] = saved_add[i];
    }
}

void test_math_misfire_threshold(void) {
    using namespace ems::engine;
    section("MATH: misfire threshold kMisfireThresholdQ8=287 (=1.12×256)");
    // Threshold de detecção: power_sum > (predicted_sum × 287) >> 8
    // = predicted_sum × 1.12109...
    //
    // Com kMisfireWindowTeeth=10, kNormalPeriod=10000 ticks, 16ns/tick:
    //   normal_ns      = 160000
    //   predicted_sum  = 10 × 160000 = 1600000
    //   threshold      = (1600000 × 287) >> 8 = 1793750
    //
    //   1.2× slow: power_sum = 10×192000 = 1920000 > 1793750 → misfire
    //   1.1× slow: power_sum = 10×176000 = 1760000 < 1793750 → NO misfire

    misfire_init();
    misfire_reset();

    const uint32_t normal_ns = kNormalPeriod * 16u;   // 160000 ns
    const uint32_t slow_1_2  = static_cast<uint32_t>(normal_ns * 12u / 10u);  // 192000 ns
    const uint32_t slow_1_1  = static_cast<uint32_t>(normal_ns * 11u / 10u);  // 176000 ns

    // Snap para cyl0: phase_A=true, tooth_index 0..9
    // g_tooth_to_cyl[0][0..kMisfireWindowTeeth-1] = cyl0 (mapa preenchido por misfire_init)
    auto feed_window = [&](uint32_t period_ns, uint32_t pred_ns) {
        ems::drv::CkpSnapshot sn{};
        sn.state                    = ems::drv::SyncState::FULL_SYNC;
        sn.phase_A                  = true;
        sn.tooth_period_ns          = period_ns;
        sn.predicted_tooth_period_ns = pred_ns;
        for (uint8_t t = 0u; t < kMisfireWindowTeeth; ++t) {
            sn.tooth_index = t;
            ems::drv::misfire_on_tooth(sn);
        }
    };

    // Alimentar 3 janelas lentas (1.2×) → kMisfireDebounceCycles=3 → evento confirmado
    for (uint8_t w = 0u; w < kMisfireDebounceCycles; ++w) {
        feed_window(slow_1_2, normal_ns);
    }
    CHECK_TRUE(misfire_get_event_count(0u) >= 1u,
               "misfire detectado: 1.2× (power=1920000 > threshold=1793750)");

    misfire_reset();
    // 1.1× slow: power_sum=1760000 < threshold=1793750 → NÃO deve detectar
    for (uint8_t w = 0u; w < kMisfireDebounceCycles; ++w) {
        feed_window(slow_1_1, normal_ns);
    }
    CHECK_EQ(misfire_get_event_count(0u), 0u,
             "sem misfire: 1.1× (power=1760000 < threshold=1793750)");

    section("MATH: misfire threshold sensitivity — proporcionalidade");
    // Verificar que o threshold é exactamente (predicted × 287) >> 8
    // ao calcular a janela mínima de detecção manualmente:
    //   threshold = (1600000 × 287) >> 8 = 459200000 >> 8 = 1793750
    //   min_power_per_tooth = 1793750/10 = 179375 ns (per tooth)
    //   min_period_factor = 179375 / 160000 = 1.1210... (= 287/256)
    const uint32_t threshold_check = (static_cast<uint64_t>(kMisfireWindowTeeth) *
                                       normal_ns * kMisfireThresholdQ8) >> 8u;
    CHECK_EQ(threshold_check, 1793750u, "threshold = predicted_sum × 287/256 = 1793750");
}

void test_trigger_offset(void) {
    using namespace ems::engine::cfg;

    // ----------------------------------------------------------------
    // Baseline: offset = 0°
    // TDC 540° = posição 3 da firing order {0,2,3,1} = cilindro físico 1
    // → canal ECU_CH_IGN2 (convenção: canal = cilindro físico).
    // Com advance=15°, dwell=84°:
    //   dwell_engine_angle = (525+720-84)%720 = 441°
    //   trigger_angle(441, offset=0) = (441+720-0)%720 = 441°
    //   ang=81, pos_x256=81*256/6=3456, tooth=13, frac=128, phase_B
    // ----------------------------------------------------------------
    section("trigger offset=0°: cyl1 (tdc=540°) DWELL_START scheduled at tooth 13");
    g_eng_cfg.trigger_tooth0_engine_deg = 0u;
    ecu_sched_test_reset();
    ecu_sched_test_reset_ccr();
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // Force cmp_confirms>=2 so Calculate_Sequential_Cycle() runs (not presync).
    ckp_test_set_cmp_confirms(2u);
    ckp_feed_n_then_gap(55u);  // trigger sequential scheduling at next gap
    ecu_sched_test_reset_ccr();

    // Inspect angle table: find ECU_CH_IGN2 DWELL_START entry (cyl 1, tdc=540°)
    uint8_t tooth_off0 = 0xFFu;
    for (uint8_t i = 0u; i < ecu_sched_test_angle_table_size(); ++i) {
        uint8_t t, f, ch, act, ph;
        if (ecu_sched_test_get_angle_event(i, &t, &f, &ch, &act, &ph)) {
            if (ch == ECU_CH_IGN2 && act == ECU_ACT_DWELL_START) { tooth_off0 = t; }
        }
    }
    CHECK_EQ(tooth_off0, 13u,
             "offset=0°: cyl1 DWELL_START tooth=13 (trigger=441°, ang=81, 81*256/6=3456>>8=13)");

    // Events from tooth 0 (gap) may already be in queue; reset for clean check.
    ecu_sched_test_reset_ccr();

    // Fire 13 teeth → at tooth 13 DWELL_START for cyl3 fires → event inserted.
    for (uint32_t i = 0u; i < 13u; ++i) { ckp_fire(kNormalPeriod); }
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u || ecu_sched_test_get_tim5_ccr3() > 0u,
               "offset=0°: TIM5 event queued after tooth 13 DWELL_START");

    // ----------------------------------------------------------------
    // Non-zero offset: 78°
    // trigger_angle(441, offset=78) = (441+720-78)%720 = 363°
    //   ang=363%360=3, pos_x256=3*256/6=128, tooth=0, frac=128, phase_B
    // Event shifts to tooth 0 → fires AT the FULL_SYNC gap (tooth 0 is
    // processed immediately when Calculate_Sequential_Cycle completes).
    // ----------------------------------------------------------------
    section("trigger offset=78°: cyl1 DWELL_START shifts to tooth 0");
    g_eng_cfg.trigger_tooth0_engine_deg = 78u;
    ecu_sched_test_reset();
    ecu_sched_test_reset_ccr();
    ecu_sched_test_set_tim2_cnt(1000u);
    ecu_sched_set_advance_deg(15u);
    ecu_sched_set_dwell_ticks(140625u);
    ecu_sched_set_inj_pw_ticks(125000u);
    ecu_sched_set_eoi_lead_deg(60u);
    g_ckp_cap = 0u;
    ckp_reach_full_sync();
    // Force cmp_confirms>=2 so Calculate_Sequential_Cycle() runs (not presync).
    ckp_test_set_cmp_confirms(2u);
    // 57 teeth: presync SPARK at tooth 57 clears arm_ticks; gap triggers sequential.
    // Do NOT call reset_ccr after — event at tooth 0 fires during the gap itself.
    ckp_feed_n_then_gap(57u);

    // Angle table: cyl1 (tdc=540°, canal IGN2) DWELL_START must be at tooth 0
    uint8_t tooth_off78 = 0xFFu;
    for (uint8_t i = 0u; i < ecu_sched_test_angle_table_size(); ++i) {
        uint8_t t, f, ch, act, ph;
        if (ecu_sched_test_get_angle_event(i, &t, &f, &ch, &act, &ph)) {
            if (ch == ECU_CH_IGN2 && act == ECU_ACT_DWELL_START) { tooth_off78 = t; }
        }
    }
    CHECK_EQ(tooth_off78, 0u,
             "offset=78°: cyl1 DWELL_START shifts to tooth 0 (trigger=363°, ang=3, 3*256/6=128>>8=0)");
    CHECK_TRUE(tooth_off0 != tooth_off78,
               "offset change moves the event: tooth 13 (offset=0) ≠ tooth 0 (offset=78)");

    // With offset=78°, cyl3 DWELL_START fires at tooth 0 (at the FULL_SYNC gap).
    // Verify event was queued immediately at FULL_SYNC.
    CHECK_TRUE(ecu_sched_test_get_evt_count() > 0u || ecu_sched_test_get_tim5_ccr3() > 0u,
               "offset=78°: TIM5 event queued immediately at FULL_SYNC gap (tooth 0)");

    // Confirm: with offset=0 the CCR was 0 at gap; with offset=78 it is 1800 at gap.
    // Both CCR values are numerically 1800 because frac=128 in both cases,
    // but offset=0 fires 13 teeth LATER whereas offset=78 fires IMMEDIATELY.
    // The table inspection above is the definitive proof of timing difference.

    // Restore default so subsequent tests are unaffected
    g_eng_cfg.trigger_tooth0_engine_deg = 0u;
}

