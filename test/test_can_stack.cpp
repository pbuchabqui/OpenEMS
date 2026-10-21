#include "test/harness.h"

#include <cstdint>

#include "app/can_stack.h"
#include "engine/engine_config.h"
#include "hal/can.h"

using namespace ems::app;
using namespace ems::hal;

namespace {

ems::drv::CkpSnapshot ckp_at(uint32_t rpm) {
    ems::drv::CkpSnapshot c{};
    c.rpm_x10 = rpm * 10u;
    return c;
}

uint32_t count_id(uint32_t id) {
    uint32_t n = 0u;
    for (uint16_t i = 0u; i < can0_test_tx_count(); ++i) {
        if (can0_test_tx_at(i).id == id) { ++n; }
    }
    return n;
}

void wbo2_frame(uint16_t lambda_milli) {
    CanFrame f{};
    f.id = 0x180u;
    f.dlc = 3u;
    f.data[0] = static_cast<uint8_t>(lambda_milli & 0xFFu);
    f.data[1] = static_cast<uint8_t>(lambda_milli >> 8u);
    f.data[2] = 0u;
    can0_test_push_rx(f);
}

}  // namespace

void test_can_stack_tx_single_buffer(void) {
    section("can_stack: com um único buffer de TX nenhum frame é cronicamente recusado");
    can0_test_reset();
    can_stack_init(0x180u);
    can0_test_model_single_tx_buffer(true);   // can0_tx recusa até o "TX complete"
    const ems::drv::SensorData s{};
    // can_stack_process roda no bloco de 20 ms do main; 3 s de simulação.
    for (uint32_t t = 20u; t <= 3000u; t += 20u) {
        can0_test_tx_complete();               // o frame da chamada anterior já saiu
        can_stack_process(t, ckp_at(3000u), s, 10, 50u, 0, 0u, 0u, 0u);
    }
    CHECK_TRUE(count_id(0x400u) >= 100u, "0x400 (10 ms) transmitido com frequência");
    CHECK_TRUE(count_id(0x401u) >= 25u, "0x401 (100 ms) NÃO é mais morto de fome pelo 0x400");
    CHECK_TRUE(count_id(0x402u) >= 5u, "0x402 (500 ms) transmitido");
    can0_test_model_single_tx_buffer(false);
}

void test_can_stack_refused_tx_is_retried(void) {
    section("can_stack: frame recusado não é carimbado — é tentado de novo na chamada seguinte");
    can0_test_reset();
    can_stack_init(0x180u);
    const ems::drv::SensorData s{};
    can0_test_model_single_tx_buffer(true);
    can_stack_process(1000u, ckp_at(3000u), s, 0, 0u, 0, 0u, 0u, 0u);   // aceito (402 vencido)
    const uint16_t after_first = can0_test_tx_count();
    CHECK_TRUE(after_first == 1u, "1 frame por chamada");
    can_stack_process(1020u, ckp_at(3000u), s, 0, 0u, 0, 0u, 0u, 0u);   // buffer ocupado: recusado
    CHECK_EQ(can0_test_tx_count(), after_first, "buffer ocupado: nada novo transmitido");
    can0_test_tx_complete();
    can_stack_process(1040u, ckp_at(3000u), s, 0, 0u, 0, 0u, 0u, 0u);
    CHECK_TRUE(can0_test_tx_count() > after_first, "após liberar o buffer, o frame pendente sai");
    can0_test_model_single_tx_buffer(false);
}

void test_can_stack_fco_uses_real_dt(void) {
    section("can_stack: combustível acumulado (0x402) usa o dt real entre chamadas");
    can0_test_reset();
    can_stack_init(0x180u);
    const ems::drv::SensorData s{};
    // 5,0 ms de pulso (pw_ms_x10=50), 3000 rpm, 4 cil., 450 cc/min:
    // vazão = 50*450*4*3000/72 nl/s = 3750 µl/s → ~3750 µl em 1 s, chamando a cada 20 ms.
    for (uint32_t t = 20u; t <= 1020u; t += 20u) {
        can_stack_process(t, ckp_at(3000u), s, 0, 50u, 0, 0u, 0u, 0u);
    }
    uint32_t accum = 0u;
    bool found = false;
    for (uint16_t i = 0u; i < can0_test_tx_count(); ++i) {
        const CanFrame& f = can0_test_tx_at(i);
        if (f.id == 0x402u) {
            accum = static_cast<uint32_t>(f.data[0]) | (static_cast<uint32_t>(f.data[1]) << 8u) |
                    (static_cast<uint32_t>(f.data[2]) << 16u) | (static_cast<uint32_t>(f.data[3]) << 24u);
            found = true;   // fica com o último
        }
    }
    CHECK_TRUE(found, "0x402 transmitido");
    CHECK_TRUE(accum >= 3000u && accum <= 4000u, "≈3750 µl em 1 s (antes: ~10x menos)");
}

void test_can_stack_fco_uses_configured_flow(void) {
    section("can_stack: 0x402 usa injector_flow_cc_min da calibração");
    const uint16_t saved = ems::engine::cfg::g_eng_cfg.injector_flow_cc_min;
    uint32_t accum[2] = {};
    const uint16_t flows[2] = {450u, 900u};
    for (int k = 0; k < 2; ++k) {
        ems::engine::cfg::g_eng_cfg.injector_flow_cc_min = flows[k];
        can0_test_reset();
        can_stack_init(0x180u);
        const ems::drv::SensorData s{};
        for (uint32_t tm = 20u; tm <= 1020u; tm += 20u) {
            can_stack_process(tm, ckp_at(3000u), s, 0, 50u, 0, 0u, 0u, 0u);
        }
        for (uint16_t i = 0u; i < can0_test_tx_count(); ++i) {
            const CanFrame& f = can0_test_tx_at(i);
            if (f.id == 0x402u) {
                accum[k] = static_cast<uint32_t>(f.data[0]) | (static_cast<uint32_t>(f.data[1]) << 8u) |
                           (static_cast<uint32_t>(f.data[2]) << 16u) | (static_cast<uint32_t>(f.data[3]) << 24u);
            }
        }
    }
    ems::engine::cfg::g_eng_cfg.injector_flow_cc_min = saved;
    CHECK_TRUE(accum[0] > 0u, "450 cc/min acumula");
    CHECK_TRUE(accum[1] >= 2u * accum[0] - 2u && accum[1] <= 2u * accum[0] + 2u, "900 cc/min ≈ 2× o consumo");
}

void test_can_stack_wbo2_fault_reaches_0x401(void) {
    section("can_stack: WBO2_FAULT (bit 9) chega ao 0x401 data[6] bit 1");
    can0_test_reset();
    can_stack_init(0x180u);   // sem frame do WBO2: expira e levanta a falha
    const ems::drv::SensorData s{};
    for (uint32_t tm = 1000u; tm <= 2000u; tm += 20u) {
        can_stack_process(tm, ckp_at(3000u), s, 0, 0u, 0, 0u, 0u, 0u);
    }
    bool found = false;
    uint8_t hi = 0u;
    for (uint16_t i = 0u; i < can0_test_tx_count(); ++i) {
        const CanFrame& f = can0_test_tx_at(i);
        if (f.id == 0x401u) { hi = f.data[6]; found = true; }
    }
    CHECK_TRUE(found, "0x401 transmitido");
    CHECK_TRUE((hi & 0x02u) != 0u, "0x401 data[6] bit 1 = WBO2_FAULT");
}

void test_can_stack_wbo2_plausibility(void) {
    section("can_stack: WBO2 fora de λ 0,4..3,0 não conta como leitura fresca");
    can0_test_reset();
    can_stack_init(0x180u);
    const ems::drv::SensorData s{};
    wbo2_frame(1000u);
    can_stack_process(100u, ckp_at(1000u), s, 0, 0u, 0, 0u, 0u, 0u);
    CHECK_TRUE(can_stack_wbo2_fresh(100u), "λ 1,000 aceito");
    CHECK_EQ(can_stack_lambda_milli(), 1000u, "λ guardado");

    wbo2_frame(0u);
    can_stack_process(200u, ckp_at(1000u), s, 0, 0u, 0, 0u, 0u, 0u);
    CHECK_EQ(can_stack_lambda_milli(), 1000u, "λ 0 ignorado");
    wbo2_frame(0xFFFFu);
    can_stack_process(300u, ckp_at(1000u), s, 0, 0u, 0, 0u, 0u, 0u);
    CHECK_EQ(can_stack_lambda_milli(), 1000u, "λ 65,535 ignorado");

    // Só frames inválidos desde 100 ms: o timeout de 500 ms vale.
    can_stack_process(700u, ckp_at(1000u), s, 0, 0u, 0, 0u, 0u, 0u);
    CHECK_FALSE(can_stack_wbo2_fresh(700u), "sem leitura válida há >500 ms: não fresco");
}

void test_can_rx_ignores_extended_and_remote(void) {
    section("can_stack: frame estendido com bits [28:18]=0x180 não é lido como o WBO2");
    can0_test_reset();
    can_stack_init(0x180u);
    const ems::drv::SensorData s{};
    CanFrame f{};
    f.id = (0x180u << 18u);
    f.dlc = 3u;
    f.data[0] = 0xE8u; f.data[1] = 0x03u;   // λ 1,000
    f.extended = true;
    can0_test_push_rx(f);
    can_stack_process(100u, ckp_at(1000u), s, 0, 0u, 0, 0u, 0u, 0u);
    CHECK_FALSE(can_stack_wbo2_fresh(100u), "estendido: ignorado");
}
