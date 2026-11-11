#include "test/harness.h"

#include <cstdint>
#include <cstring>

#include "app/can_rx_map.h"
#include "engine/vehicle_inputs.h"
#include "hal/sd.h"
#include "hal/stm32h562/usb_cdc.h"
#include "hal/uart.h"

using namespace ems::hal;

// Stub de host do UART: contrato usado pelos testes de protocolo (ring de 256 B).
namespace ems::hal {
void uart_test_reset() noexcept;
void uart_test_inject_rx(const uint8_t* data, uint16_t len) noexcept;
uint16_t uart_test_tx_len() noexcept;
const uint8_t* uart_test_tx_buf() noexcept;
}

void test_uart_host_contract(void) {
    section("uart (stub de host): TX/RX, capacidade e espaço livre");

    uart_test_reset();
    uart0_init(115200u);
    uart0_enable_rx();
    uart0_poll_rx();
    CHECK_TRUE(uart0_tx_ready(), "TX sempre pronto no host");
    CHECK_EQ(uart0_tx_free(), 256u, "ring TX vazio: 256 livres");
    CHECK_FALSE(uart0_rx_available(), "RX vazio");

    CHECK_TRUE(uart0_tx_byte(0xA5u), "tx_byte aceita");
    CHECK_TRUE(uart0_tx_push(0x5Au), "tx_push aceita");
    CHECK_EQ(uart_test_tx_len(), 2u, "2 bytes no TX");
    CHECK_TRUE(uart_test_tx_buf()[0] == 0xA5u && uart_test_tx_buf()[1] == 0x5Au, "ordem preservada");
    CHECK_EQ(uart0_tx_free(), 254u, "tx_free acompanha");
    uart0_tx_poll();
    uart0_tx_poll_nb();

    // Enche o TX: push recusa no 257º byte, byte direto é descartado sem travar.
    for (uint16_t i = 2u; i < 256u; ++i) { (void)uart0_tx_push(static_cast<uint8_t>(i)); }
    CHECK_EQ(uart0_tx_free(), 0u, "TX cheio: 0 livres");
    CHECK_FALSE(uart0_tx_push(0x11u), "TX cheio: push recusado");
    CHECK_TRUE(uart0_tx_byte(0x22u), "TX cheio: tx_byte não bloqueia");
    CHECK_EQ(uart_test_tx_len(), 256u, "TX não passa de 256");

    // RX injetado sai em ordem e esgota.
    const uint8_t in[3] = {1u, 2u, 3u};
    uart_test_inject_rx(in, 3u);
    CHECK_TRUE(uart0_rx_available(), "RX disponível");
    uint8_t b = 0u;
    CHECK_TRUE(uart0_rx_pop(b) && b == 1u, "pop 1");
    CHECK_TRUE(uart0_rx_pop(b) && b == 2u, "pop 2");
    CHECK_TRUE(uart0_rx_pop(b) && b == 3u, "pop 3");
    CHECK_FALSE(uart0_rx_pop(b), "pop vazio recusado");
    CHECK_FALSE(uart0_rx_available(), "RX esgotado");

    // Injeção maior que o buffer é truncada em 256.
    static uint8_t big[300];
    for (uint16_t i = 0u; i < 300u; ++i) { big[i] = static_cast<uint8_t>(i); }
    uart_test_inject_rx(big, 300u);
    uint16_t n = 0u;
    while (uart0_rx_pop(b)) { ++n; }
    CHECK_EQ(n, 256u, "injeção truncada em 256 B");

    uart_test_reset();
    CHECK_EQ(uart_test_tx_len(), 0u, "reset limpa o TX");
}

void test_sd_state_machine(void) {
    section("sd (cartão simulado): operações de bloco, latência, falhas");

    static uint8_t wr[kSdBlockBytes], rd[kSdBlockBytes];
    for (uint16_t i = 0u; i < kSdBlockBytes; ++i) { wr[i] = static_cast<uint8_t>(i * 5u + 3u); }

    sd_sim_reset();
    sd_init();
    CHECK_TRUE(sd_state() == SdState::READY, "cartão presente: READY");
    CHECK_TRUE(sd_op_status() == SdOpStatus::IDLE, "sem operação: IDLE");

    // Escrita/leitura imediatas (latência 0).
    CHECK_TRUE(sd_write_start(10u, wr), "write_start aceita");
    CHECK_TRUE(sd_op_status() == SdOpStatus::OK, "latência 0: OK de imediato");
    CHECK_FALSE(sd_read_start(10u, rd), "nova operação antes do ack: recusada");
    sd_op_ack();
    CHECK_TRUE(sd_op_status() == SdOpStatus::IDLE, "ack volta a IDLE");
    CHECK_EQ(sd_sim_write_count(), 1u, "uma escrita contada");
    CHECK_EQ(std::memcmp(sd_sim_block(10u), wr, kSdBlockBytes), 0, "bloco gravado no cartão");

    std::memset(rd, 0, sizeof(rd));
    CHECK_TRUE(sd_read_start(10u, rd), "read_start aceita");
    sd_op_ack();
    CHECK_EQ(std::memcmp(rd, wr, kSdBlockBytes), 0, "leitura devolve o que foi gravado");

    // Latência: BUSY até o número de polls pedido; ack em BUSY não faz nada.
    sd_sim_set_op_latency_polls(3u);
    CHECK_TRUE(sd_write_start(11u, wr), "write com latência aceita");
    CHECK_TRUE(sd_op_status() == SdOpStatus::BUSY, "BUSY durante a latência");
    sd_op_ack();
    CHECK_TRUE(sd_op_status() == SdOpStatus::BUSY, "ack em BUSY é ignorado");
    sd_poll(); sd_poll();
    CHECK_TRUE(sd_op_status() == SdOpStatus::BUSY, "ainda BUSY após 2 polls");
    CHECK_EQ(sd_sim_write_count(), 1u, "nada gravado antes de concluir");
    sd_poll();
    CHECK_TRUE(sd_op_status() == SdOpStatus::OK, "OK após 3 polls");
    CHECK_EQ(sd_sim_write_count(), 2u, "gravação concluída");
    sd_op_ack();
    sd_sim_set_op_latency_polls(0u);

    // Falha de operação → ERROR até o ack; fora do cartão também.
    sd_sim_set_fail_ops(true);
    CHECK_TRUE(sd_write_start(12u, wr), "write aceita mesmo que vá falhar");
    CHECK_TRUE(sd_op_status() == SdOpStatus::ERROR, "operação falha: ERROR");
    sd_op_ack();
    CHECK_TRUE(sd_op_status() == SdOpStatus::IDLE, "ack limpa ERROR");
    sd_sim_set_fail_ops(false);
    CHECK_TRUE(sd_read_start(kSdSimBlocks + 5u, rd), "LBA fora do cartão aceita...");
    CHECK_TRUE(sd_op_status() == SdOpStatus::ERROR, "...mas termina em ERROR");
    sd_op_ack();
    CHECK_TRUE(sd_sim_block(kSdSimBlocks) == nullptr, "sd_sim_block fora do cartão: nullptr");

    // Sem cartão: nada inicia; cartão inserido depois volta a funcionar após init.
    sd_sim_set_present(false);
    CHECK_TRUE(sd_state() == SdState::ABSENT, "sem cartão: ABSENT");
    CHECK_FALSE(sd_write_start(1u, wr), "sem cartão: write recusado");
    CHECK_FALSE(sd_read_start(1u, rd), "sem cartão: read recusado");
    sd_sim_set_present(true);
    sd_init();
    CHECK_TRUE(sd_state() == SdState::READY, "cartão de volta: READY");
    CHECK_TRUE(sd_write_start(1u, wr), "write volta a ser aceito");
    sd_op_ack();

    // Estado inicial do processo: sem cartão (o snapshot de telemetria lê sd_state()).
    sd_sim_reset();
    sd_sim_set_present(false);
}

void test_usb_cdc_host_ring(void) {
    section("usb_cdc (stub de host): ring de TX, controle e orçamento");

    usb_cdc_init();
    CHECK_TRUE(usb_cdc_dtr() && usb_cdc_rts(), "host: DTR/RTS sempre ativos");
    CHECK_FALSE(usb_cdc_configured(), "host: nunca enumerado");
    CHECK_FALSE(usb_cdc_available(), "RX vazio");
    CHECK_EQ(usb_cdc_read_byte(), 0u, "leitura vazia devolve 0");
    uint8_t buf[8];
    CHECK_EQ(usb_cdc_read_bytes(buf, 8u), 0u, "read_bytes vazio: 0");
    CHECK_EQ(usb_cdc_read_bytes(nullptr, 8u), 0u, "buffer nulo: 0");
    CHECK_EQ(usb_cdc_read_bytes(buf, 0u), 0u, "max_len 0: 0");
    CHECK_TRUE(usb_cdc_dbg_stage() >= 1u, "estágio de diagnóstico ≥ 1");

    usb_cdc_poll();                        // esvazia o ring
    const uint16_t cap = usb_cdc_tx_free();
    CHECK_EQ(cap, 511u, "ring de 512 B: 511 úteis");
    usb_cdc_send_byte(0x42u);
    CHECK_EQ(usb_cdc_tx_free(), cap - 1u, "1 byte consome 1");
    static uint8_t blob[100];
    usb_cdc_send_bytes(blob, 100u);
    CHECK_EQ(usb_cdc_tx_free(), cap - 101u, "100 bytes consomem 100");
    usb_cdc_send_bytes(nullptr, 10u);
    CHECK_EQ(usb_cdc_tx_free(), cap - 101u, "ponteiro nulo ignorado");
    usb_cdc_poll();
    CHECK_EQ(usb_cdc_tx_free(), cap, "poll drena o ring");

    // Excesso é descartado (sem estourar nem corromper o ring).
    static uint8_t huge[600];
    usb_cdc_send_bytes(huge, 600u);
    CHECK_EQ(usb_cdc_tx_free(), 0u, "ring cheio: 0 livres");
    usb_cdc_send_byte(0x99u);
    CHECK_EQ(usb_cdc_tx_free(), 0u, "byte extra descartado");
    usb_cdc_poll();
    CHECK_EQ(usb_cdc_tx_free(), cap, "após drenar volta à capacidade total");

    // Orçamento de TX.
    UsbTxBudget a = usb_cdc_tx_budget(100u, 5u, false);
    CHECK_TRUE(a.n == 32u && !a.mirror_usb, "sem enumerar: só UART, teto 32");
    UsbTxBudget c = usb_cdc_tx_budget(100u, 5u, true);
    CHECK_TRUE(c.n == 5u && c.mirror_usb, "enumerado: menor dos dois");
    UsbTxBudget d = usb_cdc_tx_budget(10u, 200u, true);
    CHECK_TRUE(d.n == 10u && d.mirror_usb, "UART menor que USB: usa UART");
    UsbTxBudget z = usb_cdc_tx_budget(0u, 0u, true);
    CHECK_EQ(z.n, 0u, "sem espaço: 0");
}

void test_vehicle_inputs_bridge(void) {
    section("vehicle_inputs: ponte engine → can_rx_map");

    using namespace ems::app;
    CanSignalDef off = {};
    can_rx_map_set(CanRxSignal::GEAR, off);
    can_rx_map_set(CanRxSignal::SPEED_KMH, off);
    can_rx_map_set(CanRxSignal::WHEEL_SPEED_KMH, off);

    uint8_t gear = 99u;
    uint16_t v = 0u;
    CHECK_FALSE(ems::engine::vehicle_gear(gear, 1000u), "marcha não configurada: false");
    CHECK_FALSE(ems::engine::vehicle_speed_kmh(v, 1000u), "velocidade não configurada: false");
    CHECK_FALSE(ems::engine::vehicle_wheel_speed_kmh(v, 1000u), "roda não configurada: false");

    CanSignalDef g = {};
    g.id = 0x300u; g.byte_lo = 0u; g.byte_hi = 0xFFu; g.mask = 0x0Fu; g.timeout_ms = 500u;
    CanSignalDef s = {};
    s.id = 0x301u; s.byte_lo = 0u; s.byte_hi = 1u; s.mask = 0xFFFFu; s.timeout_ms = 500u;
    CanSignalDef w = s;
    w.id = 0x302u;
    can_rx_map_set(CanRxSignal::GEAR, g);
    can_rx_map_set(CanRxSignal::SPEED_KMH, s);
    can_rx_map_set(CanRxSignal::WHEEL_SPEED_KMH, w);

    const uint8_t p_gear[8] = {3u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    const uint8_t p_spd[8]  = {90u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    const uint8_t p_whl[8]  = {120u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    can_rx_map_process(0x300u, p_gear, 8u, 1000u);
    can_rx_map_process(0x301u, p_spd, 8u, 1000u);
    can_rx_map_process(0x302u, p_whl, 8u, 1000u);

    CHECK_TRUE(ems::engine::vehicle_gear(gear, 1100u) && gear == 3u, "marcha 3 via ponte");
    CHECK_TRUE(ems::engine::vehicle_speed_kmh(v, 1100u) && v == 90u, "90 km/h via ponte");
    CHECK_TRUE(ems::engine::vehicle_wheel_speed_kmh(v, 1100u) && v == 120u, "roda 120 km/h via ponte");

    // Timeout propaga pela ponte.
    CHECK_FALSE(ems::engine::vehicle_gear(gear, 2000u), "marcha expirada (>500 ms): false");
    CHECK_FALSE(ems::engine::vehicle_speed_kmh(v, 2000u), "velocidade expirada: false");
    CHECK_FALSE(ems::engine::vehicle_wheel_speed_kmh(v, 2000u), "roda expirada: false");

    // Marcha fora de faixa satura em 6.
    const uint8_t p_hi[8] = {0x0Fu, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    can_rx_map_process(0x300u, p_hi, 8u, 3000u);
    CHECK_TRUE(ems::engine::vehicle_gear(gear, 3000u) && gear == 6u, "marcha 15 satura em 6");

    can_rx_map_set(CanRxSignal::GEAR, off);
    can_rx_map_set(CanRxSignal::SPEED_KMH, off);
    can_rx_map_set(CanRxSignal::WHEEL_SPEED_KMH, off);
}
