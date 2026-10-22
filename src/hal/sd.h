#pragma once

#include <cstdint>

namespace ems::hal {

// Cartão SD em blocos de 512 B, sem sistema de arquivos. Só a VGT6 tem o
// SDMMC1 livre (PC8..PC12 + PD2); na RGT6 esses pinos são INJ3/4 e IGN3/4.
//
// Nada aqui bloqueia: sd_init() arma a máquina de estados e sd_poll() a
// avança com trabalho limitado por chamada. Uma operação por vez.

constexpr uint16_t kSdBlockBytes = 512u;

enum class SdState : uint8_t {
    ABSENT = 0,  // sem cartão (ou ainda não iniciado)
    INIT   = 1,  // identificando o cartão
    READY  = 2,  // pronto para ler e escrever
    FAULT  = 3,  // erro de comando/timeout: não tenta de novo até o reset
};

enum class SdOpStatus : uint8_t {
    IDLE  = 0,  // nenhuma operação pendente
    BUSY  = 1,  // operação em andamento
    OK    = 2,  // terminou bem; aguardando sd_op_ack()
    ERROR = 3,  // falhou; aguardando sd_op_ack()
};

void sd_init() noexcept;
void sd_poll() noexcept;
SdState sd_state() noexcept;

// Inicia a operação sobre um bloco; false se o cartão não está READY ou já
// há operação em curso. `buf` precisa continuar válido até o status sair de BUSY.
bool sd_read_start(uint32_t lba, uint8_t* dst) noexcept;
bool sd_write_start(uint32_t lba, const uint8_t* src) noexcept;
SdOpStatus sd_op_status() noexcept;
void sd_op_ack() noexcept;  // OK/ERROR -> IDLE

#if defined(EMS_HOST_TEST)
// Cartão simulado em RAM (src/hal/sd_sim.cpp).
constexpr uint32_t kSdSimBlocks = 2560u;  // cobre o início da região do datalog + folga

void sd_sim_reset() noexcept;                       // cartão novo, zerado, presente e READY
void sd_sim_set_present(bool present) noexcept;
void sd_sim_set_op_latency_polls(uint32_t polls) noexcept;  // polls em BUSY por operação
void sd_sim_set_fail_ops(bool fail) noexcept;
uint8_t* sd_sim_block(uint32_t lba) noexcept;       // nullptr fora do cartão simulado
uint32_t sd_sim_write_count() noexcept;
#endif

}  // namespace ems::hal
