#pragma once

#include <cstdint>

namespace ems::app {

// Sinais CAN RX configuráveis por aplicação
enum class CanRxSignal : uint8_t { GEAR = 0, SPEED_KMH = 1, COUNT = 2 };

// Definição de como extrair um sinal de um frame CAN
struct CanSignalDef {
    uint16_t id;           // ID do frame (0 = sinal desabilitado)
    uint8_t  byte_lo;      // índice do byte LSB no payload (0-7)
    uint8_t  byte_hi;      // índice do byte MSB; 0xFF = apenas 8 bits
    uint8_t  shift_right;  // deslocamento direito após extração
    uint8_t  mask;         // máscara após shift (ex.: 0x0F para nibble de 4 bits)
    int16_t  offset;       // offset aditivo ao valor mascarado
    uint16_t timeout_ms;   // ms sem frame → sinal inválido (0 = sem timeout)
};

// Configura/lê definição de sinal (chamado pelo servidor HTTP ao salvar parâmetros)
void         can_rx_map_set(CanRxSignal sig, const CanSignalDef& def) noexcept;
CanSignalDef can_rx_map_get(CanRxSignal sig) noexcept;

// Processa frame recebido; chamado internamente pelo can_stack
void can_rx_map_process(uint16_t id, const uint8_t* data, uint8_t dlc,
                        uint32_t now_ms) noexcept;

// Leitura dos sinais decodificados
// Retorna false se sinal não configurado (id=0) ou em timeout
bool can_rx_gear(uint8_t&  out_gear, uint32_t now_ms) noexcept;
bool can_rx_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept;

} // namespace ems::app
