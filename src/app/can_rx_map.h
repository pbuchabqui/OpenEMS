#pragma once

#include <cstdint>

namespace ems::app {

// Sinais CAN RX configuráveis por aplicação
// SPEED_KMH     = velocidade de referência (carro/ABS body), km/h
// WHEEL_SPEED_KMH = roda motriz (ou mais rápida), km/h — para slip TC
enum class CanRxSignal : uint8_t {
    GEAR            = 0,
    SPEED_KMH       = 1,
    WHEEL_SPEED_KMH = 2,
    COUNT           = 3
};

// Definição de como extrair um sinal de um frame CAN
struct CanSignalDef {
    uint16_t id;           // ID do frame (0 = sinal desabilitado)
    uint8_t  byte_lo;      // índice do byte LSB no payload (0-7)
    uint8_t  byte_hi;      // índice do byte MSB; 0xFF = apenas 8 bits
    uint8_t  shift_right;  // deslocamento direito após extração
    uint8_t  _pad;         // keep wire packing aligned (was unused high of mask)
    uint16_t mask;         // máscara após shift (0xFFFF = full 16-bit km/h)
    int16_t  offset;       // offset aditivo ao valor mascarado
    uint16_t timeout_ms;   // ms sem frame → sinal inválido (0 = sem timeout)
};

// Wire size: 12 bytes LE per signal (see can_rx_map_serialize_to_page0).
// Layout: id u16, byte_lo, byte_hi, shift, pad, mask u16, offset i16, timeout u16
static constexpr uint16_t kCanRxSignalWireLen = 12u;
// page0 216–245: 3 signals × 10 B
static constexpr uint16_t kCanRxMapPage0Off = 216u;
static constexpr uint16_t kCanRxMapPage0Len =
    static_cast<uint16_t>(static_cast<uint8_t>(CanRxSignal::COUNT) * kCanRxSignalWireLen);

// Configura/lê definição de sinal
void         can_rx_map_set(CanRxSignal sig, const CanSignalDef& def) noexcept;
CanSignalDef can_rx_map_get(CanRxSignal sig) noexcept;

// Processa frame recebido; chamado internamente pelo can_stack
void can_rx_map_process(uint16_t id, const uint8_t* data, uint8_t dlc,
                        uint32_t now_ms) noexcept;

// Leitura dos sinais decodificados
// Retorna false se sinal não configurado (id=0) ou em timeout
bool can_rx_gear(uint8_t&  out_gear, uint32_t now_ms) noexcept;
bool can_rx_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept;
bool can_rx_wheel_speed_kmh(uint16_t& out_kmh, uint32_t now_ms) noexcept;

// page0 helpers (layout after launch/TC @191–215). Safe no-ops on short buffers.
void can_rx_map_serialize_to_page0(uint8_t* page0, uint16_t len) noexcept;
void can_rx_map_apply_from_page0(const uint8_t* page0, uint16_t len) noexcept;

} // namespace ems::app
