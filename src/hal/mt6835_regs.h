#pragma once
/**
 * @file mt6835_regs.h
 * @brief Mapa de registradores e protocolo SPI do MagnTek MT6835.
 *
 * Fonte: MT6835_Rev.1.3.pdf (MagnTek, 2022.12, magntek.com.cn — datasheet
 * oficial do fabricante, não uma análise de terceiros). Capítulos 7.6 (SPI
 * Interface) e 10 (Register Map). Verificado byte a byte contra o texto
 * extraído do PDF nesta sessão — ver docs/dev/mt6835_encoder_fork.md, gate 1.
 *
 * PROTOCOLO SPI (§7.6.3, Figura 18)
 * ──────────────────────────────────
 * Modo 3 (CPOL=1, CPHA=1). Frame de 24 bits em 3 bytes:
 *   byte0 = C3 C2 C1 C0 A11 A10 A9 A8   (comando + 4 bits altos do endereço)
 *   byte1 = A7 A6 A5 A4 A3 A2 A1 A0     (8 bits baixos do endereço)
 *   byte2 = D7..D0 (escrita) ou DO7..DO0 (leitura, MISO durante este byte)
 *
 * CSN começa LOW antes do byte0 e só sobe depois do byte2 — os 3 bytes têm de
 * ser transmitidos na MESMA sessão CS (não é possível fazer 3 transações SPI
 * separadas com CS a subir entre elas).
 */

#include <cstdint>

namespace ems::hal::mt6835 {

// Comando de operação (C3~C0), §7.6.3
enum class Cmd : uint8_t {
    kRead          = 0x3u,  // 0011 — leitura de registrador único
    kWrite         = 0x6u,  // 0110 — escrita de registrador único
    kProgramEeprom = 0xCu,  // 1100 — grava o register map inteiro na EEPROM
    kAutoZero      = 0x5u,  // 0101 — define a posição atual como zero
    kBurstAngle    = 0xAu,  // 1010 — leitura contínua dos registradores de ângulo
};

// Endereços de registrador (12 bits), §10 Register Map
constexpr uint16_t kRegUserId       = 0x001u;  // EEPROM, livre para o utilizador
constexpr uint16_t kRegAngle2013    = 0x003u;  // Read-only: ANGLE[20:13]
constexpr uint16_t kRegAngle125     = 0x004u;  // Read-only: ANGLE[12:5]
constexpr uint16_t kRegAngle40Stat  = 0x005u;  // Read-only: ANGLE[4:0] | STATUS[2:0]
constexpr uint16_t kRegCrc          = 0x006u;  // Read-only: CRC[7:0]
constexpr uint16_t kRegAbzResHi     = 0x007u;  // EEPROM: ABZ_RES[13:6]
constexpr uint16_t kRegAbzResLo     = 0x008u;  // EEPROM: ABZ_RES[5:0]|ABZ_OFF|AB_SWAP
constexpr uint16_t kRegZeroPosHi    = 0x009u;  // EEPROM: ZERO_POS[11:4]
constexpr uint16_t kRegZeroPosLo    = 0x00Au;  // EEPROM: ZERO_POS[3:0]|Z_EDGE|Z_PUL_WID

// STATUS[2:0] — bits 2:0 do byte lido em kRegAngle40Stat (§10.2)
constexpr uint8_t kStatusOverspeedBit  = 0u;  // 1 = rotação acima do limite garantido
constexpr uint8_t kStatusWeakFieldBit  = 1u;  // 1 = campo magnético fraco (ver Bpk, gate 1)
constexpr uint8_t kStatusUndervoltBit  = 2u;  // 1 = subtensão

// CRC-8 sobre ANGLE[20:0]+STATUS[2:0] (24 bits = os 3 bytes 0x003,0x004,0x005),
// MSB primeiro (ANGLE[20] entra primeiro). Polinómio X8+X2+X+1 = 0x07.
// ⚠️ Valor inicial NÃO afirmado explicitamente pelo datasheet — assumido 0x00
// (convenção usual para este polinómio, ex. CRC-8/SMBus). Confirmar contra uma
// leitura real do sensor antes de tratar mismatches de CRC como falha de sensor
// em vez de falha de suposição de init.
constexpr uint8_t kCrc8Poly = 0x07u;
constexpr uint8_t kCrc8Init = 0x00u;

// ABZ_RES[13:0]: valor do registrador = PPR - 1 (0x0000=1 PPR .. 0x3FFF=16384 PPR).
// Decisão travada: PPR=4096 → registrador = 4095 = 0x0FFF.
constexpr uint16_t kAbzResPpr4096 = 4096u - 1u;

}  // namespace ems::hal::mt6835
