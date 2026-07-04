#pragma once

#include <cstdint>

namespace ems::hal {

// CRC-32 ISO-HDLC (zlib/Ethernet): polinómio refletido 0xEDB88320,
// init 0xFFFFFFFF, XOR final. Usado pelo RuntimeSyncSeed (flash.cpp) e pelo
// envelope do protocolo TunerStudio (ui_protocol.cpp) — ambos exigem
// exatamente esta variante.
inline uint32_t crc32_update(uint32_t crc, uint8_t data) noexcept {
    crc ^= data;
    for (uint8_t i = 0u; i < 8u; ++i) {
        const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
        crc = (crc >> 1u) ^ (0xEDB88320u & mask);
    }
    return crc;
}

inline uint32_t crc32_calc(const uint8_t* data, uint32_t len) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < len; ++i) {
        crc = crc32_update(crc, data[i]);
    }
    return ~crc;
}

}  // namespace ems::hal
