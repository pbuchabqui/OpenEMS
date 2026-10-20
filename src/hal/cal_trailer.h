#pragma once
/**
 * @file hal/cal_trailer.h
 * Rodapé de integridade das páginas de calibração (um quad-word antes + um depois dos dados).
 *
 * Cada página ocupa um setor de flash e é gravada na ordem: erase → quad-word BEGIN → dados →
 * quad-word END (com CRC32 dos dados). Uma queda de energia entre o erase e o fim deixa BEGIN
 * sem END (ou CRC errado) e a página é recusada, em vez de ser carregada pela metade — o
 * carregador só testava "tudo 0xFF" e confiava em qualquer outro conteúdo.
 *
 * Páginas gravadas antes deste rodapé (BEGIN e END apagados) continuam sendo aceitas.
 * Lógica pura (sem acesso a flash) para ser testada no host.
 */
#include <cstdint>
#include <cstring>

#include "hal/crc32.h"

namespace ems::hal {

inline constexpr uint32_t kCalTrailerOffset = 8160u;   // últimos 32 bytes do setor de 8 KB
inline constexpr uint32_t kCalTrailerBytes  = 32u;
inline constexpr uint32_t kCalMagicBegin    = 0x424C4143u;  // 'CALB'
inline constexpr uint32_t kCalMagicEnd      = 0x45414C43u;  // 'CALE'

enum class CalTrailerState : uint8_t {
    LEGACY = 0,      // sem rodapé (página antiga ou nunca gravada): aceita como antes
    VALID = 1,       // BEGIN + END + CRC conferem
    INCOMPLETE = 2,  // BEGIN sem END: gravação interrompida
    CORRUPT = 3,     // rodapé presente mas inconsistente / CRC não bate
};

struct CalTrailerWords { uint32_t w[4]; };

inline CalTrailerWords cal_trailer_begin(uint32_t len) noexcept {
    return CalTrailerWords{{kCalMagicBegin, len, ~len, 0xFFFFFFFFu}};
}

inline CalTrailerWords cal_trailer_end(const uint8_t* data, uint32_t len) noexcept {
    return CalTrailerWords{{kCalMagicEnd, crc32_calc(data, len), len, 0xFFFFFFFFu}};
}

inline bool cal_words_erased(const uint8_t* p) noexcept {
    for (uint8_t i = 0u; i < 16u; ++i) { if (p[i] != 0xFFu) { return false; } }
    return true;
}

// `sector` aponta para o início do setor da página (kCalTrailerOffset+32 bytes legíveis).
inline CalTrailerState cal_trailer_check(const uint8_t* sector) noexcept {
    const uint8_t* b = sector + kCalTrailerOffset;
    const uint8_t* e = b + 16u;
    const bool b_erased = cal_words_erased(b);
    const bool e_erased = cal_words_erased(e);
    if (b_erased && e_erased) { return CalTrailerState::LEGACY; }
    uint32_t bw[4], ew[4];
    std::memcpy(bw, b, 16u);
    std::memcpy(ew, e, 16u);
    if (bw[0] != kCalMagicBegin || bw[2] != ~bw[1] || bw[1] > kCalTrailerOffset) {
        return CalTrailerState::CORRUPT;
    }
    if (e_erased) { return CalTrailerState::INCOMPLETE; }
    if (ew[0] != kCalMagicEnd || ew[2] != bw[1]) { return CalTrailerState::CORRUPT; }
    return (crc32_calc(sector, bw[1]) == ew[1]) ? CalTrailerState::VALID : CalTrailerState::CORRUPT;
}

}  // namespace ems::hal
