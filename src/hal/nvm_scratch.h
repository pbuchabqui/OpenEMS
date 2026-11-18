#pragma once
// Setor de scratch da Flash para a operação sacrificial do boot (errata ES0565 §2.2.33).
// Lógica pura (sem acesso a hardware) para poder ser testada no host.
#include <cstdint>

namespace ems::hal {

// Tamanho de um slot = 1 flash word (128 bits), a unidade mínima de program na Flash do H5.
constexpr uint32_t kNvmScratchSlotBytes = 16u;

// Próximo slot livre do setor: o seguinte ao ÚLTIMO slot que não está todo em 0xFF.
// Devolve -1 se o setor está cheio (o chamador apaga o setor, pelo caminho normal).
// Slots são gravados em ordem; um slot interrompido no meio (não-0xFF) é pulado, nunca reusado.
inline int32_t nvm_scratch_next_slot(const uint8_t* sector, uint32_t sector_bytes) noexcept {
    const uint32_t slots = sector_bytes / kNvmScratchSlotBytes;
    int32_t last = -1;
    for (uint32_t s = 0u; s < slots; ++s) {
        const uint8_t* p = sector + s * kNvmScratchSlotBytes;
        for (uint32_t i = 0u; i < kNvmScratchSlotBytes; ++i) {
            if (p[i] != 0xFFu) { last = static_cast<int32_t>(s); break; }
        }
    }
    const uint32_t next = static_cast<uint32_t>(last + 1);
    return next < slots ? static_cast<int32_t>(next) : -1;
}

}  // namespace ems::hal
