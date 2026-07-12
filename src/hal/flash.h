#pragma once

#include <cstdint>

namespace ems::hal {

// ── Dimensões NVM dos mapas adaptativos ──────────────────────────────────────
// HAL não inclui headers do engine: estas constantes DEVEM espelhar
// engine::kTableAxisSize / kLtftAddAxisSize — static_assert em fuel_calc.cpp
// (que vê ambos os headers) garante o lockstep.
constexpr uint8_t kNvmLtftDim    = 20u;
constexpr uint8_t kNvmLtftAddDim = (kNvmLtftDim + 1u) / 2u;

// ── Layout do Setor 0 (LTFT-mult + knock + LTFT-add + magic + seed) ─────────
// Offsets derivados das dimensões; magic e seed alinhados a 16 bytes
// (quad-word de flash). Mudar kNvmLtftDim muda o layout — o magic invalida
// setores com layout antigo no boot (mapas zerados e regravados no flush).
constexpr uint32_t kNvmOffLtft        = 0u;
constexpr uint32_t kNvmOffKnock       = static_cast<uint32_t>(kNvmLtftDim) * kNvmLtftDim;
constexpr uint32_t kNvmOffLtftAdd     = kNvmOffKnock + 64u;  // knock fixo 8×8
constexpr uint32_t kNvmOffLayoutMagic =
    (kNvmOffLtftAdd + static_cast<uint32_t>(kNvmLtftAddDim) * kNvmLtftAddDim + 15u) & ~15u;
constexpr uint32_t kNvmLayoutMagic    = 0x4C544632u;  // "LTF2"
constexpr uint32_t kNvmSeedOffset     = kNvmOffLayoutMagic + 16u;

// Valida o layout do setor adaptativo (magic na posição do layout atual).
// Pura (recebe a imagem do setor) para ser testável em host.
bool nvm_adaptive_sector_valid(const uint8_t* sector) noexcept;

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept;
int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept;

// LTFT aditivo: unidade 50 µs/count, range ±6350 µs.
// Índices 0..kNvmLtftAddDim-1 (mapeados do grid principal via >>1).
bool nvm_write_ltft_add(uint8_t rpm_i, uint8_t load_i, int8_t val_50us) noexcept;
int8_t nvm_read_ltft_add(uint8_t rpm_i, uint8_t load_i) noexcept;

bool nvm_load_adaptive_maps() noexcept;
bool nvm_flush_adaptive_maps() noexcept;

// knock_map[8×8]: retraso de ignição por cilindro (unidade: 0.1°, range –12.7°..+12.7°)
// Mapeado em SRAM (EEPROM emulada) logo após o LTFT, offset 256 bytes.
bool nvm_write_knock(uint8_t rpm_i, uint8_t load_i, int8_t retard_deci_deg) noexcept;
int8_t nvm_read_knock(uint8_t rpm_i, uint8_t load_i) noexcept;
void nvm_reset_knock_map() noexcept;  // zera todo o mapa (e.g. ao ligar)

bool nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept;
bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept;

#if defined(EMS_HOST_TEST)
void nvm_test_reset() noexcept;
void flash_test_set_busy_polls(uint32_t polls) noexcept;
uint32_t nvm_test_erase_count() noexcept;
uint32_t nvm_test_program_count() noexcept;
#endif

}  // namespace ems::hal
