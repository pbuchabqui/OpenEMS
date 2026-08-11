 * DEFERRED: not linked in the firmware Makefile until SD hardware exists.
 * Keep sources in-tree; do not call from main_stm32.cpp.
#pragma once

#include <cstdint>

namespace ems::hal {

bool     sdmmc_init() noexcept;
bool     sdmmc_write_block(uint32_t lba, const uint8_t* data) noexcept;
bool     sdmmc_card_present() noexcept;
uint32_t sdmmc_error_count() noexcept;

}  // namespace ems::hal
