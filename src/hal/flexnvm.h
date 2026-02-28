#pragma once

#include <cstdint>

namespace ems::hal {

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept;
int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept;

bool nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept;
bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept;

#if defined(EMS_HOST_TEST)
void nvm_test_reset() noexcept;
void nvm_test_set_ccif_busy_polls(uint32_t polls) noexcept;
uint32_t nvm_test_erase_count() noexcept;
uint32_t nvm_test_program_count() noexcept;
#endif

}  // namespace ems::hal
