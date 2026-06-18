#pragma once
#include <cstdint>

namespace ems::hal {

void tle8888_init() noexcept;
void tle8888_poll_diag() noexcept;
bool tle8888_ok() noexcept;
uint16_t tle8888_fault_count() noexcept;
// ch 0-3 = INJ, 4-7 = IGN. Returns: 0=OK, 1=open-load, 2=short-GND, 3=short-VBAT
uint8_t tle8888_channel_fault(uint8_t ch) noexcept;
// Bitmap: bit N set = channel N has a fault
uint8_t tle8888_fault_bitmap() noexcept;

}  // namespace ems::hal
