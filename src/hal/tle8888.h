#pragma once
#include <cstdint>

namespace ems::hal {

void tle8888_init() noexcept;
void tle8888_poll_diag() noexcept;
bool tle8888_ok() noexcept;
uint16_t tle8888_fault_count() noexcept;

}  // namespace ems::hal
