#pragma once
#include <cstdint>

namespace ems::hal {

// Pedido de A/C (compressor ligado) em PB11: entrada, pull-down, ativo alto.
void ac_request_init() noexcept;
// Amostra o pino com debounce de 50 ms. Devolve o estado filtrado.
bool ac_request_poll(uint32_t now_ms) noexcept;

}  // namespace ems::hal
