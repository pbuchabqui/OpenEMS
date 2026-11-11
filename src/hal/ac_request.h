#pragma once
#include <cstdint>

namespace ems::hal {

// Pedido de A/C (compressor ligado) em PB11: entrada, pull-down, ativo alto.
void ac_request_init() noexcept;
// Amostra o pino com debounce de 50 ms. Devolve o estado filtrado.
bool ac_request_poll(uint32_t now_ms) noexcept;


// Debounce puro (sem acesso a pino): o .cpp do target amostra o PB11 e chama step();
// os testes de host exercitam esta mesma lógica.
struct AcDebounce {
    static constexpr uint32_t kDebounceMs = 50u;
    bool     state = false;          // estado filtrado
    bool     raw = false;            // última amostra crua
    uint32_t raw_since_ms = 0u;

    bool step(bool sample, uint32_t now_ms) noexcept {
        if (sample != raw) {
            raw = sample;
            raw_since_ms = now_ms;
        } else if (sample != state && (now_ms - raw_since_ms) >= kDebounceMs) {
            state = sample;
        }
        return state;
    }
};

}  // namespace ems::hal
