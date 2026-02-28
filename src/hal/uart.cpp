#include "hal/uart.h"

#if defined(EMS_HOST_TEST)
namespace ems::hal {
void uart0_init(uint32_t) noexcept {}
void uart0_poll_rx(uint16_t) noexcept {}
bool uart0_tx_ready() noexcept { return true; }
bool uart0_tx_byte(uint8_t) noexcept { return true; }
}  // namespace ems::hal
#else

#include <Arduino.h>

#include <cstdint>

#include "app/tuner_studio.h"

namespace ems::hal {

void uart0_init(uint32_t baud) noexcept {
    if (baud == 0u) {
        baud = 115200u;
    }

    // Non-blocking USB CDC init: do not wait for host enumeration/DTR.
    Serial.begin(baud);
}

void uart0_poll_rx(uint16_t max_bytes) noexcept {
    uint16_t budget = max_bytes;
    while (budget > 0u && Serial.available() > 0) {
        const int byte = Serial.read();
        if (byte < 0) {
            break;
        }
        ems::app::ts_uart0_rx_isr_byte(static_cast<uint8_t>(byte));
        --budget;
    }
}

bool uart0_tx_ready() noexcept {
    return true;
}

bool uart0_tx_byte(uint8_t byte) noexcept {
    if (!uart0_tx_ready()) {
        return false;
    }
    const size_t written = Serial.write(&byte, 1u);
    return written == 1u;
}

}  // namespace ems::hal

#endif
