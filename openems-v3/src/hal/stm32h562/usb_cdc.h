#pragma once

#include <cstdint>

// USB CDC Device Driver for STM32H562
// Implements CDC-ACM (Abstract Control Model)
// Purpose: TunerStudio communication via USB (Phase 3 feature)

namespace ems::hal {

// Initialization
void usb_cdc_init() noexcept;
void usb_cdc_poll() noexcept;

// Data transmission
void usb_cdc_send_byte(uint8_t byte) noexcept;
void usb_cdc_send_bytes(const uint8_t* data, uint16_t len) noexcept;

// Data reception
bool usb_cdc_available() noexcept;
uint8_t usb_cdc_read_byte() noexcept;
uint16_t usb_cdc_read_bytes(uint8_t* buffer, uint16_t max_len) noexcept;

// Control signals
bool usb_cdc_dtr() noexcept;  // Data Terminal Ready
bool usb_cdc_rts() noexcept;  // Request To Send

} // namespace ems::hal
