#pragma once
/**
 * @file hal/stm32h562/usb_cdc.h
 * @brief USB CDC Device Driver para STM32H562
 *
 * Exporta:
 *   usb_cdc_init()        — inicializa USB FS como CDC-ACM device
 *   usb_cdc_poll()        — processa eventos USB (non-blocking)
 *   usb_cdc_send_byte()   — transmite byte único
 *   usb_cdc_send_bytes()  — transmite múltiplos bytes
 *   usb_cdc_available()   — verifica se dados RX estão disponíveis
 *   usb_cdc_read_byte()   — lê byte único
 *   usb_cdc_read_bytes()  — lê múltiplos bytes
 *   usb_cdc_dtr()         — verifica Data Terminal Ready
 *   usb_cdc_rts()         — verifica Request To Send
 *
 * Uso: TunerStudio protocol via USB (Phase 3 feature)
 */

#include <cstdint>

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
