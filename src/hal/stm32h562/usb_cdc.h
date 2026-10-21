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
 * Uso: transporte USB da UI proprietaria (pos-MVP)
 */

#include <cstdint>

namespace ems::hal {

// Initialization
void usb_cdc_init() noexcept;
void usb_cdc_poll() noexcept;

// Data transmission
void usb_cdc_send_byte(uint8_t byte) noexcept;
void usb_cdc_send_bytes(const uint8_t* data, uint16_t len) noexcept;
// Espaço livre no ring de TX interno — usar para orçamentar envios e nunca
// perder bytes silenciosamente (usb_cdc_send_bytes descarta o que não couber).
uint16_t usb_cdc_tx_free() noexcept;
bool usb_cdc_configured() noexcept;
// Sem enumeração o orçamento é só o do UART e o caller não espelha no ring USB.
// Enumerado: o menor dos dois, para um frame não partir. Teto 32 bytes por volta.
struct UsbTxBudget {
    uint16_t n;
    bool mirror_usb;
};
inline UsbTxBudget usb_cdc_tx_budget(uint16_t uart_free, uint16_t usb_free,
                                    bool configured) noexcept {
    UsbTxBudget b{};
    b.n = uart_free;
    b.mirror_usb = configured;
    if (configured && usb_free < b.n) { b.n = usb_free; }
    if (b.n > 32u) { b.n = 32u; }
    return b;
}

// Data reception
bool usb_cdc_available() noexcept;
uint8_t usb_cdc_read_byte() noexcept;
uint16_t usb_cdc_read_bytes(uint8_t* buffer, uint16_t max_len) noexcept;

// Control signals
bool usb_cdc_dtr() noexcept;  // Data Terminal Ready
bool usb_cdc_rts() noexcept;  // Request To Send

// Diagnóstico: maior estágio de enumeração já alcançado (1..6).
// 1=vivo  2=ISR USB disparou  3=RESET do host  4=SETUP recebido
// 5=GET_DESCRIPTOR pedido  6=SET_CONFIGURATION (enumerado)
uint8_t usb_cdc_dbg_stage() noexcept;

} // namespace ems::hal
