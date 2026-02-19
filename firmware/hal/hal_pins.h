/**
 * @file hal_pins.h
 * @brief Hardware pin assignments for ESP32-S3
 *
 * ESP32-S3 GPIO layout:
 * - 45 total GPIOs
 * - GPIO 0-21: Regular GPIOs
 * - GPIO 26-48: Extended range (use sparingly)
 * - STRAPPING pins: 0, 45, 46 (hold at boot)
 *
 * IMPORTANT: This is a NEW pin map for ESP32-S3. Previous ESP32 map
 * used GPIOs 34-39 which don't exist on S3. All pins reassigned.
 *
 * Change this file to adapt to a different board layout.
 * All pin assignments live here and nowhere else.
 */

#ifndef HAL_PINS_H
#define HAL_PINS_H

#include "driver/gpio.h"

// ── Trigger inputs ───────────────────────────────────────────────────────────
#define HAL_PIN_CKP         GPIO_NUM_1    // Crankshaft position (PCNT input)
#define HAL_PIN_CMP         GPIO_NUM_2    // Camshaft position (GPIO ISR)

// ── Injectors (low-side, active HIGH) ────────────────────────────────────────
#define HAL_PIN_INJ_1       GPIO_NUM_12   // Injector 1 (MCPWM)
#define HAL_PIN_INJ_2       GPIO_NUM_13   // Injector 2 (MCPWM)
#define HAL_PIN_INJ_3       GPIO_NUM_15   // Injector 3 (MCPWM)
#define HAL_PIN_INJ_4       GPIO_NUM_16   // Injector 4 (MCPWM)

// ── Ignition (logic-level COP, active HIGH) ──────────────────────────────────
#define HAL_PIN_IGN_1       GPIO_NUM_17   // Ignition 1 (MCPWM)
#define HAL_PIN_IGN_2       GPIO_NUM_18   // Ignition 2 (MCPWM)
#define HAL_PIN_IGN_3       GPIO_NUM_19   // Ignition 3 (MCPWM)
#define HAL_PIN_IGN_4       GPIO_NUM_20   // Ignition 4 (MCPWM)

// ── CAN / TWAI ───────────────────────────────────────────────────────────────
#define HAL_PIN_CAN_TX      GPIO_NUM_10   // CAN TX
#define HAL_PIN_CAN_RX      GPIO_NUM_11   // CAN RX

// ── Analog inputs ────────────────────────────────────────────────────────────
// Note: ADC1 channels available on GPIO 3-8 (CH0-CH5) and other pins
#define HAL_PIN_MAP         GPIO_NUM_3    // MAP sensor (ADC1_CH2)
#define HAL_PIN_TPS         GPIO_NUM_4    // TPS (ADC1_CH3)
#define HAL_PIN_CLT         GPIO_NUM_5    // CLT NTC (ADC1_CH4)
#define HAL_PIN_IAT         GPIO_NUM_6    // IAT NTC (ADC1_CH5)
#define HAL_PIN_OIL_PRESS   GPIO_NUM_7    // Oil pressure (ADC1_CH6)
#define HAL_PIN_FUEL_PRESS  GPIO_NUM_8    // Fuel pressure (ADC1_CH7)
#define HAL_PIN_VBAT        GPIO_NUM_9    // Battery voltage divider (ADC1_CH8)

// ── Digital inputs ───────────────────────────────────────────────────────────
#define HAL_PIN_FLEX        GPIO_NUM_26   // Flex fuel sensor (frequency input)
#define HAL_PIN_VSS         GPIO_NUM_27   // Vehicle speed (pulse input)
#define HAL_PIN_CLUTCH      GPIO_NUM_28   // Clutch switch
#define HAL_PIN_BRAKE       GPIO_NUM_29   // Brake switch

// ── Knock sensors ────────────────────────────────────────────────────────────
// S1-05 WARNING: GPIO 30/31 map to ADC2_CH9/CH10 on ESP32-S3.
// ADC2 is shared with the Wi-Fi/ESP-NOW RF subsystem. Concurrent ADC2 reads
// while ESP-NOW is active will fail with ESP_ERR_TIMEOUT or return garbage.
// Mitigation options (choose one):
//   A) Disable ESP-NOW entirely if knock sensing is required at all times.
//   B) Gate ADC2 reads to windows where ESP-NOW is known idle (e.g., between TX slots).
//   C) Re-spin PCB to route knock inputs to ADC1-capable pins (GPIO 1-10).
// Currently option B is used: knock sampling is suspended during ESP-NOW TX.
// See espnow_link.c:espnow_pre_tx_hook() and sensor_processing.c:knock_adc_gate().
#define HAL_PIN_KNOCK_1     GPIO_NUM_30   // Knock sensor 1 (ADC2_CH9)  — see warning above
#define HAL_PIN_KNOCK_2     GPIO_NUM_31   // Knock sensor 2 (ADC2_CH10) — see warning above

// ── PWM outputs (auxiliary) ──────────────────────────────────────────────────
#define HAL_PIN_VVT_INTAKE  GPIO_NUM_38   // VVT intake solenoid PWM
#define HAL_PIN_VVT_EXHAUST GPIO_NUM_39   // VVT exhaust solenoid PWM
#define HAL_PIN_IAC         GPIO_NUM_40   // Idle air control PWM
#define HAL_PIN_BOOST       GPIO_NUM_41   // Boost solenoid PWM

// ── Digital outputs ──────────────────────────────────────────────────────────
#define HAL_PIN_CEL         GPIO_NUM_42   // Check Engine Light
// S1-01 fix: GPIO 43 and 44 are UART0 TX/RX on ESP32-S3 (used by HAL_UART_TX/RX).
// Remapped fuel pump and fan to unused GPIOs 35 and 34 to eliminate the conflict.
// Original: HAL_PIN_FUEL_PUMP=43, HAL_PIN_FAN=44 (both overlapped with UART).
#define HAL_PIN_FUEL_PUMP   GPIO_NUM_35   // Fuel pump relay  (was 43 — UART conflict)
#define HAL_PIN_FAN         GPIO_NUM_34   // Cooling fan relay (was 44 — UART conflict)
#define HAL_PIN_AUX_1       GPIO_NUM_48   // Auxiliary output 1
#define HAL_PIN_AUX_2       GPIO_NUM_47   // Auxiliary output 2

// ── SD card (SPI) ────────────────────────────────────────────────────────────
// Standard ESP32 VSPI (FSPI on S3):
#define HAL_PIN_SD_CS       GPIO_NUM_21   // Chip Select (GPIO 21)
#define HAL_PIN_SD_CLK      GPIO_NUM_14   // Clock (GPIO 14)
#define HAL_PIN_SD_MOSI     GPIO_NUM_37   // MOSI (GPIO 37)
#define HAL_PIN_SD_MISO     GPIO_NUM_36   // MISO (GPIO 36)

// ── UART (debug / TunerStudio) ───────────────────────────────────────────────
#define HAL_UART_TX         GPIO_NUM_43   // UART0 TX (USB-to-serial on dev board)
#define HAL_UART_RX         GPIO_NUM_44   // UART0 RX
#define HAL_UART_BAUD       115200        // Serial baud rate

// ── GPIO validation ─────────────────────────────────────────────────────────
// Verify all pins are within ESP32-S3 range (0-21, 26-48)
#if defined(__GNUC__)
  // Compile-time checks could go here if needed
#endif

#endif // HAL_PINS_H
