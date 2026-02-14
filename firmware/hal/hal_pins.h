/**
 * @file hal_pins.h
 * @brief Hardware pin assignments — ESP32 / ESP32-S3
 *
 * Change this file to adapt to a different board layout.
 * All pin assignments live here and nowhere else.
 */

#ifndef HAL_PINS_H
#define HAL_PINS_H

#include "driver/gpio.h"

// ── Trigger inputs ───────────────────────────────────────────────────────────
#define HAL_PIN_CKP         GPIO_NUM_34   // Crankshaft position (PCNT input)
#define HAL_PIN_CMP         GPIO_NUM_35   // Camshaft position (GPIO ISR)

// ── Injectors (low-side, active HIGH) ────────────────────────────────────────
#define HAL_PIN_INJ_1       GPIO_NUM_12
#define HAL_PIN_INJ_2       GPIO_NUM_13
#define HAL_PIN_INJ_3       GPIO_NUM_15
#define HAL_PIN_INJ_4       GPIO_NUM_2

// ── Ignition (logic-level COP, active HIGH) ──────────────────────────────────
#define HAL_PIN_IGN_1       GPIO_NUM_16
#define HAL_PIN_IGN_2       GPIO_NUM_17
#define HAL_PIN_IGN_3       GPIO_NUM_18
#define HAL_PIN_IGN_4       GPIO_NUM_21

// ── CAN / TWAI ───────────────────────────────────────────────────────────────
#define HAL_PIN_CAN_TX      GPIO_NUM_4
#define HAL_PIN_CAN_RX      GPIO_NUM_5

// ── Analog inputs ────────────────────────────────────────────────────────────
#define HAL_PIN_MAP         GPIO_NUM_36   // MAP sensor (ADC1_CH0)
#define HAL_PIN_TPS         GPIO_NUM_39   // TPS (ADC1_CH3)
#define HAL_PIN_CLT         GPIO_NUM_32   // CLT NTC (ADC1_CH4)
#define HAL_PIN_IAT         GPIO_NUM_33   // IAT NTC (ADC1_CH5)
#define HAL_PIN_OIL_PRESS   GPIO_NUM_25   // Oil pressure (ADC2_CH8)
#define HAL_PIN_FUEL_PRESS  GPIO_NUM_26   // Fuel pressure (ADC2_CH9)
#define HAL_PIN_VBAT        GPIO_NUM_34   // Battery voltage divider

// ── Digital inputs ───────────────────────────────────────────────────────────
#define HAL_PIN_FLEX        GPIO_NUM_27   // Flex fuel sensor (frequency input)
#define HAL_PIN_VSS         GPIO_NUM_14   // Vehicle speed (pulse input)
#define HAL_PIN_CLUTCH      GPIO_NUM_22   // Clutch switch
#define HAL_PIN_BRAKE       GPIO_NUM_23   // Brake switch

// ── Knock sensors ────────────────────────────────────────────────────────────
#define HAL_PIN_KNOCK_1     GPIO_NUM_37   // Knock sensor 1 (ADC1_CH1)
#define HAL_PIN_KNOCK_2     GPIO_NUM_38   // Knock sensor 2 (ADC1_CH2) — optional

// ── PWM outputs (auxiliary) ──────────────────────────────────────────────────
#define HAL_PIN_VVT_INTAKE  GPIO_NUM_19   // VVT intake solenoid PWM
#define HAL_PIN_VVT_EXHAUST GPIO_NUM_20   // VVT exhaust solenoid PWM
#define HAL_PIN_IAC         GPIO_NUM_6    // Idle air control PWM
#define HAL_PIN_BOOST       GPIO_NUM_7    // Boost solenoid PWM

// ── Digital outputs ──────────────────────────────────────────────────────────
#define HAL_PIN_CEL         GPIO_NUM_8    // Check Engine Light
#define HAL_PIN_FUEL_PUMP   GPIO_NUM_9    // Fuel pump relay
#define HAL_PIN_FAN         GPIO_NUM_10   // Cooling fan relay
#define HAL_PIN_AUX_1       GPIO_NUM_11   // Auxiliary output 1
#define HAL_PIN_AUX_2       GPIO_NUM_3    // Auxiliary output 2

// ── SD card (SPI) ────────────────────────────────────────────────────────────
#define HAL_PIN_SD_MOSI     GPIO_NUM_23
#define HAL_PIN_SD_MISO     GPIO_NUM_19
#define HAL_PIN_SD_CLK      GPIO_NUM_18
#define HAL_PIN_SD_CS       GPIO_NUM_5

// ── UART (debug / TunerStudio) ───────────────────────────────────────────────
#define HAL_UART_BAUD       115200

#endif // HAL_PINS_H
