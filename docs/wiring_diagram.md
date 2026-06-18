# OpenEMS — Electrical Wiring Diagram

## STM32H562RGT6 (LQFP64) + TLE8888 Smart Power Stage

```
                            ┌─────────────────────────────────┐
                            │        STM32H562RGT6            │
                            │       250 MHz Cortex-M33        │
                            │                                 │
  8 MHz HSE ───────────────►│ PH0/PH1 (OSC_IN/OUT)           │
                            │                                 │
  ┌─ CKP (60-2) ──────────►│ PA0  TIM5_CH1  (AF2) ──────────┤── Input Capture (16ns/tick)
  │  CMP (cam) ───────────►│ PA1  TIM5_CH2  (AF2) ──────────┤── Phase detection
  │                         │                                 │
  │  ┌── MAP sensor ──────►│ PA3  ADC1_IN15 ─────────────────┤
  │  │   TPS sensor ──────►│ PA4  ADC1_IN18 ─────────────────┤── ADC1 (TIM6 trigger)
  │  │   Knock / O2 ──────►│ PA5  ADC1_IN6  ─────────────────┤
  │  │   CLT sensor ──────►│ PB0  ADC2      ─────────────────┤
  │  │   IAT sensor ──────►│ PB1  ADC2      ─────────────────┤── ADC2
  │  │   APP1 pedal ──────►│ PC0  ADC2      ─────────────────┤
  │  │   APP2 pedal ──────►│ PC1  ADC2      ─────────────────┤
  │  │                      │                                 │
  │  │                      │         INJECTION (TIM2 OC)     │
  │  │                      │ PC6  TIM2_CH1  (AF3) ──────────┼──► TLE8888 IN0 (INJ1)
  │  │                      │ PC7  TIM2_CH2  (AF3) ──────────┼──► TLE8888 IN1 (INJ2)
  │  │                      │ PB10 TIM2_CH3  (AF1) ──────────┼──► TLE8888 IN2 (INJ3)
  │  │                      │ PB11 TIM2_CH4  (AF1) ──────────┼──► TLE8888 IN3 (INJ4)
  │  │                      │                                 │
  │  │                      │         IGNITION (TIM8 OC)      │
  │  │                      │ PC8  TIM8_CH1  (AF3) ──────────┼──► TLE8888 IGN0 (COIL1)
  │  │                      │ PC9  TIM8_CH2  (AF3) ──────────┼──► TLE8888 IGN1 (COIL2)
  │  │                      │ PA15 TIM8_CH3  (AF1) ──────────┼──► TLE8888 IGN2 (COIL3)
  │  │                      │ PB3  TIM8_CH4  (AF1) ──────────┼──► TLE8888 IGN3 (COIL4)
  │  │                      │                                 │
  │  │                      │         SPI2 → TLE8888          │
  │  │                      │ PB12 GPIO OUT  (CS)  ──────────┼──► TLE8888 CSN
  │  │                      │ PB13 SPI2_SCK  (AF5) ──────────┼──► TLE8888 SCLK
  │  │                      │ PB14 SPI2_MISO (AF5) ◄─────────┼─── TLE8888 SDO
  │  │                      │ PB15 SPI2_MOSI (AF5) ──────────┼──► TLE8888 SDI
  │  │                      │                                 │
  │  │                      │         ETB (TIM1 PWM)          │
  │  │                      │ PA8  TIM1_CH1  (AF1) ──────────┼──► H-Bridge PWM
  │  │                      │ PA10 GPIO OUT  (IN1)  ──────────┼──► H-Bridge DIR1
  │  │                      │ PB2  GPIO OUT  (IN2)  ──────────┼──► H-Bridge DIR2
  │  │                      │ ETB TPS1 ◄──── AN3 (ADC) ──────┤
  │  │                      │ ETB TPS2 ◄──── AN4 (ADC) ──────┤
  │  │                      │                                 │
  │  │                      │         AUXILIARIES (TIM3/4)    │
  │  │                      │ PA6  TIM3_CH1  (AF2) ──────────┼──► IACV PWM
  │  │                      │ PA7  TIM3_CH2  (AF2) ──────────┼──► Wastegate PWM
  │  │                      │ PB6  TIM4_CH1  (AF2) ──────────┼──► VVT Escape PWM
  │  │                      │ PB7  TIM4_CH2  (AF2) ──────────┼──► VVT Intake PWM
  │  │                      │                                 │
  │  │                      │         CAN BUS (FDCAN1)        │
  │  │                      │ PB8  FDCAN1_RX (AF9) ◄─────────┼─── CAN transceiver RX
  │  │                      │ PB9  FDCAN1_TX (AF9) ──────────┼──► CAN transceiver TX
  │  │                      │                                 │
  │  │                      │         COMMS                   │
  │  │                      │ PA9  USART1_TX (AF7) ──────────┼──► USB-UART adapter
  │  │                      │ PA10 USART1_RX (AF7) ◄─────────┼───   (bench debug)
  │  │                      │ PA11 USB_DM    (AF10) ─────────┼──► USB CDC (tuning)
  │  │                      │ PA12 USB_DP    (AF10) ─────────┼──►
  │  │                      │                                 │
  │  │                      │ PB2  GPIO OUT ─────────────────┼──► LED heartbeat
  │  │                      └─────────────────────────────────┘
  │  │
  │  │
  │  │  ┌─────────────────────────────────────────────────────────┐
  │  │  │                    TLE8888                              │
  │  │  │              Smart Power Stage                          │
  │  │  │                                                         │
  │  │  │  SPI Interface (config/diag only, 3.9 MHz)              │
  │  │  │    CSN  ◄──── PB12                                      │
  │  │  │    SCLK ◄──── PB13                                      │
  │  │  │    SDI  ◄──── PB15                                      │
  │  │  │    SDO  ────► PB14                                      │
  │  │  │                                                         │
  │  │  │  INJ Channels (Low-Side Switch, OC 10A, fast slew)      │
  │  │  │    IN0 ◄── PC6  ────► OUT0 ─────────────► Injector 1   │
  │  │  │    IN1 ◄── PC7  ────► OUT1 ─────────────► Injector 2   │
  │  │  │    IN2 ◄── PB10 ────► OUT2 ─────────────► Injector 3   │
  │  │  │    IN3 ◄── PB11 ────► OUT3 ─────────────► Injector 4   │
  │  │  │                                                         │
  │  │  │  IGN Channels (Push-Pull, OC 6A, fast slew)             │
  │  │  │    IGN0 ◄── PC8  ────► COIL0 ───────────► Coil 1       │
  │  │  │    IGN1 ◄── PC9  ────► COIL1 ───────────► Coil 2       │
  │  │  │    IGN2 ◄── PA15 ────► COIL2 ───────────► Coil 3       │
  │  │  │    IGN3 ◄── PB3  ────► COIL3 ───────────► Coil 4       │
  │  │  │                                                         │
  │  │  │  Protection (hardware, independent of MCU):             │
  │  │  │    • Overcurrent shutdown per channel                   │
  │  │  │    • Thermal shutdown                                   │
  │  │  │    • Open-load detection                                │
  │  │  │    • Short-to-GND / Short-to-VBAT detection             │
  │  │  │    • SPI watchdog (100ms refresh from MCU)              │
  │  │  │                                                         │
  │  │  │  VBAT ◄────────────────────────────── +12V battery      │
  │  │  │  GND  ◄────────────────────────────── chassis ground    │
  │  │  └─────────────────────────────────────────────────────────┘
  │  │
  │  │  ┌─────────────────────────────────┐
  │  └──│  Sensors                        │
  │     │  MAP: PA3 (0.5-4.5V, 0-3 bar)  │
  │     │  TPS: PA4 (0.5-4.5V)           │
  │     │  CLT: PB0 (NTC thermistor)     │
  │     │  IAT: PB1 (NTC thermistor)     │
  │     │  APP1: PC0 (pedal pos 1)       │
  │     │  APP2: PC1 (pedal pos 2)       │
  │     │  Knock: PA5 (piezo)            │
  │     └─────────────────────────────────┘
  │
  │     ┌─────────────────────────────────┐
  └─────│  Crankshaft / Camshaft          │
        │  CKP: PA0 (60-2 reluctor)      │
        │  CMP: PA1 (cam sensor)         │
        │  TIM5 @ 62.5 MHz (16ns/tick)   │
        └─────────────────────────────────┘

        ┌─────────────────────────────────┐
        │  CAN Bus (500 kbps)             │
        │  PB8/PB9 → transceiver → bus   │
        │  WBO2 lambda: ID 0x180 (cfg)   │
        │  Tx: 0x400 (10ms), 0x401       │
        │       (100ms), 0x402 (500ms)   │
        └─────────────────────────────────┘
```

## Pin Allocation Summary (LQFP64)

| Pin  | Function         | Peripheral | AF  | Notes                    |
|------|------------------|------------|-----|--------------------------|
| PA0  | CKP input        | TIM5_CH1   | AF2 | 60-2 tooth wheel         |
| PA1  | CMP input        | TIM5_CH2   | AF2 | Cam phase sensor         |
| PA3  | MAP sensor       | ADC1_IN15  | -   | 0-3 bar                  |
| PA4  | TPS sensor       | ADC1_IN18  | -   |                          |
| PA5  | Knock sensor     | ADC1_IN6   | -   |                          |
| PA6  | IACV PWM         | TIM3_CH1   | AF2 |                          |
| PA7  | Wastegate PWM    | TIM3_CH2   | AF2 |                          |
| PA8  | ETB PWM          | TIM1_CH1   | AF1 |                          |
| PA9  | USART1 TX        | USART1     | AF7 | Debug/bench              |
| PA10 | ETB DIR / RX     | GPIO/UART  | AF7 |                          |
| PA11 | USB DM           | USB        | AF10| CDC tuning               |
| PA12 | USB DP           | USB        | AF10|                          |
| PA15 | IGN3 output      | TIM8_CH3   | AF1 | → TLE8888 IGN2           |
| PB0  | CLT sensor       | ADC2       | -   | NTC                      |
| PB1  | IAT sensor       | ADC2       | -   | NTC                      |
| PB2  | LED / ETB DIR2   | GPIO       | -   |                          |
| PB3  | IGN4 output      | TIM8_CH4   | AF1 | → TLE8888 IGN3           |
| PB6  | VVT escape PWM   | TIM4_CH1   | AF2 |                          |
| PB7  | VVT intake PWM   | TIM4_CH2   | AF2 |                          |
| PB8  | CAN RX           | FDCAN1     | AF9 |                          |
| PB9  | CAN TX           | FDCAN1     | AF9 |                          |
| PB10 | INJ3 output      | TIM2_CH3   | AF1 | → TLE8888 IN2            |
| PB11 | INJ4 output      | TIM2_CH4   | AF1 | → TLE8888 IN3            |
| PB12 | TLE8888 CS       | GPIO       | -   | SPI2 software CS         |
| PB13 | TLE8888 SCK      | SPI2_SCK   | AF5 | 3.9 MHz                  |
| PB14 | TLE8888 MISO     | SPI2_MISO  | AF5 |                          |
| PB15 | TLE8888 MOSI     | SPI2_MOSI  | AF5 |                          |
| PC0  | APP1 pedal       | ADC2       | -   |                          |
| PC1  | APP2 pedal       | ADC2       | -   |                          |
| PC6  | INJ1 output      | TIM2_CH1   | AF3 | → TLE8888 IN0            |
| PC7  | INJ2 output      | TIM2_CH2   | AF3 | → TLE8888 IN1            |
| PC8  | IGN1 output      | TIM8_CH1   | AF3 | → TLE8888 IGN0           |
| PC9  | IGN2 output      | TIM8_CH2   | AF3 | → TLE8888 IGN1           |

## Signal Flow

```
Sensors → ADC → fuel_calc / ign_calc → ecu_sched
                                           │
                   TIM2 OC (INJ) ──────────┼──► TLE8888 low-side → Injectors
                   TIM8 OC (IGN) ──────────┼──► TLE8888 push-pull → Coils
                                           │
                   SPI2 (config/diag) ─────┼──► TLE8888 registers
                                           │
CKP/CMP → TIM5 IC → ckp driver → sync ────┘
```
