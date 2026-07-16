# OpenEMS — Electrical Wiring Diagram

> **⚠️ STALE DIAGRAM (pre-GPIOE scheduler):** the ASCII art below still shows
> TIM2/TIM8 INJ/IGN on PC/PA/PB and dual-use ETB/UART pins. **Firmware truth is
> README.md §5 “Pinout congelado”** (INJ/IGN PE0–15 BSRR, OIL PC1, ETB DIR PE7/8,
> ETB PWM PE5/TIM15). Re-draw this file against that table before production.

## STM32H562 (LQFP100 / GPIOE) + TLE8888 Smart Power Stage

```
                            ┌─────────────────────────────────┐
                            │        STM32H562RGT6            │
                            │       250 MHz Cortex-M33        │
                            │                                 │
  8 MHz HSE ───────────────►│ PH0/PH1 (OSC_IN/OUT)           │
                            │                                 │
  ┌─ CKP (60-2) ──► TLE8888 VRS_IN ──► VRS_OUT ──►│ PA0  TIM5_CH1 (AF2) ─┤── Input Capture (16ns)
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
  │  │                      │         EWG + AUXILIARIES        │
  │  │                      │ PA6  TIM3_CH1  (AF2) ──────────┼──► EWG H-bridge PWM
  │  │                      │ PA7  GPIO OUT  ─────────────────┼──► EWG H-bridge IN1
  │  │                      │ PB4  GPIO OUT  ─────────────────┼──► EWG H-bridge IN2
  │  │                      │ PB6  TIM4_CH1  (AF2) ──────────┼──► VVT Escape PWM
  │  │                      │ PB7  TIM4_CH2  (AF2) ──────────┼──► VVT Intake PWM
  │  │                      │                                 │
  │  │                      │         FLEX FUEL SENSOR         │
  │  │                      │ PB5  EXTI5 (input) ◄────────────┼─── Flex fuel freq signal
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
  │  │  │  VRS Conditioner (CKP signal conditioning)              │
  │  │  │    VRS_IN  ◄──── CKP reluctor (60-2)                   │
  │  │  │    VRS_OUT ────► PA0 (TIM5_CH1, input capture)          │
  │  │  │    Filter: medium, Hysteresis: 20mV, adaptive threshold │
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
  │  │  │  CAN Transceiver (HS-CAN, ISO 11898-2, integrated):      │
  │  │  │    TXD  ◄──── PB9 (FDCAN1_TX)                          │
  │  │  │    RXD  ────► PB8 (FDCAN1_RX)                          │
  │  │  │    CANH ────► CAN bus (120Ω termination each end)       │
  │  │  │    CANL ────►                                           │
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
| PA6  | EWG PWM          | TIM3_CH1   | AF2 | 10 kHz H-bridge          |
| PA7  | EWG DIR IN1      | GPIO       | -   | H-bridge direction       |
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
| PB4  | EWG DIR IN2      | GPIO       | -   | TODO(VGT6): dedicated pin|
| PB5  | Flex fuel sensor  | EXTI5      | -   | 50-150Hz freq input      |
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
CKP (reluctor) → TLE8888 VRS → VRS_OUT → PA0/TIM5 IC → ckp driver → sync
CMP (cam)      → PA1/TIM5 IC ──────────────────────────────────────────┘
                                                                        │
Sensors → ADC → fuel_calc / ign_calc → ecu_sched ◄─────────────────────┘
                                           │
                   TIM2 OC (INJ) ──────────┼──► TLE8888 low-side → Injectors
                   TIM8 OC (IGN) ──────────┼──► TLE8888 push-pull → Coils
                                           │
                   SPI2 (config/diag) ─────┼──► TLE8888 registers (VRS+INJ+IGN+WD)
                                           │
                   EWG cascade ────────────┼──► boost PI (20ms) → pos PID (2ms) → motor
```

## Power Supply

```
                    ┌─────────────────────────────────────────────────┐
                    │              POWER DISTRIBUTION                 │
                    │                                                 │
  Battery 12V ──►──┤ P-MOSFET (reverse polarity protection)         │
                    │    │                                            │
                    │  Fuse 30A                                       │
                    │    │                                            │
                    │  Main Relay (key-on or MCU-controlled)          │
                    │    │                                            │
                    │    ├── VBAT rail ──────────────────────────────│
                    │    │    │                                       │
                    │    │    ├── TLE8888 VBAT (100µF + 100nF)      │
                    │    │    ├── Fuel pump relay (fuse 15A)         │
                    │    │    ├── WBO2 controller (fuse 5A)         │
                    │    │    └── Flex fuel sensor 12V               │
                    │    │                                            │
                    │    │    └── SMBJ24CA TVS clamp on VBAT rail    │
                    │    │                                            │
                    │    ├── DC-DC Buck 5V (LM2596-HV or TPS54302)  │
                    │    │    │  Vin(max) 45V, load-dump survivable  │
                    │    │    │  10µF in + 22µF + 100nF out          │
                    │    │    ├── Sensor supply (MAP, TPS, APP1/2)  │
                    │    │    ├── NTC pull-ups (CLT, IAT)           │
                    │    │    ├── EWG position sensor                │
                    │    │    ├── ETB TPS1/TPS2                     │
                    │    │    └── Flex fuel pull-up                  │
                    │    │                                            │
                    │    └── LDO 3.3V from 5V (AMS1117-3.3, 10µF)  │
                    │         │  Vin=5V → Vdrop=1.7V, cool in SOT223│
                    │         ├── STM32 VDD (100nF per VDD pin)     │
                    │         └── STM32 VDDA (1µF + 100nF)          │
                    │                                                 │
                    └─────────────────────────────────────────────────┘
```

## Signal Conditioning

```
Generic analog input circuit (MAP, TPS, APP1/2, EWG pos):

  Sensor (0.5-4.5V) ──[R1 10k]──┬──[R_filt 1k]──┬──► STM32 ADC (3.3V max)
                                 │               │
                               [R2 15k]      [C 100nF]
                                 │               │
                                GND             GND
                                 │
                             [TVS 3.3V]
                                 │
                                GND

NTC thermistor input (CLT, IAT):

  5V ──[R_pull 2.49kΩ]──┬── NTC to GND
                         │
                    [R1 10k]──┬──[R_filt 1k]──┬──► STM32 ADC
                              │               │
                            [R2 15k]      [C 100nF]
                              │               │
                             GND             GND
```

| Sensor | Input Range | Divider | ADC Range | Filter | Protection |
|--------|-------------|---------|-----------|--------|------------|
| MAP | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| TPS | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| APP1/APP2 | 0.5–4.5V | 10k/15k | 0.3–2.7V | RC 1kΩ+100nF | TVS 3.3V |
| CLT (NTC) | 0–5V (via pull-up) | 10k/15k | 0–3.0V | RC 1kΩ+100nF | TVS 3.3V |
| IAT (NTC) | 0–5V (via pull-up) | 10k/15k | 0–3.0V | RC 1kΩ+100nF | TVS 3.3V |
| EWG pos | 0–5V (pot) | 10k/15k | 0–3.0V | RC 1kΩ+100nF | TVS 3.3V |
| Knock | Piezo AC | Bandpass 6–8kHz + amp | ±1.5V | Dedicated | Clamping diodes |
| Flex fuel | 0–12V square | 10k/3.3k | 0–3.0V | — | TVS 3.3V |

## External Actuators

```
┌─────────────────────────────────────────────────────────────┐
│          CAN Bus — TLE8888 Integrated Transceiver           │
│                                                             │
│  STM32 PB9 (FDCAN1_TX) ──► TLE8888 TXD                    │
│  STM32 PB8 (FDCAN1_RX) ◄── TLE8888 RXD                    │
│  TLE8888 CANH ──┬── bus ──┬── 120Ω termination             │
│  TLE8888 CANL ──┘         └── (each end)                   │
│                                                             │
│  No external CAN transceiver needed — HS-CAN PHY           │
│  (ISO 11898-2) integrated in TLE8888.                       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│          ETB — External H-Bridge (BTS7960 / VNH5019)        │
│                                                             │
│  STM32 PA8  (TIM1_CH1 PWM) ──► H-bridge PWM               │
│  STM32 PA10 (GPIO OUT)     ──► H-bridge DIR1 (IN1)        │
│  STM32 PB2  (GPIO OUT)     ──► H-bridge DIR2 (IN2)        │
│  ETB TPS1/TPS2 ──► ADC (position feedback)                 │
│  12V ──► H-bridge VCC      GND ──► H-bridge GND           │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│          EWG — External H-Bridge (BTS7960 / VNH5019)        │
│                                                             │
│  STM32 PA6  (TIM3_CH1 PWM) ──► H-bridge PWM (10kHz)       │
│  STM32 PA7  (GPIO OUT)     ──► H-bridge DIR1 (IN1)        │
│  STM32 PB4  (GPIO OUT)     ──► H-bridge DIR2 (IN2)        │
│  EWG position pot ──► divider ──► ADC2 (feedback)          │
│  12V ──► H-bridge VCC      GND ──► H-bridge GND           │
│  TODO(VGT6): PB4 shared with LED — dedicate on LQFP100    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│          WBO2 Lambda Controller                             │
│          (Bosch CJ125 / AEM 30-0300 / similar)             │
│                                                             │
│  12V ──► power (fuse 5A)                                    │
│  CANH/CANL ──► CAN bus (RX ID 0x180, configurable)         │
│  LSU 4.9 sonda ──► 6-pin connector                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│          Flex Fuel Sensor (GM / Continental)                 │
│                                                             │
│  12V ──► sensor power                                       │
│  Signal ──► [R1 10k]──┬──► PB5 (EXTI5)                    │
│                       [R2 3.3k]                              │
│                        │                                     │
│                       GND                                    │
│  Pull-up 10kΩ → 5V (if open-collector output)               │
│  Frequency: 50Hz = 0% ethanol, 150Hz = 100%                 │
│  Duty cycle: 10–90% = -40°C to +125°C fuel temp            │
└─────────────────────────────────────────────────────────────┘
```

## ECU Connector Pinout (TBD — ~47 pins)

| Group | Count | Signals |
|-------|-------|---------|
| Power | 4 | VBAT, Main relay ctrl, Fuel pump ctrl, PGND |
| Injection | 4 | INJ1–4 (TLE8888 OUT0–3 → injectors) |
| Ignition | 4 | IGN1–4 (TLE8888 COIL0–3 → coils) |
| CKP/CMP | 4 | CKP+ (VRS+), CKP- (VRS-), CMP signal, Shield GND |
| Analog sensors | 10 | MAP, TPS, CLT, IAT, APP1, APP2, Knock, EWG pos, 5V ref, SGND |
| ETB | 5 | Motor+, Motor-, TPS1, TPS2, 5V ref |
| EWG | 4 | Motor+, Motor-, Position, 5V ref |
| VVT | 2 | VVT escape (PB6), VVT intake (PB7) |
| CAN | 3 | CANH, CANL, Shield GND |
| Flex fuel | 3 | 12V supply, Signal, GND |
| USB | 2 | USB_DM, USB_DP (internal connector) |
| Reserve | 2 | Future expansion |
| **Total** | **~47** | |

## Grounding

```
                   Chassis stud (single star point)
                            │
                 ┌──────────┼──────────┐
                 │          │          │
               PGND       SGND     Shield GND
            (power)     (signal)  (shielding)
                 │          │          │
           ┌─────┤    ┌─────┤    ┌─────┤
           │ TLE8888   │ All     │ CAN bus
           │ LDO regs  │ sensors │ CKP cable
           │ H-bridge  │ ADC AGND│
           │ Fuel pump │ Pull-ups│
           └───────────┘─────────┘

  PCB rules:
  • Separate copper pours for PGND and SGND, joined at star point
  • Dedicated AGND pour for STM32 VSSA pin
  • Shielded cables: CKP and CAN (shield grounded at ECU end only)
  • 100nF bypass cap on every VDD pin of STM32
  • Bulk capacitor 100µF at TLE8888 VBAT
```
