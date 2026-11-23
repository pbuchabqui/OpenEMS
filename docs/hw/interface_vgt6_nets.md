# Placa de interface VGT6 — nets e conectores

> **Status:** vigente · validado 2026-11-23 (contra `src/hal/out_pins.h`, `src/hal/etb_driver.cpp`, `src/hal/sd.cpp`, `docs/hw/pinout.md` e o esquemático `hardware/interface_vgt6/`; `tools/interface_netcheck.py` confere os pinos de P1/P2)

Lista de nets e de conectores da carrier descrita em `interface_vgt6.md` (o porquê de cada bloco vive lá). O esquemático KiCad em `hardware/interface_vgt6/` é a implementação; este documento é a especificação que o `tools/interface_netcheck.py` confronta com a netlist. A numeração de P1/P2 vem de `weact_h562_schematic.md` e ainda precisa ser conferida na peça física. O slot SD fica na WeAct e não aparece na carrier (PC8..PC12 e PD2 sem uso).

Convenções: `_N` = ativo em baixo. `*_MCU` = lado 3,3 V do MCU (pino do header). `*_CON` e `*_SIG` = lado do conector. `*_DRV` = dreno do MOSFET (lado da carga). Quando há um estágio entre o MCU e o conector, os dois lados têm nomes diferentes: a net do MCU é a que `tools/interface_netcheck.py` compara.

## 1. Rails e terras

| Net | Origem | Destino | Notas |
|---|---|---|---|
| `VBAT_RAW_A`, `VBAT_RAW_B` | J1A, J1B | fusível de 15 A de cada entrada | 6–18 V |
| `VBAT_A_F`, `VBAT_B_F` | fusível de 15 A | PFET de proteção de inversão | |
| `VBAT_A_P` | PFET da entrada A | fusíveis de INJ e IGN, TVS SMBJ20A | |
| `VBAT_PROT` | PFET da entrada B | fusíveis de ETB e AUX, buck, `VBATT` | |
| `VBAT_INJ`, `VBAT_IGN`, `VBAT_ETB`, `VBAT_AUX` | fusíveis por banco | J2, J3, ponte do ETB, J7/J8 | 7,5 A / 10 A / 5 A / 10 A |
| `+5V` | buck AP63205 | 74AHCT125/1G08, TJA1051T/3, LEDs, `VCC` do P4 da WeAct | o D4 da WeAct bloqueia o USB |
| `+5V_F` | `+5V` → ferrite | TPS2553 | |
| `+5V_SENS` | TPS2553 | sensores, MAX9924, pull-ups de Hall | limite ≈ 350 mA (314–397 mA) |
| `+3V3` | P3 da WeAct (`VDD33`) | Schmitts, XOR, VIO do CAN, pull-ups de NTC, clamps BAT54S | |
| `VREF+` | header P1 pino 6 | só 100 nF + 1 µF | a carrier **não** alimenta `VREF+` (conferir na WeAct) |
| `PGND` | J1A, J1B | retorno de injetores, bobinas, ETB, relés, VVT | |
| `SGND` | NetTie com `PGND` | sensores, MCU, ADC, lógica | único ponto de junção |

Não existe net chamada `GND`.

## 2. Nets de sinal por função

### 2.1 INJ/IGN e ARM

| Net MCU | Pino | Buffer (entrada → saída) | Net no conector | Observação |
|---|---|---|---|---|
| `INJ1_MCU` | PE0 | 74AHCT125 #1 → `INJ1_G` → 220 Ω (e diodo + 47 Ω via `INJ1_GD`, só ligar) → `INJ1_GG` | `INJ1_CON` (dreno do MOSFET) | pull-down 10 kΩ em entrada e gate |
| `INJ2_MCU` | PE2 | #1 | `INJ2_CON` | |
| `INJ3_MCU` | PE4 | #1 | `INJ3_CON` | |
| `INJ4_MCU` | PE6 | #1 | `INJ4_CON` | |
| `IGN1_MCU` | PE9 | 74AHCT125 #2 → `IGN1_G` → 33 Ω | `IGN1_CON` (5 V) | 10 kΩ para PGND, TVS SMF6.0A no conector |
| `IGN2_MCU` | PE11 | #2 | `IGN2_CON` | |
| `IGN3_MCU` | PE13 | #2 | `IGN3_CON` | |
| `IGN4_MCU` | PE15 | #2 | `IGN4_CON` | |
| `OE_N` | — | `OE#` dos três 74AHCT125 | — | pull-up 10 kΩ para `+5V`; chave ARM para `SGND`; LED |
| `ARM_SENSE` | PE14 | divisor 100 k / 150 k de `OE_N` | — | firmware não usa hoje; `PE3` fica reserva |

### 2.2 ETB

| Net | Pino | Caminho | Observação |
|---|---|---|---|
| `ETB_PWM_MCU` | PE5 | 74AHCT125 #3 → `ETB_PWM_B` | 10 kHz |
| `ETB_OPEN_MCU` | PE7 | 74AHCT125 #3 → `ETB_OPEN_B` | IN1, abrir |
| `ETB_CLOSE_MCU` | PE8 | 74AHCT125 #3 → `ETB_CLOSE_B` | IN2, fechar |
| `ETB_IN1`, `ETB_IN2` | — | 2 × 74AHCT1G08 → DRV8871 | `IN1 = PWM & ABRIR`, `IN2 = PWM & FECHAR` |
| `ETB_M+`, `ETB_M-` | — | saída da ponte | para J4 |
| `ETB_TPS1_MCU` | PA2 | divisor + RC | pot. 1 (`ETB_TPS1_SIG` no J4) |
| `ETB_TPS2_MCU` | PC5 | divisor + RC | pot. 2 (`ETB_TPS2_SIG` no J4) |

O 74AHCT125 #3 usa o mesmo `OE_N`: sem ARM, a ponte vai a *coast*. A quarta unidade do #3 fica com entrada em `SGND` e saída sem uso.

### 2.3 Auxiliares

| Net MCU | Pino | Circuito | Net no conector |
|---|---|---|---|
| `PUMP_MCU` | PE10 | 100 Ω → gate do AO3400A, pull-down 10 kΩ; diodo de roda livre | `PUMP_DRV` (J7) |
| `FAN_MCU` | PE12 | idem | `FAN_DRV` (J7) |
| `VVT_INT_MCU` | PB6 | 100 Ω → gate do AO3400A + Schottky | `VVT_INT` (J7) |
| `VVT_EXH_MCU` | PB7 | idem | `VVT_EXH` (J7) |
| `FLEX_MCU` | PB5 | 74LVC1G17 ← 100 pF ← 1 kΩ ← `FLEX_IN` (pull-up 3,3 kΩ em `+5V_SENS`) | `FLEX_IN` (J6) |

### 2.4 CKP/CMP

| Net | Pino | Caminho |
|---|---|---|
| `CKP_VR+`, `CKP_VR-` | — | J5 → 10 kΩ → MAX9924 (modo A2) |
| `CKP_VR_OUT` | — | `COUT` do MAX9924, dreno aberto, pull-up 10 kΩ em `+3V3` |
| `CKP_HALL` | — | J5 → TVS 6 V → pull-up 4,7 kΩ (`JP_CKP_PU`) → 1 kΩ + 100 pF → 74LVC1G17 → `CKP_H_S` |
| `CKP_SEL` | — | `JP_CKP`: 1-2 = `CKP_VR_OUT`, 2-3 = `CKP_H_S` |
| `CKP_X` | — | XOR 74LVC1G86 de `CKP_SEL` com `CKP_POL` (`JP_POL` fechado inverte) |
| `CKP_MCU` | PA0 | `CKP_X` → 1 kΩ |
| `CMP_HALL` | — | J5 → TVS 6 V → pull-up 4,7 kΩ → 1 kΩ + 100 pF → 74LVC1G17 → `CMP_H_S` |
| `CMP_MCU` | PA1 | `CMP_H_S` → 1 kΩ |

### 2.5 Analógicas (todas no ADC, 0–3,3 V)

| Net MCU | Pino | Sensor no conector | Condicionamento |
|---|---|---|---|
| `MAP_MCU` | PA3 | `MAP_SIG` | divisor + RC ≈ 2 kHz (10 nF) |
| `TPS_MCU` | PA4 | `TPS_SIG` | divisor + RC (39 nF) |
| `APP1_MCU` | PC0 | `APP1_SIG` | divisor + RC (39 nF) |
| `APP2_MCU` | PC2 | `APP2_SIG` | divisor + RC (39 nF) |
| `CLT_MCU` | PB0 | `CLT_SIG` | pull-up 2,49 kΩ para `+3V3`, 1 kΩ + 100 nF |
| `IAT_MCU` | PB1 | `IAT_SIG` | idem |
| `FUEL_MCU` | PC4 | `FUEL_SIG` | divisor + RC (220 nF) |
| `OIL_MCU` | PC1 | `OIL_SIG` | divisor + RC (220 nF) |
| `VBATT_MCU` | PC3 | `VBAT_PROT` | 100 k / 22 k, 1 kΩ, 100 nF |

`KNOCK_MCU` (PA5) fica NC no header (DNP). Todo canal analógico tem clamp BAT54S para `SGND` e `+3V3`.

### 2.6 Comunicação e depuração

| Net | Pino(s) | Para |
|---|---|---|
| `CAN_RX_MCU`, `CAN_TX_MCU` | PB8, PB9 | TJA1051T/3 (`CAN_H`, `CAN_L` → choke → J8) |
| `UART_TX`, `UART_RX` | PA9, PA10 | J9 (serviço) |
| `SWDIO`, `SWCLK`, `NRST` | PA13, PA14, — | só no P5 da WeAct (não saem em P1/P2); sem net na carrier |
| `USB_DM`, `USB_DP` | PA11, PA12 | USB-C da WeAct; sem net na carrier |
| `LED_STATUS` | PB2 | LED |

## 3. Conectores

Pinagem inicial; quantidade de vias pode crescer com a escolha de sensores. Plugáveis, passo 5,08 mm (potência) e 3,81 mm (sinal).

| Ref | Função | Vias | Net por via |
|---|---|---|---|
| J1A | Alimentação INJ + IGN | 4 | `VBAT_RAW_A` ×2, `PGND` ×2 |
| J1B | Alimentação ETB + AUX + lógica | 4 | `VBAT_RAW_B` ×2, `PGND` ×2 |
| J2 | Injetores | 5 | `VBAT_INJ`, `INJ1_CON`..`INJ4_CON` |
| J3 | Ignição | 8 | `VBAT_IGN` ×2, `PGND` ×2, `IGN1_CON`..`IGN4_CON` (5 V de sinal) |
| J4 | Corpo de borboleta | 6 | `ETB_M+`, `ETB_M-`, `ETB_TPS1_SIG`, `ETB_TPS2_SIG`, `+5V_SENS`, `SGND` |
| J5 | Roda 60-2 e fase | 7 | `CKP_VR+`, `CKP_VR-`, `CKP_HALL`, `CMP_HALL`, `+5V_SENS`, `SGND`, `SGND` (blindagem) |
| J6 | Sensores analógicos e digitais | 13 | `MAP_SIG`, `TPS_SIG`, `APP1_SIG`, `APP2_SIG`, `CLT_SIG`, `IAT_SIG`, `FUEL_SIG`, `OIL_SIG`, `FLEX_IN`, `+5V_SENS` ×2, `SGND` ×2 |
| J7 | Auxiliares | 6 | `VBAT_AUX`, `PGND`, `PUMP_DRV`, `FAN_DRV`, `VVT_INT`, `VVT_EXH` |
| J8 | CAN e WBO2 | 4 | `CAN_H`, `CAN_L`, `VBAT_AUX` (para o WBO2), `PGND` |
| J9 | Serviço | 4 | `+3V3`, `UART_TX`, `UART_RX`, `SGND` |
| JP_CKP (`JP1`), `JP_CKP_PU` (`JP2`), `JP_POL` (`JP3`), `JP_TERM` (`JP4`) | Jumpers | 3 / 2 / 2 / 2 | ver 2.4 e bloco CAN |
| SW_ARM | Chave ARM | 2 | `SGND`, `OE_N` |
| P1, P2 | Headers da WeAct | 2×20 | pino a pino conforme `weact_h562_schematic.md` |
| P3, P4 | Alimentação da WeAct (2×2) | 4 | pinos 1 e 2 = `+3V3` (P3) ou `+5V` (P4); pinos 3 e 4 = `SGND` |

Observações: o `ETB_TPS*_SIG` e o `+5V_SENS` do J4 evitam um conector extra só para os potenciômetros. J3 leva `PGND` porque o retorno de bobina nunca passa pelo plano de sensores. A blindagem de J5 aterra só no lado da ECU. O `JP_CKP_PU` fechado liga o pull-up de 4,7 kΩ ao `CKP_HALL`; abra se o sensor Hall for push-pull. Com sensor VR o `CKP_HALL` fica sem fio e o pull-up não interfere (os pinos do J5 são separados). P3 e P4: pinos 1 e 2 são o trilho e 3 e 4 o GND, pelo SchDoc (conferir na peça).

## 4. O que ainda não está fechado

- Numeração de P1/P2 e de P3/P4 contra a peça física.
- `VREF+` na WeAct: se já está ligado a `VDDA`, a carrier só desacopla; se estiver flutuante, o ADC precisa dele.
- Quantidade e passo definitivos dos conectores (dependem de sensores, bobinas e do ambiente bancada/veículo).
- Valores dos componentes passivos do esquemático ainda são de projeto e não foram validados em bancada.
