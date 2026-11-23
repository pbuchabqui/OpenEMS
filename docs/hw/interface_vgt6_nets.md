# Placa de interface VGT6 — nets e conectores

> **Status:** vigente · validado 2026-11-22 (contra `src/hal/out_pins.h`, `src/hal/etb_driver.cpp`, `src/hal/sd.cpp` e `docs/hw/pinout.md`)

Lista de nets e de conectores da carrier descrita em `interface_vgt6.md` (o porquê de cada bloco vive lá). Os nomes das nets seguem o pino do MCU. **Não há número de pino de header WeAct aqui:** o pino 1 de P1/P2 ainda precisa ser conferido no PDF e na peça física (`weact_h562_schematic.md`). O slot SD fica na WeAct e não aparece na carrier (PC8..PC12 e PD2 sem uso). Cada net abaixo liga ao MCU pelo nome do pino (`PE0` etc.), e o mapeamento para o header sai do esquemático da WeAct.

Convenções: `_N` = ativo em baixo. `*_MCU` = lado 3,3 V do MCU. `*_CON` = lado do conector. Rails e terras: `VBAT_RAW_A`/`VBAT_RAW_B` (entradas J1A/J1B, antes do fusível), `VBAT_INJ`/`VBAT_IGN`/`VBAT_ETB`/`VBAT_AUX` (depois dos fusíveis por banco), `VBAT_PROT` (depois da proteção de inversão, para o buck), `+5V`, `+5V_SENS`, `+3V3`, `VDDA`, `VREF+`, `PGND` (potência), `SGND` (sinal).

## 1. Rails e terras

| Net | Origem | Destino | Notas |
|---|---|---|---|
| `VBAT_RAW_A`, `VBAT_RAW_B` | J1A, J1B | fusível de 15 A de cada entrada | 6–18 V; TVS aqui |
| `VBAT_PROT` | proteção de inversão | buck, relés | |
| `+5V` | buck 5 V | buffers 74AHCT125/08, ignitores, TJA1051T/3, ponte H lógica, pino 5 V da WeAct | conferir o diodo de backfeed do USB |
| `+5V_SENS` | `+5V` → ferrite → TPS2553 | sensores, MAX9924, pull-ups de Hall | limitado em corrente |
| `+3V3` | LDO da WeAct | Schmitt 74LVC1G17, VIO do CAN, pull-ups de NTC | |
| `VDDA`, `VREF+` | WeAct | ADC, divisores, clamps BAT54S | ferrite + 100 nF + 1 µF; `VREF+` ligado a `VDDA` |
| `PGND` | J1A, J1B | retorno de injetores, bobinas, ETB, relés | |
| `SGND` | ponto único com `PGND` | sensores, MCU, ADC | |

## 2. Nets de sinal por função

### 2.1 INJ/IGN e ARM

| Net MCU | Pino | Buffer (entrada → saída) | Net no conector | Observação |
|---|---|---|---|---|
| `INJ1_MCU` | PE0 | 74AHCT125 #1 | `INJ1_CON` (dreno do MOSFET) | gate 22–33 Ω, pull-down 10 kΩ |
| `INJ2_MCU` | PE2 | #1 | `INJ2_CON` | |
| `INJ3_MCU` | PE4 | #1 | `INJ3_CON` | |
| `INJ4_MCU` | PE6 | #1 | `INJ4_CON` | |
| `IGN1_MCU` | PE9 | 74AHCT125 #2 | `IGN1_CON` (5 V) | 33 Ω série, 10 kΩ para PGND |
| `IGN2_MCU` | PE11 | #2 | `IGN2_CON` | |
| `IGN3_MCU` | PE13 | #2 | `IGN3_CON` | |
| `IGN4_MCU` | PE15 | #2 | `IGN4_CON` | |
| `OE_N` | — | `OE#` dos buffers de INJ/IGN e do ETB | — | pull-up 10 kΩ para `+5V`; chave ARM para `SGND`; LED |
| `ARM_SENSE` | PE14 (opcional) | leitura de `OE_N` | — | firmware não usa hoje; `PE3` fica reserva |

### 2.2 ETB

| Net | Pino | Caminho | Observação |
|---|---|---|---|
| `ETB_PWM_MCU` | PE5 | 74AHCT125 #3 → AND | 10 kHz |
| `ETB_OPEN_MCU` | PE7 | 74AHCT125 #3 → AND | IN1, abrir |
| `ETB_CLOSE_MCU` | PE8 | 74AHCT125 #3 → AND | IN2, fechar |
| `ETB_IN1`, `ETB_IN2` | — | 74AHCT08 → DRV8871 | `IN1 = PWM & ABRIR`, `IN2 = PWM & FECHAR` |
| `ETB_M+`, `ETB_M-` | — | saída da ponte | para J4 |
| `ETB_TPS1_MCU` | PA2 | divisor + RC | pot. 1 |
| `ETB_TPS2_MCU` | PC5 | divisor + RC | pot. 2 |

O 74AHCT125 #3 usa o mesmo `OE_N`: sem ARM, a ponte vai a *coast*.

### 2.3 Auxiliares

| Net | Pino | Circuito | Conector |
|---|---|---|---|
| `PUMP_DRV` | PE10 | MOSFET + diodo + relé | J7 |
| `FAN_DRV` | PE12 | idem | J7 |
| `VVT_INT` | PB6 | MOSFET + Schottky | J7 |
| `VVT_EXH` | PB7 | idem | J7 |
| `AC_REQ` | PB11 | divisor 47 k/22 k + 10 nF | J6 |
| `FLEX_IN` | PB5 | pull-up 3,3 kΩ → Schmitt | J6 |

### 2.4 CKP/CMP

| Net | Pino | Caminho |
|---|---|---|
| `CKP_VR+`, `CKP_VR-` | — | J5 → MAX9924 |
| `CKP_HALL` | — | J5 → pull-up/Schmitt |
| `CKP_SEL` | — | `JP_CKP`: VR ou Hall |
| `CKP_MCU` | PA0 | 1 kΩ série; XOR opcional `JP_POL` |
| `CMP_HALL` | — | J5 → pull-up/Schmitt |
| `CMP_MCU` | PA1 | |

### 2.5 Analógicas (todas no ADC, 0–3,3 V)

| Net MCU | Pino | Sensor no conector | Condicionamento |
|---|---|---|---|
| `MAP_MCU` | PA3 | `MAP_SIG` | divisor + RC ≈ 2 kHz |
| `TPS_MCU` | PA4 | `TPS_SIG` | divisor + RC |
| `APP1_MCU` | PC0 | `APP1_SIG` | divisor + RC |
| `APP2_MCU` | PC2 | `APP2_SIG` | divisor + RC |
| `CLT_MCU` | PB0 | `CLT_SIG` | pull-up 2,49 kΩ para `+3V3`, 100 nF |
| `IAT_MCU` | PB1 | `IAT_SIG` | idem |
| `FUEL_MCU` | PC4 | `FUEL_SIG` | divisor + RC |
| `OIL_MCU` | PC1 | `OIL_SIG` | divisor + RC |
| `VBATT_MCU` | PC3 | `VBAT_PROT` | 100 k / 22 k, 100 nF |
| `KNOCK_MCU` | PA5 | `KNOCK_SIG` | DNP |

### 2.6 Comunicação e depuração

| Net | Pino(s) | Para |
|---|---|---|
| `CAN_RX_MCU`, `CAN_TX_MCU` | PB8, PB9 | TJA1051T/3 (`CAN_H`, `CAN_L` → J8) |
| `UART_TX`, `UART_RX` | PA9, PA10 | header de serviço |
| `USB_DM`, `USB_DP` | PA11, PA12 | USB-C da WeAct |
| `SWDIO`, `SWCLK`, `NRST` | PA13, PA14, — | header SWD |
| `LED_STATUS` | PB2 | LED |

## 3. Conectores

Pinagem inicial; quantidade de vias pode crescer com a escolha de sensores. Plugáveis, passo 5,08 mm (potência) e 3,5/3,81 mm (sinal).

| Ref | Função | Vias | Net por via |
|---|---|---|---|
| J1A | Alimentação INJ + IGN | 2 (×2 vias em paralelo) | `VBAT_RAW_A`, `PGND` |
| J1B | Alimentação ETB + AUX + lógica | 2 (×2 vias em paralelo) | `VBAT_RAW_B`, `PGND` |
| J2 | Injetores | 5 | `VBAT_INJ`, `INJ1_CON`, `INJ2_CON`, `INJ3_CON`, `INJ4_CON` |
| J3 | Ignição | 6 | `VBAT_IGN`, `PGND`, `IGN1_CON`..`IGN4_CON` (5 V de sinal) |
| J4 | Corpo de borboleta | 6 | `ETB_M+`, `ETB_M-`, `ETB_TPS1_SIG`, `ETB_TPS2_SIG`, `+5V_SENS`, `SGND` |
| J5 | Roda 60-2 e fase | 6 | `CKP_VR+`, `CKP_VR-`/`CKP_HALL` (cabo conforme sensor), `CMP_HALL`, `+5V_SENS`, `SGND`, blindagem |
| J6 | Sensores analógicos e digitais | 14 | `MAP_SIG`, `TPS_SIG`, `APP1_SIG`, `APP2_SIG`, `CLT_SIG`, `IAT_SIG`, `FUEL_SIG`, `OIL_SIG`, `FLEX_IN`, `AC_REQ`, `+5V_SENS`, `+5V_SENS`, `SGND`, `SGND` |
| J7 | Auxiliares | 6 | `VBAT_AUX`, `PGND`, `PUMP_DRV`, `FAN_DRV`, `VVT_INT`, `VVT_EXH` |
| J8 | CAN e WBO2 | 4 | `CAN_H`, `CAN_L`, `VBAT_AUX` (para o WBO2), `PGND` |
| J9 | Serviço | 6 | `+3V3`, `SWDIO`, `SWCLK`, `NRST`, `UART_TX`, `SGND` |
| JP_CKP, JP_CKP_PU, JP_POL | Jumpers | 3 / 2 / 2 | ver 2.4 |
| P1, P2 | Headers da WeAct | 2×20 | pino a pino conforme esquemático da WeAct |

Observações: o `ETB_TPS*_SIG` e o `+5V_SENS` do J4 evitam um conector extra só para os potenciômetros. J3 leva `PGND` porque o retorno de bobina nunca passa pelo plano de sensores. O blindagem de J5 aterra só no lado da ECU.

## 4. O que ainda não está fechado

- Numeração de P1/P2 contra a peça física.
- Quantidade e passo definitivos dos conectores (dependem de sensores, bobinas e do ambiente bancada/veículo).
- Corrente por via de J2, J3, J4, J7 e J8 (depende do orçamento de corrente, ainda não feito).
