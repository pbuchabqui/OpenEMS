# Pinmap lógico KiCad — OpenEMS interface v1

Fonte: `docs/hw/netlist_v1.md` (secção Conector) + `weact_h562_schematic.md`.  
⚠️ Numeração de **cavidade física** AMPSEAL (3 filas) valida-se no PDF TE; este mapa é a
**alocação lógica** materializada no sheet 09.

## J2 — 23-pos potência (`1-770669-1` / plug `770680-1`)

| Pos | Net global KiCad | Notas |
|----:|------------------|-------|
| 1–2 | `VBAT_RAW` | entrada bateria → reverse FET (sheet 01) |
| 3–5 | `PGND` | |
| 6 | `INJ1` | TLE OUT1 A+B |
| 7 | `INJ2` | |
| 8 | `INJ3` | |
| 9 | `INJ4` | |
| 10 | `IGN1` | gate IGBT |
| 11 | `IGN2` | |
| 12 | `IGN3` | |
| 13 | `IGN4` | |
| 14 | `VVT_EXH` | OUT5 A+B+C |
| 15 | `VVT_INT` | OUT6 A+B+C |
| 16 | `PUMP_RLY` | OUT14 |
| 17 | `FAN_RLY` | OUT15 |
| 18 | `MAIN_RLY` | MR pin55 **DNP v1** |
| 19–20 | `ETB_MOTOR_P` | duplicado |
| 21–22 | `ETB_MOTOR_N` | duplicado |
| 23 | *(livre)* | no-connect |

## J1 — 35-pos sinais (`1-776180-1` / plug `776164-1`)

| Pos | Net | Notas |
|----:|-----|-------|
| 1 | `CKP_P` | VR+ |
| 2 | `CKP_N` | VR− |
| 3 | `CKP_SHLD` | → `SHIELD_GND` |
| 4 | `CMP_SIG` | |
| 5 | `CMP_5V` | = `+5V_SENS_A` |
| 6 | `CMP_GND` | = `SGND` |
| 7 | `MAP` | |
| 8 | `CLT` | |
| 9 | `IAT` | |
| 10 | `APP1` | rail A |
| 11 | `APP2` | rail B |
| 12 | `FUEL_PRESS` | |
| 13 | `OIL_PRESS` | |
| 14 | `ETB_TPS1` | |
| 15 | `ETB_TPS2` | |
| 16 | `+5V_SENS_A` | T5V1 |
| 17 | `+5V_SENS_B` | T5V2 |
| 18–19 | `SGND` | |
| 20 | `CANH` | |
| 21 | `CANL` | |
| 22 | `CAN_SHLD` | → `SHIELD_GND` |
| 23 | `FLEX_12V` | = VBAT |
| 24 | `FLEX_SIG` | → MCU_PB5 |
| 25 | `FLEX_GND` | = SGND |
| 26 | `KNOCK_SIG` | DNP v1 |
| 27 | `KNOCK_SHLD` | DNP |
| 28–30 | `CMP2_*` | reserva |
| 31 | `TPS_INDEP` | reserva |
| 32–35 | *(livres)* | |

## MCU nets (prefixo `MCU_`)

| Net | Função |
|-----|--------|
| `MCU_PA0` | CKP dig |
| `MCU_PA1` | CMP dig |
| `MCU_PA2/3/4/5` | ETB_TPS1 / MAP / TPS / KNOCK |
| `MCU_PA11/12` | USB DM/DP |
| `MCU_PB0/1` | CLT / IAT |
| `MCU_PB5` | Flex |
| `MCU_PB6/7` | VVT |
| `MCU_PB8/9` | CAN RX/TX |
| `MCU_PB12–15` | SPI2 CS/SCK/MISO/MOSI |
| `MCU_PC0–5` | APP1 / OIL / APP2 / VBATT / FUEL / ETB_TPS2 |
| `MCU_PE0/2/4/6` | INJ1–4 |
| `MCU_PE1/3` | INJEN / IGNEN |
| `MCU_PE5/7/8` | ETB PWM / DIR1 / DIR2 |
| `MCU_PE9/11/13/15` | IGN1–4 |
| `MCU_PE10/12` | Pump / Fan |

## SPI straps (TLE)

| TLE pin | Strap |
|--------:|-------|
| 6 SIN | `AGND` |
| 8 FCLN | `+3V3` (VDDIO) |

## Footprints físicos (KiCad)

| Conector | Footprint | Lib |
|----------|-----------|-----|
| J2 23 | `TE_770669_AMPSEAL_23_RA` | `OpenEMS.pretty` |
| J1 35 | `TE_776180_AMPSEAL_35_RA` | `OpenEMS.pretty` |
| WeAct P1/P2 | `WeAct_PinHeader_2x25_P2.54mm` | `OpenEMS.pretty` |

### Matriz de pads TE (não linear 1…N na grelha)

**23-pos** (padrão TE / KiCad `TE_AMPSEAL_1-776087`):

```
y=0:  1  2  3  4  5  6  7  8     (x = 0,4,…,28)
y=4:   9 10 11 12 13 14 15       (x = 2,6,…,26)  staggered
y=8: 16 17 18 19 20 21 22 23     (x = 0,4,…,28)
```

**35-pos** (face TE 776180: 1–12 / 13–23 / 24–35):

```
y=0:  1  2 … 12                  (x = 0,4,…,44)
y=4: 13 14 … 23                  (x = 2,6,…,42)  staggered
y=8: 24 25 … 35                  (x = 0,4,…,44)
```

Furos: contacto Ø1.75 · montagem Ø2.85 a 6.5 mm dos extremos.  
Detalhe: `libs/OpenEMS.pretty/README.md`.
