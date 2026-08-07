# Pinout mRE bring-up (H562 no cobre microRusEFI)

> **Autoridade do cobre:** [rusefi/hw_microRusEfi](https://github.com/rusefi/hw_microRusEfi.git)  
> inventariado em `hardware/vendor/hw_microRusEfi/micro_rusEFI.kicad_pcb`  
> (U1 = STM32F427VGT6 LQFP100, U2 = TLE8888QK).  
> Firmware: `make firmware BOARD=mre` (`-DEMS_BOARD_MRE`).

## Objectivo

Correr **STM32H562VGTx** na PCB microRusEFI com o **mesmo mapa de GPIO**
que o layout rusEFI já roda, sem re-rotear MCU↔TLE.

- **Não** usar o mapa OpenEMS VGT6 “ideal” (`INJ PE0/2/4/6`, `IGN PE9/…`, SPI `PB12–15`).
- **Sim** rework eléctrico dos **6 pads** F4→H5 (VCAP, VDDUSB, VSSA, …) — ver
  `mcu_f407_to_h562_pin_map.md`.

## MCU ↔ TLE8888 (nets partilhadas no PCB vendor)

| Função | TLE pad | Net PCB mRE | MCU GPIO | VGT6 OpenEMS (alvo outro board) |
|--------|---------|-------------|----------|----------------------------------|
| INJ1–4 (IN1–4) | 28–31 | `/MCU/PE14`…`/MCU/PE11` | **PE14, PE13, PE12, PE11** | PE0/2/4/6 |
| IGN1–4 (IN5–8) | 32–35 | `/PD12`…`/PD15` | **PD12–15** | PE9/11/13/15 |
| IN9 | 36 | `/MCU/PE10` | **PE10** | PE10 (pump) |
| IN10 | 37 | `/MCU/PE9` | **PE9** | PE12 (fan) |
| IN11 / IN12 | 38 / 39 | `/MCU/PE8` / `/MCU/PE7` | **PE8 / PE7** | PB6/7 VVT |
| INJEN / IGNEN | 24 / 27 | `/MCU/PD11` / `/MCU/PD10` | **PD11 / PD10** | PE14 / PE3 |
| SPI CSN | 3 | `/CS_TLE` | **PD5** | PB12 |
| SPI SCK / MISO / MOSI | 7 / 4 / 5 | `/SCK` `/SO` `/SI` | **PB3 / PB4 / PB5** | PB13/14/15 |
| CAN RX / TX (via TLE) | 43 / 44 | `/MCU/PB12` / `/MCU/PB6` | **PB12 / PB6** | PB8 / PB9 |
| CKP dig (VROUT) | 21 | `/CRANK` | **PC6** | PA0 |
| LIN (opcional) | 22 / 23 | `/MCU/PD8` / `/MCU/PD9` | PD8 / PD9 | off OpenEMS v1 |

Trilhas já existem no vendor PCB (`tracks` contados no inventário: dezenas por net).

## Canais BSRR (`out_pins.h`, ordem ECU_CH_*)

| channel | Função | mRE pin |
|---------|--------|---------|
| 0 | INJ3 | PE12 |
| 1 | INJ4 | PE11 |
| 2 | INJ1 | PE14 |
| 3 | INJ2 | PE13 |
| 4 | IGN4 | PD15 |
| 5 | IGN3 | PD14 |
| 6 | IGN2 | PD13 |
| 7 | IGN1 | PD12 |

`power_stage_enable`: INJEN=**PD11**, IGNEN=**PD10** (HIGH = enable).

## SPI TLE no mRE

| Sinal | Pino | Peripheral H562 (bring-up) |
|-------|------|----------------------------|
| CSN | PD5 | GPIO push-pull |
| SCK | PB3 | **SPI1** AF5 |
| MISO | PB4 | **SPI1** AF5 |
| MOSI | PB5 | **SPI1** AF5 |

(VGT6 usa SPI2 em PB12–15 AF5.)

⚠️ PB3/4/5 partilham JTAG — usar **SWD** (PA13/14). Flash mRE no mesmo barramento SPI: manter CS da flash high se não usada.

## H562 vs F427 neste board

| Tema | Notas |
|------|--------|
| Land pattern | LQFP-100 14×14 P0.5 — idêntico |
| GPIO da tabela | Existem no H562VGTx |
| Pad 98 (PE1 no F4) | **VCAP** no H562 — mRE INJEN é PD11, não PE1 |
| Pad 48 (PB11 no F4) | **VCAP** no H562 — mRE não usa PB11 para TLE |
| AF numbers | Confirmar RM0481; SPI1 AF5 em PB3/4/5 alinhado a família ST |

## Builds

```bash
make firmware BOARD=mre      # /tmp/openems-build/bin/openems-mre.bin
make firmware BOARD=vgt6    # mapa PE* OpenEMS (WeAct / board limpo)
make firmware BOARD=rgt6    # LQFP64 default
```

## Relação com `openems_ecu`

O projecto de trabalho `hardware/openems_ecu/` é derivado do mRE. Se as nets
tiverem sido renomeadas para o mapa VGT6 e re-roteadas, o bring-up mRE exige
**restaurar nets/cobre TLE do vendor** (ou de um backup pré-VGT6) e manter só
o rework dos 6 pads H562.

## Ver também

- `microruseefi_as_base.md` — decisão de base KiCad  
- `mcu_f407_to_h562_pin_map.md` — 6 pads power  
- `pinout.md` — RGT6 vs VGT6 (alvo)  
- `tle8888_pinout.md` — números de package TLE  
