# OpenEMS ECU — KiCad (produção)

Cópia de trabalho a partir de [rusefi/hw_microRusEfi](https://github.com/rusefi/hw_microRusEfi),
adaptada ao firmware OpenEMS (**STM32H562VGT6** + **TLE8888-2QK**).

## Abrir

```bash
kicad hardware/openems_ecu/openems_ecu.kicad_pro
```

## Sheets

| Ficheiro | Função |
|----------|--------|
| `openems_ecu.kicad_sch` | Raiz hierárquica |
| `openems_ecu.kicad_pcb` | Layout (base mRE + rework H562) |
| `mcu_h562.kicad_sch` | MCU **STM32H562VGTx** LQFP100 (activo) |
| `stm32.kicad_sch` | Arquivo F407 — **não** está na hierarquia |
| `TLE8888-1QK.kicad_sch` | Hub TLE8888 |
| `TLE9201SG.kicad_sch` | ETB H-bridge mRE |
| `adc.kicad_sch` | Entradas analógicas |
| `hi-lo.kicad_sch` | USB / níveis |
| `pair.kicad_sch` | Low-sides (×2) |
| `FlashMemory.kicad_sch` | Flash opcional |

## Libs

| Caminho | Uso |
|---------|-----|
| `rusefi_lib/` | Símbolos legacy mRE (TLE, molex, …) |
| `rusefi_lib_external/` | Footprints / libs estendidas |
| `hellen-one` → `../vendor/hw_microRusEfi/hellen-one` | Footprint USB mini-B |

```bash
# Se libs faltarem após clone limpo:
git submodule update --init --recursive hardware/vendor/hw_microRusEfi
# rsync opcional a partir do vendor se rusefi_lib_external estiver incompleto
```

## Estado OpenEMS

1. ~~MCU H562 sheet~~ ✅ `mcu_h562.kicad_sch`, INJEN=`PE14`
2. ~~Rework 6 pads no PCB~~ ✅ VCAP×2, VDDUSB, VSSA/VSS; enables TLE→PE14/PE3
3. Auditoria TLE vs `docs/hw/tle8888_pinout.md`
4. Conector AMPSEAL 35+23 vs 48-pin mRE (aberto)
5. ETB TLE9201 vs BTS7960 (aberto)
6. DRC / refill zones / Gerber

Arquitectura: [`docs/hw/microruseefi_as_base.md`](../../docs/hw/microruseefi_as_base.md).

## Créditos

Hardware base © rusEFI / microRusEFI contributors.
