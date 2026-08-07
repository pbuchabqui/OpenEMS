# microRusEFI como base de hardware OpenEMS

> **Decisão (2026-07-21):** o desenho da ECU OpenEMS parte do projecto KiCad  
> [rusefi/hw_microRusEfi](https://github.com/rusefi/hw_microRusEfi) (rev. vendor ~0.5.x),  
> **não** de um scaffold KiCad paralelo nem do merge Hellen-One como board de produção.

## Porquê

1. **TLE8888-2QK** já está no esquemático e no layout do microRusEFI (mRE) — alinhado ao firmware OpenEMS.  
2. Hierarquia real (TLE, ADC, MCU, H-bridge, hi-lo, pairs) e **PCB já fabricável**.  
3. Evita reinventar cobre fora do layout mRE já fabricável.  
4. Hellen-One continua útil **depois**, para adaptadores PnP por carro (submodule no próprio mRE).

## Vendor no monorepo

```bash
git submodule update --init hardware/vendor/hw_microRusEfi
# libs KiCad do mRE (após clone do vendor):
cd hardware/vendor/hw_microRusEfi && git submodule update --init --recursive
```

Caminho: `hardware/vendor/hw_microRusEfi/`

## Decisões de arquitectura OpenEMS

### MCU: STM32H562 **soldado** (LQFP100) — fechado

| Opção | Uso |
|-------|-----|
| **H562 LQFP100 na PCB principal** | **Produção / cabine** (igual filosofia mRE) |
| WeAct H562 | **Só bancada de firmware** — não entra na board de produção |
| Socket + soldado em paralelo | **Rejeitado** (espaço, BOM, confusão) |

Obrigações de design com MCU soldado: SWD acessível, USB, BOOT0, decoupling por pinos VDD, cristal conforme firmware H562, silkscreen pin 1.

### Ainda abertas (defaults sugeridos)

| Tema | Default sugerido | Alternativa |
|------|------------------|-------------|
| Conector | **AMPSEAL 35+23** (`interface_board_v1.md`) | 48-pin mRE + case CKKB |
| ETB | **TLE9201** (HW mRE) + adaptar SW | BTS7960 (firmware actual 3 pinos) |

## Mapa de sheets mRE → OpenEMS

| Sheet mRE | Ficheiro | Acção OpenEMS |
|-----------|----------|---------------|
| Raiz | `micro_rusEFI.kicad_sch` | → `openems_ecu.kicad_sch` |
| TLE8888 | `TLE8888-1QK.kicad_sch` | Adoptar; auditar vs `tle8888_pinout.md` |
| MCU | `stm32.kicad_sch` (F7/F4) | ✅ **`mcu_h562.kicad_sch`** (H562VGTx, INJEN=PE14) |
| ADC | `adc.kicad_sch` | Adaptar pinos H562 |
| H-bridge | `TLE9201SG.kicad_sch` | Manter ou trocar por BTS7960 |
| hi-lo | `hi-lo.kicad_sch` | USB / níveis |
| LowSides | `pair.kicad_sch` | Rever vs outs TLE |
| Flash | `FlashMemory.kicad_sch` | Opcional datalog |
| PCB | `micro_rusEFI.kicad_pcb` | Base de layout |

## Capacidades mRE (referência)

- 4× INJ high-Z, 4× IGN logic, 2× LS power, 4× LS relay  
- VR/Hall configurável, ETB, CAN, USB no plug  
- Caixa CKKB48-1-A (se conector 48 pin)

## O que **não** fazer

- Não reinventar a PCB fora de `hardware/openems_ecu/`.  
- Não montar WeAct na ECU de motor (grau consumidor + headers).  
- Não assumir pinout F767 = H562 — mapa explícito obrigatório.

## Autoridade de sinais

| Artefacto | Papel |
|-----------|--------|
| `docs/hw/netlist_v1.md`, `tle8888_pinout.md` | O *quê* ligar (alvo eléctrico OpenEMS / board limpo) |
| `docs/hw/pinout_mre_bringup.md` | **Bring-up:** GPIO = cobre [hw_microRusEfi](https://github.com/rusefi/hw_microRusEfi.git) |
| `src/hal/out_pins.h` + `BOARD=mre` | Firmware H562 no pinout mRE |
| `src/hal/out_pins.h` + `BOARD=vgt6` | Firmware mapa PE* OpenEMS (WeAct / PCB nativo) |
| Projecto mRE / `openems_ecu` | *Como* está desenhado no KiCad |

**H562 na PCB mRE:** sim — land pattern LQFP100 igual; 6 pads power H5; firmware
`make firmware BOARD=mre` (não forçar re-route VGT6 neste board).

## Próximos passos (implementação)

1. ~~`hardware/openems_ecu/` = cópia de trabalho do mRE~~ ✅  
2. ~~Sheet `mcu_h562` + footprint LQFP100~~ ✅ (`hardware/openems_ecu/mcu_h562.kicad_sch`)  
3. ~~Rework 6 pads H562 no PCB + VCAP 2,2 µF~~ ✅ (cobre + C25/C100; DRC visual pendente)  
4. Auditoria TLE + decisões conector/ETB  
5. Layout final / refill zones / Gerber  

Abrir: `kicad hardware/openems_ecu/openems_ecu.kicad_pro`  


## Créditos

Hardware base © rusEFI / microRusEFI contributors. OpenEMS adapta com atribuição no README do board.
