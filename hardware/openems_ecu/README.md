# OpenEMS ECU — projecto KiCad (base microRusEFI)

> **Caminho de fabrico activo.** Cópia de trabalho a partir de  
> [rusefi/hw_microRusEfi](https://github.com/rusefi/hw_microRusEfi) (~0.5.x),  
> adaptada ao firmware OpenEMS (**STM32H562** + **TLE8888-2QK**).

## Não confundir

| Caminho | Papel |
|---------|--------|
| **`hardware/openems_ecu/`** | **Board de produção / desenho** (este projecto) |
| `hardware/vendor/hw_microRusEfi/` | Vendor imutável (submodule git) — só referência |
| `hardware/kicad/openems_interface_v1/` | Scaffold experimental — **não fabricar** |
| Hellen-One | **Não** é a base desta ECU (opcional no futuro: adaptadores PnP) |

Documentação de arquitectura: `docs/hw/microruseefi_as_base.md`.

## Abrir no KiCad 7+

```bash
kicad hardware/openems_ecu/openems_ecu.kicad_pro
```

### Dependências de libs (submodules do vendor mRE)

O projecto referencia `rusefi_lib_external` e, em alguns footprints, `hellen-one`
(só como lib do mRE original — **não** como fluxo de merge de boards).

```bash
cd hardware/vendor/hw_microRusEfi
git submodule update --init --recursive
# se openems_ecu não tiver libs completas, copiar/atualizar a partir do vendor:
# rsync -a --exclude='.git' rusefi_lib_external/ ../../openems_ecu/rusefi_lib_external/
```

Neste working tree as libs estão em `rusefi_lib/` e `rusefi_lib_external/`.

## Ficheiros principais

| Ficheiro | Conteúdo |
|----------|----------|
| `openems_ecu.kicad_pro` | Projecto KiCad |
| `openems_ecu.kicad_sch` | Raiz hierárquica |
| `openems_ecu.kicad_pcb` | Layout (herdado do mRE) |
| `TLE8888-1QK.kicad_sch` | Hub TLE8888 (auditar vs docs OpenEMS) |
| `stm32.kicad_sch` | MCU **vendor F7/F4** — a substituir por H562 |
| `TLE9201SG.kicad_sch` | ETB H-bridge mRE |
| `adc.kicad_sch`, `hi-lo.kicad_sch`, `pair.kicad_sch`, … | I/O e suporte |

## Plano de adaptação OpenEMS

1. **MCU:** substituir sheet `stm32` → `mcu_h562` (LQFP100 soldado, não WeAct).  
2. **TLE:** cruzar pinos/nets com `docs/hw/tle8888_pinout.md` e `netlist_v1.md`.  
3. **Conector:** default OpenEMS AMPSEAL 35+23 *ou* manter 48-pin mRE (decisão aberta).  
4. **ETB:** TLE9201 (HW mRE) vs BTS7960 (firmware actual) — decisão aberta.  
5. Layout / DRC / Gerber a partir **deste** PCB, não do scaffold.

## Créditos

Hardware base © rusEFI / microRusEFI contributors. OpenEMS adapta com atribuição.
