# OpenEMS — hardware

Caminho de fabrico da ECU OpenEMS: **microRusEFI adaptado**, não scaffold próprio.

## Estrutura

```
hardware/
├── README.md                 ← este ficheiro
├── openems_ecu/              ← projecto KiCad de produção (editar aqui)
│   ├── openems_ecu.kicad_pro
│   ├── openems_ecu.kicad_sch  ← raiz hierárquica
│   ├── openems_ecu.kicad_pcb
│   ├── mcu_h562.kicad_sch    ← STM32H562VGTx (activo)
│   ├── TLE8888-1QK.kicad_sch
│   ├── adc / hi-lo / pair / Flash / TLE9201 …
│   ├── rusefi_lib/           ← símbolos locais mRE
│   ├── rusefi_lib_external/  ← footprints/libs mRE
│   └── hellen-one → vendor   ← só USB footprint (symlink)
└── vendor/
    └── hw_microRusEfi/       ← submodule git (referência imutável)
```

## Abrir no KiCad 7+

```bash
kicad hardware/openems_ecu/openems_ecu.kicad_pro
```

## Vendor (submodule)

```bash
git submodule update --init hardware/vendor/hw_microRusEfi
cd hardware/vendor/hw_microRusEfi && git submodule update --init --recursive
```

Só consultar / copiar libs. **Não** editar o vendor como board de produção.

## Documentação

| Doc | Papel |
|-----|--------|
| [`docs/hw/microruseefi_as_base.md`](../docs/hw/microruseefi_as_base.md) | Decisão mRE + plano |
| [`docs/hw/netlist_v1.md`](../docs/hw/netlist_v1.md) | O que ligar a quê |
| [`docs/hw/pinout.md`](../docs/hw/pinout.md) | Pinos firmware H562 |
| [`docs/hw/mcu_f407_to_h562_pin_map.md`](../docs/hw/mcu_f407_to_h562_pin_map.md) | 6 pads F407→H562 |
| [`docs/hw/pinmap_logical.md`](../docs/hw/pinmap_logical.md) | Mapa lógico nets |

## O que **não** está aqui (de propósito)

- Scaffold `openems_interface_v1` (removido — não era fabricável)
- Clones avulsos de libs rusEFI/Speeduino sob `hardware/kicad/vendor`
- Hellen-One como base de ECU (só footprint USB via symlink)

## Créditos

Hardware base © rusEFI / microRusEFI. OpenEMS adapta com atribuição.
