# Carrier VGT6 (`interface_vgt6/`)

> **Status:** vigente · validado 2026-11-23 (esquemático gerado e conferido com `kicad-cli` 10.0.7; sem PCB)

Esquemático KiCad 10 de 5 folhas (alimentação, saídas INJ/IGN, ETB e auxiliares, entradas, CAN e headers) da carrier que recebe a WeAct H562 VGT6. Especificação: `docs/hw/interface_vgt6.md` e `docs/hw/interface_vgt6_nets.md`. Abra `interface_vgt6.kicad_pro` no KiCad.

## Arquivos

- `interface_vgt6.kicad_sch` (raiz) e `power`, `outputs`, `etb_aux`, `inputs`, `comms` (`.kicad_sch`).
- `OpenEMS.kicad_sym` e `sym-lib-table`: símbolos do MAX9924 e do TPS2553, que a biblioteca oficial não tem.
- `gen/build.py`, `gen/kicadgen.py`: gerador.

## Regenerar

`python3 hardware/interface_vgt6/gen/build.py`. **O gerador é de uma passada**: ele reescreve todos os `.kicad_sch`, então depois de editar no KiCad não rode de novo sobre os mesmos arquivos.

## Conferir

```bash
kicad-cli sch erc hardware/interface_vgt6/interface_vgt6.kicad_sch
python3 tools/interface_netcheck.py
```

O `interface_netcheck.py` compara os pinos de P1/P2 da netlist com `docs/hw/interface_vgt6_nets.md` e `docs/hw/weact_h562_schematic.md`, e exige `PGND` e `SGND` separadas.

## Em aberto

Footprints, valores e posições dos componentes ainda são de projeto; não há PCB nem BOM fechada.
