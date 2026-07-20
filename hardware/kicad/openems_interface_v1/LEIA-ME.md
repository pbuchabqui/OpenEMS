# OpenEMS Interface Board v1 — projecto KiCad 7

## Abrir (sem saber KiCad)

1. Instalar **KiCad 7** (já tens 7.0.x).
2. Duplo-clique em:

   `hardware/kicad/openems_interface_v1/openems_interface_v1.kicad_pro`

   ou: `kicad openems_interface_v1.kicad_pro`
3. **Schematic Editor** → folha raiz com 10 módulos → duplo-clique para entrar.
4. **Backspace** para voltar à raiz.

## O que está pronto (scaffold v1 — **não fabricar ainda**)

| Item | Estado |
|------|--------|
| Hierarquia 10 sheets + netlist docs | ✅ |
| J1 rusEFI AMPSEAL 35 + J2 TE 23 | ✅ |
| WeAct J3/J4 + MH Φ3.2 + NetTie star | ✅ |
| U3 TLE8888 LQFP-100 + straps SPI | ✅ |
| Power: Q1/F1/D1/C bulk/U1 buck/U2 LDO/FB1 | ✅ |
| Copper: VBAT/PGND, INJ/IGN, CKP, SPI, CAN, drive | ✅ scaffold (~127 segs) |
| Pours PGND F+B | ✅ Fill Zones no Pcbnew |
| Esquemático fios densos | ⚠️ labels; PCB pinmap é autoridade |
| DRC manual / pin1 silk / FET DS | ❌ **bloqueia fabrico** |

## Mapa sheets ↔ docs

| Sheet | Doc |
|-------|-----|
| 01–10 | `docs/hw/schematic/0N_*.md` |
| pinos J1/J2 | `docs/hw/netlist_v1.md` (secção Conector) |
| TLE package | `docs/hw/tle8888_pinout.md` |
| WeAct headers | `docs/hw/weact_h562_schematic.md` |
| Pinmap KiCad | `resources/pinmap_logical.md` |

## Atalhos

| Ação | Tecla |
|------|-------|
| Fio | `W` |
| Global label | menu Place |
| Zoom fit | `Home` |
| ERC | Inspect → Electrical Rules Checker |
| Annotate | Tools → Annotate Schematic |

## Regenerar

```bash
# Só esquemático (cuidado: reescreve .kicad_sch)
python3 hardware/kicad/openems_interface_v1/scripts/generate_project.py

# PCB: placement + nets + routing
python3 hardware/kicad/openems_interface_v1/scripts/build_pcb_placement.py
```

⚠️ `generate_project.py` reescreve os sheets. Commit/backup se editaste à mão.

No Pcbnew: **Edit → Fill All Zones** (pours PGND).

## Próximos passos manuais

1. Fill All Zones + DRC.
2. Colocar TLE8888 LQFP-100 + FET/fuse/buck na zona Eco1 “TLE/PWR”.
3. Completar copper SPI (WeAct PB12–15 → TLE) e ADC.
4. Ligar J3/J4 pin-a-pin (tabela sheet 09).
5. Gerber só depois.

## Speeduino / rusEFI

Ver `docs/hw/kicad_vendor_review.md`. J1 usa footprint **rusEFI AMPSEAL 35 RA**;
J2 o nosso TE 23; star **NetTie-4** (NT1).

## Bibliotecas

- `libs/OpenEMS.kicad_sym` + `sym-lib-table`
- Sistema: Device, Regulator_*, …
