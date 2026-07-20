# OpenEMS Interface Board v1 — projecto KiCad 7

## Abrir (sem saber KiCad)

1. Instalar **KiCad 7** (já tens 7.0.x).
2. Duplo-clique em:

   `hardware/kicad/openems_interface_v1/openems_interface_v1.kicad_pro`

   ou: `kicad openems_interface_v1.kicad_pro`
3. **Schematic Editor** → folha raiz com 10 módulos → duplo-clique para entrar.
4. **Backspace** para voltar à raiz.

## O que está pronto

| Item | Estado |
|------|--------|
| Hierarquia 10 sheets | ✅ |
| TLE8888 multi-unit A–G + straps SPI + shorts A/B | ✅ |
| J2 23-pos pin→net completo (netlist) | ✅ lógicos |
| J1 35-pos pin→net completo (netlist) | ✅ lógicos |
| WeAct P1/P2 tabela crítica MCU | ✅ labels |
| Star GND + aliases shield/CMP | ✅ |
| Flex PB5, USB PA11/12, ADC nets | ✅ |
| Footprints TE AMPSEAL 23/35 RA | ✅ rusEFI 35 + TE 23 |
| Footprint WeAct 2×25 + NetTie star | ✅ |
| PCB outline 130×100 + keepouts | ✅ |
| Routing VBAT/PGND/ETB/CKP stubs | ✅ `route_power.py` (34 segs) |
| Pours PGND F+B | ✅ (Fill Zones no Pcbnew) |
| Fios TLE pin-exact no canvas | ⚠️ labels+fios aprox. |
| U3 TLE8888 LQFP-100 + nets chave | ✅ `place_power_stage.py` |
| Power chain Q1/F1/D1/C1/C2/U1 buck | ✅ colocados + stubs |
| Layout final / fabrico | ❌ |

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
