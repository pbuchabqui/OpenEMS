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
| Layout limpo (placement + bridges locais) | ✅ ~22 segs, sem spaghetti |
| Pours PGND F+B | ✅ Fill Zones no Pcbnew |
| Routing fino completo | ❌ **à mão no KiCad** |
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

# PCB limpo (recomendado)
bash hardware/kicad/openems_interface_v1/scripts/build_all.sh
# = python3 scripts/layout_clean.py
```

⚠️ `generate_project.py` reescreve os sheets. Commit/backup se editaste à mão.

### Ver o PCB limpo

**Importante:** fecha o KiCad por completo e volta a abrir (senão podes estar a ver o ficheiro antigo em memória).



1. `kicad …/openems_interface_v1.kicad_pro`
2. **PCB Editor**
3. Tecla **Home** (zoom fit)
4. **Edit → Fill All Zones** (pours PGND)
5. Zonas: conectores em baixo · WeAct centro · power · U3 TLE

Só há **bridges locais** (pares J2, A+B do TLE, cadeia Q1–F1–C1–BAT).  
O resto rotea-se **à mão** no KiCad — não há mais auto-route emaranhado.

## Próximos passos manuais

1. Fill All Zones + DRC.
2. Rotear SPI (J3.29–32 → U3), CAN, INJ/IGN à mão.
3. Confirmar pin1 AMPSEAL/WeAct e pinout FET/LDO.
4. Gerber só depois.

## Speeduino / rusEFI

Ver `docs/hw/kicad_vendor_review.md`. J1 usa footprint **rusEFI AMPSEAL 35 RA**;
J2 o nosso TE 23; star **NetTie-4** (NT1).

## Bibliotecas

- `libs/OpenEMS.kicad_sym` + `sym-lib-table`
- Sistema: Device, Regulator_*, …
