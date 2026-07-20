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
| Footprints TE AMPSEAL 23/35 RA | ✅ `OpenEMS.pretty` (TE PDF + padrão KiCad) |
| Footprint WeAct 2×25 | ✅ |
| PCB outline 130×100 + keepouts | ✅ (não fabricar) |
| Fios TLE pin-exact no canvas | ⚠️ labels+fios aprox. — afinar no KiCad |
| Layout copper / pours | ❌ |

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
python3 hardware/kicad/openems_interface_v1/scripts/generate_project.py
```

⚠️ Reescreve os `.kicad_sch`. Commit ou backup antes se editaste à mão.

## Próximos passos manuais

1. Annotate + ERC (muitos avisos de pin não ligado nos headers WeAct são esperados).
2. Ligar J3/J4 pin-a-pin com a tabela P1/P2 do sheet 09.
3. Footprints TE (página RECOMMENDED PCB LAYOUT dos PDFs).
4. Layout: star, pours, CKP longe de potência.
5. Gerber só depois.

## Speeduino / rusEFI

Ver `docs/hw/kicad_vendor_review.md`. J1 usa footprint **rusEFI AMPSEAL 35 RA**;
J2 o nosso TE 23; star **NetTie-4** (NT1).

## Bibliotecas

- `libs/OpenEMS.kicad_sym` + `sym-lib-table`
- Sistema: Device, Regulator_*, …
