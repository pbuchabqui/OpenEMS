# Revisão KiCad: Speeduino + rusEFI (vs OpenEMS)

> **Data:** 2026-07-20  
> **Regra de projecto:** consultar Speeduino/rusEFI **antes** de inventar HW.  
> Clones de referência (não versionados no git): `hardware/kicad/vendor/`

## Fontes

| Projecto | Repo | Notas |
|----------|------|-------|
| **rusEFI** (actual) | [rusefi/kicad6-libraries](https://github.com/rusefi/kicad6-libraries) | KiCad 6+ mods/syms soltos |
| **rusEFI** (legacy) | [rusefi/kicad-legacy-libraries](https://github.com/rusefi/kicad-legacy-libraries) | `.lib` + `rusefi_lib.pretty` |
| **Speeduino** | [speeduino/kicad-parts](https://github.com/speeduino/kicad-parts) | Automotive connectors + ICs |

Re-clonar:

```bash
mkdir -p hardware/kicad/vendor && cd hardware/kicad/vendor
git clone --depth 1 https://github.com/rusefi/kicad6-libraries.git rusefi-kicad6-libraries
git clone --depth 1 https://github.com/rusefi/kicad-legacy-libraries.git rusefi-kicad-legacy
git clone --depth 1 https://github.com/speeduino/kicad-parts.git speeduino-kicad-parts
```

---

## TLE8888

| Item | rusEFI | OpenEMS | Acção |
|------|--------|---------|-------|
| Símbolo | `tle8888qk.lib` (legacy, 101 pinos incl. tab GND=101) | `OpenEMS:TLE8888` multi-unit A–G | **Adoptamos** nomes/pinos DS; multi-unit nosso é mais legível no sch |
| Pin numbers | 1–100 + 101 tab | 1–100 (tab = PGND no copper) | ✅ Alinhado a `tle8888_pinout.md` |
| Conversão .lib→.kicad_sym | `kicad-cli sym upgrade` falhou no .lib cru | — | Manter multi-unit OpenEMS; legacy só referência |
| Footprint LQFP-100 | LQFP-64/80 em kicad6; 100 no KiCad system | `Package_QFP:LQFP-100_14x14mm_P0.5mm` | **Adoptamos** system KiCad |

**Rejeitamos** copiar o monobloco .lib para o sch diário — hierarquia A–G encaixa nos sheets 02/04.

---

## AMPSEAL

| Item | rusEFI | Speeduino | OpenEMS |
|------|--------|-----------|---------|
| 35 RA | ✅ `AMPSEAL_35_RIGHT_ANGLE` (tags **776180 / 1-776180**) | ❌ (Delphi Sicma, TE 174917, …) | **Importado** → `rusEFI_AMPSEAL_35_RA_776180` |
| 35 vert | tags 776230/776231 | — | DNP path (usamos RA) |
| 23 RA | 3D only (`AMPSEAL_23_STRAIGHT.stp`); sem .kicad_mod 23 no legacy | ❌ | **Nosso** `TE_770669_AMPSEAL_23_RA` (padrão TE + KiCad TE_AMPSEAL_1-776087) |

### Geometria 35 (rusEFI vs nosso gerador)

| Parâmetro | rusEFI | OpenEMS gerado | Match |
|-----------|-------:|---------------:|:-----:|
| Pitch | 4.0 mm | 4.0 mm | ✅ |
| Row pitch | 4.0 mm | 4.0 mm | ✅ |
| Fila média stagger | +2 mm | +2 mm | ✅ |
| Drill | 1.75 mm | 1.75 mm | ✅ |
| Pad size | **2.45** mm | **2.45** (ajustado) | ✅ |
| Mount Ø | 2.85 mm | 2.85 mm | ✅ |
| Mount span | 57.0 mm | 57.0 mm | ✅ |
| Origem | centro | pad1 (nosso) / centro (import rusEFI) | ⚠️ |

**Adoptamos** o footprint rusEFI para **J1** no PCB (proveniente de boards microRusEFI/Hellen).  
**Mantemos** o nosso 23 para **J2** (não há 23 RA no legacy).

---

## Speeduino — o que é útil

| Módulo | Conteúdo | OpenEMS v1 |
|--------|----------|------------|
| `IC_Automotive` | MAX9926, MC33810, VN7004, TC4424, … | **Rejeitamos** MAX9926 (CKP = TLE VR); MC33810 não é o path TLE direct-drive |
| `Connector_Automotive` | Delphi Sicma 24/39, TE 1123038, 174917 | **Rejeitamos** (decisão AMPSEAL 35+23) |
| `Shields` | Mega2560, Teensy | N/A (WeAct H562) |
| `Misc` | MAP MPX*, logos | Opcional se MAP bare-die |

**Adoptamos o estilo Speeduino:** libs por domínio (Connector / IC / Misc) — já reflectido em `OpenEMS.pretty` + docs de módulos.

---

## Outros (rusEFI kicad6)

- Muitos conectores TE/OEM (OBD, Mercedes 128-pin, …) — **não** para v1.  
- Passivos 0402–1206 3D — opcional depois.  
- `NetTie` — usamos **KiCad system** `NetTie-4_THT` copiado para `OpenEMS.pretty` (star GND).

---

## Integração no projecto OpenEMS

| Ficheiro | Origem |
|----------|--------|
| `libs/OpenEMS.pretty/rusEFI_AMPSEAL_35_RA_776180.kicad_mod` | rusEFI legacy (import + modernize) |
| `libs/OpenEMS.pretty/TE_770669_AMPSEAL_23_RA.kicad_mod` | TE PDF + padrão KiCad / alinhado pad 2.45 |
| `libs/OpenEMS.pretty/NetTie-4_THT_Pad1.0mm.kicad_mod` | KiCad system |
| `libs/OpenEMS.kicad_sym` TLE multi-unit | OpenEMS (pinout cruzado com rusEFI .lib + DS) |

PCB: `build_pcb_placement.py` usa **J1 = rusEFI 35**, **J2 = TE 23**, **NT1 = NetTie-4**.

---

## Checklist futuro

- [ ] 3D STEP AMPSEAL do rusEFI `3d/` se render for necessário  
- [ ] Footprint AMPSEAL **23 RA** se aparecer no kicad6-libraries (hoje só straight 3D)  
- [ ] Opcional: símbolo TLE monobloco rusEFI como unit “overview”  
- [ ] Speeduino MAP footprints se escolhermos sensor bare no BOM  
