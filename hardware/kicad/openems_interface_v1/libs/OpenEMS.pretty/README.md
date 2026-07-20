# OpenEMS footprints

## TE AMPSEAL RA

| Footprint | PN alvo | Pins | Fonte |
|-----------|---------|-----:|-------|
| `TE_770669_AMPSEAL_23_RA` | `1-770669-1` (gold, black) | 23 | `docs/hw/TE_770669_header_RA_23.pdf` + padrão KiCad `TE_AMPSEAL_1-776087` |
| `TE_776180_AMPSEAL_35_RA` | `1-776180-1` (gold, black) | 35 | `docs/hw/TE_776180_header_RA_35.pdf` (face: 1–12 / 13–23 / 24–35) |

### Dimensões PCB (TE, 1.57 mm board)

| Item | Valor |
|------|------:|
| Pitch contactos | **4.0 mm** |
| Row pitch | **4.0 mm** (fila do meio staggered +2 mm) |
| Furo contacto | **Ø 1.75 mm** after plating |
| Pad | 3.0 mm |
| Furos montagem | **Ø 2.85 mm** NPTH, 2× |
| Offset mount | **6.5 mm** para fora dos pinos extremos da fila exterior |

### Numeração física vs lógica OpenEMS

A **numeração dos pads** no footprint segue o **drawing TE** (face do header).  
A **alocação de nets** OpenEMS (J1.1 = CKP_P, …) é a **lógica** em `resources/pinmap_logical.md` /
`docs/hw/netlist_v1.md`.

⚠️ **Antes de fabricar:** cruzar pin 1 + key color (black COD1) com a peça física e o PDF.
A ordem lógica 1…N do netlist **pode** exigir remap se a cavidade TE não coincidir com a
sequência linear do chicote.

### WeAct

| Footprint | Uso |
|-----------|-----|
| `WeAct_PinHeader_2x25_P2.54mm` | Socket P1/P2 dual-row (Odd_Even) |

Furos M3 da board: ver `docs/hw/weact_h562_coreboard.md` (não neste .pretty).
