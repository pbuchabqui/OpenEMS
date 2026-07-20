# AMPSEAL — plugues e headers PCB (TE)

## Fontes

| Documento | Conteúdo | Arquivo no repo |
|-----------|----------|-----------------|
| ENG_CD_**2293782** Rev E5 | Família de **plugues** (fêmea chicote) | `ENG_CD_2293782_E5.pdf` |
| TE **776180** | Header **35-pos right-angle** (PCB) | `TE_776180_header_RA_35.pdf` |
| TE **776230** | Header **35-pos vertical** (PCB) | `TE_776230_header_vert_35.pdf` |
| TE **770669** | Header **23-pos right-angle** (PCB) | `TE_770669_header_RA_23.pdf` |

Pitch de contacto: **4,0 mm** centerline, **3 filas**, pin Ø **1,3 mm**.  
Corrente: **8 A** (estanho) / **17 A** (ouro) por contacto — daí pinos ETB duplicados e preferência ouro no J2.

---

## BOM recomendada OpenEMS v1 (cabine, mesma aresta)

**Decisão de orientação:** headers **right-angle (90°)** — o chicote sai paralelo à placa, na aresta
dos conectores (plano: ambos na mesma aresta). Vertical (180°) só se a caixa for “plug de cima”.

| Função | Pos | Plug (chicote) | Header PCB (black = COD1) | Plating |
|--------|-----|----------------|---------------------------|---------|
| **J1 sinais** | 35 | **`776164-1`** | **`1-776180-1`** (RA, gold) ou `776180-1` (tin) | **Gold** recomendado |
| **J2 potência** | 23 | **`770680-1`** | **`1-770669-1`** (RA, gold) | **Gold** (ETB stall) |

Alternativa vertical 35-pos: `1-776230-1` / `776230-1` (mates `776164-1`).  
Cores do housing = **key mecânica** — plug e header da **mesma cor**.

Crimp plug: **770520** (carretel) / **770854** (loose); ver Table 1 no 2293782.

---

## Envelopes

### Plugues (lado chicote) — 2293782

| Size | Dim A | Dim B | Dim C |
|------|------:|------:|------:|
| **23-pos** | **47,4** | **50** | **27,60** |
| **35-pos** | **63,4** | **66** | **27,45** |

### Headers RA (PCB) — cotas principais dos drawings

| Header | Notas de envelope (mm, aprox. do CD) |
|--------|--------------------------------------|
| **776180** 35 RA | Comprimento corpo ≈ **60,5**; altura acima PCB ≈ **18,1**; mounting screws **M2,5**; PCB **1,57 mm** |
| **770669** 23 RA | Largura ≈ **47,4**; comprimento ≈ **50,4**; altura ≈ **18,1**; secções de filas a **5,8 / 9,8 / 13,8** |

⚠️ Usar sempre a página **“RECOMMENDED P.C. BOARD LAYOUT”** do PDF do header para o
footprint KiCad (não inventar pads). Fixações: painel/placa **4,75 ± 0,15 mm** no processo TE;
parafuso **2,5×12** Delta PT (validação TE).

### Aresta mínima da placa (dois RA lado a lado)

```
≈ 60,5 (35) + 50,4 (23) + folga entre caixas (recomendado ≥ 8 mm)
≈ 119 mm  →  projectar ≥ 125–130 mm de aresta de conector
```

Profundidade (RA): ≈ **18 mm** de corpo + plug encaixado (~40 mm extra no chicote) — dimensionar
caixa e alívio de tração em cima disto.

---

## Materiais (plug 2293782)

- Housing UL94V-0 PBT+PC; wedge vermelho; cover preto; seals silicone.  
- IP67 típico da família; −40…+125 °C (plugs); headers listam −40…+105 °C em algumas PNs.  
- Variantes com cavidades bloqueadas — confirmar na peça ao pinoutar o chicote.

---

## Footprints KiCad (gerados)

| Header | Footprint | Path |
|--------|-----------|------|
| 23 RA `1-770669-1` | `TE_770669_AMPSEAL_23_RA` | `hardware/kicad/openems_interface_v1/libs/OpenEMS.pretty/` |
| 35 RA `1-776180-1` | `TE_776180_AMPSEAL_35_RA` | idem |

Cotas: pitch 4 mm, furo Ø1.75, mount Ø2.85 @ 6.5 mm. Ver `libs/OpenEMS.pretty/README.md`.

## Checklist KiCad

- [ ] Footprint 35 RA a partir de `TE_776180_header_RA_35.pdf` (layout page)  
- [ ] Footprint 23 RA a partir de `TE_770669_header_RA_23.pdf`  
- [ ] Pin 1 / key orientation idênticos entre J1 e J2 e a tabela de nets  
- [ ] Furos de fixação M2,5 + washers isolantes se painel metálico  
- [ ] ETB Motor+/− em **dois contactos cada** no 23-pos (gold)  
- [ ] Separação física J1 (sinais) / J2 (potência) — tamanhos diferentes  
