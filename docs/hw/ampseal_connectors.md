# AMPSEAL — plugues e headers (TE)

> **Fonte:** TE Connectivity Customer Drawing **ENG_CD_2293782** Rev **E5**  
> (*AMPSEAL Family Drawing*, sheet 1–2). PDF: `docs/hw/ENG_CD_2293782_E5.pdf`  
> (origem: `~/Downloads/ENG_CD_2293782_E5.pdf`).

Desenho de **família de plugues** (fêmea, lado chicote). Inclui as referências do plano
OpenEMS **`776164-*` (35 vias)** e **`770680-*` (23 vias)**.

---

## Part numbers OpenEMS v1

| Função | Posições | Plug (chicote) | Cor housing típica | Mating header PCB (tabela TE) |
|--------|----------|----------------|--------------------|-------------------------------|
| **J1 sinais** | **35** | **`776164-1`** (black) | black / natural / blue / grey / orange | Vertical: **776230 / 776231**; right-angle: **776180 / 776163** (RED key) |
| **J2 potência** | **23** | **`770680-1`** (black) | black / natural / grey / blue / green | Vertical: **776200 / 776228**; right-angle: **770669 / 776087** (GREEN key) |

Outras cores do mesmo tamanho (ex. `776164-2` natural) são **keyed** — só encaixam no
header da mesma cor. **Tamanhos 35 vs 23 são fisicamente incompatíveis** (plano).

---

## Dimensões de envelope (plug, mm) — tabela do desenho

| Size | Dim A | Dim B | Dim C |
|------|------:|------:|------:|
| 8-pos | 27,4 | 40 | 27,50 |
| 14-pos | 35,4 | 44 | 27,60 |
| **23-pos** (`770680`) | **47,4** | **50** | **27,60** |
| **35-pos** (`776164`) | **63,4** | **66** | **27,45** |

Interpretação típica (ver vistas no PDF): **A/B ≈ comprimento/largura de envelope do
plug montado**; **C ≈ altura**. Usar A+B+C + folga de manobra na aresta da caixa.

Cotas laterais no desenho (23-pos mostrado): comprimento montado ≈ **38,7 / 41,4 mm**
(com/sem detalhe de cover — ver vista). **Confirmar no header PCB drawing** da
referência escolhida (vertical vs cotovelo) antes de fixar o outline da placa.

---

## Materiais e crimp (notas TE)

- Housing: UL94V-0 PBT+PC; locking wedge vermelho; cover preto; seals silicone.  
- Terminais plug: **770520** (carretel) ou **770854** (loose).  
  Ex.: 770520-1 → 0,5–0,8 mm² / 20–18 AWG; 770520-3 → 1,0–1,5 mm² / 16 AWG.  
- Alguns circuitos **bloqueados** em variantes (ex. 23-pos: cavidades 3, 7, 16, 17, 21, 22
  em certas revisões) — **confirmar na peça e na tabela de cavidades** ao pinoutar o chicote.

---

## Implicações layout OpenEMS

1. Dois conectores **na mesma aresta** (plano): largura mínima ≈ **63,4 + 47,4 + folga**
   entre caixas (~5–10 mm) ≈ **120 mm** de aresta útil só para conectores — dimensionar
   caixa/placa em torno disto **depois** de escolher vertical vs right-angle.  
2. **Header PCB** é peça distinta do plug — desenhos 776230/776180 (35) e 770669/776087 (23).  
3. Contactos: ETB motor em pinos **duplicados** (stall 8–10 A) — preferir contactos dourados
   se disponíveis no sistema AMPSEAL escolhido.  
4. IP67 / −40…+125 °C (família) — coerente com cabine; compartimento de motor exige rever
   caixa, não só o conector.

---

## Ainda em falta

- Customer drawing do **header PCB** exacto (footprint de solda, pin 1, altura acima da PCB).  
- Mapa cavidade 1…N → nets OpenEMS (após escolher cor/key e variante vertical/RA).
