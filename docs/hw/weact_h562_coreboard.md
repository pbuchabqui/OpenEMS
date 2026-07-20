# WeAct STM32H5xx VxTx CoreBoard V1.0 — forma e headers

> **Fonte:** `WeAct-STM32H5xxVxTxCoreBoard_V10 Board Shape 外形.pdf`  
> (Altium mechanical / Board Shape, 3 páginas; local: `~/Downloads/…`).  
> Placa **V1.0 WeAct Studio** com **LQFP100 / GPIOE** (pinos E0–E15 nos headers) — alvo
> da interface OpenEMS **VGT6**, não a variante 64 pinos.

---

## Dimensões de contorno (mm)

| Grandeza | Valor | Notas |
|----------|------:|-------|
| Largura (eixo curto) | **38,62** | Entre bordas exteriores |
| Comprimento (eixo longo) | **69,10** | |
| Distância entre centros dos furos (largura) | **30,48** | = 12 × 2,54 mm |
| Distância entre centros dos furos (comprimento) | ≈ **63,50** | 69,10 − 2×2,80 |
| Offset furo → borda (canto) | **2,80** | Centros dos 4 furos |
| Diâmetro furo montagem | **Φ 3,2** | |
| Topo → 1.ª fila de pads (aprox.) | **7,88** | Dimensão no desenho |

Sistema de coordenadas sugerido para footprint da carrier (origem = canto inferior esquerdo da placa):

```
Furos (centro), origem no canto SW da placa:
  SW  ( 2,80 ;  2,80 )
  SE  ( 2,80 + 30,48 = 33,28 ;  2,80 )
  NW  ( 2,80 ;  2,80 + 63,50 ≈ 66,30 )
  NE  ( 33,28 ; 66,30 )
```

⚠️ Confirmar no PDF a distância longitudinal exacta dos furos (63,50 mm é
derivada: 69,10 − 2×2,80). A largura 30,48 mm está cotada no desenho.

---

## Headers

- **Dois conectores laterais** (esquerda / direita), **dupla fila**, passo típico
  **2,54 mm** (compatível com a cota 30,48 mm entre furos).
- Pinagem silkscreen (resumo útil OpenEMS VGT6):

| Lado | Conteúdo relevante |
|------|--------------------|
| Esquerda (top→bottom, típico) | PC0–PC5, GND, V+, PA0–PA7, PB0–PB1, …, **PE7–PE15**, GND, PB10–PB15, PD8–… |
| Direita | … **PE0–PE6**, PB3–PB9, PD0–PD7, PC10–PC12, PA8–PA15, 5V, GND |
| Extra | USB-C na base; botões **NRST** / **BOOT0**; LED **PC13**; SWD (3V3, DIO, CLK, GND, NR, TX, RX) |

**I/O OpenEMS críticos presentes no header (VGT6):**
- CKP/CMP: **PA0 / PA1**
- USB: **PA11 / PA12**
- SPI2 TLE: **PB12–PB15**
- CAN: **PB8 / PB9**
- Flex: **PB5**; VVT PWM: **PB6 / PB7**
- INJ/IGN/ETB/enables: **PE0–PE15** (porto E completo no VGT6)

---

## Implicações para a placa de interface

1. **Socket / pin headers** da carrier: dual-row 2,54 mm, footprint interno ≈ **38,6 × 69,1 mm**
   + folga de manobra (recomendado ≥ 0,5 mm por lado no cutout se for janela).
2. **Furos M3** (Φ3,2 mm) nos 4 cantos — alinhar standoffs da caixa com a mesma grelha.
3. **USB-C** sai na base curta: isolar galvânico **nessa aresta**, não sob o MCU.
4. **Altura:** PDF de shape não dá z; medir na placa física (USB + headers + componentes altos).
5. Coreboard **grau consumidor** — preferir montagem em **cabine** (já decidido).

---

## Estado face ao plano

| Item README §5 | Estado |
|----------------|--------|
| Comprimento × largura | ✅ **69,10 × 38,62 mm** |
| Furos montagem | ✅ Φ **3,2**, offset **2,80**, pitch X **30,48** |
| Passo headers | ✅ **2,54 mm** (inferido + dual row) |
| Altura com USB / pinos | ⚠️ **ainda medir** na placa física |
| Numeração exacta pin 1 de cada header | ⚠️ Conferir silkscreen na peça (pág. 2–3 do PDF) |
