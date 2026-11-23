# Placa de interface VGT6 — orçamento de corrente

> **Status:** vigente · validado 2026-11-22 (premissas de carga **não medidas**; confronta `interface_vgt6.md` e `interface_vgt6_nets.md`)

Dimensiona fusíveis, vias de conector e cobre da carrier descritos em `interface_vgt6.md`. **Todo valor de carga abaixo é premissa de projeto, não medição:** nenhum injetor, bobina, corpo de borboleta ou sensor foi especificado ou medido. Substitua pelas fichas dos componentes reais antes de fabricar.

## 1. Premissas de carga

| Carga | Premissa | Como saiu |
|---|---|---|
| Injetor (×4) | 12 Ω, 1,2 A a 14,4 V; 1,5 A a 18 V | alta impedância, saturado |
| Bobina (×4) | 8 A de pico no primário, dwell 3–4 ms | smart coil típica; sem sobreposição relevante entre bobinas até 8500 rpm |
| Motor do ETB | 3,6 A de pico (limite da ponte), 1–2 A típico | limite de corrente do DRV8871 |
| Relés bomba/ventoinha | 150 mA de bobina cada (contatos fora da carrier) | contatos ligam direto ao fusível do veículo, nunca pela carrier |
| VVT (×2) | 1,5 A de pico cada, PWM | solenoide típico |
| WBO2 | 3 A de pico no aquecimento, ~1 A depois | sensor de banda larga típico |
| Lógica 5 V | ver tabela 2 | |
| 5 V de sensores | ≈110 mA | tabela 3 |

## 2. Rail de 5 V lógico (buck AP63205)

| Consumidor | Típico | Pico |
|---|---|---|
| WeAct (MCU + LDO, pino 5 V) | 150 mA | 200 mA |
| TJA1051T/3 | 10 mA | 70 mA (dominante) |
| 74AHCT125 ×3 e 74AHCT08 | 2 mA | 5 mA |
| Ignitores (4 entradas a 5 V) | 5 mA | 20 mA |
| DRV8871 (lógica) | 5 mA | 10 mA |
| MAX9924 | 5 mA | 5 mA |
| LEDs de status | 10 mA | 15 mA |
| 5 V de sensores (via TPS2553) | 110 mA | 300 mA (limite do TPS2553) |
| **Total** | **≈ 0,3 A** | **≈ 0,63 A** |

O buck de 2 A tem folga de mais de 3×. Entrada: ≈ 0,63 A × 5 V / 0,85 ≈ 3,7 W, ou ≈ 0,3 A a 12 V.

Ajuste do limitador de 5 V de sensores: 300–400 mA (acima dos 110 mA típicos, abaixo do que queima trilha de sensor).

## 3. 5 V de sensores (≈110 mA)

MAP 10 mA, TPS 10 mA, APP ×2 10 mA, ETB TPS ×2 10 mA, pressão de combustível 10 mA, pressão de óleo 10 mA, flex 10 mA, CKP Hall 15 mA, CMP Hall 15 mA, pull-ups de Hall (≈1 mA × 3) 3 mA, MAX9924 5 mA. Soma ≈ 108 mA.

## 4. Rail de 3,3 V

| Consumidor | Típico | Pico |
|---|---|---|
| MCU (250 MHz) | 150 mA | 200 mA |
| Schmitts, pull-ups de NTC (2 × 1,3 mA) | 3 mA | 4 mA |
| **Total** | **≈ 0,15 A** | **≈ 0,20 A** |

O slot SD é da WeAct e sai do orçamento da carrier; o consumo dele vem do LDO da própria placa.

## 5. Bancos de potência (VBAT)

| Banco | Médio | Pico | Fusível proposto | Observação |
|---|---|---|---|---|
| INJ (`VBAT_INJ`) | 2 A | 4,8 A a 14,4 V; 6 A a 18 V (4 abertos juntos, pior caso) | **7,5 A** | antes eu havia escrito 5 A: não cobria 4 injetores simultâneos |
| IGN (`VBAT_IGN`) | ≈ 3,6 A a 6000 rpm (8 A × 3 ms / 20 ms × 4) | 8 A (uma bobina) | **10 A** | capacitor de reservatório junto aos conectores |
| ETB (`VBAT_ETB`) | 1–2 A | 3,6 A | **5 A** | limite de corrente da ponte em 3,6 A |
| AUX (`VBAT_AUX`) | 1 A | 6,3 A (0,3 relés + 3 VVT + 3 WBO2) | **10 A** | contatos de bomba/ventoinha ficam fora |
| Lógica (via buck) | 0,3 A | 0,7 A | no fusível geral | |

Pior caso instantâneo somado: 4,8 + 8 + 3,6 + 6,3 + 0,7 ≈ **23 A**; uso real fica abaixo de 12 A porque os bancos não picam juntos.

### Fusível geral e entrada

- **Duas entradas de alimentação:** `J1A` (INJ + IGN, até ~13 A de pico) e `J1B` (ETB + AUX + lógica, até ~11 A de pico), cada uma com **fusível de 15 A** e proteção de inversão próprias. Um conector único de 5,08 mm tem corrente nominal em torno de 12 A (típico da família; conferir a ficha) e não carregaria 23 A.
- Alternativa: uma entrada de 25 A com conector de maior corrente e cobre largo. Mais difícil de rotear; não recomendado.
- Proteção de inversão: P-MOSFET com margem ≥ 2× na corrente (≥ 30 A) em cada entrada.

## 6. Conectores — corrente por via

| Conector | Via | Corrente esperada | Vias/fio |
|---|---|---|---|
| J2 | `VBAT_INJ` | 4,8 A (6 A a 18 V) | 1 via de 12 A, fio ≥ 1,0 mm² |
| J2 | `INJn_CON` | 1,2–1,5 A | 1 via, fio ≥ 0,5 mm² |
| J3 | `VBAT_IGN` e `PGND` | 8 A de pico | **2 vias em paralelo** cada |
| J3 | `IGNn_CON` | 5 mA | 1 via |
| J4 | `ETB_M+` / `ETB_M-` | 3,6 A | 1 via cada, fio ≥ 0,75 mm² |
| J7 | `VBAT_AUX`, `PGND` | 6,3 A | 1 via cada (ou 2 se a ficha do conector não garantir) |
| J7 | `PUMP_DRV`, `FAN_DRV` | 150 mA | 1 via |
| J7 | `VVT_*` | 1,5 A | 1 via |
| J8 | `VBAT_AUX` (WBO2), `PGND` | 3 A | 1 via |
| J6, J4, J5 | `+5V_SENS` | 110 mA total | 2 vias por conector, 1 por sensor |
| J1A / J1B | entrada | 13 A / 11 A de pico | 1 via de 12 A é o limite; usar 2 vias em paralelo |

## 7. Cobre e trilhas (IPC-2221, estimativa)

Cobre de 2 oz externo, elevação de 20 °C:

| Corrente | Largura mínima |
|---|---|
| 3 A | ≈ 0,8 mm |
| 6 A | ≈ 2 mm |
| 8 A | ≈ 3 mm |
| 13 A | ≈ 6 mm (polígono) |

Para os 13 A de `J1A`, use polígono em duas camadas com vias de costura. Regras de layout: injetores, bobinas e ETB voltam por `PGND`, sem passar pelo plano de sensores; o ponto único `PGND`–`SGND` fica junto à entrada.

## 8. Capacitores de reservatório

- `VBAT_IGN`: ≥ 470 µF/35 V low-ESR junto a J3, mais 10 µF cerâmico. Sustenta o pulso de 8 A.
- `VBAT_ETB`: ≥ 220 µF/35 V junto à ponte (exigência da ficha do DRV8871).
- `VBAT_INJ`: 220 µF/35 V.

## 9. O que muda nos outros documentos

- `interface_vgt6.md`: fusíveis por banco passam a INJ 7,5 A, IGN 10 A, ETB 5 A, AUX 10 A, e o fusível geral vira duas entradas de 15 A.
- `interface_vgt6_nets.md`: o conector J1 vira `J1A` e `J1B`.

## 10. Em aberto (decide o resultado)

- Corrente real de bobina, injetor, ETB, VVT e WBO2 (fichas dos componentes escolhidos).
- Capacidade do LDO de 3,3 V da WeAct com o SD em escrita (o consumo do SD não é da carrier).
- Corrente nominal real do conector escolhido.
