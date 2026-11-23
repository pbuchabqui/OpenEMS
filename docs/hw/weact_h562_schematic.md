# WeAct STM32H5xx VxTx CoreBoard V1.0 — esquemático (resumo OpenEMS)

> **Status:** vigente · validado 2026-11-17 (doc-check + conferência pontual contra o código)

> **Fonte:** `WeAct-STM32H5xxVxTxCoreBoard_V10 SchDoc.pdf` (Altium, 1 página).  
> Arquivo: `docs/hw/weact_h562_v10_schdoc.pdf`.  
> Forma mecânica: `weact_h562_coreboard.md`.

Não substitui o PDF. Extrai o que a **placa de interface** precisa: rails, headers dual-row
**P1 / P2**, USB, SWD, e presença de **GPIOE** (VGT6).

---

## Rails e alimentação (coreboard)

| Net no sch | Uso |
|------------|-----|
| **VDD33** | 3,3 V LDO onboard → VDD MCU |
| **VCC / 5V** | 5 V (USB / header) |
| **VB** | alimentação de backup do RTC (P2 pino 1): entra no `VBAT` do MCU por um BAT54C. **Não é entrada de tensão larga: não ligar 12 V aqui** |
| **GND** | comum |
| **VREF+** | pino MCU VREF+ exposto no header P1 |

A carrier OpenEMS **não deve alimentar sensores ratiométricos pelo 5 V ruidoso do USB**; gera o próprio 5 V de sensores.
O 3,3 V do MCU vem do LDO ME6231A33 (entrada 3,3–6 V) alimentado por `VCC`, que recebe o `VBUS` do USB por um Schottky
(D4, DSK34, sem retorno para o USB) e pelo header P4 (`VCC`). A carrier pode alimentar a placa por `VCC` (P4) com 5 V,
sem backfeed no USB pelo diodo; o interruptor S1 (`VBUS`–`VCC`) não deve ficar fechado com a carrier alimentando.

---

## Conectores laterais dual-row (P1 / P2)

Passo **2,54 mm**, ~**40 vias por lado** (2×20). Numeração do sch: pin ímpar/par por fila
(`PIP10x` = P1, `PIP20x` = P2).

### P1 (2×20, pino ímpar à esquerda / par à direita) — lido do SchDoc renderizado em 2026-11-22

| Pinos | Sinal | Pinos | Sinal |
|---:|---|---:|---|
| 1 / 2 | PC0 / PC1 | 21 / 22 | PE8 / PE9 |
| 3 / 4 | PC2 / PC3 | 23 / 24 | PE10 / PE11 |
| 5 / 6 | GND / VREF+ | 25 / 26 | PE12 / PE13 |
| 7 / 8 | PA0 / PA1 | 27 / 28 | PE14 / PE15 |
| 9 / 10 | PA2 / PA3 | 29 / 30 | PB10 / GND |
| 11 / 12 | PA4 / PA5 | 31 / 32 | PB12 / PB13 |
| 13 / 14 | PA6 / PA7 | 33 / 34 | PB14 / PB15 |
| 15 / 16 | PC4 / PC5 | 35 / 36 | PD8 / PD9 |
| 17 / 18 | PB0 / PB1 | 37 / 38 | PD10 / PD11 |
| 19 / 20 | PB2 / PE7 | 39 / 40 | PD12 / PD13 |

### P2 (2×20)

| Pinos | Sinal | Pinos | Sinal |
|---:|---|---:|---|
| 1 / 2 | VB / GND | 21 / 22 | PD3 / PD4 |
| 3 / 4 | PE6 / PC13 | 23 / 24 | PD1 / PD2 |
| 5 / 6 | PE4 / PE5 | 25 / 26 | PC12 / PD0 |
| 7 / 8 | PE2 / PE3 | 27 / 28 | PC10 / PC11 |
| 9 / 10 | PE0 / GND | 29 / 30 | PA12 / PA15 |
| 11 / 12 | PB8 / PB9 | 31 / 32 | PA10 / PA11 |
| 13 / 14 | PB6 / PB7 | 33 / 34 | PA8 / PA9 |
| 15 / 16 | PB4 / PB5 | 35 / 36 | PC8 / PC9 |
| 17 / 18 | PD7 / PB3 | 37 / 38 | PC6 / PC7 |
| 19 / 20 | PD5 / PD6 | 39 / 40 | PD14 / PD15 |

P3 (2×2): `VDD33` e `GND`. P4 (2×2): `VCC` e `GND`. P5 (2×4): `VDD33`, `SWDIO`, `SWCLK`, `GND`, `NRST`, `PA9` (TX), `PA10` (RX), `GND`.

**Pinos que não saem em P1/P2:** `PB11`, `PA13` (SWDIO), `PA14` (SWCLK) e `NRST`. SWD e reset só existem no P5 e nos botões. O pedido de A/C (`PB11`, `src/hal/ac_request.cpp`) não é acessível pela carrier.

**SD:** a WeAct tem microSD (U3) em `PC8`..`PC11` (D0..D3), `PC12` (CK) e `PD2` (CMD), com pull-ups de 47 kΩ, e o `PA8` ligado por 10 kΩ. Esses pinos também aparecem em P1/P2; a carrier não os usa.

## USB / SWD / botões

- **USB-C** na base: DP/DM → PA12/PA11 (via ESD típico no sch).  
- **SWD** header 6 vias: 3V3, SWDIO (PA13), SWCLK (PA14), GND, NRST, …  
- **NRST**, **BOOT0** botões; LED **PC13** / PWR.

Isolador USB da interface: cortar o laço **no cabo** ou no conector da carrier, não
assumir que o USB-C do WeAct fica acessível com a board encaixada na caixa.

---

## Checklist carrier ↔ WeAct

- [ ] Footprint dual-row 2,54 mm P1/P2 com pin 1 no canto correcto  
- [ ] Furos M3 Φ3,2 @ grelha `weact_h562_coreboard.md`  
- [ ] Todos os PE* de INJ/IGN/ETB/enables passam por header  
- [ ] PA0/PA1 CKP/CMP; PB8/9 CAN  
- [ ] VREF+ / VDDA conforme opção (a)  
