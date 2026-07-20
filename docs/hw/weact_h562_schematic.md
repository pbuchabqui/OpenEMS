# WeAct STM32H5xx VxTx CoreBoard V1.0 — esquemático (resumo OpenEMS)

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
| **VB** | Vin wide (header P2 pin 1) — entrada do regulador da board |
| **GND** | comum |
| **VREF+** | pino MCU VREF+ exposto no header P1 |

A carrier OpenEMS **não deve alimentar sensores ratiométricos pelo 5 V ruidoso do USB**;
usa trackers T5V1/T5V2 do TLE8888. O coreboard pode ser alimentado por **+3V3** da carrier
ou por VB/5V conforme escolha de montagem — documentar no esquemático da interface.

---

## Conectores laterais dual-row (P1 / P2)

Passo **2,54 mm**, ~**40 vias por lado** (2×20). Numeração do sch: pin ímpar/par por fila
(`PIP10x` = P1, `PIP20x` = P2).

### P1 (COP1) — sinais relevantes OpenEMS

| Pins P1 | Sinal | OpenEMS |
|--------:|-------|---------|
| 1–2 | PC0 / PC1 | APP1 / OIL (ADC) |
| 3–4 | PC2 / PC3 | APP2 / VBATT ADC |
| 5–6 | GND / **VREF+** | AGND / VREF (opção (a) liga a VDDA) |
| 7–8 | **PA0 / PA1** | **CKP / CMP** |
| 9–10 | PA2 / PA3 | ETB_TPS1 / MAP |
| 11–12 | PA4 / PA5 | TPS / KNOCK |
| 13–14 | PA6 / PA7 | (RGT6 ETB PWM etc.; VGT6 ETB em PE) |
| 15–16 | PC4 / PC5 | FUEL_PRESS / ETB_TPS2 |
| 17–18 | PB0 / PB1 | CLT / IAT |
| 19–20 | PB2 / PE7 | LED / **ETB DIR open** |
| 21–22 | **PE8 / PE9** | **ETB DIR close / IGN1** |
| 23–24 | **PE10 / PE11** | **bomba / IGN2** |
| 25–26 | **PE12 / PE13** | **ventoinha / IGN3** |
| 29–30 | **PB12 / PB13** | **SPI2 CS / SCK → TLE** |
| 31–32 | **PB14 / PB15** | **SPI2 MISO / MOSI** |
| 33–38 | PD8–PD13 | livres / SDMMC potencial |
| … | PD… | ver PDF |

### P2 (COP2) — sinais relevantes OpenEMS

| Pins P2 | Sinal | OpenEMS |
|--------:|-------|---------|
| 1–2 | **VB** / PE6 | Vin board / **INJ4** |
| 3–4 | **PE4 / PE5** | **INJ3 / ETB PWM** |
| 5–6 | **PE2 / PE3** | **INJ2 / IGNEN** |
| 7–8 | **PE0** / … | **INJ1** / … |
| 9–10 | **PB8 / PB9** | **CAN RX / TX** |
| 11–12 | **PB6 / PB7** | **VVT TIM4** |
| 13–14 | PB4 / … | |
| 15–16 | PD7 / PB3 | |
| 27–28 | **PA10 / PA11** | UART RX / **USB DM** |
| 29–30 | **PA8 / PA9** | / **USB? / UART TX** — ver PDF (USB em PA11/12) |
| 31–36 | PC8–PC9, PC6–PC7, PD14–15 | SDMMC / livres |

⚠️ A tabela acima foi **derivada do SchDoc export** (nomes PIP + nets); cruzar **pin 1**
com silkscreen da peça física antes de mandar fabricar a carrier. O PDF Board Shape +
SchDoc juntos fecham forma e netlist de headers.

---

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
- [ ] PB12–15 só TLE SPI (sem bomba/fan)  
- [ ] PA0/PA1 CKP/CMP; PB8/9 CAN  
- [ ] VREF+ / VDDA conforme opção (a)  
