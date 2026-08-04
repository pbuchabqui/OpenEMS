# OpenEMS — Pinout (RGT6 / VGT6)

**Fonte de verdade do pinout.** Detalhe completo por pino dos dois packages
suportados. O `README.md` mantém apenas o resumo das diferenças e aponta para
aqui. Código de referência: `ecu_sched.cpp` (INJ/IGN BSRR), `hal/out_pins.h`
(mapas + `static_assert` por board), `etb_driver.cpp`, `timer.cpp`
(CKP/CMP/ETB PWM), `adc.cpp`.

O firmware selecciona o package no **build** (não em runtime):

```bash
make firmware BOARD=rgt6   # default — WeAct LQFP64
make firmware BOARD=vgt6   # LQFP100 GPIOE (INJ/IGN/ETB em PE*)
# bins: /tmp/openems-build/bin/openems-rgt6.bin | openems-vgt6.bin
```

## RGT6 vs VGT6 — diferença de pinout

| | **RGT6** (`BOARD=rgt6`, default) | **VGT6** (`BOARD=vgt6`) |
|--|----------------------------------|---------------------------|
| Package | LQFP64 | LQFP100 |
| Portas GPIO | A, B, C | A–E |
| WeAct típica | H562RGT6 | H562VGT6 |
| Flag C | `-DEMS_BOARD_RGT6` | `-DEMS_BOARD_VGT6` |
| Bin | `openems-rgt6.bin` | `openems-vgt6.bin` |

**Porquê divergir:** o RGT6 **não tem port E**. O mapa VGT6 concentrou INJ/IGN/ETB
DIR/PWM em **PE\*** (BSRR único, fiação limpa no LQFP100). No RGT6 esses pinos
**não existem** — INJ/IGN/ETB foram redistribuídos por A/B/C.

### Comparação lado a lado (o que muda)

| Função | **RGT6** (`BOARD=rgt6`) | **VGT6** (`BOARD=vgt6`) | Notas |
|--------|--------------------------|--------------------------|--------|
| CKP | **PA0** TIM5_CH1 AF2 | PA0 TIM5_CH1 AF2 | Igual |
| CMP | **PA1** TIM5_CH2 AF2 | PA1 TIM5_CH2 AF2 | Igual |
| UART1 TX/RX | **PA9 / PA10** | PA9 / PA10 | Igual |
| USB DM/DP | **PA11 / PA12** | PA11 / PA12 | Igual |
| CAN | **PB8 / PB9** | PB8 / PB9 | Igual |
| LED | **PB2** | PB2 | Igual |
| **INJ1** | **PA15** | **PE0** | Remap RGT6 |
| **INJ2** | **PB3** | **PE2** | Remap RGT6 |
| **INJ3** | **PC10** | **PE4** | Remap; WeAct: PB10/11 **não** no header |
| **INJ4** | **PC11** | **PE6** | Remap RGT6 |
| **IGN1** | **PC6** | **PE9** | Remap RGT6 |
| **IGN2** | **PC7** | **PE11** | Remap RGT6 |
| **IGN3** | **PC8** | **PE13** | Remap; ⚠️ vs SDMMC PC8 |
| **IGN4** | **PC9** | **PE15** | Remap RGT6 |
| **ETB PWM** | **PA6** TIM3_CH1 AF2 | **PE5** TIM15_CH1 AF4 | Timer e pino diferentes |
| **ETB DIR open** | **PA8** | **PE7** | Remap RGT6 |
| **ETB DIR close** | **PB4** | **PE8** | Remap RGT6 |
| ETB_TPS1 / TPS2 | PA2 / PC5 | PA2 / PC5 | Igual (ADC) |
| EWG PWM | PB10 TIM2_CH3 | PB10 TIM2_CH3 | PB10 livre de INJ no mapa actual (WeAct: pino pode não sair no header) |
| EWG DIR | PA7 (+ PD3 no driver, **sem GPIOD no RGT6**) | PA7 / pino dedicado VGT6 | PD3 inválido no RGT6 |
| Flex / VVT | PB5 / PB6–7 | PB5 / PB6–7 | Igual |
| ADC sensores (MAP…OIL) | ver tabela ADC | mesma atribuição ADC actual | Igual no tree actual |

### Drive eléctrico

| | RGT6 | VGT6 |
|--|------|------|
| INJ/IGN | GPIO **BSRR** multi-porto (A/B/C) | GPIO **BSRR** só **GPIOE** |
| ETB PWM | TIM3 @ PA6 | TIM15 @ PE5 |
| TIM OC (TIM2/TIM8) | **Não usado** | **Não usado** (também BSRR) |

### Conflitos / limitações WeAct RGT6

| Pino | Nota |
|------|------|
| **PB10 / PB11** | No coreboard WeAct **não saem nos headers** — **não usar para INJ** (por isso INJ3/4 = PC10/11) |
| **PC8 / PC9** | IGN3/4; se microSD (SDMMC) activo, colide com D0/D1 |
| **PC10 / PC11** | INJ3/4; colidem com SDMMC D2/D3 se cartão |
| **PD3** | EWG DIR IN2 no driver — **inexistente no RGT6** |

### Fiação ao mudar de placa

- **VGT6 → RGT6:** religar todos os PE\* de INJ/IGN/ETB para a coluna RGT6;
  CKP/CMP/UART/USB/CAN/ADC iguais.
- **RGT6 ↔ VGT6:** gravar o binário correcto (`BOARD=…`). Flash cruzado = pinos
  errados.

## Crank / cam / comms (comum)

| Funcao | Peripheral | Pino |
|---|---|---|
| CKP | TIM5_CH1 AF2 | **PA0** (pull-down) |
| CMP | TIM5_CH2 AF2 | **PA1** (pull-down) |
| UART TX | USART1 AF7 | **PA9** |
| UART RX | USART1 AF7 | **PA10** (pull-up) |
| USB DM/DP | USB AF10 | **PA11 / PA12** |
| CAN RX/TX | FDCAN1 AF9 | **PB8 / PB9** |
| LED heartbeat | GPIO | **PB2** |

## Injecção / ignição — por board

**RGT6** (`BOARD=rgt6`):

| Funcao | Pino | Notas |
|---|---|---|
| INJ1 | **PA15** | low-side → TLE8888 (header WeAct) |
| INJ2 | **PB3** | header WeAct |
| INJ3 | **PC10** | header WeAct (não PB10) |
| INJ4 | **PC11** | header WeAct (não PB11) |
| IGN1 | **PC6** | push-pull → TLE8888 |
| IGN2 | **PC7** | |
| IGN3 | **PC8** | ⚠️ vs SDMMC D0 |
| IGN4 | **PC9** | ⚠️ vs SDMMC D1 |

**VGT6** (`BOARD=vgt6`):

| Funcao | Pino | Notas |
|---|---|---|
| INJ1–4 | **PE0 / PE2 / PE4 / PE6** | BSRR GPIOE |
| IGN1–4 | **PE9 / PE11 / PE13 / PE15** | BSRR GPIOE |
| INJEN / IGNEN | **PE14 / PE3** | enables TLE8888 (não PE1 — sem bond no LQFP100) |

Ordem de canais BSRR = `ECU_CH_*`:
`[INJ3, INJ4, INJ1, INJ2, IGN4, IGN3, IGN2, IGN1]`. Actuadores active-high;
safe = LOW. Boot safe: `ecu_sched_outputs_safe_early()` → `out_pins_hw_init()`
(RGT6: PA15 JTDI pull-up; VGT6: PE\* LOW no arranque).

## ETB / EWG / AUX — por board

**RGT6:**

| Funcao | Peripheral | Pino |
|---|---|---|
| ETB PWM | TIM3_CH1 AF2 | **PA6** |
| ETB DIR open (IN1) | GPIO | **PA8** |
| ETB DIR close (IN2) | GPIO | **PB4** |
| ETB_TPS1 / TPS2 | ADC1 | **PA2 / PC5** |
| EWG PWM | TIM2_CH3 | **PB10** (WeAct: pode não sair no header) |
| EWG DIR | GPIO | PA7 / PD3 (PD3 só c/ GPIOD) |
| Flex / VVT | EXTI / TIM4 | **PB5** / **PB6–PB7** |

**VGT6:**

| Funcao | Peripheral | Pino |
|---|---|---|
| ETB PWM | TIM15_CH1 AF4 | **PE5** |
| ETB DIR open / close | GPIO | **PE7 / PE8** |
| ETB_TPS / Flex / VVT / EWG PWM | | iguais à coluna RGT6 (ADC/PB5–7/PB10) |

> Os limites TPS1/TPS2 do ETB são recalibrados a cada power-on por varredura dos
> batentes (`etb_autocal`); a última calibração boa é persistida (EtbCalRecord,
> setor adaptativo) como fallback. Ver README §"ETB".

## Relés — bomba e ventoinha (**difere por package**)

| Funcao | **RGT6** | **VGT6** | Constante |
|---|---|---|---|
| Ventoinha | **PB12** (BSRR GPIOB) | **PE12** (BSRR GPIOE) | `kFanPin` |
| Bomba de combustível | **PB13** (BSRR GPIOB) | **PE10** (BSRR GPIOE) | `kPumpPin` |

Definidos em `src/engine/auxiliaries.cpp` sob `#if EMS_BOARD_IS_VGT6`.
Lógica: `run_pump_control()` faz prime de 2 s no key-on, mantém ligada com
RPM > 0 e **corta 2 s após RPM = 0** (`kPumpOffDelayMs = 2000`); `run_fan_control()`
usa histerese CLT 95 °C liga / 90 °C desliga. Ambos suspensos durante
`output_test_active()`.

**Porquê o remap no VGT6:** `PB12`/`PB13` são o **CSN** e o **SCK** do SPI2 do
**TLE8888**. Como `auxiliaries_init()` corre *depois* de `tle8888_init()`
(`main_stm32.cpp:638` vs `:500`), reescrevia o `GPIOB_MODER` e **matava o
SPI2_SCK no arranque** — o CI nunca era clockado e o seu watchdog nunca
alimentado, o que com o TLE8888 montado desliga injecção e ignição.

No VGT6 os relés passaram para `GPIOE` (`PE10`/`PE12`). Os enables do TLE8888
são `INJEN=PE14` e `IGNEN=PE3`. ⚠️ **`PE1` não existe no STM32H562VGTx LQFP100**
(pad 98 = VCAP) — ver `stm32h562_ref.md` §3.1. Restantes PE: INJ PE0/2/4/6,
IGN PE9/11/13/15, ETB PE5/7/8. Partilhar o porto com INJ/IGN é seguro: o
acionamento é por **BSRR**, set/reset atómico por bit, sem read-modify-write.

> ⚠️ **O RGT6 mantém o conflito** — `PB12`/`PB13` continuam a colidir com o SPI2
> lá. O RGT6 não é o alvo da placa de interface; se vier a ser, tem de resolver
> isto (não tem GPIOE). O mesmo SPI2 é candidato a um futuro **TPIC8101** (knock).

## ADC (sensores) — comum nos dois packages (tree actual)

| Funcao | ADC | Pino / INP |
|---|---|---|
| MAP | ADC1 | **PA3** / INP15 |
| TPS | ADC1 | **PA4** / INP18 |
| KNOCK | ADC1 | **PA5** / INP19 |
| ETB_TPS1 | ADC1 | **PA2** / INP14 |
| ETB_TPS2 | ADC1 | **PC5** / INP8 |
| APP1 | ADC1 | **PC0** / INP10 |
| APP2 | ADC1 | **PC2** / INP12 |
| CLT | ADC2 | **PB0** / INP9 |
| IAT | ADC2 | **PB1** / INP5 |
| FUEL_PRESS | ADC2 | **PC4** / INP4 |
| OIL_PRESS | ADC2 | **PC1** / INP11 |
| VBATT | ADC2 | **PC3** / INP13 |

**Nota:** WBO2 lambda exclusivamente via CAN (FDCAN1). Sem ADC O2.

**Nota (VBATT em PC3):** o canal era `EWG_POS`. Com o EWG diferido na placa de
interface v1, `PC3` passou a VBATT com divisor externo 0–18 V → 0–3,3 V. Antes, VBATT
partilhava o pino do `ETB_TPS2` e era **fixada em 12000 mV** com o chicote do ETB
ligado — o que subestimava o dead-time do injetor e encurtava o dwell num cranking
real (a bateria cai a 9–10 V). A realimentação de posição do EWG ficou sem canal:
`ewg_driver_read_position_raw()` devolve 0 até o EWG voltar na v2.

**Histórico de pinout (2026-07):**

- OIL: PC5 → **PC1** (PC5 fica ETB_TPS2).
- Dual-board: `-DEMS_BOARD_RGT6` / `-DEMS_BOARD_VGT6` (`src/hal/board_pinout.h`).
- RGT6 WeAct: INJ3/4 em **PC10/PC11** (PB10/11 não expostos nos headers).
- Boot safe: `ecu_sched_outputs_safe_early()` — PA15 JTDI pull-up e PE\* LOW no arranque.

## Ver também

- **`docs/hw/README.md`** — **ponto de entrada** (autoridade, decisões, em aberto).
- **`docs/hw/interface_board_v1.md`** — **plano da placa de interface v1**
  (TLE8888-**2QK** como estágio de potência, INJ/IGN por direct drive, alocação de
  pinos, orçamento de erro de timing, sequência de verificação).
- **`docs/wiring_diagram.md`** — ⚠️ **DESACTUALIZADO em vários eixos**: mapa de
  pinos ASCII legado (TIM2/TIM8 OC), e os números do TLE8888 ali ("INJ OC 10 A,
  IGN push-pull 6 A") **estão errados** — o datasheet Rev 1.2 dá injectores
  2,2 A e ignição como driver de gate de 20 mA. Usar apenas como referência de
  topologia eléctrica (terra, conector), e mesmo assim com desconfiança.
- **`docs/hw/vr_input_conditioning.md`** — condicionamento VR. ⚠️ Recomenda o
  MAX9926; no plano v1 o CKP passou a usar a **interface VR integrada do
  TLE8888** (zero-crossing + armamento por pico, clamp e diagnóstico internos).
