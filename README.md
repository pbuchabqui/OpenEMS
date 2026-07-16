# OpenEMS

Fonte unica da verdade do projeto OpenEMS.

## Objetivo

OpenEMS e uma ECU para motores a combustao interna, atualmente focada no STM32H562RGT6. O codigo deve priorizar baixa latencia, baixo jitter e previsibilidade temporal para controle de injecao, ignicao, sensores e comunicacao de calibracao.

Este documento substitui os documentos historicos de plano, status e revisao. Qualquer decisao tecnica duravel deve ser atualizada aqui, nao em novos arquivos Markdown paralelos.

## Plataforma Alvo

- MCU: STM32H562RGT6, Cortex-M33.
- Revisao de silicio considerada: X.
- Linguagem: C++17 embarcado.
- Restricoes: sem STL em caminho critico, sem alocacao dinamica em runtime, sem excecoes.
- Build atual: `make firmware` gera `.elf`, `.hex` e `.bin` em `/tmp/openems-build`.

## Regras De Arquitetura

O fluxo de dependencias e descendente:

```text
APP     protocolos, calibracao, diagnostico
ENGINE  calculos de motor, agendamento angular, tabelas
DRV     decodificacao CKP/CMP, sensores, servicos independentes de MCU
HAL     registradores e perifericos STM32H562
HW      timers, ADC, GPIO, USB, CAN
```

Regras praticas:

- `APP` nao deve entrar no caminho critico de captura/agendamento.
- `ENGINE` decide tempo, angulo, pulso, dwell e estrategia.
- `DRV` transforma sinais fisicos em eventos de motor confiaveis.
- `HAL` contem apenas detalhe de periferico.
- O codigo ativo deve usar somente perifericos e nomes STM32H562 reais, sem aliases de compatibilidade.
- Novos documentos Markdown paralelos nao devem ser criados; atualize este `README.md`.

## Pipeline De Controle Do Motor

### 1. Captura CKP/CMP

- Modulo principal: `src/drv/ckp.cpp`.
- Backend STM32: `src/hal/stm32h562/timer.cpp`.
- Timer usado: TIM5.
- Canais:
  - CKP: TIM5 CH1 em PA0.
  - CMP: TIM5 CH2 em PA1.

A captura mede bordas do virabrequim e comando, detecta dente faltante e alimenta a maquina de sincronismo.

Estados principais:

- `WAIT_GAP`: aguardando padrao confiavel de gap.
- `HALF_SYNC`: fase angular parcial suficiente para estrategias de partida rapida.
- `FULL_SYNC`: fase e ciclo conhecidos para injecao sequencial e ignicao correta.
- `LOSS_OF_SYNC`: falha de coerencia, ruido, timeout ou perda de padrao.

### 2. Quick Crank E Pre-Sync

- Modulos principais: `src/engine/quick_crank.cpp`, `src/engine/ecu_sched.cpp`.
- Objetivo: permitir primeira combustao antes do full sync quando tecnicamente seguro.

Estrategias permitidas antes de full sync:

- Injecao simultanea durante cranking.
- Injecao semi-sequencial quando a fase parcial permitir.
- Ignicao wasted spark antes da confirmacao completa de fase.
- Uso de posicao/fase persistida apenas quando valida, recente e coerente com o novo padrao CKP/CMP.

A estrategia pre-sync deve degradar para modo conservador se houver duvida de fase. Partida rapida nao pode vencer seguranca de sincronismo.

### 3. Calculos De Combustivel E Ignicao

- Combustivel: `src/engine/fuel_calc.cpp`.
- Ignicao: `src/engine/ign_calc.cpp`.
- Tabelas: `src/engine/table3d.cpp`.
- Sensores: `src/drv/sensors.cpp`.

Fluxo esperado:

```text
ADC/TIM6 -> sensors -> fuel_calc/ign_calc -> ecu_sched -> TIM2/TIM8 -> atuadores
```

Os calculos usam RPM, MAP, TPS, CLT, IAT, lambda e calibracoes. Tabelas devem operar com interpolacao deterministica e sem custo imprevisivel no caminho critico.

Aceleracoes repentinas apos o calculo principal devem ser tratadas por atualizacao near-time quando disponivel, especialmente para largura de pulso, SOI, dwell e avanco. O objetivo e reduzir erro entre o ultimo calculo e o evento fisico.

### 4. Scheduling De Injecao E Ignicao

- Modulo principal: `src/engine/ecu_sched.cpp`.
- Injecao: TIM2 output compare.
- Ignicao: TIM8 output compare.
- Base temporal: 10 MHz, 100 ns por tick.

Premissa de projeto:

- O caminho critico deve ser agendado por hardware output compare.
- ISR deve armar eventos e atualizar estado, nao bit-bangar saidas criticas.
- Janelas longas devem lidar com limite de contador por rearmamento/near-time, sem perder precisao perto do evento.
- Eventos vencidos devem ser tratados explicitamente, nunca silenciosamente aceitos.

Eventos de injecao ficam codificados no scheduler do motor, nao em um driver legado. O scheduler recebe dentes/angulo do CKP, calcula quando cada canal deve abrir/fechar e programa TIM2.

Eventos de ignicao usam o mesmo conceito, mas programam TIM8 para dwell e centelha.

### 4a. Detecção De Knock (Detonação)

- Sensor piezoelétrico knock conectado em PA5/ADC1_IN6.
- Hardware: filtro passa-banda externo → PA5 → ADC1.
- Detecção: software via threshold ADC (STM32H562 não possui periférico COMP).
- Amostragem: `knock_adc_update(raw)` chamado de `sample_fast_channels()` a cada dente CKP durante janela ativa.
- Threshold ADC: padrão 2048 (12-bit), range [256, 4000].
  - Adaptativo: -64 por evento de knock, +32 após 100 ciclos limpos.
- Janela de knock: aberta/fechada por `knock_window_cycle_end()` no evento `ECU_ACT_DWELL_START` (modo sequencial).
- Retardo: +2,0° por evento de knock, máximo 10,0°.
- Recuperação: -0,1° por ciclo limpo após 10 ciclos consecutivos limpos.
- Persistência NVM: retardo em slot knock, threshold armazenado como int8_t (/32).

### 5. Atuadores e pinout congelado (firmware)

**Fonte de verdade:** este README + `ecu_sched.cpp` (INJ/IGN BSRR), `etb_driver.cpp`,
`timer.cpp` (CKP/CMP/ETB PWM), `adc.cpp`.  
**Firmware:** selecciona o package no **build** (não em runtime):

```bash
make firmware BOARD=rgt6   # default — WeAct LQFP64
make firmware BOARD=vgt6   # LQFP100 GPIOE (INJ/IGN/ETB em PE*)
# bins: /tmp/openems-build/bin/openems-rgt6.bin | openems-vgt6.bin
```

#### 5.0 RGT6 vs VGT6 — diferença de pinout

| | **RGT6** (`BOARD=rgt6`, default) | **VGT6** (`BOARD=vgt6`) |
|--|----------------------------------|---------------------------|
| Package | LQFP64 | LQFP100 |
| Portas GPIO | A, B, C | A–E |
| WeAct típica | H562RGT6 | H562VGT6 |
| Flag C | `-DEMS_BOARD_RGT6` | `-DEMS_BOARD_VGT6` |
| Bin | `openems-rgt6.bin` | `openems-vgt6.bin` |

**Porquê divergir:** o RGT6 **não tem port E**. O mapa VGT6 concentrou INJ/IGN/ETB DIR/PWM em **PE\*** (BSRR único, fiação limpa no LQFP100). No RGT6 esses pinos **não existem** — INJ/IGN/ETB foram redistribuídos por A/B/C.

##### Comparação lado a lado (o que muda)

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
| ADC sensores (MAP…OIL) | ver tabela §5.4 | mesma atribuição ADC actual | Igual no tree actual |

##### Drive eléctrico

| | RGT6 | VGT6 |
|--|------|------|
| INJ/IGN | GPIO **BSRR** multi-porto (A/B/C) | GPIO **BSRR** só **GPIOE** |
| ETB PWM | TIM3 @ PA6 | TIM15 @ PE5 |
| TIM OC (TIM2/TIM8) | **Não usado** | **Não usado** (também BSRR) |

##### Conflitos / limitações WeAct RGT6

| Pino | Nota |
|------|------|
| **PB10 / PB11** | No coreboard WeAct **não saem nos headers** — **não usar para INJ** (por isso INJ3/4 = PC10/11) |
| **PC8 / PC9** | IGN3/4; se microSD (SDMMC) activo, colide com D0/D1 |
| **PC10 / PC11** | INJ3/4; colidem com SDMMC D2/D3 se cartão |
| **PD3** | EWG DIR IN2 no driver — **inexistente no RGT6** |

##### Fiação ao mudar de placa

- **VGT6 → RGT6:** religar todos os PE\* de INJ/IGN/ETB para a coluna RGT6; CKP/CMP/UART/USB/CAN/ADC iguais.
- **RGT6 ↔ VGT6:** gravar o binário correcto (`BOARD=…`). Flash cruzado = pinos errados.

---

#### 5.1 Crank / cam / comms (comum)

| Funcao | Peripheral | Pino |
|---|---|---|
| CKP | TIM5_CH1 AF2 | **PA0** (pull-down) |
| CMP | TIM5_CH2 AF2 | **PA1** (pull-down) |
| UART TX | USART1 AF7 | **PA9** |
| UART RX | USART1 AF7 | **PA10** (pull-up) |
| USB DM/DP | USB AF10 | **PA11 / PA12** |
| CAN RX/TX | FDCAN1 AF9 | **PB8 / PB9** |
| LED heartbeat | GPIO | **PB2** |

#### 5.2 Injecção / ignição — por board

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

#### 5.3 ETB / EWG / AUX — por board

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

#### 5.4 ADC (sensores) — comum nos dois packages (tree actual)

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
| EWG_POS | ADC2 | **PC3** / INP13 |

**Nota:** WBO2 lambda exclusivamente via CAN (FDCAN1). Sem ADC O2.

**Histórico de pinout (2026-07):**
- OIL: PC5 → **PC1** (PC5 fica ETB_TPS2).
- Dual-board: `-DEMS_BOARD_RGT6` / `-DEMS_BOARD_VGT6` (`src/hal/board_pinout.h`).
- RGT6 WeAct: INJ3/4 em **PC10/PC11** (PB10/11 não expostos nos headers).
- Boot safe: `ecu_sched_outputs_safe_early()` — PA15 JTDI pull-up e PE* LOW no arranque.

## ADC E Sensores

- Modulos: `src/hal/adc.cpp`, `src/hal/stm32h562/adc.cpp`, `src/drv/sensors.cpp`.
- ADC primario/secundario representam ADC1/ADC2 no STM32.
- TIM6 deve ser o gatilho periodico de amostragem.
- Validacao de sensores deve bloquear valores absurdos e preservar estado de falha para diagnostico.

## Comunicacao

- Protocolo em `src/app/ui_protocol.cpp`, dual-mode com auto-detect por frame:
  - **Legacy** (ASCII cru, sem envelope): comandos `Q/S/F/C/A/O/r/w/x/b/d/B/G/P/V/D`,
    usados por `tools/openems_dash/protocol.py`, `tools/lib/ecu_link.py`, HIL e diag.
  - **TunerStudio** (envelope `msEnvelope_1.0`): `[size u16 BE][cmd+dados][CRC32 BE]`;
    detectado quando o primeiro byte em IDLE e < 0x20 (byte alto do size). Respostas
    levam response code (0x00 OK, 0x82 CRC, 0x83 cmd, 0x84 range, 0x85 busy) + CRC32.
    Projeto TunerStudio: `tools/ts/openems.ini` (assinatura `OpenEMS_v1.2`).
    No envelope, `w` e chunk-write **RAM-only**; burn e explicito via `b`.
- Burn de Flash via protocolo (`w` legacy sem RAM-only, `b` em ambos os modos) e
  bloqueado quando `rpm_x10 > kFlashWriteSafeRpmX10` (errata ES0565: primeira
  escrita/erase pode congelar fetch ~120 us). Legacy devolve NACK; envelope 0x85.
- Pagina 11 (64 B): eixos das tabelas 16x16 (16xu16 RPM + 16xu16 load bar x100),
  editaveis com validacao de monotonicidade estrita; NVM setor 10 (slot 9).
- MVP de bancada: UART 115200 8N1 em `USART1` (`PA9=TX`, `PA10=RX`). O shuttle
  UART<->protocolo roda no slot de 2 ms (`comms_pump()`), nao-bloqueante, com
  TX FIFO (FIFOEN) — throughput efetivo ~4 kB/s, suficiente para realtime TS a
  10-20 Hz. USB CDC espelha o TX e mantem RX no slot de 20 ms.
- USB CDC: pos-MVP; o backend atual permanece stub/no-op e nao deve ser tratado como transporte validado.
- CAN/FDCAN: diagnostico e integracao com sensores externos.

Comunicacao nao deve bloquear decode, sync, scheduling ou atuadores.

## Errata STM32H562 ES0565

Referencia local revisada: `es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf`, Rev 8, January 2026.

Impactos diretos no projeto:

- Revisao operacional considerada: X. Antes de teste real, confirmar a revisao fisica por marcacao do chip e `REV_ID` em `DBGMCU_IDCODE`, e registrar essa evidencia antes de liberar ensaio com atuadores.
- PA1 tem errata de histerese na revisao A: a histerese de entrada de PA1 so e habilitada quando PA0 esta configurado como entrada. Como CMP usa `PA1/TIM5_CH2`, revisao A exige condicionamento externo robusto ou troca de pino/placa. Em revisoes Z/X/W a limitacao consta como ausente.
- Flash tem limitacao de endurance de 1 kcycle nas revisoes A/Z. NVM/calibracao/seed de sincronismo nao podem gravar com alta frequencia. Para teste real, tratar Flash como recurso de baixa taxa e preferir commit explicito/event-driven.
- A primeira operacao de erase/program apos power-on ou Standby pode congelar fetch/read de Flash por cerca de 120 us. O caminho critico de CKP/scheduler nao deve depender de escrita Flash durante motor girando. Workaround completo exige vetor/handlers criticos e rotina da primeira escrita em SRAM.
- Read-while-write em Flash aumenta latencia em revisoes A/Z. Nao executar erase/program em Bank2 durante janela critica de injecao/ignicao/CKP.
- ADC: manter amostragem regular disparada por TIM6. Nao usar fila de conversoes injetadas, modo dual interleaved, watchdog analogico misturado com canais nao guardados ou stop de conversao injetada sem aplicar os workarounds da errata.
- FDCAN: nao habilitar edge filtering (`EFBI`) e evitar mistura de dedicated Tx buffer com Tx FIFO em prioridades diferentes. Se FDCAN1/FDCAN2 forem usados juntos, manter o mesmo nivel TrustZone/privilegio nos dois. O backend atual deve permanecer simples: sem edge filtering e com disciplina de fila unica/buffer unico.
- USB: em transferencias OUT, a SRAM USB pode ainda estar sendo atualizada quando o CTR dispara. Driver USB real deve inserir atraso minimo antes de ler buffer OUT: 800 ns em Full Speed.
- TIM: conexao de USB SOF para TIM2/TIM5 ITR12 pode falhar; o projeto nao deve usar USB SOF como referencia temporal de motor. TIM5 CKP e TIM2/TIM8 scheduler permanecem independentes.
- IWDG nao acorda o sistema de Stop mode. O projeto de ECU nao deve depender de Stop mode para recuperacao por watchdog.

Conclusao operacional: considerando silicio Rev X, a limitacao de histerese em PA1/CMP e as limitacoes de endurance/read-while-write de Flash das revisoes A/Z nao sao bloqueios esperados. Continuam relevantes em Rev X: congelamento da CPU na primeira escrita/erase de Flash, cautelas de ADC, FDCAN, USB, TIM USB SOF e IWDG em Stop mode. Placa WeAct e firmware atual seguem adequados para bancada sem atuadores energizados. Teste real de motor ainda exige confirmar fisicamente a Rev X, limitar escrita Flash durante motor girando e validar USB/FDCAN/ADC com os workarounds acima.

## Build

Compilar objetos do firmware:

```bash
make firmware
```

Executar regressao host minima do MVP de bancada:

```bash
make host-test
```

Limpar artefatos:

```bash
make clean
```

## MVP Bancada Segura

Definicao de pronto para o MVP de bancada:

- `make firmware` gera `.elf`, `.hex` e `.bin`.
- `make host-test` cobre regressao minima de CKP 60-2, quick crank, scheduler, tabelas, protocolo da UI proprietaria, ETB control, plausibilidade de pedal/ETB e autocalibracao X-τ.
- Firmware sobe na STM32H562 com ST-Link, sem bobinas/injetores energizados, sem reset loop ou hard fault.
- TIM5 recebe CKP/CMP sintetico de 200 a 8500 rpm e telemetria mostra transicoes `WAIT_GAP`, `HALF_SYNC`, `FULL_SYNC`, perda e retomada de sync.
- TIM2/TIM8 geram sinais verificaveis em osciloscopio para cranking, half-sync, full-sync, loss-of-sync e zero RPM.
- ETB (TIM1/PA8) permanece desligado até `etb_cal_valid==1 && !throttle_fault_bits`.
- UI bridge usa UART `PA9/PA10` a 115200 8N1 para assinatura, realtime data e escrita pequena de calibracao.
- Durante ensaio, registrar `loop2_last`, `loop2_max`, `late_event_count`, `cycle_schedule_drop_count` e `calibration_clamp_count`.

### CAN TX Frames

| ID | Cadência | Layout |
|---|---|---|
| 0x400 | 10 ms | RPM(2LE) · MAP_kpa(1) · TPS_pct(1) · CLT+40(1) · advance+40(1) · PW_x10(1) · StatusBits[7:0](1) |
| 0x401 | 100 ms | FuelP_kpa(2LE) · OilP_kpa(2LE) · IAT+40(1) · STFT+100(1) · StatusBits[15:8](1) · VVT_ex_pct(1) |
| 0x402 | 500 ms | FuelAccum_ul(4LE) · FuelDelta_ul(2LE) · reservado(2) |

### Status Bits (16-bit, split entre 0x400 e 0x401)

- Bits 0-7 transmitidos em `0x400 data[7]`; bits 8-15 em `0x401 data[6]`.
- Bits definidos em `src/app/status_bits.h`:
  - `STATUS_SYNC_FULL` (bit 0): sincronismo CKP/CMP completo.
  - `STATUS_PHASE_A` (bit 1): fase parcial disponível.
  - `STATUS_SENSOR_FAULT` (bit 2): falha em sensores analógicos.
  - `STATUS_LIMP_MODE` (bit 3): modo de emergência ativo.
  - `STATUS_ETB_LIMP` (bit 4): ETB em modo limp (limite reduzido).
  - `STATUS_XTAU_LEARN` (bit 5): autocalibracao X-τ em progresso.
  - `STATUS_SCHED_LATE` (bit 6): evento de scheduler atrasado.
  - `STATUS_SCHED_DROP` (bit 7): evento descartado no scheduler.
  - `STATUS_SCHED_CLAMP` (bit 8): ajuste limitado na calibracao — em `0x401 data[6]` bit 0.
  - `STATUS_WBO2_FAULT` (bit 9): sensor WBO2 offline — em `0x401 data[6]` bit 1.
  - `STATUS_TLE8888_FAULT` (bit 10), `STATUS_IGN_SEQUENTIAL` (bit 11), `STATUS_REV_LIMIT` (bit 12).
  - `STATUS_LAUNCH_ACTIVE` (bit 13): launch control holding ETB/RPM.
  - `STATUS_TC_ACTIVE` (bit 14): traction control reducing torque.
  - OCH also exposes `tcReduction` (%×10 @ off 45) and `torqueSparkRetard` (deg @ off 47).

### ETB (Electronic Throttle Body)

- PWM: TIM1 CH1 em PA8, frequência configurável (default 2kHz).
- H-bridge: GPIO PB14 (DIR) + PB15 (EN) para driver DRV8701-style.
- Boot policy: ETB desativado (`EN=0`, PWM=0) até calibracao valida e sensores OK.
- Plausibilidade dual-sensor:
  - APP plausibilidade: compara APP1 vs APP2 (pinos PB0/PB1), gera `THROTTLE_FAULT_APP_PLAUS` se delta exceder limite.
  - ETB_TPS plausibilidade: compara ETB_TPS1 vs ETB_TPS2 (pinos PC0/PC1), gera `THROTTLE_FAULT_ETB_PLAUS` se delta exceder limite.
  - Delta máximo default: 12% entre sensores redundantes.
- Falhas: `THROTTLE_FAULT_APP1/APP2/APP_PLAUS` bloqueiam demanda; `THROTTLE_FAULT_ETB_*` entram em limp mode com `etb_max_open_pct_x10_limp` (default 25%).
- RATE limits: `etb_max_rate_pct_per_s` (default 500 %/s).
- Gating do torque manager:
  - Falhas APP (`THROTTLE_FAULT_APP1/APP2/APP_PLAUS`): limitam abertura do ETB (limp home) mas nao desativam o enable request.
  - Falhas ETB (`THROTTLE_FAULT_ETB_TPS1/ETB_TPS2/ETB_PLAUS`): desativam o enable request do ETB via `etb_enable_request = false`.

### X-τ Autocalibracao

- Executa no slot 100ms, gated por: RPM > 2000, STFT ±500 limites, lambda valido, AE inativo.
- Bloqueio de intervalo: `kLearnIntervalMs = 60000u` (mínimo 60s entre aprendizados consecutivos).
- Deadband STFT: não aprende quando `|STFT| <= 20` (±20% deadband).
- Ajuste lento: 1 LSB por minuto max, clamp aos limites de tabela (1..255 ciclos).
- Persistencia: RAM only, salva em NVM page 6 via burn explícito (nunca durante motor girando).
- Safety: nunca executa erase/program durante janela critica de CKP/scheduler.

Fora do MVP de bancada:

- Primeira partida de motor real.
- USB CDC real.
- Operacao com atuadores reais energizados sem validacao previa de pinout, polaridade, cargas dummy e limites de Flash.

## Estado Atual

- Codigo ativo direcionado ao STM32H562.
- Backend de timers STM32 presente para TIM2/TIM3/TIM4/TIM5/TIM8.
- Scheduler principal consolidado em `src/engine/ecu_sched.cpp`.
- Fonte ativa nao deve conter aliases de compatibilidade herdados de outras plataformas.
- `make firmware` e `make host-test` devem continuar passando antes de qualquer entrega de MVP de bancada.
- Esta pagina e a unica fonte de verdade documental do projeto.

## Proximos Passos De Hardening

### Feito no firmware (defensivo — 2026-07-16)

- Cranking/idle harden: `quick_crank_update` 1×/tick (HALF+FULL); `is_cranking`/`is_afterstart`
  gates AE/STFT/X-τ (não só RPM&lt;exit); flood-clear APP≥70% corta fuel+prime; ETB open-loop
  em crank (`etb_idle_max`) + taper 1 s + histerese RPM de idle phase.
- Fuel angular: FULL = VE/running; **HALF+cranking** = batch SIMULTANEOUS (REQ×mult+min_pw,
  sem VE/STFT/AE); corte imediato em exit-crank / flood / protect / !sync.
- Prime + protect: prime bloqueado se flood/protect/half-lockout/rev-limit; `force_output`
  honra inj/ign inhibit (prime não bypassa mask); boot com mask=0x0F e `inj_pw=0` até 1.º tick.
- CKP: sem `schedule_on_tooth` em SPIKE; hist contaminado → LOSS; sem wrap 57→0 sem gap.
- Scheduler: dwell WD arma no pin HIGH + purge no trip; inj inhibit = force OFF + purge fila; lead assinado wrap-safe.
- Scheduler: injector open WD (1.2× PW / hard 36 ms) + overflow da fila prefere dropar ON/DWELL (nunca OFF/SPARK se houver assert).
- Sensores: commit MAP/TPS/faults a cada amostra rapida (nao so no tick 100 ms).
- Limp: falha MAP corta combustivel a qualquer RPM; telemetria PW alinhada ao mask.
- Fuel: `calc_final_pw_us` clampa corr Q8 0.25–2.0× e satura a 100 ms; AE interp signed.
- Page0 apply: rev limit, STFT, decel hysteresis, CMP window, trim por cilindro clampados.
- Flash: layout **LTF3** = magic + CRC-32 dos mapas adaptativos; seed no mesmo
  SM de flush (sem erase independente do setor 0); seed finaliza magic/CRC.
- ETB: PID `etb_control_update` → `etb_driver_set_motor_pwm`; disable → shutdown.
- CKP: `ticks_to_ns`/gap/normal em math overflow-safe; CMP expected em u64; 1ª borda
  CMP só arma timestamp; LOSS zera `cmp_confirms` (exige 2 bordas p/ sequencial).
- Sched: flush fila + safe pins em transição presync↔sequencial.
- Protect: oil fault @ RPM>1500 corta fuel+ign; fuel-rail fault @ RPM>500 corta fuel.
- TS `openems.ini`: page0 80–85 closed-loop/LTFT; 175 layout ver; 176–190 LTFT authority/LEARN;
  191–215 Launch/TC; 216–251 CAN RX map (gear/vehicle/driven wheel); `ochBlockSize` = 86.
- TC slip: CAN `WHEEL_SPEED_KMH` vs `SPEED_KMH` (can_rx_map) → torque cut; else RPM-dot proxy.

### Pinout — estado

- **Dual board:** `make firmware BOARD=rgt6|vgt6` → `openems-rgt6.bin` / `openems-vgt6.bin` (§5.0).
- **RGT6:** INJ PA15/PB3/PC10/PC11 · IGN PC6–9 · ETB PA6/PA8/PB4 · OIL PC1.
- **VGT6:** INJ PE0/2/4/6 · IGN PE9/11/13/15 · ETB PE5/7/8.
- **Boot safe:** `ecu_sched_outputs_safe_early()` logo após clock (RGT6: PA15 JTDI; VGT6: PE* LOW).
- **DFU:** `dfu-util -a 0 -s 0x08000000 -D openems-<board>.bin` + power-cycle BOOT0=0.
- **docs/wiring_diagram.md** stale — usar README §5.

### P1 — restante (decode / cam / proteccao)

- ~~Math overflow-safe `ticks_to_ns` / CMP u64; 1ª borda CMP; LOSS zera `cmp_confirms`~~ → feito.
- ~~Flush fila + safe pins presync↔sequencial; oil/fuel-rail cut~~ → feito.
- ~~Pinout dual RGT6/VGT6 (`BOARD=`, board_pinout.h)~~ → no código.
- ~~Fuel HALF+crank batch + prime/protect + boot INJ LOW~~ → no código.
- ~~Fuel só em FULL_SYNC (HALF: spark wasted, inj inhibit); `close_cmp_seq_gate` em todos os drops de sync (incl. anomaly→WAIT_GAP, stall)~~ → feito.
- ~~Overtemp CLT 105/115 °C (WARN/CRIT) + DiagnosticManager CRITICAL → inj/ign inhibit~~ → feito.
- ~~Multi-spark: gate ≤1500 RPM; angle table 48 + event queue 48 (3 extras × 4 cyl); host margin test~~ → feito.
- DiagnosticManager: debounce/tick/persistência NVM ainda aspiracional (`protection_system.md`).

### P2 — bancada / HIL / docs

- Validar imagem `.bin` com ST-Link, sem bobinas/injetores no primeiro boot.
- Scope TIM2/TIM8 (latência/jitter) e TIM5 CKP/CMP 200–8500 rpm.
- USB CDC real: IRQ/endpoints, sem bloqueio do caminho crítico; RX ≥ envelope.
- Host/HIL expandido: decode, sync, quick crank, scheduler, fuel/ign, sensores (já há host-test 1000+; HIL físico pendente).
- Actualizar `docs/wiring_diagram.md` ao pinout §5 (ainda tem mapa TIM2/PC legado).

## Politica Documental

Este `README.md` e a fonte unica da verdade. Nao criar `STATUS.md`, `ROADMAP.md`, `PLAN.md`, `CLAUDE.md`, `spec.md` ou documentos similares paralelos. Se uma informacao precisa persistir, ela entra aqui ou vira comentario tecnico proximo ao codigo que a executa.
