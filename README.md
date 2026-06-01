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

### 5. Atuadores

Mapeamento atual pretendido:

| Funcao | Timer/Canal | Pino |
|---|---:|---|
| INJ1 | TIM2 CH1 | PA15 |
| INJ2 | TIM2 CH2 | PB3 |
| INJ3 | TIM2 CH3 | PB10 |
| INJ4 | TIM2 CH4 | PB11 |
| IGN1 | TIM8 CH1 | PC6 |
| IGN2 | TIM8 CH2 | PC7 |
| IGN3 | TIM8 CH3 | PC8 |
| IGN4 | TIM8 CH4 | PC9 |
| CKP | TIM5 CH1 | PA0 |
| CMP | TIM5 CH2 | PA1 |
| AUX PWM 1 | TIM3 CH1 | PA6 |
| AUX PWM 2 | TIM3 CH2 | PA7 |
| AUX PWM 3 | TIM4 CH1 | PB6 |
| AUX PWM 4 | TIM4 CH2 | PB7 |
| ETB PWM | TIM1 CH1 | PA8 (PA9 reservado USART1\_TX \- sem CH1N) |
| ETB DIR | GPIO | PB14 |
| ETB EN | GPIO | PB15 |
| UI proprietaria UART TX | USART1 TX | PA9 |
| UI proprietaria UART RX | USART1 RX | PA10 |

Pinos ADC (expansão):

| Funcao | Canal | Pino |
|---|---:|---|
| KNOCK | ADC1_IN6 | PA5 |
| APP1 | AN1/ADC1_IN7 | PB0 |
| APP2 | AN2/ADC1_IN8 | PB1 |
| ETB_TPS1 | AN3/ADC1_IN9 | PC0 |
| ETB_TPS2 | AN4/ADC1_IN10 | PC1 |

**Nota:** Sensor lambda/O2 (wideband) é recebido exclusivamente via CAN (FDCAN1, ID 0x180). Não há entrada ADC para O2.

Limitacoes de placa/pino:

- PC8/PC9 podem conflitar com microSD em algumas placas WeAct.
- PA15/PB3 compartilham funcoes de debug JTAG/SWJ; a configuracao de debug deve preservar SWD funcional ou liberar esses pinos conscientemente.
- PB10/PB11 nao devem ser reutilizados por perifericos concorrentes se TIM2 CH3/CH4 estiver ativo.
- USART3 em PB10/PB11 nao e permitido no MVP de bancada porque conflita com INJ3/INJ4; usar USART1 em PA9/PA10.

## ADC E Sensores

- Modulos: `src/hal/adc.cpp`, `src/hal/stm32h562/adc.cpp`, `src/drv/sensors.cpp`.
- ADC primario/secundario representam ADC1/ADC2 no STM32.
- TIM6 deve ser o gatilho periodico de amostragem.
- Validacao de sensores deve bloquear valores absurdos e preservar estado de falha para diagnostico.

## Comunicacao

- UI proprietaria: protocolo em `src/app/ui_protocol.cpp`, servido pelo bridge `scripts/bridge.py` e pela interface em `scripts/ui/`.
- MVP de bancada: UI proprietaria via UART 115200 8N1 em `USART1` (`PA9=TX`, `PA10=RX`).
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

1. Validar imagem `.bin` em bancada com ST-Link, sem bobinas/injetores energizados no primeiro boot.
2. Validar TIM2/TIM8 em bancada com osciloscopio, medindo latencia e jitter reais.
3. Validar TIM5 com sinal CKP/CMP sintetico de 200 a 8500 rpm.
4. Endurecer USB CDC real com IRQ/endpoints e sem bloqueio do caminho critico.
5. Reintroduzir testes host ou HIL para decode, sync, quick crank, scheduler, fuel/ignition e sensores.
6. Validar conflitos de pinos na placa final antes de congelar pinout.

## Politica Documental

Este `README.md` e a fonte unica da verdade. Nao criar `STATUS.md`, `ROADMAP.md`, `PLAN.md`, `CLAUDE.md`, `spec.md` ou documentos similares paralelos. Se uma informacao precisa persistir, ela entra aqui ou vira comentario tecnico proximo ao codigo que a executa.
