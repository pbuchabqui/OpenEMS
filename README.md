# OpenEMS

Fonte unica da verdade do projeto OpenEMS.

## Objetivo

OpenEMS e uma ECU para motores a combustao interna, atualmente focada no STM32H562RGT6. O codigo deve priorizar baixa latencia, baixo jitter e previsibilidade temporal para controle de injecao, ignicao, sensores e comunicacao de calibracao.

Este documento substitui os documentos historicos de plano, status e revisao. Qualquer decisao tecnica duravel deve ser atualizada aqui, nao em novos arquivos Markdown paralelos.

## Plataforma Alvo

- MCU: STM32H562RGT6, Cortex-M33.
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

Limitacoes de placa/pino:

- PC8/PC9 podem conflitar com microSD em algumas placas WeAct.
- PA15/PB3 compartilham funcoes de debug JTAG/SWJ; a configuracao de debug deve preservar SWD funcional ou liberar esses pinos conscientemente.
- PB10/PB11 nao devem ser reutilizados por perifericos concorrentes se TIM2 CH3/CH4 estiver ativo.

## ADC E Sensores

- Modulos: `src/hal/adc.cpp`, `src/hal/stm32h562/adc.cpp`, `src/drv/sensors.cpp`.
- ADC primario/secundario representam ADC1/ADC2 no STM32.
- TIM6 deve ser o gatilho periodico de amostragem.
- Validacao de sensores deve bloquear valores absurdos e preservar estado de falha para diagnostico.

## Comunicacao

- TunerStudio: camada em `src/app/tuner_studio.cpp`.
- USB CDC: alvo principal para calibracao e telemetria.
- CAN/FDCAN: diagnostico e integracao com sensores externos.

Comunicacao nao deve bloquear decode, sync, scheduling ou atuadores.

## Build

Compilar objetos do firmware:

```bash
make firmware
```

Limpar artefatos:

```bash
make clean
```

A suite host foi removida/adiada durante a migracao STM32-only. O proximo ciclo de hardening deve reintroduzir testes focados no pipeline critico: CKP, sync, quick crank, scheduler, tabelas, limites de calibracao e atuadores.

## Estado Atual

- Codigo ativo direcionado ao STM32H562.
- Backend de timers STM32 presente para TIM2/TIM3/TIM4/TIM5/TIM8.
- Scheduler principal consolidado em `src/engine/ecu_sched.cpp`.
- Fonte ativa nao deve conter aliases de compatibilidade herdados de outras plataformas.
- `make firmware` deve continuar passando antes de qualquer entrega.
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
