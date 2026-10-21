# Pipeline de controle do motor

## Pipeline De Controle Do Motor

### 1. Captura CKP/CMP

- Modulo principal: `src/drv/ckp.cpp`.
- Backend STM32: `src/hal/stm32h562/timer.cpp`.
- Timer usado: TIM5.
- Canais:
  - CKP: TIM5 CH1 em PA0.
  - CMP: TIM5 CH2 em PA1.

A captura mede bordas do virabrequim e comando, detecta dente faltante e alimenta a maquina de sincronismo. Classificacao por razao r = periodo / periodo de referencia:

- r < 0,5: ruido; descartado sem mover a referencia (o dente seguinte e medido certo).
- 0,5 ≤ r < 1,5: dente.
- r ≥ 1,5: candidato a gap; em FULL_SYNC so e aceito com exatamente 57 dentes desde o ultimo gap. Dente perdido ou a mais = perda de sync explicita, nunca ±6° silencioso.
- Pulso de ruido logo apos um dente curto e substituido quando o par fecha um periodo limpo.

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
- Sem sensor de fase o motor roda em wasted spark por pares (0↔3, 1↔2), cada bobina no seu PMS.

A estrategia pre-sync deve degradar para modo conservador se houver duvida de fase. Partida rapida nao pode vencer seguranca de sincronismo.

### 3. Calculos De Combustivel E Ignicao

- Laco de 2 ms (sensores → PW/avanco/dwell → agendador): `src/engine/engine_calc.cpp` (`engine_calc_step`), testado de ponta a ponta em `test/test_engine_calc.cpp` e no motor virtual; `main_stm32.cpp` so le sensores, chama e publica telemetria.
- Combustivel: `src/engine/fuel_calc.cpp`.
- Ignicao: `src/engine/ign_calc.cpp`.
- Tabelas: `src/engine/table3d.cpp`.
- Sensores: `src/drv/sensors.cpp`.

Fluxo esperado:

```text
ADC/TIM6 -> sensors -> fuel_calc/ign_calc -> ecu_sched -> TIM5_CH3/BSRR -> atuadores
```

Os calculos usam RPM, MAP, TPS, CLT, IAT, lambda e calibracoes. Tabelas devem operar com interpolacao deterministica e sem custo imprevisivel no caminho critico.

Combustivel por speed density com oraculo fisico nos testes (`test/test_fuel_physics.cpp`): PW = REQ_FUEL × VE × MAP/101,325 kPa × T_ref/T, ΔP do injetor por √ΔP, filme de parede X-τ (Aquino), LTFT integrando o STFT. Interpolacao de tabelas e curvas arredonda ao mais proximo (sem vies). Partida: REQ_FUEL × multiplicador(CLT), igual em HALF e FULL sync.

Avanco em decimos de grau de ponta a ponta (tabela interpolada a 0,1°, correcoes em 0,1°, faixa unica −20° a 60°). Retardo de knock por cilindro (`ecu_sched_set_cyl_retard_x10`). O laco de 2 ms recalcula e faz commit a cada tick (`ecu_sched_commit_calibration_x10`).

### 4. Scheduling De Injecao E Ignicao

- Modulo principal: `src/engine/ecu_sched.cpp`.
- Injecao **e** ignicao: mesmo dispatcher **TIM5_CH3** (output compare) + fila
  ordenada de eventos absolutos; a saída física é feita por **GPIO BSRR**
  (`hal/out_pins.h`, `out_pin_write` inline). Sem TIM2/TIM8 OC no caminho INJ/IGN.
- Base temporal: TIM5 32-bit @ **62.5 MHz** (16 ns/tick), livre-corrente e
  partilhado com a captura CKP/CMP (CH1/CH2).

Premissa de projeto:

- O caminho critico é agendado por hardware output compare (TIM5_CH3): o OC arma
  o tick de despacho e a ISR escreve o BSRR do canal — não há bit-bang em espera.
- A ISR arma o proximo evento e actualiza estado; a escrita GPIO só é permitida
  header-inline (`out_pin_write`), nunca lógica de fila noutro `.cpp`.
- Janelas longas devem lidar com limite de contador por rearmamento/near-time, sem perder precisao perto do evento.
- Eventos vencidos devem ser tratados explicitamente, nunca silenciosamente aceitos.

Precisao (medida no motor virtual, `make precision-test`):

- SPARK e INJ_ON sao angulares: tabela por dente (dente de armamento + fracao
  1/256 de dente), ancorada na captura do dente, nao no instante da ISR.
  Eventos dentro do gap (348–360°) sao agendados corretamente.
- INJ_OFF = INJ_ON + PW em ticks; DWELL_START = centelha − dwell em ticks:
  largura de pulso e dwell exatos em tempo, independentes de aceleracao.
- Erro de centelha ≤ 0,01° em regime, ≤ 0,5° a ±2700 rpm/s, ≤ 0,2° com jitter de 1 µs.

Eventos de injecao e ignicao ficam codificados no scheduler do motor, nao num
driver legado. O scheduler recebe dentes/angulo do CKP, calcula quando cada canal
deve abrir/fechar (INJ) ou fazer dwell/centelha (IGN) e insere-os na mesma fila
despachada por TIM5_CH3. Ordem de canais BSRR em `docs/hw/pinout.md`.

### 4a. Detecção De Knock (Detonação)

- Sensor piezoelétrico knock conectado em PA5/ADC1_IN6.
- Hardware: filtro passa-banda externo → PA5 → ADC1.
- Detecção: software via threshold ADC (STM32H562 não possui periférico COMP).
- Amostragem: `knock_adc_update(raw)` chamado de `sample_fast_channels()` 12×/rev (acumulador `kFastSamplesPerRev`, não a cada dente CKP) durante janela ativa.
- Threshold ADC: padrão 2048 (12-bit), range [256, 4000].
  - Adaptativo: -64 por evento de knock; ciclos limpos devolvem +32, nunca acima do valor calibrado (sem deriva que desligue a deteccao).
- Janela de knock: `knock_window_cycle_end()`/`knock_window_open()` ligados a `arm_channel()` no evento `ECU_ACT_DWELL_START` (modo sequencial), atrás da flag `EMS_KNOCK_HW_PRESENT` (`hal/board_pinout.h`, default 0 — front-end analógico DNP na v1, `docs/hw/schematic/10_knock_dnp.md`). Com a flag em 0 a janela nunca abre de facto.
- Retardo: +2,0° por evento de knock, máximo 10,0°.
- Recuperação: -0,1° por ciclo limpo após 10 ciclos consecutivos limpos.
- Retardo por cilindro: so a bobina do cilindro que detonou atrasa.
- Persistência NVM: retardo em slot knock; o threshold nao e gravado (volta ao calibrado no boot).

### 5. Atuadores e pinout (firmware)

**Pinout completo: [`docs/hw/pinout.md`](docs/hw/pinout.md)** — mapa por pino dos
dois packages, conflitos WeAct e fiação. Código de referência: `ecu_sched.cpp`
(INJ/IGN BSRR), `hal/out_pins.h` (`static_assert` por board), `etb_driver.cpp`,
`timer.cpp`, `adc.cpp`. O firmware selecciona o package no **build**:

```bash
make firmware BOARD=rgt6   # default — WeAct LQFP64 (INJ/IGN em A/B/C)
make firmware BOARD=vgt6   # LQFP100 (INJ/IGN/ETB em PE*, BSRR único)
# bins: /tmp/openems-build/bin/openems-{rgt6,vgt6}.bin
```

Resumo das diferenças (o RGT6 **não tem port E** → INJ/IGN/ETB redistribuídos):

| Função | **RGT6** (default) | **VGT6** |
|--------|--------------------|----------|
| CKP / CMP | PA0 / PA1 (TIM5) | igual |
| UART / USB / CAN / LED | PA9-10 / PA11-12 / PB8-9 / PB2 | igual |
| INJ1–4 | PA15 / PB3 / PC10 / PC11 | PE0 / PE2 / PE4 / PE6 |
| IGN1–4 | PC6 / PC7 / PC8 / PC9 | PE9 / PE11 / PE13 / PE15 |
| ETB PWM / DIR | PA6 (TIM3) / PA8 / PB4 | PE5 (TIM15) / PE7 / PE8 |
| INJ/IGN drive | GPIO BSRR (A/B/C) | GPIO BSRR (só GPIOE) |

ADC (comum aos dois): MAP PA3 · TPS PA4 · KNOCK PA5 · ETB_TPS1/2 PA2/PC5 · APP1/2
PC0/PC2 · CLT PB0 · IAT PB1 · FUEL_P PC4 · OIL_P PC1 · EWG_POS PC3. WBO2 λ só via
CAN (sem ADC O2). Tabela ADC completa (ADC1/2 + INP): `docs/hw/pinout.md`.

**Trocar de placa:** gravar o binário correcto (`BOARD=…`); flash cruzado = pinos
errados. Boot safe: `ecu_sched_outputs_safe_early()` → `out_pins_hw_init()`
(RGT6: PA15 JTDI pull-up; VGT6: PE\* LOW). Actuadores active-high.

## ADC E Sensores

- Modulos: `src/hal/adc.cpp`, `src/hal/stm32h562/adc.cpp`, `src/drv/sensors.cpp`.
- ADC primario/secundario representam ADC1/ADC2 no STM32.
- ADC 12-bit 0–3.3 V (raw 4095), tensão aceite pelo STM32H5. MAP 0–3.3 V → 0–3.00 bar (`raw × 3000 / 4095`). TPS/APP/ETB default 0–4095.
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
- USB CDC: driver USB DRD FS completo (`hal/stm32h562/usb_cdc.cpp`), ainda nao validado em
  hardware; ate la a UART e o transporte de referencia.
- CAN/FDCAN: diagnostico e integracao com sensores externos.

Comunicacao nao deve bloquear decode, sync, scheduling ou atuadores.

