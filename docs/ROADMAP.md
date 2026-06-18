# Roadmap OpenEMS

## Pendente

### 1. Pinos/canais do ADC errados no H562 (firmware+fiação) — HANDOFF
A math do HIL está **validada 21/21** via bench-mode (ver Concluído). Causa-raiz
**definitiva** do "ADC nunca leu sensor real" (DS14258, 2026-06-15): a atribuição de
pinos analógicos em `src/hal/adc.cpp` é incompatível com o STM32H562.
- **PA2 (MAP), PC1 (AN4), PC2 (CLT), PC3 (IAT) NÃO têm ADC** no H562. E os nº de
  canais estão todos errados. Mapa REAL: PA3=INP15, PA4=INP18, PA6=INP3, PA7=INP7,
  PB0=INP9, PB1=INP5, PC0=INP10, PC4=INP4, PC5=INP8.
- O ADC, a DMA e o trigger **funcionam** (corrigido pelo caminho: GPDMA circular via
  LLI auto-referente + DMACFG/OVRMOD; ADSTART=1, init_faults 4→0). Mas o ADC lê pinos
  errados → o MAP no PA2 (sem ADC) nunca chega; sweep do DAC não bate em canal nenhum.
- **FIX**: reatribuir sensores a pinos com ADC, corrigir `kAdc1Sqr1/2`+`gpio_set_analog`
  +header de pinagem, e refazer a fiação da bancada (ESP32 DAC→pino certo). Ex.:
  MAP→PA4(INP18)/PC4(INP4); TPS→PA3(INP15)/PA6(INP3); CLT→PB0(INP9); IAT→PB1(INP5).
  VREF+ é interno ao VDDA (pino 13, ferrite FB1 do 3.3V — OK). Ver `adc1-nao-converte`.
- CMP ainda em LOW (FULL_SYNC usa só o crank); para phase_A, adicionar 2º canal RMT.

### 2. Datalog em SD card (registrado 2026-06-12)
Logging interno no slot TF da placa (STM32H562 tem SDMMC com DMA), em vez de
depender do polling USB a ~30 Hz.

- **Driver SDMMC bare-metal** (sem ST HAL, padrão do projeto): init do cartão,
  blocos 512B via DMA. Conferir pinos do slot (tip. PC8–PC12+PD2) vs pinos em uso.
- **Sistema de arquivos**: FatFs (cartão legível no PC) ou ring bruto de blocos
  com extração via USB (mais robusto a queda de energia).
- **Escrita fora do caminho crítico**: buffer em RAM, flush por blocos no loop
  de fundo — picos de latência do SD (dezenas de ms) absorvidos pelo buffer.
- **Ganhos**: taxa de 100 Hz–1 kHz (captura transientes que 30 Hz perde),
  autonomia sem notebook, log sobrevive a desconexão de USB.
- **Estimativa**: alguns dias de bancada (driver é o grosso do trabalho).
- O relatório de tuning do dashboard (`/api/log/export`) já serve para analisar
  os logs do cartão — só importar o CSV/binário para `tools/openems_dash/logs/`.

## Concluído (referência)

- **HIL 21/21 — math da ECU validada ponta-a-ponta** (2026-06-15): VE interpolado vivo,
  avanço e PW de injeção batem com o mirror Python em 800/1500/3000 RPM. Como o ADC
  está com bug (Pendente §1), usa-se **bench-mode** (cmd USB `B`) que força MAP/TPS/
  CLT/IAT e limpa `SENSOR_FAULT` desses canais. Firmware ganhou: VE vivo interpolado
  em `reserved[49]` do snapshot (`get_ve`), fix do RPM real na plausibilidade MAP×TPS.
  Script: corrigidas as tabelas CLT/IAT corr (8× int16, não 16× u8) e a base de PW
  (usa VE interpolado). Ver memórias `bench-mode-clt-iat`, `hil-pw-ve-snapshot`.
- **Infra HIL automática validada** (2026-06-14, `tools/hil_test/hil_test.py`):
  `StimClient` dirige o esp32_stimulator por serial (`/dev/ttyUSB0`) ou TCP
  (`192.168.15.169:3333`); reusa o mirror de math + leitores de tabela. Run
  end-to-end (800/1500/3000 RPM): **FULL_SYNC ✓, RPM tracking ✓ (<0,2%)**, sem late
  events. Fórmula de PW esperada corrigida para espelhar o firmware (MAP/baro,
  λ-target e dead-time +900µs). Checks de VE/avanço/PW dependem do wiring analógico
  (ver Pendente §1). Lembrete de bancada: power-cycle real após DFU (`:leave` é jump,
  não POR — memória trustzone-blocks-gpio-tim5).
- Clock 250 MHz real + USB CDC estável (cadeia de 6 bugs de regs.h, 2026-06-11)
- Comando `A` com ZLP; IWDG de produção (boot 10s → runtime 100ms)
- Dashboard de calibração (`tools/openems_dash/`): telemetria WS 30 Hz, editores
  VE/Spark/Lambda, parâmetros 1D/2D, config do motor, calibração de sensores
  com captura ao vivo, datalog CSV + relatório de tuning Markdown
- Eixo RPM máx 8000 (16 pontos redistribuídos)
- Página 7 (dwell 2D) acessível via protocolo; calibração de sensores ligada
  aos drivers (antes inalcançável)
