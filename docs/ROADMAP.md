# Roadmap OpenEMS

## Pendente

### 1. Bug ADC1/ADC2 não converte continuamente (firmware) — HANDOFF de bancada
A math do HIL está **validada 21/21** via bench-mode (ver Concluído). O bug que
restou é de firmware e **afeta sensores reais também**: o ADC captura UMA sequência
e congela (raw não recicla), mesmo com o DAC do estimulador a varrer o pino (PA2
medido 0.22–3.19 V real; o caminho analógico está OK).
- **Já corrigido e em produção**: `DMACFG=1`+`OVRMOD=1` no CFGR1 e re-arm do GPDMA
  por dente (`adc_trigger_on_tooth`) → ADSTART=1, init_faults 4→0, sem `SENSOR_FAULT`.
- **Falta**: a DMA não recicla (modo comum ADC1+ADC2). Precisa de bancada com
  analisador lógico / single-step — não dá pra fechar por flash às cegas. Tentar
  GPDMA circular real via linked-list (CLLR auto-apontando) ou polling de `ADC1_DR`
  por EOS sem DMA p/ isolar DMA vs conversão. Ver memória `adc1-nao-converte`.
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
