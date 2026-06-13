# Roadmap OpenEMS

## Pendente

### 1. Teste HIL automático (`hil_test.py` adaptado ao estimulador)
**Caminho de hardware VALIDADO** (2026-06-13): estimulador esp32_stimulator gera
CKP 60-2 por RMT, a ECU atinge FULL_SYNC e rastreia RPM <0,2% em 600–5000 RPM
(sweep bidirecional 9/9, estável em regime). Falta o software:
- Adaptar `tools/hil_test/hil_test.py`: substituir `ESP32Client` (esp32_combined)
  por um `StimClient` que fala o protocolo do estimulador (`RPM n`, `MAP n`, …)
  via serial `/dev/ttyUSB0` ou TCP `192.168.15.169:3333`. Reusar o mirror de
  math + leitores de tabela (`STM32Client`, `Tables`, `EngineConfig`).
- Sweep RPM×MAP×CLT comparando PW/avanço/VE computados vs medidos; relatório MD.
- **Nota de bancada**: após gravar a STM32 por DFU, fazer **power-cycle** (o
  `dfu-util :leave` é jump, não POR; sem POR o RCC fica em estado do bootloader e
  GPIO/ADC/TIM5 não clocam — ver memória trustzone-blocks-gpio-tim5).
- CMP ainda em LOW (FULL_SYNC usa só o crank); para phase_A correto, adicionar 2º
  canal RMT sincronizado depois.

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

- Clock 250 MHz real + USB CDC estável (cadeia de 6 bugs de regs.h, 2026-06-11)
- Comando `A` com ZLP; IWDG de produção (boot 10s → runtime 100ms)
- Dashboard de calibração (`tools/openems_dash/`): telemetria WS 30 Hz, editores
  VE/Spark/Lambda, parâmetros 1D/2D, config do motor, calibração de sensores
  com captura ao vivo, datalog CSV + relatório de tuning Markdown
- Eixo RPM máx 8000 (16 pontos redistribuídos)
- Página 7 (dwell 2D) acessível via protocolo; calibração de sensores ligada
  aos drivers (antes inalcançável)
