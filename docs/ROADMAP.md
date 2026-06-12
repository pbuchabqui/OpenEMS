# Roadmap OpenEMS

## Pendente

### 1. Validação HIL (bloqueada por hardware)
`tools/hil_test/hil_test.py` + ESP32 com `tools/esp32_combined/esp32_combined.ino`
(gera CKP 60-2 e mede IGN/INJ). RPM sweep com verificação matemática de
VE/spark/dwell contra as tabelas; critério: FULL_SYNC nos status bits.
**Aguarda**: ESP32 conectado na bancada (porta `/dev/ttyUSB*`).

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
