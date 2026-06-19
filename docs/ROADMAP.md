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

### 3. Migração para STM32H562VGT6 (LQFP100)
Mais pinos disponíveis. Permite:
- Pino dedicado para EWG H-bridge IN2 (atualmente PB4, compartilhado com LED)
- Restaurar LED heartbeat em pino próprio
- ADC adicionais para sensores (EGT, fuel pressure, oil pressure)
- Marcado no código com `// TODO(VGT6)`

## Concluído (referência)

- **Diagrama elétrico completo** (2026-06-19): power supply (buck 5V wide-Vin +
  LDO 3.3V + TVS SMBJ24CA), condicionamento de sinais (divisores, filtros RC,
  proteção TVS por canal), atuadores externos (ETB/EWG H-bridge, WBO2, flex fuel),
  conector ECU genérico (~47 pinos com VVT), star grounding (PGND/SGND/Shield).
  CAN transceiver integrado no TLE8888 (sem PHY externo). Correções ultrareview:
  divider flex fuel (3.3kΩ), ADC range NTC (0–3V), reguladores automotivos.
- **Flex fuel sensor** (2026-06-18): PB5/EXTI5, 50–150Hz → 0–100% etanol, duty
  → temperatura combustível. Stoich AFR auto-ajustado (14.7→9.8 conforme E%).
- **TLE8888 VRS conditioner** (2026-06-18): CKP reluctor entra direto no TLE8888
  VRS_IN → saída digital VRS_OUT → PA0/TIM5. Sem comparador externo.
- **Electronic Wastegate (EWG)** (2026-06-18): motor DC + H-bridge externo (PA6 PWM
  10kHz + PA7/PB4 DIR), PID de posição (2ms), cascata boost PI (20ms) → position PID.
  Calibração EWG gains + pos min/max em NVM page 0.
- **Remoção IACV** (2026-06-18): idle controlado exclusivamente pela ETB. PA6 liberado
  para EWG PWM.
- **TLE8888 integração completa** (2026-06-18): SPI2 driver (config/diag, 3.9MHz,
  timeout 500µs), INJ low-side (OC 10A), IGN push-pull (OC 6A), VRS conditioner,
  per-channel fault bitmap na telemetria + dashboard, watchdog 100ms.
- **Closed-loop fuel** (2026-06-18): STFT (PI, params em NVM), LTFT (mult+add,
  reset via dashboard), lambda target table editor, gauges λ tgt / LTFT% / E%.
  X-τ auto-calibration params em NVM.
- **WBO2 CAN ID configurável** (2026-06-17): offset 138 page 0, default 0x180.
  Idle RPM vs CLT movido para seção IDLE no dashboard.
- **Dashboard full English** (2026-06-17): tradução completa, Pedal Map Send/Burn/
  Reload pattern, Reload button restaurado em params groups.
- **DBW puro + boost por marcha** (2026-06-16): dashboard estilo MoTeC M1, subgrupos
  de parâmetros.
- **Trim combustível/ignição por cilindro** (2026-06-16): janela dente CMP.
- **Idle ETB + dirigibilidade** (2026-06-16): marcha lenta via ETB, floor + integrador
  I, warmup CLT table.
- **Unidades reais no dashboard** (2026-06-16): sem ×10/×100/µs nos valores exibidos.
- **Redistribuição eixo MAP** (2026-06-16): 9 linhas vácuo (20–110kPa) + 7 boost.
- **HIL 21/21 — math da ECU validada ponta-a-ponta** (2026-06-15): VE interpolado vivo,
  avanço e PW de injeção batem com o mirror Python em 800/1500/3000 RPM. Bench-mode
  (cmd USB `B`) força MAP/TPS/CLT/IAT e limpa SENSOR_FAULT. Script HIL corrigido
  (tabelas CLT/IAT corr 8× int16, PW usa VE interpolado).
- Clock 250 MHz real + USB CDC estável (cadeia de 6 bugs de regs.h, 2026-06-11)
- Comando `A` com ZLP; IWDG de produção (boot 10s → runtime 100ms)
- Dashboard de calibração (`tools/openems_dash/`): telemetria WS 30 Hz, editores
  VE/Spark/Lambda, parâmetros 1D/2D, config do motor, calibração de sensores
  com captura ao vivo, datalog CSV + relatório de tuning Markdown
- Eixo RPM máx 8000 (16 pontos redistribuídos)
- Página 7 (dwell 2D) acessível via protocolo; calibração de sensores ligada
  aos drivers (antes inalcançável)
