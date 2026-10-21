# MVP de bancada

## MVP Bancada Segura

Definicao de pronto para o MVP de bancada:

- `make firmware` gera `.elf`, `.hex` e `.bin`.
- `make host-test` cobre regressao minima (suites em `test/test_*.cpp` + `run_all.cpp`): CKP 60-2, quick crank, scheduler, tabelas, protocolo UI, ETB, torque, fuel/ign, X-τ, etc.
- Firmware sobe na STM32H562 (gravado por DFU), sem bobinas/injetores energizados, sem reset loop ou hard fault.
- TIM5 recebe CKP/CMP sintetico de 200 a 8500 rpm e telemetria mostra transicoes `WAIT_GAP`, `HALF_SYNC`, `FULL_SYNC`, perda e retomada de sync.
- As saídas INJ/IGN (GPIO BSRR, despachadas por TIM5_CH3) geram sinais verificaveis em osciloscopio para cranking, half-sync, full-sync, loss-of-sync e zero RPM.
- ETB (PWM PA6/TIM3 no rgt6, PE5/TIM15 no vgt6) permanece desligado até `etb_cal_valid==1 && !throttle_fault_bits`.
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
  - bit 10 reservado (era TLE8888_FAULT; sempre 0), `STATUS_IGN_SEQUENTIAL` (bit 11), `STATUS_REV_LIMIT` (bit 12).
  - `STATUS_LAUNCH_ACTIVE` (bit 13): launch control holding ETB/RPM.
  - `STATUS_TC_ACTIVE` (bit 14): traction control reducing torque.
  - OCH also exposes `tcReduction` (%×10 @ off 45) and `torqueSparkRetard` (deg @ off 47).

### ETB (Electronic Throttle Body)

- PWM: **PA6 TIM3_CH1** (rgt6) ou **PE5 TIM15_CH1** (vgt6), 20 kHz (`etb_pwm_init`).
- H-bridge dual-DIR (BTS7960/VNH5019-style): IN1 open / IN2 close — rgt6 **PA8/PB4**,
  vgt6 **PE7/PE8** (ver `docs/hw/pinout.md`).
- Boot policy: ETB desativado (`EN=0`, PWM=0) até calibracao valida e sensores OK.
- **Auto-cal de power-on** (`src/engine/etb_autocal.{h,cpp}`): a cada boot varre a
  borboleta aos batentes (fechado→aberto) no tick de 2 ms, ANTES do torque manager
  assumir, e aplica os limites TPS1/TPS2 em RAM. Aborta mantendo a cal de flash se o
  motor girar, o driver faltar ou o curso for implausível (span/invertido). O
  `EtbCalRecord` (16 B, magic `EC` + CRC-32, setor adaptativo) persiste a última cal
  boa como **fallback** quando uma partida interrompe a varredura do power-on
  seguinte (`nvm_save_etb_cal` / `nvm_load_etb_cal`). O pedal (APP) continua manual.
- Plausibilidade dual-sensor (pinos = `src/hal/adc.cpp` / `docs/hw/pinout.md`):
  - APP plausibilidade: compara APP1 vs APP2 (**PC0 / PC2**), gera `THROTTLE_FAULT_APP_PLAUS` se delta exceder limite.
  - ETB_TPS plausibilidade: compara ETB_TPS1 vs ETB_TPS2 (**PA2 / PC5**), gera `THROTTLE_FAULT_ETB_PLAUS` se delta exceder limite.
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

