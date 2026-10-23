# Estado atual e hardening

## Estado Atual

- Codigo ativo direcionado ao STM32H562.
- Backend de timers STM32: TIM5 (CKP/CMP + dispatcher INJ/IGN CH3), TIM6 (trigger ADC), TIM2 (EWG PWM), TIM3 (ETB PWM rgt6), TIM15 (ETB PWM vgt6), TIM4 (VVT).
- Scheduler principal consolidado em `src/engine/ecu_sched.cpp`.
- Fonte ativa nao deve conter aliases de compatibilidade herdados de outras plataformas.
- `make firmware` e `make host-test` devem continuar passando antes de qualquer entrega de MVP de bancada.

## Proximos Passos De Hardening

### Feito no firmware (defensivo — 2026-07-16)

- Cranking/idle harden: `quick_crank_update` 1×/tick (HALF+FULL); `is_cranking`/`is_afterstart`
  gates AE/STFT/X-τ (não só RPM&lt;exit); flood-clear APP≥70% corta fuel+prime; ETB open-loop
  em crank (`etb_idle_max`) + taper (blend 1 s, piso mínimo até `etbTaperMs`/afterstart, `idle_system.md`) + histerese RPM de idle phase.
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
- EOI 2D RPM×CLT (page6 79-114): `eoi_rpm_axis_x10` / `eoi_clt_axis_x10` /
  `eoi_table_deg` (bilinear 3×3). Sem bump de magic v6; page0 164-168 deixa de
  guardar o blend 1D.
- Flash: layout **LTF3** = magic + CRC-32 dos mapas adaptativos; seed no mesmo
  SM de flush (sem erase independente do setor 0); seed finaliza magic/CRC.
- ETB: PID `etb_control_update` → `etb_driver_set_motor_pwm`; disable → shutdown.
- CKP: `ticks_to_ns`/gap/normal em math overflow-safe; CMP expected em u64; 1ª borda
  CMP só arma timestamp; LOSS zera `cmp_confirms` (exige 2 bordas p/ sequencial).
- Sched: flush fila + safe pins em transição presync↔sequencial.
- Protect: oil fault @ RPM>1500 corta fuel+ign; fuel-rail fault @ RPM>500 corta fuel.
- TS `openems.ini`: page0 80–85 closed-loop/LTFT; 175 layout ver; 176–190 LTFT authority/LEARN;
  191–215 Launch/TC; 216–251 CAN RX map (gear/vehicle/driven wheel); `ochBlockSize` = 86.
- TC slip: CAN wheel vs vehicle via `vehicle_inputs` (bridge → can_rx_map) →
  torque cut; else flare proxy (`docs/torque_tc.md`).

### Pinout — estado

- **Dual board:** `make firmware BOARD=rgt6|vgt6` → `openems-rgt6.bin` / `openems-vgt6.bin` (`docs/hw/pinout.md`).
- **RGT6:** INJ PA15/PB3/PC10/PC11 · IGN PC6–9 · ETB PA6/PA8/PB4 · OIL PC1.
- **VGT6:** INJ PE0/2/4/6 · IGN PE9/11/13/15 · ETB PE5/7/8.
- **Mapas INJ/IGN:** `hal/out_pins.h` (fonte unica; `static_assert` por board).
- **Boot safe:** `ecu_sched_outputs_safe_early()` → `out_pins_hw_init()` logo apos
  clock (RGT6: PA15 JTDI; VGT6: PE* LOW). Actuadores active-high.
- **DFU:** `dfu-util -a 0 -s 0x08000000 -D openems-<board>.bin` + power-cycle BOOT0=0.
- **docs/wiring_diagram.md** stale (mapa de pinos ASCII legado) — pinout válido em `docs/hw/pinout.md`.

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

- Validar imagem `.bin` (gravada por DFU), sem bobinas/injetores no primeiro boot.
- Scope das saídas INJ/IGN (BSRR/TIM5_CH3, latência/jitter) e TIM5 CKP/CMP 200–8500 rpm.
- USB CDC real: IRQ/endpoints, sem bloqueio do caminho crítico; RX ≥ envelope.
- Host/HIL expandido: decode, sync, quick crank, scheduler, fuel/ign, sensores (já há host-test 1000+; HIL físico pendente).
- Actualizar `docs/wiring_diagram.md` (esquemáticos eléctricos) ao pinout de `docs/hw/pinout.md` (ainda tem mapa TIM2/PC legado no ASCII).

