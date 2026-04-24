# OpenEMS-v3 Logic Review — USB-only TunerStudio

**Date**: 2026-04-24  
**Scope requested**: cranking, decode, sync, scheduling, fuel, ignition, sensors, and TunerStudio transport path.

## 1) Transport decision implemented

- TunerStudio path in `main_stm32.cpp` now services only USB CDC (`usb_cdc_poll/read/send`) and no longer depends on UART polling/transmit for protocol traffic.
- USB CDC is initialized during platform bring-up in `openems_init()`.
- `ts_rx_byte()` is now the generic entry point for protocol parser input; `ts_uart0_rx_isr_byte()` remains as compatibility wrapper only.

## 2) Logic walk-through checklist (static review)

### Cranking / decode / sync

- CKP decode and synchronization state machine API remains in `drv/ckp.h` (`WAIT_GAP`, `HALF_SYNC`, `FULL_SYNC`, `LOSS_OF_SYNC`).
- Runtime seed fast-reacquire flow remains guarded in `main_stm32.cpp` during init and 2ms loop handling.
- Main loop still gates engine calculations on `FULL_SYNC`.

### Scheduling

- Scheduler stack (`engine/cycle_sched.*` + `engine/ecu_sched.*`) remains invoked from main control loop and tooth callbacks.
- Calibration commit path and runtime diagnostics feeding TunerStudio remain connected.

### Fuel / ignition / sensors

- Fuel computation path remains `get_ve` → `calc_base_pw_us` → correction chain → `calc_final_pw_us`.
- Ignition path remains driven by `get_advance`/scheduler calibration flow.
- Sensor snapshot path remains read via atomic copy (`sensors_get()` by value) and used in 2ms/50ms/100ms cadence in main loop.

## 3) Current caveat

- USB CDC low-level implementation is currently a Phase-3 placeholder backend (`usb_cdc.cpp`) to preserve a single USB-only integration path while hardware-specific endpoint/interrupt plumbing is completed.
- Because host test suite was intentionally removed, this review is static/code-path verification, not automated regression evidence.

## 4) Next recommended hardening tasks

1. Replace placeholder USB backend with STM32H562 USB FS endpoint/IRQ implementation.
2. Add bench validation script/checklist for connect/disconnect and sustained telemetry.
3. Reintroduce automated regression coverage for CKP/sync/schedule/fuel/ignition/sensors once test strategy is restored.
