# OpenEMS

Open-source Engine Management System for 4-cylinder 4-stroke engines.
Built on ESP32 / ESP32-S3 using ESP-IDF (pure, no Arduino layer).

**License:** GPL v3  
**Status:** Active development — not yet production-ready  
**Language:** C (C99), with HAL headers in C/C++ compatible style

---

## Hardware

- **MCU:** ESP32 or ESP32-S3 (dual-core Xtensa LX6/LX7, 240 MHz)
- **Trigger:** 60-2 tooth crankshaft wheel + Hall camshaft sensor
- **Injection:** Sequential multipoint, saturated (high-impedance) injectors
- **Ignition:** Sequential COP (Coil-On-Plug), logic-level output
- **Lambda:** Wideband O2 via CAN bus
- **Comms:** ESP-NOW wireless tuning + TunerStudio protocol + OBD2 CAN

## Architecture

```
Core 0 (bare metal / ISR)           Core 1 (FreeRTOS tasks)
────────────────────────            ───────────────────────
decoder/trigger_60_2.c              control/fuel_calc.c
scheduler/event_scheduler.c         control/closed_loop_fuel.c
scheduler/injector_driver.c         control/ignition_timing.c
scheduler/ignition_driver.c         control/vvt_control.c
hal/hal_gpio.h  (inline)            control/idle_control.c
hal/hal_timer.h (inline)            control/boost_control.c
                                    sensors/sensor_processing.c
                                    comms/can_wideband.c
                                    comms/espnow_link.c
                                    comms/tunerstudio.c
                                    logging/sd_logger.c
                                    diagnostics/fault_manager.c
```

Core 0 ↔ Core 1 data exchange via `utils/atomic_buffer.h` (seqlock, no mutex).

## Key features

- 60-2 trigger decoding via PCNT + ETM + GPTimer (hardware-timestamped, zero CPU load per tooth)
- Angle-based event scheduling (rusefi-inspired): events scheduled in crank degrees, converted to µs at dispatch time — no error accumulation when RPM changes
- MCPWM absolute-compare injection and ignition: no timer restart, minimal jitter
- High-precision timing with phase predictor, hardware latency compensation, jitter measurement
- 16×16 interpolated VE / ignition / lambda maps with NVS persistence
- Closed-loop lambda: STFT + LTFT with learning stability gating
- Flex fuel (E0–E100) via frequency sensor
- Dual VVT closed-loop PID
- Knock control with per-cylinder retard and recovery
- Boost PID with overboost cut
- Idle PID (IAC)
- Launch control, traction control (cylinder cut)
- ESP-NOW dashboard + TunerStudio wireless tuning
- OBD2 PIDs via CAN
- SD card data logging with event triggers
- Limp mode with per-sensor fault detection
- DTC storage in NVS with freeze frame

## Repository structure

```
openems/
├── firmware/
│   ├── hal/          HAL — zero-latency inline wrappers
│   ├── decoder/      Trigger wheel decoder (Core 0)
│   ├── scheduler/    Angle-based event scheduler (Core 0)
│   ├── sensors/      Sensor reading and processing
│   ├── control/      Control loops (fuel, ignition, VVT, idle, boost)
│   ├── tables/       16x16 interpolated maps
│   ├── comms/        CAN, ESP-NOW, TunerStudio, OBD2
│   ├── diagnostics/  Fault manager, limp mode, DTC
│   ├── logging/      SD card logger
│   ├── config/       Engine parameters, NVS persistence
│   └── utils/        Math, logger, CLI, atomic buffer
├── tests/            Host-side unit tests (Google Test, no hardware needed)
├── tuning/           TunerStudio .ini and base maps
├── hardware/         KiCad schematics and PCB
├── docs/             Architecture, calibration, wiring guides
└── tools/            Python: trigger simulator, log analyzer
```

## Building

```bash
# Install ESP-IDF v5.x
. $IDF_PATH/export.sh

# Configure
idf.py set-target esp32s3   # or esp32

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash monitor
```

## Pin assignments

See `firmware/hal/hal_pins.h`. Change only this file to adapt to a different board.

## Calibration

1. Set `TRIGGER_TDC_OFFSET_DEG` in `firmware/config/engine_config.h` to match your engine.  
   (Distance in crank degrees from the 60-2 gap to TDC cylinder 1.)
2. Set `REQ_FUEL_US` based on injector size and displacement.
3. Load a base VE map via TunerStudio and tune on a dyno or with wideband feedback.

## Contributing

See `docs/contributing.md`. All code must pass `tests/` before merge.
Critical path (Core 0) changes require documented worst-case ISR timing analysis.

## License

GPL v3 — see `LICENSE`.
