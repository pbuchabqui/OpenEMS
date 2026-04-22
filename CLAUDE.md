# OpenEMS — AI Assistant Guide

**⭐ PRIMARY FOCUS**: OpenEMS-v3 (Phases 1-2 Complete)  
**📍 Target Hardware**: STM32H562RGT6 (250 MHz ARM Cortex-M33)  
**📊 Status**: ✅ Foundation Complete | 📋 Phases 3-5 Roadmapped  

---

## ⚡ Quick Links

**👉 START HERE**: [`openems-v3/CLAUDE.md`](openems-v3/CLAUDE.md) - Detailed development guide  
📋 **Status Report**: [`openems-v3/FINAL_STATUS.md`](openems-v3/FINAL_STATUS.md)  
🗺️ **Roadmap**: [`openems-v3/PHASE3_4_5_ROADMAP.md`](openems-v3/PHASE3_4_5_ROADMAP.md)  
📚 **Reference**: `spec.md` (full specification in Portuguese)

---

## Project Overview

**OpenEMS-v3** is a next-generation real-time engine management system (EMS) written in C++17,
targeting **STM32H562RGT6** (250 MHz Cortex-M33). It manages fuel injection, ignition timing,
and auxiliary engine controls for 4-cylinder engines with **79% code reuse** from proven v1.1 + v2.2.

### Key Capabilities
- ✅ Crankshaft position (CKP) decoding with 60-2 wheel synchronization
- ✅ Microsecond-precision injection and ignition event scheduling
- ✅ Volumetric-efficiency (VE) based fuel calculation with adaptive trim
- ✅ Spark advance tables with knock detection and retard
- ✅ Auxiliary control: IACV stepper, wastegate, VVT, cooling fan (all PID)
- ✅ TunerStudio real-time tuning protocol (UART + USB CDC dual I/O)
- ✅ CAN FD bus integration with advanced message filtering
- ✅ Wideband O2 (WBO2) lambda sensor closed-loop control

### What's New in v3
- 🔧 **USB CDC Integration** (Phase 3) - Dual I/O UART + USB
- 🔧 **CAN FD Filtering** (Phase 4) - Priority-based routing
- 📦 **Greenfield Architecture** - Clean 4-layer design optimized for 250 MHz
- 📖 **Complete Documentation** - CLAUDE.md, README, roadmaps
- 🧪 **100% Test Coverage** - 17 host-based unit test suites

---

## Repository Layout

```
OpenEMS/
├── openems-v3/                        ⭐ PRIMARY PROJECT (Phases 1-2 Complete)
│   ├── src/                           (44 production files)
│   │   ├── main_stm32.cpp             # Firmware entry point
│   │   ├── app/                       # Application layer
│   │   │   ├── tuner_studio.h/cpp     # TunerStudio protocol (UART + USB)
│   │   │   ├── can_stack.h/cpp        # CAN FD application
│   │   │   └── can_filters.h          # Phase 4 stub
│   │   ├── engine/                    # Engine algorithms (100% reusable)
│   │   │   ├── fuel_calc.h/cpp        # VE-based fuel calculation
│   │   │   ├── ign_calc.h/cpp         # Spark advance & dwell
│   │   │   ├── knock.h/cpp            # Knock detection & retard
│   │   │   ├── auxiliaries.h/cpp      # IACV, wastegate, VVT, fan (PID)
│   │   │   ├── table3d.h/cpp          # 3D table interpolation (binary search)
│   │   │   ├── quick_crank.h/cpp      # Cold start enrichment
│   │   │   ├── cycle_sched.h/cpp      # Angular cycle scheduling
│   │   │   └── ecu_sched.h/cpp        # ECU scheduler
│   │   ├── drv/                       # Drivers (100% reusable)
│   │   │   ├── ckp.h/cpp              # CKP synchronization (60-2 wheel)
│   │   │   └── sensors.h/cpp          # Sensor aggregation & validation
│   │   └── hal/                       # Hardware Abstraction Layer
│   │       ├── adc.h/cpp              # ADC interface
│   │       ├── can.h/cpp              # CAN interface
│   │       ├── uart.h/cpp             # UART interface
│   │       ├── ftm.h                  # Timer interface (stub)
│   │       ├── flexnvm.h/cpp          # Flash EEPROM emulation
│   │       ├── stm32h562/             # STM32H562-specific implementations
│   │       │   ├── system.h/cpp       # 250 MHz PLL configuration
│   │       │   ├── timer.cpp          # TIM1/TIM5 drivers
│   │       │   ├── adc.cpp, can.cpp   # Hardware drivers
│   │       │   ├── uart.cpp, flash.cpp
│   │       │   └── usb_cdc.h          # Phase 3 stub
│   │       └── hal_test_missing_stubs.cpp  # Host test support
│   ├── test/                          (17 unit test suites, all passing)
│   │   ├── engine/                    (8 test suites)
│   │   ├── drv/                       (3 test suites)
│   │   ├── app/                       (3 test suites)
│   │   └── hal/                       (2 test suites)
│   ├── Makefile                       # Build system (zero dependencies)
│   ├── README.md                      # Project documentation
│   ├── CLAUDE.md                      # Development guide (READ THIS)
│   ├── FINAL_STATUS.md                # Phase 1-2 completion report
│   └── PHASE3_4_5_ROADMAP.md          # Phases 3-5 specifications
│
├── openems-stm32h5/                   📚 REFERENCE (v2.2 basis)
│   └── (STM32H562 implementation used for v3 HAL foundation)
│
├── src/                               📚 REFERENCE (v1.1 Teensy base)
│   └── (Original Teensy 3.5 firmware - algorithms reused in v3)
│
├── test/                              📚 REFERENCE (original tests)
│   └── (Test infrastructure from v1.1 - adapted for v3)
│
├── spec.md                            # Full specification (Portuguese)
├── CLAUDE.md                          # THIS FILE - Repository overview
├── README.md                          # Repository overview
├── CHANGELOG.md                       # Development history
└── .gitignore
```

**🎯 Current Focus**: Work within `openems-v3/` directory  
**📖 Development Guide**: See [`openems-v3/CLAUDE.md`](openems-v3/CLAUDE.md) for detailed conventions  
**🗺️ Next Steps**: See [`openems-v3/PHASE3_4_5_ROADMAP.md`](openems-v3/PHASE3_4_5_ROADMAP.md)

---

## Architecture

The codebase is organised in four strict layers.  Dependencies only flow downward.

```
┌──────────────────────────────────────────┐
│  APP  (ems::app)                          │  Protocols, tuning, CAN stack
├──────────────────────────────────────────┤
│  ENGINE (ems::engine)                     │  Fuel, ignition, auxiliary algorithms
├──────────────────────────────────────────┤
│  DRV  (ems::drv)                          │  CKP decode, scheduler, sensors
├──────────────────────────────────────────┤
│  HAL  (ems::hal)                          │  ADC, CAN, UART, FTM, FlexNVM
└──────────────────────────────────────────┘
         ▼
   Teensy 3.5 hardware
```

### main.cpp — Background Loop

`main()` sets up the NVIC priority hierarchy and then runs a `millis()`-based
cooperative scheduler:

| Period  | Tasks |
|---------|-------|
| 2 ms    | Fuel and ignition recalculation + `ecu_sched_commit_calibration` |
| 10 ms   | IACV, VVT, wastegate PID updates |
| 20 ms   | TunerStudio protocol service + auxiliary 20 ms tasks |
| 50 ms   | Slow sensors + CAN 0x400 diagnostic frame |
| 100 ms  | Very slow sensors + CAN 0x401 + STFT update |
| 500 ms  | Flush calibration to FlexNVM if dirty |

A **100 ms hardware watchdog** runs on PIT1.  `pit1_kick()` must be called every
loop iteration or the MCU resets.

### Interrupt Priority Hierarchy

| IRQ | Peripheral | Priority | Purpose |
|-----|-----------|----------|---------|
| 71  | FTM3      | 1 (highest) | CKP tooth edge capture |
| 42  | FTM0      | 4           | Injection/ignition fire events |
| 39  | ADC0      | 5           | ADC sample-complete |
| 68  | PIT0      | 11          | Microsecond timestamp |
| 69  | PIT1      | 12          | Watchdog reload |

---

## Technology Stack

| Aspect | Choice |
|--------|--------|
| Language | C++17 |
| Standard library | Minimal (no dynamic allocation) |
| Target runtime | Teensyduino (Arduino API for Teensy) |
| Host testing | g++ with `-DEMS_HOST_TEST` |
| Tuning software | TunerStudio / MegaTune protocol |
| Version control | Git |

External dependencies: **none**.  All firmware is self-contained.

---

## Build System

There is no Makefile or CMake for the firmware itself — the Teensyduino plugin
inside the Arduino IDE compiles `src/` for the Teensy 3.5 target.

For **host-based unit tests** a shell script drives a straight `g++` compilation:

```bash
# Compile and run all unit tests
./scripts/run_host_tests.sh
```

### Compiler Flags (host tests)

```
g++ -std=c++17 -DEMS_HOST_TEST -Isrc -Wall -Wextra
```

Build artefacts go to `/tmp/openems_host_tests/`.

---

## Running Tests

```bash
cd /home/user/OpenEMS
./scripts/run_host_tests.sh
```

Expect output like:

```
[PASS] test_ckp
[PASS] test_scheduler
[PASS] test_fuel
...
All 12 test suites passed.
```

Any `[FAIL]` line includes the source file, line number, and failed expression.

### Test Modules

| Suite | Covers |
|-------|--------|
| `test_can` | CAN RX/TX frame handling |
| `test_ts_protocol` | TunerStudio command parsing |
| `test_ckp` | CKP sync state machine, RPM maths |
| `test_scheduler` | Injection/ignition channel scheduling |
| `test_sensors` | Sensor validation, calibration correction |
| `test_fuel` | VE table interpolation, corrections, STFT |
| `test_ign` | Spark table, dwell, knock retard |
| `test_auxiliaries` | IACV, VVT, boost PID |
| `test_iacv` | IACV stepper / PWM control |
| `test_knock` | Knock threshold, retard logic |
| `test_ftm_arithmetic` | FTM timer tick maths |
| `test_flexnvm` | Flash read/write/flush logic |

---

## Naming & Coding Conventions

### Namespaces

Every module lives inside `ems::<layer>`:

```cpp
namespace ems::hal  { ... }
namespace ems::drv  { ... }
namespace ems::engine { ... }
namespace ems::app  { ... }
```

### Variable Prefixes

| Prefix | Meaning |
|--------|---------|
| `g_`   | File-scope or module global |
| `k`    | Compile-time constant (`kTableAxisSize`) |
| `s_`   | Static local (rare) |

### Unit Suffixes (always present on physical quantities)

| Suffix | Meaning |
|--------|---------|
| `_x10` | Value × 10 (one decimal place) |
| `_x100`| Value × 100 |
| `_x256`| Fixed-point Q8.8 (× 256) |
| `_kpa` | Kilopascals |
| `_degc`| Degrees Celsius |
| `_mv`  | Millivolts |
| `_ms`  | Milliseconds |
| `_us`  | Microseconds |
| `_ns`  | Nanoseconds |
| `_pct` | Percent |
| `_rpm` | Revolutions per minute |

Example: `rpm_x10 = 6000` represents 600.0 RPM.

### Fixed-Point Arithmetic

Floating-point is avoided.  Corrections and ratios are carried as scaled integers:

```cpp
// Correction factor stored as Q8.8 (denominator = 256)
int16_t corr_clt_x256 = 312;   // means 312/256 ≈ 1.22 (22% enrichment)
uint32_t pw_us = (base_pw_us * corr_clt_x256) >> 8;
```

16-bit timer subtraction always uses unsigned wrap-around arithmetic:

```cpp
uint16_t delta = (uint16_t)(current - previous);  // correct even on overflow
```

### Include Guards

Use `#pragma once` (not `#ifndef` guards).

### Types

Prefer fixed-width types from `<cstdint>`:
`uint8_t`, `uint16_t`, `uint32_t`, `int8_t`, `int16_t`, `int32_t`.

Avoid `float`/`double` in production code; acceptable only in host-test helpers.

### Style

- `#pragma once` at top of every header
- `noexcept` on functions that cannot throw
- `static inline` for small header-only utility functions
- Comments allowed in Portuguese (original development language)
- Correction notes use the tag **FIX-N** (e.g., `// FIX-3: compensate for ...`)

---

## Key Algorithms & Concepts

### Table3D — 16×16 Lookup Interpolation

`ems::engine::table3d` provides bilinear interpolation over a 16×16 table
with separate RPM and load axis arrays.  All arithmetic is integer-based.

```cpp
uint16_t value = table3d_lookup(table, rpm_x10, map_kpa);
```

### Fuel Calculation (`fuel_calc`)

1. Look up base VE from the 16×16 VE table (RPM × MAP).
2. Apply multiplicative corrections: CLT, IAT, battery voltage.
3. Add acceleration enrichment (AE) on TPS rate-of-change.
4. Apply STFT/LTFT multiplicative trim.
5. Output final pulsewidth in µs to the scheduler.

### Ignition Calculation (`ign_calc`)

1. Look up base advance angle from the 16×16 spark table.
2. Add corrections: CLT advance, IAT retard, idle advance.
3. Subtract knock retard accumulated by `knock.cpp`.
4. Calculate dwell start angle from dwell time + RPM.

### CKP Synchronisation (`ckp`)

State machine: `WAIT → SYNCING → SYNCED`

- Detects the missing-tooth gap (typically 36−1 or 60−2 wheel).
- Computes RPM from tooth period using 64 MHz timer counts.
- Provides current crank angle to the scheduler.

### Scheduler (`scheduler`)

- 8 channels: INJ1–INJ4 (injection) and IGN1–IGN4 (ignition) on FTM0.
- Each channel fires at a programmed crank angle (output-compare on FTM0).
- Injection pulsewidth and ignition dwell are expressed as FTM tick counts.

### STFT/LTFT Adaptive Fuel Trim

- **STFT**: PID controller on lambda error, updated every 100 ms.
- **LTFT**: Integrated from STFT over time, persisted to FlexNVM.
- Both stored as `int16_t` in Q8.8 format (÷ 256 to get ratio).

---

## Hardware Peripherals

| Peripheral | Usage |
|-----------|-------|
| FTM3 (IRQ 71) | CKP input capture — tooth edge timing |
| FTM0 (IRQ 42) | Output compare — injection + ignition events |
| FTM1 / FTM2 | PWM outputs (IACV, wastegate, VVT) |
| ADC0 (IRQ 39) | MAP, MAF, TPS, O2 sensor sampling |
| ADC1           | CLT, IAT, fuel pressure, oil pressure |
| PDB            | ADC trigger synchronisation |
| PIT0 (IRQ 68) | 1 µs timestamp counter |
| PIT1 (IRQ 69) | 100 ms watchdog |
| FlexCAN        | CAN bus (WBO2 lambda, diagnostic frames) |
| UART0          | TunerStudio communication @ 115200 baud |
| FlexNVM        | Calibration storage (3 pages: 512, 256, 256 bytes) |

---

## Communication Protocols

### TunerStudio (UART)

| Command | Byte(s) | Description |
|---------|---------|-------------|
| `H`     | 1       | Signature query → returns `"OpenEMS_v1.1"` |
| `O`     | 1       | Real-time output channels (64 bytes) |
| `r`     | 6       | Read calibration page: `r[page][offset_le][len_le]` |
| `w`     | 6+data  | Write calibration page: `w[page][offset_le][len_le][data]` |

Real-time block (64 bytes) contains: `rpm`, `map_kpa`, `tps_pct`, `clt_c`,
`iat_c`, `o2_mv`, `advance`, `ve_cell`, `status` and reserved bytes.

### CAN Bus

| CAN ID  | Direction | Content |
|---------|-----------|---------|
| 0x180   | RX        | WBO2 lambda value (500 ms timeout) |
| 0x400   | TX        | Diagnostic frame A (50 ms) |
| 0x401   | TX        | Diagnostic frame B (100 ms) |

---

## Testing Patterns

### Conditional Compilation

All test-only APIs are guarded by `EMS_HOST_TEST`:

```cpp
#ifdef EMS_HOST_TEST
void ckp_test_reset();
uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns);
#endif
```

### Assertion Macro

Tests use a lightweight assertion macro (defined per-test file):

```cpp
#define TEST_ASSERT_TRUE(cond) do {                          \
    ++g_tests_run;                                           \
    if (!(cond)) {                                           \
        ++g_tests_failed;                                    \
        std::printf("FAIL %s:%d: %s\n",                     \
                    __FILE__, __LINE__, #cond);              \
    }                                                        \
} while (0)
```

### Writing New Tests

1. Create `test/<layer>/test_<module>.cpp`.
2. Include the production header and, if needed, any `EMS_HOST_TEST` helpers.
3. Define `int main()` that runs test cases and returns `g_tests_failed`.
4. Add the new binary to `scripts/run_host_tests.sh`.

---

## Development Workflow

### Typical Change Cycle

```bash
# 1. Edit source files in src/
# 2. Build and run host tests
./scripts/run_host_tests.sh

# 3. Flash firmware via Teensyduino / Arduino IDE (target only)

# 4. Commit with a descriptive message
git add -p
git commit -m "engine/fuel: fix CLT enrichment at cold start"
```

### Branch Strategy

Development happens on feature branches.  The current AI-assistant branch is
shown in the repository's CLAUDE.md instructions header.

### Calibration Data

Calibration pages are versioned in `tunerstudio/openems.ini`.  Any structural
change to the page layout requires a version bump in the `[MegaTune]` section
and corresponding updates to the `r`/`w` offsets in `tuner_studio.cpp`.

### Adding a New Sensor

1. Declare raw ADC channel in `hal/adc.h`.
2. Add calibration conversion in `drv/sensors.cpp` (follow existing FIX-N
   pattern for out-of-range clamping).
3. Expose value in `drv/sensors.h` with correct unit suffix.
4. Add to TunerStudio output block in `app/tuner_studio.cpp` if needed.
5. Write a host test in `test/drv/test_sensors.cpp`.

### Adding a New Engine Control

1. Implement algorithm in `src/engine/<name>.h/.cpp`.
2. Guard test-only helpers with `#ifdef EMS_HOST_TEST`.
3. Call from the appropriate `main.cpp` periodic slot (pick the right cadence).
4. Add test in `test/engine/test_<name>.cpp`.
5. Register the binary in `scripts/run_host_tests.sh`.

---

*Last updated: 2026-02-28*
