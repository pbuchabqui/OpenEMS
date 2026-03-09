# OpenEMS — AI Assistant Guide

This document describes the codebase structure, conventions, and development workflows
for **OpenEMS**, an embedded engine management system targeting the Teensy 3.5
microcontroller (NXP MK64FX512VMD12, ARM Cortex-M4 @ 120 MHz).

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Layout](#repository-layout)
3. [Architecture](#architecture)
4. [Technology Stack](#technology-stack)
5. [Build System](#build-system)
6. [Running Tests](#running-tests)
7. [Naming & Coding Conventions](#naming--coding-conventions)
8. [Key Algorithms & Concepts](#key-algorithms--concepts)
9. [Hardware Peripherals](#hardware-peripherals)
10. [Communication Protocols](#communication-protocols)
11. [Testing Patterns](#testing-patterns)
12. [Development Workflow](#development-workflow)

---

## Project Overview

OpenEMS is a real-time engine management system (EMS) written in C++17.
It manages fuel injection, ignition timing, and auxiliary engine controls
for a 4-cylinder engine.  The target platform is the **Teensy 3.5** board.

Key capabilities:
- Crankshaft position (CKP) decoding with 60-2 tooth wheel synchronisation (HALF_SYNC → FULL_SYNC with CAM phase)
- Angle-domain injection and ignition event scheduling (ECU Scheduler v3)
- Volumetric-efficiency (VE) based fuel calculation with bilinear interpolation
- Spark advance tables with knock retard
- Short/Long-term fuel trim (STFT/LTFT) adaptive learning
- Cold-start cranking enrichment, afterstart decay, and prime pulse injection
- IACV, wastegate, VVT, and cooling-fan auxiliary control
- TunerStudio real-time tuning protocol (USB CDC / UART)
- Wideband O2 (WBO2) lambda sensor via CAN bus (ID 0x180)
- Runtime sync seed for fast CKP re-acquisition after key-off
- Limp mode: rev-cut above 3000 RPM on critical sensor fault

For a comprehensive Portuguese-language functional specification see `spec.md`.

---

## Repository Layout

```
OpenEMS/
├── src/                    # Production firmware source
│   ├── main.cpp            # Entry point, NVIC setup, background loop
│   ├── app/                # Application layer (protocols)
│   │   ├── tuner_studio.h/cpp   # TunerStudio UART protocol
│   │   ├── can_stack.h/cpp      # CAN bus application (WBO2, diagnostics)
│   │   └── status_bits.h        # Status byte bit-mask constants
│   ├── drv/                # Driver layer (system-level components)
│   │   ├── ckp.h/cpp            # Crankshaft position sensor decode (60-2)
│   │   └── sensors.h/cpp        # Sensor aggregation and validation
│   ├── engine/             # Engine control algorithms
│   │   ├── fuel_calc.h/cpp      # Fuel injection calculation
│   │   ├── ign_calc.h/cpp       # Ignition timing calculation
│   │   ├── auxiliaries.h/cpp    # IACV, wastegate, VVT, fan
│   │   ├── knock.h/cpp          # Knock detection and retard
│   │   ├── table3d.h/cpp        # 16×16 lookup-table interpolation
│   │   ├── quick_crank.h/cpp    # Cold-start cranking/afterstart enrichment + prime pulse
│   │   ├── cycle_sched.h/cpp    # Angular cycle pre-calculator (tooth-trigger lookup)
│   │   └── ecu_sched.h/cpp      # Hardware angle-domain scheduler (FTM0, v3)
│   └── hal/                # Hardware Abstraction Layer
│       ├── adc.h/cpp            # ADC0/ADC1 with PDB
│       ├── can.h/cpp            # FlexCAN driver
│       ├── uart.h/cpp           # UART0/USB CDC driver
│       ├── ftm.h/cpp            # FlexTimer (PWM, input capture)
│       ├── flexnvm.h/cpp        # Flash calibration + LTFT + knock map storage
│       └── runtime_seed.h       # RuntimeSyncSeed struct & NVM helpers
├── test/                   # Host-based unit tests (mirrors src/ structure)
│   ├── app/
│   │   ├── stub_ecu_sched_ivc.cpp   # Stub for ecu_sched IVC in ts_protocol test
│   │   ├── test_can.cpp
│   │   └── test_ts_protocol.cpp
│   ├── drv/
│   │   ├── test_ckp.cpp
│   │   ├── test_sensors.cpp
│   │   └── test_sensors_validation.cpp
│   ├── engine/
│   │   ├── test_auxiliaries.cpp
│   │   ├── test_ecu_sched.c             # ECU scheduler core tests (compiled as C++)
│   │   ├── test_ecu_sched_fixes.cpp     # ECU scheduler regression suite
│   │   ├── test_fuel.cpp
│   │   ├── test_fuel_calc_assertions.cpp  # Out-of-range clamping (NDEBUG build)
│   │   ├── test_iacv.cpp
│   │   ├── test_ign.cpp
│   │   ├── test_knock.cpp
│   │   ├── test_pipeline_backbone.cpp   # Integration: CKP + cycle_sched + ecu_sched
│   │   └── test_quick_crank.cpp
│   └── hal/
│       ├── test_flexnvm.cpp
│       └── test_ftm_arithmetic.cpp
├── scripts/
│   └── run_host_tests.sh   # Build and run all 17 test suites
├── tunerstudio/
│   └── openems.ini         # TunerStudio project configuration
├── docs/
│   └── OpenEMS_Engineering_Prompt_v1.2.docx
├── spec.md                 # Full functional specification (Portuguese)
├── CHANGELOG.md            # Change log (Portuguese)
└── README.md
```

---

## Architecture

The codebase is organised in four strict layers.  Dependencies only flow downward.

```
┌──────────────────────────────────────────┐
│  APP  (ems::app)                          │  Protocols, tuning, CAN stack
├──────────────────────────────────────────┤
│  ENGINE (ems::engine)                     │  Fuel, ignition, auxiliary algorithms,
│                                           │  cycle_sched, ecu_sched, quick_crank
├──────────────────────────────────────────┤
│  DRV  (ems::drv)                          │  CKP decode, sensors
├──────────────────────────────────────────┤
│  HAL  (ems::hal)                          │  ADC, CAN, UART, FTM, FlexNVM,
│                                           │  runtime_seed
└──────────────────────────────────────────┘
         ▼
   Teensy 3.5 hardware
```

### main.cpp — Background Loop

`main()` sets up the NVIC priority hierarchy and then runs a `millis()`-based
cooperative scheduler:

| Period  | Tasks |
|---------|-------|
| 2 ms    | Fuel/ignition recalculation + `ecu_sched_commit_calibration` + quick_crank |
| 10 ms   | IACV, VVT, wastegate PID updates |
| 20 ms   | TunerStudio protocol service + auxiliary 20 ms tasks |
| 50 ms   | Slow sensors + CAN 0x400 diagnostic frame |
| 100 ms  | Very slow sensors + CAN 0x401 + STFT update + runtime seed save |
| 500 ms  | Flush calibration to FlexNVM if dirty + knock NVM save |

A **100 ms hardware watchdog** runs on PIT1.  `pit1_kick()` must be called every
loop iteration (first statement) or the MCU resets.

### Interrupt Priority Hierarchy

| IRQ | Peripheral | Priority | Purpose |
|-----|-----------|----------|---------|
| 71  | FTM3      | 1 (highest) | CKP tooth edge capture |
| 42  | FTM0      | 4           | Injection/ignition fire events (angle-domain) |
| 39  | ADC0      | 5           | ADC sample-complete (reserved, not yet enabled) |
| 68  | PIT0      | 11          | Microsecond timestamp (`g_datalog_us`) |
| 69  | PIT1      | 12          | Watchdog reload (triggers `system_reset()` on timeout) |

---

## Technology Stack

| Aspect | Choice |
|--------|--------|
| Language | C++17 |
| Standard library | Minimal (no dynamic allocation — no `new`/`delete`) |
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

One test suite (`test_fuel_calc_assertions`) is built separately with `-DNDEBUG`
to exercise clamping behaviour on out-of-range paths without debug assertions firing.

`test_ecu_sched.c` is compiled as C++ despite the `.c` extension (see the script).

---

## Running Tests

```bash
cd /home/user/OpenEMS
./scripts/run_host_tests.sh
```

Expect output ending with:

```
All host tests passed.
```

Any failure prints `FAIL <file>:<line>: <expression>` and exits non-zero.

### Test Modules (17 suites)

| Suite | Covers |
|-------|--------|
| `test_ckp` | CKP sync state machine (WAIT_GAP/HALF_SYNC/FULL_SYNC/LOSS_OF_SYNC), RPM maths, seed |
| `test_sensors` | Sensor calibration conversion, IIR filters |
| `test_sensors_validation` | Out-of-range fault detection and fallback values |
| `test_ftm_arithmetic` | FTM timer tick maths |
| `test_fuel` | VE table interpolation, CLT/IAT corrections, STFT |
| `test_fuel_calc_assertions` | Clamping on out-of-range inputs (NDEBUG build) |
| `test_ign` | Spark table, dwell calculation, knock retard |
| `test_quick_crank` | Cranking enrichment, afterstart decay, prime pulse |
| `test_auxiliaries` | IACV warmup table, VVT, boost PID |
| `test_iacv` | IACV PID controller, anti-windup |
| `test_knock` | Knock threshold, retard logic, VOSEL adjustment |
| `test_ts_protocol` | TunerStudio command parsing (`Q`, `H`, `r`, `w`, `A`/`O`) |
| `test_can` | CAN RX/TX frame handling, WBO2 timeout |
| `test_flexnvm` | Flash read/write/flush, LTFT, knock map, runtime seed |
| `test_ecu_sched` | Angle-domain event table, channel arming, IVC clamp |
| `test_ecu_sched_fixes` | ECU scheduler regression suite |
| `test_pipeline_backbone` | Integration: CKP → cycle_sched → ecu_sched pipeline |

---

## Naming & Coding Conventions

### Namespaces

Every module lives inside `ems::<layer>`:

```cpp
namespace ems::hal    { ... }
namespace ems::drv    { ... }
namespace ems::engine { ... }
namespace ems::app    { ... }
```

`ecu_sched` uses a C-compatible `extern "C"` API (no namespace) to allow mixed
C/C++ builds.  Its header wraps declarations in `extern "C" { }` when compiled
as C++.

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

Use `#pragma once` (not `#ifndef` guards) in C++ headers.
`ecu_sched.h` uses `#ifndef ENGINE_ECU_SCHED_H` for C compatibility.

### Types

Prefer fixed-width types from `<cstdint>`:
`uint8_t`, `uint16_t`, `uint32_t`, `int8_t`, `int16_t`, `int32_t`.

Avoid `float`/`double` in production code; acceptable only in host-test helpers.

### Style

- `#pragma once` at top of every C++ header; `#ifndef` guard in C-compatible headers
- `noexcept` on functions that cannot throw
- `static inline` for small header-only utility functions
- `static_cast<>` preferred over C-style casts in C++ code
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
2. Apply multiplicative corrections: CLT, IAT, battery voltage dead-time.
3. Apply warmup enrichment (additional cold-start factor).
4. Add acceleration enrichment (AE) on TPS rate-of-change.
5. Apply STFT/LTFT multiplicative trim.
6. Pass final pulsewidth in µs to `ecu_sched_commit_calibration`.

### Ignition Calculation (`ign_calc`)

1. Look up base advance angle from the 16×16 spark table.
2. Add corrections: CLT advance, IAT retard.
3. Subtract knock retard accumulated by `knock.cpp`.
4. Clamp to [−10°, +40°] BTDC.
5. Calculate dwell start angle from dwell time (voltage-dependent) + RPM.

### CKP Synchronisation (`ckp`) — 60-2 Wheel

State machine:

```
WAIT_GAP ──(gap + ≥55 teeth)──► HALF_SYNC
                                      │
                         (2nd gap + ≥55 teeth + CAM pulse)
                                      ▼
LOSS_OF_SYNC ◄──(false gap)───── FULL_SYNC
```

- Gap detection: `current_period × 2 > 3 × average_period` (period > 1.5× avg)
- Normal tooth tolerance: ±20% of average
- Minimum period: 50 FTM3 ticks (EMC noise rejection)
- RPM: `RPM_x10 = 600_000_000_000 / (60 × tooth_period_ns)`
- Phase sensor (CAM) on FTM3 CH1: alternates `phase_A` each pulse to identify the 720° cycle half
- FULL_SYNC requires two confirmed gaps plus at least one CAM edge

### Quick Crank — Cold Start (`quick_crank`)

Manages three phases triggered from the background loop:

1. **Prime pulse**: fired on the 5th CKP tooth (no sync required); simultaneous
   injection in all 4 cylinders via `ecu_sched_fire_prime_pulse()`.
2. **Cranking enrichment**: active while RPM < 450 with FULL_SYNC; uses a
   CLT-based multiplier (up to 3.0× at −40°C) and fixes spark at 8° BTDC.
3. **Afterstart decay**: linear decay from an initial CLT-based multiplier back
   to 1.0× over a CLT-dependent duration (500 ms at 80°C → 2400 ms at −40°C).

### Angle-Domain Scheduler (`ecu_sched`) — v3

FTM0 runs free at 120 MHz / PS=64 = **1.875 MHz** (533 ns/tick).

Events are stored as `AngleEvent_t {tooth_index, sub_frac_x256, channel, action, phase_A}`.
Each CKP tooth ISR (FTM3) calls `ecu_sched_on_tooth_hook()` which:
1. Scans the angle table for events matching the current tooth.
2. Computes `offset = (sub_frac × tooth_period_ftm0) >> 8`.
3. Arms the FTM0 channel via output-compare.

This is immune to RPM variation between scheduling and firing.

**FTM0 channel wiring:**

| Channel | Signal | Mode |
|---------|--------|------|
| CH0 | INJ3 | Set-on-match (HIGH = injector energised) |
| CH1 | INJ4 | Set-on-match |
| CH2 | INJ1 | Set-on-match |
| CH3 | INJ2 | Set-on-match |
| CH4 | IGN4 | Clear-on-match (LOW = spark) |
| CH5 | IGN3 | Clear-on-match |
| CH6 | IGN2 | Clear-on-match |
| CH7 | IGN1 | Clear-on-match |

**Ignition firing order:** 1–3–4–2 with TDC offsets 0°, 180°, 360°, 540° in a 720° cycle.

**Safety limits enforced in `ecu_sched_commit_calibration()`:**

| Parameter | Limit |
|-----------|-------|
| Advance | 0–60° BTDC |
| Dwell | max 18750 ticks (10 ms) |
| Injector PW | max 37500 ticks (20 ms) |

**IVC clamp:** prevents injection past Intake Valve Closing in open-valve mode.
Configured via `ecu_sched_set_ivc(ivc_abdc_deg)`.  Inactive with the default
closed-valve strategy (SOI lead = 62°).

### Cycle Scheduler (`cycle_sched`)

Pre-calculates per-cylinder trigger tooth indices from crank angle geometry.
Called by the background loop; provides tooth-trigger data consumed by `ecu_sched`.
Currently used as an intermediate layer; the active scheduling path in production
goes through `ecu_sched` directly via the CKP tooth hook.

### STFT/LTFT Adaptive Fuel Trim

- **STFT**: PID controller on lambda error (Kp=3/100, Ki=1/200), updated every
  100 ms when CLT > 70°C and WBO2 is fresh.  Stored as `int16_t` in %×10 units.
- **LTFT**: Integrated from STFT per cell (16×16), persisted to FlexRAM (256 bytes).
- Both stored as `int16_t` in Q8.8 format (÷ 256 to get ratio).

### Runtime Sync Seed

At engine stop (RPM drops to zero after running), the CKP sync state is saved
to FlexNVM (`RuntimeSyncSeed` struct, 8 rotating slots with CRC32).
On next boot, if a valid seed is found (`tooth_index = 0`, FULL_SYNC + PHASE_A
flags), CKP can reach FULL_SYNC after just **one gap** instead of two, speeding
cold-start.  The seed is cleared immediately after loading (one-shot policy).

### Status Byte (`status_bits.h`)

An 8-bit status field is included in TunerStudio RT data and CAN 0x400 frames:

| Bit | Mask | Name | Meaning |
|-----|------|------|---------|
| 0 | `0x01` | `STATUS_SYNC_FULL` | CKP FULL_SYNC active |
| 1 | `0x02` | `STATUS_PHASE_A` | CAM phase A detected |
| 2 | `0x04` | `STATUS_SENSOR_FAULT` | Any sensor fault active |
| 3 | `0x08` | `STATUS_LIMP_MODE` | Limp mode active |
| 4 | `0x10` | `STATUS_SCHED_LATE` | Scheduler late event(s) |
| 5 | `0x20` | `STATUS_SCHED_DROP` | Scheduler cycle drop(s) |
| 6 | `0x40` | `STATUS_SCHED_CLAMP` | Calibration clamp(s) fired |
| 7 | `0x80` | `STATUS_WBO2_FAULT` | WBO2 CAN timeout (> 500 ms) |

### Limp Mode

Activated when MAP **or** CLT has an active fault **and** RPM > 3000.
Action: injection PW is forced to 1000 µs (1 ms safe fallback); normal
fuel/ignition calculation is bypassed.

---

## Hardware Peripherals

| Peripheral | Usage |
|-----------|-------|
| FTM3 CH0 (PTD0, ALT4) | CKP input capture — tooth edge timing (PS=2, 60 MHz) |
| FTM3 CH1 (PTD1, ALT4) | CAM phase input capture |
| FTM0 CH0–7 (IRQ 42)   | Angle-domain output-compare: INJ3/4/1/2 + IGN4/3/2/1 (PS=64, 1.875 MHz) |
| FTM1 CH0 (PTA8)       | PWM IACV — 15 Hz |
| FTM1 CH1 (PTA9)       | PWM wastegate — 15 Hz |
| FTM2 CH0 (PTA10)      | PWM VVT exhaust — 15 Hz |
| FTM2 CH1 (PTA11)      | PWM VVT intake — 15 Hz |
| ADC0 (IRQ 39)         | MAP, MAF, TPS, O2, AN1–AN4 (12-bit, hardware avg ×4) |
| ADC1                  | CLT, IAT, fuel pressure, oil pressure |
| PDB0                  | ADC trigger synchronised to FTM0 (TRGSEL=0x8) |
| PIT0 (IRQ 68)         | 1 µs timestamp counter (`g_datalog_us`) |
| PIT1 (IRQ 69)         | 100 ms watchdog — triggers `system_reset()` |
| FlexCAN (PTA12/13)    | CAN bus @ 500 kbps (WBO2 lambda, diagnostic frames) |
| UART0 / USB CDC       | TunerStudio communication @ 115200 baud |
| FlexNVM / FlexRAM     | LTFT (256 B), knock map (64 B), sync seeds (256 B), calibration pages |
| CMP0                  | Knock detection — 6-bit DAC (VOSEL) threshold |

---

## Communication Protocols

### TunerStudio (USB CDC / UART0)

Firmware signature: `"OpenEMS_v1.1"`

| Command byte | Description | Response |
|---|---|---|
| `Q` | Signature query | `"OpenEMS_v1.1"` (13 bytes) |
| `H` | Signature query (legacy) | `"OpenEMS_v1.1"` |
| `S` | Firmware version string | `"OpenEMS_fw_1.1"` |
| `F` | Protocol version | `"001"` |
| `C` | Communication test | `0x00 0xAA` (2 bytes) |
| `A` / `O` | Real-time output channels | 64-byte block (see below) |
| `r` | Read calibration page | `r[page:u8][offset:u16 LE][len:u16 LE]` → data |
| `w` | Write calibration page | `w[page:u8][offset:u16 LE][len:u16 LE][data]` → `0x00` OK / `0x01` err |

**Real-time block layout (64 bytes):**

| Offset | Type | Field | Scale |
|--------|------|-------|-------|
| 0–1 | U16 LE | RPM | raw |
| 2 | U8 | MAP (kPa) | ÷10 |
| 3 | U8 | TPS (%) | ÷10 |
| 4 | S8 | CLT (°C) | value + 40 |
| 5 | S8 | IAT (°C) | value + 40 |
| 6 | U8 | Lambda O2 | λ×1000 ÷ 4 |
| 7 | U8 | Injector PW | ms×10 |
| 8 | U8 | Advance (°) | value + 40 |
| 9 | U8 | VE cell (%) | raw |
| 10 | S8 | STFT (%) | value + 100 |
| 11 | U8 | Status bits | see status_bits.h |
| 12–15 | U32 LE | `late_events` | counter |
| 16–19 | U32 LE | `late_max_delay_ticks` | FTM0 ticks |
| 20 | U8 | `queue_depth_peak` | — |
| 21 | U8 | `queue_depth_last_cycle` | — |
| 22–25 | U32 LE | `cycle_schedule_drops` | counter |
| 26–29 | U32 LE | `calibration_clamps` | counter |
| 30 | U8 | `sync_state_raw` | enum CkpSyncState |

**Calibration pages:**

| Page | Size | Content |
|------|------|---------|
| 0 | 512 bytes | General config; byte 0 = `ivc_abdc_deg` |
| 1 | 256 bytes | VE table 16×16 |
| 2 | 256 bytes | Spark advance table 16×16 |

Any structural change to page layout requires a version bump in
`tunerstudio/openems.ini` and corresponding offset updates in `tuner_studio.cpp`.

### CAN Bus

| CAN ID | Direction | Content |
|--------|-----------|---------|
| 0x180  | RX | WBO2 lambda (U16 LE bytes 0–1: λ×1000); 500 ms timeout |
| 0x400  | TX every 50 ms | Diagnostic A: RPM, MAP, TPS, CLT, advance, PW, status |
| 0x401  | TX every 100 ms | Diagnostic B: fuel/oil pressure, IAT, STFT, VVT |

WBO2 fallback on timeout: λ = 1050 (λ = 1.05).  WBO2 RX ID is configurable
at runtime via `can_stack_set_wbo2_rx_id()`.

---

## Testing Patterns

### Conditional Compilation

All test-only APIs are guarded by `EMS_HOST_TEST`:

```cpp
#ifdef EMS_HOST_TEST
void ckp_test_reset();
uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns);
void ecu_sched_test_reset(void);
uint8_t ecu_sched_test_angle_table_size(void);
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

Tests return `g_tests_failed` from `main()`.

### Writing New Tests

1. Create `test/<layer>/test_<module>.cpp`.
2. Include the production header and, if needed, any `EMS_HOST_TEST` helpers.
3. Define `int main()` that runs test cases and returns `g_tests_failed`.
4. Add the new binary to `scripts/run_host_tests.sh` using `run_test`.

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

### Scheduler Interaction Pattern

When changing injection or ignition parameters from the background loop:

```cpp
// 1. Compute parameters
const uint32_t advance_deg = ...;
const uint32_t dwell_ticks = (dwell_ms_x10 * ECU_FTM0_TICKS_PER_MS) / 100u;
const uint32_t inj_pw_ticks = (base_pw_us * ECU_FTM0_TICKS_PER_MS) / 1000u;

// 2. Commit atomically — the ISR reads a consistent snapshot
ecu_sched_commit_calibration(advance_deg, dwell_ticks, inj_pw_ticks, kDefaultSoiLeadDeg);
```

Never write individual `ecu_sched_set_*` fields from the background loop while
the scheduler is running; use `ecu_sched_commit_calibration()` instead.

---

*Last updated: 2026-03-09*
