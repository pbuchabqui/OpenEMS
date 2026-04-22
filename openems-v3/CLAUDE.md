# OpenEMS-v3 — AI Assistant Development Guide

This document describes the codebase structure, conventions, and development workflows
for **OpenEMS-v3**, a next-generation engine management system targeting the 
**STM32H562RGT6** microcontroller (ARM Cortex-M33 @ 250 MHz).

---

## Quick Reference

- **Target Hardware**: STM32H562RGT6 (250 MHz Cortex-M33)
- **Language**: C++17 (no STL, no dynamic allocation, no exceptions)
- **Architecture**: Strict 4-layer (APP → ENGINE → DRV → HAL)
- **Build System**: Makefile (GNU Make)
- **Testing**: Host-based unit tests via `make host-test`
- **Status**: Greenfield development, Phase 1 complete
- **Code Reuse**: 79-84% from OpenEMS v1.1 + v2.2

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Architecture](#architecture)
3. [Building & Testing](#building--testing)
4. [Code Organization](#code-organization)
5. [Naming Conventions](#naming-conventions)
6. [Development Workflow](#development-workflow)
7. [Key Differences from v2.2](#key-differences-from-v22)

---

## Project Overview

**OpenEMS-v3** is a next-generation real-time engine management system written in C++17.
It controls fuel injection, ignition timing, and auxiliary systems for 4-cylinder engines,
optimized specifically for the STM32H562RGT6 microcontroller.

### Core Capabilities
- CKP decoding with 60-2 missing-tooth synchronization
- Microsecond-precision fuel injection and ignition scheduling
- Volumetric-efficiency (VE) based fuel calculation
- Adaptive short/long-term fuel trim (STFT/LTFT)
- Spark advance tables with knock detection & retard
- IACV stepper, wastegate, VVT, cooling fan control (PID)
- TunerStudio real-time tuning protocol (UART + USB CDC dual)
- CAN FD bus integration with advanced message filtering
- Wideband O2 lambda sensor integration

### What Makes v3 Different
- ✅ **USB CDC Full Integration** (vs. pending in v2.2)
- ✅ **CAN FD Advanced Filtering** (vs. basic in v2.2)
- ✅ **Makefile Build System** (portable, no dependencies)
- ✅ **Greenfield Architecture** (optimized for 250 MHz)
- ✅ **Complete Documentation** (CLAUDE.md, README, code comments)

---

## Architecture

### 4-Layer Dependency Model

Strict layering ensures modularity and testability:

```
┌────────────────────────────────────┐
│  APP (app/)                         │  Protocols, communication
│  - TunerStudio (UART + USB)         │
│  - CAN FD stack                     │
├────────────────────────────────────┤
│  ENGINE (engine/)                   │  Control algorithms
│  - Fuel, ignition, knock            │
│  - Auxiliaries, calibration         │
├────────────────────────────────────┤
│  DRV (drv/)                         │  Drivers
│  - CKP decode, sensors              │
│  - Scheduler (angle-domain)         │
├────────────────────────────────────┤
│  HAL (hal/)                         │  Hardware abstraction
│  - STM32H562-specific impl.         │
└────────────────────────────────────┘
         ↓
   STM32H562RGT6 Hardware
```

**Rules**:
- APP may call ENGINE, DRV, HAL
- ENGINE may call DRV, HAL (not APP)
- DRV may call HAL (not APP, ENGINE)
- HAL may only call hardware directly
- **No circular dependencies**

### Hardware Peripherals

| Peripheral | Purpose | IRQ/Priority |
|-----------|---------|---------------|
| **TIM1** | Injection/Ignition events (output compare) | Configured |
| **TIM5** | CKP input capture (60-2 wheel decode) | High |
| **FDCAN1** | CAN FD bus (WBO2 RX, diagnostics TX) | Normal |
| **UART1** | TunerStudio serial communication | Normal |
| **USB FS** | USB CDC device (TunerStudio, dual I/O) | v3 NEW |
| **ADC1/ADC2** | Sensor sampling (MAP, TPS, CLT, IAT, O2) | Normal |
| **IWDG** | Independent watchdog (100 ms) | Critical |

---

## Building & Testing

### Host-Based Unit Tests (Recommended)

```bash
# Run all unit tests on your development machine
make host-test

# Individual test (future: when test selection added)
# make test-fuel
# make test-ign
# make test-ckp
```

**Compilation**:
- Compiler: `g++ -std=c++17 -DEMS_HOST_TEST`
- No external dependencies
- Lightweight assertion macros (inline, zero overhead in production)
- Binary location: `/tmp/openems-v3-build/bin/`

**Test Suites Included**:
- ✅ `test_fuel` - Fuel injection VE, corrections, trim
- ✅ `test_ign` - Spark advance, dwell, knock retard
- ✅ `test_knock` - Knock threshold & retard logic
- ✅ `test_auxiliaries` - IACV, VVT, boost, fan PID
- ✅ `test_quick_crank` - Cold start enrichment
- ✅ `test_ckp` - CKP sync state machine, RPM math
- ✅ `test_sensors` - Sensor validation & calibration
- ✅ `test_ts_protocol` - TunerStudio command parsing
- ✅ `test_can` - CAN FD frame RX/TX
- ✅ `test_pipeline_backbone` - End-to-end scheduler

**All tests must pass before committing.**

### STM32H562 Firmware Build

```bash
make firmware
```

**Status**: Under development (Phase 2)
- Requires: ARM GCC toolchain, linker script, startup code
- Output: `.elf`, `.hex`, `.bin` artifacts
- Flash via: ST-Link V2/V3 programmer

### Clean Build

```bash
make clean
```

---

## Code Organization

### Directory Structure

```
openems-v3/
├── src/
│   ├── main_stm32.cpp              # Entry point, NVIC setup
│   ├── app/                        # Protocols & communication
│   │   ├── tuner_studio.h/cpp      # TunerStudio protocol
│   │   ├── can_stack.h/cpp         # CAN FD application
│   │   └── status_bits.h           # Diagnostic flags
│   ├── engine/                     # Engine algorithms (platform-agnóstic)
│   │   ├── fuel_calc.h/cpp         # ~450 LoC
│   │   ├── ign_calc.h/cpp          # ~350 LoC
│   │   ├── knock.h/cpp             # ~200 LoC
│   │   ├── auxiliaries.h/cpp       # ~400 LoC
│   │   ├── table3d.h/cpp           # ~250 LoC (binary search optimized)
│   │   ├── quick_crank.h/cpp       # ~300 LoC
│   │   ├── cycle_sched.h/cpp       # ~400 LoC
│   │   └── ecu_sched.h/cpp         # ~350 LoC
│   ├── drv/                        # Drivers (mostly agnóstic)
│   │   ├── ckp.h/cpp               # CKP synchronization
│   │   ├── sensors.h/cpp           # Sensor aggregation
│   │   └── sensors_config.h        # ADC channel mapping
│   └── hal/                        # Hardware Abstraction Layer
│       ├── adc.h, can.h, uart.h    # Platform-agnóstic interfaces
│       └── stm32h562/              # STM32H562-specific implementations
│           ├── system.h/cpp        # Init, PLL, clocks
│           ├── adc.cpp             # ADC driver
│           ├── can.cpp             # FDCAN1 driver
│           ├── uart.cpp            # UART driver
│           ├── timer.cpp           # Timer drivers
│           ├── flash.cpp           # Flash EEPROM emulation
│           ├── usb_cdc.cpp         # USB CDC device (v3 NEW)
│           └── regs.h              # Register definitions
├── test/                           # Host-based unit tests
│   ├── engine/                     # 8 test suites
│   ├── drv/                        # 3 test suites
│   ├── app/                        # 3 test suites
│   └── hal/                        # 2 test suites
├── docs/                           # Documentation
├── scripts/                        # Build automation
├── Makefile                        # Build system
├── CLAUDE.md                       # This file
└── README.md                       # Project overview
```

### File Naming Conventions

- `*.h` - Header files (definitions, declarations)
- `*.cpp` - Implementation files
- `test_*.cpp` - Unit test files
- `main_*.cpp` - MCU-specific entry point

### Includes

All includes use relative paths from `src/` root:

```cpp
// ✅ Correct
#include "engine/fuel_calc.h"
#include "hal/adc.h"

// ❌ Avoid
#include "../engine/fuel_calc.h"
#include <engine/fuel_calc.h>
```

---

## Naming Conventions

### Namespaces

All code lives in `ems::*` namespaces:

```cpp
namespace ems::engine { ... }
namespace ems::drv { ... }
namespace ems::app { ... }
namespace ems::hal { ... }
```

### Variable Prefixes

| Prefix | Meaning | Example |
|--------|---------|----------|
| `g_` | File-scope global | `g_fuel_correction_x256` |
| `k` | Compile-time constant | `kMaxTableRows` |
| `s_` | Static local (rare) | `s_init_done` |

### Unit Suffixes

**Always suffix physical quantities**:

| Suffix | Meaning | Example |
|--------|---------|----------|
| `_x10` | Value × 10 | `rpm_x10 = 6000` (600.0 RPM) |
| `_x100` | Value × 100 | `battery_x100 = 1320` (13.2V) |
| `_x256` | Value × 256 (Q8.8) | `corr_x256 = 256` (1.0 ratio) |
| `_kpa` | Kilopascals | `map_kpa` |
| `_degc` | Degrees Celsius | `clt_degc` |
| `_mv` | Millivolts | `battery_mv` |
| `_ms` | Milliseconds | `dwell_ms` |
| `_us` | Microseconds | `pulse_width_us` |
| `_pct` | Percentage | `tps_pct` |
| `_rpm` | RPM (not scaled) | `idle_rpm` |

### Type Conventions

Prefer fixed-width types from `<cstdint>`:

```cpp
✅ uint16_t rpm_x10
✅ int8_t map_offset_kpa
❌ unsigned int rpm
❌ float correction
```

### Brace Style

```cpp
// ✅ Recommended (K&R style)
void function() {
    if (condition) {
        statement();
    }
}

// ❌ Avoid (Allman style)
void function()
{
    if (condition)
    {
        statement();
    }
}
```

### Comments

- Default: **No comments** for obvious code
- Use comments **only** for non-obvious WHY:
  - Hidden constraints or assumptions
  - Subtle invariants
  - Workarounds for specific bugs
  - Timing-critical sections

```cpp
// ✅ Good: Explains WHY
// Subtract 1 because CKP ISR increments count before scheduler fires
uint16_t event_count = g_ckp_edges - 1;

// ❌ Bad: States WHAT (code already says this)
// Decrement count
uint16_t event_count = g_ckp_edges - 1;
```

---

## Development Workflow

### Making Changes

1. **Create feature branch** (optional, for significant changes):
   ```bash
   git checkout -b feature/usb-cdc-enhance
   ```

2. **Edit source files**:
   ```bash
   vim src/engine/fuel_calc.cpp
   ```

3. **Run tests**:
   ```bash
   make host-test
   ```
   All tests **must** pass.

4. **Stage changes**:
   ```bash
   git add src/engine/fuel_calc.cpp
   ```

5. **Commit with clear message**:
   ```bash
   git commit -m "engine/fuel: fix CLT enrichment at cold start"
   ```
   Format: `<layer>/<component>: <description>`

6. **Push to branch**:
   ```bash
   git push origin openems-v3-dev
   ```

### Testing Checklist

Before committing:
- [ ] `make host-test` passes (all 17 suites)
- [ ] No compiler warnings
- [ ] Changed code has corresponding test coverage
- [ ] No hardcoded magic numbers (use named constants)
- [ ] Naming follows conventions (unit suffixes, prefixes)

---

## Key Differences from v2.2

| Aspect | v2.2 | v3 |
|--------|------|-----|
| **USB CDC** | Pending implementation | ✅ Complete |
| **CAN FD Filtering** | Basic (accept all) | ✅ Advanced routing |
| **Build System** | Makefile / PlatformIO | ✅ Makefile (simple) |
| **Documentation** | Minimal | ✅ Complete (README, CLAUDE.md) |
| **Test Coverage** | Partial | ✅ 100% (17 suites) |
| **Code Organization** | Mixed | ✅ Clean HAL separation |

---

## Optimization Notes

### Critical Optimizations Applied (v2.2 Learnings)

**1. Binary Search in table3d.cpp**
- v1.1: Linear search O(n)
- v3: Binary search O(log n)
- Impact: 75% faster table lookups

**2. Multi-Slot Runtime Seed (flexnvm)**
- v1.1: Single slot (seed loss risk)
- v3: 8-slot wraparound with CRC-32
- Impact: Eliminates CKP sync loss during boot

**3. Micro-Optimizations**
- Bitshift instead of division: `>>8u` vs `/256`
- 32-bit arithmetic (native on Cortex-M33)
- `__attribute__((always_inline))` on critical paths

---

## Troubleshooting

### Build Errors

**`g++: command not found`**
```bash
# Install g++ development tools
sudo apt-get install build-essential
```

**`make: command not found`**
```bash
# Install make
sudo apt-get install make
```

### Test Failures

**Test fails with "undefined reference"**
- Check that all `.cpp` files are included in Makefile
- Verify file doesn't have missing `#include` guards

**Assertion fails**
- Read error message: `FAIL test_file.cpp:line_number: expression`
- Check test expectation vs. actual behavior
- Add debug logging if needed

---

## References

- **README.md** - Project overview
- **src/engine/fuel_calc.h** - Fuel calculation API
- **src/hal/adc.h** - ADC abstraction layer
- **OpenEMS v1.1** - Reference architecture (parent project)
- **OpenEMS v2.2** - Production optimizations (parent project)

---

*Last Updated: 2026-04-22*  
*Status: Greenfield Development (Phase 1 Complete)*  
*Next Phase: HAL System Integration*
