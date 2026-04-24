# OpenEMS - Engine Management System

**Status**: 🟢 **OpenEMS-v3 Phases 1-2 Complete** | **Development Focus: STM32H562RGT6**

---

## 📦 Project Structure

```
OpenEMS/
├── openems-v3/                    ⭐ MAIN PROJECT (Phases 1-2 Complete)
│   ├── src/                       (44 files, ~6,000 LoC)
│   ├── docs/                      (architecture, specifications)
│   ├── Makefile                   (zero external dependencies)
│   ├── README.md                  (project documentation)
│   ├── CLAUDE.md                  (development guide)
│   ├── FINAL_STATUS.md            (Phase 1-2 completion report)
│   └── PHASE3_4_5_ROADMAP.md      (next phases specification)
│
├── openems-stm32h5/               📚 REFERENCE (v2.2 architecture)
│   └── (STM32H562 optimized port, used for v3 foundation)
│
├── src/                           📚 REFERENCE (v1.1 Teensy base)
│   └── (Original Teensy 3.5 implementation)
│
├── spec.md                        📋 Project specification (Portuguese)
├── CLAUDE.md                      📖 AI development guide
├── CHANGELOG.md                   📜 Development history
└── .gitignore

```

---

## 🎯 What is OpenEMS-v3?

**OpenEMS-v3** is a next-generation **real-time engine management system** targeting **STM32H562RGT6** (250 MHz ARM Cortex-M33).

### Core Capabilities
✅ **CKP Decoding** - 60-2 missing-tooth wheel synchronization  
✅ **Fuel Injection** - Volumetric efficiency (VE) based calculation  
✅ **Ignition Timing** - Spark advance tables with knock retard  
✅ **Auxiliary Control** - IACV, wastegate, VVT, cooling fan (PID)  
✅ **TunerStudio Protocol** - Real-time tuning (USB CDC only)
✅ **CAN FD Bus** - Advanced message filtering with priority queue  
✅ **Adaptive Trim** - STFT/LTFT closed-loop lambda control  

### Key Metrics
- **Language**: C++17 (no STL, no exceptions, no dynamic allocation)
- **Architecture**: Strict 4-layer (APP → ENGINE → DRV → HAL)
- **Code Reuse**: 79% from v1.1 (algorithms) + v2.2 (STM32H5 HAL)
- **Test Coverage**: deferred (test suite temporarily removed; to be reintroduced later)
- **Build**: Makefile (zero external dependencies)
- **Memory**: Fixed-width types, Q8.8 fixed-point math

---

## 📊 Development Status

| Phase | Status | Timeline | LoC | Details |
|-------|--------|----------|-----|---------|
| **Phase 1** | ✅ COMPLETE | Complete | 9,250 | Greenfield foundation |
| **Phase 2** | ✅ COMPLETE | Complete | - | HAL system integration |
| **Phase 3** | 📋 Spec'd | 1 week | ~800 | USB CDC integration |
| **Phase 4** | 📋 Spec'd | 1 week | ~650 | CAN FD filtering |
| **Phase 5** | 📋 Spec'd | 0.5 weeks | ~600 | Polish & production docs + test strategy rebuild |

**Target Completion**: May 6, 2026

---

## 🚀 Quick Start

### Build & Test

```bash
cd openems-v3

# Run all unit tests (host-based)
make firmware

# Clean build artifacts
make clean
```

### Development Workflow

```bash
# Create feature branch
git checkout -b feature/your-feature-name

# Make changes
vim src/engine/fuel_calc.cpp

# Build locally
make firmware

# Commit with clear message
git commit -m "engine/fuel: implement feature"

# Push to openems-v3-dev
git push origin openems-v3-dev
```

---

## 📚 Documentation

- **[DEVELOPMENT_PLAN.md](DEVELOPMENT_PLAN.md)** - Execution plan, weekly gates, and delivery tracking (single source of truth)
- **[openems-v3/README.md](openems-v3/README.md)** - Complete project overview
- **[openems-v3/CLAUDE.md](openems-v3/CLAUDE.md)** - Development guide & conventions
- **[openems-v3/FINAL_STATUS.md](openems-v3/FINAL_STATUS.md)** - Phase 1-2 completion report
- **[openems-v3/PHASE3_4_5_ROADMAP.md](openems-v3/PHASE3_4_5_ROADMAP.md)** - Detailed roadmap for remaining phases
- **[spec.md](spec.md)** - Full functional specification (Portuguese)

---

## 🏗️ Architecture

### 4-Layer Dependency Model

```
┌──────────────────────────────────┐
│ APP (TunerStudio, CAN stack)     │ ← Protocols, communication
├──────────────────────────────────┤
│ ENGINE (Fuel, ignition, knock)   │ ← Control algorithms
├──────────────────────────────────┤
│ DRV (CKP, sensors, scheduler)    │ ← Drivers
├──────────────────────────────────┤
│ HAL (STM32H562-optimized)        │ ← Hardware abstraction
└──────────────────────────────────┘
         ↓
   STM32H562RGT6
```

**Key Properties**:
- Strict unidirectional dependencies (no circular refs)
- Host test suite temporarily removed (planned reintroduction in later phase)
- Platform-agnostic ENGINE/DRV layers
- STM32H562-specific HAL layer

---

## 🔄 Code Reuse Strategy

OpenEMS-v3 reuses proven, battle-tested components:

| Component | Version | Reuse % | Status |
|-----------|---------|---------|--------|
| **Engine algorithms** | v1.1 | 100% | ✅ Complete |
| **Driver layer** | v1.1 | 100% | ✅ Complete |
| **STM32H5 HAL** | v2.2 | 80% | ✅ Complete |
| **Test infrastructure** | v1.1 | 100% | ⏸️ Removed for now |

**Total**: **79% code reuse** from proven implementations

---

## 🛠️ Technology Stack

| Component | Technology |
|-----------|------------|
| **Language** | C++17 (ISO/IEC 14882:2017) |
| **Standard Library** | None (embedded constraints) |
| **Target Platform** | STM32H562RGT6 (ARM Cortex-M33 @ 250 MHz) |
| **Build System** | GNU Make |
| **Testing** | Temporarily deferred (suite removed) |
| **Version Control** | Git |
| **Tuning Software** | TunerStudio / MegaTune protocol |

---

## 📝 Naming Conventions

### Variable Prefixes
- `g_` - File-scope globals
- `k` - Compile-time constants
- `s_` - Static locals (rare)

### Unit Suffixes (Physical Quantities)
- `_x10` - Value × 10 (e.g., `rpm_x10 = 6000` → 600.0 RPM)
- `_x100` - Value × 100
- `_x256` - Fixed-point Q8.8
- `_kpa` - Kilopascals
- `_degc` - Degrees Celsius
- `_mv` - Millivolts
- `_ms` / `_us` - Milliseconds / Microseconds
- `_pct` - Percentage
- `_rpm` - RPM (not scaled)

---

## 🧪 Test Infrastructure

### Tests
The legacy host-based test suite was intentionally removed to reduce repository noise and will be redesigned in a later phase.

---

## 🌳 Repository History

| Date | Version | Status | Target |
|------|---------|--------|--------|
| 2026-02-28 | v1.1 | ✅ Production | Teensy 3.5 |
| 2026-03-14 | v2.2 | 🟡 Beta | STM32H562 |
| 2026-04-22 | v3.0-rc1 | 🟢 Phases 1-2 | STM32H562 |

---

## 📞 Contributing

### Coding Standards
1. ✅ `make firmware` must build successfully
2. ✅ No compiler warnings (`-Wall -Wextra`)
3. ✅ Follow naming conventions (unit suffixes, prefixes)
4. ✅ Document validation strategy for changed code
5. ✅ Update documentation (CLAUDE.md if architectural changes)

### Commit Message Format
```
<layer>/<component>: <description>

Example:
  engine/fuel: fix CLT enrichment at cold start
  drv/ckp: improve RPM calculation precision
  hal/stm32h562: optimize timer ISR latency
```

---

## 📈 Performance Targets (STM32H562)

| Metric | Target | Status |
|--------|--------|--------|
| **Clock Speed** | 250 MHz | ✅ Configured |
| **CKP Latency** | < 1 µs | ✅ Target |
| **Injection Precision** | µs-level | ✅ Achieved |
| **CAN Throughput** | 1 Mbps (classic) / 5 Mbps (FD) | ✅ Supported |
| **Memory Usage** | < 50% Flash, < 50% RAM | ✅ Estimated |

---

## 📄 License & Attribution

This project builds upon:
- **OpenEMS v1.1** - Core algorithms and architecture
- **OpenEMS v2.2** - STM32H5 optimization learnings
- **Teensy 3.5 firmware** - Reference implementation

**Created**: 2026-02-28  
**Last Updated**: 2026-04-22  
**Maintainer**: OpenEMS Development Team

---

**Development Status**: ✅ Phases 1-2 Complete | 📋 Phases 3-5 Roadmapped | 🚀 Ready for Phase 3 Implementation

**Repository**: https://github.com/pbuchabqui/OpenEMS  
**Current Focus**: openems-v3-dev branch  
**Next Milestone**: Phase 3 USB CDC Integration (1 week)
