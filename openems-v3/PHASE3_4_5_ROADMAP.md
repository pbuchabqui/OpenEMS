# OpenEMS-v3: Phases 3, 4, 5 Roadmap

**Status**: Foundation Complete (Phases 1-2)  
**Next Milestones**: Phases 3-5 Implementation  
**Timeline**: 2.5 weeks total (Phase 3: 1 week, Phase 4: 1 week, Phase 5: 0.5 weeks)  

---

## Phase 3: USB CDC Full Integration (1 week)

### Objectives
- ✅ Complete USB CDC device class implementation (stub in place)
- ✅ Dual I/O: UART + USB simultaneous operation
- ✅ TunerStudio protocol over USB

### Files to Create/Modify

**New Files**:
```
src/hal/stm32h562/usb_cdc.cpp  (~500 LoC)
  - USB device controller initialization
  - Endpoint configuration (control, bulk IN/OUT)
  - CDC-ACM class handlers
  - Data transmission/reception state machines

src/hal/stm32h562/usb_interrupts.cpp  (~300 LoC)
  - USB interrupt handlers (reset, setup, data)
  - Endpoint interrupt processing
```

**Modified Files**:
```
src/main_stm32.cpp
  - Add usb_cdc_init() call in system initialization
  - Add usb_cdc_poll() in main background loop

src/app/tuner_studio.cpp  
  - Support both UART and USB CDC I/O
  - Auto-detect which interface is active
  - Maintain protocol compatibility on both transports
```

### Key Features
- ✅ Automatic baud rate detection
- ✅ DTR/RTS signal handling
- ✅ Packet framing (no character loss)
- ✅ Dual I/O switching (use UART if no USB, switch to USB when connected)

### Testing
- Host test: usb_cdc mock (no hardware needed)
- Hardware test: connect STM32H562 to PC via USB
- Verify TunerStudio communication over USB

### Acceptance Criteria
- [ ] `make host-test` passes with USB CDC mocks
- [ ] `make firmware` compiles STM32H562 with USB support
- [ ] TunerStudio connects and communicates over USB
- [ ] Fallback to UART when USB disconnected

---

## Phase 4: CAN FD Advanced Filtering (1 week)

### Objectives
- ✅ Implement advanced FDCAN1 message filtering
- ✅ Priority-based message routing
- ✅ CAN FD with extended features (FDF, BRS, ESI bits)
- ✅ Improved diagnostics frame handling

### Files to Create/Modify

**New Files**:
```
src/app/can_filters.cpp  (~400 LoC)
  - Filter configuration database
  - Priority queue for message processing
  - Extended frame ID support
  - Global and group filtering

src/hal/stm32h562/can_fd_config.cpp  (~250 LoC)
  - FDCAN peripheral configuration (250 MHz clock)
  - Bit timing for CAN and CAN FD
  - Message RAM organization
  - Filter element storage
```

**Modified Files**:
```
src/hal/can.cpp
  - Add can_filters_init() integration
  - Support extended filtering callbacks

src/hal/stm32h562/can.cpp
  - FDCAN register configuration
  - Frame reception with FDF/BRS/ESI
```

### Key Features
- ✅ 16 hardware message filters (configurable)
- ✅ Dual-mode filtering (classic CAN + CAN FD)
- ✅ Priority-based routing (LOW, NORMAL, HIGH, CRITICAL)
- ✅ Global masking + individual ID filtering

### Testing
- Host test: can_filters module (packet routing)
- Hardware test: WBO2 sensor frames + diagnostic frames
- Verify frame filtering under load (multi-frame reception)

### Acceptance Criteria
- [ ] `make host-test` includes can_filters tests
- [ ] Priority-based frame routing working
- [ ] CAN FD extended frames (29-bit ID) supported
- [ ] Diagnostic frames routed correctly

---

## Phase 5: Polish & Production Documentation (0.5 weeks)

### Objectives
- ✅ Complete build system refinement
- ✅ Hardware flashing procedures
- ✅ Production-ready documentation
- ✅ Release preparation

### Files to Create/Modify

**New Files**:
```
scripts/flash_stm32h562.sh  (~100 lines)
  - OpenOCD configuration for STM32H562
  - Firmware flashing via ST-Link
  - Verification checksums

docs/BUILD_INSTRUCTIONS.md  (~200 lines)
  - Prerequisites & environment setup
  - Compilation steps
  - Troubleshooting guide

docs/DEPLOYMENT.md  (~300 lines)
  - Hardware connection diagram
  - CAN termination configuration
  - USB connection guide
  - UART fallback procedures

openems-v3/VERSION.txt
  - Version string: "3.0.0-rc1"
  - Build metadata
```

**Modified Files**:
```
README.md
  - Update with Phases 3-5 completion status
  - Link to new documentation

CLAUDE.md
  - Add USB CDC development notes
  - CAN FD filtering architectural details

Makefile
  - Add firmware-flash target
  - Add firmware-verify target
  - Optimize build for release

PHASE3_4_5_ROADMAP.md (this file)
  - Mark completed items
```

### Key Documentation
- ✅ Getting Started guide (5 minutes to first compile)
- ✅ Troubleshooting FAQ
- ✅ Architecture deep-dive
- ✅ Performance benchmarks (timing, CAN throughput)

### Testing
- CI/CD pipeline validation (`make host-test` always passing)
- Release candidate testing on hardware
- Long-duration stability tests (24+ hours continuous CKP decode)

### Acceptance Criteria
- [ ] `make firmware` creates release binary
- [ ] `./scripts/flash_stm32h562.sh` flashes device
- [ ] All documentation generated (HTML, markdown, PDF)
- [ ] Version bump committed (`3.0.0-rc1`)
- [ ] Release notes generated (CHANGELOG.md updated)

---

## Implementation Priority

### High Priority (Do First)
1. **Phase 3 USB CDC** - Enables wireless TunerStudio (very valuable)
2. **Phase 4 CAN FD** - Improves reliability & throughput
3. **Phase 5 Documentation** - Enables community use

### Medium Priority (Do Second)
- Performance benchmarking
- Extended hardware testing
- Compatibility validation with various CAN/USB interfaces

### Low Priority (Polish Later)
- Optimization passes
- Extended error handling
- Additional diagnostic features

---

## Success Criteria (All Phases)

### Functional
- [ ] `make host-test` passes all 17 test suites
- [ ] `make firmware` builds without errors/warnings
- [ ] `./scripts/flash_stm32h562.sh` flashes hardware
- [ ] TunerStudio connects via USB AND UART
- [ ] CAN bus communicates (WBO2 frames received correctly)
- [ ] Fuel/ignition calculations validated on hardware

### Non-Functional
- [ ] Code compiles with `-Wall -Wextra` (no warnings)
- [ ] Memory usage < 50% of STM32H562 capacity
- [ ] CKP synchronization latency < 1 microsecond
- [ ] Build time < 10 seconds

### Documentation
- [ ] README.md complete & up-to-date
- [ ] CLAUDE.md covers all features
- [ ] Build instructions clear & tested
- [ ] Hardware connection diagrams provided
- [ ] Troubleshooting FAQ published

---

## Estimated Effort

| Phase | Component | LoC | Time | Complexity |
|-------|-----------|-----|------|-----------|
| 3 | USB CDC device | 500 | 3 days | High |
| 3 | USB integration | 300 | 2 days | Medium |
| 4 | CAN FD filtering | 400 | 3 days | High |
| 4 | FDCAN config | 250 | 2 days | Medium |
| 5 | Docs & scripts | 600 | 2 days | Low |
| 5 | Testing & release | 0 | 1 day | Medium |
| **TOTAL** | **All** | **~2,050** | **~13 days** | **High** |

---

## Known Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| USB timing/buffering issues | Early hardware testing, exhaustive unit tests |
| CAN FD bit timing miscalculation | Reference FDCAN spec, validate with oscilloscope |
| Memory exhaustion | Profile memory usage, optimize data structures |
| Build system complexity | Keep Makefile simple, test on multiple platforms |

---

## Next Immediate Steps (After Phase 2)

1. **Start Phase 3** (next iteration):
   - Implement USB CDC in `src/hal/stm32h562/usb_cdc.cpp`
   - Create USB initialization & interrupt handlers
   - Integrate with main.cpp and TunerStudio protocol

2. **Parallel Phase 4** (during Phase 3):
   - Design CAN filter database (priority queue)
   - Implement FDCAN configuration
   - Validate CAN FD timing

3. **Phase 5** (final week):
   - Consolidate all documentation
   - Create build/flash/test scripts
   - Prepare release package

---

## References

- STM32H562 Reference Manual (ARM M33 core, USB FS, FDCAN)
- USB 2.0 Specification (CDC-ACM class)
- CAN FD Protocol Specification (Bosch ISO 20498)
- TunerStudio Protocol Documentation
- OpenEMS v2.2 USB CDC stub (reference)

---

**Status**: Roadmap Complete  
**Created**: 2026-04-22  
**Target Completion**: 2026-05-06 (2 weeks)  
**Maintainer**: OpenEMS-v3 Development Team
