# OpenEMS Improvement Plan

## Executive Summary

This document outlines critical improvements for the OpenEMS engine control unit firmware targeting STM32H562RGT6. Issues are prioritized by risk level (P0-P3) with actionable implementation guidance.

---

## P0: Critical Bug Fixes (Engine Damage Risk)

### 1. TIM8 16-bit Overflow in Ignition Scheduling

**Location:** `src/engine/ecu_sched.cpp:313`

**Problem:**
```cpp
*ccr = (uint16_t)((TIM8_CNT & 0xFFFFU) + delta);
```

TIM8 is configured as 32-bit timer (`TIM8_ARR = 0xFFFFU` sets 16-bit mode), but the scheduling logic assumes 32-bit operation. When `TIM8_CNT` approaches 0xFFFF and wraps, the calculated CCR value will be incorrect, causing:
- Missed ignition events
- Potential spark at wrong cylinder position
- Engine damage from misfire

**Fix:**
```cpp
// Option A: Use full 32-bit range (recommended for ignition)
TIM8_ARR = 0xFFFFFFFFU;  // Line 376

// Option B: Properly handle 16-bit wraparound
uint32_t current_cnt = TIM8_CNT & 0xFFFFU;
uint32_t target = current_cnt + delta;
if (target > 0xFFFFU) {
    target -= 0x10000U;  // Wrap within 16-bit range
}
*ccr = static_cast<uint16_t>(target);
```

**Verification:** Check TIM8 initialization matches scheduling assumptions throughout `ecu_sched.cpp`.

---

### 2. Flash Writes During Engine Operation

**Location:** `src/main_stm32.cpp:633-660`, `src/hal/flash.cpp:225-311`

**Problem:**
- `nvm_flush_adaptive_maps()` executes every 500ms regardless of engine state
- Flash erase/program operations can take several milliseconds
- Per errata ES0565: first Flash operation after power-on freezes CPU for ~120µs
- Read-while-write increases latency during motor operation

**Risk:** Flash write latency during critical ignition/injection windows causes timing jitter or missed events.

**Fix:**
```cpp
// In main_stm32.cpp main loop
static bool adaptive_flush_pending = false;

// Only schedule flush when engine stopped or at safe RPM
if (elapsed(now, g_t500ms_, 500u)) {
    g_t500ms_ = now;
    
    // FIX: Only flush when engine is stopped or below cranking speed
    const bool engine_running = (snap.rpm_x10 > 3000u);  // >300 RPM
    
    if (!engine_running && adaptive_flush_pending) {
        adaptive_flush_pending = !ems::hal::nvm_flush_adaptive_maps();
    } else if (!adaptive_flush_pending && g_ltft_dirty) {
        // Mark for later flush when engine stops
        adaptive_flush_pending = true;
    }
}

// Force flush on engine stop
if (g_engine_was_running && snap.rpm_x10 == 0u) {
    adaptive_flush_pending = true;
    // Block until flush completes (safe when engine stopped)
    while (adaptive_flush_pending) {
        adaptive_flush_pending = !ems::hal::nvm_flush_adaptive_maps();
    }
}
```

**Additional Protection:** Add runtime check in `nvm_flush_adaptive_maps()`:
```cpp
bool nvm_flush_adaptive_maps() noexcept {
    // Prevent flash writes above safe RPM threshold
    extern uint32_t get_current_rpm_x10();  // From ckp_snapshot
    if (get_current_rpm_x10() > 3000u) {   // >300 RPM
        return false;  // Defer flash operation
    }
    // ... existing implementation
}
```

---

### 3. ADC Recovery Sequence Not Verified

**Location:** `src/drv/sensors.cpp` (ADC recovery after timeout)

**Problem:** After ADC timeout, recovery sequence is initiated but success is not verified before using sensor data. This can lead to:
- Stale sensor readings (old MAP/TPS values)
- Incorrect fuel calculations
- Unsafe air/fuel ratios

**Fix:**
```cpp
// Add recovery status flag
static volatile bool g_adc_recovering = false;
static volatile bool g_adc_recovery_failed = false;

// In ADC timeout handler
void adc_timeout_isr() {
    g_adc_recovering = true;
    g_adc_recovery_failed = false;
    
    // Initiate recovery sequence per errata
    ADC_CR |= ADC_CR_ADSTP;  // Stop conversion
    // Wait for EOSMP flag
    ADC_CR |= ADC_CR_ADDIS;  // Disable ADC
    // Wait for ADREADY flag
    ADC_CR |= ADC_CR_ADEN;   // Re-enable ADC
    
    // Verify recovery success
    for (uint32_t i = 0; i < kAdcRecoveryTimeout; ++i) {
        if (ADC_ISR & ADC_ISR_ADRDY) {
            g_adc_recovering = false;
            return;
        }
    }
    g_adc_recovery_failed = true;
}

// In sensor read functions
uint16_t get_map_kpa() {
    if (g_adc_recovering || g_adc_recovery_failed) {
        return kMapDefaultLimpHome;  // Use safe default
    }
    // ... normal reading
}
```

---

## P1: Architecture Improvements

### 4. Critical Section RAII Wrapper

**Problem:** Manual `enter_critical()`/`exit_critical()` calls are error-prone. Early returns cause deadlocks.

**Current Pattern (Error-Prone):**
```cpp
enter_critical();
if (condition) {
    exit_critical();
    return false;  // Easy to forget exit_critical()
}
// ... more code
exit_critical();
```

**Solution: Create `src/hal/critical_section.h`:**
```cpp
#pragma once
#include <cstdint>

namespace ems::hal {

class CriticalSectionGuard {
public:
    CriticalSectionGuard() noexcept {
#if defined(__arm__) || defined(__thumb__)
        asm volatile("cpsid i" ::: "memory");
#endif
    }
    
    ~CriticalSectionGuard() noexcept {
#if defined(__arm__) || defined(__thumb__)
        asm volatile("cpsie i" ::: "memory");
#endif
    }
    
    // Non-copyable, non-movable
    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard(CriticalSectionGuard&&) = delete;
    CriticalSectionGuard& operator=(CriticalSectionGuard&&) = delete;
};

} // namespace ems::hal
```

**Usage:**
```cpp
void ecu_schedule_event(uint8_t ch, uint32_t delta) {
    ems::hal::CriticalSectionGuard guard;
    
    if (tim_ch == 0U) { 
        ++g_cycle_schedule_drop_count; 
        return;  // Automatically exits critical section
    }
    
    // ... rest of function
}  // guard destructor runs here
```

**Migration:** Replace all manual critical sections in:
- `src/engine/ecu_sched.cpp` (lines 155-162, 288-315, 494-503)
- `src/drv/ckp.cpp` (lines 291-301, 391-397)

---

### 5. Magic Number Replacement

**Locations:** Throughout codebase

**Critical Constants to Define:**

```cpp
// src/engine/constants.h
#pragma once
#include <cstdint>

namespace ems::engine {

// Timing constants
inline constexpr uint32_t kTim5TickPeriodNs = 16u;        // 62.5 MHz → 16 ns/tick
inline constexpr uint32_t kSchedulerTickPeriodNs = 100u;  // 10 MHz → 100 ns/tick
inline constexpr uint32_t kMinCompareLeadTicks = 50u;     // Minimum scheduler lead time

// Engine physical constants
inline constexpr uint16_t kTeethPerRev60_2 = 60u;
inline constexpr uint16_t kRealTeeth60_2 = 58u;
inline constexpr uint16_t kToothAngleMillideg = 6000u;    // 6.0° × 1000
inline constexpr uint8_t kCylinderCount = 4u;

// Fuel calculation constants
inline constexpr uint32_t kAirDensityMgPerCcX1000 = 1184u;  // At 100 kPa, 25°C
inline constexpr uint32_t kFuelDensityMgPerCc = 740u;       // Gasoline
inline constexpr uint16_t kStoichAfrX100 = 1464u;           // 14.64:1

// Sensor limits
inline constexpr int16_t kCltMinX10 = -400;   // -40.0°C
inline constexpr int16_t kCltMaxX10 = 1500;   // 150.0°C
inline constexpr uint16_t kMapMinKpa = 10u;
inline constexpr uint16_t kMapMaxKpa = 300u;

// Flash timing (from errata)
inline constexpr uint32_t kFlashEraseMaxTimeUs = 5000u;
inline constexpr uint32_t kFlashProgramMaxTimeUs = 200u;
inline constexpr uint32_t kFlashFirstOpFreezeTimeUs = 120u;

} // namespace ems::engine
```

**Example Refactor:**
```cpp
// Before
return ticks * 16u;

// After
return ticks * ems::engine::kTim5TickPeriodNs;
```

---

### 6. Unified Error Reporting System

**Problem:** Errors are handled inconsistently across modules. Some use return codes, others set flags, many silently fail.

**Solution: Create `src/app/diagnostics.h`:**
```cpp
#pragma once
#include <cstdint>

namespace ems::app {

enum class FaultCode : uint16_t {
    NONE = 0u,
    
    // Sensor faults (0x0100 - 0x01FF)
    MAP_SENSOR_FAULT = 0x0100u,
    TPS_SENSOR_FAULT = 0x0101u,
    CLT_SENSOR_FAULT = 0x0102u,
    IAT_SENSOR_FAULT = 0x0103u,
    O2_SENSOR_FAULT = 0x0104u,
    
    // Actuator faults (0x0200 - 0x02FF)
    INJ_CIRCUIT_FAULT = 0x0200u,
    IGN_CIRCUIT_FAULT = 0x0201u,
    VVT_CIRCUIT_FAULT = 0x0202u,
    
    // System faults (0x0300 - 0x03FF)
    FLASH_WRITE_FAULT = 0x0300u,
    ADC_RECOVERY_FAULT = 0x0301u,
    SYNC_LOSS_FAULT = 0x0302u,
    SCHEDULER_DROP_FAULT = 0x0303u,
    
    // Calibration faults (0x0400 - 0x04FF)
    CALIBRATION_CHECKSUM_FAULT = 0x0400u,
    CALIBRATION_RANGE_FAULT = 0x0401u,
};

struct FaultRecord {
    FaultCode code;
    uint32_t timestamp_ms;
    uint32_t rpm_x10;
    uint16_t map_kpa;
    int16_t clt_x10;
};

class DiagnosticManager {
public:
    static DiagnosticManager& instance();
    
    void report_fault(FaultCode code);
    void clear_fault(FaultCode code);
    bool has_active_fault(FaultCode code) const;
    uint16_t get_active_fault_count() const;
    const FaultRecord* get_fault_history() const;
    
    // Limp mode management
    bool is_limp_mode_active() const;
    void enter_limp_mode();
    void exit_limp_mode();
    
private:
    DiagnosticManager() = default;
    // ... implementation
};

} // namespace ems::app
```

**Usage Example:**
```cpp
// In fuel_calc.cpp
if (!ems::hal::nvm_write_ltft(rpm_idx, map_idx, static_cast<int8_t>(rounded_pct))) {
    ++g_nvm_write_faults;
    ems::app::DiagnosticManager::instance().report_fault(
        ems::app::FaultCode::FLASH_WRITE_FAULT);
}

// In sensors.cpp
if (g_adc_recovery_failed) {
    ems::app::DiagnosticManager::instance().report_fault(
        ems::app::FaultCode::ADC_RECOVERY_FAULT);
    return kMapDefaultLimpHome;
}
```

---

## P2: Best Practices & Code Quality

### 7. Const Correctness Enforcement

**Issues Found:**
- Global state variables lack `const` where appropriate
- Function parameters passed by value instead of const reference
- Missing `constexpr` for compile-time constants

**Examples to Fix:**

```cpp
// In table3d.h - make lookup tables const
extern const uint8_t ve_table[kTableAxisSize][kTableAxisSize];
extern const int16_t lambda_target_table_x1000[kTableAxisSize][kTableAxisSize];

// In calibration.h - config should be const after initialization
struct EngineConfig {
    // ... fields
};

extern const EngineConfig g_eng_cfg;  // Not just 'EngineConfig'

// Function parameters - use const reference for large structs
// Before
void table3d_lookup_u8(Table2dLookup lookup);

// After
void table3d_lookup_u8(const Table2dLookup& lookup);
```

---

### 8. Input Validation & Bounds Checking

**Problem:** Several functions perform arithmetic without overflow checks.

**Critical Locations:**

```cpp
// In fuel_calc.cpp:369
const uint64_t num = static_cast<uint64_t>(base_pw_us) * corr_clt_x256 * corr_iat_x256;
// Risk: If base_pw_us is near max, multiplication could overflow before cast

// Fix: Validate inputs before calculation
if (base_pw_us > kMaxBasePwUs || corr_clt_x256 > kMaxCorr || corr_iat_x256 > kMaxCorr) {
    return 0u;  // Or safe default
}
```

```cpp
// In ckp.cpp:217
inline uint32_t ticks_to_ns(uint32_t ticks) noexcept {
    return ticks * 16u;  // Overflow at ticks > 268 million
}

// Better: Document max input or add check
inline uint32_t ticks_to_ns(uint32_t ticks) noexcept {
    // Max safe input: 0xFFFFFFF (268,435,455 ticks ≈ 4.3 seconds @ 62.5 MHz)
    return ticks * 16u;
}
```

---

### 9. Explicit Variable Initialization

**Problem:** Some global/static variables rely on zero-initialization, which may not occur correctly in all linker configurations.

**Fix:**
```cpp
// Before
static DecoderState g_state;

// After
static DecoderState g_state = {};  // Explicit zero-initialization

// Before
static volatile uint32_t g_flash_wait_timeouts;

// After
static volatile uint32_t g_flash_wait_timeouts = 0u;
```

Apply to all globals in:
- `src/drv/ckp.cpp`
- `src/engine/ecu_sched.cpp`
- `src/hal/flash.cpp`

---

## P3: Testing & Documentation

### 10. Expand Test Coverage

**Missing Test Scenarios:**

1. **TIM8 Overflow Edge Case:**
   ```cpp
   TEST(ECUSched, HandlesTimerWraparound) {
       // Set TIM8_CNT near 0xFFFF boundary
       // Schedule event with various delta values
       // Verify CCR calculation handles wrap correctly
   }
   ```

2. **Flash Write During Engine Run:**
   ```cpp
   TEST(Flash, BlocksWriteDuringEngineOperation) {
       // Simulate engine running at 1000 RPM
       // Attempt nvm_flush_adaptive_maps()
       // Verify operation is deferred
   }
   ```

3. **ADC Recovery Failure:**
   ```cpp
   TEST(Sensors, HandlesAdcRecoveryFailure) {
       // Simulate ADC timeout
       // Force recovery to fail
       // Verify limp-home values used
   }
   ```

4. **Critical Section Exception Safety:**
   ```cpp
   TEST(CriticalSection, RAIIUnwindsOnEarlyReturn) {
       // Call function with early return inside critical section
       // Verify interrupts re-enabled after function exits
   }
   ```

---

### 11. Hardware Mocking Framework

**Current State:** Basic mocks exist in test files but lack sophistication.

**Needed Enhancements:**

```cpp
// test/mocks/adc_mock.h
class AdcMock {
public:
    enum class FaultMode {
        NORMAL,
        TIMEOUT,
        RECOVERY_FAIL,
        NOISE_SPIKE
    };
    
    void set_fault_mode(FaultMode mode);
    void inject_noise_spike(uint32_t channel, int16_t deviation);
    void simulate_timeout();
    
    uint16_t read_channel(uint8_t channel);
    
private:
    FaultMode fault_mode_ = FaultMode::NORMAL;
    uint32_t timeout_counter_ = 0;
};

// test/mocks/flash_mock.h
class FlashMock {
public:
    void set_erase_time_us(uint32_t time_us);
    void set_program_time_us(uint32_t time_us);
    void simulate_first_op_freeze();
    void set_endurance_remaining(uint32_t cycles);
    
    bool erase_sector(uint32_t sector);
    bool program_word(uint32_t addr, uint32_t data);
    
    uint32_t get_erase_count() const;
    uint32_t get_program_count() const;
    
private:
    uint32_t erase_count_ = 0;
    uint32_t program_count_ = 0;
    uint32_t endurance_remaining_ = 1000;
};
```

---

### 12. Doxygen Documentation

**Add comprehensive documentation:**

```cpp
/**
 * @brief Schedule injection or ignition event at specified angle before TDC.
 * 
 * @param channel Hardware channel (0-3 for injection, 4-7 for ignition)
 * @param angle_deg Angle before TDC in degrees (0-719 for 4-stroke cycle)
 * @param action Action to perform (ECU_ACT_INJ_ON, ECU_ACT_INJ_OFF, ECU_ACT_SPARK)
 * 
 * @pre Must be called with valid sync state (FULL_SYNC or HALF_SYNC)
 * @pre Channel must be configured via ecu_hw_init()
 * 
 * @note Uses hardware output compare for precise timing
 * @note Minimum advance warning: kMinCompareLeadTicks (typically 5 µs)
 * 
 * @return true if event scheduled successfully
 * @return false if event would occur too soon (dropped)
 * 
 * @throws None (noexcept)
 * 
 * @see ecu_sched_set_advance_deg()
 * @see ecu_sched_set_dwell_ticks()
 */
void ecu_schedule_angle_event(uint8_t channel, uint32_t angle_deg, EcuAction action);
```

**Required Documentation:**
- All public API functions in `drv/`, `engine/`, `hal/` namespaces
- All struct/class members with non-obvious semantics
- All state machines with transition diagrams
- Errata compliance matrix (which workarounds implemented where)

---

## Implementation Priority Matrix

| Issue | Severity | Effort | Risk if Unfixed | Priority |
|-------|----------|--------|-----------------|----------|
| TIM8 overflow | P0 | Low | Engine damage | 1 |
| Flash during run | P0 | Medium | Timing jitter | 2 |
| ADC recovery | P0 | Medium | Unsafe AFR | 3 |
| Critical section RAII | P1 | Low | Deadlock risk | 4 |
| Magic numbers | P2 | High | Maintainability | 5 |
| Error reporting | P1 | Medium | Debug difficulty | 6 |
| Const correctness | P2 | Medium | Code quality | 7 |
| Input validation | P2 | Medium | Overflow bugs | 8 |
| Variable init | P2 | Low | Undefined behavior | 9 |
| Test coverage | P3 | High | Regression risk | 10 |
| Hardware mocks | P3 | High | Test quality | 11 |
| Documentation | P3 | High | Knowledge loss | 12 |

---

## Verification Checklist

After implementing fixes:

- [ ] TIM8 configured consistently (16-bit vs 32-bit) throughout codebase
- [ ] Flash writes blocked above 300 RPM threshold
- [ ] ADC recovery status checked before sensor reads
- [ ] All critical sections use RAII wrapper
- [ ] Zero magic numbers in calculation paths
- [ ] All fault conditions logged via DiagnosticManager
- [ ] All global variables explicitly initialized
- [ ] Host tests pass with new edge case scenarios
- [ ] Bench test shows no timing jitter during Flash operations
- [ ] Oscilloscope validates TIM8 output at wraparound boundary

---

## References

- STM32H562 Reference Manual (RM0481)
- Device Errata (ES0565 Rev 8, January 2026)
- RusEFI Issue #1488 (timestamp corruption)
- ISO 26262 (functional safety guidelines for automotive)
