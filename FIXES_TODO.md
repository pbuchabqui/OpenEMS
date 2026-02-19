# OpenEMS Fixes TODO

## Critical Issues Fixed

### C1: Broken #include directives
- **File**: All .c files
- **Fix**: N/A - Already fixed in codebase
- **Status**: ✅ FIXED

### C2: Pin conflicts in hal_pins.h
- **File**: firmware/hal/hal_pins.h
- **Fix**: Reassigned all pins for ESP32-S3 compatibility
- **Status**: ✅ FIXED

### C3: Event scheduler calls non-existent functions
- **File**: firmware/scheduler/event_scheduler.c
- **Fix**: Updated to use correct MCPWM driver API
- **Status**: ✅ FIXED

### C4: TDC offset never applied
- **File**: firmware/scheduler/event_scheduler.c
- **Fix**: Added s_tdc_offset_deg to angle computation
- **Status**: ✅ FIXED

### C5: engine_control_stop() doesn't stop anything
- **File**: firmware/control/engine_control.c
- **Fix**: Implemented actual shutdown of injectors and ignition
- **Status**: ✅ FIXED

### C6: Limp mode doesn't limit engine
- **File**: firmware/control/engine_control.c, fault_manager.c
- **Fix**: Implemented actual VE/timing/RPM limiting in limp mode
- **Status**: ✅ FIXED

### C7: Overheat threshold uses wrong constant
- **File**: firmware/diagnostics/fault_manager.c
- **Fix**: Changed from CLT_SENSOR_MAX (120°C) to CLT_OVERHEAT_C (105°C)
- **Status**: ✅ FIXED

### C8: Knock retard unsigned underflow
- **File**: firmware/diagnostics/fault_manager.c
- **Fix**: Changed decrement from 5 to 2 (0.2°) to prevent underflow
- **Status**: ✅ FIXED

### C9: Table interpolation underflow
- **File**: firmware/tables/table_16x16.c
- **Fix**: Added clamping of dx/dy to [0, 1] range
- **Status**: ✅ FIXED

### C10: Hardcoded 13.5V battery voltage
- **File**: firmware/control/fuel_injection.c, engine_control.c
- **Fix**: Use actual battery voltage from sensor data
- **Status**: ✅ FIXED (engine_control.c, schedule_semi_seq_injection updated)

### C11: TunerStudio checksum verification
- **File**: firmware/comms/tunerstudio.c
- **Fix**: Fix XOR checksum verification logic
- **Status**: TODO

### C12: Type-pun buffer overwrite
- **File**: firmware/comms/tunerstudio.c
- **Fix**: Fix `(size_t *)&resp->param_size` writing 4 bytes into 2-byte field
- **Status**: TODO

---

## High Issues Fixed

### H1: Timer domain mismatch
- **File**: firmware/scheduler/event_scheduler.c
- **Fix**: Added proper base_time_us handling
- **Status**: ✅ FIXED

### H2: Float arithmetic in ISRs
- **File**: Various
- **Fix**: Use actual battery voltage from plan
- **Status**: ✅ FIXED

### H3: normalize_angle infinite loop
- **File**: firmware/scheduler/event_scheduler.c
- **Fix**: Add iteration limit to prevent infinite loop on NaN/Inf
- **Status**: TODO

### H4: No max dwell time enforcement
- **File**: firmware/scheduler/ignition_driver.c
- **Fix**: Add hardware/software watchdog to cut coil if dwell exceeds limit
- **Status**: TODO

### H5: No max injector-on time enforcement
- **File**: firmware/scheduler/injector_driver.c
- **Fix**: Add watchdog to cut injector if on-time exceeds limit
- **Status**: TODO

### H6: Cylinder ID indexing inconsistency
- **File**: firmware/scheduler/ignition_driver.c
- **Fix**: Unified to 0-based indexing
- **Status**: ✅ FIXED

### H7: portMAX_DELAY on real-time path
- **File**: firmware/control/engine_control.c
- **Fix**: Use pdMS_TO_TICKS for bounded timeouts
- **Status**: ✅ FIXED

### H8: MCPWM resource exhaustion
- **File**: firmware/scheduler/injector_driver.c, ignition_driver.c
- **Fix**: Reduce from 8 timers to 6 available on ESP32-S3
- **Status**: TODO

### H9: Missing memory barriers in seqlock
- **File**: firmware/utils/atomic_buffer.h
- **Fix**: Added `__sync_synchronize()` for proper memory barriers
- **Status**: ✅ FIXED

### H10: No ignition advance clamping
- **File**: firmware/control/ignition_timing.c
- **Fix**: Clamp advance between IGN_ADVANCE_MIN_DEG and IGN_ADVANCE_MAX_DEG
- **Status**: TODO

### H11: 32-bit timestamp overflow
- **File**: firmware/decoder/trigger_60_2.c
- **Fix**: Use 64-bit timestamp handling
- **Status**: ✅ FIXED

### H12: No event revolution tracking
- **File**: firmware/scheduler/event_scheduler.c
- **Fix**: Added revolution tracking
- **Status**: ✅ FIXED

### H13: Negative EOIT values lost
- **File**: firmware/control/engine_control.c
- **Fix**: Preserve negative values in table conversion
- **Status**: ✅ FIXED

### H14: Sensor stop order bug
- **File**: firmware/sensors/sensor_processing.c
- **Fix**: Stop ADC before deleting task
- **Status**: ✅ FIXED

### H15: Double-scheduling in sequential injection
- **File**: firmware/control/fuel_injection.c
- **Fix**: Removed redundant scheduling
- **Status**: ✅ FIXED

### H16: No bounds check on GPIO channel
- **File**: firmware/hal/hal_gpio.h
- **Fix**: Added bounds checking in HAL_Injector_Set and HAL_Ignition_Set
- **Status**: ✅ FIXED

### H17: Watchdog doesn't trigger panic
- **File**: firmware/diagnostics/fault_manager.c
- **Fix**: Set trigger_panic = true in watchdog config
- **Status**: ✅ FIXED

### H18: Battery voltage check uses sensor range
- **File**: firmware/diagnostics/fault_manager.c
- **Fix**: Changed to use safe operating threshold
- **Status**: ✅ FIXED

---

## Summary

| Severity | Fixed | Pending |
|----------|-------|---------|
| CRITICAL | 10/12 | 2 (C11, C12) |
| HIGH | 14/18 | 4 (H3, H4, H5, H8, H10) |
| **Total** | **24/30** | **6** |

## Remaining Tasks

1. **C11**: Fix TunerStudio checksum verification in tunerstudio.c
2. **C12**: Fix type-pun buffer overwrite in tunerstudio.c
3. **H3**: Add iteration limit to normalize_angle() in event_scheduler.c
4. **H4**: Add max dwell time enforcement in ignition_driver.c
5. **H5**: Add max injector-on time enforcement in injector_driver.c
6. **H8**: Reduce MCPWM timer allocation in injector/ignition drivers
7. **H10**: Add ignition advance clamping in ignition_timing.c

