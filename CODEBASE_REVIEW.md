# OpenEMS Codebase Overview & Peer Review

## What Is This Project?

OpenEMS is an **open-source Engine Control Unit (ECU)** firmware for 4-cylinder, 4-stroke gasoline engines. It runs on an **ESP32-S3** microcontroller using ESP-IDF (no Arduino layer). It controls **fuel injection** and **ignition timing** in real time by decoding a 60-2 crankshaft trigger wheel and scheduling hardware-timed events.

Think of it as an open-source replacement for a factory ECU — the kind of thing you'd find in a standalone aftermarket engine management system like MegaSquirt or rusefi.

---

## Tech Stack

| Layer | Technology |
|---|---|
| **MCU** | ESP32-S3 (dual-core Xtensa LX7, 240 MHz) |
| **Framework** | ESP-IDF v5.x (FreeRTOS, MCPWM, PCNT, TWAI, ADC, NVS, SDMMC) |
| **Language** | C (C99), HAL headers in C/C++ compatible style |
| **Build** | CMake via `idf.py` |
| **Comms** | CAN bus (TWAI), ESP-NOW (wireless), USB CDC (CLI), TunerStudio protocol |
| **Storage** | NVS flash (calibration), SD card (data logging), OTA partitions |
| **License** | GPL v3 |

---

## Architecture: Dual-Core Split

```
Core 0 (ISR / bare-metal)              Core 1 (FreeRTOS tasks)
─────────────────────────               ─────────────────────────
decoder/trigger_60_2.c                  control/engine_control.c   (planner + executor)
scheduler/event_scheduler.c             control/fuel_calc.c
scheduler/injector_driver.c             control/fuel_injection.c
scheduler/ignition_driver.c             control/ignition_timing.c
scheduler/hp_timing.c                   control/closed_loop_fuel.c
scheduler/hp_state.c                    sensors/sensor_processing.c
hal/hal_gpio.h (inline)                 comms/can_wideband.c
hal/hal_timer.h (inline)                comms/espnow_link.c
                                        comms/tunerstudio.c
                                        diagnostics/fault_manager.c
                                        logging/sd_logger.c
                                        utils/cli_interface.c
```

**Core 0** handles time-critical ISR work: decoding the crankshaft trigger wheel (PCNT + ETM hardware capture), computing crank angle, and firing MCPWM compare events for injectors and ignition coils. Target jitter: <1 μs.

**Core 1** runs FreeRTOS tasks: a **planner** task (builds fuel/ignition plans from sensor data and lookup tables), an **executor** task (dispatches plans to Core 0 hardware drivers), and supporting tasks for CAN, wireless comms, data logging, and diagnostics.

**Cross-core data exchange** uses a seqlock pattern (`utils/atomic_buffer.h`) — no mutexes on the hot path.

---

## Main Entry Points

| Entry Point | File | Purpose |
|---|---|---|
| **`app_main()`** | `firmware/main/main.c:11` | ESP-IDF entry. Calls `engine_control_init()`, then loops logging status at 1 Hz |
| **`engine_control_init()`** | `firmware/control/engine_control.c` | Initializes all subsystems, creates planner/executor/monitor FreeRTOS tasks |
| **`sync_init()`** | `firmware/decoder/trigger_60_2.c` | Configures PCNT + ETM + GPIO ISR for crankshaft/camshaft decoding |
| **`evt_scheduler_init()`** | `firmware/scheduler/event_scheduler.c` | Sets up angle-based event queue |
| **`mcpwm_injection_hp_init()`** | `firmware/scheduler/injector_driver.c` | Creates MCPWM timers/comparators for 4 injector channels |
| **`mcpwm_ignition_hp_init()`** | `firmware/scheduler/ignition_driver.c` | Creates MCPWM timers/comparators for 4 ignition channels |

---

## Repository Stats

| Metric | Value |
|---|---|
| Source files (.c) | 23 |
| Header files (.h) | 32 |
| Total lines of code | ~13,500 |
| Largest file | `engine_control.c` (1,586 lines) |
| Second largest | `cli_interface.c` (1,048 lines) |

---

## Key Design Decisions

1. **Angle-based scheduling** (inspired by rusefi): Events are scheduled in crank degrees, converted to μs at dispatch time. This avoids timing error accumulation during RPM changes.

2. **MCPWM absolute compare**: Uses continuous-running hardware timers with absolute compare values. No timer restart needed — avoids the jitter of start/stop/restart cycles.

3. **Phase predictor**: Adaptive filter (`hp_timing.c`) predicts the next tooth period for more accurate event scheduling during acceleration/deceleration.

4. **16x16 bilinear interpolation tables**: VE, ignition timing, lambda target, and EOIT calibration all use the same table format with configurable RPM and load bins.

5. **Speed-density fueling**: `Pulsewidth = REQ_FUEL × (VE/100) × (MAP/100) × enrichments × corrections`. No MAF sensor needed.

---

# Peer Review Findings

## Severity Summary

| Severity | Count | Description |
|---|---|---|
| **CRITICAL** | 12 | Will cause engine damage, won't compile, or defeats safety systems |
| **HIGH** | 18 | Incorrect behavior, data corruption, or timing errors |
| **MEDIUM** | 20 | Race conditions, code smells, maintainability issues |
| **LOW** | 12 | Style, dead code, minor inefficiencies |

---

## CRITICAL Issues

### C1. Code won't compile — broken `#include` directives
**Files:** `trigger_60_2.c`, `can_wideband.c`, `config_manager.c`, and others
```c
#include sync.h"          // ← missing opening quote
#include engine_control.h" // ← same
```
Every file has `#include filename.h"` instead of `#include "filename.h"`. This is a hard compilation blocker.

### C2. Pin conflicts in `hal_pins.h` make hardware untestable
- **GPIO 34** is assigned to both `HAL_PIN_CKP` (crankshaft sensor) and `HAL_PIN_VBAT` (battery voltage). The crankshaft sensor is the single most critical input — sharing it with ADC reads will destroy sync.
- **GPIO 18** is assigned to both `HAL_PIN_IGN_3` (ignition coil 3) and `HAL_PIN_SD_CLK` (SD card clock). SD card operations will fire ignition coil 3.
- **GPIO 5** is assigned to both `HAL_PIN_CAN_RX` and `HAL_PIN_SD_CS`. CAN communication and SD card are mutually exclusive.
- **GPIO 23** is `HAL_PIN_BRAKE` and `HAL_PIN_SD_MOSI`. **GPIO 19** is `HAL_PIN_VVT_INTAKE` and `HAL_PIN_SD_MISO`.

### C3. Event scheduler calls non-existent functions
**File:** `event_scheduler.c:88-97`
```c
mcpwm_injection_hp_schedule_open(evt->cylinder, fire_abs);   // doesn't exist
mcpwm_ignition_hp_schedule_dwell(evt->cylinder, fire_abs);   // doesn't exist
```
The actual driver APIs have different names and signatures. This code will not link.

### C4. TDC offset is never applied
**File:** `event_scheduler.c:136`
`s_tdc_offset_deg` is set by `evt_set_tdc_offset()` but never referenced in tooth angle computation. All angle-based events (injection, ignition) fire at the wrong crank angle. This is the most fundamental calibration parameter.

### C5. `engine_control_stop()` doesn't stop anything
**File:** `engine_control.c:1322-1328`
The function just logs and returns. Tasks keep running, injectors keep firing. If called for safety reasons, the engine continues uncontrolled.

### C6. Limp mode doesn't actually limit the engine
**File:** `fault_manager.c` / `engine_control.c`
`safety_activate_limp_mode()` sets `rpm_limit = 3000` but nothing reads this limit. When limp mode triggers, `build_plan()` returns `ESP_FAIL` — meaning the last plan in the ring buffer keeps executing with stale RPM/timing/fuel. There is no degraded-but-safe operating mode.

### C7. Overheat threshold uses wrong constant
**File:** `fault_manager.c:68`
Uses `CLT_SENSOR_MAX` (120°C) instead of `CLT_OVERHEAT_C` (105°C). The engine won't enter limp mode until 15°C past the safe limit.

### C8. Knock retard unsigned underflow
**File:** `fault_manager.c:218-219`
```c
if (knock_prot->timing_retard > 0) {
    knock_prot->timing_retard -= 5;  // uint16_t: 3 - 5 = 65534
}
```
If `timing_retard` is 1-4, subtracting 5 wraps to ~65534, causing massive timing retard.

### C9. Table interpolation underflow for sub-bin values
**File:** `table_16x16.c:58`
If RPM is below the first bin (e.g., RPM=300, bin[0]=500), `rpm - x0` underflows in `uint16_t` arithmetic, producing a wildly wrong interpolation factor. This affects VE, ignition timing, and lambda target at low RPM.

### C10. Hardcoded 13.5V for injector dead-time compensation
**File:** `fuel_injection.c:65-66`
```c
float battery_voltage = 13.5f;  // never reads actual sensor
float temperature = 25.0f;
```
Injector latency compensation is completely wrong at any voltage other than 13.5V. At cranking voltage (~10V), injectors open slower; under-compensating causes a lean condition.

### C11. Tuning protocol checksum verification always fails
**File:** `tunerstudio.c:439`
The receiver XORs all header bytes including the stored checksum, always producing 0, then compares against the non-zero stored checksum. Every non-trivial message is rejected.

### C12. Type-pun buffer overwrite in tuning protocol
**File:** `tunerstudio.c:235`
`(size_t *)&resp->param_size` writes 4 bytes into a 2-byte `uint16_t` field, corrupting adjacent memory on the stack.

---

## HIGH Issues

### H1. Timer domain mismatch
**File:** `event_scheduler.c:84`
`fire_abs = base_time_us + delay_us` is in system-time domain, but MCPWM timers use their own counter domain (period = 30,000,000 ticks). No conversion exists. All scheduled times are wrong.

### H2. Float arithmetic in ISRs without FPU context save
**Files:** `event_scheduler.c`, `hp_timing.c`
`CONFIG_FREERTOS_FPU_IN_ISR` must be enabled or FPU registers are corrupted when ISRs interrupt float-using tasks. Not verified anywhere.

### H3. `normalize_angle()` can spin forever
**File:** `event_scheduler.c:53-57`
```c
while (a >= 720.0f) a -= 720.0f;
while (a <    0.0f) a += 720.0f;
```
If `a` is NaN, +Inf, or very large (corrupted data), this loops indefinitely in an ISR, locking Core 0.

### H4. No maximum dwell time enforcement
**Files:** `ignition_driver.c`, `event_scheduler.c`
If the spark compare event never fires (timer overflow), the ignition coil stays energized indefinitely. This is a **fire hazard** — ignition coils overheat and can catch fire with extended dwell.

### H5. No maximum injector-on time enforcement
**File:** `injector_driver.c`
Same issue as H4 for injectors. A stuck-open injector floods the cylinder with fuel, causing hydro-lock risk and catalytic converter damage.

### H6. Cylinder ID indexing inconsistency (0-based vs 1-based)
**File:** `ignition_driver.c`
`schedule_one_shot_absolute` uses 1-based IDs (1-4), but `get_counter` uses 0-based (0-3). If the scheduler passes 0-based IDs, cylinder 0 is rejected as invalid.

### H7. `portMAX_DELAY` mutex on the real-time hot path
**File:** `engine_control.c:715, 359`
The planner task blocks indefinitely waiting for `g_map_mutex`. If the monitor task holds it during NVS flash writes, injection/ignition events are missed.

### H8. MCPWM resource exhaustion
**Files:** `injector_driver.c`, `ignition_driver.c`
Both drivers allocate 4 MCPWM timers each (8 total), but ESP32-S3 only has 6 (2 groups × 3 timers). Channels 6-7 will fail at init.

### H9. Missing memory barriers in seqlock (`atomic_buffer.h`)
Only compiler barriers (`__asm__ volatile("" ::: "memory")`) are used, not hardware barriers. On dual-core ESP32-S3, Core 1 can see the updated sequence number before the data, reading partially stale values.

### H10. No ignition advance clamping anywhere
**Files:** `engine_control.c`, `ignition_timing.c`
`IGN_ADVANCE_MIN_DEG` (-5) and `IGN_ADVANCE_MAX_DEG` (45) are defined but never checked. A corrupt table entry of 65535 produces 6553.5° advance.

### H11. 32-bit timestamp overflow after ~71 minutes
**File:** `trigger_60_2.c:345`
`uint32_t now_us = (uint32_t)esp_timer_get_time();` wraps after 71.6 minutes. The overflow calculation at line 361 is also off-by-one.

### H12. No event revolution tracking
**File:** `event_scheduler.c`
`schedule_rev` field exists but is never set or checked. Events armed for revolution N can fire again on revolution N+1, causing double-firing of injectors/coils.

### H13. Negative EOIT values lost in table conversion
**File:** `engine_control.c:234-243`
`clamp_eoit_normal()` allows range [-8.0, 16.0], but `eoit_normal_to_table()` maps all negatives to 0, making them indistinguishable.

### H14. Sensor stop order bug
**File:** `sensor_processing.c:148-160`
`vTaskDelete` is called before `adc_continuous_stop()`. The ADC DMA callback may reference the deleted task.

### H15. Double-scheduling in sequential injection
**File:** `fuel_injection.c:114-126`
`fuel_injection_schedule_sequential()` calls individual per-cylinder scheduling (which fires hardware) then also calls `mcpwm_injection_hp_schedule_sequential_absolute()` — double-scheduling cylinder 0.

### H16. No bounds check on GPIO channel in HAL
**File:** `hal_gpio.h:57-58`
`HAL_Injector_Set(channel, active)` accesses `pins[channel]` without bounds checking. If `channel >= 4`, this reads past the array and toggles an arbitrary GPIO.

### H17. Watchdog doesn't trigger panic
**File:** `fault_manager.c:165`
`trigger_panic = false` means a completely locked-up system silently continues instead of resetting.

### H18. Battery voltage safety check uses sensor range, not safe range
**File:** `fault_manager.c:77-78`
Checks voltage against `VBAT_SENSOR_MIN` (7.0V) instead of a safe operating threshold. 7.1V passes the check but is far too low for reliable ignition.

---

## Security Issues

### S1. No CAN bus authentication
**File:** `can_wideband.c`
CAN filter accepts all IDs. Any device on the bus can inject EOIT calibration commands (ID 0x6E0) to alter fueling, potentially causing engine damage. No rate limiting, no challenge-response.

### S2. No ESP-NOW authentication
**File:** `espnow_link.c`
Message types `ESPNOW_MSG_TABLE_UPDATE` and `ESPNOW_MSG_PARAM_SET` allow wireless ECU configuration changes from any nearby ESP32. XOR checksum provides zero tamper protection. Encryption is disabled by default.

### S3. Tuning protocol auth bypassed
**File:** `tunerstudio.c:183, 200-204`
Every connecting client is immediately granted `permissions = 0xFFFF` (full access). The AUTH message handler always succeeds.

### S4. CLI admin mode has no password
**File:** `cli_interface.c:1031-1037`
`cli_enter_admin()` accepts any input. Anyone with USB access gets full admin privileges.

---

## Systemic Patterns

1. **Unsafe task termination everywhere**: CAN, ESP-NOW, logger, CLI all use `vTaskDelay(100-200ms)` + NULL the handle, hoping the task has exited. Every instance is a race condition.

2. **Broken `#include` directives**: Every `.c` file is missing the opening `"` in include directives. The code cannot compile as-is.

3. **Stub functions that report success**: Config save/load, several CLI commands, `engine_control_stop()`, test result JSON all return success without doing anything.

4. **`log_entry_t` type collision**: Both `logger.h` and `sd_logger.h` define `log_entry_t` with different layouts. Including both causes compilation failure.

5. **Macro redefinitions**: `PW_MAX_US`, `PW_MIN_US`, `STFT_LIMIT`, `LTFT_LIMIT`, `LTFT_ALPHA` are defined in both `engine_config.h` and `engine_control.c`.

6. **`static const` arrays in headers**: `DEFAULT_RPM_BINS` and `DEFAULT_LOAD_BINS` duplicate 64 bytes per translation unit.

---

## Recommendations (Priority Order)

1. **Fix the `#include` directives** — the code doesn't compile
2. **Resolve pin conflicts in `hal_pins.h`** — the hardware can't function
3. **Implement the missing bridge functions** in `event_scheduler.c` or update to call real driver APIs
4. **Add fail-safe output shutdown** — a single function that forces all injectors closed and all coils de-energized, called on sync loss, watchdog timeout, and fault conditions
5. **Add maximum dwell and injector-on time enforcement** in hardware (MCPWM force-off timer) or software watchdog
6. **Apply TDC offset** in the angle computation
7. **Fix `table_16x16.c` underflow** for sub-bin values
8. **Use actual sensor readings** for battery voltage in injector dead-time compensation
9. **Add CAN message filtering and authentication** for calibration commands
10. **Implement actual limp mode** with reduced RPM limit, retarded timing, and enriched fueling
11. **Add hardware memory barriers** to the seqlock
12. **Unify cylinder ID convention** to 0-based everywhere
13. **Replace `portMAX_DELAY`** with bounded timeouts on the real-time path
14. **Add MISRA-C static analysis** to the CI pipeline
