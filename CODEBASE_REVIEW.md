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

---

# Detailed Per-File Analysis

## Core Engine Path

### `firmware/main/main.c` (60 lines)
The application entry point. Clean and minimal.

- **Line 16-18**: If `engine_control_init()` fails, the system enters an infinite `vTaskDelay` loop. No watchdog feed, no error LED, no retry. The engine is permanently dead until power cycle.
- **Lines 29-54**: The main loop only logs. All other subsystems (CAN, ESP-NOW, tuning, logger, CLI) are either initialized inside `engine_control_init()` or never started.

### `firmware/control/engine_control.c` (1,586 lines)
The central orchestrator. Contains planner/executor task pair, plan ring buffer, EOIT calibration, LTFT learning, NVS persistence, and ESP-NOW telemetry.

**Architecture**: The planner task wakes on tooth callback notification, builds a plan (sensor + table lookups + closed-loop correction), pushes it to a ring buffer. The executor task pops the latest plan and dispatches it to injection/ignition drivers. This is a good design that decouples sensor processing from hardware dispatch.

**Issues found**:
- **Line 1 (and 2-18)**: All 18 local `#include` directives are missing the opening `"`. Hard compilation blocker.
- **Line 359, 388, 402, 715**: `xSemaphoreTake(g_map_mutex, portMAX_DELAY)` on the real-time planner path. If the monitor task holds this mutex during NVS flash (which can take 10-50ms), planner misses its tooth deadline. Use `pdMS_TO_TICKS(2)` or a try-lock.
- **Line 430-449**: `perf_percentile` uses O(n^2) bubble sort on 128 elements inside a spinlock-protected snapshot. At 128 elements this is ~16K comparisons. Not a correctness issue but wastes CPU on a real-time system.
- **Line 544-554**: `runtime_state_publish()` uses `__atomic_fetch_add` for the seqlock sequence, which provides proper hardware memory barriers. This is correct (unlike `atomic_buffer.h` which only has compiler barriers).
- **Line 668-669**: Ignition `schedule_one_shot_absolute` is called with hardcoded `13.5f` battery voltage. Same issue as C10.
- **Line 745**: Shadow variable `uint32_t now_ms` declared inside the lambda correction block, shadowing the outer `now_ms` from line 689. Not a bug (both are the same computation) but confusing.
- **Line 793-796**: Per-cylinder EOI scheduling in a loop. If one cylinder fails, the remaining still schedule. The `scheduling_ok` flag captures this but only logs after all 4 attempts.
- **Line 876**: Stale plan discard: if `queue_age > 3ms`, the plan is dropped. This means at RPMs below ~600 (tooth period > 3ms), every plan is discarded. Consider scaling the deadline with RPM.
- **Line 1322-1328**: `engine_control_stop()` is a no-op (CRITICAL C5).
- **Line 1338-1348**: `engine_control_deinit()` calls `vTaskDelete()` externally on 3 tasks. The tasks may hold spinlocks or mutexes at the time, causing deadlock on next access.

### `firmware/control/fuel_calc.c` (174 lines)
Speed-density fuel calculation with interpolation caching and enrichments.

- **Lines 29-51**: `lookup_with_cache` uses a deadband (`INTERP_CACHE_RPM_DEADBAND`, `INTERP_CACHE_LOAD_DEADBAND`). If the table checksum doesn't change but values are modified and checksum collides (16-bit), stale cached results are returned. The checksum is a simple 16-bit sum — collisions are likely.
- **Line 137-174**: `fuel_calc_pulsewidth_us` — the core fuel equation. The formula `base_pw = REQ_FUEL_US * (ve/100) * (map_kpa/100)` is the standard speed-density equation. Warmup, acceleration, and lambda corrections are applied multiplicatively. Clean implementation.
- **No injector dead-time added here**. The dead-time is supposed to come from `fuel_injection.c:69` via `mcpwm_injection_hp_apply_latency_compensation`, but that uses hardcoded 13.5V.

### `firmware/control/fuel_injection.c` (127 lines)
Translates EOI degree targets into MCPWM one-shot absolute scheduling.

- **Line 42-93**: `fuel_injection_schedule_eoi_ex` — the core scheduling function. Takes a target EOI angle, computes SOI, converts to delay_us, and calls the MCPWM HP driver. The angle math is correct for the 720-degree 4-stroke cycle.
- **Line 65-66**: **CRITICAL C10** — hardcoded `battery_voltage = 13.5f` and `temperature = 25.0f`.
- **Line 102-127**: `fuel_injection_schedule_sequential` calls `fuel_injection_schedule_eoi_ex` per cylinder (which triggers hardware scheduling), then also calls `mcpwm_injection_hp_schedule_sequential_absolute`. This double-schedules — the per-cylinder calls already armed the hardware via `schedule_one_shot_absolute`. The final `schedule_sequential_absolute` overwrites cylinder 0 with `pulsewidth_us[0]` but uses `offsets[]` from the individual calls, creating a timing conflict.

### `firmware/control/ignition_timing.c` (referenced but not read — 171 lines)
Applies ignition timing via wasted-spark or sequential scheduling.

---

## Trigger Decoder

### `firmware/decoder/trigger_60_2.c` (834 lines)
Crankshaft/camshaft position decoder using PCNT + ETM hardware capture.

**Architecture**: Uses ESP-IDF PCNT for tooth counting with a watch-step/watch-point callback on every tooth. On ESP32-S3 with ETM support, a GPIO edge triggers a GPTimer capture (hardware timestamp) via the ETM interconnect — this gives sub-microsecond capture precision without ISR latency. A separate GPIO ISR captures the camshaft (CMP) signal for phase detection.

**Issues found**:
- **Line 1-3**: Missing opening `"` on includes.
- **Line 345**: `uint32_t now_us = (uint32_t)esp_timer_get_time();` — truncates 64-bit to 32-bit, overflows after 71.6 minutes. The overflow handling at line 357-362 uses `now_us >= data->last_capture_time` which is incorrect when `now_us` has wrapped but `last_capture_time` hasn't (both are 32-bit from a 64-bit source).
- **Line 361**: Off-by-one in overflow calculation: `(UINT32_MAX - data->last_capture_time) + now_us` should be `(UINT32_MAX - data->last_capture_time) + now_us + 1` for correct modular arithmetic. However, since the source is 64-bit truncated to 32-bit (not a naturally wrapping 32-bit counter), the wrap semantics are different and this entire approach is fragile.
- **Line 450**: Gap period estimation: `g_sync_data.tooth_period = tooth_period / 3` when a gap is detected. This assumes the gap is exactly 3 tooth widths (60-2 pattern: 2 missing = 3x normal period). The division is correct for a 60-2 wheel but hardcoded.
- **Line 492**: `time_per_degree` calculation uses integer arithmetic with a rounding term `+ 180U`. This is `(period * 60) / 360 = period / 6`, which is correct for a 60-tooth wheel (6 degrees per tooth).
- **Line 506**: RPM calculation: `(1000000 * 60) / time_per_revolution` = `60000000 / time_per_revolution`. At 500 RPM, `time_per_revolution` = 120ms = 120000µs, giving RPM = 500. Correct. But `1000000 * 60 = 60000000` fits in uint32_t, so no overflow concern.
- **Line 527**: `ESP_LOGI` called from ISR context (the function is `IRAM_ATTR` and `emit_log` is true only from non-ISR callers, controlled by `sync_ckp_gpio_isr` passing `false`). However, `sync_pcnt_on_reach` calls with `emit_log=false`, so this is safe. The log at line 527 will only fire from non-ISR context.
- **Line 577-604**: The PCNT callback (`sync_pcnt_on_reach`) reads the tooth callback pointer without holding the spinlock (line 598-599). This is a read of two words (`cb` and `cb_ctx`) that could tear if another core is in `sync_register_tooth_callback`. On Xtensa, pointer reads are atomic, but the pair {cb, ctx} is not. A registered callback could be called with the wrong context pointer.

### `firmware/decoder/trigger_60_2.h` (55 lines)
- **Line 26**: `gap_detected` is `uint32_t` but used as a boolean. Should be `bool` for clarity.
- Clean API surface. Good separation of data types vs function prototypes.

---

## Event Scheduler

### `firmware/scheduler/event_scheduler.c` (259 lines)
Angle-based event queue checked on every tooth ISR.

**Architecture**: Fixed-size array of 16 events. On each tooth, the current crank angle is computed, and events within the next tooth window are fired via MCPWM absolute compare. Cross-core safety via spinlock. Simple and clean.

**Issues found**:
- **Line 88-98**: `fire_event` calls `mcpwm_injection_hp_schedule_open`, `_close`, `mcpwm_ignition_hp_schedule_dwell`, `_spark` — **none of these functions exist**. The actual APIs are `mcpwm_injection_hp_schedule_one_shot_absolute` and `mcpwm_ignition_hp_schedule_one_shot_absolute` with different signatures. This code will not link (CRITICAL C3).
- **Line 84**: `fire_abs = base_time_us + delay_us` — `base_time_us` is `tooth_time_us` (system timer domain), but MCPWM timers run in their own counter domain (0 to 30,000,000). This time domain mismatch means `fire_abs` is meaningless to the MCPWM hardware (HIGH H1).
- **Line 136**: `float tooth_angle = rev_offset + (float)tooth_index * s_deg_per_tooth;` — `s_tdc_offset_deg` is set at line 249 but never subtracted from the angle. Every event fires 114° (or whatever the offset is) from the correct position (CRITICAL C4).
- **Line 53-57**: `normalize_angle` while-loops can spin forever on NaN/Inf (HIGH H3).
- **Line 61**: `schedule_rev` field on `engine_event_t` is never populated in `evt_schedule()`. Events persist across revolutions and can double-fire (HIGH H12).
- **Line 147-156**: Linear scan of all 16 slots on every tooth. At 8000 RPM with 58 teeth/rev, that's 7.7K scans/second. Each scan checks 16 slots with float arithmetic. The total per-tooth overhead is ~1-2µs, which is acceptable.

### `firmware/scheduler/event_scheduler.h` (148 lines)
- Well-documented with usage instructions and angle convention.
- `EVT_QUEUE_SIZE = 16` is "must be power of 2" per comment, but nothing uses this as a mask (no `& (SIZE-1)` indexing). The constraint is unnecessary.

---

## MCPWM Drivers

### `firmware/scheduler/injector_driver.c` (292 lines)
MCPWM injection driver with continuous timers and absolute compare.

- **Line 79**: `int group_id = i / SOC_MCPWM_TIMERS_PER_GROUP;` — ESP32-S3 has `SOC_MCPWM_TIMERS_PER_GROUP = 3`, `SOC_MCPWM_GROUPS = 2`. So channels 0-2 use group 0, channel 3 uses group 1. 4 timers from 2 groups (3+1). This works. But the ignition driver also allocates 4 timers the same way. Total = 4+4 = 8, but ESP32-S3 only has 6 (2 groups x 3). **Channels 6 and 7 will fail** (HIGH H8).
- **Line 176**: `start_ticks = delay_us` — compare values are set directly to `delay_us`. But `delay_us` comes from `engine_control.c`'s `angle_delta_to_delay_us()` which returns a relative delay in microseconds from "now". The MCPWM timer counter is running continuously. The compare should be `current_counter + delay_us`, not just `delay_us`. This means all injection events fire at the wrong time.
- **Line 180**: `if (delay_us <= current_counter)` — this check assumes `delay_us` is an absolute timer value, but it's a relative delay. These semantics are confused.
- **No maximum on-time enforcement**: If `cmp_end` is set past the timer period (30,000,000), the injector stays open until the timer wraps (30 seconds). A hardware watchdog comparator at, say, 20ms would prevent flooding.

### `firmware/scheduler/ignition_driver.c` (241 lines)
MCPWM ignition driver. Similar architecture to injector driver.

- **Line 58-63**: `calculate_dwell_time_hp` uses a stepped lookup based on battery voltage. At 11V: 4.5ms, at 12.5V: 3.5ms, at 14V: 3.0ms. These are reasonable values for modern COP coils.
- **Line 162-166**: `cylinder_id` is 1-based (rejects `< 1 || > 4`), but `get_counter` at line 218 is 0-based (`cylinder_id >= 4` rejects 4). If the caller passes cylinder_id=0 to `schedule`, it's rejected. If the caller passes cylinder_id=4 to `get_counter`, it's rejected. This inconsistency is confirmed at `engine_control.c:668-669` which passes 1-based IDs to `schedule` and 0-based to `get_counter`.
- **Line 175**: `dwell_start_ticks = (target_us > dwell_ticks) ? (target_us - dwell_ticks) : 0` — if `target_us < dwell_ticks`, dwell starts immediately at tick 0 (near the beginning of the 30-second timer cycle). This could fire at completely the wrong time rather than not firing at all. Should return false.
- **No maximum dwell enforcement**: If the spark compare never fires, the coil stays energized for up to 30 seconds. This will destroy the coil and is a fire hazard (HIGH H4).

---

## High-Precision Timing

### `firmware/scheduler/hp_timing.c` (238 lines)
Adaptive phase predictor, hardware latency compensation, and jitter measurement.

- **Line 37-70**: Phase predictor update. Uses an adaptive alpha filter that increases reactivity during acceleration. The math is: `predicted = current + (acceleration * dt * alpha)`. This is a simple first-order predictor with adaptive gain. Reasonable for the use case.
- **Line 47-49**: `if (dt > 100000.0f || dt < 0)` — `dt` is a `float`, computed from `hp_cycles_to_us(timestamp - predictor->last_timestamp)`. Since `timestamp` is `uint32_t` and subtraction wraps correctly, `dt` should never be negative (it's always a positive float). The `< 0` check is dead code.
- **Line 132-133**: `jitter_sum += jitter` — `jitter_sum` is `uint64_t`, `jitter` is `uint32_t`. No overflow concern for practical sample counts.

### `firmware/scheduler/hp_timing.h` (401 lines)
- **Line 50-53**: `hp_get_cycle_count()` reads the CCOUNT register via inline ASM. This is Xtensa-specific and correct for ESP32-S3.
- **Line 354-360**: `hp_delay_us()` busy-waits using CCOUNT. Uses `(int32_t)` cast for the comparison, which correctly handles the 32-bit CCOUNT wrap (every ~17.9 seconds at 240MHz). Good.
- **Line 297-301**: `hp_calculate_schedule_delay` — if `target_cycles < current_cycles`, returns 0. This means "already past deadline" is treated as "fire immediately" rather than "missed, skip". For engine timing, firing immediately is worse than skipping — a spark at the wrong time causes knock.

---

## Sensors

### `firmware/sensors/sensor_processing.c` (350 lines)
ADC continuous-mode sensor acquisition with filtering.

- **Line 1-2**: Broken includes.
- **Line 97-104**: ADC channel assignments: MAP=CH0, TPS=CH1, CLT=CH2, IAT=CH3, O2=CH4, VBAT=CH5, SPARE=CH6. But `hal_pins.h` maps MAP to GPIO 36 (ADC1_CH0), TPS to GPIO 39 (ADC1_CH3), CLT to GPIO 32 (ADC1_CH4), IAT to GPIO 33 (ADC1_CH5). These channel numbers don't match — TPS is configured as ADC_CHANNEL_1 but the pin is ADC1_CH3. **ADC channels don't correspond to GPIOs** on ESP32-S3; this mismatch means TPS reads the wrong physical pin.
- **Line 243-329**: Sensor processing inside a mutex but with seqlock for fast reads. The `__atomic_fetch_add` at line 245 marks write-in-progress, then at line 329 marks complete. Between these, the mutex is held. But `sensor_get_data_fast` at line 175 uses only the seqlock (no mutex), while `sensor_set_config` uses only the mutex. This dual-locking strategy is correct but unusual — the seqlock protects the data, the mutex protects the config.
- **Line 275-278**: TPS filter uses a `static float` declared inside a `switch` case. This is legal C but the static local survives across calls, which is the intended behavior.
- **Line 148-152**: `sensor_stop()` deletes the task before stopping ADC. The ADC DMA may deliver data to the deleted task's stack. Should stop ADC first, then delete task. (HIGH H14)

---

## Lookup Tables

### `firmware/tables/table_16x16.c` (106 lines)
Bilinear interpolation on 16x16 RPM/load tables.

- **Line 4-11**: `find_bin_index` returns the bin where `value < bins[i+1]`. If `value < bins[0]`, returns 0 (clamped to first bin). Correct — but then at line 58, `dx = (rpm - x0) / (x1 - x0)`, and if `rpm < x0` (below first bin), `rpm - x0` underflows as `uint16_t`. **This produces a very large positive `dx`** since it's cast to float, e.g., `(300 - 500) = 65336` as uint16_t. The result is wildly wrong interpolation (CRITICAL C9).
- **Line 83-99**: Checksum is a 16-bit truncated sum of all bins and values. This is extremely weak — any rearrangement of values that preserves the sum passes validation. Acceptable as a corruption detector but not as integrity verification.

---

## Safety / Diagnostics

### `firmware/diagnostics/fault_manager.c` (260 lines)
Safety monitoring, limp mode management, and watchdog.

- **Line 1-3**: Broken includes.
- **Line 68**: `if (temp > (int16_t)CLT_SENSOR_MAX)` — uses `CLT_SENSOR_MAX` (120°C, the sensor range limit) instead of `CLT_OVERHEAT_C` (105°C, the safe operating limit). The engine won't limp until 15°C past the danger point (CRITICAL C7).
- **Line 77-78**: Battery check uses sensor range (`VBAT_SENSOR_MIN = 7.0V`, `VBAT_SENSOR_MAX = 18.0V`). A voltage of 7.1V passes but is dangerously low (HIGH H18).
- **Line 86-96**: `safety_activate_limp_mode()` correctly uses spinlock and only activates once.
- **Line 98-136**: `safety_deactivate_limp_mode()` implements a hysteresis-based recovery: minimum 5 seconds in limp, then conditions must be safe for 2 seconds. This is a good pattern — but nothing calls `safety_deactivate_limp_mode()` in the current codebase. Recovery is never attempted.
- **Line 161-165**: `trigger_panic = false` on the watchdog. A completely locked-up engine control system will not reset (HIGH H17).
- **Line 217-218**: `knock_prot->timing_retard -= 5` on a `uint16_t`. If `timing_retard` is 1-4, this wraps to ~65534 (CRITICAL C8).

---

## HAL

### `firmware/hal/hal_pins.h` (75 lines)
All GPIO assignments in one place. Clean design — but with 5 pin conflicts (CRITICAL C2):

| GPIO | Conflict |
|------|----------|
| 34 | `HAL_PIN_CKP` (crankshaft) vs `HAL_PIN_VBAT` (battery ADC) |
| 18 | `HAL_PIN_IGN_3` (ignition coil 3) vs `HAL_PIN_SD_CLK` (SD card) |
| 5 | `HAL_PIN_CAN_RX` (CAN bus) vs `HAL_PIN_SD_CS` (SD card) |
| 23 | `HAL_PIN_BRAKE` (brake switch) vs `HAL_PIN_SD_MOSI` (SD card) |
| 19 | `HAL_PIN_VVT_INTAKE` (VVT solenoid) vs `HAL_PIN_SD_MISO` (SD card) |

ESP32-S3 has 45 GPIOs (0-21, 26-48). The current mapping uses ESP32 GPIOs (34-39 are input-only on ESP32 but don't exist on S3). This entire pin map needs to be rewritten for ESP32-S3.

### `firmware/hal/hal_gpio.h` (99 lines)
Zero-overhead inline register writes. Correct use of W1TS/W1TC for atomic set/clear.

- **Line 57-58**: `HAL_Injector_Set(channel, active)` — no bounds check on `channel`. Passing `channel >= 4` reads past the `pins[4]` array (stack or adjacent global data) and writes an arbitrary GPIO (HIGH H16).
- **Line 67-71**: Same issue for `HAL_Ignition_Set`.

---

## Cross-Core Data Exchange

### `firmware/utils/atomic_buffer.h` (78 lines)
Lock-free seqlock for ISR-to-task data transfer.

- **Lines 44-48**: Writer uses `__asm__ volatile("" ::: "memory")` — compiler barriers only. On dual-core ESP32-S3, the Xtensa LX7 has a weakly-ordered memory model for cross-core access. Core 1 can see the updated `sequence` before the `memcpy` data lands in shared memory. **Need `__sync_synchronize()` or Xtensa `memw` instruction** between the sequence update and data write (HIGH H9).
- **Note**: The seqlocks in `engine_control.c` (runtime_state, injection_diag) correctly use `__atomic_fetch_add` with `__ATOMIC_RELEASE/__ATOMIC_ACQUIRE` ordering, which provides proper hardware barriers. The generic `atomic_buffer.h` is the one with the bug.

---

## Communications (Summary — detailed analysis in first pass)

### `firmware/comms/can_wideband.c`
- Broken includes (lines 1, 7, 8)
- CAN filter accepts all IDs — no authentication for EOIT calibration commands
- `vTaskDelete` from outside the task is unsafe

### `firmware/comms/espnow_link.c`
- XOR checksum provides zero tamper protection
- No peer authentication for configuration changes
- Mutex created but never actually used (`xSemaphoreTake`/`xSemaphoreGive` absent)
- `g_espnow.started` is plain `bool`, not `volatile`

### `firmware/comms/tunerstudio.c`
- Checksum verification always fails (XOR includes stored checksum)
- Type-pun buffer overwrite at `(size_t *)&resp->param_size`
- Authentication bypassed — every client gets full permissions
- Streaming task function exists but is never started

---

## What's Done Well

Despite the issues above, the codebase shows several strong engineering decisions:

1. **Angle-based scheduling** avoids timing drift during RPM transients — the right approach for an ECU.
2. **MCPWM absolute compare with continuous timers** eliminates the jitter from timer restart cycles. This is a meaningful improvement over the start/stop pattern used by most hobbyist ECU projects.
3. **ETM-based hardware capture** for crankshaft timing provides sub-microsecond precision without ISR overhead. This is a sophisticated use of ESP32-S3 peripherals.
4. **Planner/executor separation** with a ring buffer decouples sensor processing from hardware dispatch, preventing sensor read latency from affecting ignition/injection timing.
5. **EOIT calibration system** with NVS persistence, CRC validation, config versioning, and v1-to-v2 migration is well-engineered for a tunable ECU.
6. **Comprehensive rollback on init failure** in `engine_control_init()` tracks 11 boolean flags to cleanly undo partial initialization. This is more robust than most embedded init sequences.
7. **LTFT (Long-Term Fuel Trim) learning** with stability detection, EMA filtering, and table write-back is a real autotune feature.
8. **Performance instrumentation** (p95/p99 latency, deadline miss counting, queue depth tracking) provides the visibility needed to diagnose real-time issues on a running engine.
9. **Adaptive phase predictor** in `hp_timing.c` adjusts its filter coefficient based on acceleration magnitude — more reactive during transients, more stable at steady state.
10. **Good use of `portENTER_CRITICAL` / `portENTER_CRITICAL_ISR`** throughout the ISR paths for cross-core spinlock safety
