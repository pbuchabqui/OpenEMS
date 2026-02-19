#!/usr/bin/env bash
# ============================================================================
# OpenEMS — GitHub Issues Batch Creator
# ============================================================================
# Run this script from a machine with `gh` CLI authenticated:
#   chmod +x github-issues.sh && ./github-issues.sh
#
# Prerequisites:
#   gh auth login
#   gh label create critical --color B60205 --description "Will cause engine damage, won't compile, or defeats safety"
#   gh label create high     --color D93F0B --description "Incorrect behavior, data corruption, or timing errors"
#   gh label create security --color 5319E7 --description "Authentication, authorization, or integrity bypass"
#   gh label create systemic --color 0E8A16 --description "Pattern that recurs across multiple files"
# ============================================================================

set -euo pipefail
REPO="pbuchabqui/OpenEMS"

create_issue() {
  local title="$1"
  local labels="$2"
  local body="$3"
  echo "Creating: $title"
  gh issue create -R "$REPO" --title "$title" --label "$labels" --body "$body"
  sleep 1  # rate limit
}

# ── Setup labels ────────────────────────────────────────────────────────────
echo "=== Creating labels ==="
gh label create critical --repo "$REPO" --color B60205 --description "Will cause engine damage, won't compile, or defeats safety" 2>/dev/null || true
gh label create high     --repo "$REPO" --color D93F0B --description "Incorrect behavior, data corruption, or timing errors" 2>/dev/null || true
gh label create security --repo "$REPO" --color 5319E7 --description "Authentication, authorization, or integrity bypass" 2>/dev/null || true
gh label create systemic --repo "$REPO" --color 0E8A16 --description "Pattern that recurs across multiple files" 2>/dev/null || true
gh label create safety   --repo "$REPO" --color FF0000 --description "Could cause physical damage to engine or vehicle" 2>/dev/null || true

# ============================================================================
# CRITICAL ISSUES (C1–C12)
# ============================================================================
echo ""
echo "=== Creating CRITICAL issues ==="

create_issue \
  "[C1] Broken #include directives — code won't compile" \
  "critical,bug" \
  "$(cat <<'BODY'
## Description

Every `.c` file is missing the opening double-quote on local `#include` directives:

```c
#include sync.h"          // ← missing opening quote
#include engine_control.h" // ← same
```

Should be:
```c
#include "sync.h"
#include "engine_control.h"
```

## Affected files

- `trigger_60_2.c` (lines 1-3)
- `can_wideband.c` (lines 1, 7, 8)
- `config_manager.c` (lines 1-2)
- `sensor_processing.c` (lines 1-2)
- `fuel_calc.c` (line 1-3)
- `fuel_injection.c` (lines 10-15)
- `fault_manager.c` (lines 1-3)
- `table_16x16.c` (line 1)

## Impact

**Hard compilation blocker.** Nothing builds.

## Fix

Add the opening `"` to every local include directive.
BODY
)"

create_issue \
  "[C2] GPIO pin conflicts in hal_pins.h — 5 pins double-assigned" \
  "critical,bug,safety" \
  "$(cat <<'BODY'
## Description

Multiple GPIOs are assigned to two different peripherals in `firmware/hal/hal_pins.h`:

| GPIO | Conflict |
|------|----------|
| 34 | `HAL_PIN_CKP` (crankshaft) vs `HAL_PIN_VBAT` (battery ADC) |
| 18 | `HAL_PIN_IGN_3` (ignition coil 3) vs `HAL_PIN_SD_CLK` (SD card) |
| 5  | `HAL_PIN_CAN_RX` vs `HAL_PIN_SD_CS` |
| 23 | `HAL_PIN_BRAKE` vs `HAL_PIN_SD_MOSI` |
| 19 | `HAL_PIN_VVT_INTAKE` vs `HAL_PIN_SD_MISO` |

Additionally, GPIOs 34-39 are input-only pins on ESP32 that **do not exist on ESP32-S3**. The entire pin map targets the wrong MCU.

## Impact

- **GPIO 34 conflict**: SD card reads will interfere with crankshaft signal, destroying sync.
- **GPIO 18 conflict**: SD card clock toggles will fire ignition coil 3 randomly.
- The pin map is incompatible with the ESP32-S3 target MCU.

## Fix

Rewrite `hal_pins.h` for ESP32-S3 GPIO range (0-21, 26-48), eliminating all conflicts.
BODY
)"

create_issue \
  "[C3] Event scheduler calls non-existent functions — won't link" \
  "critical,bug" \
  "$(cat <<'BODY'
## Description

`firmware/scheduler/event_scheduler.c:88-97` calls functions that don't exist:

```c
mcpwm_injection_hp_schedule_open(evt->cylinder, fire_abs);
mcpwm_injection_hp_schedule_close(evt->cylinder, fire_abs);
mcpwm_ignition_hp_schedule_dwell(evt->cylinder, fire_abs);
mcpwm_ignition_hp_schedule_spark(evt->cylinder, fire_abs);
```

The actual driver APIs are:
- `mcpwm_injection_hp_schedule_one_shot_absolute(cylinder_id, delay_us, pulsewidth_us, current_counter)`
- `mcpwm_ignition_hp_schedule_one_shot_absolute(cylinder_id, target_us, rpm, battery_voltage, current_counter)`

## Impact

Linker error. The event scheduler cannot dispatch events to hardware.

## Fix

Update `fire_event()` to call the actual driver APIs with correct signatures.
BODY
)"

create_issue \
  "[C4] TDC offset is never applied — all events fire at wrong crank angle" \
  "critical,bug,safety" \
  "$(cat <<'BODY'
## Description

`firmware/scheduler/event_scheduler.c:136`:

```c
float tooth_angle = rev_offset + (float)tooth_index * s_deg_per_tooth;
```

`s_tdc_offset_deg` is set by `evt_set_tdc_offset()` (line 249) but is **never subtracted** from the angle computation. This is the most fundamental calibration parameter — it tells the ECU where TDC is relative to the trigger wheel's missing teeth.

## Impact

All injection and ignition events fire at an offset from the intended crank angle. A 114° TDC offset (common for 60-2 wheels) means spark fires 114° from where it should, causing severe engine damage or failure to run.

## Fix

```c
float tooth_angle = rev_offset + (float)tooth_index * s_deg_per_tooth - s_tdc_offset_deg;
```
BODY
)"

create_issue \
  "[C5] engine_control_stop() is a no-op — cannot safely stop engine" \
  "critical,bug,safety" \
  "$(cat <<'BODY'
## Description

`firmware/control/engine_control.c:1322-1328`: `engine_control_stop()` just logs and returns. Tasks keep running, injectors keep firing.

## Impact

If called for safety reasons (overheat, over-rev, sync loss), the engine continues running uncontrolled. There is no way to programmatically shut down fuel and ignition.

## Fix

Implement a fail-safe shutdown sequence:
1. Force all ignition coils de-energized (`mcpwm_generator_set_force_level(gen, 0, true)`)
2. Force all injectors closed
3. Signal planner/executor tasks to stop
4. Set a global "engine stopped" flag checked in all scheduling paths
BODY
)"

create_issue \
  "[C6] Limp mode sets RPM limit but nothing reads it" \
  "critical,bug,safety" \
  "$(cat <<'BODY'
## Description

`safety_activate_limp_mode()` in `fault_manager.c` sets `rpm_limit = 3000`, but no code in the planner, executor, or scheduler checks this limit. When limp mode triggers, `build_plan()` returns `ESP_FAIL`, causing the executor to use stale plans from the ring buffer.

## Impact

Limp mode doesn't limit the engine. Overheating, over-rev, and sensor failures trigger limp mode activation, but the engine runs on the last good plan indefinitely — which could be at redline RPM with aggressive timing.

## Fix

1. When limp mode is active, `build_plan()` should produce a degraded plan (3000 RPM limit, 5° retarded timing, rich lambda target) rather than returning `ESP_FAIL`.
2. Add fuel cut above `rpm_limit` in the executor.
BODY
)"

create_issue \
  "[C7] Overheat threshold uses CLT_SENSOR_MAX (120°C) instead of CLT_OVERHEAT_C (105°C)" \
  "critical,bug,safety" \
  "$(cat <<'BODY'
## Description

`fault_manager.c:68`:
```c
if (temp > (int16_t)CLT_SENSOR_MAX) {  // 120°C — sensor range limit
```

Should use `CLT_OVERHEAT_C` (105°C), the safe operating temperature limit.

## Impact

The engine won't enter limp mode until 15°C past the safe limit. Sustained operation at 106-119°C can cause head gasket failure, warped heads, or piston seizure.
BODY
)"

create_issue \
  "[C8] Knock retard unsigned underflow — uint16_t wraps to 65534" \
  "critical,bug,safety" \
  "$(cat <<'BODY'
## Description

`fault_manager.c:217-218`:
```c
if (knock_prot->timing_retard > 0) {
    knock_prot->timing_retard -= 5;  // uint16_t
}
```

If `timing_retard` is 1-4, subtracting 5 wraps to ~65534.

## Impact

A single knock event adds 10 to `timing_retard`. When knock stops, the retard decrements by 5 per cycle. If `timing_retard` is 3 after several decrements, `3 - 5 = 65531` (uint16_t). This applies massive timing retard (6553.1°), which is nonsensical.

## Fix

```c
knock_prot->timing_retard = (knock_prot->timing_retard > 5) ?
    knock_prot->timing_retard - 5 : 0;
```
BODY
)"

create_issue \
  "[C9] Table interpolation underflow for RPM below first bin" \
  "critical,bug" \
  "$(cat <<'BODY'
## Description

`table_16x16.c:58`:
```c
dx = (float)(rpm - x0) / (float)(x1 - x0);
```

If `rpm < x0` (RPM below first bin), `rpm - x0` underflows as `uint16_t`. Example: `(300 - 500) = 65336` as uint16_t, producing `dx = 65336.0 / 500.0 = 130.7`, wildly wrong.

## Impact

VE, ignition timing, and lambda target are all wrong at low RPM (cranking, idle). This can cause lean conditions during startup or excessive advance at idle.

## Fix

Cast to `int32_t` before subtraction, or clamp `rpm` to `x0` before computing `dx`.
BODY
)"

create_issue \
  "[C10] Hardcoded 13.5V battery voltage for injector dead-time" \
  "critical,bug,safety" \
  "$(cat <<'BODY'
## Description

`fuel_injection.c:65-66`:
```c
float battery_voltage = 13.5f;  // never reads actual VBAT sensor
float temperature = 25.0f;
```

Same hardcoded values appear at `engine_control.c:668`.

## Impact

Injector dead-time compensation is wrong at any voltage other than 13.5V. During cranking (~10V), injectors open slower; under-compensating by ~0.5ms causes lean mixture. At high alternator output (~14.5V), over-compensating causes rich conditions.

## Fix

Read `sensors->vbat_dv / 10.0f` for actual battery voltage. Read `sensors->clt_c` or `sensors->iat_c` for temperature.
BODY
)"

create_issue \
  "[C11] TunerStudio checksum verification always fails" \
  "critical,bug" \
  "$(cat <<'BODY'
## Description

`tunerstudio.c:439`: The receiver computes XOR over all header bytes **including the stored checksum**, producing 0. It then compares 0 against the non-zero stored checksum, which always fails.

**Sender**: sets checksum=0, computes XOR of header → stores result X.
**Receiver**: computes XOR of header (including X) → gets X ⊕ X = 0 for those bits → result is 0, not X.

## Impact

Every non-trivial TunerStudio message is rejected. The tuning protocol is non-functional for receiving.

## Fix

Zero the checksum field before computing the check on the receiver side (same as the sender does).
BODY
)"

create_issue \
  "[C12] Type-pun buffer overwrite in TunerStudio param handler" \
  "critical,bug" \
  "$(cat <<'BODY'
## Description

`tunerstudio.c:235`:
```c
(size_t *)&resp->param_size
```

`resp->param_size` is `uint16_t` (2 bytes). `size_t` is 4 bytes on ESP32. Writing through a `size_t *` overwrites 2 adjacent bytes on the stack.

## Impact

Stack corruption in the tuning message handler. Could corrupt return address or other local variables, leading to crashes or undefined behavior.

## Fix

Use a local `size_t` variable and copy back to `resp->param_size` after the callback returns.
BODY
)"

# ============================================================================
# HIGH ISSUES (selected most impactful)
# ============================================================================
echo ""
echo "=== Creating HIGH issues ==="

create_issue \
  "[H1] Timer domain mismatch — event_scheduler uses system time, MCPWM uses counter ticks" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

`event_scheduler.c:84`: `fire_abs = base_time_us + delay_us` uses system-time (esp_timer) domain, but MCPWM timers run in their own counter domain (0 to 30,000,000 ticks at 1MHz).

## Impact

All scheduled times passed to MCPWM compare registers are in the wrong domain. Events fire at arbitrary times.

## Fix

Convert system time to MCPWM counter ticks, or pass `current_counter` (from `mcpwm_timer_get_phase()`) and compute `current_counter + delay_ticks`.
BODY
)"

create_issue \
  "[H3] normalize_angle() infinite loop on NaN/Inf" \
  "high,bug,safety" \
  "$(cat <<'BODY'
## Description

`event_scheduler.c:53-57`:
```c
while (a >= 720.0f) a -= 720.0f;
while (a <    0.0f) a += 720.0f;
```

If `a` is NaN, +Inf, or -Inf (from corrupted sensor data or division by zero), neither condition will terminate. This runs in ISR context on Core 0.

## Impact

Core 0 locks up permanently. No more tooth processing, no injection/ignition events. Engine stalls immediately.

## Fix

Add a guard: `if (!isfinite(a)) return 0.0f;` or use `fmodf` with a single adjustment.
BODY
)"

create_issue \
  "[H4] No maximum dwell time — fire hazard from stuck coil" \
  "high,bug,safety" \
  "$(cat <<'BODY'
## Description

In `ignition_driver.c`, if the spark compare event never fires (e.g., timer overflow, scheduling error), the ignition coil stays energized indefinitely. The MCPWM timer period is 30 seconds.

## Impact

Ignition coils overheat and can catch fire with extended dwell (>10ms for most COP coils). This is a physical safety hazard.

## Fix

Add a hardware safety comparator at max dwell (e.g., 5ms) that forces the output LOW, or add a software watchdog timer that de-energizes coils after max_dwell_ms.
BODY
)"

create_issue \
  "[H5] No maximum injector-on time — hydro-lock risk" \
  "high,bug,safety" \
  "$(cat <<'BODY'
## Description

Same as H4 for injectors. `injector_driver.c` has no maximum on-time enforcement. If `cmp_end` exceeds the timer period (30M ticks), the injector stays open for up to 30 seconds.

## Impact

A stuck-open injector floods the cylinder with fuel, risking hydro-lock (liquid-locking the piston), catalytic converter damage, and oil dilution.

## Fix

Add a safety comparator or software watchdog at max injector on-time (e.g., `PW_MAX_US` = 18000µs).
BODY
)"

create_issue \
  "[H6] Cylinder ID convention inconsistency (0-based vs 1-based)" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

- `ignition_driver.c:166`: `schedule_one_shot_absolute` uses **1-based** IDs (rejects `< 1 || > 4`), indexes array as `[cylinder_id - 1]`
- `ignition_driver.c:218`: `get_counter` uses **0-based** IDs (rejects `>= 4`), indexes array as `[cylinder_id]`
- `injector_driver.c:170`: `schedule_one_shot_absolute` uses **0-based** IDs (rejects `>= 4`)
- `engine_control.c:668`: passes 1-based to ignition, 0-based to injection

## Impact

Calling `get_counter(4)` is rejected when it should access cylinder 4 (if 1-based convention). Off-by-one cylinder misfire.

## Fix

Standardize on 0-based (0-3) everywhere.
BODY
)"

create_issue \
  "[H8] MCPWM resource exhaustion — 8 timers needed, ESP32-S3 has 6" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

Both `injector_driver.c` and `ignition_driver.c` allocate 4 MCPWM timers each (8 total). ESP32-S3 has 2 groups × 3 timers = 6 timers.

Timer allocation: injectors take group 0 (timers 0-2) + group 1 (timer 0). Ignition then tries group 0 (timers 0-2) — already taken.

## Impact

Channels 6 and 7 fail at init. At most 6 of 8 outputs can work.

## Fix

Share timers between injection and ignition (each operator can share a timer), or use 2 comparators per operator with a single shared timer per group.
BODY
)"

create_issue \
  "[H9] Seqlock in atomic_buffer.h uses compiler barriers, not hardware barriers" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

`atomic_buffer.h:44-48` uses `__asm__ volatile("" ::: "memory")` which is a **compiler barrier only**. On dual-core ESP32-S3 (Xtensa LX7), cross-core memory visibility requires hardware barriers.

Note: The seqlocks in `engine_control.c` correctly use `__atomic_fetch_add` with `__ATOMIC_RELEASE/__ATOMIC_ACQUIRE`, which provides hardware barriers. Only `atomic_buffer.h` is affected.

## Impact

Core 1 can see the updated sequence number before the `memcpy` data is visible, reading a partially stale snapshot.

## Fix

Replace compiler barriers with `__sync_synchronize()` or Xtensa `memw` instruction.
BODY
)"

create_issue \
  "[H10] No ignition advance clamping — table corruption causes runaway advance" \
  "high,bug,safety" \
  "$(cat <<'BODY'
## Description

`IGN_ADVANCE_MIN_DEG` (-5) and `IGN_ADVANCE_MAX_DEG` (45) are defined in `engine_config.h` but never checked in `engine_control.c` or `ignition_timing.c`.

## Impact

A corrupt table entry of 65535 (from flash corruption, bad CAN command, etc.) produces 6553.5° advance, which is nonsensical. Excessive advance causes knock and potential engine destruction.

## Fix

Clamp ignition advance to `[IGN_ADVANCE_MIN_DEG, IGN_ADVANCE_MAX_DEG]` after table lookup.
BODY
)"

create_issue \
  "[H14] sensor_stop() deletes task before stopping ADC DMA" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

`sensor_processing.c:148-152`:
```c
vTaskDelete(g_sensor_task_handle);   // task deleted
// ...
adc_continuous_stop(adc_handle);     // ADC still running
adc_continuous_deinit(adc_handle);   // DMA may still deliver to deleted task's stack
```

## Impact

The ADC DMA callback may reference the deleted task's stack, causing memory corruption.

## Fix

Stop ADC first, then delete the task:
```c
adc_continuous_stop(adc_handle);
vTaskDelete(g_sensor_task_handle);
adc_continuous_deinit(adc_handle);
```
BODY
)"

create_issue \
  "[H15] Double-scheduling in fuel_injection_schedule_sequential()" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

`fuel_injection.c:102-127`: The function calls `fuel_injection_schedule_eoi_ex()` per cylinder in a loop (which triggers hardware scheduling via `schedule_one_shot_absolute`), then also calls `mcpwm_injection_hp_schedule_sequential_absolute()` — which re-schedules all 4 cylinders again.

## Impact

Cylinder 0 is double-scheduled with potentially different timing parameters (the sequential call uses `pulsewidth_us[0]` for all cylinders but `offsets[]` from individual calls).

## Fix

Either use per-cylinder scheduling OR batch scheduling, not both.
BODY
)"

create_issue \
  "[H17] Watchdog trigger_panic=false — locked system won't reset" \
  "high,bug,safety" \
  "$(cat <<'BODY'
## Description

`fault_manager.c:165`:
```c
.trigger_panic = false,
```

## Impact

If the engine control system completely locks up (infinite loop, deadlock), the watchdog timer expires but does **not** trigger a system reset. The engine continues in an undefined state.

## Fix

Set `trigger_panic = true` so a watchdog timeout triggers an ESP32 panic handler and system reset.
BODY
)"

# ============================================================================
# SECURITY ISSUES (S1–S4)
# ============================================================================
echo ""
echo "=== Creating SECURITY issues ==="

create_issue \
  "[S1] No CAN bus authentication for EOIT calibration commands" \
  "security,safety" \
  "$(cat <<'BODY'
## Description

`can_wideband.c`:
- CAN filter is `TWAI_FILTER_CONFIG_ACCEPT_ALL()` — accepts all CAN IDs
- EOIT command handler at ID `0x6E0` accepts `SET_CAL`, `SET_MAP_ENABLE`, `SET_MAP_CELL` without authentication
- No rate limiting on calibration changes
- No bounds validation on `rpm_idx` and `load_idx` in `SET_MAP_CELL`

## Impact

Any device on the CAN bus can alter engine calibration (fuel maps, timing), potentially causing lean conditions, excessive advance, or engine damage.

## Fix

1. Add CAN ID filtering to accept only known IDs
2. Add a challenge-response handshake before accepting calibration commands
3. Validate `rpm_idx < 16` and `load_idx < 16` before array access
4. Rate-limit calibration changes
BODY
)"

create_issue \
  "[S2] No ESP-NOW authentication — wireless ECU recalibration from any nearby device" \
  "security,safety" \
  "$(cat <<'BODY'
## Description

`espnow_link.c/h`:
- Message types `ESPNOW_MSG_TABLE_UPDATE` (0x12) and `ESPNOW_MSG_PARAM_SET` (0x13) allow remote configuration
- XOR checksum provides zero tamper protection
- ESP-NOW encryption is disabled by default (`encrypt = false`)
- No MAC address whitelist in receive path
- Mutex created but never used

## Impact

Any ESP32 within Wi-Fi range (~100m) can modify engine calibration wirelessly.

## Fix

1. Enable ESP-NOW encryption (per-peer PMK)
2. Replace XOR checksum with HMAC-SHA256 or AES-CMAC
3. Add MAC address whitelist
4. Require authentication before accepting TABLE_UPDATE/PARAM_SET
BODY
)"

create_issue \
  "[S3] TunerStudio authentication bypassed — full access for all clients" \
  "security" \
  "$(cat <<'BODY'
## Description

`tunerstudio.c:183, 200-204`:
```c
ack.auth_required = 0;                  // auth not required
g_tuning.session.authenticated = true;  // always authenticated
g_tuning.session.permissions = 0xFFFF;  // full permissions
```

Every connecting client immediately gets full read/write access to all ECU parameters.

## Impact

Any connected client can modify safety limits, fuel maps, ignition timing, and (when implemented) flash firmware.
BODY
)"

create_issue \
  "[S4] CLI admin mode has no password check" \
  "security" \
  "$(cat <<'BODY'
## Description

`cli_interface.c:1031-1037`: `cli_enter_admin()` accepts any input:
```c
(void)password;  // password parameter ignored
```

## Impact

Anyone with USB access gets full admin privileges (config changes, factory reset, safety limit modification).

## Fix

Implement password verification or at minimum a hardware jumper/button requirement for admin mode.
BODY
)"

# ============================================================================
# SYSTEMIC ISSUES
# ============================================================================
echo ""
echo "=== Creating SYSTEMIC issues ==="

create_issue \
  "[SYS1] Unsafe FreeRTOS task termination pattern across all modules" \
  "systemic,bug" \
  "$(cat <<'BODY'
## Description

Multiple modules use this unsafe pattern to stop tasks:

```c
some_flag = false;
vTaskDelay(pdMS_TO_TICKS(100));  // hope the task exits in 100ms
g_task_handle = NULL;
```

## Affected files

- `can_wideband.c` (`twai_lambda_deinit`)
- `espnow_link.c` (`espnow_link_stop`)
- `sd_logger.c` (`data_logger_stop`)
- `cli_interface.c` (`cli_stop`)
- `engine_control.c` (`engine_control_deinit` — uses `vTaskDelete` externally)

## Impact

Race condition: if the task hasn't exited within the delay, the handle is dangled. The task may hold spinlocks or mutexes, causing deadlock.

## Fix

Use a task notification or event group:
1. Set flag to false
2. Task calls `xTaskNotifyGive(caller)` before `vTaskDelete(NULL)`
3. Caller waits with `ulTaskNotifyTake(pdTRUE, timeout)`
BODY
)"

create_issue \
  "[SYS2] ADC channel-to-GPIO mapping mismatch" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

`sensor_processing.c:97-104` configures ADC channels as sequential (CH0-CH6), but `hal_pins.h` maps sensors to non-sequential GPIO/ADC channels:

| Sensor | Code uses | Pin is | ADC Channel |
|--------|-----------|--------|-------------|
| MAP | ADC_CHANNEL_0 | GPIO 36 | ADC1_CH0 | OK |
| TPS | ADC_CHANNEL_1 | GPIO 39 | ADC1_CH3 | WRONG |
| CLT | ADC_CHANNEL_2 | GPIO 32 | ADC1_CH4 | WRONG |
| IAT | ADC_CHANNEL_3 | GPIO 33 | ADC1_CH5 | WRONG |

## Impact

TPS, CLT, and IAT read from wrong physical ADC channels. Fuel and ignition calculations use wrong sensor values.

## Fix

Use the correct ADC channels matching the GPIO assignments:
```c
{.channel = ADC_CHANNEL_0, ...}, // MAP  (GPIO 36)
{.channel = ADC_CHANNEL_3, ...}, // TPS  (GPIO 39)
{.channel = ADC_CHANNEL_4, ...}, // CLT  (GPIO 32)
{.channel = ADC_CHANNEL_5, ...}, // IAT  (GPIO 33)
```
BODY
)"

create_issue \
  "[SYS3] log_entry_t type name collision between logger.h and sd_logger.h" \
  "high,bug" \
  "$(cat <<'BODY'
## Description

Both `utils/logger.h` and `logging/sd_logger.h` define a type named `log_entry_t` with completely different fields. Including both headers in any translation unit causes a compilation error.

## Fix

Rename one of them, e.g., `sd_log_entry_t` in `sd_logger.h`.
BODY
)"

echo ""
echo "=== Done! ==="
echo "Created issues for OpenEMS peer review findings."
