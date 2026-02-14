# EOIT Calibration and Diagnostics

## Formula

The firmware computes injection end target (EOIT) using:

`EOIT_deg = ((Boundary + Normal) * 90) - 784`

- `Boundary`: base calibration value
- `Normal`: main timing value (single value or 16x16 map)
- `fallback_normal`: value used when sync is partial/fallback

Practical rule:
- `+0.1` on `Normal` shifts EOIT by about `+9 deg`.

## Angular Reference

- Sequential mode uses a **720 deg** engine-cycle reference.
- Semi-sequential fallback uses a **360 deg** reference.
- Injector scheduling uses `SOI = EOI - pulsewidth_deg`.

## Safety Limits

Values are clamped before being saved/applied:

- `Boundary`: `[0.0, 20.0]`
- `Normal`: `[-8.0, 16.0]`
- `fallback_normal`: `[-8.0, 16.0]`

## EOIT Map Strategy (RPM x Load)

A 16x16 EOIT-normal map is implemented.

- Map enable/disable: `engine_control_set_eoit_map_enabled(bool)`
- Map cell update: `engine_control_set_eoit_map_cell(rpm_idx, load_idx, normal)`
- Map cell read: `engine_control_get_eoit_map_cell(rpm_idx, load_idx, &normal)`

When enabled, `normal_used` is interpolated from the map for current RPM/load.

## Diagnostics Telemetry

The firmware exposes real injection diagnostics:

- API: `engine_control_get_injection_diag(...)`
- Fields include:
  - `eoit_target_deg`, `eoit_fallback_target_deg`
  - `normal_used`, `boundary`
  - `soi_deg[4]`, `delay_us[4]`
  - `sync_acquired`, `map_mode_enabled`

## TWAI/CAN Runtime Configuration

Command ID: `0x6E0`
Response ID: `0x6E1`

Commands:

1. `0xA1` Set calibration
- Payload: `[cmd, boundary_x100(i16), normal_x100(i16), fallback_x100(i16)]`

2. `0xA2` Set map enable
- Payload: `[cmd, enable(0|1)]`

3. `0xA3` Set map cell
- Payload: `[cmd, rpm_idx(u8), load_idx(u8), normal_x100(i16)]`

4. `0xA4` Get diag
- Response: `[cmd, status, eoit_x10(i16), normal_x100(i16), delay_cyl1_us(u16)]`

5. `0xA5` Get calibration
- Response: `[cmd, status, boundary_x100(i16), normal_x100(i16), fallback_x100(i16)]`

Status:
- `0` = OK
- `1` = error
