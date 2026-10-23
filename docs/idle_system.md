# Idle Air Control (Idle) System Architecture

The idle subsystem keeps the engine running smoothly when the driver is off the throttle.
It manipulates an Idle Air Control (IAC) actuator - typically a stepper motor, DC motor, or
PWM solenoid - to admit just enough air past the closed throttle plate, and may also
nudge ignition timing for fast-response trim.

## The Idle Control Pipeline

1.  **Target RPM**: A target idle RPM is interpolated from a coolant-temperature curve
    (cold engines idle higher), with an optional bump while the A/C button is pressed.
    Hysteresis thresholds (`IdleEntryRpm` / `IdleExitRpm`) are computed to prevent
    oscillation between idle and coasting states.
2.  **Phase Classification**: Engine state is classified as `Cranking`, `CrankToIdleTaper`,
    `Idling`, `Coasting`, or `Running` based on RPM, TPS, VSS, and the cranking taper.
3.  **Open-Loop Position**: A base IAC position is computed from a CLT/RPM correction
    table, with additive bumps for A/C, cooling fans, antilag, Lua scripts, and TPS/RPM
    tapers. While cranking, a dedicated CLT-only curve is used.
4.  **Closed-Loop PID**: When in `Idling` phase with valid TPS in automatic mode, an RPM
    PID adds a correction on top of the open-loop value, with anti-windup, dead-zone, and
    soft-entry handling.
5.  **Timing Trim**: An optional separate PID applies small ignition-advance corrections
    at idle to react faster than airflow changes can.
6.  **Modeled-Flow Idle**: An alternate path converts the desired IAC % into a target
    airmass, splits the high-frequency component into the timing PID, and maps the
    remaining airflow back to a valve position via an idle-flow estimate table.
7.  **Actuation**: The final percentage is sent to the IAC hardware (`applyIACposition`).

## Key Files

- `firmware/controllers/actuators/idle_thread.cpp`: Main idle controller logic - phase
  detection, target RPM, open-loop, closed-loop PID, modeled-flow, and the fast-callback
  entry point.
- `firmware/controllers/actuators/idle_thread.h`: `IdleController` / `IIdleController`
  interface and `Phase` enum.
- `firmware/controllers/actuators/idle_hardware.cpp`: Hardware abstraction that drives
  the actual IAC actuator from the controller's output percentage.
- `firmware/controllers/actuators/dc_motors.cpp` and `stepper.cpp`: Concrete IAC actuator
  drivers.

## Implementation Notes

- **Hysteresis**: Re-enter idle when RPM drops below `target + idlePidRpmUpperLimit`, but
  declare coasting only above `target + 1.5 * idlePidRpmUpperLimit` to avoid flapping.
- **Cranking Taper**: After the crank exit the ETB floor is blended over 1 s from the
  cranking opening (`etb_idle_max`) to the idle I, then held there (never below
  `etb_idle_min`, even above the idle RPM band) until the taper ends. The end is
  `etbTaperMs` (page 0, 354) or, when 0, the end of the afterstart enrichment (CLT
  dependent, capped at 10 s). The idle I does not decay below `etb_idle_min` while the
  taper runs, so the floor no longer closes to 0 % and steps back at idle re-entry. After
  the taper the blade follows the pedal and the idle RPM band as usual. Host-tested only;
  the real RPM response needs the engine or the ETB on the bench.
- **A/C Pre-Bump**: We respond to the A/C *button* rather than the relay, because the relay
  is intentionally delayed so airflow bump arrives before the compressor load.
- **Dashpot**: After a throttle release, TPS used for the taper is held and then decayed
  exponentially toward zero, smoothing the return-to-idle and reducing stall risk.
- **Timing PID**: Only active in `Phase::Idling`; resets otherwise to avoid carrying stale
  integration into transients.
