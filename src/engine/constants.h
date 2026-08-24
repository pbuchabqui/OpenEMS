#pragma once
#include <cstdint>

namespace ems::engine {

// ============================================================================
// Timing Constants
// ============================================================================

/** TIM5 timer tick period in nanoseconds (62.5 MHz = 16 ns/tick) */
inline constexpr uint32_t kTim5TickPeriodNs = 16u;

/** Scheduler timer tick period in nanoseconds (10 MHz = 100 ns/tick) */
inline constexpr uint32_t kSchedulerTickPeriodNs = 100u;

/** Minimum scheduler advance warning in ticks (typically 5 µs @ 10 MHz = 50 ticks) */
inline constexpr uint32_t kMinCompareLeadTicks = 50u;

/** Maximum safe ignition timer delta for 16-bit mode */
inline constexpr uint32_t kTimIgnMaxDelta16 = 0xFFFFu;

// ============================================================================
// Engine Physical Constants (60-2 tooth wheel)
// ============================================================================

/** Total teeth on 60-2 wheel including missing teeth */
inline constexpr uint16_t kTeethPerRev60_2 = 60u;

/** Actual physical teeth (60 - 2 missing) */
inline constexpr uint16_t kRealTeeth60_2 = 58u;

/** Angle per tooth in millidegrees (720° / 58 teeth ≈ 12.4138° = 12413.8 mdeg) */
inline constexpr uint32_t kToothAngleMillideg = (720000u / kRealTeeth60_2);

/** Number of cylinders in engine */
inline constexpr uint8_t kCylinderCount = 4u;

/** Crank degrees per engine cycle (4-stroke) */
inline constexpr uint32_t kCrankDegreesPerCycle = 720u;

// ============================================================================
// Fuel Calculation Constants
// ============================================================================

/** Air density at 1.00 bar, 25°C in mg/cc × 1000 (1.184 mg/cc) */
inline constexpr uint32_t kAirDensityMgPerCcX1000 = 1184u;

/** Gasoline fuel density in mg/cc */
inline constexpr uint32_t kFuelDensityMgPerCc = 740u;

/** Stoichiometric air-fuel ratio × 100 (14.64:1) */
inline constexpr uint16_t kStoichAfrX100 = 1464u;

/** Default VE table value (percentage × 100) */
inline constexpr uint16_t kDefaultVeX100 = 8000u;  // 80.0%

/** Minimum base pulse width in microseconds */
inline constexpr uint32_t kMinBasePwUs = 100u;

/** Maximum base pulse width in microseconds */
inline constexpr uint32_t kMaxBasePwUs = 20000u;

// ============================================================================
// Sensor Limits & Defaults
// ============================================================================

/** Coolant temperature minimum in °C × 10 (-40.0°C) */
inline constexpr int16_t kCltMinX10 = -400;

/** Coolant temperature maximum in °C × 10 (150.0°C) */
inline constexpr int16_t kCltMaxX10 = 1500;

/** MAP sensor minimum in bar */
inline constexpr uint16_t kMapMinBarX100 = 10u;

/** MAP sensor maximum in bar */
inline constexpr uint16_t kMapMaxBarX100 = 300u;

/** Default MAP value for limp-home mode in bar */
inline constexpr uint16_t kMapDefaultLimpHome = 50u;

/** Default IAT value for limp-home mode in °C × 10 (25.0°C) */
inline constexpr int16_t kIatDefaultLimpHomeX10 = 250;

/** Default CLT value for limp-home mode in °C × 10 (90.0°C) */
inline constexpr int16_t kCltDefaultLimpHomeX10 = 900;

// ============================================================================
// Flash Memory Timing (from STM32H562 errata ES0565)
// ============================================================================

/** Maximum flash sector erase time in microseconds */
inline constexpr uint32_t kFlashEraseMaxTimeUs = 5000u;

/** Maximum flash word program time in microseconds */
inline constexpr uint32_t kFlashProgramMaxTimeUs = 200u;

/** First flash operation after power-on CPU freeze time in microseconds */
inline constexpr uint32_t kFlashFirstOpFreezeTimeUs = 120u;

/** Safe RPM threshold below which flash writes are allowed (300 RPM × 10) */
inline constexpr uint32_t kFlashWriteSafeRpmX10 = 3000u;

/** Minimum interval between calibration saves in milliseconds */
inline constexpr uint32_t kCalibSaveMinIntervalMs = 10000u;

/** Delay before saving runtime seed after engine stop in milliseconds */
inline constexpr uint32_t kRuntimeSeedSaveDelayMs = 2000u;

// ============================================================================
// ADC Recovery Constants
// ============================================================================

/** ADC recovery timeout in loop iterations */
inline constexpr uint32_t kAdcRecoveryTimeout = 1000u;

// ============================================================================
// Ignition System Constants
// ============================================================================

/** Typical ignition coil dwell time in milliseconds */
inline constexpr uint32_t kTypicalDwellTimeMs = 3u;

/** Minimum dwell time in milliseconds */
inline constexpr uint32_t kMinDwellTimeMs = 1u;

/** Maximum dwell time in milliseconds */
inline constexpr uint32_t kMaxDwellTimeMs = 8u;

// ============================================================================
// Fault gating (limp_gating) — 0-threshold knobs stay off for benches
// ============================================================================

/** Boost cut MAP threshold, bar × 100 (≡ kPa). 0 = disabled. */
inline constexpr uint16_t kBoostCutMapBarX100 = 0u;
/** Resume hysteresis after boost cut (20 kPa). */
inline constexpr uint16_t kBoostCutHystBarX100 = 20u;
/** Min oil pressure after start, bar × 1000. Sensor range-fault skips this. */
inline constexpr uint16_t kOilMinAfterStartBarX1000 = 1500u;
/** After-start oil-pressure timeout. */
inline constexpr uint32_t kOilAfterStartTimeoutMs = 5000u;
/** Running oil-pressure timeout (sensor live, pressure below min). */
inline constexpr uint32_t kOilRunningTimeoutMs = 500u;
/** Overtemp protect RPM floor (×10). Oil *range-fault* cuts at any RPM>0
 *  once cranking ends — a dead sensor must not idle. */
inline constexpr uint32_t kOilProtectRpmX10 = 15000u;
/** Fuel-rail range-fault RPM floor (×10). */
inline constexpr uint32_t kFuelRailProtectRpmX10 = 5000u;
/** ETB-problem fault rev limit (RPM × 10). Fuel cut above this. */
inline constexpr uint32_t kEtbFaultRevLimitRpmX10 = 15000u;
/** Injector-duty resume threshold (percent). FOME: lift until < 20%. */
inline constexpr uint8_t kInjDutyResumePct = 20u;
/** Lambda protection timeout (lean at load). */
inline constexpr uint16_t kLambdaProtectTimeoutMs = 2000u;
/** Allowed lambda above target before the timeout starts (λ × 1000). */
inline constexpr uint16_t kLambdaProtectDevX1000 = 200u;
inline constexpr uint32_t kLambdaProtectMinRpmX10 = 20000u;
inline constexpr uint16_t kLambdaProtectMinLoadBarX100 = 50u;

} // namespace ems::engine
