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

/** Maximum safe TIM8 delta for 16-bit mode */
inline constexpr uint32_t kTim8MaxDelta16 = 0xFFFFu;

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

/** Air density at 100 kPa, 25°C in mg/cc × 1000 (1.184 mg/cc) */
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

/** MAP sensor minimum in kPa */
inline constexpr uint16_t kMapMinKpa = 10u;

/** MAP sensor maximum in kPa */
inline constexpr uint16_t kMapMaxKpa = 300u;

/** Default MAP value for limp-home mode in kPa */
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

} // namespace ems::engine
