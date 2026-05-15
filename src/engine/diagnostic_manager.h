#pragma once
/**
 * @file engine/diagnostic_manager.h
 * @brief Unified diagnostic and error reporting system
 * 
 * Provides standardized fault codes, recovery state machines, and
 * sensor plausibility checking across all subsystems.
 */

#include <cstdint>

namespace ems::engine {

/**
 * @brief Standardized diagnostic trouble codes (DTC)
 * 
 * Follows ISO 14229-1 UDS format with subsystem prefix:
 * - P0xxx: Powertrain (engine management)
 * - P1xxx: Reserved for manufacturer
 * - C0xxx: Chassis
 * - B0xxx: Body
 * - U0xxx: Network
 * 
 * Our codes use P0xxx for engine-specific faults.
 */
enum class DiagnosticCode : uint16_t {
    // No fault
    NONE = 0x0000,
    
    // Sensor faults (P01xx)
    MAP_SENSOR_RANGE = 0x0100,
    MAP_SENSOR_PLAUSIBILITY = 0x0101,
    MAF_SENSOR_RANGE = 0x0102,
    MAF_SENSOR_PLAUSIBILITY = 0x0103,
    TPS_SENSOR_RANGE = 0x0104,
    TPS_SENSOR_PLAUSIBILITY = 0x0105,
    CLT_SENSOR_RANGE = 0x0106,
    CLT_SENSOR_PLAUSIBILITY = 0x0107,
    IAT_SENSOR_RANGE = 0x0108,
    IAT_SENSOR_PLAUSIBILITY = 0x0109,
    O2_SENSOR_RANGE = 0x010A,
    O2_SENSOR_HEATER = 0x010B,
    
    // Pressure sensor correlation (P012x)
    MAP_TPS_CORRELATION = 0x0120,
    MAP_BARO_CORRELATION = 0x0121,
    FUEL_PRESS_LOW = 0x0122,
    FUEL_PRESS_HIGH = 0x0123,
    OIL_PRESS_LOW = 0x0124,
    OIL_PRESS_HIGH = 0x0125,
    
    // Ignition system (P03xx)
    MISFIRE_CYLINDER_1 = 0x0300,
    MISFIRE_CYLINDER_2 = 0x0301,
    MISFIRE_CYLINDER_3 = 0x0302,
    MISFIRE_CYLINDER_4 = 0x0303,
    KNOCK_DETECTED = 0x0310,
    KNOCK_SENSOR_FAULT = 0x0311,
    
    // Fuel system (P017x)
    FUEL_TRIM_LEAN = 0x0170,
    FUEL_TRIM_RICH = 0x0171,
    LTFT_LIMIT_REACHED = 0x0172,
    STFT_LIMIT_REACHED = 0x0173,
    
    // Timing system (P00xx)
    CKP_SIGNAL_FAULT = 0x0001,
    CMP_SIGNAL_FAULT = 0x0002,
    TIMING_OVER_ADVANCED = 0x0010,
    TIMING_OVER_RETARDED = 0x0011,
    
    // Electrical system (P05xx)
    VBATT_LOW = 0x0500,
    VBATT_HIGH = 0x0501,
    ADC_TIMEOUT = 0x0510,
    ADC_RECOVERY_FAILED = 0x0511,
    FLASH_WRITE_FAULT = 0x0520,
    
    // Engine protection (P02xx)
    OVERTEMP_CRITICAL = 0x0200,
    OVERTEMP_WARNING = 0x0201,
    OVERSPEED = 0x0210,
    LOW_OIL_PRESSURE = 0x0220,
    
    // Recovery states (internal use)
    RECOVERY_ADC_INITIATED = 0xF000,
    RECOVERY_ADC_SUCCESS = 0xF001,
    RECOVERY_ADC_FAILED = 0xF002,
};

/**
 * @brief Fault severity levels
 */
enum class FaultSeverity : uint8_t {
    INFO = 0,       // Logged but no action required
    WARNING = 1,    // Driver notification, monitor closely
    ERROR = 2,      // Limp mode, reduced performance
    CRITICAL = 3,   // Engine shutdown, immediate action
};

/**
 * @brief Recovery state machine states
 */
enum class RecoveryState : uint8_t {
    IDLE = 0,           // Normal operation
    DETECTED = 1,       // Fault detected, evaluating
    RECOVERING = 2,     // Recovery in progress
    RECOVERED = 3,      // Successfully recovered
    FAILED = 4,         // Recovery failed, fallback active
    PERMANENT = 5,      // Permanent fault, service required
};

/**
 * @brief Diagnostic event structure
 */
struct DiagnosticEvent {
    DiagnosticCode code;
    FaultSeverity severity;
    uint32_t timestamp_ms;      // Time of occurrence
    uint16_t occurrence_count;  // Number of times seen
    uint16_t freeze_frame[4];   // Snapshot of key parameters
};

/**
 * @brief Unified Diagnostic Manager
 * 
 * Centralizes fault detection, logging, and recovery management.
 * Provides consistent interface for all subsystems to report
 * and query diagnostic information.
 */
class DiagnosticManager {
public:
    /**
     * @brief Initialize diagnostic system
     */
    static void init() noexcept;
    
    /**
     * @brief Report a fault
     * @param code Diagnostic trouble code
     * @param severity Fault severity level
     * @param param1 Optional parameter 1 (e.g., sensor value)
     * @param param2 Optional parameter 2 (e.g., expected value)
     * @return true if fault is new, false if already active
     */
    static bool report_fault(DiagnosticCode code, 
                            FaultSeverity severity,
                            uint16_t param1 = 0,
                            uint16_t param2 = 0) noexcept;
    
    /**
     * @brief Clear a fault
     * @param code Diagnostic trouble code to clear
     * @return true if fault was cleared
     */
    static bool clear_fault(DiagnosticCode code) noexcept;
    
    /**
     * @brief Check if a fault is currently active
     * @param code Diagnostic trouble code
     * @return true if fault is active
     */
    static bool is_fault_active(DiagnosticCode code) noexcept;
    
    /**
     * @brief Get fault count
     * @return Total number of active faults
     */
    static uint8_t get_active_fault_count() noexcept;
    
    /**
     * @brief Get highest severity among active faults
     * @return Highest severity level
     */
    static FaultSeverity get_highest_severity() noexcept;
    
    /**
     * @brief Update recovery state machine
     * @param code Associated fault code
     * @param success Whether recovery attempt succeeded
     * @return Current recovery state
     */
    static RecoveryState update_recovery(DiagnosticCode code, 
                                        bool success) noexcept;
    
    /**
     * @brief Get current recovery state for a fault
     * @param code Diagnostic trouble code
     * @return Current recovery state
     */
    static RecoveryState get_recovery_state(DiagnosticCode code) noexcept;
    
    /**
     * @brief Record freeze frame data for a fault
     * @param code Diagnostic trouble code
     * @param frame Array of 4 parameter values
     */
    static void record_freeze_frame(DiagnosticCode code,
                                   const uint16_t frame[4]) noexcept;
    
    /**
     * @brief Get diagnostic event details
     * @param code Diagnostic trouble code
     * @return Pointer to event structure, nullptr if not found
     */
    static const DiagnosticEvent* get_event(DiagnosticCode code) noexcept;
    
    /**
     * @brief Perform sensor plausibility check
     * @param map_kpa_x10 MAP sensor value
     * @param tps_pct_x10 TPS sensor value
     * @param rpm_x10 Engine RPM
     * @return true if values are plausible together
     */
    static bool check_sensor_plausibility(uint16_t map_kpa_x10,
                                         uint16_t tps_pct_x10,
                                         uint32_t rpm_x10) noexcept;
    
    /**
     * @brief Clear all non-permanent faults (for diagnostic tool)
     */
    static void clear_all_faults() noexcept;
    
    /**
     * @brief Get system ready status
     * @return true if no critical faults prevent operation
     */
    static bool is_system_ready() noexcept;
    
    // Constants must be public for static array sizing
    static constexpr uint8_t kMaxEvents = 16;

private:
    static constexpr uint16_t kFaultDebounceCount = 3;
    static constexpr uint32_t kRecoveryTimeoutMs = 5000;
};

}  // namespace ems::engine
