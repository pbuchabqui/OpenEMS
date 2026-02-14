#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sensor status enumeration
 * Defines possible states for sensor validation
 */
typedef enum {
    SENSOR_OK = 0,           /**< Sensor reading is valid */
    SENSOR_SHORT_GND,        /**< Sensor short to ground */
    SENSOR_SHORT_VCC,        /**< Sensor short to VCC */
    SENSOR_OPEN,             /**< Sensor open circuit */
    SENSOR_OUT_RANGE         /**< Sensor reading out of expected range */
} sensor_status_t;

/**
 * @brief Knock protection structure
 * Contains knock detection and timing retard information
 */
typedef struct {
    uint8_t knock_count;     /**< Number of consecutive knock detections */
    uint16_t timing_retard;  /**< Timing retard value in 0.1 degree increments */
    bool knock_detected;     /**< Flag indicating knock detection */
} knock_protection_t;

/**
 * @brief Limp mode structure
 * Contains limp mode configuration and status
 */
typedef struct {
    bool active;             /**< Limp mode activation status */
    uint16_t rpm_limit;      /**< RPM limit in limp mode */
    uint16_t ve_value;       /**< VE value in limp mode */
    uint16_t timing_value;   /**< Timing value in limp mode */
    uint16_t lambda_target;  /**< Lambda target in limp mode */
    uint32_t activation_time;/**< Limp mode activation timestamp */
} limp_mode_t;

/**
 * @brief Watchdog configuration structure
 * Contains watchdog timer settings
 */
typedef struct {
    bool enabled;            /**< Watchdog enable status */
    uint32_t timeout_ms;     /**< Watchdog timeout in milliseconds */
    uint32_t last_feed_time; /**< Last watchdog feed timestamp */
} watchdog_config_t;

/**
 * @brief Initialize safety monitoring system
 * 
 * Initializes all safety monitoring components including:
 * - Watchdog timer
 * - Limp mode configuration
 * - Mutex protection
 * - Global variables
 * 
 * @return true if initialization successful, false otherwise
 */
bool safety_monitor_init(void);

/**
 * @brief Validate sensor reading
 * 
 * Validates an ADC sensor reading against expected thresholds
 * 
 * @param adc_value Raw ADC value to validate
 * @param min_expected Minimum expected ADC value
 * @param max_expected Maximum expected ADC value
 * @return sensor_status_t Status of the sensor reading
 */
sensor_status_t safety_validate_sensor(int adc_value, int min_expected, int max_expected);

/**
 * @brief Check for engine over-rev condition
 * 
 * Monitors engine RPM and detects over-rev conditions
 * 
 * @param rpm Current engine RPM
 * @return true if over-rev detected, false otherwise
 */
bool safety_check_over_rev(uint16_t rpm);

/**
 * @brief Check for engine overheating
 * 
 * Monitors coolant temperature and detects overheating
 * 
 * @param temp Coolant temperature in degrees Celsius
 * @return true if overheating detected, false otherwise
 */
bool safety_check_overheat(int16_t temp);

/**
 * @brief Check for battery voltage issues
 * 
 * Monitors battery voltage and detects abnormal conditions
 * 
 * @param voltage Battery voltage in decivolts (dV)
 * @return true if voltage issue detected, false otherwise
 */
bool safety_check_battery_voltage(uint16_t voltage);

/**
 * @brief Activate limp mode
 * 
 * Activates limp mode with predefined safety parameters
 */
void safety_activate_limp_mode(void);

/**
 * @brief Deactivate limp mode
 * 
 * Deactivates limp mode and restores normal operation.
 * Auto-recovery requires:
 * - Minimum 5 seconds in limp mode
 * - Conditions safe for 2 seconds (hysteresis)
 */
void safety_deactivate_limp_mode(void);

/**
 * @brief Mark conditions as safe or unsafe for limp mode recovery
 * 
 * Call with safe=true when all safety conditions are met.
 * Call with safe=false when any safety condition fails.
 * 
 * @param safe true if conditions are safe for recovery, false otherwise
 */
void safety_mark_conditions_safe(bool safe);

/**
 * @brief Check if limp mode is active
 * 
 * @return true if limp mode is currently active, false otherwise
 */
bool safety_is_limp_mode_active(void);

/**
 * @brief Get limp mode status
 * 
 * @return limp_mode_t Current limp mode configuration and status
 */
limp_mode_t safety_get_limp_mode_status(void);

/**
 * @brief Initialize watchdog timer
 * 
 * Configures and starts the FreeRTOS watchdog timer
 * 
 * @param timeout_ms Watchdog timeout in milliseconds
 * @return true if watchdog initialized successfully, false otherwise
 */
bool safety_watchdog_init(uint32_t timeout_ms);

/**
 * @brief Feed watchdog timer
 * 
 * Resets the watchdog timer to prevent timeout
 * 
 * @return true if watchdog fed successfully, false otherwise
 */
bool safety_watchdog_feed(void);

/**
 * @brief Check watchdog status
 * 
 * Verifies if the watchdog timer is still healthy
 * 
 * @return true if watchdog is healthy, false if timeout occurred
 */
bool safety_watchdog_check(void);

/**
 * @brief Handle knock detection
 * 
 * Processes knock detection events and applies timing retard
 * 
 * @param knock_prot Pointer to knock protection structure
 */
void safety_handle_knock(knock_protection_t *knock_prot);

/**
 * @brief Log safety event
 * 
 * Logs safety-related events with timestamp and value
 * 
 * @param event_type Type of safety event
 * @param value Event value
 */
void safety_log_event(const char* event_type, uint32_t value);

/**
 * @brief Validate MAP sensor reading
 *
 * Validates the MAP sensor reading against maximum pressure threshold
 *
 * @param map_value Raw MAP sensor ADC value
 * @return sensor_status_t Status of the MAP sensor reading
 */
sensor_status_t safety_validate_map_sensor(int map_value);

/**
 * @brief Check for acceleration enrichment condition
 *
 * Determines if acceleration enrichment should be applied based on MAP delta
 *
 * @param current_map Current MAP sensor reading
 * @param previous_map Previous MAP sensor reading
 * @return true if acceleration enrichment should be applied, false otherwise
 */
bool safety_check_acceleration_enrichment(int current_map, int previous_map);

/**
 * @brief Get acceleration enrichment parameters
 *
 * @return uint16_t Acceleration enrichment factor in %
 */
uint16_t safety_get_accel_enrichment_factor(void);

/**
 * @brief Get acceleration enrichment duration
 *
 * @return uint32_t Acceleration enrichment duration in milliseconds
 */
uint32_t safety_get_accel_enrichment_duration(void);

#ifdef __cplusplus
}
#endif

#endif // SAFETY_MONITOR_H
