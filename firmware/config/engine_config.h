/**
 * @file engine_config.h
 * @brief Engine parameters — motor-specific, hardware-independent
 *
 * Hardware pin assignments live in hal/hal_pins.h.
 * This file contains only engine characteristics and tuning parameters.
 *
 * All #define values here are compile-time defaults.
 * Runtime-adjustable parameters are loaded from NVS by config_manager.
 */

#ifndef ENGINE_CONFIG_H
#define ENGINE_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Engine geometry ───────────────────────────────────────────────────────────
#define ENGINE_CYLINDERS        4
#define ENGINE_STROKE           4
#define ENGINE_FIRING_ORDER     {1, 3, 4, 2}

// ── Trigger wheel ─────────────────────────────────────────────────────────────
#define TRIGGER_WHEEL_TEETH     60
#define TRIGGER_WHEEL_MISSING   2
#define TRIGGER_TDC_OFFSET_DEG  114.0f

// ── RPM limits ────────────────────────────────────────────────────────────────
#define RPM_MIN                 300
#define RPM_MAX                 8000
#define RPM_IDLE_TARGET         800
#define RPM_FUEL_CUT            7500
#define RPM_FUEL_CUT_RESTORE    7000

// ── Fuel system ───────────────────────────────────────────────────────────────
#define INJECTOR_FLOW_CC_MIN    420.0f
#define INJECTOR_RATED_PRESS_KPA 300.0f
#define INJECTOR_DEADTIME_US    500
#define FUEL_PRESS_TARGET_KPA   300.0f
#define REQ_FUEL_US             7730
#define PW_MIN_US               500
#define PW_MAX_US               18000

// ── Ignition ──────────────────────────────────────────────────────────────────
#define IGN_ADVANCE_MIN_DEG     (-5.0f)
#define IGN_ADVANCE_MAX_DEG     45.0f
#define IGN_DWELL_MS_DEFAULT    3.0f
#define IGN_DWELL_MS_MIN        1.5f
#define IGN_DWELL_MS_MAX        5.0f

// ── Knock ─────────────────────────────────────────────────────────────────────
#define KNOCK_RETARD_STEP_DEG   1.0f
#define KNOCK_RETARD_MAX_DEG    10.0f
#define KNOCK_RECOVER_STEP_DEG  0.2f
#define KNOCK_FILTER_FREQ_HZ    6000

// ── Flex fuel ─────────────────────────────────────────────────────────────────
#define FLEX_FUEL_ENABLED       true
#define FLEX_SENSOR_MIN_HZ      50.0f
#define FLEX_SENSOR_MAX_HZ      150.0f
#define FLEX_VE_CORRECTION_MAX  1.40f

// ── Sensor ranges ─────────────────────────────────────────────────────────────
#define MAP_MIN_KPA             20.0f
#define MAP_MAX_KPA             250.0f
#define TPS_MIN_PCT             0.0f
#define TPS_MAX_PCT             100.0f
#define CLT_MIN_C               (-40.0f)
#define CLT_MAX_C               120.0f
#define IAT_MIN_C               (-40.0f)
#define IAT_MAX_C               120.0f
#define VBAT_MIN_V              7.0f
#define VBAT_MAX_V              17.0f

// ── Warmup enrichment ─────────────────────────────────────────────────────────
#define WARMUP_TEMP_MIN_C       0
#define WARMUP_TEMP_MAX_C       70
#define WARMUP_ENRICH_MAX_PCT   140

// ── Acceleration enrichment ───────────────────────────────────────────────────
#define TPS_DOT_THRESHOLD       5
#define TPS_DOT_ENRICH_MAX_PCT  150

// ── Lambda / AFR ──────────────────────────────────────────────────────────────
#define LAMBDA_SCALE            1000
#define IAT_REF_K10             2931

// ── Closed loop ───────────────────────────────────────────────────────────────
#define STFT_LIMIT              0.25f
#define LTFT_LIMIT              0.20f
#define LTFT_ALPHA              0.01f

// ── VVT ───────────────────────────────────────────────────────────────────────
#define VVT_DUAL_ENABLED        true
#define VVT_INTAKE_MAX_DEG      40.0f
#define VVT_EXHAUST_MAX_DEG     30.0f
#define VVT_PID_KP              2.0f
#define VVT_PID_KI              0.5f
#define VVT_PID_KD              0.1f

// ── Boost ─────────────────────────────────────────────────────────────────────
#define BOOST_ENABLED           true
#define BOOST_MAX_KPA           200.0f
#define BOOST_OVERBOOST_KPA     220.0f
#define BOOST_PID_KP            3.0f
#define BOOST_PID_KI            0.8f
#define BOOST_PID_KD            0.2f

// ── Idle ──────────────────────────────────────────────────────────────────────
#define IDLE_IAC_ENABLED        true
#define IDLE_PID_KP             5.0f
#define IDLE_PID_KI             1.0f
#define IDLE_PID_KD             0.5f

// ── Safety ────────────────────────────────────────────────────────────────────
#define CLT_OVERHEAT_C          105
#define OIL_PRESS_MIN_KPA       150.0f
#define LIMP_RPM_LIMIT          3000
#define LIMP_VE_VALUE           80
#define LIMP_TIMING_DEG         10

// ── CAN ──────────────────────────────────────────────────────────────────────
#define CAN_SPEED_BPS           500000

// ── FreeRTOS tasks (all Core 1) ───────────────────────────────────────────────
#define TASK_PRIO_CONTROL       10
#define TASK_PRIO_SENSOR        9
#define TASK_PRIO_COMMS         8
#define TASK_PRIO_MONITOR       7
#define TASK_PRIO_LOGGER        5
#define TASK_STACK_CONTROL      4096
#define TASK_STACK_SENSOR       4096
#define TASK_STACK_COMMS        4096
#define TASK_STACK_MONITOR      3072
#define TASK_STACK_LOGGER       4096
#define TASK_CORE_CONTROL       1
#define TASK_CORE_SENSOR        1
#define TASK_CORE_COMMS         1
#define TASK_CORE_MONITOR       1
#define TASK_CORE_LOGGER        1

// ── Interpolation ────────────────────────────────────────────────────────────
#define INTERP_CACHE_RPM_DEADBAND   50
#define INTERP_CACHE_LOAD_DEADBAND  20
#define FIXED_POINT_SCALE           10
#define MAP_FILTER_ALPHA            3

// ── 16x16 table type (used across the firmware) ──────────────────────────────
#pragma pack(push, 1)
typedef struct {
    uint16_t rpm_bins[16];
    uint16_t load_bins[16];
    uint16_t values[16][16];
    uint16_t checksum;
} table_16x16_t;
#pragma pack(pop)

static const uint16_t DEFAULT_RPM_BINS[16] = {
    500, 800, 1200, 1600, 2000, 2500, 3000, 3500,
    4000, 4500, 5000, 5500, 6000, 6500, 7000, 8000
};
static const uint16_t DEFAULT_LOAD_BINS[16] = {
    200, 300, 400, 500, 600, 650, 700, 750,
    800, 850, 900, 950, 1000, 1020, 1050, 1100
};

// ── Utility macros ────────────────────────────────────────────────────────────
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define DEBUG_MODE              true
#define SERIAL_BAUD_RATE        115200

// Legacy aliases so existing source files still compile without changes
#define CKP_GPIO                HAL_PIN_CKP
#define CMP_GPIO                HAL_PIN_CMP
#define CAN_SPEED               CAN_SPEED_BPS
#define CAN_TX_GPIO             HAL_PIN_CAN_TX
#define CAN_RX_GPIO             HAL_PIN_CAN_RX
#define INJECTOR_GPIO_1         HAL_PIN_INJ_1
#define INJECTOR_GPIO_2         HAL_PIN_INJ_2
#define INJECTOR_GPIO_3         HAL_PIN_INJ_3
#define INJECTOR_GPIO_4         HAL_PIN_INJ_4
#define IGNITION_GPIO_1         HAL_PIN_IGN_1
#define IGNITION_GPIO_2         HAL_PIN_IGN_2
#define IGNITION_GPIO_3         HAL_PIN_IGN_3
#define IGNITION_GPIO_4         HAL_PIN_IGN_4
#define INJECTOR_FLOW_RATE      INJECTOR_FLOW_CC_MIN
#define INJECTOR_PULSE_WIDTH_MIN PW_MIN_US
#define INJECTOR_PULSE_WIDTH_MAX PW_MAX_US
#define MAX_RPM                 RPM_MAX
#define IDLE_RPM                RPM_IDLE_TARGET
#define FUEL_CUTOFF_RPM         RPM_FUEL_CUT
#define MAP_SENSOR_MIN          MAP_MIN_KPA
#define MAP_SENSOR_MAX          MAP_MAX_KPA
#define CLT_SENSOR_MIN          CLT_MIN_C
#define CLT_SENSOR_MAX          CLT_MAX_C
#define TPS_SENSOR_MIN          TPS_MIN_PCT
#define TPS_SENSOR_MAX          TPS_MAX_PCT
#define IAT_SENSOR_MIN          IAT_MIN_C
#define IAT_SENSOR_MAX          IAT_MAX_C
#define VBAT_SENSOR_MIN         VBAT_MIN_V
#define VBAT_SENSOR_MAX         VBAT_MAX_V
#define WARMUP_TEMP_MIN         WARMUP_TEMP_MIN_C
#define WARMUP_TEMP_MAX         WARMUP_TEMP_MAX_C
#define WARMUP_ENRICH_MAX       WARMUP_ENRICH_MAX_PCT
// C12 fix: TPS_DOT_ENRICH_MAX was referenced in fuel_calc.c and fault_manager.c
// but never defined — only TPS_DOT_ENRICH_MAX_PCT existed.
#define TPS_DOT_ENRICH_MAX      TPS_DOT_ENRICH_MAX_PCT
#define PW_MAX_US               18000
#define PW_MIN_US               500
#define RPM_MAX_SAFE            12000
#define IGNITION_ADVANCE_BASE   10
#define MAX_IGNITION_ADVANCE    35
#define MIN_IGNITION_ADVANCE    (-5)
#define CONTROL_TASK_PRIORITY   TASK_PRIO_CONTROL
#define SENSOR_TASK_PRIORITY    TASK_PRIO_SENSOR
#define COMM_TASK_PRIORITY      TASK_PRIO_COMMS
#define MONITOR_TASK_PRIORITY   TASK_PRIO_MONITOR
#define CONTROL_TASK_STACK      TASK_STACK_CONTROL
#define SENSOR_TASK_STACK       TASK_STACK_SENSOR
#define COMM_TASK_STACK         TASK_STACK_COMMS
#define MONITOR_TASK_STACK      TASK_STACK_MONITOR
#define CONTROL_TASK_CORE       TASK_CORE_CONTROL
#define SENSOR_TASK_CORE        TASK_CORE_SENSOR
#define COMM_TASK_CORE          TASK_CORE_COMMS
#define MONITOR_TASK_CORE       TASK_CORE_MONITOR

#ifdef __cplusplus
}
#endif

#endif // ENGINE_CONFIG_H
