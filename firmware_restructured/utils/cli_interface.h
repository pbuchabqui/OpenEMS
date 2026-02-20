/**
 * @file cli_interface.h
 * @brief Command Line Interface Module for ESP32-S3 EFI
 * 
 * This module provides a text-based interface for debugging, monitoring,
 * and tuning the ECU via USB CDC serial interface.
 * 
 * Features:
 * - Real-time sensor monitoring
 * - Fuel/ignition table viewing and editing
 * - Configuration save/load operations
 * - Diagnostic information display
 * - Safety limit configuration
 * - Streaming data mode for tuning
 */

#ifndef CLI_INTERFACE_H
#define CLI_INTERFACE_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants and Configuration
 *============================================================================*/

/** @brief Maximum input line length */
#define CLI_MAX_INPUT_LEN           256

/** @brief Maximum output buffer length */
#define CLI_MAX_OUTPUT_LEN          1024

/** @brief Command history size */
#define CLI_HISTORY_SIZE            16

/** @brief Maximum number of arguments */
#define CLI_MAX_ARGS                16

/** @brief Maximum registered commands */
#define CLI_MAX_COMMANDS            32

/** @brief CLI task stack size */
#define CLI_TASK_STACK_SIZE         4096

/** @brief CLI task priority */
#define CLI_TASK_PRIORITY           3

/** @brief Default stream interval (ms) */
#define CLI_DEFAULT_STREAM_INTERVAL 100

/*============================================================================
 * Types and Structures
 *============================================================================*/

/**
 * @brief Stream output format
 */
typedef enum {
    CLI_STREAM_CSV,     /**< CSV format */
    CLI_STREAM_JSON,    /**< JSON format */
    CLI_STREAM_TABLE,   /**< Table format */
} cli_stream_format_t;

/**
 * @brief Command flags
 */
typedef enum {
    CLI_FLAG_NONE       = 0x00,  /**< No flags */
    CLI_FLAG_STREAMING  = 0x01,  /**< Command produces streaming output */
    CLI_FLAG_CONFIRM    = 0x02,  /**< Requires confirmation */
    CLI_FLAG_ADMIN      = 0x04,  /**< Requires admin mode */
} cli_command_flags_t;

/**
 * @brief Forward declaration of command structure
 */
typedef struct cli_command cli_command_t;

/**
 * @brief Subcommand definition
 */
typedef struct {
    const char *name;                   /**< Subcommand name */
    int (*handler)(int argc, char **argv); /**< Handler function */
    const char *description;            /**< Short description */
} cli_subcommand_t;

/**
 * @brief Command definition
 */
struct cli_command {
    const char *name;                   /**< Command name */
    const char *description;            /**< Short description */
    const char *usage;                  /**< Usage string */
    int (*handler)(int argc, char **argv); /**< Handler function */
    const cli_subcommand_t *subcommands; /**< Subcommands (NULL if none) */
    uint8_t flags;                      /**< Command flags */
};

/**
 * @brief CLI context structure
 */
typedef struct {
    // State
    bool                initialized;
    bool                streaming;
    bool                admin_mode;
    
    // Input buffer
    char                input_buffer[CLI_MAX_INPUT_LEN];
    uint16_t            input_pos;
    
    // Command history
    char                history[CLI_HISTORY_SIZE][CLI_MAX_INPUT_LEN];
    uint8_t             history_count;
    uint8_t             history_pos;
    
    // Output buffer
    char                output_buffer[CLI_MAX_OUTPUT_LEN];
    uint16_t            output_pos;
    
    // Stream mode
    uint32_t            stream_interval_ms;
    uint32_t            last_stream_ms;
    cli_stream_format_t stream_format;
    
    // Task handles
    TaskHandle_t        cli_task;
    QueueHandle_t       output_queue;
    
    // Mutex
    SemaphoreHandle_t   mutex;
    
    // Registered commands
    const cli_command_t *commands[CLI_MAX_COMMANDS];
    uint8_t             command_count;
} cli_context_t;

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize CLI interface
 * 
 * Initializes USB CDC, creates CLI task, and registers default commands.
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t cli_init(void);

/**
 * @brief Deinitialize CLI interface
 * 
 * Stops CLI task and frees resources.
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t cli_deinit(void);

/**
 * @brief Start CLI task
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized or already started
 */
esp_err_t cli_start(void);

/**
 * @brief Stop CLI task
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not started
 */
esp_err_t cli_stop(void);

/**
 * @brief Register custom command
 * 
 * @param command Command definition
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if command is NULL or name is empty
 * @return ESP_ERR_NO_MEM if command table is full
 */
esp_err_t cli_register_command(const cli_command_t *command);

/**
 * @brief Print to CLI output
 * 
 * @param fmt Format string
 * @param ... Arguments
 */
void cli_print(const char *fmt, ...);

/**
 * @brief Print to CLI output with va_list
 * 
 * @param fmt Format string
 * @param args Argument list
 */
void cli_vprint(const char *fmt, va_list args);

/**
 * @brief Print line to CLI output
 * 
 * @param fmt Format string
 * @param ... Arguments
 */
void cli_println(const char *fmt, ...);

/**
 * @brief Check if streaming is active
 * 
 * @return true if streaming
 */
bool cli_is_streaming(void);

/**
 * @brief Stop streaming mode
 */
void cli_stop_streaming(void);

/**
 * @brief Enter admin mode
 * 
 * @param password Password (can be NULL if no password required)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if password is incorrect
 */
esp_err_t cli_enter_admin(const char *password);

/**
 * @brief Exit admin mode
 */
void cli_exit_admin(void);

/**
 * @brief Check if in admin mode
 * 
 * @return true if in admin mode
 */
bool cli_is_admin(void);

/**
 * @brief Process a single input character
 * 
 * Called by the CLI task when data is received from USB CDC.
 * 
 * @param c Input character
 */
void cli_process_char(char c);

/**
 * @brief Process a complete input line
 * 
 * Parses and executes a command line.
 * 
 * @param line Input line (null-terminated)
 * @return 0 on success, negative on error
 */
int cli_process_line(const char *line);

/*============================================================================
 * Output Formatting Helpers
 *============================================================================*/

/**
 * @brief Print a table header
 * 
 * @param title Table title
 * @param width Table width in characters
 */
void cli_print_table_header(const char *title, uint8_t width);

/**
 * @brief Print a table row
 * 
 * @param label Row label
 * @param value Row value
 */
void cli_print_table_row(const char *label, const char *value);

/**
 * @brief Print a table separator line
 */
void cli_print_table_separator(void);

/**
 * @brief Print a table footer
 */
void cli_print_table_footer(void);

/**
 * @brief Print a value with unit
 * 
 * @param label Label string
 * @param value Numeric value
 * @param unit Unit string
 * @param width Total width for alignment
 */
void cli_print_value(const char *label, float value, const char *unit, uint8_t width);

/*============================================================================
 * ANSI Color Codes
 *============================================================================*/

#define CLI_COLOR_RESET   "\033[0m"
#define CLI_COLOR_RED     "\033[31m"
#define CLI_COLOR_GREEN   "\033[32m"
#define CLI_COLOR_YELLOW  "\033[33m"
#define CLI_COLOR_BLUE    "\033[34m"
#define CLI_COLOR_MAGENTA "\033[35m"
#define CLI_COLOR_CYAN    "\033[36m"
#define CLI_COLOR_WHITE   "\033[37m"
#define CLI_COLOR_BOLD    "\033[1m"

/*============================================================================
 * Box Drawing Characters (UTF-8)
 *============================================================================*/

#define BOX_TL      "╔"
#define BOX_TR      "╗"
#define BOX_BL      "╚"
#define BOX_BR      "╝"
#define BOX_H       "═"
#define BOX_V       "║"
#define BOX_LT      "╠"
#define BOX_RT      "╣"
#define BOX_TT      "╦"
#define BOX_BT      "╩"
#define BOX_CROSS   "╬"

#ifdef __cplusplus
}
#endif

#endif /* CLI_INTERFACE_H */
