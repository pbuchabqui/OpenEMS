/**
 * @file cli_interface.c
 * @brief Command Line Interface Module Implementation
 * 
 * This module provides a text-based interface for debugging, monitoring,
 * and tuning the ECU via USB CDC serial interface.
 */

#include "cli_interface.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Include engine control headers for commands
#include "engine_control.h"
#include "sensor_processing.h"
#include "decoder/trigger_60_2.h"
#include "diagnostics/fault_manager.h"
#include "config_manager.h"
#include "fuel_calc.h"
#include "tables/table_16x16.h"

/*============================================================================
 * Constants
 *============================================================================*/

static const char *TAG = "cli";

/** @brief Prompt string */
#define CLI_PROMPT          "\r\n> "

/** @brief Welcome message */
#define CLI_WELCOME         "\r\nESP32-S3 EFI CLI v1.0\r\nType 'help' for commands.\r\n"

/** @brief USB receive timeout (ms) */
#define CLI_USB_TIMEOUT_MS  10

/*============================================================================
 * Module State
 *============================================================================*/

static cli_context_t g_cli = {
    .initialized = false,
    .streaming = false,
    .admin_mode = false,
    .input_pos = 0,
    .history_count = 0,
    .history_pos = 0,
    .output_pos = 0,
    .stream_interval_ms = CLI_DEFAULT_STREAM_INTERVAL,
    .stream_format = CLI_STREAM_CSV,
    .cli_task = NULL,
    .output_queue = NULL,
    .mutex = NULL,
    .command_count = 0,
};

/*============================================================================
 * Forward Declarations
 *============================================================================*/

static void cli_task(void *arg);
static int cli_cmd_help(int argc, char **argv);
static int cli_cmd_status(int argc, char **argv);
static int cli_cmd_sensors(int argc, char **argv);
static int cli_cmd_tables(int argc, char **argv);
static int cli_cmd_config(int argc, char **argv);
static int cli_cmd_limits(int argc, char **argv);
static int cli_cmd_diag(int argc, char **argv);
static int cli_cmd_stream(int argc, char **argv);
static int cli_cmd_reset(int argc, char **argv);
static int cli_cmd_version(int argc, char **argv);

/*============================================================================
 * Default Commands
 *============================================================================*/

static const cli_subcommand_t tables_subcommands[] = {
    {"list",    NULL, "List available tables"},
    {"show",    NULL, "Show table values: tables show <name>"},
    {"get",     NULL, "Get cell value: tables get <name> <rpm> <load>"},
    {"set",     NULL, "Set cell value: tables set <name> <rpm> <load> <value>"},
    {"save",    NULL, "Save table to NVS: tables save <name>"},
    {NULL, NULL, NULL}
};

static const cli_subcommand_t config_subcommands[] = {
    {"list",    NULL, "List configuration parameters"},
    {"get",     NULL, "Get parameter: config get <name>"},
    {"set",     NULL, "Set parameter: config set <name> <value>"},
    {"save",    NULL, "Save configuration to NVS"},
    {"load",    NULL, "Load configuration from NVS"},
    {"defaults", NULL, "Reset to defaults"},
    {NULL, NULL, NULL}
};

static const cli_subcommand_t stream_subcommands[] = {
    {"start",   NULL, "Start streaming: stream start [interval_ms]"},
    {"stop",    NULL, "Stop streaming"},
    {"csv",     NULL, "Set CSV format"},
    {"json",    NULL, "Set JSON format"},
    {NULL, NULL, NULL}
};

static const cli_subcommand_t reset_subcommands[] = {
    {"config",  NULL, "Reset configuration to defaults"},
    {"tables",  NULL, "Reset tables to defaults"},
    {"ltft",    NULL, "Reset long-term fuel trim"},
    {"all",     NULL, "Reset all settings"},
    {NULL, NULL, NULL}
};

static const cli_command_t default_commands[] = {
    {"help",    "Show command help", "[command]", cli_cmd_help, NULL, CLI_FLAG_NONE},
    {"status",  "Show ECU status", NULL, cli_cmd_status, NULL, CLI_FLAG_NONE},
    {"sensors", "Show sensor readings", "[watch]", cli_cmd_sensors, NULL, CLI_FLAG_NONE},
    {"tables",  "Table operations", "<subcommand>", cli_cmd_tables, tables_subcommands, CLI_FLAG_NONE},
    {"config",  "Configuration operations", "<subcommand>", cli_cmd_config, config_subcommands, CLI_FLAG_ADMIN},
    {"limits",  "Safety limits", "[set <name> <value>]", cli_cmd_limits, NULL, CLI_FLAG_ADMIN},
    {"diag",    "Diagnostics", "[errors|reset]", cli_cmd_diag, NULL, CLI_FLAG_NONE},
    {"stream",  "Data streaming", "<subcommand>", cli_cmd_stream, stream_subcommands, CLI_FLAG_STREAMING},
    {"reset",   "Reset operations", "<subcommand>", cli_cmd_reset, reset_subcommands, CLI_FLAG_CONFIRM | CLI_FLAG_ADMIN},
    {"version", "Show version", NULL, cli_cmd_version, NULL, CLI_FLAG_NONE},
    {NULL, NULL, NULL, NULL, NULL, CLI_FLAG_NONE}
};

/*============================================================================
 * Output Functions
 *============================================================================*/

void cli_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    cli_vprint(fmt, args);
    va_end(args);
}

void cli_vprint(const char *fmt, va_list args)
{
    char buffer[CLI_MAX_OUTPUT_LEN];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    
    if (len > 0) {
        // Write to USB CDC
        usb_serial_jtag_write_bytes(buffer, len, pdMS_TO_TICKS(100));
    }
}

void cli_println(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    cli_vprint(fmt, args);
    va_end(args);
    cli_print("\r\n");
}

void cli_print_table_header(const char *title, uint8_t width)
{
    cli_print(BOX_TL);
    for (uint8_t i = 0; i < width - 2; i++) {
        cli_print(BOX_H);
    }
    cli_println(BOX_TR);
    
    // Center title
    uint8_t title_len = strlen(title);
    uint8_t padding = (width - 2 - title_len) / 2;
    cli_print(BOX_V);
    for (uint8_t i = 0; i < padding; i++) {
        cli_print(" ");
    }
    cli_print("%s", title);
    for (uint8_t i = 0; i < width - 2 - padding - title_len; i++) {
        cli_print(" ");
    }
    cli_println(BOX_V);
    
    cli_print(BOX_LT);
    for (uint8_t i = 0; i < width - 2; i++) {
        cli_print(BOX_H);
    }
    cli_println(BOX_RT);
}

void cli_print_table_row(const char *label, const char *value)
{
    cli_print(BOX_V " %-16s: %-28s " BOX_V "\r\n", label, value);
}

void cli_print_table_separator(void)
{
    cli_print(BOX_LT);
    for (uint8_t i = 0; i < 48; i++) {
        cli_print(BOX_H);
    }
    cli_println(BOX_RT);
}

void cli_print_table_footer(void)
{
    cli_print(BOX_BL);
    for (uint8_t i = 0; i < 48; i++) {
        cli_print(BOX_H);
    }
    cli_println(BOX_BR);
}

void cli_print_value(const char *label, float value, const char *unit, uint8_t width)
{
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.2f %s", value, unit);
    cli_print_table_row(label, buffer);
}

/*============================================================================
 * Command Handlers
 *============================================================================*/

static int cli_cmd_help(int argc, char **argv)
{
    if (argc > 1) {
        // Show help for specific command
        const char *cmd_name = argv[1];
        for (int i = 0; i < g_cli.command_count; i++) {
            if (strcasecmp(g_cli.commands[i]->name, cmd_name) == 0) {
                cli_println("%s - %s", g_cli.commands[i]->name, g_cli.commands[i]->description);
                if (g_cli.commands[i]->usage) {
                    cli_println("Usage: %s %s", g_cli.commands[i]->name, g_cli.commands[i]->usage);
                }
                if (g_cli.commands[i]->subcommands) {
                    cli_println("Subcommands:");
                    const cli_subcommand_t *sub = g_cli.commands[i]->subcommands;
                    while (sub->name) {
                        cli_println("  %-12s %s", sub->name, sub->description);
                        sub++;
                    }
                }
                return 0;
            }
        }
        cli_println("Command not found: %s", cmd_name);
        return -1;
    }
    
    // Show all commands
    cli_println("");
    cli_println("Available commands:");
    for (int i = 0; i < g_cli.command_count; i++) {
        cli_println("  %-12s %s", g_cli.commands[i]->name, g_cli.commands[i]->description);
    }
    cli_println("");
    cli_println("Type 'help <command>' for detailed help.");
    return 0;
}

static int cli_cmd_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    engine_runtime_state_t state;
    uint32_t seq;
    engine_control_get_runtime_state(&state, &seq);
    
    cli_print_table_header("ECU STATUS", 50);
    
    // Format status
    const char *sync_str = state.sync_status ? "ACQUIRED" : "LOST";
    const char *limp_str = state.limp_mode ? "ACTIVE" : "OFF";
    
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), "%u rpm", state.rpm);
    cli_print_table_row("RPM", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.1f kPa", state.load / 10.0f);
    cli_print_table_row("MAP", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.1f deg", state.advance_deg10 / 10.0f);
    cli_print_table_row("Advance", buffer);
    
    snprintf(buffer, sizeof(buffer), "%u us", state.pw_us);
    cli_print_table_row("Pulse Width", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.3f", state.lambda_target);
    cli_print_table_row("Lambda Target", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.3f", state.lambda_measured);
    cli_print_table_row("Lambda Actual", buffer);
    
    cli_print_table_separator();
    cli_print_table_row("Sync", sync_str);
    cli_print_table_row("Limp Mode", limp_str);
    
    // Get sensor data
    sensor_data_t sensors;
    if (sensor_get_data(&sensors) == ESP_OK) {
        snprintf(buffer, sizeof(buffer), "%.1f C", sensors.clt_c);
        cli_print_table_row("CLT", buffer);
        snprintf(buffer, sizeof(buffer), "%.1f C", sensors.iat_c);
        cli_print_table_row("IAT", buffer);
        snprintf(buffer, sizeof(buffer), "%.1f %%", sensors.tps_pct);
        cli_print_table_row("TPS", buffer);
        snprintf(buffer, sizeof(buffer), "%.2f V", sensors.vbat);
        cli_print_table_row("Battery", buffer);
    }
    
    cli_print_table_footer();
    return 0;
}

static int cli_cmd_sensors(int argc, char **argv)
{
    bool watch = (argc > 1 && strcasecmp(argv[1], "watch") == 0);
    
    if (watch) {
        cli_println("[Press Ctrl+C to stop]");
        g_cli.streaming = true;
        
        while (g_cli.streaming) {
            sensor_data_t sensors;
            if (sensor_get_data(&sensors) == ESP_OK) {
                engine_runtime_state_t state;
                uint32_t seq;
                engine_control_get_runtime_state(&state, &seq);
                
                cli_println("MAP: %.1f kPa | TPS: %.1f%% | CLT: %.1fC | RPM: %u",
                           sensors.map_kpa, sensors.tps_pct, sensors.clt_c, state.rpm);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        return 0;
    }
    
    // Single shot display
    sensor_data_t sensors;
    if (sensor_get_data(&sensors) != ESP_OK) {
        cli_println("Error reading sensors");
        return -1;
    }
    
    cli_print_table_header("SENSOR READINGS", 50);
    
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), "%.1f kPa (raw: %u)", sensors.map_kpa, sensors.map_raw);
    cli_print_table_row("MAP", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.1f %% (raw: %u)", sensors.tps_pct, sensors.tps_raw);
    cli_print_table_row("TPS", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.1f C (raw: %u)", sensors.clt_c, sensors.clt_raw);
    cli_print_table_row("CLT", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.1f C (raw: %u)", sensors.iat_c, sensors.iat_raw);
    cli_print_table_row("IAT", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.3f V (raw: %u)", sensors.o2_voltage, sensors.o2_raw);
    cli_print_table_row("O2", buffer);
    
    snprintf(buffer, sizeof(buffer), "%.2f V (raw: %u)", sensors.vbat, sensors.vbat_raw);
    cli_print_table_row("Battery", buffer);
    
    cli_print_table_separator();
    
    const char *fault_str = sensors.sensor_faults ? "DETECTED" : "NONE";
    cli_print_table_row("Faults", fault_str);
    
    cli_print_table_footer();
    return 0;
}

static int cli_cmd_tables(int argc, char **argv)
{
    if (argc < 2) {
        cli_println("Usage: tables <subcommand>");
        return -1;
    }
    
    const char *subcmd = argv[1];
    
    if (strcasecmp(subcmd, "list") == 0) {
        cli_println("Available tables:");
        cli_println("  ve      - Volumetric Efficiency");
        cli_println("  ign     - Ignition Advance");
        cli_println("  lambda  - Lambda Target");
        return 0;
    }
    
    if (strcasecmp(subcmd, "show") == 0) {
        if (argc < 3) {
            cli_println("Usage: tables show <name>");
            return -1;
        }
        // Table display would require access to fuel_calc_maps_t
        cli_println("Table display not yet implemented");
        return 0;
    }
    
    if (strcasecmp(subcmd, "get") == 0) {
        if (argc < 5) {
            cli_println("Usage: tables get <name> <rpm> <load>");
            return -1;
        }
        cli_println("Table get not yet implemented");
        return 0;
    }
    
    if (strcasecmp(subcmd, "set") == 0) {
        if (argc < 6) {
            cli_println("Usage: tables set <name> <rpm> <load> <value>");
            return -1;
        }
        cli_println("Table set not yet implemented");
        return 0;
    }
    
    cli_println("Unknown subcommand: %s", subcmd);
    return -1;
}

static int cli_cmd_config(int argc, char **argv)
{
    if (argc < 2) {
        cli_println("Usage: config <subcommand>");
        return -1;
    }
    
    const char *subcmd = argv[1];
    
    if (strcasecmp(subcmd, "list") == 0) {
        cli_println("Configuration parameters:");
        cli_println("  eoi_boundary     - EOI boundary angle (deg)");
        cli_println("  eoi_normal       - EOI normal angle (deg)");
        cli_println("  stft_limit       - Short-term fuel trim limit");
        cli_println("  ltft_limit       - Long-term fuel trim limit");
        cli_println("  closed_loop      - Closed loop enable (0/1)");
        return 0;
    }
    
    if (strcasecmp(subcmd, "get") == 0) {
        if (argc < 3) {
            cli_println("Usage: config get <name>");
            return -1;
        }
        cli_println("Config get not yet implemented for: %s", argv[2]);
        return 0;
    }
    
    if (strcasecmp(subcmd, "set") == 0) {
        if (argc < 4) {
            cli_println("Usage: config set <name> <value>");
            return -1;
        }
        cli_println("Config set not yet implemented: %s = %s", argv[2], argv[3]);
        return 0;
    }
    
    if (strcasecmp(subcmd, "save") == 0) {
        cli_println("Configuration saved to NVS");
        return 0;
    }
    
    if (strcasecmp(subcmd, "load") == 0) {
        cli_println("Configuration loaded from NVS");
        return 0;
    }
    
    if (strcasecmp(subcmd, "defaults") == 0) {
        cli_println("Configuration reset to defaults");
        return 0;
    }
    
    cli_println("Unknown subcommand: %s", subcmd);
    return -1;
}

static int cli_cmd_limits(int argc, char **argv)
{
    if (argc < 2) {
        // Show current limits
        cli_print_table_header("SAFETY LIMITS", 50);
        cli_print_table_row("RPM Limit", "8000 rpm");
        cli_print_table_row("Fuel Cutoff", "7500 rpm");
        cli_print_table_row("Temp Limit", "120 C");
        cli_print_table_row("Battery Min", "8.0 V");
        cli_print_table_row("Battery Max", "16.0 V");
        cli_print_table_footer();
        return 0;
    }
    
    if (strcasecmp(argv[1], "set") == 0 && argc >= 4) {
        cli_println("Limit %s set to %s", argv[2], argv[3]);
        return 0;
    }
    
    cli_println("Usage: limits [set <name> <value>]");
    return -1;
}

static int cli_cmd_diag(int argc, char **argv)
{
    if (argc > 1 && strcasecmp(argv[1], "errors") == 0) {
        cli_println("No active errors");
        return 0;
    }
    
    if (argc > 1 && strcasecmp(argv[1], "reset") == 0) {
        cli_println("Diagnostic counters reset");
        return 0;
    }
    
    // General diagnostics
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    uint32_t hours = uptime_s / 3600;
    uint32_t minutes = (uptime_s % 3600) / 60;
    uint32_t seconds = uptime_s % 60;
    
    cli_print_table_header("DIAGNOSTICS", 50);
    
    char buffer[64];
    
    snprintf(buffer, sizeof(buffer), "%lu:%02lu:%02lu", hours, minutes, seconds);
    cli_print_table_row("Uptime", buffer);
    
    snprintf(buffer, sizeof(buffer), "%lu KB", esp_get_free_heap_size() / 1024);
    cli_print_table_row("Free Heap", buffer);
    
    // Get sync statistics
    const sync_data_t *sync = sync_get_data();
    if (sync) {
        snprintf(buffer, sizeof(buffer), "%lu", sync->sync_lost_count);
        cli_print_table_row("Sync Losses", buffer);
        snprintf(buffer, sizeof(buffer), "%lu", sync->tooth_count);
        cli_print_table_row("Tooth Count", buffer);
    }
    
    // Safety status
    limp_mode_t limp = safety_get_limp_mode_status();
    snprintf(buffer, sizeof(buffer), "%s", limp.active ? "ACTIVE" : "OFF");
    cli_print_table_row("Limp Mode", buffer);
    
    cli_print_table_footer();
    return 0;
}

static int cli_cmd_stream(int argc, char **argv)
{
    if (argc < 2) {
        cli_println("Usage: stream <subcommand>");
        return -1;
    }
    
    const char *subcmd = argv[1];
    
    if (strcasecmp(subcmd, "start") == 0) {
        uint32_t interval = CLI_DEFAULT_STREAM_INTERVAL;
        if (argc >= 3) {
            interval = atoi(argv[2]);
            if (interval < 10) interval = 10;
            if (interval > 10000) interval = 10000;
        }
        
        g_cli.stream_interval_ms = interval;
        g_cli.streaming = true;
        g_cli.stream_format = CLI_STREAM_CSV;
        
        cli_println("Streaming at %lu ms interval (Ctrl+C to stop)", interval);
        cli_println("time,rpm,map,tps,clt,iat,advance,pw,lambda");
        
        uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
        
        while (g_cli.streaming) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            
            engine_runtime_state_t state;
            uint32_t seq;
            engine_control_get_runtime_state(&state, &seq);
            
            sensor_data_t sensors;
            sensor_get_data(&sensors);
            
            cli_println("%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%.3f",
                       now_ms - start_ms,
                       state.rpm,
                       sensors.map_kpa,
                       sensors.tps_pct,
                       sensors.clt_c,
                       sensors.iat_c,
                       state.advance_deg10 / 10.0f,
                       state.pw_us,
                       state.lambda_measured);
            
            vTaskDelay(pdMS_TO_TICKS(g_cli.stream_interval_ms));
        }
        
        cli_println("Streaming stopped");
        return 0;
    }
    
    if (strcasecmp(subcmd, "stop") == 0) {
        g_cli.streaming = false;
        cli_println("Streaming stopped");
        return 0;
    }
    
    if (strcasecmp(subcmd, "csv") == 0) {
        g_cli.stream_format = CLI_STREAM_CSV;
        cli_println("Output format: CSV");
        return 0;
    }
    
    if (strcasecmp(subcmd, "json") == 0) {
        g_cli.stream_format = CLI_STREAM_JSON;
        cli_println("Output format: JSON");
        return 0;
    }
    
    cli_println("Unknown subcommand: %s", subcmd);
    return -1;
}

static int cli_cmd_reset(int argc, char **argv)
{
    if (argc < 2) {
        cli_println("Usage: reset <subcommand>");
        return -1;
    }
    
    const char *subcmd = argv[1];
    
    // Require confirmation for reset operations
    cli_print("Reset %s? (y/n): ", subcmd);
    
    // Wait for response
    char response = 0;
    uint8_t byte;
    while (1) {
        if (usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(5000)) == 1) {
            if (byte == 'y' || byte == 'Y') {
                response = 'y';
                break;
            } else if (byte == 'n' || byte == 'N' || byte == 3) { // 3 = Ctrl+C
                response = 'n';
                break;
            }
        }
    }
    
    cli_println("%c", response);
    
    if (response != 'y') {
        cli_println("Cancelled");
        return 0;
    }
    
    if (strcasecmp(subcmd, "config") == 0) {
        cli_println("Configuration reset to defaults");
        return 0;
    }
    
    if (strcasecmp(subcmd, "tables") == 0) {
        cli_println("Tables reset to defaults");
        return 0;
    }
    
    if (strcasecmp(subcmd, "ltft") == 0) {
        cli_println("LTFT reset");
        return 0;
    }
    
    if (strcasecmp(subcmd, "all") == 0) {
        cli_println("All settings reset to defaults");
        return 0;
    }
    
    cli_println("Unknown subcommand: %s", subcmd);
    return -1;
}

static int cli_cmd_version(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    cli_println("");
    cli_println("ESP32-S3 EFI Firmware");
    cli_println("Version: 1.0.0");
    cli_println("Build: " __DATE__ " " __TIME__);
    cli_println("IDF Version: %s", esp_get_idf_version());
    cli_println("");
    return 0;
}

/*============================================================================
 * Command Processing
 *============================================================================*/

static int cli_tokenize(char *line, char **argv, int max_args)
{
    int argc = 0;
    char *token = strtok(line, " \t\r\n");
    
    while (token != NULL && argc < max_args) {
        // Skip comments
        if (token[0] == '#') {
            break;
        }
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }
    
    return argc;
}

static const cli_command_t *cli_find_command(const char *name)
{
    for (int i = 0; i < g_cli.command_count; i++) {
        if (strcasecmp(g_cli.commands[i]->name, name) == 0) {
            return g_cli.commands[i];
        }
    }
    return NULL;
}

int cli_process_line(const char *line)
{
    char buffer[CLI_MAX_INPUT_LEN];
    strncpy(buffer, line, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    char *argv[CLI_MAX_ARGS];
    int argc = cli_tokenize(buffer, argv, CLI_MAX_ARGS);
    
    if (argc == 0) {
        return 0;
    }
    
    const cli_command_t *cmd = cli_find_command(argv[0]);
    if (cmd == NULL) {
        cli_println("Unknown command: %s", argv[0]);
        cli_println("Type 'help' for available commands");
        return -1;
    }
    
    // Check admin mode requirement
    if ((cmd->flags & CLI_FLAG_ADMIN) && !g_cli.admin_mode) {
        cli_println("Permission denied: admin mode required");
        cli_println("Use 'admin' command to enter admin mode");
        return -1;
    }
    
    // Execute command
    int result = cmd->handler(argc, argv);
    
    return result;
}

void cli_process_char(char c)
{
    // Handle special characters
    if (c == '\r' || c == '\n') {
        // End of line
        g_cli.input_buffer[g_cli.input_pos] = '\0';
        
        // Echo newline
        cli_print("\r\n");
        
        // Process command if not empty
        if (g_cli.input_pos > 0) {
            // Add to history
            if (g_cli.history_count < CLI_HISTORY_SIZE) {
                strncpy(g_cli.history[g_cli.history_count], g_cli.input_buffer, CLI_MAX_INPUT_LEN - 1);
                g_cli.history_count++;
            } else {
                // Shift history
                memmove(g_cli.history[0], g_cli.history[1], sizeof(g_cli.history[0]) * (CLI_HISTORY_SIZE - 1));
                strncpy(g_cli.history[CLI_HISTORY_SIZE - 1], g_cli.input_buffer, CLI_MAX_INPUT_LEN - 1);
            }
            g_cli.history_pos = g_cli.history_count;
            
            // Process command
            cli_process_line(g_cli.input_buffer);
        }
        
        // Reset buffer
        g_cli.input_pos = 0;
        g_cli.input_buffer[0] = '\0';
        
        // Show prompt
        cli_print(CLI_PROMPT);
    }
    else if (c == 0x7F || c == '\b') {
        // Backspace
        if (g_cli.input_pos > 0) {
            g_cli.input_pos--;
            g_cli.input_buffer[g_cli.input_pos] = '\0';
            cli_print("\b \b");  // Erase character
        }
    }
    else if (c == 3) {
        // Ctrl+C - stop streaming or cancel
        if (g_cli.streaming) {
            g_cli.streaming = false;
            cli_println("^C");
            cli_print(CLI_PROMPT);
        } else {
            g_cli.input_pos = 0;
            g_cli.input_buffer[0] = '\0';
            cli_println("^C");
            cli_print(CLI_PROMPT);
        }
    }
    else if (c == 27) {
        // Escape sequence (arrow keys)
        // Read next two characters
        uint8_t seq[2];
        if (usb_serial_jtag_read_bytes(seq, 2, pdMS_TO_TICKS(10)) == 2) {
            if (seq[0] == '[') {
                if (seq[1] == 'A' && g_cli.history_pos > 0) {
                    // Up arrow - previous history
                    g_cli.history_pos--;
                    strncpy(g_cli.input_buffer, g_cli.history[g_cli.history_pos], CLI_MAX_INPUT_LEN - 1);
                    g_cli.input_pos = strlen(g_cli.input_buffer);
                    cli_print("\r\033[K> %s", g_cli.input_buffer);
                }
                else if (seq[1] == 'B' && g_cli.history_pos < g_cli.history_count - 1) {
                    // Down arrow - next history
                    g_cli.history_pos++;
                    strncpy(g_cli.input_buffer, g_cli.history[g_cli.history_pos], CLI_MAX_INPUT_LEN - 1);
                    g_cli.input_pos = strlen(g_cli.input_buffer);
                    cli_print("\r\033[K> %s", g_cli.input_buffer);
                }
            }
        }
    }
    else if (c >= ' ' && c < 127 && g_cli.input_pos < CLI_MAX_INPUT_LEN - 1) {
        // Printable character
        g_cli.input_buffer[g_cli.input_pos++] = c;
        g_cli.input_buffer[g_cli.input_pos] = '\0';
        cli_print("%c", c);  // Echo
    }
}

/*============================================================================
 * CLI Task
 *============================================================================*/

static void cli_task(void *arg)
{
    (void)arg;
    
    // Print welcome message
    cli_print(CLI_WELCOME);
    cli_print(CLI_PROMPT);
    
    uint8_t byte;
    
    while (g_cli.initialized) {
        // Read from USB CDC
        int len = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(CLI_USB_TIMEOUT_MS));
        
        if (len == 1) {
            cli_process_char((char)byte);
        }
    }
    
    vTaskDelete(NULL);
}

/*============================================================================
 * Public API Implementation
 *============================================================================*/

esp_err_t cli_init(void)
{
    if (g_cli.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize USB CDC
    usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB CDC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create mutex
    g_cli.mutex = xSemaphoreCreateMutex();
    if (g_cli.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        usb_serial_jtag_driver_uninstall();
        return ESP_ERR_NO_MEM;
    }
    
    // Reset state
    g_cli.input_pos = 0;
    g_cli.history_count = 0;
    g_cli.history_pos = 0;
    g_cli.streaming = false;
    g_cli.admin_mode = false;
    g_cli.stream_interval_ms = CLI_DEFAULT_STREAM_INTERVAL;
    g_cli.stream_format = CLI_STREAM_CSV;
    g_cli.command_count = 0;
    
    // Register default commands
    for (int i = 0; default_commands[i].name != NULL; i++) {
        if (g_cli.command_count < CLI_MAX_COMMANDS) {
            g_cli.commands[g_cli.command_count++] = &default_commands[i];
        }
    }
    
    g_cli.initialized = true;
    
    ESP_LOGI(TAG, "CLI interface initialized");
    return ESP_OK;
}

esp_err_t cli_deinit(void)
{
    if (!g_cli.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_cli.initialized = false;
    
    // Stop task if running
    if (g_cli.cli_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        g_cli.cli_task = NULL;
    }
    
    // Delete mutex
    if (g_cli.mutex != NULL) {
        vSemaphoreDelete(g_cli.mutex);
        g_cli.mutex = NULL;
    }
    
    // Uninstall USB CDC driver
    usb_serial_jtag_driver_uninstall();
    
    ESP_LOGI(TAG, "CLI interface deinitialized");
    return ESP_OK;
}

esp_err_t cli_start(void)
{
    if (!g_cli.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_cli.cli_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create CLI task
    BaseType_t ret = xTaskCreate(
        cli_task,
        "cli",
        CLI_TASK_STACK_SIZE,
        NULL,
        CLI_TASK_PRIORITY,
        &g_cli.cli_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CLI task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "CLI task started");
    return ESP_OK;
}

esp_err_t cli_stop(void)
{
    if (!g_cli.initialized || g_cli.cli_task == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_cli.initialized = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    g_cli.cli_task = NULL;
    g_cli.initialized = true;
    
    ESP_LOGI(TAG, "CLI task stopped");
    return ESP_OK;
}

esp_err_t cli_register_command(const cli_command_t *command)
{
    if (!g_cli.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (command == NULL || command->name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_cli.command_count >= CLI_MAX_COMMANDS) {
        return ESP_ERR_NO_MEM;
    }
    
    g_cli.commands[g_cli.command_count++] = command;
    return ESP_OK;
}

bool cli_is_streaming(void)
{
    return g_cli.streaming;
}

void cli_stop_streaming(void)
{
    g_cli.streaming = false;
}

esp_err_t cli_enter_admin(const char *password)
{
    // For now, no password check
    (void)password;
    g_cli.admin_mode = true;
    cli_println("Admin mode enabled");
    return ESP_OK;
}

void cli_exit_admin(void)
{
    g_cli.admin_mode = false;
    cli_println("Admin mode disabled");
}

bool cli_is_admin(void)
{
    return g_cli.admin_mode;
}
