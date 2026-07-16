#pragma once
/**
 * @file app/ui_protocol_internal.h
 * Shared UI protocol state/helpers (hygiene PR-10). Not a public API.
 */
#include <cstdint>
#include <cstring>

#include "app/ui_protocol.h"
#include "engine/calibration.h"
#include "engine/constants.h"
#include "engine/fuel_trim.h"
#include "engine/table3d.h"

namespace ems::app::ui_detail {

constexpr uint16_t kRxSize = 256u;
constexpr uint16_t kTxSize = 2048u;
constexpr uint16_t kRxMask = kRxSize - 1u;
constexpr uint16_t kTxMask = kTxSize - 1u;

constexpr uint8_t kAckOk = 0x00u;
constexpr uint8_t kAckErr = 0x01u;
constexpr uint8_t kCommsTestMagic = 0xAAu;

constexpr char kSignature[] = "OpenEMS_v1.3";
constexpr char kFwVersion[] = "OpenEMS_fw_1.3";
constexpr char kProtocolVersion[] = "001";

enum class ParseState : uint8_t {
    IDLE = 0u,
    READ_ARGS = 1u,
    WRITE_ARGS = 2u,
    WRITE_DATA = 3u,
    BURN_ARGS = 4u,
    BENCH_ARG = 5u,
    TEST_ARGS = 9u,
    ENV_SIZE_LO = 6u,
    ENV_PAYLOAD = 7u,
    ENV_CRC = 8u,
};

constexpr uint16_t kEnvMaxChunk   = 800u;
constexpr uint16_t kEnvMaxPayload = 807u;

constexpr uint8_t kTsRcOk       = 0x00u;
constexpr uint8_t kTsRcCrcErr   = 0x82u;
constexpr uint8_t kTsRcUnknown  = 0x83u;
constexpr uint8_t kTsRcRangeErr = 0x84u;
constexpr uint8_t kTsRcBusyErr  = 0x85u;

constexpr uint16_t kTestEnterMagic = 0xA55Au;

// ── Page buffers ────────────────────────────────────────────────────────────
extern uint8_t g_page0[512];
extern uint8_t g_page1_ve[ems::engine::kTableCells];
extern uint8_t g_page2_spark[ems::engine::kTableCells];
extern uint8_t g_page3_rt[86];
extern uint8_t g_page4_lambda[2u * ems::engine::kTableCells];
extern uint8_t g_page5_corr[256];
extern uint8_t g_page6_xtau[80];
extern uint8_t g_page7_dwell2d[32];
extern uint8_t g_page8_pedalmap[80];
extern uint8_t g_page9_boost[112];
extern uint8_t g_page10_ltft[ems::engine::kTableCells +
    static_cast<uint16_t>(ems::engine::kLtftAddAxisSize) * ems::engine::kLtftAddAxisSize];
extern uint8_t g_page11_axes[4u * ems::engine::kTableAxisSize];
extern uint8_t g_page12_ltft_accum[ems::engine::kLtftAccumPageSize];

extern uint8_t g_env_buf[kEnvMaxPayload];
extern uint16_t g_env_size;
extern uint16_t g_env_pos;
extern uint32_t g_env_rx_crc;
extern uint8_t  g_env_crc_pos;

extern volatile uint8_t g_rx_buf[kRxSize];
extern volatile uint16_t g_rx_head;
extern volatile uint16_t g_rx_tail;
extern volatile bool g_rx_flag;

extern volatile uint8_t g_tx_buf[kTxSize];
extern volatile uint16_t g_tx_head;
extern volatile uint16_t g_tx_tail;

extern uint8_t  g_rt_pw_ms_x10;
extern int8_t   g_rt_advance_deg;
extern int8_t   g_rt_stft_p100;
extern uint8_t  g_rt_lambda_target_d4;
extern int8_t   g_rt_ltft_pct;
extern uint16_t g_rt_map_fused_bar_x100;
extern uint16_t g_rt_net_pw_us;
extern uint32_t g_rt_sched_late_events;
extern uint32_t g_rt_sched_cycle_schedule_drop_count;
extern uint32_t g_rt_sched_calibration_clamp_count;
extern uint32_t g_rt_seed_loaded_count;
extern uint32_t g_rt_seed_confirmed_count;
extern uint32_t g_rt_seed_rejected_count;
extern uint8_t  g_rt_sync_state_raw;
extern bool     g_rt_rev_limit_active;
extern uint32_t g_rt_loop2ms_last_us;
extern uint32_t g_rt_loop2ms_max_us;

extern ParseState g_state;
extern uint8_t g_cmd_page;
extern uint16_t g_cmd_off;
extern uint16_t g_cmd_len;
extern uint8_t g_arg_pos;
extern uint8_t g_test_args[4];
extern uint16_t g_write_pos;
extern bool g_write_ram_only;
extern uint16_t g_dirty_page_mask;

// ── Helpers / commands ──────────────────────────────────────────────────────
void enter_critical() noexcept;
void exit_critical() noexcept;
uint16_t page_size(uint8_t page) noexcept;
uint8_t* page_ptr(uint8_t page) noexcept;
uint8_t normalize_page_id(uint8_t page) noexcept;
bool tx_push(uint8_t byte) noexcept;
void tx_push_bytes(const uint8_t* ptr, uint16_t len) noexcept;
bool rx_pop(uint8_t& byte) noexcept;
void write_u32_le(uint8_t* dst, uint32_t v) noexcept;
void update_realtime_page() noexcept;
void reset_parser() noexcept;
void handle_test_cmd() noexcept;
bool bounds_ok(uint8_t page, uint16_t off, uint16_t len) noexcept;
bool command_bounds_ok() noexcept;
bool burn_rpm_safe() noexcept;
void sync_page_from_table(uint8_t page) noexcept;
bool sync_table_from_page(uint8_t page) noexcept;
uint16_t editable_page_bit(uint8_t page) noexcept;
void mark_page_dirty(uint8_t page) noexcept;
void clear_page_dirty(uint8_t page) noexcept;
bool burn_page_to_flash(uint8_t page) noexcept;
void handle_read_done() noexcept;
void handle_write_done() noexcept;
uint16_t tx_free() noexcept;
void env_send_response(uint8_t code, const uint8_t* data, uint16_t len) noexcept;
int env_try_write(const uint8_t* a, uint16_t data_n) noexcept;
void env_dispatch(const uint8_t* p, uint16_t n) noexcept;
void parse_byte(uint8_t b) noexcept;
void reset_pages() noexcept;

}  // namespace ems::app::ui_detail
