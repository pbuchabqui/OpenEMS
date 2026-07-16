#include "app/ui_protocol.h"
#include "app/ui_protocol_internal.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app/can_stack.h"
#include "app/can_rx_map.h"
#include "hal/tle8888.h"
#include "hal/flex_fuel.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/calibration.h"
#include "engine/ecu_sched.h"
#include "engine/etb_control.h"
#include "engine/fuel_calc.h"
#include "engine/torque_manager.h"
#include "engine/map_estimator.h"
#include "app/status_bits.h"
#include "engine/ign_calc.h"
#include "engine/math_utils.h"
#include "engine/output_test.h"
#include "hal/timer.h"
#include "engine/constants.h"
#include "engine/table3d.h"
#include "hal/crc32.h"
#include "hal/flash.h"
#include "engine/engine_config.h"

namespace ems::app::ui_detail {

// Page / ring / RT / parser state (single definition TU).

alignas(4) uint8_t g_page0[512] = {};
alignas(4) uint8_t g_page1_ve[ems::engine::kTableCells] = {};
alignas(4) uint8_t g_page2_spark[ems::engine::kTableCells] = {};
alignas(4) uint8_t g_page3_rt[86]      = {};
alignas(4) uint8_t g_page4_lambda[2u * ems::engine::kTableCells] = {};   // lambda_target_table_x1000
alignas(4) uint8_t g_page5_corr[256]   = {};   // tabelas de correção 1D
alignas(4) uint8_t g_page6_xtau[80]    = {};   // X-Tau, AE rate curve, quick crank
alignas(4) uint8_t g_page7_dwell2d[32] = {};   // Dwell 2D: eixo RPM + factores Q8
alignas(4) uint8_t g_page8_pedalmap[80] = {};  // Pedal map: 4 modos × 10 × uint16
alignas(4) uint8_t g_page9_boost[112]   = {};  // Boost map: 7 marchas × 8 RPM × uint16
alignas(4) uint8_t g_page10_ltft[ems::engine::kTableCells +
    static_cast<uint16_t>(ems::engine::kLtftAddAxisSize) * ems::engine::kLtftAddAxisSize] = {};
alignas(4) uint8_t g_page11_axes[4u * ems::engine::kTableAxisSize] = {};
alignas(4) uint8_t g_page12_ltft_accum[ems::engine::kLtftAccumPageSize] = {};
alignas(4) uint8_t g_env_buf[kEnvMaxPayload] = {};
uint16_t g_env_size = 0u;
uint16_t g_env_pos = 0u;
uint32_t g_env_rx_crc = 0u;
uint8_t  g_env_crc_pos = 0u;
volatile uint8_t g_rx_buf[kRxSize] = {};
volatile uint16_t g_rx_head = 0u;
volatile uint16_t g_rx_tail = 0u;
volatile bool g_rx_flag = false;
volatile uint8_t g_tx_buf[kTxSize] = {};
volatile uint16_t g_tx_head = 0u;
volatile uint16_t g_tx_tail = 0u;
uint8_t  g_rt_pw_ms_x10   = 0u;
int8_t   g_rt_advance_deg  = 0;
int8_t   g_rt_stft_p100   = 0;
uint8_t  g_rt_lambda_target_d4 = 0u;
int8_t   g_rt_ltft_pct    = 0;
uint16_t g_rt_map_fused_bar_x100 = 0u;
uint16_t g_rt_net_pw_us   = 0u;
uint32_t g_rt_sched_late_events = 0u;
uint32_t g_rt_sched_cycle_schedule_drop_count = 0u;
uint32_t g_rt_sched_calibration_clamp_count = 0u;
uint32_t g_rt_seed_loaded_count = 0u;
uint32_t g_rt_seed_confirmed_count = 0u;
uint32_t g_rt_seed_rejected_count = 0u;
uint8_t  g_rt_sync_state_raw = 0u;
bool     g_rt_rev_limit_active = false;
uint32_t g_rt_loop2ms_last_us = 0u;
uint32_t g_rt_loop2ms_max_us = 0u;
ParseState g_state = ParseState::IDLE;
uint8_t g_cmd_page = 0u;
uint16_t g_cmd_off = 0u;
uint16_t g_cmd_len = 0u;
uint8_t g_arg_pos = 0u;
uint8_t g_test_args[4] = {};  // 'T': subcmd, arg1, arg2_lo, arg2_hi
uint16_t g_write_pos = 0u;
bool g_write_ram_only = false;
uint16_t g_dirty_page_mask = 0u;

}  // namespace ems::app::ui_detail
