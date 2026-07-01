#include "app/ui_protocol.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app/can_stack.h"
#include "hal/tle8888.h"
#include "hal/flex_fuel.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/calibration.h"
#include "engine/ecu_sched.h"
#include "engine/etb_control.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/math_utils.h"
#include "hal/flash.h"
#include "engine/engine_config.h"

namespace {

constexpr uint16_t kRxSize = 256u;
constexpr uint16_t kTxSize = 512u;
constexpr uint16_t kRxMask = kRxSize - 1u;
constexpr uint16_t kTxMask = kTxSize - 1u;

constexpr uint8_t kAckOk = 0x00u;
constexpr uint8_t kAckErr = 0x01u;
constexpr uint8_t kCommsTestMagic = 0xAAu;

constexpr char kSignature[] = "OpenEMS_v1.1";
constexpr char kFwVersion[] = "OpenEMS_fw_1.1";
constexpr char kProtocolVersion[] = "001";

enum class ParseState : uint8_t {
    IDLE = 0u,
    READ_ARGS = 1u,
    WRITE_ARGS = 2u,
    WRITE_DATA = 3u,
    BURN_ARGS = 4u,
    BENCH_ARG = 5u,   // 'B' + 1 byte: bench-mode CLT/IAT (0=off, 1=on)
};

alignas(4) static uint8_t g_page0[512] = {};
alignas(4) static uint8_t g_page1_ve[256] = {};
alignas(4) static uint8_t g_page2_spark[256] = {};
alignas(4) static uint8_t g_page3_rt[66]      = {};
alignas(4) static uint8_t g_page4_lambda[512] = {};   // lambda_target_table_x1000
alignas(4) static uint8_t g_page5_corr[256]   = {};   // tabelas de correção 1D
alignas(4) static uint8_t g_page6_xtau[80]    = {};   // X-Tau, AE rate curve, quick crank
alignas(4) static uint8_t g_page7_dwell2d[32] = {};   // Dwell 2D: eixo RPM + factores Q8
alignas(4) static uint8_t g_page8_pedalmap[80] = {};  // Pedal map: 4 modos × 10 × uint16
alignas(4) static uint8_t g_page9_boost[112]   = {};  // Boost map: 7 marchas × 8 RPM × uint16
alignas(4) static uint8_t g_page10_ltft[320]   = {};  // LTFT: mult 16×16 int8 + add 8×8 int8

static volatile uint8_t g_rx_buf[kRxSize] = {};
static volatile uint16_t g_rx_head = 0u;
static volatile uint16_t g_rx_tail = 0u;
static volatile bool g_rx_flag = false;

static volatile uint8_t g_tx_buf[kTxSize] = {};
static volatile uint16_t g_tx_head = 0u;
static volatile uint16_t g_tx_tail = 0u;

static uint8_t  g_rt_pw_ms_x10   = 0u;
static int8_t   g_rt_advance_deg  = 0;
static int8_t   g_rt_stft_p100   = 0;
static uint8_t  g_rt_lambda_target_d4 = 0u;
static int8_t   g_rt_ltft_pct    = 0;
static uint32_t g_rt_sched_late_events = 0u;
static uint32_t g_rt_sched_cycle_schedule_drop_count = 0u;
static uint32_t g_rt_sched_calibration_clamp_count = 0u;
static uint32_t g_rt_seed_loaded_count = 0u;
static uint32_t g_rt_seed_confirmed_count = 0u;
static uint32_t g_rt_seed_rejected_count = 0u;
static uint8_t  g_rt_sync_state_raw = 0u;
static uint32_t g_rt_ivc_clamp_count = 0u;
static uint32_t g_rt_loop2ms_last_us = 0u;
static uint32_t g_rt_loop2ms_max_us = 0u;

static ParseState g_state = ParseState::IDLE;
static uint8_t g_cmd_page = 0u;
static uint16_t g_cmd_off = 0u;
static uint16_t g_cmd_len = 0u;
static uint8_t g_arg_pos = 0u;
static uint16_t g_write_pos = 0u;
static bool g_write_ram_only = false;
static uint8_t g_dirty_page_mask = 0u;

inline void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

inline void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

inline uint16_t page_size(uint8_t page) noexcept {
    if (page == 0x00u || page == 0x04u) {
        return 512u;
    }
    if (page == 0x01u || page == 0x02u || page == 0x05u) {
        return 256u;
    }
    if (page == 0x03u) {
        return static_cast<uint16_t>(sizeof(g_page3_rt));
    }
    if (page == 0x06u) { return static_cast<uint16_t>(sizeof(g_page6_xtau)); }
    if (page == 0x07u) { return static_cast<uint16_t>(sizeof(g_page7_dwell2d)); }
    if (page == 0x08u) { return static_cast<uint16_t>(sizeof(g_page8_pedalmap)); }
    if (page == 0x09u) { return static_cast<uint16_t>(sizeof(g_page9_boost)); }
    if (page == 0x0Au) { return static_cast<uint16_t>(sizeof(g_page10_ltft)); }
    return 0u;
}

inline uint8_t* page_ptr(uint8_t page) noexcept {
    if (page == 0x00u) { return g_page0; }
    if (page == 0x01u) { return g_page1_ve; }
    if (page == 0x02u) { return g_page2_spark; }
    if (page == 0x03u) { return g_page3_rt; }
    if (page == 0x04u) { return g_page4_lambda; }
    if (page == 0x05u) { return g_page5_corr; }
    if (page == 0x06u) { return g_page6_xtau; }
    if (page == 0x07u) { return g_page7_dwell2d; }
    if (page == 0x08u) { return g_page8_pedalmap; }
    if (page == 0x09u) { return g_page9_boost; }
    if (page == 0x0Au) { return g_page10_ltft; }
    return nullptr;
}

inline uint8_t normalize_page_id(uint8_t page) noexcept {
    if (page >= static_cast<uint8_t>('0') && page <= static_cast<uint8_t>('9')) {
        return static_cast<uint8_t>(page - static_cast<uint8_t>('0'));
    }
    return page;
}

inline bool tx_push(uint8_t byte) noexcept {
    enter_critical();
    const uint16_t next = static_cast<uint16_t>((g_tx_head + 1u) & kTxMask);
    bool ok = false;
    if (next != g_tx_tail) {
        g_tx_buf[g_tx_head] = byte;
        g_tx_head = next;
        ok = true;
    }
    exit_critical();
    return ok;
}

inline void tx_push_bytes(const uint8_t* ptr, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (!tx_push(ptr[i])) {
            return;
        }
    }
}

inline bool rx_pop(uint8_t& byte) noexcept {
    bool ok = false;
    enter_critical();
    if (g_rx_head != g_rx_tail) {
        byte = g_rx_buf[g_rx_tail];
        g_rx_tail = static_cast<uint16_t>((g_rx_tail + 1u) & kRxMask);
        ok = true;
    } else {
        g_rx_flag = false;
    }
    exit_critical();
    return ok;
}

inline void write_u32_le(uint8_t* dst, uint32_t v) noexcept {
    dst[0] = static_cast<uint8_t>(v & 0xFFu);
    dst[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
    dst[2] = static_cast<uint8_t>((v >> 16u) & 0xFFu);
    dst[3] = static_cast<uint8_t>((v >> 24u) & 0xFFu);
}

inline void update_realtime_page() noexcept {
    ems::app::UiRealtimeData rt = {};
    const ems::drv::CkpSnapshot c = ems::drv::ckp_snapshot();
    const ems::drv::SensorData s = ems::drv::sensors_get();  // cópia atômica

    rt.rpm = static_cast<uint16_t>((c.rpm_x10 > 655350u) ? 65535u : (c.rpm_x10 / 10u));
    rt.map_bar_x100 = ems::engine::clamp_u8(s.map_bar_x1000 / 10u);
    rt.tps_pct = ems::engine::clamp_u8(s.etb_tps_pct_x10 / 10u);

    rt.clt_p40 = static_cast<int8_t>(ems::engine::clamp_i16((static_cast<int32_t>(s.clt_degc_x10) / 10) + 40, static_cast<int16_t>(-128), static_cast<int16_t>(127)));
    rt.iat_p40 = static_cast<int8_t>(ems::engine::clamp_i16((static_cast<int32_t>(s.iat_degc_x10) / 10) + 40, static_cast<int16_t>(-128), static_cast<int16_t>(127)));

    // WBO2 via CAN (lambda × 1000); ÷4 para caber em uint8_t (0..375 range típico)
    rt.o2_mv_d4 = ems::engine::clamp_u8(ems::app::can_stack_lambda_milli() / 4u);
    rt.pw1_ms_x10  = g_rt_pw_ms_x10;
    rt.advance_p40 = static_cast<uint8_t>(static_cast<int16_t>(g_rt_advance_deg) + 40);
    rt.ve          = g_page1_ve[0];
    rt.stft_p100   = g_rt_stft_p100;
    // VE interpolado vivo (get_ve no ponto rpm×map atual) — rt.ve só expõe VE[0][0].
    rt.reserved[49] = ems::engine::get_ve(
        c.rpm_x10, static_cast<uint16_t>(s.map_bar_x1000 / 10u));

    uint16_t status = 0u;
    if (c.state == ems::drv::SyncState::FULL_SYNC) {
        status |= ems::app::STATUS_SYNC_FULL;
    }
    if (c.phase_A) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_PHASE_A);
    }
    if (s.fault_bits != 0u) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_SENSOR_FAULT);
    }
    if (g_rt_sched_late_events != 0u) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_SCHED_LATE);
    }
    if (g_rt_sched_cycle_schedule_drop_count != 0u) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_SCHED_DROP);
    }
    if (g_rt_sched_calibration_clamp_count != 0u) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_SCHED_CLAMP);
    }
    if (ems::app::can_stack_wbo2_fault()) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_WBO2_FAULT);
    }
    if (!ems::hal::tle8888_ok()) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_TLE8888_FAULT);
    }
    rt.status_bits = status;
    write_u32_le(&rt.reserved[0], g_rt_sched_late_events);
    rt.reserved[4] = g_rt_lambda_target_d4;
    rt.reserved[5] = static_cast<uint8_t>(g_rt_ltft_pct);
    const uint32_t cmp_glitch_cnt = ems::drv::ckp_get_cmp_glitch_count();
    rt.reserved[6] = cmp_glitch_cnt > 255u ? 255u : static_cast<uint8_t>(cmp_glitch_cnt);
    rt.reserved[8] = ems::hal::tle8888_fault_bitmap();
    rt.reserved[9] = ems::hal::flex_fuel_valid()
                    ? ems::hal::flex_fuel_ethanol_pct() : 0u;
    write_u32_le(&rt.reserved[10], g_rt_sched_cycle_schedule_drop_count);
    write_u32_le(&rt.reserved[14], g_rt_sched_calibration_clamp_count);
    write_u32_le(&rt.reserved[18], g_rt_seed_loaded_count);
    write_u32_le(&rt.reserved[22], g_rt_seed_confirmed_count);
    write_u32_le(&rt.reserved[26], g_rt_seed_rejected_count);
    rt.reserved[30] = g_rt_sync_state_raw;
    write_u32_le(&rt.reserved[31], g_rt_ivc_clamp_count);
    write_u32_le(&rt.reserved[35], g_rt_loop2ms_last_us);
    write_u32_le(&rt.reserved[39], g_rt_loop2ms_max_us);
    // ADC bruto p/ calibração (AN1=APP1, AN2=APP2, AN3=ETB TPS1, AN4=ETB TPS2)
    rt.reserved[43] = static_cast<uint8_t>(s.an1_raw & 0xFFu);
    rt.reserved[44] = static_cast<uint8_t>((s.an1_raw >> 8u) & 0xFFu);
    rt.reserved[45] = static_cast<uint8_t>(s.an2_raw & 0xFFu);
    rt.reserved[46] = static_cast<uint8_t>((s.an2_raw >> 8u) & 0xFFu);
    rt.reserved[47] = static_cast<uint8_t>(s.an3_raw & 0xFFu);
    rt.reserved[48] = static_cast<uint8_t>((s.an3_raw >> 8u) & 0xFFu);
    rt.reserved[50] = static_cast<uint8_t>(s.an4_raw & 0xFFu);
    rt.reserved[51] = static_cast<uint8_t>((s.an4_raw >> 8u) & 0xFFu);

    std::memcpy(g_page3_rt, &rt, sizeof(rt));
}

inline void reset_parser() noexcept {
    g_state = ParseState::IDLE;
    g_cmd_page = 0u;
    g_cmd_off = 0u;
    g_cmd_len = 0u;
    g_arg_pos = 0u;
    g_write_pos = 0u;
    g_write_ram_only = false;
}

inline bool command_bounds_ok() noexcept {
    const uint16_t psize = page_size(g_cmd_page);
    if (psize == 0u) {
        return false;
    }
    if (g_cmd_off > psize) {
        return false;
    }
    if (g_cmd_len > static_cast<uint16_t>(psize - g_cmd_off)) {
        return false;
    }
    return true;
}

inline void sync_page_from_table(uint8_t page) noexcept {
    if (page == 0x00u) {
        // Popula bytes 2-15 do buffer UI a partir de g_eng_cfg.
        // Byte 0 (ivc_abdc_deg) é gerido separadamente por ecu_sched_set_ivc.
        ems::engine::cfg::engine_config_serialize(g_page0, 16u);
        // Bytes 16-55: calibração de sensores APP/ETB/TPS + plausibilidade
        ems::engine::sync_etb_calibration_to_page(g_page0 + 16, 40u);
        // Bytes 56-63: trim de combustível e ignição por cilindro (int8 × 4 cada)
        std::memcpy(g_page0 + 56, ems::engine::cyl_fuel_trim_pct, 4u);
        std::memcpy(g_page0 + 60, ems::engine::cyl_ign_trim_deg,  4u);
        // Bytes 64-65: janela de dente CMP
        g_page0[64] = ems::engine::cmp_window_open_tooth;
        g_page0[65] = ems::engine::cmp_window_close_tooth;
        // Bytes 66-99: dirigibilidade (anti-jerk, rev limit, decel cut, LTFT)
        std::memcpy(g_page0 + 66, &ems::engine::antijerk_tpsdot_threshold_x10, 2u);
        std::memcpy(g_page0 + 68, &ems::engine::antijerk_retard_deg,            2u);
        g_page0[70] = ems::engine::antijerk_decay_cycles;
        g_page0[71] = 0u;  // pad
        std::memcpy(g_page0 + 72, &ems::engine::rev_limit_rpm_x10,         4u);
        std::memcpy(g_page0 + 76, &ems::engine::rev_limit_soft_window_x10, 4u);
        std::memcpy(g_page0 + 80, &ems::engine::rev_limit_spark_window_x10, 4u);
        std::memcpy(g_page0 + 84, &ems::engine::rev_limit_max_retard_deg,  2u);
        std::memcpy(g_page0 + 86, &ems::engine::ltft_add_pw_threshold_us,  2u);
        std::memcpy(g_page0 + 88, &ems::engine::decel_cut_tps_threshold_x10, 2u);
        std::memcpy(g_page0 + 90, &ems::engine::decel_cut_entry_rpm_x10,   4u);
        std::memcpy(g_page0 + 94, &ems::engine::decel_cut_exit_rpm_x10,    4u);
        std::memcpy(g_page0 + 98, &ems::engine::decel_cut_min_clt_x10,     2u);
        // Bytes 100-105: marcha lenta ETB
        std::memcpy(g_page0 + 100, &ems::engine::etb_idle_rpm_target,       2u);
        std::memcpy(g_page0 + 102, &ems::engine::etb_idle_min_opening_x10,  2u);
        std::memcpy(g_page0 + 104, &ems::engine::etb_idle_max_opening_x10,  2u);
        // Bytes 106-121: idle RPM target vs CLT (8 × int16 CLT + 8 × uint16 RPM)
        std::memcpy(g_page0 + 106, ems::engine::iac_clt_axis_x10,        16u);
        std::memcpy(g_page0 + 122, ems::engine::iac_idle_target_rpm_x10, 16u);
        // Byte 138: WBO2 CAN ID (uint16)
        std::memcpy(g_page0 + 138, &ems::engine::wbo2_can_id, 2u);
        // Bytes 140-145: STFT closed-loop tuning
        std::memcpy(g_page0 + 140, &ems::engine::stft_kp_x100,       2u);
        std::memcpy(g_page0 + 142, &ems::engine::stft_ki_x1000,      2u);
        std::memcpy(g_page0 + 144, &ems::engine::stft_clamp_pct_x10, 2u);
        // Bytes 146-153: X-τ auto-calibration limits
        std::memcpy(g_page0 + 146, &ems::engine::xtau_x_min_q8,  2u);
        std::memcpy(g_page0 + 148, &ems::engine::xtau_x_max_q8,  2u);
        std::memcpy(g_page0 + 150, &ems::engine::xtau_tau_min,    2u);
        std::memcpy(g_page0 + 152, &ems::engine::xtau_tau_max,    2u);
        // Bytes 154-163: EWG position PID + sensor cal
        std::memcpy(g_page0 + 154, &ems::engine::ewg_kp_x10,       2u);
        std::memcpy(g_page0 + 156, &ems::engine::ewg_ki_x10,       2u);
        std::memcpy(g_page0 + 158, &ems::engine::ewg_kd_x10,       2u);
        std::memcpy(g_page0 + 160, &ems::engine::ewg_pos_min_raw,  2u);
        std::memcpy(g_page0 + 162, &ems::engine::ewg_pos_max_raw,  2u);
    } else if (page == 0x01u) {
        std::memcpy(g_page1_ve, ems::engine::ve_table, sizeof(g_page1_ve));
    } else if (page == 0x02u) {
        std::memcpy(g_page2_spark, ems::engine::spark_table, sizeof(g_page2_spark));
    } else if (page == 0x04u) {
        std::memcpy(g_page4_lambda, ems::engine::lambda_target_table_x1000, 512u);
    } else if (page == 0x05u) {
        uint8_t* p = g_page5_corr;
        std::memcpy(p +   0, ems::engine::clt_corr_axis_x10,          16u);
        std::memcpy(p +  16, ems::engine::clt_corr_x256,              16u);
        std::memcpy(p +  32, ems::engine::iat_corr_axis_x10,          16u);
        std::memcpy(p +  48, ems::engine::iat_corr_x256,              16u);
        std::memcpy(p +  64, ems::engine::warmup_corr_axis_x10,       16u);
        std::memcpy(p +  80, ems::engine::warmup_corr_x256,           16u);
        std::memcpy(p +  96, ems::engine::vbatt_corr_axis_mv,         16u);
        std::memcpy(p + 112, ems::engine::injector_dead_time_us,      16u);
        std::memcpy(p + 128, ems::engine::ae_clt_corr_axis_x10,       16u);
        std::memcpy(p + 144, ems::engine::ae_clt_sens,                16u);
        std::memcpy(p + 160, ems::engine::dwell_vbatt_axis_mv,        16u);
        std::memcpy(p + 176, ems::engine::dwell_ms_x10_table,         16u);
        std::memcpy(p + 192, ems::engine::lambda_delay_rpm_axis_x10,  12u);
        std::memcpy(p + 204, ems::engine::lambda_delay_load_axis_bar_x100, 12u);
        std::memcpy(p + 216, ems::engine::lambda_delay_ms_table,      18u);
        std::memcpy(p + 234, &ems::engine::ae_tpsdot_threshold_x10, 2u);
        std::memcpy(p + 236, &ems::engine::ae_taper_cycles,         2u);
        std::memcpy(p + 238, &ems::engine::ae_max_pw_us,            2u);
        std::memcpy(p + 240, &ems::engine::idle_spark_tps_max_x10,             2u);
        std::memcpy(p + 242, &ems::engine::idle_spark_map_max_bar_x100,             2u);
        std::memcpy(p + 244, &ems::engine::idle_spark_rpm_min_x10,             2u);
        std::memcpy(p + 246, &ems::engine::idle_spark_window_above_target_x10, 2u);
        std::memcpy(p + 248, &ems::engine::idle_spark_deadband_rpm_x10,        2u);
        std::memcpy(p + 250, &ems::engine::idle_spark_rpm_per_deg_x10,         2u);
        std::memcpy(p + 252, &ems::engine::idle_spark_retard_limit_deg,        2u);
        std::memcpy(p + 254, &ems::engine::idle_spark_advance_limit_deg,       2u);
    } else if (page == 0x06u) {
        uint8_t* p = g_page6_xtau;
        std::memset(p, 0, sizeof(g_page6_xtau));
        std::memcpy(p +  0, ems::engine::xtau_clt_axis_x10,     16u);
        std::memcpy(p + 16, ems::engine::xtau_x_fraction_q8,    16u);
        std::memcpy(p + 32, ems::engine::xtau_tau_cycles,       16u);
        std::memcpy(p + 48, ems::engine::ae_tpsdot_axis_x10,     8u);
        std::memcpy(p + 56, ems::engine::ae_pw_adder_us,         8u);
        std::memcpy(p + 64, &ems::engine::crank_enter_rpm_x10,    2u);
        std::memcpy(p + 66, &ems::engine::crank_exit_rpm_x10,     2u);
        std::memcpy(p + 68, &ems::engine::crank_spark_deg,        2u);
        std::memcpy(p + 70, &ems::engine::crank_min_pw_us,        2u);
        std::memcpy(p + 72, &ems::engine::crank_prime_tooth,      2u);
        std::memcpy(p + 74, &ems::engine::crank_prime_max_pw_us,  2u);
    } else if (page == 0x07u) {
        uint8_t* p = g_page7_dwell2d;
        std::memset(p, 0, sizeof(g_page7_dwell2d));
        std::memcpy(p + 0,  ems::engine::dwell_rpm_axis_rpm,  8u);
        std::memcpy(p + 8,  ems::engine::dwell_rpm_factor_q8, 8u);
    } else if (page == 0x08u) {
        std::memcpy(g_page8_pedalmap, ems::engine::etb_pedal_map, sizeof(g_page8_pedalmap));
    } else if (page == 0x09u) {
        std::memcpy(g_page9_boost, ems::engine::boost_target_bar_x1000, sizeof(g_page9_boost));
    } else if (page == 0x0Au) {
        for (uint8_t m = 0u; m < 16u; ++m) {
            for (uint8_t r = 0u; r < 16u; ++r) {
                g_page10_ltft[m * 16u + r] = static_cast<uint8_t>(
                    ems::hal::nvm_read_ltft(r, m));
            }
        }
        for (uint8_t m = 0u; m < 8u; ++m) {
            for (uint8_t r = 0u; r < 8u; ++r) {
                g_page10_ltft[256u + m * 8u + r] = static_cast<uint8_t>(
                    ems::hal::nvm_read_ltft_add(r, m));
            }
        }
    }
}

inline void sync_table_from_page(uint8_t page) noexcept {
    if (page == 0x00u) {
        ::ecu_sched_set_ivc(g_page0[0]);
        // Aplica engine config (displacement, injector, AFR, trigger offset, etc.)
        // engine_config_load valida magic 0x4543 em bytes [14-15] — a escrita via
        // 'w' deve sempre incluir os 16 bytes completos com magic correcto.
        ems::engine::cfg::engine_config_load(g_page0, 16u);
        // Calibração de sensores (bytes 16-55) → globals + drivers
        ems::engine::apply_etb_calibration_from_page(g_page0 + 16, 40u);
        ems::engine::push_sensor_calibration_to_drivers();
        // Trim por cilindro e janela CMP (bytes 56-65)
        std::memcpy(ems::engine::cyl_fuel_trim_pct, g_page0 + 56, 4u);
        std::memcpy(ems::engine::cyl_ign_trim_deg,  g_page0 + 60, 4u);
        ems::engine::cmp_window_open_tooth  = g_page0[64];
        ems::engine::cmp_window_close_tooth = g_page0[65];
        // Dirigibilidade (bytes 66-99)
        std::memcpy(&ems::engine::antijerk_tpsdot_threshold_x10, g_page0 + 66, 2u);
        std::memcpy(&ems::engine::antijerk_retard_deg,            g_page0 + 68, 2u);
        ems::engine::antijerk_decay_cycles = g_page0[70];
        std::memcpy(&ems::engine::rev_limit_rpm_x10,          g_page0 + 72, 4u);
        std::memcpy(&ems::engine::rev_limit_soft_window_x10,  g_page0 + 76, 4u);
        std::memcpy(&ems::engine::rev_limit_spark_window_x10, g_page0 + 80, 4u);
        std::memcpy(&ems::engine::rev_limit_max_retard_deg,   g_page0 + 84, 2u);
        std::memcpy(&ems::engine::ltft_add_pw_threshold_us,   g_page0 + 86, 2u);
        std::memcpy(&ems::engine::decel_cut_tps_threshold_x10, g_page0 + 88, 2u);
        std::memcpy(&ems::engine::decel_cut_entry_rpm_x10,    g_page0 + 90, 4u);
        std::memcpy(&ems::engine::decel_cut_exit_rpm_x10,     g_page0 + 94, 4u);
        std::memcpy(&ems::engine::decel_cut_min_clt_x10,      g_page0 + 98, 2u);
        // Marcha lenta ETB (bytes 100-105)
        std::memcpy(&ems::engine::etb_idle_rpm_target,      g_page0 + 100, 2u);
        std::memcpy(&ems::engine::etb_idle_min_opening_x10, g_page0 + 102, 2u);
        std::memcpy(&ems::engine::etb_idle_max_opening_x10, g_page0 + 104, 2u);
        // Idle RPM target vs CLT (bytes 106-121)
        std::memcpy(ems::engine::iac_clt_axis_x10,        g_page0 + 106, 16u);
        std::memcpy(ems::engine::iac_idle_target_rpm_x10, g_page0 + 122, 16u);
        std::memcpy(&ems::engine::wbo2_can_id,             g_page0 + 138, 2u);
        ems::app::can_stack_set_wbo2_rx_id(ems::engine::wbo2_can_id);
        std::memcpy(&ems::engine::stft_kp_x100,       g_page0 + 140, 2u);
        std::memcpy(&ems::engine::stft_ki_x1000,      g_page0 + 142, 2u);
        std::memcpy(&ems::engine::stft_clamp_pct_x10, g_page0 + 144, 2u);
        std::memcpy(&ems::engine::xtau_x_min_q8,  g_page0 + 146, 2u);
        std::memcpy(&ems::engine::xtau_x_max_q8,  g_page0 + 148, 2u);
        std::memcpy(&ems::engine::xtau_tau_min,    g_page0 + 150, 2u);
        std::memcpy(&ems::engine::xtau_tau_max,    g_page0 + 152, 2u);
        std::memcpy(&ems::engine::ewg_kp_x10,       g_page0 + 154, 2u);
        std::memcpy(&ems::engine::ewg_ki_x10,       g_page0 + 156, 2u);
        std::memcpy(&ems::engine::ewg_kd_x10,       g_page0 + 158, 2u);
        std::memcpy(&ems::engine::ewg_pos_min_raw,  g_page0 + 160, 2u);
        std::memcpy(&ems::engine::ewg_pos_max_raw,  g_page0 + 162, 2u);
        etb_apply_idle_calibration();
    } else if (page == 0x01u) {
        std::memcpy(ems::engine::ve_table, g_page1_ve, sizeof(g_page1_ve));
    } else if (page == 0x02u) {
        std::memcpy(ems::engine::spark_table, g_page2_spark, sizeof(g_page2_spark));
    } else if (page == 0x04u) {
        std::memcpy(ems::engine::lambda_target_table_x1000, g_page4_lambda, 512u);
    } else if (page == 0x05u) {
        const uint8_t* p = g_page5_corr;
        std::memcpy(ems::engine::clt_corr_axis_x10,          p +   0, 16u);
        std::memcpy(ems::engine::clt_corr_x256,              p +  16, 16u);
        std::memcpy(ems::engine::iat_corr_axis_x10,          p +  32, 16u);
        std::memcpy(ems::engine::iat_corr_x256,              p +  48, 16u);
        std::memcpy(ems::engine::warmup_corr_axis_x10,       p +  64, 16u);
        std::memcpy(ems::engine::warmup_corr_x256,           p +  80, 16u);
        std::memcpy(ems::engine::vbatt_corr_axis_mv,         p +  96, 16u);
        std::memcpy(ems::engine::injector_dead_time_us,      p + 112, 16u);
        std::memcpy(ems::engine::ae_clt_corr_axis_x10,       p + 128, 16u);
        std::memcpy(ems::engine::ae_clt_sens,                p + 144, 16u);
        std::memcpy(ems::engine::dwell_vbatt_axis_mv,        p + 160, 16u);
        std::memcpy(ems::engine::dwell_ms_x10_table,         p + 176, 16u);
        std::memcpy(ems::engine::lambda_delay_rpm_axis_x10,  p + 192, 12u);
        std::memcpy(ems::engine::lambda_delay_load_axis_bar_x100, p + 204, 12u);
        std::memcpy(ems::engine::lambda_delay_ms_table,      p + 216, 18u);
        std::memcpy(&ems::engine::ae_tpsdot_threshold_x10, p + 234, 2u);
        std::memcpy(&ems::engine::ae_taper_cycles,         p + 236, 2u);
        std::memcpy(&ems::engine::ae_max_pw_us,            p + 238, 2u);
        std::memcpy(&ems::engine::idle_spark_tps_max_x10,             p + 240, 2u);
        std::memcpy(&ems::engine::idle_spark_map_max_bar_x100,             p + 242, 2u);
        std::memcpy(&ems::engine::idle_spark_rpm_min_x10,             p + 244, 2u);
        std::memcpy(&ems::engine::idle_spark_window_above_target_x10, p + 246, 2u);
        std::memcpy(&ems::engine::idle_spark_deadband_rpm_x10,        p + 248, 2u);
        std::memcpy(&ems::engine::idle_spark_rpm_per_deg_x10,         p + 250, 2u);
        std::memcpy(&ems::engine::idle_spark_retard_limit_deg,        p + 252, 2u);
        std::memcpy(&ems::engine::idle_spark_advance_limit_deg,       p + 254, 2u);
    } else if (page == 0x06u) {
        const uint8_t* p = g_page6_xtau;
        std::memcpy(ems::engine::xtau_clt_axis_x10,  p +  0, 16u);
        std::memcpy(ems::engine::xtau_x_fraction_q8, p + 16, 16u);
        std::memcpy(ems::engine::xtau_tau_cycles,    p + 32, 16u);
        std::memcpy(ems::engine::ae_tpsdot_axis_x10, p + 48,  8u);
        std::memcpy(ems::engine::ae_pw_adder_us,     p + 56,  8u);
        std::memcpy(&ems::engine::crank_enter_rpm_x10,   p + 64, 2u);
        std::memcpy(&ems::engine::crank_exit_rpm_x10,    p + 66, 2u);
        std::memcpy(&ems::engine::crank_spark_deg,       p + 68, 2u);
        std::memcpy(&ems::engine::crank_min_pw_us,       p + 70, 2u);
        std::memcpy(&ems::engine::crank_prime_tooth,     p + 72, 2u);
        std::memcpy(&ems::engine::crank_prime_max_pw_us, p + 74, 2u);
    } else if (page == 0x07u) {
        const uint8_t* p = g_page7_dwell2d;
        std::memcpy(ems::engine::dwell_rpm_axis_rpm,  p + 0,  8u);
        std::memcpy(ems::engine::dwell_rpm_factor_q8, p + 8,  8u);
    } else if (page == 0x08u) {
        std::memcpy(ems::engine::etb_pedal_map, g_page8_pedalmap, sizeof(g_page8_pedalmap));
    } else if (page == 0x09u) {
        std::memcpy(ems::engine::boost_target_bar_x1000, g_page9_boost, sizeof(g_page9_boost));
    }
}

inline uint8_t editable_page_bit(uint8_t page) noexcept {
    if (page == 0x01u) { return 0x01u; }
    if (page == 0x02u) { return 0x02u; }
    if (page == 0x04u) { return 0x04u; }
    if (page == 0x05u) { return 0x08u; }
    if (page == 0x06u) { return 0x10u; }
    if (page == 0x07u) { return 0x20u; }
    if (page == 0x08u) { return 0x40u; }
    if (page == 0x09u) { return 0x80u; }
    return 0u;
}

inline void mark_page_dirty(uint8_t page) noexcept {
    g_dirty_page_mask = static_cast<uint8_t>(g_dirty_page_mask | editable_page_bit(page));
}

inline void clear_page_dirty(uint8_t page) noexcept {
    g_dirty_page_mask = static_cast<uint8_t>(g_dirty_page_mask & static_cast<uint8_t>(~editable_page_bit(page)));
}

inline bool burn_page_to_flash(uint8_t page) noexcept {
    if (page == 0x00u) {
        // Serializa g_eng_cfg → g_page0[2-15] e guarda o slot NVM 0 completo.
        ems::engine::cfg::engine_config_serialize(g_page0, 16u);
        const bool ok = ems::hal::nvm_save_calibration(0u, g_page0,
                                            static_cast<uint16_t>(sizeof(g_page0)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x01u) {
        const bool ok = ems::hal::nvm_save_calibration(1u, g_page1_ve, static_cast<uint16_t>(sizeof(g_page1_ve)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x02u) {
        const bool ok = ems::hal::nvm_save_calibration(2u, g_page2_spark, static_cast<uint16_t>(sizeof(g_page2_spark)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x04u) {
        const bool ok = ems::hal::nvm_save_calibration(3u, g_page4_lambda, 512u);
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x05u) {
        const bool ok = ems::hal::nvm_save_calibration(4u, g_page5_corr, static_cast<uint16_t>(sizeof(g_page5_corr)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x06u) {
        const bool ok = ems::hal::nvm_save_calibration(5u, g_page6_xtau, static_cast<uint16_t>(sizeof(g_page6_xtau)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x07u) {
        const bool ok = ems::hal::nvm_save_calibration(6u, g_page7_dwell2d, static_cast<uint16_t>(sizeof(g_page7_dwell2d)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x08u) {
        const bool ok = ems::hal::nvm_save_calibration(7u, g_page8_pedalmap, static_cast<uint16_t>(sizeof(g_page8_pedalmap)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    if (page == 0x09u) {
        const bool ok = ems::hal::nvm_save_calibration(8u, g_page9_boost, static_cast<uint16_t>(sizeof(g_page9_boost)));
        if (!ok) { return false; }
        clear_page_dirty(page);
        return true;
    }
    return false;
}

inline void handle_read_done() noexcept {
    if (!command_bounds_ok()) {
        tx_push(kAckErr);
        reset_parser();
        return;
    }

    if (g_cmd_page == 0x03u) {
        update_realtime_page();
    } else {
        sync_page_from_table(g_cmd_page);
    }

    const uint8_t* ptr = page_ptr(g_cmd_page);
    if (ptr == nullptr) { tx_push(kAckErr); reset_parser(); return; }
    tx_push_bytes(ptr + g_cmd_off, g_cmd_len);
    reset_parser();
}

inline void handle_write_done() noexcept {
    if (!command_bounds_ok() || g_cmd_page == 0x03u) {
        tx_push(kAckErr);
        reset_parser();
        return;
    }

    sync_table_from_page(g_cmd_page);
    mark_page_dirty(g_cmd_page);
    if (!g_write_ram_only) {
        if (!burn_page_to_flash(g_cmd_page)) {
            tx_push(kAckErr);
            reset_parser();
            return;
        }
    }
    tx_push(kAckOk);
    reset_parser();
}

inline void parse_byte(uint8_t b) noexcept {
    if (g_state == ParseState::IDLE) {
        // Ignore line-state reset probe bytes used by some host stacks.
        if (b == 0xF0u) {
            return;
        }
        if (b == static_cast<uint8_t>('Q')) {
            tx_push_bytes(reinterpret_cast<const uint8_t*>(kSignature), static_cast<uint16_t>(sizeof(kSignature) - 1u));
            return;
        }
        if (b == static_cast<uint8_t>('H')) {
            tx_push_bytes(reinterpret_cast<const uint8_t*>(kSignature), static_cast<uint16_t>(sizeof(kSignature) - 1u));
            return;
        }
        if (b == static_cast<uint8_t>('S')) {
            tx_push_bytes(reinterpret_cast<const uint8_t*>(kFwVersion), static_cast<uint16_t>(sizeof(kFwVersion) - 1u));
            return;
        }
        if (b == static_cast<uint8_t>('F')) {
            tx_push_bytes(reinterpret_cast<const uint8_t*>(kProtocolVersion), static_cast<uint16_t>(sizeof(kProtocolVersion) - 1u));
            return;
        }
        if (b == static_cast<uint8_t>('C')) {
            tx_push(kAckOk);
            tx_push(kCommsTestMagic);
            return;
        }
        if (b == static_cast<uint8_t>('A')) {
            update_realtime_page();
            tx_push_bytes(g_page3_rt, sizeof(g_page3_rt));
            return;
        }
        if (b == static_cast<uint8_t>('O')) {
            update_realtime_page();
            tx_push_bytes(g_page3_rt, sizeof(g_page3_rt));
            return;
        }
        if (b == static_cast<uint8_t>('r')) {
            g_state = ParseState::READ_ARGS;
            g_arg_pos = 0u;
            g_cmd_page = 0u;
            g_cmd_off = 0u;
            g_cmd_len = 0u;
            return;
        }
        if (b == static_cast<uint8_t>('w')) {
            g_state = ParseState::WRITE_ARGS;
            g_arg_pos = 0u;
            g_cmd_page = 0u;
            g_cmd_off = 0u;
            g_cmd_len = 0u;
            g_write_pos = 0u;
            g_write_ram_only = false;
            return;
        }
        if (b == static_cast<uint8_t>('x')) {
            g_state = ParseState::WRITE_ARGS;
            g_arg_pos = 0u;
            g_cmd_page = 0u;
            g_cmd_off = 0u;
            g_cmd_len = 0u;
            g_write_pos = 0u;
            g_write_ram_only = true;
            return;
        }
        if (b == static_cast<uint8_t>('b')) {
            g_state = ParseState::BURN_ARGS;
            g_arg_pos = 0u;
            g_cmd_page = 0u;
            return;
        }
        if (b == static_cast<uint8_t>('d')) {
            tx_push(g_dirty_page_mask);
            return;
        }
        if (b == static_cast<uint8_t>('B')) {
            g_state = ParseState::BENCH_ARG;
            return;
        }
        if (b == static_cast<uint8_t>('G')) {
            // Angle measurement: gap_ts + 8 latest {ts, high} from dispatch ring
            // Format: [gap_ts:4] [idx:1] [8×{ts:4, high:1}] = 45 bytes
            extern volatile uint32_t g_last_gap_ts __asm("g_last_gap_ts");
            extern volatile uint8_t g_ts_ring_idx __asm("g_ts_ring_idx");
            // TsEntry is {uint32_t ts; uint8_t high} — 8 bytes with padding
            // Read raw memory: each entry is 8 bytes at g_ts_ring base
            extern volatile uint32_t g_ts_ring __asm("g_ts_ring");
            const uint32_t gap = g_last_gap_ts;
            const uint8_t ridx = g_ts_ring_idx;
            tx_push_bytes(reinterpret_cast<const uint8_t*>(&gap), 4U);
            tx_push(ridx);
            const volatile uint8_t* base = reinterpret_cast<const volatile uint8_t*>(&g_ts_ring);
            for (uint8_t i = 0; i < 8U; ++i) {
                const uint8_t ri = (ridx + 32U - 8U + i) & 31U;
                // TsEntry is 8 bytes: ts(4) + high(1) + channel(1) + pad(2)
                const uint32_t ts = *reinterpret_cast<const volatile uint32_t*>(base + ri * 8U);
                const uint8_t h = *(base + ri * 8U + 4U);
                const uint8_t ch = *(base + ri * 8U + 5U);
                tx_push_bytes(reinterpret_cast<const uint8_t*>(&ts), 4U);
                tx_push(h);
                tx_push(ch);
            }
            return;
        }
        if (b == static_cast<uint8_t>('P')) {
            extern volatile uint8_t g_inj_pw_override __asm("g_inj_pw_override");
            g_inj_pw_override = 2U;  // 2=write-once then lock
            ecu_sched_commit_calibration(10U, 22500U, 50000U, 30U);  // eoi_lead=30° (EOI targeting, valor de bench)
            tx_push(0x00u);
            return;
        }
        if (b == static_cast<uint8_t>('V')) {
            extern volatile uint32_t g_pin_high_count[] __asm("g_pin_high_count");
            extern volatile uint32_t g_pin_low_count[] __asm("g_pin_low_count");
            extern volatile uint32_t g_pin_seq_error[] __asm("g_pin_seq_error");
            // Todos os 8 canais: [0-3]=INJ1-4, [4-7]=IGN1-4
            const uint32_t v[24] = {
                g_pin_high_count[0], g_pin_low_count[0], g_pin_seq_error[0],
                g_pin_high_count[1], g_pin_low_count[1], g_pin_seq_error[1],
                g_pin_high_count[2], g_pin_low_count[2], g_pin_seq_error[2],
                g_pin_high_count[3], g_pin_low_count[3], g_pin_seq_error[3],
                g_pin_high_count[4], g_pin_low_count[4], g_pin_seq_error[4],
                g_pin_high_count[5], g_pin_low_count[5], g_pin_seq_error[5],
                g_pin_high_count[6], g_pin_low_count[6], g_pin_seq_error[6],
                g_pin_high_count[7], g_pin_low_count[7], g_pin_seq_error[7],
            };
            tx_push_bytes(reinterpret_cast<const uint8_t*>(v), 96U);
            return;
        }
        if (b == static_cast<uint8_t>('D')) {
            extern volatile uint32_t g_dbg_tim3_isr_count __asm("g_dbg_tim3_isr_count");
            extern volatile uint32_t g_dbg_tim1cc_isr_count __asm("g_dbg_tim1cc_isr_count");
            extern volatile uint32_t g_dbg_inj_force_early __asm("g_dbg_inj_force_early");
            extern volatile uint32_t g_dbg_ign_force_early __asm("g_dbg_ign_force_early");
            extern volatile uint32_t g_dbg_inj1_arm __asm("g_dbg_inj1_arm");
            extern volatile uint32_t g_dbg_seq_calls __asm("g_dbg_seq_calls");
            extern volatile uint32_t g_dbg_evt_overflow __asm("g_dbg_evt_overflow");
            extern volatile uint32_t g_dbg_clear_all_count __asm("g_dbg_clear_all_count");
            extern volatile uint32_t g_diag_isr_count __asm("_ZN3ems3drv16g_diag_isr_countE");
            extern volatile uint32_t g_dbg_tc_gap __asm("_ZN3ems3drv12g_dbg_tc_gapE");
            extern volatile uint32_t g_dbg_tc_spike __asm("_ZN3ems3drv14g_dbg_tc_spikeE");
            extern volatile uint32_t g_dbg_tc_normal __asm("_ZN3ems3drv15g_dbg_tc_normalE");
            extern volatile uint32_t g_dbg_presync_count __asm("g_dbg_presync_count");
            extern volatile uint32_t g_dbg_phase_skip __asm("g_dbg_phase_skip");
            extern volatile uint32_t g_dbg_phase_fire __asm("g_dbg_phase_fire");
            extern volatile uint32_t g_dbg_evt_inserted __asm("g_dbg_evt_inserted");
            extern volatile uint32_t g_dbg_evt_dispatched __asm("g_dbg_evt_dispatched");
            extern volatile uint32_t g_diag_presync_revs __asm("g_diag_presync_revs");
            extern volatile uint32_t g_diag_seq_revs __asm("g_diag_seq_revs");
            extern volatile uint32_t g_diag_clear_all_count __asm("g_diag_clear_all_count");
            const uint32_t diag[19] = {
                g_late_event_count,
                g_cycle_schedule_drop_count,
                g_dbg_inj1_arm,
                g_dbg_seq_calls,
                g_dbg_evt_overflow,
                g_dbg_clear_all_count,
                g_dbg_presync_count,
                ecu_sched_dwell_watchdog_count(),
                g_diag_isr_count,
                g_dbg_tc_gap,
                g_dbg_tc_spike,
                g_dbg_tc_normal,
                g_dbg_phase_skip,
                g_dbg_phase_fire,
                g_dbg_evt_inserted,
                g_dbg_evt_dispatched,
                g_diag_presync_revs,
                g_diag_seq_revs,
                g_diag_clear_all_count,
            };
            tx_push_bytes(reinterpret_cast<const uint8_t*>(diag), 76U);
            return;
        }
        return;
    }

    if (g_state == ParseState::BENCH_ARG) {
        // Bench-mode CLT/IAT p/ HIL: 0=off (ADC normal), !=0=on (90°C/25°C fixos,
        // sem SENSOR_FAULT pelos canais CLT/IAT). Ver sensors_set_bench_clt_iat.
        ems::drv::sensors_set_bench_clt_iat(b != 0u, 900, 250);
        tx_push(kAckOk);
        reset_parser();
        return;
    }

    if (g_state == ParseState::BURN_ARGS) {
        g_cmd_page = normalize_page_id(b);
        if (burn_page_to_flash(g_cmd_page)) {
            tx_push(kAckOk);
        } else {
            tx_push(kAckErr);
        }
        reset_parser();
        return;
    }

    if (g_state == ParseState::READ_ARGS || g_state == ParseState::WRITE_ARGS) {
        switch (g_arg_pos) {
            case 0u:
                g_cmd_page = normalize_page_id(b);
                break;
            case 1u:
                g_cmd_off = b;
                break;
            case 2u:
                g_cmd_off = static_cast<uint16_t>(g_cmd_off | (static_cast<uint16_t>(b) << 8u));
                break;
            case 3u:
                g_cmd_len = b;
                break;
            case 4u:
                g_cmd_len = static_cast<uint16_t>(g_cmd_len | (static_cast<uint16_t>(b) << 8u));
                break;
            default:
                break;
        }
        ++g_arg_pos;

        if (g_arg_pos < 5u) {
            return;
        }

        if (g_state == ParseState::READ_ARGS) {
            handle_read_done();
            return;
        }

        if (!command_bounds_ok() || g_cmd_page == 0x03u) {
            tx_push(kAckErr);
            reset_parser();
            return;
        }

        if (g_cmd_len == 0u) {
            tx_push(kAckOk);
            reset_parser();
            return;
        }

        g_state = ParseState::WRITE_DATA;
        g_write_pos = 0u;
        return;
    }

    if (g_state == ParseState::WRITE_DATA) {
        uint8_t* ptr = page_ptr(g_cmd_page);
        // FIX-3: guarda defensiva — page_ptr() retorna nullptr para page inválida.
        // Em teoria g_cmd_page já foi validado em WRITE_ARGS, mas a guarda aqui
        // protege contra refatorações futuras que criem caminhos alternativos para
        // WRITE_DATA sem validação prévia.
        if (ptr == nullptr) {
            reset_parser();
            return;
        }
        ptr[g_cmd_off + g_write_pos] = b;
        ++g_write_pos;
        if (g_write_pos >= g_cmd_len) {
            handle_write_done();
        }
    }
}

inline void reset_pages() noexcept {
    std::memset(g_page0, 0, sizeof(g_page0));
    g_page0[0] = 50u;  /* ivc_abdc_deg padrão: 50° ABDC */
    ::ecu_sched_set_ivc(g_page0[0]);
    // Popula campos de engine config com valores actuais (de NVM ou defaults de
    // compilação). Garante que 'r' page 0 devolve valores coerentes antes de
    // qualquer 'w'.
    ems::engine::cfg::engine_config_serialize(g_page0, 16u);
    ems::engine::sync_etb_calibration_to_page(g_page0 + 16, 40u);
    std::memcpy(g_page1_ve,    ems::engine::ve_table,    sizeof(g_page1_ve));
    std::memcpy(g_page2_spark, ems::engine::spark_table, sizeof(g_page2_spark));
    std::memset(g_page3_rt, 0, sizeof(g_page3_rt));
    sync_page_from_table(0x04u);
    sync_page_from_table(0x05u);
    sync_page_from_table(0x06u);
    sync_page_from_table(0x07u);
    g_dirty_page_mask = 0u;
}

}  // namespace

namespace ems::app {

void ui_init() noexcept {
    enter_critical();
    g_rx_head = 0u;
    g_rx_tail = 0u;
    g_rx_flag = false;
    g_tx_head = 0u;
    g_tx_tail = 0u;
    exit_critical();

    reset_pages();
    reset_parser();
    ui_update_rt_metrics(0u, 0, 0);
    ui_update_rt_sched_diag(0u, 0u, 0u, 0u, 0u, 0u, 0u);
}

void ui_rx_byte(uint8_t byte) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_rx_head + 1u) & kRxMask);
    if (next != g_rx_tail) {
        g_rx_buf[g_rx_head] = byte;
        g_rx_head = next;
        g_rx_flag = true;
    }
}


void ui_uart0_rx_isr_byte(uint8_t byte) noexcept {
    ui_rx_byte(byte);
}

void ui_process() noexcept {
    if (!g_rx_flag && g_rx_head == g_rx_tail) {
        return;
    }

    uint8_t b = 0u;
    while (rx_pop(b)) {
        parse_byte(b);
    }
}

bool ui_tx_pop(uint8_t& byte) noexcept {
    if (g_tx_head == g_tx_tail) {
        return false;
    }
    byte = g_tx_buf[g_tx_tail];
    g_tx_tail = static_cast<uint16_t>((g_tx_tail + 1u) & kTxMask);
    return true;
}

uint16_t ui_tx_available() noexcept {
    return static_cast<uint16_t>((g_tx_head - g_tx_tail) & kTxMask);
}

void ui_update_rt_metrics(uint8_t pw_ms_x10, int8_t advance_deg, int8_t stft_p100,
                          uint8_t lambda_target_d4, int8_t ltft_pct) noexcept {
    g_rt_pw_ms_x10  = pw_ms_x10;
    g_rt_advance_deg = advance_deg;
    g_rt_stft_p100  = stft_p100;
    g_rt_lambda_target_d4 = lambda_target_d4;
    g_rt_ltft_pct   = ltft_pct;
}

void ui_update_rt_sched_diag(uint32_t late_events,
                             uint32_t cycle_schedule_drop_count,
                             uint32_t calibration_clamp_count,
                             uint32_t seed_loaded_count,
                             uint32_t seed_confirmed_count,
                             uint32_t seed_rejected_count,
                             uint8_t sync_state_raw) noexcept {
    g_rt_sched_late_events = late_events;
    g_rt_sched_cycle_schedule_drop_count = cycle_schedule_drop_count;
    g_rt_sched_calibration_clamp_count = calibration_clamp_count;
    g_rt_seed_loaded_count = seed_loaded_count;
    g_rt_seed_confirmed_count = seed_confirmed_count;
    g_rt_seed_rejected_count = seed_rejected_count;
    g_rt_sync_state_raw = sync_state_raw;
}

void ui_update_loop_diag(uint32_t loop2ms_last_us,
                         uint32_t loop2ms_max_us) noexcept {
    g_rt_loop2ms_last_us = loop2ms_last_us;
    g_rt_loop2ms_max_us = loop2ms_max_us;
}

void ui_update_ivc_diag(uint32_t ivc_clamp_count) noexcept {
    g_rt_ivc_clamp_count = ivc_clamp_count;
}

#if defined(EMS_HOST_TEST)
void ui_test_reset() noexcept {
    ui_init();
}
#endif

}  // namespace ems::app
