#include "app/tuner_studio.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app/can_stack.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/ecu_sched.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"

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
};

alignas(4) static uint8_t g_page0[512] = {};
alignas(4) static uint8_t g_page1_ve[256] = {};
alignas(4) static uint8_t g_page2_spark[256] = {};
alignas(4) static uint8_t g_page3_rt[64] = {};

static volatile uint8_t g_rx_buf[kRxSize] = {};
static volatile uint16_t g_rx_head = 0u;
static volatile uint16_t g_rx_tail = 0u;
static volatile bool g_rx_flag = false;

static uint8_t g_tx_buf[kTxSize] = {};
static uint16_t g_tx_head = 0u;
static uint16_t g_tx_tail = 0u;

static uint8_t  g_rt_pw_ms_x10   = 0u;
static int8_t   g_rt_advance_deg  = 0;
static int8_t   g_rt_stft_p100   = 0;
static uint32_t g_rt_sched_late_events = 0u;
static uint32_t g_rt_sched_late_max_delay_ticks = 0u;
static uint8_t  g_rt_sched_queue_depth_peak = 0u;
static uint8_t  g_rt_sched_queue_depth_last_cycle_peak = 0u;
static uint32_t g_rt_sched_cycle_schedule_drop_count = 0u;
static uint32_t g_rt_sched_calibration_clamp_count = 0u;
static uint32_t g_rt_seed_loaded_count = 0u;
static uint32_t g_rt_seed_confirmed_count = 0u;
static uint32_t g_rt_seed_rejected_count = 0u;
static uint8_t  g_rt_sync_state_raw = 0u;
static uint32_t g_rt_ivc_clamp_count = 0u;

static ParseState g_state = ParseState::IDLE;
static uint8_t g_cmd_page = 0u;
static uint16_t g_cmd_off = 0u;
static uint16_t g_cmd_len = 0u;
static uint8_t g_arg_pos = 0u;
static uint16_t g_write_pos = 0u;

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
    if (page == 0x00u) {
        return 512u;
    }
    if (page == 0x01u || page == 0x02u) {
        return 256u;
    }
    if (page == 0x03u) {
        return 64u;
    }
    return 0u;
}

inline uint8_t* page_ptr(uint8_t page) noexcept {
    if (page == 0x00u) {
        return g_page0;
    }
    if (page == 0x01u) {
        return g_page1_ve;
    }
    if (page == 0x02u) {
        return g_page2_spark;
    }
    if (page == 0x03u) {
        return g_page3_rt;
    }
    return nullptr;
}

inline bool tx_push(uint8_t byte) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_tx_head + 1u) & kTxMask);
    if (next == g_tx_tail) {
        return false;
    }
    g_tx_buf[g_tx_head] = byte;
    g_tx_head = next;
    return true;
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

inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return static_cast<int16_t>(v);
}

inline uint8_t clamp_u8(uint32_t v) noexcept {
    return static_cast<uint8_t>((v > 255u) ? 255u : v);
}

inline void write_u32_le(uint8_t* dst, uint32_t v) noexcept {
    dst[0] = static_cast<uint8_t>(v & 0xFFu);
    dst[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
    dst[2] = static_cast<uint8_t>((v >> 16u) & 0xFFu);
    dst[3] = static_cast<uint8_t>((v >> 24u) & 0xFFu);
}

inline void update_realtime_page() noexcept {
    ems::app::TsRealtimeData rt = {};
    const ems::drv::CkpSnapshot c = ems::drv::ckp_snapshot();
    const ems::drv::SensorData s = ems::drv::sensors_get();  // cópia atômica

    rt.rpm = static_cast<uint16_t>((c.rpm_x10 > 655350u) ? 65535u : (c.rpm_x10 / 10u));
    rt.map_kpa = clamp_u8(s.map_kpa_x10 / 10u);
    rt.tps_pct = clamp_u8(s.tps_pct_x10 / 10u);

    rt.clt_p40 = static_cast<int8_t>(clamp_i16((static_cast<int32_t>(s.clt_degc_x10) / 10) + 40, -128, 127));
    rt.iat_p40 = static_cast<int8_t>(clamp_i16((static_cast<int32_t>(s.iat_degc_x10) / 10) + 40, -128, 127));

    // WBO2 via CAN (lambda × 1000); ÷4 para caber em uint8_t (0..375 range típico)
    rt.o2_mv_d4 = clamp_u8(ems::app::can_stack_lambda_milli() / 4u);
    rt.pw1_ms_x10  = g_rt_pw_ms_x10;
    rt.advance_p40 = static_cast<uint8_t>(static_cast<int16_t>(g_rt_advance_deg) + 40);
    rt.ve          = g_page1_ve[0];
    rt.stft_p100   = g_rt_stft_p100;

    uint8_t status = 0u;
    if (c.state == ems::drv::SyncState::FULL_SYNC) {
        status |= ems::app::STATUS_SYNC_FULL;
    }
    if (c.phase_A) {
        status = static_cast<uint8_t>(status | ems::app::STATUS_PHASE_A);
    }
    if (s.fault_bits != 0u) {
        status = static_cast<uint8_t>(status | ems::app::STATUS_SENSOR_FAULT);
    }
    if (g_rt_sched_late_events != 0u) {
        status = static_cast<uint8_t>(status | ems::app::STATUS_SCHED_LATE);
    }
    if (g_rt_sched_cycle_schedule_drop_count != 0u) {
        status = static_cast<uint8_t>(status | ems::app::STATUS_SCHED_DROP);
    }
    if (g_rt_sched_calibration_clamp_count != 0u) {
        status = static_cast<uint8_t>(status | ems::app::STATUS_SCHED_CLAMP);
    }
    if (ems::app::can_stack_wbo2_fault()) {
        status = static_cast<uint8_t>(status | ems::app::STATUS_WBO2_FAULT);
    }
    rt.status_bits = status;
    write_u32_le(&rt.reserved[0], g_rt_sched_late_events);
    write_u32_le(&rt.reserved[4], g_rt_sched_late_max_delay_ticks);
    rt.reserved[8] = g_rt_sched_queue_depth_peak;
    rt.reserved[9] = g_rt_sched_queue_depth_last_cycle_peak;
    write_u32_le(&rt.reserved[10], g_rt_sched_cycle_schedule_drop_count);
    write_u32_le(&rt.reserved[14], g_rt_sched_calibration_clamp_count);
    write_u32_le(&rt.reserved[18], g_rt_seed_loaded_count);
    write_u32_le(&rt.reserved[22], g_rt_seed_confirmed_count);
    write_u32_le(&rt.reserved[26], g_rt_seed_rejected_count);
    rt.reserved[30] = g_rt_sync_state_raw;
    write_u32_le(&rt.reserved[31], g_rt_ivc_clamp_count);

    std::memcpy(g_page3_rt, &rt, sizeof(rt));
}

inline void reset_parser() noexcept {
    g_state = ParseState::IDLE;
    g_cmd_page = 0u;
    g_cmd_off = 0u;
    g_cmd_len = 0u;
    g_arg_pos = 0u;
    g_write_pos = 0u;
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
    if (page == 0x01u) {
        std::memcpy(g_page1_ve, ems::engine::ve_table, sizeof(g_page1_ve));
    } else if (page == 0x02u) {
        std::memcpy(g_page2_spark, ems::engine::spark_table, sizeof(g_page2_spark));
    }
}

inline void sync_table_from_page(uint8_t page) noexcept {
    if (page == 0x00u) {
        /* Page 0, byte 0: ivc_abdc_deg */
        ::ecu_sched_set_ivc(g_page0[0]);
    } else if (page == 0x01u) {
        std::memcpy(ems::engine::ve_table, g_page1_ve, sizeof(g_page1_ve));
    } else if (page == 0x02u) {
        std::memcpy(ems::engine::spark_table, g_page2_spark, sizeof(g_page2_spark));
    }
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
        }
        return;
    }

    if (g_state == ParseState::READ_ARGS || g_state == ParseState::WRITE_ARGS) {
        switch (g_arg_pos) {
            case 0u:
                g_cmd_page = b;
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
    std::memcpy(g_page1_ve,    ems::engine::ve_table,    sizeof(g_page1_ve));
    std::memcpy(g_page2_spark, ems::engine::spark_table, sizeof(g_page2_spark));
    std::memset(g_page3_rt, 0, sizeof(g_page3_rt));
}

}  // namespace

namespace ems::app {

void ts_init() noexcept {
    enter_critical();
    g_rx_head = 0u;
    g_rx_tail = 0u;
    g_rx_flag = false;
    g_tx_head = 0u;
    g_tx_tail = 0u;
    exit_critical();

    reset_pages();
    reset_parser();
    ts_update_rt_metrics(0u, 0, 0);
    ts_update_rt_sched_diag(0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u);
}

void ts_uart0_rx_isr_byte(uint8_t byte) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_rx_head + 1u) & kRxMask);
    if (next != g_rx_tail) {
        g_rx_buf[g_rx_head] = byte;
        g_rx_head = next;
        g_rx_flag = true;
    }
}

void ts_process() noexcept {
    if (!g_rx_flag && g_rx_head == g_rx_tail) {
        return;
    }

    uint8_t b = 0u;
    while (rx_pop(b)) {
        parse_byte(b);
    }
}

bool ts_tx_pop(uint8_t& byte) noexcept {
    if (g_tx_head == g_tx_tail) {
        return false;
    }
    byte = g_tx_buf[g_tx_tail];
    g_tx_tail = static_cast<uint16_t>((g_tx_tail + 1u) & kTxMask);
    return true;
}

uint16_t ts_tx_available() noexcept {
    return static_cast<uint16_t>((g_tx_head - g_tx_tail) & kTxMask);
}

void ts_update_rt_metrics(uint8_t pw_ms_x10, int8_t advance_deg, int8_t stft_p100) noexcept {
    g_rt_pw_ms_x10  = pw_ms_x10;
    g_rt_advance_deg = advance_deg;
    g_rt_stft_p100  = stft_p100;
}

void ts_update_rt_sched_diag(uint32_t late_events,
                             uint32_t late_max_delay_ticks,
                             uint8_t queue_depth_peak,
                             uint8_t queue_depth_last_cycle_peak,
                             uint32_t cycle_schedule_drop_count,
                             uint32_t calibration_clamp_count,
                             uint32_t seed_loaded_count,
                             uint32_t seed_confirmed_count,
                             uint32_t seed_rejected_count,
                             uint8_t sync_state_raw) noexcept {
    g_rt_sched_late_events = late_events;
    g_rt_sched_late_max_delay_ticks = late_max_delay_ticks;
    g_rt_sched_queue_depth_peak = queue_depth_peak;
    g_rt_sched_queue_depth_last_cycle_peak = queue_depth_last_cycle_peak;
    g_rt_sched_cycle_schedule_drop_count = cycle_schedule_drop_count;
    g_rt_sched_calibration_clamp_count = calibration_clamp_count;
    g_rt_seed_loaded_count = seed_loaded_count;
    g_rt_seed_confirmed_count = seed_confirmed_count;
    g_rt_seed_rejected_count = seed_rejected_count;
    g_rt_sync_state_raw = sync_state_raw;
}

void ts_update_ivc_diag(uint32_t ivc_clamp_count) noexcept {
    g_rt_ivc_clamp_count = ivc_clamp_count;
}

#if defined(EMS_HOST_TEST)
void ts_test_reset() noexcept {
    ts_init();
}
#endif

}  // namespace ems::app
