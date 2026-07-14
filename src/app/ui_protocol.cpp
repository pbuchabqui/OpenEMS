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
#include "engine/output_test.h"
#include "hal/timer.h"
#include "engine/constants.h"
#include "engine/table3d.h"
#include "hal/crc32.h"
#include "hal/flash.h"
#include "engine/engine_config.h"

namespace {

constexpr uint16_t kRxSize = 256u;
// 2048: precisa caber [size(2)+code(1)+dados(até 800, página 4 20×20 i16)+
// crc(4)] = 807 B de uma só vez, mais folga p/ tráfego legacy concorrente.
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
    BENCH_ARG = 5u,   // 'B' + 1 byte: bench-mode CLT/IAT (0=off, 1=on)
    TEST_ARGS = 9u,   // 'T' + 4 bytes: subcmd + arg1 + arg2 u16 LE (teste de saídas)
    ENV_SIZE_LO = 6u,  // envelope TS: byte baixo do tamanho (BE)
    ENV_PAYLOAD = 7u,  // envelope TS: cmd + args + dados
    ENV_CRC = 8u,      // envelope TS: CRC32 BE (4 bytes)
};

// ── Envelope TunerStudio (msEnvelope_1.0) ────────────────────────────────────
// Frame de pedido:  [size u16 BE][cmd+args+dados][CRC32 u32 BE sobre payload]
// Frame de resposta:[size u16 BE][code][dados][CRC32 u32 BE sobre code+dados]
// Coexiste com o protocolo legacy: comandos legacy são letras ASCII (≥0x20),
// o byte alto do size de um frame <8 KB é 0x00-0x1F — detecção por 1º byte.
// 807 = 'w' + canId + page + off(2) + len(2) + 800 dados (página 4 = lambda
// 20×20 i16, a maior declarada no .ini). kEnvMaxChunk tinha ficado em 256 (=
// blockingFactor), mas o Comm Manager real do TunerStudio pode pedir a
// página inteira numa só transação em vez de fatiar por blockingFactor —
// confirmado empiricamente: 'r' page0/page4 com count=página devolvia 0x84
// (range) e travava a conexão em "Unsupported Controller Firmware".
// Maior página whole-read no envelope: page4 lambda 20×20 = 800 B.
constexpr uint16_t kEnvMaxChunk   = 800u;
constexpr uint16_t kEnvMaxPayload = 807u;

// Códigos de resposta (convenção TS/Speeduino: 0x00 OK, não-zero erro)
constexpr uint8_t kTsRcOk       = 0x00u;
constexpr uint8_t kTsRcCrcErr   = 0x82u;
constexpr uint8_t kTsRcUnknown  = 0x83u;
constexpr uint8_t kTsRcRangeErr = 0x84u;
constexpr uint8_t kTsRcBusyErr  = 0x85u;  // burn bloqueado com motor girando

alignas(4) static uint8_t g_page0[512] = {};
alignas(4) static uint8_t g_page1_ve[ems::engine::kTableCells] = {};
alignas(4) static uint8_t g_page2_spark[ems::engine::kTableCells] = {};
alignas(4) static uint8_t g_page3_rt[86]      = {};
alignas(4) static uint8_t g_page4_lambda[2u * ems::engine::kTableCells] = {};   // lambda_target_table_x1000
alignas(4) static uint8_t g_page5_corr[256]   = {};   // tabelas de correção 1D
alignas(4) static uint8_t g_page6_xtau[80]    = {};   // X-Tau, AE rate curve, quick crank
alignas(4) static uint8_t g_page7_dwell2d[32] = {};   // Dwell 2D: eixo RPM + factores Q8
alignas(4) static uint8_t g_page8_pedalmap[80] = {};  // Pedal map: 4 modos × 10 × uint16
alignas(4) static uint8_t g_page9_boost[112]   = {};  // Boost map: 7 marchas × 8 RPM × uint16
alignas(4) static uint8_t g_page10_ltft[ems::engine::kTableCells +
    static_cast<uint16_t>(ems::engine::kLtftAddAxisSize) * ems::engine::kLtftAddAxisSize] = {};  // LTFT: mult N×N int8 + add sub-grid int8
alignas(4) static uint8_t g_page11_axes[4u * ems::engine::kTableAxisSize]    = {};  // Eixos: 16×u16 RPM + 16×u16 load bar×100
// Page 12: LTFT accum viz (read-only) — hits u8 + mean_stft i8, 800 B
alignas(4) static uint8_t g_page12_ltft_accum[ems::engine::kLtftAccumPageSize] = {};

alignas(4) static uint8_t g_env_buf[kEnvMaxPayload] = {};
static uint16_t g_env_size = 0u;
static uint16_t g_env_pos = 0u;
static uint32_t g_env_rx_crc = 0u;
static uint8_t  g_env_crc_pos = 0u;

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
static uint16_t g_rt_map_fused_bar_x100 = 0u;
static uint16_t g_rt_net_pw_us   = 0u;
static uint32_t g_rt_sched_late_events = 0u;
static uint32_t g_rt_sched_cycle_schedule_drop_count = 0u;
static uint32_t g_rt_sched_calibration_clamp_count = 0u;
static uint32_t g_rt_seed_loaded_count = 0u;
static uint32_t g_rt_seed_confirmed_count = 0u;
static uint32_t g_rt_seed_rejected_count = 0u;
static uint8_t  g_rt_sync_state_raw = 0u;
static bool     g_rt_rev_limit_active = false;
static uint32_t g_rt_ivc_clamp_count = 0u;
static uint32_t g_rt_loop2ms_last_us = 0u;
static uint32_t g_rt_loop2ms_max_us = 0u;

static ParseState g_state = ParseState::IDLE;
static uint8_t g_cmd_page = 0u;
static uint16_t g_cmd_off = 0u;
static uint16_t g_cmd_len = 0u;
static uint8_t g_arg_pos = 0u;
static uint8_t g_test_args[4] = {};  // 'T': subcmd, arg1, arg2_lo, arg2_hi
static uint16_t g_write_pos = 0u;
static bool g_write_ram_only = false;
static uint16_t g_dirty_page_mask = 0u;

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
    if (page == 0x00u) { return 512u; }
    if (page == 0x04u) { return static_cast<uint16_t>(sizeof(g_page4_lambda)); }
    if (page == 0x01u) { return static_cast<uint16_t>(sizeof(g_page1_ve)); }
    if (page == 0x02u) { return static_cast<uint16_t>(sizeof(g_page2_spark)); }
    if (page == 0x05u) { return 256u; }
    if (page == 0x03u) {
        return static_cast<uint16_t>(sizeof(g_page3_rt));
    }
    if (page == 0x06u) { return static_cast<uint16_t>(sizeof(g_page6_xtau)); }
    if (page == 0x07u) { return static_cast<uint16_t>(sizeof(g_page7_dwell2d)); }
    if (page == 0x08u) { return static_cast<uint16_t>(sizeof(g_page8_pedalmap)); }
    if (page == 0x09u) { return static_cast<uint16_t>(sizeof(g_page9_boost)); }
    if (page == 0x0Au) { return static_cast<uint16_t>(sizeof(g_page10_ltft)); }
    if (page == 0x0Bu) { return static_cast<uint16_t>(sizeof(g_page11_axes)); }
    if (page == 0x0Cu) { return static_cast<uint16_t>(sizeof(g_page12_ltft_accum)); }
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
    if (page == 0x0Bu) { return g_page11_axes; }
    if (page == 0x0Cu) { return g_page12_ltft_accum; }
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

    // WBO2 via CAN (lambda × 1000); ÷5 para caber em uint8_t até λ=1.275
    // (÷4 saturava em 1.020 — o default 1050 exibia 1.02 no dashboard)
    rt.o2_mv_d4 = ems::engine::clamp_u8(ems::app::can_stack_lambda_milli() / 5u);
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
    if (ecu_sched_is_sequential()) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_IGN_SEQUENTIAL);
    }
    if (g_rt_rev_limit_active) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_REV_LIMIT);
    }
    rt.status_bits = status;
    write_u32_le(&rt.reserved[0], g_rt_sched_late_events);
    rt.reserved[4] = g_rt_lambda_target_d4;
    rt.reserved[5] = static_cast<uint8_t>(g_rt_ltft_pct);
    const uint32_t cmp_glitch_cnt = ems::drv::ckp_get_cmp_glitch_count();
    rt.reserved[6] = cmp_glitch_cnt > 255u ? 255u : static_cast<uint8_t>(cmp_glitch_cnt);
    // Gate do sequencial: cmp_confirms (0/1/2). Torna observável no dash porquê a
    // transição wasted→sequencial ocorre ou não (0=CMP ausente/rejeitado, 2=confirmado).
    rt.reserved[7] = c.cmp_confirms;
    rt.reserved[8] = ems::hal::tle8888_fault_bitmap();
    rt.reserved[9] = ems::hal::flex_fuel_valid()
                    ? ems::hal::flex_fuel_ethanol_pct() : 0u;
    write_u32_le(&rt.reserved[10], g_rt_sched_cycle_schedule_drop_count);
    write_u32_le(&rt.reserved[14], g_rt_sched_calibration_clamp_count);
    write_u32_le(&rt.reserved[18], g_rt_seed_loaded_count);
    write_u32_le(&rt.reserved[22], g_rt_seed_confirmed_count);
    write_u32_le(&rt.reserved[26], g_rt_seed_rejected_count);
    const uint8_t inj_mode = ::ecu_sched_is_sequential() ? 2u
                            : ::ecu_sched_presync_inj_mode();
    rt.reserved[30] = static_cast<uint8_t>((inj_mode << 4u) | (g_rt_sync_state_raw & 0x0Fu));
    // reserved[30] bits: [7:4]=inj_mode (0=simultaneous,1=semi_seq,2=sequential),
    //                     [3:0]=sync_state (0=WAIT_GAP..3=LOSS_OF_SYNC)
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
    rt.map_fused_bar_x100 = g_rt_map_fused_bar_x100;
    rt.net_pw_us = g_rt_net_pw_us;

    // Diagnóstico CKP/CMP: bordas cruas + idade da última borda (TIM5 62.5MHz
    // → ms). last_tick==0 = nenhuma borda desde o boot → idade saturada.
    write_u32_le(&rt.ckpcmp_diag[0], ems::drv::g_diag_isr_count);
    write_u32_le(&rt.ckpcmp_diag[4], ems::drv::g_diag_cmp_isr_count);
    write_u32_le(&rt.ckpcmp_diag[8], c.tooth_period_ns);
    const uint32_t now_ticks = ems::hal::tim5_count();
    const auto edge_age_ms = [now_ticks](uint32_t last_tick) -> uint16_t {
        if (last_tick == 0u) { return 65535u; }
        const uint32_t age = (now_ticks - last_tick) / 62500u;  // ticks → ms
        return age > 65535u ? 65535u : static_cast<uint16_t>(age);
    };
    const uint16_t ckp_age = edge_age_ms(ems::drv::g_diag_last_ckp_edge_tick);
    const uint16_t cmp_age = edge_age_ms(ems::drv::g_diag_last_cmp_edge_tick);
    rt.ckpcmp_diag[12] = static_cast<uint8_t>(ckp_age & 0xFFu);
    rt.ckpcmp_diag[13] = static_cast<uint8_t>(ckp_age >> 8u);
    rt.ckpcmp_diag[14] = static_cast<uint8_t>(cmp_age & 0xFFu);
    rt.ckpcmp_diag[15] = static_cast<uint8_t>(cmp_age >> 8u);

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
    g_env_size = 0u;
    g_env_pos = 0u;
    g_env_rx_crc = 0u;
    g_env_crc_pos = 0u;
}

// ── Teste de saídas ('T') ───────────────────────────────────────────────────
// Formato: 'T' + subcmd(1) + arg1(1) + arg2(u16 LE). Resposta: 1 byte ACK,
// excepto STATUS (0x03) → 4 bytes {active, abort_reason, keepalive_s, busy}.
constexpr uint16_t kTestEnterMagic = 0xA55Au;

static void handle_test_cmd() noexcept {
    const uint8_t sub  = g_test_args[0];
    const uint8_t a1   = g_test_args[1];
    const uint16_t a2  = static_cast<uint16_t>(g_test_args[2] |
                         (static_cast<uint16_t>(g_test_args[3]) << 8u));
    bool ok = false;
    switch (sub) {
        case 0x00u:  // EXIT
            ems::engine::output_test_exit();
            ok = true;
            break;
        case 0x01u:  // ENTER (magic contra armamento acidental por ruído)
            ok = (a2 == kTestEnterMagic) && ems::engine::output_test_enter();
            break;
        case 0x02u:  // KEEPALIVE
            ok = ems::engine::output_test_active();
            ems::engine::output_test_keepalive();
            break;
        case 0x03u: {  // STATUS — resposta de 4 bytes, sem ACK
            uint8_t st[4];
            ems::engine::output_test_status(st);
            tx_push_bytes(st, 4u);
            return;
        }
        case 0x10u: ok = ems::engine::output_test_fire_injector(a1, a2); break;
        case 0x11u: ok = ems::engine::output_test_fire_coil(a1, a2); break;
        case 0x20u: ok = ems::engine::output_test_set_pump(a1 != 0u); break;
        case 0x21u: ok = ems::engine::output_test_set_fan(a1 != 0u); break;
        case 0x30u: ok = ems::engine::output_test_set_vvt(a1, a2); break;
        case 0x40u: ok = ems::engine::output_test_set_etb(static_cast<int16_t>(a2)); break;
        case 0x41u: ok = ems::engine::output_test_set_ewg(static_cast<int16_t>(a2)); break;
        default: break;
    }
    tx_push(ok ? kAckOk : kAckErr);
}

inline bool bounds_ok(uint8_t page, uint16_t off, uint16_t len) noexcept {
    const uint16_t psize = page_size(page);
    if (psize == 0u) {
        return false;
    }
    if (off > psize) {
        return false;
    }
    if (len > static_cast<uint16_t>(psize - off)) {
        return false;
    }
    return true;
}

inline bool command_bounds_ok() noexcept {
    return bounds_ok(g_cmd_page, g_cmd_off, g_cmd_len);
}

// Burn de Flash só com motor parado/lento (errata ES0565: erase/program pode
// congelar fetch por ~120 µs — inaceitável durante janela de CKP/scheduler).
inline bool burn_rpm_safe() noexcept {
    return ems::drv::ckp_snapshot().rpm_x10 <= ems::engine::kFlashWriteSafeRpmX10;
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
        // Offsets 80-85: closed-loop enable / LEARN burn / post-start / LTFT min RPM
        g_page0[80] = ems::engine::closed_loop_enable;
        g_page0[81] = ems::engine::ltft_apply_burn_ve;
        std::memcpy(g_page0 + 82, &ems::engine::closed_loop_post_start_s, 2u);
        std::memcpy(g_page0 + 84, &ems::engine::ltft_adapt_min_rpm_x10,   2u);
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
        std::memcpy(g_page0 + 164, &ems::engine::eoi_idle_deg,      2u);
        std::memcpy(g_page0 + 166, &ems::engine::eoi_blend_rpm_lo,  2u);
        std::memcpy(g_page0 + 168, &ems::engine::eoi_blend_rpm_hi,  2u);
        std::memcpy(g_page0 + 170, &ems::engine::mspark_max_rpm_x10, 2u);
        g_page0[172] = ems::engine::mspark_count;
        std::memcpy(g_page0 + 173, &ems::engine::mspark_inter_dwell_ms_x10, 2u);
        // Versão do layout de calibração — gate do boot contra blobs de
        // tabela com dimensão antiga (ver kCalLayoutVersion em table3d.h).
        g_page0[ems::engine::kCalLayoutVersionOffset] = ems::engine::kCalLayoutVersion;
        // LTFT authority / rates (176-183)
        std::memcpy(g_page0 + 176, &ems::engine::ltft_mult_clamp_pct_x10, 2u);
        std::memcpy(g_page0 + 178, &ems::engine::ltft_add_clamp_us,       2u);
        g_page0[180] = ems::engine::ltft_learn_div;
        g_page0[181] = ems::engine::ltft_commit_gain_pct;
        std::memcpy(g_page0 + 182, &ems::engine::ltft_max_step_x10,       2u);
    } else if (page == 0x01u) {
        std::memcpy(g_page1_ve, ems::engine::ve_table, sizeof(g_page1_ve));
    } else if (page == 0x02u) {
        std::memcpy(g_page2_spark, ems::engine::spark_table, sizeof(g_page2_spark));
    } else if (page == 0x04u) {
        std::memcpy(g_page4_lambda, ems::engine::lambda_target_table_x1000, sizeof(g_page4_lambda));
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
        constexpr uint8_t  kN   = ems::engine::kTableAxisSize;
        constexpr uint8_t  kNA  = ems::engine::kLtftAddAxisSize;
        constexpr uint16_t kOffAdd = ems::engine::kTableCells;
        for (uint8_t m = 0u; m < kN; ++m) {
            for (uint8_t r = 0u; r < kN; ++r) {
                g_page10_ltft[m * kN + r] = static_cast<uint8_t>(
                    ems::hal::nvm_read_ltft(r, m));
            }
        }
        for (uint8_t m = 0u; m < kNA; ++m) {
            for (uint8_t r = 0u; r < kNA; ++r) {
                g_page10_ltft[kOffAdd + m * kNA + r] = static_cast<uint8_t>(
                    ems::hal::nvm_read_ltft_add(r, m));
            }
        }
    } else if (page == 0x0Bu) {
        uint16_t rpm[ems::engine::kTableAxisSize];
        uint16_t load[ems::engine::kTableAxisSize];
        ems::engine::table_axes_get(rpm, load);
        constexpr uint16_t kAxisBytes = 2u * ems::engine::kTableAxisSize;
        std::memcpy(g_page11_axes + 0,          rpm,  kAxisBytes);
        std::memcpy(g_page11_axes + kAxisBytes, load, kAxisBytes);
    } else if (page == 0x0Cu) {
        // Read-only: snapshot do acumulador LTFT (hits + mean STFT).
        ems::engine::fuel_ltft_accum_export(
            g_page12_ltft_accum, static_cast<uint16_t>(sizeof(g_page12_ltft_accum)));
    }
}

// Aplica o buffer da página aos globals do engine. Retorna false se a página
// tiver validação própria e o conteúdo for rejeitado (buffer fica incoerente —
// caller deve restaurar com sync_page_from_table()).
inline bool sync_table_from_page(uint8_t page) noexcept {
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
        ems::engine::closed_loop_enable =
            (g_page0[80] != 0u) ? 1u : 0u;
        ems::engine::ltft_apply_burn_ve = (g_page0[81] != 0u) ? 1u : 0u;
        std::memcpy(&ems::engine::closed_loop_post_start_s, g_page0 + 82, 2u);
        std::memcpy(&ems::engine::ltft_adapt_min_rpm_x10,   g_page0 + 84, 2u);
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
        std::memcpy(&ems::engine::eoi_idle_deg,      g_page0 + 164, 2u);
        std::memcpy(&ems::engine::eoi_blend_rpm_lo,  g_page0 + 166, 2u);
        std::memcpy(&ems::engine::eoi_blend_rpm_hi,  g_page0 + 168, 2u);
        // count=0 é válido: desliga multi-spark (calibration.h). Só valores >3
        // (corrupção/página antiga) são rejeitados.
        if (g_page0[172] <= 3u) {
            std::memcpy(&ems::engine::mspark_max_rpm_x10, g_page0 + 170, 2u);
            ems::engine::mspark_count = g_page0[172];
            std::memcpy(&ems::engine::mspark_inter_dwell_ms_x10, g_page0 + 173, 2u);
        }
        // eoi_idle_deg fora de [0,719] seria clampado pelo blend; normaliza aqui
        if (ems::engine::eoi_idle_deg > 719u) { ems::engine::eoi_idle_deg = 719u; }
        // LTFT authority (blob antigo = 0 → defaults de compilação)
        {
            uint16_t mult_c = 0u, add_c = 0u, max_s = 0u;
            std::memcpy(&mult_c, g_page0 + 176, 2u);
            std::memcpy(&add_c,  g_page0 + 178, 2u);
            std::memcpy(&max_s,  g_page0 + 182, 2u);
            if (mult_c != 0u) { ems::engine::ltft_mult_clamp_pct_x10 = mult_c; }
            if (add_c  != 0u) { ems::engine::ltft_add_clamp_us = add_c; }
            if (g_page0[180] != 0u) { ems::engine::ltft_learn_div = g_page0[180]; }
            if (g_page0[181] != 0u) { ems::engine::ltft_commit_gain_pct = g_page0[181]; }
            ems::engine::ltft_max_step_x10 = max_s;  // 0 = sem cap (válido)
        }
        etb_apply_idle_calibration();
    } else if (page == 0x01u) {
        std::memcpy(ems::engine::ve_table, g_page1_ve, sizeof(g_page1_ve));
    } else if (page == 0x02u) {
        std::memcpy(ems::engine::spark_table, g_page2_spark, sizeof(g_page2_spark));
    } else if (page == 0x04u) {
        std::memcpy(ems::engine::lambda_target_table_x1000, g_page4_lambda, sizeof(g_page4_lambda));
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
    } else if (page == 0x0Bu) {
        uint16_t rpm[ems::engine::kTableAxisSize];
        uint16_t load[ems::engine::kTableAxisSize];
        constexpr uint16_t kAxisBytes = 2u * ems::engine::kTableAxisSize;
        std::memcpy(rpm,  g_page11_axes + 0,          kAxisBytes);
        std::memcpy(load, g_page11_axes + kAxisBytes, kAxisBytes);
        return ems::engine::table_axes_set(rpm, load);
    }
    return true;
}

inline uint16_t editable_page_bit(uint8_t page) noexcept {
    if (page == 0x01u) { return 0x01u; }
    if (page == 0x02u) { return 0x02u; }
    if (page == 0x04u) { return 0x04u; }
    if (page == 0x05u) { return 0x08u; }
    if (page == 0x06u) { return 0x10u; }
    if (page == 0x07u) { return 0x20u; }
    if (page == 0x08u) { return 0x40u; }
    if (page == 0x09u) { return 0x80u; }
    if (page == 0x0Bu) { return 0x100u; }
    return 0u;
}

inline void mark_page_dirty(uint8_t page) noexcept {
    g_dirty_page_mask = static_cast<uint16_t>(g_dirty_page_mask | editable_page_bit(page));
}

inline void clear_page_dirty(uint8_t page) noexcept {
    g_dirty_page_mask = static_cast<uint16_t>(g_dirty_page_mask & static_cast<uint16_t>(~editable_page_bit(page)));
}

inline bool burn_page_to_flash(uint8_t page) noexcept {
    if (page == 0x00u) {
        // Serializa g_eng_cfg → g_page0[2-15] e guarda o slot NVM 0 completo.
        ems::engine::cfg::engine_config_serialize(g_page0, 16u);
        g_page0[ems::engine::kCalLayoutVersionOffset] = ems::engine::kCalLayoutVersion;
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
        const bool ok = ems::hal::nvm_save_calibration(3u, g_page4_lambda, sizeof(g_page4_lambda));
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
    if (page == 0x0Bu) {
        sync_page_from_table(0x0Bu);  // serializa eixos atuais → buffer
        const bool ok = ems::hal::nvm_save_calibration(9u, g_page11_axes, static_cast<uint16_t>(sizeof(g_page11_axes)));
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

    if (!sync_table_from_page(g_cmd_page)) {
        // Conteúdo rejeitado (ex.: eixos não monotónicos) — restaura o buffer
        // a partir dos globals para não servir dados incoerentes num 'r'.
        sync_page_from_table(g_cmd_page);
        tx_push(kAckErr);
        reset_parser();
        return;
    }
    mark_page_dirty(g_cmd_page);
    if (!g_write_ram_only) {
        if (!burn_rpm_safe() || !burn_page_to_flash(g_cmd_page)) {
            tx_push(kAckErr);
            reset_parser();
            return;
        }
    }
    tx_push(kAckOk);
    reset_parser();
}

// ── Envelope TS: resposta e dispatcher ───────────────────────────────────────

inline uint16_t tx_free() noexcept {
    return static_cast<uint16_t>((kTxSize - 1u) -
                                 ((g_tx_head - g_tx_tail) & kTxMask));
}

static void env_send_response(uint8_t code, const uint8_t* data, uint16_t len) noexcept {
    const uint16_t psize = static_cast<uint16_t>(len + 1u);
    // Sem espaço para o frame inteiro → não envia nada (frame parcial seria
    // corrupção; o TunerStudio faz timeout e re-tenta).
    if (tx_free() < static_cast<uint16_t>(psize + 6u)) {
        return;
    }
    tx_push(static_cast<uint8_t>(psize >> 8u));
    tx_push(static_cast<uint8_t>(psize & 0xFFu));
    uint32_t crc = 0xFFFFFFFFu;
    crc = ems::hal::crc32_update(crc, code);
    tx_push(code);
    for (uint16_t i = 0u; i < len; ++i) {
        crc = ems::hal::crc32_update(crc, data[i]);
        tx_push(data[i]);
    }
    crc = ~crc;
    tx_push(static_cast<uint8_t>(crc >> 24u));
    tx_push(static_cast<uint8_t>(crc >> 16u));
    tx_push(static_cast<uint8_t>(crc >> 8u));
    tx_push(static_cast<uint8_t>(crc & 0xFFu));
}

// Chunk write (RAM-only; burn é explícito via 'b'). `a` aponta para
// page + off(u16 LE) + len(u16 LE) + dados; data_n = bytes de dados no frame.
// Retorna código TS, ou -1 se o header declarado não corresponder a data_n
// (permite ao caller tentar a forma com canId à frente).
static int env_try_write(const uint8_t* a, uint16_t data_n) noexcept {
    const uint8_t page = normalize_page_id(a[0]);
    const uint16_t off = static_cast<uint16_t>(a[1] | (static_cast<uint16_t>(a[2]) << 8u));
    const uint16_t len = static_cast<uint16_t>(a[3] | (static_cast<uint16_t>(a[4]) << 8u));
    if (len != data_n) {
        return -1;
    }
    if (!bounds_ok(page, off, len) || page == 0x03u || len > kEnvMaxChunk) {
        return kTsRcRangeErr;
    }
    uint8_t* ptr = page_ptr(page);
    if (ptr == nullptr) {
        return kTsRcRangeErr;
    }
    if (len != 0u) {
        std::memcpy(ptr + off, a + 5u, len);
        if (!sync_table_from_page(page)) {
            sync_page_from_table(page);
            return kTsRcRangeErr;
        }
        mark_page_dirty(page);
    }
    return kTsRcOk;
}

static void env_dispatch(const uint8_t* p, uint16_t n) noexcept {
    const uint8_t cmd = p[0];

    if (cmd == static_cast<uint8_t>('Q') || cmd == static_cast<uint8_t>('H')) {
        env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kSignature),
                          static_cast<uint16_t>(sizeof(kSignature) - 1u));
        return;
    }
    if (cmd == static_cast<uint8_t>('S')) {
        env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kFwVersion),
                          static_cast<uint16_t>(sizeof(kFwVersion) - 1u));
        return;
    }
    if (cmd == static_cast<uint8_t>('F')) {
        env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kProtocolVersion),
                          static_cast<uint16_t>(sizeof(kProtocolVersion) - 1u));
        return;
    }
    if (cmd == static_cast<uint8_t>('C')) {
        const uint8_t magic = kCommsTestMagic;
        env_send_response(kTsRcOk, &magic, 1u);
        return;
    }
    if (cmd == static_cast<uint8_t>('A') || cmd == static_cast<uint8_t>('O')) {
        update_realtime_page();
        env_send_response(kTsRcOk, g_page3_rt,
                          static_cast<uint16_t>(sizeof(g_page3_rt)));
        return;
    }
    if (cmd == static_cast<uint8_t>('d')) {
        const uint8_t mask[2] = {
            static_cast<uint8_t>(g_dirty_page_mask & 0xFFu),
            static_cast<uint8_t>(g_dirty_page_mask >> 8u),
        };
        env_send_response(kTsRcOk, mask, 2u);
        return;
    }
    if (cmd == static_cast<uint8_t>('r')) {
        // 'r' [canId] page off(u16 LE) len(u16 LE) → 6 ou 7 bytes no payload
        if (n != 6u && n != 7u) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        const uint8_t* a = (n == 7u) ? (p + 2u) : (p + 1u);
        const uint8_t page = normalize_page_id(a[0]);
        const uint16_t off = static_cast<uint16_t>(a[1] | (static_cast<uint16_t>(a[2]) << 8u));
        const uint16_t len = static_cast<uint16_t>(a[3] | (static_cast<uint16_t>(a[4]) << 8u));
        // page=0x0F: pseudo-página "signature" — convenção real do TunerStudio/
        // Speeduino (comms.cpp: "cmd == 0x0f → Request for signature"), usada
        // pelo Comm Manager para validar o controlador após a conexão real
        // (distinta do probe leve 'Q' cru da fase de deteção/wizard).
        if (page == 0x0Fu) {
            env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kSignature),
                              static_cast<uint16_t>(sizeof(kSignature) - 1u));
            return;
        }
        if (!bounds_ok(page, off, len) || len > kEnvMaxChunk) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        if (page == 0x03u) {
            update_realtime_page();
        } else {
            sync_page_from_table(page);
        }
        const uint8_t* ptr = page_ptr(page);
        if (ptr == nullptr) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        env_send_response(kTsRcOk, ptr + off, len);
        return;
    }
    if (cmd == static_cast<uint8_t>('w') || cmd == static_cast<uint8_t>('x')) {
        int rc = (n >= 6u) ? env_try_write(p + 1u, static_cast<uint16_t>(n - 6u)) : -1;
        if (rc < 0 && n >= 7u) {
            rc = env_try_write(p + 2u, static_cast<uint16_t>(n - 7u));  // forma com canId
        }
        env_send_response((rc < 0) ? kTsRcRangeErr : static_cast<uint8_t>(rc), nullptr, 0u);
        return;
    }
    if (cmd == static_cast<uint8_t>('b')) {
        // 'b' [canId] page → 2 ou 3 bytes no payload
        if (n != 2u && n != 3u) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        const uint8_t page = normalize_page_id(p[n - 1u]);
        if (!burn_rpm_safe()) {
            env_send_response(kTsRcBusyErr, nullptr, 0u);
            return;
        }
        env_send_response(burn_page_to_flash(page) ? kTsRcOk : kTsRcRangeErr,
                          nullptr, 0u);
        return;
    }
    env_send_response(kTsRcUnknown, nullptr, 0u);
}

inline void parse_byte(uint8_t b) noexcept {
    if (g_state == ParseState::IDLE) {
        // Ignore line-state reset probe bytes used by some host stacks.
        if (b == 0xF0u) {
            return;
        }
        // Auto-detect envelope TS: comandos legacy são ASCII ≥ 0x20; um frame
        // envelope começa pelo byte alto do size BE (0x00-0x01 para ≤ 263 B).
        if (b < 0x20u) {
            g_env_size = static_cast<uint16_t>(static_cast<uint16_t>(b) << 8u);
            g_state = ParseState::ENV_SIZE_LO;
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
            // Legacy: só o byte baixo (páginas 1-9); página 11 (bit 8) é
            // reportada apenas no 'd' do envelope, que devolve os 16 bits.
            tx_push(static_cast<uint8_t>(g_dirty_page_mask & 0xFFu));
            return;
        }
        if (b == static_cast<uint8_t>('B')) {
            g_state = ParseState::BENCH_ARG;
            return;
        }
        if (b == static_cast<uint8_t>('Z')) {
            // LEARN session reset: STFT + accum + LTFT NVM-shadow (dirty) + dbg.
            // Não burn de page0/VE. Sector adaptativo pode flushar LTFT zeros.
            ems::engine::fuel_reset_learn_session();
            tx_push(kAckOk);
            return;
        }
        if (b == static_cast<uint8_t>('Y')) {
            // Apply manual: bake-in de todas as células LEARN ready na VE (RAM).
            // Resposta: [ACK][n_commits u8] — n satura em 255.
            const uint16_t n = ems::engine::fuel_ltft_accum_apply_all_ready();
            tx_push(kAckOk);
            tx_push(static_cast<uint8_t>((n > 255u) ? 255u : n));
            return;
        }
        if (b == static_cast<uint8_t>('T')) {
            g_state = ParseState::TEST_ARGS;
            g_arg_pos = 0u;
            return;
        }
        if (b == static_cast<uint8_t>('K')) {
            // Osciloscópio CKP/CMP: [ckp_idx][cmp_idx][cmp_ref_tooth]
            // + 64×u32 LE (ring CKP) + 8×u32 LE (ring CMP)
            // + âncora angular: [tooth_index u8][phase_A u8][sync_state u8]
            //   do snapshot no instante do dump — a borda CKP mais recente
            //   corresponde a tooth_index (±1 dente), permitindo ao host
            //   propagar o ângulo 0-720° borda-a-borda. Total = 294 bytes.
            // Leitura dos rings sem critical section: u32 alinhado é atômico
            // no M33; tearing entre elementos é aceitável para visualização.
            tx_push(ems::drv::g_scope_ckp_idx);
            tx_push(ems::drv::g_scope_cmp_idx);
            tx_push(ems::drv::ckp_get_cmp_ref_tooth());
            uint8_t tmp[4];
            for (uint8_t i = 0u; i < 64u; ++i) {
                write_u32_le(tmp, ems::drv::g_scope_ckp_ts[i]);
                tx_push_bytes(tmp, 4u);
            }
            for (uint8_t i = 0u; i < 8u; ++i) {
                write_u32_le(tmp, ems::drv::g_scope_cmp_ts[i]);
                tx_push_bytes(tmp, 4u);
            }
            const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
            tx_push(static_cast<uint8_t>(snap.tooth_index > 57u ? 57u
                                         : snap.tooth_index));
            tx_push(snap.phase_A ? 1u : 0u);
            tx_push(static_cast<uint8_t>(snap.state));
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
            // 37×u32 = 148 B (era 33×u32=132; +4 campos auto-learn/accum)
            const uint32_t diag[37] = {
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
                ems::drv::g_dbg_gap_accepted,
                ems::drv::g_dbg_gap_premature,
                ems::drv::g_dbg_gap_last_tc,
                ems::drv::g_dbg_loss_missing_gap,
                ems::drv::g_dbg_loss_stall,
                ems::drv::g_dbg_loss_avg,
                ems::drv::g_dbg_loss_delta,
                ems::engine::g_dbg_stft_blocked_clt,
                ems::engine::g_dbg_stft_blocked_o2,
                ems::engine::g_dbg_stft_blocked_ae,
                ems::engine::g_dbg_stft_blocked_cut,
                ems::engine::g_dbg_stft_runs,
                static_cast<uint32_t>(ems::engine::g_dbg_stft_last_err),
                static_cast<uint32_t>(ems::engine::g_stft_integrator_x1000),
                ems::engine::g_dbg_ltft_accum_accepted,
                ems::engine::g_dbg_ltft_accum_rejected,
                ems::engine::g_dbg_ltft_accum_commits,
                // flags: b8-15 burn_ve, b16 burn_pending (b0-7 pad reserved=0)
                (static_cast<uint32_t>(ems::engine::ltft_apply_burn_ve) << 8) |
                    (ems::engine::fuel_ltft_ve_burn_pending() ? (1u << 16) : 0u),
            };
            tx_push_bytes(reinterpret_cast<const uint8_t*>(diag), sizeof(diag));
            return;
        }
        return;
    }

    if (g_state == ParseState::ENV_SIZE_LO) {
        g_env_size = static_cast<uint16_t>(g_env_size | b);
        if (g_env_size == 0u || g_env_size > kEnvMaxPayload) {
            reset_parser();
            return;
        }
        g_env_pos = 0u;
        g_state = ParseState::ENV_PAYLOAD;
        return;
    }

    if (g_state == ParseState::ENV_PAYLOAD) {
        g_env_buf[g_env_pos] = b;
        ++g_env_pos;
        if (g_env_pos >= g_env_size) {
            g_env_rx_crc = 0u;
            g_env_crc_pos = 0u;
            g_state = ParseState::ENV_CRC;
        }
        return;
    }

    if (g_state == ParseState::ENV_CRC) {
        g_env_rx_crc = (g_env_rx_crc << 8u) | b;
        ++g_env_crc_pos;
        if (g_env_crc_pos >= 4u) {
            if (ems::hal::crc32_calc(g_env_buf, g_env_size) == g_env_rx_crc) {
                env_dispatch(g_env_buf, g_env_size);
            } else {
                env_send_response(kTsRcCrcErr, nullptr, 0u);
            }
            reset_parser();
        }
        return;
    }

    if (g_state == ParseState::TEST_ARGS) {
        g_test_args[g_arg_pos] = b;
        ++g_arg_pos;
        if (g_arg_pos < 4u) { return; }
        handle_test_cmd();
        reset_parser();
        return;
    }

    if (g_state == ParseState::BENCH_ARG) {
        // Bench-mode CLT/IAT p/ HIL: 0=off (ADC normal), !=0=on (90°C/25°C fixos,
        // sem SENSOR_FAULT pelos canais CLT/IAT). Ver sensors_set_bench_clt_iat.
        ems::drv::sensors_set_bench_clt_iat(b != 0u, 900, 250);
        // Lambda simulado λ=1.000: liberta o closed-loop (STFT/LTFT) sem WBO2
        // físico — com λ medido fixo, o trim caminha até o alvo da tabela
        // (exercita integrador e aprendizagem; não converge, por design).
        ems::app::can_stack_set_bench_lambda(b != 0u, 1000u);
        tx_push(kAckOk);
        reset_parser();
        return;
    }

    if (g_state == ParseState::BURN_ARGS) {
        g_cmd_page = normalize_page_id(b);
        if (burn_rpm_safe() && burn_page_to_flash(g_cmd_page)) {
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
    sync_page_from_table(0x0Bu);
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
    // Auto-learn: flush VE → flash se pedido e RPM seguro (nunca em alta rotação).
    if (ems::engine::fuel_ltft_ve_burn_pending() &&
        ems::engine::ltft_apply_burn_ve != 0u &&
        burn_rpm_safe()) {
        sync_page_from_table(0x01u);
        if (burn_page_to_flash(0x01u)) {
            ems::engine::fuel_ltft_ve_burn_clear();
        }
    }

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

void ui_set_rev_limit_active(bool active) noexcept {
    g_rt_rev_limit_active = active;
}

void ui_update_loop_diag(uint32_t loop2ms_last_us,
                         uint32_t loop2ms_max_us) noexcept {
    g_rt_loop2ms_last_us = loop2ms_last_us;
    g_rt_loop2ms_max_us = loop2ms_max_us;
}

void ui_update_ivc_diag(uint32_t ivc_clamp_count) noexcept {
    g_rt_ivc_clamp_count = ivc_clamp_count;
}

void ui_update_rt_map_fuel(uint16_t map_fused_bar_x100, uint32_t net_pw_us) noexcept {
    g_rt_map_fused_bar_x100 = map_fused_bar_x100;
    g_rt_net_pw_us = net_pw_us > 65535u ? 65535u : static_cast<uint16_t>(net_pw_us);
}

#if defined(EMS_HOST_TEST)
void ui_test_reset() noexcept {
    ui_init();
}
#endif

}  // namespace ems::app
