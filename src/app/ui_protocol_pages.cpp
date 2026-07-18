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

void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

uint16_t page_size(uint8_t page) noexcept {
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

uint8_t* page_ptr(uint8_t page) noexcept {
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

uint8_t normalize_page_id(uint8_t page) noexcept {
    if (page >= static_cast<uint8_t>('0') && page <= static_cast<uint8_t>('9')) {
        return static_cast<uint8_t>(page - static_cast<uint8_t>('0'));
    }
    return page;
}

bool tx_push(uint8_t byte) noexcept {
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

void tx_push_bytes(const uint8_t* ptr, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (!tx_push(ptr[i])) {
            return;
        }
    }
}

bool rx_pop(uint8_t& byte) noexcept {
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

void write_u32_le(uint8_t* dst, uint32_t v) noexcept {
    dst[0] = static_cast<uint8_t>(v & 0xFFu);
    dst[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
    dst[2] = static_cast<uint8_t>((v >> 16u) & 0xFFu);
    dst[3] = static_cast<uint8_t>((v >> 24u) & 0xFFu);
}

void update_realtime_page() noexcept {
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
    // Launch / TC from torque_manager latches (updated every ETB 2 ms tick).
    const uint8_t launch_act = ems::engine::torque_manager_get_launch_active();
    const uint16_t tc_red = ems::engine::torque_manager_get_tc_reduction();
    const int16_t spark_ret = ems::engine::torque_manager_get_spark_retard();
    if (launch_act != 0u) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_LAUNCH_ACTIVE);
    }
    if (tc_red > 0u) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_TC_ACTIVE);
    }
    if (ems::drv::sensors_is_bench_mode()) {
        status = static_cast<uint16_t>(status | ems::app::STATUS_BENCH_MODE);
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
    // reserved[31..34]: was IVC clamp (always 0) — reused for torque RT (layout stable size):
    //   [31..32] tc_reduction_pct_x10 (u16 LE, 0–1000)
    //   [33]     spark_retard_deg (u8, 0–30)
    //   [34]     sensors fault_bits (bitmask SensorId 0..7)
    rt.reserved[31] = static_cast<uint8_t>(tc_red & 0xFFu);
    rt.reserved[32] = static_cast<uint8_t>((tc_red >> 8u) & 0xFFu);
    {
        int16_t sr = spark_ret;
        if (sr < 0) { sr = 0; }
        if (sr > 30) { sr = 30; }
        rt.reserved[33] = static_cast<uint8_t>(sr);
    }
    rt.reserved[34] = s.fault_bits;
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

void reset_parser() noexcept {
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

void handle_test_cmd() noexcept {
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

bool bounds_ok(uint8_t page, uint16_t off, uint16_t len) noexcept {
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

bool command_bounds_ok() noexcept {
    return bounds_ok(g_cmd_page, g_cmd_off, g_cmd_len);
}

// Burn de Flash só com motor parado/lento (errata ES0565: erase/program pode
// congelar fetch por ~120 µs — inaceitável durante janela de CKP/scheduler).
bool burn_rpm_safe() noexcept {
    return ems::drv::ckp_snapshot().rpm_x10 <= ems::engine::kFlashWriteSafeRpmX10;
}

void sync_page_from_table(uint8_t page) noexcept {
    if (page == 0x00u) {
        // Popula bytes 2-15 do buffer UI a partir de g_eng_cfg.
        // Byte 0: reserved (was IVC ABDC; unused after EOI targeting).
        ems::engine::cfg::engine_config_serialize(g_page0, 16u);
        g_page0[0] = 0u;
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
        g_page0[184] = ems::engine::ltft_adapt_enable;
        // LEARN thresholds 185-190
        std::memcpy(g_page0 + 185, &ems::engine::ltft_learn_ready_hits, 2u);
        g_page0[187] = ems::engine::ltft_learn_max_err_x1000;
        g_page0[188] = ems::engine::ltft_learn_ready_max_mean_err;
        g_page0[189] = ems::engine::ltft_learn_ready_min_stft_x10;
        g_page0[190] = ems::engine::ltft_learn_ready_max_stft_x10;
        // Launch + TC (191-215, layout v5)
        ems::engine::launch_tc_serialize_to_page0(g_page0, sizeof(g_page0));
        // CAN RX map: gear / vehicle speed / driven wheel (216-245)
        ems::app::can_rx_map_serialize_to_page0(g_page0, sizeof(g_page0));
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
bool sync_table_from_page(uint8_t page) noexcept {
    if (page == 0x00u) {
        // page0[0] reserved (IVC removed from wire); ignore host writes to that byte.
        g_page0[0] = 0u;
        // Aplica engine config (displacement, injector, AFR, trigger offset, etc.)
        // engine_config_load valida magic 0x4543 em bytes [14-15] — a escrita via
        // 'w' deve sempre incluir os 16 bytes completos com magic correcto.
        ems::engine::cfg::engine_config_load(g_page0, 16u);
        ems::engine::map_estimator_sync_engine_config();
        // Calibração de sensores (bytes 16-55) → globals + drivers
        ems::engine::apply_etb_calibration_from_page(g_page0 + 16, 40u);
        ems::engine::push_sensor_calibration_to_drivers();
        // Trim por cilindro e janela CMP (bytes 56-65)
        std::memcpy(ems::engine::cyl_fuel_trim_pct, g_page0 + 56, 4u);
        std::memcpy(ems::engine::cyl_ign_trim_deg,  g_page0 + 60, 4u);
        for (uint8_t i = 0u; i < 4u; ++i) {
            int8_t& ft = ems::engine::cyl_fuel_trim_pct[i];
            if (ft > 50) { ft = 50; } else if (ft < -50) { ft = -50; }
            int8_t& it = ems::engine::cyl_ign_trim_deg[i];
            if (it > 15) { it = 15; } else if (it < -15) { it = -15; }
        }
        ems::engine::cmp_window_open_tooth  = g_page0[64];
        ems::engine::cmp_window_close_tooth = g_page0[65];
        if (ems::engine::cmp_window_open_tooth > 57u) {
            ems::engine::cmp_window_open_tooth = 57u;
        }
        if (ems::engine::cmp_window_close_tooth > 57u) {
            ems::engine::cmp_window_close_tooth = 57u;
        }
        // Dirigibilidade (bytes 66-99)
        std::memcpy(&ems::engine::antijerk_tpsdot_threshold_x10, g_page0 + 66, 2u);
        std::memcpy(&ems::engine::antijerk_retard_deg,            g_page0 + 68, 2u);
        ems::engine::antijerk_decay_cycles = g_page0[70];
        std::memcpy(&ems::engine::rev_limit_rpm_x10,          g_page0 + 72, 4u);
        std::memcpy(&ems::engine::rev_limit_soft_window_x10,  g_page0 + 76, 4u);
        // Safety clamps: corrupt/host typos must not disable the rev limiter
        // (0) or push it past physical range. Defaults match calibration.cpp.
        if (ems::engine::rev_limit_rpm_x10 < 10000u) {
            ems::engine::rev_limit_rpm_x10 = 10000u;   // 1000 RPM floor
        } else if (ems::engine::rev_limit_rpm_x10 > 120000u) {
            ems::engine::rev_limit_rpm_x10 = 120000u;  // 12000 RPM ceiling
        }
        if (ems::engine::rev_limit_soft_window_x10 > ems::engine::rev_limit_rpm_x10) {
            ems::engine::rev_limit_soft_window_x10 = ems::engine::rev_limit_rpm_x10 / 2u;
        }
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
        // Decel cut: entry must be ≥ exit (hysteresis). Swap if inverted.
        if (ems::engine::decel_cut_entry_rpm_x10 < ems::engine::decel_cut_exit_rpm_x10) {
            const uint32_t tmp = ems::engine::decel_cut_entry_rpm_x10;
            ems::engine::decel_cut_entry_rpm_x10 = ems::engine::decel_cut_exit_rpm_x10;
            ems::engine::decel_cut_exit_rpm_x10 = tmp;
        }
        if (ems::engine::decel_cut_tps_threshold_x10 > 200u) {
            ems::engine::decel_cut_tps_threshold_x10 = 200u;  // 20% max
        }
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
        // STFT clamp as uint16 cast to int16 for PI — keep 1..500 (±0.1..50%).
        if (ems::engine::stft_clamp_pct_x10 == 0u) {
            ems::engine::stft_clamp_pct_x10 = 250u;  // default 25%
        } else if (ems::engine::stft_clamp_pct_x10 > 500u) {
            ems::engine::stft_clamp_pct_x10 = 500u;
        }
        if (ems::engine::stft_kp_x100 > 1000u) { ems::engine::stft_kp_x100 = 1000u; }
        if (ems::engine::stft_ki_x1000 > 1000u) { ems::engine::stft_ki_x1000 = 1000u; }
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
            uint16_t ms_rpm = 0u;
            std::memcpy(&ms_rpm, g_page0 + 170, 2u);
            if (ms_rpm == 0u) {
                ms_rpm = ems::engine::kMsparkRpmCeilingX10;
            } else if (ms_rpm > ems::engine::kMsparkRpmCeilingX10) {
                ms_rpm = ems::engine::kMsparkRpmCeilingX10;  // hard max 1500 RPM
            }
            ems::engine::mspark_max_rpm_x10 = ms_rpm;
            ems::engine::mspark_count = g_page0[172];
            std::memcpy(&ems::engine::mspark_inter_dwell_ms_x10, g_page0 + 173, 2u);
        }
        // eoi_idle_deg fora de [0,719] seria clampado pelo blend; normaliza aqui
        if (ems::engine::eoi_idle_deg > 719u) { ems::engine::eoi_idle_deg = 719u; }
        // LTFT authority (176-184): só layout v3+
        if (g_page0[ems::engine::kCalLayoutVersionOffset] ==
            ems::engine::kCalLayoutVersion) {
            uint16_t mult_c = 0u, add_c = 0u, max_s = 0u;
            std::memcpy(&mult_c, g_page0 + 176, 2u);
            std::memcpy(&add_c,  g_page0 + 178, 2u);
            std::memcpy(&max_s,  g_page0 + 182, 2u);
            if (mult_c != 0u) { ems::engine::ltft_mult_clamp_pct_x10 = mult_c; }
            if (add_c  != 0u) { ems::engine::ltft_add_clamp_us = add_c; }
            if (g_page0[180] != 0u) { ems::engine::ltft_learn_div = g_page0[180]; }
            if (g_page0[181] != 0u) { ems::engine::ltft_commit_gain_pct = g_page0[181]; }
            ems::engine::ltft_max_step_x10 = max_s;  // 0 = sem cap (válido)
            if (g_page0[184] <= 1u) {
                ems::engine::ltft_adapt_enable = g_page0[184];
            }
            {
                uint16_t hits = 0u;
                std::memcpy(&hits, g_page0 + 185, 2u);
                if (hits != 0u) {
                    ems::engine::ltft_learn_ready_hits = hits;
                }
                if (g_page0[187] != 0u) {
                    ems::engine::ltft_learn_max_err_x1000 = g_page0[187];
                }
                if (g_page0[188] != 0u) {
                    ems::engine::ltft_learn_ready_max_mean_err = g_page0[188];
                }
                if (g_page0[189] != 0u) {
                    ems::engine::ltft_learn_ready_min_stft_x10 = g_page0[189];
                }
                if (g_page0[190] != 0u) {
                    ems::engine::ltft_learn_ready_max_stft_x10 = g_page0[190];
                }
            }
            // Launch + TC (191-215): only layout v5+ — older blobs are zeros/garbage.
            ems::engine::launch_tc_apply_from_page0(g_page0, sizeof(g_page0));
            // CAN RX map 216-245 (id=0 disables each signal — safe on blank flash)
            ems::app::can_rx_map_apply_from_page0(g_page0, sizeof(g_page0));
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

uint16_t editable_page_bit(uint8_t page) noexcept {
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

void mark_page_dirty(uint8_t page) noexcept {
    g_dirty_page_mask = static_cast<uint16_t>(g_dirty_page_mask | editable_page_bit(page));
}

void clear_page_dirty(uint8_t page) noexcept {
    g_dirty_page_mask = static_cast<uint16_t>(g_dirty_page_mask & static_cast<uint16_t>(~editable_page_bit(page)));
}

bool burn_page_to_flash(uint8_t page) noexcept {
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

void handle_read_done() noexcept {
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

void handle_write_done() noexcept {
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


}  // namespace ems::app::ui_detail
