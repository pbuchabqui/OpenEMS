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

void parse_byte(uint8_t b) noexcept {
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
            // Angle measurement via sched API (no raw symbol peeks).
            // Format: [gap_ts:4] [idx:1] [8×{ts:4, high:1, ch:1}] = 45 bytes
            uint32_t gap = 0U;
            uint8_t ridx = 0U;
            EcuSchedTsSample samples[8];
            ecu_sched_get_angle_trace(&gap, &ridx, samples);
            tx_push_bytes(reinterpret_cast<const uint8_t*>(&gap), 4U);
            tx_push(ridx);
            for (uint8_t i = 0; i < 8U; ++i) {
                tx_push_bytes(reinterpret_cast<const uint8_t*>(&samples[i].ts), 4U);
                tx_push(samples[i].high);
                tx_push(samples[i].channel);
            }
            return;
        }
        if (b == static_cast<uint8_t>('P')) {
            ecu_sched_bench_pw_lock_next_commit();
            ecu_sched_commit_calibration(10U, 22500U, 50000U, 30U);  // eoi_lead=30° (EOI targeting, valor de bench)
            tx_push(0x00u);
            return;
        }
        if (b == static_cast<uint8_t>('V')) {
            uint32_t v[24];
            ecu_sched_get_pin_counts_u32x24(v);
            tx_push_bytes(reinterpret_cast<const uint8_t*>(v), 96U);
            return;
        }
        if (b == static_cast<uint8_t>('D')) {
            EcuSchedDiagSnapshot sd{};
            ecu_sched_get_diag_snapshot(&sd);
            // 37×u32 = 148 B (era 33×u32=132; +4 campos auto-learn/accum)
            const uint32_t diag[37] = {
                sd.late_event_count,
                sd.cycle_schedule_drop_count,
                sd.inj1_arm,
                sd.seq_calls,
                sd.evt_overflow,
                sd.clear_all_count,
                sd.presync_count,
                sd.dwell_watchdog_count,
                ems::drv::g_diag_isr_count,
                ems::drv::g_dbg_tc_gap,
                ems::drv::g_dbg_tc_spike,
                ems::drv::g_dbg_tc_normal,
                sd.phase_skip,
                sd.phase_fire,
                sd.evt_inserted,
                sd.evt_dispatched,
                sd.diag_presync_revs,
                sd.diag_seq_revs,
                sd.diag_clear_all_count,
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

void reset_pages() noexcept {
    std::memset(g_page0, 0, sizeof(g_page0));
    // page0[0] reserved (IVC removed)
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

}  // namespace ems::app::ui_detail

namespace ems::app {

using namespace ems::app::ui_detail;


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
