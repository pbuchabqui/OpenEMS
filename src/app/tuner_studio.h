#pragma once

#include <cstdint>

namespace ems::app {

struct TsRealtimeData {
    uint16_t rpm;
    uint8_t map_kpa;
    uint8_t tps_pct;
    int8_t clt_p40;
    int8_t iat_p40;
    uint8_t o2_mv_d4;
    uint8_t pw1_ms_x10;
    uint8_t advance_p40;
    uint8_t ve;
    int8_t stft_p100;
    uint8_t status_bits;
    uint8_t reserved[52];
};

static_assert(sizeof(TsRealtimeData) == 64u, "TsRealtimeData must be 64 bytes");

void ts_init() noexcept;
void ts_uart0_rx_isr_byte(uint8_t byte) noexcept;
void ts_process() noexcept;
void ts_update_rt_metrics(uint8_t pw_ms_x10, int8_t advance_deg, int8_t stft_p100) noexcept;
void ts_update_rt_sched_diag(uint32_t late_events,
                             uint32_t late_max_delay_ticks,
                             uint8_t queue_depth_peak,
                             uint8_t queue_depth_last_cycle_peak,
                             uint32_t cycle_schedule_drop_count,
                             uint32_t calibration_clamp_count,
                             uint32_t seed_loaded_count,
                             uint32_t seed_confirmed_count,
                             uint32_t seed_rejected_count,
                             uint8_t sync_state_raw) noexcept;

/// Atualiza o contador de clamp IVC para o bloco de dados em tempo real.
/// Chamado do loop de fundo (20 ms) junto com ts_update_rt_sched_diag.
void ts_update_ivc_diag(uint32_t ivc_clamp_count) noexcept;

bool ts_tx_pop(uint8_t& byte) noexcept;
uint16_t ts_tx_available() noexcept;

#if defined(EMS_HOST_TEST)
void ts_test_reset() noexcept;
#endif

}  // namespace ems::app
