#pragma once

#include <cstdint>

namespace ems::app {

struct UiRealtimeData {
    uint16_t rpm;
    uint8_t map_bar_x100;
    uint8_t tps_pct;
    int8_t clt_p40;
    int8_t iat_p40;
    uint8_t o2_mv_d4;
    uint8_t pw1_ms_x10;
    uint8_t advance_p40;
    uint8_t ve;
    int8_t stft_p100;
    uint16_t status_bits;
    uint8_t reserved[52];
    // MAP fundido (sensor+modelo, bar×100) e PW de fluxo líquido (µs, pré-dead-time/
    // pré-xtau/pré-ΔP/pré-S-curve) latchados no mesmo tick de 2ms que alimenta o
    // cálculo de combustível — diagnóstico do descasamento entre o MAP bruto exibido
    // (reamostrado a cada poll) e o valor que de facto gerou o PW.
    uint16_t map_fused_bar_x100;
    uint16_t net_pw_us;
    // Diagnóstico de sensores CKP/CMP: bordas cruas acumuladas (pré-filtro —
    // ruído conta), período de dente corrente e idade da última borda. O host
    // deriva a taxa de bordas (Hz) do delta entre polls; idade saturada em
    // 65535ms; 65535 também quando nunca houve borda desde o boot.
    // Bloco de bytes (LE) para evitar padding — offset 70 não é 4-alinhado:
    //   [0..3]=ckp_edge_count u32, [4..7]=cmp_edge_count u32,
    //   [8..11]=tooth_period_ns u32, [12..13]=ckp_edge_age_ms u16,
    //   [14..15]=cmp_edge_age_ms u16
    uint8_t ckpcmp_diag[16];
};

static_assert(sizeof(UiRealtimeData) == 86u, "UiRealtimeData must be 86 bytes");

void ui_init() noexcept;
void ui_rx_byte(uint8_t byte) noexcept;
void ui_uart0_rx_isr_byte(uint8_t byte) noexcept;  // compat wrapper
void ui_process() noexcept;
void ui_update_rt_metrics(uint8_t pw_ms_x10, int8_t advance_deg, int8_t stft_p100,
                          uint8_t lambda_target_d4 = 0u, int8_t ltft_pct = 0) noexcept;
void ui_update_rt_sched_diag(uint32_t late_events,
                             uint32_t cycle_schedule_drop_count,
                             uint32_t calibration_clamp_count,
                             uint32_t seed_loaded_count,
                             uint32_t seed_confirmed_count,
                             uint32_t seed_rejected_count,
                             uint8_t sync_state_raw) noexcept;

void ui_update_loop_diag(uint32_t loop2ms_last_us,
                         uint32_t loop2ms_max_us) noexcept;
void ui_set_rev_limit_active(bool active) noexcept;

/// Atualiza o contador de clamp IVC para o bloco de dados em tempo real.
/// Chamado do loop de fundo (20 ms) junto com ui_update_rt_sched_diag.
void ui_update_ivc_diag(uint32_t ivc_clamp_count) noexcept;

/// Latch do MAP fundido (bar×100) e PW de fluxo líquido (µs) do tick de 2ms de
/// cálculo de combustível — chamado a cada iteração, independente de sync.
void ui_update_rt_map_fuel(uint16_t map_fused_bar_x100, uint32_t net_pw_us) noexcept;

bool ui_tx_pop(uint8_t& byte) noexcept;
uint16_t ui_tx_available() noexcept;

#if defined(EMS_HOST_TEST)
void ui_test_reset() noexcept;
#endif

}  // namespace ems::app
