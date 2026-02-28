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

bool ts_tx_pop(uint8_t& byte) noexcept;
uint16_t ts_tx_available() noexcept;

#if defined(EMS_HOST_TEST)
void ts_test_reset() noexcept;
#endif

}  // namespace ems::app
