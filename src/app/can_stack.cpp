#include "app/can_stack.h"

#include <cstdint>

#include "hal/can.h"

namespace {

static uint16_t g_wbo2_rx_id      = 0x180u;
static uint16_t g_lambda_milli    = ems::app::WBO2_SAFE_LAMBDA_MILLI;
static uint8_t  g_wbo2_status     = 0u;
static uint32_t g_wbo2_last_rx_ms = 0u;
static bool     g_wbo2_seen       = false;
static bool     g_wbo2_fault      = true;   // começa em fault até primeiro frame

static uint32_t g_last_tx_400_ms  = 0u;
static uint32_t g_last_tx_401_ms  = 0u;

inline uint8_t clamp_u8(uint32_t v) noexcept {
    return static_cast<uint8_t>((v > 255u) ? 255u : v);
}

inline uint8_t clamp_u8_i32(int32_t v) noexcept {
    if (v < 0)    { return 0u; }
    if (v > 255)  { return 255u; }
    return static_cast<uint8_t>(v);
}

inline bool elapsed(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms) noexcept {
    return static_cast<uint32_t>(now_ms - last_ms) >= period_ms;
}

// Retorna true se WBO2 nunca recebeu frame ou se o último frame foi há > 500 ms
inline bool wbo2_timed_out(uint32_t now_ms) noexcept {
    if (!g_wbo2_seen) { return true; }
    return static_cast<uint32_t>(now_ms - g_wbo2_last_rx_ms) > 500u;
}

inline void process_rx(uint32_t now_ms) noexcept {
    ems::hal::CanFrame frame = {};
    while (ems::hal::can0_rx_pop(frame)) {
        if (frame.extended || frame.id != g_wbo2_rx_id || frame.dlc < 3u) {
            continue;
        }
        g_lambda_milli    = static_cast<uint16_t>(
                                frame.data[0] |
                                (static_cast<uint16_t>(frame.data[1]) << 8u));
        g_wbo2_status     = frame.data[2];
        g_wbo2_last_rx_ms = now_ms;
        g_wbo2_seen       = true;
        g_wbo2_fault      = false;
    }
}

// TX 0x400 — 10 ms
// status_bits vem do chamador; o bit STATUS_WBO2_FAULT é forçado se sensor offline
inline void tx_0x400(const ems::drv::CkpSnapshot& ckp,
                     const ems::drv::SensorData&  sensors,
                     int8_t  advance_deg,
                     uint8_t pw_ms_x10,
                     uint8_t status_bits) noexcept {
    ems::hal::CanFrame out = {};
    out.id       = 0x400u;
    out.dlc      = 8u;
    out.extended = false;

    const uint16_t rpm = static_cast<uint16_t>(
        (ckp.rpm_x10 > 655350u) ? 65535u : (ckp.rpm_x10 / 10u));

    out.data[0] = static_cast<uint8_t>(rpm & 0xFFu);
    out.data[1] = static_cast<uint8_t>((rpm >> 8u) & 0xFFu);
    out.data[2] = clamp_u8(sensors.map_kpa_x10 / 10u);
    out.data[3] = clamp_u8(sensors.tps_pct_x10 / 10u);
    out.data[4] = clamp_u8_i32((static_cast<int32_t>(sensors.clt_degc_x10) / 10) + 40);
    out.data[5] = clamp_u8_i32(static_cast<int32_t>(advance_deg) + 40);
    out.data[6] = pw_ms_x10;

    // Força bit de falha WBO2 no byte de status antes de transmitir
    const uint8_t effective_status = g_wbo2_fault
        ? (status_bits | ems::app::STATUS_WBO2_FAULT)
        : (status_bits & static_cast<uint8_t>(~ems::app::STATUS_WBO2_FAULT));
    out.data[7] = effective_status;

    static_cast<void>(ems::hal::can0_tx(out));
}

// TX 0x401 — 100 ms
inline void tx_0x401(const ems::drv::SensorData& sensors,
                     int8_t  stft_pct,
                     uint8_t vvt_intake_pct,
                     uint8_t vvt_exhaust_pct) noexcept {
    ems::hal::CanFrame out = {};
    out.id       = 0x401u;
    out.dlc      = 8u;
    out.extended = false;

    out.data[0] = static_cast<uint8_t>(sensors.fuel_press_kpa_x10 & 0xFFu);
    out.data[1] = static_cast<uint8_t>((sensors.fuel_press_kpa_x10 >> 8u) & 0xFFu);
    out.data[2] = static_cast<uint8_t>(sensors.oil_press_kpa_x10 & 0xFFu);
    out.data[3] = static_cast<uint8_t>((sensors.oil_press_kpa_x10 >> 8u) & 0xFFu);
    out.data[4] = clamp_u8_i32((static_cast<int32_t>(sensors.iat_degc_x10) / 10) + 40);
    out.data[5] = clamp_u8_i32(static_cast<int32_t>(stft_pct) + 100);
    out.data[6] = vvt_intake_pct;
    out.data[7] = vvt_exhaust_pct;

    static_cast<void>(ems::hal::can0_tx(out));
}

} // namespace

namespace ems::app {

void can_stack_init(uint16_t wbo2_rx_id) noexcept {
    g_wbo2_rx_id      = static_cast<uint16_t>(wbo2_rx_id & 0x7FFu);
    g_lambda_milli    = WBO2_SAFE_LAMBDA_MILLI;
    g_wbo2_status     = 0u;
    g_wbo2_last_rx_ms = 0u;
    g_wbo2_seen       = false;
    g_wbo2_fault      = true;
    g_last_tx_400_ms  = 0u;
    g_last_tx_401_ms  = 0u;
    ems::hal::can0_init();
}

void can_stack_set_wbo2_rx_id(uint16_t id) noexcept {
    g_wbo2_rx_id = static_cast<uint16_t>(id & 0x7FFu);
}

void can_stack_process(uint32_t now_ms,
                       const ems::drv::CkpSnapshot& ckp,
                       const ems::drv::SensorData&  sensors,
                       int8_t  advance_deg,
                       uint8_t pw_ms_x10,
                       int8_t  stft_pct,
                       uint8_t vvt_intake_pct,
                       uint8_t vvt_exhaust_pct,
                       uint8_t status_bits) noexcept {
    process_rx(now_ms);

    // Atualiza flag de falha após processar RX
    g_wbo2_fault = wbo2_timed_out(now_ms);

    if (elapsed(now_ms, g_last_tx_400_ms, 10u)) {
        g_last_tx_400_ms = now_ms;
        tx_0x400(ckp, sensors, advance_deg, pw_ms_x10, status_bits);
    }

    if (elapsed(now_ms, g_last_tx_401_ms, 100u)) {
        g_last_tx_401_ms = now_ms;
        tx_0x401(sensors, stft_pct, vvt_intake_pct, vvt_exhaust_pct);
    }
}

uint16_t can_stack_lambda_milli() noexcept {
    return g_lambda_milli;
}

// Retorna último lambda recebido se sensor fresco,
// ou WBO2_SAFE_LAMBDA_MILLI (1050) se offline
uint16_t can_stack_lambda_milli_safe(uint32_t now_ms) noexcept {
    if (wbo2_timed_out(now_ms)) {
        return WBO2_SAFE_LAMBDA_MILLI;
    }
    return g_lambda_milli;
}

bool can_stack_wbo2_fresh(uint32_t now_ms) noexcept {
    return !wbo2_timed_out(now_ms);
}

bool can_stack_wbo2_fault() noexcept {
    return g_wbo2_fault;
}

uint8_t can_stack_wbo2_status() noexcept {
    return g_wbo2_status;
}

#if defined(EMS_HOST_TEST)
void can_stack_test_reset() noexcept {
    can_stack_init(0x180u);
}
#endif

} // namespace ems::app
