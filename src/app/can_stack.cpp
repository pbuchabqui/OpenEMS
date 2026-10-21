#include "app/can_stack.h"

#include <cstdint>

#include "hal/can.h"
#include "engine/math_utils.h"
#include "app/can_rx_map.h"

namespace {

static uint16_t g_wbo2_rx_id      = 0x180u;
static uint16_t g_lambda_milli    = ems::app::WBO2_SAFE_LAMBDA_MILLI;
static uint8_t  g_wbo2_status     = 0u;
static uint32_t g_wbo2_last_rx_ms = 0u;
static bool     g_wbo2_seen       = false;
static bool     g_wbo2_fault      = true;   // começa em fault até primeiro frame
// Lambda simulado p/ bancada (bench-mode 'B'): substitui o barramento CAN —
// wbo2 sempre "fresco" com λ fixo, libertando o gate do STFT/LTFT sem WBO2 físico.
static bool     g_bench_lambda_on    = false;
static uint16_t g_bench_lambda_milli = 1000u;

static uint32_t g_last_tx_400_ms  = 0u;
static uint32_t g_last_tx_401_ms  = 0u;
static uint32_t g_last_tx_402_ms  = 0u;

// ── FCO — accumulated fuel consumption (MS42 §2.2.7) ──────────────────────
// Acumulador em nanolitros (fracção) + contador inteiro em microlitros.
// Fórmula (derivação em can_stack.cpp): delta_nl = pw_ms_x10 × flow × n_cyl × rpm × dt_ms / 72000
// com flow = kInjectorFlowCcMin (cc/min), n_cyl = kCylinderCount, rpm em RPM.
// Constantes inline para manter can_stack.cpp independente de engine_config.h.
static constexpr uint32_t kFcoFlowCcMin  = 450u;  // injector flow cc/min
static constexpr uint32_t kFcoCylCount   = 4u;    // number of cylinders
static constexpr uint64_t kFcoDivisor    = 72000u; // vazão [nl/s] = pw_ms_x10 × flow × n_cyl × rpm / 72; × dt_ms / 1000

static uint32_t g_fco_accum_ul   = 0u;  // accumulated fuel [µl], wraps at ~4295 L
static uint32_t g_fco_nl_frac    = 0u;  // sub-µl remainder [nl]
static uint32_t g_fco_snap_ul    = 0u;  // accum at last 0x402 TX (for delta)
static uint32_t g_fco_last_ms    = 0u;  // instante da última acumulação (0 = nenhuma ainda)

using ems::engine::clamp_u8;

inline uint8_t clamp_u8_i32(int32_t v) noexcept {
    if (v < 0)    { return 0u; }
    if (v > 255)  { return 255u; }
    return static_cast<uint8_t>(v);
}

inline bool elapsed(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms) noexcept {
    return static_cast<uint32_t>(now_ms - last_ms) >= period_ms;
}

// Retorna true se WBO2 nunca recebeu frame ou se o último frame foi há > 500 ms
static constexpr uint16_t kWbo2LambdaMinMilli = 400u;    // λ 0,400
static constexpr uint16_t kWbo2LambdaMaxMilli = 3000u;   // λ 3,000

inline bool wbo2_timed_out(uint32_t now_ms) noexcept {
    if (g_bench_lambda_on) { return false; }  // bench: lambda simulado sempre fresco
    if (!g_wbo2_seen) { return true; }
    return static_cast<uint32_t>(now_ms - g_wbo2_last_rx_ms) > 500u;
}

inline void process_rx(uint32_t now_ms) noexcept {
    ems::hal::CanFrame frame = {};
    while (ems::hal::can0_rx_pop(frame)) {
        if (frame.extended) { continue; }

        // WBO2
        if (frame.id == g_wbo2_rx_id && frame.dlc >= 3u) {
            const uint16_t lambda_raw = static_cast<uint16_t>(
                                    frame.data[0] |
                                    (static_cast<uint16_t>(frame.data[1]) << 8u));
            g_wbo2_status     = frame.data[2];
            // Plausibilidade: 0, 0xFFFF e qualquer valor fora de λ 0,4..3,0 (sensor aquecendo,
            // erro, frame lixo) não contam como leitura fresca — antes liberavam o closed-loop
            // e o aprendizado de STFT/LTFT com um λ inválido. O timeout de 500 ms vale.
            if (lambda_raw >= kWbo2LambdaMinMilli && lambda_raw <= kWbo2LambdaMaxMilli) {
                g_lambda_milli    = lambda_raw;
                g_wbo2_last_rx_ms = now_ms;
                g_wbo2_seen       = true;
                g_wbo2_fault      = false;
            }
        }

        // Sinais configuráveis (marcha, velocidade, …)
        ems::app::can_rx_map_process(frame.id, frame.data, frame.dlc, now_ms);
    }
}

// TX 0x400 — 10 ms
// byte[6] = pw_ms_x10
// byte[7] = status_bits[7:0]  (bits 8-15 go in 0x401 byte[6])
// WBO2_FAULT (bit 9) is forced here on the full 16-bit word before splitting.
inline bool tx_0x400(const ems::drv::CkpSnapshot& ckp,
                     const ems::drv::SensorData&  sensors,
                     int8_t   advance_deg,
                     uint8_t  pw_ms_x10,
                     uint16_t status_bits) noexcept {
    ems::hal::CanFrame out = {};
    out.id       = 0x400u;
    out.dlc      = 8u;
    out.extended = false;

    const uint16_t rpm = static_cast<uint16_t>(
        (ckp.rpm_x10 > 655350u) ? 65535u : (ckp.rpm_x10 / 10u));

    // Force WBO2_FAULT on the full 16-bit word so bit 9 is preserved before split
    const uint16_t effective_status = g_wbo2_fault
        ? (status_bits | ems::app::STATUS_WBO2_FAULT)
        : (status_bits & ~static_cast<uint16_t>(ems::app::STATUS_WBO2_FAULT));

    out.data[0] = static_cast<uint8_t>(rpm & 0xFFu);
    out.data[1] = static_cast<uint8_t>((rpm >> 8u) & 0xFFu);
    out.data[2] = clamp_u8(sensors.map_bar_x1000 / 10u);
    out.data[3] = clamp_u8(sensors.etb_tps_pct_x10 / 10u);
    out.data[4] = clamp_u8_i32((static_cast<int32_t>(sensors.clt_degc_x10) / 10) + 40);
    out.data[5] = clamp_u8_i32(static_cast<int32_t>(advance_deg) + 40);
    out.data[6] = pw_ms_x10;
    out.data[7] = static_cast<uint8_t>(effective_status & 0xFFu);  // bits 0-7

    return ems::hal::can0_tx(out);
}

// TX 0x401 — 100 ms
// byte[6] = status_bits[15:8] (bits 8-15 of 16-bit status word, e.g. STATUS_WBO2_FAULT bit9)
// byte[7] = vvt_exhaust_pct (reserved, 0 until VVT implemented)
inline bool tx_0x401(const ems::drv::SensorData& sensors,
                     int8_t   stft_pct,
                     uint8_t  status_hi,
                     uint8_t  vvt_exhaust_pct) noexcept {
    ems::hal::CanFrame out = {};
    out.id       = 0x401u;
    out.dlc      = 8u;
    out.extended = false;

    out.data[0] = static_cast<uint8_t>(sensors.fuel_press_bar_x1000 & 0xFFu);
    out.data[1] = static_cast<uint8_t>((sensors.fuel_press_bar_x1000 >> 8u) & 0xFFu);
    out.data[2] = static_cast<uint8_t>(sensors.oil_press_bar_x1000 & 0xFFu);
    out.data[3] = static_cast<uint8_t>((sensors.oil_press_bar_x1000 >> 8u) & 0xFFu);
    out.data[4] = clamp_u8_i32((static_cast<int32_t>(sensors.iat_degc_x10) / 10) + 40);
    out.data[5] = clamp_u8_i32(static_cast<int32_t>(stft_pct) + 100);
    out.data[6] = status_hi;       // bits 8-15 of status_bits (STATUS_WBO2_FAULT = bit1 here)
    out.data[7] = vvt_exhaust_pct; // reserved, always 0 until VVT implemented

    return ems::hal::can0_tx(out);
}

// Acumula combustível injectado. O período real de chamada é medido (dt_ms): can_stack_process
// roda no bloco de 20 ms, não a cada 2 ms, e assumir 2 ms subestimava o consumo em ~10x.
inline void fco_accumulate(uint8_t pw_ms_x10, uint32_t rpm, uint32_t dt_ms) noexcept {
    if (rpm == 0u || pw_ms_x10 == 0u || dt_ms == 0u) { return; }
    // vazão [nl/s] = pw_ms_x10 × flow × n_cyl × rpm / 72   (pw em 0,1 ms, flow cc/min, rpm)
    // delta_nl = vazão × dt_ms / 1000 → pw × flow × n_cyl × rpm × dt_ms / 72000
    // uint64 para evitar overflow
    const uint64_t delta_nl = static_cast<uint64_t>(pw_ms_x10)
                              * kFcoFlowCcMin * kFcoCylCount * rpm * dt_ms / kFcoDivisor;
    g_fco_nl_frac  += static_cast<uint32_t>(delta_nl);
    g_fco_accum_ul += g_fco_nl_frac / 1000u;
    g_fco_nl_frac  %= 1000u;
}

// TX 0x402 — 500 ms
// Byte 0-3: accumulated fuel [µl] (uint32 LE, wraps at ~4295 L)
// Byte 4-5: fuel consumed in last 500 ms window [µl] (uint16 LE)
// Byte 6-7: reserved
inline bool tx_0x402() noexcept {
    const uint32_t delta_ul = g_fco_accum_ul - g_fco_snap_ul;

    ems::hal::CanFrame out = {};
    out.id       = 0x402u;
    out.dlc      = 8u;
    out.extended = false;

    out.data[0] = static_cast<uint8_t>( g_fco_accum_ul         & 0xFFu);
    out.data[1] = static_cast<uint8_t>((g_fco_accum_ul >>  8u) & 0xFFu);
    out.data[2] = static_cast<uint8_t>((g_fco_accum_ul >> 16u) & 0xFFu);
    out.data[3] = static_cast<uint8_t>((g_fco_accum_ul >> 24u) & 0xFFu);

    const uint16_t delta_u16 = (delta_ul > 65535u) ? 65535u : static_cast<uint16_t>(delta_ul);
    out.data[4] = static_cast<uint8_t>( delta_u16        & 0xFFu);
    out.data[5] = static_cast<uint8_t>((delta_u16 >> 8u) & 0xFFu);
    out.data[6] = 0u;
    out.data[7] = 0u;

    const bool ok = ems::hal::can0_tx(out);
    if (ok) { g_fco_snap_ul = g_fco_accum_ul; }  // só avança o delta se o frame foi aceito
    return ok;
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
    g_last_tx_402_ms  = 0u;
    g_fco_accum_ul    = 0u;
    g_fco_nl_frac     = 0u;
    g_fco_snap_ul     = 0u;
    g_fco_last_ms     = 0u;
    ems::hal::can0_init();
}

void can_stack_set_wbo2_rx_id(uint16_t id) noexcept {
    g_wbo2_rx_id = static_cast<uint16_t>(id & 0x7FFu);
}

void can_stack_process(uint32_t now_ms,
                       const ems::drv::CkpSnapshot& ckp,
                       const ems::drv::SensorData&  sensors,
                       int8_t   advance_deg,
                       uint8_t  pw_ms_x10,
                       int8_t   stft_pct,
                       uint8_t  /*vvt_intake_pct*/,  // reserved — replaced by status_hi in 0x401[6]
                       uint8_t  vvt_exhaust_pct,
                       uint16_t status_bits) noexcept {
    process_rx(now_ms);

    // Atualiza flag de falha após processar RX
    g_wbo2_fault = wbo2_timed_out(now_ms);

    // Acumula combustível injectado, com o dt real desde a chamada anterior.
    {
        const uint32_t dt = (g_fco_last_ms == 0u || now_ms < g_fco_last_ms) ? 0u : (now_ms - g_fco_last_ms);
        g_fco_last_ms = now_ms;
        fco_accumulate(pw_ms_x10, ckp.rpm_x10 / 10u, dt > 200u ? 200u : dt);
    }

    // Um único buffer de TX (can0_tx recusa se o anterior ainda está pendente) e uma chamada a
    // cada 20 ms: o 0x400 (10 ms) vence a cada chamada e, enviado primeiro, fazia o 0x401 e o
    // 0x402 serem recusados sempre. Agora envia no máximo UM frame por chamada, dos menos
    // frequentes para o mais frequente, e só carimba o instante se o envio foi aceito (um frame
    // recusado é tentado de novo na chamada seguinte).
    if (elapsed(now_ms, g_last_tx_402_ms, 500u)) {
        if (tx_0x402()) { g_last_tx_402_ms = now_ms; }
    } else if (elapsed(now_ms, g_last_tx_401_ms, 100u)) {
        // FIX: pass status_hi (bits 8-15) so STATUS_WBO2_FAULT (bit 9) reaches the bus.
        const uint8_t status_hi = static_cast<uint8_t>((status_bits >> 8u) & 0xFFu);
        if (tx_0x401(sensors, stft_pct, status_hi, vvt_exhaust_pct)) { g_last_tx_401_ms = now_ms; }
    } else if (elapsed(now_ms, g_last_tx_400_ms, 10u)) {
        if (tx_0x400(ckp, sensors, advance_deg, pw_ms_x10, status_bits)) { g_last_tx_400_ms = now_ms; }
    }
}

uint16_t can_stack_lambda_milli() noexcept {
    if (g_bench_lambda_on) { return g_bench_lambda_milli; }
    return g_lambda_milli;
}

// Retorna último lambda recebido se sensor fresco,
// ou WBO2_SAFE_LAMBDA_MILLI (1050) se offline
uint16_t can_stack_lambda_milli_safe(uint32_t now_ms) noexcept {
    if (g_bench_lambda_on) { return g_bench_lambda_milli; }
    if (wbo2_timed_out(now_ms)) {
        return WBO2_SAFE_LAMBDA_MILLI;
    }
    return g_lambda_milli;
}

bool can_stack_wbo2_fresh(uint32_t now_ms) noexcept {
    return !wbo2_timed_out(now_ms);
}

bool can_stack_wbo2_fault() noexcept {
    if (g_bench_lambda_on) { return false; }
    return g_wbo2_fault;
}

void can_stack_set_bench_lambda(bool enable, uint16_t lambda_milli) noexcept {
    g_bench_lambda_on    = enable;
    g_bench_lambda_milli = lambda_milli;
}

#if defined(EMS_HOST_TEST)
#endif

} // namespace ems::app
