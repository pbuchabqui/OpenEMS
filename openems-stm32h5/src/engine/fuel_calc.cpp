#include "engine/fuel_calc.h"

#include <cstdint>
#include <cassert>

// CRITICAL FIX: Add debug assertions for safety-critical parameters
#ifndef NDEBUG
#define ASSERT_VALID_RPM_X10(rpm) assert((rpm) >= 0 && (rpm) <= 200000)  // 0-20000 RPM ×10
#define ASSERT_VALID_MAP_KPA(map) assert((map) >= 10 && (map) <= 250)   // 10-250 kPa
#define ASSERT_VALID_TEMP_X10(temp) assert((temp) >= -400 && (temp) <= 1500)  // -40°C to +150°C ×10
#define ASSERT_VALID_VE(ve) assert((ve) <= 255)  // VE 0-255%
#define ASSERT_VALID_VOLTAGE_MV(v) assert((v) >= 6000 && (v) <= 18000)  // 6-18V
#else
#define ASSERT_VALID_RPM_X10(rpm) ((void)0)
#define ASSERT_VALID_MAP_KPA(map) ((void)0)
#define ASSERT_VALID_TEMP_X10(temp) ((void)0)
#define ASSERT_VALID_VE(ve) ((void)0)
#define ASSERT_VALID_VOLTAGE_MV(v) ((void)0)
#endif

namespace {

constexpr uint8_t kCorrPoints = 8u;

constexpr int16_t kCltAxisX10[kCorrPoints] = {-400, -100, 0, 200, 400, 700, 900, 1100};
constexpr uint16_t kCorrCltX256[kCorrPoints] = {384u, 352u, 320u, 288u, 272u, 256u, 248u, 240u};

constexpr int16_t kIatAxisX10[kCorrPoints] = {-200, 0, 200, 400, 600, 800, 1000, 1200};
constexpr uint16_t kCorrIatX256[kCorrPoints] = {272u, 264u, 256u, 248u, 240u, 232u, 224u, 216u};

constexpr int16_t kWarmupAxisX10[kCorrPoints] = {-400, -100, 0, 200, 400, 700, 900, 1100};
constexpr uint16_t kWarmupX256[kCorrPoints] = {420u, 380u, 350u, 320u, 290u, 256u, 256u, 256u};

constexpr uint16_t kVbattAxisMv[kCorrPoints] = {9000u, 10000u, 11000u, 12000u, 13000u, 14000u, 15000u, 16000u};
constexpr uint16_t kDeadTimeUs[kCorrPoints] = {1400u, 1200u, 1050u, 900u, 800u, 700u, 650u, 600u};

constexpr int16_t kAeCltAxisX10[kCorrPoints] = {-400, -100, 0, 200, 400, 700, 900, 1100};
constexpr uint16_t kAeSens[kCorrPoints] = {11u, 10u, 9u, 8u, 7u, 6u, 5u, 4u};

constexpr int16_t kStftKpNum = 3;     // 0.03 por erro_x1000 -> x10
constexpr int16_t kStftKiNum = 1;     // 0.005 por amostra
constexpr int16_t kStftKiDen = 200;
constexpr int16_t kStftClampX10 = 250;

uint16_t g_ae_threshold_tpsdot_x10 = 5u;
uint8_t g_ae_taper_cycles = 8u;
uint8_t g_ae_decay_cycles = 0u;
int32_t g_ae_pulse_us = 0;

int16_t g_stft_pct_x10 = 0;
int32_t g_stft_integrator_x10 = 0;
int16_t g_ltft_pct_x10[ems::engine::kTableAxisSize][ems::engine::kTableAxisSize] = {};

int16_t fuel_ltft_load_cell(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    (void)map_idx;
    (void)rpm_idx;
    return 0;
}

void fuel_ltft_store_cell(uint8_t map_idx, uint8_t rpm_idx, int16_t value_x10) noexcept {
    (void)map_idx;
    (void)rpm_idx;
    (void)value_x10;
}

uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

uint16_t interp_u16_8pt(const int16_t* x_axis,
                        const uint16_t* table,
                        int16_t x) noexcept {
    if (x <= x_axis[0]) {
        return table[0];
    }
    if (x >= x_axis[kCorrPoints - 1u]) {
        return table[kCorrPoints - 1u];
    }

    uint8_t idx = 0u;
    for (uint8_t i = 0u; i < (kCorrPoints - 1u); ++i) {
        if (x <= x_axis[i + 1u]) {
            idx = i;
            break;
        }
    }

    const int16_t x0 = x_axis[idx];
    const int16_t x1 = x_axis[idx + 1u];
    const uint16_t y0 = table[idx];
    const uint16_t y1 = table[idx + 1u];

    const int32_t dx = static_cast<int32_t>(x) - x0;
    const int32_t span = static_cast<int32_t>(x1) - x0;
    if (span <= 0) {
        return y0;
    }

    const int32_t dy = static_cast<int32_t>(y1) - y0;
    const int32_t y = static_cast<int32_t>(y0) + ((dy * dx) / span);
    if (y <= 0) {
        return 0u;
    }
    if (y >= 65535) {
        return 65535u;
    }
    return static_cast<uint16_t>(y);
}

uint16_t interp_u16_8pt_u16x(const uint16_t* x_axis,
                             const uint16_t* table,
                             uint16_t x) noexcept {
    if (x <= x_axis[0]) {
        return table[0];
    }
    if (x >= x_axis[kCorrPoints - 1u]) {
        return table[kCorrPoints - 1u];
    }

    uint8_t idx = 0u;
    for (uint8_t i = 0u; i < (kCorrPoints - 1u); ++i) {
        if (x <= x_axis[i + 1u]) {
            idx = i;
            break;
        }
    }

    const uint16_t x0 = x_axis[idx];
    const uint16_t x1 = x_axis[idx + 1u];
    const uint16_t y0 = table[idx];
    const uint16_t y1 = table[idx + 1u];

    const uint32_t dx = static_cast<uint32_t>(x - x0);
    const uint32_t span = static_cast<uint32_t>(x1 - x0);
    if (span == 0u) {
        return y0;
    }

    const int32_t dy = static_cast<int32_t>(y1) - static_cast<int32_t>(y0);
    const int32_t y = static_cast<int32_t>(y0) + static_cast<int32_t>((dy * static_cast<int32_t>(dx)) / static_cast<int32_t>(span));
    if (y <= 0) {
        return 0u;
    }
    if (y >= 65535) {
        return 65535u;
    }
    return static_cast<uint16_t>(y);
}

uint8_t clt_bucket(int16_t clt_x10) noexcept {
    for (uint8_t i = 0u; i < (kCorrPoints - 1u); ++i) {
        if (clt_x10 < kAeCltAxisX10[i + 1u]) {
            return i;
        }
    }
    return static_cast<uint8_t>(kCorrPoints - 1u);
}

bool closed_loop_allowed(int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut) noexcept {
    return (clt_x10 > 700) && o2_valid && (!ae_active) && (!rev_cut);
}

}  // namespace

namespace ems::engine {

uint8_t ve_table[kTableAxisSize][kTableAxisSize] = {
    {50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u},
    {54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u},
    {58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u, 58u},
    {62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u, 62u},
    {66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u, 66u},
    {70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u, 70u},
    {74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u, 74u},
    {78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u, 78u},
    {82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u, 82u},
    {86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u, 86u},
    {90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u, 90u},
    {94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u, 94u},
    {98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u, 98u},
    {102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u, 102u},
    {106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u, 106u},
    {110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u, 110u},
};

uint8_t get_ve(uint16_t rpm_x10, uint16_t map_kpa) noexcept {
    // CRITICAL FIX: Validate input parameters
    ASSERT_VALID_RPM_X10(rpm_x10);
    ASSERT_VALID_MAP_KPA(map_kpa);
    
    return table3d_lookup_u8(ve_table, kRpmAxisX10, kLoadAxisKpa, rpm_x10, map_kpa);
}

uint32_t calc_base_pw_us(uint16_t req_fuel_us,
                         uint8_t ve,
                         uint16_t map_kpa,
                         uint16_t map_ref_kpa) noexcept {
    // Verificações de produção (ativas mesmo em release): retorno seguro 0
    // evita divisão por zero e overflow de uint64_t na fórmula abaixo.
    if (map_ref_kpa == 0u || ve == 0u || req_fuel_us == 0u) {
        return 0u;
    }
    if (map_kpa > 300u) {
        return 0u;  // MAP > 300 kPa: sensor em fault, não calcular PW
    }
    if (req_fuel_us > 50000u) {
        return 0u;  // REQ_FUEL > 50 ms: valor absurdo, não calcular PW
    }

    ASSERT_VALID_MAP_KPA(map_kpa);
    ASSERT_VALID_MAP_KPA(map_ref_kpa);
    ASSERT_VALID_VE(ve);

    // Base pulse width:
    // PW = REQ_FUEL * (VE / 100) * (MAP / MAP_REF)
    const uint64_t num = static_cast<uint64_t>(req_fuel_us) *
                         static_cast<uint64_t>(ve) *
                         static_cast<uint64_t>(map_kpa);
    const uint32_t den = 100u * static_cast<uint32_t>(map_ref_kpa);
    uint32_t temp = static_cast<uint32_t>(num / den);

    if (temp > 100000u) {
        temp = 100000u;
    }

    // CRITICAL FIX: Validate result
    assert(temp <= 100000);  // Max 100ms pulse width

    return temp;
}

uint16_t corr_clt(int16_t clt_x10) noexcept {
    // CRITICAL FIX: Validate temperature input
    ASSERT_VALID_TEMP_X10(clt_x10);
    
    return interp_u16_8pt(kCltAxisX10, kCorrCltX256, clt_x10);
}

uint16_t corr_iat(int16_t iat_x10) noexcept {
    // CRITICAL FIX: Validate temperature input
    ASSERT_VALID_TEMP_X10(iat_x10);
    
    return interp_u16_8pt(kIatAxisX10, kCorrIatX256, iat_x10);
}

uint16_t corr_vbatt(uint16_t vbatt_mv) noexcept {
    ASSERT_VALID_VOLTAGE_MV(vbatt_mv);
    // Clamp ao range da tabela kVbattAxisMv [9000, 16000] mV.
    // Abaixo de 9V: usa dead-time máximo da tabela (injetor mais lento).
    // Acima de 16V: usa dead-time mínimo (tensão de carga alta).
    // Range do assert (6–18V) é mais amplo para aceitar leituras de sensor
    // ruidosas sem assert falso; o clamp garante interpolação dentro da tabela.
    const uint16_t v = clamp_u16(vbatt_mv,
                                  kVbattAxisMv[0],
                                  kVbattAxisMv[kCorrPoints - 1u]);
    return interp_u16_8pt_u16x(kVbattAxisMv, kDeadTimeUs, v);
}

uint16_t corr_warmup(int16_t clt_x10) noexcept {
    // CRITICAL FIX: Validate temperature input
    ASSERT_VALID_TEMP_X10(clt_x10);
    
    return interp_u16_8pt(kWarmupAxisX10, kWarmupX256, clt_x10);
}

uint32_t calc_final_pw_us(uint32_t base_pw_us,
                          uint16_t corr_clt_x256,
                          uint16_t corr_iat_x256,
                          uint16_t dead_time_us) noexcept {
    const uint64_t num = static_cast<uint64_t>(base_pw_us) * corr_clt_x256 * corr_iat_x256;
    const uint32_t corrected = static_cast<uint32_t>(num >> 16u);  // ÷(256×256) via shift garantido
    return corrected + dead_time_us;
}

void fuel_ae_set_threshold(uint16_t threshold_tpsdot_x10) noexcept {
    g_ae_threshold_tpsdot_x10 = threshold_tpsdot_x10;
}

void fuel_ae_set_taper(uint8_t taper_cycles) noexcept {
    g_ae_taper_cycles = (taper_cycles == 0u) ? 1u : taper_cycles;
}

int32_t calc_ae_pw_us(uint16_t tps_now_x10,
                      uint16_t tps_prev_x10,
                      uint16_t dt_ms,
                      int16_t clt_x10) noexcept {
    if (dt_ms == 0u) {
        return 0;
    }

    int16_t delta_tps_x10 = static_cast<int16_t>(tps_now_x10) - static_cast<int16_t>(tps_prev_x10);
    if (delta_tps_x10 < 0) {
        delta_tps_x10 = 0;
    }

    const int32_t tpsdot_x10 = static_cast<int32_t>(delta_tps_x10) / dt_ms;

    if (tpsdot_x10 > static_cast<int32_t>(g_ae_threshold_tpsdot_x10)) {
        const uint8_t b = clt_bucket(clt_x10);
        g_ae_pulse_us = tpsdot_x10 * static_cast<int32_t>(kAeSens[b]);
        g_ae_decay_cycles = g_ae_taper_cycles;
        return g_ae_pulse_us;
    }

    if (g_ae_decay_cycles > 0u && g_ae_taper_cycles > 0u) {
        --g_ae_decay_cycles;
        return (g_ae_pulse_us * g_ae_decay_cycles) / g_ae_taper_cycles;
    }

    g_ae_pulse_us = 0;
    return 0;
}


void fuel_reset_adaptives() noexcept {
    g_stft_pct_x10 = 0;
    g_stft_integrator_x10 = 0;
    g_ae_decay_cycles = 0u;
    g_ae_pulse_us = 0;

    for (uint8_t y = 0u; y < kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < kTableAxisSize; ++x) {
            g_ltft_pct_x10[y][x] = fuel_ltft_load_cell(y, x);
        }
    }
}

int16_t fuel_update_stft(uint16_t rpm_x10,
                         uint16_t map_kpa,
                         int16_t lambda_target_x1000,
                         int16_t lambda_measured_x1000,
                         int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut) noexcept {
    if (!closed_loop_allowed(clt_x10, o2_valid, ae_active, rev_cut)) {
        g_stft_integrator_x10 = (g_stft_integrator_x10 * 15) / 16;
        g_stft_pct_x10 = static_cast<int16_t>((g_stft_pct_x10 * 15) / 16);
        return g_stft_pct_x10;
    }

    const int16_t error_x1000 = static_cast<int16_t>(lambda_target_x1000 - lambda_measured_x1000);
    const int32_t p_x10 = (static_cast<int32_t>(error_x1000) * kStftKpNum) / 100;
    g_stft_integrator_x10 += (static_cast<int32_t>(error_x1000) * kStftKiNum) / kStftKiDen;

    if (g_stft_integrator_x10 > kStftClampX10) {
        g_stft_integrator_x10 = kStftClampX10;
    } else if (g_stft_integrator_x10 < -kStftClampX10) {
        g_stft_integrator_x10 = -kStftClampX10;
    }

    const int32_t stft = p_x10 + g_stft_integrator_x10;
    g_stft_pct_x10 = clamp_i16(static_cast<int16_t>(stft), -kStftClampX10, kStftClampX10);

    const uint8_t rpm_idx = table_axis_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t map_idx = table_axis_index(kLoadAxisKpa, kTableAxisSize, map_kpa);

    int16_t& cell = g_ltft_pct_x10[map_idx][rpm_idx];
    cell = static_cast<int16_t>(cell + (g_stft_pct_x10 - cell) / 64);
    // Clamp explícito: impede acumulação ilimitada quando fuel_ltft_store_cell
    // é no-op (implementação futura). Limite igual ao do STFT para consistência.
    cell = clamp_i16(cell, -kStftClampX10, kStftClampX10);
    fuel_ltft_store_cell(map_idx, rpm_idx, cell);

    return g_stft_pct_x10;
}

int16_t fuel_get_stft_pct_x10() noexcept {
    return g_stft_pct_x10;
}

int16_t fuel_get_ltft_pct_x10(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        return 0;
    }
    return g_ltft_pct_x10[map_idx][rpm_idx];
}

}  // namespace ems::engine
