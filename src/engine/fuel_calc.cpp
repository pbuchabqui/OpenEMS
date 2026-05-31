#include "engine/fuel_calc.h"
#include "engine/calibration.h"
#include "engine/engine_config.h"
#include "engine/math_utils.h"
#include "engine/table3d.h"
#include "hal/flash.h"

#include <cstdint>
#include <cassert>

// FIX BUG-10: fault counter para falhas de escrita NVM.
// nvm_write_ltft/knock retornam false apenas quando os índices estão fora do
// intervalo válido — indica erro de programação, mas contamos para diagnóstico.
static volatile uint32_t g_nvm_write_faults = 0u;

#ifndef NDEBUG
#define ASSERT_VALID_RPM_X10(rpm) assert((rpm) <= 200000)
#define ASSERT_VALID_MAP_KPA(map) assert((map) >= 10 && (map) <= 300)
#define ASSERT_VALID_TEMP_X10(temp) assert((temp) >= -400 && (temp) <= 1500)
#define ASSERT_VALID_VE(ve) assert((ve) <= 255)
#define ASSERT_VALID_VOLTAGE_MV(v) assert((v) >= 6000 && (v) <= 18000)
#else
#define ASSERT_VALID_RPM_X10(rpm) ((void)0)
#define ASSERT_VALID_MAP_KPA(map) ((void)0)
#define ASSERT_VALID_TEMP_X10(temp) ((void)0)
#define ASSERT_VALID_VE(ve) ((void)0)
#define ASSERT_VALID_VOLTAGE_MV(v) ((void)0)
#endif

namespace {

using ems::engine::clamp_i16;
using ems::engine::clamp_u16;
using ems::engine::interp_u16_8pt;

constexpr uint8_t kCorrPoints = ems::engine::kCorrectionTableSize;

constexpr int16_t kStftKpNum = 3;     // 0.03 por erro_x1000 -> x10
constexpr int16_t kStftKiNum = 1;     // 0.005 por amostra
constexpr int16_t kStftKiDen = 200;
constexpr int16_t kStftClampX10 = 250;
constexpr uint8_t kLambdaHistorySize = 16u;

struct LambdaHistorySample {
    uint32_t time_ms;
    uint32_t rpm_x10;
    uint16_t map_bar_x100;
    int16_t lambda_target_x1000;
    bool valid;
};

uint8_t g_ae_decay_cycles = 0u;
int32_t g_ae_pulse_us = 0;

int16_t g_stft_pct_x10 = 0;
int32_t g_stft_integrator_x10 = 0;
int16_t g_ltft_pct_x10[ems::engine::kTableAxisSize][ems::engine::kTableAxisSize] = {};
// LTFT aditivo: offset em µs, indexado 0-7 (rpm_idx>>1, map_idx>>1)
int16_t g_ltft_add_us[8][8] = {};
bool g_decel_cut = false;
// Referência barométrica: inicializada com map_ref estático, atualizada no key-on
static uint16_t g_baro_bar_x100 = ems::engine::cfg::kMapRefBarX100;
LambdaHistorySample g_lambda_history[kLambdaHistorySize] = {};
uint8_t g_lambda_history_pos = 0u;

int16_t fuel_ltft_load_cell(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    const int8_t stored_pct = ems::hal::nvm_read_ltft(rpm_idx, map_idx);
    return static_cast<int16_t>(stored_pct) * 10;
}

int16_t fuel_ltft_add_load_cell(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    const int8_t stored = ems::hal::nvm_read_ltft_add(rpm_idx >> 1u, map_idx >> 1u);
    return static_cast<int16_t>(stored) * 50;  // 50 µs/count
}

void fuel_ltft_add_store_cell(uint8_t map_idx, uint8_t rpm_idx, int16_t value_us) noexcept {
    const int16_t clamped = clamp_i16(value_us, -6350, 6350);
    const int16_t rounded = static_cast<int16_t>(
        clamped >= 0 ? (clamped + 25) / 50 : (clamped - 25) / 50);
    const bool ok = ems::hal::nvm_write_ltft_add(
        rpm_idx >> 1u, map_idx >> 1u, static_cast<int8_t>(rounded));
    if (!ok) { ++g_nvm_write_faults; }
}

void fuel_ltft_store_cell(uint8_t map_idx, uint8_t rpm_idx, int16_t value_x10) noexcept {
    const int16_t clamped_x10 = clamp_i16(value_x10, -250, 250);
    const int16_t rounded_pct = (clamped_x10 >= 0)
        ? static_cast<int16_t>((clamped_x10 + 5) / 10)
        : static_cast<int16_t>((clamped_x10 - 5) / 10);
    // FIX BUG-10: verifica retorno de nvm_write — false significa índice inválido
    // (erro de programação). Em release, o fault counter capta o problema.
    if (!ems::hal::nvm_write_ltft(rpm_idx, map_idx, static_cast<int8_t>(rounded_pct))) {
        ++g_nvm_write_faults;
    }
}

uint16_t interp_u16_4pt_u16x(const uint16_t* x_axis,
                             const uint16_t* table,
                             uint16_t x) noexcept {
    constexpr uint8_t n = ems::engine::kAeRateTableSize;
    if (x <= x_axis[0]) {
        return table[0];
    }
    if (x >= x_axis[n - 1u]) {
        return table[n - 1u];
    }

    uint8_t idx = 0u;
    while (idx < (n - 2u) && x > x_axis[idx + 1u]) { ++idx; }

    const uint16_t x0 = x_axis[idx];
    const uint16_t x1 = x_axis[idx + 1u];
    const uint16_t y0 = table[idx];
    const uint16_t y1 = table[idx + 1u];
    const uint16_t span = static_cast<uint16_t>(x1 - x0);
    if (span == 0u) {
        return y0;
    }

    const uint32_t y = static_cast<uint32_t>(y0) +
        ((static_cast<uint32_t>(y1 - y0) * static_cast<uint32_t>(x - x0)) / span);
    return static_cast<uint16_t>(y > 65535u ? 65535u : y);
}

uint16_t interp_lambda_delay_3x3(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept {
    const uint8_t xi = ems::engine::table_axis_index(
        ems::engine::lambda_delay_rpm_axis_x10,
        ems::engine::kLambdaDelayTableSize,
        rpm_x10);
    const uint8_t yi = ems::engine::table_axis_index(
        ems::engine::lambda_delay_load_axis_bar_x100,
        ems::engine::kLambdaDelayTableSize,
        map_bar_x100);
    const uint8_t fx = ems::engine::table_axis_frac_q8(
        ems::engine::lambda_delay_rpm_axis_x10, xi, rpm_x10);
    const uint8_t fy = ems::engine::table_axis_frac_q8(
        ems::engine::lambda_delay_load_axis_bar_x100, yi, map_bar_x100);

    const int32_t v00 = ems::engine::lambda_delay_ms_table[yi][xi];
    const int32_t v10 = ems::engine::lambda_delay_ms_table[yi][xi + 1u];
    const int32_t v01 = ems::engine::lambda_delay_ms_table[yi + 1u][xi];
    const int32_t v11 = ems::engine::lambda_delay_ms_table[yi + 1u][xi + 1u];

    const int32_t v0 = v00 + (((v10 - v00) * static_cast<int32_t>(fx)) >> 8u);
    const int32_t v1 = v01 + (((v11 - v01) * static_cast<int32_t>(fx)) >> 8u);
    const int32_t v = v0 + (((v1 - v0) * static_cast<int32_t>(fy)) >> 8u);
    return static_cast<uint16_t>(clamp_i16(static_cast<int16_t>(v), 0, 2000));
}

void lambda_history_push(uint32_t now_ms,
                         uint32_t rpm_x10,
                         uint16_t map_bar_x100,
                         int16_t lambda_target_x1000) noexcept {
    LambdaHistorySample& sample = g_lambda_history[g_lambda_history_pos];
    sample.time_ms = now_ms;
    sample.rpm_x10 = rpm_x10;
    sample.map_bar_x100 = map_bar_x100;
    sample.lambda_target_x1000 = lambda_target_x1000;
    sample.valid = true;
    g_lambda_history_pos = static_cast<uint8_t>((g_lambda_history_pos + 1u) % kLambdaHistorySize);
}

bool lambda_history_get_delayed(uint32_t now_ms,
                                uint16_t delay_ms,
                                LambdaHistorySample& out) noexcept {
    const uint32_t target_ms = now_ms - delay_ms;
    bool found = false;
    uint32_t best_age = 0xFFFFFFFFu;

    for (uint8_t i = 0u; i < kLambdaHistorySize; ++i) {
        const LambdaHistorySample& sample = g_lambda_history[i];
        if (!sample.valid) {
            continue;
        }
        if (sample.time_ms > target_ms) { continue; }
        const uint32_t age = target_ms - sample.time_ms;
        if (age <= 2000u && age < best_age) {
            best_age = age;
            out = sample;
            found = true;
        }
    }

    return found;
}

uint8_t clt_bucket(int16_t clt_x10) noexcept {
    for (uint8_t i = 0u; i < (kCorrPoints - 1u); ++i) {
        if (clt_x10 < ems::engine::ae_clt_corr_axis_x10[i + 1u]) {
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

uint8_t get_ve(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept {
    ASSERT_VALID_RPM_X10(rpm_x10);
    ASSERT_VALID_MAP_KPA(map_bar_x100);
    return table3d_lookup_u8(ve_table, kRpmAxisX10, kLoadAxisBarX100, rpm_x10, map_bar_x100);
}

uint8_t get_ve_prepared(const Table2dLookup& lookup) noexcept {
    return table3d_lookup_u8_prepared(ve_table, lookup);
}

uint16_t get_lambda_target_x1000(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept {
    ASSERT_VALID_RPM_X10(rpm_x10);
    ASSERT_VALID_MAP_KPA(map_bar_x100);

    const int16_t target = table3d_lookup_s16(
        lambda_target_table_x1000, kRpmAxisX10, kLoadAxisBarX100, rpm_x10, map_bar_x100);
    return static_cast<uint16_t>(clamp_i16(target, 650, 1200));
}

uint16_t get_lambda_target_x1000_prepared(const Table2dLookup& lookup) noexcept {
    const int16_t target = table3d_lookup_s16_prepared(lambda_target_table_x1000, lookup);
    return static_cast<uint16_t>(clamp_i16(target, 650, 1200));
}

uint32_t calc_req_fuel_us(uint16_t displacement_cc,
                          uint8_t cylinders,
                          uint16_t injector_flow_cc_min,
                          uint16_t stoich_afr_x100) noexcept {
    if (displacement_cc == 0u || cylinders == 0u ||
        injector_flow_cc_min == 0u || stoich_afr_x100 == 0u) {
        return 0u;
    }

    // REQ_FUEL @ 1.00 bar, 100% VE, lambda 1.00:
    // air/cyl = (displacement / cylinders) * air_density
    // fuel/cyl = air/cyl / stoich_afr
    // pulse = fuel/cyl / injector_mass_flow
    const uint64_t num = static_cast<uint64_t>(displacement_cc) *
                         cfg::kAirDensityMgPerCcX1000 *
                         100u *
                         60000000u;
    const uint64_t den = static_cast<uint64_t>(cylinders) *
                         stoich_afr_x100 *
                         injector_flow_cc_min *
                         cfg::kFuelDensityMgPerCc *
                         1000u;
    uint32_t req = static_cast<uint32_t>(num / den);
    if (req > 50000u) {
        req = 50000u;
    }
    return req;
}

uint32_t default_req_fuel_us() noexcept {
    return calc_req_fuel_us(cfg::g_eng_cfg.displacement_cc,
                            cfg::kCylinderCount,
                            cfg::g_eng_cfg.injector_flow_cc_min,
                            cfg::g_eng_cfg.stoich_afr_x100);
}

uint32_t calc_base_pw_us(uint16_t req_fuel_us,
                         uint8_t ve,
                         uint16_t map_bar_x100,
                         uint16_t map_ref_bar_x100) noexcept {
    // Verificações de produção (ativas mesmo em release): retorno seguro 0
    // evita divisão por zero e overflow de uint64_t na fórmula abaixo.
    if (map_ref_bar_x100 == 0u || ve == 0u || req_fuel_us == 0u) {
        return 0u;
    }
    if (map_bar_x100 > 300u) {
        return 0u;  // MAP > 3.00 bar: sensor em fault, não calcular PW
    }
    if (req_fuel_us > 50000u) {
        return 0u;  // REQ_FUEL > 50 ms: valor absurdo, não calcular PW
    }

    ASSERT_VALID_MAP_KPA(map_bar_x100);
    ASSERT_VALID_MAP_KPA(map_ref_bar_x100);
    ASSERT_VALID_VE(ve);

    // PW = REQ_FUEL * (VE / 100) * (MAP / MAP_REF)
    const uint64_t num = static_cast<uint64_t>(req_fuel_us) *
                         static_cast<uint64_t>(ve) *
                         static_cast<uint64_t>(map_bar_x100);
    const uint32_t den = 100u * static_cast<uint32_t>(map_ref_bar_x100);
    uint32_t temp = static_cast<uint32_t>(num / den);
    if (temp > 100000u) {
        temp = 100000u;
    }
    return temp;
}

uint32_t calc_base_pw_us_default(uint8_t ve,
                                 uint16_t map_bar_x100) noexcept {
    if (ve == 0u) {
        return 0u;
    }
    if (map_bar_x100 > 300u) {
        return 0u;
    }

    const uint32_t req_fuel_us = default_req_fuel_us();
    const uint32_t num = req_fuel_us *
                         static_cast<uint32_t>(ve) *
                         static_cast<uint32_t>(map_bar_x100);
    uint32_t out = num / (100u * static_cast<uint32_t>(cfg::g_eng_cfg.map_ref_bar_x100));
    if (out > 100000u) {
        out = 100000u;
    }
    return out;
}

uint32_t apply_lambda_target_pw_us(uint32_t base_pw_us,
                                        uint16_t lambda_target_x1000) noexcept {
    if (base_pw_us == 0u) {
        return 0u;
    }
    if (lambda_target_x1000 < 650u || lambda_target_x1000 > 1200u) {
        return base_pw_us;
    }

    uint32_t out = (base_pw_us * 1000u) / lambda_target_x1000;
    if (out > 100000u) {
        out = 100000u;
    }
    return out;
}

uint32_t apply_fuel_trim_pw_us(uint32_t base_pw_us,
                                    int16_t trim_pct_x10) noexcept {
    if (base_pw_us == 0u) {
        return 0u;
    }

    const int16_t trim = clamp_i16(trim_pct_x10, -500, 500);
    const int32_t mult_x1000 = 1000 + static_cast<int32_t>(trim);
    if (mult_x1000 <= 0) {
        return 0u;
    }

    uint32_t out = (base_pw_us * static_cast<uint32_t>(mult_x1000)) / 1000u;
    if (out > 100000u) {
        out = 100000u;
    }
    return out;
}

uint16_t corr_clt(int16_t clt_x10) noexcept {
    ASSERT_VALID_TEMP_X10(clt_x10);
    return interp_u16_8pt(clt_corr_axis_x10, clt_corr_x256, kCorrPoints, clt_x10);
}

uint16_t corr_iat(int16_t iat_x10) noexcept {
    ASSERT_VALID_TEMP_X10(iat_x10);
    return interp_u16_8pt(iat_corr_axis_x10, iat_corr_x256, kCorrPoints, iat_x10);
}

uint16_t corr_vbatt(uint16_t vbatt_mv) noexcept {
    ASSERT_VALID_VOLTAGE_MV(vbatt_mv);
    // Clamp ao range da tabela vbatt_corr_axis_mv [9000, 16000] mV.
    // Abaixo de 9V: usa dead-time máximo da tabela (injetor mais lento).
    // Acima de 16V: usa dead-time mínimo (tensão de carga alta).
    // Range do assert (6–18V) é mais amplo para aceitar leituras de sensor
    // ruidosas sem assert falso; o clamp garante interpolação dentro da tabela.
    const uint16_t v = clamp_u16(vbatt_mv,
                                  vbatt_corr_axis_mv[0],
                                  vbatt_corr_axis_mv[kCorrPoints - 1u]);
    return interp_u16_8pt_u16x(vbatt_corr_axis_mv, injector_dead_time_us, kCorrPoints, v);
}

uint16_t corr_warmup(int16_t clt_x10) noexcept {
    ASSERT_VALID_TEMP_X10(clt_x10);
    return interp_u16_8pt(warmup_corr_axis_x10, warmup_corr_x256, kCorrPoints, clt_x10);
}

uint32_t calc_final_pw_us(uint32_t base_pw_us,
                          uint16_t corr_clt_x256,
                          uint16_t corr_iat_x256,
                          uint16_t dead_time_us) noexcept {
    if (base_pw_us == 0u) {
        return 0u;
    }
    const uint64_t num = static_cast<uint64_t>(base_pw_us) * corr_clt_x256 * corr_iat_x256;
    const uint32_t corrected = static_cast<uint32_t>(num / (256u * 256u));
    return corrected + dead_time_us;
}

uint32_t calc_fuel_pw_us_default_fast(uint8_t ve,
                                      uint16_t map_bar_x100,
                                      uint16_t lambda_target_x1000,
                                      int16_t trim_pct_x10,
                                      uint16_t corr_clt_x256,
                                      uint16_t corr_iat_x256,
                                      uint16_t dead_time_us) noexcept {
    uint32_t base_pw_us = 0u;
    if (ve != 0u && map_bar_x100 <= 300u) {
        const uint32_t req_fuel_us = default_req_fuel_us();
        const uint64_t num = static_cast<uint64_t>(req_fuel_us) *
                             static_cast<uint64_t>(ve) *
                             static_cast<uint64_t>(map_bar_x100);
        // Denominador usa baro dinâmico (MS42 TI_FAC_ALTI): MAP/baro em vez de MAP/100.
        // A altitude reduz o baro → denominador menor → PW sobe para compensar VE
        // não calibrada na altitude (WOT a 0.90bar não é igual a 90% carga no nível do mar).
        const uint16_t baro = (g_baro_bar_x100 != 0u)
                              ? g_baro_bar_x100 : cfg::g_eng_cfg.map_ref_bar_x100;
        base_pw_us = static_cast<uint32_t>(
            num / (100u * static_cast<uint64_t>(baro)));
        if (base_pw_us > 100000u) {
            base_pw_us = 100000u;
        }
    }

    uint32_t lambda_pw_us = 0u;
    if (base_pw_us != 0u &&
        lambda_target_x1000 >= 650u &&
        lambda_target_x1000 <= 1200u) {
        lambda_pw_us = (base_pw_us * 1000u) / lambda_target_x1000;
        if (lambda_pw_us > 100000u) {
            lambda_pw_us = 100000u;
        }
    }

    uint32_t trimmed_pw_us = 0u;
    if (lambda_pw_us != 0u) {
        const int16_t trim = clamp_i16(trim_pct_x10, -500, 500);
        const int32_t mult_x1000 = 1000 + static_cast<int32_t>(trim);
        if (mult_x1000 > 0) {
            trimmed_pw_us = (lambda_pw_us * static_cast<uint32_t>(mult_x1000)) / 1000u;
            if (trimmed_pw_us > 100000u) {
                trimmed_pw_us = 100000u;
            }
        }
    }

    if (trimmed_pw_us == 0u) {
        return 0u;
    }
    return calc_final_pw_us(trimmed_pw_us, corr_clt_x256, corr_iat_x256, dead_time_us);
}

void fuel_ae_set_threshold(uint16_t threshold_tpsdot_x10) noexcept {
    ae_tpsdot_threshold_x10 = threshold_tpsdot_x10;
}

void fuel_ae_set_taper(uint8_t taper_cycles) noexcept {
    ae_taper_cycles = (taper_cycles == 0u) ? 1u : taper_cycles;
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

    if (tpsdot_x10 > static_cast<int32_t>(ae_tpsdot_threshold_x10)) {
        const uint8_t b = clt_bucket(clt_x10);
        const uint16_t taper = ae_taper_cycles > 255u
            ? 255u
            : (ae_taper_cycles == 0u ? 1u : ae_taper_cycles);
        const uint16_t tpsdot_u16 = static_cast<uint16_t>(
            tpsdot_x10 > 65535 ? 65535 : tpsdot_x10);
        const uint16_t base_pw_us =
            interp_u16_4pt_u16x(ae_tpsdot_axis_x10, ae_pw_adder_us, tpsdot_u16);
        g_ae_pulse_us =
            (static_cast<int32_t>(base_pw_us) * static_cast<int32_t>(ae_clt_sens[b])) / 8;
        if (g_ae_pulse_us > static_cast<int32_t>(ae_max_pw_us)) {
            g_ae_pulse_us = static_cast<int32_t>(ae_max_pw_us);
        }
        g_ae_decay_cycles = static_cast<uint8_t>(taper);
        return g_ae_pulse_us;
    }

    if (g_ae_decay_cycles > 0u) {
        const uint16_t taper = ae_taper_cycles > 255u
            ? 255u
            : (ae_taper_cycles == 0u ? 1u : ae_taper_cycles);
        --g_ae_decay_cycles;
        return (g_ae_pulse_us * g_ae_decay_cycles) /
               static_cast<int32_t>(taper);
    }

    g_ae_pulse_us = 0;
    return 0;
}


void fuel_reset_adaptives() noexcept {
    g_stft_pct_x10 = 0;
    g_stft_integrator_x10 = 0;
    g_ae_decay_cycles = 0u;
    g_ae_pulse_us = 0;
    g_decel_cut = false;
    fuel_lambda_delay_reset();

    for (uint8_t y = 0u; y < kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < kTableAxisSize; ++x) {
            g_ltft_pct_x10[y][x] = fuel_ltft_load_cell(y, x);
        }
    }
    for (uint8_t y = 0u; y < 8u; ++y) {
        for (uint8_t x = 0u; x < 8u; ++x) {
            // Carrega via índice 16×16 equivalente (dobra o índice)
            g_ltft_add_us[y][x] = fuel_ltft_add_load_cell(
                static_cast<uint8_t>(y << 1u), static_cast<uint8_t>(x << 1u));
        }
    }
}

void fuel_lambda_delay_reset() noexcept {
    for (uint8_t i = 0u; i < kLambdaHistorySize; ++i) {
        g_lambda_history[i] = {};
    }
    g_lambda_history_pos = 0u;
}

uint16_t lambda_delay_ms_from_rpm_load(uint32_t rpm_x10,
                                       uint16_t map_bar_x100) noexcept {
    return interp_lambda_delay_3x3(rpm_x10, map_bar_x100);
}

int16_t fuel_update_stft(uint32_t rpm_x10,
                         uint16_t map_bar_x100,
                         int16_t lambda_target_x1000,
                         int16_t lambda_measured_x1000,
                         int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut,
                         uint32_t net_pw_us) noexcept {
    if (!closed_loop_allowed(clt_x10, o2_valid, ae_active, rev_cut)) {
        g_stft_integrator_x10 = (g_stft_integrator_x10 * 15) / 16;
        g_stft_pct_x10 = static_cast<int16_t>((g_stft_pct_x10 * 15) / 16);
        return g_stft_pct_x10;
    }

    const int16_t error_x1000 = static_cast<int16_t>(lambda_measured_x1000 - lambda_target_x1000);
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
    const uint8_t map_idx = table_axis_index(kLoadAxisBarX100, kTableAxisSize, map_bar_x100);

    if (net_pw_us > 0u && net_pw_us < static_cast<uint32_t>(ltft_add_pw_threshold_us)) {
        // PW pequeno: erros de offset do injetor dominam — integrar LTFT aditivo.
        // Converte correção percentual em µs: error_us = stft_pct × net_pw / 100
        const int32_t error_us = (static_cast<int32_t>(g_stft_pct_x10) *
                                  static_cast<int32_t>(net_pw_us)) / 1000;  // x10 → /1000
        const uint8_t ri = rpm_idx >> 1u;
        const uint8_t mi = map_idx >> 1u;
        int16_t& cell_add = g_ltft_add_us[mi][ri];
        cell_add = clamp_i16(
            static_cast<int16_t>(cell_add + (error_us - cell_add) / 64), -6350, 6350);
        fuel_ltft_add_store_cell(map_idx, rpm_idx, cell_add);
    } else {
        // PW normal: integrar LTFT multiplicativo (comportamento original).
        int16_t& cell = g_ltft_pct_x10[map_idx][rpm_idx];
        cell = static_cast<int16_t>(cell + (g_stft_pct_x10 - cell) / 64);
        cell = clamp_i16(cell, -kStftClampX10, kStftClampX10);
        fuel_ltft_store_cell(map_idx, rpm_idx, cell);
    }

    return g_stft_pct_x10;
}

int16_t fuel_update_stft_delayed(uint32_t now_ms,
                                 uint32_t rpm_x10,
                                 uint16_t map_bar_x100,
                                 int16_t lambda_target_x1000,
                                 int16_t lambda_measured_x1000,
                                 int16_t clt_x10,
                                 bool o2_valid,
                                 bool ae_active,
                                 bool rev_cut,
                                 uint32_t net_pw_us) noexcept {
    lambda_history_push(now_ms, rpm_x10, map_bar_x100, lambda_target_x1000);

    const uint16_t delay_ms = lambda_delay_ms_from_rpm_load(rpm_x10, map_bar_x100);
    LambdaHistorySample delayed{};
    if (!lambda_history_get_delayed(now_ms, delay_ms, delayed)) {
        return fuel_update_stft(rpm_x10,
                                map_bar_x100,
                                lambda_target_x1000,
                                lambda_measured_x1000,
                                clt_x10,
                                false,
                                ae_active,
                                rev_cut,
                                net_pw_us);
    }

    return fuel_update_stft(delayed.rpm_x10,
                            delayed.map_bar_x100,
                            delayed.lambda_target_x1000,
                            lambda_measured_x1000,
                            clt_x10,
                            o2_valid,
                            ae_active,
                            rev_cut,
                            net_pw_us);
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

int16_t fuel_get_ltft_add_us(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        return 0;
    }
    return g_ltft_add_us[map_idx >> 1u][rpm_idx >> 1u];
}

// ── Compensação barométrica (MS42 TI_FAC_ALTI) ───────────────────────────────

void fuel_set_baro_bar_x100(uint16_t baro) noexcept {
    // Clamp: aceita apenas valores plausíveis de pressão barométrica
    // 70 = 700 mbar (≈5000m altitude) … 110 = 1100 mbar (abaixo do nível do mar)
    if (baro < 70u || baro > 110u) { return; }
    g_baro_bar_x100 = baro;
}

uint16_t fuel_get_baro_bar_x100() noexcept {
    return g_baro_bar_x100;
}

// ── Corte de combustível na desaceleração (MS42 TI_PUR) ──────────────────────

bool fuel_decel_cut_update(uint32_t rpm_x10,
                           uint16_t tps_pct_x10,
                           int16_t clt_x10) noexcept {
    const bool throttle_closed = tps_pct_x10 <= decel_cut_tps_threshold_x10;
    const bool engine_warm     = clt_x10 >= decel_cut_min_clt_x10;

    if (!g_decel_cut) {
        if (throttle_closed && engine_warm && rpm_x10 >= decel_cut_entry_rpm_x10) {
            g_decel_cut = true;
        }
    } else {
        // Sai do corte se o acelerador abrir OU o RPM cair abaixo do limiar de saída.
        // A histerese (entry > exit) evita oscilações ao redor do limiar.
        if (!throttle_closed || rpm_x10 < decel_cut_exit_rpm_x10) {
            g_decel_cut = false;
        }
    }
    return g_decel_cut;
}

bool fuel_decel_cut_active() noexcept {
    return g_decel_cut;
}

void fuel_decel_cut_reset() noexcept {
    g_decel_cut = false;
}

}  // namespace ems::engine
