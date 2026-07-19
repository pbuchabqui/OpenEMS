#include "engine/fuel_calc.h"
#include "engine/calibration.h"
#include "engine/engine_config.h"
#include "engine/math_utils.h"
#include "engine/table3d.h"

#include <cstdint>
#include <cassert>

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

uint32_t isqrt_u32(uint32_t x) noexcept {
    if (x == 0u) {
        return 0u;
    }
    uint32_t res = 0u;
    uint32_t bit = 1u << 30;  // maior potência de 4 que cabe em uint32_t
    while (bit > x) {
        bit >>= 2u;
    }
    while (bit != 0u) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1u) + bit;
        } else {
            res >>= 1u;
        }
        bit >>= 2u;
    }
    return res;
}

uint8_t g_ae_decay_cycles = 0u;
int32_t g_ae_pulse_us = 0;

bool g_decel_cut = false;
// Referência barométrica: inicializada com map_ref estático, atualizada no key-on
static uint16_t g_baro_bar_x100 = ems::engine::cfg::kMapRefBarX100;
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

    // Signed dy: non-monotonic AE tables (y1 < y0) must not wrap to huge PW.
    const int32_t dy = static_cast<int32_t>(y1) - static_cast<int32_t>(y0);
    const int32_t y = static_cast<int32_t>(y0) +
        (dy * static_cast<int32_t>(x - x0)) / static_cast<int32_t>(span);
    if (y <= 0) { return 0u; }
    if (y > 65535) { return 65535u; }
    return static_cast<uint16_t>(y);
}

uint8_t clt_bucket(int16_t clt_x10) noexcept {
    for (uint8_t i = 0u; i < (kCorrPoints - 1u); ++i) {
        if (clt_x10 < ems::engine::ae_clt_corr_axis_x10[i + 1u]) {
            return i;
        }
    }
    return static_cast<uint8_t>(kCorrPoints - 1u);
}

}  // namespace

namespace ems::engine {

void fuel_ae_reset() noexcept {
    g_ae_decay_cycles = 0u;
    g_ae_pulse_us = 0;
}

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

uint32_t apply_injector_scurve(uint32_t pw_us) noexcept {
    if (pw_us == 0u) {
        return 0u;
    }
    const uint16_t pw_clamped = static_cast<uint16_t>(pw_us > 65535u ? 65535u : pw_us);
    const uint16_t corr_q8 = ems::engine::interp_u16_8pt_u16x(
        injector_scurve_pw_axis_us, injector_scurve_corr_q8, kCorrPoints, pw_clamped);
    if (corr_q8 == 0u) {
        return pw_us;  // tabela mal calibrada — não divide por zero, sem correção
    }
    const uint64_t pw_corrected = (static_cast<uint64_t>(pw_us) * 256u) / corr_q8;
    return static_cast<uint32_t>(pw_corrected > 200000u ? 200000u : pw_corrected);
}

uint32_t apply_delta_p_compensation(uint32_t pw_us,
                                    uint16_t fuel_press_bar_x1000,
                                    uint16_t map_bar_x100) noexcept {
    if (pw_us == 0u) {
        return 0u;
    }
    // Sensor sem leitura válida: usa o nominal, sem correção (ratio_q8 = 256).
    const uint16_t actual_press_bar_x1000 =
        (fuel_press_bar_x1000 > 0u) ? fuel_press_bar_x1000 : fuel_press_nominal_bar_x1000;

    // ΔP absoluto no bico = pressão do rail - pressão do coletor (ambas absolutas,
    // bar × 1000). map_bar_x100 (bar × 100) → × 10 para bar × 1000.
    int32_t delta_p_actual = static_cast<int32_t>(actual_press_bar_x1000) -
                             static_cast<int32_t>(map_bar_x100) * 10;
    int32_t delta_p_nominal = static_cast<int32_t>(fuel_press_nominal_bar_x1000) -
                              static_cast<int32_t>(map_bar_x100) * 10;
    // Piso de 0.2 bar: evita divisão por ~0 / raiz de negativo em falhas de sensor.
    if (delta_p_actual < 200) delta_p_actual = 200;
    if (delta_p_nominal < 200) delta_p_nominal = 200;

    // Fluxo do bico ∝ sqrt(ΔP) → PW_corrigido = PW_base × sqrt(ΔP_nominal / ΔP_atual)
    const uint32_t ratio_q8 = static_cast<uint32_t>(
        (static_cast<int64_t>(delta_p_nominal) * 256) / delta_p_actual);
    const uint32_t sqrt_factor_q8 = isqrt_u32(ratio_q8 * 256u);

    const uint64_t pw_corrected = (static_cast<uint64_t>(pw_us) * sqrt_factor_q8) / 256u;
    return static_cast<uint32_t>(pw_corrected > 200000u ? 200000u : pw_corrected);
}

uint32_t calc_final_pw_us(uint32_t base_pw_us,
                          uint16_t corr_clt_x256,
                          uint16_t corr_iat_x256,
                          uint16_t dead_time_us) noexcept {
    if (base_pw_us == 0u) {
        return 0u;
    }
    // Clamp Q8 corrections to [0.25× .. 2.0×] so corrupt page-5 cal cannot
    // wrap the product / cast. Matches practical enrichment range.
    constexpr uint16_t kCorrMinQ8 = 64u;
    constexpr uint16_t kCorrMaxQ8 = 512u;
    constexpr uint32_t kMaxFinalPwUs = 100000u;  // 100 ms — same cap as base path
    const uint16_t c_clt = (corr_clt_x256 < kCorrMinQ8) ? kCorrMinQ8
                         : (corr_clt_x256 > kCorrMaxQ8) ? kCorrMaxQ8 : corr_clt_x256;
    const uint16_t c_iat = (corr_iat_x256 < kCorrMinQ8) ? kCorrMinQ8
                         : (corr_iat_x256 > kCorrMaxQ8) ? kCorrMaxQ8 : corr_iat_x256;
    const uint64_t num = static_cast<uint64_t>(base_pw_us) * c_clt * c_iat;
    uint64_t corrected = num / (256u * 256u);
    // Injector 2-slope (modelo rusEFI): o PW líquido pedido assume fluxo
    // linear (slope principal). Abaixo do breakpoint o bico rende só
    // r = rate_q8/256 desse fluxo → tempo comandado = net/r nessa região e
    // net + t_break·(1−r) acima (contínuo em net_break = r·t_break).
    // rate_q8 = 0 desliga (net inalterado — comportamento anterior).
    const uint16_t r_q8 = inj_small_pulse_rate_q8;
    const uint32_t t_break = inj_small_pulse_break_us;
    if (r_q8 != 0u && r_q8 < 256u && t_break != 0u) {
        const uint64_t net_break = (static_cast<uint64_t>(t_break) * r_q8) / 256u;
        if (corrected < net_break) {
            corrected = (corrected * 256u) / r_q8;
        } else {
            corrected += (static_cast<uint64_t>(t_break) * (256u - r_q8)) / 256u;
        }
    }
    const uint64_t total = corrected + static_cast<uint64_t>(dead_time_us);
    if (total > kMaxFinalPwUs) { return kMaxFinalPwUs; }
    return static_cast<uint32_t>(total);
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

int32_t calc_ae_pw_from_tpsdot(int16_t tpsdot_x10, int16_t clt_x10) noexcept {
    const int16_t thr = static_cast<int16_t>(ae_tpsdot_threshold_x10);
    const int16_t abs_dot = (tpsdot_x10 >= 0) ? tpsdot_x10 : static_cast<int16_t>(-tpsdot_x10);
    const bool tip_in  = (tpsdot_x10 > thr);
    const bool tip_out = (tpsdot_x10 < -thr);

    if (tip_in || tip_out) {
        const uint8_t b = clt_bucket(clt_x10);
        const uint16_t taper = ae_taper_cycles > 255u
            ? 255u
            : (ae_taper_cycles == 0u ? 1u : ae_taper_cycles);
        const uint16_t tpsdot_u16 = static_cast<uint16_t>(
            abs_dot > 1000 ? 1000 : abs_dot);
        const uint16_t base_pw_us =
            interp_u16_4pt_u16x(ae_tpsdot_axis_x10, ae_pw_adder_us, tpsdot_u16);
        int32_t pulse =
            (static_cast<int32_t>(base_pw_us) * static_cast<int32_t>(ae_clt_sens[b])) / 8;
        if (pulse > static_cast<int32_t>(ae_max_pw_us)) {
            pulse = static_cast<int32_t>(ae_max_pw_us);
        }
        // Tip-out (DE): mesma magnitude, sinal negativo (enleanment).
        // Authority DE = 50% do AE tip-in — evita lean hole agressivo.
        if (tip_out) {
            pulse = -(pulse / 2);
        }
        g_ae_pulse_us = pulse;
        g_ae_decay_cycles = static_cast<uint8_t>(taper);
        return g_ae_pulse_us;
    }

    if (g_ae_decay_cycles > 0u) {
        const uint16_t taper = ae_taper_cycles > 255u
            ? 255u
            : (ae_taper_cycles == 0u ? 1u : ae_taper_cycles);
        --g_ae_decay_cycles;
        return (g_ae_pulse_us * static_cast<int32_t>(g_ae_decay_cycles)) /
               static_cast<int32_t>(taper);
    }

    g_ae_pulse_us = 0;
    return 0;
}

int32_t calc_ae_pw_us(uint16_t tps_now_x10,
                      uint16_t tps_prev_x10,
                      uint16_t dt_ms,
                      int16_t clt_x10) noexcept {
    if (dt_ms == 0u) {
        return 0;
    }

    // Signed ΔTPS → tip-in (AE) e tip-out (DE).
    const int32_t delta_tps_x10 =
        static_cast<int32_t>(tps_now_x10) - static_cast<int32_t>(tps_prev_x10);

    // (%×10)/ms → %/s×10
    int32_t tpsdot_x10 =
        (delta_tps_x10 * 1000) / static_cast<int32_t>(dt_ms);
    if (tpsdot_x10 > 1000) {
        tpsdot_x10 = 1000;
    }
    if (tpsdot_x10 < -1000) {
        tpsdot_x10 = -1000;
    }
    return calc_ae_pw_from_tpsdot(static_cast<int16_t>(tpsdot_x10), clt_x10);
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

// Contexto extra do DFCO (setters — a assinatura do update fica estável).
namespace {
uint16_t g_dfco_map_bar_x100   = 0u;
uint8_t  g_dfco_gear           = 0u;
bool     g_dfco_gear_seen      = false;
bool     g_dfco_gear_changed   = false;  // já houve ≥1 troca (valida timestamp)
uint32_t g_dfco_gear_change_ms = 0u;
uint32_t g_dfco_now_ms         = 0u;
}  // namespace

void fuel_decel_cut_notify_map(uint16_t map_bar_x100) noexcept {
    g_dfco_map_bar_x100 = map_bar_x100;
}

void fuel_decel_cut_notify_gear(uint8_t gear, uint32_t now_ms) noexcept {
    g_dfco_now_ms = now_ms;
    if (!g_dfco_gear_seen) {
        g_dfco_gear_seen = true;
        g_dfco_gear = gear;
        return;
    }
    if (gear != g_dfco_gear) {
        g_dfco_gear = gear;
        g_dfco_gear_changed = true;
        g_dfco_gear_change_ms = now_ms;
        // Troca de marcha derruba um corte activo (anti-jerk na transmissão).
        if (decel_cut_gear_inhibit_ms10 != 0u) {
            g_decel_cut = false;
        }
    }
}

bool fuel_decel_cut_update(uint32_t rpm_x10,
                           uint16_t tps_pct_x10,
                           int16_t clt_x10) noexcept {
    const bool throttle_closed = tps_pct_x10 <= decel_cut_tps_threshold_x10;
    const bool engine_warm     = clt_x10 >= decel_cut_min_clt_x10;
    // Gate de MAP: só corta com vácuo real (carga baixa de facto). 0 = off.
    const bool map_ok = (decel_cut_map_max_bar_x100 == 0u) ||
                        (g_dfco_map_bar_x100 <= decel_cut_map_max_bar_x100);
    // Inibição pós-troca de marcha (aritmética circular de uint32 em ms).
    const uint32_t inhibit_ms =
        static_cast<uint32_t>(decel_cut_gear_inhibit_ms10) * 10u;
    const bool shift_inhibit = (inhibit_ms != 0u) && g_dfco_gear_changed &&
        ((g_dfco_now_ms - g_dfco_gear_change_ms) < inhibit_ms);

    if (!g_decel_cut) {
        if (throttle_closed && engine_warm && map_ok && !shift_inhibit &&
            rpm_x10 >= decel_cut_entry_rpm_x10) {
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
    g_dfco_map_bar_x100 = 0u;
    g_dfco_gear_seen = false;
    g_dfco_gear_changed = false;
    g_dfco_gear_change_ms = 0u;
    g_dfco_now_ms = 0u;
}

// ── Protecção de duty do injector (FOME #215) ────────────────────────────────
namespace {
uint16_t g_inj_duty_pct_x10 = 0u;
uint32_t g_inj_duty_over_ms = 0u;
bool     g_inj_duty_cut     = false;
}  // namespace

bool fuel_inj_duty_update(uint32_t pw_us, uint32_t rpm_x10,
                          uint16_t dt_ms) noexcept {
    // Duty vs ciclo de 720° (1 injecção/injector/ciclo, sequencial):
    // ciclo_us = 1.2e9 / rpm_x10 → duty%×10 = pw_us × rpm_x10 / 1.2e6.
    const uint64_t duty_x10 =
        (static_cast<uint64_t>(pw_us) * rpm_x10) / 1200000u;
    g_inj_duty_pct_x10 = static_cast<uint16_t>(duty_x10 > 65535u ? 65535u
                                                                 : duty_x10);
    const uint8_t max_pct = inj_duty_max_pct;
    if (max_pct == 0u || rpm_x10 == 0u) {
        g_inj_duty_cut = false;
        g_inj_duty_over_ms = 0u;
        return false;
    }
    const uint16_t limit_x10 = static_cast<uint16_t>(max_pct) * 10u;
    const uint32_t tol_ms = (inj_duty_tol_ms10 != 0u)
        ? static_cast<uint32_t>(inj_duty_tol_ms10) * 10u
        : 300u;  // habilitado sem tolerância explícita → 300 ms
    if (!g_inj_duty_cut) {
        if (g_inj_duty_pct_x10 > limit_x10) {
            g_inj_duty_over_ms += dt_ms;
            if (g_inj_duty_over_ms >= tol_ms) {
                g_inj_duty_cut = true;
            }
        } else {
            g_inj_duty_over_ms = 0u;
        }
    } else {
        // Retoma com histerese de 5%: o PW comandado continua a ser calculado
        // durante o corte (a mask é que suprime), logo o duty pedido cai
        // quando o RPM/carga descem — não há deadlock.
        const uint16_t resume_x10 = (limit_x10 > 50u) ? limit_x10 - 50u : 0u;
        if (g_inj_duty_pct_x10 <= resume_x10) {
            g_inj_duty_cut = false;
            g_inj_duty_over_ms = 0u;
        }
    }
    return g_inj_duty_cut;
}

bool fuel_inj_duty_cut_active() noexcept {
    return g_inj_duty_cut;
}

uint16_t fuel_inj_duty_pct_x10() noexcept {
    return g_inj_duty_pct_x10;
}

void fuel_inj_duty_reset() noexcept {
    g_inj_duty_pct_x10 = 0u;
    g_inj_duty_over_ms = 0u;
    g_inj_duty_cut = false;
}

uint16_t calc_eoi_lead_deg(uint32_t rpm_x10) noexcept
{
    const uint16_t main_deg = cfg::g_eng_cfg.default_eoi_lead_deg;
    const uint16_t lo = eoi_blend_rpm_lo;
    const uint16_t hi = eoi_blend_rpm_hi;

    if (hi <= lo) {  // desligado (inclui 0/0 — page 0 antiga zerada)
        return (main_deg > 719u) ? 719u : main_deg;
    }

    const uint32_t rpm = rpm_x10 / 10u;
    uint16_t idle = eoi_idle_deg;
    if (idle > 719u) { idle = 719u; }

    if (rpm <= lo) { return idle; }
    if (rpm >= hi) { return (main_deg > 719u) ? 719u : main_deg; }

    // Interpolação linear em int32: |main−idle| ≤ 719 e (rpm−lo) < 65535
    // → |produto| < 47.2M — folga ampla em int32. Divisor > 0 garantido
    // pelo gate hi > lo acima.
    const int32_t span   = static_cast<int32_t>(main_deg) - static_cast<int32_t>(idle);
    const int32_t num    = span * static_cast<int32_t>(rpm - lo);
    const int32_t eoi    = static_cast<int32_t>(idle) + num / static_cast<int32_t>(hi - lo);

    if (eoi < 0)    { return 0u; }
    if (eoi > 719)  { return 719u; }
    return static_cast<uint16_t>(eoi);
}

}  // namespace ems::engine
