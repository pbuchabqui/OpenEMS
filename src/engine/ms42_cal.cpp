#include "engine/ms42_cal.h"

#include <cstring>

namespace ems::engine {

Ms42Cal ms42 = {};

namespace {

// Visita os campos na ordem do layout da page0. Acrescentar campos só no fim
// (e subir kMs42BlockVer se a semântica de um campo existente mudar).
template <typename V>
void visit_fields(Ms42Cal& c, V& v) noexcept {
    v(c.vvt_enable); v(c.vvt_kp_x10); v(c.vvt_ki_x100); v(c.vvt_hold_duty_pct);
    v(c.vvt_cam_ref_x10); v(c.vvt_min_clt_x10); v(c.vvt_max_adv_deg);
    v(c.vvt_rpm_axis); v(c.vvt_load_axis); v(c.vvt_target_deg);

    v(c.vbatt_filter_shift); v(c.vbatt_low_mv); v(c.vbatt_high_mv);

    v(c.dfco_clt_axis_x10); v(c.dfco_entry_rpm_x10);
    v(c.dfco_hyst_rpm_x10); v(c.dfco_entry_delay_ms);

    v(c.rev_roll_enable); v(c.rev_roll_window_rpm_x10);

    v(c.spark_grad_inc_x10); v(c.spark_grad_dec_x10);

    v(c.crank_taper_cycles); v(c.crank_taper_end_pct);
    v(c.hot_restart_clt_x10); v(c.hot_restart_pct);
    v(c.crank_baro_enable); v(c.afterstart_by_cycles);
    v(c.crank_clt_axis_x10); v(c.crank_mult_x256);
    v(c.afterstart_start_x256); v(c.afterstart_ms); v(c.afterstart_cycles);

    v(c.knock_step_x10); v(c.knock_max_x10); v(c.knock_recovery_x10);
    v(c.knock_clean_cycles); v(c.knock_rel_enable); v(c.knock_noise_shift);
    v(c.knock_gain_rpm_axis); v(c.knock_gain_x10);

    v(c.idle_kp_x10); v(c.idle_persist); v(c.idle_ff_x10);
    v(c.cat_heat_rpm_x10); v(c.cat_heat_s);

    v(c.stft_rpm_axis); v(c.stft_load_axis); v(c.stft_gain_pct);
    v(c.stft_min_rpm_x10);

    v(c.fan_on_x10); v(c.fan_off_x10);
}

// Header: magic (u16 LE) + versão do bloco.
constexpr uint16_t kHdrLen = 3u;

struct Sizer {
    uint16_t n = 0u;
    template <typename T> void operator()(T&) noexcept { n = static_cast<uint16_t>(n + sizeof(T)); }
};

struct Writer {
    uint8_t* p;
    uint16_t off;
    uint16_t len;
    template <typename T> void operator()(T& x) noexcept {
        if (static_cast<uint32_t>(off) + sizeof(T) <= len) {
            std::memcpy(p + off, &x, sizeof(T));  // little-endian (host e ARM)
        }
        off = static_cast<uint16_t>(off + sizeof(T));
    }
};

struct Reader {
    const uint8_t* p;
    uint16_t off;
    uint16_t len;
    template <typename T> void operator()(T& x) noexcept {
        if (static_cast<uint32_t>(off) + sizeof(T) <= len) {
            std::memcpy(&x, p + off, sizeof(T));
        }
        off = static_cast<uint16_t>(off + sizeof(T));
    }
};

template <typename T>
void clamp_field(T& v, T lo, T hi) noexcept {
    if (v < lo) { v = lo; }
    if (v > hi) { v = hi; }
}

template <typename T, uint8_t N>
bool axis_ascending(const T (&a)[N]) noexcept {
    for (uint8_t i = 1u; i < N; ++i) {
        if (!(a[i] > a[i - 1u])) { return false; }
    }
    return true;
}

void fill_defaults(Ms42Cal& c) noexcept;

void sanitize(Ms42Cal& c) noexcept {
    Ms42Cal d;
    fill_defaults(d);
    c.vvt_enable = (c.vvt_enable != 0u) ? 1u : 0u;
    clamp_field<uint8_t>(c.vvt_hold_duty_pct, 0u, 100u);
    clamp_field<uint16_t>(c.vvt_cam_ref_x10, 0u, 3599u);
    clamp_field<uint8_t>(c.vvt_max_adv_deg, 0u, 60u);
    if (!axis_ascending(c.vvt_rpm_axis))  { std::memcpy(c.vvt_rpm_axis, d.vvt_rpm_axis, sizeof(c.vvt_rpm_axis)); }
    if (!axis_ascending(c.vvt_load_axis)) { std::memcpy(c.vvt_load_axis, d.vvt_load_axis, sizeof(c.vvt_load_axis)); }

    clamp_field<uint8_t>(c.vbatt_filter_shift, 0u, 6u);

    if (!axis_ascending(c.dfco_clt_axis_x10)) {
        std::memcpy(c.dfco_clt_axis_x10, d.dfco_clt_axis_x10, sizeof(c.dfco_clt_axis_x10));
    }
    clamp_field<uint16_t>(c.dfco_entry_delay_ms, 0u, 5000u);

    c.rev_roll_enable = (c.rev_roll_enable != 0u) ? 1u : 0u;
    clamp_field<uint16_t>(c.rev_roll_window_rpm_x10, 0u, 10000u);

    clamp_field<uint8_t>(c.crank_taper_end_pct, 10u, 100u);
    clamp_field<uint8_t>(c.hot_restart_pct, 50u, 200u);
    c.crank_baro_enable = (c.crank_baro_enable != 0u) ? 1u : 0u;
    c.afterstart_by_cycles = (c.afterstart_by_cycles != 0u) ? 1u : 0u;
    if (!axis_ascending(c.crank_clt_axis_x10)) {
        std::memcpy(c.crank_clt_axis_x10, d.crank_clt_axis_x10, sizeof(c.crank_clt_axis_x10));
    }
    for (uint8_t i = 0u; i < kCrankCalPts; ++i) {
        clamp_field<uint16_t>(c.crank_mult_x256[i], 0u, 2048u);        // ≤ 8×
        clamp_field<uint16_t>(c.afterstart_start_x256[i], 0u, 1024u);  // ≤ +400 %
        clamp_field<uint16_t>(c.afterstart_ms[i], 0u, 30000u);
        clamp_field<uint16_t>(c.afterstart_cycles[i], 0u, 2000u);
    }

    if (c.knock_step_x10 == 0u) { c.knock_step_x10 = d.knock_step_x10; }
    if (c.knock_max_x10 == 0u)  { c.knock_max_x10 = d.knock_max_x10; }
    if (c.knock_clean_cycles == 0u) { c.knock_clean_cycles = d.knock_clean_cycles; }
    c.knock_rel_enable = (c.knock_rel_enable != 0u) ? 1u : 0u;
    clamp_field<uint8_t>(c.knock_noise_shift, 1u, 7u);
    if (!axis_ascending(c.knock_gain_rpm_axis)) {
        std::memcpy(c.knock_gain_rpm_axis, d.knock_gain_rpm_axis, sizeof(c.knock_gain_rpm_axis));
    }
    for (uint8_t i = 0u; i < kKnockGainPts; ++i) {
        clamp_field<uint8_t>(c.knock_gain_x10[i], 11u, 100u);  // > 1,0×
    }

    c.idle_persist = (c.idle_persist != 0u) ? 1u : 0u;
    for (uint8_t i = 0u; i < kIdleFfPts; ++i) {
        clamp_field<uint16_t>(c.idle_ff_x10[i], 0u, 1000u);
    }
    clamp_field<uint16_t>(c.cat_heat_rpm_x10, 0u, 5000u);

    if (!axis_ascending(c.stft_rpm_axis))  { std::memcpy(c.stft_rpm_axis, d.stft_rpm_axis, sizeof(c.stft_rpm_axis)); }
    if (!axis_ascending(c.stft_load_axis)) { std::memcpy(c.stft_load_axis, d.stft_load_axis, sizeof(c.stft_load_axis)); }
    for (uint8_t y = 0u; y < kStftGainPts; ++y) {
        for (uint8_t x = 0u; x < kStftGainPts; ++x) {
            clamp_field<uint8_t>(c.stft_gain_pct[y][x], 0u, 200u);
        }
    }

    if (c.fan_off_x10 >= c.fan_on_x10) {
        c.fan_on_x10 = d.fan_on_x10;
        c.fan_off_x10 = d.fan_off_x10;
    }
}

void fill_defaults(Ms42Cal& c) noexcept {
    c = Ms42Cal{};

    c.vvt_enable = 0u;
    c.vvt_kp_x10 = 12u;
    c.vvt_ki_x100 = 5u;
    c.vvt_hold_duty_pct = 50u;
    c.vvt_cam_ref_x10 = 0u;
    c.vvt_min_clt_x10 = 600;
    c.vvt_max_adv_deg = 40u;
    static constexpr uint8_t kVvtRpm[kVvtCalPts]  = {10u, 20u, 30u, 40u, 55u, 70u};
    static constexpr uint8_t kVvtLoad[kVvtCalPts] = {30u, 45u, 60u, 75u, 90u, 100u};
    // Admissão: pouco avanço em vazio/alta rotação, máximo no binário médio.
    static constexpr uint8_t kVvtTgt[kVvtCalPts][kVvtCalPts] = {
        { 0u, 10u, 15u, 15u, 10u,  5u},
        { 0u, 15u, 20u, 20u, 15u, 10u},
        { 5u, 20u, 30u, 25u, 20u, 10u},
        {10u, 25u, 35u, 30u, 20u, 10u},
        {15u, 30u, 40u, 35u, 25u, 15u},
        {15u, 30u, 40u, 35u, 25u, 15u},
    };
    std::memcpy(c.vvt_rpm_axis, kVvtRpm, sizeof(kVvtRpm));
    std::memcpy(c.vvt_load_axis, kVvtLoad, sizeof(kVvtLoad));
    std::memcpy(c.vvt_target_deg, kVvtTgt, sizeof(kVvtTgt));

    c.vbatt_filter_shift = 2u;   // a 100 ms: τ ≈ 0,35 s (MS42 média 8 amostras rápidas)
    c.vbatt_low_mv = 10500u;
    c.vbatt_high_mv = 16000u;

    static constexpr int16_t kDfcoClt[kDfcoCltPts] = {-300, 0, 400, 800};
    std::memcpy(c.dfco_clt_axis_x10, kDfcoClt, sizeof(kDfcoClt));
    // dfco_entry_rpm_x10 = 0 → escalares decel_cut_entry/exit (comportamento antigo)
    c.dfco_hyst_rpm_x10 = 3000u;
    c.dfco_entry_delay_ms = 0u;

    c.rev_roll_enable = 0u;
    c.rev_roll_window_rpm_x10 = 2000u;

    c.spark_grad_inc_x10 = 0u;
    c.spark_grad_dec_x10 = 0u;

    c.crank_taper_cycles = 0u;
    c.crank_taper_end_pct = 70u;
    c.hot_restart_clt_x10 = 900;
    c.hot_restart_pct = 100u;
    c.crank_baro_enable = 0u;
    c.afterstart_by_cycles = 0u;
    // Valores idênticos aos antigos constexpr de quick_crank.cpp.
    static constexpr int16_t  kCrClt[kCrankCalPts]  = {-400, 0, 200, 400, 700, 900, 1100};
    static constexpr uint16_t kCrMult[kCrankCalPts] = {768u, 614u, 512u, 435u, 358u, 320u, 294u};
    static constexpr uint16_t kAsStart[kCrankCalPts] = {346u, 333u, 320u, 307u, 294u, 281u, 269u};
    static constexpr uint16_t kAsMs[kCrankCalPts] = {2400u, 2000u, 1700u, 1400u, 1000u, 700u, 500u};
    // Equivalente a ~1000 rpm (1 ciclo = 120 ms).
    static constexpr uint16_t kAsCyc[kCrankCalPts] = {20u, 17u, 14u, 12u, 8u, 6u, 4u};
    std::memcpy(c.crank_clt_axis_x10, kCrClt, sizeof(kCrClt));
    std::memcpy(c.crank_mult_x256, kCrMult, sizeof(kCrMult));
    std::memcpy(c.afterstart_start_x256, kAsStart, sizeof(kAsStart));
    std::memcpy(c.afterstart_ms, kAsMs, sizeof(kAsMs));
    std::memcpy(c.afterstart_cycles, kAsCyc, sizeof(kAsCyc));

    c.knock_step_x10 = 20u;
    c.knock_max_x10 = 100u;
    c.knock_recovery_x10 = 1u;
    c.knock_clean_cycles = 10u;
    c.knock_rel_enable = 0u;
    c.knock_noise_shift = 3u;
    static constexpr uint8_t kKnRpm[kKnockGainPts] = {10u, 30u, 50u, 70u};
    static constexpr uint8_t kKnGain[kKnockGainPts] = {25u, 22u, 20u, 18u};
    std::memcpy(c.knock_gain_rpm_axis, kKnRpm, sizeof(kKnRpm));
    std::memcpy(c.knock_gain_x10, kKnGain, sizeof(kKnGain));

    c.idle_kp_x10 = 0u;
    c.idle_persist = 0u;
    // idle_ff_x10 = 0 → sem feed-forward
    c.cat_heat_rpm_x10 = 0u;
    c.cat_heat_s = 0u;

    static constexpr uint8_t kStRpm[kStftGainPts]  = {8u, 15u, 30u, 50u};
    static constexpr uint8_t kStLoad[kStftGainPts] = {30u, 50u, 70u, 100u};
    std::memcpy(c.stft_rpm_axis, kStRpm, sizeof(kStRpm));
    std::memcpy(c.stft_load_axis, kStLoad, sizeof(kStLoad));
    std::memset(c.stft_gain_pct, 100, sizeof(c.stft_gain_pct));
    c.stft_min_rpm_x10 = 0u;

    c.fan_on_x10 = 950;
    c.fan_off_x10 = 900;
}

template <typename V>
void visit_ext_fields(Ms42Ext& c, V& v) noexcept {
    v(c.cat_heat_retard_x10); v(c.cat_heat_clt_max_c);
    v(c.as_pw_fall_cycles); v(c.as_pw_fall_cold_pct); v(c.as_pw_fall_hot_pct);
    v(c.misfire_rpm_axis); v(c.misfire_map_axis); v(c.misfire_excess_q8);
}

static_assert(2u + sizeof(Ms42Ext) <= kMs42ExtLen, "extensão MS42 não cabe nos 32 bytes da page5");

void fill_ext_defaults(Ms42Ext& c) noexcept {
    c = Ms42Ext{};
    c.cat_heat_retard_x10 = 0u;
    c.cat_heat_clt_max_c = 60;
    c.as_pw_fall_cycles = 0u;
    c.as_pw_fall_cold_pct = 5u;
    c.as_pw_fall_hot_pct = 15u;
    static constexpr uint8_t kMfRpm[kMisfireCalPts] = {10u, 25u, 45u, 65u};
    static constexpr uint8_t kMfMap[kMisfireCalPts] = {30u, 50u, 75u, 100u};
    std::memcpy(c.misfire_rpm_axis, kMfRpm, sizeof(kMfRpm));
    std::memcpy(c.misfire_map_axis, kMfMap, sizeof(kMfMap));
    // 31 = 287/256 − 1: o limiar único anterior (kMisfireThresholdQ8).
    std::memset(c.misfire_excess_q8, 31, sizeof(c.misfire_excess_q8));
}

void sanitize_ext(Ms42Ext& c) noexcept {
    Ms42Ext d;
    fill_ext_defaults(d);
    clamp_field<uint8_t>(c.cat_heat_retard_x10, 0u, 150u);
    clamp_field<uint8_t>(c.as_pw_fall_cold_pct, 1u, 100u);
    clamp_field<uint8_t>(c.as_pw_fall_hot_pct, 1u, 100u);
    if (!axis_ascending(c.misfire_rpm_axis)) {
        std::memcpy(c.misfire_rpm_axis, d.misfire_rpm_axis, sizeof(c.misfire_rpm_axis));
    }
    if (!axis_ascending(c.misfire_map_axis)) {
        std::memcpy(c.misfire_map_axis, d.misfire_map_axis, sizeof(c.misfire_map_axis));
    }
    for (uint8_t y = 0u; y < kMisfireCalPts; ++y) {
        for (uint8_t x = 0u; x < kMisfireCalPts; ++x) {
            clamp_field<uint8_t>(c.misfire_excess_q8[y][x], 3u, 255u);  // ≥ 1,01×
        }
    }
}

// Defaults já na inicialização estática: o firmware e os testes de host que
// nunca aplicam a page0 veem os valores de compilação, não zeros.
struct DefaultsInit {
    DefaultsInit() noexcept { fill_defaults(ms42); fill_ext_defaults(ms42x); }
};

}  // namespace

Ms42Ext ms42x = {};

namespace {
// Depois de ms42x: a ordem de inicialização dentro da unidade é a de definição.
DefaultsInit s_defaults_init;
}  // namespace

void ms42_ext_defaults() noexcept { fill_ext_defaults(ms42x); }

void ms42_ext_serialize_to_page5(uint8_t* page5, uint16_t len) noexcept {
    if (page5 == nullptr || len < kMs42ExtPage5Off + kMs42ExtLen) { return; }
    std::memset(page5 + kMs42ExtPage5Off, 0, kMs42ExtLen);
    std::memcpy(page5 + kMs42ExtPage5Off, &kMs42ExtMagic, 2u);
    Writer w{page5, static_cast<uint16_t>(kMs42ExtPage5Off + 2u),
             static_cast<uint16_t>(kMs42ExtPage5Off + kMs42ExtLen)};
    visit_ext_fields(ms42x, w);
}

void ms42_ext_apply_page5(const uint8_t* page5, uint16_t len) noexcept {
    uint16_t magic = 0u;
    if (page5 != nullptr && len >= kMs42ExtPage5Off + kMs42ExtLen) {
        std::memcpy(&magic, page5 + kMs42ExtPage5Off, 2u);
    }
    if (magic != kMs42ExtMagic) {
        ms42_ext_defaults();
        return;
    }
    Ms42Ext c = {};
    Reader r{page5, static_cast<uint16_t>(kMs42ExtPage5Off + 2u),
             static_cast<uint16_t>(kMs42ExtPage5Off + kMs42ExtLen)};
    visit_ext_fields(c, r);
    sanitize_ext(c);
    ms42x = c;
}

void ms42_cal_defaults() noexcept { fill_defaults(ms42); }

uint16_t ms42_cal_page0_len() noexcept {
    Sizer s;
    visit_fields(ms42, s);
    return static_cast<uint16_t>(kHdrLen + s.n);
}

void ms42_serialize_to_page0(uint8_t* page0, uint16_t len) noexcept {
    if (page0 == nullptr || len < kMs42Page0Off + kHdrLen) { return; }
    std::memcpy(page0 + kMs42Page0Off, &kMs42Magic, 2u);
    page0[kMs42Page0Off + 2u] = kMs42BlockVer;
    Writer w{page0, static_cast<uint16_t>(kMs42Page0Off + kHdrLen), len};
    visit_fields(ms42, w);
}

void ms42_apply_page0(const uint8_t* page0, uint16_t len) noexcept {
    if (page0 == nullptr || len < kMs42Page0Off + kHdrLen) {
        ms42_cal_defaults();
        return;
    }
    uint16_t magic = 0u;
    std::memcpy(&magic, page0 + kMs42Page0Off, 2u);
    if (magic != kMs42Magic || page0[kMs42Page0Off + 2u] != kMs42BlockVer) {
        ms42_cal_defaults();
        return;
    }
    Ms42Cal c = {};
    Reader r{page0, static_cast<uint16_t>(kMs42Page0Off + kHdrLen), len};
    visit_fields(c, r);
    sanitize(c);
    ms42 = c;
}

uint16_t ms42_interp_u8(const uint8_t* axis, const uint8_t* vals, uint8_t n,
                        uint16_t x) noexcept {
    if (n == 0u) { return 0u; }
    if (x <= axis[0]) { return vals[0]; }
    for (uint8_t i = 1u; i < n; ++i) {
        if (x <= axis[i]) {
            const int32_t x0 = axis[i - 1u];
            const int32_t x1 = axis[i];
            const int32_t y0 = vals[i - 1u];
            const int32_t y1 = vals[i];
            const int32_t y = y0 + ((y1 - y0) * (static_cast<int32_t>(x) - x0)) / (x1 - x0);
            return static_cast<uint16_t>(y < 0 ? 0 : y);
        }
    }
    return vals[n - 1u];
}

uint16_t ms42_interp_u8_2d(const uint8_t* x_axis, const uint8_t* y_axis,
                           const uint8_t* table, uint8_t n,
                           uint16_t x, uint16_t y) noexcept {
    constexpr uint8_t kMaxN = 8u;
    if (n == 0u || n > kMaxN) { return 0u; }
    // Resolve x em cada linha, depois interpola em y sobre a coluna obtida.
    uint8_t col[kMaxN];
    for (uint8_t r = 0u; r < n; ++r) {
        col[r] = static_cast<uint8_t>(ms42_interp_u8(x_axis, table + r * n, n, x));
    }
    return ms42_interp_u8(y_axis, col, n, y);
}

}  // namespace ems::engine
