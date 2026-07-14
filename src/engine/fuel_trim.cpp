#include "engine/fuel_trim.h"

#include "engine/calibration.h"
#include "engine/engine_config.h"
#include "engine/math_utils.h"
#include "engine/table3d.h"
#include "hal/flash.h"

#include <cstdint>

namespace ems::engine {
void fuel_ae_reset() noexcept;
void fuel_decel_cut_reset() noexcept;
}

static volatile uint32_t g_nvm_write_faults = 0u;

namespace {

using ems::engine::clamp_i16;
using ems::engine::clamp_u16;

constexpr uint8_t kLambdaHistorySize = 16u;

struct LambdaHistorySample {
    uint32_t time_ms;
    uint32_t rpm_x10;
    uint16_t map_bar_x100;
    int16_t lambda_target_x1000;
    bool valid;
};

int16_t g_stft_pct_x10 = 0;

int16_t g_ltft_pct_x10[ems::engine::kTableAxisSize][ems::engine::kTableAxisSize] = {};
// LTFT aditivo: offset em µs, sub-grid do principal (rpm_idx>>1, map_idx>>1)
int16_t g_ltft_add_us[ems::engine::kLtftAddAxisSize][ems::engine::kLtftAddAxisSize] = {};

// Lockstep HAL↔engine: as dimensões NVM (flash.h — HAL não vê headers do
// engine) têm de espelhar as do grid; este TU vê ambos os headers.
static_assert(ems::hal::kNvmLtftDim == ems::engine::kTableAxisSize,
              "kNvmLtftDim deve espelhar kTableAxisSize");
static_assert(ems::hal::kNvmLtftAddDim == ems::engine::kLtftAddAxisSize,
              "kNvmLtftAddDim deve espelhar kLtftAddAxisSize");
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

int16_t ltft_add_clamp() noexcept {
    const uint16_t c = ems::engine::ltft_add_clamp_us;
    return static_cast<int16_t>((c == 0u) ? 6350u : c);
}

int16_t ltft_mult_clamp() noexcept {
    const uint16_t c = ems::engine::ltft_mult_clamp_pct_x10;
    return static_cast<int16_t>((c == 0u) ? 250u : c);
}

uint8_t ltft_iir_div() noexcept {
    const uint8_t d = ems::engine::ltft_learn_div;
    return (d == 0u) ? 64u : d;
}

// IIR: current + (target−current)/div, com cap opcional de passo (%×10).
int16_t ltft_iir_toward(int16_t current, int16_t target, int16_t clamp_lim) noexcept {
    const uint8_t div = ltft_iir_div();
    int32_t next = static_cast<int32_t>(current) +
        (static_cast<int32_t>(target) - static_cast<int32_t>(current)) /
            static_cast<int32_t>(div);
    const uint16_t max_step = ems::engine::ltft_max_step_x10;
    if (max_step != 0u) {
        const int32_t d = next - static_cast<int32_t>(current);
        const int32_t cap = static_cast<int32_t>(max_step);
        if (d > cap) {
            next = static_cast<int32_t>(current) + cap;
        } else if (d < -cap) {
            next = static_cast<int32_t>(current) - cap;
        }
    }
    return clamp_i16(static_cast<int16_t>(next),
                     static_cast<int16_t>(-clamp_lim), clamp_lim);
}

void fuel_ltft_add_store_cell(uint8_t map_idx, uint8_t rpm_idx, int16_t value_us) noexcept {
    const int16_t lim = ltft_add_clamp();
    const int16_t clamped = clamp_i16(value_us, static_cast<int16_t>(-lim), lim);
    const int16_t rounded = static_cast<int16_t>(
        clamped >= 0 ? (clamped + 25) / 50 : (clamped - 25) / 50);
    const bool ok = ems::hal::nvm_write_ltft_add(
        rpm_idx >> 1u, map_idx >> 1u, static_cast<int8_t>(rounded));
    if (!ok) { ++g_nvm_write_faults; }
}

void fuel_ltft_store_cell(uint8_t map_idx, uint8_t rpm_idx, int16_t value_x10) noexcept {
    const int16_t lim = ltft_mult_clamp();
    const int16_t clamped_x10 =
        clamp_i16(value_x10, static_cast<int16_t>(-lim), lim);
    // NVM int8 %: wire max ±25%; RAM pode usar clamp calibrável maior.
    int16_t store_x10 = clamped_x10;
    if (store_x10 > 250) { store_x10 = 250; }
    if (store_x10 < -250) { store_x10 = -250; }
    const int16_t rounded_pct = (store_x10 >= 0)
        ? static_cast<int16_t>((store_x10 + 5) / 10)
        : static_cast<int16_t>((store_x10 - 5) / 10);
    if (!ems::hal::nvm_write_ltft(rpm_idx, map_idx, static_cast<int8_t>(rounded_pct))) {
        ++g_nvm_write_faults;
    }
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

bool closed_loop_allowed(int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut) noexcept {
    if (ems::engine::closed_loop_enable == 0u) {
        return false;
    }
    return (clt_x10 > 700) && o2_valid && (!ae_active) && (!rev_cut);
}

// Regime estável entre amostras consecutivas em closed-loop.
constexpr uint32_t kLtftAccumMaxRpmDeltaX10 = 2000u;  // 200 RPM
constexpr uint16_t kLtftAccumMaxTpsDeltaX10 = 20u;    // 2.0 %  (APP ou ETB do caller)
// MAP-dot: |ΔMAP| > 8 kPa entre amostras STFT (~100 ms) → freeze LTFT/LEARN.
constexpr uint16_t kLtftAccumMaxMapDeltaBarX100 = 8u;

inline int32_t abs_i32(int32_t v) noexcept {
    return (v < 0) ? -v : v;
}

// Post-start: âncora no 1º instante com CLT+O2 OK (now_ms≠0).
static uint32_t g_cl_warm_since_ms = 0u;
static bool     g_cl_warm_latched  = false;
static uint16_t g_ltft_prev_map_bar_x100 = 0u;
static bool     g_ltft_have_prev_map     = false;

void fuel_closed_loop_timers_reset() noexcept {
    g_cl_warm_since_ms = 0u;
    g_cl_warm_latched  = false;
    g_ltft_prev_map_bar_x100 = 0u;
    g_ltft_have_prev_map     = false;
}

// true se já passou post_start_s desde warm CLT+O2. now_ms==0 → skip (tests).
bool post_start_elapsed(uint32_t now_ms, int16_t clt_x10, bool o2_valid) noexcept {
    if (now_ms == 0u) {
        return true;
    }
    if (!(clt_x10 > 700 && o2_valid)) {
        g_cl_warm_latched = false;
        g_cl_warm_since_ms = 0u;
        return false;
    }
    if (!g_cl_warm_latched) {
        g_cl_warm_latched  = true;
        g_cl_warm_since_ms = now_ms;
    }
    const uint32_t need_ms =
        static_cast<uint32_t>(ems::engine::closed_loop_post_start_s) * 1000u;
    if (need_ms == 0u) {
        return true;
    }
    return (now_ms - g_cl_warm_since_ms) >= need_ms;
}

}  // namespace

namespace ems::engine {

// Integrador em percent×1000 (não ×10): com Ki=0.005 default, um erro de
// λ 0.07 contribui 0.35 x10-unidades/ciclo — em ×10 inteiro truncava a 0.
// Em ems::engine (não anon) porque o comando 'D' exporta p/ diagnóstico.
int32_t g_stft_integrator_x1000 = 0;
// DIAG da malha fechada: por que o update foi bloqueado + último erro visto.
volatile uint32_t g_dbg_stft_blocked_clt = 0u;
volatile uint32_t g_dbg_stft_blocked_o2  = 0u;
volatile uint32_t g_dbg_stft_blocked_ae  = 0u;
volatile uint32_t g_dbg_stft_blocked_cut = 0u;
volatile uint32_t g_dbg_stft_runs        = 0u;
volatile int32_t  g_dbg_stft_last_err    = 0;
volatile uint32_t g_dbg_ltft_accum_accepted = 0u;
volatile uint32_t g_dbg_ltft_accum_rejected = 0u;
volatile uint32_t g_dbg_ltft_accum_commits  = 0u;

LtftCellStats g_ltft_stats[kTableAxisSize][kTableAxisSize] = {};

static uint32_t g_ltft_accum_prev_rpm_x10 = 0u;
static uint16_t g_ltft_accum_prev_tps_x10 = 0u;
static bool     g_ltft_accum_have_prev    = false;
static volatile bool g_ltft_ve_burn_pending = false;

bool fuel_ltft_ve_burn_pending() noexcept {
    return g_ltft_ve_burn_pending;
}

void fuel_ltft_ve_burn_clear() noexcept {
    g_ltft_ve_burn_pending = false;
}

void fuel_ltft_accum_export(uint8_t* dst, uint16_t cap) noexcept {
    if (dst == nullptr || cap < kLtftAccumPageSize) {
        return;
    }
    for (uint8_t m = 0u; m < kTableAxisSize; ++m) {
        for (uint8_t r = 0u; r < kTableAxisSize; ++r) {
            const uint16_t idx =
                static_cast<uint16_t>(m) * kTableAxisSize + r;
            // hits nos 7 bits baixos (sat. 127); bit7 = ready (fonte única).
            const uint16_t hits = g_ltft_stats[m][r].hits;
            uint8_t wire = static_cast<uint8_t>((hits > 127u) ? 127u : hits);
            if (fuel_ltft_accum_cell_ready(m, r)) {
                wire = static_cast<uint8_t>(wire | 0x80u);
            }
            dst[idx] = wire;

            int16_t mean = fuel_ltft_accum_mean_stft_x10(m, r);
            if (mean > 127) {
                mean = 127;
            } else if (mean < -128) {
                mean = -128;
            }
            dst[kTableCells + idx] =
                static_cast<uint8_t>(static_cast<int8_t>(mean));
        }
    }
}

void fuel_reset_ltft() noexcept {
    fuel_ltft_accum_reset();
    for (uint8_t y = 0u; y < kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < kTableAxisSize; ++x) {
            g_ltft_pct_x10[y][x] = 0;
            ems::hal::nvm_write_ltft(x, y, 0);
        }
    }
    for (uint8_t y = 0u; y < kLtftAddAxisSize; ++y) {
        for (uint8_t x = 0u; x < kLtftAddAxisSize; ++x) {
            g_ltft_add_us[y][x] = 0;
            ems::hal::nvm_write_ltft_add(x, y, 0);
        }
    }
    g_stft_pct_x10 = 0;
    g_stft_integrator_x1000 = 0;
}

void fuel_reset_adaptives() noexcept {
    g_stft_pct_x10 = 0;
    g_stft_integrator_x1000 = 0;
    fuel_ae_reset();
    fuel_decel_cut_reset();
    fuel_lambda_delay_reset();
    fuel_ltft_accum_reset();
    fuel_closed_loop_timers_reset();

    for (uint8_t y = 0u; y < kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < kTableAxisSize; ++x) {
            g_ltft_pct_x10[y][x] = fuel_ltft_load_cell(y, x);
        }
    }
    for (uint8_t y = 0u; y < kLtftAddAxisSize; ++y) {
        for (uint8_t x = 0u; x < kLtftAddAxisSize; ++x) {
            // Carrega via índice do grid principal equivalente (dobra o índice)
            g_ltft_add_us[y][x] = fuel_ltft_add_load_cell(
                static_cast<uint8_t>(y << 1u), static_cast<uint8_t>(x << 1u));
        }
    }
}

void fuel_reset_learn_session() noexcept {
    // Uma entrada para 'Z': zera shadows LTFT (dirty NVM adaptativo), STFT,
    // acumulador LEARN, AE/delay e contadores. Não burn de page0/VE.
    fuel_reset_ltft();
    fuel_ae_reset();
    fuel_decel_cut_reset();
    fuel_lambda_delay_reset();
    fuel_closed_loop_timers_reset();
    // LTFT já está a zero em RAM e shadow; não re-ler NVM.
    g_dbg_ltft_accum_accepted = 0u;
    g_dbg_ltft_accum_rejected = 0u;
    g_dbg_ltft_accum_commits  = 0u;
    g_dbg_stft_runs = 0u;
    g_dbg_stft_last_err = 0;
    fuel_ltft_ve_burn_clear();
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

bool ltft_accum_sample_valid(uint32_t rpm_x10,
                             uint32_t prev_rpm_x10,
                             uint16_t tps_x10,
                             uint16_t prev_tps_x10,
                             bool have_prev_sample,
                             int16_t lambda_target_x1000,
                             int16_t lambda_measured_x1000,
                             int16_t stft_pct_x10,
                             int16_t clt_x10,
                             bool o2_valid,
                             bool ae_active,
                             bool rev_cut) noexcept {
    // Bake-in: só acumula com malha fechada, regime estável e λ convergida.
    // |err|≈0 é válido (trim STFT estável no alvo) — não exigir erro residual.
    if (!closed_loop_allowed(clt_x10, o2_valid, ae_active, rev_cut)) {
        return false;
    }
    if (!have_prev_sample) {
        return false;
    }
    if (abs_i32(static_cast<int32_t>(rpm_x10) - static_cast<int32_t>(prev_rpm_x10)) >
        static_cast<int32_t>(kLtftAccumMaxRpmDeltaX10)) {
        return false;
    }
    if (abs_i32(static_cast<int32_t>(tps_x10) - static_cast<int32_t>(prev_tps_x10)) >
        static_cast<int32_t>(kLtftAccumMaxTpsDeltaX10)) {
        return false;
    }
    if (lambda_target_x1000 < 650 || lambda_target_x1000 > 1200) {
        return false;
    }
    if (lambda_measured_x1000 < 650 || lambda_measured_x1000 > 1200) {
        return false;
    }
    const int16_t err_x1000 =
        static_cast<int16_t>(lambda_measured_x1000 - lambda_target_x1000);
    // Rejeita outlier / ainda a convergir — não rejeita erro ~0.
    if (abs_i32(err_x1000) > kLtftAccumMaxErrX1000) {
        return false;
    }
    // Rejeita STFT saturado (não fiável para bake-in); vieses reais até 15% OK.
    if (abs_i32(stft_pct_x10) > kLtftAccumMaxStftX10) {
        return false;
    }
    return true;
}

void fuel_ltft_accum_reset() noexcept {
    for (uint8_t y = 0u; y < kTableAxisSize; ++y) {
        for (uint8_t x = 0u; x < kTableAxisSize; ++x) {
            g_ltft_stats[y][x] = {};
        }
    }
    g_ltft_accum_prev_rpm_x10 = 0u;
    g_ltft_accum_prev_tps_x10 = 0u;
    g_ltft_accum_have_prev    = false;
}

void fuel_ltft_accum_reset_cell(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        return;
    }
    g_ltft_stats[map_idx][rpm_idx] = {};
}

uint16_t fuel_ltft_accum_hits(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        return 0u;
    }
    return g_ltft_stats[map_idx][rpm_idx].hits;
}

bool fuel_ltft_accum_cell_ready(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        return false;
    }
    const LtftCellStats& cell = g_ltft_stats[map_idx][rpm_idx];
    if (cell.hits < kLtftAccumReadyHits) {
        return false;
    }
    const int16_t mean_stft =
        static_cast<int16_t>(cell.sum_stft_x10 / static_cast<int32_t>(cell.hits));
    const int16_t mean_err =
        static_cast<int16_t>(cell.sum_err_x1000 / static_cast<int32_t>(cell.hits));
    // Convergida: erro médio baixo (λ no alvo com trim a segurar).
    if (abs_i32(mean_err) > kLtftAccumReadyMaxMeanErrX1000) {
        return false;
    }
    // Vale a pena commitar: há viés real de mapa (não só ruído ~0).
    if (abs_i32(mean_stft) < kLtftAccumReadyMinMeanStftX10) {
        return false;
    }
    // Ainda fiável: STFT médio não saturado.
    if (abs_i32(mean_stft) > kLtftAccumReadyMaxMeanStftX10) {
        return false;
    }
    return true;
}

int16_t fuel_ltft_accum_mean_stft_x10(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        return 0;
    }
    const LtftCellStats& cell = g_ltft_stats[map_idx][rpm_idx];
    if (cell.hits == 0u) {
        return 0;
    }
    return static_cast<int16_t>(cell.sum_stft_x10 / static_cast<int32_t>(cell.hits));
}

int16_t fuel_ltft_accum_mean_err_x1000(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        return 0;
    }
    const LtftCellStats& cell = g_ltft_stats[map_idx][rpm_idx];
    if (cell.hits == 0u) {
        return 0;
    }
    return static_cast<int16_t>(cell.sum_err_x1000 / static_cast<int32_t>(cell.hits));
}

static void fuel_ltft_accum_tick(uint8_t map_idx,
                                 uint8_t rpm_idx,
                                 int16_t stft_pct_x10,
                                 int16_t err_x1000,
                                 bool sample_valid) noexcept {
    if (!sample_valid) {
        ++g_dbg_ltft_accum_rejected;
        return;
    }
    if (map_idx >= kTableAxisSize || rpm_idx >= kTableAxisSize) {
        ++g_dbg_ltft_accum_rejected;
        return;
    }
    LtftCellStats& cell = g_ltft_stats[map_idx][rpm_idx];
    // Congela no teto: somar sem ++hits corromperia a média (sum/hits).
    if (cell.hits >= 65535u) {
        ++g_dbg_ltft_accum_accepted;  // amostra válida, célula saturada
        return;
    }
    ++g_dbg_ltft_accum_accepted;
    ++cell.hits;
    cell.sum_stft_x10 += stft_pct_x10;
    cell.sum_err_x1000 += err_x1000;
}

// unroll_global_stft: true em commit de célula única (desenrola o trim global).
// false em apply_all — N células não devem subtrair N×bake do mesmo STFT.
static bool ltft_accum_commit_cell(uint8_t map_idx,
                                   uint8_t rpm_idx,
                                   bool unroll_global_stft) noexcept {
    if (!fuel_ltft_accum_cell_ready(map_idx, rpm_idx)) {
        return false;
    }

    const int16_t mean_stft = fuel_ltft_accum_mean_stft_x10(map_idx, rpm_idx);
    // bake = mean * gain / 100  (calibrável; 0 no blob → default 50)
    uint8_t gain = ltft_commit_gain_pct;
    if (gain == 0u) {
        gain = static_cast<uint8_t>(kLtftAccumCommitGainPct);
    }
    int16_t bake_x10 = static_cast<int16_t>(
        (static_cast<int32_t>(mean_stft) * static_cast<int32_t>(gain)) / 100);
    if (bake_x10 == 0) {
        // mean na fronteira do min mas gain arredondou a 0 — não commitar lixo
        fuel_ltft_accum_reset_cell(map_idx, rpm_idx);
        return false;
    }

    // VE[map][rpm] = VE * (1000 + bake_x10) / 1000
    // bake_x10 em %×10: +40 → +4.0% → factor 1040/1000.
    // Arredonda ao mais próximo; se o factor truncar a 0 em VE pequena,
    // garante ΔVE mínimo de ±1 para o bake não se perder em inteiros.
    uint8_t& ve_cell = ve_table[map_idx][rpm_idx];
    const int32_t ve_old = static_cast<int32_t>(ve_cell);
    const int32_t factor = 1000 + static_cast<int32_t>(bake_x10);
    int32_t ve_new = (ve_old * factor + ((factor >= 0) ? 500 : -500)) / 1000;
    if (ve_new == ve_old) {
        ve_new = ve_old + ((bake_x10 > 0) ? 1 : -1);
    }
    if (ve_new < static_cast<int32_t>(kLtftAccumVeMin)) {
        ve_new = static_cast<int32_t>(kLtftAccumVeMin);
    }
    if (ve_new > static_cast<int32_t>(kLtftAccumVeMax)) {
        ve_new = static_cast<int32_t>(kLtftAccumVeMax);
    }
    // Se o clamp matou a mudança (VE já no limite), não desenrola trims —
    // evita drift de trim sem mudança de base. Limpa stats e tenta de novo depois.
    if (ve_new == ve_old) {
        fuel_ltft_accum_reset_cell(map_idx, rpm_idx);
        return false;
    }
    ve_cell = static_cast<uint8_t>(ve_new);

    // Desenrola LTFT multiplicativo da célula (evita double-count com VE nova).
    int16_t& ltft = g_ltft_pct_x10[map_idx][rpm_idx];
    ltft = static_cast<int16_t>(ltft - bake_x10);
    const int16_t ltft_clamp = ltft_mult_clamp();
    ltft = clamp_i16(ltft, static_cast<int16_t>(-ltft_clamp), ltft_clamp);
    fuel_ltft_store_cell(map_idx, rpm_idx, ltft);

    if (unroll_global_stft) {
        // Desenrola STFT global (integrador em ×1000: stft ≈ I/100).
        const int16_t stft_clamp = static_cast<int16_t>(ems::engine::stft_clamp_pct_x10);
        g_stft_pct_x10 = clamp_i16(
            static_cast<int16_t>(g_stft_pct_x10 - bake_x10),
            static_cast<int16_t>(-stft_clamp),
            stft_clamp);
        g_stft_integrator_x1000 -= static_cast<int32_t>(bake_x10) * 100;
        const int32_t clamp_x1000 = static_cast<int32_t>(stft_clamp) * 100;
        if (g_stft_integrator_x1000 > clamp_x1000) {
            g_stft_integrator_x1000 = clamp_x1000;
        } else if (g_stft_integrator_x1000 < -clamp_x1000) {
            g_stft_integrator_x1000 = -clamp_x1000;
        }
    }

    fuel_ltft_accum_reset_cell(map_idx, rpm_idx);
    ++g_dbg_ltft_accum_commits;
    // Burn opcional: pedido assíncrono — ui_process grava page1 se RPM seguro.
    if (ltft_apply_burn_ve != 0u) {
        g_ltft_ve_burn_pending = true;
    }
    return true;
}

bool fuel_ltft_accum_try_commit(uint8_t map_idx, uint8_t rpm_idx) noexcept {
    // Manual-only: closed-loop não chama isto. Unroll STFT nesta célula.
    return ltft_accum_commit_cell(map_idx, rpm_idx, /*unroll_global_stft=*/true);
}

uint16_t fuel_ltft_accum_apply_all_ready() noexcept {
    // Bulk APPLY: VE + LTFT por célula; STFT global fica — re-converge no loop.
    // (N commits × unroll STFT derrubava o trim global de forma incorrecta.)
    uint16_t n = 0u;
    for (uint8_t m = 0u; m < kTableAxisSize; ++m) {
        for (uint8_t r = 0u; r < kTableAxisSize; ++r) {
            if (ltft_accum_commit_cell(m, r, /*unroll_global_stft=*/false)) {
                ++n;
            }
        }
    }
    return n;
}

int16_t fuel_update_stft(uint32_t rpm_x10,
                         uint16_t map_bar_x100,
                         int16_t lambda_target_x1000,
                         int16_t lambda_measured_x1000,
                         int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut,
                         uint32_t net_pw_us,
                         uint16_t tps_x10,
                         uint32_t now_ms) noexcept {
    // prev só avança em closed-loop (mais abaixo). Em AE/rev_cut/CLT frio a
    // âncora do último regime estável mantém-se — evita ΔTPS falso e perda
    // da referência pós-bloqueio.
    const bool prev_valid = g_ltft_accum_have_prev;
    const uint32_t prev_rpm = g_ltft_accum_prev_rpm_x10;
    const uint16_t prev_tps = g_ltft_accum_prev_tps_x10;

    if (!closed_loop_allowed(clt_x10, o2_valid, ae_active, rev_cut)) {
        // DIAG: conta o motivo do bloqueio (prioridade na ordem do gate)
        if (closed_loop_enable == 0u) { /* master off — sem contador dedicado */ }
        else if (clt_x10 <= 700)      { ++g_dbg_stft_blocked_clt; }
        else if (!o2_valid)      { ++g_dbg_stft_blocked_o2; }
        else if (ae_active)      { ++g_dbg_stft_blocked_ae; }
        else                     { ++g_dbg_stft_blocked_cut; }
        // Anti-windup: congela o trim em vez de decair para zero.
        return g_stft_pct_x10;
    }

    // Post-start: congela STFT+LTFT até decorrer closed_loop_post_start_s.
    if (!post_start_elapsed(now_ms, clt_x10, o2_valid)) {
        return g_stft_pct_x10;
    }

    g_ltft_accum_prev_rpm_x10 = rpm_x10;
    g_ltft_accum_prev_tps_x10 = tps_x10;
    g_ltft_accum_have_prev    = true;

    ++g_dbg_stft_runs;

    const int16_t clamp = static_cast<int16_t>(ems::engine::stft_clamp_pct_x10);
    const int32_t clamp_x1000 = static_cast<int32_t>(clamp) * 100;
    const int16_t error_x1000 = static_cast<int16_t>(lambda_measured_x1000 - lambda_target_x1000);
    g_dbg_stft_last_err = error_x1000;
    const int32_t p_x10 = (static_cast<int32_t>(error_x1000) * static_cast<int32_t>(ems::engine::stft_kp_x100)) / 100;
    // incremento em ×1000: error×ki/10 (era /1000 em ×10 — truncava a zero)
    g_stft_integrator_x1000 += (static_cast<int32_t>(error_x1000) * static_cast<int32_t>(ems::engine::stft_ki_x1000)) / 10;

    if (g_stft_integrator_x1000 > clamp_x1000) {
        g_stft_integrator_x1000 = clamp_x1000;
    } else if (g_stft_integrator_x1000 < -clamp_x1000) {
        g_stft_integrator_x1000 = -clamp_x1000;
    }

    const int32_t stft = p_x10 + g_stft_integrator_x1000 / 100;
    g_stft_pct_x10 = clamp_i16(static_cast<int16_t>(stft), -clamp, clamp);

    // Célula de crédito = nó dominante (nearest), igual ao trace do VE no dash.
    const uint8_t rpm_idx =
        table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t map_idx =
        table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, map_bar_x100);

    // LTFT IIR + LEARN: min RPM + MAP estável. STFT global corre sempre em CL.
    bool ltft_adapt_ok = (rpm_x10 >= static_cast<uint32_t>(ltft_adapt_min_rpm_x10));
    if (ltft_adapt_ok && g_ltft_have_prev_map) {
        if (abs_i32(static_cast<int32_t>(map_bar_x100) -
                    static_cast<int32_t>(g_ltft_prev_map_bar_x100)) >
            static_cast<int32_t>(kLtftAccumMaxMapDeltaBarX100)) {
            ltft_adapt_ok = false;
        }
    }
    g_ltft_prev_map_bar_x100 = map_bar_x100;
    g_ltft_have_prev_map     = true;

    const bool multiplicative_path =
        !(net_pw_us > 0u && net_pw_us < static_cast<uint32_t>(ltft_add_pw_threshold_us));

    if (ltft_adapt_ok) {
        if (!multiplicative_path) {
            // PW pequeno: LTFT aditivo (offset de bico).
            const int32_t error_us = (static_cast<int32_t>(g_stft_pct_x10) *
                                      static_cast<int32_t>(net_pw_us)) / 1000;
            const uint8_t ri = rpm_idx >> 1u;
            const uint8_t mi = map_idx >> 1u;
            int16_t& cell_add = g_ltft_add_us[mi][ri];
            const int16_t add_lim = ltft_add_clamp();
            // IIR em µs: mesmo div; max_step_x10 não aplica (unidade diferente).
            const uint8_t div = ltft_iir_div();
            cell_add = clamp_i16(
                static_cast<int16_t>(
                    static_cast<int32_t>(cell_add) +
                    (error_us - static_cast<int32_t>(cell_add)) /
                        static_cast<int32_t>(div)),
                static_cast<int16_t>(-add_lim), add_lim);
            fuel_ltft_add_store_cell(map_idx, rpm_idx, cell_add);
        } else {
            // PW normal: LTFT multiplicativo (clamp/rate calibráveis).
            int16_t& cell = g_ltft_pct_x10[map_idx][rpm_idx];
            cell = ltft_iir_toward(cell, g_stft_pct_x10, ltft_mult_clamp());
            fuel_ltft_store_cell(map_idx, rpm_idx, cell);
        }

        // LEARN → VE só no caminho multiplicativo.
        if (multiplicative_path) {
            const int16_t err_x1000 =
                static_cast<int16_t>(lambda_measured_x1000 - lambda_target_x1000);
            fuel_ltft_accum_tick(
                map_idx,
                rpm_idx,
                g_stft_pct_x10,
                err_x1000,
                ltft_accum_sample_valid(rpm_x10,
                                        prev_rpm,
                                        tps_x10,
                                        prev_tps,
                                        prev_valid,
                                        lambda_target_x1000,
                                        lambda_measured_x1000,
                                        g_stft_pct_x10,
                                        clt_x10,
                                        o2_valid,
                                        ae_active,
                                        rev_cut));
        }
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
                                 uint32_t net_pw_us,
                                 uint16_t tps_x10) noexcept {
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
                                net_pw_us,
                                tps_x10,
                                now_ms);
    }

    return fuel_update_stft(delayed.rpm_x10,
                            delayed.map_bar_x100,
                            delayed.lambda_target_x1000,
                            lambda_measured_x1000,
                            clt_x10,
                            o2_valid,
                            ae_active,
                            rev_cut,
                            net_pw_us,
                            tps_x10,
                            now_ms);
}

int16_t fuel_get_stft_pct_x10() noexcept {
    return g_stft_pct_x10;
}

int16_t fuel_get_ltft_at(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept {
    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, map_bar_x100);
    return g_ltft_pct_x10[mi][ri];
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

int16_t fuel_get_ltft_add_at(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept {
    const uint8_t ri = table_axis_nearest_index(kRpmAxisX10, kTableAxisSize, rpm_x10);
    const uint8_t mi = table_axis_nearest_index(kLoadAxisBarX100, kTableAxisSize, map_bar_x100);
    return fuel_get_ltft_add_us(mi, ri);
}


}  // namespace ems::engine
