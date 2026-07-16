#include "engine/xtau_autocalib.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"
#include "engine/transient_fuel.h"
#include "hal/system.h"

#include <cstdint>

namespace {

using ems::engine::clamp_u16;
using ems::engine::interp_u16_8pt;
using ems::engine::table_axis_index;
using ems::engine::table_axis_frac_q8;

// Estado global do sistema de auto-calibração
ems::engine::WallFuelState g_wall_state = {};

// Tabela 2D (RPM × MAP) de parâmetros X-τ aprendidos. wall_fuel_us_q8 continua
// escalar (há fisicamente uma só parede de coletor); só X e τ variam com o
// ponto de operação — a velocidade do ar no coletor depende de RPM e carga.
constexpr uint8_t kXTauTableSize = 4u;
uint32_t g_xtau_rpm_axis_x10[kXTauTableSize] = {8000u, 25000u, 45000u, 70000u};
uint32_t g_xtau_map_axis_bar_x100[kXTauTableSize] = {2500u, 5000u, 8000u, 15000u};
ems::engine::XTauParams g_learned_table[kXTauTableSize][kXTauTableSize] = {};
bool g_xtau_table_seeded = false;

// Índice da célula aprendida no último xtau_autocalib_update(), usado para
// detetar mudança de ponto de operação a meio de uma janela de aprendizado.
uint8_t g_last_cell_rpm_idx = 0u;
uint8_t g_last_cell_map_idx = 0u;

// Histórico de erro de lambda durante transientes para calibração
struct LambdaErrorSample {
    uint32_t timestamp_ms;
    int16_t error_x1000;  // lambda_measured - lambda_target
    bool valid;
};

constexpr uint8_t kLambdaErrorHistorySize = 8u;
LambdaErrorSample g_lambda_error_history[kLambdaErrorHistorySize] = {};
uint8_t g_lambda_error_pos = 0u;

// Contadores de diagnóstico
volatile uint32_t g_xtau_calib_updates = 0u;
volatile uint32_t g_xtau_learning_cycles = 0u;

// Janela de transiente contínuo para learn (evita 1 amostra de ruído tpsdot).
uint32_t g_transient_start_ms = 0u;
bool g_transient_active = false;

// Limites de aprendizado
constexpr int16_t kLambdaErrorThresholdX1000 = 50;   // 0.05 λ de erro mínimo para aprender
constexpr int16_t kMaxLambdaErrorX1000 = 150;        // 0.15 λ de erro máximo para confiar
constexpr uint16_t kMinTransientDurationMs = 100u;   // Transiente deve durar ≥100ms
constexpr uint8_t kLearningRateQ8 = 32u;             // 12.5% de atualização por amostra (32/256)

void lambda_error_push(int16_t error_x1000) noexcept {
    g_lambda_error_history[g_lambda_error_pos].timestamp_ms = 
        millis();
    g_lambda_error_history[g_lambda_error_pos].error_x1000 = error_x1000;
    g_lambda_error_history[g_lambda_error_pos].valid = true;
    g_lambda_error_pos = (g_lambda_error_pos + 1u) % kLambdaErrorHistorySize;
}

int16_t lambda_error_get_average() noexcept {
    int32_t sum = 0;
    uint8_t count = 0;
    
    for (uint8_t i = 0u; i < kLambdaErrorHistorySize; ++i) {
        if (g_lambda_error_history[i].valid) {
            sum += g_lambda_error_history[i].error_x1000;
            ++count;
        }
    }
    
    return (count > 0) ? static_cast<int16_t>(sum / count) : 0;
}

void lambda_error_reset() noexcept {
    for (uint8_t i = 0u; i < kLambdaErrorHistorySize; ++i) {
        g_lambda_error_history[i].valid = false;
    }
    g_lambda_error_pos = 0u;
}

void xtau_seed_table_if_needed() noexcept {
    if (g_xtau_table_seeded) {
        return;
    }
    ems::engine::XTauParams seed = {};
    seed.x_fraction_q8 = ems::engine::xtau_x_fraction_q8[0];  // baseline a 0°C
    seed.tau_cycles = ems::engine::xtau_tau_cycles[0];
    seed.learned_x_min_q8 = 64u;   // 25%
    seed.learned_x_max_q8 = 192u;  // 75%
    seed.learned_tau_min = 10u;
    seed.learned_tau_max = 255u;
    for (uint8_t ri = 0u; ri < kXTauTableSize; ++ri) {
        for (uint8_t mi = 0u; mi < kXTauTableSize; ++mi) {
            g_learned_table[ri][mi] = seed;
        }
    }
    g_xtau_table_seeded = true;
}

// Célula mais próxima (arredondamento, não interpolação) — usada para escrita
// durante o aprendizado: cada amostra atualiza um único ponto de operação.
void find_xtau_nearest_cell(uint32_t rpm_x10, uint32_t map_bar_x100,
                            uint8_t& out_rpm_idx, uint8_t& out_map_idx) noexcept {
    const uint8_t ri = table_axis_index(g_xtau_rpm_axis_x10, kXTauTableSize, rpm_x10);
    const uint8_t mi = table_axis_index(g_xtau_map_axis_bar_x100, kXTauTableSize, map_bar_x100);
    const uint8_t fr = table_axis_frac_q8(g_xtau_rpm_axis_x10, ri, rpm_x10);
    const uint8_t fm = table_axis_frac_q8(g_xtau_map_axis_bar_x100, mi, map_bar_x100);
    out_rpm_idx = (fr >= 128u && ri < (kXTauTableSize - 1u)) ? static_cast<uint8_t>(ri + 1u) : ri;
    out_map_idx = (fm >= 128u && mi < (kXTauTableSize - 1u)) ? static_cast<uint8_t>(mi + 1u) : mi;
}

// Leitura interpolada bilinear (RPM × MAP), mesmo padrão de interp_lambda_delay_3x3
// em fuel_calc.cpp.
ems::engine::XTauParams interpolate_xtau_2d(uint32_t rpm_x10, uint32_t map_bar_x100) noexcept {
    const uint8_t xi = table_axis_index(g_xtau_rpm_axis_x10, kXTauTableSize, rpm_x10);
    const uint8_t yi = table_axis_index(g_xtau_map_axis_bar_x100, kXTauTableSize, map_bar_x100);
    const uint8_t fx = table_axis_frac_q8(g_xtau_rpm_axis_x10, xi, rpm_x10);
    const uint8_t fy = table_axis_frac_q8(g_xtau_map_axis_bar_x100, yi, map_bar_x100);

    const auto& c00 = g_learned_table[xi][yi];
    const auto& c10 = g_learned_table[xi + 1u][yi];
    const auto& c01 = g_learned_table[xi][yi + 1u];
    const auto& c11 = g_learned_table[xi + 1u][yi + 1u];

    auto lerp2d = [fx, fy](uint16_t v00, uint16_t v10, uint16_t v01, uint16_t v11) noexcept -> uint16_t {
        const int32_t a0 = static_cast<int32_t>(v00) +
            (((static_cast<int32_t>(v10) - v00) * static_cast<int32_t>(fx)) >> 8);
        const int32_t a1 = static_cast<int32_t>(v01) +
            (((static_cast<int32_t>(v11) - v01) * static_cast<int32_t>(fx)) >> 8);
        return static_cast<uint16_t>(a0 + (((a1 - a0) * static_cast<int32_t>(fy)) >> 8));
    };

    ems::engine::XTauParams result;
    result.x_fraction_q8 = lerp2d(c00.x_fraction_q8, c10.x_fraction_q8,
                                   c01.x_fraction_q8, c11.x_fraction_q8);
    result.tau_cycles = lerp2d(c00.tau_cycles, c10.tau_cycles,
                                c01.tau_cycles, c11.tau_cycles);
    result.learned_x_min_q8 = c00.learned_x_min_q8;
    result.learned_x_max_q8 = c00.learned_x_max_q8;
    result.learned_tau_min = c00.learned_tau_min;
    result.learned_tau_max = c00.learned_tau_max;
    return result;
}

// Calcula X e τ ideais baseado em erro de lambda acumulado, partindo dos
// valores aprendidos na célula RPM×MAP atual.
void calculate_ideal_xtau(const ems::engine::XTauParams& current,
                          int16_t avg_error_x1000,
                          uint16_t& out_x_q8,
                          uint16_t& out_tau) noexcept {
    // Se erro positivo (lambda medido > target = mistura pobre):
    //   - Aumentar X (mais combustível na parede)
    //   - Diminuir τ (evaporação mais rápida)
    // Se erro negativo (lambda medido < target = mistura rica):
    //   - Diminuir X (menos combustível na parede)
    //   - Aumentar τ (evaporação mais lenta)

    const int16_t abs_error = (avg_error_x1000 >= 0) ?
        avg_error_x1000 : -avg_error_x1000;

    // Ponto de partida: valores já aprendidos para esta célula RPM×MAP
    out_x_q8 = current.x_fraction_q8;
    out_tau = current.tau_cycles;

    // Correção baseada em erro (apenas se dentro de limites confiáveis)
    if (abs_error <= kMaxLambdaErrorX1000 && abs_error >= kLambdaErrorThresholdX1000) {
        // Ajuste de X: ±25% baseado em erro
        const int16_t x_correction_q8 = (avg_error_x1000 * 64) / 1000;  // 64 Q8 = 0.25
        int32_t new_x_q8 = static_cast<int32_t>(out_x_q8) + x_correction_q8;
        new_x_q8 = ems::engine::clamp_i16(static_cast<int16_t>(new_x_q8),
            static_cast<int16_t>(ems::engine::xtau_x_min_q8),
            static_cast<int16_t>(ems::engine::xtau_x_max_q8));
        out_x_q8 = static_cast<uint16_t>(new_x_q8);
        
        // Ajuste de τ: ∓20% baseado em erro (direção oposta)
        const int16_t tau_correction = (avg_error_x1000 * 51) / 1000;  // 51 ≈ 20% de 256
        int32_t new_tau = static_cast<int32_t>(out_tau) - tau_correction;
        new_tau = ems::engine::clamp_i16(static_cast<int16_t>(new_tau),
            static_cast<int16_t>(ems::engine::xtau_tau_min),
            static_cast<int16_t>(ems::engine::xtau_tau_max));
        out_tau = static_cast<uint16_t>(new_tau);
    }
}

}  // namespace

namespace ems::engine {

void xtau_autocalib_init() noexcept {
    g_wall_state = {};
    g_xtau_table_seeded = false;
    xtau_seed_table_if_needed();
    g_last_cell_rpm_idx = 0u;
    g_last_cell_map_idx = 0u;
    g_transient_start_ms = 0u;
    g_transient_active = false;

    lambda_error_reset();
    g_xtau_calib_updates = 0u;
    g_xtau_learning_cycles = 0u;
}

void xtau_autocalib_reset() noexcept {
    xtau_autocalib_init();
}

void xtau_wall_fuel_reset() noexcept {
    g_wall_state.wall_fuel_us_q8 = 0;
    g_wall_state.last_update_ms = 0u;
}

bool xtau_autocalib_update(uint32_t rpm_x10,
                           uint16_t map_bar_x100,
                           int16_t lambda_target_x1000,
                           int16_t lambda_measured_x1000,
                           int16_t /*clt_x10*/,
                           bool is_transient) noexcept {
    xtau_seed_table_if_needed();

    const uint32_t now_ms = millis();

    // Só aprende em transiente real. Regime estável limpa a janela.
    if (!is_transient) {
        lambda_error_reset();
        g_transient_active = false;
        g_transient_start_ms = 0u;
        // Não derruba calibrated→inactive: filme/tabela 2D permanecem válidos.
        if (g_wall_state.calibration_state == 1u) {
            g_wall_state.calibration_state = 0u;
        }
        return false;
    }

    // Marca início da janela de transiente contínuo.
    if (!g_transient_active) {
        g_transient_active = true;
        g_transient_start_ms = now_ms;
        lambda_error_reset();
    }

    // Exige duração mínima antes de aceitar amostras (anti-ruído tpsdot).
    const uint32_t elapsed = now_ms - g_transient_start_ms;
    if (elapsed < kMinTransientDurationMs) {
        if (g_wall_state.calibration_state != 2u) {
            g_wall_state.calibration_state = 1u;  // learning (warm-up)
        }
        return false;
    }

    // Validações básicas
    if (lambda_target_x1000 < 650 || lambda_target_x1000 > 1200) {
        return false;
    }
    if (lambda_measured_x1000 < 650 || lambda_measured_x1000 > 1200) {
        return false;
    }

    // Célula RPM×MAP mais próxima do ponto de operação atual. Se mudou desde a
    // última amostra, a janela de aprendizado em curso mistura pontos de
    // operação diferentes — descarta e recomeça nesta célula.
    uint8_t rpm_idx = 0u;
    uint8_t map_idx = 0u;
    find_xtau_nearest_cell(rpm_x10, map_bar_x100, rpm_idx, map_idx);
    if (rpm_idx != g_last_cell_rpm_idx || map_idx != g_last_cell_map_idx) {
        lambda_error_reset();
        g_last_cell_rpm_idx = rpm_idx;
        g_last_cell_map_idx = map_idx;
        // Reinicia duração na nova célula (evita misturar e aprender cedo demais).
        g_transient_start_ms = now_ms;
        return false;
    }

    // Calcula erro de lambda
    const int16_t error_x1000 = lambda_measured_x1000 - lambda_target_x1000;

    // Ignora erros muito pequenos (ruído) ou muito grandes (falha)
    if (error_x1000 > -kLambdaErrorThresholdX1000 &&
        error_x1000 < kLambdaErrorThresholdX1000) {
        return false;
    }
    if (error_x1000 > kMaxLambdaErrorX1000 ||
        error_x1000 < -kMaxLambdaErrorX1000) {
        return false;
    }

    if (g_wall_state.calibration_state != 2u) {
        g_wall_state.calibration_state = 1u;  // learning
    }

    // Adiciona ao histórico
    lambda_error_push(error_x1000);
    ++g_xtau_learning_cycles;

    // Precisa de pelo menos 4 amostras válidas para tomar decisão
    uint8_t valid_count = 0;
    for (uint8_t i = 0u; i < kLambdaErrorHistorySize; ++i) {
        if (g_lambda_error_history[i].valid) {
            ++valid_count;
        }
    }

    if (valid_count < 4u) {
        return false;
    }

    // Calcula erro médio
    const int16_t avg_error_x1000 = lambda_error_get_average();

    ems::engine::XTauParams& cell = g_learned_table[rpm_idx][map_idx];

    // Calcula parâmetros ideais para esta célula
    uint16_t ideal_x_q8 = 0u;
    uint16_t ideal_tau = 0u;
    calculate_ideal_xtau(cell, avg_error_x1000, ideal_x_q8, ideal_tau);

    // Aplica aprendizado com blending suave (rate-limit)
    // Blend: 87.5% valor antigo + 12.5% novo valor
    const int32_t x_delta = static_cast<int32_t>(ideal_x_q8) -
                            static_cast<int32_t>(cell.x_fraction_q8);
    const int32_t tau_delta = static_cast<int32_t>(ideal_tau) -
                              static_cast<int32_t>(cell.tau_cycles);

    cell.x_fraction_q8 = static_cast<uint16_t>(
        static_cast<int32_t>(cell.x_fraction_q8) +
        ((x_delta * kLearningRateQ8) >> 8));

    cell.tau_cycles = static_cast<uint16_t>(
        static_cast<int32_t>(cell.tau_cycles) +
        ((tau_delta * kLearningRateQ8) >> 8));

    // Atualiza limites aprendidos (desta célula)
    if (cell.x_fraction_q8 < cell.learned_x_min_q8) {
        cell.learned_x_min_q8 = cell.x_fraction_q8;
    }
    if (cell.x_fraction_q8 > cell.learned_x_max_q8) {
        cell.learned_x_max_q8 = cell.x_fraction_q8;
    }
    if (cell.tau_cycles < cell.learned_tau_min) {
        cell.learned_tau_min = cell.tau_cycles;
    }
    if (cell.tau_cycles > cell.learned_tau_max) {
        cell.learned_tau_max = cell.tau_cycles;
    }

    // Atualiza estado
    g_wall_state.calibration_state = 2u;  // calibrated
    ++g_xtau_calib_updates;

    // Reseta histórico para próximo ciclo de aprendizado
    lambda_error_reset();

    return true;
}

XTauParams xtau_get_current_params(int16_t clt_x10) noexcept {
    // Fallback 1D por temperatura do motor — usado antes de haver aprendizado
    // 2D suficiente na célula RPM×MAP atual (ver transient_fuel_xtau_with_autocalib).
    XTauParams params = {};
    params.x_fraction_q8 = interp_u16_8pt(
        xtau_clt_axis_x10, xtau_x_fraction_q8, kCorrectionTableSize, clt_x10);
    params.tau_cycles = interp_u16_8pt(
        xtau_clt_axis_x10, xtau_tau_cycles, kCorrectionTableSize, clt_x10);
    params.learned_x_min_q8 = 64u;   // 25%
    params.learned_x_max_q8 = 192u;  // 75%
    params.learned_tau_min = 10u;
    params.learned_tau_max = 255u;
    return params;
}

XTauParams xtau_get_current_params_2d(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept {
    xtau_seed_table_if_needed();
    return interpolate_xtau_2d(rpm_x10, map_bar_x100);
}

uint32_t transient_fuel_xtau_with_autocalib(uint32_t fuel_pw_us,
                                             uint32_t rpm_x10,
                                             uint16_t map_bar_x100,
                                             int16_t clt_x10,
                                             bool enabled,
                                             uint16_t period_ms) noexcept {
    if (!enabled || fuel_pw_us == 0u) {
        // Zera filme de produção + legado (transient_fuel_reset → xtau_wall_fuel_reset).
        // Não apaga tabela 2D: calibration_state==2 permanece se já calibrado.
        transient_fuel_reset();
        if (g_wall_state.calibration_state == 1u) {
            g_wall_state.calibration_state = 0u;
        }
        return fuel_pw_us;
    }

    // Enquanto não há aprendizado suficiente, usa o fallback 1D por CLT;
    // depois de calibrado, usa a tabela 2D RPM×MAP interpolada.
    xtau_seed_table_if_needed();
    const XTauParams params = (g_wall_state.calibration_state >= 2u)
        ? interpolate_xtau_2d(rpm_x10, map_bar_x100)
        : xtau_get_current_params(clt_x10);

    uint16_t x_q8 = params.x_fraction_q8;
    if (x_q8 > 192u) {
        x_q8 = 192u;
    }

    uint16_t tau = params.tau_cycles;
    if (tau == 0u) {
        tau = 1u;
    }
    if (tau > 255u) {
        tau = 255u;
    }

    // τ em ciclos de motor (720°). Escala evaporação ao wall-clock do loop:
    //   cycle_ms = 1200000 / rpm_x10   (4-stroke: 2 revs por ciclo)
    //   evap = wall/τ × (dt / cycle_ms)
    // Assim idle (ciclo longo) evapora devagar; alto RPM mais rápido.
    uint16_t dt = period_ms;
    if (dt == 0u) {
        dt = 2u;
    }
    if (dt > 50u) {
        dt = 50u;  // clamp glitch de loop
    }
    uint32_t rpm = rpm_x10 / 10u;
    if (rpm < 200u) {
        rpm = 200u;  // piso: evita cycle_ms enorme / div0
    }
    if (rpm > 15000u) {
        rpm = 15000u;
    }
    // cycle_ms = 120000 / rpm  (720°); em inteiros: 120000 / rpm
    const uint32_t cycle_ms = 120000u / rpm;
    // denom = τ × cycle_ms  (mín. 1)
    const uint32_t tau_cycle_ms =
        static_cast<uint32_t>(tau) * ((cycle_ms < 1u) ? 1u : cycle_ms);

    constexpr uint32_t kMaxPwUs = 100000u;
    const uint32_t clamped_pw = fuel_pw_us > kMaxPwUs ? kMaxPwUs : fuel_pw_us;
    const int32_t desired_q8 = static_cast<int32_t>(clamped_pw << 8u);

    // evap_q8 = wall × dt / (τ × cycle_ms); clamp a wall (não evapora mais que existe)
    int32_t evap_q8 = static_cast<int32_t>(
        (static_cast<int64_t>(g_wall_state.wall_fuel_us_q8) * static_cast<int32_t>(dt)) /
        static_cast<int64_t>(tau_cycle_ms));
    if (evap_q8 > g_wall_state.wall_fuel_us_q8) {
        evap_q8 = g_wall_state.wall_fuel_us_q8;
    }
    if (evap_q8 < 0) {
        evap_q8 = 0;
    }

    int32_t numerator_q8 = desired_q8 - evap_q8;
    if (numerator_q8 < 0) {
        numerator_q8 = 0;
    }

    const int32_t dry_fraction_q8 = 256 - static_cast<int32_t>(x_q8);
    int32_t injected_q8 = static_cast<int32_t>(
        (static_cast<int64_t>(numerator_q8) * 256) / dry_fraction_q8);
    const int32_t max_q8 = static_cast<int32_t>(kMaxPwUs << 8u);
    if (injected_q8 > max_q8) {
        injected_q8 = max_q8;
    }

    g_wall_state.wall_fuel_us_q8 +=
        ((injected_q8 * static_cast<int32_t>(x_q8)) >> 8) - evap_q8;
    if (g_wall_state.wall_fuel_us_q8 < 0) {
        g_wall_state.wall_fuel_us_q8 = 0;
    }
    if (g_wall_state.wall_fuel_us_q8 > max_q8) {
        g_wall_state.wall_fuel_us_q8 = max_q8;
    }

    g_wall_state.last_update_ms = millis();

    return static_cast<uint32_t>(injected_q8 >> 8);
}

bool xtau_is_learning() noexcept {
    return (g_wall_state.calibration_state == 1u);
}

WallFuelState xtau_get_state() noexcept {
    return g_wall_state;
}

}  // namespace ems::engine
