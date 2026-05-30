#include "engine/xtau_autocalib.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"
#include "engine/transient_fuel.h"
#include "hal/system.h"

#include <cstdint>

namespace {

using ems::engine::clamp_u16;
using ems::engine::interp_u16_8pt;

// Estado global do sistema de auto-calibração
ems::engine::WallFuelState g_wall_state = {};
ems::engine::XTauParams g_learned_params = {};

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

// Calcula X e τ ideais baseado em erro de lambda acumulado
void calculate_ideal_xtau(int16_t avg_error_x1000,
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
    
    // Ponto de partida: valores da tabela baseados em CLT
    out_x_q8 = g_learned_params.x_fraction_q8;
    out_tau = g_learned_params.tau_cycles;
    
    // Correção baseada em erro (apenas se dentro de limites confiáveis)
    if (abs_error <= kMaxLambdaErrorX1000 && abs_error >= kLambdaErrorThresholdX1000) {
        // Ajuste de X: ±25% baseado em erro
        const int16_t x_correction_q8 = (avg_error_x1000 * 64) / 1000;  // 64 Q8 = 0.25
        int32_t new_x_q8 = static_cast<int32_t>(out_x_q8) + x_correction_q8;
        new_x_q8 = ems::engine::clamp_i16(static_cast<int16_t>(new_x_q8), 64, 192);  // 25%-75%
        out_x_q8 = static_cast<uint16_t>(new_x_q8);
        
        // Ajuste de τ: ∓20% baseado em erro (direção oposta)
        const int16_t tau_correction = (avg_error_x1000 * 51) / 1000;  // 51 ≈ 20% de 256
        int32_t new_tau = static_cast<int32_t>(out_tau) - tau_correction;
        new_tau = ems::engine::clamp_i16(static_cast<int16_t>(new_tau), 10, 255);
        out_tau = static_cast<uint16_t>(new_tau);
    }
}

}  // namespace

namespace ems::engine {

void xtau_autocalib_init() noexcept {
    g_wall_state = {};
    g_learned_params = {};
    
    // Inicializa com valores da tabela de calibração
    g_learned_params.x_fraction_q8 = xtau_x_fraction_q8[0];  // Valor a 0°C como baseline
    g_learned_params.tau_cycles = xtau_tau_cycles[0];
    g_learned_params.learned_x_min_q8 = 64u;   // 25%
    g_learned_params.learned_x_max_q8 = 192u;  // 75%
    g_learned_params.learned_tau_min = 10u;
    g_learned_params.learned_tau_max = 255u;
    
    lambda_error_reset();
    g_xtau_calib_updates = 0u;
    g_xtau_learning_cycles = 0u;
}

void xtau_autocalib_reset() noexcept {
    xtau_autocalib_init();
}

bool xtau_autocalib_update(uint32_t rpm_x10,
                           uint16_t map_bar_x100,
                           int16_t lambda_target_x1000,
                           int16_t lambda_measured_x1000,
                           int16_t clt_x10,
                           bool is_transient) noexcept {
    // Só aprende em condições controladas
    if (!is_transient) {
        lambda_error_reset();
        return false;
    }
    
    // Validações básicas
    if (lambda_target_x1000 < 650 || lambda_target_x1000 > 1200) {
        return false;
    }
    if (lambda_measured_x1000 < 650 || lambda_measured_x1000 > 1200) {
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
    
    // Calcula parâmetros ideais
    uint16_t ideal_x_q8 = 0u;
    uint16_t ideal_tau = 0u;
    calculate_ideal_xtau(avg_error_x1000, ideal_x_q8, ideal_tau);
    
    // Aplica aprendizado com blending suave (rate-limit)
    // Blend: 87.5% valor antigo + 12.5% novo valor
    const int32_t x_delta = static_cast<int32_t>(ideal_x_q8) - 
                            static_cast<int32_t>(g_learned_params.x_fraction_q8);
    const int32_t tau_delta = static_cast<int32_t>(ideal_tau) - 
                              static_cast<int32_t>(g_learned_params.tau_cycles);
    
    g_learned_params.x_fraction_q8 = static_cast<uint16_t>(
        static_cast<int32_t>(g_learned_params.x_fraction_q8) + 
        ((x_delta * kLearningRateQ8) >> 8));
    
    g_learned_params.tau_cycles = static_cast<uint16_t>(
        static_cast<int32_t>(g_learned_params.tau_cycles) + 
        ((tau_delta * kLearningRateQ8) >> 8));
    
    // Atualiza limites aprendidos
    if (g_learned_params.x_fraction_q8 < g_learned_params.learned_x_min_q8) {
        g_learned_params.learned_x_min_q8 = g_learned_params.x_fraction_q8;
    }
    if (g_learned_params.x_fraction_q8 > g_learned_params.learned_x_max_q8) {
        g_learned_params.learned_x_max_q8 = g_learned_params.x_fraction_q8;
    }
    if (g_learned_params.tau_cycles < g_learned_params.learned_tau_min) {
        g_learned_params.learned_tau_min = g_learned_params.tau_cycles;
    }
    if (g_learned_params.tau_cycles > g_learned_params.learned_tau_max) {
        g_learned_params.learned_tau_max = g_learned_params.tau_cycles;
    }
    
    // Atualiza estado
    g_wall_state.calibration_state = 2u;  // calibrated
    ++g_xtau_calib_updates;
    
    // Reseta histórico para próximo ciclo de aprendizado
    lambda_error_reset();
    
    return true;
}

XTauParams xtau_get_current_params(int16_t clt_x10) noexcept {
    XTauParams params = {};
    
    // Se já temos parâmetros aprendidos, usa-os
    if (g_wall_state.calibration_state >= 1u) {
        params.x_fraction_q8 = g_learned_params.x_fraction_q8;
        params.tau_cycles = g_learned_params.tau_cycles;
    } else {
        // Caso contrário, interpola da tabela baseada em CLT
        params.x_fraction_q8 = interp_u16_8pt(
            xtau_clt_axis_x10, xtau_x_fraction_q8, kCorrectionTableSize, clt_x10);
        params.tau_cycles = interp_u16_8pt(
            xtau_clt_axis_x10, xtau_tau_cycles, kCorrectionTableSize, clt_x10);
    }
    
    params.learned_x_min_q8 = g_learned_params.learned_x_min_q8;
    params.learned_x_max_q8 = g_learned_params.learned_x_max_q8;
    params.learned_tau_min = g_learned_params.learned_tau_min;
    params.learned_tau_max = g_learned_params.learned_tau_max;
    
    return params;
}

uint32_t transient_fuel_xtau_with_autocalib(uint32_t fuel_pw_us,
                                             int16_t clt_x10,
                                             bool enabled) noexcept {
    if (!enabled || fuel_pw_us == 0u) {
        transient_fuel_reset();
        g_wall_state.calibration_state = 0u;
        return fuel_pw_us;
    }
    
    // Obtém parâmetros atuais (aprendidos ou de tabela)
    const XTauParams params = xtau_get_current_params(clt_x10);
    
    // Reutiliza lógica do transient_fuel.cpp com parâmetros aprendidos
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
    
    constexpr uint32_t kMaxPwUs = 100000u;
    const uint32_t clamped_pw = fuel_pw_us > kMaxPwUs ? kMaxPwUs : fuel_pw_us;
    const int32_t desired_q8 = static_cast<int32_t>(clamped_pw << 8u);
    const int32_t evap_q8 = g_wall_state.wall_fuel_us_q8 / static_cast<int32_t>(tau);
    int32_t numerator_q8 = desired_q8 - evap_q8;
    if (numerator_q8 < 0) {
        numerator_q8 = 0;
    }
    
    const int32_t dry_fraction_q8 = 256 - static_cast<int32_t>(x_q8);
    // numerator_q8 chega a ~25.6M (kMaxPwUs<<8); ×256 estoura int32_t.
    // Intermediário de 64 bits evita overflow; o resultado pós-clamp cabe em int32_t.
    int32_t injected_q8 = static_cast<int32_t>(
        (static_cast<int64_t>(numerator_q8) * 256) / dry_fraction_q8);
    const int32_t max_q8 = static_cast<int32_t>(kMaxPwUs << 8u);
    if (injected_q8 > max_q8) {
        injected_q8 = max_q8;
    }
    
    g_wall_state.wall_fuel_us_q8 += ((injected_q8 * static_cast<int32_t>(x_q8)) >> 8) - evap_q8;
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
