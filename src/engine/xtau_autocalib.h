#pragma once

#include <cstdint>

namespace ems::engine {

// Estrutura de estado do modelo de parede de combustível (X-τ)
struct WallFuelState {
    int32_t wall_fuel_us_q8;      // Combustível acumulado na parede (Q8)
    uint32_t last_update_ms;       // Timestamp da última atualização
    uint8_t calibration_state;     // 0=inactive, 1=learning, 2=calibrated
    uint8_t learning_attempts;     // Contador de tentativas de calibração
};

// Parâmetros X-τ auto-calibráveis
struct XTauParams {
    uint16_t x_fraction_q8;        // Fração que vai para a parede (Q8)
    uint16_t tau_cycles;           // Constante de tempo em ciclos
    uint16_t learned_x_min_q8;     // X mínimo aprendido
    uint16_t learned_x_max_q8;     // X máximo aprendido
    uint16_t learned_tau_min;      // Tau mínimo aprendido
    uint16_t learned_tau_max;      // Tau máximo aprendido
};

// Inicializa o sistema de auto-calibração X-τ
void xtau_autocalib_init() noexcept;

// Reseta o estado de aprendizado
void xtau_autocalib_reset() noexcept;

// Atualiza parâmetros X-τ baseado em erro de lambda durante transientes
// Retorna true se parâmetros foram atualizados
bool xtau_autocalib_update(uint32_t rpm_x10,
                           uint16_t map_bar_x100,
                           int16_t lambda_target_x1000,
                           int16_t lambda_measured_x1000,
                           int16_t clt_x10,
                           bool is_transient) noexcept;

// Obtém parâmetros X-τ atuais (fallback 1D por CLT)
XTauParams xtau_get_current_params(int16_t clt_x10) noexcept;

// Obtém parâmetros X-τ aprendidos na célula RPM×MAP (interpolado), para
// diagnóstico/dash e testes — não aplica fallback 1D por CLT.
XTauParams xtau_get_current_params_2d(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;

// Aplica modelo X-τ com parâmetros aprendidos, indexados por RPM×MAP (carga)
uint32_t transient_fuel_xtau_with_autocalib(uint32_t fuel_pw_us,
                                             uint32_t rpm_x10,
                                             uint16_t map_bar_x100,
                                             int16_t clt_x10,
                                             bool enabled) noexcept;

// Verifica se sistema está em modo de aprendizado
bool xtau_is_learning() noexcept;

// Obtém estado atual para diagnóstico
WallFuelState xtau_get_state() noexcept;

}  // namespace ems::engine
