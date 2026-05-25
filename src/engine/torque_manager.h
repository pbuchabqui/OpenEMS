/**
 * @file torque_manager.h
 * @brief Gerenciador Central de Torque - Arbitragem de Pedal, Idle e Segurança
 * 
 * Camada que unifica:
 * - Pedido do motorista (pedal com mapa de resposta)
 * - Pedido de marcha lenta
 * - Limites de proteção (RPM, velocidade, temperatura)
 * - Controles auxiliares (tração, cruise, limiters)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// ESTRUTURAS DE DADOS
// ============================================================================

typedef struct {
    // Entradas
    float pedal_percent;        // 0-100% do acelerador
    float engine_rpm;           // RPM atual
    float vehicle_speed;        // km/h (opcional)
    float coolant_temp;         // °C
    float intake_air_temp;      // °C
    
    // Estados
    bool idle_mode;             // true se controle de idle ativo
    bool traction_active;       // Controle de tração atuando
    bool launch_control;        // Launch control ativo
    bool limp_mode;             // Modo emergência
    
    // Pedidos de subsistemas
    float cruise_target;        // Pedido cruise control (%)
    float tc_reduction;         // Redução por tração (%)
} torque_manager_inputs_t;

typedef struct {
    // Saídas principais
    float throttle_target;      // Alvo final borboleta (%)
    float torque_limit;         // Limite de torque (% do máximo)
    float spark_trim;           // Trim de ignição por controle torque
    
    // Diagnósticos
    uint32_t intervention_count; // Quantas vezes limitou torque
    uint32_t rpm_cutoff_count;   // Cortes por RPM
    uint32_t tc_intervention;    // Intervenções tração
    float last_update_ms;
} torque_manager_outputs_t;

typedef struct {
    // Limites de proteção
    float max_rpm;              // Corte giro
    float max_rpm_hot;          // Corte se motor quente
    float max_speed;            // Limitador velocidade
    float min_coolant_temp;     // Não acelerar se frio extremo
    
    // Launch control
    float launch_rpm;           // RPM limite launch
    float launch_throttle;      // Throttle máximo launch
    
    // Traction control
    float tc_max_slip;          // Slip máximo permitido (%)
    float tc_reduction_rate;    // Taxa redução torque (%/s)
    
    // Filtros
    float pedal_filter_alpha;   // Filtro passa-baixa pedal
} torque_manager_config_t;

// ============================================================================
// API PÚBLICA
// ============================================================================

/**
 * @brief Inicializa gerenciador de torque
 */
bool torque_manager_init(void);

/**
 * @brief Loop principal (chamar a 1kHz ou mais rápido)
 * @param inputs Estrutura de entradas
 * @param outputs Estrutura de saídas (preenchida pela função)
 */
void torque_manager_loop(const torque_manager_inputs_t* inputs,
                         torque_manager_outputs_t* outputs);

/**
 * @brief Define configuração do gerenciador
 */
void torque_manager_set_config(const torque_manager_config_t* config);

/**
 * @brief Obtém configuração atual
 */
const torque_manager_config_t* torque_manager_get_config(void);

/**
 * @brief Força modo emergência
 */
void torque_manager_enter_limp(void);

/**
 * @brief Verifica se sistema está pronto
 */
bool torque_manager_is_ready(void);

#ifdef __cplusplus
}
#endif
