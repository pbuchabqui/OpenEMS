/**
 * @file etb_control.h
 * @brief Controle de Borboleta Eletrônica (ETB) com PID em Cascata
 * 
 * - Mapa de resposta ajustável (Eco/Normal/Sport/Rain)
 * - Controle de marcha lenta integrado
 * - PID duplo: posição + velocidade
 * - Segurança e diagnósticos
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "etb_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// MODOS DE CONDUÇÃO (DRIVE MODES)
// ============================================================================

typedef enum {
    ETB_MODE_ECO = 0,     // Resposta suave, economia máxima
    ETB_MODE_NORMAL,      // Resposta linear OEM
    ETB_MODE_SPORT,       // Resposta agressiva nos primeiros 30%
    ETB_MODE_RAIN,        // Resposta muito suave, tração
    ETB_MODE_COUNT
} etb_drive_mode_t;

// ============================================================================
// ESTRUTURAS DE CONTROLE
// ============================================================================

typedef struct {
    // Ganhos PID Posição
    float kp_pos;
    float ki_pos;
    float kd_pos;
    
    // Ganhos PID Velocidade (cascata interna)
    float kp_vel;
    float ki_vel;
    float kd_vel;
    
    // Feed-forward (atrito + inércia)
    float ff_friction;    // Tensão mínima para vencer atrito estático
    float ff_inertia;     // Compensação de aceleração
    
    // Limites
    float max_opening;    // Abertura máxima (%) - limite físico
    float min_closing;    // Fechamento mínimo (%) - idle air
    float ramp_rate;      // Taxa máxima de variação (%/s)
    
    // Deadband mecânica
    float deadband_center; // Ponto médio da folga mecânica
    float deadband_width;  // Largura da zona morta
} etb_pid_config_t;

typedef struct {
    // Estado atual
    float pedal_percent;          // 0-100% do acelerador
    float throttle_target;        // Alvo calculado (pós-mapa)
    float throttle_actual;        // Leitura real TPS
    float throttle_command;       // Comando enviado ao driver
    
    // PID interno
    float pos_error;
    float pos_integral;
    float pos_derivative;
    float pos_filtered_deriv;  // derivative low-pass state (was an unresetable static)
    float vel_error;
    float vel_integral;
    
    // Marcha lenta
    float idle_offset;            // Adição de abertura para idle
    float idle_spark_trim;        // Trim de ignição para idle (graus x10)
    bool idle_active;
    
    // Diagnósticos
    uint32_t homing_count;
    uint32_t fault_count;
    float last_update_ms;
} etb_control_data_t;

// ============================================================================
// CONFIGURAÇÃO GLOBAL
// ============================================================================

typedef struct {
    etb_drive_mode_t current_mode;
    etb_pid_config_t pid_configs[ETB_MODE_COUNT];
    
    // Mapas de resposta por modo (pedal → throttle target)
    // 10 pontos por modo para interpolação
    float response_maps[ETB_MODE_COUNT][10];
    
    // Parâmetros de marcha lenta
    float idle_rpm_target;
    float idle_min_opening;
    float idle_max_opening;
    float idle_spark_advance;   // Avanço base para idle
    float idle_spark_retard;    // Retardo máximo para correção
    
    // Segurança
    float rpm_cutoff;           // Corte por RPM
    float tps_rate_limit;       // Limite de derivada TPS (%/s)
    float limp_opening;         // Abertura em modo emergência
} etb_system_config_t;

// ============================================================================
// API PÚBLICA
// ============================================================================

/**
 * @brief Inicializa sistema ETB (carrega configs, faz homing)
 * @return true se sucesso
 */
bool etb_control_init(void);

/**
 * @brief Loop de controle principal (chamar a 1kHz)
 * @param pedal Percentual do pedal (0-100)
 * @param rpm RPM atual do motor
 * @param dt Tempo desde última chamada (ms)
 */
void etb_control_loop(float pedal, float rpm, float dt);

/**
 * @brief Define modo de condução
 */
void etb_set_drive_mode(etb_drive_mode_t mode);

/**
 * @brief Obtém modo atual
 */
etb_drive_mode_t etb_get_drive_mode(void);

/**
 * @brief Ativa/desativa controle de marcha lenta
 * @param active true para ativar
 * @param target_rpm RPM alvo
 */
void etb_set_idle_control(bool active, float target_rpm);

/**
 * @brief Obtém trim de ignição para controle de idle
 * @return Trim em graus x10 (ex: -50 = -5°)
 */
int16_t etb_get_idle_spark_trim(void);

/**
 * @brief Obtém abertura atual da borboleta
 * @return 0-100%
 */
float etb_get_throttle_position(void);

/**
 * @brief Obtém estado do sistema
 */
bool etb_is_ready(void);

/**
 * @brief Força modo LIMP (emergência)
 */
void etb_enter_limp_mode(void);

#ifdef __cplusplus
}
#endif
