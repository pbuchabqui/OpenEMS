/**
 * @file etb_control.cpp
 * @brief Implementação Controle Borboleta Eletrônica (ETB)
 * 
 * - PID em cascata (posição + velocidade)
 * - Mapas de resposta por modo (Eco/Normal/Sport/Rain)
 * - Controle de marcha lenta integrado
 * - Segurança e diagnósticos
 */

#include "etb_control.h"
#include "hal/etb_driver.h"
#include <string.h>
#include <math.h>

// ============================================================================
// CONSTANTES E CONFIGURAÇÕES PADRÃO
// ============================================================================

// Configurações PID padrão (ajustar por veículo)
static const etb_pid_config_t g_default_pid_config = {
    .kp_pos = 2.5f,
    .ki_pos = 0.1f,
    .kd_pos = 0.05f,
    .kp_vel = 1.8f,
    .ki_vel = 0.05f,
    .kd_vel = 0.02f,
    .ff_friction = 8.0f,     // 8% PWM para vencer atrito
    .ff_inertia = 0.3f,      // Compensação inércia
    .max_opening = 95.0f,    // Limite físico
    .min_closing = 3.0f,     // Idle air mínimo
    .ramp_rate = 300.0f,     // 300%/s máximo
    .deadband_center = 5.0f,
    .deadband_width = 2.0f
};

// Mapas de resposta por modo (10 pontos: 0%, 10%, ..., 90%, 100% pedal)
static const float g_response_maps[ETB_MODE_COUNT][10] = {
    // ECO: Suave, progressivo
    {0.0f, 8.0f, 15.0f, 22.0f, 30.0f, 40.0f, 52.0f, 65.0f, 80.0f, 100.0f},
    
    // NORMAL: Linear OEM
    {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 100.0f},
    
    // SPORT: Agressivo nos primeiros 30%
    {0.0f, 18.0f, 35.0f, 50.0f, 60.0f, 70.0f, 78.0f, 85.0f, 92.0f, 100.0f},
    
    // RAIN: Muito suave, tração
    {0.0f, 5.0f, 10.0f, 15.0f, 22.0f, 30.0f, 40.0f, 52.0f, 65.0f, 100.0f}
};

// ============================================================================
// VARIÁVEIS GLOBAIS
// ============================================================================

static etb_system_config_t g_config = {};
static etb_control_data_t g_data = {};
static etb_driver_data_t g_driver_data = {};
static bool g_initialized = false;
static bool g_limp_mode = false;

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

// Interpolação linear em mapa de resposta
static float interpolate_response(const float* map, float pedal) {
    // Clamp pedal 0-100
    if (pedal < 0.0f) pedal = 0.0f;
    if (pedal > 100.0f) pedal = 100.0f;
    
    // Índice no mapa (10 pontos = 0-9)
    float index = (pedal / 100.0f) * 9.0f;
    int idx_low = (int)index;
    int idx_high = idx_low + 1;
    
    if (idx_high >= 10) {
        return map[9];
    }
    
    // Fração para interpolação
    float frac = index - (float)idx_low;
    
    return map[idx_low] + (map[idx_high] - map[idx_low]) * frac;
}

// Limitador de taxa (ramp rate limiter)
static float apply_ramp_limit(float current, float target, float max_rate, float dt_ms) {
    float max_change = (max_rate * dt_ms) / 1000.0f; // % neste ciclo
    
    if (target > current + max_change) {
        return current + max_change;
    } else if (target < current - max_change) {
        return current - max_change;
    }
    
    return target;
}

// PID with conditional anti-windup and resetable derivative state.
// filtered_deriv must be stored in caller's struct so it survives across calls
// and can be zeroed on re-init (was previously an unresetable static local).
// Conditional anti-windup: integral only accumulates when the output is not
// saturated, preventing integrator wind-up during throttle stops or RPM cuts.
static float calculate_pid(float error, float* integral, float* prev_error,
                           float* filtered_deriv,
                           float kp, float ki, float kd, float dt_ms,
                           float max_integral, float output_limit) {
    float p = kp * error;

    // Conditional anti-windup: accumulate only when not saturated in error direction
    const float candidate_integral = *integral + error * (dt_ms / 1000.0f);
    const float candidate_out = p + ki * candidate_integral;
    const bool saturated_open  = (candidate_out >  output_limit) && (error > 0.0f);
    const bool saturated_close = (candidate_out < -output_limit) && (error < 0.0f);
    if (!saturated_open && !saturated_close) {
        *integral = candidate_integral;
    }
    if (*integral >  max_integral) { *integral =  max_integral; }
    if (*integral < -max_integral) { *integral = -max_integral; }
    float i = ki * (*integral);

    float deriv = (error - (*prev_error)) / (dt_ms / 1000.0f);
    *prev_error = error;
    *filtered_deriv = (*filtered_deriv) * 0.8f + deriv * 0.2f;
    float d = kd * (*filtered_deriv);

    return p + i + d;
}

// ============================================================================
// INICIALIZAÇÃO
// ============================================================================

bool etb_control_init(void) {
    // 1. Inicializar driver HAL
    if (!etb_driver_init()) {
        return false;
    }
    
    // 2. Reset dados
    memset(&g_config, 0, sizeof(g_config));
    memset(&g_data, 0, sizeof(g_data));
    
    // 3. Carregar configurações padrão
    g_config.current_mode = ETB_MODE_NORMAL;
    
    // Copiar PID config para todos os modos
    for (int i = 0; i < ETB_MODE_COUNT; i++) {
        memcpy(&g_config.pid_configs[i], &g_default_pid_config, sizeof(etb_pid_config_t));
        memcpy(g_config.response_maps[i], g_response_maps[i], sizeof(float) * 10);
    }
    
    // 4. Parâmetros de marcha lenta
    g_config.idle_rpm_target = 850.0f;
    g_config.idle_min_opening = 3.0f;
    g_config.idle_max_opening = 8.0f;
    g_config.idle_spark_advance = 15.0f;   // 15° base
    g_config.idle_spark_retard = 10.0f;    // Até 10° retardo
    
    // 5. Segurança
    g_config.rpm_cutoff = 7000.0f;
    g_config.tps_rate_limit = 500.0f;  // 500%/s
    g_config.limp_opening = 5.0f;
    
    // 6. Homing (aprendizado posição fechada)
    // NOTA: Implementar rotina de homing que move borboleta até batente
    // e aprende a posição mínima real
    g_data.homing_count = 1;
    
    // 7. Ler sensores inicialmente
    etb_driver_read_sensors(&g_driver_data);
    
    g_initialized = true;
    g_limp_mode = false;
    
    return true;
}

// ============================================================================
// LOOP DE CONTROLE PRINCIPAL (1kHz)
// ============================================================================

void etb_control_loop(float pedal, float rpm, float dt) {
    if (!g_initialized) return;
    
    // Atualizar timestamp
    g_data.last_update_ms += dt;
    
    // 1. Ler sensores TPS
    etb_driver_fault_t fault = etb_driver_read_sensors(&g_driver_data);
    
    if (fault != ETB_DRV_OK) {
        g_data.fault_count++;
        etb_enter_limp_mode();
        return;
    }
    
    g_data.throttle_actual = g_driver_data.tps_validated;
    g_data.pedal_percent = pedal;
    
    // 2. Verificar cortes de segurança
    if (rpm > g_config.rpm_cutoff) {
        // Corte por RPM - fecha borboleta
        g_data.throttle_target = g_config.idle_min_opening;
    } else if (g_limp_mode) {
        // Modo emergência
        g_data.throttle_target = g_config.limp_opening;
    } else {
        // 3. Aplicar mapa de resposta conforme modo
        float raw_target = interpolate_response(
            g_config.response_maps[g_config.current_mode], 
            pedal
        );
        
        // 4. Adicionar offset de marcha lenta se ativo
        if (g_data.idle_active && rpm < (g_config.idle_rpm_target + 200.0f)) {
            // Calcular offset necessário para manter idle
            float idle_error = g_config.idle_rpm_target - rpm;
            
            // Proporcional simples para idle offset
            float idle_add = idle_error * 0.05f; // 0.05% por RPM de erro
            
            // Clamp
            if (idle_add < 0.0f) idle_add = 0.0f;
            if (idle_add > (g_config.idle_max_opening - g_config.idle_min_opening)) {
                idle_add = g_config.idle_max_opening - g_config.idle_min_opening;
            }
            
            g_data.idle_offset = g_config.idle_min_opening + idle_add;
            
            // Idle adiciona ao target (sempre abre mais)
            if (raw_target < g_data.idle_offset) {
                raw_target = g_data.idle_offset;
            }
            
            // 5. Calcular trim de ignição para idle (controle fino)
            // Erro pequeno → usa ignição, erro grande → usa ar
            if (fabsf(idle_error) < 100.0f) {
                // ±5° de trim baseado no erro
                g_data.idle_spark_trim = (int16_t)(idle_error * 0.5f); // 0.5° por 100 RPM
                
                // Clamp ±10° (±100 em x10)
                if (g_data.idle_spark_trim > 100) g_data.idle_spark_trim = 100;
                if (g_data.idle_spark_trim < -100) g_data.idle_spark_trim = -100;
            } else {
                g_data.idle_spark_trim = 0;
            }
        } else {
            g_data.idle_offset = 0.0f;
            g_data.idle_spark_trim = 0;
            g_data.idle_active = false;
        }
        
        // 6. Aplicar ramp limit (suavização)
        g_data.throttle_target = apply_ramp_limit(
            g_data.throttle_target,
            raw_target,
            g_config.tps_rate_limit,
            dt
        );
    }
    
    // 7. PID de Posição (cascata externa)
    g_data.pos_error = g_data.throttle_target - g_data.throttle_actual;

    float pos_output = calculate_pid(
        g_data.pos_error,
        &g_data.pos_integral,
        &g_data.pos_derivative,
        &g_data.pos_filtered_deriv,
        g_config.pid_configs[g_config.current_mode].kp_pos,
        g_config.pid_configs[g_config.current_mode].ki_pos,
        g_config.pid_configs[g_config.current_mode].kd_pos,
        dt,
        50.0f,   // max_integral
        100.0f   // output_limit (maps to ±1023 motor PWM via ×10.23)
    );
    
    // 8. Feed-forward (atrito + inércia)
    float ff_output = g_config.pid_configs[g_config.current_mode].ff_friction;
    
    // Adicionar compensação de inércia se acelerando
    if (g_data.pos_error > 2.0f) { // Abrindo
        ff_output += g_config.pid_configs[g_config.current_mode].ff_inertia * 
                     (g_data.throttle_target - g_data.throttle_actual);
    }
    
    // 9. Comando final ao driver
    g_data.throttle_command = pos_output + ff_output;
    
    // Converter para PWM do driver (-1023 a +1023)
    int16_t motor_pwm = (int16_t)(g_data.throttle_command * 10.23f);
    
    // Clamp
    if (motor_pwm > 1023) motor_pwm = 1023;
    if (motor_pwm < -1023) motor_pwm = -1023;
    
    // 10. Enviar comando ao driver HAL
    etb_driver_set_motor_pwm(motor_pwm);
}

// ============================================================================
// CONTROLE DE MARCHA LENTA
// ============================================================================

void etb_set_idle_control(bool active, float target_rpm) {
    g_data.idle_active = active;
    if (active) {
        g_config.idle_rpm_target = target_rpm;
    }
}

// FIX C2: must be in ems::engine namespace to override the weak symbol in
// ign_calc.cpp. Defined outside the namespace it was unreachable from
// ems::engine::etb_get_idle_spark_trim() — linker kept the weak stub (return 0),
// so idle spark trim was always zero and idle control via ignition was broken.
namespace ems::engine {
int16_t etb_get_idle_spark_trim() noexcept {
    return g_data.idle_spark_trim;
}
}  // namespace ems::engine

// ============================================================================
// MODOS DE CONDUÇÃO
// ============================================================================

void etb_set_drive_mode(etb_drive_mode_t mode) {
    if (mode >= ETB_MODE_COUNT) return;
    g_config.current_mode = mode;
}

etb_drive_mode_t etb_get_drive_mode(void) {
    return g_config.current_mode;
}

// ============================================================================
// ESTADO E DIAGNÓSTICOS
// ============================================================================

float etb_get_throttle_position(void) {
    return g_data.throttle_actual;
}

bool etb_is_ready(void) {
    return g_initialized && !g_limp_mode && 
           (etb_driver_get_state() == ETB_DRV_STATE_READY);
}

void etb_enter_limp_mode(void) {
    g_limp_mode = true;
    g_data.fault_count++;
    
    // Força borboleta em posição segura
    g_data.throttle_target = g_config.limp_opening;
}

// ─── New integer API (ems::engine namespace) ─────────────────────────────────
#include "engine/calibration.h"
#include "engine/math_utils.h"

namespace ems::engine {

static int32_t g_integrator_x10 = 0;
static int16_t g_prev_error_x10 = 0;

void etb_control_reset() noexcept {
    g_integrator_x10 = 0;
    g_prev_error_x10 = 0;
}

EtbControlState etb_control_update(uint16_t target_pct_x10,
                                   uint16_t measured_pct_x10,
                                   bool     enable_request,
                                   uint16_t period_ms) noexcept
{
    EtbControlState out{};

    if (!enable_request || etb_cal_valid == 0u) {
        etb_control_reset();
        return out;
    }

    out.active = true;

    const int16_t error_x10 = clamp_i16(
        static_cast<int32_t>(target_pct_x10) - static_cast<int32_t>(measured_pct_x10),
        -1000, 1000);

    const int32_t kp_x10 = static_cast<int32_t>(etb_kp_x10);
    const int32_t ki_x10 = static_cast<int32_t>(etb_ki_x10);
    const int32_t kd_x10 = static_cast<int32_t>(etb_kd_x10);

    // P term
    int32_t output_x10 = (kp_x10 * static_cast<int32_t>(error_x10)) / 10;

    // I term with anti-windup — guard must check P+I saturation, not P alone.
    // Old code checked output_x10 (P-only) which allowed unlimited windup against
    // mechanical stops: small error → small P (within ±100%) → integral accumulates
    // unbounded → large overshoot on throttle release.
    if (period_ms > 0u) {
        const int32_t ki_inc = (ki_x10 * static_cast<int32_t>(error_x10)
                                * static_cast<int32_t>(period_ms)) / 1000;
        const int32_t raw_i = g_integrator_x10 + ki_inc;
        const int32_t candidate_i = (raw_i > 2000) ? 2000 : (raw_i < -2000 ? -2000 : raw_i);
        const int32_t candidate_out = output_x10 + candidate_i;  // P + I
        // Reject accumulation only when P+I is saturated in the same direction as error
        const bool saturating = (candidate_out >  1000 && error_x10 > 0) ||
                                (candidate_out < -1000 && error_x10 < 0);
        if (!saturating) {
            g_integrator_x10 = static_cast<int16_t>(candidate_i);
        }
    }
    g_integrator_x10 = clamp_i16(static_cast<int32_t>(g_integrator_x10), -2000, 2000);
    output_x10 += g_integrator_x10;

    // D term
    if (period_ms > 0u) {
        const int32_t deriv = ((static_cast<int32_t>(error_x10) - g_prev_error_x10)
                               * 1000) / static_cast<int32_t>(period_ms);
        output_x10 += (kd_x10 * deriv) / 10;
    }
    g_prev_error_x10 = error_x10;

    out.output_pct_x10     = clamp_i16(output_x10, -1000, 1000);
    out.position_error_x10 = error_x10;
    return out;
}

int32_t etb_control_test_get_integrator() noexcept {
    return g_integrator_x10;
}

}  // namespace ems::engine
