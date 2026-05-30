/**
 * @file torque_manager.cpp
 * @brief Implementação Gerenciador Central de Torque
 * 
 * Arbitra entre:
 * - Pedido do motorista (pedal)
 * - Controle de marcha lenta
 * - Limites de segurança (RPM, temperatura)
 * - Controles auxiliares (tração, launch, cruise)
 */

#include "torque_manager.h"
#include <string.h>
#include <math.h>

// ============================================================================
// VARIÁVEIS GLOBAIS
// ============================================================================

static torque_manager_config_t g_config = {};
static torque_manager_outputs_t g_outputs = {};
static bool g_initialized = false;
static bool g_limp_active = false;

// Filtro do pedal
static float g_filtered_pedal = 0.0f;

// ============================================================================
// INICIALIZAÇÃO
// ============================================================================

bool torque_manager_init(void) {
    memset(&g_config, 0, sizeof(g_config));
    memset(&g_outputs, 0, sizeof(g_outputs));
    
    // Configurações padrão de segurança
    g_config.max_rpm = 7000.0f;           // Corte giro padrão
    g_config.max_rpm_hot = 7200.0f;       // Corte se motor > 105°C
    g_config.max_speed = 220.0f;          // Limitador 220 km/h
    g_config.min_coolant_temp = 0.0f;     // Não acelerar se < 0°C (proteção)
    
    // Launch control
    g_config.launch_rpm = 4500.0f;        // RPM launch padrão
    g_config.launch_throttle = 60.0f;     // 60% throttle em launch
    
    // Traction control
    g_config.tc_max_slip = 15.0f;         // 15% slip máximo
    g_config.tc_reduction_rate = 50.0f;   // 50%/s redução suave
    
    // Filtro pedal (suavização)
    g_config.pedal_filter_alpha = 0.3f;   // 0.3 = resposta rápida mas filtrada
    
    g_filtered_pedal = 0.0f;
    g_initialized = true;
    g_limp_active = false;
    
    return true;
}

// ============================================================================
// LOOP PRINCIPAL DE ARBITRAGEM
// ============================================================================

void torque_manager_loop(const torque_manager_inputs_t* inputs,
                         torque_manager_outputs_t* outputs) {
    if (!g_initialized || inputs == NULL || outputs == NULL) {
        return;
    }
    
    // Atualizar timestamp
    g_outputs.last_update_ms += 1.0f; // Assumindo 1kHz
    
    // 1. Filtrar sinal do pedal (suavização)
    g_filtered_pedal = g_config.pedal_filter_alpha * inputs->pedal_percent +
                       (1.0f - g_config.pedal_filter_alpha) * g_filtered_pedal;
    
    float requested_throttle = g_filtered_pedal;
    
    // 2. Verificar modo emergência
    if (g_limp_active || inputs->limp_mode) {
        outputs->throttle_target = 5.0f;  // Abertura mínima segurança
        outputs->torque_limit = 30.0f;    // 30% torque máximo
        outputs->spark_trim = 0.0f;
        g_outputs.intervention_count++;
        return;
    }
    
    // 3. Controle de Marcha Lenta (prioridade alta)
    if (inputs->idle_mode) {
        // Idle assume controle total da borboleta
        // O valor exato será definido pelo etb_control
        // Aqui apenas garantimos que não haja interferência
        requested_throttle = 0.0f; // ETB calculará idle_offset
    }
    
    // 4. Launch Control (se ativo)
    if (inputs->launch_control) {
        if (inputs->engine_rpm > g_config.launch_rpm) {
            // Limitar throttle para manter RPM no launch
            requested_throttle = g_config.launch_throttle;
            g_outputs.intervention_count++;
        }
    }
    
    // 5. Corte por RPM (over-rev)
    if (inputs->engine_rpm >= g_config.max_rpm) {
        // Corte total ou parcial
        if (inputs->engine_rpm >= g_config.max_rpm + 200.0f) {
            requested_throttle = 0.0f;  // Corte total
        } else {
            // Corte progressivo (50-100% acima do limite)
            float cutoff_factor = 1.0f - ((inputs->engine_rpm - g_config.max_rpm) / 200.0f);
            requested_throttle *= cutoff_factor;
        }
        g_outputs.rpm_cutoff_count++;
    }
    
    // Limite RPM com motor quente
    if (inputs->coolant_temp > 105.0f && inputs->engine_rpm >= g_config.max_rpm_hot) {
        requested_throttle = 0.0f;
        g_outputs.rpm_cutoff_count++;
    }
    
    // 6. Limitador de Velocidade
    if (inputs->vehicle_speed > 0.1f && inputs->vehicle_speed >= g_config.max_speed) {
        // Não permitir mais aceleração
        if (requested_throttle > 10.0f) {
            requested_throttle = 10.0f; // Apenas mantém velocidade
        }
        g_outputs.intervention_count++;
    }
    
    // 7. Traction Control (redução de torque)
    if (inputs->traction_active) {
        // Reduzir throttle baseado no slip
        float reduction = inputs->tc_reduction;
        
        // Clamp redução
        if (reduction > 80.0f) reduction = 80.0f;
        
        requested_throttle *= (1.0f - (reduction / 100.0f));
        g_outputs.tc_intervention++;
    }
    
    // 8. Cruise Control (se ativo e sem override do motorista)
    if (inputs->cruise_target > 0.1f && inputs->pedal_percent < 5.0f) {
        // Cruise control assume quando motorista não está acelerando
        requested_throttle = inputs->cruise_target;
    }
    
    // 9. Proteção motor frio extremo
    if (inputs->coolant_temp < g_config.min_coolant_temp) {
        // Limitar throttle para proteger motor
        if (requested_throttle > 30.0f) {
            requested_throttle = 30.0f;
        }
    }
    
    // 10. Clamp final
    if (requested_throttle < 0.0f) requested_throttle = 0.0f;
    if (requested_throttle > 100.0f) requested_throttle = 100.0f;
    
    // 11. Calcular torque limite (para fuel_calc)
    // Torque limite é proporcional ao throttle target
    outputs->torque_limit = requested_throttle;
    
    // 12. Spark trim (para ignição - controle fino de torque)
    // Usado para correções rápidas de torque (ex: traction control)
    if (inputs->traction_active) {
        // Retardar ignição para reduzir torque rapidamente
        outputs->spark_trim = -(inputs->tc_reduction * 0.3f); // 0.3° por % redução
        
        // Clamp ±15°
        if (outputs->spark_trim < -150.0f) outputs->spark_trim = -150.0f;
        if (outputs->spark_trim > 50.0f) outputs->spark_trim = 50.0f;
    } else {
        outputs->spark_trim = 0.0f;
    }
    
    // 13. Saída final
    outputs->throttle_target = requested_throttle;
    
    // Copiar diagnósticos
    outputs->intervention_count = g_outputs.intervention_count;
    outputs->rpm_cutoff_count = g_outputs.rpm_cutoff_count;
    outputs->tc_intervention = g_outputs.tc_intervention;
}

// ============================================================================
// CONFIGURAÇÃO
// ============================================================================

void torque_manager_set_config(const torque_manager_config_t* config) {
    if (config == NULL) return;
    memcpy(&g_config, config, sizeof(torque_manager_config_t));
}

const torque_manager_config_t* torque_manager_get_config(void) {
    return &g_config;
}

// ============================================================================
// ESTADO E EMERGÊNCIA
// ============================================================================

void torque_manager_enter_limp(void) {
    g_limp_active = true;
}

bool torque_manager_is_ready(void) {
    return g_initialized && !g_limp_active;
}

// ─── New integer API (ems::engine namespace) ─────────────────────────────────
#include "engine/calibration.h"
#include "engine/math_utils.h"

namespace ems::engine {

static uint16_t g_etb_target_x10   = 0u;
static uint8_t  g_limp_reason      = 0u;

void torque_manager_reset() noexcept {
    g_etb_target_x10 = 0u;
    g_limp_reason    = 0u;
}

TorqueOutput torque_manager_update(
    const ems::drv::CkpSnapshot& snap,
    const ems::drv::SensorData&  sensors,
    bool key_on, bool map_clt_limp, bool rev_cut,
    uint16_t idle_target_rpm_x10, uint16_t /*period_ms*/) noexcept
{
    TorqueOutput out{};
    out.limp_reason = 0u;

    if (!key_on) {
        out.etb_enable_request = false;
        g_etb_target_x10 = 0u;
        g_limp_reason    = 0u;
        return out;
    }

    if (etb_cal_valid == 0u) {
        out.limp_reason |= TORQUE_LIMP_NO_CALIB;
    }
    if (map_clt_limp) {
        out.limp_reason |= TORQUE_LIMP_MAP_CLT;
    }
    if (rev_cut) {
        out.limp_reason |= TORQUE_LIMP_REV_CUT;
    }
    if ((sensors.throttle_fault_bits & (ems::drv::THROTTLE_FAULT_APP1 |
                                         ems::drv::THROTTLE_FAULT_APP2 |
                                         ems::drv::THROTTLE_FAULT_APP_PLAUS)) != 0u) {
        out.limp_reason |= TORQUE_LIMP_APP_FAULT;
    }
    if ((sensors.throttle_fault_bits & (ems::drv::THROTTLE_FAULT_ETB_TPS1 |
                                         ems::drv::THROTTLE_FAULT_ETB_TPS2 |
                                         ems::drv::THROTTLE_FAULT_ETB_PLAUS)) != 0u) {
        out.limp_reason |= TORQUE_LIMP_ETB_FAULT;
    }

    // Base target from driver pedal
    uint16_t target_x10 = sensors.app_pct_x10;

    // Limp mode: clamp to etb_max_open_pct_x10_limp
    if ((out.limp_reason & (TORQUE_LIMP_MAP_CLT | TORQUE_LIMP_APP_FAULT)) != 0u) {
        if (target_x10 > etb_max_open_pct_x10_limp) {
            target_x10 = etb_max_open_pct_x10_limp;
        }
    }
    if ((out.limp_reason & (TORQUE_LIMP_REV_CUT | TORQUE_LIMP_NO_CALIB)) != 0u) {
        target_x10 = 0u;
    }

    // Idle boost: if RPM below target, open slightly
    if (snap.rpm_x10 > 0u && snap.rpm_x10 < idle_target_rpm_x10 &&
        sensors.app_pct_x10 < etb_idle_open_pct_x10) {
        const uint16_t idle_add =
            static_cast<uint16_t>(
                (static_cast<uint32_t>(etb_idle_open_pct_x10 - sensors.app_pct_x10) *
                 (idle_target_rpm_x10 - snap.rpm_x10)) /
                static_cast<uint32_t>(idle_target_rpm_x10 > 0u ? idle_target_rpm_x10 : 1u));
        target_x10 = static_cast<uint16_t>(
            clamp_u16(static_cast<uint16_t>(target_x10 + idle_add), 0u, 1000u));
    }

    g_etb_target_x10 = target_x10;
    g_limp_reason    = out.limp_reason;

    out.etb_target_pct_x10     = target_x10;
    out.etb_max_rate_pct_per_s = etb_max_rate_pct_per_s;
    out.etb_enable_request     = (out.limp_reason & TORQUE_LIMP_ETB_FAULT) == 0u;
    return out;
}

uint16_t torque_manager_test_get_target()      noexcept { return g_etb_target_x10; }
uint8_t  torque_manager_test_get_limp_reason() noexcept { return g_limp_reason; }

}  // namespace ems::engine
