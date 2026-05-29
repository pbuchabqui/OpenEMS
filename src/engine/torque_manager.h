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

// =====================================================================