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

// =====================================================================