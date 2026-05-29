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
#include "etb_driver.h"
#include <string.h>
#include <math.h>

// =====================================================================