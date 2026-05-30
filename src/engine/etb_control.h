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
#include "hal/etb_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================