/**
 * @file etb_driver.h
 * @brief HAL Driver para Borboleta Eletrônica (ETB)
 * 
 * Controle de motor DC com ponte H, leitura dual de TPS e proteção de hardware.
 * STM32H562 - Timer PWM com dead-time inserido por hardware.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================