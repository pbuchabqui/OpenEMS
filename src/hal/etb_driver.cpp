/**
 * @file etb_driver.cpp
 * @brief Implementação HAL Driver Borboleta Eletrônica (ETB)
 * 
 * STM32H562 - TIM1 PWM com dead-time, ADC dual para TPS
 */

#include "etb_driver.h"
#include "stm32h5xx.h"
#include "adc.h"      // HAL ADC existente
#include "timer.h"    // HAL Timer existente
#include "stm32h562/gpio.h"  // GPIO HAL específico
#include <string.h>

// =====================================================================