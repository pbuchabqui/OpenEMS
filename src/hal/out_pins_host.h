#pragma once
/**
 * Host-test GPIO mock registers for out_pins hot path + init.
 * TIM5 mocks stay in ecu_sched.cpp (not covered here).
 */
#if defined(EMS_HOST_TEST)

#include <cstdint>

namespace ems::hal::out_pins_host {
extern uint32_t rcc_ahb2enr1;
extern uint32_t gpioa_moder, gpiob_moder, gpioc_moder, gpioe_moder;
extern uint32_t gpioa_otyper, gpiob_otyper, gpioc_otyper, gpioe_otyper;
extern uint32_t gpioa_pupdr, gpiob_pupdr, gpioc_pupdr, gpioe_pupdr;
extern uint32_t gpioa_afrh;
extern uint32_t gpioa_bsrr, gpiob_bsrr, gpioc_bsrr, gpioe_bsrr;
}  // namespace ems::hal::out_pins_host

#define RCC_AHB2ENR1 ems::hal::out_pins_host::rcc_ahb2enr1
#define GPIOA_MODER  ems::hal::out_pins_host::gpioa_moder
#define GPIOB_MODER  ems::hal::out_pins_host::gpiob_moder
#define GPIOC_MODER  ems::hal::out_pins_host::gpioc_moder
#define GPIOE_MODER  ems::hal::out_pins_host::gpioe_moder
#define GPIOA_OTYPER ems::hal::out_pins_host::gpioa_otyper
#define GPIOB_OTYPER ems::hal::out_pins_host::gpiob_otyper
#define GPIOC_OTYPER ems::hal::out_pins_host::gpioc_otyper
#define GPIOE_OTYPER ems::hal::out_pins_host::gpioe_otyper
#define GPIOA_PUPDR  ems::hal::out_pins_host::gpioa_pupdr
#define GPIOB_PUPDR  ems::hal::out_pins_host::gpiob_pupdr
#define GPIOC_PUPDR  ems::hal::out_pins_host::gpioc_pupdr
#define GPIOE_PUPDR  ems::hal::out_pins_host::gpioe_pupdr
#define GPIOA_AFRH   ems::hal::out_pins_host::gpioa_afrh
#define GPIOA_BSRR   ems::hal::out_pins_host::gpioa_bsrr
#define GPIOB_BSRR   ems::hal::out_pins_host::gpiob_bsrr
#define GPIOC_BSRR   ems::hal::out_pins_host::gpioc_bsrr
#define GPIOE_BSRR   ems::hal::out_pins_host::gpioe_bsrr

#ifndef RCC_AHB2ENR1_GPIOAEN
#define RCC_AHB2ENR1_GPIOAEN 1U
#define RCC_AHB2ENR1_GPIOBEN 2U
#define RCC_AHB2ENR1_GPIOCEN 4U
#define RCC_AHB2ENR1_GPIOEEN 16U
#endif

#endif  // EMS_HOST_TEST
