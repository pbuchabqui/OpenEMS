/**
 * @file etb_autocal.h
 * @brief Auto-calibração dos limites do ETB TPS a cada power-on.
 *
 * Varre a borboleta até os batentes mecânicos (fechado → aberto) com PWM
 * moderado, captura os raw ADC de TPS1/TPS2 em cada batente e aplica a
 * calibração em RAM (globals de calibration + drv::sensors). Não grava em
 * flash: cada power-on recalibra. O pedal (APP) continua manual/persistente.
 *
 * Roda como máquina de estados no tick de 2 ms do ETB, ANTES do torque
 * manager assumir. Aborta (mantendo a calibração de flash) se o motor girar,
 * se o driver faltar ou se o curso medido for implausível.
 */
#pragma once

#include <cstdint>

namespace ems::engine {

enum class EtbAutocalState : uint8_t {
    Idle = 0,      // nunca iniciou (ou start() pulou por harness ausente)
    Closing,       // PWM negativo até batente fechado
    Opening,       // PWM positivo até batente aberto
    Release,       // motor solto — mola retorna à posição de repouso
    Done,          // calibração aplicada (etb_cal_valid=1 em RAM)
    Skipped,       // harness ausente ou driver não-READY — mantém cal de flash
    Failed,        // abortada — mantém cal de flash
};

enum class EtbAutocalFail : uint8_t {
    None = 0,
    EngineTurning,   // rpm_x10 != 0 durante a varredura
    DriverFault,     // etb_driver_read_sensors devolveu fault
    Timeout,         // batente não estabilizou dentro do tempo de fase
    SpanTooSmall,    // curso raw < mínimo plausível (TPS1 ou TPS2)
    Inverted,        // raw aberto <= raw fechado (sensor invertido/curso nulo)
};

/** Arma a auto-cal (chamar 1× no boot, após etb_control_init com sucesso). */
void etb_autocal_start() noexcept;

/** true enquanto a varredura está em curso (main deve pular torque/PID). */
bool etb_autocal_active() noexcept;

/** Avança a máquina de estados; chamar a cada tick de 2 ms do ETB. */
void etb_autocal_tick(uint16_t period_ms, uint32_t rpm_x10) noexcept;

EtbAutocalState etb_autocal_state() noexcept;
EtbAutocalFail  etb_autocal_fail_reason() noexcept;

#if defined(EMS_HOST_TEST)
void etb_autocal_test_reset() noexcept;
#endif

}  // namespace ems::engine
