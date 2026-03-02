#pragma once

#include <cstdint>

#include "drv/ckp.h"

namespace ems::drv {

// 8 canais: 4 injeção (INJ1-4 = FTM0_CH0-3) + 4 ignição (IGN1-4 = FTM0_CH4-7)
enum class Channel : uint8_t {
    INJ3 = 0,
    INJ4 = 1,
    INJ1 = 2,
    INJ2 = 3,
    IGN4 = 4,
    IGN3 = 5,
    IGN2 = 6,
    IGN1 = 7,
};

enum class Action : uint8_t { SET, CLEAR };

// Retorna false se fila cheia ou evento já passou.
bool sched_event(Channel ch, uint16_t ftm0_ticks, Action act) noexcept;
void sched_cancel(Channel ch) noexcept;
void sched_cancel_all() noexcept;
void sched_recalc(const CkpSnapshot& snap) noexcept;

// ── Módulo 3: SCHEDULE via FTM Output Compare ─────────────────────────────────
/**
 * @brief Agenda um evento de ignição usando Output Compare de hardware (FTM0).
 *
 * Realiza a conversão ângulo → tempo baseada na velocidade angular atual
 * (current_tooth_delta_ticks) e programa o comparador do canal FTM0
 * correspondente ao cilindro alvo. O pino de ignição é acionado PURAMENTE
 * por hardware quando FTM0_CNT == CnV — sem latência de ISR.
 *
 * VANTAGEM vs sched_event() software-GPIO:
 *   O Output Compare do FTM0 dispara o pino exatamente quando o contador
 *   atinge o valor programado, eliminando o jitter de atendimento da IRQ
 *   (0,5–3 µs → ≤0,05° a 3000 RPM). O canal pode ser configurado para
 *   SET, CLEAR ou TOGGLE — sem nenhuma ação do CPU no momento do disparo.
 *
 * SETUP do FTM0_CnSC (K64 RM §43.3.5) realizado internamente:
 *   MSnB:MSnA = 10 → Output Compare
 *   ELSnB:ELSnA = 01 → Clear output on match (pino vai LOW = disparo bobina)
 *   CHIE = 1          → Interrupção para cleanup pós-disparo (re-arm, dwell etc.)
 *   CHF  = 0          → Clear flag anterior (W0C: escrita de 0 limpa o bit)
 *
 * @param target_angle_btdc_x10    Ângulo alvo em miligraus BTDC.
 *                                  Ex: 20000 = 20,0° BTDC.
 * @param current_tooth_angle_x10  Ângulo do dente atual desde TDC, em miligraus.
 *                                  Ex: 60000 = 60,0° ATDC (= 300° BTDC).
 * @param current_tooth_delta_ticks Ticks FTM3 do último período inter-dente.
 *                                  Representa 6,0° de crank para roda 60-2.
 */
void Schedule_Ignition_Event(uint16_t target_angle_btdc_x10,
                              uint16_t current_tooth_angle_x10,
                              uint32_t current_tooth_delta_ticks) noexcept;

#if defined(EMS_HOST_TEST)
void sched_test_reset() noexcept;
uint8_t sched_test_size() noexcept;
bool sched_test_event(uint8_t index, uint16_t& ticks, Channel& ch, Action& act, bool& valid) noexcept;
#endif

}  // namespace ems::drv
