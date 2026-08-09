/**
 * @file ecu_sched_angle_encoder.cpp
 * @brief Dispatcher em domínio de ângulo (MT6835/TIM2), paralelo a
 *        ecu_sched_angle.cpp — que fica intocado (caminho Hall/roda-dentada
 *        continua a compilar e a funcionar sem alteração nenhuma).
 *
 * Só existe (e só é chamado) quando EMS_MT6835_ENCODER=1
 * (hal/board_pinout.h). Com a flag em 0 (default de produção), este
 * ficheiro compila mas fica inerte — nada o chama.
 *
 * Desenho completo: docs/dev/mt6835_encoder_fork.md (secção "Dispatcher em
 * domínio de ângulo") e o plano que o originou. Resumo do porquê de um
 * ficheiro novo em vez de estender ecu_sched.cpp: a fila TIM5/CH3 existente
 * em ecu_sched.cpp serve ecu_sched_on_tooth_hook() (inerte por construção em
 * modo encoder — CC1IE nunca é ligado, ver tim5_freerun_init() vs.
 * tim5_ic_init()) E ecu_sched_fire_prime_pulse()/test_pulse_inj()/
 * test_pulse_ign() (motor parado, sempre por tempo — estes continuam a usar
 * TIM5 em qualquer um dos dois builds). Uma fila em contagens de TIM2 não
 * pode partilhar esse mecanismo: um alvo em ângulo é inalcançável com o
 * motor parado, um alvo em tempo é sempre alcançável — não são a mesma
 * unidade. Por isso esta fila é nova e separada, não uma variante
 * parametrizada da existente.
 *
 * Estrutura prevista (preenchida pelas tarefas seguintes, ainda vazias):
 *   - Fila TIM2/CH3 própria (evt_insert/dispatch equivalentes, unidade
 *     counts) — TODO.
 *   - Rastreador de fase: anchor absoluto de 32 bits, atualizado por
 *     definição absoluta a cada flanco do CMP (sensor Hall inalterado,
 *     cmp_angle_snapshot()) — TODO.
 *   - Estimador de ω: ΔTIM2_CNT/ΔTIM5_CNT, delta com sinal (sobrevive a
 *     bounce de cranking) — TODO.
 *   - Heartbeat TIM2_CH4: recompute barato de dwell/PW, bank-toggle do
 *     presync, verificação de deriva do CMP — TODO (HAL em
 *     hal/stm32h562/timer.cpp; este ficheiro só a lógica de resposta).
 *   - Conversão graus→counts + função de recompute partilhada com
 *     ecu_sched_commit_calibration() — TODO.
 */

#include "engine/ecu_sched_internal.h"

#include <stdint.h>

namespace ems::engine::sched_internal::encoder {

// Placeholder — preenchido pelas tarefas do plano (fila TIM2/CH3, rastreador
// de fase, estimador de ω, heartbeat CH4, conversão graus→counts).

}  // namespace ems::engine::sched_internal::encoder
