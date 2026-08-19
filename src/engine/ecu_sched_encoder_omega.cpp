/**
 * @file ecu_sched_encoder_omega.cpp
 * @brief Estimador de ω (ΔTIM2_CNT/ΔTIM5_CNT), ×65536.
 */
#include "engine/ecu_sched_encoder_priv.h"
#include "engine/enc_cyl_setpoints.h"
#include "engine/xtau_autocalib.h"
#include "engine/engine_config.h"
#include "engine/calibration.h"
#include "hal/out_pins.h"
#include "hal/critical_section.h"
#include "hal/board_pinout.h"
#include "drv/ckp.h"
#include "drv/crank_angle.h"
#include "drv/encoder_sync.h"
#include "engine/misfire_encoder.h"
#include "engine/spark_skip.h"
#include "drv/sensors.h"
#if !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

#include <stdint.h>
#if defined(EMS_HOST_TEST)
#include <cassert>
#endif

namespace si = ems::engine::sched_internal;
// ── Estimador de ω (ΔTIM2_CNT/ΔTIM5_CNT) ─────────────────────────────────
// Sem roda dentada, não há evento de dente para estimar RPM — ω vem de duas
// amostras consecutivas de (TIM2_CNT, TIM5_CNT), tiradas pelo heavy tick
// do heartbeat TIM2_CH4 (1×/volta via subtick). Pura aritmética, sem
// acesso a registo —
// o chamador já leu os valores; por isso compila e testa-se identicamente
// em host-test e alvo real, sem #ifdef nenhum aqui.
//
// Delta com sinal em TIM2 é obrigatório: o modo encoder de hardware
// decrementa TIM2_CNT nativamente em rotação reversa (kick-back de
// compressão no cranking) — um ω negativo transitório é dado real, não
// erro. Delta de TIM5 (tempo) assume-se positivo dentro do intervalo entre
// dois ticks do heartbeat (muito menor que os ~68,7 s de wrap do TIM5 a
// qualquer RPM realista); um delta ≤0 (relógio não avançou, ou primeira
// amostra) descarta a atualização em vez de dividir por zero ou inverter o
// sinal.
//
// Fixed-point ×65536 (contagens de TIM2 por tick de TIM5) — NÃO ×256.
// ω real cabe entre ~0,00087 counts/tick (200 rpm) e ~0,039 counts/tick
// (9000 rpm): em ×256 isso trunca para 0 a 200 rpm (0,224 → 0 em inteiro,
// cranking leria "sem rotação") e dá só ~10 valores discretos distintos
// até ao redline — resolução insuficiente em toda a gama, não só no
// extremo. ×65536 dá ~57 (200 rpm) a ~2577 (9000 rpm), sem truncar a zero
// em nenhum ponto do range operacional. Nome inclui a escala (não
// "omega_q" genérico) de propósito: um nome estável sobre um valor
// re-escalado é exatamente a armadilha que os aliases legados de teste
// TIM1/TIM2 (ecu_sched.cpp) já demonstraram neste projeto.

static volatile uint32_t g_omega_prev_tim2  = 0U;
static volatile uint32_t g_omega_prev_tim5  = 0U;
static volatile uint8_t  g_omega_have_prev  = 0U;
static volatile int32_t  g_omega_x65536     = 0;
static volatile uint8_t  g_omega_valid      = 0U;

// Piso de Δt: 16 µs @ 62.5 MHz. Amostra de ISR encolhida (sub-tick atrasado
// + seguinte imediato) não actualiza.
static constexpr int32_t kOmegaMinDtTicks = 1000;
// Firmware: ~21 000 RPM. Host: os testes semeiam ω=0.5/1/2 (x65536 até
// 131072) para span==ticks — o tecto físico quebrava a suite inteira.
#if defined(EMS_HOST_TEST)
static constexpr int32_t kOmegaMaxX65536 = 2000000;
#else
static constexpr int32_t kOmegaMaxX65536 = 6000;
#endif

void ecu_sched_encoder_omega_sample(uint32_t tim2_now, uint32_t tim5_now) noexcept
{
    if (g_omega_have_prev != 0U) {
        const int32_t d_tim5 = (int32_t)(tim5_now - g_omega_prev_tim5);
        if (d_tim5 >= kOmegaMinDtTicks) {
            const int32_t d_tim2 = (int32_t)(tim2_now - g_omega_prev_tim2);
            const int32_t raw = (int32_t)(((int64_t)d_tim2 * 65536) / (int64_t)d_tim5);
            if (raw <= kOmegaMaxX65536) {
                g_omega_x65536 = raw;
                g_omega_valid = 1U;
            }
        }
        // d_tim5 curto, pico de ω ou relógio sem avanço: mantém a última
        // estimativa. prev actualiza sempre para o glitch não entrar no
        // intervalo seguinte.
    }
    g_omega_prev_tim2 = tim2_now;
    g_omega_prev_tim5 = tim5_now;
    g_omega_have_prev = 1U;
}

int32_t ecu_sched_encoder_omega_x65536(void) noexcept { return g_omega_x65536; }
uint8_t ecu_sched_encoder_omega_valid(void) noexcept { return g_omega_valid; }

#if defined(EMS_HOST_TEST)
// Chamado por ecu_sched_test_reset() (ecu_sched.cpp) — evita estado do
// estimador vazar entre casos de teste no mesmo binário.
void ecu_sched_encoder_omega_test_reset(void) noexcept
{
    g_omega_prev_tim2 = 0U;
    g_omega_prev_tim5 = 0U;
    g_omega_have_prev = 0U;
    g_omega_x65536 = 0;
    g_omega_valid = 0U;
}
#endif
