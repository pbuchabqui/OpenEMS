/**
 * @file ecu_sched_encoder_phase.cpp
 * @brief Rastreador de fase A/B (âncora CMP) + calibração TDC1/CMP.
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
// ── Rastreador de fase (anchor CMP → ECU_PHASE_A/B) ─────────────────────
// TIM2_CNT só dá posição mod 360° (1 volta de cambota); o motor tem ciclo
// de 720°. O CMP (sensor Hall inalterado, tim3_cmp_ic_init()) desambigua
// qual metade — mas ESTE módulo não decide a que fase corresponde um
// flanco do CMP (constante de calibração de hardware — cfg::g_eng_cfg.
// cmp_phase_state, NVM, calibrável ao vivo via comando 'M'/dash, ver
// engine_config.h); recebe a fase já resolvida via
// ecu_sched_encoder_phase_set_anchor() e só responde "que fase é agora"
// contando voltas completas (16384 counts) desde o anchor absoluto mais
// recente — nunca por toggle incremental, sempre recalculado do anchor.
//
// Pura aritmética, sem acesso a registo — mesmo raciocínio de
// host-testabilidade do estimador de ω acima.

static volatile uint32_t g_phase_anchor_raw   = 0U;
static volatile uint8_t  g_phase_anchor_value = ECU_PHASE_A;
static volatile uint8_t  g_phase_valid        = 0U;
// Confirm-count do rastreador CMP vive aqui (junto de phase_valid) para
// ecu_sched_encoder_phase_invalidate() o poder zerar: depois de uma perda
// de fase, um único flanco aceite não pode re-ancorar. Ver heartbeat_tick.
uint8_t g_cmp_confirm_count = 0U;
// Floor division por 16384 (2^14) — só a paridade do quociente interessa
// (nº de voltas completas desde o anchor, par/ímpar decide a fase), mas a
// paridade só está correta se a divisão arredondar para -∞, não para zero.
// Ex.: delta=-1 (1 count antes do anchor) tem de cair na volta anterior
// (quociente -1, ímpar/fase invertida) — divisão truncada daria 0 (par),
// errado: fisicamente esse count já pertence à revolução anterior.
static inline int32_t floor_div_16384(int32_t delta) noexcept
{
    int32_t q = delta / 16384;
    if ((delta % 16384 != 0) && (delta < 0)) { --q; }
    return q;
}

void ecu_sched_encoder_phase_set_anchor(uint32_t tim2_raw_at_cmp_edge,
                                        uint8_t phase) noexcept
{
    g_phase_anchor_raw = tim2_raw_at_cmp_edge;
    g_phase_anchor_value = phase;
    g_phase_valid = 1U;
}

// Confia cegamente no anchor recebido — pura aritmética de paridade de
// revolução, sem verificação própria nenhuma. Isso é seguro só porque o
// ÚNICO chamador de ecu_sched_encoder_phase_set_anchor() (heartbeat_tick(),
// abaixo) nunca re-ancora sem o flanco CMP ter passado primeiro por
// evaluate_cmp_edge() (encoder_sync.h/.cpp) — um salto de dente da
// corrente/correia de distribuição apareceria ali como flanco REJEITADO,
// nunca chegaria a esta função com um anchor errado.
uint8_t ecu_sched_encoder_phase_at(uint32_t tim2_raw_now) noexcept
{
    const int32_t delta = (int32_t)(tim2_raw_now - g_phase_anchor_raw);
    const int32_t revs = floor_div_16384(delta);
    const uint8_t flipped = (uint8_t)((uint32_t)revs & 1U);
    if (flipped == 0U) { return g_phase_anchor_value; }
    return (g_phase_anchor_value == ECU_PHASE_A) ? ECU_PHASE_B : ECU_PHASE_A;
}

uint8_t ecu_sched_encoder_phase_valid(void) noexcept { return g_phase_valid; }

// Único caminho de produção que limpa g_phase_valid — até esta função existir
// só havia SET (phase_set_anchor) e um clear host-test-only. Sem isto, um
// fallback de staleness que publicasse HALF_SYNC no CkpSnapshot deixaria
// phase_valid() preso em 1 — o dispatcher branca em phase_valid(), não em
// snap.state. Também zera o gate de 2 flancos (Fix B): senão o primeiro
// flanco aceite depois da perda (ruído no pino flutuante, captura espúria
// do rearm TIM3) re-ancorava já com confirm_count ainda em 2.
// Chamador: ecu_sched_encoder_heartbeat_tick() após staleness_exceeded()
// ou o watchdog de stall do builder sequencial.
void ecu_sched_encoder_phase_invalidate(void) noexcept
{
    g_phase_valid = 0U;
    g_cmp_confirm_count = 0U;
}

#if defined(EMS_HOST_TEST)
void ecu_sched_encoder_phase_test_reset(void) noexcept
{
    g_phase_anchor_raw = 0U;
    g_phase_anchor_value = ECU_PHASE_A;
    g_phase_valid = 0U;
    g_cmp_confirm_count = 0U;
}
#endif

// Calcula o valor de cfg::g_eng_cfg.encoder_tdc1_origin_deg a partir de uma
// leitura crua de TIM2->CNT feita com o cilindro 1 fisicamente no PMS de
// compressão (mesmo procedimento de bancada de trigger_tooth0_engine_deg —
// dial indicator/roda de graus, ver engine_config.h e
// docs/dev/mt6835_encoder_fork.md, "Procedimento de bancada"). Pura — não lê
// nem escreve g_eng_cfg; o chamador (UI protocol, comando 'X') decide
// se/quando persistir o valor devolvido.
//
// Inverte engine_deg_to_counts_in_rev(0) abaixo: engine_angle_deg=0 é o TDC
// do cilindro 1 (cyl_tdc_deg(0)==0, engine_config.h) e tem de mapear para
// counts_in_rev==tim2_raw_at_tdc1 (mod 16384), logo
// origin_mod360 = (360 − round(counts_in_rev×360/16384)) mod 360.
uint16_t ecu_sched_encoder_tdc1_calibrate_from_raw(uint32_t tim2_raw_at_tdc1) noexcept
{
    const uint32_t counts_in_rev = tim2_raw_at_tdc1 & 0x3FFFU;
    const uint32_t origin_deg = (counts_in_rev * 360U + 8192U) / 16384U;  // arredonda
    return static_cast<uint16_t>((360U - (origin_deg % 360U)) % 360U);
}

// Calcula cfg::g_eng_cfg.cmp_phase_state a partir de duas leituras cruas de
// TIM2->CNT: agora (cilindro 1 no PMS de COMPRESSÃO — por convenção,
// engine_deg=0 cai sempre na janela de ECU_PHASE_A, [0°,360°)) e a do
// último flanco CMP capturado (ems::hal::cmp_angle_snapshot()). Mesma
// aritmética de floor_div_16384/ecu_sched_encoder_phase_at() acima —
// paridade de voltas completas de 360° entre os dois pontos: par → o
// flanco caiu na MESMA janela do PMS1 (fase A); ímpar → janela oposta
// (fase B). Pura — não lê nem escreve g_eng_cfg nem o anchor de fase ao
// vivo; o chamador (UI protocol, comando 'M') decide se/quando persistir.
uint8_t ecu_sched_encoder_cmp_phase_calibrate_from_raw(
    uint32_t tim2_raw_at_tdc1_compression, uint32_t tim2_raw_at_cmp_edge) noexcept
{
    const int32_t delta = static_cast<int32_t>(
        tim2_raw_at_tdc1_compression - tim2_raw_at_cmp_edge);
    const int32_t revs = floor_div_16384(delta);
    return ((revs & 1) == 0) ? ems::engine::cfg::kCmpPhaseCalibratedA
                             : ems::engine::cfg::kCmpPhaseCalibratedB;
}
