#pragma once

#include <cstdint>

namespace ems::hal {

void tim5_ic_init(void);
uint32_t tim5_count() noexcept;

// TIM5 free-running, sem input capture (MT6835 apenas — VGT6). CKP/CMP saíram
// de TIM5 (TIM2 encoder + TIM3_CH1/PC6). CNT continua a ser o relógio dos
// watchdogs e de ω. CH3 é compare interno (sem pino) para prime/teste:
// evt_insert liga CC3IE; NVIC TIM5 é armado aqui. Sem captura CH1/CH2.
// Mutuamente exclusivo com tim5_ic_init() (PA0/PA1 já são TIM2 AB).
void tim5_freerun_init() noexcept;

// Polaridade de captura TIM5 + pull GPIOA (CKP=PA0 / CMP=PA1).
// falling=true → CC1P/CC2P e pull-up; false → subida e pull-down (default actual).
// Sequência: limpar CCxE → CCxP → repor CCxE (evitar captura espúria).
// Chamar após carregar page0 (tim5_ic_init corre antes da NVM).
void tim5_ic_set_capture_polarity(bool ckp_falling, bool cmp_falling) noexcept;

// TIM2_CH3 (PB10) — PWM EWG; ⚠️ conflito com INJ3 no RGT6.
// ⚠️ TAMBÉM conflito com tim2_encoder_init() abaixo — mesmo periférico, ver
// aviso em hal/stm32h562/regs.h junto a TIM2_CR1. Nunca chamar os dois.
void tim2_pwm_init(uint32_t freq_hz);
void tim2_set_duty(uint16_t duty_pct_x10) noexcept;

// ── TIM2 modo encoder — MT6835 (VGT6 apenas) ────────────────────────────────
// CH1=PA0 (AF1, canal A) · CH2=PA1 (AF1, canal B) · CH3 = compare-match em
// domínio de ângulo, sem pino físico (ver docs/dev/mt6835_encoder_fork.md,
// "Arquitetura base"). PA1 sai de TIM5_CH2/CMP — CMP move para
// TIM3_CH1/PC6, ver tim3_cmp_ic_init() abaixo.
// ⚠️ Conflita com tim2_pwm_init() (EWG) — ver aviso acima e em regs.h.
void tim2_encoder_init() noexcept;
uint32_t tim2_encoder_count() noexcept;
void tim2_encoder_set_count(uint32_t counts) noexcept;

// Heartbeat TIM2_CH4 — independente da fila de disparo (CH3), sem pino
// físico (mesmo padrão "frozen" do CH3). Rearme a cada 256 counts (~64×/volta);
// o heavy tick (ω, CMP, rebuild) corre a cada 64º sub-tick = 1×/volta
// (ver ecu_sched_encoder_heartbeat_subtick). Chamar depois de mt6835_init()
// ter pré-carregado TIM2_CNT.
void tim2_heartbeat_start() noexcept;

// Próximo CCR4 do heartbeat, dado o valor já incrementado (+256) e o
// TIM2_CNT actual no momento do IRQ. Caminho saudável: o incremento normal
// já deixa CCR4 à frente de CNT — devolve-o sem alteração. Caminho
// degradado (auto-recuperação): se o atendimento da IRQ atrasou mais de 256
// counts, CCR4 já incrementado continua atrás de CNT — nesse caso o próximo
// compare-match só dispararia depois de o contador de 32 bits dar a volta
// completa (TIM2_ARR = 0xFFFFFFFF em modo encoder), travando o heartbeat.
// Devolve CNT+256 nesse caso, para o comparador voltar a estar à frente.
// Função pura (sem acesso a registo) para ser testável em host — chamada
// pelo TIM2_IRQHandler (hal/stm32h562/timer.cpp) logo após `TIM2_CCR4 += 256u`.
inline uint32_t tim2_heartbeat_next_ccr4(uint32_t ccr4_after_increment,
                                         uint32_t cnt) noexcept
{
    if (static_cast<int32_t>(ccr4_after_increment - cnt) < 0) {
        return cnt + 256u;
    }
    return ccr4_after_increment;
}

// ── CMP via TIM3_CH1/PC6 (VGT6 apenas, MT6835) ──────────────────────────────
// Substitui TIM5_CH2/PA1 (que agora é canal B do encoder). PC6/AF2/TIM3_CH1
// verificado livre na VGT6 nas duas tabelas AF do DS14258 e contra
// out_pins.cpp (o bloco GPIOC de IGN só compila para RGT6). IRQ_TIM3=46 já
// nomeado no vetor (startup_stm32h562.cpp) — sem número adivinhado, ao
// contrário da alternativa EXTI descartada (ver design doc).
// ⚠️ Em RGT6, TIM3 é também o PWM do ETB (etb_pwm_init) — não misturar
// com tim3_cmp_ic_init() no mesmo build.
// Captura na descida (Hall idle-HIGH aberto-coletor, mesmo raciocínio de
// tim5_ic_set_capture_polarity). No CC1IF, o ISR grava TIM2->CNT (ângulo do
// encoder no instante do flanco do CMP) — não um timestamp de tempo.
// ⚠️ cmp_angle_snapshot()/cmp_edge_count() não são amostrados juntos: cada
// leitura é atômica isoladamente (uint32_t alinhado em M33), mas o par pode
// ser lido a meio de uma atualização do ISR — um consumidor futuro não pode
// assumir que os dois vêm do mesmo flanco. Mesma classe de corrida já vista
// neste projeto entre ISR e main loop (ver [[ckp-cmp-scope-diag]]); usar
// CriticalSectionGuard se precisar dos dois consistentes entre si.
void tim3_cmp_ic_init() noexcept;
uint32_t cmp_angle_snapshot() noexcept;
uint32_t cmp_edge_count() noexcept;

// ── Z (índice) via TIM3_CH2/PC7 (VGT6 apenas, MT6835) ───────────────────────
// Correção de drift via Z — docs/dev/mt6835_encoder_fork.md, "Correção de
// drift via Z". Mesmo periférico/init que o CMP acima (tim3_cmp_ic_init()
// configura os dois canais); captura só na borda de subida (CC2P=0,
// consistente com Z_EDGE=0 programado no MT6835, ver mt6835.cpp
// configure_z_pulse()). No CC2IF, o ISR grava TIM2->CNT (mesmo princípio do
// CMP: ângulo do encoder no instante do flanco Z, não um timestamp de
// tempo). Requer par Z em cabo blindado dedicado, fisicamente separado de
// AB (ver docs/wiring_diagram.md) — sem isso, Z não é independente de AB
// contra EMI de linha (achado da 2ª mesa de consultores).
// ⚠️ Mesma ressalva de não-atomicidade do par que cmp_angle_snapshot()/
// cmp_edge_count() acima: cada leitura é atômica isoladamente, mas o par
// pode vir de flancos diferentes se lido a meio de uma actualização do ISR.
uint32_t z_angle_snapshot() noexcept;
uint32_t z_edge_count() noexcept;

void tim4_pwm_init(uint32_t freq_hz);
void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

// ETB motor PWM: VGT6=TIM15_CH1 PE5; RGT6=TIM3_CH1 PA6.
void etb_pwm_init(uint32_t freq_hz);
void etb_pwm_set_duty_x10(uint16_t duty_pct_x10) noexcept;

} // namespace ems::hal
