#pragma once

#include <cstdint>

namespace ems::hal {

void tim5_ic_init(void);
uint32_t tim5_count() noexcept;

// TIM5 free-running, sem input capture (MT6835 apenas — VGT6). CKP/CMP saíram
// de TIM5 (TIM2 encoder + TIM3_CH1/PC6), mas ecu_sched.cpp lê TIM5_CNT
// diretamente para os watchdogs de dwell/injeção — precisa do contador vivo,
// não de captura. Usar em vez de tim5_ic_init() quando o pipeline MT6835
// estiver ativo (tim5_ic_init() reclamaria PA0/PA1, já usados pelo encoder).
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

void tim4_pwm_init(uint32_t freq_hz);
void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;

// ETB motor PWM: VGT6=TIM15_CH1 PE5; RGT6=TIM3_CH1 PA6.
void etb_pwm_init(uint32_t freq_hz);
void etb_pwm_set_duty_x10(uint16_t duty_pct_x10) noexcept;

} // namespace ems::hal
