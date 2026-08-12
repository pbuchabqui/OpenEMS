/**
 * @file hal/stm32h562/timer.cpp
 * @brief Implementacao da HAL de timers para STM32H562RGT6
 *        - backend STM32-only.
 *
 * Mapeamento de perifericos (produção, tim5_ic_init() — CKP/CMP via Hall):
 *   TIM5_CH1 PA0: CKP input capture (62.5 MHz, 16 ns/tick)
 *   TIM5_CH2 PA1: CMP input capture
 *   TIM5_CH3   --: event dispatcher — sem modo OC, compare puro (CCR3) +
 *                  GPIO BSRR por software na ISR (ver ecu_sched.cpp)
 *   TIM2_CH3 PB10: EWG PWM (motor wastegate)
 *   TIM4_CH1 PB6: VVT escape PWM
 *   TIM4_CH2 PB7: VVT admissao PWM
 *   TIM3_CH1 PA6: ETB PWM (RGT6) — ver etb_pwm_init()
 *
 * Mapeamento alternativo (fork MT6835 — VGT6, tim5_freerun_init()):
 *   TIM2_CH1/CH2 PA0/PA1: encoder MT6835 (ver tim2_encoder_init())
 *   TIM2_CH3: event dispatcher em domínio de ângulo — mesmo mecanismo do
 *             TIM5_CH3 acima (compare + BSRR), unidade counts em vez de ticks
 *             (ver ecu_sched_angle_encoder.cpp)
 *   TIM3_CH1 PC6: CMP input capture (ver tim3_cmp_ic_init())
 *   TIM5: free-running sem captura, só watchdog de dwell/injeção
 *   docs/dev/mt6835_encoder_fork.md, "Gap TIM5_CEN" — os dois mapeamentos
 *   são mutuamente exclusivos, nunca chamar tim5_ic_init() e
 *   tim2_encoder_init()/tim3_cmp_ic_init() no mesmo boot.
 *
 * Injeção/ignição (INJ/IGN): nenhum canal TIM OC — sempre GPIOE BSRR por
 * software, disparado pela ISR do event dispatcher acima (ecu_sched.cpp /
 * ecu_sched_angle_encoder.cpp), não por este ficheiro.
 *
 * Clock dos timers:
 *   TIM5, TIM3, TIM4, TIM2 (APB1): timer clock = 250 MHz (timer doubler ativo)
 *   TIM5 prescaler = 3 -> tick = 250 MHz / 4 = 62.5 MHz -> 16 ns/tick
 */

#ifndef EMS_HOST_TEST

#include "hal/timer.h"
#include "hal/regs.h"
#include "drv/ckp.h"    // ckp_tim5_ch1_isr / ckp_tim5_ch2_isr
#include "engine/ecu_sched.h"  // ecu_sched_evt_dispatch

// -- Constantes de clock --------------------------------------------------------
static constexpr uint32_t kTimPrescaler = 3u;
static constexpr uint32_t kTimClockHz   = 62500000u;

namespace ems::hal {

// ------------------------------------------------------------------------------
// CKP via TIM5_CH1 (PA0/AF2) + CMP via TIM5_CH2 (PA1/AF2)
// Input capture hardware com 32-bit, 62.5 MHz, 16 ns/tick — zero jitter de ISR.
// ------------------------------------------------------------------------------

void tim5_ic_init(void) {
    // PA0 = TIM5_CH1/CKP (AF2), PA1 = TIM5_CH2/CMP (AF2)
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 0u, GPIO_AF2);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 1u, GPIO_AF2);
    // Pull-down em PA0 (CKP) e PA1 (CMP): sensores idle-LOW/pulsam-HIGH, captura por
    // borda de subida. Sem pull, um sensor desligado/fio partido deixa o pino a
    // flutuar → ruído gera bordas fantasma que fingem sync (FULL_SYNC falso → injeção
    // batch espúria nos 4 injetores). Pull-down força LOW estável quando desligado =
    // sem bordas de subida. Complementa o filtro IC (que corta glitch fino mas não
    // ruído de baixa frequência num pino aberto). (cf. uart.cpp:50, PA10.)
    GPIOA_PUPDR = (GPIOA_PUPDR & ~((0x3u << 0u) | (0x3u << 2u)))
                | (0x2u << 0u) | (0x2u << 2u);  // PA0(CKP)+PA1(CMP), 0b10 = pull-down

    // Ativar clocks
    RCC_APB1LENR |= RCC_APB1LENR_TIM5EN;
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;

    // TIM5 — 32-bit free-running @ 62.5 MHz, input capture CH1=CKP, CH2=CMP
    TIM5_CR1 = 0u; TIM5_PSC = 3u; TIM5_ARR = 0xFFFFFFFFu; TIM5_EGR = 1u;
    // CH1: CKP input capture (TI1, rising edge)
    // Filtro de entrada ~256 ns (IC1F/IC2F) rejeita glitches EMC no HW antes da
    // captura — CKP (PA0) e CMP (PA1) sem filtro deixavam ruído gerar bordas
    // fantasma que disparavam prime/falso-sync com o motor parado.
    TIM5_CCMR1 = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_IC1F_N8_DTS8
               | TIM_CCMR1_CC2S_TI2 | TIM_CCMR1_IC2F_N8_DTS8;
    TIM5_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM5_CCMR2 = 0u;
    TIM5_DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE;

    // TIM2_CH1 — já não usado para CKP (apenas PWM EWG no CH3)
    TIM2_CCER = 0u;
    TIM2_DIER = 0u;

    nvic_set_priority(IRQ_TIM5, 1u); nvic_enable_irq(IRQ_TIM5);
    TIM5_CR1 = TIM_CR1_CEN;
}

void tim5_ic_set_capture_polarity(bool ckp_falling, bool cmp_falling) noexcept {
    // CCER: limpar CCxE antes de mudar CCxP, depois repor CCxE (+ CC3E do dispatcher).
    // PUPDR: 01=pull-up, 10=pull-down. Captura na descida → idle HIGH → pull-up;
    // captura na subida → idle LOW → pull-down (fix de falso-sync).
    uint32_t ccer = TIM5_CCER;
    ccer &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM5_CCER = ccer;

    if (ckp_falling) {
        ccer |= TIM_CCER_CC1P;
    }
    if (cmp_falling) {
        ccer |= TIM_CCER_CC2P;
    }
    ccer |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM5_CCER = ccer;

    uint32_t pupdr = GPIOA_PUPDR;
    pupdr &= ~((0x3u << 0u) | (0x3u << 2u));
    // bit field: 01 = pull-up, 10 = pull-down
    pupdr |= ckp_falling ? (0x1u << 0u) : (0x2u << 0u);
    pupdr |= cmp_falling ? (0x1u << 2u) : (0x2u << 2u);
    GPIOA_PUPDR = pupdr;
}

uint32_t tim5_count() noexcept {
    return TIM5_CNT;
}

// ------------------------------------------------------------------------------
// TIM5 free-running, sem input capture (MT6835 apenas — VGT6).
// CKP e CMP saíram de TIM5 (TIM2 modo encoder + TIM3_CH1/PC6, ver
// docs/dev/mt6835_encoder_fork.md, "Gap TIM5_CEN"), mas ecu_sched.cpp
// continua a ler TIM5_CNT diretamente para os watchdogs de dwell/injeção
// (ecu_sched_dwell_watchdog(), ecu_sched_inj_watchdog()) — depende do mesmo
// tick de 62,5 MHz/16 ns que tim5_ic_init() sempre configurou
// (ECU_SCHED_CLOCK_HZ, ecu_sched.h:47, static_assert em ecu_sched.cpp:49-52).
// Sem captura/GPIO/NVIC: só o contador livre-corrente que o watchdog
// precisa. Usar no lugar de tim5_ic_init() quando o pipeline MT6835
// estiver ativo — nunca os dois (tim5_ic_init() reclama PA0/PA1, que já
// são TIM2_CH1/CH2 do encoder).
// ------------------------------------------------------------------------------

void tim5_freerun_init() noexcept {
    RCC_APB1LENR |= RCC_APB1LENR_TIM5EN;
    TIM5_CR1  = 0u;
    TIM5_PSC  = kTimPrescaler;   // 250 MHz / 4 = 62,5 MHz — mesmo tick de sempre
    TIM5_ARR  = 0xFFFFFFFFu;
    TIM5_CCER = 0u;              // nenhuma captura
    TIM5_DIER = 0u;              // nenhuma interrupção — não precisa de NVIC
    TIM5_EGR  = 1u;
    TIM5_CR1  = TIM_CR1_CEN;
}

// ----------------------------------------------------------------------------
// TIM4 - PWM (VVT Exhaust CH1 + VVT Intake CH2)
// ----------------------------------------------------------------------------

void tim4_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }
    RCC_APB1LENR |= RCC_APB1LENR_TIM4EN;

    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 6u, GPIO_AF2);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 7u, GPIO_AF2);

    uint32_t arr = kTimClockHz / freq_hz;
    if (arr > 0xFFFFu) { arr = 0xFFFFu; }
    if (arr > 0u) { arr -= 1u; }

    TIM4_CR1 = 0u;
    TIM4_PSC = 0u;
    TIM4_ARR = arr;
    TIM4_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE
               | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    TIM4_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM4_CCR1 = 0u;
    TIM4_CCR2 = 0u;
    TIM4_EGR  = 1u;
    TIM4_CR1  = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    if (duty_pct_x10 > 1000u) { duty_pct_x10 = 1000u; }
    const uint32_t arr = TIM4_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    if (ch == 0u) {
        TIM4_CCR1 = ccr;
    } else {
        TIM4_CCR2 = ccr;
    }
}

// ----------------------------------------------------------------------------
// TIM2 - PWM do motor EWG (CH3 em PB10, AF1)
// ----------------------------------------------------------------------------

void tim2_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;

    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 10u, GPIO_AF1);

    uint32_t psc = 0u;
    uint32_t arr = kTimClockHz / freq_hz;
    while (arr > 0xFFFFu) { ++psc; arr = kTimClockHz / (freq_hz * (psc + 1u)); }
    if (arr > 0u) { arr -= 1u; }

    TIM2_CR1   = 0u;
    TIM2_PSC   = psc;
    TIM2_ARR   = arr;
    TIM2_CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
    TIM2_CCER  = TIM_CCER_CC3E;
    TIM2_CCR3  = 0u;
    TIM2_EGR   = 1u;
    TIM2_CR1   = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void tim2_set_duty(uint16_t duty_pct_x10) noexcept {
    if (duty_pct_x10 > 1000u) { duty_pct_x10 = 1000u; }
    const uint32_t arr = TIM2_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    TIM2_CCR3 = ccr;
}

// ----------------------------------------------------------------------------
// TIM2 modo encoder — MT6835 (VGT6 apenas). CH1=PA0/AF1 (canal A), CH2=PA1/AF1
// (canal B). CH3 = compare-match em domínio de ângulo, sem GPIO associado (a
// interrupção não depende de CC3E, só de CC3IE — confirmado contra 3 fontes
// independentes, ver docs/dev/mt6835_encoder_fork.md, "Arquitetura base").
// ⚠️ Conflita com tim2_pwm_init() (EWG) — mesmo ARR/PSC, nunca chamar os dois.
// ⚠️ PA1 sai de TIM5_CH2/CMP (AF2) e passa a TIM2_CH2 (AF1) — CMP move para
// TIM3_CH1/PC6 (tim3_cmp_ic_init). Mutuamente exclusivo com tim5_ic_init().
// ----------------------------------------------------------------------------

void tim2_encoder_init() noexcept {
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN;
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;

    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 0u, GPIO_AF1);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 1u, GPIO_AF1);

    TIM2_CR1  = 0u;
    TIM2_PSC  = 0u;             // não se aplica à contagem em modo encoder (RM) — 0 por padrão
    TIM2_ARR  = 0xFFFFFFFFu;    // 32-bit livre-corrente, mesmo padrão do "virabrequim virtual" do TIM5

    // CH1→TI1, CH2→TI2 (mesmos bits de mapeamento usados para input capture em
    // TIM5), com filtro de entrada N=8 amostras — mesma proteção anti-EMI que
    // já existe no CKP/CMP atual (tim5_ic_init(), ~256 ns de janela).
    TIM2_CCMR1 = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_IC1F_N8_DTS8
               | TIM_CCMR1_CC2S_TI2 | TIM_CCMR1_IC2F_N8_DTS8;

    // CH3: "Frozen" (sem ação de saída) — só precisamos do comparador interno
    // e da flag CC3IF/interrupção CC3IE, não de um pino físico. CC3E fica em 0
    // de propósito.
    TIM2_CCMR2 = 0u;
    TIM2_CCER  = 0u;

    // Modo encoder 3 (SMS=011): conta em ambas as bordas de TI1 e TI2 →
    // decodificação X4. A 4.096 PPR isto dá 16.384 contagens/volta.
    TIM2_SMCR = TIM_SMCR_SMS_ENCODER_MODE3;

    TIM2_EGR = 1u;
    // CC3IE fica desligado no boot (mesmo padrão do TIM5_DIER em tim5_ic_init():
    // só o que já tem dado válido fica ligado). CCR3 não é inicializado aqui de
    // propósito — o dispatcher (ecu_sched, ver docs/dev/mt6835_encoder_fork.md,
    // "Dispatcher em domínio de ângulo") liga CC3IE dinamicamente ao inserir o
    // primeiro evento na fila, e desliga quando ela esvazia — exatamente como
    // evt_insert()/ecu_sched_evt_dispatch() já fazem para TIM5_DIER/CC3IE hoje.
    // Sem isto, CC3IE ligado com CCR3 no valor de reset dispara um CC3 espúrio
    // na primeira passagem por esse valor, antes de existir qualquer evento real.
    TIM2_DIER = 0u;

    nvic_set_priority(IRQ_TIM2, 1u);
    nvic_enable_irq(IRQ_TIM2);
    TIM2_CR1 = TIM_CR1_CEN;
}

uint32_t tim2_encoder_count() noexcept {
    return TIM2_CNT;
}

void tim2_encoder_set_count(uint32_t counts) noexcept {
    // Usado no key-on: pré-carrega TIM2->CNT com o ângulo absoluto lido por
    // SPI do MT6835 (ems::hal::mt6835_angle21_to_tim2_counts()), decisão 3 da
    // arquitetura base.
    TIM2_CNT = counts;
}

// ----------------------------------------------------------------------------
// Heartbeat TIM2_CH4 — ver aviso em hal/timer.h. CC4E fica em 0 de propósito
// (mesmo "frozen" do CH3): só o comparador interno + CC4IF/CC4IE interessam.
//
// Rearme a cada 256 counts (~64×/volta), não 16384 (1×/volta) — split
// light/heavy do heartbeat (ver ecu_sched_encoder_heartbeat_subtick(),
// ecu_sched_angle_encoder.cpp): o caminho leve (misfire) precisa de
// cadência fina, o pesado (ω/CMP/presync/publish) continua 1×/volta,
// chamado internamente a cada 64º sub-tick — cadência total idêntica à
// anterior a esta tarefa.
// ----------------------------------------------------------------------------

void tim2_heartbeat_start() noexcept {
    const uint32_t now = TIM2_CNT;
    TIM2_CCR4 = now + 256u;
    TIM2_DIER |= TIM_DIER_CC4IE;
}

// ----------------------------------------------------------------------------
// CMP via TIM3_CH1/PC6 (VGT6 apenas, MT6835) — ver aviso em hal/timer.h.
// Captura de hardware, não EXTI: PC6/AF2 dá TIM3_CH1 de verdade (Tabela 15
// do DS14258), livre na VGT6, com IRQ_TIM3=46 já nomeado no vetor — ao
// contrário de PB3 (sem canal de captura em nenhuma AF) e do número de
// vetor EXTI não confirmado que essa rota exigiria.
// ----------------------------------------------------------------------------

namespace {
volatile uint32_t g_cmp_angle_snapshot = 0u;
volatile uint32_t g_cmp_edge_count     = 0u;
}  // namespace

void tim3_cmp_ic_init() noexcept {
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOCEN;
    RCC_APB1LENR |= RCC_APB1LENR_TIM3EN;

    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 6u, GPIO_AF2);
    // Pull-up: sensor Hall idle-HIGH aberto-coletor, mesmo raciocínio de
    // tim5_ic_set_capture_polarity() para o CMP antigo em PA1.
    GPIOC_PUPDR = (GPIOC_PUPDR & ~(0x3u << 12u)) | (0x1u << 12u);

    TIM3_CR1 = 0u;
    TIM3_PSC = 0u;
    TIM3_ARR = 0xFFFFu;  // TIM3 é 16-bit; CNT não interessa, só o IRQ de captura
    TIM3_CCMR1 = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_IC1F_N8_DTS8;
    TIM3_CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P;  // captura na descida
    TIM3_DIER  = TIM_DIER_CC1IE;
    TIM3_EGR   = 1u;

    nvic_set_priority(IRQ_TIM3, 1u);
    nvic_enable_irq(IRQ_TIM3);
    TIM3_CR1 = TIM_CR1_CEN;
}

uint32_t cmp_angle_snapshot() noexcept { return g_cmp_angle_snapshot; }
uint32_t cmp_edge_count() noexcept { return g_cmp_edge_count; }

// ----------------------------------------------------------------------------
// ETB motor PWM (etb_pwm_*):
//   VGT6: PE5 / TIM15_CH1 AF4
//   RGT6: PA6 / TIM3_CH1  AF2
// ----------------------------------------------------------------------------

#include "hal/board_pinout.h"

void etb_pwm_init(uint32_t freq_hz) {
    if (freq_hz == 0u) { return; }

#if EMS_BOARD_IS_VGT6
    RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN;
    gpio_set_af(&GPIOE_MODER, &GPIOE_AFRL, &GPIOE_AFRH, &GPIOE_OSPEEDR, 5u, GPIO_AF4);

    uint32_t arr = kTimClockHz / freq_hz;
    if (arr > 0xFFFFu) { arr = 0xFFFFu; }
    if (arr > 0u) { arr -= 1u; }

    TIM15_CR1 = 0u;
    TIM15_PSC = 0u;
    TIM15_ARR = arr;
    TIM15_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
    TIM15_CCER = TIM_CCER_CC1E;
    TIM15_CCR1 = 0u;
    TIM15_BDTR = (1u << 15);  // MOE
    TIM15_EGR = 1u;
    TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
#else
    RCC_APB1LENR |= RCC_APB1LENR_TIM3EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN;
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 6u, GPIO_AF2);

    uint32_t psc = 0u;
    uint32_t arr = kTimClockHz / freq_hz;
    while (arr > 0xFFFFu) {
        ++psc;
        arr = kTimClockHz / (freq_hz * (psc + 1u));
    }
    if (arr > 0u) { arr -= 1u; }

    TIM3_CR1 = 0u;
    TIM3_PSC = psc;
    TIM3_ARR = arr;
    TIM3_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
    TIM3_CCER = TIM_CCER_CC1E;
    TIM3_CCR1 = 0u;
    TIM3_EGR = 1u;
    TIM3_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
#endif
}

void etb_pwm_set_duty_x10(uint16_t duty_pct_x10) noexcept {
    if (duty_pct_x10 > 1000u) { duty_pct_x10 = 1000u; }
#if EMS_BOARD_IS_VGT6
    const uint32_t arr = TIM15_ARR;
    TIM15_CCR1 = ((arr + 1u) * duty_pct_x10) / 1000u;
#else
    const uint32_t arr = TIM3_ARR;
    TIM3_CCR1 = ((arr + 1u) * duty_pct_x10) / 1000u;
#endif
}

// ----------------------------------------------------------------------------
// ISR Handlers
// ----------------------------------------------------------------------------

/**
 * @brief TIM5_IRQHandler — CKP (CH1) + CMP (CH2) + event dispatcher (CH3)
 */
extern "C" void TIM5_IRQHandler(void) {
    uint32_t sr = TIM5_SR;
    if (sr & TIM_SR_CC1IF) {
        TIM5_SR = ~TIM_SR_CC1IF;
        ems::drv::ckp_tim5_ch1_isr();
    }
    sr = TIM5_SR;
    if (sr & TIM_SR_CC2IF) {
        TIM5_SR = ~TIM_SR_CC2IF;
        ems::drv::ckp_tim5_ch2_isr();
    }
    sr = TIM5_SR;
    if (sr & TIM_SR_CC3IF) {
        TIM5_SR = ~TIM_SR_CC3IF;
        ecu_sched_evt_dispatch();
    }
}

/**
 * @brief TIM2_IRQHandler — CH3 = dispatcher de eventos em domínio de ângulo,
 * CH4 = sub-tick do heartbeat (256 counts, ~64×/volta — o caminho pesado
 * dentro de ecu_sched_encoder_heartbeat_subtick() continua 1×/volta, ver
 * ecu_sched_angle_encoder.cpp). Mesmo periférico/vetor que a
 * fila TIM2/CH3 (ecu_sched_angle_encoder.cpp) — CC3IF e CC4IF chegam pela
 * mesma IRQ, tratados em sequência com releitura de SR entre um e outro
 * (mesmo padrão do TIM5_IRQHandler acima, para não perder um flag que suba
 * durante o tratamento do outro).
 */
extern "C" void TIM2_IRQHandler(void) {
    uint32_t sr = TIM2_SR;
    if (sr & TIM_SR_CC3IF) {
        TIM2_SR = ~TIM_SR_CC3IF;
        ecu_sched_encoder_evt_dispatch();
    }
    sr = TIM2_SR;
    if (sr & TIM_SR_CC4IF) {
        TIM2_SR = ~TIM_SR_CC4IF;
        TIM2_CCR4 += 256u;  // auto-rearma para o próximo sub-tick, sem UEV/ARR
        const uint32_t tim2_now = TIM2_CNT;
        const uint32_t tim5_now = TIM5_CNT;
        const uint32_t cmp_angle = cmp_angle_snapshot();
        const uint32_t cmp_edges = cmp_edge_count();
        ecu_sched_encoder_heartbeat_subtick(tim2_now, tim5_now, cmp_angle, cmp_edges);
    }
}

/**
 * @brief TIM3_IRQHandler — captura do CMP (CH1/PC6). Grava TIM2->CNT (ângulo
 * do encoder no flanco do CMP), não um timestamp de tempo — ver aviso em
 * hal/timer.h.
 */
extern "C" void TIM3_IRQHandler(void) {
    const uint32_t sr = TIM3_SR;
    if (sr & TIM_SR_CC1IF) {
        TIM3_SR = ~TIM_SR_CC1IF;
        g_cmp_angle_snapshot = TIM2_CNT;
        ++g_cmp_edge_count;
    }
}

} // namespace ems::hal

#else  // EMS_HOST_TEST -------------------------------------------------------

#include "hal/timer.h"
namespace ems::hal {
static uint32_t g_mock_tim5_cnt = 0u;
static uint32_t g_mock_tim2_cnt = 0u;
void tim5_ic_init(void) {}
void tim5_ic_set_capture_polarity(bool, bool) noexcept {}
void tim5_freerun_init() noexcept {}
void tim4_pwm_init(uint32_t) {}
void tim2_pwm_init(uint32_t) {}
void tim2_set_duty(uint16_t) noexcept {}
void tim4_set_duty(uint8_t, uint16_t) noexcept {}
void etb_pwm_init(uint32_t) {}
void etb_pwm_set_duty_x10(uint16_t) noexcept {}
uint32_t tim5_count() noexcept { return g_mock_tim5_cnt; }
void tim2_encoder_init() noexcept {}
uint32_t tim2_encoder_count() noexcept { return g_mock_tim2_cnt; }
void tim2_encoder_set_count(uint32_t counts) noexcept { g_mock_tim2_cnt = counts; }
void tim2_heartbeat_start() noexcept {}
void tim3_cmp_ic_init() noexcept {}
uint32_t cmp_angle_snapshot() noexcept { return 0u; }
uint32_t cmp_edge_count() noexcept { return 0u; }
} // namespace ems::hal

#endif  // EMS_HOST_TEST
