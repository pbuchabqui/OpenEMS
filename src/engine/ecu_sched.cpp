#include "engine/ecu_sched.h"
#include "drv/ckp.h"
#include "engine/engine_config.h"
#include "engine/constants.h"
#include "engine/knock.h"
#include "hal/regs.h"
#include "hal/critical_section.h"

#include <stddef.h>
#include <stdint.h>

#if defined(EMS_HOST_TEST)
static uint32_t ems_test_tim2_cnt;
static uint32_t ems_test_tim2_ccr1;
static uint32_t ems_test_tim2_ccr2;
static uint32_t ems_test_tim2_ccr3;
static uint32_t ems_test_tim2_ccr4;
static uint32_t ems_test_tim2_sr;
static uint32_t ems_test_tim2_cr1;
static uint32_t ems_test_tim2_dier;
static uint32_t ems_test_tim2_psc;
static uint32_t ems_test_tim2_arr;
static uint32_t ems_test_tim2_ccmr1;
static uint32_t ems_test_tim2_ccmr2;
static uint32_t ems_test_tim2_ccer;
static uint32_t ems_test_tim2_egr;
static uint32_t ems_test_tim8_cnt;
static uint32_t ems_test_tim8_ccr1;
static uint32_t ems_test_tim8_ccr2;
static uint32_t ems_test_tim8_ccr3;
static uint32_t ems_test_tim8_ccr4;
static uint32_t ems_test_tim8_sr;
static uint32_t ems_test_tim8_cr1;
static uint32_t ems_test_tim8_dier;
static uint32_t ems_test_tim8_psc;
static uint32_t ems_test_tim8_arr;
static uint32_t ems_test_tim8_ccmr1;
static uint32_t ems_test_tim8_ccmr2;
static uint32_t ems_test_tim8_ccer;
static uint32_t ems_test_tim8_bdtr;
static uint32_t ems_test_tim8_egr;
static uint32_t ems_test_rcc_ahb2enr1;
static uint32_t ems_test_rcc_apb1lenr;
static uint32_t ems_test_rcc_apb2enr;
static uint32_t ems_test_gpio_moder;
static uint32_t ems_test_gpio_afrl;
static uint32_t ems_test_gpio_afrh;
static uint32_t ems_test_gpio_ospeedr;

#define TIM2_CNT ems_test_tim2_cnt
#define TIM2_CCR1 ems_test_tim2_ccr1
#define TIM2_CCR2 ems_test_tim2_ccr2
#define TIM2_CCR3 ems_test_tim2_ccr3
#define TIM2_CCR4 ems_test_tim2_ccr4
#define TIM2_SR ems_test_tim2_sr
#define TIM2_CR1 ems_test_tim2_cr1
#define TIM2_DIER ems_test_tim2_dier
#define TIM2_PSC ems_test_tim2_psc
#define TIM2_ARR ems_test_tim2_arr
#define TIM2_CCMR1 ems_test_tim2_ccmr1
#define TIM2_CCMR2 ems_test_tim2_ccmr2
#define TIM2_CCER ems_test_tim2_ccer
#define TIM2_EGR ems_test_tim2_egr
#define TIM8_CNT ems_test_tim8_cnt
#define TIM8_CCR1 ems_test_tim8_ccr1
#define TIM8_CCR2 ems_test_tim8_ccr2
#define TIM8_CCR3 ems_test_tim8_ccr3
#define TIM8_CCR4 ems_test_tim8_ccr4
#define TIM8_SR ems_test_tim8_sr
#define TIM8_CR1 ems_test_tim8_cr1
#define TIM8_DIER ems_test_tim8_dier
#define TIM8_PSC ems_test_tim8_psc
#define TIM8_ARR ems_test_tim8_arr
#define TIM8_CCMR1 ems_test_tim8_ccmr1
#define TIM8_CCMR2 ems_test_tim8_ccmr2
#define TIM8_CCER ems_test_tim8_ccer
#define TIM8_BDTR ems_test_tim8_bdtr
#define TIM8_EGR ems_test_tim8_egr
#define RCC_AHB2ENR1 ems_test_rcc_ahb2enr1
#define RCC_APB1LENR ems_test_rcc_apb1lenr
#define RCC_APB2ENR ems_test_rcc_apb2enr
#define GPIOA_MODER ems_test_gpio_moder
#define GPIOA_AFRL ems_test_gpio_afrl
#define GPIOA_AFRH ems_test_gpio_afrh
#define GPIOA_OSPEEDR ems_test_gpio_ospeedr
#define GPIOB_MODER ems_test_gpio_moder
#define GPIOB_AFRL ems_test_gpio_afrl
#define GPIOB_AFRH ems_test_gpio_afrh
#define GPIOB_OSPEEDR ems_test_gpio_ospeedr
#define GPIOC_MODER ems_test_gpio_moder
#define GPIOC_AFRL ems_test_gpio_afrl
#define GPIOC_AFRH ems_test_gpio_afrh
#define GPIOC_OSPEEDR ems_test_gpio_ospeedr
#define RCC_AHB2ENR1_GPIOAEN 1U
#define RCC_AHB2ENR1_GPIOBEN 2U
#define RCC_AHB2ENR1_GPIOCEN 4U
#define RCC_APB1LENR_TIM2EN 1U
#define RCC_APB2ENR_TIM8EN 1U
#define GPIO_AF1 1U
#define GPIO_AF3 3U
#define TIM_SR_CC1IF 0x2U
#define TIM_CR1_CEN 1U
#define TIM_CCMR1_OC1M_ACTIVE 0x10U
#define TIM_CCMR1_OC1M_INACTIVE 0x20U
#define TIM_CCMR1_OC2M_ACTIVE 0x1000U
#define TIM_CCMR1_OC2M_INACTIVE 0x2000U
#define TIM_CCMR2_OC3M_ACTIVE 0x10U
#define TIM_CCMR2_OC3M_INACTIVE 0x20U
#define TIM_CCMR2_OC4M_ACTIVE 0x1000U
#define TIM_CCMR2_OC4M_INACTIVE 0x2000U
#define TIM_CCMR1_OC1M_FORCE_ACTIVE 0x50U
#define TIM_CCMR1_OC1M_FORCE_INACTIVE 0x40U
#define TIM_CCMR1_OC2M_FORCE_ACTIVE 0x5000U
#define TIM_CCMR1_OC2M_FORCE_INACTIVE 0x4000U
#define TIM_CCMR2_OC3M_FORCE_ACTIVE 0x50U
#define TIM_CCMR2_OC3M_FORCE_INACTIVE 0x40U
#define TIM_CCMR2_OC4M_FORCE_ACTIVE 0x5000U
#define TIM_CCMR2_OC4M_FORCE_INACTIVE 0x4000U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E 0x10U
#define TIM_CCER_CC3E 0x100U
#define TIM_CCER_CC4E 0x1000U
static inline void gpio_set_af(volatile uint32_t*, volatile uint32_t*, volatile uint32_t*,
                               volatile uint32_t*, uint8_t, uint8_t) noexcept {}
#endif

#define ECU_CHANNELS      8U
#define ECU_IGN_CH_FIRST  4U
#define ECU_CYCLE_DEG     720U
#define STM32_TIM_PSC_10MHZ 24U
#define STM32_MIN_COMPARE_LEAD_TICKS 20U

// Verificações de consistência do clock em tempo de compilação.
// Se qualquer uma falhar, a fórmula TIM5_ns → scheduler_ticks está errada.
// TIM2/TIM8: APB1_timer(250MHz) / (PSC+1) = 250MHz/25 = 10MHz = ECU_SCHED_CLOCK_HZ
static_assert(STM32_TIM_PSC_10MHZ == 24U,
    "PSC 10MHz: APB1_timer=250MHz, PSC+1=25, 250/25=10MHz");
static_assert(ECU_SCHED_CLOCK_HZ == 10000000U,
    "ECU_SCHED_CLOCK_HZ deve ser 10 000 000 Hz (100 ns/tick)");
static_assert(ECU_SCHED_NS_PER_TICK == 100U,
    "ECU_SCHED_NS_PER_TICK deve ser 100 ns @ 10 MHz");
static_assert(ECU_SCHED_TICKS_PER_US == 10U,
    "ECU_SCHED_TICKS_PER_US deve ser 10 @ 10 MHz");
#define TOOTH_NS_TO_SCHED(ns) ((uint32_t)((ns) / ECU_SCHED_NS_PER_TICK))

static AngleEvent_t g_angle_table[ECU_ANGLE_TABLE_SIZE];
static uint8_t g_angle_table_count;
static uint32_t g_angle_tooth_mask_lo;
static uint32_t g_angle_tooth_mask_hi;

volatile uint32_t g_late_event_count = 0U;
volatile uint32_t g_calibration_clamp_count = 0U;
volatile uint32_t g_cycle_schedule_drop_count = 0U;

// ── Dwell watchdog (MS42 §2.2.2.1.3 — TD × 1.4) ──────────────────────────
// Escrito pela ISR (arm_channel), lido pelo main loop (ecu_sched_dwell_watchdog).
// volatile necessário: compilador não pode cachear em registo entre os dois contextos.
static volatile uint32_t g_dwell_arm_tick[4]  = {0U, 0U, 0U, 0U};  // TIM2_CNT no arm de DWELL_START; 0 = inactivo
static volatile uint32_t g_dwell_wdog_ticks[4] = {0U, 0U, 0U, 0U};  // 1.4 × dwell_ticks no momento do arm
static volatile uint32_t g_dwell_watchdog_count = 0U;

// ── Per-cylinder inhibit masks (MS42 §2.2.5) ──────────────────────────────
// Escrito pelo main loop, lido pela ISR (arm_channel). bit N = cilindro N.
static volatile uint8_t g_inj_inhibit_mask = 0U;
// Ignition inhibit: suprime ECU_ACT_DWELL_START → bobina não carrega → sem faísca.
// Usado pelo soft rev limiter por ignição (retardo + corte alternado de cilindros).
static volatile uint8_t g_ign_inhibit_mask = 0U;

// ── Multi-spark (MS42 §2.2.3) ──────────────────────────────────────────────
// count=0 desabilitado. Valores escritos por ecu_sched_set_mspark() (main loop),
// lidos por Calculate_Sequential_Cycle() (ISR context via tooth hook).
static volatile uint8_t  g_mspark_count            = 0U;
static volatile uint32_t g_mspark_inter_dwell_ticks = 0U;
static volatile uint32_t g_mspark_atdc_limit_deg    = 18U;

static volatile uint32_t g_advance_deg = 10U;
static volatile uint32_t g_dwell_ticks = 30000U;
static volatile uint32_t g_inj_pw_ticks = 30000U;
static volatile uint32_t g_soi_lead_deg = 62U;
static volatile uint8_t g_presync_enable = 1U;
static volatile uint8_t g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
static volatile uint8_t g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
static volatile uint8_t g_presync_bank_toggle = 0U;
static volatile uint8_t g_hook_prev_valid = 0U;
static volatile uint16_t g_hook_prev_tooth = 0U;
static volatile uint8_t g_hook_schedule_this_gap = 1U;
static volatile uint8_t g_knock_sequential = 0U;  // 1 when running Calculate_Sequential_Cycle (full sync, per-cyl knock valid)
static volatile uint8_t g_ivc_abdc_deg = ems::engine::cfg::kIvcAbdcDeg;  // FIX: volatile — escrita por ecu_sched_set_ivc (cs), leitura por clamp_inj_pw_to_ivc (contexto ISR)
static uint32_t g_ivc_clamp_count = 0U;
static uint32_t g_oc_mode_shadow[ECU_CHANNELS];

static inline void enter_critical(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsid i" ::: "memory");
#endif
}

static inline void exit_critical(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsie i" ::: "memory");
#endif
}

static inline uint32_t scheduler_counter(void)
{
    return TIM2_CNT;
}

static inline uint8_t stm32_inj_tim_ch(uint8_t ch)
{
    switch (ch) {
        case ECU_CH_INJ1: return 1U;
        case ECU_CH_INJ2: return 2U;
        case ECU_CH_INJ3: return 3U;
        case ECU_CH_INJ4: return 4U;
        default: return 0U;
    }
}

// Mapeia canal de injecção para bit de cilindro no g_inj_inhibit_mask.
// Retorna 0 para canais de ignição ou desconhecidos.
static inline uint8_t inj_ch_to_cyl_bit(uint8_t ch)
{
    switch (ch) {
        case ECU_CH_INJ1: return (1U << 0U);
        case ECU_CH_INJ2: return (1U << 1U);
        case ECU_CH_INJ3: return (1U << 2U);
        case ECU_CH_INJ4: return (1U << 3U);
        default: return 0U;
    }
}

// Mapeia canal de ignição para bit de cilindro no g_ign_inhibit_mask.
static inline uint8_t ign_ch_to_cyl_bit(uint8_t ch)
{
    switch (ch) {
        case ECU_CH_IGN1: return (1U << 0U);
        case ECU_CH_IGN2: return (1U << 1U);
        case ECU_CH_IGN3: return (1U << 2U);
        case ECU_CH_IGN4: return (1U << 3U);
        default: return 0U;
    }
}

static inline uint8_t stm32_ign_tim_ch(uint8_t ch)
{
    switch (ch) {
        case ECU_CH_IGN1: return 1U;
        case ECU_CH_IGN2: return 2U;
        case ECU_CH_IGN3: return 3U;
        case ECU_CH_IGN4: return 4U;
        default: return 0U;
    }
}

static inline volatile uint32_t *stm32_tim_ccr(uint8_t is_inj, uint8_t tim_ch)
{
    if (is_inj != 0U) {
        switch (tim_ch) {
            case 1U: return &TIM2_CCR1;
            case 2U: return &TIM2_CCR2;
            case 3U: return &TIM2_CCR3;
            default: return &TIM2_CCR4;
        }
    }
    switch (tim_ch) {
        case 1U: return &TIM8_CCR1;
        case 2U: return &TIM8_CCR2;
        case 3U: return &TIM8_CCR3;
        default: return &TIM8_CCR4;
    }
}

static inline uint32_t stm32_tim_cc_flag(uint8_t tim_ch)
{
    return (uint32_t)(TIM_SR_CC1IF << (tim_ch - 1U));
}

static inline void stm32_write_oc_mode(uint8_t is_inj, uint8_t tim_ch, uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4)
{
    volatile uint32_t *ccmr1 = (is_inj != 0U) ? &TIM2_CCMR1 : &TIM8_CCMR1;
    volatile uint32_t *ccmr2 = (is_inj != 0U) ? &TIM2_CCMR2 : &TIM8_CCMR2;
    const uint8_t shadow_base = (is_inj != 0U) ? 0U : ECU_IGN_CH_FIRST;
    const uint8_t shadow_idx = (uint8_t)(shadow_base + tim_ch - 1U);
    uint32_t desired;

    switch (tim_ch) {
        case 1U: desired = m1; break;
        case 2U: desired = m2; break;
        case 3U: desired = m3; break;
        default: desired = m4; break;
    }
    if (g_oc_mode_shadow[shadow_idx] == desired) { return; }
    g_oc_mode_shadow[shadow_idx] = desired;

    switch (tim_ch) {
        case 1U: *ccmr1 = (*ccmr1 & ~0x70U) | desired; break;
        case 2U: *ccmr1 = (*ccmr1 & ~0x7000U) | desired; break;
        case 3U: *ccmr2 = (*ccmr2 & ~0x70U) | desired; break;
        default: *ccmr2 = (*ccmr2 & ~0x7000U) | desired; break;
    }
}

static inline void stm32_set_oc_mode(uint8_t is_inj, uint8_t tim_ch, uint8_t make_high)
{
    stm32_write_oc_mode(is_inj, tim_ch,
        (make_high != 0U) ? TIM_CCMR1_OC1M_ACTIVE : TIM_CCMR1_OC1M_INACTIVE,
        (make_high != 0U) ? TIM_CCMR1_OC2M_ACTIVE : TIM_CCMR1_OC2M_INACTIVE,
        (make_high != 0U) ? TIM_CCMR2_OC3M_ACTIVE : TIM_CCMR2_OC3M_INACTIVE,
        (make_high != 0U) ? TIM_CCMR2_OC4M_ACTIVE : TIM_CCMR2_OC4M_INACTIVE);
}

static inline void stm32_force_oc(uint8_t is_inj, uint8_t tim_ch, uint8_t high)
{
    stm32_write_oc_mode(is_inj, tim_ch,
        (high != 0U) ? TIM_CCMR1_OC1M_FORCE_ACTIVE : TIM_CCMR1_OC1M_FORCE_INACTIVE,
        (high != 0U) ? TIM_CCMR1_OC2M_FORCE_ACTIVE : TIM_CCMR1_OC2M_FORCE_INACTIVE,
        (high != 0U) ? TIM_CCMR2_OC3M_FORCE_ACTIVE : TIM_CCMR2_OC3M_FORCE_INACTIVE,
        (high != 0U) ? TIM_CCMR2_OC4M_FORCE_ACTIVE : TIM_CCMR2_OC4M_FORCE_INACTIVE);
}

static void sanitize_runtime_calibration(void)
{
    uint8_t clamped = 0U;
    if (g_advance_deg > 60U) { g_advance_deg = 60U; clamped = 1U; }
    if (g_dwell_ticks > 100000U) { g_dwell_ticks = 100000U; clamped = 1U; }
    if (g_inj_pw_ticks > 200000U) { g_inj_pw_ticks = 200000U; clamped = 1U; }
    if (g_soi_lead_deg >= ECU_CYCLE_DEG) { g_soi_lead_deg = ECU_CYCLE_DEG - 1U; clamped = 1U; }
    if (g_presync_inj_mode > ECU_PRESYNC_INJ_SEMI_SEQUENTIAL) { g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS; clamped = 1U; }
    if (g_presync_ign_mode > ECU_PRESYNC_IGN_WASTED_SPARK) { g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK; clamped = 1U; }
    if (clamped != 0U) { ++g_calibration_clamp_count; }
}

static void force_output(uint8_t ch, uint8_t action)
{
    const uint8_t is_inj = (ch < ECU_IGN_CH_FIRST) ? 1U : 0U;
    const uint8_t tim_ch = (is_inj != 0U) ? stm32_inj_tim_ch(ch) : stm32_ign_tim_ch(ch);
    const uint8_t high = ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U;
    if (tim_ch != 0U) { stm32_force_oc(is_inj, tim_ch, high); }
}

static void arm_channel(uint8_t ch, uint32_t target_cnv, uint8_t action)
{
    // FIX BUG-6: toda a operação (leitura do contador + escrita do CCR) deve ser
    // atômica. Sem seção crítica, TIM2_IRQ poderia disparar entre a leitura de
    // scheduler_counter() e a escrita do registrador de compare, corrompendo o
    // agendamento de eventos já pendentes.
    ems::hal::CriticalSectionGuard guard;
    
    const uint8_t is_inj = (ch < ECU_IGN_CH_FIRST) ? 1U : 0U;
    const uint8_t tim_ch = (is_inj != 0U) ? stm32_inj_tim_ch(ch) : stm32_ign_tim_ch(ch);
    const uint32_t now = scheduler_counter();
    const uint32_t delta = target_cnv - now;
    volatile uint32_t *ccr;

    if (tim_ch == 0U) { ++g_cycle_schedule_drop_count; return; }
    
    // FIX P0 (BUG-7): TIM8 é 16-bit — validar delta antes de descartar evento
    // Se delta > max_delta_16, o evento NÃO cabe no timer e deve ser descartado
    // Isso previne perda de ignição em baixa rotação onde dwell + advance > 65535 ticks
    if ((is_inj == 0U) && (delta > ems::engine::kTim8MaxDelta16)) { 
        ++g_cycle_schedule_drop_count; 
        return; 
    }
    
    // FIX P2 (BUG-9): Eventos atrasados devem ser DESCARTADOS, não forçados
    // Forçar saída em eventos atrasados causa spark/injeção no ângulo errado
    // Para ignição: pode disparar em cilindro em admissão (sem combustível)
    // Para injeção: injeta fora da janela de válvula aberta
    if (delta < STM32_MIN_COMPARE_LEAD_TICKS) { 
        ++g_late_event_count;
        // FIX C3: se DWELL_START já armou o watchdog mas o SPARK chega tarde e
        // é descartado aqui, o watchdog fica armado para sempre e vai disparar
        // após 1.4× dwell — cortando a bobina sem que ela tenha disparado.
        // Desarma o watchdog para qualquer evento de ignição tardio.
        if (is_inj == 0U && action == ECU_ACT_SPARK && tim_ch != 0U) {
            const uint8_t ign_idx = static_cast<uint8_t>(tim_ch - 1U);
            g_dwell_arm_tick[ign_idx] = 0U;
        }
        return; 
    }

    // ── Injection inhibit mask (MS42 §2.2.5 — corte por cilindro) ────────
    // Suprime ECU_ACT_INJ_ON se o bit do cilindro estiver activo.
    // INJ_OFF passa normalmente — fechar um injetor já fechado é inócuo.
    if (is_inj != 0U && action == ECU_ACT_INJ_ON) {
        const uint8_t cyl_bit = inj_ch_to_cyl_bit(ch);
        if (cyl_bit != 0U && (g_inj_inhibit_mask & cyl_bit) != 0U) { return; }
    }

    // ── Ignition inhibit mask (rev limiter por faísca) ────────────────────
    // Suprime DWELL_START e SPARK: drivers IGBT com detecção de corrente podem
    // interpretar o toggle do pino sem carga prévia como condição de falha.
    if (is_inj == 0U && (action == ECU_ACT_DWELL_START || action == ECU_ACT_SPARK)) {
        const uint8_t cyl_bit = ign_ch_to_cyl_bit(ch);
        if (cyl_bit != 0U && (g_ign_inhibit_mask & cyl_bit) != 0U) { return; }
    }

    // ── Dwell watchdog tracking (apenas canais de ignição) ────────────────
    // Executado depois de todos os guards de rejeição — só para eventos que
    // realmente serão programados no hardware.
    if (is_inj == 0U && tim_ch != 0U) {
        const uint8_t ign_idx = (uint8_t)(tim_ch - 1U);
        if (action == ECU_ACT_DWELL_START) {
            // Arma watchdog: timeout = 1.4 × dwell = 7/5 × dwell (sem ponto flutuante)
            g_dwell_arm_tick[ign_idx]   = now;
            g_dwell_wdog_ticks[ign_idx] = (g_dwell_ticks * 7U) / 5U;

            // Knock window management — only in sequential (full-sync) mode.
            // knock_window_cycle_end() closes the previous cylinder's window and
            // evaluates its knock count (one call per combustion interval, ~180 deg).
            // knock_window_open() arms CMP0 for the new cylinder.
            // ch → cyl: ECU_CH_IGN1=7→0, IGN2=6→1, IGN3=5→2, IGN4=4→3
            if (g_knock_sequential != 0U) {
                const uint8_t knock_cyl = static_cast<uint8_t>(7U - ch);
                ems::engine::knock_window_cycle_end();
                ems::engine::knock_window_open(knock_cyl);
            }
        } else if (action == ECU_ACT_SPARK) {
            // Evento SPARK programado com sucesso — watchdog já não é necessário
            g_dwell_arm_tick[ign_idx] = 0U;
        }
    }

	stm32_set_oc_mode(is_inj, tim_ch, ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U);
	// Clear any pending match flag BEFORE programming CCR to avoid missing an edge
	if (is_inj != 0U) {
		TIM2_SR &= ~stm32_tim_cc_flag(tim_ch);
	} else {
		TIM8_SR &= ~stm32_tim_cc_flag(tim_ch);
	}
#if defined(__arm__) || defined(__thumb__)
	__asm__ volatile("dmb" ::: "memory"); // Ensure OC mode + flag clear complete before CCR update
#endif
	ccr = stm32_tim_ccr(is_inj, tim_ch);
	if (is_inj != 0U) {
		*ccr = target_cnv;
	} else {
		// FIX P0: Handle 16-bit timer wraparound correctly
		// TIM8 is configured as 16-bit (ARR = 0xFFFF), so we must handle wrap
		const uint32_t current_cnt = TIM8_CNT & 0xFFFFU;
		uint32_t target = current_cnt + delta;
		if (target > 0xFFFFU) {
			target -= 0x10000U;  // Wrap within 16-bit range
		}
		*ccr = static_cast<uint16_t>(target);
	}
}

static void clear_all_events_and_drive_safe_outputs(void)
{
    g_angle_table_count = 0U;
    g_angle_tooth_mask_lo = 0U;
    g_angle_tooth_mask_hi = 0U;
    for (uint8_t i = 0U; i < ECU_CHANNELS; ++i) { force_output(i, (i < ECU_IGN_CH_FIRST) ? ECU_ACT_INJ_OFF : ECU_ACT_SPARK); }
    // Bobinas forçadas a LOW — watchdog já não é necessário para nenhum canal
    for (uint8_t i = 0U; i < 4U; ++i) { g_dwell_arm_tick[i] = 0U; }
    // Close any open knock window — sync lost means no valid combustion cylinder
    g_knock_sequential = 0U;
    ems::engine::knock_window_cycle_end();
}

static void angle_to_tooth_event(uint32_t angle_deg, uint8_t *out_tooth, uint8_t *out_sub_frac, uint8_t *out_phase_A)
{
    const uint32_t ang = angle_deg % 360U;
    uint32_t pos_x256 = (ang * 256U) / 6U;
    uint8_t tooth = (uint8_t)(pos_x256 >> 8U);
    uint8_t frac = (uint8_t)(pos_x256 & 0xFFU);
    if (tooth > 57U) { tooth = 57U; frac = 255U; }
    *out_phase_A = (angle_deg < 360U) ? ECU_PHASE_A : ECU_PHASE_B;
    *out_tooth = tooth;
    *out_sub_frac = frac;
}

static uint32_t engine_angle_to_trigger_angle(uint32_t engine_angle_deg, uint32_t cycle_deg)
{
    // FIX: usar valor em runtime (NVM/UART) em vez da constante de compilação.
    // kTriggerTooth0EngineDeg era o default hard-coded (0°); o campo de runtime
    // g_eng_cfg.trigger_tooth0_engine_deg é o valor calibrável por engine.
    const uint32_t trigger_offset =
        static_cast<uint32_t>(ems::engine::cfg::g_eng_cfg.trigger_tooth0_engine_deg) % cycle_deg;
    return (engine_angle_deg + cycle_deg - trigger_offset) % cycle_deg;
}

static void table_add(uint8_t tooth, uint8_t sub_frac, uint8_t phase_A, uint8_t channel, uint8_t action)
{
    if (g_angle_table_count >= ECU_ANGLE_TABLE_SIZE) { ++g_cycle_schedule_drop_count; return; }
    AngleEvent_t *e = &g_angle_table[g_angle_table_count++];
    e->tooth_index = tooth;
    e->sub_frac_x256 = sub_frac;
    e->phase_A = phase_A;
    e->channel = channel;
    e->action = action;
    e->valid = 1U;
    if (tooth < 32U) {
        g_angle_tooth_mask_lo |= (1UL << tooth);
    } else {
        g_angle_tooth_mask_hi |= (1UL << (tooth - 32U));
    }
}

void ECU_Hardware_Init(void)
{
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOBEN | RCC_AHB2ENR1_GPIOCEN;
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;
    RCC_APB2ENR  |= RCC_APB2ENR_TIM8EN;

    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 6U, GPIO_AF3);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 7U, GPIO_AF3);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 8U, GPIO_AF3);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 9U, GPIO_AF3);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 15U, GPIO_AF1);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 3U, GPIO_AF1);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 10U, GPIO_AF1);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 11U, GPIO_AF1);

    TIM8_CR1 = 0U; TIM8_DIER = 0U; TIM8_SR = 0U; TIM8_PSC = STM32_TIM_PSC_10MHZ; TIM8_CNT = 0U; TIM8_ARR = 0xFFFFU;
    TIM8_CCMR1 = TIM_CCMR1_OC1M_FORCE_INACTIVE | TIM_CCMR1_OC2M_FORCE_INACTIVE;
    TIM8_CCMR2 = TIM_CCMR2_OC3M_FORCE_INACTIVE | TIM_CCMR2_OC4M_FORCE_INACTIVE;
    TIM8_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; TIM8_BDTR = (1U << 15); TIM8_EGR = 1U;

    TIM2_CR1 = 0U; TIM2_DIER = 0U; TIM2_SR = 0U; TIM2_PSC = STM32_TIM_PSC_10MHZ; TIM2_CNT = 0U; TIM2_ARR = 0xFFFFFFFFU;
    TIM2_CCMR1 = TIM_CCMR1_OC1M_FORCE_INACTIVE | TIM_CCMR1_OC2M_FORCE_INACTIVE;
    TIM2_CCMR2 = TIM_CCMR2_OC3M_FORCE_INACTIVE | TIM_CCMR2_OC4M_FORCE_INACTIVE;
    TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; TIM2_EGR = 1U;

    g_oc_mode_shadow[0] = TIM_CCMR1_OC1M_FORCE_INACTIVE;
    g_oc_mode_shadow[1] = TIM_CCMR1_OC2M_FORCE_INACTIVE;
    g_oc_mode_shadow[2] = TIM_CCMR2_OC3M_FORCE_INACTIVE;
    g_oc_mode_shadow[3] = TIM_CCMR2_OC4M_FORCE_INACTIVE;
    g_oc_mode_shadow[4] = TIM_CCMR1_OC1M_FORCE_INACTIVE;
    g_oc_mode_shadow[5] = TIM_CCMR1_OC2M_FORCE_INACTIVE;
    g_oc_mode_shadow[6] = TIM_CCMR2_OC3M_FORCE_INACTIVE;
    g_oc_mode_shadow[7] = TIM_CCMR2_OC4M_FORCE_INACTIVE;

    TIM8_CR1 = TIM_CR1_CEN;
    TIM2_CR1 = TIM_CR1_CEN;
    clear_all_events_and_drive_safe_outputs();
}

static uint32_t ticks_to_cycle_degrees(uint32_t ticks, uint32_t tooth_period_ns, uint32_t cycle_deg)
{
    const uint64_t tooth_ticks = static_cast<uint64_t>(TOOTH_NS_TO_SCHED(tooth_period_ns));
    const uint64_t factor = (cycle_deg == ECU_CYCLE_DEG) ? 120ULL : 60ULL;
    const uint64_t denom  = tooth_ticks * factor;
    return (denom > 0ULL)
        ? static_cast<uint32_t>((static_cast<uint64_t>(ticks) * cycle_deg) / denom)
        : 0U;
}

static uint32_t clamp_inj_pw_to_ivc(uint32_t tdc_deg, uint32_t inj_on_deg, uint32_t inj_pw_deg)
{
    const uint32_t ivc_cycle_deg = (tdc_deg + 540U + (uint32_t)g_ivc_abdc_deg) % ECU_CYCLE_DEG;
    const uint32_t soi_to_ivc = (ivc_cycle_deg + ECU_CYCLE_DEG - inj_on_deg) % ECU_CYCLE_DEG;
    if ((soi_to_ivc < (ECU_CYCLE_DEG / 2U)) && (inj_pw_deg > soi_to_ivc)) { ++g_ivc_clamp_count; return soi_to_ivc; }
    return inj_pw_deg;
}

static void Calculate_Sequential_Cycle(const ems::drv::CkpSnapshot& snap)
{
    static_assert(ems::engine::cfg::kCylinderCount == 4u, "ign_ch/inj_ch hardcoded for 4 cylinders");
    static const uint8_t ign_ch[ems::engine::cfg::kCylinderCount] = {ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};
    static const uint8_t inj_ch[ems::engine::cfg::kCylinderCount] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};

    g_knock_sequential = 1U;  // knock window per-cylinder valid in sequential mode
    g_angle_table_count = 0U;
    g_angle_tooth_mask_lo = 0U;
    g_angle_tooth_mask_hi = 0U;

    const uint32_t dwell_deg = ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, ECU_CYCLE_DEG);
    const uint32_t base_inj_pw_deg = ticks_to_cycle_degrees(g_inj_pw_ticks, snap.tooth_period_ns, ECU_CYCLE_DEG);

    for (uint8_t seq = 0U; seq < ems::engine::cfg::kCylinderCount; ++seq) {
        const uint8_t cyl = ems::engine::cfg::kFiringOrder[seq];
        const uint32_t tdc = ems::engine::cfg::cyl_tdc_deg(cyl);
        const uint32_t spark = (tdc + ECU_CYCLE_DEG - g_advance_deg) % ECU_CYCLE_DEG;
        const uint32_t dwell = (spark + ECU_CYCLE_DEG - dwell_deg) % ECU_CYCLE_DEG;
        const uint32_t inj_on = (tdc + ECU_CYCLE_DEG - g_soi_lead_deg) % ECU_CYCLE_DEG;
        const uint32_t inj_pw = clamp_inj_pw_to_ivc(tdc, inj_on, base_inj_pw_deg);
        const uint32_t inj_off = (inj_on + inj_pw) % ECU_CYCLE_DEG;
        uint8_t tooth, frac, phase;

        angle_to_tooth_event(engine_angle_to_trigger_angle(dwell, ECU_CYCLE_DEG), &tooth, &frac, &phase);
        table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_DWELL_START);
        angle_to_tooth_event(engine_angle_to_trigger_angle(spark, ECU_CYCLE_DEG), &tooth, &frac, &phase);
        table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_SPARK);

        // ── Additional sparks (MS42 §2.2.3 — multi-spark) ──────────────
        // Cada spark adicional n usa o mesmo canal de ignição do cilindro.
        // add_dwell_n arranca 1° após SPARK_{n-1} para evitar conflito no mesmo tooth.
        // add_spark_n dispara inter_dwell_deg depois de add_dwell_n.
        // Interrompido se o spark ultrapassar o limite ATDC configurado.
        {
            const uint8_t ms_count = g_mspark_count;
            if (ms_count > 0U && snap.tooth_period_ns > 0U) {
                const uint32_t inter_deg = ticks_to_cycle_degrees(
                    g_mspark_inter_dwell_ticks, snap.tooth_period_ns, ECU_CYCLE_DEG);
                const uint32_t step      = inter_deg + 1U;  // +1° = epsilon anti-conflito
                const uint32_t window    = g_advance_deg + g_mspark_atdc_limit_deg;
                for (uint8_t n = 1U; n <= ms_count; ++n) {
                    const uint32_t add_spark_off = (uint32_t)n * step;
                    if (add_spark_off >= window) { break; }
                    const uint32_t add_dwell_off = (uint32_t)(n - 1U) * step + 1U;
                    const uint32_t add_dwell_ang = (spark + add_dwell_off) % ECU_CYCLE_DEG;
                    const uint32_t add_spark_ang = (spark + add_spark_off) % ECU_CYCLE_DEG;
                    angle_to_tooth_event(engine_angle_to_trigger_angle(add_dwell_ang, ECU_CYCLE_DEG), &tooth, &frac, &phase);
                    table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_DWELL_START);
                    angle_to_tooth_event(engine_angle_to_trigger_angle(add_spark_ang, ECU_CYCLE_DEG), &tooth, &frac, &phase);
                    table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_SPARK);
                }
            }
        }

        angle_to_tooth_event(engine_angle_to_trigger_angle(inj_on, ECU_CYCLE_DEG), &tooth, &frac, &phase);
        table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_ON);
        angle_to_tooth_event(engine_angle_to_trigger_angle(inj_off, ECU_CYCLE_DEG), &tooth, &frac, &phase);
        table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_OFF);
    }
}

static void calculate_presync_revolution(const ems::drv::CkpSnapshot& snap)
{
    static const uint8_t inj_all[4U] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
    static const uint8_t inj_a[2U] = {ECU_CH_INJ1, ECU_CH_INJ4};
    static const uint8_t inj_b[2U] = {ECU_CH_INJ2, ECU_CH_INJ3};
    static const uint8_t ign[4U] = {ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};
    uint8_t tooth, frac, phase;

    g_knock_sequential = 0U;  // wasted-spark presync: no per-cylinder knock tracking
    g_angle_table_count = 0U;
    g_angle_tooth_mask_lo = 0U;
    g_angle_tooth_mask_hi = 0U;

    const uint32_t dwell_deg = ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, 360U);
    const uint32_t inj_pw_deg = ticks_to_cycle_degrees(g_inj_pw_ticks, snap.tooth_period_ns, 360U);
    const uint32_t spark = (360U - (g_advance_deg % 360U)) % 360U;
    const uint32_t dwell = (spark + 360U - dwell_deg) % 360U;
    const uint32_t inj_on = (360U - (g_soi_lead_deg % 360U)) % 360U;
    const uint32_t inj_off = (inj_on + inj_pw_deg) % 360U;

    angle_to_tooth_event(engine_angle_to_trigger_angle(dwell, 360U), &tooth, &frac, &phase);
    for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_DWELL_START); }
    angle_to_tooth_event(engine_angle_to_trigger_angle(spark, 360U), &tooth, &frac, &phase);
    for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_SPARK); }

    angle_to_tooth_event(engine_angle_to_trigger_angle(inj_on, 360U), &tooth, &frac, &phase);
    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
        for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_ON); }
    } else {
        const uint8_t *bank = (g_presync_bank_toggle == 0U) ? inj_a : inj_b;
        for (uint8_t i = 0U; i < 2U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, bank[i], ECU_ACT_INJ_ON); }
        g_presync_bank_toggle ^= 1U;
    }

    angle_to_tooth_event(engine_angle_to_trigger_angle(inj_off, 360U), &tooth, &frac, &phase);
    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
        for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_OFF); }
    } else {
        /* close the bank that was opened this revolution; toggle already advanced */
        const uint8_t *bank = (g_presync_bank_toggle == 1U) ? inj_a : inj_b;
        for (uint8_t i = 0U; i < 2U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, bank[i], ECU_ACT_INJ_OFF); }
    }
}

void ecu_sched_commit_calibration(uint32_t advance_deg, uint32_t dwell_ticks, uint32_t inj_pw_ticks, uint32_t soi_lead_deg)
{
    ems::hal::CriticalSectionGuard guard;
    g_advance_deg = advance_deg;
    g_dwell_ticks = dwell_ticks;
    g_inj_pw_ticks = inj_pw_ticks;
    g_soi_lead_deg = soi_lead_deg;
    sanitize_runtime_calibration();
}
void ecu_sched_set_advance_deg(uint32_t adv) { ems::hal::CriticalSectionGuard guard; g_advance_deg = adv; sanitize_runtime_calibration(); }
void ecu_sched_set_dwell_ticks(uint32_t dwell) { ems::hal::CriticalSectionGuard guard; g_dwell_ticks = dwell; sanitize_runtime_calibration(); }
void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks) { ems::hal::CriticalSectionGuard guard; g_inj_pw_ticks = pw_ticks; sanitize_runtime_calibration(); }
void ecu_sched_set_soi_lead_deg(uint32_t soi_lead_deg) { ems::hal::CriticalSectionGuard guard; g_soi_lead_deg = soi_lead_deg; sanitize_runtime_calibration(); }
void ecu_sched_set_presync_enable(uint8_t enable) { ems::hal::CriticalSectionGuard guard; g_presync_enable = (enable != 0U) ? 1U : 0U; }
void ecu_sched_set_presync_inj_mode(uint8_t mode) { ems::hal::CriticalSectionGuard guard; g_presync_inj_mode = mode; sanitize_runtime_calibration(); }
void ecu_sched_set_presync_ign_mode(uint8_t mode) { ems::hal::CriticalSectionGuard guard; g_presync_ign_mode = mode; sanitize_runtime_calibration(); }
void ecu_sched_set_ivc(uint8_t ivc_abdc_deg) { ems::hal::CriticalSectionGuard guard; g_ivc_abdc_deg = (ivc_abdc_deg > 180U) ? 180U : ivc_abdc_deg; }
uint32_t ecu_sched_ivc_clamp_count(void) { return g_ivc_clamp_count; }

void ecu_sched_dwell_watchdog(void)
{
    // TIM2 lido UMA vez fora do loop — contador 32-bit, não há risco de wrap
    // num intervalo de 2 ms entre chamadas (~20 000 ticks @ 10 MHz).
    const uint32_t now = TIM2_CNT;
    for (uint8_t i = 0U; i < 4U; ++i) {
        // FIX C1: leitura + avaliação + acção dentro de UMA secção crítica.
        // A versão anterior lia arm/tout dentro de CS, saía, avaliava fora,
        // depois entrava numa segunda CS para agir. A ISR arm_channel() pode
        // zerar g_dwell_arm_tick[i] (acção SPARK, linha 389) nessa janela,
        // causando disparo espúrio do watchdog e corte indevido da bobina.
        enter_critical();
        const uint32_t arm  = g_dwell_arm_tick[i];
        const uint32_t tout = g_dwell_wdog_ticks[i];
        if (arm != 0U && (now - arm) >= tout) {
            stm32_force_oc(0U, (uint8_t)(i + 1U), 0U);
            g_dwell_arm_tick[i] = 0U;
            ++g_dwell_watchdog_count;
        }
        exit_critical();
    }
}

uint32_t ecu_sched_dwell_watchdog_count(void) { return g_dwell_watchdog_count; }

void ecu_sched_reset_diagnostic_counters(void)
{
    ems::hal::CriticalSectionGuard guard;
    g_late_event_count = 0U;
    g_cycle_schedule_drop_count = 0U;
    g_calibration_clamp_count = 0U;
    g_ivc_clamp_count = 0U;
    g_dwell_watchdog_count = 0U;
}

void ecu_sched_fire_prime_pulse(uint32_t pw_us)
{
    static const uint8_t inj[4U] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
    if (pw_us == 0U) { return; }
    if (pw_us > 30000U) { pw_us = 30000U; }
    const uint32_t off_cnv = scheduler_counter() + ((pw_us * ECU_SCHED_TICKS_PER_MS) / 1000U);
    for (uint8_t i = 0U; i < 4U; ++i) { force_output(inj[i], ECU_ACT_INJ_ON); }
    for (uint8_t i = 0U; i < 4U; ++i) { arm_channel(inj[i], off_cnv, ECU_ACT_INJ_OFF); }
}

void ecu_sched_set_mspark(uint8_t count, uint32_t inter_dwell_ticks, uint32_t atdc_limit_deg)
{
    ems::hal::CriticalSectionGuard guard;
    g_mspark_count            = (count > 3U) ? 3U : count;
    g_mspark_inter_dwell_ticks = inter_dwell_ticks;
    g_mspark_atdc_limit_deg   = (atdc_limit_deg == 0U) ? 18U : atdc_limit_deg;
}

void ecu_sched_set_inj_inhibit_mask(uint8_t mask)
{
    ems::hal::CriticalSectionGuard guard;
    g_inj_inhibit_mask = mask & 0x0FU;
}
uint8_t ecu_sched_get_inj_inhibit_mask(void) { return g_inj_inhibit_mask; }

void ecu_sched_set_ign_inhibit_mask(uint8_t mask)
{
    ems::hal::CriticalSectionGuard guard;
    g_ign_inhibit_mask = mask & 0x0FU;
}
uint8_t ecu_sched_get_ign_inhibit_mask(void) { return g_ign_inhibit_mask; }

namespace ems::engine {
void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept
{
    if ((snap.state != ems::drv::SyncState::FULL_SYNC) && (snap.state != ems::drv::SyncState::HALF_SYNC)) {
        if (g_hook_prev_valid != 0U) { clear_all_events_and_drive_safe_outputs(); }
        g_hook_prev_valid = 0U; g_hook_prev_tooth = 0U; g_hook_schedule_this_gap = 1U; return;
    }

    const uint8_t rev_boundary = ((g_hook_prev_valid != 0U) && (snap.tooth_index == 0U) && (g_hook_prev_tooth != 0U)) ? 1U : 0U;
    if (rev_boundary != 0U) {
        if ((snap.state == ems::drv::SyncState::HALF_SYNC) && (g_presync_enable != 0U)) {
            calculate_presync_revolution(snap);
        } else {
            if (g_hook_schedule_this_gap != 0U) {
                Calculate_Sequential_Cycle(snap);
                g_hook_schedule_this_gap = 0U;
            } else {
                g_hook_schedule_this_gap = 1U;
            }
        }
    }

    const uint8_t tooth_index = (uint8_t)snap.tooth_index;
    const uint32_t tooth_mask = (tooth_index < 32U)
        ? (g_angle_tooth_mask_lo & (1UL << tooth_index))
        : (g_angle_tooth_mask_hi & (1UL << (tooth_index - 32U)));
    if (tooth_mask != 0U) {
        const uint32_t period_ns = (snap.predicted_tooth_period_ns != 0U)
            ? snap.predicted_tooth_period_ns
            : snap.tooth_period_ns;
        uint32_t tooth_ticks = TOOTH_NS_TO_SCHED(period_ns);
        const uint32_t now = scheduler_counter();
        const uint8_t current_phase = snap.phase_A ? ECU_PHASE_A : ECU_PHASE_B;
        for (uint8_t i = 0U; i < g_angle_table_count; ++i) {
            const AngleEvent_t *e = &g_angle_table[i];
            if (e->tooth_index != tooth_index) { continue; }
            if ((e->phase_A != ECU_PHASE_ANY) && (e->phase_A != current_phase)) { continue; }
            arm_channel(e->channel, now + ((e->sub_frac_x256 * tooth_ticks) >> 8U), e->action);
        }
    }

    g_hook_prev_valid = 1U; g_hook_prev_tooth = snap.tooth_index;
}
}

namespace ems::drv {
void schedule_on_tooth(const CkpSnapshot& snap) noexcept { ems::engine::ecu_sched_on_tooth_hook(snap); }
}

#if defined(EMS_HOST_TEST)
void ecu_sched_test_reset(void)
{
    g_late_event_count = 0U; g_cycle_schedule_drop_count = 0U; g_calibration_clamp_count = 0U;
    g_presync_enable = 1U; g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS; g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
    g_presync_bank_toggle = 0U; g_hook_prev_valid = 0U; g_hook_prev_tooth = 0U; g_hook_schedule_this_gap = 1U;
    g_advance_deg = 10U; g_dwell_ticks = 22500U; g_inj_pw_ticks = 22500U; g_soi_lead_deg = 62U;
    g_angle_table_count = 0U; g_angle_tooth_mask_lo = 0U; g_angle_tooth_mask_hi = 0U;
    g_ivc_abdc_deg = ems::engine::cfg::kIvcAbdcDeg; g_ivc_clamp_count = 0U;
    g_inj_inhibit_mask = 0U;
    g_ign_inhibit_mask = 0U;
    g_mspark_count = 0U; g_mspark_inter_dwell_ticks = 0U; g_mspark_atdc_limit_deg = 18U;
}
uint8_t ecu_sched_test_angle_table_size(void) { return g_angle_table_count; }
uint8_t ecu_sched_test_get_angle_event(uint8_t index, uint8_t *tooth, uint8_t *sub_frac, uint8_t *ch, uint8_t *action, uint8_t *phase)
{
    if ((index >= g_angle_table_count) || (g_angle_table[index].valid == 0U)) { return 0U; }
    *tooth = g_angle_table[index].tooth_index; *sub_frac = g_angle_table[index].sub_frac_x256; *ch = g_angle_table[index].channel; *action = g_angle_table[index].action; *phase = g_angle_table[index].phase_A; return 1U;
}
void ecu_sched_test_set_advance_deg(uint32_t adv) { ecu_sched_set_advance_deg(adv); }
void ecu_sched_test_set_dwell_ticks(uint32_t dwell) { ecu_sched_set_dwell_ticks(dwell); }
void ecu_sched_test_set_inj_pw_ticks(uint32_t pw_ticks) { ecu_sched_set_inj_pw_ticks(pw_ticks); }
void ecu_sched_test_set_soi_lead_deg(uint32_t soi_lead_deg) { ecu_sched_set_soi_lead_deg(soi_lead_deg); }
uint32_t ecu_sched_test_get_advance_deg(void) { return g_advance_deg; }
uint32_t ecu_sched_test_get_dwell_ticks(void) { return g_dwell_ticks; }
uint32_t ecu_sched_test_get_inj_pw_ticks(void) { return g_inj_pw_ticks; }
uint32_t ecu_sched_test_get_soi_lead_deg(void) { return g_soi_lead_deg; }
uint32_t ecu_sched_test_get_calibration_clamp_count(void) { return g_calibration_clamp_count; }
uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void) { return g_cycle_schedule_drop_count; }
uint32_t ecu_sched_test_get_late_event_count(void) { return g_late_event_count; }
void ecu_sched_test_set_ivc(uint8_t ivc_abdc_deg) { ecu_sched_set_ivc(ivc_abdc_deg); }
uint32_t ecu_sched_test_get_ivc_clamp_count(void) { return g_ivc_clamp_count; }
void ecu_sched_test_set_mspark(uint8_t count, uint32_t inter_dwell_ticks, uint32_t atdc_limit_deg) {
    ecu_sched_set_mspark(count, inter_dwell_ticks, atdc_limit_deg);
}
uint8_t ecu_sched_test_get_mspark_count(void) { return g_mspark_count; }
void ecu_sched_test_set_tim8_cnt(uint32_t cnt) noexcept { ems_test_tim8_cnt = cnt; }
void ecu_sched_test_set_tim2_cnt(uint32_t cnt) noexcept { ems_test_tim2_cnt = cnt; }
uint32_t ecu_sched_test_get_tim8_ccr(uint8_t ch) noexcept {
    switch (ch) {
        case 1u: return ems_test_tim8_ccr1;
        case 2u: return ems_test_tim8_ccr2;
        case 3u: return ems_test_tim8_ccr3;
        case 4u: return ems_test_tim8_ccr4;
        default: return 0u;
    }
}
#endif
