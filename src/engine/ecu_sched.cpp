#include "engine/ecu_sched.h"
#include "drv/ckp.h"
#include "hal/regs.h"

#include <stddef.h>
#include <stdint.h>

#define ECU_CHANNELS      8U
#define ECU_IGN_CH_FIRST  4U
#define ECU_CYCLE_DEG     720U
#define ECU_NUM_CYL       4U
#define STM32_TIM_PSC_10MHZ 24U
#define STM32_MIN_COMPARE_LEAD_TICKS 20U
#define TOOTH_NS_TO_SCHED(ns) ((uint32_t)((ns) / ECU_SCHED_NS_PER_TICK))

static AngleEvent_t g_angle_table[ECU_ANGLE_TABLE_SIZE];
static uint8_t g_angle_table_count;

volatile uint32_t g_late_event_count = 0U;
volatile uint32_t g_calibration_clamp_count = 0U;
volatile uint32_t g_cycle_schedule_drop_count = 0U;

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
static uint8_t g_ivc_abdc_deg = 50U;
static uint32_t g_ivc_clamp_count = 0U;

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
    switch (tim_ch) {
        case 1U: *ccmr1 = (*ccmr1 & ~0x70U) | m1; break;
        case 2U: *ccmr1 = (*ccmr1 & ~0x7000U) | m2; break;
        case 3U: *ccmr2 = (*ccmr2 & ~0x70U) | m3; break;
        default: *ccmr2 = (*ccmr2 & ~0x7000U) | m4; break;
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
    const uint8_t is_inj = (ch < ECU_IGN_CH_FIRST) ? 1U : 0U;
    const uint8_t tim_ch = (is_inj != 0U) ? stm32_inj_tim_ch(ch) : stm32_ign_tim_ch(ch);
    const uint32_t now = scheduler_counter();
    const uint32_t delta = target_cnv - now;
    const uint32_t timer_now = (is_inj != 0U) ? TIM2_CNT : (uint16_t)(TIM8_CNT & 0xFFFFU);
    volatile uint32_t *ccr;

    if (tim_ch == 0U) { ++g_cycle_schedule_drop_count; return; }
    if ((is_inj == 0U) && (delta > 0xFFFFU)) { ++g_cycle_schedule_drop_count; return; }
    if (delta < STM32_MIN_COMPARE_LEAD_TICKS) { ++g_late_event_count; force_output(ch, action); return; }

    stm32_set_oc_mode(is_inj, tim_ch, ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U);
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("dmb" ::: "memory");
#endif
    ccr = stm32_tim_ccr(is_inj, tim_ch);
    *ccr = (is_inj != 0U) ? (timer_now + delta) : (uint16_t)(timer_now + delta);
    if (is_inj != 0U) { TIM2_SR = ~stm32_tim_cc_flag(tim_ch); }
    else { TIM8_SR = ~stm32_tim_cc_flag(tim_ch); }
}

static void clear_all_events_and_drive_safe_outputs(void)
{
    for (uint8_t i = 0U; i < ECU_ANGLE_TABLE_SIZE; ++i) { g_angle_table[i].valid = 0U; }
    g_angle_table_count = 0U;
    for (uint8_t i = 0U; i < ECU_CHANNELS; ++i) { force_output(i, (i < ECU_IGN_CH_FIRST) ? ECU_ACT_INJ_OFF : ECU_ACT_SPARK); }
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

    TIM8_CR1 = TIM_CR1_CEN;
    TIM2_CR1 = TIM_CR1_CEN;
    clear_all_events_and_drive_safe_outputs();
}

static uint32_t ticks_to_cycle_degrees(uint32_t ticks, uint32_t tooth_period_ns, uint32_t cycle_deg)
{
    uint32_t tooth_ticks = TOOTH_NS_TO_SCHED(tooth_period_ns);
    uint32_t denom = tooth_ticks * ((cycle_deg == ECU_CYCLE_DEG) ? 120U : 60U);
    return (denom > 0U) ? ((ticks * cycle_deg) / denom) : 0U;
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
    static const uint8_t fire_order[ECU_NUM_CYL] = {0U, 2U, 3U, 1U};
    static const uint32_t tdc_deg[ECU_NUM_CYL] = {0U, 180U, 360U, 540U};
    static const uint8_t ign_ch[ECU_NUM_CYL] = {ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};
    static const uint8_t inj_ch[ECU_NUM_CYL] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};

    sanitize_runtime_calibration();
    g_angle_table_count = 0U;
    for (uint8_t i = 0U; i < ECU_ANGLE_TABLE_SIZE; ++i) { g_angle_table[i].valid = 0U; }

    const uint32_t dwell_deg = ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, ECU_CYCLE_DEG);
    const uint32_t base_inj_pw_deg = ticks_to_cycle_degrees(g_inj_pw_ticks, snap.tooth_period_ns, ECU_CYCLE_DEG);

    for (uint8_t seq = 0U; seq < ECU_NUM_CYL; ++seq) {
        const uint8_t cyl = fire_order[seq];
        const uint32_t tdc = tdc_deg[cyl];
        const uint32_t spark = (tdc + ECU_CYCLE_DEG - g_advance_deg) % ECU_CYCLE_DEG;
        const uint32_t dwell = (spark + ECU_CYCLE_DEG - dwell_deg) % ECU_CYCLE_DEG;
        const uint32_t inj_on = (tdc + ECU_CYCLE_DEG - g_soi_lead_deg) % ECU_CYCLE_DEG;
        const uint32_t inj_pw = clamp_inj_pw_to_ivc(tdc, inj_on, base_inj_pw_deg);
        const uint32_t inj_off = (inj_on + inj_pw) % ECU_CYCLE_DEG;
        uint8_t tooth, frac, phase;

        angle_to_tooth_event(dwell, &tooth, &frac, &phase); table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_DWELL_START);
        angle_to_tooth_event(spark, &tooth, &frac, &phase); table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_SPARK);
        angle_to_tooth_event(inj_on, &tooth, &frac, &phase); table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_ON);
        angle_to_tooth_event(inj_off, &tooth, &frac, &phase); table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_OFF);
    }
}

static void calculate_presync_revolution(const ems::drv::CkpSnapshot& snap)
{
    static const uint8_t inj_all[4U] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
    static const uint8_t inj_a[2U] = {ECU_CH_INJ1, ECU_CH_INJ4};
    static const uint8_t inj_b[2U] = {ECU_CH_INJ2, ECU_CH_INJ3};
    static const uint8_t ign[4U] = {ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};
    uint8_t tooth, frac, phase;

    sanitize_runtime_calibration();
    g_angle_table_count = 0U;
    for (uint8_t i = 0U; i < ECU_ANGLE_TABLE_SIZE; ++i) { g_angle_table[i].valid = 0U; }

    const uint32_t dwell_deg = ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, 360U);
    const uint32_t inj_pw_deg = ticks_to_cycle_degrees(g_inj_pw_ticks, snap.tooth_period_ns, 360U);
    const uint32_t spark = (360U - (g_advance_deg % 360U)) % 360U;
    const uint32_t dwell = (spark + 360U - dwell_deg) % 360U;
    const uint32_t inj_on = (360U - (g_soi_lead_deg % 360U)) % 360U;
    const uint32_t inj_off = (inj_on + inj_pw_deg) % 360U;

    angle_to_tooth_event(dwell, &tooth, &frac, &phase); for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_DWELL_START); }
    angle_to_tooth_event(spark, &tooth, &frac, &phase); for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_SPARK); }
    angle_to_tooth_event(inj_on, &tooth, &frac, &phase);
    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) { for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_ON); } }
    else { const uint8_t *bank = (g_presync_bank_toggle == 0U) ? inj_a : inj_b; for (uint8_t i = 0U; i < 2U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, bank[i], ECU_ACT_INJ_ON); } g_presync_bank_toggle ^= 1U; }
    angle_to_tooth_event(inj_off, &tooth, &frac, &phase);
    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) { for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_OFF); } }
    else { for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_OFF); } }
}

void ecu_sched_commit_calibration(uint32_t advance_deg, uint32_t dwell_ticks, uint32_t inj_pw_ticks, uint32_t soi_lead_deg)
{
    enter_critical(); g_advance_deg = advance_deg; g_dwell_ticks = dwell_ticks; g_inj_pw_ticks = inj_pw_ticks; g_soi_lead_deg = soi_lead_deg; sanitize_runtime_calibration(); exit_critical();
}
void ecu_sched_set_advance_deg(uint32_t adv) { enter_critical(); g_advance_deg = adv; sanitize_runtime_calibration(); exit_critical(); }
void ecu_sched_set_dwell_ticks(uint32_t dwell) { enter_critical(); g_dwell_ticks = dwell; sanitize_runtime_calibration(); exit_critical(); }
void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks) { enter_critical(); g_inj_pw_ticks = pw_ticks; sanitize_runtime_calibration(); exit_critical(); }
void ecu_sched_set_soi_lead_deg(uint32_t soi_lead_deg) { enter_critical(); g_soi_lead_deg = soi_lead_deg; sanitize_runtime_calibration(); exit_critical(); }
void ecu_sched_set_presync_enable(uint8_t enable) { enter_critical(); g_presync_enable = (enable != 0U) ? 1U : 0U; exit_critical(); }
void ecu_sched_set_presync_inj_mode(uint8_t mode) { enter_critical(); g_presync_inj_mode = mode; sanitize_runtime_calibration(); exit_critical(); }
void ecu_sched_set_presync_ign_mode(uint8_t mode) { enter_critical(); g_presync_ign_mode = mode; sanitize_runtime_calibration(); exit_critical(); }
void ecu_sched_set_ivc(uint8_t ivc_abdc_deg) { enter_critical(); g_ivc_abdc_deg = (ivc_abdc_deg > 180U) ? 180U : ivc_abdc_deg; exit_critical(); }
uint32_t ecu_sched_ivc_clamp_count(void) { return g_ivc_clamp_count; }

void ecu_sched_reset_diagnostic_counters(void)
{
    enter_critical(); g_late_event_count = 0U; g_cycle_schedule_drop_count = 0U; g_calibration_clamp_count = 0U; g_ivc_clamp_count = 0U; exit_critical();
}

void ecu_sched_fire_prime_pulse(uint32_t pw_us)
{
    static const uint8_t inj[4U] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
    if (pw_us == 0U) { return; }
    if (pw_us > 30000U) { pw_us = 30000U; }
    const uint32_t off_cnv = scheduler_counter() + ((pw_us * ECU_SCHED_TICKS_PER_MS) / 1000U);
    enter_critical();
    for (uint8_t i = 0U; i < 4U; ++i) { force_output(inj[i], ECU_ACT_INJ_ON); }
    for (uint8_t i = 0U; i < 4U; ++i) { arm_channel(inj[i], off_cnv, ECU_ACT_INJ_OFF); }
    exit_critical();
}

namespace ems::engine {
void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept
{
    if ((snap.state != ems::drv::SyncState::FULL_SYNC) && (snap.state != ems::drv::SyncState::HALF_SYNC)) {
        if (g_hook_prev_valid != 0U) { clear_all_events_and_drive_safe_outputs(); }
        g_hook_prev_valid = 0U; g_hook_prev_tooth = 0U; g_hook_schedule_this_gap = 1U; return;
    }

    uint32_t tooth_ticks = TOOTH_NS_TO_SCHED(snap.tooth_period_ns);
    const uint32_t now = scheduler_counter();
    const uint8_t current_phase = snap.phase_A ? ECU_PHASE_A : ECU_PHASE_B;
    for (uint8_t i = 0U; i < g_angle_table_count; ++i) {
        const AngleEvent_t *e = &g_angle_table[i];
        if ((e->valid == 0U) || (e->tooth_index != (uint8_t)snap.tooth_index)) { continue; }
        if ((e->phase_A != ECU_PHASE_ANY) && (e->phase_A != current_phase)) { continue; }
        arm_channel(e->channel, now + ((e->sub_frac_x256 * tooth_ticks) >> 8U), e->action);
    }

    const uint8_t rev_boundary = ((g_hook_prev_valid != 0U) && (snap.tooth_index == 0U) && (g_hook_prev_tooth != 0U)) ? 1U : 0U;
    g_hook_prev_valid = 1U; g_hook_prev_tooth = snap.tooth_index;
    if (rev_boundary == 0U) { return; }
    if ((snap.state == ems::drv::SyncState::HALF_SYNC) && (g_presync_enable != 0U)) { calculate_presync_revolution(snap); return; }
    if (g_hook_schedule_this_gap == 0U) { g_hook_schedule_this_gap = 1U; return; }
    g_hook_schedule_this_gap = 0U;
    Calculate_Sequential_Cycle(snap);
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
    g_angle_table_count = 0U; for (uint8_t i = 0U; i < ECU_ANGLE_TABLE_SIZE; ++i) { g_angle_table[i].valid = 0U; }
    g_ivc_abdc_deg = 50U; g_ivc_clamp_count = 0U;
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
#endif
