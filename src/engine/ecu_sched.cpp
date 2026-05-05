     1|#include "engine/ecu_sched.h"
     2|#include "drv/ckp.h"
     3|#include "engine/engine_config.h"
     4|#include "hal/regs.h"
     5|
     6|#include <stddef.h>
     7|#include <stdint.h>
     8|
     9|#if defined(EMS_HOST_TEST)
    10|static uint32_t ems_test_tim2_cnt;
    11|static uint32_t ems_test_tim2_ccr1;
    12|static uint32_t ems_test_tim2_ccr2;
    13|static uint32_t ems_test_tim2_ccr3;
    14|static uint32_t ems_test_tim2_ccr4;
    15|static uint32_t ems_test_tim2_sr;
    16|static uint32_t ems_test_tim2_cr1;
    17|static uint32_t ems_test_tim2_dier;
    18|static uint32_t ems_test_tim2_psc;
    19|static uint32_t ems_test_tim2_arr;
    20|static uint32_t ems_test_tim2_ccmr1;
    21|static uint32_t ems_test_tim2_ccmr2;
    22|static uint32_t ems_test_tim2_ccer;
    23|static uint32_t ems_test_tim2_egr;
    24|static uint32_t ems_test_tim8_cnt;
    25|static uint32_t ems_test_tim8_ccr1;
    26|static uint32_t ems_test_tim8_ccr2;
    27|static uint32_t ems_test_tim8_ccr3;
    28|static uint32_t ems_test_tim8_ccr4;
    29|static uint32_t ems_test_tim8_sr;
    30|static uint32_t ems_test_tim8_cr1;
    31|static uint32_t ems_test_tim8_dier;
    32|static uint32_t ems_test_tim8_psc;
    33|static uint32_t ems_test_tim8_arr;
    34|static uint32_t ems_test_tim8_ccmr1;
    35|static uint32_t ems_test_tim8_ccmr2;
    36|static uint32_t ems_test_tim8_ccer;
    37|static uint32_t ems_test_tim8_bdtr;
    38|static uint32_t ems_test_tim8_egr;
    39|static uint32_t ems_test_rcc_ahb2enr1;
    40|static uint32_t ems_test_rcc_apb1lenr;
    41|static uint32_t ems_test_rcc_apb2enr;
    42|static uint32_t ems_test_gpio_moder;
    43|static uint32_t ems_test_gpio_afrl;
    44|static uint32_t ems_test_gpio_afrh;
    45|static uint32_t ems_test_gpio_ospeedr;
    46|
    47|#define TIM2_CNT ems_test_tim2_cnt
    48|#define TIM2_CCR1 ems_test_tim2_ccr1
    49|#define TIM2_CCR2 ems_test_tim2_ccr2
    50|#define TIM2_CCR3 ems_test_tim2_ccr3
    51|#define TIM2_CCR4 ems_test_tim2_ccr4
    52|#define TIM2_SR ems_test_tim2_sr
    53|#define TIM2_CR1 ems_test_tim2_cr1
    54|#define TIM2_DIER ems_test_tim2_dier
    55|#define TIM2_PSC ems_test_tim2_psc
    56|#define TIM2_ARR ems_test_tim2_arr
    57|#define TIM2_CCMR1 ems_test_tim2_ccmr1
    58|#define TIM2_CCMR2 ems_test_tim2_ccmr2
    59|#define TIM2_CCER ems_test_tim2_ccer
    60|#define TIM2_EGR ems_test_tim2_egr
    61|#define TIM8_CNT ems_test_tim8_cnt
    62|#define TIM8_CCR1 ems_test_tim8_ccr1
    63|#define TIM8_CCR2 ems_test_tim8_ccr2
    64|#define TIM8_CCR3 ems_test_tim8_ccr3
    65|#define TIM8_CCR4 ems_test_tim8_ccr4
    66|#define TIM8_SR ems_test_tim8_sr
    67|#define TIM8_CR1 ems_test_tim8_cr1
    68|#define TIM8_DIER ems_test_tim8_dier
    69|#define TIM8_PSC ems_test_tim8_psc
    70|#define TIM8_ARR ems_test_tim8_arr
    71|#define TIM8_CCMR1 ems_test_tim8_ccmr1
    72|#define TIM8_CCMR2 ems_test_tim8_ccmr2
    73|#define TIM8_CCER ems_test_tim8_ccer
    74|#define TIM8_BDTR ems_test_tim8_bdtr
    75|#define TIM8_EGR ems_test_tim8_egr
    76|#define RCC_AHB2ENR1 ems_test_rcc_ahb2enr1
    77|#define RCC_APB1LENR ems_test_rcc_apb1lenr
    78|#define RCC_APB2ENR ems_test_rcc_apb2enr
    79|#define GPIOA_MODER ems_test_gpio_moder
    80|#define GPIOA_AFRL ems_test_gpio_afrl
    81|#define GPIOA_AFRH ems_test_gpio_afrh
    82|#define GPIOA_OSPEEDR ems_test_gpio_ospeedr
    83|#define GPIOB_MODER ems_test_gpio_moder
    84|#define GPIOB_AFRL ems_test_gpio_afrl
    85|#define GPIOB_AFRH ems_test_gpio_afrh
    86|#define GPIOB_OSPEEDR ems_test_gpio_ospeedr
    87|#define GPIOC_MODER ems_test_gpio_moder
    88|#define GPIOC_AFRL ems_test_gpio_afrl
    89|#define GPIOC_AFRH ems_test_gpio_afrh
    90|#define GPIOC_OSPEEDR ems_test_gpio_ospeedr
    91|#define RCC_AHB2ENR1_GPIOAEN 1U
    92|#define RCC_AHB2ENR1_GPIOBEN 2U
    93|#define RCC_AHB2ENR1_GPIOCEN 4U
    94|#define RCC_APB1LENR_TIM2EN 1U
    95|#define RCC_APB2ENR_TIM8EN 1U
    96|#define GPIO_AF1 1U
    97|#define GPIO_AF3 3U
    98|#define TIM_SR_CC1IF 0x2U
    99|#define TIM_CR1_CEN 1U
   100|#define TIM_CCMR1_OC1M_ACTIVE 0x10U
   101|#define TIM_CCMR1_OC1M_INACTIVE 0x20U
   102|#define TIM_CCMR1_OC2M_ACTIVE 0x1000U
   103|#define TIM_CCMR1_OC2M_INACTIVE 0x2000U
   104|#define TIM_CCMR2_OC3M_ACTIVE 0x10U
   105|#define TIM_CCMR2_OC3M_INACTIVE 0x20U
   106|#define TIM_CCMR2_OC4M_ACTIVE 0x1000U
   107|#define TIM_CCMR2_OC4M_INACTIVE 0x2000U
   108|#define TIM_CCMR1_OC1M_FORCE_ACTIVE 0x50U
   109|#define TIM_CCMR1_OC1M_FORCE_INACTIVE 0x40U
   110|#define TIM_CCMR1_OC2M_FORCE_ACTIVE 0x5000U
   111|#define TIM_CCMR1_OC2M_FORCE_INACTIVE 0x4000U
   112|#define TIM_CCMR2_OC3M_FORCE_ACTIVE 0x50U
   113|#define TIM_CCMR2_OC3M_FORCE_INACTIVE 0x40U
   114|#define TIM_CCMR2_OC4M_FORCE_ACTIVE 0x5000U
   115|#define TIM_CCMR2_OC4M_FORCE_INACTIVE 0x4000U
   116|#define TIM_CCER_CC1E 1U
   117|#define TIM_CCER_CC2E 0x10U
   118|#define TIM_CCER_CC3E 0x100U
   119|#define TIM_CCER_CC4E 0x1000U
   122|#endif
   123|
   124|#define ECU_CHANNELS      8U
   125|#define ECU_IGN_CH_FIRST  4U
   126|#define ECU_CYCLE_DEG     720U
   127|#define STM32_TIM_PSC_10MHZ 24U
   128|#define STM32_MIN_COMPARE_LEAD_TICKS 20U
   129|#define TOOTH_NS_TO_SCHED(ns) ((uint32_t)((ns) / ECU_SCHED_NS_PER_TICK))
   130|
   131|static AngleEvent_t g_angle_table[ECU_ANGLE_TABLE_SIZE];
   132|static uint8_t g_angle_table_count;
   133|static uint32_t g_angle_tooth_mask_lo;
   134|static uint32_t g_angle_tooth_mask_hi;
   135|
   136|volatile uint32_t g_late_event_count = 0U;
   137|volatile uint32_t g_calibration_clamp_count = 0U;
   138|volatile uint32_t g_cycle_schedule_drop_count = 0U;
   139|
   140|static volatile uint32_t g_advance_deg = 10U;
   141|static volatile uint32_t g_dwell_ticks = 30000U;
   142|static volatile uint32_t g_inj_pw_ticks = 30000U;
   143|static volatile uint32_t g_soi_lead_deg = 62U;
   144|static volatile uint8_t g_presync_enable = 1U;
   145|static volatile uint8_t g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
   146|static volatile uint8_t g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
   147|static volatile uint8_t g_presync_bank_toggle = 0U;
   148|static volatile uint8_t g_hook_prev_valid = 0U;
   149|static volatile uint16_t g_hook_prev_tooth = 0U;
   150|static volatile uint8_t g_hook_schedule_this_gap = 1U;
   151|static volatile uint8_t g_ivc_abdc_deg = ems::engine::cfg::kIvcAbdcDeg;  // FIX: volatile — escrita por ecu_sched_set_ivc (cs), leitura por clamp_inj_pw_to_ivc (contexto ISR)
   152|static uint32_t g_ivc_clamp_count = 0U;
   153|static uint32_t g_oc_mode_shadow[ECU_CHANNELS];
   154|
   155|static inline void enter_critical(void)
   156|{
   157|#if defined(__arm__) || defined(__thumb__)
   158|    __asm__ volatile("cpsid i" ::: "memory");
   159|#endif
   160|}
   161|
   162|static inline void exit_critical(void)
   163|{
   164|#if defined(__arm__) || defined(__thumb__)
   165|    __asm__ volatile("cpsie i" ::: "memory");
   166|#endif
   167|}
   168|
   169|static inline uint32_t scheduler_counter(void)
   170|{
   171|    return TIM2_CNT;
   172|}
   173|
   174|static inline uint8_t stm32_inj_tim_ch(uint8_t ch)
   175|{
   176|    switch (ch) {
   177|        case ECU_CH_INJ1: return 1U;
   178|        case ECU_CH_INJ2: return 2U;
   179|        case ECU_CH_INJ3: return 3U;
   180|        case ECU_CH_INJ4: return 4U;
   181|        default: return 0U;
   182|    }
   183|}
   184|
   185|static inline uint8_t stm32_ign_tim_ch(uint8_t ch)
   186|{
   187|    switch (ch) {
   188|        case ECU_CH_IGN1: return 1U;
   189|        case ECU_CH_IGN2: return 2U;
   190|        case ECU_CH_IGN3: return 3U;
   191|        case ECU_CH_IGN4: return 4U;
   192|        default: return 0U;
   193|    }
   194|}
   195|
   196|static inline volatile uint32_t *stm32_tim_ccr(uint8_t is_inj, uint8_t tim_ch)
   197|{
   198|    if (is_inj != 0U) {
   199|        switch (tim_ch) {
   200|            case 1U: return &TIM2_CCR1;
   201|            case 2U: return &TIM2_CCR2;
   202|            case 3U: return &TIM2_CCR3;
   203|            default: return &TIM2_CCR4;
   204|        }
   205|    }
   206|    switch (tim_ch) {
   207|        case 1U: return &TIM8_CCR1;
   208|        case 2U: return &TIM8_CCR2;
   209|        case 3U: return &TIM8_CCR3;
   210|        default: return &TIM8_CCR4;
   211|    }
   212|}
   213|
   214|static inline uint32_t stm32_tim_cc_flag(uint8_t tim_ch)
   215|{
   216|    return (uint32_t)(TIM_SR_CC1IF << (tim_ch - 1U));
   217|}
   218|
   219|static inline void stm32_write_oc_mode(uint8_t is_inj, uint8_t tim_ch, uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4)
   220|{
   221|    volatile uint32_t *ccmr1 = (is_inj != 0U) ? &TIM2_CCMR1 : &TIM8_CCMR1;
   222|    volatile uint32_t *ccmr2 = (is_inj != 0U) ? &TIM2_CCMR2 : &TIM8_CCMR2;
   223|    const uint8_t shadow_base = (is_inj != 0U) ? 0U : ECU_IGN_CH_FIRST;
   224|    const uint8_t shadow_idx = (uint8_t)(shadow_base + tim_ch - 1U);
   225|    uint32_t desired;
   226|
   227|    switch (tim_ch) {
   228|        case 1U: desired = m1; break;
   229|        case 2U: desired = m2; break;
   230|        case 3U: desired = m3; break;
   231|        default: desired = m4; break;
   232|    }
   233|    if (g_oc_mode_shadow[shadow_idx] == desired) { return; }
   234|    g_oc_mode_shadow[shadow_idx] = desired;
   235|
   236|    switch (tim_ch) {
   237|        case 1U: *ccmr1 = (*ccmr1 & ~0x70U) | desired; break;
   238|        case 2U: *ccmr1 = (*ccmr1 & ~0x7000U) | desired; break;
   239|        case 3U: *ccmr2 = (*ccmr2 & ~0x70U) | desired; break;
   240|        default: *ccmr2 = (*ccmr2 & ~0x7000U) | desired; break;
   241|    }
   242|}
   243|
   244|static inline void stm32_set_oc_mode(uint8_t is_inj, uint8_t tim_ch, uint8_t make_high)
   245|{
   246|    stm32_write_oc_mode(is_inj, tim_ch,
   247|        (make_high != 0U) ? TIM_CCMR1_OC1M_ACTIVE : TIM_CCMR1_OC1M_INACTIVE,
   248|        (make_high != 0U) ? TIM_CCMR1_OC2M_ACTIVE : TIM_CCMR1_OC2M_INACTIVE,
   249|        (make_high != 0U) ? TIM_CCMR2_OC3M_ACTIVE : TIM_CCMR2_OC3M_INACTIVE,
   250|        (make_high != 0U) ? TIM_CCMR2_OC4M_ACTIVE : TIM_CCMR2_OC4M_INACTIVE);
   251|}
   252|
   253|static inline void stm32_force_oc(uint8_t is_inj, uint8_t tim_ch, uint8_t high)
   254|{
   255|    stm32_write_oc_mode(is_inj, tim_ch,
   256|        (high != 0U) ? TIM_CCMR1_OC1M_FORCE_ACTIVE : TIM_CCMR1_OC1M_FORCE_INACTIVE,
   257|        (high != 0U) ? TIM_CCMR1_OC2M_FORCE_ACTIVE : TIM_CCMR1_OC2M_FORCE_INACTIVE,
   258|        (high != 0U) ? TIM_CCMR2_OC3M_FORCE_ACTIVE : TIM_CCMR2_OC3M_FORCE_INACTIVE,
   259|        (high != 0U) ? TIM_CCMR2_OC4M_FORCE_ACTIVE : TIM_CCMR2_OC4M_FORCE_INACTIVE);
   260|}
   261|
   262|static void sanitize_runtime_calibration(void)
   263|{
   264|    uint8_t clamped = 0U;
   265|    if (g_advance_deg > 60U) { g_advance_deg = 60U; clamped = 1U; }
   266|    if (g_dwell_ticks > 100000U) { g_dwell_ticks = 100000U; clamped = 1U; }
   267|    if (g_inj_pw_ticks > 200000U) { g_inj_pw_ticks = 200000U; clamped = 1U; }
   268|    if (g_soi_lead_deg >= ECU_CYCLE_DEG) { g_soi_lead_deg = ECU_CYCLE_DEG - 1U; clamped = 1U; }
   269|    if (g_presync_inj_mode > ECU_PRESYNC_INJ_SEMI_SEQUENTIAL) { g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS; clamped = 1U; }
   270|    if (g_presync_ign_mode > ECU_PRESYNC_IGN_WASTED_SPARK) { g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK; clamped = 1U; }
   271|    if (clamped != 0U) { ++g_calibration_clamp_count; }
   272|}
   273|
   274|static void force_output(uint8_t ch, uint8_t action)
   275|{
   276|    const uint8_t is_inj = (ch < ECU_IGN_CH_FIRST) ? 1U : 0U;
   277|    const uint8_t tim_ch = (is_inj != 0U) ? stm32_inj_tim_ch(ch) : stm32_ign_tim_ch(ch);
   278|    const uint8_t high = ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U;
   279|    if (tim_ch != 0U) { stm32_force_oc(is_inj, tim_ch, high); }
   280|}
   281|
   282|static void arm_channel(uint8_t ch, uint32_t target_cnv, uint8_t action)
   283|{
   284|    // FIX BUG-6: toda a operação (leitura do contador + escrita do CCR) deve ser
   285|    // atômica. Sem seção crítica, TIM2_IRQ poderia disparar entre a leitura de
   286|    // scheduler_counter() e a escrita do registrador de compare, corrompendo o
   287|    // agendamento de eventos já pendentes.
   288|    enter_critical();
   289|    const uint8_t is_inj = (ch < ECU_IGN_CH_FIRST) ? 1U : 0U;
   290|    const uint8_t tim_ch = (is_inj != 0U) ? stm32_inj_tim_ch(ch) : stm32_ign_tim_ch(ch);
   291|    const uint32_t now = scheduler_counter();
   292|    const uint32_t delta = target_cnv - now;
   293|    volatile uint32_t *ccr;
   294|
   295|    if (tim_ch == 0U) { ++g_cycle_schedule_drop_count; exit_critical(); return; }
   296|    if ((is_inj == 0U) && (delta > 0xFFFFU)) { ++g_cycle_schedule_drop_count; exit_critical(); return; }
   297|    if (delta < STM32_MIN_COMPARE_LEAD_TICKS) { ++g_late_event_count; force_output(ch, action); exit_critical(); return; }
   298|
   299|    stm32_set_oc_mode(is_inj, tim_ch, ((action == ECU_ACT_INJ_ON) || (action == ECU_ACT_DWELL_START)) ? 1U : 0U);
   300|    // Clear any pending match flag BEFORE programming CCR to avoid missing an edge
   301|    if (is_inj != 0U) {
   302|        TIM2_SR &= ~stm32_tim_cc_flag(tim_ch);
   303|    } else {
   304|        TIM8_SR &= ~stm32_tim_cc_flag(tim_ch);
   305|    }
   306|#if defined(__arm__) || defined(__thumb__)
   307|    __asm__ volatile("dmb" ::: "memory");  // Ensure previous writes complete before CCR update
   308|#endif
   309|    ccr = stm32_tim_ccr(is_inj, tim_ch);
   310|    if (is_inj != 0U) {
   311|        *ccr = target_cnv;
   312|    } else {
   313|        *ccr = (uint16_t)((TIM8_CNT & 0xFFFFU) + delta);
   314|    }
   315|    exit_critical();
   316|}
   317|
   318|static void clear_all_events_and_drive_safe_outputs(void)
   319|{
   320|    g_angle_table_count = 0U;
   321|    g_angle_tooth_mask_lo = 0U;
   322|    g_angle_tooth_mask_hi = 0U;
   323|    for (uint8_t i = 0U; i < ECU_CHANNELS; ++i) { force_output(i, (i < ECU_IGN_CH_FIRST) ? ECU_ACT_INJ_OFF : ECU_ACT_SPARK); }
   324|}
   325|
   326|static void angle_to_tooth_event(uint32_t angle_deg, uint8_t *out_tooth, uint8_t *out_sub_frac, uint8_t *out_phase_A)
   327|{
   328|    const uint32_t ang = angle_deg % 360U;
   329|    uint32_t pos_x256 = (ang * 256U) / 6U;
   330|    uint8_t tooth = (uint8_t)(pos_x256 >> 8U);
   331|    uint8_t frac = (uint8_t)(pos_x256 & 0xFFU);
   332|    if (tooth > 57U) { tooth = 57U; frac = 255U; }
   333|    *out_phase_A = (angle_deg < 360U) ? ECU_PHASE_A : ECU_PHASE_B;
   334|    *out_tooth = tooth;
   335|    *out_sub_frac = frac;
   336|}
   337|
   338|static uint32_t engine_angle_to_trigger_angle(uint32_t engine_angle_deg, uint32_t cycle_deg)
   339|{
   340|    const uint32_t trigger_offset = ems::engine::cfg::kTriggerTooth0EngineDeg % cycle_deg;
   341|    return (engine_angle_deg + cycle_deg - trigger_offset) % cycle_deg;
   342|}
   343|
   344|static void table_add(uint8_t tooth, uint8_t sub_frac, uint8_t phase_A, uint8_t channel, uint8_t action)
   345|{
   346|    if (g_angle_table_count >= ECU_ANGLE_TABLE_SIZE) { ++g_cycle_schedule_drop_count; return; }
   347|    AngleEvent_t *e = &g_angle_table[g_angle_table_count++];
   348|    e->tooth_index = tooth;
   349|    e->sub_frac_x256 = sub_frac;
   350|    e->phase_A = phase_A;
   351|    e->channel = channel;
   352|    e->action = action;
   353|    e->valid = 1U;
   354|    if (tooth < 32U) {
   355|        g_angle_tooth_mask_lo |= (1UL << tooth);
   356|    } else {
   357|        g_angle_tooth_mask_hi |= (1UL << (tooth - 32U));
   358|    }
   359|}
   360|
   361|void ECU_Hardware_Init(void)
   362|{
   363|    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOBEN | RCC_AHB2ENR1_GPIOCEN;
   364|    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;
   365|    RCC_APB2ENR  |= RCC_APB2ENR_TIM8EN;
   366|
   367|    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 6U, GPIO_AF3);
   368|    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 7U, GPIO_AF3);
   369|    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 8U, GPIO_AF3);
   370|    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 9U, GPIO_AF3);
   371|    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 15U, GPIO_AF1);
   372|    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 3U, GPIO_AF1);
   373|    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 10U, GPIO_AF1);
   374|    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 11U, GPIO_AF1);
   375|
   376|    TIM8_CR1 = 0U; TIM8_DIER = 0U; TIM8_SR = 0U; TIM8_PSC = STM32_TIM_PSC_10MHZ; TIM8_CNT = 0U; TIM8_ARR = 0xFFFFU;
   377|    TIM8_CCMR1 = TIM_CCMR1_OC1M_FORCE_INACTIVE | TIM_CCMR1_OC2M_FORCE_INACTIVE;
   378|    TIM8_CCMR2 = TIM_CCMR2_OC3M_FORCE_INACTIVE | TIM_CCMR2_OC4M_FORCE_INACTIVE;
   379|    TIM8_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; TIM8_BDTR = (1U << 15); TIM8_EGR = 1U;
   380|
   381|    TIM2_CR1 = 0U; TIM2_DIER = 0U; TIM2_SR = 0U; TIM2_PSC = STM32_TIM_PSC_10MHZ; TIM2_CNT = 0U; TIM2_ARR = 0xFFFFFFFFU;
   382|    TIM2_CCMR1 = TIM_CCMR1_OC1M_FORCE_INACTIVE | TIM_CCMR1_OC2M_FORCE_INACTIVE;
   383|    TIM2_CCMR2 = TIM_CCMR2_OC3M_FORCE_INACTIVE | TIM_CCMR2_OC4M_FORCE_INACTIVE;
   384|    TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; TIM2_EGR = 1U;
   385|
   386|    g_oc_mode_shadow[0] = TIM_CCMR1_OC1M_FORCE_INACTIVE;
   387|    g_oc_mode_shadow[1] = TIM_CCMR1_OC2M_FORCE_INACTIVE;
   388|    g_oc_mode_shadow[2] = TIM_CCMR2_OC3M_FORCE_INACTIVE;
   389|    g_oc_mode_shadow[3] = TIM_CCMR2_OC4M_FORCE_INACTIVE;
   390|    g_oc_mode_shadow[4] = TIM_CCMR1_OC1M_FORCE_INACTIVE;
   391|    g_oc_mode_shadow[5] = TIM_CCMR1_OC2M_FORCE_INACTIVE;
   392|    g_oc_mode_shadow[6] = TIM_CCMR2_OC3M_FORCE_INACTIVE;
   393|    g_oc_mode_shadow[7] = TIM_CCMR2_OC4M_FORCE_INACTIVE;
   394|
   395|    TIM8_CR1 = TIM_CR1_CEN;
   396|    TIM2_CR1 = TIM_CR1_CEN;
   397|    clear_all_events_and_drive_safe_outputs();
   398|}
   399|
   400|static uint32_t ticks_to_cycle_degrees(uint32_t ticks, uint32_t tooth_period_ns, uint32_t cycle_deg)
   401|{
   402|    uint32_t tooth_ticks = TOOTH_NS_TO_SCHED(tooth_period_ns);
   403|    uint32_t denom = tooth_ticks * ((cycle_deg == ECU_CYCLE_DEG) ? 120U : 60U);
   404|    return (denom > 0U) ? ((ticks * cycle_deg) / denom) : 0U;
   405|}
   406|
   407|static uint32_t clamp_inj_pw_to_ivc(uint32_t tdc_deg, uint32_t inj_on_deg, uint32_t inj_pw_deg)
   408|{
   409|    const uint32_t ivc_cycle_deg = (tdc_deg + 540U + (uint32_t)g_ivc_abdc_deg) % ECU_CYCLE_DEG;
   410|    const uint32_t soi_to_ivc = (ivc_cycle_deg + ECU_CYCLE_DEG - inj_on_deg) % ECU_CYCLE_DEG;
   411|    if ((soi_to_ivc < (ECU_CYCLE_DEG / 2U)) && (inj_pw_deg > soi_to_ivc)) { ++g_ivc_clamp_count; return soi_to_ivc; }
   412|    return inj_pw_deg;
   413|}
   414|
   415|static void Calculate_Sequential_Cycle(const ems::drv::CkpSnapshot& snap)
   416|{
   417|    static_assert(ems::engine::cfg::kCylinderCount == 4u, "ign_ch/inj_ch hardcoded for 4 cylinders");
   418|    static const uint8_t ign_ch[ems::engine::cfg::kCylinderCount] = {ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};
   419|    static const uint8_t inj_ch[ems::engine::cfg::kCylinderCount] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
   420|
   421|    g_angle_table_count = 0U;
   422|    g_angle_tooth_mask_lo = 0U;
   423|    g_angle_tooth_mask_hi = 0U;
   424|
   425|    const uint32_t dwell_deg = ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, ECU_CYCLE_DEG);
   426|    const uint32_t base_inj_pw_deg = ticks_to_cycle_degrees(g_inj_pw_ticks, snap.tooth_period_ns, ECU_CYCLE_DEG);
   427|
   428|    for (uint8_t seq = 0U; seq < ems::engine::cfg::kCylinderCount; ++seq) {
   429|        const uint8_t cyl = ems::engine::cfg::kFiringOrder[seq];
   430|        const uint32_t tdc = ems::engine::cfg::cyl_tdc_deg(cyl);
   431|        const uint32_t spark = (tdc + ECU_CYCLE_DEG - g_advance_deg) % ECU_CYCLE_DEG;
   432|        const uint32_t dwell = (spark + ECU_CYCLE_DEG - dwell_deg) % ECU_CYCLE_DEG;
   433|        const uint32_t inj_on = (tdc + ECU_CYCLE_DEG - g_soi_lead_deg) % ECU_CYCLE_DEG;
   434|        const uint32_t inj_pw = clamp_inj_pw_to_ivc(tdc, inj_on, base_inj_pw_deg);
   435|        const uint32_t inj_off = (inj_on + inj_pw) % ECU_CYCLE_DEG;
   436|        uint8_t tooth, frac, phase;
   437|
   438|        angle_to_tooth_event(engine_angle_to_trigger_angle(dwell, ECU_CYCLE_DEG), &tooth, &frac, &phase);
   439|        table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_DWELL_START);
   440|        angle_to_tooth_event(engine_angle_to_trigger_angle(spark, ECU_CYCLE_DEG), &tooth, &frac, &phase);
   441|        table_add(tooth, frac, phase, ign_ch[cyl], ECU_ACT_SPARK);
   442|        angle_to_tooth_event(engine_angle_to_trigger_angle(inj_on, ECU_CYCLE_DEG), &tooth, &frac, &phase);
   443|        table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_ON);
   444|        angle_to_tooth_event(engine_angle_to_trigger_angle(inj_off, ECU_CYCLE_DEG), &tooth, &frac, &phase);
   445|        table_add(tooth, frac, phase, inj_ch[cyl], ECU_ACT_INJ_OFF);
   446|    }
   447|}
   448|
   449|static void calculate_presync_revolution(const ems::drv::CkpSnapshot& snap)
   450|{
   451|    static const uint8_t inj_all[4U] = {ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4};
   452|    static const uint8_t inj_a[2U] = {ECU_CH_INJ1, ECU_CH_INJ4};
   453|    static const uint8_t inj_b[2U] = {ECU_CH_INJ2, ECU_CH_INJ3};
   454|    static const uint8_t ign[4U] = {ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4};
   455|    uint8_t tooth, frac, phase;
   456|
   457|    g_angle_table_count = 0U;
   458|    g_angle_tooth_mask_lo = 0U;
   459|    g_angle_tooth_mask_hi = 0U;
   460|
   461|    const uint32_t dwell_deg = ticks_to_cycle_degrees(g_dwell_ticks, snap.tooth_period_ns, 360U);
   462|    const uint32_t inj_pw_deg = ticks_to_cycle_degrees(g_inj_pw_ticks, snap.tooth_period_ns, 360U);
   463|    const uint32_t spark = (360U - (g_advance_deg % 360U)) % 360U;
   464|    const uint32_t dwell = (spark + 360U - dwell_deg) % 360U;
   465|    const uint32_t inj_on = (360U - (g_soi_lead_deg % 360U)) % 360U;
   466|    const uint32_t inj_off = (inj_on + inj_pw_deg) % 360U;
   467|
   468|    angle_to_tooth_event(engine_angle_to_trigger_angle(dwell, 360U), &tooth, &frac, &phase);
   469|    for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_DWELL_START); }
   470|    angle_to_tooth_event(engine_angle_to_trigger_angle(spark, 360U), &tooth, &frac, &phase);
   471|    for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, ign[i], ECU_ACT_SPARK); }
   472|
   473|    angle_to_tooth_event(engine_angle_to_trigger_angle(inj_on, 360U), &tooth, &frac, &phase);
   474|    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
   475|        for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_ON); }
   476|    } else {
   477|        const uint8_t *bank = (g_presync_bank_toggle == 0U) ? inj_a : inj_b;
   478|        for (uint8_t i = 0U; i < 2U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, bank[i], ECU_ACT_INJ_ON); }
   479|        g_presync_bank_toggle ^= 1U;
   480|    }
   481|
   482|    angle_to_tooth_event(engine_angle_to_trigger_angle(inj_off, 360U), &tooth, &frac, &phase);
   483|    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
   484|        for (uint8_t i = 0U; i < 4U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, inj_all[i], ECU_ACT_INJ_OFF); }
   485|    } else {
   486|        /* close the bank that was opened this revolution; toggle already advanced */
   487|        const uint8_t *bank = (g_presync_bank_toggle == 1U) ? inj_a : inj_b;
   488|        for (uint8_t i = 0U; i < 2U; ++i) { table_add(tooth, frac, ECU_PHASE_ANY, bank[i], ECU_ACT_INJ_OFF); }
   489|    }
   490|}
   491|
   492|void ecu_sched_commit_calibration(uint32_t advance_deg, uint32_t dwell_ticks, uint32_t inj_pw_ticks, uint32_t soi_lead_deg)
   493|{
   494|    enter_critical();
   495|    g_advance_deg = advance_deg;
   496|    g_dwell_ticks = dwell_ticks;
   497|    g_inj_pw_ticks = inj_pw_ticks;
   498|    g_soi_lead_deg = soi_lead_deg;
   499|    sanitize_runtime_calibration();
   500|    exit_critical();
   501|