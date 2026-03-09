/**
 * @file engine/ecu_sched.cpp
 * @brief ECU Scheduling Core v3 — Domínio Angular (Angle-Domain Scheduling)
 *
 * Eventos de injeção e ignição são armazenados como (tooth_index, sub_frac_x256,
 * phase_A). Na ISR CKP de cada dente, o offset FTM0 é calculado usando o
 * período do dente ATUAL — correto para o RPM instântaneo, sem queue, sem
 * overflow count, sem retimestamping.
 *
 * Hardware:
 *   FTM0 @ 120 MHz / PS_64 = 1,875 MHz ~ 533,3 ns/tick, free-running 16-bit.
 *   FTM3 @ 120 MHz / PS_2  = 60 MHz  ~ 16,7 ns/tick  (input capture CKP).
 *   PDB0: disparado por FTM0 output trigger (TRGSEL=0x8).
 *   ADC0: hardware averaging 4 amostras.
 *
 * Convenções MISRA-C:2012 (inspired):
 *   - Sem alocação dinâmica.
 *   - Tipos explícitos stdint.h.
 *   - Sem recursão.
 *   - Sufixos U/UL em constantes inteiras.
 */

#include "engine/ecu_sched.h"
#include "drv/ckp.h"

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * Host-test peripheral mocks
 * ========================================================================= */

#if defined(EMS_HOST_TEST)
extern FTM_Type  g_mock_ftm0;
extern PDB_Type  g_mock_pdb0;
extern ADC_Type  g_mock_adc0;
#undef  FTM0
#define FTM0  (&g_mock_ftm0)
#undef  PDB0
#define PDB0  (&g_mock_pdb0)
#undef  ADC0
#define ADC0  (&g_mock_adc0)

static volatile uint32_t g_mock_sim_scgc6;
#undef  SIM_SCGC6_ADDR
#define SIM_SCGC6_REG  g_mock_sim_scgc6
#else
#define SIM_SCGC6_REG  (*((volatile uint32_t *)SIM_SCGC6_ADDR))
#endif /* EMS_HOST_TEST */

/* ============================================================================
 * Internal constants
 * ========================================================================= */

#define ECU_CHANNELS    8U
#define ECU_IGN_CH_FIRST 4U
#define ECU_CYCLE_DEG   720U
#define ECU_NUM_CYL     4U

/* Conversão tooth_period_ns → ticks FTM0 (PS=64: 533,33 ns/tick)
 * tooth_ftm0 = tooth_period_ns / 533,33 ≈ tooth_period_ns * 3 / 1600
 * Análise: max tooth_period_ns @ 29 RPM = ~34 480 000 ns
 *   34 480 000 * 3 / 1600 = 64 650 ticks → cabe em uint16_t (< 65535) ✓
 * Piso operacional seguro: 90 RPM → ~11 111 111 ns → 20 833 ticks (margem 3×) */
#define TOOTH_NS_TO_FTM0(ns)  ((uint32_t)((ns) * 3U) / 1600U)

/* ============================================================================
 * Angle-domain event table (substitui a fila de timestamps)
 * ========================================================================= */

static AngleEvent_t g_angle_table[ECU_ANGLE_TABLE_SIZE];
static uint8_t      g_angle_table_count;

/* ============================================================================
 * Module globals (exported via header)
 * ========================================================================= */

volatile uint32_t g_late_event_count        = 0U;
volatile uint32_t g_cycle_schedule_drop_count = 0U;
volatile uint32_t g_calibration_clamp_count  = 0U;

/* Prime pulse OFF tracking (set by ecu_sched_fire_prime_pulse, consumed by FTM0 ISR) */
static volatile uint16_t g_prime_off_cnv[4U]; /*!< OFF target CnV por canal injetor (0-3) */
static volatile uint8_t  g_prime_off_pending;  /*!< Bitmask: bit i = canal i aguarda OFF   */

/* ============================================================================
 * Module-private configuration
 * ========================================================================= */

static volatile uint32_t g_advance_deg      = 10U;
static volatile uint32_t g_dwell_ticks      = 5625U;  /* ~3 ms @ PS=64 (1875 tick/ms) */
static volatile uint32_t g_inj_pw_ticks     = 5625U;  /* ~3 ms @ PS=64 */
static volatile uint32_t g_soi_lead_deg     = 62U;
static volatile uint8_t  g_presync_enable   = 1U;
static volatile uint8_t  g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
static volatile uint8_t  g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
static volatile uint8_t  g_presync_bank_toggle = 0U;
static volatile uint8_t  g_hook_prev_valid  = 0U;
static uint8_t  g_ivc_abdc_deg   = 50U;   /* IVC em graus ABDC; padrão: 50° */
static uint32_t g_ivc_clamp_count = 0U;   /* eventos onde EOI foi clampeado ao IVC */
static volatile uint16_t g_hook_prev_tooth  = 0U;
static volatile uint8_t  g_hook_schedule_this_gap = 1U;

/* ============================================================================
 * Forward declarations
 * ========================================================================= */

static void calculate_presync_revolution(const ems::drv::CkpSnapshot& snap);

/* ============================================================================
 * Critical section helpers
 * ========================================================================= */

static void enter_critical(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsid i" ::: "memory");
#endif
}

static void exit_critical(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsie i" ::: "memory");
#endif
}

/* ============================================================================
 * sanitize_runtime_calibration
 * Limites em domínio angular (PS=64, 1 875 ticks/ms):
 *   dwell   : máx 18 750 ticks (10 ms) — bobina nunca deve carregar mais que isso
 *   inj_pw  : máx 37 500 ticks (20 ms) — PW máxima razoável em marcha lenta
 *   advance : máx 60° BTDC
 *   soi_lead: máx 359°
 * ========================================================================= */

static void sanitize_runtime_calibration(void)
{
    static const uint32_t kMaxDwellTicks  = 18750U;   /* 10 ms × 1875 ticks/ms */
    static const uint32_t kMaxInjPwTicks  = 37500U;   /* 20 ms × 1875 ticks/ms */
    static const uint32_t kMinPulseTicks  = 1U;
    static const uint32_t kMaxAdvanceDeg  = 60U;
    uint8_t clamped = 0U;

    if (g_advance_deg > kMaxAdvanceDeg) {
        g_advance_deg = kMaxAdvanceDeg;
        clamped = 1U;
    }

    if ((g_dwell_ticks > 0U) && (g_dwell_ticks < kMinPulseTicks)) {
        g_dwell_ticks = kMinPulseTicks;
        clamped = 1U;
    } else if (g_dwell_ticks > kMaxDwellTicks) {
        g_dwell_ticks = kMaxDwellTicks;
        clamped = 1U;
    }

    if ((g_inj_pw_ticks > 0U) && (g_inj_pw_ticks < kMinPulseTicks)) {
        g_inj_pw_ticks = kMinPulseTicks;
        clamped = 1U;
    } else if (g_inj_pw_ticks > kMaxInjPwTicks) {
        g_inj_pw_ticks = kMaxInjPwTicks;
        clamped = 1U;
    }

    if (g_soi_lead_deg >= ECU_CYCLE_DEG) {
        g_soi_lead_deg = ECU_CYCLE_DEG - 1U;
        clamped = 1U;
    }

    if (g_presync_inj_mode > ECU_PRESYNC_INJ_SEMI_SEQUENTIAL) {
        g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
        clamped = 1U;
    }
    if (g_presync_ign_mode > ECU_PRESYNC_IGN_WASTED_SPARK) {
        g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
        clamped = 1U;
    }

    if (clamped != 0U) {
        ++g_calibration_clamp_count;
    }
}

/* ============================================================================
 * force_output: força saída de canal via output compare de zero atraso.
 * Usado para colocar canais em estado seguro ao perder sincronia.
 * ========================================================================= */

static void force_output(uint8_t ch, uint8_t action)
{
    uint32_t cnsc_val;
    if ((action == ECU_ACT_SPARK) || (action == ECU_ACT_INJ_OFF)) {
        cnsc_val = FTM_CnSC_OC_CLEAR;
    } else {
        cnsc_val = FTM_CnSC_OC_SET;
    }
    /* FIX-11: CnSC antes de CnV — ação configurada antes do disparo do comparador. */
    FTM0->CH[ch].CnSC = cnsc_val;
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("dmb" ::: "memory");
#endif
    FTM0->CH[ch].CnV = (uint32_t)((FTM0->CNT + 2U) & 0xFFFFU);
}

/* ============================================================================
 * arm_channel: arma canal FTM0 para disparar em target_cnv (16 bits).
 * O hardware aciona o pino autonomamente quando FTM0->CNT == CnV.
 * ========================================================================= */

static void arm_channel(uint8_t ch, uint16_t target_cnv, uint8_t action)
{
    uint32_t cnsc_val;
    if ((action == ECU_ACT_SPARK) || (action == ECU_ACT_INJ_OFF)) {
        cnsc_val = FTM_CnSC_OC_CLEAR;
    } else {
        cnsc_val = FTM_CnSC_OC_SET;
    }
    /* FIX-11: CnSC antes de CnV. */
    FTM0->CH[ch].CnSC = cnsc_val;
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("dmb" ::: "memory");
#endif
    FTM0->CH[ch].CnV = (uint32_t)target_cnv;
}

/* ============================================================================
 * clear_all_events_and_drive_safe_outputs
 * Chamada em LOSS_OF_SYNC / WAIT_GAP: zera tabela angular e coloca pinos
 * em estado seguro (injetores fechados, bobinas de-energizadas).
 * ========================================================================= */

static void clear_all_events_and_drive_safe_outputs(void)
{
    uint8_t i;

    /* Zera tabela angular */
    for (i = 0U; i < ECU_ANGLE_TABLE_SIZE; ++i) {
        g_angle_table[i].valid = 0U;
    }
    g_angle_table_count = 0U;

    /* Estado seguro: injetores fechados (LOW), bobinas de-energizadas (LOW) */
    for (i = 0U; i < ECU_CHANNELS; ++i) {
        uint8_t safe_action = (i < ECU_IGN_CH_FIRST) ? ECU_ACT_INJ_OFF : ECU_ACT_SPARK;
        force_output(i, safe_action);
    }
}

/* ============================================================================
 * angle_to_tooth_event
 * Converte ângulo no ciclo de 720° para (tooth_index, sub_frac_x256, phase_A).
 *
 * Mapeamento:
 *   0–359°   → phase_A = ECU_PHASE_A (primeira revolução)
 *   360–719° → phase_A = ECU_PHASE_B (segunda revolução)
 *
 * Dentro de cada revolução (0–359°):
 *   60 posições de 6° cada (roda fônica 60-2)
 *   pos_x256 = (angle_in_rev * 256) / 6
 *   tooth    = pos_x256 >> 8           (0–59, clamped a 57 — dentes 58-59 ausentes)
 *   sub_frac = pos_x256 & 0xFF         (fração × 256 do período do dente)
 * ========================================================================= */

static void angle_to_tooth_event(uint32_t angle_deg,
                                  uint8_t *out_tooth,
                                  uint8_t *out_sub_frac,
                                  uint8_t *out_phase_A)
{
    uint32_t ang;
    uint32_t pos_x256;
    uint8_t  tooth;
    uint8_t  sub_frac;

    *out_phase_A = (angle_deg < 360U) ? ECU_PHASE_A : ECU_PHASE_B;
    ang = angle_deg % 360U;

    /* 60 posições × 6°/posição: pos = ang / 6, frac = (ang % 6) / 6 em Q8 */
    pos_x256 = (ang * 256U) / 6U;
    tooth    = (uint8_t)(pos_x256 >> 8U);
    sub_frac = (uint8_t)(pos_x256 & 0xFFU);

    /* Clamp: posições 58 e 59 são os dentes ausentes (gap).
     * Eventos nessa região são atribuídos ao último dente real (57). */
    if (tooth > 57U) {
        tooth    = 57U;
        sub_frac = 255U;
    }

    *out_tooth    = tooth;
    *out_sub_frac = sub_frac;
}

/* ============================================================================
 * table_add: insere evento na tabela angular.
 * Se a tabela estiver cheia, incrementa g_cycle_schedule_drop_count e ignora.
 * ========================================================================= */

static void table_add(uint8_t tooth, uint8_t sub_frac, uint8_t phase_A,
                       uint8_t channel, uint8_t action)
{
    AngleEvent_t *e;

    if (g_angle_table_count >= ECU_ANGLE_TABLE_SIZE) {
        ++g_cycle_schedule_drop_count;
        return;
    }
    e               = &g_angle_table[g_angle_table_count];
    e->tooth_index  = tooth;
    e->sub_frac_x256 = sub_frac;
    e->phase_A      = phase_A;
    e->channel      = channel;
    e->action       = action;
    e->valid        = 1U;
    ++g_angle_table_count;
}

/* ============================================================================
 * ECU_Hardware_Init
 * ========================================================================= */

void ECU_Hardware_Init(void)
{
    uint8_t ch;

    /* 1. Clock gating: FTM0 + PDB0 + ADC0 via SIM_SCGC6 */
    SIM_SCGC6_REG |= (SIM_SCGC6_FTM0_MASK | SIM_SCGC6_PDB_MASK | SIM_SCGC6_ADC0_MASK);

    /* 2. FTM0 configuration */
    /* 2a. Write-protect disable + FTMEN (deve ser a primeira escrita) */
    FTM0->MODE = (FTM_MODE_WPDIS | FTM_MODE_FTMEN);

    /* 2b. Para o contador para configuração segura */
    FTM0->SC = 0U;

    /* 2c. Free-running, MOD = 0xFFFF (range máximo 16-bit) */
    FTM0->CNT = 0U;
    FTM0->MOD = 0xFFFFU;

    /* 2d. CH0-CH3 (injetores): Output Compare, Set on match (HIGH = aberto) */
    for (ch = 0U; ch < ECU_IGN_CH_FIRST; ++ch) {
        FTM0->CH[ch].CnSC = FTM_CnSC_OC_SET;
        FTM0->CH[ch].CnV  = 0U;
    }

    /* 2e. CH4-CH7 (bobinas ignição): Output Compare, Clear on match (LOW = faísca) */
    for (ch = ECU_IGN_CH_FIRST; ch < ECU_CHANNELS; ++ch) {
        FTM0->CH[ch].CnSC = FTM_CnSC_OC_CLEAR;
        FTM0->CH[ch].CnV  = 0U;
    }

    /* 2f. Inicia FTM0: system clock, PS=64.
     * Sem TOIE — no domínio angular, overflow do FTM0 é irrelevante.
     * Cada CnV é programado com offset relativo ao CNT atual (< 1 tooth period),
     * portanto nunca excede 65535 ticks para RPM >= 29 (piso operacional: 90 RPM). */
    FTM0->SC = (FTM_SC_CLKS_SYSTEM | FTM_SC_PS_64);

    /* 3. PDB0: trigger source = FTM0 output trigger (TRGSEL=0x8), CH0 para ADC0 */
    PDB0->SC      = 0U;
    PDB0->IDLY    = 0U;
    PDB0->MOD     = 0xFFFFU;
    PDB0->CH0C1   = PDB_CHnC1_EN0_MASK;
    PDB0->CH0DLY0 = 0U;
    PDB0->SC      = (PDB_SC_PDBEN_MASK | PDB_SC_TRGSEL_FTM0 | PDB_SC_LDOK_MASK);

    /* 4. ADC0: 12-bit, bus clock / 2; hardware averaging 4 samples */
    ADC0->CFG1 = ADC_CFG1_12B_DIV2;
    ADC0->CFG2 = 0U;
    ADC0->SC3  = ADC_SC3_AVG4;
}

/* ============================================================================
 * FTM0_IRQHandler
 * Simplificado: sem bloco TOF (g_overflow_count eliminado).
 * Apenas limpa CHF por canal — o pino já mudou em hardware via output compare.
 * ========================================================================= */

void FTM0_IRQHandler(void)
{
    uint8_t ch;

    for (ch = 0U; ch < ECU_CHANNELS; ++ch) {
        uint32_t cnsc = FTM0->CH[ch].CnSC;
        if ((cnsc & FTM_CnSC_CHF_MASK) != 0U) {
            /* Canais de injeção (0-3): checar prime pulse OFF pendente.
             * Se pendente, re-armar em CLEAR mode — isso limpa CHF implicitamente
             * (bit 7 ausente em FTM_CnSC_OC_CLEAR; W0C se escreve 0). */
            if (ch < ECU_IGN_CH_FIRST) {
                const uint8_t bit = (uint8_t)(1U << ch);
                if ((g_prime_off_pending & bit) != 0U) {
                    FTM0->CH[ch].CnSC = FTM_CnSC_OC_CLEAR;
                    FTM0->CH[ch].CnV  = (uint32_t)g_prime_off_cnv[ch];
                    g_prime_off_pending = (uint8_t)(g_prime_off_pending & (uint8_t)(~bit));
                    continue;
                }
            }
            /* Normal: limpa CHF (W0C: escreve 0 no bit, preserva os demais) */
            FTM0->CH[ch].CnSC = (cnsc & ~FTM_CnSC_CHF_MASK);
        }
    }
}

/* ============================================================================
 * Calculate_Sequential_Cycle (domínio angular)
 * Chamada uma vez por revolução (tooth_index==0, FULL_SYNC).
 * Preenche g_angle_table[] com 16 eventos (4 cil × 4 ações).
 * ========================================================================= */

static void Calculate_Sequential_Cycle(const ems::drv::CkpSnapshot& snap)
{
    /* Firing order: 1-3-4-2 (cylinder 0-indexed: 0,2,3,1) */
    static const uint8_t  k_fire_order[ECU_NUM_CYL] = {0U, 2U, 3U, 1U};
    /* TDC compression angles (degrees in 720° cycle) */
    static const uint32_t k_tdc_deg[ECU_NUM_CYL]    = {0U, 180U, 360U, 540U};
    /* Ignition channel per cylinder (0-indexed) */
    static const uint8_t  k_ign_ch[ECU_NUM_CYL] = {
        ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4
    };
    /* Injection channel per cylinder (0-indexed) */
    static const uint8_t  k_inj_ch[ECU_NUM_CYL] = {
        ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4
    };

    uint32_t seq;
    uint32_t advance_deg  = g_advance_deg;
    uint32_t dwell_ticks  = g_dwell_ticks;
    uint32_t inj_pw_ticks = g_inj_pw_ticks;
    uint32_t soi_lead_deg = g_soi_lead_deg;

    /* Converte dwell e PW (tempo em ticks FTM0) para ângulo no ciclo de 720°.
     * tooth_period_ftm0 = tooth_period_ns / 533,33 ≈ tooth_period_ns * 3 / 1600
     * ticks_per_rev_720 = tooth_period_ftm0 * 60 posições × 2 revoluções
     * dwell_deg = dwell_ticks * 720 / ticks_per_rev_720 */
    uint32_t tooth_ftm0      = TOOTH_NS_TO_FTM0(snap.tooth_period_ns);
    uint32_t ticks_per_rev720 = tooth_ftm0 * 120U; /* 60 posições × 2 revoluções */

    uint32_t dwell_deg  = 0U;
    uint32_t inj_pw_deg = 0U;
    if (ticks_per_rev720 > 0U) {
        dwell_deg  = (dwell_ticks  * ECU_CYCLE_DEG) / ticks_per_rev720;
        inj_pw_deg = (inj_pw_ticks * ECU_CYCLE_DEG) / ticks_per_rev720;
    }

    g_angle_table_count = 0U;

    for (seq = 0U; seq < ECU_NUM_CYL; ++seq) {
        uint8_t  cyl     = k_fire_order[seq];
        uint32_t tdc_deg = k_tdc_deg[seq];
        uint8_t  ign_ch  = k_ign_ch[cyl];
        uint8_t  inj_ch  = k_inj_ch[cyl];
        uint8_t  tooth, sf, phase;

        /* Ângulos no ciclo de 720°, todos positivos (mod 720) */
        uint32_t spark_deg   = (tdc_deg + ECU_CYCLE_DEG - advance_deg)  % ECU_CYCLE_DEG;
        uint32_t dwell_start = (spark_deg + ECU_CYCLE_DEG - dwell_deg)  % ECU_CYCLE_DEG;
        uint32_t inj_on_deg  = (tdc_deg + ECU_CYCLE_DEG - soi_lead_deg) % ECU_CYCLE_DEG;

        /* IVC constraint — limita EOI ao ângulo de fechamento da válvula de admissão.
         * IVC no ciclo de 720°: tdc_deg + 540° (BDC admissão) + ivc_abdc_deg.
         * Clamp ativo APENAS em open-valve injection (SOI antes do IVC):
         *   soi_to_ivc < 360° implica que o IVC está na janela de admissão à frente do SOI.
         * Em closed-valve injection (padrão, soi_lead_deg=62), soi_to_ivc ≥ 360° → sem clamp. */
        uint32_t eff_inj_pw_deg = inj_pw_deg;
        {
            const uint32_t ivc_cycle_deg =
                (tdc_deg + 540U + (uint32_t)g_ivc_abdc_deg) % ECU_CYCLE_DEG;
            const uint32_t soi_to_ivc =
                (ivc_cycle_deg + ECU_CYCLE_DEG - inj_on_deg) % ECU_CYCLE_DEG;
            if ((soi_to_ivc < (ECU_CYCLE_DEG / 2U)) && (eff_inj_pw_deg > soi_to_ivc)) {
                eff_inj_pw_deg = soi_to_ivc;
                ++g_ivc_clamp_count;
            }
        }

        uint32_t inj_off_deg = (inj_on_deg + eff_inj_pw_deg)            % ECU_CYCLE_DEG;

        /* Converte para (tooth, sub_frac, phase_A) e insere na tabela */
        angle_to_tooth_event(dwell_start, &tooth, &sf, &phase);
        table_add(tooth, sf, phase, ign_ch, ECU_ACT_DWELL_START);

        angle_to_tooth_event(spark_deg, &tooth, &sf, &phase);
        table_add(tooth, sf, phase, ign_ch, ECU_ACT_SPARK);

        if (inj_pw_ticks > 0U) {
            angle_to_tooth_event(inj_on_deg, &tooth, &sf, &phase);
            table_add(tooth, sf, phase, inj_ch, ECU_ACT_INJ_ON);

            angle_to_tooth_event(inj_off_deg, &tooth, &sf, &phase);
            table_add(tooth, sf, phase, inj_ch, ECU_ACT_INJ_OFF);
        }
    }
}

/* ============================================================================
 * calculate_presync_revolution (domínio angular, HALF_SYNC)
 * Wasted spark em toda revolução (phase_A = ECU_PHASE_ANY).
 * Injeção simultânea ou semi-sequencial, também em toda revolução.
 * ========================================================================= */

static void calculate_presync_revolution(const ems::drv::CkpSnapshot& snap)
{
    static const uint8_t k_ign_pair_a[2] = {ECU_CH_IGN1, ECU_CH_IGN4}; /* cil 1+4 */
    static const uint8_t k_ign_pair_b[2] = {ECU_CH_IGN2, ECU_CH_IGN3}; /* cil 2+3 */
    static const uint8_t k_inj_bank_a[2] = {ECU_CH_INJ1, ECU_CH_INJ4};
    static const uint8_t k_inj_bank_b[2] = {ECU_CH_INJ2, ECU_CH_INJ3};

    uint32_t advance_deg  = g_advance_deg % 360U;
    uint32_t dwell_ticks  = g_dwell_ticks;
    uint32_t inj_pw_ticks = g_inj_pw_ticks;
    uint32_t soi_lead_deg = g_soi_lead_deg % 360U;

    /* Converte dwell para ângulo (domínio 360°, uma revolução) */
    uint32_t tooth_ftm0      = TOOTH_NS_TO_FTM0(snap.tooth_period_ns);
    uint32_t ticks_per_rev360 = tooth_ftm0 * 60U;
    uint32_t dwell_deg  = 0U;
    uint32_t inj_pw_deg = 0U;
    if (ticks_per_rev360 > 0U) {
        dwell_deg  = (dwell_ticks  * 360U) / ticks_per_rev360;
        inj_pw_deg = (inj_pw_ticks * 360U) / ticks_per_rev360;
    }

    /* Ângulos no ciclo de 360° (HALF_SYNC não conhece a fase) */
    uint32_t spark_a = (360U - advance_deg) % 360U;
    uint32_t spark_b = (180U + 360U - advance_deg) % 360U;
    uint32_t dwell_a = (spark_a + 360U - dwell_deg) % 360U;
    uint32_t dwell_b = (spark_b + 360U - dwell_deg) % 360U;

    g_angle_table_count = 0U;

    /* Wasted spark: dispara em toda revolução (ECU_PHASE_ANY) */
    {
        uint8_t p, tooth, sf, phase_ignored;

        /* Par A (cil 1+4) */
        angle_to_tooth_event(dwell_a, &tooth, &sf, &phase_ignored);
        for (p = 0U; p < 2U; ++p) {
            table_add(tooth, sf, ECU_PHASE_ANY, k_ign_pair_a[p], ECU_ACT_DWELL_START);
        }
        angle_to_tooth_event(spark_a, &tooth, &sf, &phase_ignored);
        for (p = 0U; p < 2U; ++p) {
            table_add(tooth, sf, ECU_PHASE_ANY, k_ign_pair_a[p], ECU_ACT_SPARK);
        }

        /* Par B (cil 2+3) */
        angle_to_tooth_event(dwell_b, &tooth, &sf, &phase_ignored);
        for (p = 0U; p < 2U; ++p) {
            table_add(tooth, sf, ECU_PHASE_ANY, k_ign_pair_b[p], ECU_ACT_DWELL_START);
        }
        angle_to_tooth_event(spark_b, &tooth, &sf, &phase_ignored);
        for (p = 0U; p < 2U; ++p) {
            table_add(tooth, sf, ECU_PHASE_ANY, k_ign_pair_b[p], ECU_ACT_SPARK);
        }
    }

    if (inj_pw_ticks == 0U) {
        return;
    }

    /* Injeção presync */
    {
        uint32_t soi    = (360U - soi_lead_deg) % 360U;

        /* IVC constraint (domínio 360°) — mesmo critério da função sequencial.
         * IVC presync: usa tdc=0 como referência (360° de 720° equivalem a 180°+offset).
         * Simplificação: IVC em 360° = (180U + ivc_abdc_deg) % 360U
         * Guarda: soi_to_ivc < 180° → open-valve → clamp ativo. */
        uint32_t eff_inj_pw_deg = inj_pw_deg;
        {
            const uint32_t ivc_360 = (180U + (uint32_t)g_ivc_abdc_deg) % 360U;
            const uint32_t soi_to_ivc = (ivc_360 + 360U - soi) % 360U;
            if ((soi_to_ivc < 180U) && (eff_inj_pw_deg > soi_to_ivc)) {
                eff_inj_pw_deg = soi_to_ivc;
                ++g_ivc_clamp_count;
            }
        }

        uint32_t inj_off_deg = (soi + eff_inj_pw_deg) % 360U;
        uint8_t  tooth_on, sf_on, tooth_off, sf_off, phase_ignored;

        angle_to_tooth_event(soi,         &tooth_on,  &sf_on,  &phase_ignored);
        angle_to_tooth_event(inj_off_deg, &tooth_off, &sf_off, &phase_ignored);

        if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
            uint8_t ch;
            static const uint8_t k_all_inj[4] = {
                ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4
            };
            for (ch = 0U; ch < 4U; ++ch) {
                table_add(tooth_on,  sf_on,  ECU_PHASE_ANY, k_all_inj[ch], ECU_ACT_INJ_ON);
                table_add(tooth_off, sf_off, ECU_PHASE_ANY, k_all_inj[ch], ECU_ACT_INJ_OFF);
            }
        } else {
            /* Semi-sequencial: alterna bancos a cada revolução */
            const uint8_t *bank = (g_presync_bank_toggle == 0U)
                                  ? k_inj_bank_a : k_inj_bank_b;
            table_add(tooth_on,  sf_on,  ECU_PHASE_ANY, bank[0], ECU_ACT_INJ_ON);
            table_add(tooth_on,  sf_on,  ECU_PHASE_ANY, bank[1], ECU_ACT_INJ_ON);
            table_add(tooth_off, sf_off, ECU_PHASE_ANY, bank[0], ECU_ACT_INJ_OFF);
            table_add(tooth_off, sf_off, ECU_PHASE_ANY, bank[1], ECU_ACT_INJ_OFF);
            g_presync_bank_toggle ^= 1U;
        }
    }
}

/* ============================================================================
 * ecu_sched_commit_calibration
 * ========================================================================= */

void ecu_sched_commit_calibration(uint32_t advance_deg,
                                  uint32_t dwell_ticks,
                                  uint32_t inj_pw_ticks,
                                  uint32_t soi_lead_deg)
{
    enter_critical();
    g_advance_deg  = advance_deg;
    g_dwell_ticks  = dwell_ticks;
    g_inj_pw_ticks = inj_pw_ticks;
    g_soi_lead_deg = soi_lead_deg;
    sanitize_runtime_calibration();
    exit_critical();
}

/* ============================================================================
 * Individual setters (mantidos para compatibilidade com callers existentes)
 * ========================================================================= */

void ecu_sched_set_advance_deg(uint32_t adv)
{
    ecu_sched_commit_calibration(adv, g_dwell_ticks, g_inj_pw_ticks, g_soi_lead_deg);
}

void ecu_sched_set_dwell_ticks(uint32_t dwell)
{
    ecu_sched_commit_calibration(g_advance_deg, dwell, g_inj_pw_ticks, g_soi_lead_deg);
}

void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks)
{
    ecu_sched_commit_calibration(g_advance_deg, g_dwell_ticks, pw_ticks, g_soi_lead_deg);
}

void ecu_sched_set_soi_lead_deg(uint32_t soi_lead_deg)
{
    ecu_sched_commit_calibration(g_advance_deg, g_dwell_ticks, g_inj_pw_ticks, soi_lead_deg);
}

void ecu_sched_set_presync_enable(uint8_t enable)
{
    enter_critical();
    g_presync_enable = (enable != 0U) ? 1U : 0U;
    exit_critical();
}

void ecu_sched_set_presync_inj_mode(uint8_t mode)
{
    enter_critical();
    g_presync_inj_mode = mode;
    sanitize_runtime_calibration();
    exit_critical();
}

void ecu_sched_set_presync_ign_mode(uint8_t mode)
{
    enter_critical();
    g_presync_ign_mode = mode;
    sanitize_runtime_calibration();
    exit_critical();
}

void ecu_sched_reset_diagnostic_counters(void)
{
    enter_critical();
    g_calibration_clamp_count   = 0U;
    g_late_event_count          = 0U;
    g_cycle_schedule_drop_count = 0U;
    exit_critical();
}

void ecu_sched_set_ivc(uint8_t ivc_abdc_deg)
{
    enter_critical();
    g_ivc_abdc_deg = ivc_abdc_deg;
    exit_critical();
}

uint32_t ecu_sched_ivc_clamp_count(void)
{
    return g_ivc_clamp_count;
}

void ecu_sched_fire_prime_pulse(uint32_t pw_us)
{
    static const uint32_t kPrimePwMaxUs  = 30000U;
    static const uint8_t  kInjChannels[] = {
        ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4  /* CH2,CH3,CH0,CH1 */
    };
    uint8_t i;

    if (pw_us == 0U) { return; }
    if (pw_us > kPrimePwMaxUs) { pw_us = kPrimePwMaxUs; }

    const uint32_t pw_ticks = (pw_us * ECU_FTM0_TICKS_PER_MS) / 1000U;
    const uint16_t on_cnv   = (uint16_t)((FTM0->CNT + 20U) & 0xFFFFU);
    const uint16_t off_cnv  = (uint16_t)((on_cnv + pw_ticks) & 0xFFFFU);

    enter_critical();

    /* Prepara OFF targets indexados pelo número de canal FTM0 (0-3) */
    for (i = 0U; i < 4U; ++i) {
        g_prime_off_cnv[kInjChannels[i]] = off_cnv;
    }
    g_prime_off_pending = 0x0FU;  /* CH0,CH1,CH2,CH3 aguardam OFF */

    /* Arma todos os injetores para abrir simultaneamente */
    for (i = 0U; i < 4U; ++i) {
        arm_channel(kInjChannels[i], on_cnv, ECU_ACT_INJ_ON);
    }

    exit_critical();
}

/* ============================================================================
 * ecu_sched_on_tooth_hook (domínio angular)
 * Chamada pela ISR CKP (FTM3, prioridade 1) a cada dente detectado.
 *
 * Operações por dente:
 *   1. Filtro de estado (só processa em HALF_SYNC ou FULL_SYNC)
 *   2. Loop na tabela angular: para cada evento com tooth_index == snap.tooth_index
 *      e phase compatível, calcula offset e arma canal FTM0
 *   3. Detecção de boundary de revolução (tooth_index == 0)
 *   4. No boundary: recalcula tabela para próximo ciclo
 * ========================================================================= */

namespace ems::engine {

void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept
{
    uint8_t i;

    /* ── 1. Filtro de estado ─────────────────────────────────────────────── */
    if ((snap.state != ems::drv::SyncState::FULL_SYNC) &&
        (snap.state != ems::drv::SyncState::HALF_SYNC)) {
        if (g_hook_prev_valid != 0U) {
            clear_all_events_and_drive_safe_outputs();
        }
        g_hook_prev_valid = 0U;
        g_hook_prev_tooth = 0U;
        g_hook_schedule_this_gap = 1U;
        return;
    }

    /* ── 2. Disparo de eventos que correspondem ao dente atual ───────────── */
    /* Calcula período do dente em ticks FTM0 (PS=64: 533,33 ns/tick)
     * tooth_ftm0 = tooth_period_ns * 3 / 1600
     * Clamp a 0xFFFF para cranking muito lento (< ~29 RPM): aceita pequeno
     * erro de timing nessa condição extrema em vez de overflow. */
    uint32_t tooth_ftm0 = TOOTH_NS_TO_FTM0(snap.tooth_period_ns);
    if (tooth_ftm0 > 0xFFFFU) { tooth_ftm0 = 0xFFFFU; }

    uint16_t ftm0_now = (uint16_t)(FTM0->CNT & 0xFFFFU);

    uint8_t current_phase = snap.phase_A ? ECU_PHASE_A : ECU_PHASE_B;

    for (i = 0U; i < g_angle_table_count; ++i) {
        const AngleEvent_t *e = &g_angle_table[i];
        if (e->valid == 0U) { continue; }
        if (e->tooth_index != (uint8_t)snap.tooth_index) { continue; }
        /* ECU_PHASE_ANY dispara em toda revolução (HALF_SYNC wasted spark) */
        if ((e->phase_A != ECU_PHASE_ANY) && (e->phase_A != current_phase)) { continue; }

        uint16_t offset = (uint16_t)((e->sub_frac_x256 * tooth_ftm0) >> 8U);
        uint16_t target = (uint16_t)(ftm0_now + offset);
        arm_channel(e->channel, target, e->action);
    }

    /* ── 3. Detecção de boundary de revolução ────────────────────────────── */
    /* SCH-02: detecta wrap genuíno (tooth_index 57→0) descartando o primeiro
     * tooth_index=0 após resync (g_hook_prev_valid==0). */
    uint8_t rev_boundary = 0U;
    if ((g_hook_prev_valid != 0U) &&
        (snap.tooth_index == 0U) &&
        (g_hook_prev_tooth != 0U)) {
        rev_boundary = 1U;
    }
    g_hook_prev_valid = 1U;
    g_hook_prev_tooth = snap.tooth_index;

    if (rev_boundary == 0U) { return; }

    /* ── 4. Boundary: recalcula tabela angular para próxima revolução ───── */
    if ((snap.state == ems::drv::SyncState::HALF_SYNC) &&
        (g_presync_enable != 0U)) {
        calculate_presync_revolution(snap);
        return;
    }

    /* SCH-02 guard: descarta o primeiro ciclo após FULL_SYNC para evitar
     * disparar com parâmetros potencialmente desatualizados. */
    if (g_hook_schedule_this_gap == 0U) {
        g_hook_schedule_this_gap = 1U;
        return;
    }
    g_hook_schedule_this_gap = 0U;

    Calculate_Sequential_Cycle(snap);
}

}  // namespace ems::engine

/* ============================================================================
 * Test-only API
 * ========================================================================= */

#if defined(EMS_HOST_TEST)

void ecu_sched_test_reset(void)
{
    uint8_t i;

    g_late_event_count          = 0U;
    g_cycle_schedule_drop_count = 0U;
    g_calibration_clamp_count   = 0U;
    g_presync_enable            = 1U;
    g_presync_inj_mode          = ECU_PRESYNC_INJ_SIMULTANEOUS;
    g_presync_ign_mode          = ECU_PRESYNC_IGN_WASTED_SPARK;
    g_presync_bank_toggle       = 0U;
    g_hook_prev_valid           = 0U;
    g_hook_prev_tooth           = 0U;
    g_hook_schedule_this_gap    = 1U;
    g_advance_deg               = 10U;
    g_dwell_ticks               = 22500U;
    g_inj_pw_ticks              = 22500U;
    g_soi_lead_deg              = 62U;
    g_angle_table_count         = 0U;

    for (i = 0U; i < ECU_ANGLE_TABLE_SIZE; ++i) {
        g_angle_table[i].valid = 0U;
    }

    g_prime_off_pending = 0U;
    for (i = 0U; i < 4U; ++i) {
        g_prime_off_cnv[i] = 0U;
    }

    g_ivc_abdc_deg    = 50U;
    g_ivc_clamp_count = 0U;
}

uint8_t ecu_sched_test_angle_table_size(void)
{
    return g_angle_table_count;
}

uint8_t ecu_sched_test_get_angle_event(uint8_t index,
                                        uint8_t *tooth,
                                        uint8_t *sub_frac,
                                        uint8_t *ch,
                                        uint8_t *action,
                                        uint8_t *phase)
{
    if ((index >= g_angle_table_count) ||
        (tooth == NULL) || (sub_frac == NULL) ||
        (ch == NULL) || (action == NULL) || (phase == NULL)) {
        return 0U;
    }
    if (g_angle_table[index].valid == 0U) { return 0U; }
    *tooth    = g_angle_table[index].tooth_index;
    *sub_frac = g_angle_table[index].sub_frac_x256;
    *ch       = g_angle_table[index].channel;
    *action   = g_angle_table[index].action;
    *phase    = g_angle_table[index].phase_A;
    return 1U;
}

void ecu_sched_test_set_advance_deg(uint32_t adv)
{
    ecu_sched_set_advance_deg(adv);
}

void ecu_sched_test_set_dwell_ticks(uint32_t dwell)
{
    ecu_sched_set_dwell_ticks(dwell);
}

void ecu_sched_test_set_inj_pw_ticks(uint32_t pw_ticks)
{
    ecu_sched_set_inj_pw_ticks(pw_ticks);
}

void ecu_sched_test_set_soi_lead_deg(uint32_t soi_lead_deg)
{
    ecu_sched_set_soi_lead_deg(soi_lead_deg);
}

uint32_t ecu_sched_test_get_advance_deg(void)
{
    return g_advance_deg;
}

uint32_t ecu_sched_test_get_dwell_ticks(void)
{
    return g_dwell_ticks;
}

uint32_t ecu_sched_test_get_inj_pw_ticks(void)
{
    return g_inj_pw_ticks;
}

uint32_t ecu_sched_test_get_soi_lead_deg(void)
{
    return g_soi_lead_deg;
}

uint32_t ecu_sched_test_get_calibration_clamp_count(void)
{
    return g_calibration_clamp_count;
}

uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void)
{
    return g_cycle_schedule_drop_count;
}

uint32_t ecu_sched_test_get_late_event_count(void)
{
    return g_late_event_count;
}

void ecu_sched_test_set_ivc(uint8_t ivc_abdc_deg)
{
    ecu_sched_set_ivc(ivc_abdc_deg);
}

uint32_t ecu_sched_test_get_ivc_clamp_count(void)
{
    return g_ivc_clamp_count;
}

#endif /* EMS_HOST_TEST */
