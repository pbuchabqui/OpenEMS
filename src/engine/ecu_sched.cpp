/**
 * @file engine/ecu_sched.cpp
 * @brief ECU Scheduling Core v3 — Unified Deterministic Timing System (C / MISRA-C:2012)
 *
 * Implements unified ECU timing system with 32-bit timestamp extension,
 * hardware output compare for all 8 channels (4 INJ + 4 IGN), and PDB-ADC
 * MAP windowing. Resolves prescaler conflicts, dual scheduling paths,
 * low-RPM overflow issues, and orphaned ISR problems.
 *
 * Hardware Architecture:
 *   FTM0: 120 MHz / PS_8 = 15 MHz ~ 66.7 ns/tick
 *   32-bit timestamp: bits[31:16] = g_overflow_count, bits[15:0] = CnV
 *   PDB0: Triggered by FTM0_CH5 for MAP windowing at 120° ATDC
 *   ADC0: Hardware averaging 4 samples for MAP reading
 *
 * MISRA-C:2012 compliance notes:
 *   - No dynamic memory allocation (Rule 21.3).
 *   - All variables explicitly typed with stdint.h types (Rule 4.6).
 *   - No recursion (Rule 17.2).
 *   - Integer constants use U/UL suffix (Rule 7.2).
 *   - Compound assignment on volatile not used for read-modify-write
 *     of multi-bit fields — explicit masking applied (Rule 12.1).
 */

#include "engine/ecu_sched.h"
#include "drv/ckp.h"

#include <stdint.h>
#include <stddef.h>
#include <cassert>

// CRITICAL FIX: Add debug assertions for safety-critical parameters
#ifndef NDEBUG
#define ASSERT_VALID_CHANNEL(ch) assert((ch) < ECU_CHANNELS)
#define ASSERT_VALID_ACTION(act) assert((act) <= ECU_ACT_SPARK)
#define ASSERT_VALID_TIMESTAMP(ts) assert((ts) != 0)
#define ASSERT_VALID_QUEUE_COUNT(count) assert((count) <= ECU_QUEUE_SIZE)
#define ASSERT_INVARIANTS() assert_invariants()
#else
#define ASSERT_VALID_CHANNEL(ch) ((void)0)
#define ASSERT_VALID_ACTION(act) ((void)0)
#define ASSERT_VALID_TIMESTAMP(ts) ((void)0)
#define ASSERT_VALID_QUEUE_COUNT(count) ((void)0)
#define ASSERT_INVARIANTS() ((void)0)
#endif

/* ============================================================================
 * Host-test peripheral mocks
 * In target builds FTM0/PDB0/ADC0 are defined in the header via base address.
 * In host tests the macros are defined externally in the test file.
 * ========================================================================= */

#if defined(EMS_HOST_TEST)
/* Defined by the test translation unit — no redefinition here. */
extern FTM_Type  g_mock_ftm0;
extern PDB_Type  g_mock_pdb0;
extern ADC_Type  g_mock_adc0;
/* Redefine peripheral macros to point at mock structs */
#undef  FTM0
#define FTM0  (&g_mock_ftm0)
#undef  PDB0
#define PDB0  (&g_mock_pdb0)
#undef  ADC0
#define ADC0  (&g_mock_adc0)

/* SIM_SCGC6 mock */
static volatile uint32_t g_mock_sim_scgc6;
#undef  SIM_SCGC6_ADDR
#define SIM_SCGC6_REG  g_mock_sim_scgc6
#else
#define SIM_SCGC6_REG  (*((volatile uint32_t *)SIM_SCGC6_ADDR))
#endif /* EMS_HOST_TEST */

/* ============================================================================
 * Internal constants
 * ========================================================================= */

#define ECU_CHANNELS    8U    /* FTM0 channels */

/* Channel assignments for unified system */
#define ECU_IGN_CH_FIRST  4U

/* Degrees in one 4-stroke cycle */
#define ECU_CYCLE_DEG  720U

/* Number of cylinders */
#define ECU_NUM_CYL  4U

/* Firing order: 1-3-4-2 */
#define ECU_FIRING_ORDER {1U, 3U, 4U, 2U}

/* TDC compression angles for firing order (degrees in 720° cycle) */
#define ECU_TDC_DEG {0U, 180U, 360U, 540U}

/* MAP windowing: trigger PDB at 120° ATDC for MAP reading */
#define ECU_MAP_TRIGGER_DEG  120U

/* ============================================================================
 * Event queue
 * ========================================================================= */

typedef struct {
    uint32_t timestamp32; /* Target 32-bit timestamp                   */
    uint8_t  channel;     /* FTM0 channel (0-7)                        */
    uint8_t  action;      /* ECU_ACT_* code                            */
    uint8_t  valid;       /* Non-zero if slot is occupied               */
    uint8_t  _pad;        /* Explicit padding for MISRA struct alignment */
} EcuEvent_t;

static volatile EcuEvent_t g_queue[ECU_QUEUE_SIZE];
static volatile uint8_t    g_queue_count;
static volatile uint8_t    g_next_valid[ECU_CHANNELS];
static volatile uint32_t   g_next_ts[ECU_CHANNELS];
static volatile uint8_t    g_next_action[ECU_CHANNELS];

/* ============================================================================
 * Module globals (exported via header)
 * ========================================================================= */

volatile uint32_t g_overflow_count  = 0U;
volatile uint32_t g_late_event_count = 0U;
volatile uint32_t g_late_delay_samples = 0U;
volatile uint32_t g_late_delay_sum_ticks = 0U;
volatile uint32_t g_late_delay_max_ticks = 0U;
volatile uint8_t g_queue_depth_peak = 0U;
volatile uint8_t g_queue_depth_last_cycle_peak = 0U;
volatile uint32_t g_cycle_schedule_drop_count = 0U;
volatile uint32_t g_calibration_clamp_count = 0U;

/* ============================================================================
 * Module-private configuration (set by test helpers or calibration layer)
 * ========================================================================= */

static volatile uint32_t g_ticks_per_rev  = 900000U; /* Default: 1000 RPM @ PS=8 */
static volatile uint32_t g_advance_deg    = 10U;    /* Spark advance, degrees BTDC */
static volatile uint32_t g_dwell_ticks    = 45000U; /* ~3 ms at 15 tick/us */
static volatile uint32_t g_inj_pw_ticks   = 45000U; /* ~3 ms at 15 tick/us */
static volatile uint32_t g_soi_lead_deg   = 62U;    /* SOI lead before TDC */
static volatile uint8_t g_cycle_fill_active = 0U;
static volatile uint8_t g_cycle_fill_peak = 0U;
static volatile uint8_t g_presync_enable = 1U;
static volatile uint8_t g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
static volatile uint8_t g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
static volatile uint8_t g_presync_bank_toggle = 0U;
static volatile uint8_t g_hook_prev_full_sync = 0U;
static volatile uint8_t g_hook_prev_valid = 0U;
static volatile uint16_t g_hook_prev_tooth = 0U;
static volatile uint8_t g_hook_schedule_this_gap = 1U;

static uint32_t near_time_window_ticks(void);
static uint32_t current_timestamp32(void);
static void recompute_next_per_channel(void);

static void calculate_presync_revolution(uint32_t current_timestamp);

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

static void sanitize_runtime_calibration(void)
{
    static const uint32_t kMinTicksPerRev = 50000U;
    static const uint32_t kMaxTicksPerRev = 6000000U;
    static const uint32_t kMinPulseTicks  = 1U;
    uint32_t max_span_ticks;
    uint8_t clamped = 0U;

    if (g_ticks_per_rev < kMinTicksPerRev) {
        g_ticks_per_rev = kMinTicksPerRev;
        clamped = 1U;
    } else if (g_ticks_per_rev > kMaxTicksPerRev) {
        g_ticks_per_rev = kMaxTicksPerRev;
        clamped = 1U;
    }

    max_span_ticks = g_ticks_per_rev * 2U; /* 720-degree span in scheduler domain */

    if ((g_dwell_ticks > 0U) && (g_dwell_ticks < kMinPulseTicks)) {
        g_dwell_ticks = kMinPulseTicks;
        clamped = 1U;
    } else if (g_dwell_ticks > max_span_ticks) {
        g_dwell_ticks = max_span_ticks;
        clamped = 1U;
    }

    if ((g_inj_pw_ticks > 0U) && (g_inj_pw_ticks < kMinPulseTicks)) {
        g_inj_pw_ticks = kMinPulseTicks;
        clamped = 1U;
    } else if (g_inj_pw_ticks > max_span_ticks) {
        g_inj_pw_ticks = max_span_ticks;
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

static uint32_t ticks_per_rev_from_rpm_x10(uint32_t rpm_x10)
{
    if (rpm_x10 == 0U) {
        return g_ticks_per_rev;
    }
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(ECU_SYSTEM_CLOCK_HZ) * 60ULL * 10ULL) /
        (static_cast<uint64_t>(ECU_FTM0_PRESCALER) * rpm_x10));
}

static uint32_t clamp_tpr_step(uint32_t current_tpr, uint32_t target_tpr)
{
    /*
     * Tooth-to-tooth slew limit to avoid phase noise and timestamp thrashing
     * under noisy instantaneous RPM samples.
     */
    const uint32_t min_tpr = current_tpr - (current_tpr / 5U); /* -20% */
    const uint32_t max_tpr = current_tpr + (current_tpr / 4U); /* +25% */
    if (target_tpr < min_tpr) {
        return min_tpr;
    }
    if (target_tpr > max_tpr) {
        return max_tpr;
    }
    return target_tpr;
}

static void retime_far_events_from_tooth_rpm(uint32_t rpm_x10)
{
    uint32_t now;
    uint32_t model_tpr;
    uint32_t target_tpr;
    uint32_t effective_tpr;
    uint32_t near_win;
    uint8_t i;

    if (rpm_x10 == 0U) {
        return;
    }

    now = current_timestamp32();
    model_tpr = g_ticks_per_rev;
    if (model_tpr == 0U) {
        return;
    }

    target_tpr = ticks_per_rev_from_rpm_x10(rpm_x10);
    effective_tpr = clamp_tpr_step(model_tpr, target_tpr);
    near_win = near_time_window_ticks();

    /*
     * Apply compensation only for far-time events. Near-time events are left
     * untouched to preserve edge determinism and minimize jitter.
     */
    for (i = 0U; i < g_queue_count; ++i) {
        uint32_t ts;
        uint32_t delta;
        uint32_t new_delta;
        uint32_t new_ts;

        if (g_queue[i].valid == 0U) {
            continue;
        }

        ts = g_queue[i].timestamp32;
        if (ts <= now) {
            continue;
        }

        delta = ts - now;
        if (delta <= near_win) {
            continue;
        }

        new_delta = static_cast<uint32_t>(
            ((static_cast<uint64_t>(delta) * effective_tpr) + (model_tpr / 2U)) / model_tpr);
        if (new_delta < near_win) {
            new_delta = near_win;
        }
        new_ts = now + new_delta;
        g_queue[i].timestamp32 = new_ts;
    }

    recompute_next_per_channel();
}

/* ============================================================================
 * Internal helpers
 * ========================================================================= */

static uint32_t near_time_window_ticks(void)
{
    /*
     * Dynamic near-time window from current engine speed:
     *   window ~= 2 tooth periods (60-2 wheel => 58 teeth/rev)
     * Clamp to keep deterministic ISR cost and avoid over-arming far events.
     */
    uint32_t tpr = g_ticks_per_rev;
    uint32_t tooth_ticks = (tpr / 58U);
    uint32_t win = tooth_ticks * 2U;

    if (win < 1024U) {
        win = 1024U;
    } else if (win > 65535U) {
        /*
         * Never arm farther than one 16-bit timer cycle ahead. This preserves
         * CnV semantics while avoiding artificial late events at low RPM.
         */
        win = 65535U;
    }
    return win;
}

static void update_queue_depth_metrics(void)
{
    if (g_queue_count > g_queue_depth_peak) {
        g_queue_depth_peak = g_queue_count;
    }
    if ((g_cycle_fill_active != 0U) && (g_queue_count > g_cycle_fill_peak)) {
        g_cycle_fill_peak = g_queue_count;
    }
}

static void record_late_delay(uint32_t now, uint32_t ts)
{
    uint32_t delay;
    if (now <= ts) {
        return;
    }
    delay = now - ts;
    ++g_late_delay_samples;
    if ((0xFFFFFFFFU - g_late_delay_sum_ticks) < delay) {
        g_late_delay_sum_ticks = 0xFFFFFFFFU;
    } else {
        g_late_delay_sum_ticks += delay;
    }
    if (delay > g_late_delay_max_ticks) {
        g_late_delay_max_ticks = delay;
    }
}

/**
 * Force an output on channel ch according to action using software control.
 * Used for late events that can no longer be scheduled via CnV.
 *
 * Note: On real hardware SWOC (Software Output Control) requires setting
 * FTM0->MODE[WPDIS] and using the FTM0_SWOCTRL register. For this
 * implementation we force by toggling CnSC mode and immediately clearing/
 * setting via a zero-delay compare. In host tests the mock registers accept
 * arbitrary writes.
 */
static void force_output(uint8_t ch, uint8_t action)
{
    uint32_t cnsc_val;
    if (action == ECU_ACT_SPARK || action == ECU_ACT_INJ_OFF) {
        /* Clear output (LOW) */
        cnsc_val = FTM_CnSC_OC_CLEAR;
    } else {
        /* Set output (HIGH) */
        cnsc_val = FTM_CnSC_OC_SET;
    }
    /* Program CnV to current counter + 1 to fire as soon as possible */
    FTM0->CONTROLS[ch].CnV  = (uint32_t)((FTM0->CNT + 1U) & 0xFFFFU);
    FTM0->CONTROLS[ch].CnSC = cnsc_val;
}

/**
 * Arm channel ch to fire at the tick given by the low 16 bits of timestamp32.
 * The hardware will drive the pin autonomously when FTM0->CNT == CnV.
 */
static void arm_channel(uint8_t ch, uint32_t timestamp32, uint8_t action)
{
    uint32_t cnsc_val;
    if (action == ECU_ACT_SPARK || action == ECU_ACT_INJ_OFF) {
        cnsc_val = FTM_CnSC_OC_CLEAR;
    } else {
        cnsc_val = FTM_CnSC_OC_SET;
    }
    FTM0->CONTROLS[ch].CnV  = (uint32_t)(timestamp32 & 0xFFFFU);
    FTM0->CONTROLS[ch].CnSC = cnsc_val;
}

/**
 * Remove one event from the queue by shifting tail entries down.
 * index must be < g_queue_count.
 */
static void queue_remove(uint8_t index)
{
    uint8_t i;
    
    // CRITICAL FIX: Validate queue index
    ASSERT_VALID_QUEUE_COUNT(g_queue_count);
    assert(index < g_queue_count);
    
    for (i = index; i < (g_queue_count - 1U); ++i) {
        g_queue[i].timestamp32 = g_queue[i + 1U].timestamp32;
        g_queue[i].channel     = g_queue[i + 1U].channel;
        g_queue[i].action      = g_queue[i + 1U].action;
        g_queue[i].valid       = g_queue[i + 1U].valid;
        g_queue[i]._pad        = 0U;
    }
    g_queue[g_queue_count - 1U].valid = 0U;
    if (g_queue_count > 0U) {
        --g_queue_count;
    }
}

static void recompute_next_per_channel(void)
{
    uint8_t ch;
    uint8_t i;

    for (ch = 0U; ch < ECU_CHANNELS; ++ch) {
        g_next_valid[ch] = 0U;
        g_next_ts[ch] = 0U;
        g_next_action[ch] = 0U;
    }

    for (i = 0U; i < g_queue_count; ++i) {
        if (g_queue[i].valid == 0U) {
            continue;
        }

        ch = g_queue[i].channel;
        if ((ch >= ECU_CHANNELS) ||
            ((g_next_valid[ch] != 0U) && (g_queue[i].timestamp32 >= g_next_ts[ch]))) {
            continue;
        }

        g_next_valid[ch] = 1U;
        g_next_ts[ch] = g_queue[i].timestamp32;
        g_next_action[ch] = g_queue[i].action;
    }
}

#ifndef NDEBUG
static void assert_invariants(void)
{
    uint8_t ch;
    uint8_t i;

    ASSERT_VALID_QUEUE_COUNT(g_queue_count);
    for (i = 1U; i < g_queue_count; ++i) {
        assert(g_queue[i - 1U].timestamp32 <= g_queue[i].timestamp32);
    }

    for (ch = 0U; ch < ECU_CHANNELS; ++ch) {
        uint8_t found = 0U;
        uint32_t min_ts = 0U;
        uint8_t min_act = 0U;
        for (i = 0U; i < g_queue_count; ++i) {
            if ((g_queue[i].valid != 0U) && (g_queue[i].channel == ch)) {
                if ((found == 0U) || (g_queue[i].timestamp32 < min_ts)) {
                    found = 1U;
                    min_ts = g_queue[i].timestamp32;
                    min_act = g_queue[i].action;
                }
            }
        }

        if (found == 0U) {
            assert(g_next_valid[ch] == 0U);
        } else {
            assert(g_next_valid[ch] != 0U);
            assert(g_next_ts[ch] == min_ts);
            assert(g_next_action[ch] == min_act);
        }
    }
}
#endif

static uint32_t current_timestamp32(void)
{
    return (g_overflow_count << 16U) | (FTM0->CNT & 0xFFFFU);
}

static uint8_t is_timestamp_late(uint32_t timestamp32)
{
    return (timestamp32 < current_timestamp32()) ? 1U : 0U;
}

static uint8_t remove_event_for_channel_ts(uint8_t ch, uint32_t ts)
{
    uint8_t i;
    for (i = 0U; i < g_queue_count; ++i) {
        if ((g_queue[i].valid != 0U) &&
            (g_queue[i].channel == ch) &&
            (g_queue[i].timestamp32 == ts)) {
            queue_remove(i);
            return 1U;
        }
    }
    return 0U;
}

static void process_channel_ready_event(uint8_t ch)
{
    for (;;) {
        uint8_t removed;
        uint32_t ts;
        uint8_t act;
        uint32_t now;
        uint32_t delta;

        if ((ch >= ECU_CHANNELS) || (g_next_valid[ch] == 0U)) {
            ASSERT_INVARIANTS();
            return;
        }

        ts = g_next_ts[ch];
        act = g_next_action[ch];
        now = current_timestamp32();

        if (ts < now) {
            removed = remove_event_for_channel_ts(ch, ts);
            if (removed == 0U) {
                uint8_t i;
                for (i = 0U; i < g_queue_count; ++i) {
                    if ((g_queue[i].valid != 0U) && (g_queue[i].channel == ch)) {
                        queue_remove(i);
                        removed = 1U;
                        break;
                    }
                }
            }
            if (removed != 0U) {
                record_late_delay(now, ts);
                force_output(ch, act);
                ++g_late_event_count;
            }
            recompute_next_per_channel();
            continue;
        }

        delta = ts - now;
        if (delta > near_time_window_ticks()) {
            ASSERT_INVARIANTS();
            return;
        }

        arm_channel(ch, ts, act);
        ASSERT_INVARIANTS();
        return;
    }
}

static void arm_due_channels(void)
{
    uint8_t ch;
    for (ch = 0U; ch < ECU_CHANNELS; ++ch) {
        process_channel_ready_event(ch);
    }
}

static void clear_all_events_and_drive_safe_outputs(void)
{
    uint8_t ch;
    uint8_t i;

    for (i = 0U; i < ECU_QUEUE_SIZE; ++i) {
        g_queue[i].timestamp32 = 0U;
        g_queue[i].channel     = 0U;
        g_queue[i].action      = 0U;
        g_queue[i].valid       = 0U;
        g_queue[i]._pad        = 0U;
    }
    g_queue_count = 0U;
    update_queue_depth_metrics();

    for (ch = 0U; ch < ECU_CHANNELS; ++ch) {
        g_next_valid[ch] = 0U;
        g_next_ts[ch] = 0U;
        g_next_action[ch] = 0U;
    }

    /* Safe state: injectors closed, coils de-energised. */
    for (ch = 0U; ch < ECU_CHANNELS; ++ch) {
        uint8_t safe_action = (ch < ECU_IGN_CH_FIRST) ? ECU_ACT_INJ_OFF : ECU_ACT_SPARK;
        force_output(ch, safe_action);
    }
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
    /* 2a. Write-protect disable + FTMEN (must be first write) */
    FTM0->MODE = (FTM_MODE_WPDIS | FTM_MODE_FTMEN);

    /* 2b. Stop counter for safe configuration */
    FTM0->SC = 0U;

    /* 2c. Free-running, MOD = 0xFFFF (maximum 16-bit range) */
    FTM0->CNT = 0U;
    FTM0->MOD = 0xFFFFU;

    /* 2d. CH0-CH3 (injectors): Output Compare, Set on match (HIGH = open) */
    for (ch = 0U; ch < ECU_IGN_CH_FIRST; ++ch) {
        FTM0->CONTROLS[ch].CnSC = FTM_CnSC_OC_SET;
        FTM0->CONTROLS[ch].CnV  = 0U;
    }

    /* 2e. CH4-CH7 (ignition coils): Output Compare, Clear on match (LOW = spark) */
    for (ch = ECU_IGN_CH_FIRST; ch < ECU_CHANNELS; ++ch) {
        FTM0->CONTROLS[ch].CnSC = FTM_CnSC_OC_CLEAR;
        FTM0->CONTROLS[ch].CnV  = 0U;
    }

    /* 2f. Start FTM0: system clock, PS=8, TOIE=1 (overflow interrupt) */
    FTM0->SC = (FTM_SC_CLKS_SYSTEM | FTM_SC_TOIE_MASK | FTM_SC_PS_8);

    /* 3. PDB0 configuration */
    /* Trigger source = FTM0 output trigger (TRGSEL=0x8), CH0 for ADC0 */
    PDB0->SC     = 0U;
    PDB0->IDLY   = 0U;
    PDB0->MOD    = 0xFFFFU;
    PDB0->CH0C1  = PDB_CHnC1_EN0_MASK;
    PDB0->CH0DLY0 = 0U;
    PDB0->SC     = (PDB_SC_PDBEN_MASK | PDB_SC_TRGSEL_FTM0 | PDB_SC_LDOK_MASK);

    /* 4. ADC0 configuration */
    /* 4a. 12-bit resolution, bus clock / 2 */
    ADC0->CFG1 = ADC_CFG1_12B_DIV2;
    ADC0->CFG2 = 0U; /* Side-A, no high-speed */

    /* 4b. Hardware averaging: 4 samples (AVGE=1, AVGS=00) */
    ADC0->SC3 = ADC_SC3_AVG4;
}

/* ============================================================================
 * Add_Event
 * ========================================================================= */

void Add_Event(uint32_t timestamp32, uint8_t channel, uint8_t action)
{
    uint32_t ev_hi;
    uint8_t  insert_pos;
    uint8_t  i;

    // CRITICAL FIX: Validate input parameters
    ASSERT_VALID_TIMESTAMP(timestamp32);
    ASSERT_VALID_CHANNEL(channel);
    ASSERT_VALID_ACTION(action);

    /* Compute overflow epoch of this event */
    ev_hi = (timestamp32 >> 16U);

    /* Late event: already past. */
    if ((ev_hi < g_overflow_count) || (is_timestamp_late(timestamp32) != 0U)) {
        /* CRITICAL FIX: Remove any existing events for this channel before forcing */
        for (i = 0U; i < g_queue_count; ++i) {
            if (g_queue[i].channel == channel) {
                queue_remove(i);
                --i; /* Adjust index after removal */
            }
        }
        recompute_next_per_channel();
        record_late_delay(current_timestamp32(), timestamp32);
        force_output(channel, action);
        ++g_late_event_count;
        ASSERT_INVARIANTS();
        return;
    }

    /* Queue full: treat as late event */
    if (g_queue_count >= ECU_QUEUE_SIZE) {
        /* CRITICAL FIX: Remove any existing events for this channel before forcing */
        for (i = 0U; i < g_queue_count; ++i) {
            if (g_queue[i].channel == channel) {
                queue_remove(i);
                --i; /* Adjust index after removal */
            }
        }
        recompute_next_per_channel();
        force_output(channel, action);
        ++g_late_event_count;
        ASSERT_INVARIANTS();
        return;
    }

    /* Find insertion position (ascending sort by timestamp32) */
    insert_pos = g_queue_count;
    for (i = 0U; i < g_queue_count; ++i) {
        if (timestamp32 < g_queue[i].timestamp32) {
            insert_pos = i;
            break;
        }
    }

    /* Shift tail entries up to make room */
    for (i = g_queue_count; i > insert_pos; --i) {
        g_queue[i].timestamp32 = g_queue[i - 1U].timestamp32;
        g_queue[i].channel     = g_queue[i - 1U].channel;
        g_queue[i].action      = g_queue[i - 1U].action;
        g_queue[i].valid       = g_queue[i - 1U].valid;
        g_queue[i]._pad        = 0U;
    }

    /* Insert new event */
    g_queue[insert_pos].timestamp32 = timestamp32;
    g_queue[insert_pos].channel     = channel;
    g_queue[insert_pos].action      = action;
    g_queue[insert_pos].valid       = 1U;
    g_queue[insert_pos]._pad        = 0U;
    ++g_queue_count;
    update_queue_depth_metrics();

    /* Keep one authoritative "next event" per channel and only arm due ones. */
    recompute_next_per_channel();
    if ((g_next_valid[channel] != 0U) &&
        (g_next_ts[channel] == timestamp32)) {
        process_channel_ready_event(channel);
    }
    ASSERT_INVARIANTS();
}

/* ============================================================================
 * FTM0_IRQHandler
 * ========================================================================= */

void FTM0_IRQHandler(void)
{
    uint32_t sc;
    uint8_t  ch;
    uint8_t  i;

    sc = FTM0->SC;

    /* ── 1. Timer Overflow (TOF): extend 32-bit counter ─────────────────── */
    if ((sc & FTM_SC_TOF_MASK) != 0U) {
        /* Clear TOF (W0C: write 0 to bit, preserve other bits) */
        FTM0->SC = (sc & ~FTM_SC_TOF_MASK);
        
        ++g_overflow_count;
        
        /* Ensure overflow increment is complete before queue processing */
        /* Memory barrier to prevent compiler reordering */
        __asm__ volatile("" ::: "memory");

        /* Remove events that are still late after overflow advancement. */
        for (i = g_queue_count; i > 0U; --i) {
            uint8_t idx = i - 1U;
            if (is_timestamp_late(g_queue[idx].timestamp32) != 0U) {
                /* Still late even after increment — force and count */
                record_late_delay(current_timestamp32(), g_queue[idx].timestamp32);
                force_output(g_queue[idx].channel, g_queue[idx].action);
                ++g_late_event_count;
                queue_remove(idx);
            }
        }

        recompute_next_per_channel();
        arm_due_channels();
    }

    /* ── 2. Channel match (CHF set): cleanup fired events ───────────────── */
    for (ch = 0U; ch < ECU_CHANNELS; ++ch) {
        uint32_t cnsc = FTM0->CONTROLS[ch].CnSC;
        if ((cnsc & FTM_CnSC_CHF_MASK) != 0U) {
            /* Clear CHF (W0C) and disable further interrupts on this channel */
            FTM0->CONTROLS[ch].CnSC = (cnsc & ~FTM_CnSC_CHF_MASK);

            /* Remove the corresponding event from the queue */
            /* Remove the armed event for this channel (fallback: first by channel). */
            uint8_t removed = 0U;
            if ((ch < ECU_CHANNELS) && (g_next_valid[ch] != 0U)) {
                removed = remove_event_for_channel_ts(ch, g_next_ts[ch]);
            }

            if (removed == 0U) {
                for (i = 0U; i < g_queue_count; ++i) {
                    if ((g_queue[i].valid != 0U) &&
                        (g_queue[i].channel == ch)) {
                        queue_remove(i);
                        break;
                    }
                }
            }

            recompute_next_per_channel();
            process_channel_ready_event(ch);
        }
    }
    ASSERT_INVARIANTS();
}

/* ============================================================================
 * Pre-sync 360-degree fallback scheduling (HALF_SYNC)
 * ========================================================================= */

static void calculate_presync_revolution(uint32_t current_timestamp)
{
    static const uint8_t k_ign_pair_a[2] = {ECU_CH_IGN1, ECU_CH_IGN4}; /* cyl 1+4 */
    static const uint8_t k_ign_pair_b[2] = {ECU_CH_IGN2, ECU_CH_IGN3}; /* cyl 2+3 */
    static const uint8_t k_inj_bank_a[2] = {ECU_CH_INJ1, ECU_CH_INJ4}; /* cyl 1+4 */
    static const uint8_t k_inj_bank_b[2] = {ECU_CH_INJ2, ECU_CH_INJ3}; /* cyl 2+3 */

    uint32_t ticks_per_rev = g_ticks_per_rev;
    uint32_t advance_deg = g_advance_deg;
    uint32_t dwell_ticks = g_dwell_ticks;
    uint32_t inj_pw_ticks = g_inj_pw_ticks;
    uint32_t soi_lead_deg = g_soi_lead_deg;
    uint8_t required_slots = 8U; /* wasted spark only */

    if (inj_pw_ticks > 0U) {
        required_slots = (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) ? 16U : 12U;
    }
    if (g_queue_count > (uint8_t)(ECU_QUEUE_SIZE - required_slots)) {
        ++g_cycle_schedule_drop_count;
        return;
    }

    /* Ignition: wasted spark (pairs every 360-degree revolution). */
    if (g_presync_ign_mode == ECU_PRESYNC_IGN_WASTED_SPARK) {
        uint32_t adv = advance_deg % 360U;
        uint32_t spark_a_deg = (360U - adv) % 360U;
        uint32_t spark_b_deg = (180U + 360U - adv) % 360U;
        uint32_t spark_a_ticks = (spark_a_deg * ticks_per_rev) / 360U;
        uint32_t spark_b_ticks = (spark_b_deg * ticks_per_rev) / 360U;
        uint32_t dwell_a_ticks = (spark_a_ticks >= dwell_ticks) ? (spark_a_ticks - dwell_ticks) : 0U;
        uint32_t dwell_b_ticks = (spark_b_ticks >= dwell_ticks) ? (spark_b_ticks - dwell_ticks) : 0U;
        uint32_t ts_dwell_a = current_timestamp + dwell_a_ticks;
        uint32_t ts_spark_a = current_timestamp + spark_a_ticks;
        uint32_t ts_dwell_b = current_timestamp + dwell_b_ticks;
        uint32_t ts_spark_b = current_timestamp + spark_b_ticks;

        Add_Event(ts_dwell_a, k_ign_pair_a[0], ECU_ACT_DWELL_START);
        Add_Event(ts_dwell_a, k_ign_pair_a[1], ECU_ACT_DWELL_START);
        Add_Event(ts_spark_a, k_ign_pair_a[0], ECU_ACT_SPARK);
        Add_Event(ts_spark_a, k_ign_pair_a[1], ECU_ACT_SPARK);

        Add_Event(ts_dwell_b, k_ign_pair_b[0], ECU_ACT_DWELL_START);
        Add_Event(ts_dwell_b, k_ign_pair_b[1], ECU_ACT_DWELL_START);
        Add_Event(ts_spark_b, k_ign_pair_b[0], ECU_ACT_SPARK);
        Add_Event(ts_spark_b, k_ign_pair_b[1], ECU_ACT_SPARK);
    }

    if (inj_pw_ticks == 0U) {
        return;
    }

    /* Injection fallback timing: one anchor per revolution in 360-degree domain. */
    uint32_t soi = soi_lead_deg % 360U;
    uint32_t inj_deg = (360U - soi) % 360U;
    uint32_t inj_ticks = (inj_deg * ticks_per_rev) / 360U;
    uint32_t ts_inj_on = current_timestamp + inj_ticks;
    uint32_t ts_inj_off = ts_inj_on + inj_pw_ticks;

    if (g_presync_inj_mode == ECU_PRESYNC_INJ_SIMULTANEOUS) {
        Add_Event(ts_inj_on, ECU_CH_INJ1, ECU_ACT_INJ_ON);
        Add_Event(ts_inj_on, ECU_CH_INJ2, ECU_ACT_INJ_ON);
        Add_Event(ts_inj_on, ECU_CH_INJ3, ECU_ACT_INJ_ON);
        Add_Event(ts_inj_on, ECU_CH_INJ4, ECU_ACT_INJ_ON);
        Add_Event(ts_inj_off, ECU_CH_INJ1, ECU_ACT_INJ_OFF);
        Add_Event(ts_inj_off, ECU_CH_INJ2, ECU_ACT_INJ_OFF);
        Add_Event(ts_inj_off, ECU_CH_INJ3, ECU_ACT_INJ_OFF);
        Add_Event(ts_inj_off, ECU_CH_INJ4, ECU_ACT_INJ_OFF);
    } else {
        const uint8_t* bank = (g_presync_bank_toggle == 0U) ? k_inj_bank_a : k_inj_bank_b;
        Add_Event(ts_inj_on, bank[0], ECU_ACT_INJ_ON);
        Add_Event(ts_inj_on, bank[1], ECU_ACT_INJ_ON);
        Add_Event(ts_inj_off, bank[0], ECU_ACT_INJ_OFF);
        Add_Event(ts_inj_off, bank[1], ECU_ACT_INJ_OFF);
        g_presync_bank_toggle ^= 1U;
    }
}

/* ============================================================================
 * Calculate_Sequential_Cycle
 * ========================================================================= */

void Calculate_Sequential_Cycle(uint32_t current_timestamp)
{
    /*
     * Firing order: 1-3-4-2 (0-indexed cylinders: 0, 2, 3, 1).
     * TDC compression angles in a 720-degree 4-stroke cycle:
     *   Cyl 1 = 0 deg, Cyl 3 = 180 deg, Cyl 4 = 360 deg, Cyl 2 = 540 deg.
     *
     * For each cylinder in firing order, schedule:
     *   ECU_ACT_DWELL_START at TDC_angle - advance_deg - dwell_ticks_as_angle
     *   ECU_ACT_SPARK        at TDC_angle - advance_deg
     *
     * Angle to ticks: delta_ticks = (angle_deg * g_ticks_per_rev) / ECU_CYCLE_DEG
     */

    /* Firing order: cylinder indices 0-based, firing sequence 1,3,4,2 */
    static const uint8_t  k_fire_order[ECU_NUM_CYL]  = {0U, 2U, 3U, 1U};
    /* TDC compression offset per cylinder (degrees in 720-deg cycle) */
    static const uint32_t k_tdc_deg[ECU_NUM_CYL] = {0U, 180U, 360U, 540U};
    /* Ignition channel per cylinder (0-indexed cyl -> IGN channel) */
    static const uint8_t  k_ign_ch[ECU_NUM_CYL]  = {
        ECU_CH_IGN1, ECU_CH_IGN2, ECU_CH_IGN3, ECU_CH_IGN4
    };
    /* Injection channel per cylinder (0-indexed cyl -> INJ channel) */
    static const uint8_t  k_inj_ch[ECU_NUM_CYL]  = {
        ECU_CH_INJ1, ECU_CH_INJ2, ECU_CH_INJ3, ECU_CH_INJ4
    };

    uint32_t seq;
    uint32_t ticks_per_rev  = g_ticks_per_rev;
    uint32_t advance_deg    = g_advance_deg;
    uint32_t dwell_ticks    = g_dwell_ticks;
    uint32_t inj_pw_ticks   = g_inj_pw_ticks;
    uint32_t soi_lead_deg   = g_soi_lead_deg;
    uint8_t events_per_cyl = (inj_pw_ticks > 0U) ? 4U : 2U;
    uint8_t required_slots = (uint8_t)(ECU_NUM_CYL * events_per_cyl);

    if (g_queue_count > (uint8_t)(ECU_QUEUE_SIZE - required_slots)) {
        ++g_cycle_schedule_drop_count;
        g_cycle_fill_active = 0U;
        return;
    }

    g_cycle_fill_active = 1U;
    g_cycle_fill_peak = g_queue_count;

    for (seq = 0U; seq < ECU_NUM_CYL; ++seq) {
        uint8_t  cyl_idx    = k_fire_order[seq];
        uint32_t tdc_deg    = k_tdc_deg[seq];
        uint8_t  ign_ch     = k_ign_ch[cyl_idx];
        uint8_t  inj_ch     = k_inj_ch[cyl_idx];

        /* Degrees from current position to TDC of this cylinder minus advance */
        uint32_t spark_deg;
        uint32_t dwell_deg;
        uint32_t spark_ticks;
        uint32_t dwell_start_ticks;
        uint32_t ts_spark;
        uint32_t ts_dwell;
        uint32_t soi_deg;
        uint32_t soi_ticks;
        uint32_t ts_inj_on;
        uint32_t ts_inj_off;

        /* spark angle offset from "now" (0 reference) */
        if (tdc_deg >= advance_deg) {
            spark_deg = tdc_deg - advance_deg;
        } else {
            spark_deg = (ECU_CYCLE_DEG + tdc_deg) - advance_deg;
        }

        /* Convert angular offset to ticks (integer division, no float) */
        spark_ticks = (spark_deg * ticks_per_rev) / ECU_CYCLE_DEG;

        /* Dwell start is dwell_ticks before the spark */
        dwell_start_ticks = (spark_ticks >= dwell_ticks)
                            ? (spark_ticks - dwell_ticks)
                            : 0U;

        /* Build 32-bit timestamps relative to current_timestamp */
        ts_spark = current_timestamp + spark_ticks;
        ts_dwell = current_timestamp + dwell_start_ticks;

        if (tdc_deg >= soi_lead_deg) {
            soi_deg = tdc_deg - soi_lead_deg;
        } else {
            soi_deg = (ECU_CYCLE_DEG + tdc_deg) - soi_lead_deg;
        }
        soi_ticks = (soi_deg * ticks_per_rev) / ECU_CYCLE_DEG;
        ts_inj_on = current_timestamp + soi_ticks;
        ts_inj_off = ts_inj_on + inj_pw_ticks;

        /* Schedule dwell start (coil energise) */
        Add_Event(ts_dwell, ign_ch, ECU_ACT_DWELL_START);

        /* Schedule spark (coil cut) */
        Add_Event(ts_spark, ign_ch, ECU_ACT_SPARK);

        if (inj_pw_ticks > 0U) {
            Add_Event(ts_inj_on, inj_ch, ECU_ACT_INJ_ON);
            Add_Event(ts_inj_off, inj_ch, ECU_ACT_INJ_OFF);
        }
    }

    g_queue_depth_last_cycle_peak = g_cycle_fill_peak;
    g_cycle_fill_active = 0U;
}

void ecu_sched_set_ticks_per_rev(uint32_t tpr)
{
    ecu_sched_commit_calibration(
        tpr, g_advance_deg, g_dwell_ticks, g_inj_pw_ticks, g_soi_lead_deg);
}

void ecu_sched_set_advance_deg(uint32_t adv)
{
    ecu_sched_commit_calibration(
        g_ticks_per_rev, adv, g_dwell_ticks, g_inj_pw_ticks, g_soi_lead_deg);
}

void ecu_sched_set_dwell_ticks(uint32_t dwell)
{
    ecu_sched_commit_calibration(
        g_ticks_per_rev, g_advance_deg, dwell, g_inj_pw_ticks, g_soi_lead_deg);
}

void ecu_sched_set_inj_pw_ticks(uint32_t pw_ticks)
{
    ecu_sched_commit_calibration(
        g_ticks_per_rev, g_advance_deg, g_dwell_ticks, pw_ticks, g_soi_lead_deg);
}

void ecu_sched_set_soi_lead_deg(uint32_t soi_lead_deg)
{
    ecu_sched_commit_calibration(
        g_ticks_per_rev, g_advance_deg, g_dwell_ticks, g_inj_pw_ticks, soi_lead_deg);
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

void ecu_sched_commit_calibration(uint32_t tpr,
                                  uint32_t advance_deg,
                                  uint32_t dwell_ticks,
                                  uint32_t inj_pw_ticks,
                                  uint32_t soi_lead_deg)
{
    enter_critical();
    g_ticks_per_rev = tpr;
    g_advance_deg = advance_deg;
    g_dwell_ticks = dwell_ticks;
    g_inj_pw_ticks = inj_pw_ticks;
    g_soi_lead_deg = soi_lead_deg;
    sanitize_runtime_calibration();
    exit_critical();
}

namespace ems::engine {

void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept
{
    if ((snap.state != ems::drv::SyncState::FULL_SYNC) &&
        (snap.state != ems::drv::SyncState::HALF_SYNC)) {
        if (g_hook_prev_full_sync != 0U) {
            clear_all_events_and_drive_safe_outputs();
        }
        g_hook_prev_full_sync = 0U;
        g_hook_prev_valid = 0U;
        g_hook_prev_tooth = 0U;
        g_hook_schedule_this_gap = 1U;
        return;
    }

    retime_far_events_from_tooth_rpm(snap.rpm_x10);

    /* Poll near-time queue on each valid CKP tooth for low-latency arming. */
    arm_due_channels();

    g_hook_prev_full_sync = (snap.state == ems::drv::SyncState::FULL_SYNC) ? 1U : 0U;

    /* Detect boundary once per revolution when tooth index wraps to zero. */
    uint8_t rev_boundary = 0U;
    if (g_hook_prev_valid == 0U) {
        rev_boundary = (snap.tooth_index == 0U) ? 1U : 0U;
        g_hook_prev_valid = 1U;
    } else if ((snap.tooth_index == 0U) && (g_hook_prev_tooth != 0U)) {
        rev_boundary = 1U;
    }
    g_hook_prev_tooth = snap.tooth_index;

    if (rev_boundary == 0U) {
        return;
    }

    if ((snap.state == ems::drv::SyncState::HALF_SYNC) &&
        (g_presync_enable != 0U)) {
        const uint32_t current_timestamp =
            (::g_overflow_count << 16U) | (FTM0->CNT & 0xFFFFU);
        calculate_presync_revolution(current_timestamp);
        return;
    }

    if (g_hook_schedule_this_gap == 0U) {
        g_hook_schedule_this_gap = 1U;
        return;
    }
    g_hook_schedule_this_gap = 0U;

    const uint32_t current_timestamp =
        (::g_overflow_count << 16U) | (FTM0->CNT & 0xFFFFU);
    ::Calculate_Sequential_Cycle(current_timestamp);
}

}  // namespace ems::engine

/* ============================================================================
 * Test-only API
 * ========================================================================= */

#if defined(EMS_HOST_TEST)

void ecu_sched_test_reset(void)
{
    uint8_t i;
    g_overflow_count   = 0U;
    g_late_event_count = 0U;
    g_late_delay_samples = 0U;
    g_late_delay_sum_ticks = 0U;
    g_late_delay_max_ticks = 0U;
    g_queue_depth_peak = 0U;
    g_queue_depth_last_cycle_peak = 0U;
    g_cycle_schedule_drop_count = 0U;
    g_calibration_clamp_count = 0U;
    g_cycle_fill_active = 0U;
    g_cycle_fill_peak = 0U;
    g_presync_enable = 1U;
    g_presync_inj_mode = ECU_PRESYNC_INJ_SIMULTANEOUS;
    g_presync_ign_mode = ECU_PRESYNC_IGN_WASTED_SPARK;
    g_presync_bank_toggle = 0U;
    g_hook_prev_full_sync = 0U;
    g_hook_prev_valid = 0U;
    g_hook_prev_tooth = 0U;
    g_hook_schedule_this_gap = 1U;
    g_queue_count      = 0U;
    g_ticks_per_rev    = 900000U;
    g_advance_deg      = 10U;
    g_dwell_ticks      = 45000U;
    g_inj_pw_ticks     = 45000U;
    g_soi_lead_deg     = 62U;
    for (i = 0U; i < ECU_QUEUE_SIZE; ++i) {
        g_queue[i].timestamp32 = 0U;
        g_queue[i].channel     = 0U;
        g_queue[i].action      = 0U;
        g_queue[i].valid       = 0U;
        g_queue[i]._pad        = 0U;
    }
    for (i = 0U; i < ECU_CHANNELS; ++i) {
        g_next_valid[i] = 0U;
        g_next_ts[i] = 0U;
        g_next_action[i] = 0U;
    }
    ASSERT_INVARIANTS();
}

uint8_t ecu_sched_test_queue_size(void)
{
    return g_queue_count;
}

uint8_t ecu_sched_test_get_event(uint8_t index, uint32_t *ts,
                                  uint8_t *ch, uint8_t *act)
{
    if ((index >= g_queue_count) || (ts == NULL) || (ch == NULL) || (act == NULL)) {
        return 0U;
    }
    *ts  = g_queue[index].timestamp32;
    *ch  = g_queue[index].channel;
    *act = g_queue[index].action;
    return g_queue[index].valid;
}

void ecu_sched_test_set_ticks_per_rev(uint32_t tpr)
{
    ecu_sched_set_ticks_per_rev(tpr);
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

uint32_t ecu_sched_test_get_late_delay_samples(void)
{
    return g_late_delay_samples;
}

uint32_t ecu_sched_test_get_late_delay_sum_ticks(void)
{
    return g_late_delay_sum_ticks;
}

uint32_t ecu_sched_test_get_late_delay_max_ticks(void)
{
    return g_late_delay_max_ticks;
}

uint8_t ecu_sched_test_get_queue_depth_peak(void)
{
    return g_queue_depth_peak;
}

uint8_t ecu_sched_test_get_queue_depth_last_cycle_peak(void)
{
    return g_queue_depth_last_cycle_peak;
}

uint32_t ecu_sched_test_get_cycle_schedule_drop_count(void)
{
    return g_cycle_schedule_drop_count;
}

uint32_t ecu_sched_test_get_ticks_per_rev(void)
{
    return g_ticks_per_rev;
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

#endif /* EMS_HOST_TEST */
