/**
 * @file drv/ckp.cpp
 * @brief Módulo 1 (DECODE) + Módulo 2 (SYNC) — Engine Position Core — OpenEMS
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * MÓDULO 1: DECODE via TIM5 input capture (Crank)
 * ───────────────────────────────────────────────
 *   Hardware: TIM5 CH1 (PA0/CKP), rising edge input capture.
 *             Em targets embarcados o TIM5 é mapeado via hal/stm32h562/timer.cpp.
 *             Em host tests, TIM5_CKP_CAPTURE e TIM5_CAM_CAPTURE sao mocks volateis.
 *
 *   Fluxo da ISR (ckp_tim5_ch1_isr):
 *     1. Leitura de TIM5_CKP_CAPTURE (registrador de captura — travado pelo HW)
 *        ► NÃO lemos TIM5_CNT: o contador avançou enquanto a CPU atendia a IRQ.
 *          TIM5_CKP_CAPTURE contém o timestamp EXATO da borda de subida (RusEFI #1488).
 *     2. delta_ticks = capture_now - prev_capture com aritmetica circular uint32_t.
 *        Correto mesmo em overflow do TIM5 de 32 bits.
 *     3. Conversao para nanossegundos: period_ns = delta_ticks * 16
 *        TIM5: 62.5 MHz no STM32H562 -> 16 ns/tick
 *     4. Classificação pela razão r = Δ / período de referência:
 *        r < 0,5 NOISE · 0,5 ≤ r < 1,5 TOOTH · r ≥ 1,5 GAP
 *     5. Atualizacao da máquina de estados (Módulo 2)
 *     6. Disparo dos hooks sensors_on_tooth() / schedule_on_tooth()
 *
 * VANTAGEM DO INPUT CAPTURE vs GPIO/EXTI:
 *   O periférico TIM5 registra o timestamp da borda em hardware no exato
 *   instante do evento, independente do atraso de atendimento da IRQ
 *   (tipicamente dezenas de ciclos no Cortex-M33).
 *   A 6000 RPM, 0,2 µs de jitter ≈ 0,07° — inaceitável sem input capture.
 *   Com input capture: resolução = 1 tick = 16 ns ≈ 0,006° @ 6000 RPM.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * MÓDULO 2: SYNC — Máquina de Estados
 * ─────────────────────────────────────
 *   Roda fônica 60-2: 60 posições × 6°; 2 dentes ausentes consecutivos.
 *   O gap ocorre 1× por revolução; a ISR identifica-o por razão de período.
 *
 *   Estados (enum SyncState — definido em ckp.h):
 *     WAIT_GAP     → inicial: aguarda o 1º gap (≥3 dentes antes)
 *     HALF_SYNC    → 1º gap: contando dentes para confirmar
 *     FULL_SYNC    → gap com exatamente 57 dentes: tooth_index válido
 *     LOSS_OF_SYNC → contagem errada no gap, 58º dente sem gap, ruído
 *                    persistente ou stall
 *
 *   Transições:
 *     WAIT_GAP     + gap, count≥3     → HALF_SYNC
 *     LOSS_OF_SYNC + gap, count==57   → HALF_SYNC
 *     HALF_SYNC    + gap, count==57   → FULL_SYNC
 *     FULL_SYNC    + gap, count==57   → FULL_SYNC (tooth_index ← 0)
 *     HALF/FULL    + gap, count≠57    → LOSS_OF_SYNC (dente perdido/extra)
 *     HALF/FULL    + 58º dente        → LOSS_OF_SYNC (gap perdido)
 *
 * REJEIÇÃO DE RUÍDO:
 *   Uma borda com r < 0,5 é descartada SEM mover prev_capture: o dente real
 *   seguinte continua medido a partir do último dente real. Se a borda
 *   descartada fecha exatamente um período a partir da borda anterior à
 *   última aceite, a última era o ruído e é substituída. O período de
 *   referência só muda com dentes limpos (±25 %) ou com dois desvios
 *   coerentes seguidos (mudança real de rotação).
 */

#include "drv/ckp.h"
#include <cstdint>
#include <cstring>
#include "hal/timer.h"
#include "hal/critical_section.h"
#include "engine/calibration.h"
#include "drv/sensors.h"
#if defined(TARGET_STM32H562) && !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

// ── Mock de registradores para testes host ───────────────────────────────────
#if defined(EMS_HOST_TEST)
#include "hal/tim5_host.h"
#define TIM5_CNT ems_test_tim5_cnt
#endif

// FASTRUN coloca ISRs críticas em SRAM (zero cache miss).
// Em host/embedded: __attribute__((section(".fastrun"))) via WProgram.h / core_pins.h.
// Em host tests: indefinida — defini-la vazia garante compilação sem modificações.
#if !defined(FASTRUN)
#define FASTRUN
#endif

namespace ems::drv {
    extern volatile uint32_t g_dbg_gap_accepted;
    extern volatile uint32_t g_dbg_gap_premature;
    extern volatile uint32_t g_dbg_gap_acq_reject;
    extern volatile uint32_t g_dbg_gap_last_tc;
    extern volatile uint32_t g_dbg_loss_missing_gap;
    extern volatile uint32_t g_dbg_loss_stall;
    extern volatile uint32_t g_dbg_loss_avg;
    extern volatile uint32_t g_dbg_loss_delta;
}

namespace {

// ── Constantes da roda fônica 60-2 ───────────────────────────────────────────

// 60-2: 60 posições, 2 dentes ausentes consecutivos = 58 dentes reais.
// Espaçamento por posição: 360°/60 = 6,0°.
// Gap: 3 posições ausentes × 6° = 18° ≈ 3× período normal.
static constexpr uint16_t kRealTeethPerRev    = 58u;  // dentes físicos — uso exclusivo da máquina de estados (gap detection)
static constexpr uint16_t kTeethPositionsPerRev = 60u; // posições angulares uniformes — uso em cálculos de RPM e ângulo

// Fallback sequencial→wasted por ausência de CMP. O came dispara 1×/720° = a cada
// 2 revoluções, logo em operação normal o contador chega no máximo a ~2 antes de
// cada borda o zerar. 6 revs = 3 ciclos de came sem uma borda válida → assume-se
// que o sinal do came se perdeu e reverte-se a wasted-spark (cmp_confirms=0).
// Threshold >> 2 evita falso fallback com jitter/rejeição pontual.
// Em bench-mode o estimulador pára o RMT ao reprogramar (gaps longos sem borda),
// por isso a janela sobe para 60 revs (7.2s @ 500 RPM) — NUNCA em produção.
static constexpr uint16_t kMaxRevsWithoutCmp      = 6u;
static constexpr uint16_t kMaxRevsWithoutCmpBench = 60u;
static inline uint16_t max_revs_without_cmp() noexcept {
    return ems::drv::sensors_is_bench_mode() ? kMaxRevsWithoutCmpBench : kMaxRevsWithoutCmp;
}

// Teeth between two gap edges on a 60-2 wheel (teeth 1..57 after the gap
// edge). A gap is accepted in sync only with EXACTLY this count: a lost or an
// extra tooth becomes an explicit sync loss, never a wrong angle.
static constexpr uint16_t kTeethBetweenGaps = kRealTeethPerRev - 1u;  // 57
// Teeth needed after power-up before the very first gap may give HALF_SYNC.
static constexpr uint16_t kMinTeethBeforeFirstGap = 3u;
// Consecutive too-early edges (noise) before the reference is re-learned —
// also the recovery path for an abrupt speed rise (bench stimulator steps).
static constexpr uint8_t kNoiseResync = 4u;

// Preditor conservador para agendamento intra-dente:
// tendência entre os dois últimos períodos de referência, limitada a ±12,5 %.
static constexpr uint32_t kPredictionClampDen = 8u;

// Mínimo de ticks entre bordas para descartar glitches de EMC (<800 ns @ 62.5 MHz).
static constexpr uint32_t kMinToothTicks = 50u;

// ── Stall watchdog ─────────────────────────────────────────────────────────
// Tempo máximo sem dente antes de declarar motor parado: 200 ms @ 62.5 MHz.
// Resolve o caso em que o virabrequim para entre dois dentes — tooth_count
// para de incrementar e a detecção por contagem nunca dispara.
// Em bench-mode o estimulador pára o RMT ao reprogramar; 2 s evita falso stall
// nesses gaps. Em produção mantém-se 200 ms — injectar 2 s num motor parado
// afoga o motor e manda combustível cru para o escape.
static constexpr uint32_t kMinStallTimeoutTicks      = 12500000u;   // 200 ms @ 62.5 MHz
static constexpr uint32_t kMinStallTimeoutTicksBench = 125000000u;  // 2 s @ 62.5 MHz
static inline uint32_t min_stall_timeout_ticks() noexcept {
    return ems::drv::sensors_is_bench_mode() ? kMinStallTimeoutTicksBench : kMinStallTimeoutTicks;
}

// ---- Acesso a registradores TIM5 ------------------------------------------------
// STM32H562 TIM5 e GPIO sao configurados em hal/stm32h562/timer.cpp.
// O modulo usa aliases HAL para manter o decode desacoplado de offsets.
//
// CRÍTICO: Lemos TIM5_CKP_CAPTURE (registrador de CAPTURA travado pelo hardware),
//   não TIM5_CNT (contador livre que avançou durante o atendimento da ISR).
//   Esta distinção elimina o jitter de software: o valor em C0V reflete o
//   exato instante da borda de subida, independente da latência da IRQ.
//   Referência: RusEFI issue #1488 ("timestamp corruption from CNT vs CnV").
#if defined(EMS_HOST_TEST)
#ifndef TIM5_CKP_CAPTURE
#define TIM5_CKP_CAPTURE      ems_test_tim5_ccr1
#endif
#ifndef TIM5_CAM_CAPTURE
#define TIM5_CAM_CAPTURE      ems_test_tim5_ccr2
#endif
#ifndef CKP_CAM_GPIO_IDR
#define CKP_CAM_GPIO_IDR    ems_test_cam_gpio_idr
#endif
#else
#define TIM5_CKP_CAPTURE TIM5_CCR1  // TIM5_CH1 (PA0/AF2) — hardware capture sem jitter
#define TIM5_CAM_CAPTURE TIM5_CCR2
#define CKP_CAM_GPIO_IDR GPIOA_IDR
#endif

// ── Estado interno do decodificador ──────────────────────────────────────────
//
// INVARIANTE DE ACESSO — NUNCA VIOLAR:
//   g_state é escrito EXCLUSIVAMENTE pela ISR ckp_tim5_ch1_isr() (prioridade 1).
//   Qualquer outro contexto (main loop, ISRs de prioridade < 1) DEVE usar
//   ckp_snapshot() para ler g_state.snap — que aplica seção crítica CPSID/CPSIE.
//
//   Acessar g_state.snap diretamente fora da ISR de prioridade 1 é PROIBIDO
//   porque a leitura pode observar um snapshot parcialmente actualizado
//   (ex: tooth_index actualizado mas rpm_x10 ainda com valor anterior).
//
//   Se uma nova ISR de prioridade < 1 for adicionada e precisar de dados CKP,
//   ela DEVE chamar ckp_snapshot() ou ser elevada para prioridade 1 (com
//   revisão cuidadosa das implicações de latência para as demais ISRs).
struct DecoderState {
    ems::drv::CkpSnapshot snap;
    uint32_t prev_capture;              // last ACCEPTED edge (noise never moves it)
    uint32_t prev_prev_capture;         // accepted edge before that
    uint32_t ref_ticks;                 // reference tooth period (0 = learning)
    uint32_t prev_ref_ticks;            // previous reference (trend for prediction)
    uint32_t outlier_ticks;             // last tooth outside ±25 % of ref (speed-step detector)
    uint16_t tooth_count;               // teeth since the last gap edge
    uint8_t  have_edge;                 // prev_capture valid
    uint8_t  noise_streak;              // consecutive rejected edges
    uint8_t  cmp_confirms;              // confirmações do cam sensor (CH1)
    uint8_t  phase_half;               // 0/1: which 360° half of the 720° cycle (toggles at each gap)
    uint8_t  cmp_phase_pending;       // 1 = CMP validated, apply kCmpRefHalf at next gap instead of toggle
    uint8_t  cmp_ref_value;           // the value to SET at next gap (= kCmpRefHalf XOR 1, pre-toggle)
    uint32_t cmp_glitch_count;          // FIX P0: contador de glitches CMP rejeitados (diagnóstico)
};

static DecoderState g_state{};  // zero-init; SyncState::WAIT_GAP == 0
// ── Utilitários inline ────────────────────────────────────────────────────────

// Ticks TIM5 (16 ns) → ns, saturado (o snapshot exporta períodos em ns).
inline uint32_t ticks_to_ns(uint32_t ticks) noexcept {
    return (ticks > 0x0FFFFFFFu) ? 0xFFFFFFFFu : ticks * 16u;
}

// Calcula RPM × 10 a partir do período de um dente (nanossegundos).
// Cada dente ocupa 6° = 1/60 de revolução (roda 60-2: 60 posições uniformes).
// rpm × 10 = (60 s/min × 10⁹ ns/s × 10) / (60 × tooth_period_ns)
//           = 600.000.000.000 / (60 × tooth_period_ns)
inline uint32_t rpm_x10_from_period_ns(uint32_t period_ns) noexcept {
    if (period_ns == 0u) { return 0u; }
    return static_cast<uint32_t>(
        600000000000ULL / (static_cast<uint64_t>(kTeethPositionsPerRev) * period_ns));
}

// Caminho quente do ISR: period_ns = period_ticks * 16 ns, entao:
// rpm x10 = 600000000000 / (60 * 16 * period_ticks) = 625000000 / period_ticks.
inline uint32_t rpm_x10_from_period_ticks(uint32_t period_ticks) noexcept {
    if (period_ticks == 0u) { return 0u; }
    return 625000000u / period_ticks;
}

// RPM reportado só com referência angular (HALF/FULL_SYNC). Sem sync devolve
// 0: bordas periódicas de ruído (rede 60 Hz num CKP flutuante ≙ 60 RPM) nunca
// geram gap, e o valor cru ficava exposto como RPM real na telemetria e nos
// gates de motor-parado (burn de flash, teste de saídas).
inline uint32_t rpm_if_synced(uint32_t period_ticks) noexcept {
    const ems::drv::SyncState st = g_state.snap.state;
    if (st != ems::drv::SyncState::HALF_SYNC &&
        st != ems::drv::SyncState::FULL_SYNC) { return 0u; }
    return rpm_x10_from_period_ticks(period_ticks);
}

inline bool is_synced() noexcept {
    return g_state.snap.state == ems::drv::SyncState::HALF_SYNC ||
           g_state.snap.state == ems::drv::SyncState::FULL_SYNC;
}

// Next tooth period: reference + its last change, change clamped to ±12.5 %.
inline uint32_t predict_next_period_ticks() noexcept {
    const uint32_t cur = g_state.ref_ticks;
    const uint32_t prev = g_state.prev_ref_ticks;
    if (prev == 0u || cur > 0x7FFFFFFFu || prev > 0x7FFFFFFFu) { return cur; }
    int32_t trend = static_cast<int32_t>(cur) - static_cast<int32_t>(prev);
    const int32_t limit = static_cast<int32_t>(prev / kPredictionClampDen);
    if (trend > limit) { trend = limit; }
    if (trend < -limit) { trend = -limit; }
    const int32_t predicted = static_cast<int32_t>(cur) + trend;
    return (predicted > 0) ? static_cast<uint32_t>(predicted) : cur;
}

inline void set_reference(uint32_t ticks) noexcept {
    g_state.prev_ref_ticks = g_state.ref_ticks;
    g_state.ref_ticks = ticks;
}

inline void publish_periods() noexcept {
    g_state.snap.tooth_period_ns = ticks_to_ns(g_state.ref_ticks);
    g_state.snap.predicted_tooth_period_ns = ticks_to_ns(predict_next_period_ticks());
    g_state.snap.rpm_x10 = rpm_if_synced(g_state.ref_ticks);
}

static uint32_t s_prev_cmp_capture = 0u;
// Revoluções (gaps aceites em FULL_SYNC) desde a última borda CMP validada.
// Zerado na ISR do came; se exceder kMaxRevsWithoutCmp, força fallback a wasted.
static uint16_t s_revs_since_cmp = 0u;
// tooth_index da última borda CMP aceite (0xFF = não-ancorado). Uma borda de came
// real recorre sempre no mesmo dente; ruído (came desligado, PA1 a flutuar) ocorre
// em posições aleatórias → gate de consistência rejeita-o. ±kCmpToothTol dentes.
static uint8_t s_cmp_ref_tooth = 0xFFu;
static constexpr uint8_t kCmpToothTol = 3u;
// Rejeições temporais CONSECUTIVAS. Ao atingir kCmpRejectResync, s_prev_cmp_capture
// é considerado obsoleto (came reconectado após ausência/ruído → bordas reais caem
// fora da janela relativa a uma referência morta) e é largado p/ permitir recuperação.
static uint8_t s_cmp_reject_streak = 0u;
static constexpr uint8_t kCmpRejectResync = 3u;
// Dentes ainda por descartar pelo skip pós-silêncio (recarregado quando
// delta ≥ timeout de stall e ckp_skip_pulses_after_gap > 0).
static uint8_t s_skip_remaining = 0u;
// Fase medida do came (VVT, MS42 S15): ângulo de virabrequim ×10 (0..3599) da
// última borda CMP validada, contado a partir do gap — dente + fração do
// período de referência. seq incrementa a cada borda (0 = nunca medido).
static volatile uint16_t s_cam_angle_x10 = 0u;
static volatile uint32_t s_cam_edge_seq = 0u;

// After any CKP LOSS: require 2 fresh CMP edges before sequential (cmp_confirms>=2).
// Also drop inter-edge CMP timestamp so a stale s_prev (many revs old) cannot
// reject every reconnect edge until kCmpRejectResync trips.
inline void close_cmp_seq_gate() noexcept {
    g_state.cmp_confirms = 0u;
    g_state.snap.cmp_confirms = 0u;
    s_prev_cmp_capture = 0u;
    s_cmp_ref_tooth = 0xFFu;
    s_cmp_reject_streak = 0u;
}

// Edge classification against the reference period r = Δ / ref:
//   r < 0.5        NOISE  — dropped; prev_capture is NOT moved, so the next
//                           real edge is still measured from the last real one
//   0.5 ≤ r < 1.5  TOOTH  — tolerates cranking compression and fast transients
//   r ≥ 1.5        GAP    — the 60-2 gap is ≈3; accepted only with the exact
//                           tooth count (process_gap_event)
enum class Edge : uint8_t { NOISE, TOOTH, GAP };

inline Edge classify_edge(uint32_t delta) noexcept {
    const uint64_t d2 = static_cast<uint64_t>(delta) * 2u;
    const uint64_t ref = g_state.ref_ticks;
    ems::drv::g_diag_tn1 = g_state.ref_ticks;
    ems::drv::g_diag_tn2 = g_state.prev_ref_ticks;
    ems::drv::g_diag_delta = delta;
    if (d2 < ref) { return Edge::NOISE; }
    if (d2 >= ref * 3u) { return Edge::GAP; }
    return Edge::TOOTH;
}

inline void drop_sync() noexcept {
    g_state.snap.state = ems::drv::SyncState::LOSS_OF_SYNC;
    g_state.tooth_count = 0u;
    close_cmp_seq_gate();
}

// ── Seção crítica ARM Cortex-M4 ──────────────────────────────────────────────
// CPSID I: mascara todas as interrupções maskable (PRIMASK=1).
// Uso: proteger leitura coerente de g_state.snap pelo main loop (ckp_snapshot).
// A própria ISR CKP (prioridade 1) NÃO usa seção crítica interna.
inline void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

inline void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

// ── Processamento de gap na máquina de estados ───────────────────────────────
// Chamado pela ISR quando period > 1,5 × avg E tooth_count satisfaz a condição.
// Retorna true se o gap foi aceito (transição válida).
// last_tim5_capture já foi escrito pela ISR (linha antes de is_gap) para todos os
// eventos válidos — não precisa ser repetido aqui. process_gap_event foca apenas
// nas transições de estado e nos resets de contagem/índice.
// Advance phase at each accepted gap (=360°).
// If CMP validated since last gap, SET to reference value instead of toggle.
static inline void advance_phase_half() noexcept {
    if (g_state.cmp_phase_pending != 0u) {
        // CMP arrived since last gap — SET to pre-toggle value so that
        // after the XOR below, phase_half lands on kCmpRefHalf.
        g_state.phase_half = g_state.cmp_ref_value;
        g_state.cmp_phase_pending = 0u;
    }
    g_state.phase_half ^= 1u;
    g_state.snap.phase_A = (g_state.phase_half == 0u);
}

// A gap candidate that cannot be a gap here: two in a row means the speed
// fell (re-learn the reference from it); otherwise it is probably the real
// gap at an unknown position — reference = one third of it.
inline void relearn_from_gap(uint32_t delta) noexcept {
    set_reference((g_state.tooth_count == 0u) ? delta : delta / 3u);
    g_state.tooth_count = 0u;
}

// First-gap acquisition window (MS42 accepts 2.5..3.5). In WAIT_GAP nothing
// counts the teeth yet, so only the ratio vouches for the gap: a starter
// pause or a stall-and-restart is ≫ 3 and must not give HALF_SYNC at a random
// tooth (fuel and crank spark would fire off-angle until the next gap).
static constexpr uint32_t kAcqGapMinX2 = 4u;  // ratio ≥ 2.0
static constexpr uint32_t kAcqGapMaxX2 = 8u;  // ratio ≤ 4.0

// Returns true if the gap was accepted (tooth_index restarts at 0).
inline bool process_gap_event(uint32_t delta) noexcept {
    const uint16_t n = g_state.tooth_count;
    switch (g_state.snap.state) {
        case ems::drv::SyncState::WAIT_GAP: {
            if (n < kMinTeethBeforeFirstGap) { relearn_from_gap(delta); return false; }
            const uint64_t d2 = static_cast<uint64_t>(delta) * 2u;
            const uint64_t ref = g_state.ref_ticks;
            if (d2 > ref * kAcqGapMaxX2) {
                // Pause, not a gap: start learning again from the next edge.
                ++ems::drv::g_dbg_gap_acq_reject;
                g_state.ref_ticks = 0u;
                g_state.prev_ref_ticks = 0u;
                g_state.tooth_count = 0u;
                return false;
            }
            if (d2 < ref * kAcqGapMinX2) {
                // Slow tooth (cranking compression), not a gap.
                ++ems::drv::g_dbg_gap_acq_reject;
                set_reference(delta);
                if (g_state.tooth_count < 0xFFFFu) { ++g_state.tooth_count; }
                return false;
            }
            break;
        }
        case ems::drv::SyncState::LOSS_OF_SYNC:
            if (n != kTeethBetweenGaps) { relearn_from_gap(delta); return false; }
            break;
        case ems::drv::SyncState::HALF_SYNC:
            if (n != kTeethBetweenGaps) { drop_sync(); return false; }
            g_state.snap.state = ems::drv::SyncState::FULL_SYNC;
            break;
        case ems::drv::SyncState::FULL_SYNC:
            if (n != kTeethBetweenGaps) {
                // Lost or extra tooth: the angle of every tooth since the last
                // gap is suspect — lose sync instead of firing off-angle.
                ++ems::drv::g_dbg_gap_premature;
                ems::drv::g_dbg_gap_last_tc = n;
                drop_sync();
                return false;
            }
            ++ems::drv::g_dbg_gap_accepted;
            // Fallback CMP-ausente: conta revoluções desde a última borda de came
            // validada. Ultrapassado o limite, o came presume-se perdido → zera
            // cmp_confirms para o agendador reverter a wasted-spark.
            if (s_revs_since_cmp < 0xFFFFu) { ++s_revs_since_cmp; }
            if (s_revs_since_cmp >= max_revs_without_cmp() && g_state.cmp_confirms != 0u) {
                close_cmp_seq_gate();
            }
            break;
    }
    if (g_state.snap.state == ems::drv::SyncState::WAIT_GAP ||
        g_state.snap.state == ems::drv::SyncState::LOSS_OF_SYNC) {
        g_state.snap.state = ems::drv::SyncState::HALF_SYNC;
    }
    set_reference(delta / 3u);   // the gap spans 3 tooth positions
    g_state.tooth_count = 0u;
    g_state.snap.tooth_index = 0u;
    advance_phase_half();
    return true;
}

}  // namespace

// ── Símbolos fracos (hooks) ───────────────────────────────────────────────────
namespace ems::drv {

volatile uint32_t g_dbg_isr_max_ticks = 0u;
volatile uint32_t g_dbg_isr_last_ticks = 0u;
volatile uint32_t g_dbg_tc_gap = 0u;
volatile uint32_t g_dbg_tc_spike = 0u;
volatile uint32_t g_dbg_tc_normal = 0u;
volatile uint32_t g_dbg_gap_accepted = 0u;
volatile uint32_t g_dbg_gap_premature = 0u;
volatile uint32_t g_dbg_gap_acq_reject = 0u;  // first-gap ratio outside 2..4
volatile uint32_t g_dbg_gap_last_tc = 0u;
// Perdas de sync por caminho (protocolo 'D'):
//   wrap      = 58º dente sem gap (gap perdido); avg/delta = ref e Δ nesse instante
//   histogram = ruído persistente (≥4 bordas rejeitadas seguidas) → re-aprende
//   stall     = sem bordas por > timeout
// missing_gap e hist_mn/mx ficam a 0 (caminhos removidos; slots mantidos).
volatile uint32_t g_dbg_loss_missing_gap = 0u;
volatile uint32_t g_dbg_loss_stall = 0u;
volatile uint32_t g_dbg_loss_avg = 0u;
volatile uint32_t g_dbg_loss_delta = 0u;
volatile uint32_t g_dbg_loss_histogram = 0u;
volatile uint32_t g_dbg_loss_wrap = 0u;
volatile uint32_t g_dbg_loss_hist_mn = 0u;
volatile uint32_t g_dbg_loss_hist_mx = 0u;
// Dentes descartados pelo skip pós-silêncio (ckp_skip_pulses_after_gap).
volatile uint32_t g_dbg_skip_after_silence = 0u;

// Instant RPM 360°: timestamp TIM5 do mesmo slot de dente na volta anterior +
// último dt de volta completa (escrito só na ISR; leitura atómica de u32).
static uint32_t g_tooth_rev_ts[58];  // kRealTeethPerRev (definido abaixo)
static volatile uint32_t g_instant_rev_dt_ticks = 0u;

// Osciloscópio CKP/CMP: rings de timestamps TIM5 das bordas cruas (pré-filtro).
// 64 bordas CKP ≈ 1.1 revolução — cobre um gap inteiro; 8 CMP ≈ 4 ciclos de came.
// idx aponta para a PRÓXIMA posição a escrever (a mais antiga do ring).
volatile uint32_t g_scope_ckp_ts[64] = {};
volatile uint8_t  g_scope_ckp_idx = 0u;
volatile uint32_t g_scope_cmp_ts[8] = {};
volatile uint8_t  g_scope_cmp_idx = 0u;

#if defined(__GNUC__)
__attribute__((weak))
#endif
void sensors_on_tooth(const CkpSnapshot& snap) noexcept { static_cast<void>(snap); }

#if defined(__GNUC__)
__attribute__((weak))
#endif
void schedule_on_tooth(const CkpSnapshot& snap) noexcept { static_cast<void>(snap); }

#if defined(__GNUC__)
__attribute__((weak))
#endif
void prime_on_tooth(const CkpSnapshot& snap) noexcept { static_cast<void>(snap); }

#if defined(__GNUC__)
__attribute__((weak))
#endif
void misfire_on_tooth(const CkpSnapshot& snap) noexcept { static_cast<void>(snap); }

}  // namespace ems::drv

// ── API pública ───────────────────────────────────────────────────────────────
namespace ems::drv {

// DIAG: últimos valores de classify_tooth
volatile uint32_t g_diag_tn1 = 0u;
volatile uint32_t g_diag_tn2 = 0u;
volatile uint32_t g_diag_delta = 0u;
volatile uint32_t g_diag_isr_count = 0u;
volatile uint32_t g_diag_hist_ready = 0u;
volatile uint32_t g_diag_tooth_count = 0u;
volatile uint32_t g_diag_consec_anom = 0u;
// Telemetria de diagnóstico de sensores CKP/CMP: bordas cruas (antes de
// qualquer filtro) e timestamp TIM5 da última borda — permitem ver ruído
// (ex.: 60 Hz de rede) e fio partido mesmo com RPM gated a 0.
volatile uint32_t g_diag_cmp_isr_count = 0u;
volatile uint32_t g_diag_last_ckp_edge_tick = 0u;
volatile uint32_t g_diag_last_cmp_edge_tick = 0u;

CkpSnapshot ckp_snapshot() noexcept {
    CkpSnapshot out;
    ems::hal::CriticalSectionGuard guard;
    // memcpy garante cópia byte-a-byte determinística; CriticalSectionGuard
    // desabilita interrupções, impedindo que a ISR TIM5 modifique g_state.snap
    // durante a cópia. Sem volatile + critical section, o compilador poderia
    // reordenar ou cachear leituras de campos individuais de g_state.snap.
    std::memcpy(&out, &g_state.snap, sizeof(out));
    return out;
}

// ── ISR do CKP: TIM5 CH1 (PA0/CKP, rising edge) ─────────────────────────────
//
// CONTEXTO: chamada por TIM5_IRQHandler() em hal/stm32h562/timer.cpp, NVIC prioridade 1.
// Não há chamada direta por código de usuário.
//
// SETUP do TIM5 CH1 no STM32H562:
//   CCMR1.CC1S = 01 -> TI1 input capture
//   CCER.CC1P  = 0  -> rising edge
//   DIER.CC1IE = 1  -> interrupt enable
//   Configurado em hal/stm32h562/timer.cpp → tim5_ic_init() durante boot.
//
// VANTAGEM vs GPIO/EXTI:
//   O periférico TIM5 trava o valor do contador em CnV (≡ TIM5_CKP_CAPTURE) no exato
//   instante da borda de subida, em hardware. A CPU pode atender a IRQ
//   vários ciclos depois — o timestamp em C0V permanece válido.
//   Isso é impossível com GPIO/EXTI onde a CPU leria o contador atual (atrasado).
FASTRUN void ckp_tim5_ch1_isr() noexcept {
    ++g_diag_isr_count;  // DIAG: borda crua, pré-filtro
    g_diag_tooth_count = g_state.tooth_count;
    g_diag_consec_anom = g_state.noise_streak;
    g_diag_hist_ready = (g_state.ref_ticks != 0u) ? 1u : 0u;

    // ── 1. Timestamp de hardware (captura), nunca TIM5_CNT ────────────────
    const uint32_t capture_now = TIM5_CKP_CAPTURE;
    g_diag_last_ckp_edge_tick = capture_now;
    g_scope_ckp_ts[g_scope_ckp_idx] = capture_now;
    g_scope_ckp_idx = static_cast<uint8_t>((g_scope_ckp_idx + 1u) & 63u);

    if (g_state.have_edge == 0u) {           // first edge since reset
        g_state.have_edge = 1u;
        g_state.prev_capture = capture_now;
        g_state.snap.last_tim5_capture = capture_now;
        return;
    }
    // Subtracção circular: correcta através do wrap de 32 bits do TIM5.
    const uint32_t delta_ticks = capture_now - g_state.prev_capture;
    if (delta_ticks < kMinToothTicks) {
        return;  // glitch EMC < 800 ns
    }

    // ── 2. Skip de dentes pós-silêncio (estilo FOME triggerSkipPulses) ────
    // Após ≥ timeout de stall sem bordas, os primeiros N pulsos podem ser
    // transiente do sensor; descarta-os e re-aprende o período.
    if (ems::engine::ckp_skip_pulses_after_gap != 0u) {
        if (delta_ticks >= min_stall_timeout_ticks()) {
            s_skip_remaining = ems::engine::ckp_skip_pulses_after_gap;
        }
        if (s_skip_remaining != 0u) {
            --s_skip_remaining;
            ++g_dbg_skip_after_silence;
            g_state.prev_capture           = capture_now;
            g_state.snap.last_tim5_capture = capture_now;
            g_state.ref_ticks              = 0u;
            g_state.prev_ref_ticks         = 0u;
            g_state.tooth_count            = 0u;
            g_state.noise_streak           = 0u;
            if (is_synced()) {
                drop_sync();
                sensors_on_tooth(g_state.snap);
                schedule_on_tooth(g_state.snap);  // advances unsync clear path
            }
            return;
        }
    }

    // ── 3. Learning: no reference yet ─────────────────────────────────────
    if (g_state.ref_ticks == 0u) {
        g_state.prev_capture = capture_now;
        g_state.snap.last_tim5_capture = capture_now;
        set_reference(delta_ticks);
        g_state.tooth_count = 1u;
        publish_periods();
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 4. Classificação ─────────────────────────────────────────────────
    const Edge edge = classify_edge(delta_ticks);
    if (edge == Edge::NOISE) {
        ++g_dbg_tc_spike;
        // Was the LAST accepted edge the noise? It was if it came short
        // (< 0.9 of a period) and this edge closes one clean period from the
        // edge before it: this is the real tooth, it takes the last edge's
        // place (count/index unchanged) and the reference is restored, so
        // the next tooth is not mistaken for a gap.
        const uint32_t span = capture_now - g_state.prev_prev_capture;
        const uint32_t last = g_state.prev_capture - g_state.prev_prev_capture;
        const uint64_t ref = g_state.prev_ref_ticks != 0u ? g_state.prev_ref_ticks : g_state.ref_ticks;
        if (g_state.prev_prev_capture != 0u &&
            static_cast<uint64_t>(last) * 10u < ref * 9u &&
            static_cast<uint64_t>(span) * 4u >= ref * 3u &&
            static_cast<uint64_t>(span) * 3u <= ref * 4u) {
            g_state.prev_capture = capture_now;
            g_state.snap.last_tim5_capture = capture_now;
            g_state.ref_ticks = span;
            g_state.outlier_ticks = 0u;
            g_state.noise_streak = 0u;
            publish_periods();
            return;
        }
        if (++g_state.noise_streak >= kNoiseResync) {
            // Persistent "noise" = the engine really sped up (or the signal
            // is garbage): re-learn from here; never keep scheduling on it.
            if (is_synced()) {
                ++g_dbg_loss_histogram;
                drop_sync();
                schedule_on_tooth(g_state.snap);
            }
            g_state.prev_capture = capture_now;
            g_state.snap.last_tim5_capture = capture_now;
            g_state.ref_ticks = 0u;
            g_state.prev_ref_ticks = 0u;
            g_state.noise_streak = 0u;
        }
        return;
    }
    g_state.noise_streak = 0u;
    g_state.prev_prev_capture = g_state.prev_capture;
    g_state.prev_capture = capture_now;
    g_state.snap.last_tim5_capture = capture_now;

    if (edge == Edge::GAP) {
        ++g_dbg_tc_gap;
        static_cast<void>(process_gap_event(delta_ticks));
        publish_periods();
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 5. Dente normal ───────────────────────────────────────────────────
    ++g_dbg_tc_normal;
    {
        // Reference = this period when it is within ±25 % of the reference
        // (clean tooth). Outside that band it is either noise that replaced
        // a real edge (one-off: keep the reference) or a real speed change
        // (two consistent outliers in a row: adopt it). Unsynced: always adopt.
        const uint64_t d = delta_ticks;
        const uint64_t ref = g_state.ref_ticks;
        const bool clean = (d * 4u >= ref * 3u) && (d * 3u <= ref * 4u);
        const uint64_t out = g_state.outlier_ticks;
        const bool consistent = out != 0u && (d * 4u >= out * 3u) && (d * 3u <= out * 4u);
        if (!is_synced() || clean || consistent) {
            set_reference(delta_ticks);
            g_state.outlier_ticks = 0u;
        } else {
            g_state.outlier_ticks = delta_ticks;
        }
    }

    // Instant RPM 360°: dt entre o MESMO dente em voltas consecutivas (cancela
    // o erro de geometria da roda). A divisão fica no getter.
    if (is_synced()) {
        const uint16_t ti = g_state.snap.tooth_index;
        if (ti < kRealTeethPerRev) {
            const uint32_t prev_rev_ts = g_tooth_rev_ts[ti];
            g_tooth_rev_ts[ti] = capture_now;
            if (prev_rev_ts != 0u) {
                g_instant_rev_dt_ticks = capture_now - prev_rev_ts;
            }
        }
    }

    if (g_state.tooth_count < 0xFFFFu) { ++g_state.tooth_count; }
    if (is_synced()) {
        if (g_state.snap.tooth_index < kTeethBetweenGaps) {
            ++g_state.snap.tooth_index;
        } else {
            // 58th tooth without a gap: the gap was missed.
            ++ems::drv::g_dbg_loss_wrap;
            ems::drv::g_dbg_loss_avg   = g_state.ref_ticks;
            ems::drv::g_dbg_loss_delta = delta_ticks;
            drop_sync();
        }
    }

    publish_periods();
    sensors_on_tooth(g_state.snap);
    schedule_on_tooth(g_state.snap);
    prime_on_tooth(g_state.snap);
    misfire_on_tooth(g_state.snap);

    {
        const uint32_t elapsed = TIM5_CNT - capture_now;  // ISR duration (diag)
        g_dbg_isr_last_ticks = elapsed;
        if (elapsed > g_dbg_isr_max_ticks) { g_dbg_isr_max_ticks = elapsed; }
    }
}

// ── ISR do cam sensor: TIM5 CH2 (PA1/CMP, rising edge) ──────────────────────
// Cada borda de subida do cam sensor indica meio ciclo de motor (180° de virabrequim).
// phase_A alterna para permitir ao agendador identificar qual par de cilindros está
// no tempo de injeção (cilindros 1/4 vs 2/3 para motor 4 cilindros em linha).
//
// FIX P0 (BUG-11): Validação temporal CMP × CKP — detecta glitches que invertem fase
// Um glitch no CMP pode inverter phase_A silenciosamente, causando ignição/injeção
// no cilindro errado. Esta ISR valida coerência temporal usando o período CKP como
// referência: o período entre bordas CMP deve ser ~2× o período do CKP (CMP = 1 rev,
// CKP gap = 2 rev). Se delta for muito pequeno ou muito grande, é glitch.
FASTRUN void ckp_tim5_ch2_isr() noexcept {
    // Read capture register now — clears CHF flag; value is the TIM5 timestamp
    // of this CMP edge. Must be read before any other logic that might be slow.
    const uint32_t cmp_capture_now = TIM5_CAM_CAPTURE;
    ++g_diag_cmp_isr_count;                       // DIAG: borda crua, pré-validação
    g_diag_last_cmp_edge_tick = cmp_capture_now;  // DIAG: última borda crua
    g_scope_cmp_ts[g_scope_cmp_idx] = cmp_capture_now;
    g_scope_cmp_idx = static_cast<uint8_t>((g_scope_cmp_idx + 1u) & 7u);

    // ── Validação temporal CMP inter-edge (FIX Major #5) ─────────────────
    // Valida o período entre bordas CMP consecutivas contra o período esperado
    // de 58 dentes CKP (= 1 revolução de virabrequim). A verificação anterior
    // (estabilidade do período CKP) não detecta glitches que chegam durante
    // operação steady-state — o período CKP não muda, mas a borda CMP chega cedo.
    // Esta verificação usa a captura TIM5 real para medir o delta entre bordas.
    const uint32_t prev_period_ticks = g_state.ref_ticks;
    // First edge after boot/stall: arm timestamp only — no phase/confirm until
    // a second edge passes the temporal window (floating CMP one-shot).
    if (s_prev_cmp_capture == 0u) {
        s_prev_cmp_capture = cmp_capture_now;
        return;
    }
    if (prev_period_ticks > 0u) {
        const uint32_t cmp_delta = cmp_capture_now - s_prev_cmp_capture; // circular uint32
        // Expected: 2 × 60 × tooth_period (one cam cycle = 720°). Use uint64 —
        // at very low RPM 120 * prev_period overflows uint32 (~35.7e6 ticks).
        constexpr uint32_t kMaxPrevForExpected =
            0xFFFFFFFFu / (2u * kTeethPositionsPerRev);
        if (prev_period_ticks > kMaxPrevForExpected) {
            ++g_state.cmp_glitch_count;
            s_prev_cmp_capture = 0u;  // re-arm as first edge next time
            s_cmp_ref_tooth = 0xFFu;
            return;
        }
        const uint64_t expected = 2ull * kTeethPositionsPerRev *
                                  static_cast<uint64_t>(prev_period_ticks);
        // FIX C10: at cranking/low RPM widen tolerance ±50%; else ±25%.
        constexpr uint32_t kLowRpmThreshTicks = 130000u;  // ~500 RPM @ 62.5 MHz TIM5
        const uint32_t tolerance = (prev_period_ticks > kLowRpmThreshTicks)
                                   ? 2u   // ÷2 → ±50% at low RPM
                                   : 4u;  // ÷4 → ±25% at normal RPM
        const uint64_t min_valid = expected - (expected / tolerance);
        const uint64_t max_valid = expected + (expected / tolerance);
        if (static_cast<uint64_t>(cmp_delta) < min_valid ||
            static_cast<uint64_t>(cmp_delta) > max_valid) {
            ++g_state.cmp_glitch_count;
            // Consecutive rejects → drop dead reference so next edge re-anchors.
            if (++s_cmp_reject_streak >= kCmpRejectResync) {
                s_prev_cmp_capture = 0u;
                s_cmp_ref_tooth = 0xFFu;
                s_cmp_reject_streak = 0u;
            }
            return;
        }
    }
    s_cmp_reject_streak = 0u;  // passou o gate temporal → limpa a contagem de rejeições
    // ── Validação de janela de dente CMP (configurável) ──────────────────
    // Se open != 0 || close != 0 verifica se tooth_index cai dentro da janela.
    // open=0 close=0 → desabilitado (comportamento padrão).
    const uint8_t cmp_open  = ems::engine::cmp_window_open_tooth;
    const uint8_t cmp_close = ems::engine::cmp_window_close_tooth;
    if ((cmp_open != 0u || cmp_close != 0u) &&
        g_state.snap.state == SyncState::FULL_SYNC) {
        const uint16_t ti = g_state.snap.tooth_index;
        const bool in_window = (cmp_open <= cmp_close)
            ? (ti >= cmp_open && ti <= cmp_close)
            : (ti >= cmp_open || ti <= cmp_close);  // janela que envolve o wrap
        if (!in_window) {
            ++g_state.cmp_glitch_count;
            return;
        }
    }

    // ── Consistência de posição do came (auto-ancorada) ──────────────────
    // Uma borda de came real recorre sempre no mesmo tooth_index. Ruído (came
    // desligado, PA1 a flutuar) chega em posições aleatórias: mesmo que passe o
    // gate temporal por acaso, falha aqui → não confirma sync → fica em wasted.
    {
        const uint8_t ti = static_cast<uint8_t>(g_state.snap.tooth_index);
        if (s_cmp_ref_tooth != 0xFFu && s_prev_cmp_capture != 0u) {
            uint8_t diff = (ti >= s_cmp_ref_tooth)
                         ? static_cast<uint8_t>(ti - s_cmp_ref_tooth)
                         : static_cast<uint8_t>(s_cmp_ref_tooth - ti);
            if (diff > (kRealTeethPerRev / 2u)) {
                diff = static_cast<uint8_t>(kRealTeethPerRev - diff);  // wrap na roda
            }
            if (diff > kCmpToothTol) {
                // Salto de posição → não é came coerente. Re-ancora (auto-cura para
                // came real que ligue após ruído) e derruba a confirmação a 0
                // (escolha agressiva: exige 2 bordas coerentes p/ re-sincronizar).
                // TUNING: se surgir "flapping" com came ligado mas ruidoso na
                // bancada, baixar para 1 em vez de 0 (histerese mais suave).
                ++g_state.cmp_glitch_count;
                s_cmp_ref_tooth = ti;
                s_prev_cmp_capture = cmp_capture_now;
                g_state.cmp_confirms = 0u; g_state.snap.cmp_confirms = 0u;
                return;
            }
        }
        s_cmp_ref_tooth = ti;
    }

    s_prev_cmp_capture = cmp_capture_now;
    // Ângulo da borda: dente atual × 6° + fração desde a última borda CKP.
    // Perto do gap o intervalo pode valer até 3 posições → limita a 18°.
    if (g_state.snap.state == SyncState::FULL_SYNC && prev_period_ticks > 0u) {
        const uint32_t since = cmp_capture_now - g_state.snap.last_tim5_capture;
        uint64_t frac_x10 = (static_cast<uint64_t>(since) * 60u) / prev_period_ticks;
        if (frac_x10 > 180u) { frac_x10 = 180u; }
        uint32_t ang = static_cast<uint32_t>(g_state.snap.tooth_index) * 60u +
                       static_cast<uint32_t>(frac_x10);
        ang %= 3600u;
        s_cam_angle_x10 = static_cast<uint16_t>(ang);
        s_cam_edge_seq = s_cam_edge_seq + 1u;
    }
    // CMP validated: defer phase correction to next gap to avoid mid-revolution split.
    // Store pre-toggle value: after XOR in advance_phase_half, result = kCmpRefHalf.
    g_state.cmp_phase_pending = 1u;
    g_state.cmp_ref_value = ems::engine::cfg::kCmpRefHalf ^ 1u;
    if (g_state.cmp_confirms < 2u) {
        ++g_state.cmp_confirms;
    }
    g_state.snap.cmp_confirms = g_state.cmp_confirms;
    // Borda de came validada: zera o contador de fallback CMP-ausente.
    // (glitches saem antes deste ponto → não zeram o contador.)
    s_revs_since_cmp = 0u;
}

bool ckp_stall_poll(uint32_t tim5_cnt_now) noexcept {
    const SyncState state = g_state.snap.state;
    // Subtracção circular em SIGNED: se a ISR capturar um dente entre a leitura
    // de TIM5_CNT no main loop e esta comparação, prev_capture fica À FRENTE de
    // tim5_cnt_now e a subtração unsigned daria ~2^32 → falso stall (a causa
    // real do "false stall triggers sync loss" que desativou este poll em jun).
    // Negativo = captura mais recente que a leitura → obviamente não é stall.
    const int32_t elapsed_signed =
        static_cast<int32_t>(tim5_cnt_now - g_state.prev_capture);
    const uint32_t elapsed_ticks =
        (elapsed_signed < 0) ? 0u : static_cast<uint32_t>(elapsed_signed);
    if (state != SyncState::HALF_SYNC && state != SyncState::FULL_SYNC) {
        // Sem sync o rpm_x10 também é escrito a cada captura (bootstrap/normal)
        // — ruído num CKP desligado deixava RPM fantasma congelado para sempre,
        // bloqueando gates de motor-parado (burn, teste de saídas). Decai a 0
        // após o timeout sem tocar na máquina de sync.
        if (g_state.snap.rpm_x10 != 0u &&
            elapsed_ticks >= min_stall_timeout_ticks()) {
            enter_critical();
            g_state.snap.rpm_x10 = 0u;
            exit_critical();
        }
        return false;
    }
    if (elapsed_ticks < min_stall_timeout_ticks()) {
        return false;
    }
    // Stall confirmado — seção crítica necessária: escrevemos g_state.snap fora
    // da ISR TIM5 (que normalmente tem exclusividade sobre esse campo).
    // Re-verificação dentro da secção evita race com ISR que possa ter disparado
    // no intervalo entre o teste acima e o CPSID.
    enter_critical();
    // Revalida o elapsed com prev_capture fresco: um dente pode ter chegado
    // entre o teste acima e o CPSID (mesma corrida do falso stall).
    const int32_t elapsed_now =
        static_cast<int32_t>(tim5_cnt_now - g_state.prev_capture);
    const bool still_stalled = elapsed_now >= 0 &&
        static_cast<uint32_t>(elapsed_now) >= min_stall_timeout_ticks();
    bool transitioned = false;
    if (still_stalled &&
        (g_state.snap.state == SyncState::HALF_SYNC ||
         g_state.snap.state == SyncState::FULL_SYNC)) {
        ++ems::drv::g_dbg_loss_stall;
        g_state.snap.state   = SyncState::LOSS_OF_SYNC;
        g_state.snap.rpm_x10 = 0u;
        g_state.tooth_count  = 0u;
        close_cmp_seq_gate();
        s_revs_since_cmp = 0u;
        // Instant RPM: motor parado → dt inválido; limpa também os timestamps
        // por dente para o resync não medir contra bordas da sessão anterior.
        g_instant_rev_dt_ticks = 0u;
        for (uint16_t i = 0u; i < kRealTeethPerRev; ++i) {
            g_tooth_rev_ts[i] = 0u;
        }
        transitioned = true;
    }
    exit_critical();
    return transitioned;
}

uint32_t ckp_get_cmp_glitch_count() noexcept {
    return g_state.cmp_glitch_count;
}

uint8_t ckp_get_cmp_ref_tooth() noexcept {
    return s_cmp_ref_tooth;
}

uint32_t ckp_cam_edge_angle(uint16_t& angle_x10) noexcept {
    enter_critical();
    angle_x10 = s_cam_angle_x10;
    const uint32_t seq = s_cam_edge_seq;
    exit_critical();
    return seq;
}

uint32_t ckp_instant_rpm_x10() noexcept {
    const uint32_t dt = g_instant_rev_dt_ticks;
    if (dt == 0u) {
        return 0u;
    }
    // TIM5 62.5 MHz → 3.75e9 ticks/min; ×10 p/ rpm_x10.
    return static_cast<uint32_t>(37500000000ULL / dt);
}

// ── API de teste (host only) ──────────────────────────────────────────────────
#if defined(EMS_HOST_TEST)
void ckp_test_reset() noexcept {
    g_state = DecoderState{};  // zero-init; SyncState::WAIT_GAP == 0
    g_instant_rev_dt_ticks = 0u;
    for (uint16_t i = 0u; i < kRealTeethPerRev; ++i) {
        g_tooth_rev_ts[i] = 0u;
    }
    ems_test_tim5_ccr1   = 0u;
    ems_test_tim5_ccr2   = 0u;
    ems_test_cam_gpio_idr = 0u;
    s_prev_cmp_capture = 0u;
    s_revs_since_cmp = 0u;
    s_cmp_ref_tooth = 0xFFu;
    s_cmp_reject_streak = 0u;
    s_skip_remaining = 0u;
    s_cam_angle_x10 = 0u;
    s_cam_edge_seq = 0u;
}

uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept {
    return rpm_x10_from_period_ns(period_ns);
}
void ckp_test_set_cmp_confirms(uint8_t n) noexcept {
    g_state.cmp_confirms = n;
    g_state.snap.cmp_confirms = n;
}
#endif

}  // namespace ems::drv
