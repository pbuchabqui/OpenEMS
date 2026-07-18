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
 *     4. Calculo de médias → classificação: GAP | NORMAL_TOOTH | NOISE
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
 *     WAIT_GAP     → inicial / pós-falha: aguarda qualquer gap
 *     HALF_SYNC    → 1º gap detectado; contando dentes para confirmar
 *     FULL_SYNC    → 2º gap na posição correta: tooth_index válido
 *     LOSS_OF_SYNC → gap ausente por >61 dentes, ou gap prematuro (<55 dentes)
 *
 *   Transições:
 *     WAIT_GAP     + gap              → HALF_SYNC   (tooth_count reset=0)
 *     HALF_SYNC    + gap, count≥55    → FULL_SYNC   (tooth_index reset=0)
 *     HALF_SYNC    + gap, count<55    → LOSS_OF_SYNC (pulso espúrio)
 *     HALF_SYNC    + count>61         → LOSS_OF_SYNC (gap ausente)
 *     FULL_SYNC    + gap, count≥55    → FULL_SYNC   (gap confirmado, reinicia)
 *     FULL_SYNC    + gap, count<55    → LOSS_OF_SYNC (wheel slip / ruído)
 *     FULL_SYNC    + count>61         → LOSS_OF_SYNC (gap ausente)
 *     LOSS_OF_SYNC + gap              → HALF_SYNC   (tentativa re-sync)
 *
 * FILTRO DINÂMICO ±20% (rejeição de ruído):
 *   Dentes normais aceitos apenas se:  0,8×avg ≤ period ≤ 1,2×avg
 *   Gap identificado por razão:        period × 2 > avg × 3  (≡ period > 1,5×avg)
 *   Período fora das duas faixas:      descartado como ruído (hist não atualizado)
 *
 *   Para roda 60-2: gap ≈ 3×normal → 3,0 >> 1,5: margem 100% acima do limiar.
 *   O filtro ±20% cobre desaceleração típica (<15%/dente no ciclo NEDC).
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
volatile uint32_t ems_test_tim5_ccr1  = 0u;
volatile uint32_t ems_test_tim5_ccr2  = 0u;
volatile uint32_t ems_test_cam_gpio_idr = 0u;
volatile uint32_t ems_test_tim5_cnt   = 0u;
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

// Mínimo de dentes contados desde o último gap para aceitar novo gap.
// 55 << 58: descarta pulsos espúrios no início de cada revolução.
static constexpr uint16_t kGapThresholdTooth  = 55u;

// Máximo de dentes sem gap antes de declarar LOSS_OF_SYNC.
// SCH-04: margem aumentada de 60 para 63 (58 real + 5 de margem).
// A margem anterior de 2 dentes era insuficiente durante desaceleração brusca:
// se o filtro ±20% rejeitar 2 dentes consecutivos como "muito lentos" (período
// crescendo > 20%/dente), o contador de dentes válidos ficaria curto de 58,
// disparando LOSS_OF_SYNC espuriamente ("tropeço" em desaceleração agressiva).
// Com 5 dentes de margem, suporta até 5 rejeições consecutivas por ruído antes
// de declarar perda — cobre condições normais de desaceleração em estrada.
static constexpr uint16_t kMaxTeethBeforeLoss = 63u;

// ── Limiares do filtro ───────────────────────────────────────────────────────
// Detecção de gap por razão:  period × kDen > avg × kNum  ≡  period > 1,5 × avg
// Para gap 60-2 (≈3×normal): separação real ≈ 3,0 >> 1,5 → margem robusta.
static constexpr uint32_t kGapRatioNum = 3u;
static constexpr uint32_t kGapRatioDen = 2u;

// Tolerância ±20% para dentes normais:
//   period ∈ [avg × 4/5, avg × 6/5]  →  dente aceito
//   period fora desta faixa e < limiar de gap  →  ruído, descartado
static constexpr uint32_t kTolNumLow  = 4u;   // 80% = 4/5
static constexpr uint32_t kTolDenLow  = 5u;
static constexpr uint32_t kTolNumHigh = 6u;   // 120% = 6/5
static constexpr uint32_t kTolDenHigh = 5u;

// Preditor conservador para agendamento intra-dente:
// usa a tendência do último período aceito, limitada a ±12,5%/dente.
static constexpr uint32_t kPredictionClampDen = 8u;

// Tamanho da janela deslizante de histórico (em número de períodos).
// 3 amostras são suficientes. A validação pós-bootstrap (secção 4b) deteta e
// rejeita outliers como o dente de gap que podia entrar durante o bootstrap.
static constexpr uint8_t kHistSize = 3u;

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

// ── Limiares TOOTH_GRD (MS42 §1.2.3.1.3, NC_TOOTH_GRD_MIN/MAX_GAP) ──────
// TOOTH_GRD(n) = [T(n) × T(n-2)] / T(n-1)²
//
// Gap válido:  1.5 < TOOTH_GRD ≤ 3.5  (janela que cobre o gap 60-2 ≈ 3.0×)
//   Limite inf:  delta×t_n2×2 > t_n1²×3  (TOOTH_GRD > 3/2)
//   Limite sup:  delta×t_n2×2 > t_n1²×7  → fora da janela → SPIKE_NOISE
//
// Spike/glitch: TOOTH_GRD < 1/4 → delta×t_n2×4 < t_n1²
//
// Usa uint64_t: T(n-1)² estoura uint32_t abaixo de ~650 RPM.
static constexpr uint32_t kGrdGapNum       = 3u;  // gap inf: 3/2 = 1.5
static constexpr uint32_t kGrdGapDen       = 2u;
static constexpr uint32_t kGrdGapMaxNum    = 7u;  // gap sup: 7/2 = 3.5
static constexpr uint32_t kGrdGapMaxDen    = 2u;
static constexpr uint32_t kGrdSpikeDen     = 4u;  // spike:   1/4 = 0.25

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
    uint32_t prev_capture;              // último timestamp TIM5_CKP_CAPTURE (para delta circular)
    uint32_t prev_period_ticks;         // último período normal aceito, para predição dente-a-dente
    uint32_t tooth_hist[kHistSize];     // janela deslizante de períodos (ticks) — dentes normais
    uint8_t  hist_ready;                // quantas entradas válidas em tooth_hist (máx kHistSize)
    uint16_t tooth_count;               // dentes desde o último gap aceito
    uint8_t  cmp_confirms;              // confirmações do cam sensor (CH1)
    uint8_t  phase_half;               // 0/1: which 360° half of the 720° cycle (toggles at each gap)
    uint8_t  cmp_phase_pending;       // 1 = CMP validated, apply kCmpRefHalf at next gap instead of toggle
    uint8_t  cmp_ref_value;           // the value to SET at next gap (= kCmpRefHalf XOR 1, pre-toggle)
    uint32_t cmp_glitch_count;          // FIX P0: contador de glitches CMP rejeitados (diagnóstico)
    uint16_t consecutive_anomalies;     // gaps+spikes seguidos — re-bootstrap se histórico defasar
};

// Após este nº de classificações anómalas SEGUIDAS (gap OU spike) sem nenhum
// dente NORMAL, o histórico é considerado defasado/envenenado e força-se
// re-bootstrap (hist_ready=0) para reaprender o período real. Causas:
//  • mudança de RPM em degrau (período novo ≫ histórico antigo ⇒ tudo vira GAP);
//  • glitch curto que entrou na janela (⇒ tudo vira SPIKE).
// Num 60-2 normal há no máximo 1 gap isolado entre dentes normais, então um
// valor alto nunca ocorre em operação saudável. Sem isto, um período espúrio
// ou um salto de RPM trava o sync para sempre (gap/spike não atualizam histórico).
static constexpr uint16_t kAnomalyResyncThreshold = 60u;

static DecoderState g_state{};  // zero-init; SyncState::WAIT_GAP == 0
// FIX-5: volatile nas variáveis escritas pela ISR TIM5 (prio 1) e lidas pelo
// background loop sem seção crítica. Sem volatile, o compilador pode elevar
// as leituras para fora de loops ou cacheá-las em registradores, observando
// valores desatualizados. volatile força um fresh load de memória a cada acesso.
// NOTA: g_state NÃO é volatile porque é usada diretamente dentro de ISRs,
// onde o acesso volatileness é desnecessário (a ISR não pode ser interrompida
// por si mesma). A proteção de snapshot usa critical section + memcpy.
static volatile bool g_seed_armed = false;
static volatile bool g_seed_phase_a = false;
static volatile bool g_seed_probation = false;
static volatile uint16_t g_seed_probation_teeth = 0u;
static volatile uint32_t g_seed_loaded_count = 0u;
static volatile uint32_t g_seed_confirmed_count = 0u;
static volatile uint32_t g_seed_rejected_count = 0u;
static constexpr uint16_t kSeedCamConfirmMaxTeeth = 70u;

// ── Utilitários inline ────────────────────────────────────────────────────────

// Converte delta de ticks TIM5 para nanossegundos.
// STM32H562 TIM5: 62.5 MHz -> 1 tick = 16 ns.
// Satura em u32 max: ticks > 0x0FFFFFFF (~4.3 s) overflows *16.
inline uint32_t ticks_to_ns(uint32_t ticks) noexcept {
    if (ticks > 0x0FFFFFFFu) { return 0xFFFFFFFFu; }
    return ticks * 16u;
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

inline uint32_t predict_next_period_ticks(uint32_t current_ticks) noexcept {
    const uint32_t prev = g_state.prev_period_ticks;
    if (prev == 0u) { return current_ticks; }
    // Pathological periods (noise / stall residue) must not enter signed math.
    if (current_ticks > 0x7FFFFFFFu || prev > 0x7FFFFFFFu) {
        return current_ticks;
    }

    int32_t trend = static_cast<int32_t>(current_ticks) - static_cast<int32_t>(prev);
    const int32_t limit = static_cast<int32_t>(prev / kPredictionClampDen);
    if (trend > limit) { trend = limit; }
    if (trend < -limit) { trend = -limit; }

    const int32_t predicted = static_cast<int32_t>(current_ticks) + trend;
    return (predicted > 0) ? static_cast<uint32_t>(predicted) : current_ticks;
}

// Insere novo período na janela deslizante (shift FIFO).
// Chamado APENAS para períodos aceitos como dente normal.
inline void hist_push(uint32_t period_ticks) noexcept {
    g_state.tooth_hist[2] = g_state.tooth_hist[1];
    g_state.tooth_hist[1] = g_state.tooth_hist[0];
    g_state.tooth_hist[0] = period_ticks;
    if (g_state.hist_ready < kHistSize) {
        ++g_state.hist_ready;
    }
}

// Média dos períodos no histórico (em ticks).
// Retorna 1 para evitar divisão por zero antes do histórico estar pronto.
inline uint32_t hist_avg() noexcept {
    if (g_state.hist_ready == 0u) { return 1u; }
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < g_state.hist_ready; ++i) {
        sum += g_state.tooth_hist[i];
    }
    return sum / static_cast<uint32_t>(g_state.hist_ready);
}

// Teste de gap por razão (sem divisão — operação pura de multiplicação).
// Equivalente a: period > 1,5 × avg   →   period × 2 > avg × 3
// uint64: periods near stall timeout would wrap uint32 multiplies.
inline bool is_gap(uint32_t period, uint32_t avg) noexcept {
    return (static_cast<uint64_t>(period) * kGapRatioDen >
            static_cast<uint64_t>(avg) * kGapRatioNum);
}

static uint32_t g_prev_valid_period_ns = 0u;
static uint8_t g_coherent_periods_count = 0u;
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

// Teste de dente normal dentro da janela de tolerância ±20%.
// 0,8×avg ≤ period ≤ 1,2×avg → dente aceito para atualização do histórico.
// Multiplica ambos os lados por kTolDen (5) para eliminar as divisões da ISR.
// kTolDenLow == kTolDenHigh == 5, então period×5 substitui period/den em ambas as comparações.
inline bool is_normal_tooth(uint32_t period, uint32_t avg) noexcept {
    const uint64_t p5 = static_cast<uint64_t>(period) * kTolDenLow;
    return (p5 >= static_cast<uint64_t>(avg) * kTolNumLow) &&
           (p5 <= static_cast<uint64_t>(avg) * kTolNumHigh);
}

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

// ── TOOTH_GRD (MS42 §1.2.3.1.3) ──────────────────────────────────────────────
// gap:   delta × t_n2 × kGrdGapDen  > t_n1² × kGrdGapNum
// spike: delta × t_n2 × kGrdSpikeDen < t_n1²
inline bool tooth_grd_is_gap(uint32_t delta, uint32_t t_n1, uint32_t t_n2) noexcept {
    const uint64_t lhs = static_cast<uint64_t>(delta) * t_n2 * kGrdGapDen;
    const uint64_t rhs = static_cast<uint64_t>(t_n1) * t_n1 * kGrdGapNum;
    return lhs > rhs;
}

inline bool tooth_grd_is_spike(uint32_t delta, uint32_t t_n1, uint32_t t_n2) noexcept {
    const uint64_t lhs = static_cast<uint64_t>(delta) * t_n2 * kGrdSpikeDen;
    const uint64_t rhs = static_cast<uint64_t>(t_n1) * t_n1;
    return lhs < rhs;
}

// TOOTH_GRD > 3.5: dente demasiado longo para ser o gap 60-2 (que é ≈3.0×).
// Ocorre durante recuperação de stall ou escorregamento de roda — não é um gap válido.
inline bool tooth_grd_over_gap_max(uint32_t delta, uint32_t t_n1, uint32_t t_n2) noexcept {
    const uint64_t lhs = static_cast<uint64_t>(delta) * t_n2 * kGrdGapMaxDen;
    const uint64_t rhs = static_cast<uint64_t>(t_n1) * t_n1 * kGrdGapMaxNum;
    return lhs > rhs;
}

// Classifica o período atual do dente.
// Com histórico completo (hist_ready == kHistSize) usa TOOTH_GRD — robusto a
// aceleração/desaceleração rápida e rejeita spikes bidirecionais.
// No bootstrap (hist_ready < kHistSize) recai em is_gap / is_normal_tooth.
enum class ToothClass : uint8_t { GAP, SPIKE_NOISE, NORMAL };

inline ToothClass classify_tooth(uint32_t delta_ticks) noexcept {
    // Classificador por razão de média (equivalente Speeduino/rusEFI ratio).
    // GAP:   delta × 2 > avg × 3  (delta > 1,5× média)
    // NORMAL: avg × 0,8 ≤ delta ≤ avg × 1,2
    // SPIKE: fora da janela normal mas não gap
    //
    // NOTA: NÃO usa TOOTH_GRD (Δ×t_n2 vs t_n1²). O TOOTH_GRD depende de t_n1 e
    // t_n2 estarem limpos, mas ringing e/ou re-bootstrap contaminam-nos, fazendo
    // com que o gap 3× passe despercebido. A razão de média é mais robusta a
    // contaminação porque usa TODOS os valores do histórico.
    const uint32_t avg = hist_avg();
    ems::drv::g_diag_tn1 = avg;
    ems::drv::g_diag_tn2 = g_state.tooth_hist[0];
    ems::drv::g_diag_delta = delta_ticks;
    if (is_gap(delta_ticks, avg))          { return ToothClass::GAP; }
    if (!is_normal_tooth(delta_ticks, avg)) { return ToothClass::SPIKE_NOISE; }
    return ToothClass::NORMAL;
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

// Valida se período CKP é coerente com rotação forward estável
// Períodos coerentes: variação < 25% entre amostras consecutivas
inline bool is_forward_rotation_coherent(uint32_t period_ns) noexcept {
    if (period_ns == 0u || period_ns > 10000000u) {  // > 10ms = RPM < 100
        return false;
    }

    if (g_prev_valid_period_ns == 0u) {
        g_prev_valid_period_ns = period_ns;
        g_coherent_periods_count = 1u;
        return true;
    }

    // Verifica se variação está dentro de ±25% (rotação estável forward)
    const uint32_t max_valid = g_prev_valid_period_ns + (g_prev_valid_period_ns >> 2u);
    const uint32_t min_valid = g_prev_valid_period_ns - (g_prev_valid_period_ns >> 2u);

    if (period_ns >= min_valid && period_ns <= max_valid) {
        g_prev_valid_period_ns = period_ns;
        if (g_coherent_periods_count < 255u) {
            ++g_coherent_periods_count;
        }
        // Requer 3 períodos coerentes consecutivos para validar forward rotation
        return g_coherent_periods_count >= 3u;
    } else {
        // Variação brusca: possível reversão ou ruído
        g_prev_valid_period_ns = period_ns;
        g_coherent_periods_count = 1u;
        return false;
    }
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

inline bool process_gap_event() noexcept {
    switch (g_state.snap.state) {

        case ems::drv::SyncState::WAIT_GAP:
            // WAIT_GAP: primeiro gap aceite com count ≥ kHistSize (bootstrap minimo).
            // Após bootstrap, o classificador ja tem media valida. O gap 3× e o
            // UNICO evento com esta razao neste estado — baixo risco de falso gap.
            if (g_state.tooth_count < kHistSize) {
                g_state.tooth_count = 0u;
                return false;
            }
            goto sync_transition_common;

        case ems::drv::SyncState::LOSS_OF_SYNC:
            // LOSS_OF_SYNC: exigir kGapThresholdTooth para evitar re-sync falso
            // apos perda por ruido (gap espurio seguido de poucos dentes).
            if (g_state.tooth_count < kGapThresholdTooth) {
                g_state.tooth_count = 0u;
                return false;
            }
            goto sync_transition_common;

        sync_transition_common:
            // FIX 2026-06-29: seed desativado p/ diagnóstico. O seed armado pela
            // NVM pode impedir sync se is_forward_rotation_coherent nunca retornar
            // true devido a contaminação do tooth_period_ns.
            // TODO: re-activar seed (via is_forward_rotation_coherent) quando
            //       a classificação de dentes estiver robusta.
            g_state.snap.state = ems::drv::SyncState::HALF_SYNC;
            g_state.tooth_count      = 0u;
            g_state.snap.tooth_index = 0u;
            // RPM é gated por sync (rpm_if_synced): popula já na transição com
            // o último período de dente normal, senão ficaria 0 até ao próximo.
            g_state.snap.rpm_x10 = rpm_x10_from_period_ticks(g_state.prev_period_ticks);
            advance_phase_half();
            return true;

        case ems::drv::SyncState::HALF_SYNC:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                // 2º gap na posição correta → sincronismo completo.
                g_state.snap.state       = ems::drv::SyncState::FULL_SYNC;
                g_state.tooth_count      = 0u;
                g_state.snap.tooth_index = 0u;
                advance_phase_half();
                return true;
            }
            // Gap prematuro: pulso espúrio (EMC, dente danificado).
            // Preserva CMP state — perda de sync do CKP não invalida o came —
            // mas fecha o gate sequencial exportado até 1 borda CMP fresca
            // pós-resync (o ISR do came restaura snap.cmp_confirms e re-ancora
            // a fase via cmp_phase_pending): evita retomar sequencial 360° fora.
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            close_cmp_seq_gate();
            return false;

        case ems::drv::SyncState::FULL_SYNC:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                g_state.tooth_count      = 0u;
                g_state.snap.tooth_index = 0u;
                advance_phase_half();
                ++ems::drv::g_dbg_gap_accepted;
                // Fallback CMP-ausente: conta revoluções desde a última borda de came
                // validada. Ultrapassado o limite, o came presume-se perdido → zera
                // cmp_confirms para o agendador reverter a wasted-spark (o gate lê
                // cmp_confirms>=2). Não toca em s_prev_cmp_capture: uma futura borda
                // re-valida temporalmente e reconstrói a confirmação do zero.
                if (s_revs_since_cmp < 0xFFFFu) { ++s_revs_since_cmp; }
                if (s_revs_since_cmp >= max_revs_without_cmp() && g_state.cmp_confirms != 0u) {
                    close_cmp_seq_gate();
                }
                return true;
            }
            ++ems::drv::g_dbg_gap_premature;
            ems::drv::g_dbg_gap_last_tc = g_state.tooth_count;
            // Gap prematuro em FULL_SYNC → LOSS_OF_SYNC; re-require 2 CMP edges.
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            close_cmp_seq_gate();
            return false;

        default:
            return false;
    }
}

}  // namespace

// ── Símbolos fracos (hooks) ───────────────────────────────────────────────────
namespace ems::drv {

volatile uint32_t g_dbg_isr_max_ticks = 0u;
volatile uint32_t g_dbg_isr_last_ticks = 0u;
volatile uint32_t g_dbg_tc_gap = 0u;
volatile uint32_t g_dbg_tc_spike = 0u;
volatile uint32_t g_dbg_tc_normal = 0u;
volatile uint32_t g_dbg_bootstrap_reject = 0u;
volatile uint32_t g_dbg_hist_ready_max = 0u;
volatile uint32_t g_dbg_gap_accepted = 0u;
volatile uint32_t g_dbg_gap_premature = 0u;
volatile uint32_t g_dbg_gap_last_tc = 0u;
// Perdas de sync por caminho: gap ausente (tooth_count > kMaxTeethBeforeLoss)
// vs stall watchdog. avg/delta capturados no instante da perda por gap ausente
// — média inflada = histórico contaminado; delta anômalo = borda distorcida.
volatile uint32_t g_dbg_loss_missing_gap = 0u;
volatile uint32_t g_dbg_loss_stall = 0u;
volatile uint32_t g_dbg_loss_avg = 0u;
volatile uint32_t g_dbg_loss_delta = 0u;
// Discriminação dos 3 gatilhos de perda de FULL_SYNC (sem debounce hoje):
//   histogram = gate de dispersão do hist (mx > 1.5×mn) — re-bootstrap + drop
//   wrap      = tooth_index chegou a 57 sem gap aceite (gap classificado normal)
//   (o overrun tooth_count>kMaxTeethBeforeLoss continua em g_dbg_loss_missing_gap)
// hist_mn/hist_mx capturam o par min/max do último trip de histograma:
//   mx≈1.5×mn → gate no limiar (dispersão real de roda) → candidato a relaxar;
//   mx≫mn      → contaminação grosseira (dente perdido real) → drop correto.
volatile uint32_t g_dbg_loss_histogram = 0u;
volatile uint32_t g_dbg_loss_wrap = 0u;
volatile uint32_t g_dbg_loss_hist_mn = 0u;
volatile uint32_t g_dbg_loss_hist_mx = 0u;

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
    ++g_diag_isr_count;  // DIAG: incrementa em cada ISR (borda crua, pré-filtro)

    // snapshot estado interno para diagnóstico
    g_diag_hist_ready = g_state.hist_ready;
    g_diag_tooth_count = g_state.tooth_count;
    g_diag_consec_anom = g_state.consecutive_anomalies;

    // ── 1. Timestamp sem jitter de ISR ────────────────────────────────────
    // CRÍTICO: lemos TIM5_CKP_CAPTURE ANTES de qualquer outra operação.
    // O registrador de captura foi travado pelo HW no instante exato da borda;
    // leituras posteriores (CKP_CAM_GPIO_IDR, etc.) não afetam o valor capturado.
    // NÃO ler TIM5_CNT: o contador avançou durante a latência de IRQ.
    const uint32_t capture_now = TIM5_CKP_CAPTURE;
    g_diag_last_ckp_edge_tick = capture_now;  // DIAG: última borda crua
    g_scope_ckp_ts[g_scope_ckp_idx] = capture_now;
    g_scope_ckp_idx = static_cast<uint8_t>((g_scope_ckp_idx + 1u) & 63u);

    // -- 2. Delta de ticks (aritmetica circular uint32_t) -------------------
    // Subtracao circular: correta mesmo se o contador passou por 0xFFFFFFFF -> 0.
    const uint32_t delta_ticks = capture_now - g_state.prev_capture;

    // ── 3. Anti-glitch por período mínimo ────────────────────────────────
    // Estrategia preferida a checar GPIO depois da captura:
    // o pino pode ter retornado LOW antes da CPU ler o registrador de GPIO,
    // descartando capturas legítimas em alta rotação (> 4000 RPM).
    // usamos 50 ticks como limite inferior conservador para rejeitar glitches
    // de EMC (< 800 ns) sem afetar operacao normal.
    if (delta_ticks < kMinToothTicks) {
        return;  // pulso muito curto → ruído EMC, descartar
    }

    g_state.prev_capture                = capture_now;
    g_state.snap.last_tim5_capture      = capture_now;

    const uint32_t period_ns = ticks_to_ns(delta_ticks);

    // ── 4. Construção do histórico (primeiros kHistSize dentes) ───────────
    // Antes do histórico estar completo, aceitamos todos os dentes
    // incondicionalmente para inicializar a janela de média.
    // O primeiro dente (prev_period_ticks==0) é SEMPRE aceite — o filtro
    // delta <= prev_period*2 = 0 rejeitaria sempre o 1º dente.
    if (g_state.hist_ready < kHistSize) {
        if (g_state.prev_period_ticks == 0u || delta_ticks <= g_state.prev_period_ticks * 2u) {
            hist_push(delta_ticks);
        } else {
            extern volatile uint32_t g_dbg_bootstrap_reject;
            ++g_dbg_bootstrap_reject;
        }
        {
            extern volatile uint32_t g_dbg_hist_ready_max;
            if (g_state.hist_ready > g_dbg_hist_ready_max) g_dbg_hist_ready_max = g_state.hist_ready;
        }
        ++g_state.tooth_count;
        g_state.snap.tooth_period_ns = period_ns;
        g_state.snap.predicted_tooth_period_ns = period_ns;
        // RPM só é reportado com referência angular (HALF/FULL_SYNC) — estilo
        // Speeduino/rusEFI. Ruído periódico (ex.: 60 Hz de rede num CKP
        // flutuante = 60 RPM exactos) nunca produz gap e ficava reportado
        // como RPM real, bloqueando gates de motor-parado.
        g_state.snap.rpm_x10 = rpm_if_synced(delta_ticks);
        g_state.prev_period_ticks = delta_ticks;
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 4b. Validação de consistência do histograma pós-bootstrap ────────
    // Após bootstrap, verificar se os valores em tooth_hist são coerentes.
    // Se um gap entrou no hist durante o bootstrap, o TOOTH_GRD compara
    // dentes normais contra ele e classifica-os como GAP/SPIKE — nunca
    // se atinge FULL_SYNC.
    {
        const uint32_t h0 = g_state.tooth_hist[0];
        const uint32_t h1 = g_state.tooth_hist[1];
        const uint32_t h2 = g_state.tooth_hist[2];
        const uint32_t mn = (h0 < h1) ? ((h0 < h2) ? h0 : h2) : ((h1 < h2) ? h1 : h2);
        const uint32_t mx = (h0 > h1) ? ((h0 > h2) ? h0 : h2) : ((h1 > h2) ? h1 : h2);
        // mx > mn×1.5: tolerância de 50% cobre gap 3T enquanto rejeita t_n2
        // contaminado (ex: 1.6T vs T normal que causa falso GAP no TOOTH_GRD)
        if (mn > 0u && mx > mn + mn / 2u) {
            // Hist contaminado → re-bootstrap. Drop sync first so schedule_on_tooth
            // cannot re-arm inject/ign on a stale tooth_index while HALF/FULL.
            g_state.hist_ready = 0u;
            g_state.tooth_count = 0u;
            g_state.consecutive_anomalies = 0u;
            if (g_state.snap.state == ems::drv::SyncState::HALF_SYNC ||
                g_state.snap.state == ems::drv::SyncState::FULL_SYNC) {
                // Só conta como perda de sync se estava sincronizado — bootstrap
                // re-boot não é blip de PW. Captura mn/mx do trip p/ decidir gate.
                ++ems::drv::g_dbg_loss_histogram;
                ems::drv::g_dbg_loss_hist_mn = mn;
                ems::drv::g_dbg_loss_hist_mx = mx;
                g_state.snap.state = ems::drv::SyncState::LOSS_OF_SYNC;
                close_cmp_seq_gate();
            }
            sensors_on_tooth(g_state.snap);
            schedule_on_tooth(g_state.snap);  // advances unsync clear path
            return;
        }
    }

    // ── 5. Classificação do dente ─────────────────────────────────────────
    // Com histórico completo usa TOOTH_GRD (MS42 §1.2.3.1.3) — trend-compensado,
    // robusto a aceleração/desaceleração rápida e rejeita spikes de ambos os
    // sentidos. No bootstrap (hist_ready < kHistSize) recai em razão de média.
    const ToothClass tc = classify_tooth(delta_ticks);
    {
        extern volatile uint32_t g_dbg_tc_gap, g_dbg_tc_spike, g_dbg_tc_normal;
        if (tc == ToothClass::GAP) ++g_dbg_tc_gap;
        else if (tc == ToothClass::SPIKE_NOISE) ++g_dbg_tc_spike;
        else ++g_dbg_tc_normal;
    }
    switch (tc) {

        case ToothClass::GAP:
            // Auto-recuperação: muitos gaps seguidos ⇒ histórico defasado (ex.:
            // salto de RPM). Re-bootstrap reaprende o período; senão trava.
            if (++g_state.consecutive_anomalies >= kAnomalyResyncThreshold) {
                g_state.hist_ready         = 0u;
                g_state.tooth_count        = 0u;
                g_state.consecutive_anomalies = 0u;
                g_state.snap.state         = ems::drv::SyncState::WAIT_GAP;
                // Drop sync → re-require 2 CMP edges before sequential.
                close_cmp_seq_gate();
                sensors_on_tooth(g_state.snap);
                schedule_on_tooth(g_state.snap);
                return;
            }
            static_cast<void>(process_gap_event());
            sensors_on_tooth(g_state.snap);
            schedule_on_tooth(g_state.snap);
            return;

        case ToothClass::SPIKE_NOISE:
            // Ruído: não atualiza hist nem tooth_count. NÃO re-agenda inj/ign —
            // tooth_index não avança; schedule_on_tooth rearmaria o mesmo dente
            // (multi-pulse em ruído enquanto HALF/FULL).
            if (++g_state.consecutive_anomalies >= kAnomalyResyncThreshold) {
                g_state.hist_ready         = 0u;
                g_state.tooth_count        = 0u;
                g_state.consecutive_anomalies = 0u;
                g_state.snap.state         = ems::drv::SyncState::WAIT_GAP;
                close_cmp_seq_gate();
                sensors_on_tooth(g_state.snap);
                schedule_on_tooth(g_state.snap);  // only on resync drop
                return;
            }
            sensors_on_tooth(g_state.snap);
            return;

        case ToothClass::NORMAL:
            // DECAI (não zera): num salto de RPM de 2.5-3.75× o GAP real cai
            // na banda "normal" da média defasada e zerava o contador a cada
            // 58 dentes — 57 spikes consecutivos nunca chegavam ao threshold
            // de 60 e o re-bootstrap NUNCA disparava (deadlock eterno, CKP
            // perfeito rejeitado; visto em bancada no salto 800→3000 RPM).
            // Com decaimento -2: tempestade acumula +56/volta → reset em ~2
            // voltas; spike esporádico em operação sã decai sem re-bootstrap.
            g_state.consecutive_anomalies =
                (g_state.consecutive_anomalies >= 2u)
                    ? static_cast<uint16_t>(g_state.consecutive_anomalies - 2u)
                    : 0u;
            break;
    }

    // ── 6. Processamento de dente normal ──────────────────────────────────
    const uint32_t predicted_ticks = predict_next_period_ticks(delta_ticks);
    hist_push(delta_ticks);

    g_state.snap.tooth_period_ns = period_ns;
    g_state.snap.predicted_tooth_period_ns = ticks_to_ns(predicted_ticks);
    g_state.snap.rpm_x10 = rpm_if_synced(delta_ticks);
    g_state.prev_period_ticks = delta_ticks;

    // tooth_count incrementa em todos os estados — alimenta o limiar de gap e
    // a detecção de LOSS_OF_SYNC por excesso de dentes sem gap.
    ++g_state.tooth_count;
    // tooth_index só avança quando há referência angular (HALF_SYNC / FULL_SYNC).
    // Wrap 57→0 sem gap aceite é falso 2º meio-ciclo (ruído classificado normal);
    // força LOSS em vez de re-agendar dentes 0..N outra vez na mesma volta.
    if (g_state.snap.state != ems::drv::SyncState::WAIT_GAP &&
        g_state.snap.state != ems::drv::SyncState::LOSS_OF_SYNC) {
        if (g_state.snap.tooth_index < (kRealTeethPerRev - 1u)) {
            g_state.snap.tooth_index =
                static_cast<uint16_t>(g_state.snap.tooth_index + 1u);
        } else {
            // WRAP: gap real classificado NORMAL → tooth_index chegou a 57.
            // Distinto do overrun (tooth_count>kMaxTeethBeforeLoss) abaixo.
            ++ems::drv::g_dbg_loss_wrap;
            ems::drv::g_dbg_loss_avg   = hist_avg();
            ems::drv::g_dbg_loss_delta = delta_ticks;
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            close_cmp_seq_gate();
        }
    }

    // ── 7. Verificação de perda de sincronia por contagem excessiva ───────
    // Se passaram mais de kMaxTeethBeforeLoss dentes sem um gap:
    //   → o gap foi perdido (interferência, aceleração brusca, falha de sensor)
    // OVERRUN: g_dbg_loss_missing_gap conta APENAS este caminho (o wrap 57→0
    // migrou para g_dbg_loss_wrap acima).
    if (g_state.tooth_count > kMaxTeethBeforeLoss) {
        if (g_state.snap.state == ems::drv::SyncState::HALF_SYNC ||
            g_state.snap.state == ems::drv::SyncState::FULL_SYNC) {
            // Gap ausente → LOSS_OF_SYNC; re-require 2 CMP edges for sequential.
            ++ems::drv::g_dbg_loss_missing_gap;
            ems::drv::g_dbg_loss_avg   = hist_avg();
            ems::drv::g_dbg_loss_delta = delta_ticks;
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            close_cmp_seq_gate();
        }
    }

    // ── 8. Hooks ──────────────────────────────────────────────────────────
    if (g_seed_probation) {
        ++g_seed_probation_teeth;
        if (g_seed_probation_teeth > kSeedCamConfirmMaxTeeth) {
            // Seed could not be validated by cam edge in time: fallback to safe sync path.
            g_seed_probation = false;
            g_seed_probation_teeth = 0u;
            ++g_seed_rejected_count;
            g_state.snap.state = ems::drv::SyncState::HALF_SYNC;
            g_state.tooth_count = 0u;
            g_state.snap.tooth_index = 0u;
        }
    }
    sensors_on_tooth(g_state.snap);
    schedule_on_tooth(g_state.snap);
    prime_on_tooth(g_state.snap);
    misfire_on_tooth(g_state.snap);

    // Measure ISR duration (TIM5 free-running counter)
    // Usamos TIM5_CNT para tempo decorrido (não TIM5_CCR1 que é o timestamp da borda)
    {
        extern volatile uint32_t g_dbg_isr_max_ticks;
        extern volatile uint32_t g_dbg_isr_last_ticks;
        const uint32_t elapsed = TIM5_CNT - capture_now;
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
    const uint32_t prev_period_ticks = g_state.prev_period_ticks;
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
    // CMP validated: defer phase correction to next gap to avoid mid-revolution split.
    // Store pre-toggle value: after XOR in advance_phase_half, result = kCmpRefHalf.
    g_state.cmp_phase_pending = 1u;
    g_state.cmp_ref_value = ems::engine::cfg::kCmpRefHalf ^ 1u;
    if (g_seed_probation) {
        g_seed_probation = false;
        g_seed_probation_teeth = 0u;
        ++g_seed_confirmed_count;
    }
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
        transitioned = true;
    }
    exit_critical();
    return transitioned;
}

void ckp_seed_arm(bool phase_A) noexcept {
    g_seed_armed = true;
    g_seed_phase_a = phase_A;
    ++g_seed_loaded_count;
}

void ckp_seed_disarm() noexcept {
    g_seed_armed = false;
}

uint32_t ckp_seed_loaded_count() noexcept {
    return g_seed_loaded_count;
}

uint32_t ckp_seed_confirmed_count() noexcept {
    return g_seed_confirmed_count;
}

uint32_t ckp_seed_rejected_count() noexcept {
    return g_seed_rejected_count;
}

uint32_t ckp_get_cmp_glitch_count() noexcept {
    return g_state.cmp_glitch_count;
}

uint8_t ckp_get_cmp_ref_tooth() noexcept {
    return s_cmp_ref_tooth;
}

// ── API de teste (host only) ──────────────────────────────────────────────────
#if defined(EMS_HOST_TEST)
void ckp_test_reset() noexcept {
    g_state = DecoderState{};  // zero-init; SyncState::WAIT_GAP == 0
    ems_test_tim5_ccr1   = 0u;
    ems_test_tim5_ccr2   = 0u;
    ems_test_cam_gpio_idr = 0u;
    g_seed_armed = false;
    g_seed_phase_a = false;
    g_seed_probation = false;
    g_seed_probation_teeth = 0u;
    g_seed_loaded_count = 0u;
    g_seed_confirmed_count = 0u;
    g_seed_rejected_count = 0u;
    g_prev_valid_period_ns = 0u;
    g_coherent_periods_count = 0u;
    s_prev_cmp_capture = 0u;
    s_revs_since_cmp = 0u;
    s_cmp_ref_tooth = 0xFFu;
    s_cmp_reject_streak = 0u;
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
