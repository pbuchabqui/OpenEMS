/**
 * @file drv/ckp.cpp
 * @brief Módulo 1 (DECODE) + Módulo 2 (SYNC) — Engine Position Core — OpenEMS
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * MÓDULO 1: DECODE via FTM Input Capture (Crank)
 * ───────────────────────────────────────────────
 *   Hardware: FTM3 Canal 0 (PTD0), rising edge capture.
 *             Em targets embarcados o FTM3 é mapeado via hal/ftm.cpp.
 *             Em host tests, FTM3_C0V é um volatile mock.
 *
 *   Fluxo da ISR (ckp_ftm3_ch0_isr):
 *     1. Verificação anti-glitch: pino PTD0 ainda HIGH? (não é noise falling edge)
 *     2. Leitura de FTM3_C0V (registrador de captura — travado pelo HW)
 *        ► NÃO lemos FTM3_CNT: o contador avançou enquanto a CPU atendia a IRQ.
 *          FTM3_C0V contém o timestamp EXATO da borda de subida (RusEFI #1488).
 *     3. delta_ticks = (uint16_t)(capture_now - prev_capture)  ← aritmética circular
 *        Correto mesmo em overflow do contador de 16 bits (≈1,09 ms @ 60 MHz).
 *     4. Conversão para nanossegundos: period_ns = (delta_ticks × 16667) / 1000
 *        FTM3: 120 MHz / prescaler 2 = 60 MHz → 16,667 ns/tick
 *     5. Cálculo de médias → classificação: GAP | NORMAL_TOOTH | NOISE
 *     6. Atualização da máquina de estados (Módulo 2)
 *     7. Disparo dos hooks sensors_on_tooth() / schedule_on_tooth()
 *
 * VANTAGEM DO INPUT CAPTURE vs GPIO/EXTI:
 *   O periférico FTM registra o timestamp da borda em hardware no exato
 *   instante do evento, independente do atraso de atendimento da IRQ
 *   (tipicamente 12–20 ciclos = 0,1–0,17 µs @ 120 MHz no Cortex-M4).
 *   A 6000 RPM, 0,2 µs de jitter ≈ 0,07° — inaceitável sem input capture.
 *   Com input capture: resolução = 1 tick = 16,67 ns ≈ 0,006° @ 6000 RPM.
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

#if __has_include("hal/ftm.h")
#include "hal/ftm.h"
#elif __has_include("ftm.h")
#include "ftm.h"
#endif

// ── Remapeamento de registradores para STM32H562 ─────────────────────────────
// FTM3_C0V → TIM5_CCR1 (0x40000C34), FTM3_C1V → TIM5_CCR2 (0x40000C38)
// GPIOD_PDIR → GPIOA_IDR (0x42020010)
// TIM5 @ 62.5 MHz (250 MHz / prescaler 4) → 16 ns/tick
#define TICKS_TO_NS_FACTOR  16000u
#define TICKS_TO_NS_DIVISOR 1000u

// ── Mock de registradores para testes host ───────────────────────────────────
#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_ftm3_c0v  = 0u;
volatile uint32_t ems_test_ftm3_c1v  = 0u;
volatile uint32_t ems_test_gpiod_pdir = 0u;
#endif

// ── FIX-15: FASTRUN — coloca ISRs críticas em SRAM (zero cache miss) ─────────
// Em Teensyduino, FASTRUN = __attribute__((section(".fastrun"))) — definido em
// WProgram.h / core_pins.h. Em host tests a macro é indefinida; defini-la vazia
// garante que o código compile sem modificações.
#if !defined(FASTRUN)
#  if defined(TARGET_STM32H562) && !defined(EMS_HOST_TEST)
#    define FASTRUN  __attribute__((optimize("O3")))
#  else
#    define FASTRUN
#  endif
#endif

namespace {

// ── Constantes da roda fônica 60-2 ───────────────────────────────────────────

// 60-2: 60 posições, 2 dentes ausentes consecutivos = 58 dentes reais.
// Espaçamento por posição: 360°/60 = 6,0°.
// Gap: 3 posições ausentes × 6° = 18° ≈ 3× período normal.
static constexpr uint16_t kRealTeethPerRev    = 58u;  // dentes físicos — uso exclusivo da máquina de estados (gap detection)
static constexpr uint16_t kTeethPositionsPerRev = 60u; // posições angulares uniformes — uso em cálculos de RPM e ângulo

// Ângulo por dente normal em miligraus (× 1000).
// 6,0° × 1000 = 6000. Usado em ckp_angle_to_ticks().
// NOTA: o nome kToothAngleX1000 sugere ×1000, que é "miligraus".
//   angle_x10 passado à função é TAMBÉM em miligraus (não em ×10 como o nome diz).
static constexpr uint16_t kToothAngleX1000 = 6000u;

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

// Tamanho da janela deslizante de histórico (em número de períodos).
// 3 amostras são suficientes para filtrar ruído transitório.
static constexpr uint8_t kHistSize = 3u;

// ── Acesso a registradores FTM3 ──────────────────────────────────────────────
// FTM3 Base: 0x400B9000 (K64P144M120SF5 RM, Table 3-1)
// CnSC CH0:  base + 0x0C + 0*8 = 0x400B900C  (Status and Control — §43.3.5)
// CnV  CH0:  base + 0x0C + 0*8 + 4 = 0x400B9010 (Channel Value — §43.3.6)
//
// CRÍTICO: Lemos FTM3_C0V (registrador de CAPTURA travado pelo hardware),
//   não FTM3_CNT (contador livre que avançou durante o atendimento da ISR).
//   Esta distinção elimina o jitter de software: o valor em C0V reflete o
//   exato instante da borda de subida, independente da latência da IRQ.
//   Referência: RusEFI issue #1488 ("timestamp corruption from CNT vs CnV").
#if defined(EMS_HOST_TEST)
#define FTM3_C0V      ems_test_ftm3_c0v
#define FTM3_C1V      ems_test_ftm3_c1v
#define GPIOD_PDIR    ems_test_gpiod_pdir
#else
// STM32H562: TIM5_CCR1 (TIM5 base 0x40000C00, CCR1 offset 0x34)
#define FTM3_C0V      (*reinterpret_cast<volatile uint32_t*>(0x40000C34u))
// STM32H562: TIM5_CCR2 (CCR2 offset 0x38)
#define FTM3_C1V      (*reinterpret_cast<volatile uint32_t*>(0x40000C38u))
// GPIOA_IDR (GPIOA base 0x42020000, IDR offset 0x10) — verificação anti-glitch PA0
#define GPIOD_PDIR    (*reinterpret_cast<volatile uint32_t*>(0x42020010u))
#endif

// ── Estado interno do decodificador ──────────────────────────────────────────
//
// INVARIANTE DE ACESSO — NUNCA VIOLAR:
//   g_state é escrito EXCLUSIVAMENTE pela ISR ckp_ftm3_ch0_isr() (prioridade 1).
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
    uint16_t prev_capture;              // último timestamp FTM3_C0V (para delta circular)
    uint32_t tooth_hist[kHistSize];     // janela deslizante de períodos (ticks) — dentes normais
    uint8_t  hist_ready;                // quantas entradas válidas em tooth_hist (máx kHistSize)
    uint16_t tooth_count;               // dentes desde o último gap aceito
    uint8_t  cmp_confirms;              // confirmações do cam sensor (CH1)
};

static DecoderState g_state = {
    ems::drv::CkpSnapshot{0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false},
    0u,
    {0u, 0u, 0u},
    0u,
    0u,
    0u,
};
// FIX-5: volatile nas variáveis escritas pela ISR FTM3 (prio 1) e lidas pelo
// background loop sem seção crítica. Sem volatile, o compilador pode elevar
// as leituras para fora de loops ou cacheá-las em registradores, observando
// valores desatualizados. volatile força um fresh load de memória a cada acesso.
static volatile bool g_seed_armed = false;
static volatile bool g_seed_phase_a = false;
static volatile bool g_seed_probation = false;
static volatile uint16_t g_seed_probation_teeth = 0u;
static volatile uint32_t g_seed_loaded_count = 0u;
static volatile uint32_t g_seed_confirmed_count = 0u;
static volatile uint32_t g_seed_rejected_count = 0u;
static constexpr uint16_t kSeedCamConfirmMaxTeeth = 70u;

// ── Utilitários inline ────────────────────────────────────────────────────────

// Converte delta de ticks FTM3 para nanossegundos.
// FTM3: 120 MHz / prescaler 2 = 60 MHz → 1 tick = 16,667 ns
// Evita float: (ticks × 16667) / 1000  (equivalente exato a ticks × 16.667 ns)
// Análise de overflow: max delta_ticks = 65535 (uint16_t)
//   65535 × 16667 = 1,092,534,045 < 2^32 (4,294,967,295) → seguro em uint32_t.
// Margem restante: ~3× antes de overflow. Se o prescaler do FTM3 mudar para
// valores maiores que 2, recalcular com o novo fator de conversão.
inline uint32_t ticks_to_ns(uint16_t ticks) noexcept {
    // Kinetis FTM3: 120 MHz / PS=2 = 60.0 MHz → 16.667 ns/tick → factor = 16667
    // STM32H562 TIM5: 250 MHz / PS=4 = 62.5 MHz → 16.000 ns/tick → factor = 16000
#if defined(TICKS_TO_NS_FACTOR)
    return (static_cast<uint32_t>(ticks) * TICKS_TO_NS_FACTOR) / TICKS_TO_NS_DIVISOR;
#else
    return (static_cast<uint32_t>(ticks) * 16667u) / 1000u;
#endif
}

// Calcula RPM × 10 a partir do período de um dente (nanossegundos).
// Cada dente ocupa 6° = 1/60 de revolução (roda 60-2: 60 posições uniformes).
// rpm × 10 = (60 s/min × 10⁹ ns/s × 10) / (60 × tooth_period_ns)
//           = 600.000.000.000 / (60 × tooth_period_ns)
//           = 10.000.000.000 / tooth_period_ns   ← pré-calculado para evitar mul 64-bit na ISR
static constexpr uint64_t kRpmNumerator = 10000000000ULL;
inline uint32_t rpm_x10_from_period_ns(uint32_t period_ns) noexcept {
    if (period_ns == 0u) { return 0u; }
    return static_cast<uint32_t>(kRpmNumerator / period_ns);
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
inline bool is_gap(uint32_t period, uint32_t avg) noexcept {
    return (period * kGapRatioDen > avg * kGapRatioNum);
}

// Teste de dente normal dentro da janela de tolerância ±20%.
// 0,8×avg ≤ period ≤ 1,2×avg → dente aceito para atualização do histórico.
inline bool is_normal_tooth(uint32_t period, uint32_t avg) noexcept {
    const uint32_t lo = (avg * kTolNumLow)  / kTolDenLow;   // 80% de avg
    const uint32_t hi = (avg * kTolNumHigh) / kTolDenHigh;  // 120% de avg
    return (period >= lo && period <= hi);
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
inline bool process_gap_event(uint16_t capture_now) noexcept {
    switch (g_state.snap.state) {

        case ems::drv::SyncState::WAIT_GAP:
        case ems::drv::SyncState::LOSS_OF_SYNC:
            // Em qualquer estado de "não sincronizado", um gap inicia a tentativa de sync.
            //
            // CORREÇÃO CKP-02: exigir tooth_count >= kGapThresholdTooth mesmo aqui.
            // Durante os primeiros kHistSize dentes (bootstrap do histórico), o filtro
            // de razão ainda não está activo e tooth_count pode ser < 55. Um spike de
            // EMC que gere um período longo (aparentemente gap) sem que tenham passado
            // dentes suficientes não deve avançar o estado — seria uma sincronização falsa.
            //
            // Excepção: no estado puro WAIT_GAP com histórico ainda não preenchido
            // (hist_ready < kHistSize), a ISR retorna antes de chegar aqui, portanto
            // qualquer gap que chegue a process_gap_event() já passou pelo filtro externo
            // ou é o primeiro evento após hist_ready == kHistSize. A guarda abaixo cobre
            // o caso em que tooth_count foi incrementado mas ainda está abaixo do limiar.
            if (g_state.tooth_count < kGapThresholdTooth) {
                // Gap prematuro: < 55 dentes desde o último reset de contagem.
                // Pode ser spike de EMC, ruído mecânico ou arranque com histórico parcial.
                // Reseta a contagem mas mantém o estado — aguarda gap legítimo.
                g_state.tooth_count = 0u;
                return false;
            }
            if (g_seed_armed) {
                g_state.snap.state = ems::drv::SyncState::FULL_SYNC;
                g_state.snap.phase_A = g_seed_phase_a;
                g_seed_armed = false;
                g_seed_probation = true;
                g_seed_probation_teeth = 0u;
            } else {
                g_state.snap.state = ems::drv::SyncState::HALF_SYNC;
            }
            g_state.tooth_count         = 0u;
            g_state.snap.tooth_index    = 0u;
            g_state.snap.last_ftm3_capture = capture_now;
            return true;

        case ems::drv::SyncState::HALF_SYNC:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                // 2º gap na posição correta → sincronismo completo.
                // A partir deste ponto, tooth_index rastreia a posição angular
                // com resolução de 6° por dente.
                g_state.snap.state          = ems::drv::SyncState::FULL_SYNC;
                g_state.tooth_count         = 0u;
                g_state.snap.tooth_index    = 0u;
                g_state.snap.last_ftm3_capture = capture_now;
                return true;
            }
            // Gap antes de kGapThresholdTooth dentes: pulso espúrio (EMC, dente danificado).
            // Cai para LOSS_OF_SYNC e aguarda novo gap para tentar outra vez.
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            return false;

        case ems::drv::SyncState::FULL_SYNC:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                // Gap na posição esperada → mantém FULL_SYNC, reinicia contagem.
                g_state.tooth_count         = 0u;
                g_state.snap.tooth_index    = 0u;
                g_state.snap.last_ftm3_capture = capture_now;
                return true;
            }
            // Gap inesperado: wheel slip, dente duplo, interferência severa.
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            return false;

        default:
            return false;
    }
}

}  // namespace

// ── Símbolos fracos (hooks) ───────────────────────────────────────────────────
namespace ems::drv {

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

}  // namespace ems::drv

// ── API pública ───────────────────────────────────────────────────────────────
namespace ems::drv {

CkpSnapshot ckp_snapshot() noexcept {
    CkpSnapshot out;
    enter_critical();
    out = g_state.snap;
    exit_critical();
    return out;
}

uint16_t ckp_angle_to_ticks(uint16_t angle_mdeg, uint16_t ref_capture) noexcept {
    // RESERVADA: sem callers em produção. Domínio FTM3 (60 MHz/PS=2). Ver header.
    // FTM3: 120 MHz / prescaler 2 = 60 MHz → 16,667 ns/tick
    // tooth_period_ticks = tooth_period_ns × (60 ticks/µs) / 1000
    // ticks_para_angulo  = (angle_mdeg × tooth_period_ticks) / kToothAngleX1000
    //
    // Verificação dimensional (angle_mdeg em miligraus, kToothAngleX1000 = 6000 mg/dente):
    //   Para 6° (1 dente): angle_mdeg = 6000 → ticks = 6000 × T / 6000 = T ✓
    //   Para 1°:           angle_mdeg = 1000 → ticks = 1000 × T / 6000 = T/6 ✓
    //
    // ATENÇÃO: não passar graus inteiros (ex: 6) — causará erro de 1000×.
    const uint32_t tooth_period_ticks = (g_state.snap.tooth_period_ns * 60u) / 1000u;
    const uint32_t delta = (static_cast<uint32_t>(angle_mdeg) * tooth_period_ticks)
                           / kToothAngleX1000;
    return static_cast<uint16_t>(ref_capture + static_cast<uint16_t>(delta));
}

// ── ISR do CKP: FTM3 Canal 0 (PTD0, rising edge) ─────────────────────────────
//
// CONTEXTO: chamada por FTM3_IRQHandler() em hal/ftm.cpp, NVIC prioridade 1.
// Não há chamada direta por código de usuário.
//
// SETUP do FTM3_CnSC (Canal 0) — K64 RM §43.3.5:
//   CnSC[5:4] MSnB:MSnA = 00  → modo Input Capture (não Output Compare)
//   CnSC[3:2] ELSnB:ELSnA = 01 → Rising Edge Capture
//   CnSC[6]   CHIE = 1        → Interrupt Enable
//   Configurado em hal/ftm.cpp → ftm3_init() durante boot.
//
// VANTAGEM vs GPIO/EXTI:
//   O periférico FTM3 trava o valor do contador em CnV (≡ FTM3_C0V) no exato
//   instante da borda de subida, em hardware. A CPU pode atender a IRQ
//   vários ciclos depois — o timestamp em C0V permanece válido.
//   Isso é impossível com GPIO/EXTI onde a CPU leria o contador atual (atrasado).
FASTRUN void ckp_ftm3_ch0_isr() noexcept {
    // ── 1. Timestamp sem jitter de ISR ────────────────────────────────────
    // CRÍTICO: lemos FTM3_C0V ANTES de qualquer outra operação.
    // O registrador de captura foi travado pelo HW no instante exato da borda;
    // leituras posteriores (GPIOD_PDIR, etc.) não afetam o valor capturado.
    // NÃO ler FTM3_CNT: o contador avançou durante a latência de IRQ.
    const uint16_t capture_now = static_cast<uint16_t>(FTM3_C0V & 0xFFFFu);

    // ── 2. Delta de ticks (aritmética circular uint16_t) ──────────────────
    // Subtração circular: correto mesmo se o contador passou por 0xFFFF→0x0000
    // durante o período do dente. Ex: 0x0005 - 0xFFFD = 0x0008 (delta = 8) ✓
    const uint16_t delta_ticks = static_cast<uint16_t>(capture_now - g_state.prev_capture);

    // ── 3. Anti-glitch por período mínimo ────────────────────────────────
    // Estratégia preferida a checar GPIOD_PDIR depois da captura: a 120 MHz
    // o pino pode ter retornado LOW antes da CPU ler o registrador de GPIO,
    // descartando capturas legítimas em alta rotação (> 4000 RPM).
    // Período mínimo aceitável: dente a 20000 RPM → 60 MHz / (58 * 20000/60)
    //   = ~3103 ticks; usamos 50 como limite inferior conservador para
    //   rejeitar glitches de EMC (< ~833 ns) sem afetar operação normal.
    static constexpr uint16_t kMinToothTicks = 50u;
    if (delta_ticks < kMinToothTicks) {
        return;  // pulso muito curto → ruído EMC, descartar
    }

    g_state.prev_capture                = capture_now;
    g_state.snap.last_ftm3_capture      = capture_now;

    const uint32_t period_ns = ticks_to_ns(delta_ticks);

    // ── 4. Construção do histórico (primeiros kHistSize dentes) ───────────
    // Antes do histórico estar completo (hist_ready < kHistSize), aceitamos
    // todos os dentes incondicionalmente para inicializar a janela de média.
    // Não há referência de velocidade ainda para filtrar.
    if (g_state.hist_ready < kHistSize) {
        hist_push(delta_ticks);
        ++g_state.tooth_count;
        g_state.snap.tooth_period_ns = period_ns;
        g_state.snap.rpm_x10 = rpm_x10_from_period_ns(period_ns);
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 5. Classificação do evento por razão ──────────────────────────────
    // Calcula média dos últimos kHistSize períodos normais (em ticks).
    // avg é computado em ticks (não ns) para evitar conversão adicional.
    const uint32_t avg = hist_avg();

    // ── 5a. Teste de GAP ──────────────────────────────────────────────────
    // Condição: delta_ticks × 2 > avg × 3  ≡  delta > 1,5 × avg
    // Para 60-2: gap ≈ 3 × avg → ratio ≈ 3,0 >> 1,5 (margem de 100%)
    // O limiar 1,5× separa o gap de qualquer dente normal válido (máx 120% de avg).
    //
    // NOTA (CKP-02): A verificação de tooth_count >= kGapThresholdTooth é feita
    // DENTRO de process_gap_event() para todos os estados, incluindo WAIT_GAP e
    // LOSS_OF_SYNC. Isso garante proteção uniforme contra gaps prematuros durante
    // bootstrap do histórico (hist_ready < kHistSize) onde o filtro de razão acima
    // ainda não estava activo nas iterações anteriores.
    if (is_gap(delta_ticks, avg)) {
        // Delega para a máquina de estados — a validação de tooth_count
        // e todas as transições estão encapsuladas em process_gap_event().
        static_cast<void>(process_gap_event(capture_now));
        // Dispara hooks mesmo após gap: permite ao agendador / sensores reagir
        // à mudança de estado (ex: cancelar eventos pendentes após LOSS_OF_SYNC).
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 5b. Filtro ±20%: dente normal vs ruído ────────────────────────────
    // Período dentro de [80%, 120%] do avg → dente normal válido.
    // Fora desta janela (mas < limiar de gap) → ruído transitório: descartar.
    if (!is_normal_tooth(delta_ticks, avg)) {
        // Ruído: não atualiza hist, não incrementa tooth_count.
        // Apenas dispara hooks para que camadas superiores possam monitorar.
        sensors_on_tooth(g_state.snap);
        schedule_on_tooth(g_state.snap);
        return;
    }

    // ── 6. Processamento de dente normal ──────────────────────────────────
    hist_push(delta_ticks);

    g_state.snap.tooth_period_ns = period_ns;
    g_state.snap.rpm_x10 = rpm_x10_from_period_ns(period_ns);

    // Incrementa tooth_count e tooth_index apenas em estados sincronizados/tentando.
    if (g_state.snap.state != ems::drv::SyncState::WAIT_GAP &&
        g_state.snap.state != ems::drv::SyncState::LOSS_OF_SYNC) {
        ++g_state.tooth_count;
        // tooth_index: posição angular dentro da revolução (0 = após gap, 57 = último dente)
        g_state.snap.tooth_index =
            (g_state.snap.tooth_index < (kRealTeethPerRev - 1u))
                ? static_cast<uint16_t>(g_state.snap.tooth_index + 1u)
                : 0u;
    } else {
        // Em WAIT_GAP / LOSS_OF_SYNC: incrementa tooth_count para detectar
        // o limiar mínimo de dentes antes de aceitar o próximo gap.
        ++g_state.tooth_count;
    }

    // ── 7. Verificação de perda de sincronia por contagem excessiva ───────
    // Se passaram mais de kMaxTeethBeforeLoss dentes sem um gap:
    //   → o gap foi perdido (interferência, aceleração brusca, falha de sensor)
    if (g_state.tooth_count > kMaxTeethBeforeLoss) {
        if (g_state.snap.state == ems::drv::SyncState::HALF_SYNC ||
            g_state.snap.state == ems::drv::SyncState::FULL_SYNC) {
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
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
}

// ── ISR do cam sensor: FTM3 Canal 1 (PTD1, rising edge) ──────────────────────
// Cada borda de subida do cam sensor indica meio ciclo de motor (180° de virabrequim).
// phase_A alterna para permitir ao agendador identificar qual par de cilindros está
// no tempo de injeção (cilindros 1/4 vs 2/3 para motor 4 cilindros em linha).
FASTRUN void ckp_ftm3_ch1_isr() noexcept {
    if ((GPIOD_PDIR & (1u << 1u)) == 0u) {
        return;  // anti-glitch: apenas rising edges reais
    }
    static_cast<void>(FTM3_C1V);   // leitura limpa o CHF do canal
    g_state.snap.phase_A = !g_state.snap.phase_A;
    if (g_seed_probation) {
        g_seed_probation = false;
        g_seed_probation_teeth = 0u;
        ++g_seed_confirmed_count;
    }
    if (g_state.cmp_confirms < 2u) {
        ++g_state.cmp_confirms;
    }
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

// ── API de teste (host only) ──────────────────────────────────────────────────
#if defined(EMS_HOST_TEST)
void ckp_test_reset() noexcept {
    g_state = DecoderState{
        CkpSnapshot{0u, 0u, 0u, 0u, SyncState::WAIT_GAP, false},
        0u,
        {0u, 0u, 0u},
        0u,
        0u,
        0u,
    };
    ems_test_ftm3_c0v   = 0u;
    ems_test_ftm3_c1v   = 0u;
    ems_test_gpiod_pdir = 0u;
    g_seed_armed = false;
    g_seed_phase_a = false;
    g_seed_probation = false;
    g_seed_probation_teeth = 0u;
    g_seed_loaded_count = 0u;
    g_seed_confirmed_count = 0u;
    g_seed_rejected_count = 0u;
}

uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept {
    return rpm_x10_from_period_ns(period_ns);
}
#endif

}  // namespace ems::drv
