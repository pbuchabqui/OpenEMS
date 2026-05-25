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
#if defined(TARGET_STM32H562) && !defined(EMS_HOST_TEST)
#include "hal/regs.h"
#endif

// ── Mock de registradores para testes host ───────────────────────────────────
#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_tim5_ccr1  = 0u;
volatile uint32_t ems_test_tim5_ccr2  = 0u;
volatile uint32_t ems_test_cam_gpio_idr = 0u;
#endif

// FASTRUN coloca ISRs críticas em SRAM (zero cache miss).
// Em host/embedded: __attribute__((section(".fastrun"))) via WProgram.h / core_pins.h.
// Em host tests: indefinida — defini-la vazia garante compilação sem modificações.
#if !defined(FASTRUN)
#define FASTRUN
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

// Preditor conservador para agendamento intra-dente:
// usa a tendência do último período aceito, limitada a ±12,5%/dente.
static constexpr uint32_t kPredictionClampDen = 8u;

// Tamanho da janela deslizante de histórico (em número de períodos).
// 3 amostras são suficientes para filtrar ruído transitório.
static constexpr uint8_t kHistSize = 3u;

// Mínimo de ticks entre bordas para descartar glitches de EMC (<800 ns @ 62.5 MHz).
static constexpr uint32_t kMinToothTicks = 50u;

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
#define TIM5_CKP_CAPTURE TIM5_CCR1
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
    uint32_t cmp_glitch_count;          // FIX P0: contador de glitches CMP rejeitados (diagnóstico)
};

static DecoderState g_state = {
    ems::drv::CkpSnapshot{0u, 0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false},
    0u,
    0u,
    {0u, 0u, 0u},
    0u,
    0u,
    0u,
    0u,  // cmp_glitch_count inicializado
};
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
inline uint32_t ticks_to_ns(uint32_t ticks) noexcept {
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

inline uint32_t predict_next_period_ticks(uint32_t current_ticks) noexcept {
    const uint32_t prev = g_state.prev_period_ticks;
    if (prev == 0u) { return current_ticks; }

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
inline bool is_gap(uint32_t period, uint32_t avg) noexcept {
    return (period * kGapRatioDen > avg * kGapRatioNum);
}

// Teste de dente normal dentro da janela de tolerância ±20%.
// 0,8×avg ≤ period ≤ 1,2×avg → dente aceito para atualização do histórico.
// Multiplica ambos os lados por kTolDen (5) para eliminar as divisões da ISR.
// kTolDenLow == kTolDenHigh == 5, então period×5 substitui period/den em ambas as comparações.
inline bool is_normal_tooth(uint32_t period, uint32_t avg) noexcept {
    const uint32_t p5 = period * kTolDenLow;
    return (p5 >= avg * kTolNumLow) && (p5 <= avg * kTolNumHigh);
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
inline bool process_gap_event() noexcept {
    switch (g_state.snap.state) {

        case ems::drv::SyncState::WAIT_GAP:
        case ems::drv::SyncState::LOSS_OF_SYNC:
            // CORREÇÃO CKP-02: exigir tooth_count >= kGapThresholdTooth mesmo aqui.
            // Protege contra spikes de EMC que gerem período longo (aparentemente gap)
            // antes de dentes suficientes — evitaria sincronização falsa.
            if (g_state.tooth_count < kGapThresholdTooth) {
                g_state.tooth_count = 0u;
                return false;
            }
            
            // FIX P0 (BUG-12): Validar rotação forward antes de consumir seed
            // Se rotação for reversa ou instável, não consumir a seed para preservar
            // o benefício do fast-reacquire na próxima tentativa válida.
            if (g_seed_armed && !is_forward_rotation_coherent(g_state.snap.tooth_period_ns)) {
                // Rotação não validada como forward estável — mantém seed armada
                // mas não avança para FULL_SYNC ainda
                g_state.tooth_count = 0u;
                return false;
            }
            
            if (g_seed_armed) {
                g_state.snap.state       = ems::drv::SyncState::FULL_SYNC;
                g_state.snap.phase_A     = g_seed_phase_a;
                g_seed_armed             = false;
                g_seed_probation         = true;
                g_seed_probation_teeth   = 0u;
                // Reset contador de coerência após consumo bem-sucedido da seed
                g_prev_valid_period_ns = 0u;
                g_coherent_periods_count = 0u;
            } else {
                g_state.snap.state = ems::drv::SyncState::HALF_SYNC;
            }
            g_state.tooth_count      = 0u;
            g_state.snap.tooth_index = 0u;
            return true;

        case ems::drv::SyncState::HALF_SYNC:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                // 2º gap na posição correta → sincronismo completo.
                g_state.snap.state       = ems::drv::SyncState::FULL_SYNC;
                g_state.tooth_count      = 0u;
                g_state.snap.tooth_index = 0u;
                return true;
            }
            // Gap prematuro: pulso espúrio (EMC, dente danificado).
            g_state.snap.state  = ems::drv::SyncState::LOSS_OF_SYNC;
            g_state.tooth_count = 0u;
            return false;

        case ems::drv::SyncState::FULL_SYNC:
            if (g_state.tooth_count >= kGapThresholdTooth) {
                // Gap na posição esperada → mantém FULL_SYNC, reinicia contagem.
                g_state.tooth_count      = 0u;
                g_state.snap.tooth_index = 0u;
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
    ems::hal::CriticalSectionGuard guard;
    // memcpy garante cópia byte-a-byte determinística; CriticalSectionGuard
    // desabilita interrupções, impedindo que a ISR TIM5 modifique g_state.snap
    // durante a cópia. Sem volatile + critical section, o compilador poderia
    // reordenar ou cachear leituras de campos individuais de g_state.snap.
    std::memcpy(&out, &g_state.snap, sizeof(out));
    return out;
}

uint32_t ckp_angle_to_ticks(uint32_t angle_mdeg, uint32_t ref_capture) noexcept {
    // RESERVADA: sem callers em producao. Dominio TIM5 @ 62.5 MHz. Ver header.
    // tooth_period_ticks = tooth_period_ns / 16 ns por tick
    // ticks_para_angulo  = (angle_mdeg × tooth_period_ticks) / kToothAngleX1000
    //
    // Verificação dimensional (angle_mdeg em miligraus, kToothAngleX1000 = 6000 mg/dente):
    //   Para 6° (1 dente): angle_mdeg = 6000 → ticks = 6000 × T / 6000 = T ✓
    //   Para 1°:           angle_mdeg = 1000 → ticks = 1000 × T / 6000 = T/6 ✓
    //
    // ATENÇÃO: não passar graus inteiros (ex: 6) — causará erro de 1000×.
    const uint32_t tooth_period_ticks = g_state.snap.tooth_period_ns >> 4u;
    const uint32_t delta = (angle_mdeg * tooth_period_ticks)
                           / kToothAngleX1000;
    return ref_capture + delta;
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
    // ── 1. Timestamp sem jitter de ISR ────────────────────────────────────
    // CRÍTICO: lemos TIM5_CKP_CAPTURE ANTES de qualquer outra operação.
    // O registrador de captura foi travado pelo HW no instante exato da borda;
    // leituras posteriores (CKP_CAM_GPIO_IDR, etc.) não afetam o valor capturado.
    // NÃO ler TIM5_CNT: o contador avançou durante a latência de IRQ.
    const uint32_t capture_now = TIM5_CKP_CAPTURE;

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
    // Antes do histórico estar completo (hist_ready < kHistSize), aceitamos
    // todos os dentes incondicionalmente para inicializar a janela de média.
    // Não há referência de velocidade ainda para filtrar.
    if (g_state.hist_ready < kHistSize) {
        hist_push(delta_ticks);
        ++g_state.tooth_count;
        g_state.snap.tooth_period_ns = period_ns;
        g_state.snap.predicted_tooth_period_ns = period_ns;
        g_state.snap.rpm_x10 = rpm_x10_from_period_ticks(delta_ticks);
        g_state.prev_period_ticks = delta_ticks;
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
        static_cast<void>(process_gap_event());
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
    const uint32_t predicted_ticks = predict_next_period_ticks(delta_ticks);
    hist_push(delta_ticks);

    g_state.snap.tooth_period_ns = period_ns;
    g_state.snap.predicted_tooth_period_ns = ticks_to_ns(predicted_ticks);
    g_state.snap.rpm_x10 = rpm_x10_from_period_ticks(delta_ticks);
    g_state.prev_period_ticks = delta_ticks;

    // tooth_count incrementa em todos os estados — alimenta o limiar de gap e
    // a detecção de LOSS_OF_SYNC por excesso de dentes sem gap.
    ++g_state.tooth_count;
    // tooth_index só avança quando há referência angular (HALF_SYNC / FULL_SYNC).
    if (g_state.snap.state != ems::drv::SyncState::WAIT_GAP &&
        g_state.snap.state != ems::drv::SyncState::LOSS_OF_SYNC) {
        g_state.snap.tooth_index =
            (g_state.snap.tooth_index < (kRealTeethPerRev - 1u))
                ? static_cast<uint16_t>(g_state.snap.tooth_index + 1u)
                : 0u;
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

// FIX P0 (BUG-12): Proteger seed contra consumo em rotação reversa
// Problema: Se a seed for consumida durante partida com rotação reversa,
// ela é perdida permanentemente até próximo shutdown, mesmo que motor
// não tenha atingido sincronismo válido.
// Solução: Validar coerência temporal do período CKP antes de consumir seed.
// Rotação reversa produz períodos instáveis ou decrescentes inconsistentes.
static uint32_t g_prev_valid_period_ns = 0u;
static uint8_t g_coherent_periods_count = 0u;

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
    if ((CKP_CAM_GPIO_IDR & (1u << 1u)) == 0u) {
        return;  // anti-glitch: apenas rising edges reais
    }
    
    // Validação de coerência temporal baseada no período CKP atual
    // Período CMP esperado ≈ 2× período médio dos dentes CKP (1 revolução completa)
    const uint32_t ckp_period_ns = g_state.snap.tooth_period_ns;
    if (ckp_period_ns > 0u) {
        // Estima período CMP esperado: 58 dentes × período_dente ≈ 1 revolução
        // Para simplificar: usa-se o tooth_period_ns como referência de escala
        // CMP válido deve ter período entre 0.5× e 4.0× o período de referência
        static uint32_t prev_cmp_tooth_period = 0u;
        
        // Compara com período anterior do CKP para detectar anomalias grosseiras
        // Se tooth_period mudou drasticamente (>4× ou <0.25×), ignora esta borda CMP
        if (prev_cmp_tooth_period > 0u) {
            const uint32_t min_valid = prev_cmp_tooth_period >> 2u;   // 25%
            const uint32_t max_valid = prev_cmp_tooth_period << 2u;   // 400%
            
            if ((ckp_period_ns < min_valid) || (ckp_period_ns > max_valid)) {
                // Período CKP mudou drasticamente — possível ruído, não atualiza phase_A
                ++g_state.cmp_glitch_count;
                static_cast<void>(TIM5_CAM_CAPTURE);
                prev_cmp_tooth_period = ckp_period_ns;
                return;
            }
        }
        prev_cmp_tooth_period = ckp_period_ns;
    }
    
    static_cast<void>(TIM5_CAM_CAPTURE);   // leitura limpa o CHF do canal
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
        CkpSnapshot{0u, 0u, 0u, 0u, 0u, SyncState::WAIT_GAP, false},
        0u,
        0u,
        {0u, 0u, 0u},
        0u,
        0u,
        0u,
    };
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
}

uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept {
    return rpm_x10_from_period_ns(period_ns);
}
#endif

}  // namespace ems::drv
