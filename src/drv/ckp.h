/**
 * @file drv/ckp.h
 * @brief Decodificador de roda fônica 60-2 e máquina de sincronismo — OpenEMS
 *
 * RODA FÔNICA 60-2
 * ────────────────
 *   60 posições angulares; 2 dentes consecutivos ausentes = 58 dentes reais.
 *   Espaçamento normal: 360°/60 = 6,0° por posição.
 *   Gap: ≈ 3 × período normal (18°).
 *
 * MÁQUINA DE ESTADOS (SyncState)
 * ───────────────────────────────
 *
 *   WAIT_GAP ──gap (≥3 dentes)──► HALF_SYNC ──gap, 57 dentes──► FULL_SYNC
 *                                      │                          │
 *              gap com contagem ≠ 57, 58º dente sem gap, ruído persistente,
 *              stall ▼                                            ▼
 *                               LOSS_OF_SYNC ──gap, 57 dentes──► HALF_SYNC
 *   (detalhe e regras de ruído: drv/ckp.cpp)
 *
 * HARDWARE: TIM5 CH1 (PA0/CKP) em modo Input Capture, rising edge.
 *   ISR: ckp_tim5_ch1_isr() — chamada por TIM5_IRQHandler() em hal/stm32h562/timer.cpp
 *   Prioridade NVIC: 1 (mais alta do sistema) — §CLAUDE.md tabela IRQ
 */

#pragma once

#include <cstdint>

namespace ems::drv {

/**
 * @brief Estado da máquina de sincronismo CKP.
 *
 * @note Os valores numéricos são estáveis — não alterar sem verificar
 *       todo código que usa comparação direta com o inteiro subjacente.
 */
enum class SyncState : uint8_t {
    WAIT_GAP,       ///< Aguardando primeiro gap — sem referência angular
    HALF_SYNC,      ///< Primeiro gap detectado — contando dentes para confirmar
    FULL_SYNC,      ///< Sincronismo pleno — tooth_index e crank angle válidos
    LOSS_OF_SYNC,   ///< Sincronia perdida — aguardando re-sync via próximo gap
};

/**
 * @brief Instantâneo do decodificador CKP (sem estado mutável).
 *
 * Todos os campos são consistentes entre si no momento da chamada a
 * ckp_snapshot() (captura atômica via seção crítica).
 */
struct CkpSnapshot {
    uint32_t tooth_period_ns;    ///< Período do último dente normal (ns); 0 antes de HALF_SYNC
    uint32_t predicted_tooth_period_ns; ///< Próximo período estimado para agendamento intra-dente
    uint16_t tooth_index;        ///< Índice do dente (0–57) contado desde o último gap; válido em FULL_SYNC
    uint32_t last_tim5_capture;  ///< Timestamp TIM5 (ticks) do último dente — para angle-to-ticks
    uint32_t rpm_x10;            ///< RPM × 10 (ex: 8000 = 800,0 RPM); 0 antes de dados suficientes
    SyncState state;             ///< Estado corrente da máquina de sincronismo
    bool phase_A;                ///< Fase do ciclo de 720°: true=PHASE_A (0-360°), false=PHASE_B (360-720°). Toggles at each gap, SET by CMP.
    uint8_t cmp_confirms;        ///< Number of validated CMP edges since last sync loss (0-2). Gate for sequential mode.
    uint32_t rpm_seg_x10;        ///< RPM × 10 over the last 180° segment (MS42 FA22); 0 until measured. Table lookups only — the scheduler uses the tooth period.
};

/**
 * @brief Retorna instantâneo atômico do estado CKP.
 *
 * Seguro para chamada de qualquer contexto (main loop, ISR de menor
 * prioridade). Usa seção crítica CPSID/CPSIE internamente.
 */
CkpSnapshot ckp_snapshot() noexcept;

// ── Hooks ─────────────────────────────────────────────────────────────────────
// Chamados pela ISR de CKP a cada dente (símbolos fracos — sobrescreva para
// adicionar comportamento sem modificar este módulo).
//
// sensors_on_tooth  → drv/sensors.cpp  (amostragem sincronizada ao dente)
// schedule_on_tooth → engine/ecu_sched.cpp (agendamento injeção/ignição)
// prime_on_tooth    → engine/quick_crank.cpp (prime pulse — 5º dente de cranking)
// misfire_on_tooth  → engine/misfire_detect.cpp (detecção de falha de combustão)

void sensors_on_tooth(const CkpSnapshot& snap) noexcept;
void schedule_on_tooth(const CkpSnapshot& snap) noexcept;
void prime_on_tooth(const CkpSnapshot& snap) noexcept;
void misfire_on_tooth(const CkpSnapshot& snap) noexcept;

// ── ISR handlers (chamados de hal/stm32h562/timer.cpp) ────────────────────────────────────
void ckp_tim5_ch1_isr() noexcept;   ///< CKP rising edge (TIM5 CH1 / PA0)
void ckp_tim5_ch2_isr() noexcept;   ///< Cam sensor rising edge (TIM5 CH2 / PA1)

uint32_t ckp_get_cmp_glitch_count() noexcept;

// Monotonic counters for signal DTCs: running CKP sync losses (stalls not
// counted) and cam-absent fallbacks taken in FULL_SYNC.
void ckp_signal_fault_counts(uint32_t& ckp_sync_losses, uint32_t& cmp_timeouts) noexcept;

// DIAG: valores internos de classify_tooth (expostos para snapshot)
extern volatile uint32_t g_diag_tn1;
extern volatile uint32_t g_diag_tn2;
extern volatile uint32_t g_diag_delta;

// DIAG: contador de ISR — incrementa em cada entrada da ISR TIM5
extern volatile uint32_t g_diag_isr_count;
extern volatile uint32_t g_diag_hist_ready;
extern volatile uint32_t g_diag_tooth_count;
extern volatile uint32_t g_diag_consec_anom;
// DIAG: classificação das bordas (GAP / NOISE / dente normal)
extern volatile uint32_t g_dbg_tc_gap;
extern volatile uint32_t g_dbg_tc_spike;
extern volatile uint32_t g_dbg_tc_normal;

// DIAG sensores CKP/CMP: bordas cruas (antes de filtros/validação) e timestamp
// TIM5 da última borda. Base da telemetria de diagnóstico de sensor: ruído
// periódico aparece como taxa de bordas estável sem sync; fio partido como
// idade crescente sem bordas.
extern volatile uint32_t g_diag_cmp_isr_count;
extern volatile uint32_t g_diag_last_ckp_edge_tick;
extern volatile uint32_t g_diag_last_cmp_edge_tick;

// DIAG gap 60-2: aceites, com contagem errada (≠57 → LOSS) e a contagem do
// último rejeitado (56 = dente perdido, 58 = dente extra/ruído).
extern volatile uint32_t g_dbg_gap_accepted;
extern volatile uint32_t g_dbg_gap_premature;
extern volatile uint32_t g_dbg_gap_acq_reject;
extern volatile uint32_t g_dbg_gap_last_tc;
// Perdas de sync por caminho (ver drv/ckp.cpp): wrap = gap perdido,
// histogram = ruído persistente, stall = sem bordas. missing_gap e
// hist_mn/mx ficam a 0 (slots do protocolo 'D' mantidos).
extern volatile uint32_t g_dbg_loss_missing_gap;
extern volatile uint32_t g_dbg_loss_stall;
extern volatile uint32_t g_dbg_loss_avg;
extern volatile uint32_t g_dbg_loss_delta;
extern volatile uint32_t g_dbg_loss_histogram;
extern volatile uint32_t g_dbg_loss_wrap;
extern volatile uint32_t g_dbg_loss_hist_mn;
extern volatile uint32_t g_dbg_loss_hist_mx;
// Dentes descartados pelo skip pós-silêncio (ckp_skip_pulses_after_gap > 0).
extern volatile uint32_t g_dbg_skip_after_silence;

// Osciloscópio CKP/CMP: rings de timestamps TIM5 das bordas cruas (comando 'K').
// idx = próxima posição a escrever (elemento mais antigo do ring).
extern volatile uint32_t g_scope_ckp_ts[64];
extern volatile uint8_t  g_scope_ckp_idx;
extern volatile uint32_t g_scope_cmp_ts[8];
extern volatile uint8_t  g_scope_cmp_idx;

// tooth_index âncora da última borda CMP aceite (0xFF = não-ancorado).
uint8_t ckp_get_cmp_ref_tooth() noexcept;

// Fase medida do came (VVT de admissão): ângulo de virabrequim ×10 (0..3599)
// da última borda CMP validada, a partir do gap. Retorna o contador de bordas
// medidas (0 = nenhuma ainda; muda a cada borda nova).
uint32_t ckp_cam_edge_angle(uint16_t& angle_x10) noexcept;

// Instant RPM 360° (estilo rusEFI): rpm×10 medido entre o MESMO dente de
// voltas consecutivas — imune ao erro de geometria da roda. 0 = sem medida
// (pré-sync, stall, ou 1ª volta). Consumir só com FULL_SYNC estável.
uint32_t ckp_instant_rpm_x10() noexcept;


// ── Stall watchdog ────────────────────────────────────────────────────────────
// Detecta motor parado entre dois dentes — situação em que tooth_count para de
// incrementar e a máquina ficaria presa em FULL_SYNC indefinidamente.
// Deve ser chamado do main loop (slot 2ms) com o valor atual do contador TIM5
// obtido via ems::hal::tim5_count().
// Retorna true se stall foi detectado nesta chamada (transição → LOSS_OF_SYNC).
bool ckp_stall_poll(uint32_t tim5_cnt_now) noexcept;

// ── API de teste (somente em build host) ──────────────────────────────────────
#if defined(EMS_HOST_TEST)
void     ckp_test_reset() noexcept;
uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept;
void     ckp_test_set_cmp_confirms(uint8_t n) noexcept;
#endif

}  // namespace ems::drv
