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
 *                      gap && count≥55
 *   WAIT_GAP  ─────────────────────────►  HALF_SYNC
 *       ▲                                     │  gap && count≥55
 *       │   gap detected                      ▼
 *   LOSS_OF_SYNC  ◄─── count>61 ────  FULL_SYNC
 *       │                                     │
 *       └──────────── gap detected ───────────┘
 *                       (re-sync)
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

/**
 * @brief Arm a persisted sync seed for fast reacquire on next valid gap.
 *
 * Safety note: this does not bypass gap validation; it only allows promotion
 * WAIT_GAP/LOSS_OF_SYNC -> FULL_SYNC at the first accepted gap.
 */
void ckp_seed_arm(bool phase_A) noexcept;
void ckp_seed_disarm() noexcept;

uint32_t ckp_seed_loaded_count() noexcept;
uint32_t ckp_seed_confirmed_count() noexcept;
uint32_t ckp_seed_rejected_count() noexcept;
uint32_t ckp_get_cmp_glitch_count() noexcept;

// DIAG: valores internos de classify_tooth (expostos para snapshot)
extern volatile uint32_t g_diag_tn1;
extern volatile uint32_t g_diag_tn2;
extern volatile uint32_t g_diag_delta;

// DIAG: contador de ISR — incrementa em cada entrada da ISR TIM5
extern volatile uint32_t g_diag_isr_count;
extern volatile uint32_t g_diag_hist_ready;
extern volatile uint32_t g_diag_tooth_count;
extern volatile uint32_t g_diag_consec_anom;
// DIAG: classify_tooth class histogram (GAP / SPIKE_NOISE / normal)
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

// DIAG gap 60-2: aceites, prematuros (FULL_SYNC + count<55 → LOSS) e o
// tooth_count do último prematuro — discrimina perda por gap deslizado
// (estimulador off-by-one) vs gap ausente (kMaxTeethBeforeLoss).
extern volatile uint32_t g_dbg_gap_accepted;
extern volatile uint32_t g_dbg_gap_premature;
extern volatile uint32_t g_dbg_gap_last_tc;
// Perdas de sync por caminho + contexto da última perda por gap ausente.
// Discriminação dos 3 gatilhos de perda de FULL_SYNC (blip PW=0 intermitente):
//   g_dbg_gap_premature   → gap prematuro (count<55)   [já existente]
//   g_dbg_loss_histogram  → gate de dispersão do hist (mx > 1.5×mn)
//   g_dbg_loss_wrap       → tooth_index 57→0 sem gap aceite (gap → normal)
//   g_dbg_loss_missing_gap→ overrun (tooth_count > kMaxTeethBeforeLoss)
// hist_mn/hist_mx = par min/max do último trip de histograma (mx≈1.5×mn = gate
// no limiar → candidato a relaxar; mx≫mn = falha real → drop correto).
extern volatile uint32_t g_dbg_loss_missing_gap;
extern volatile uint32_t g_dbg_loss_stall;
extern volatile uint32_t g_dbg_loss_avg;
extern volatile uint32_t g_dbg_loss_delta;
extern volatile uint32_t g_dbg_loss_histogram;
extern volatile uint32_t g_dbg_loss_wrap;
extern volatile uint32_t g_dbg_loss_hist_mn;
extern volatile uint32_t g_dbg_loss_hist_mx;

// Osciloscópio CKP/CMP: rings de timestamps TIM5 das bordas cruas (comando 'K').
// idx = próxima posição a escrever (elemento mais antigo do ring).
extern volatile uint32_t g_scope_ckp_ts[64];
extern volatile uint8_t  g_scope_ckp_idx;
extern volatile uint32_t g_scope_cmp_ts[8];
extern volatile uint8_t  g_scope_cmp_idx;

// tooth_index âncora da última borda CMP aceite (0xFF = não-ancorado).
uint8_t ckp_get_cmp_ref_tooth() noexcept;


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
