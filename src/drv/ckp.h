/**
 * @file drv/ckp.h
 * @brief Crank snapshot publicado pelo heartbeat do encoder TIM2.
 *
 * Sem decoder 60-2. No encoder: WAIT_GAP unused, HALF_SYNC = presync
 * (ω válido, sem fase CMP), FULL_SYNC = CMP confirmado, LOSS_OF_SYNC = stall.
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
    WAIT_GAP,       ///< Unused on encoder (kept for wire/enum stability)
    HALF_SYNC,      ///< Presync: omega valid, CMP phase not confirmed
    FULL_SYNC,      ///< CMP confirmed (2 edges) — sequential 720°
    LOSS_OF_SYNC,   ///< Stall / sensor loss
};

/**
 * @brief Instantâneo do decodificador CKP (sem estado mutável).
 *
 * Todos os campos são consistentes entre si no momento da chamada a
 * ckp_snapshot() (captura atômica via seção crítica).
 */
struct CkpSnapshot {
    uint32_t tooth_period_ns;    ///< Unused on encoder (kept for struct layout)
    uint32_t predicted_tooth_period_ns; ///< Unused on encoder
    uint16_t tooth_index;        ///< Unused on encoder — use crank_deg
    uint32_t last_tim5_capture;  ///< TIM5 tick at last publish (stall)
    uint32_t rpm_x10;            ///< RPM × 10; 0 before omega / after stall
    SyncState state;
    bool phase_A;                ///< 720° half: true = 0–360°, false = 360–720°. CMP sets it.
    uint8_t cmp_confirms;        ///< Validated CMP edges since loss (0–2). Sequential gate.
    uint32_t tim2_cnt;           ///< TIM2 encoder count at publish
    uint16_t crank_deg;          ///< 0–359 from TIM2
    uint32_t cmp_reject_count;   ///< Flancos CMP rejeitados pela janela de
                                  ///< plausibilidade (encoder_sync::evaluate_cmp_edge()).
                                  ///< "Glitch count" real do sinal CMP — ver
                                  ///< ckp_get_cmp_glitch_count().
};

/**
 * @brief Retorna instantâneo atômico do estado CKP.
 *
 * Seguro para chamada de qualquer contexto (main loop, ISR de menor
 * prioridade). Usa seção crítica CPSID/CPSIE internamente.
 */
CkpSnapshot ckp_snapshot() noexcept;

/**
 * @brief Publica um CkpSnapshot vindo do caminho MT6835/TIM2 (fork encoder),
 *        não de uma ISR de dente do CKP.
 *
 * Único escritor de g_state.snap fora das ISRs TIM5 do CKP — em modo encoder
 * (EMS_MT6835_ENCODER=1) essas ISRs nunca disparam, então sem isto o
 * snapshot ficaria congelado em WAIT_GAP para sempre. Reutiliza a mesma
 * struct/enum para que todos os consumidores existentes (main loop, VVT,
 * misfire, protocolo UI) continuem a funcionar sem qualquer alteração — ver
 * docs/dev/mt6835_encoder_fork.md, secção "Sync-state em modo encoder".
 */
void ckp_publish_encoder_snapshot(const CkpSnapshot& snap) noexcept;

// ── Hooks 60-2 (unused on encoder — do not extend) ──────────────────────────
// Nunca chamados neste tree (ISRs TIM5 CKP são no-op; heartbeat TIM2 é o
// caminho vivo). Função nova: slot de tempo no main, subtick/heavy tick
// TIM2, ou EncFuelIgnPrep / finalize_cyl_setpoints — ver
// docs/dev/mt6835_encoder_fork.md "Onde encaixar uma função nova".
void sensors_on_tooth(const CkpSnapshot& snap) noexcept;
void schedule_on_tooth(const CkpSnapshot& snap) noexcept;
void prime_on_tooth(const CkpSnapshot& snap) noexcept;
void misfire_on_tooth(const CkpSnapshot& snap) noexcept;

// TIM5 CH1/CH2 ISRs: empty on encoder (TIM5 is freerun only).
void ckp_tim5_ch1_isr() noexcept;
void ckp_tim5_ch2_isr() noexcept;

// Nº de flancos CMP rejeitados pela janela de plausibilidade (ver
// CkpSnapshot::cmp_reject_count — publicado por ecu_sched_encoder_heartbeat_tick(),
// nunca escrito diretamente aqui, mesmo padrão de cmp_confirms). 0 até o
// primeiro heavy tick publicar um snapshot (arranque).
uint32_t ckp_get_cmp_glitch_count() noexcept;

// DIAG 60-2: always 0 on this tree (no tooth decoder). Kept for dump 'D'
// / wire layout. Do not treat a rising counter as a live CKP event.
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
// Dentes descartados pelo skip pós-silêncio (ckp_skip_pulses_after_gap > 0).
extern volatile uint32_t g_dbg_skip_after_silence;

// Osciloscópio encoder (comando 'K'): rings de ângulo 0–719 (ciclo 720°),
// não timestamps TIM5 (o TIM5 já não captura CKP/CMP neste fork).
// idx = próxima posição a escrever (elemento mais antigo do ring).
extern volatile uint32_t g_scope_ckp_ts[64];
extern volatile uint8_t  g_scope_ckp_idx;
extern volatile uint32_t g_scope_cmp_ts[8];
extern volatile uint8_t  g_scope_cmp_idx;

void ckp_scope_push_ckp_deg720(uint16_t deg720) noexcept;
void ckp_scope_push_cmp_deg720(uint16_t deg720) noexcept;
void ckp_scope_clear_cmp(void) noexcept;

// tooth_index âncora da última borda CMP aceite (0xFF = não-ancorado).
// Conceito de decoder 60-2 — o rastreador de fase do encoder (TIM2, ver
// ecu_sched_encoder_phase.cpp) usa um ângulo absoluto (TIM2->CNT), não um
// índice de dente; não há mapeamento 1:1 honesto para este byte nesta
// árvore. Fica 0xFF sempre (achado #4 da revisão 2fa1513..bc30ca6,
// 2026-08-19) — a informação equivalente ("CMP confirmado, gate do
// sequencial") já vive em CkpSnapshot::cmp_confirms, exposta à parte no
// protocolo 'A' (rt.reserved[7]). A UI do dash ('K' scope, app.js/drawScope)
// deixou de depender deste byte para o texto "(ancorado/não-ancorado)" —
// usa sync_state/phase_a, já presentes no mesmo payload.
uint8_t ckp_get_cmp_ref_tooth() noexcept;

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

// Equivalente para o caminho MT6835/TIM2 (EMS_MT6835_ENCODER=1): ckp_stall_poll()
// não serve aqui porque usa g_state.prev_capture, que só a ISR TIM5 do CKP
// escreve — nunca dispara em modo encoder (tim5_freerun_init() não configura
// captura). Usa snap.last_tim5_capture (timestamp TIM5 do último heartbeat
// pesado completo, ecu_sched_encoder_heartbeat_tick() — só avança se TIM2
// continuar a receber bordas A/B reais) como referência independente de
// rotação. Sem isto, se o sinal do encoder parar (fio partido, sensor morto),
// TIM2 congela, o heartbeat que o CC4IF dispara também congela (ele próprio
// depende de TIM2 avançar até ao alvo), e rpm_x10 fica preso no último valor
// válido para sempre — nunca cai a 0 sozinho.
bool ckp_stall_poll_encoder(uint32_t tim5_cnt_now) noexcept;

// ── API de teste (somente em build host) ──────────────────────────────────────
#if defined(EMS_HOST_TEST)
void     ckp_test_reset() noexcept;
uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept;
void     ckp_test_set_cmp_confirms(uint8_t n) noexcept;
#endif

}  // namespace ems::drv
