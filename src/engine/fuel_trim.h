#pragma once

#include <cstdint>

#include "engine/table3d.h"

namespace ems::engine {

// ── Lambda delay / STFT / LTFT / LEARN ───────────────────────────────────────

void fuel_reset_adaptives() noexcept;
void fuel_lambda_delay_reset() noexcept;

uint16_t lambda_delay_ms_from_rpm_load(uint32_t rpm_x10,
                                       uint16_t map_bar_x100) noexcept;

// Acumulação LTFT por célula + commit na VE (Fase 1 stats + Fase 2 bake-in).
//
// Semântica bake-in (não "filtrar ruído com erro residual"):
//  - Amostra boa = closed-loop + regime estável (ΔRPM/ΔAPP) + λ convergida
//    (|err| ≤ max) + STFT não saturado. Erro ~0 é válido (trim estável).
//  - Célula ready = hits suficientes + mean |err| baixa + |mean STFT| entre
//    min (vale a pena) e max (ainda não saturou) — indicador de qualidade
//    (page12 bit7) e gate de try_commit (1 célula).
//  - Só caminho multiplicativo (PW ≥ threshold) alimenta o acumulador —
//    PW baixo é offset de bico (LTFT add), não VE.
//  - Commit (Fase 2, MANUAL):
//      try_commit: só se ready; bake + desenrola LTFT cell + STFT global.
//      apply_all_ready ('Y'): TODAS as células com hits>0 (correcções
//      acumuladas da sessão), não só ready; bake + LTFT cell; NÃO desenrola
//      STFT N× (re-converge no loop). Nunca automático no closed-loop.
//      VE em flash só com Burn do dashboard.
//  - Sinal de estabilidade: APP (pedido do condutor); MAP+RPM definem a célula.
//
// Unidades: err λ ×1000 (1000 = 1.000); STFT % ×10 (10 = 1.0 %).
constexpr uint16_t kLtftAccumReadyHits              = 30u;
constexpr int16_t  kLtftAccumMaxErrX1000            = 30;   // |err| ≤ 0.030
constexpr int16_t  kLtftAccumMaxStftX10             = 150;  // |STFT| ≤ 15.0 %
constexpr int16_t  kLtftAccumReadyMaxMeanErrX1000   = 25;   // média |err| ≤ 0.025
constexpr int16_t  kLtftAccumReadyMinMeanStftX10    = 5;    // |mean STFT| ≥ 0.5 %
constexpr int16_t  kLtftAccumReadyMaxMeanStftX10    = 150;  // |mean STFT| ≤ 15.0 %
// Fracção do mean STFT aplicada por commit (50 = metade → evita overshoot).
constexpr int16_t  kLtftAccumCommitGainPct          = 50;
constexpr uint8_t  kLtftAccumVeMin                  = 1u;
constexpr uint8_t  kLtftAccumVeMax                  = 200u;

struct LtftCellStats {
    uint16_t hits;
    int32_t  sum_stft_x10;
    int32_t  sum_err_x1000;
};

// now_ms: 0 = sem gate de post-start (testes host). Produção passa relógio real.
int16_t fuel_update_stft(uint32_t rpm_x10,
                         uint16_t map_bar_x100,
                         int16_t lambda_target_x1000,
                         int16_t lambda_measured_x1000,
                         int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut,
                         uint32_t net_pw_us,
                         uint16_t tps_x10,
                         uint32_t now_ms = 0u) noexcept;

int16_t fuel_update_stft_delayed(uint32_t now_ms,
                                 uint32_t rpm_x10,
                                 uint16_t map_bar_x100,
                                 int16_t lambda_target_x1000,
                                 int16_t lambda_measured_x1000,
                                 int16_t clt_x10,
                                 bool o2_valid,
                                 bool ae_active,
                                 bool rev_cut,
                                 uint32_t net_pw_us,
                                 uint16_t tps_x10) noexcept;

bool ltft_accum_sample_valid(uint32_t rpm_x10,
                             uint32_t prev_rpm_x10,
                             uint16_t tps_x10,
                             uint16_t prev_tps_x10,
                             bool have_prev_sample,
                             int16_t lambda_target_x1000,
                             int16_t lambda_measured_x1000,
                             int16_t stft_pct_x10,
                             int16_t clt_x10,
                             bool o2_valid,
                             bool ae_active,
                             bool rev_cut) noexcept;

// Gate de centralidade do LEARN: true quando o ponto de operação está a ≤ ¼
// do vão do nó dominante em ambos os eixos (fora disso a bilinear divide o
// crédito entre células e o acumulador borraria a vizinha). Só gate do LEARN;
// LTFT IIR vivo e STFT não são afectados.
bool fuel_ltft_learn_point_centered(uint32_t rpm_x10,
                                    uint16_t map_bar_x100) noexcept;

void fuel_ltft_accum_reset() noexcept;
void fuel_ltft_accum_reset_cell(uint8_t map_idx, uint8_t rpm_idx) noexcept;
uint16_t fuel_ltft_accum_hits(uint8_t map_idx, uint8_t rpm_idx) noexcept;
bool fuel_ltft_accum_cell_ready(uint8_t map_idx, uint8_t rpm_idx) noexcept;
int16_t fuel_ltft_accum_mean_stft_x10(uint8_t map_idx, uint8_t rpm_idx) noexcept;
int16_t fuel_ltft_accum_mean_err_x1000(uint8_t map_idx, uint8_t rpm_idx) noexcept;

#if defined(EMS_HOST_TEST)
// Test-only: injeta amostra válida direto no acumulador (bypassa gate STFT).
void fuel_ltft_accum_tick_for_test(uint8_t map_idx, uint8_t rpm_idx,
                                   int16_t stft_pct_x10, int16_t err_x1000) noexcept;
#endif

// Fase 2 (manual): try_commit exige ready; apply_all bakeia todas as células
// com hits>0 (mean STFT × gain → VE RAM + desenrola LTFT da célula).
bool fuel_ltft_accum_try_commit(uint8_t map_idx, uint8_t rpm_idx) noexcept;
uint16_t fuel_ltft_accum_apply_all_ready() noexcept;

// Burn opcional da VE: true após commit com ltft_apply_burn_ve=1.
bool fuel_ltft_ve_burn_pending() noexcept;
void fuel_ltft_ve_burn_clear() noexcept;

// Page 12 (0x0C): hits_wire (bit7=ready) + mean_stft i8, 2·N² bytes.
constexpr uint16_t kLtftAccumPageSize =
    static_cast<uint16_t>(2u * kTableCells);
void fuel_ltft_accum_export(uint8_t* dst, uint16_t cap) noexcept;

int16_t fuel_get_stft_pct_x10() noexcept;
void fuel_reset_ltft() noexcept;

// Sessão LEARN/HIL (comando 'Z').
void fuel_reset_learn_session() noexcept;

// DIAG da malha fechada (comando 'D')
extern volatile uint32_t g_dbg_stft_blocked_clt;
extern volatile uint32_t g_dbg_stft_blocked_o2;
extern volatile uint32_t g_dbg_stft_blocked_ae;
extern volatile uint32_t g_dbg_stft_blocked_cut;
extern volatile uint32_t g_dbg_stft_runs;
extern volatile int32_t  g_dbg_stft_last_err;
extern volatile uint32_t g_dbg_ltft_accum_accepted;
extern volatile uint32_t g_dbg_ltft_accum_rejected;
extern volatile uint32_t g_dbg_ltft_accum_commits;
extern int32_t g_stft_integrator_x1000;

int16_t fuel_get_ltft_at(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;
int16_t fuel_get_ltft_pct_x10(uint8_t map_idx, uint8_t rpm_idx) noexcept;
int16_t fuel_get_ltft_add_us(uint8_t map_idx, uint8_t rpm_idx) noexcept;
int16_t fuel_get_ltft_add_at(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;

}  // namespace ems::engine
