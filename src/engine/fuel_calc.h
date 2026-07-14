#pragma once

#include <cstdint>

#include "engine/engine_config.h"
#include "engine/table3d.h"

namespace ems::engine {

constexpr uint32_t calc_req_fuel_us_constexpr(uint16_t displacement_cc,
                                              uint8_t cylinders,
                                              uint16_t injector_flow_cc_min,
                                              uint16_t stoich_afr_x100) noexcept {
    if (displacement_cc == 0u || cylinders == 0u ||
        injector_flow_cc_min == 0u || stoich_afr_x100 == 0u) {
        return 0u;
    }

    const uint64_t num = static_cast<uint64_t>(displacement_cc) *
                         cfg::kAirDensityMgPerCcX1000 *
                         100u *
                         60000000u;
    const uint64_t den = static_cast<uint64_t>(cylinders) *
                         stoich_afr_x100 *
                         injector_flow_cc_min *
                         cfg::kFuelDensityMgPerCc *
                         1000u;
    const uint32_t req = static_cast<uint32_t>(num / den);
    return (req > 50000u) ? 50000u : req;
}

inline constexpr uint32_t kDefaultReqFuelUs =
    calc_req_fuel_us_constexpr(cfg::kDisplacementCc,
                               cfg::kCylinderCount,
                               cfg::kInjectorFlowCcMin,
                               cfg::kStoichAfrX100);

uint8_t get_ve(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;
uint8_t get_ve_prepared(const Table2dLookup& lookup) noexcept;
uint16_t get_lambda_target_x1000(uint32_t rpm_x10, uint16_t map_bar_x100) noexcept;

// EOI blend de 2 pontos por RPM: devolve o eoi_lead_deg efetivo, interpolado
// linearmente entre eoi_idle_deg e g_eng_cfg.default_eoi_lead_deg na janela
// [eoi_blend_rpm_lo, eoi_blend_rpm_hi]. hi ≤ lo = desligado → devolve o main.
// Resultado sempre clampado a [0, 719]; o sanitize do scheduler clampa de novo
// (defesa em profundidade). Integer math pura — sem float, sem divisão por 0.
uint16_t calc_eoi_lead_deg(uint32_t rpm_x10) noexcept;
uint16_t get_lambda_target_x1000_prepared(const Table2dLookup& lookup) noexcept;

uint32_t calc_req_fuel_us(uint16_t displacement_cc,
                          uint8_t cylinders,
                          uint16_t injector_flow_cc_min,
                          uint16_t stoich_afr_x100) noexcept;
uint32_t default_req_fuel_us() noexcept;

uint32_t calc_base_pw_us(uint16_t req_fuel_us,
                         uint8_t ve,
                         uint16_t map_bar_x100,
                         uint16_t map_ref_bar_x100) noexcept;
uint32_t calc_base_pw_us_default(uint8_t ve,
                                 uint16_t map_bar_x100) noexcept;

uint32_t apply_lambda_target_pw_us(uint32_t base_pw_us,
                                   uint16_t lambda_target_x1000) noexcept;

uint32_t apply_fuel_trim_pw_us(uint32_t base_pw_us,
                               int16_t trim_pct_x10) noexcept;

uint16_t corr_clt(int16_t clt_x10) noexcept;
uint16_t corr_iat(int16_t iat_x10) noexcept;
uint16_t corr_vbatt(uint16_t vbatt_mv) noexcept;
uint16_t corr_warmup(int16_t clt_x10) noexcept;

// Correção não-linear do injetor em PW pequeno (curva de abertura do bico).
// Aplicar ao PW final já com dead-time/X-τ/AE/cranking incluídos.
uint32_t apply_injector_scurve(uint32_t pw_us) noexcept;

// Compensação de pressão diferencial de combustível: reescala o PW pela raiz
// quadrada de (ΔP nominal / ΔP atual), já que o fluxo do bico ∝ sqrt(ΔP).
// fuel_press_bar_x1000 == 0 (sensor sem leitura válida) → usa o nominal, sem correção.
uint32_t apply_delta_p_compensation(uint32_t pw_us,
                                    uint16_t fuel_press_bar_x1000,
                                    uint16_t map_bar_x100) noexcept;

uint32_t calc_final_pw_us(uint32_t base_pw_us,
                          uint16_t corr_clt_x256,
                          uint16_t corr_iat_x256,
                          uint16_t dead_time_us) noexcept;
uint32_t calc_fuel_pw_us_default_fast(uint8_t ve,
                                      uint16_t map_bar_x100,
                                      uint16_t lambda_target_x1000,
                                      int16_t trim_pct_x10,
                                      uint16_t corr_clt_x256,
                                      uint16_t corr_iat_x256,
                                      uint16_t dead_time_us) noexcept;

void fuel_ae_set_threshold(uint16_t threshold_tpsdot_x10) noexcept;
void fuel_ae_set_taper(uint8_t taper_cycles) noexcept;

int32_t calc_ae_pw_us(uint16_t tps_now_x10,
                      uint16_t tps_prev_x10,
                      uint16_t dt_ms,
                      int16_t clt_x10) noexcept;

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
//    min (vale a pena commitar) e max (ainda não saturou o trim).
//  - Só caminho multiplicativo (PW ≥ threshold) alimenta o acumulador —
//    PW baixo é offset de bico (LTFT add), não VE.
//  - Commit (Fase 2, MANUAL): aplica fracção do mean STFT em ve_table[map][rpm]
//    (RAM), desenrola LTFT% da célula. try_commit (1 célula) também desenrola
//    STFT global; apply_all_ready ('Y') não — evita N×unroll no mesmo STFT.
//    Nunca automático no closed-loop. VE em flash só com Burn do dashboard.
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

int16_t fuel_update_stft(uint32_t rpm_x10,
                         uint16_t map_bar_x100,
                         int16_t lambda_target_x1000,
                         int16_t lambda_measured_x1000,
                         int16_t clt_x10,
                         bool o2_valid,
                         bool ae_active,
                         bool rev_cut,
                         uint32_t net_pw_us,
                         uint16_t tps_x10) noexcept;

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

void fuel_ltft_accum_reset() noexcept;
void fuel_ltft_accum_reset_cell(uint8_t map_idx, uint8_t rpm_idx) noexcept;
uint16_t fuel_ltft_accum_hits(uint8_t map_idx, uint8_t rpm_idx) noexcept;
bool fuel_ltft_accum_cell_ready(uint8_t map_idx, uint8_t rpm_idx) noexcept;
int16_t fuel_ltft_accum_mean_stft_x10(uint8_t map_idx, uint8_t rpm_idx) noexcept;
int16_t fuel_ltft_accum_mean_err_x1000(uint8_t map_idx, uint8_t rpm_idx) noexcept;

// Fase 2 (manual): se célula ready, bakia mean STFT na VE e desenrola trims.
// Retorna true se commitou. Índices: [map_idx][rpm_idx].
// Não é chamado pelo loop STFT — só host/UI (comando 'Y' ou API).
bool fuel_ltft_accum_try_commit(uint8_t map_idx, uint8_t rpm_idx) noexcept;

// Aplica bake-in em todas as células ready. Retorna quantas commitou.
uint16_t fuel_ltft_accum_apply_all_ready() noexcept;

// Burn opcional da VE: true após commit com ltft_auto_learn_burn_ve=1.
// ui_process limpa quando grava page1 com RPM seguro.
bool fuel_ltft_ve_burn_pending() noexcept;
void fuel_ltft_ve_burn_clear() noexcept;

// Page 12 (0x0C) — visualização do acumulador (read-only), 2·N² bytes:
//   [0 .. N²-1]       hits_wire u8, row-major [map][rpm]:
//                     bits0-6 = min(hits,127); bit7 = ready (FW gate único)
//   [N² .. 2·N²-1]    mean_stft_x10 i8 (clamp ±127)
// Host deve usar bit7 p/ ready — não reimplementar thresholds.
// Índice linear: map_idx * kTableAxisSize + rpm_idx (igual VE).
constexpr uint16_t kLtftAccumPageSize =
    static_cast<uint16_t>(2u * kTableCells);
void fuel_ltft_accum_export(uint8_t* dst, uint16_t cap) noexcept;

int16_t fuel_get_stft_pct_x10() noexcept;
void fuel_reset_ltft() noexcept;

// Sessão LEARN/HIL (comando 'Z'): zera STFT, acumulador, shadows LTFT (NVM
// dirty p/ flush adaptativo) e contadores dbg. Não burn de page0/VE.
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
// LTFT aditivo: retorna offset em µs (negativo = redução de PW)
int16_t fuel_get_ltft_add_us(uint8_t map_idx, uint8_t rpm_idx) noexcept;

// Corte de combustível na desaceleração (MS42 TI_PUR).
// Chama a cada ciclo de injeção (2ms); atualiza estado interno com histerese.
// Retorna true enquanto o corte estiver ativo.
bool fuel_decel_cut_update(uint32_t rpm_x10,
                           uint16_t tps_pct_x10,
                           int16_t clt_x10) noexcept;
bool fuel_decel_cut_active() noexcept;
void fuel_decel_cut_reset() noexcept;

// Compensação barométrica (MS42 TI_FAC_ALTI).
// Leitura do MAP com motor parado → referência dinâmica de pressão atmosférica.
// Clamp de segurança: [70, 110] centibares (700–1100 mbar).
void     fuel_set_baro_bar_x100(uint16_t baro) noexcept;
uint16_t fuel_get_baro_bar_x100() noexcept;

}  // namespace ems::engine
