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
 * HARDWARE: FTM3 Canal 0 (PTD0/CKP) em modo Input Capture, rising edge.
 *   ISR: ckp_ftm3_ch0_isr() — chamada por FTM3_IRQHandler() em hal/ftm.cpp
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
    uint16_t tooth_index;        ///< Índice do dente (0–57) contado desde o último gap; válido em FULL_SYNC
    uint16_t last_ftm3_capture;  ///< Timestamp FTM3 (ticks) do último dente — para angle-to-ticks
    uint32_t rpm_x10;            ///< RPM × 10 (ex: 8000 = 800,0 RPM); 0 antes de dados suficientes
    SyncState state;             ///< Estado corrente da máquina de sincronismo
    bool phase_A;                ///< Fase do ciclo de 4 tempos — alterna a cada evento no cam sensor (CH1)
};

/**
 * @brief Retorna instantâneo atômico do estado CKP.
 *
 * Seguro para chamada de qualquer contexto (main loop, ISR de menor
 * prioridade). Usa seção crítica CPSID/CPSIE internamente.
 */
CkpSnapshot ckp_snapshot() noexcept;

/**
 * @brief Converte ângulo de virabrequim em tick-alvo no domínio FTM3.
 *
 * @param angle_mdeg  Ângulo em MILIGRAUS (×1000 de graus inteiros).
 *                    Ex: 6000 = 6,0°; 60000 = 60,0°.
 *                    ATENÇÃO: passar graus inteiros causa erro de 1000×.
 * @param ref_capture Timestamp FTM3 do dente de referência
 *                    (tipicamente snap.last_ftm3_capture).
 * @return            Valor CnV para FTM3 — NÃO compatível com FTM0.
 *
 * DOMÍNIO DE CLOCK: FTM3 @ 60 MHz (PS=2, 16,7 ns/tick).
 * FTM0 @ 15 MHz (PS=8, 66,7 ns/tick). Para converter o delta para ticks
 * FTM0 divida por 4 (60 MHz / 15 MHz).
 *
 * STATUS (2026-03): Sem callers em produção. O pipeline ecu_sched deriva
 * ângulo→ticks no domínio FTM0 via g_ticks_per_rev (ecu_sched.cpp,
 * Calculate_Sequential_Cycle). Reservada para uso futuro em saídas
 * CKP-síncronas (tach-out, came via FTM3 output-compare).
 */
uint16_t ckp_angle_to_ticks(uint16_t angle_mdeg, uint16_t ref_capture) noexcept;

// ── Hooks ─────────────────────────────────────────────────────────────────────
// Chamados pela ISR de CKP a cada dente (símbolos fracos — sobrescreva para
// adicionar comportamento sem modificar este módulo).
//
// sensors_on_tooth  → drv/sensors.cpp  (amostragem sincronizada ao dente)
// schedule_on_tooth → engine/cycle_sched.cpp (agendamento injeção/ignição)
// prime_on_tooth    → engine/quick_crank.cpp (prime pulse — 5º dente de cranking)

void sensors_on_tooth(const CkpSnapshot& snap) noexcept;
void schedule_on_tooth(const CkpSnapshot& snap) noexcept;
void prime_on_tooth(const CkpSnapshot& snap) noexcept;

// ── ISR handlers (chamados de hal/ftm.cpp) ────────────────────────────────────
void ckp_ftm3_ch0_isr() noexcept;   ///< CKP rising edge (FTM3 CH0 / PTD0)
void ckp_ftm3_ch1_isr() noexcept;   ///< Cam sensor rising edge (FTM3 CH1 / PTD1)

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

// ── API de teste (somente em build host) ──────────────────────────────────────
#if defined(EMS_HOST_TEST)
void     ckp_test_reset() noexcept;
uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept;
#endif

}  // namespace ems::drv
