#include "drv/encoder_sync.h"

// drv → engine já é padrão estabelecido neste projeto (drv/sensors.cpp inclui
// engine/diagnostic_manager.h e engine/ecu_sched.h para o mesmo tipo de
// chamada — checar/reportar plausibilidade de sensor). Ver
// docs/dev/mt6835_encoder_fork.md, "Mitigação de drift silencioso".
#include "engine/ecu_sched.h"
#include "engine/diagnostic_manager.h"
#include "hal/mt6835.h"
#include "hal/mt6835_regs.h"

namespace ems::drv::encoder_sync {

CmpEdgeResult evaluate_cmp_edge(uint32_t cmp_angle_now, bool has_prev,
                                uint32_t prev_cmp_angle,
                                uint8_t reject_streak_in) noexcept
{
    CmpEdgeResult r{};

    if (!has_prev) {
        // Primeiro flanco após boot/perda: só arma a referência, mesma
        // semântica do CKP (ckp.cpp: "arm timestamp only — no phase/confirm
        // until a second edge passes the temporal window").
        r.accepted      = true;
        r.multiple      = 0u;
        r.reject_streak = 0u;
        r.streak_resync = false;
        return r;
    }

    // Wrap-safe: aritmética unsigned de 32 bits, mesmo padrão já usado em
    // todo o resto do fork para deltas de TIM2/TIM5.
    const uint32_t delta = cmp_angle_now - prev_cmp_angle;
    const uint32_t n = (delta + kCmpSpanCounts / 2u) / kCmpSpanCounts;

    bool within_tolerance = false;
    if (n >= 1u && n <= kCmpMaxAcceptedMultiple) {
        const int32_t expected  = static_cast<int32_t>(n * kCmpSpanCounts);
        const int32_t remainder = static_cast<int32_t>(delta) - expected;
        within_tolerance = (remainder >= -static_cast<int32_t>(kCmpSpanToleranceCounts)) &&
                           (remainder <=  static_cast<int32_t>(kCmpSpanToleranceCounts));
    }

    if (within_tolerance) {
        r.accepted      = true;
        r.multiple      = static_cast<uint8_t>(n);
        r.reject_streak = 0u;
        r.streak_resync = false;
        return r;
    }

    r.accepted = false;
    r.multiple = 0u;
    const uint8_t streak = static_cast<uint8_t>(reject_streak_in + 1u);
    if (streak >= kCmpRejectResyncThreshold) {
        r.reject_streak = 0u;
        r.streak_resync = true;
    } else {
        r.reject_streak = streak;
        r.streak_resync = false;
    }
    return r;
}

ZEdgeResult evaluate_z_edge(uint32_t z_angle_now, bool has_prev,
                            uint32_t prev_z_angle,
                            uint8_t reject_streak_in) noexcept
{
    ZEdgeResult r{};

    if (!has_prev) {
        r.accepted      = true;
        r.multiple      = 0u;
        r.reject_streak = 0u;
        r.streak_resync = false;
        return r;
    }

    const uint32_t delta = z_angle_now - prev_z_angle;
    const uint32_t n = (delta + kZSpanCounts / 2u) / kZSpanCounts;

    bool within_tolerance = false;
    if (n >= 1u && n <= kZMaxAcceptedMultiple) {
        const int32_t expected  = static_cast<int32_t>(n * kZSpanCounts);
        const int32_t remainder = static_cast<int32_t>(delta) - expected;
        within_tolerance = (remainder >= -static_cast<int32_t>(kZSpanToleranceCounts)) &&
                           (remainder <=  static_cast<int32_t>(kZSpanToleranceCounts));
    }

    if (within_tolerance) {
        r.accepted      = true;
        r.multiple      = static_cast<uint8_t>(n);
        r.reject_streak = 0u;
        r.streak_resync = false;
        return r;
    }

    r.accepted = false;
    r.multiple = 0u;
    const uint8_t streak = static_cast<uint8_t>(reject_streak_in + 1u);
    if (streak >= kZRejectResyncThreshold) {
        r.reject_streak = 0u;
        r.streak_resync = true;
    } else {
        r.reject_streak = streak;
        r.streak_resync = false;
    }
    return r;
}

bool staleness_exceeded(uint32_t heartbeats_since_accepted, bool bench_mode) noexcept
{
    const uint32_t limit = bench_mode ? kMaxHeartbeatsWithoutCmpBench
                                       : kMaxHeartbeatsWithoutCmp;
    return heartbeats_since_accepted >= limit;
}

static volatile bool g_health_ok = true;

void set_health_ok(bool ok) noexcept { g_health_ok = ok; }
bool health_ok() noexcept { return g_health_ok; }

// ── Mitigação de drift TIM2(AB) vs. MT6835(SPI) ───────────────────────────
// docs/dev/mt6835_encoder_fork.md, "Mitigação de drift silencioso".

int32_t circular_diff16384(uint32_t a, uint32_t b) noexcept
{
    constexpr uint32_t kDomain = 16384u;
    constexpr uint32_t kMask   = kDomain - 1u;
    const uint32_t diff = (a - b) & kMask;  // 0..16383, wrap-safe
    return (diff > (kDomain / 2u))
        ? static_cast<int32_t>(diff) - static_cast<int32_t>(kDomain)
        : static_cast<int32_t>(diff);
}

bool evaluate_angle_plausibility(uint32_t tim2_counts_mod16384,
                                  uint32_t spi_counts_mod16384,
                                  uint32_t tolerance_counts) noexcept
{
    const int32_t delta = circular_diff16384(spi_counts_mod16384, tim2_counts_mod16384);
    const int32_t abs_delta = (delta < 0) ? -delta : delta;
    return static_cast<uint32_t>(abs_delta) <= tolerance_counts;
}

namespace {

// STATUS[2:0] do MT6835 (kRegAngle40Stat, hal/mt6835_regs.h) — qualquer um
// setado é o sintoma físico precursor de miscontagem em AB (campo fraco,
// subtensão, rotação acima do limite garantido de INL). Custo zero: mesmo
// dado da mesma transação SPI que já é lida a cada poll de 100 ms.
namespace R = ems::hal::mt6835;
constexpr uint8_t kStatusFaultMask =
    static_cast<uint8_t>((1u << R::kStatusOverspeedBit) |
                          (1u << R::kStatusWeakFieldBit) |
                          (1u << R::kStatusUndervoltBit));

// Escada de severidade de poll_100ms() — estado privado deste ficheiro.
uint8_t s_fault_strikes  = 0u;
bool    s_fault_active   = false;
bool    s_fault_critical = false;
constexpr uint8_t kFaultCriticalStrikes = 3u;

}  // namespace

void poll_100ms(uint32_t angle21, uint8_t status, uint32_t tim2_raw_now) noexcept
{
    using ems::engine::DiagnosticManager;
    using ems::engine::DiagnosticCode;
    using ems::engine::FaultSeverity;

    const uint32_t tim2_mod = tim2_raw_now & 0x3FFFu;
    const uint32_t spi_mod  = ems::hal::mt6835_angle21_to_tim2_counts(angle21);

    // Camada 2 — correção contínua, gated internamente por
    // spi_reference_trustworthy(); nunca toca TIM2_CNT/CCR3/CCR4.
    ecu_sched_encoder_phase_correction_update(spi_mod, tim2_mod);

    // Correção via Z — validação cruzada do alvo fixo contra o SPI (achado
    // do advisor, ver docs/dev/mt6835_encoder_fork.md, "Correção de drift
    // via Z"): sem isto, um alvo Z ancorado em cima de drift de AB
    // pré-existente ficaria permanentemente errado.
    ecu_sched_encoder_z_target_check(spi_mod);

    // Camada 3 — rede de segurança para o que a Camada 2 não alcança (RPM
    // alto, ou drift já além do teto de correção).
    const bool status_bad = (status & kStatusFaultMask) != 0u;
    const uint32_t tolerance = ecu_sched_encoder_gross_drift_tolerance_counts();
    const bool angle_implausible =
        !evaluate_angle_plausibility(tim2_mod, spi_mod, tolerance);
    const bool bad = status_bad || angle_implausible;

    if (bad) {
        if (s_fault_strikes < 255u) { ++s_fault_strikes; }
        if (!s_fault_active) {
            DiagnosticManager::report_fault(DiagnosticCode::CKP_SIGNAL_FAULT,
                FaultSeverity::WARNING,
                static_cast<uint16_t>(tim2_mod), static_cast<uint16_t>(spi_mod));
            s_fault_active = true;
        } else if (!s_fault_critical && s_fault_strikes >= kFaultCriticalStrikes) {
            // report_fault() sobre um código JÁ activo só actualiza
            // occurrence_count/freeze_frame — NÃO troca a severidade
            // (diagnostic_manager.cpp:29-49). Por isso limpar antes de
            // re-reportar em CRITICAL, senão a escalada nunca aconteceria.
            DiagnosticManager::clear_fault(DiagnosticCode::CKP_SIGNAL_FAULT);
            DiagnosticManager::report_fault(DiagnosticCode::CKP_SIGNAL_FAULT,
                FaultSeverity::CRITICAL,
                static_cast<uint16_t>(tim2_mod), static_cast<uint16_t>(spi_mod));
            s_fault_critical = true;
        }
    } else {
        if (s_fault_active) {
            DiagnosticManager::clear_fault(DiagnosticCode::CKP_SIGNAL_FAULT);
        }
        s_fault_strikes  = 0u;
        s_fault_active   = false;
        s_fault_critical = false;
    }
}

#if defined(EMS_HOST_TEST)
void test_reset(void) noexcept
{
    g_health_ok      = true;
    s_fault_strikes  = 0u;
    s_fault_active   = false;
    s_fault_critical = false;
}
#endif

}  // namespace ems::drv::encoder_sync
