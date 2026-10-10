#include "engine/quick_crank.h"
#include "engine/calibration.h"
#include "engine/math_utils.h"
#include "engine/ms42_cal.h"
#include "drv/ckp.h"
#include "engine/fuel_calc.h"

#include <cstdint>

namespace {

constexpr uint16_t kCrankExitRpmMinX10 = 5000u;
constexpr uint16_t kCrankExitRpmMaxX10 = 12000u;
constexpr uint16_t kPrimePwMaxClampUs = 30000u;

// Tabelas de partida/pós-partida por CLT: calibráveis no bloco MS42 da
// page0 (defaults = os antigos constexpr 3,00×…1,15× etc.).
constexpr uint8_t kCrankPts = ems::engine::kCrankCalPts;
constexpr uint32_t kBaroRefBarX100 = 101u;

volatile bool g_prev_cranking = false;
volatile bool g_afterstart_active = false;
uint32_t g_afterstart_start_ms = 0u;
uint32_t g_afterstart_duration_ms = 0u;
// Ciclos do motor (×1000) desde o início da partida / da pós-partida.
uint32_t g_crank_cyc_x1000 = 0u;
uint32_t g_afterstart_cyc_x1000 = 0u;
uint32_t g_afterstart_dur_cyc_x1000 = 0u;
uint32_t g_qc_last_ms = 0u;
bool     g_qc_have_last = false;

// ── Estado do prime pulse (ISR-safe) ──────────────────────────────────────────
volatile uint8_t  g_prime_tooth_count = 0u;   ///< Dentes contados desde cranking
volatile bool     g_prime_done        = false; ///< Já disparado neste ciclo de partida
volatile bool     g_prime_pending     = false; ///< Sinaliza loop de fundo
volatile uint32_t g_prime_pw_us       = 0u;   ///< PW calculada para disparo
int16_t           g_prime_clt_x10    = 900;   ///< CLT mais recente (do loop de fundo)
uint16_t          g_prime_dead_time_us = 900u; ///< Dead time mais recente, corr. por Vbatt

uint16_t crank_curve(const uint16_t* table, int16_t clt_x10) noexcept {
    return ems::engine::interp_u16_8pt(ems::engine::ms42.crank_clt_axis_x10, table,
                                       kCrankPts, clt_x10);
}

uint16_t sanitized_crank_exit_rpm_x10() noexcept {
    uint16_t exit_rpm = ems::engine::crank_exit_rpm_x10;
    if (exit_rpm < kCrankExitRpmMinX10) {
        exit_rpm = kCrankExitRpmMinX10;
    }
    if (exit_rpm > kCrankExitRpmMaxX10) {
        exit_rpm = kCrankExitRpmMaxX10;
    }
    return exit_rpm;
}

uint16_t sanitized_crank_enter_rpm_x10() noexcept {
    uint16_t enter_rpm = ems::engine::crank_enter_rpm_x10;
    if (enter_rpm < 1000u) {
        enter_rpm = 1000u;
    }
    const uint16_t exit_rpm = sanitized_crank_exit_rpm_x10();
    if (enter_rpm > exit_rpm) {
        enter_rpm = exit_rpm;
    }
    return enter_rpm;
}

bool detect_cranking(uint32_t rpm_x10, bool sync_available) noexcept {
    if (!sync_available || rpm_x10 == 0u) {
        return false;
    }
    if (g_prev_cranking) {
        return rpm_x10 < sanitized_crank_exit_rpm_x10();
    }
    return rpm_x10 <= sanitized_crank_enter_rpm_x10();
}

uint8_t sanitized_prime_tooth() noexcept {
    const uint16_t tooth = ems::engine::crank_prime_tooth;
    if (tooth < 1u) {
        return 1u;
    }
    if (tooth > 20u) {
        return 20u;
    }
    return static_cast<uint8_t>(tooth);
}

uint16_t sanitized_prime_max_pw_us() noexcept {
    const uint16_t max_pw = ems::engine::crank_prime_max_pw_us;
    if (max_pw < 1000u) {
        return 1000u;
    }
    if (max_pw > kPrimePwMaxClampUs) {
        return kPrimePwMaxClampUs;
    }
    return max_pw;
}

// Decai em ms ou, com afterstart_by_cycles (MS42), em ciclos do motor —
// coerente com qualquer rotação de marcha lenta.
uint16_t afterstart_mult_x256(uint32_t now_ms, int16_t clt_x10) noexcept {
    uint32_t elapsed = 0u;
    uint32_t duration = 0u;
    if (ems::engine::ms42.afterstart_by_cycles != 0u) {
        elapsed = g_afterstart_cyc_x1000;
        duration = g_afterstart_dur_cyc_x1000;
    } else {
        elapsed = now_ms - g_afterstart_start_ms;
        duration = g_afterstart_duration_ms;
    }
    if (duration == 0u || elapsed >= duration) {
        return 256u;
    }
    const uint16_t start = crank_curve(ems::engine::ms42.afterstart_start_x256, clt_x10);
    if (start <= 256u) {
        return 256u;
    }
    const uint64_t decay = static_cast<uint64_t>(start - 256u) * elapsed;
    const uint32_t mult = static_cast<uint32_t>(start) - static_cast<uint32_t>(decay / duration);
    return static_cast<uint16_t>(ems::engine::clamp_u32(mult, 256u, 512u));
}

// Multiplicador de partida (MS42 S04): base por CLT × redução pelos ciclos
// já dados × repartida a quente × baro. Também usado pelo prime pulse.
uint32_t crank_fuel_mult_x256(int16_t clt_x10, uint32_t crank_cyc_x1000) noexcept {
    const ems::engine::Ms42Cal& c = ems::engine::ms42;
    uint32_t mult = crank_curve(c.crank_mult_x256, clt_x10);
    if (c.crank_taper_cycles != 0u) {
        const uint32_t span = static_cast<uint32_t>(c.crank_taper_cycles) * 1000u;
        const uint32_t done = (crank_cyc_x1000 > span) ? span : crank_cyc_x1000;
        const uint32_t pct_x1000 = 100000u -
            ((100u - c.crank_taper_end_pct) * done * 1000u) / span;  // 100 %→end
        mult = (mult * pct_x1000) / 100000u;
    }
    if (clt_x10 >= c.hot_restart_clt_x10) {
        mult = (mult * c.hot_restart_pct) / 100u;
    }
    if (c.crank_baro_enable != 0u) {
        const uint32_t baro = ems::engine::fuel_get_baro_bar_x100();
        if (baro >= 50u && baro <= 110u) {
            mult = (mult * baro) / kBaroRefBarX100;
        }
    }
    return ems::engine::clamp_u32(mult, 64u, 2048u);
}

static inline void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

static inline void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

}  // namespace

// ── Override do hook de dente (ISR de CKP, prioridade 1) ────────────────────
// Conta os primeiros dentes recebidos durante cranking.
// Quando o dente-alvo chega, calcula a PW e sinaliza o loop de fundo via
// g_prime_pending. Não requer sincronização — opera em qualquer SyncState.
namespace ems::drv {

void prime_on_tooth(const CkpSnapshot& snap) noexcept {
    using namespace ems::engine;  // acessa g_prime_* e constantes

    if (g_prime_done) { return; }

    // Só conta enquanto RPM indica cranking. Deriva do período de dente cru:
    // snap.rpm_x10 é gated por sync (0 até ao primeiro gap) e o prime dispara
    // de propósito nos primeiros dentes, ANTES de haver referência angular.
    // rpm_x10 = 600e9 / (60 dentes × period_ns) × 10 = 1e10 / period_ns.
    const uint32_t period_ns = snap.tooth_period_ns;
    if (period_ns == 0u) { return; }
    const uint32_t raw_rpm_x10 = static_cast<uint32_t>(10000000000ull / period_ns);
    if (raw_rpm_x10 == 0u || raw_rpm_x10 >= sanitized_crank_exit_rpm_x10()) { return; }

    ++g_prime_tooth_count;
    
    // FIX P1 (BUG-8): Resetar contador se ultrapassar o dente alvo sem disparar
    // Previne que falha na partida anterior impeça prime pulse na próxima tentativa
    const uint8_t target_tooth = sanitized_prime_tooth();
    if (g_prime_tooth_count > target_tooth + 5u) {
        // Ultrapassou o dente alvo com margem de segurança — reseta para próxima tentativa
        g_prime_tooth_count = 0u;
        g_prime_done = false;
        return;
    }
    
    if (g_prime_tooth_count < target_tooth) { return; }

    // Dente-alvo: calcula PW usando a tabela de enriquecimento de cranking,
    // CLT e dead time mais recentes atualizados pelo loop de fundo.
    const uint32_t mult = crank_fuel_mult_x256(g_prime_clt_x10, 0u);
    // REQ_FUEL from the configured engine (NVM), not the compile-time default.
    uint32_t pw = ((ems::engine::default_req_fuel_us() * static_cast<uint32_t>(mult)) >> 8u) +
        static_cast<uint32_t>(g_prime_dead_time_us);
    const uint16_t prime_max_pw_us = sanitized_prime_max_pw_us();
    if (pw > prime_max_pw_us) { pw = prime_max_pw_us; }

    g_prime_pw_us  = pw;
    g_prime_pending = true;
    g_prime_done   = true;
}

}  // namespace ems::drv

namespace ems::engine {

// Flood-clear APP threshold (pct×10). 700 = 70% pedal/TPS during crank → cut fuel.
uint16_t crank_flood_tps_x10 = 700u;

void quick_crank_reset() noexcept {
    g_prev_cranking = false;
    g_afterstart_active = false;
    g_afterstart_start_ms = 0u;
    g_afterstart_duration_ms = 0u;
    g_crank_cyc_x1000 = 0u;
    g_afterstart_cyc_x1000 = 0u;
    g_afterstart_dur_cyc_x1000 = 0u;
    g_qc_last_ms = 0u;
    g_qc_have_last = false;
    g_prime_tooth_count = 0u;
    g_prime_done        = false;
    g_prime_pending     = false;
    g_prime_pw_us       = 0u;
}

QuickCrankOutput quick_crank_update(uint32_t now_ms,
                                    uint32_t rpm_x10,
                                    bool sync_available,
                                    int16_t clt_x10,
                                    int16_t base_spark_deg) noexcept {
    QuickCrankOutput out{};
    out.spark_deg = base_spark_deg;
    out.fuel_mult_x256 = 256u;
    out.min_pw_us = 0u;
    out.prime_pw_us = g_prime_pw_us;  // informativo — disparo é feito via consume_prime()

    const bool cranking = detect_cranking(rpm_x10, sync_available);
    out.cranking = cranking;

    // Ciclos (2 voltas) desde o último update: rpm×10 · dt_ms / 1 200 000,
    // contados ×1000.
    const uint32_t dt_ms = g_qc_have_last ? (now_ms - g_qc_last_ms) : 0u;
    g_qc_last_ms = now_ms;
    g_qc_have_last = true;
    const uint32_t dcyc_x1000 = (dt_ms < 1000u) ? (rpm_x10 * dt_ms) / 1200u : 0u;

    if (rpm_x10 == 0u) {
        g_crank_cyc_x1000 = 0u;
        g_prime_tooth_count = 0u;
        g_prime_done = false;
        g_prime_pending = false;
        g_prime_pw_us = 0u;
    }

    if (cranking) {
        out.spark_deg = crank_spark_deg;
        out.min_pw_us = crank_min_pw_us;
        g_crank_cyc_x1000 += dcyc_x1000;
        out.fuel_mult_x256 = static_cast<uint16_t>(
            crank_fuel_mult_x256(clt_x10, g_crank_cyc_x1000));
        g_afterstart_duration_ms = 0u;
        g_afterstart_dur_cyc_x1000 = 0u;
    } else {
        if (g_prev_cranking && rpm_x10 >= sanitized_crank_exit_rpm_x10()) {
            g_afterstart_start_ms = now_ms;
            g_afterstart_duration_ms = crank_curve(ms42.afterstart_ms, clt_x10);
            g_afterstart_cyc_x1000 = 0u;
            g_afterstart_dur_cyc_x1000 =
                static_cast<uint32_t>(crank_curve(ms42.afterstart_cycles, clt_x10)) * 1000u;
        } else {
            g_afterstart_cyc_x1000 += dcyc_x1000;
        }
        g_crank_cyc_x1000 = 0u;
        const uint16_t as_mult = afterstart_mult_x256(now_ms, clt_x10);
        out.afterstart_active = (as_mult > 256u);
        out.fuel_mult_x256 = as_mult;
    }

    g_prev_cranking = cranking;
    g_afterstart_active = out.afterstart_active;
    return out;
}

uint32_t quick_crank_apply_pw_us(uint32_t base_pw_us,
                                 uint16_t fuel_mult_x256,
                                 uint32_t min_pw_us) noexcept {
    uint32_t out = static_cast<uint32_t>(
        (static_cast<uint64_t>(base_pw_us) * fuel_mult_x256) / 256u);
    if (out < min_pw_us) {
        out = min_pw_us;
    }
    if (out > 100000u) {
        out = 100000u;
    }
    return out;
}

uint32_t quick_crank_flow_us(const QuickCrankOutput& qc, uint32_t running_flow_us) noexcept {
    const uint32_t base = qc.cranking ? default_req_fuel_us() : running_flow_us;
    return quick_crank_apply_pw_us(base, qc.fuel_mult_x256, qc.min_pw_us);
}

void quick_crank_set_prime_context(int16_t clt_x10, uint16_t dead_time_us) noexcept {
    // Escrita atômica de int16_t em Cortex-M4 (single instrução STRH).
    g_prime_clt_x10 = clt_x10;
    g_prime_dead_time_us = dead_time_us;
}

void quick_crank_set_clt(int16_t clt_x10) noexcept {
    // Mantém compatibilidade com chamadas antigas.
    g_prime_clt_x10 = clt_x10;
}

uint32_t quick_crank_consume_prime() noexcept {
    enter_critical();
    uint32_t pw = 0u;
    if (g_prime_pending) {
        pw = g_prime_pw_us;
        g_prime_pending = false;
    }
    exit_critical();
    return pw;
}

bool is_cranking() noexcept {
    return g_prev_cranking;
}

bool is_afterstart() noexcept {
    return g_afterstart_active;
}

bool crank_flood_clear_active(uint16_t app_pct_x10) noexcept {
    if (!g_prev_cranking) {
        return false;
    }
    uint16_t thr = crank_flood_tps_x10;
    if (thr < 200u) {
        thr = 200u;  // never below 20% — avoid noise trips
    }
    if (thr > 1000u) {
        thr = 1000u;
    }
    return app_pct_x10 >= thr;
}

}  // namespace ems::engine
